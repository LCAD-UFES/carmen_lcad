/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/task_manager_interface.h>
#include <carmen/task_manager_messages.h>
#include <carmen/voice_interface_messages.h>
#include <carmen/voice_interface_interface.h>
#include <jaus.h>				// Header file for JAUS types, structs and messages
#include <openJaus.h>				// Header file for the OpenJAUS specific C/C++ code base
#include <torc.h>
#include <torcInterface.h>
#include <car_model.h>
#include <control.h>
#include "ford_escape_hybrid.h"

//#define LATENCY_TEST_PRINT

//#define FORD_ESCAPE_COMMUNICATION_DUMP
//#define PLOT_PHI
//#define PLOT_VELOCITY
#define FORD_ESCAPE_CYCLE_TIME	(1.0 / 40.0)

#define ROBOT_NAME_FORD_ESCAPE 	0
#define ROBOT_NAME_ECOTECH4 	1
#define ROBOT_NAME_MPW700 		2
#define ROBOT_NAME_ASTRU 		3

#define STEERING_INITIALIZATION_ERROR_CODE 9101 // manual Torc p. 89


static ford_escape_hybrid_config_t *ford_escape_hybrid_config = NULL;

static carmen_robot_and_trailer_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_VECTORS][NUM_MOTION_COMMANDS_PER_VECTOR];
static int nun_motion_commands[NUM_MOTION_COMMANDS_VECTORS];
static int current_motion_command_vetor = 0;

// TORC/JAUS variables
static int *xgv_ccu_service_connections;
static OjCmpt XGV_CCU;

// Global variables
static double g_atan_desired_curvature = 0.0;
static double g_desired_velocity = 0.0;

static double phi_multiplier;
static double phi_bias;
static double v_multiplier;

int robot_model_name = ROBOT_NAME_FORD_ESCAPE;
int tune_pid_mode = 0;

int soft_stop_on = 0;

carmen_robot_ackerman_velocity_message alternative_odometry;

int g_go_state = 0;

carmen_behavior_selector_low_level_state_t behavior_selector_low_level_state = Stopped;
int behavior_selector_going_backwards = 0;

double g_XGV_velocity_temp = 0.0;

carmen_localize_ackerman_globalpos_message global_pos, previous_global_pos;

carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message = NULL;


static double
get_delta_t(double *previous_t)
{
	double t;
	double delta_t;

	t = carmen_get_time();
	if (*previous_t == 0.0)
	{
		*previous_t = t;
		return (0.0);
	}
	delta_t = t - *previous_t;
	*previous_t = t;

	return (delta_t);
}


static double
get_steering_delta_t()
{
	static double previous_t = 0.0;

	return (get_delta_t(&previous_t));
}


static double
get_velocity_delta_t()
{
	static double previous_t = 0.0;

	return (get_delta_t(&previous_t));
}


double
get_curvature_from_phi(double phi, double v, ford_escape_hybrid_config_t *ford_escape_hybrid_config)
{	// See Torc, "ByWire XGVTM User Manual", page 42
	double curvature;
	
	curvature = tan(phi / (1.0 + v * v * ford_escape_hybrid_config->understeer_coeficient2)) / ford_escape_hybrid_config->distance_between_front_and_rear_axles;
	
	return (curvature);
}


double
get_phi_from_curvature(double curvature, ford_escape_hybrid_config_t *ford_escape_hybrid_config)
{
	double phi, v;
	
	v = ford_escape_hybrid_config->filtered_v;
	phi = atan(curvature * ford_escape_hybrid_config->distance_between_front_and_rear_axles) * (1.0 + v * v * ford_escape_hybrid_config->understeer_coeficient2);
	
	return (phi);
}


double
decreased_v_according_to_acceptable_acceleations(double v)
{
	double new_v;

	double robot_desired_decelaration_forward = 5.0;
	new_v = v - robot_desired_decelaration_forward * FORD_ESCAPE_CYCLE_TIME;

	return (new_v);
}


double
increased_v_according_to_acceptable_acceleations(double v)
{
	double new_v;

	double robot_maximum_acceleration_forward = 5.0;
	new_v = v + robot_maximum_acceleration_forward * FORD_ESCAPE_CYCLE_TIME;

	return (new_v);
}


static void
set_wrench_efforts_desired_v_curvature_and_gear()
{
	int i;
	double v, phi;
	double current_time, command_time;

	current_time = carmen_get_time();
	command_time = ford_escape_hybrid_config->time_of_last_command;
	for (i = 0; i < ford_escape_hybrid_config->nun_motion_commands; i++)
	{
		command_time += ford_escape_hybrid_config->current_motion_command_vector[i].time;

		if (current_time <= command_time)
			break;
	}

	if (i < ford_escape_hybrid_config->nun_motion_commands)
	{
		v = ford_escape_hybrid_config->current_motion_command_vector[i].v;
		if (!ford_escape_hybrid_config->use_mpc)
			phi = (1.0 + v / (6.94 / 0.3)) * ford_escape_hybrid_config->current_motion_command_vector[i].phi;
		else
			phi = ford_escape_hybrid_config->current_motion_command_vector[i].phi;
	}
	else
	{
		v = 0.0;
		phi = 0.0;
	}

//	printf("timestamp %lf, v %lf, i %d, n %d\n", current_time, v, i, ford_escape_hybrid_config->nun_motion_commands);

//	if ((fabs(ford_escape_hybrid_config->filtered_v) > 1.0) && path_goals_and_annotations_message && (path_goals_and_annotations_message->goal_list_size != 0) && path_goals_and_annotations_message->goal_list)
//	{
//		double v_goal = path_goals_and_annotations_message->goal_list[0].v;
//
//		if (v_goal > 0.0)
//		{
//			if (fabs(v - v_goal) < 0.3)
//				v = v_goal;
//			else if (v > v_goal)
//				v = decreased_v_according_to_acceptable_acceleations(v);
//			else
//				v = increased_v_according_to_acceptable_acceleations(v);
//		}
//	}
//
//	if (v > ford_escape_hybrid_config->max_velocity)
//		v = ford_escape_hybrid_config->max_velocity;
//	if (v < ford_escape_hybrid_config->max_velocity_reverse)
//		v = ford_escape_hybrid_config->max_velocity_reverse;

	// The function carmen_ford_escape_hybrid_steering_PID_controler() uses g_atan_desired_curvature to compute the g_steering_command that is sent to the car.
	// This function is called when new info about the current measured velocity (g_XGV_velocity) arrives from the car via Jaus messages handled
	// by the torc_report_velocity_state_message_handler() callback function.

	//	double desired_curvature = -atan(get_curvature_from_phi(phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)); // @@@ Alberto: a curvatura do carro ee ao contrario de carmen
	//	double delta_curvature = fabs(desired_curvature - g_atan_desired_curvature);
	//	double command_curvature_signal = (g_atan_desired_curvature < desired_curvature) ? 1.0 : -1.0;
	//	static double previous_timestamp = 0.0;
	//	if (!previous_timestamp)
	//		previous_timestamp = current_time;
	//	double delta_t = current_time - previous_timestamp;
	//	double max_curvature_change = delta_t * 0.03;
	//	previous_timestamp = current_time;
	//
	//	g_atan_desired_curvature = g_atan_desired_curvature + command_curvature_signal * fmin(delta_curvature, max_curvature_change);

	g_atan_desired_curvature = -atan(get_curvature_from_phi(phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)); // @@@ Alberto: a curvatura do carro ee ao contrario de carmen

	// The function carmen_ford_escape_hybrid_velocity_PID_controler() uses g_desired_velocity to compute the g_throttle_command, g_brakes_command and g_gear_command
	// that are sent to the car.
	// This function is called when new info about the current velocity (g_XGV_velocity) arrives from the car via Jaus messages handled
	// by the torc_report_velocity_state_message_handler() callback function.
	if (behavior_selector_going_backwards)
		g_gear_command = 129;	// 129 = Reverse gear (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
	else
		g_gear_command = 1;		// 1 = Low; 2 = Drive (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)

	if (tune_pid_mode)
	{
		if (v < 0.0)
			g_gear_command = 129;	// 129 = Reverse gear (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		else
			g_gear_command = 1;		// 1 = Low; 2 = Drive (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)

		g_desired_velocity = v;
	}
	else
	{
		static double last_neutral_gear_timestamp = 0.0;

		if (g_go_state &&
			(behavior_selector_low_level_state != Stopped) &&
			(behavior_selector_low_level_state != End_Of_Path_Reached2))
		{
			if (carmen_get_time() - last_neutral_gear_timestamp > 3.0)	// In order to wait for the gear hardware...
				g_desired_velocity = v;
			else
				g_desired_velocity = 0.0;
		}
		else
		{
			g_desired_velocity = 0.0;
			g_gear_command = 128;	// 128 = Neutral
			last_neutral_gear_timestamp = carmen_get_time();
		}
	}

//	static double last_time_with_desired_velocity_not_zero = 0.0;
//	if (v != 0.0)
//		last_time_with_desired_velocity_not_zero = carmen_get_time();

//	if ((robot_model_name == ROBOT_NAME_MPW700) && ((carmen_get_time() - last_time_with_desired_velocity_not_zero) > 0.3))
//		g_gear_command = 128;	// 128 = Neutral
}


void
build_combined_visual_and_car_odometry()
{
	switch (combine_odometry_vel)
	{
	case ALTENATIVE_ODOMETRY_VEL:
		g_XGV_velocity = alternative_odometry.v;
		break;
	case CAR_ODOMETRY_VEL:
		break;

	case ALTERNATIVE_COMBINED_WITH_CAR_ODOMETRY_VEL:
		//TODO
		g_XGV_velocity = (g_XGV_velocity + alternative_odometry.v) / 2.0;
		break;
	default:

		break;
	}

	switch (combine_odometry_phi)
	{
	case ALTERNATIVE_ODOMETRY_PHI:
		g_XGV_atan_curvature = get_curvature_from_phi(alternative_odometry.phi, g_XGV_velocity, ford_escape_hybrid_config);
		break;
	case CAR_ODOMETRY_PHI:
		break;
	case ALTERNATIVE_COMBINED_WITH_CAR_ODOMETRY_PHI:
		g_XGV_atan_curvature = (g_XGV_atan_curvature + get_curvature_from_phi(alternative_odometry.phi, g_XGV_velocity, ford_escape_hybrid_config)) / 2.0;
		break;
	default:
		break;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_car_status()
{
//	printf("cambio %d\n", g_XGV_gear);

	carmen_ford_escape_status_message msg;

	msg.g_XGV_throttle = g_XGV_throttle;
	msg.g_XGV_steering = g_XGV_steering;
	msg.g_XGV_brakes = g_XGV_brakes;

	//	g_XGV_component_status bit Interpretation F: disengaged, T: engaged
	//  See page 62 of ByWire XGV User Manual, Version 1.5
	//  Note que está errado no manual: este campo da mensagem tem 32 bits e não 16 como diz o manual
	//  Os primeiros 16 bits não são usados
	//  ordem do campo e (bit)
	//	0 (16) Manual override
	//	1 (17) SafeStop pause relay F: run, T: pause
	//	2 (18) SafeStop stop relay F: run, T: stop
	//	3-4 (19-20) SafeStop link status 7 (23) Partial mode override: speed 0: no link, 1: bypass, 2: link good
	//	5 (21) External E-Stop button F: run, T: stop
	//	6 (22) Partial mode override: F: computer control, T: manual
	//	       steering control
	//	               F: computer control, T: manual
	//	              control
	//	8 (24) Door/Liftgate pause F: run, T: pause
	//	9 (25) Error-caused pause* F: run, T: pause
	//	10 (26) Emergency manual override* F: disengaged, T: engaged
	//	11 (27) Steering needs initialization* F: initialized, T: uninitialized
	//	        Steering initialization waiting F: initialized, T: user must init
	//	        on user input* steering manually or press OK
	msg.g_XGV_component_status = g_XGV_component_status;

	msg.g_XGV_main_propulsion = g_XGV_main_propulsion;
	msg.g_XGV_main_fuel_supply = g_XGV_main_fuel_supply;
	msg.g_XGV_parking_brake = g_XGV_parking_brake;
	msg.g_XGV_gear = g_XGV_gear;

	msg.g_XGV_turn_signal = g_XGV_turn_signal;
	msg.g_XGV_horn_status = g_XGV_horn_status;
	msg.g_XGV_headlights_status = g_XGV_headlights_status;

//	printf("g_XGV_turn_signal = 0x%x\n", g_XGV_turn_signal);

	carmen_ford_escape_publish_status_message(&msg, carmen_get_time());
}


static void
publish_car_error()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_ford_escape_error_message msg;
	int i;

	msg.num_errors = g_XGV_num_errors;
	msg.error = calloc(msg.num_errors, sizeof(int));
	for (i = 0; i < msg.num_errors; i++)
		msg.error[i] = g_XGV_error[i];

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_FORD_ESCAPE_ERROR_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_FORD_ESCAPE_ERROR_NAME);
}


static void
publish_velocity_message(void *clientData __attribute__ ((unused)), unsigned long currentTime __attribute__ ((unused)),
		unsigned long scheduledTime __attribute__ ((unused)))
{
	static int heartbeat = 0;

//	printf("v %lf, phi %lf, status %lf\n", g_XGV_velocity, get_phi_from_curvature(-tan(g_XGV_atan_curvature), ford_escape_hybrid_config), carmen_get_time());
	if (ford_escape_hybrid_config->publish_odometry)
		carmen_robot_ackerman_publish_velocity(g_XGV_velocity, get_phi_from_curvature(-tan(g_XGV_atan_curvature), ford_escape_hybrid_config),
				carmen_get_time());

	if ((heartbeat % 10) == 0)
	{
		publish_car_status();
		publish_car_error();
	}

	heartbeat++;
}


void
activate_soft_stop()
{
	if (!soft_stop_on)
	{
		carmen_voice_interface_command_message message;
		message.command_id = SET_SPEED;
		message.command = "0.0";
		message.host = carmen_get_host();
		message.timestamp = carmen_get_time();
		carmen_voice_interface_publish_command_message(&message);
		soft_stop_on = 1;
	}
}


void
deactivate_soft_stop()
{
	if (soft_stop_on)
	{
		carmen_voice_interface_command_message message;
		message.command_id = SET_SPEED;
		message.command = "MAX_SPEED";
		message.host = carmen_get_host();
		message.timestamp = carmen_get_time();
		carmen_voice_interface_publish_command_message(&message);
		soft_stop_on = 0;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Jaus Publishers                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
publish_ford_escape_steering_throttle_and_brakes_command(OjCmpt XGV_CCU)
{
	send_set_wrench_efforts_message(XGV_CCU);

#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
	FILE *caco = fopen("ford_dump.txt", "a");
	fprintf(caco, "%lf g_throttle_command %lf g_steering_command %lf g_brakes_command %lf\n", carmen_get_time(),
			g_throttle_command, g_steering_command, g_brakes_command);
	fflush(caco);
	fclose(caco);
#endif
}


void
publish_ford_escape_turn_horn_and_headlight_signals(OjCmpt XGV_CCU)
{
	send_set_signals_message(XGV_CCU);

#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
	FILE *caco = fopen("ford_dump.txt", "a");
	fprintf(caco, "%lf g_turn_signal_command %d g_horn_status_command %d g_headlights_status_command %d\n", carmen_get_time(),
			g_turn_signal_command, g_horn_status_command, g_headlights_status_command);
	fflush(caco);
	fclose(caco);
#endif
}


void
publish_ford_escape_gear_command(OjCmpt XGV_CCU)
{
	send_set_discrete_devices_message(XGV_CCU);

#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
	FILE *caco = fopen("ford_dump.txt", "a");
	fprintf(caco, "%lf presenceVector %d g_gear_command %d\n", carmen_get_time(),
			4, g_gear_command);
	fflush(caco);
	fclose(caco);
#endif
}
//////////////////////////////////////////////////////////////////////////////////////////////////


int
apply_system_latencies(carmen_robot_and_trailer_motion_command_t *current_motion_command_vector, int nun_motion_commands)
{
	int i, j;
	double lat;

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (lat = 0.0; lat < 0.2; j++)
		{
			if (j >= nun_motion_commands)
				break;
			lat += current_motion_command_vector[j].time;
		}
		if (j >= nun_motion_commands)
			break;
		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
	}

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (lat = 0.0; lat < 0.6; j++)
		{
			if (j >= nun_motion_commands)
				break;
			lat += current_motion_command_vector[j].time;
		}
		if (j >= nun_motion_commands)
			break;
		current_motion_command_vector[i].v = current_motion_command_vector[j].v;
	}

	return (i);
}


void
check_jaus_reported_errors()
{
	int i;
	int needs_soft_stop = 0;

	if (g_XGV_num_errors != 0)
	{
		for (i = 0; i < g_XGV_num_errors; i++)
		{
			if (g_XGV_error[i] == STEERING_INITIALIZATION_ERROR_CODE)
				needs_soft_stop = 1;
		}
	}

	if (needs_soft_stop)
		activate_soft_stop();
	else
		deactivate_soft_stop();
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
base_ackerman_motion_command_message_handler(carmen_base_ackerman_motion_command_message *motion_command_message)
{
//	printf("recebi a mensagem_base_ackerman\n");
	int num_motion_commands, i;

	current_motion_command_vetor = (current_motion_command_vetor + 1) % NUM_MOTION_COMMANDS_VECTORS;

	num_motion_commands = fmin(NUM_MOTION_COMMANDS_PER_VECTOR, motion_command_message->num_motion_commands);

	for (i = 0; i < num_motion_commands; i++)
	{
		motion_commands_vector[current_motion_command_vetor][i] = motion_command_message->motion_command[i];
//		printf("cv %lf cphi %lf ctime %lf numMotionsC %d\n", motion_command_message->motion_command[i].v, motion_command_message->motion_command[i].phi, motion_command_message->motion_command[i].time, num_motion_commands);
//		fflush(stdout);
	}

	nun_motion_commands[current_motion_command_vetor] = num_motion_commands;

	ford_escape_hybrid_config->current_motion_command_vector = motion_commands_vector[current_motion_command_vetor];
	ford_escape_hybrid_config->nun_motion_commands = nun_motion_commands[current_motion_command_vetor];
	ford_escape_hybrid_config->time_of_last_command = motion_command_message->timestamp;


	if (ford_escape_hybrid_config->use_mpc)
		ford_escape_hybrid_config->nun_motion_commands = apply_system_latencies(ford_escape_hybrid_config->current_motion_command_vector, ford_escape_hybrid_config->nun_motion_commands);
}


static void
fused_odometry_message_handler(carmen_fused_odometry_message *fused_odometry_message)
{
	ford_escape_hybrid_config->filtered_pitch = fused_odometry_message->pose.orientation.pitch;
}


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	previous_global_pos = global_pos;
	global_pos = *msg;
}


static void
ford_escape_signals_message_handler(carmen_ford_escape_signals_message *msg)
{
	g_turn_signal_command = msg->turn_signal;
	g_horn_status_command = msg->horn_status;
	g_headlights_status_command = msg->headlight_status;
	if (msg->high_beams)
		if ((g_headlights_status_command & 7) == 2)
			g_headlights_status_command ^= 8;
	if (msg->fog_lights)
		if ((g_headlights_status_command & 7) == 2)
			g_headlights_status_command ^= 0x10;

//	printf("g_horn_status_command %d, g_headlights_status_command %d\n", g_horn_status_command, g_headlights_status_command);
	publish_ford_escape_turn_horn_and_headlight_signals(XGV_CCU);
}


static void
task_manager_desired_engage_state_message_handler(carmen_task_manager_desired_engage_state_message *message)
{
	if (message->desired_engage_state == ENGAGED)
		g_horn_status_command &= ~0x02;
	else if (message->desired_engage_state == DISENGAGED)
		g_horn_status_command |= 0x02;
}


static void
carmen_lidar_velocity_message_handler(carmen_robot_ackerman_velocity_message *message)
{
	alternative_odometry.v = message->v;
	alternative_odometry.phi = message->phi;
}


static void
navigator_ackerman_go_message_handler()
{
	change_control_mode_to_wrench_efforts(XGV_CCU);

//	g_gear_command = 1; // 1 = Low
//	publish_ford_escape_gear_command(XGV_CCU);

	g_go_state = 1;
	carmen_warn("go\n");
}


static void
navigator_ackerman_stop_message_handler()
{
	change_control_mode_to_wrench_efforts(XGV_CCU);
	g_go_state = 0;
	carmen_warn("stop\n");
}


static void
visual_odometry_handler(carmen_visual_odometry_pose6d_message *message)
{
	alternative_odometry.v = message->v;
	alternative_odometry.phi = message->phi;
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	behavior_selector_low_level_state = msg->low_level_state;
	behavior_selector_going_backwards = msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;

	if (msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_ENGAGE_COLLISION_GEOMETRY)
		carmen_collision_detection_set_robot_collision_config(ENGAGE_GEOMETRY);
	else
		carmen_collision_detection_set_robot_collision_config(DEFAULT_GEOMETRY);
}


static void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *msg)
{
	path_goals_and_annotations_message = msg;
}



static void 
shutdown_module()
{
	static int done = 0;

	if (!done)
	{
		//send_set_engine_message(XGV_CCU, FALSE);
		terminate_xgv_ccu_service_connections(XGV_CCU, xgv_ccu_service_connections);
		ojCmptDestroy(XGV_CCU);
		carmen_ipc_disconnect();
		printf("Disconnected from IPC.\n");
		done = 1;
	}
	exit(0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// JAUS Handlers                                                                                //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
torc_report_velocity_state_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage velocity_message) 
{
	ReportVelocityStateMessage reportVelocityState;

	reportVelocityState = reportVelocityStateMessageFromJausMessage(velocity_message);
	if (reportVelocityState)
	{
		// A media da velocidade das radas trazeiras ee mais correta. A velocidade abaixo ee a media das rodas dianteiras mais um ruido estranho...
		// g_XGV_velocity_temp = reportVelocityState->velocityXMps;
#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
		FILE *caco = fopen("ford_dump.txt", "a");
		fprintf(caco, "%lf velocityXMps %lf\n", carmen_get_time(), reportVelocityState->velocityXMps);
		fflush(caco);
		fclose(caco);
#endif
		
		reportVelocityStateMessageDestroy(reportVelocityState);
	}
	else
	{
		carmen_warn("In torc_report_velocity_state_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(velocity_message));
	}
}


void
clear_current_motion_command_vector(carmen_robot_and_trailer_motion_command_t *current_motion_command_vector, int nun_motion_commands)
{
	for (int i = 0; i < nun_motion_commands; i++)
		current_motion_command_vector[i].phi = 0.0;
}


void
print_values_to_train_simulator(double atan_desired_curvature, double atan_current_curvature, double delta_t, double steering_effort)
{
	double 		error_t;		// error in time t
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	double		derivative_t;

	if (delta_t == 0.0)
		return;

	error_t = atan_desired_curvature - atan_current_curvature;
	integral_t = integral_t + error_t * delta_t;
	derivative_t = (error_t - error_t_1) / delta_t;

	error_t_1 = error_t;

	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf\n",
		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, steering_effort);
	fflush(stdout);
}


static void
torc_report_curvature_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage curvature_message)
{
	static int previous_gear_command = 128;	// 128 = Neutral
	ReportCurvatureMessage reportCurvature;
	double raw_phi;
	double delta_t;

	reportCurvature = reportCurvatureMessageFromJausMessage(curvature_message);
	if (reportCurvature)
	{
//		static double last_update = 0.0;
//		double t = ojGetTimeSec();
//		double steering_update_freq = 0.0;
//		if (last_update != 0.0)
//			steering_update_freq = 1.0 / (t - last_update);
//		last_update = t;
//
//		printf("steering_update_freq %lf\r", steering_update_freq);
//		if (steering_update_freq < 30.0)
//			printf("* steering_update_freq %lf\n", steering_update_freq);


#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
		FILE *caco = fopen("ford_dump.txt", "a");
		fprintf(caco, "%lf atanOfCurrentCurvature %lf\n", carmen_get_time(), reportCurvature->atanOfCurrentCurvature);
		fflush(caco);
		fclose(caco);
#endif
		g_XGV_atan_curvature = reportCurvature->atanOfCurrentCurvature; // @@@ Alberto: a curvatura do carro vem ao contrario de carmen
		g_XGV_velocity = g_XGV_velocity_temp;

		if (publish_combined_odometry)
			build_combined_visual_and_car_odometry();
//		printf("*v %lf, phi %lf, timestamp %lf\n", g_XGV_velocity, get_phi_from_curvature(-tan(g_atan_desired_curvature), ford_escape_hybrid_config), carmen_get_time());

		ford_escape_hybrid_config->XGV_v_and_phi_timestamp = carmen_get_time();
		raw_phi = get_phi_from_curvature(-tan(g_XGV_atan_curvature), ford_escape_hybrid_config);
		carmen_add_bias_and_multiplier_to_v_and_phi(&(ford_escape_hybrid_config->filtered_v), &(ford_escape_hybrid_config->filtered_phi), 
						    g_XGV_velocity, raw_phi, 
						    0.0, v_multiplier, phi_bias, phi_multiplier);
#ifdef LATENCY_TEST_PRINT
		//@@@VINICIUS AQUI QUE RECEBE O PHI VINDO DO OJASTRU2
		fprintf(stdout, "ford_form_oj_feedback (filt_phi_ford_form_oj_feedback, cc_ford_form_oj_feedback, t): %lf, %lf, %lf\n",
				ford_escape_hybrid_config->filtered_phi, g_XGV_atan_curvature, carmen_get_time());
		fflush(stdout);
#endif
			
		set_wrench_efforts_desired_v_curvature_and_gear();

//		printf("GEAR %d,  Previous gear %d, bh_state %s, back %d\n",
//				g_gear_command, previous_gear_command,
//				get_low_level_state_name(behavior_selector_low_level_state), behavior_selector_going_backwards);
		if (previous_gear_command != g_gear_command)
		{
//			printf("** GEAR %d,  Previous gear %d\n", g_gear_command, previous_gear_command);
			publish_ford_escape_gear_command(XGV_CCU);
		}
//		fflush(stdout);

		previous_gear_command = g_gear_command;

		delta_t = get_steering_delta_t();

		if (ford_escape_hybrid_config->use_mpc)
		{
			carmen_robot_ackerman_config_t robot_config;
			robot_config.understeer_coeficient = ford_escape_hybrid_config->understeer_coeficient2;
			robot_config.distance_between_front_and_rear_axles = ford_escape_hybrid_config->distance_between_front_and_rear_axles;
			robot_config.max_phi = ford_escape_hybrid_config->max_phi;

			if (g_go_state == 0)
				clear_current_motion_command_vector(ford_escape_hybrid_config->current_motion_command_vector, ford_escape_hybrid_config->nun_motion_commands);

			// TODO Tentar usar o phi cru direto da tork pra ver raw_phi
			g_steering_command = -carmen_libmpc_get_optimized_steering_effort_using_MPC(
					atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)),
					ford_escape_hybrid_config->current_motion_command_vector, ford_escape_hybrid_config->nun_motion_commands,
					ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->time_of_last_command,
					&robot_config, 0);

			//print_values_to_train_simulator(g_atan_desired_curvature, -atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)), delta_t, g_steering_command);


			///////////////////////// So para guardar os phi s para medir erro no modelo do carro
//			if (ford_escape_hybrid_config->nun_motion_commands > 0)
//			{
//				double motion_commands_vector_time = ford_escape_hybrid_config->current_motion_command_vector[0].time;
//				int j = 0;
//				while ((motion_commands_vector_time	< ford_escape_hybrid_config->XGV_v_and_phi_timestamp - ford_escape_hybrid_config->time_of_last_command) &&
//					   (j < ford_escape_hybrid_config->nun_motion_commands - 1))
//				{
//					j++;
//					motion_commands_vector_time += ford_escape_hybrid_config->current_motion_command_vector[j].time;
//				}
//
//				double
//				dist23(carmen_point_t v, carmen_point_t w)
//				{
//					return sqrt((carmen_square(v.x - w.x) + carmen_square(v.y - w.y)));
//				}
//
//				double L = ford_escape_hybrid_config->distance_between_front_and_rear_axles;
//				double delta_theta = global_pos.globalpos.theta - previous_global_pos.globalpos.theta;
//				double l = dist23(global_pos.globalpos, previous_global_pos.globalpos);
//				double real_phi = atan(L * (delta_theta / l));
//				printf("timestamp, x, y, theta, real_phi, phi_sent, phi_measured, v, "
//						"%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
//						ford_escape_hybrid_config->XGV_v_and_phi_timestamp,
//						global_pos.globalpos.x, global_pos.globalpos.y, global_pos.globalpos.theta,
//						real_phi, ford_escape_hybrid_config->current_motion_command_vector[j].phi, ford_escape_hybrid_config->filtered_phi,
//						ford_escape_hybrid_config->filtered_v);
//			}
			///////////////////////// Acima: So para guardar os phis para medir erro no modelo do carro
		}
		else
		{
			if (ford_escape_hybrid_config->use_rlpid)
			{
				// RL_PID
				g_steering_command = carmen_librlpid_compute_effort(-atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)), g_atan_desired_curvature, delta_t);
			}
			else
			{   // PID
				//g_steering_command = carmen_libpid_steering_PID_controler(g_atan_desired_curvature,
				//		-atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)), delta_t,
				//		g_XGV_component_status & XGV_MANUAL_OVERRIDE_FLAG);

				if (robot_model_name == ROBOT_NAME_FORD_ESCAPE)
				{
					// FUZZY
					// TODO Tentar usar o angulo direto da torc pra ver se melhora g_XGV_atan_curvature
					g_steering_command = carmen_libpid_steering_PID_controler_FUZZY(g_atan_desired_curvature,
							-atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)),
							delta_t, g_XGV_component_status & XGV_MANUAL_OVERRIDE_FLAG, ford_escape_hybrid_config->filtered_v);
				}
				else if ((robot_model_name == ROBOT_NAME_ECOTECH4) || (robot_model_name == ROBOT_NAME_MPW700) || (robot_model_name == ROBOT_NAME_ASTRU))
				{
					double plan_size;

					if (tune_pid_mode) //ford_escape_tune_pid nao preenche o x,y,theta. Falta tratar para o caso do Joystic tambem
						plan_size = 5.0;
					else if (ford_escape_hybrid_config->nun_motion_commands > 2) //calcula tamanho do plano para passar para funcao de PID.
						plan_size = DIST2D(ford_escape_hybrid_config->current_motion_command_vector[0], ford_escape_hybrid_config->current_motion_command_vector[ford_escape_hybrid_config->nun_motion_commands - 1]);
					else
						plan_size = 0.0;

					g_steering_command = carmen_libpid_steering_PID_controler(g_atan_desired_curvature,
							-atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)),
							plan_size, g_XGV_component_status & XGV_MANUAL_OVERRIDE_FLAG);
//					printf("Entrei aqui pra ver o PID e tudo, g_steering_command: %lf\n", g_steering_command);
				}
				else
					printf("ROBOT_MODEL_NAME %d wasnt recognized, check the code number\n "
							"Usage example: ford_escape_hybrid -robot_name [0 -> Ford_Escape | 1 -> ECOTECH4]", robot_model_name);
			}
			#ifdef PLOT_PHI
					pid_plot_phi(ford_escape_hybrid_config->filtered_phi, -get_phi_from_curvature(g_atan_desired_curvature, ford_escape_hybrid_config), 0.55, "phi");
			#endif
		}

		//Printf para testar a diferença das curvaturas original e modificada
//		printf("cphi %lf, dphi %lf\n", g_XGV_atan_curvature, -atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)));
//		printf(" v %lf, phi %lf, timestamp %lf\n", g_XGV_velocity, ford_escape_hybrid_config->filtered_phi, carmen_get_time());

		//////////////////////////////////////////  PRINTS VALUES TO FILE TO VERIFICATIONS  //////////////////////////////////////////
		//	fprintf(stdout, "(cc dc s v t) %lf %lf %lf %lf %lf\n", -atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config->filtered_v, ford_escape_hybrid_config)), g_atan_desired_curvature, g_steering_command, ford_escape_hybrid_config->filtered_v, carmen_get_time());
		//	fflush(stdout);
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		delta_t = get_velocity_delta_t();

		// PID VELOCITY
		carmen_libpid_velocity_PID_controler(&g_throttle_command, &g_brakes_command, &g_gear_command, g_desired_velocity,
				ford_escape_hybrid_config->filtered_v, delta_t, g_XGV_component_status & XGV_MANUAL_OVERRIDE_FLAG);

		#ifdef PLOT_VELOCITY
			pid_plot_velocity(ford_escape_hybrid_config->filtered_v, g_desired_velocity, 15.0, "vel");
		#endif

#ifdef LATENCY_TEST_PRINT
			//@@@VINICIUS Aqui envia o comando para o ojASTRU2 (a g_atan_desired nao bate com nenhuma mensagem pra baixo, sempre -0.00)
		fprintf(stdout, "ford->oj_effort (steering_command_ford->oj, dc_Ford->oj, t): %lf, %lf, %lf\n", g_steering_command, g_atan_desired_curvature, carmen_get_time());
		fflush(stdout);
#endif

//		printf("mandei pro jaus: %lf\n", g_steering_command);
		publish_ford_escape_steering_throttle_and_brakes_command(XGV_CCU);

		reportCurvatureMessageDestroy(reportCurvature);
	}
	else
	{
		carmen_warn("In torc_report_curvature_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(curvature_message));
	}
}


static void
torc_report_wheels_speed_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage wheels_speed_message)
{
	ReportWheelsSpeedMessage reportWheelsSpeed;

	reportWheelsSpeed = reportWheelsSpeedMessageFromJausMessage(wheels_speed_message);
	if (reportWheelsSpeed)
	{
		g_XGV_right_front_wheel_speed = reportWheelsSpeed->rightFront;
		g_XGV_left_front_wheel_speed = reportWheelsSpeed->leftFront;
		g_XGV_right_rear_wheel_speed = reportWheelsSpeed->rightRear;
		g_XGV_left_rear_wheel_speed = reportWheelsSpeed->leftRear;

#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
		FILE *caco = fopen("ford_dump.txt", "a");
		fprintf(caco, "%lf rightFront %lf leftFront %lf rightRear %lf leftRear %lf\n", carmen_get_time(),
				reportWheelsSpeed->rightFront, reportWheelsSpeed->leftFront, reportWheelsSpeed->rightRear, reportWheelsSpeed->leftRear);
		fflush(caco);
		fclose(caco);
#endif
		g_XGV_velocity_temp = (g_XGV_right_rear_wheel_speed + g_XGV_left_rear_wheel_speed) / 2.0;

//		printf("WHEELS (RF, LF, RR, LR, ts): %lf, %lf, %lf, %lf, %lf\n",
//				g_XGV_right_front_wheel_speed, g_XGV_left_front_wheel_speed,
//				g_XGV_right_rear_wheel_speed, g_XGV_left_rear_wheel_speed, carmen_get_time());

		reportWheelsSpeedMessageDestroy(reportWheelsSpeed);
	}
	else
	{
		carmen_warn("In torc_report_wheels_speed_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(wheels_speed_message));
	}
}


static void
torc_report_whrench_effort_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage whrench_effort_message)
{
	ReportWrenchEffortMessage reportWrenchEffort;

	reportWrenchEffort = reportWrenchEffortMessageFromJausMessage(whrench_effort_message);
	if (reportWrenchEffort)
	{
		g_XGV_throttle = reportWrenchEffort->propulsiveLinearEffortXPercent;
		g_XGV_steering = reportWrenchEffort->propulsiveRotationalEffortZPercent;
		g_XGV_brakes = reportWrenchEffort->resistiveLinearEffortXPercent;
#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
		FILE *caco = fopen("ford_dump.txt", "a");
		fprintf(caco, "%lf propulsiveLinearEffortXPercent %lf propulsiveRotationalEffortZPercent %lf resistiveLinearEffortXPercent %lf\n", carmen_get_time(),
				reportWrenchEffort->propulsiveLinearEffortXPercent, reportWrenchEffort->propulsiveRotationalEffortZPercent, reportWrenchEffort->resistiveLinearEffortXPercent);
		fflush(caco);
		fclose(caco);
#endif
		carmen_verbose("throttle %.2f\tsteering %.2f\tbrakes %.2f\n", g_XGV_throttle, g_XGV_steering, g_XGV_brakes);

		reportWrenchEffortMessageDestroy(reportWrenchEffort);
	}
	else
	{
		carmen_warn("In torc_report_whrench_effort_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(whrench_effort_message));
	}
}


static void
torc_report_discrete_devices_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage discrete_devices_message)
{
	ReportDiscreteDevicesMessage reportDiscreteDevices;

	reportDiscreteDevices = reportDiscreteDevicesMessageFromJausMessage(discrete_devices_message);
	if (reportDiscreteDevices)
	{
		g_XGV_main_propulsion = (int) reportDiscreteDevices->mainPropulsion;
		g_XGV_main_fuel_supply = (int) reportDiscreteDevices->mainFuelSupply;
		g_XGV_parking_brake = (int) reportDiscreteDevices->parkingBrake;
		g_XGV_gear = (int) reportDiscreteDevices->gear;
#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
		FILE *caco = fopen("ford_dump.txt", "a");
		fprintf(caco, "%lf mainPropulsion %d mainFuelSupply %d parkingBrake %d\n", carmen_get_time(),
				(int) reportDiscreteDevices->mainPropulsion, (int) reportDiscreteDevices->mainFuelSupply, (int) reportDiscreteDevices->parkingBrake);
		fflush(caco);
		fclose(caco);
#endif
		carmen_verbose("propulsion %d\tfuel_supply %d\tparking_brake %d\tgear %d\n",
				g_XGV_main_propulsion, g_XGV_main_fuel_supply,
				g_XGV_parking_brake, g_XGV_gear);

		reportDiscreteDevicesMessageDestroy(reportDiscreteDevices);
	}
	else
	{
		carmen_warn("In torc_report_discrete_devices_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(discrete_devices_message));
	}
}


static void
torc_report_signals_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage signals_message)
{
	ReportSignalsMessage reportSignals;

	reportSignals = reportSignalsMessageFromJausMessage(signals_message);
	if (reportSignals)
	{
		g_XGV_turn_signal = reportSignals->turnSignal;
		g_XGV_horn_status = reportSignals->hornStatus;
		g_XGV_headlights_status = reportSignals->headlightsStatus;
#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
		FILE *caco = fopen("ford_dump.txt", "a");
		fprintf(caco, "%lf turnSignal %d hornStatus %d headlightsStatus %d\n", carmen_get_time(),
				reportSignals->turnSignal, reportSignals->hornStatus, reportSignals->headlightsStatus);
		fflush(caco);
		fclose(caco);
#endif
		carmen_verbose("signal %d\thorn %d\theadlights %d\n", g_XGV_turn_signal, g_XGV_horn_status, g_XGV_headlights_status);

		reportSignalsMessageDestroy(reportSignals);
	}
	else
	{
		carmen_warn("In torc_report_signals_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(signals_message));
	}
}


static void
torc_report_component_status_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage component_status_message)
{
	ReportComponentStatusMessage reportComponentStatus;
	reportComponentStatus = reportComponentStatusMessageFromJausMessage(component_status_message);

	if (reportComponentStatus)
	{
		// bits 0-15: reserved, bits 16-31: see page 62 of ByWire XGV User Manual, Version 1.5.
		g_XGV_component_status = reportComponentStatus->secondaryStatusCode;
#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
		FILE *caco = fopen("ford_dump.txt", "a");
		fprintf(caco, "%lf secondaryStatusCode 0x%x\n", carmen_get_time(),
				reportComponentStatus->secondaryStatusCode);
		fflush(caco);
		fclose(caco);
#endif
		reportComponentStatusMessageDestroy(reportComponentStatus);
	}
	else
	{
		carmen_warn("In torc_report_component_status_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(component_status_message));
	}
}


static void
torc_report_error_count_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage error_message)
{
	ReportErrorCountMessage reportErrorCount;
	int i;

	reportErrorCount = reportErrorCountMessageFromJausMessage(error_message);
	if (reportErrorCount)
	{
		g_XGV_num_errors = reportErrorCount->numberOfErrors;
		for (i = 0; i < g_XGV_num_errors; i++)
		{
			g_XGV_error[i] = reportErrorCount->error[i];
			carmen_verbose("error[%d]: %d\t",i,g_XGV_error[i]);
		}
		carmen_verbose("\n");
#ifdef	FORD_ESCAPE_COMMUNICATION_DUMP
		FILE *caco = fopen("ford_dump.txt", "a");
		fprintf(caco, "%lf numberOfErrors %d\n", carmen_get_time(),
				reportErrorCount->numberOfErrors);
		fflush(caco);
		fclose(caco);
#endif
		reportErrorCountMessageDestroy(reportErrorCount);
		check_jaus_reported_errors();
	}
	else
	{
		carmen_warn("In torc_report_error_count_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(error_message));
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void 
read_parameters(int argc, char *argv[], ford_escape_hybrid_config_t *config)
{
	int num_items;

	carmen_param_t param_list[]= 
	{
		{"robot", "width", CARMEN_PARAM_DOUBLE, &(config->width), 1, NULL},
		{"robot", "length", CARMEN_PARAM_DOUBLE, &(config->length), 1, NULL},
		{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &(config->distance_between_front_and_rear_axles), 1, NULL},
//		{"robot", "understeer_coeficient", CARMEN_PARAM_DOUBLE, &(config->understeer_coeficient), 0, NULL},
		{"robot", "understeer_coeficient2", CARMEN_PARAM_DOUBLE, &(config->understeer_coeficient2), 0, NULL},
		{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &(config->max_phi), 1, NULL},
		{"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &(config->max_velocity), 1, NULL},
		{"robot", "max_velocity_reverse", CARMEN_PARAM_DOUBLE, &(config->max_velocity_reverse), 1, NULL},

		{"robot", "phi_multiplier", CARMEN_PARAM_DOUBLE, &phi_multiplier, 0, NULL},
		{"robot", "phi_bias", CARMEN_PARAM_DOUBLE, &phi_bias, 0, NULL},
		{"robot", "v_multiplier", CARMEN_PARAM_DOUBLE, &v_multiplier, 0, NULL},

		{"robot", "publish_odometry", CARMEN_PARAM_ONOFF, &(config->publish_odometry), 0, NULL},

		{"robot", "publish_combined_odometry", CARMEN_PARAM_ONOFF, &(publish_combined_odometry), 0, NULL},
		{"robot", "combine_odometry_phi", CARMEN_PARAM_INT, &(combine_odometry_phi), 0, NULL},
		{"robot", "combine_odometry_vel", CARMEN_PARAM_INT, &(combine_odometry_vel), 0, NULL},

		{"rrt",   "use_mpc",          CARMEN_PARAM_ONOFF, &(config->use_mpc), 0, NULL},
		{"rrt",   "use_rlpid",        CARMEN_PARAM_ONOFF, &(config->use_rlpid), 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

    carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline", (char *) "robot_model_name",	CARMEN_PARAM_INT, &(robot_model_name),0, NULL},
		{(char *) "commandline", (char *) "tune_pid_mode",		CARMEN_PARAM_ONOFF, &(tune_pid_mode),0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));
}


static void
subscribe_to_relevant_messages()
{
	if (publish_combined_odometry)
		carmen_subscribe_message(
			(char *) CARMEN_LIDAR_VELOCITY_NAME,
			(char *) CARMEN_LIDAR_VELOCITY_FMT,
			NULL, sizeof(carmen_robot_ackerman_velocity_message),
			(carmen_handler_t) carmen_lidar_velocity_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
			(char *) CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_go_message),
			(carmen_handler_t) navigator_ackerman_go_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
			(char *) CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_default_message),
			(carmen_handler_t) navigator_ackerman_stop_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_FORD_ESCAPE_SIGNAL_NAME,
			(char *) CARMEN_FORD_ESCAPE_SIGNAL_FMT,
			NULL, sizeof(carmen_ford_escape_signals_message),
			(carmen_handler_t) ford_escape_signals_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) base_ackerman_motion_command_message_handler, CARMEN_SUBSCRIBE_LATEST);

	// Handler redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
	//carmen_base_ackerman_subscribe_motion_command_2(NULL, (carmen_handler_t) (carmen_handler_t) base_ackerman_motion_command_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_message_handler,	CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
	if (publish_combined_odometry)
		carmen_visual_odometry_subscribe_pose6d_message(NULL, (carmen_handler_t) visual_odometry_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_task_manager_subscribe_desired_engage_state_message(NULL, (carmen_handler_t) task_manager_desired_engage_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(NULL, (carmen_handler_t) path_goals_and_annotations_message_handler, CARMEN_SUBSCRIBE_LATEST);

}


static int 
initialize_ipc(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_FORD_ESCAPE_STATUS_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_FORD_ESCAPE_STATUS_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_FORD_ESCAPE_STATUS_NAME);

	err = IPC_defineMsg(CARMEN_FORD_ESCAPE_ERROR_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_FORD_ESCAPE_ERROR_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_FORD_ESCAPE_ERROR_NAME);

	return 0;
}


static void
register_xgv_ccu_messages_handlers(OjCmpt XGV_CCU)
{
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_VELOCITY_STATE, torc_report_velocity_state_message_handler); 		// Ok
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_CURVATURE, torc_report_curvature_message_handler);				// Ok
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_WHEELS_SPEED, torc_report_wheels_speed_message_handler);			// Ok
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_WRENCH_EFFORT, torc_report_whrench_effort_message_handler);		// Ok
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_DISCRETE_DEVICES, torc_report_discrete_devices_message_handler);	// Ok
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_SIGNALS, torc_report_signals_message_handler);					// Ok
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_ERROR_COUNT, torc_report_error_count_message_handler);			// 4 (precisa implementar? acho que nao implementei direito no ojTorc (apenas a contagem eh mandada?))
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_COMPONENT_STATUS, torc_report_component_status_message_handler);	// Ok
}


static void
initialize_structures()
{
	// Init relevant data strutures
	ford_escape_hybrid_config = malloc(sizeof(ford_escape_hybrid_config_t));
	memset(ford_escape_hybrid_config, 0, sizeof(ford_escape_hybrid_config_t));
	ford_escape_hybrid_config->current_motion_command_vector = NULL;
	ford_escape_hybrid_config->XGV_v_and_phi_timestamp = carmen_get_time();

	memset(nun_motion_commands, 0, NUM_MOTION_COMMANDS_VECTORS * sizeof(int));

	memset(&alternative_odometry, 0, sizeof(carmen_robot_ackerman_velocity_message));
}


static void
send_default_signals_command()
{
	g_turn_signal_command = 0; // TODO: usar #define
	g_horn_status_command = 0; // TODO: usar #define
	g_headlights_status_command = 1; // TODO: usar #define
	publish_ford_escape_turn_horn_and_headlight_signals(XGV_CCU);

	g_gear_command = 128; // 128 = Neutral;
	publish_ford_escape_gear_command(XGV_CCU);
}


static void
initialize_jaus()
{
	//init TORC/JAUS
	XGV_CCU = create_xgv_ccu_component(XGV_CCU_NAME, XGV_CCU_COMPONENTE_ID, XGV_CCU_STATE_MACHINE_UPDATE_RATE);
	if (!XGV_CCU)
		exit(1);

	register_xgv_ccu_messages_handlers(XGV_CCU);
	ojCmptSetAuthority(XGV_CCU, 6);
	ojCmptRun(XGV_CCU);	// Begin running the XGV_CCU state machine

	send_default_signals_command();
	change_control_mode_to_wrench_efforts(XGV_CCU);
	xgv_ccu_service_connections = create_xgv_ccu_service_connections(XGV_CCU);

}


int 
main(int argc, char** argv)
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	initialize_structures();

	read_parameters(argc, argv, ford_escape_hybrid_config);
	//carmen_ford_escape_hybrid_read_pid_parameters(argc, argv);
	carmen_libpid_read_PID_parameters(argc, argv);
	
	initialize_ipc();

	subscribe_to_relevant_messages();

	initialize_jaus();

	carmen_ipc_addPeriodicTimer(FORD_ESCAPE_CYCLE_TIME, (TIMER_HANDLER_TYPE) publish_velocity_message, NULL);

	carmen_ipc_dispatch();

	return 0;
}
