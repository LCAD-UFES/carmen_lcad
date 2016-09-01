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

#include <jaus.h>				// Header file for JAUS types, structs and messages
#include <openJaus.h>				// Header file for the OpenJAUS specific C/C++ code base
#include <torc.h>
#include <torcInterface.h>
#include <pid.h>

#include "ford_escape_hybrid.h"


static ford_escape_hybrid_config_t *ford_escape_hybrid_config = NULL;

static carmen_ackerman_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_VECTORS][NUM_MOTION_COMMANDS_PER_VECTOR];
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

int g_go_state = 0;


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
get_curvature_from_phi(double phi, ford_escape_hybrid_config_t *ford_escape_hybrid_config)
{	// See Torc, "ByWire XGVTM User Manual", page 42
	double v;
	double curvature;
	
	v = ford_escape_hybrid_config->filtered_v;

	curvature = tan(phi / (1.0 + v * v * ford_escape_hybrid_config->understeer_coeficient)) / ford_escape_hybrid_config->distance_between_front_and_rear_axles;
	
	return (curvature);
}


double
get_phi_from_curvature(double curvature, ford_escape_hybrid_config_t *ford_escape_hybrid_config)
{
	double phi, v;
	
	v = ford_escape_hybrid_config->filtered_v;
	phi = atan(curvature * ford_escape_hybrid_config->distance_between_front_and_rear_axles) * (1.0 + v * v * ford_escape_hybrid_config->understeer_coeficient);
	
	return (phi);
}

double g_phi = 0.0;

static void
set_wrench_efforts_desired_v_and_curvature()
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
		phi = (1.0 + v / (6.94 / 0.3)) * ford_escape_hybrid_config->current_motion_command_vector[i].phi;
//		phi = ford_escape_hybrid_config->current_motion_command_vector[i].phi;
	}
	else
	{
		v = 0.0;
		phi = 0.0;
	}
	g_phi = phi / (1.0 + v / (6.94 / 0.3));
//	g_phi = phi;

	// The function carmen_ford_escape_hybrid_steering_PID_controler() uses g_atan_desired_curvature to compute the g_steering_command that is sent to the car.
	// This function is called when new info about the current measured velocity (g_XGV_velocity) arrives from the car via Jaus messages handled
	// by the torc_report_velocity_state_message_handler() callback function.
	g_atan_desired_curvature = -atan(get_curvature_from_phi(phi, ford_escape_hybrid_config)); // @@@ Alberto: a curvatura do carro ee ao contrario de carmen

	// The function carmen_ford_escape_hybrid_velocity_PID_controler() uses g_desired_velocity to compute the g_throttle_command, g_brakes_command and g_gear_command
	// that are sent to the car.
	// This function is called when new info about the current velocity (g_XGV_velocity) arrives from the car via Jaus messages handled
	// by the torc_report_velocity_state_message_handler() callback function.
	g_desired_velocity = v;
}



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_car_status()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_ford_escape_status_message msg;

	msg.g_XGV_velocity = g_XGV_velocity;
	msg.g_XGV_atan_curvature = g_XGV_atan_curvature;
	msg.g_XGV_brakes = g_XGV_brakes;
	msg.g_XGV_gear = g_XGV_gear;
	msg.g_XGV_headlights_status = g_XGV_headlights_status;
	msg.g_XGV_horn_status = g_XGV_horn_status;
	msg.g_XGV_right_front_wheel_speed = g_XGV_right_front_wheel_speed;
	msg.g_XGV_left_front_wheel_speed = g_XGV_left_front_wheel_speed;
	msg.g_XGV_right_rear_wheel_speed = g_XGV_right_rear_wheel_speed;
	msg.g_XGV_left_rear_wheel_speed = g_XGV_left_rear_wheel_speed;
	msg.g_XGV_main_fuel_supply = g_XGV_main_fuel_supply;
	msg.g_XGV_main_propulsion = g_XGV_main_propulsion;
	msg.g_XGV_parking_brake = g_XGV_parking_brake;
	msg.g_XGV_steering = g_XGV_steering;
	msg.g_XGV_throttle = g_XGV_throttle;
	msg.g_XGV_turn_signal = g_XGV_turn_signal;

//	g_XGV_component_status bit Interpretation F: disengaged, T: engaged (see page 62 of ByWire XGV User Manual, Version 1.5)
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
//	printf("g_XGV_component_status = 0x%x\n", g_XGV_component_status);
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_FORD_ESCAPE_STATUS_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_FORD_ESCAPE_STATUS_NAME);
}


static void
publish_car_error()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_ford_escape_error_message msg;
	int i;

	msg.num_errors = g_XGV_num_errors;
	msg.error = calloc(msg.num_errors,sizeof(int));
	for (i = 0; i < msg.num_errors; i++)
	{
		msg.error[i] = g_XGV_error[i];
	}
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_FORD_ESCAPE_ERROR_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_FORD_ESCAPE_ERROR_NAME);
}


static void
publish_velocity_message(void *clientData __attribute__ ((unused)), unsigned long currentTime __attribute__ ((unused)), unsigned long scheduledTime __attribute__ ((unused)))
{
	// This function publish the low level velocity and steering angle from the car

	IPC_RETURN_TYPE err = IPC_OK;
	carmen_robot_ackerman_velocity_message robot_ackerman_velocity_message;
	static int heartbeat = 0;

	robot_ackerman_velocity_message.v = g_XGV_velocity;
	robot_ackerman_velocity_message.phi = get_phi_from_curvature(-tan(g_XGV_atan_curvature), ford_escape_hybrid_config);
	robot_ackerman_velocity_message.timestamp = carmen_get_time(); // @@ Alberto: era igual a = ford_escape_hybrid_config->XGV_v_and_phi_timestamp;
	robot_ackerman_velocity_message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, &robot_ackerman_velocity_message);
	carmen_test_ipc(err, "Could not publish ford_escape_hybrid message named carmen_robot_ackerman_velocity_message", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);

	if ((heartbeat % 10) == 0)
	{
		publish_car_status();
		publish_car_error();
	}

	heartbeat++;
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Jaus Publishers                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
publish_ford_escape_throttle_and_brakes_command(OjCmpt XGV_CCU)
{
	send_set_wrench_efforts_message(XGV_CCU);
}


void
publish_ford_escape_turn_horn_and_headlight_signals(OjCmpt XGV_CCU)
{
	send_set_signals_message(XGV_CCU);
}


void
publish_ford_escape_gear_command(OjCmpt XGV_CCU)
{
	send_set_discrete_devices_message(XGV_CCU);
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
base_ackerman_motion_command_message_handler(carmen_base_ackerman_motion_command_message *motion_command_message)
{
	int num_motion_commands, i;

	current_motion_command_vetor = (current_motion_command_vetor + 1) % NUM_MOTION_COMMANDS_VECTORS;

	num_motion_commands = fmin(NUM_MOTION_COMMANDS_PER_VECTOR, motion_command_message->num_motion_commands);

	for (i = 0; i < num_motion_commands; i++)
		motion_commands_vector[current_motion_command_vetor][i] = motion_command_message->motion_command[i];

	nun_motion_commands[current_motion_command_vetor] = num_motion_commands;

	ford_escape_hybrid_config->current_motion_command_vector = motion_commands_vector[current_motion_command_vetor];
	ford_escape_hybrid_config->nun_motion_commands = nun_motion_commands[current_motion_command_vetor];
	ford_escape_hybrid_config->time_of_last_command = motion_command_message->timestamp;
}


static void
fused_odometry_message_handler(carmen_fused_odometry_message *fused_odometry_message)
{
	ford_escape_hybrid_config->filtered_pitch = fused_odometry_message->pose.orientation.pitch;
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

	publish_ford_escape_turn_horn_and_headlight_signals(XGV_CCU);
}


static void
navigator_ackerman_go_message_handler()
{
	change_control_mode_to_wrench_efforts(XGV_CCU);

	g_gear_command = 1; // Drive forward
	publish_ford_escape_gear_command(XGV_CCU);

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
		// g_XGV_velocity = reportVelocityState->velocityXMps;
		
		reportVelocityStateMessageDestroy(reportVelocityState);
	}
	else
	{
		carmen_warn("In torc_report_velocity_state_message_handler(): Error unpacking %s message.\n", jausMessageCommandCodeString(velocity_message));
	}
}


static void
torc_report_curvature_message_handler(OjCmpt XGV_CCU __attribute__ ((unused)), JausMessage curvature_message)
{
	ReportCurvatureMessage reportCurvature;
	double raw_phi;
	double delta_t;
	int previous_gear_command;

	reportCurvature = reportCurvatureMessageFromJausMessage(curvature_message);
	if (reportCurvature)
	{
		g_XGV_atan_curvature = reportCurvature->atanOfCurrentCurvature; // @@@ Alberto: a curvatura do carro vem ao contrario de carmen

		ford_escape_hybrid_config->XGV_v_and_phi_timestamp = carmen_get_time();

		raw_phi = get_phi_from_curvature(-tan(g_XGV_atan_curvature), ford_escape_hybrid_config);
		carmen_add_bias_and_multiplier_to_v_and_phi(&(ford_escape_hybrid_config->filtered_v), &(ford_escape_hybrid_config->filtered_phi), 
						    g_XGV_velocity, raw_phi, 
						    0.0, v_multiplier, phi_bias, phi_multiplier);
			
		set_wrench_efforts_desired_v_and_curvature();
		delta_t = get_steering_delta_t();

		//carmen_ford_escape_hybrid_steering_PID_controler
		carmen_libpid_steering_PID_controler(&g_steering_command, g_atan_desired_curvature,
				-atan(get_curvature_from_phi(ford_escape_hybrid_config->filtered_phi, ford_escape_hybrid_config)), delta_t);


		previous_gear_command = g_gear_command;

		//printf("%lf %lf %lf\n", carmen_radians_to_degrees(ford_escape_hybrid_config->filtered_phi), carmen_radians_to_degrees(g_phi), carmen_radians_to_degrees(ford_escape_hybrid_config->filtered_phi - g_phi));

		delta_t = get_velocity_delta_t();

		//carmen_ford_escape_hybrid_velocity_PID_controler
		carmen_libpid_velocity_PID_controler(&g_throttle_command, &g_brakes_command, &g_gear_command,
			g_desired_velocity, ford_escape_hybrid_config->filtered_v, delta_t);

		if (previous_gear_command != g_gear_command)
			publish_ford_escape_gear_command(XGV_CCU);
			
		publish_ford_escape_throttle_and_brakes_command(XGV_CCU);

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

		g_XGV_velocity = (g_XGV_right_rear_wheel_speed + g_XGV_left_rear_wheel_speed) / 2.0;

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
		reportErrorCountMessageDestroy(reportErrorCount);
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
		{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &(config->distance_between_front_and_rear_axles), 1,NULL},

		{"robot", "phi_multiplier", CARMEN_PARAM_DOUBLE, &phi_multiplier, 0, NULL},
		{"robot", "phi_bias", CARMEN_PARAM_DOUBLE, &phi_bias, 0, NULL},
		{"robot", "v_multiplier", CARMEN_PARAM_DOUBLE, &v_multiplier, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


static void
subscribe_to_relevant_messages()
{
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
	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_message_handler,	CARMEN_SUBSCRIBE_LATEST);
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
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_VELOCITY_STATE, torc_report_velocity_state_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_CURVATURE, torc_report_curvature_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_WHEELS_SPEED, torc_report_wheels_speed_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_WRENCH_EFFORT, torc_report_whrench_effort_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_DISCRETE_DEVICES, torc_report_discrete_devices_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_SIGNALS, torc_report_signals_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_ERROR_COUNT, torc_report_error_count_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_COMPONENT_STATUS, torc_report_component_status_message_handler);
}


static void
initialize_structures()
{
	// Init relevant data strutures
	ford_escape_hybrid_config = malloc(sizeof(ford_escape_hybrid_config_t));
	memset(ford_escape_hybrid_config, 0, sizeof(ford_escape_hybrid_config_t));
	ford_escape_hybrid_config->current_motion_command_vector = NULL;
	ford_escape_hybrid_config->XGV_v_and_phi_timestamp = carmen_get_time();

	memset(nun_motion_commands, 0, NUM_MOTION_COMMANDS_VECTORS);
}


static void
default_signals_command()
{
	g_turn_signal_command = 3; // TODO: usar #define
	g_horn_status_command = 0; // TODO: usar #define
	g_headlights_status_command = 1; // TODO: usar #define
	publish_ford_escape_turn_horn_and_headlight_signals(XGV_CCU);

	g_gear_command = 1; // gear = Low // 2;
	publish_ford_escape_gear_command(XGV_CCU);
}


static void
initialize_jaus()
{
	//init TORC/JAUS
	XGV_CCU = create_xgv_ccu_component(XGV_CCU_NAME, XGV_CCU_COMPONENTE_ID, XGV_CCU_STATE_MACHINE_UPDATE_RATE);
	register_xgv_ccu_messages_handlers(XGV_CCU);
	ojCmptSetAuthority(XGV_CCU, 6);
	ojCmptRun(XGV_CCU);	// Begin running the XGV_CCU state machine

	default_signals_command();
	change_control_mode_to_wrench_efforts(XGV_CCU);
	xgv_ccu_service_connections = create_xgv_ccu_service_connections(XGV_CCU);

}
//////////////////////////////////////////////////////////////////////////////////////////////////


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

	carmen_ipc_addPeriodicTimer(1.0 / 40.0, (TIMER_HANDLER_TYPE) publish_velocity_message, NULL);

	carmen_ipc_dispatch();

	return 0;
}
