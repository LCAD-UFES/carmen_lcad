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
#include <carmen/map_server_interface.h>
#include <carmen/route_planner_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/task_manager_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/ford_escape_hybrid_interface.h>

#include <control.h>

#include "simulator_ackerman.h"
#include "simulator_ackerman_simulation.h"
#include "simulator_ackerman_messages.h"

#ifdef __USE_RL_CONTROL
#include <rlcontrol_interface.h>
double time_last_message_arrived = 0;
#endif

//#define ADD_BIAS_TO_BASE_ACKERMAN

#include "objects_ackerman.h"

static carmen_simulator_ackerman_config_t simulator_conf;
static carmen_simulator_ackerman_config_t *simulator_config;

static int use_robot = 1;

static carmen_localize_ackerman_initialize_message init_msg;

//static int current_motion_command_vetor = 0;
//static carmen_ackerman_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_VECTORS][NUM_MOTION_COMMANDS_PER_VECTOR];
//static int nun_motion_commands[NUM_MOTION_COMMANDS_VECTORS];
static int publish_laser_flag = 0;
static int publish_detection_moving_objects = 0;

static int necessary_maps_available = 0;

static int use_truepos = 0;
static int use_velocity_nn = 1;
static int use_phi_nn = 1;

static int simulate_legacy_500 = 0;
static int connected_to_iron_bird = 0;
static int use_external_true_pose = 0;
static double iron_bird_v = 0.0;
static double iron_bird_phi = 0.0;

double global_steer_kp = 0.0;
double global_steer_kd = 0.0;
double global_steer_ki = 0.0;

double global_vel_kp = 0.0;
double global_vel_kd = 0.0;
double global_vel_ki = 0.0;

carmen_route_planner_road_network_message *road_network_message = NULL;
int autonomous = 0;

carmen_behavior_selector_low_level_state_t behavior_selector_low_level_state = Stopped;

int g_XGV_horn_status = 0;


//static void
//print_path(carmen_robot_and_trailer_motion_command_t *path, int size)
//{
//	for (int i = 0; (i < size) && (i < 15); i++)
//	{
//		printf("v %5.3lf, phi %5.3lf, t %5.3lf, x %5.3lf, y %5.3lf, theta %5.3lf\n",
//				path[i].v, path[i].phi, path[i].time,
//				path[i].x, path[i].y,
//				path[i].theta);
//	}
//	printf("\n");
//}


int
apply_system_latencies(carmen_robot_and_trailers_motion_command_t *current_motion_command_vector, int nun_motion_commands)
{
	int i, j;

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.2; j++)
		{
			if (j >= nun_motion_commands)
				break;
			lat += current_motion_command_vector[j].time;
		}
		if (j >= nun_motion_commands)
			break;
		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
//		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
//		current_motion_command_vector[i].v = current_motion_command_vector[j].v;
//		current_motion_command_vector[i].x = current_motion_command_vector[j].x;
//		current_motion_command_vector[i].y = current_motion_command_vector[j].y;
//		current_motion_command_vector[i].theta = current_motion_command_vector[j].theta;
//		current_motion_command_vector[i].time = current_motion_command_vector[j].time;
	}

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.6; j++)
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



carmen_moving_objects_point_clouds_message
build_moving_objects_message(int num_objects, carmen_simulator_ackerman_objects_message objects_sim)
{

	carmen_moving_objects_point_clouds_message msg;

//	vector<object> tmp_predictions = predictions;
//
//	for (int i=0; i<simulated_objects.size(); i++)
//	{
//		if (simulated_objects[i].active)
//		{
//			tmp_predictions.push_back(simulated_objects[i].p);
//			image_cartesian ic;
//			vector<image_cartesian> vic;
//			vic.push_back(ic);
//			points_lists.push_back(vic);
//		}
//	}
//	int num_objects = compute_num_measured_objects(tmp_predictions);

	//printf ("Predictions %d Poses %d, Points %d\n", (int) predictions.size(), (int) objects_poses.size(), (int) points_lists.size());

	msg.num_point_clouds = num_objects;
	msg.point_clouds = (t_point_cloud_struct *) malloc (num_objects * sizeof(t_point_cloud_struct));
	int real_number_of_objects = 0;
	for (int i = 0, l = 0; i < num_objects; i++)
	{
		if (objects_sim.objects[i].type == CARMEN_SIMULATOR_ACKERMAN_PERSON || publish_detection_moving_objects)
		{
			msg.point_clouds[l].r = 1.0;
			msg.point_clouds[l].g = 1.0;
			msg.point_clouds[l].b = 0.0;

			msg.point_clouds[l].linear_velocity = objects_sim.objects[i].v;
			msg.point_clouds[l].orientation = objects_sim.objects[i].theta;

			// printf("%lf %lf\n", tmp_predictions[i].orientation, last_globalpos->globalpos.theta);

			msg.point_clouds[l].width  = 1.0;
			msg.point_clouds[l].length = 1.0;
			msg.point_clouds[l].height = 2.0;

			msg.point_clouds[l].object_pose.x = objects_sim.objects[i].x;
			msg.point_clouds[l].object_pose.y = objects_sim.objects[i].y;
			msg.point_clouds[l].object_pose.z = 0.0;


			msg.point_clouds[l].geometric_model = 0;
			msg.point_clouds[l].model_features.geometry.width  = 1.0;
			msg.point_clouds[l].model_features.geometry.length = 1.0;
			msg.point_clouds[l].model_features.geometry.height = 2.0;
			msg.point_clouds[l].model_features.red = 1.0;
			msg.point_clouds[l].model_features.green = 1.0;
			msg.point_clouds[l].model_features.blue = 0.8;
			msg.point_clouds[l].model_features.model_name = (char *) "pedestrian";

			if (publish_detection_moving_objects)
			{
				if (objects_sim.objects[i].type == CARMEN_SIMULATOR_ACKERMAN_RANDOM_OBJECT)
					msg.point_clouds[l].model_features.model_name = (char *) "Random Object";
				if (objects_sim.objects[i].type == CARMEN_SIMULATOR_ACKERMAN_LINE_FOLLOWER)
					msg.point_clouds[l].model_features.model_name = (char *) "Line Follower";
				if (objects_sim.objects[i].type == CARMEN_SIMULATOR_ACKERMAN_OTHER_ROBOT)
					msg.point_clouds[l].model_features.model_name = (char *) "Other Robot";
				if (objects_sim.objects[i].type == CARMEN_SIMULATOR_ACKERMAN_BIKE)
					msg.point_clouds[l].model_features.model_name = (char *) "Bike";
				if (objects_sim.objects[i].type == CARMEN_SIMULATOR_ACKERMAN_CAR)
					msg.point_clouds[l].model_features.model_name = (char *) "Car";
				if (objects_sim.objects[i].type == CARMEN_SIMULATOR_ACKERMAN_TRUCK)
					msg.point_clouds[l].model_features.model_name = (char *) "Truck";
			}
			msg.point_clouds[l].num_associated = i;

			msg.point_clouds[l].point_size = num_objects;

			msg.point_clouds[l].points = (carmen_vector_3D_t *) malloc (msg.point_clouds[l].point_size * sizeof(carmen_vector_3D_t));

			for (int j = 0; j < num_objects; j++)
			{
				carmen_vector_3D_t p;

				p.x = 0;
				p.y = 0;
				p.z = 0;
				msg.point_clouds[l].points[j] = p;
			}
			l++;
			real_number_of_objects++;
		}
	}
	msg.num_point_clouds = real_number_of_objects;

	return (msg);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_odometry(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static carmen_base_ackerman_odometry_message odometry;
	static int first = 1;

	if (first)
	{
		odometry.host = carmen_get_host();
		odometry.x = 0;
		odometry.y = 0;
		odometry.theta = 0;

		odometry.v = odometry.phi = 0;
		first = 0;
	}

	if (!simulate_legacy_500)
	{
#ifdef ADD_BIAS_TO_BASE_ACKERMAN
		carmen_base_ackerman_config_t *car_config
		if(use_velocity_nn)
			odometry_v = raw_v * v_multiplier + v_bias;
		if(use_phi_nn)
			odometry_phi = raw_phi * phi_multiplier + phi_bias;
		car_config.odometry = simulator_config
		carmen_simulator_ackerman_recalc_pos(car_config)
#endif

		odometry.x     = simulator_config->odom_pose.x;
		odometry.y     = simulator_config->odom_pose.y;
		odometry.theta = simulator_config->odom_pose.theta;
		odometry.v     = simulator_config->v;
		odometry.phi   = simulator_config->phi;
		odometry.timestamp = timestamp;

		// Simula problema de odometria do MPW
//		double mpw_v = round(odometry.v * 3.6) / 3.6;
//		odometry.v = mpw_v;
//		double mpw_phi = carmen_degrees_to_radians(round(carmen_radians_to_degrees(odometry.phi) * (40.0 / 90.0)) / (90.0 / 40.0));
//		odometry.phi = mpw_phi;

		err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &odometry);
		carmen_test_ipc(err, "Could not publish base_odometry_message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
	}
	else if (simulate_legacy_500 && !connected_to_iron_bird)
	{
		carmen_robot_ackerman_velocity_message robot_ackerman_velocity_message;
		robot_ackerman_velocity_message.v = simulator_config->v;
		robot_ackerman_velocity_message.phi = simulator_config->phi;
		robot_ackerman_velocity_message.timestamp = timestamp;
		robot_ackerman_velocity_message.host = carmen_get_host();

		err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_VELOCITY_2_NAME, &robot_ackerman_velocity_message);
		carmen_test_ipc(err, "Could not publish ford_escape_hybrid message named carmen_robot_ackerman_velocity_message", CARMEN_ROBOT_ACKERMAN_VELOCITY_2_NAME);
	}
}


static void
publish_truepos(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static carmen_simulator_ackerman_truepos_message truepos;
	static int first = 1;
	if (first)
	{
		truepos.host = carmen_get_host();
		first = 0;
	}

	truepos.truepose = simulator_config->true_pose;
	truepos.odometrypose = simulator_config->odom_pose;
	truepos.num_trailers = 1;
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		truepos.trailer_theta[z] = truepos.truepose.theta;

	truepos.v = simulator_config->v;
	truepos.phi = simulator_config->phi;
	truepos.timestamp = timestamp;

	err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, &truepos);
	carmen_test_ipc(err, "Could not publish simualator_truepos_message", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME);
}


void
publish_moving_objects_message(carmen_moving_objects_point_clouds_message *msg)
{
	msg->timestamp = carmen_get_time();
	msg->host = carmen_get_host();

    carmen_moving_objects_point_clouds_publish_message_generic(0, msg);
}


static void
publish_objects(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static int first = 1;
	static carmen_simulator_ackerman_objects_message objects;

	if (first)
	{
		objects.host = carmen_get_host();
		first = 0;
	}

//	carmen_simulator_ackerman_get_object_poses(&(objects.num_objects), &(objects.objects_list));
	carmen_simulator_ackerman_get_objects(&(objects.num_objects), &(objects.objects));
	objects.timestamp = timestamp;

	if((objects.num_objects) != 0)
	{
		carmen_moving_objects_point_clouds_message msg = build_moving_objects_message(objects.num_objects, objects);
		publish_moving_objects_message(&msg);
	}
	else
	{
		carmen_moving_objects_point_clouds_message msg;
		msg.num_point_clouds = 0;
		publish_moving_objects_message(&msg);
	}

	err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_OBJECTS_NAME, &objects);
	carmen_test_ipc(err, "Could not publish simulator_objects_message",
			CARMEN_SIMULATOR_ACKERMAN_OBJECTS_NAME);

}


static void
publish_frontlaser(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static int first = 1;
	static carmen_laser_laser_message flaser;

	if (!simulator_config->use_front_laser)
		return;

	if (first)
	{
		flaser.host = carmen_get_host();
		flaser.num_readings =
				simulator_config->front_laser_config.num_lasers;
		flaser.range = (double *)calloc
				(simulator_config->front_laser_config.num_lasers, sizeof(double));
		carmen_test_alloc(flaser.range);

		flaser.num_remissions = 0;
		flaser.remission = 0;

		first = 0;
	}

	carmen_simulator_ackerman_calc_laser_msg(&flaser, simulator_config, 0);

	flaser.timestamp = timestamp;
	err = IPC_publishData(CARMEN_LASER_FRONTLASER_NAME, &flaser);
	carmen_test_ipc(err, "Could not publish laser_frontlaser_message",
			CARMEN_LASER_FRONTLASER_NAME);
}


static void
publish_rearlaser(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static int first = 1;
	static carmen_laser_laser_message rlaser;

	if (!simulator_config->use_rear_laser)
		return;

	if (first)
	{
		rlaser.host = carmen_get_host();

		rlaser.num_readings = simulator_config->rear_laser_config.num_lasers;
		rlaser.range = (double *) calloc(simulator_config->rear_laser_config.num_lasers, sizeof(double));
		carmen_test_alloc(rlaser.range);

		rlaser.num_remissions = 0;
		rlaser.remission = 0;
		first = 0;
	}

	carmen_simulator_ackerman_calc_laser_msg(&rlaser, simulator_config, 1);

	rlaser.timestamp = timestamp;
	err = IPC_publishData(CARMEN_LASER_REARLASER_NAME, &rlaser);
	carmen_test_ipc(err, "Could not publish laser_rearlaser_message", CARMEN_LASER_REARLASER_NAME);

}


static void
publish_ford_escape_status_message()
{
	static double previous_timestamp = 0.0;
	double current_timestamp = carmen_get_time();

	if ((current_timestamp - previous_timestamp) > (1.0 / 5.0))
	{
		carmen_ford_escape_status_message msg;
		memset(&msg, 0, sizeof(carmen_ford_escape_status_message));

		msg.g_XGV_horn_status = g_XGV_horn_status;

		carmen_ford_escape_publish_status_message(&msg, current_timestamp);

		previous_timestamp = current_timestamp;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
simulate_car_and_publish_readings(void *clientdata __attribute__ ((unused)),
		unsigned long currenttime __attribute__ ((unused)),
		unsigned long scheduledTime __attribute__ ((unused)))
{
	double delta_time;
	double timestamp;
	static double last_timestamp = 0.0;

	timestamp = carmen_get_time();
	if (last_timestamp == 0.0)
	{
		last_timestamp = timestamp;
		return;
	}

	simulator_config->delta_t = timestamp - last_timestamp;

	if (!simulator_config->sync_mode)
	{
		delta_time = timestamp - simulator_config->time_of_last_command;
		if ((simulator_config->v > 0 || simulator_config->phi > 0) && (delta_time > simulator_config->motion_timeout))
		{
			simulator_config->current_motion_command_vector = NULL;
			simulator_config->nun_motion_commands = 0;
			simulator_config->current_motion_command_vector_index = 0;
			simulator_config->target_v = 0;
			simulator_config->target_phi = 0;
		}
	}

	// Existem duas versoes dessa funcao, uma em base_ackerman_simulation e
	// outra em simulator_ackerman_simulation.
	if (!use_external_true_pose)
	{
		carmen_simulator_ackerman_recalc_pos(simulator_config, use_velocity_nn, use_phi_nn, connected_to_iron_bird, iron_bird_v, iron_bird_phi);
		// update beta
	}
	else
		update_target_v_and_target_phi(simulator_config);

	if (!use_truepos)
	{
		publish_odometry(timestamp);
		carmen_simulator_ackerman_update_objects(simulator_config);
		publish_objects(timestamp);
	}
	publish_truepos(timestamp);

	static unsigned int counter = 0;
	if (publish_laser_flag && ((counter % 2) == 0) && !use_truepos)
	{
//		pid_plot_phi(simulator_config->phi, simulator_config->target_phi, 0.55, "phi");
		publish_frontlaser(timestamp);
		publish_rearlaser(timestamp);
	}
	counter++;

	publish_ford_escape_status_message();

	carmen_publish_heartbeat("simulator");

	last_timestamp = timestamp;

#ifdef __USE_RL_CONTROL
	if (timestamp - time_last_message_arrived > 0.5)
	{
		set_rl_control(0,0,100);
	}
#endif
}


/*
static void
motion_command_handler(carmen_base_ackerman_motion_command_message *motion_command_message)
{
	int num_motion_commands, i;

	if (!necessary_maps_available)
		return;

//	printf("delay %lf\n", carmen_get_time() - motion_command_message->timestamp);

	current_motion_command_vetor = (current_motion_command_vetor + 1) % NUM_MOTION_COMMANDS_VECTORS;

	if (motion_command_message->num_motion_commands < NUM_MOTION_COMMANDS_PER_VECTOR)
		num_motion_commands = motion_command_message->num_motion_commands;
	else
		num_motion_commands = NUM_MOTION_COMMANDS_PER_VECTOR;

	for (i = 0; i < num_motion_commands; i++)
		motion_commands_vector[current_motion_command_vetor][i] = motion_command_message->motion_command[i];
	nun_motion_commands[current_motion_command_vetor] = num_motion_commands;

	simulator_config->current_motion_command_vector = motion_commands_vector[(NUM_MOTION_COMMANDS_VECTORS + current_motion_command_vetor - 1) % NUM_MOTION_COMMANDS_VECTORS];
	simulator_config->nun_motion_commands = nun_motion_commands[(NUM_MOTION_COMMANDS_VECTORS + current_motion_command_vetor - 1) % NUM_MOTION_COMMANDS_VECTORS];
	simulator_config->time_of_last_command = motion_command_message->timestamp;

	if (simulator_config->use_mpc)
		simulator_config->nun_motion_commands = apply_system_latencies(simulator_config->current_motion_command_vector, simulator_config->nun_motion_commands);
}
*/

static void
motion_command_handler(carmen_base_ackerman_motion_command_message *motion_command_message)
{
	int num_motion_commands;

	if (!necessary_maps_available)
		return;

//	print_path(motion_command_message->motion_command, motion_command_message->num_motion_commands);

	if (motion_command_message->num_motion_commands < NUM_MOTION_COMMANDS_PER_VECTOR)
		num_motion_commands = motion_command_message->num_motion_commands;
	else
		num_motion_commands = NUM_MOTION_COMMANDS_PER_VECTOR;

	simulator_config->current_motion_command_vector = motion_command_message->motion_command;
	simulator_config->nun_motion_commands = num_motion_commands;
	simulator_config->time_of_last_command = motion_command_message->timestamp;

	if (simulator_config->use_mpc)
		simulator_config->nun_motion_commands = apply_system_latencies(simulator_config->current_motion_command_vector, simulator_config->nun_motion_commands);
}


static void
set_object_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_simulator_ackerman_set_object_message msg;
	FORMATTER_PTR formatter;

	if (!necessary_maps_available)
		return;

	formatter = IPC_msgInstanceFormatter(msgRef);
	IPC_unmarshallData(formatter, callData, &msg, sizeof(carmen_simulator_ackerman_set_object_message));
	IPC_freeByteArray(callData);

	carmen_simulator_ackerman_create_object(msg.pose.x, msg.pose.y, msg.pose.theta,	msg.type, msg.speed);

	FILE *moving_objects_file = fopen("moving_objects_file.txt", "a");
	fprintf(moving_objects_file, "%d %lf %lf %lf %lf\n", msg.type, msg.pose.x, msg.pose.y, msg.pose.theta, msg.speed);
	fclose(moving_objects_file);
}


static void
set_truepose_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_simulator_ackerman_set_truepose_message msg;
	FORMATTER_PTR formatter;

	if (!necessary_maps_available)
		return;

	formatter = IPC_msgInstanceFormatter(msgRef);
	IPC_unmarshallData(formatter, callData, &msg, sizeof(carmen_simulator_ackerman_set_truepose_message));
	IPC_freeByteArray(callData);

	simulator_config->true_pose.x = msg.pose.x;
	simulator_config->true_pose.y = msg.pose.y;
	simulator_config->true_pose.theta = msg.pose.theta;
	simulator_config->odom_pose.theta = msg.pose.theta;
	simulator_config->phi = simulator_config->target_phi = 0.0;
	simulator_config->v = simulator_config->target_v = 0.0;
	simulator_config->initialize_neural_networks = 1;
}


static void
clear_objects_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	carmen_simulator_ackerman_clear_objects_message msg;
	FORMATTER_PTR formatter;

	if (!necessary_maps_available)
		return;

	formatter = IPC_msgInstanceFormatter(msgRef);
	IPC_unmarshallData(formatter, callData, &msg, sizeof(carmen_default_message));
	IPC_freeByteArray(callData);

	// carmen_simulator_ackerman_clear_objects();
	carmen_simulator_objects_clear_objects();
}


static void
localize_initialize_message_handler(carmen_localize_ackerman_initialize_message *init_msg)
{
	if (!necessary_maps_available)
		return;

	if (init_msg->distribution == CARMEN_INITIALIZE_GAUSSIAN)
	{
		simulator_config->true_pose.x = init_msg->mean[0].x;
		simulator_config->true_pose.y = init_msg->mean[0].y;
		simulator_config->true_pose.theta = init_msg->mean[0].theta;
		simulator_config->odom_pose.theta = init_msg->mean[0].theta;
		if (!use_truepos)
		{
			simulator_config->v = 0;
			simulator_config->phi = 0;
			simulator_config->target_v = 0;
			simulator_config->target_phi = 0;
		}
		else
		{
			simulator_config->v = simulator_config->global_pos.v;
			simulator_config->phi = simulator_config->global_pos.phi;
			simulator_config->target_v = simulator_config->global_pos.v;
			simulator_config->target_phi = simulator_config->global_pos.phi;
		}
		simulator_config->current_motion_command_vector = NULL;
		simulator_config->nun_motion_commands = 0;
		simulator_config->current_motion_command_vector_index = 0;
		simulator_config->initialize_neural_networks = 1;
	}
}


void
offline_map_update_handler(carmen_map_server_offline_map_message *offline_map_message)
{
	if (!necessary_maps_available)
		simulator_config->map.map = (double **) malloc(offline_map_message->config.x_size * sizeof(double *));

	simulator_config->map.config = offline_map_message->config;
	simulator_config->map.complete_map = offline_map_message->complete_map;
	for (int i = 0; i < simulator_config->map.config.x_size; i++)
		simulator_config->map.map[i] = simulator_config->map.complete_map + i * simulator_config->map.config.y_size;

//	carmen_map_server_copy_offline_map_from_message(&(simulator_config->map), offline_map_message);
	necessary_maps_available = 1;
}

//#define DUMP_GLOBALPOS_TO_FILE

#ifdef DUMP_GLOBALPOS_TO_FILE
#include <stdio.h>
#endif

static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
#ifdef DUMP_GLOBALPOS_TO_FILE
	static FILE *file = NULL;

	if (file == NULL)
	{
		file = fopen("./globalpos.csv","w");

		if (file == NULL)
		{
			printf("Error: cannot open file ./globalpos.csv\n");
			exit(-1);
		}

		fprintf(file, "# timestamp globalpos.x globalpos.y globalpos.theta odometrypos.x odometrypos.y odometrypos.theta v phi\n");
	}
#endif

	simulator_config->global_pos = *msg;

#ifdef DUMP_GLOBALPOS_TO_FILE
	fprintf(file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", simulator_config->global_pos.timestamp,
			simulator_config->global_pos.globalpos.x,
			simulator_config->global_pos.globalpos.y,
			simulator_config->global_pos.globalpos.theta,
			simulator_config->global_pos.odometrypos.x,
			simulator_config->global_pos.odometrypos.y,
			simulator_config->global_pos.odometrypos.theta,
			simulator_config->global_pos.v,
			simulator_config->global_pos.phi);
#endif
}


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	iron_bird_v = msg->v;
	iron_bird_phi = msg->phi;
}

//#define DUMP_EXTERNAL_TRUEPOS_TO_FILE

#ifdef DUMP_EXTERNAL_TRUEPOS_TO_FILE
#include <stdio.h>
#endif

static void
external_truepose_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
#ifdef DUMP_EXTERNAL_TRUEPOS_TO_FILE
	static FILE *file = NULL;

	if (file == NULL)
	{
		file = fopen("./external_truepos.csv","w");

		if (file == NULL)
		{
			printf("Error: cannot open file ./external_truepos.csv\n");
			exit(-1);
		}

		fprintf(file, "# timestamp true_pose.x true_pose.y true_pose.theta odom_pose.x odom_pose.y odom_pose.theta v phi\n");
	}
#endif

	simulator_config->true_pose = msg->truepose;
	simulator_config->odom_pose = msg->odometrypose;
	simulator_config->v = msg->v;
	simulator_config->phi = msg->phi;

#ifdef DUMP_GLOBALPOS_TO_FILE
	double timestamp = msg->timestamp;

	fprintf(file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", timestamp,
			simulator_config->true_pose.x,
			simulator_config->true_pose.y,
			simulator_config->true_pose.theta,
			simulator_config->odom_pose.x,
			simulator_config->odom_pose.y,
			simulator_config->odom_pose.theta,
			simulator_config->v,
			simulator_config->phi);
#endif
}


static void
truepos_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err;
	carmen_simulator_ackerman_truepos_message response;

	if (!necessary_maps_available)
		return;

	IPC_msgInstanceFormatter(msgRef);
	IPC_freeByteArray(callData);

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();
	response.truepose = simulator_config->true_pose;
	response.odometrypose = simulator_config->odom_pose;
	response.v = simulator_config->v;
	response.phi = simulator_config->phi;

	err = IPC_respondData(msgRef, CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME);
}


static void
carmen_route_planner_road_network_message_handler(carmen_route_planner_road_network_message *msg)
{
	road_network_message = msg;
}


static void
carmen_navigator_ackerman_status_message_handler(carmen_navigator_ackerman_status_message *msg)
{
	autonomous = (msg->autonomous == 1)? TRUE: FALSE;
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	behavior_selector_low_level_state = msg->low_level_state;

	if (msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_ENGAGE_COLLISION_GEOMETRY)
		carmen_collision_detection_set_robot_collision_config(ENGAGE_GEOMETRY);
	else
		carmen_collision_detection_set_robot_collision_config(DEFAULT_GEOMETRY);
}


static void
task_manager_desired_engage_state_message_handler(carmen_task_manager_desired_engage_state_message *message)
{
	if (message->desired_engage_state == ENGAGED)
		g_XGV_horn_status &= ~0x02;
	else if (message->desired_engage_state == DISENGAGED)
		g_XGV_horn_status |= 0x02;
}

static void
tune_pid_gain_steering_handler(tune_pid_gain_steering_parameters_message *msg)
{
	printf("valor antes de kp %lf\n", msg->kp);
	printf("valor antes de kd %lf\n", msg->kd);
	printf("valor antes de ki %lf\n", msg->ki);
	msg->kp++;
	msg->ki++;
	msg->kd++;
	printf("valor atual de kp %lf\n", msg->kp);
	printf("valor atual de kd %lf\n", msg->kd);
	printf("valor atual de ki %lf\n", msg->ki);
	global_steer_kp = msg->kp;
	global_steer_kd = msg->kd;
	global_steer_ki = msg->ki;
}

static void
tune_pid_gain_velocity_handler(tune_pid_gain_velocity_parameters_message *msg)
{
	printf("valor antes de kp %lf\n", msg->kp);
	printf("valor antes de kd %lf\n", msg->kd);
	printf("valor antes de ki %lf\n", msg->ki);
	msg->kp++;
	msg->ki++;
	msg->kd++;
	printf("valor atual de kp %lf\n", msg->kp);
	printf("valor atual de kd %lf\n", msg->kd);
	printf("valor atual de ki %lf\n", msg->ki);
	global_vel_kp = msg->kp;
	global_vel_kd = msg->kd;
	global_vel_ki = msg->ki;
}


void
velocity_pid_data_handler(velocity_pid_data_message *msg)
{
	printf("Velocidade atual %lf\n", msg->current_velocity);
}


void
steering_pid_data_handler(steering_pid_data_message *msg)
{
	printf("Effort ataul %lf\n", msg->effort);
}


/* handles ctrl+c */
static void
shutdown_module(int x)
{
	if (x == SIGINT)
	{
		carmen_ipc_disconnect();
		carmen_warn("\nDisconnected.\n");
		exit(0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Inicializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
fill_laser_config_data(carmen_simulator_ackerman_laser_config_t *lasercfg)
{
	lasercfg->num_lasers = 1 + carmen_round(lasercfg->fov / lasercfg->angular_resolution);
	lasercfg->start_angle = -0.5*lasercfg->fov;

	/* give a warning if it is not a standard configuration */

	if (fabs(lasercfg->fov - M_PI) > 1e-6 &&
			fabs(lasercfg->fov - 100.0/180.0 * M_PI) > 1e-6 &&
			fabs(lasercfg->fov -  90.0/180.0 * M_PI) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (fov=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->fov));

	if (fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(1.0)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.5)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.25)) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (res=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->angular_resolution));

}

#ifdef __USE_RL_CONTROL

void
rl_control_handler(carmen_rl_control_message *message)
{
	set_rl_control(message->steering, message->throttle, message->brake);
	time_last_message_arrived = message->timestamp;
}

#endif

static int
subscribe_to_relevant_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME, set_truepose_handler, NULL);
	if (err != IPC_OK)
		return -1;
	IPC_setMsgQueueLength(CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME, 100);

	err = IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME, set_object_handler, NULL);
	if (err != IPC_OK)
		return -1;
	IPC_setMsgQueueLength(CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME, 100);

	err = IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME, clear_objects_handler, NULL);
	if (err != IPC_OK)
		return -1;
	IPC_setMsgQueueLength(CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME, 1);

	err = IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_QUERY_NAME, truepos_query_handler, NULL);
	if (err != IPC_OK)
		return -1;

	memset(&init_msg, 0, sizeof(carmen_localize_ackerman_initialize_message));

	carmen_localize_ackerman_subscribe_initialize_message(&init_msg, (carmen_handler_t) localize_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_update_handler, CARMEN_SUBSCRIBE_LATEST);

	if ((!simulate_legacy_500) || (simulate_legacy_500 && connected_to_iron_bird && use_external_true_pose))
		carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) motion_command_handler, CARMEN_SUBSCRIBE_LATEST);
	else if (simulate_legacy_500 && !connected_to_iron_bird)
		carmen_base_ackerman_subscribe_motion_command_2(NULL, (carmen_handler_t) motion_command_handler, CARMEN_SUBSCRIBE_LATEST);

	if (simulate_legacy_500 && connected_to_iron_bird)
		carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (use_external_true_pose)
		carmen_simulator_ackerman_subscribe_external_truepos_message(NULL, (carmen_handler_t) external_truepose_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

#ifdef __USE_RL_CONTROL

	carmen_rl_control_subscribe_message(NULL, (carmen_handler_t) rl_control_handler, CARMEN_SUBSCRIBE_LATEST);

#endif

	carmen_route_planner_subscribe_road_network_message(NULL, (carmen_handler_t) carmen_route_planner_road_network_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_navigator_ackerman_subscribe_status_message(NULL, (carmen_handler_t) carmen_navigator_ackerman_status_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_task_manager_subscribe_desired_engage_state_message(NULL, (carmen_handler_t) task_manager_desired_engage_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ford_escape_subscribe_tune_pid_gain_steering_parameters_message(NULL, (carmen_handler_t) tune_pid_gain_steering_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ford_escape_subscribe_tune_pid_gain_velocity_parameters_message(NULL, (carmen_handler_t) tune_pid_gain_velocity_handler, CARMEN_SUBSCRIBE_LATEST);
	//carmen_ford_escape_subscribe_velocity_pid_data_message(NULL, (carmen_handler_t) velocity_pid_data_handler, CARMEN_SUBSCRIBE_LATEST);
	//carmen_ford_escape_subscribe_steering_pid_data_message(NULL, (carmen_handler_t) steering_pid_data_handler , CARMEN_SUBSCRIBE_LATEST);
	return (0);
}


static int
initialize_ipc(void)
{
	IPC_RETURN_TYPE err;

	/* define messages created by this module */
	err = IPC_defineMsg(CARMEN_LASER_FRONTLASER_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_LASER_FRONTLASER_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_LASER_REARLASER_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_LASER_FRONTLASER_FMT);
	if (err != IPC_OK)
		return -1;

	if (!simulate_legacy_500)
	{
		err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
		if (err != IPC_OK)
			return -1;
	}
	else
	{
		err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_2_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BASE_ACKERMAN_ODOMETRY_2_FMT);
		if (err != IPC_OK)
			return -1;
	}

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_VELOCITY_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_BASE_ACKERMAN_VELOCITY_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_OBJECTS_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_SIMULATOR_ACKERMAN_OBJECTS_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_NEXT_TICK_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_QUERY_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_DEFAULT_MESSAGE_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	if (err != IPC_OK)
		return -1;

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_FMT);
	if (err != IPC_OK)
		return -1;

	err = carmen_moving_objects_point_clouds_define_messages_generic(0);
	if (err != IPC_OK)
			return -1;

	return 0;
}


static void
read_parameters(int argc, char *argv[], carmen_simulator_ackerman_config_t *config)
{
	int num_items;

	carmen_param_t param_list[]=
	{
			{"simulator", "time", CARMEN_PARAM_DOUBLE, &(config->real_time), 1, NULL},
			{"simulator", "sync_mode", CARMEN_PARAM_ONOFF, &(config->sync_mode), 1, NULL},
			{"simulator", "use_robot", CARMEN_PARAM_ONOFF, &use_robot, 1, NULL},
			{"simulator", "motion_timeout", CARMEN_PARAM_DOUBLE, &(config->motion_timeout),1, NULL},
			{"simulator", "use_velocity_nn", CARMEN_PARAM_ONOFF, &use_velocity_nn,1, NULL},
			{"simulator", "use_phi_nn", CARMEN_PARAM_ONOFF, &use_phi_nn, 1, NULL},
			{"robot", "frontlaser_use", CARMEN_PARAM_ONOFF, &(config->use_front_laser), 1, NULL},
			{"robot", "frontlaser_id", CARMEN_PARAM_INT, &(config->front_laser_config.id), 0, NULL},
			{"robot", "rearlaser_use", CARMEN_PARAM_ONOFF, &(config->use_rear_laser), 1, NULL},
			{"robot", "rearlaser_id", CARMEN_PARAM_INT, &(config->rear_laser_config.id), 0, NULL},
			{"robot", "width", CARMEN_PARAM_DOUBLE, &(config->width), 1, NULL},
			{"robot", "length", CARMEN_PARAM_DOUBLE, &(config->length), 1, NULL},
			{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &(config->distance_between_rear_wheels), 1, NULL},
			{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &(config->distance_between_front_and_rear_axles), 1, NULL},
			{"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &(config->max_v), 1,NULL},
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &(config->max_phi), 1, NULL},
			{"robot", "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &(config->maximum_steering_command_rate), 0, NULL},
			{"robot", "understeer_coeficient", CARMEN_PARAM_DOUBLE, &(config->understeer_coeficient), 0, NULL},
			{"robot", "understeer_coeficient2", CARMEN_PARAM_DOUBLE, &(config->understeer_coeficient2), 0, NULL},
			{"robot", "maximum_speed_forward", CARMEN_PARAM_DOUBLE, &(config->maximum_speed_forward), 0, NULL},
			{"robot", "maximum_speed_reverse", CARMEN_PARAM_DOUBLE, &(config->maximum_speed_reverse), 0, NULL},
			{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &(config->maximum_acceleration_forward), 0, NULL},
			{"robot", "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &(config->maximum_deceleration_forward), 0, NULL},
			{"robot", "maximum_acceleration_reverse", CARMEN_PARAM_DOUBLE, &(config->maximum_acceleration_reverse), 0, NULL},
			{"robot", "maximum_deceleration_reverse", CARMEN_PARAM_DOUBLE, &(config->maximum_deceleration_reverse), 0, NULL},

			{(char *) "robot", (char *) "desired_decelaration_forward",	 CARMEN_PARAM_DOUBLE, &config->robot_config.desired_decelaration_forward,					1, NULL},
			{(char *) "robot", (char *) "desired_decelaration_reverse",	 CARMEN_PARAM_DOUBLE, &config->robot_config.desired_decelaration_reverse,					1, NULL},
			{(char *) "robot", (char *) "desired_acceleration",			 CARMEN_PARAM_DOUBLE, &config->robot_config.desired_acceleration,							1, NULL},
			{(char *) "robot", (char *) "desired_steering_command_rate", CARMEN_PARAM_DOUBLE, &config->robot_config.desired_steering_command_rate,					1, NULL},
			{(char *) "robot", (char *) "understeer_coeficient",		 CARMEN_PARAM_DOUBLE, &config->robot_config.understeer_coeficient,							1, NULL},
			{(char *) "robot", (char *) "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &config->robot_config.maximum_steering_command_rate, 					1, NULL},

			{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &(config->distance_between_rear_car_and_rear_wheels), 0, NULL},
			{"simulator_ackerman", "publish_laser", CARMEN_PARAM_ONOFF, &publish_laser_flag, 0, NULL},
			{"rrt",   "use_mpc",                    CARMEN_PARAM_ONOFF, &(config->use_mpc), 0, NULL},
			{"rrt",   "use_rlpid",                  CARMEN_PARAM_ONOFF, &(config->use_rlpid), 0, NULL},
			{"behavior_selector",  "use_truepos", 	CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL},
			{"simulator", "ackerman_publish_detection_moving_objects", CARMEN_PARAM_ONOFF, &publish_detection_moving_objects, 0, NULL},

	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	static char frontlaser_fov_string[256];
	static char frontlaser_res_string[256];
	static char rearlaser_fov_string[256];
	static char rearlaser_res_string[256];

	sprintf(frontlaser_fov_string, "laser%d_fov", config->front_laser_config.id);
	sprintf(frontlaser_res_string, "laser%d_resolution", config->front_laser_config.id);

	sprintf(rearlaser_fov_string, "laser%d_fov", config->rear_laser_config.id);
	sprintf(rearlaser_res_string, "laser%d_resolution", config->rear_laser_config.id);

	carmen_param_t param_list_front_laser[] =
	{
			{"simulator", "frontlaser_maxrange", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.max_range), 1, NULL},
			{"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.offset), 1, NULL},
			{"robot", "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.side_offset), 1, NULL},
			{"robot", "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.angular_offset), 1, NULL},
			{"laser", frontlaser_fov_string, CARMEN_PARAM_DOUBLE, &(config->front_laser_config.fov), 0, NULL},
			{"laser", frontlaser_res_string, CARMEN_PARAM_DOUBLE, &(config->front_laser_config.angular_resolution), 0, NULL},
			{"simulator", "laser_probability_of_random_max", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.prob_of_random_max), 1, NULL},
			{"simulator", "laser_probability_of_random_reading", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.prob_of_random_reading), 1, NULL},
			{"simulator", "laser_sensor_variance", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.variance), 1, NULL}
	};

	carmen_param_t param_list_rear_laser[] =
	{
			{"simulator", "rearlaser_maxrange", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.max_range), 1, NULL},
			{"robot", "rearlaser_offset", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.offset), 1, NULL},
			{"robot", "rearlaser_side_offset", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.side_offset), 1, NULL},
			{"robot", "rearlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.angular_offset), 1, NULL},
			{"laser", rearlaser_fov_string, CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.fov), 0, NULL},
			{"laser", rearlaser_res_string, CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.angular_resolution), 0, NULL},
			{"simulator", "laser_probability_of_random_max", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.prob_of_random_max), 1, NULL},
			{"simulator", "laser_probability_of_random_reading", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.prob_of_random_reading), 1, NULL},
			{"simulator", "laser_sensor_variance", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.variance), 1, NULL}
	};

	if (config->use_front_laser)
	{
		num_items = sizeof(param_list_front_laser)/
				sizeof(param_list_front_laser[0]);
		carmen_param_install_params(argc, argv, param_list_front_laser,
				num_items);
		config->front_laser_config.angular_resolution =
				carmen_degrees_to_radians(config->front_laser_config.angular_resolution);

		config->front_laser_config.fov =
				carmen_degrees_to_radians(config->front_laser_config.fov);
	}

	if (config->use_rear_laser)
	{
		num_items = sizeof(param_list_rear_laser)/
				sizeof(param_list_rear_laser[0]);
		carmen_param_install_params(argc, argv, param_list_rear_laser,
				num_items);
		config->rear_laser_config.angular_resolution =
				carmen_degrees_to_radians(config->rear_laser_config.angular_resolution);

		config->rear_laser_config.fov =
				carmen_degrees_to_radians(config->rear_laser_config.fov);
	}

	fill_laser_config_data( &(config->front_laser_config) );

	if (config->use_rear_laser)
		fill_laser_config_data( &(config->rear_laser_config));

	carmen_param_allow_unfound_variables(1);
	carmen_param_t optional_param_list[] =
	{
			{(char *) "commandline", (char *) "simulate_legacy_500", CARMEN_PARAM_ONOFF, &simulate_legacy_500, 0, NULL},
			{(char *) "commandline", (char *) "connected_to_iron_bird", CARMEN_PARAM_ONOFF, &connected_to_iron_bird, 0, NULL},
			{(char *) "commandline", (char *) "use_external_true_pose", CARMEN_PARAM_ONOFF, &use_external_true_pose, 0, NULL}
	};
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	// TODO REMOVER ESTA PARTE E LER OS PARAMETROS DIRETAMENTE PARA O ROBO_CONFIG
	config->robot_config.maximum_steering_command_rate = config->maximum_steering_command_rate;
	config->robot_config.understeer_coeficient = config->understeer_coeficient;
	config->robot_config.maximum_acceleration_forward = config->maximum_acceleration_forward;
	config->robot_config.maximum_deceleration_forward = config->maximum_deceleration_forward;
	config->robot_config.maximum_acceleration_reverse = config->maximum_acceleration_reverse;
	config->robot_config.maximum_deceleration_reverse = config->maximum_deceleration_reverse;
	config->robot_config.distance_between_rear_car_and_rear_wheels = config->distance_between_rear_car_and_rear_wheels;
	config->robot_config.distance_between_front_and_rear_axles = config->distance_between_front_and_rear_axles;
	config->robot_config.max_phi = config->max_phi;
	config->robot_config.length = config->length;
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

#ifdef __USE_RL_CONTROL
	set_rl_control(0,0,100);
#endif

	// Init relevant data strutures
	memset(&simulator_conf, 0, sizeof(carmen_simulator_ackerman_config_t));
	simulator_config = &simulator_conf;
	//memset(nun_motion_commands, 0, NUM_MOTION_COMMANDS_VECTORS);

	read_parameters(argc, argv, &simulator_conf);
	//carmen_ford_escape_hybrid_read_pid_parameters(argc, argv);
	carmen_libpid_read_PID_parameters(argc, argv);

	carmen_simulator_ackerman_initialize_object_model(argc, argv);

	signal(SIGINT, shutdown_module);

	if (initialize_ipc() < 0)
		carmen_die("Error in initializing ipc...\n");

	if (subscribe_to_relevant_messages() < 0)
		carmen_die("Error subscribing to messages...\n");

//	carmen_ipc_addPeriodicTimer(simulator_conf.real_time, simulate_car_and_publish_readings, NULL);
//	carmen_ipc_dispatch();


//	IPC_RETURN_TYPE err;
//
//	/* Set local message queue capacity */
//	err = IPC_setCapacity(4);
//	carmen_test_ipc_exit(err, "I had problems setting the IPC capacity. This is a "
//			"very strange error and should never happen.\n",
//			"IPC_setCapacity");

	while (1)
	{
		simulate_car_and_publish_readings(NULL, 0, 0);
		carmen_ipc_sleep(simulator_conf.real_time);
	}

	exit(0);
}
