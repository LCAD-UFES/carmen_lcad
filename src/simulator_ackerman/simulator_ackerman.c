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
#include <control.h>

#include "simulator_ackerman.h"
#include "simulator_ackerman_simulation.h"
#include "simulator_ackerman_messages.h"

#ifdef __USE_RL_CONTROL
#include <rlcontrol_interface.h>
double time_last_message_arrived = 0;
#endif

#include "objects_ackerman.h"

static carmen_simulator_ackerman_config_t simulator_conf;
static carmen_simulator_ackerman_config_t *simulator_config;

static int use_robot = 1;

static carmen_localize_ackerman_initialize_message init_msg;

//static int current_motion_command_vetor = 0;
//static carmen_ackerman_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_VECTORS][NUM_MOTION_COMMANDS_PER_VECTOR];
//static int nun_motion_commands[NUM_MOTION_COMMANDS_VECTORS];
static int publish_laser_flag = 0;

static int necessary_maps_available = 0;

static int use_truepos = 0;
static int use_velocity_nn = 1;
static int use_phi_nn = 1;

static int simulate_legacy_500 = 0;


static void
carmen_destroy_simulator_map(carmen_map_t *map)
{
	free(map->complete_map);
	free(map->map);
}


int
apply_system_latencies(carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands)
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
		odometry.x     = simulator_config->odom_pose.x;
		odometry.y     = simulator_config->odom_pose.y;
		odometry.theta = simulator_config->odom_pose.theta;
		odometry.v     = simulator_config->v;
		odometry.phi   = simulator_config->phi;
		odometry.timestamp = timestamp;

//		printf ("%lf %lf %lf %lf %lf\n", odometry.x, odometry.y, odometry.theta, odometry.v, odometry.phi);

		err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &odometry);
		carmen_test_ipc(err, "Could not publish base_odometry_message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
	}
	else
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
	truepos.v = simulator_config->v;
	truepos.phi = simulator_config->phi;
	truepos.timestamp = timestamp;

	err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, &truepos);
	carmen_test_ipc(err, "Could not publish simualator_truepos_message", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME);
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

	carmen_simulator_ackerman_get_object_poses(&(objects.num_objects), &(objects.objects_list));
	objects.timestamp = timestamp;
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
	{
		return;
	}

	if (first)
	{
		rlaser.host = carmen_get_host();

		rlaser.num_readings = simulator_config->rear_laser_config.num_lasers;
		rlaser.range = (double *)calloc
				(simulator_config->rear_laser_config.num_lasers, sizeof(double));
		carmen_test_alloc(rlaser.range);

		rlaser.num_remissions = 0;
		rlaser.remission = 0;
		first = 0;
	}

	carmen_simulator_ackerman_calc_laser_msg(&rlaser, simulator_config, 1);

	rlaser.timestamp = timestamp;
	err = IPC_publishData(CARMEN_LASER_REARLASER_NAME, &rlaser);
	carmen_test_ipc(err, "Could not publish laser_rearlaser_message",
			CARMEN_LASER_REARLASER_NAME);

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
	carmen_simulator_ackerman_recalc_pos(simulator_config, use_velocity_nn, use_phi_nn);
	carmen_simulator_ackerman_update_objects(simulator_config);

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
		publish_frontlaser(timestamp);
		publish_rearlaser(timestamp);
	}
	counter++;

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

//	printf("delay %lf\n", carmen_get_time() - motion_command_message->timestamp);

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

	carmen_simulator_ackerman_create_object(msg.pose.x, msg.pose.y, msg.pose.theta,	CARMEN_SIMULATOR_ACKERMAN_RANDOM_OBJECT, msg.speed);
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
	IPC_unmarshallData(formatter, callData, &msg,
			sizeof(carmen_simulator_ackerman_set_truepose_message));
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
	IPC_unmarshallData(formatter, callData, &msg,
			sizeof(carmen_default_message));
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
map_update_handler(carmen_map_t *new_map)
{
	carmen_map_p map_ptr;
	int map_x, map_y;
	carmen_point_t zero = {0, 0, 0};

	map_ptr = carmen_map_clone(new_map);

	carmen_destroy_simulator_map(&simulator_config->map);

	simulator_config->map = *map_ptr;
	free(map_ptr);

	/* Reset odometry and true pose only of the robot's pose     */
	/* is not valid given the new map. Otherwise keep old poses. */
	/* This enables to activate a new map without messing up     */
	/* the odometry. */

	map_x = (simulator_config->true_pose.x - simulator_config->map.config.x_origin) /
			simulator_config->map.config.resolution;
	map_y = (simulator_config->true_pose.y - simulator_config->map.config.y_origin) /
			simulator_config->map.config.resolution;

	if (map_x < 0 || map_x >= simulator_config->map.config.x_size ||
			map_y < 0 || map_y >= simulator_config->map.config.y_size ||
			simulator_config->map.map[map_x][map_y] > .15 ||
			carmen_simulator_object_too_close(simulator_config->true_pose.x, simulator_config->true_pose.y, -1))
	{
		simulator_config->odom_pose = zero;
		simulator_config->true_pose = zero;
	}

}

// Essa funcao nao eh usada em lugar nenhum...
void
grid_mapping_handler(carmen_map_server_offline_map_message *new_gridmap)
{
//	printf("teste\n");
	carmen_map_t new_map;

	new_map.complete_map = new_gridmap->complete_map;
	new_map.config = new_gridmap->config;

	map_update_handler(&new_map);
}


void
offline_map_update_handler(carmen_map_server_offline_map_message *offline_map_message)
{
	carmen_map_server_copy_offline_map_from_message(&(simulator_config->map), offline_map_message);
	necessary_maps_available = 1;
}


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	simulator_config->global_pos = *msg;
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
		carmen_warn("Warnung: You are not using a standard SICK configuration (fov=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->fov));

	if (fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(1.0)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.5)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.25)) > 1e-6)
		carmen_warn("Warnung: You are not using a standard SICK configuration (res=%.4f deg)\n",
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

	err = IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME,
			set_truepose_handler, NULL);
	if (err != IPC_OK)
		return -1;
	IPC_setMsgQueueLength(CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME, 100);

	err = IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME,
			set_object_handler, NULL);
	if (err != IPC_OK)
		return -1;
	IPC_setMsgQueueLength(CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME, 100);

	err = IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME,
			clear_objects_handler, NULL);
	if (err != IPC_OK)
		return -1;
	IPC_setMsgQueueLength(CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME, 1);

	err = IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_QUERY_NAME,
			truepos_query_handler, NULL);
	if (err != IPC_OK)
		return -1;

	memset(&init_msg, 0, sizeof(carmen_localize_ackerman_initialize_message));

	carmen_localize_ackerman_subscribe_initialize_message(&init_msg, (carmen_handler_t) localize_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_update_handler, CARMEN_SUBSCRIBE_LATEST);

	if (!simulate_legacy_500)
		carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) motion_command_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_base_ackerman_subscribe_motion_command_2(NULL, (carmen_handler_t) motion_command_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

#ifdef __USE_RL_CONTROL

	carmen_rl_control_subscribe_message(NULL, (carmen_handler_t) rl_control_handler, CARMEN_SUBSCRIBE_LATEST);

#endif

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
			{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &(config->distance_between_rear_car_and_rear_wheels), 0, NULL},
			{"simulator_ackerman", "publish_laser", CARMEN_PARAM_ONOFF, &publish_laser_flag, 0, NULL},
			{"rrt",   "use_mpc",                    CARMEN_PARAM_ONOFF, &(config->use_mpc), 0, NULL},
			{"rrt",   "use_rlpid",                  CARMEN_PARAM_ONOFF, &(config->use_rlpid), 0, NULL},
			{"behavior_selector",  "use_truepos", 	CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL}
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
		{(char *) "commandline", (char *) "simulate_legacy_500", CARMEN_PARAM_ONOFF, &simulate_legacy_500, 0, NULL}
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

	// carmen_ipc_addPeriodicTimer(simulator_conf.real_time, simulate_car_and_publish_readings, NULL);
	// carmen_ipc_dispatch();
	//
	while (1)
	{
		simulate_car_and_publish_readings(NULL, 0, 0);
		carmen_ipc_sleep(simulator_conf.real_time);
	}


	exit(0);
}
