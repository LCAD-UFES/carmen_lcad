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
#include "ford_escape_hybrid.h"

static ford_escape_hybrid_config_t *ford_escape_hybrid_config = NULL;

static carmen_ackerman_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_VECTORS][NUM_MOTION_COMMANDS_PER_VECTOR];
static int nun_motion_commands[NUM_MOTION_COMMANDS_VECTORS];
static int current_motion_command_vetor = 0;

// Global variables
double g_v = 10.0;
double g_phi = 0.0;



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

	msg.g_XGV_throttle = 0.0;
	msg.g_XGV_steering = 0.0;
	msg.g_XGV_brakes = 0.0;

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
	msg.g_XGV_component_status = 0;

	msg.g_XGV_main_propulsion = 0;
	msg.g_XGV_main_fuel_supply = 0;
	msg.g_XGV_parking_brake = 0;
	msg.g_XGV_gear = 0;

	msg.g_XGV_turn_signal = 0;
	msg.g_XGV_horn_status = 0;
	msg.g_XGV_headlights_status = 0;

//	printf("g_XGV_turn_signal = 0x%x\n", g_XGV_turn_signal);
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

	msg.num_errors = 0;
	msg.error = NULL;

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

	robot_ackerman_velocity_message.v = g_v;
	robot_ackerman_velocity_message.phi = g_phi;
	robot_ackerman_velocity_message.timestamp = carmen_get_time();
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


carmen_localize_ackerman_globalpos_message global_pos, previous_global_pos;
static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	previous_global_pos = global_pos;
	global_pos = *msg;
}


static void 
shutdown_module()
{
	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("Disconnected from IPC.\n");
		done = 1;
	}
	exit(0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
subscribe_to_relevant_messages()
{
	carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) base_ackerman_motion_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static int 
initialize_ipc(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_ROBOT_ACKERMAN_VELOCITY_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);

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
initialize_structures()
{
	// Init relevant data strutures
	ford_escape_hybrid_config = malloc(sizeof(ford_escape_hybrid_config_t));
	memset(ford_escape_hybrid_config, 0, sizeof(ford_escape_hybrid_config_t));
	ford_escape_hybrid_config->current_motion_command_vector = NULL;
	ford_escape_hybrid_config->XGV_v_and_phi_timestamp = carmen_get_time();

	memset(nun_motion_commands, 0, NUM_MOTION_COMMANDS_VECTORS);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


int 
main(int argc, char** argv)
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	initialize_structures();

	initialize_ipc();

	subscribe_to_relevant_messages();

	carmen_ipc_addPeriodicTimer(1.0 / 40.0, (TIMER_HANDLER_TYPE) publish_velocity_message, NULL);

	carmen_ipc_dispatch();

	return (0);
}
