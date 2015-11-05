/*
 * main.cpp
 *
 *  Created on: 30/09/2011
 *      Author: rradaelli
 */

#include <cstdlib>
#include <iostream>
#include <carmen/carmen.h>
#include <carmen/grid_mapping_messages.h>
#include "occupancy_grid.h"
#include <vector>
#include <math.h>

carmen_robot_ackerman_laser_message laser_message;
carmen_simulator_ackerman_truepos_message truepos_message;
OccupancyGrid* grid = NULL;

void laser_handler(void) {
	if(grid->cheat)
		laser_message.robot_pose = truepos_message.truepose;

	grid->add_new_scan(laser_message);
	grid->publish();
}

void laser_differencial_handler(carmen_robot_laser_message *msg) {
	laser_message.robot_pose = msg->robot_pose;

	laser_message.config = msg->config;
	laser_message.num_readings = msg->num_readings;
	laser_message.num_remissions = msg->num_remissions;
	laser_message.range = msg->range;
	laser_message.remission = msg->remission;

	grid->add_new_scan(laser_message);
	grid->publish();

}

void shutdown_module(int signal) {
	signal = 0;
	if(signal) {

	}

	//grid->print();
	grid->publish();

	carmen_ipc_disconnect();
	delete grid;
	exit(0);
}

void register_handlers() {
	carmen_simulator_ackerman_subscribe_truepos_message(
			&truepos_message,
			NULL,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_robot_subscribe_frontlaser_message(
			NULL,
			(carmen_handler_t)laser_differencial_handler,
			CARMEN_SUBSCRIBE_LATEST
	);

	carmen_robot_ackerman_subscribe_frontlaser_message(
			&laser_message,
			(carmen_handler_t)laser_handler,
			CARMEN_SUBSCRIBE_LATEST
	);
}

int main(int argc, char **argv) {
	signal(SIGINT, shutdown_module);

	grid = new OccupancyGrid();

	if(argc>1) {
		grid->cheat = atoi(argv[1]);
	}

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	register_handlers();

	carmen_ipc_dispatch();

	return EXIT_SUCCESS;
}

