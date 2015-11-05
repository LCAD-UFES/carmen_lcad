/*
 * OccupancyGrid.cpp
 *
 *  Created on: 30/09/2011
 *      Author: rradaelli
 */

#include "occupancy_grid.h"
#include <math.h>
#include <carmen/grid_mapping_messages.h>

OccupancyGrid::OccupancyGrid() {
	x_size = 400;
	y_size = 400;
	grid_map = (float*) malloc(sizeof(float)*x_size*y_size);
	resolution = 0.10;
	locc  = 0.954242509*2.5;
	lfree = -0.954242509/2.5;
	lunk = 0;
	alfa = resolution*19.45712521458785421;

	cheat = false;

	initialize_map();
}

void OccupancyGrid::initialize_map() {
	for(int i=0; i<x_size*y_size;i++) {
		grid_map[i] = lunk;
	}
}


void OccupancyGrid::add_new_scan(carmen_robot_ackerman_laser_message laser) {
	state = laser;

	state.robot_pose.x /= resolution;
	state.robot_pose.y /= resolution;

	//convert everything
	if(!cheat) {
		state.robot_pose.x += x_size/2;
		state.robot_pose.y += y_size/2;
	}

	state.config.maximum_range/=resolution;


	for(int i=0; i<state.num_readings; i++) {
		state.range[i]/=resolution;

		if(state.range[i]>=state.config.maximum_range) {
			state.range[i] = 0;
		}
	}

	occupancy_grid_mapping();
}

void OccupancyGrid::occupancy_grid_mapping() {

	for(int i=0; i<state.num_readings; i++) {
		walk_through_laser(i);
	}
}

void OccupancyGrid::walk_through_laser(int range_index) {
	int x, y;
	double distance;

	x = state.robot_pose.x;
	y = state.robot_pose.y;
	distance = state.range[range_index];
	double laser_angle = state.config.start_angle + state.config.angular_resolution*range_index + state.robot_pose.theta;

	/*if(distance>=state.config.maximum_range) {
		return;
	}*/

	/*for(double d=0; d<distance; d+=0.5)
	{
		int x_final = x + (cos(laser_angle) * d);
		int	y_final = y + (sin(laser_angle) * d);

		if(is_valid_position(x_final, y_final)) {
			double inverse_range_sensor = lfree;

			if(fabs(d-distance)<alfa/2.0) {
				inverse_range_sensor = locc;
			}

			grid_map[x_final*y_size+y_final] = grid_map[x_final*y_size+y_final] + inverse_range_sensor + lunk;
		}

	}*/

	carmen_bresenham_param_t params;
	int x_final = x + (cos(laser_angle) * distance);
	int	y_final = y + (sin(laser_angle) * distance);

	carmen_get_bresenham_parameters(x, y, x_final, y_final, &params);
	do {
		carmen_get_current_point(&params, &x_final, &y_final);

		if(is_valid_position(x_final, y_final)) {
			double inverse_range_sensor = lfree;
			double d = sqrt((x-x_final)*(x-x_final) + (y-y_final)*(y-y_final));

			if(fabs(d-distance)<alfa/2.0) {
				inverse_range_sensor = locc;
			}

			grid_map[x_final*y_size+y_final] = grid_map[x_final*y_size+y_final] + inverse_range_sensor + lunk;
		}

	} while (carmen_get_next_point(&params));

}

bool OccupancyGrid::is_valid_position(int x, int y) {
	return x>=0 && x<x_size && y>=0 && y<y_size;
}

double OccupancyGrid::log_odds_to_probability(double log_odds) {
	if(log_odds<=locc*3.5 && log_odds>=lfree*3.5)//bias
		return -1;

	return 1 - 1/(1+exp(log_odds));
}

void OccupancyGrid::print() {
	for(int i=0;i<x_size;i++) {
		for(int j=0;j<y_size;j++) {
			std::cout << grid_map[i*y_size + j] << " ";
		}
		std::cout <<  "\n";
	}
}

void OccupancyGrid::publish() {

	IPC_RETURN_TYPE err;
	static bool first_time = true;

	if(first_time) {
		err = IPC_defineMsg(CARMEN_GRID_MAPPING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_GRID_MAPPING_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_GRID_MAPPING_MESSAGE_NAME);
		first_time = false;
	}

	float *map = (float*)malloc(sizeof(float)*x_size*y_size);

	for(int x=0; x<x_size; x++) {
		for(int y=0; y<y_size; y++) {
			double val = grid_map[x*y_size + y];

			val = log_odds_to_probability(val);

			map[x*y_size + y] = val;
		}
	}

	carmen_grid_mapping_message grid_map_message;

	grid_map_message.complete_map = map;
	grid_map_message.size = x_size * y_size;
	grid_map_message.config.x_size = x_size;
	grid_map_message.config.y_size = y_size;
	grid_map_message.config.resolution = resolution;
	grid_map_message.config.map_name = (char*)"occupancy grid";
	grid_map_message.host = carmen_get_host();
	grid_map_message.timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_GRID_MAPPING_MESSAGE_NAME, &grid_map_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_GRID_MAPPING_MESSAGE_NAME);

	carmen_slam_publish_map(map, x_size, y_size, resolution);

	free(map);
}

OccupancyGrid::~OccupancyGrid() {
	free(grid_map);
}
