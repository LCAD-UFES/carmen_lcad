/*
 * OccupancyGrid.h
 *
 *  Created on: 30/09/2011
 *      Author: rradaelli
 */

#ifndef OCCUPANCYGRID_H_
#define OCCUPANCYGRID_H_
#define UNKNOWN -1

#include <carmen/carmen.h>
#include <carmen/slam_interface.h>
#include <math.h>
#include <iostream>
#include <vector>

class OccupancyGrid {
public:
	OccupancyGrid();
	virtual ~OccupancyGrid();
	void add_new_scan(carmen_robot_ackerman_laser_message laser);
	void print();
	void publish();
	bool cheat;

private:
	void initialize_map();
	bool is_in_perceptual_field(int x, int y);
	float inverse_sensor_model(int x, int y);
	void walk_through_laser(int);
	void occupancy_grid_mapping();
	bool is_valid_position(int x, int y);
	double log_odds_to_probability(double log_odds);

private:
	float *grid_map;
	double lunk;
	double locc;
	double lfree;
	double alfa;
	int x_size, y_size;
	carmen_robot_ackerman_laser_message state;
	double resolution;

};

#endif /* OCCUPANCYGRID_H_ */
