
#ifndef __CARMEN_COMM_UTIL__
#define __CARMEN_COMM_UTIL__

#include <carmen/laser_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <vector>

std::vector<double> laser_to_vec(carmen_laser_laser_message &laser_message, double theta, int view);

std::vector<double> next_goal_from_list(double x, double y, double th,
	carmen_behavior_selector_goal_list_message *goal_list_message);

void ackerman_motion_model(double *x, double *y, double *th, double v, double phi, double dt,
		double distance_between_front_and_rear_axless);

#endif
