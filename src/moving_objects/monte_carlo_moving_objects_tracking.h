#ifndef MONTE_CARLO_MOVING_OBJECTS_TRACKING_H
#define MONTE_CARLO_MOVING_OBJECTS_TRACKING_H

#include <carmen/carmen.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/velodyne_interface.h>
#include <vector>
#include <list>
#include "moving_objects.h"
#include "monte_carlo_moving_objects_tracking.h"

using std::vector;

typedef struct particle{

	carmen_point_t pose;
	double velocity;
	double weight;
	double timestamp;
} particle;

std::vector<particle> algorithm_monte_carlo(std::vector<particle> *particle_set_t_1,
		double x, double y, double delta_time, pcl::PointCloud<pcl::PointXYZ> pcl_cloud);

#endif
