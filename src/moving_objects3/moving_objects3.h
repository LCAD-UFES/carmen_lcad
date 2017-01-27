/*
 * moving_objects3.h
 *
 *  Created on: 27 de jan de 2017
 *      Author: luan
 */

#ifndef SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_H_
#define SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_H_

#include <vector>

#include <carmen/carmen.h>

#include <carmen/laser_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/virtual_scan_interface.h>

#include <carmen/moving_objects3_interface.h>
#include "moving_objects3_particle_filter.h"
#include "moving_objects3_utils.h"

enum moving_object_tracking_state { STARTING, TRACKING, ENDING };

typedef struct
{
	int virtual_scan_index;
	moving_object_tracking_state tracking_state;
	moving_objects3_particle_t best_particle;
	std::vector<moving_objects3_particle_t> particle_set;
} moving_object_data;

#endif /* SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_H_ */
