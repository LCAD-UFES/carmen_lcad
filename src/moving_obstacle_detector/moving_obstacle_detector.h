/*
 * moving_obstacle_detector.h
 *
 *  Created on: 22 de mar de 2017
 *      Author: luan
 */

#ifndef SRC_MOVING_OBSTACLE_DETECTOR_MOVING_OBSTACLE_DETECTOR_H_
#define SRC_MOVING_OBSTACLE_DETECTOR_MOVING_OBSTACLE_DETECTOR_H_

#include <vector>
#include <deque>

typedef struct
{
	std::vector<carmen_position_t> cell_vector;
	carmen_position_t centroid;
	double timestamp;
} moving_obstacle_observation_t;


typedef struct
{
	std::deque<moving_obstacle_observation_t> observations;
	unsigned int id;
	int color;
	int age;
	int associated;
	double velocity;
	double orientation;
} moving_obstacle_t;

typedef struct
{
	int index;
	double distance;
} distance_t;

#endif /* SRC_MOVING_OBSTACLE_DETECTOR_MOVING_OBSTACLE_DETECTOR_H_ */
