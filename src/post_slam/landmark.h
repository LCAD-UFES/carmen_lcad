/*
 * landmark.h
 *
 *  Created on: Apr 12, 2013
 *      Author: filipe
 */

#ifndef LANDMARK_H_
#define LANDMARK_H_

#include <vector>
#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>

using namespace std;

int is_a_possible_post(unsigned short int distance[]);

namespace post_slam
{
	class Landmark
	{
		public:

			double x, y;
			double radius;

			Landmark();
			Landmark(const Landmark &l);
			Landmark(double landmark_position_x, double landmark_position_y, double landmark_radius);
			~Landmark();
	};

	vector<Landmark*> detect_landmarks(carmen_velodyne_partial_scan_message *velodyne_partial_scan_message);
	void update_landmarks_position_in_the_world(vector<Landmark*> *landmarks, double car_x, double car_y, double car_theta);
	void initialize_transforms(carmen_pose_3D_t sensor_board_pose_g, carmen_pose_3D_t velodyne_pose_g, carmen_pose_3D_t car_pose_g);
	void update_landmarks_position_in_the_world(vector<Landmark*> *landmarks, double car_x, double car_y, double car_theta);
}

#endif /* LANDMARK_H_ */
