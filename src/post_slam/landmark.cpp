/*
 * landmark.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: filipe
 */
#include <tf.h>
#include <math.h>
#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include "landmark.h"

namespace post_slam
{
	tf::Transformer transformer;

	Landmark::Landmark()
	{
		x = 0.0;
		y = 0.0;
		radius = 0.0;
	}


	Landmark::Landmark(const Landmark &l)
	{
		x = l.x;
		y = l.y;
		radius = l.radius;
	}


	Landmark::Landmark(double landmark_position_x, double landmark_position_y, double landmark_radius)
	{
		x = landmark_position_x;
		y = landmark_position_y;
		radius = landmark_radius;
	}


	Landmark::~Landmark()
	{
	}


	/**
	 * check if the velodyne 32 laser beam can be part of a post
	 */
	int is_a_possible_post(unsigned short int distance[])
	{
		int i, first, all_lasers_have_same_range;
		double range, last_range;

		first = 1;
		all_lasers_have_same_range = 1;

		for (i = 10; i < 32; i++)
		{
			range = ((double) distance[i]) / 500.0;

			if (first)
			{
				last_range = range;
				first = 0;
			}
			else if (fabs(range - last_range) > 0.5)
			{
				all_lasers_have_same_range = 0;
				break;
			}

			last_range = range;
		}

		return all_lasers_have_same_range;
	}


	vector<Landmark*>
	detect_landmarks(carmen_velodyne_partial_scan_message *velodyne_partial_scan_message)
	{
		int i, j, k;
		double x, y, radius;
		double angle, range;
		vector<Landmark*> v;
		int post_mean_radius = 5;

		for (i = post_mean_radius; i < (velodyne_partial_scan_message->number_of_32_laser_shots - post_mean_radius); i++)
		{
			double mean_range = 0;
			double percentage_of_points_near_the_mean = 0.0;

			/** Computa o range medio na janela **/
			for (j = (i - post_mean_radius); j < (i + post_mean_radius); j++)
				for (k = 10; k < 32; k++)
					mean_range += ((double) velodyne_partial_scan_message->partial_scan[i].distance[k]) / 500.0;

			/** Computa a porcentagem de pontos proximos a essa media **/
			for (j = (i - post_mean_radius); j < (i + post_mean_radius); j++)
				for (k = 10; k < 32; k++)
					if (fabs(((double) velodyne_partial_scan_message->partial_scan[i].distance[k]) / 500.0 - mean_range) < 5.0)
						percentage_of_points_near_the_mean++;

			percentage_of_points_near_the_mean /= (2 * post_mean_radius * 22);

			/** se a porcentagem for alta, temos um possivel poste **/
			if (percentage_of_points_near_the_mean > 0.7)
			{
				angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_partial_scan_message->partial_scan[i].angle));

				double angle_left = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_partial_scan_message->partial_scan[i - post_mean_radius].angle));
				double angle_right = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_partial_scan_message->partial_scan[i + post_mean_radius - 1].angle));

				x = mean_range * cos(angle);
				y = mean_range * sin(angle);

				double x_left = mean_range * cos(angle_left);
				double y_left = mean_range * sin(angle_left);

				double x_right = mean_range * cos(angle_right);
				double y_right = mean_range * sin(angle_right);

				double dist_from_center_left = sqrt(pow(x_left - x, 2) + pow(y_left - y, 2));
				double dist_from_center_right = sqrt(pow(x_right - x, 2) + pow(y_right - y, 2));

				radius = (dist_from_center_left + dist_from_center_right) / 2.0;

				v.push_back(new Landmark(x, y, radius));

				i += post_mean_radius;
			}
		}

		return v;
	}


	void
	initialize_transforms(carmen_pose_3D_t sensor_board_pose_g, carmen_pose_3D_t velodyne_pose_g, carmen_pose_3D_t car_pose_g)
	{
		tf::Transform board_to_velodyne_pose;
		tf::Transform car_to_board_pose;
		tf::Transform world_to_car_pose;

		tf::Time::init();

		// initial car pose with respect to the world
		world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
		world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
		tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
		transformer.setTransform(world_to_car_transform, "world_to_car_transform");

		// board pose with respect to the car
		car_to_board_pose.setOrigin(tf::Vector3(sensor_board_pose_g.position.x, sensor_board_pose_g.position.y, sensor_board_pose_g.position.z));
		car_to_board_pose.setRotation(tf::Quaternion(sensor_board_pose_g.orientation.yaw, sensor_board_pose_g.orientation.pitch, sensor_board_pose_g.orientation.roll)); 				// yaw, pitch, roll
		tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
		transformer.setTransform(car_to_board_transform, "car_to_board_transform");

		// velodyne pose with respect to the board
		board_to_velodyne_pose.setOrigin(tf::Vector3(velodyne_pose_g.position.x, velodyne_pose_g.position.y, velodyne_pose_g.position.z));
		board_to_velodyne_pose.setRotation(tf::Quaternion(velodyne_pose_g.orientation.yaw, velodyne_pose_g.orientation.pitch, velodyne_pose_g.orientation.roll)); 				// yaw, pitch, roll
		tf::StampedTransform board_to_velodyne_transform(board_to_velodyne_pose, tf::Time(0), "/board", "/velodyne");
		transformer.setTransform(board_to_velodyne_transform, "board_to_velodyne_transform");
	}


	tf::StampedTransform
	get_transforms_from_velodyne_to_world(double car_x, double car_y, double car_theta)
	{
		tf::StampedTransform velodyne_to_world_transform;
		tf::Transform car_to_world_transform;

		car_to_world_transform.setOrigin(tf::Vector3(car_x, car_y, 0.0));
		car_to_world_transform.setRotation(tf::Quaternion(car_theta, 0.0, 0.0));

		tf::StampedTransform car_to_world_stamped_transform(car_to_world_transform, tf::Time(0), "/world", "/car");
		transformer.setTransform(car_to_world_stamped_transform, "car_to_world_stamped_transform");
		transformer.lookupTransform("/world", "/velodyne", tf::Time(0), velodyne_to_world_transform);

		return velodyne_to_world_transform;
	}


	void
	transform_landmark_pose(Landmark *landmark, tf::StampedTransform transform)
	{
		tf::Vector3 tf_point(landmark->x, landmark->y, 0.0);
		tf::Vector3 tf_moved_point(0.0, 0.0, 0.0);

		tf_moved_point = transform * tf_point;

		landmark->x = tf_moved_point.getX();
		landmark->y = tf_moved_point.getY();
	}


	void
	update_landmarks_position_in_the_world(vector<Landmark*> *landmarks, double car_x, double car_y, double car_theta)
	{
		unsigned int i;
		tf::StampedTransform velodyne_to_world_transform;

		velodyne_to_world_transform = get_transforms_from_velodyne_to_world(car_x, car_y, car_theta);

		for (i = 0; i < landmarks->size(); i++)
			transform_landmark_pose(landmarks->at(i), velodyne_to_world_transform);
	}
}


