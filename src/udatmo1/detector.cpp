#include "detector.h"

#include <carmen/collision_detection.h>
#include <carmen/global_graphics.h>

#include <cmath>

extern carmen_mapper_virtual_laser_message virtual_laser_message;

namespace udatmo
{

Detector::Detector(const carmen_robot_ackerman_config_t &robot_config)
{
	this->robot_config = robot_config;
	memset(moving_object, 0, sizeof(Obstacle) * MOVING_OBJECT_HISTORY_SIZE);
}


void
Detector::update_moving_object_velocity(carmen_robot_and_trailer_traj_point_t &robot_pose)
{
	int count = 0;
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0 ; i--)
	{
		double v = -1.0; // invalid v
		if (moving_object[i].valid && moving_object[i + 1].valid)
		{
			double dist = carmen_distance_ackerman_traj((carmen_robot_and_trailer_traj_point_t *)&moving_object[i].pose, (carmen_robot_and_trailer_traj_point_t *)&moving_object[i + 1].pose);
			// distance in the direction of the robot: https://en.wikipedia.org/wiki/Vector_projection
			double angle = atan2(moving_object[i].pose.y - moving_object[i + 1].pose.y, moving_object[i].pose.x - moving_object[i + 1].pose.x);
			double distance = dist * cos(angle - robot_pose.theta);
			double delta_t = moving_object[i].timestamp - moving_object[i + 1].timestamp;
			if (delta_t > 0.01 && delta_t < 0.2)
				v = distance / delta_t; // @@@ Alberto: nao acumula para tirar a media com count?

			if (v > (145.0 / 3.6))
				v = -1.0; // @@@ Alberto: como usa este -1.0?
			else
				count++; // @@@ Alberto: nao usa o count?
		}
		moving_object[i].pose.v = v;
	}
}


int
Detector::detect(carmen_obstacle_distance_mapper_map_message *current_map,
				 carmen_rddf_road_profile_message *rddf,
				 int goal_index,
				 int rddf_pose_index,
				 carmen_robot_and_trailer_traj_point_t robot_pose_,
				 double circle_radius,
				 double displacement,
				 double timestamp)
{
	carmen_robot_and_trailer_traj_point_t robot_pose = {robot_pose_.x, robot_pose_.y, robot_pose_.theta, 0.0, robot_pose_.v, robot_pose_.phi};

//	printf("w %d, i %d\n", obstacle_already_detected, rddf_pose_index);
	if (rddf_pose_index == 0)
		obstacle_already_detected = false;

	double disp = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;
	carmen_robot_and_trailer_pose_t front_car_pose_ = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&rddf->poses[rddf_pose_index], disp);
	carmen_point_t front_car_pose = {front_car_pose_.x, front_car_pose_.y, front_car_pose_.theta};
	if (displacement > 0.0)
	{
		front_car_pose.x = front_car_pose.x + displacement * cos(rddf->poses[rddf_pose_index].theta + M_PI / 2.0);
		front_car_pose.y = front_car_pose.y + displacement * sin(rddf->poses[rddf_pose_index].theta + M_PI / 2.0);
	}
	else if (displacement < 0.0)
	{
		front_car_pose.x = front_car_pose.x - displacement * cos(rddf->poses[rddf_pose_index].theta - M_PI / 2.0);
		front_car_pose.y = front_car_pose.y - displacement * sin(rddf->poses[rddf_pose_index].theta - M_PI / 2.0);
	}
	carmen_position_t obstacle = carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(&front_car_pose, current_map);
	double distance = DIST2D(front_car_pose, obstacle);

//	printf("distance %lf, ", distance);

	if ((displacement == 0.0) && !obstacle_already_detected && (distance < circle_radius) && // false)
		(DIST2D(robot_pose, rddf->poses[rddf_pose_index]) < (disp - circle_radius)))
	{	// Este if eh soh para obstaculos frontais (displacement == 0.0).
		// Se entrar ha obstaculo movel em cima do carro e este eh descartado, ja que nao faz sentido detectar obstaculo movel em cima do carro
		obstacle_already_detected = true;

		moving_object[0].valid = false;
		moving_object[0].index = -1;
		moving_object[0].rddf_pose_index = -1;
		moving_object[0].timestamp = 0.0;

		set_detected(false);
	}
	else if (!obstacle_already_detected && (distance < circle_radius))
	{
		obstacle_already_detected = true;

		moving_object[0].valid = true;
		moving_object[0].pose.x = obstacle.x;
		moving_object[0].pose.y = obstacle.y;
		moving_object[0].car_pose = robot_pose;
		moving_object[0].rddf_front_car_pose = front_car_pose;
		moving_object[0].index = goal_index;
		moving_object[0].rddf_pose_index = rddf_pose_index;
		moving_object[0].timestamp = timestamp;

		update_moving_object_velocity(robot_pose);
		speed += moving_object[0].pose.v;

		if ((goal_index == 0) && (speed_front() > 0.1))
			set_detected(true);
		else
			set_detected(false);

//		printf("aqui 2, %d\n", rddf_pose_index);

		if (speed_front() > 0.1)
			return (rddf_pose_index);
		else
			return (-1);
	}
	else if (!obstacle_already_detected)
	{
		moving_object[0].valid = false;
		moving_object[0].index = -1;
		moving_object[0].rddf_pose_index = -1;
		moving_object[0].timestamp = 0.0;
	}
	else if (obstacle_already_detected)
	{
		if (moving_object[0].valid && (speed_front() > 0.1))
			return (moving_object[0].rddf_pose_index);
		else
			return (-1);
	}

	set_detected(false);

//	printf("aqui 3, %d\n", rddf_pose_index);
	return (-1);
}


int
Detector::detect(carmen_obstacle_distance_mapper_map_message *current_map,
				 carmen_rddf_road_profile_message *rddf,
				 int goal_index,
				 int rddf_pose_index,
				 carmen_robot_and_trailer_traj_point_t robot_pose,
				 double circle_radius,
				 double timestamp)
{
	return (Detector::detect(current_map, rddf, goal_index, rddf_pose_index, robot_pose, circle_radius, 0.0, timestamp));
}


void
Detector::copy_state(Detector *detector)
{
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE; i++)
		moving_object[i] = detector->moving_object[i];

	detected = detector->detected;
	obstacle_already_detected = detector->obstacle_already_detected;
}


void
Detector::shift()
{
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0; i--)
		moving_object[i + 1] = moving_object[i];
}


double
Detector::speed_front()
{
	double average_v = 0.0;
	double count = 0.0;
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0; i--)
	{
		if (moving_object[i].valid && (moving_object[i].pose.v > -0.0001))
		{
			average_v += moving_object[i].pose.v;
			count += 1.0;
		}
	}

	if (count > 0.0)
		average_v /= count;

//	return speed();
	return (average_v);
}


carmen_robot_and_trailer_traj_point_t
Detector::get_moving_obstacle_position()
{
	return (moving_object[0].pose);
}


double
Detector::get_moving_obstacle_distance(carmen_robot_and_trailer_traj_point_t robot_pose)
{
	double average_dist = 0.0;
	double count = 0.0;
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE && i < 20; i++)
	{
		if (moving_object[i].valid)
		{
			average_dist += DIST2D(robot_pose, moving_object[i].pose);
			count += 1.0;
		}
	}

	if (count > 0.0)
		average_dist /= count;
	else
		average_dist = 1000.0;

	return (average_dist);
}
} // namespace udatmo
