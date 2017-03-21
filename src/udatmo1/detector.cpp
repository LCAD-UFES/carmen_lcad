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
Detector::update_moving_object_velocity(carmen_ackerman_traj_point_t &robot_pose)
{
	double average_v = 0.0;
	double count = 0.0;
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0 ; i--)
	{
		double v = -1.0; // invalid v
		if (moving_object[i].valid && moving_object[i + 1].valid)
		{
			double dist = carmen_distance_ackerman_traj(&moving_object[i].pose, &moving_object[i + 1].pose);
			// distance in the direction of the robot: https://en.wikipedia.org/wiki/Vector_projection
			double angle = atan2(moving_object[i].pose.y - moving_object[i + 1].pose.y, moving_object[i].pose.x - moving_object[i + 1].pose.x);
			double distance = dist * cos(angle - robot_pose.theta);
			double delta_t = moving_object[i].timestamp - moving_object[i + 1].timestamp;
			if (delta_t > 0.01 && delta_t < 0.2)
				v = distance / delta_t;
			if (v > 60.0)
				v = -1.0;
			if (v > -0.00001)
			{
				average_v += v;
				count += 1.0;
			}
		}
		moving_object[i].pose.v = v;
	}

	if (count > 0.0)
		average_v /= count;
}


int
Detector::detect(carmen_obstacle_distance_mapper_message *current_map,
				 carmen_rddf_road_profile_message *rddf,
				 int goal_index,
				 int rddf_pose_index,
				 carmen_ackerman_traj_point_t robot_pose,
				 double timestamp)
{
	moving_object[0].valid = false;
	moving_object[0].index = -1;
	moving_object[0].timestamp = 0.0;
	double circle_radius = (robot_config.width + 0.0) / 2.0; // metade da largura do carro + um espacco de guarda

	double distance = carmen_distance_ackerman_traj(&(rddf->poses[rddf_pose_index]), &robot_pose);

	double disp = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;
	if (distance < disp)
	{
		set_detected(false);
//		printf("## distance %lf, aqui 0, %d\n", distance, rddf_pose_index);
		return (-1);
	}

	carmen_point_t front_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&rddf->poses[rddf_pose_index], disp);
	carmen_position_t obstacle = carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(&front_car_pose, current_map);
	distance = sqrt((front_car_pose.x - obstacle.x) * (front_car_pose.x - obstacle.x) +
						   (front_car_pose.y - obstacle.y) * (front_car_pose.y - obstacle.y));

//	printf("distance %lf, ", distance);

	if (distance < circle_radius)
	{
		moving_object[0].valid = true;
		moving_object[0].pose.x = obstacle.x;
		moving_object[0].pose.y = obstacle.y;
		moving_object[0].car_pose = robot_pose;
		moving_object[0].index = goal_index;
		moving_object[0].timestamp = timestamp;

		update_moving_object_velocity(robot_pose);
		speed += moving_object[0].pose.v;

		if ((goal_index == 0) && (speed_front() > 0.1))
			set_detected(true);
		else
			set_detected(false);

//		printf("aqui 2, %d\n", rddf_pose_index);
		return (rddf_pose_index);
	}

	set_detected(false);

//	printf("aqui 3, %d\n", rddf_pose_index);
	return (-1);
}


void
Detector::shift()
{
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0 ; i--)
		moving_object[i + 1] = moving_object[i];
}


double
Detector::speed_front()
{
	double average_v = 0.0;
	double count = 0.0;
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0 ; i--)
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


carmen_ackerman_traj_point_t
Detector::get_moving_obstacle_position()
{
	return (moving_object[0].pose);
}


double
Detector::get_moving_obstacle_distance(carmen_ackerman_traj_point_t *robot_pose)
{
	double average_v = 0.0;
	double count = 0.0;
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE && i < 20; i++)
	{
		if (moving_object[i].valid)
		{
			average_v += DIST2D(*robot_pose, moving_object[i].pose);
			count += 1.0;
		}
	}

	if (count > 0.0)
		average_v /= count;

	return (average_v);
}
} // namespace udatmo
