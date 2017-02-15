#include "detector.h"

#include <carmen/collision_detection.h>

#include <cmath>

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
//			printf("i %d, v %lf, d %lf, df %lf, dtheta %lf, dt %lf\n", i, v, dist, distance, angle - robot_pose.theta, delta_t);
		}
//		if (moving_object[i].valid && moving_object[i + 1].valid)
//		{
//			double dist1 = carmen_distance_ackerman_traj(&moving_object[i].car_pose, &moving_object[i].pose);
//			double dist2 = carmen_distance_ackerman_traj(&moving_object[i + 1].car_pose, &moving_object[i + 1].pose);
//			double distance = dist1 - dist2;
//			double delta_t = moving_object[i].timestamp - moving_object[i + 1].timestamp;
//			if (delta_t > 0.01 && delta_t < 0.2)
//				v = moving_object[i].car_pose.v + distance / delta_t;
//			if (v > 60.0)
//				v = -1.0;
//			if (v > -0.00001)
//			{
//				average_v += v;
//				count += 1.0;
//			}
////			printf("i %d, v %lf, d %lf, df %lf, dtheta %lf, dt %lf\n", i, v, dist, distance, angle - robot_pose.theta, delta_t);
//		}
		moving_object[i].pose.v = v;
	}

	if (count > 0.0)
		average_v /= count;

//	printf("id %d, v %lf, av %lf\n", moving_object[0].index, moving_object[0].pose.v, average_v);
//	printf("\n");
//	fflush(stdout);
}

bool
Detector::detect(carmen_obstacle_distance_mapper_message *current_map,
				 carmen_rddf_road_profile_message *rddf,
				 int goal_index,
				 int rddf_pose_index,
				 carmen_ackerman_traj_point_t &car_pose,
				 carmen_ackerman_traj_point_t &robot_pose,
				 double timestamp)
{
	moving_object[0].valid = false;
	moving_object[0].index = -1;
	moving_object[0].timestamp = 0.0;
	double circle_radius = (robot_config.width + 0.0) / 2.0; // metade da largura do carro + um espacco de guarda

	if (carmen_distance_ackerman_traj(&rddf->poses[rddf_pose_index], &robot_pose) < robot_config.distance_between_front_and_rear_axles + 1.5)
		return set_detected(false);

	double disp = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;
	carmen_point_t front_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&rddf->poses[rddf_pose_index], disp);
	carmen_position_t obstacle = carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(&front_car_pose, current_map);
	double distance = sqrt(
		(front_car_pose.x - obstacle.x) * (front_car_pose.x - obstacle.x) +
		(front_car_pose.y - obstacle.y) * (front_car_pose.y - obstacle.y)
	);

//	virtual_laser_message.positions[virtual_laser_message.num_positions] = obstacle;
//	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_RED;
//	virtual_laser_message.num_positions++;
//	virtual_laser_message.positions[virtual_laser_message.num_positions].x = front_car_pose.x;
//	virtual_laser_message.positions[virtual_laser_message.num_positions].y = front_car_pose.y;
//	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_YELLOW;
//	virtual_laser_message.num_positions++;

	if (distance < circle_radius)
	{
		moving_object[0].valid = true;
		moving_object[0].pose.x = obstacle.x;
		moving_object[0].pose.y = obstacle.y;
		moving_object[0].car_pose = car_pose;
		moving_object[0].index = goal_index;
		moving_object[0].timestamp = timestamp;

		update_moving_object_velocity(robot_pose);
		speed += moving_object[0].pose.v;
		return set_detected(true);
	}

	return set_detected(false);
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

} // namespace udatmo
