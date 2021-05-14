#include <stdexcept>
#include <carmen/global_graphics.h>
#include "udatmo.h"
#include "detector.h"

using udatmo::Detector;

extern carmen_mapper_virtual_laser_message virtual_laser_message;

static Detector *detector = NULL;
static Detector *detector_center = NULL;
static Detector *detector_left = NULL;
static Detector *detector_right = NULL;


void
udatmo_init(const carmen_robot_ackerman_config_t robot_config)
{
	if (detector != NULL)
		throw std::runtime_error("uDATMO module already initialized");

	detector = new Detector(robot_config);
	detector_center = new Detector(robot_config);
	detector_left = new Detector(robot_config);
	detector_right = new Detector(robot_config);
}


bool
udatmo_obstacle_detected(double timestamp)
{
	static double last_obstacle_detected_timestamp = 0.0;

	if (detector->detected)
		last_obstacle_detected_timestamp = timestamp;

	if (fabs(timestamp - last_obstacle_detected_timestamp) < 3.0)
		return (true);
	else
		return (false);
}


void
udatmo_clear_detected(void)
{
	detector->detected = false;
	detector_center->detected = false;
	detector_left->detected = false;
	detector_right->detected = false;
}


void
udatmo_shift_history(void)
{
	detector->shift();
	detector_center->shift();
	detector_left->shift();
	detector_right->shift();
}


bool
objects_coinside(Detector *detector1, Detector *detector2)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int k = i + j - 3 / 2;
			if ((k < 0) || (k > MOVING_OBJECT_HISTORY_SIZE - 1))
				continue;

			double distance = 2.0;
			if (detector1->moving_object[i].valid && detector2->moving_object[k].valid)
				distance = DIST2D(detector1->moving_object[i].pose, detector2->moving_object[k].pose);

			if (distance < 1.0)
				return (true);
		}
	}
	return (false);
}


int
udatmo_detect_obstacle_index(carmen_obstacle_distance_mapper_map_message *current_map,
							carmen_rddf_road_profile_message *rddf,
							int goal_index,
							int rddf_pose_index,
							carmen_robot_and_trailer_traj_point_t robot_pose,
							double timestamp)
{
	int index = detector->detect(current_map, rddf, goal_index, rddf_pose_index, robot_pose,
			detector->robot_config.behaviour_selector_main_central_lane_obstacles_safe_distance,
			0.0, timestamp);
	int index_left = detector_left->detect(current_map, rddf, goal_index, rddf_pose_index, robot_pose,
			detector->robot_config.behaviour_selector_lateral_lane_obstacles_safe_distance,
			(detector->robot_config.behaviour_selector_lateral_lane_displacement), timestamp);
	int index_right = detector_right->detect(current_map, rddf, goal_index, rddf_pose_index, robot_pose,
			detector->robot_config.behaviour_selector_lateral_lane_obstacles_safe_distance,
			-(detector->robot_config.behaviour_selector_lateral_lane_displacement), timestamp);
	int index_center = detector_center->detect(current_map, rddf, goal_index, rddf_pose_index, robot_pose,
			detector->robot_config.behaviour_selector_central_lane_obstacles_safe_distance,
			0.0, timestamp);

	int moving_object_index = index;

	if ((index_center != -1) && (index_left != -1) && objects_coinside(detector_center, detector_left) && (goal_index == 0))
	{
		if (index == -1)
		{
			detector->copy_state(detector_left);
			moving_object_index = index_left;
		}
		else
		{
			if ((detector_left->get_moving_obstacle_distance(robot_pose) < detector->get_moving_obstacle_distance(robot_pose)) &&
				!objects_coinside(detector, detector_left))
			{
				detector->copy_state(detector_left);
				moving_object_index = index_left;
			}
			else
				moving_object_index = index;
		}
	}

	if ((index_center != -1) && (index_right != -1) && objects_coinside(detector_center, detector_right) && (goal_index == 0))
	{
		if (index == -1)
		{
			detector->copy_state(detector_right);
			moving_object_index = index_right;
		}
		else
		{
			if ((detector_right->get_moving_obstacle_distance(robot_pose) < detector->get_moving_obstacle_distance(robot_pose)) &&
				!objects_coinside(detector, detector_right))
			{
				detector->copy_state(detector_right);
				moving_object_index = index_right;
			}
			else
				moving_object_index = index;
		}
	}

	if (moving_object_index != -1)
	{
		virtual_laser_message.positions[virtual_laser_message.num_positions].x = detector->moving_object[0].pose.x;
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = detector->moving_object[0].pose.y;
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_GREEN;
		virtual_laser_message.num_positions++;
	}
	return (moving_object_index);
}


double
udatmo_speed_front(void)
{
	return (detector->speed_front());
}


double
udatmo_speed_left(void)
{
	return (detector_left->speed_front());
}


double
udatmo_speed_right(void)
{
	return (detector_right->speed_front());
}


double
udatmo_speed_center(void)
{
	return (detector_center->speed_front());
}


carmen_robot_and_trailer_traj_point_t
udatmo_get_moving_obstacle_position(void)
{
	return (detector->get_moving_obstacle_position());
}


carmen_robot_and_trailer_traj_point_t
udatmo_get_moving_obstacle_position_left(void)
{
	return (detector_left->get_moving_obstacle_position());
}


carmen_robot_and_trailer_traj_point_t
udatmo_get_moving_obstacle_position_right(void)
{
	return (detector_right->get_moving_obstacle_position());
}


double
udatmo_get_moving_obstacle_distance(carmen_robot_and_trailer_traj_point_t robot_pose, carmen_robot_ackerman_config_t *robot_config __attribute__ ((unused)))
{
	double distance = detector->get_moving_obstacle_distance(robot_pose);

	return (distance);
}


void
udatmo_set_behaviour_selector_central_lane_obstacles_safe_distance(double behaviour_selector_central_lane_obstacles_safe_distance)
{
	detector->robot_config.behaviour_selector_central_lane_obstacles_safe_distance = behaviour_selector_central_lane_obstacles_safe_distance;
	detector_center->robot_config.behaviour_selector_central_lane_obstacles_safe_distance = behaviour_selector_central_lane_obstacles_safe_distance;
	detector_left->robot_config.behaviour_selector_central_lane_obstacles_safe_distance = behaviour_selector_central_lane_obstacles_safe_distance;
	detector_right->robot_config.behaviour_selector_central_lane_obstacles_safe_distance = behaviour_selector_central_lane_obstacles_safe_distance;
}


void
udatmo_set_model_predictive_planner_obstacles_safe_distance(double model_predictive_planner_obstacles_safe_distance)
{
	detector->robot_config.model_predictive_planner_obstacles_safe_distance = model_predictive_planner_obstacles_safe_distance;
	detector_center->robot_config.model_predictive_planner_obstacles_safe_distance = model_predictive_planner_obstacles_safe_distance;
	detector_left->robot_config.model_predictive_planner_obstacles_safe_distance = model_predictive_planner_obstacles_safe_distance;
	detector_right->robot_config.model_predictive_planner_obstacles_safe_distance = model_predictive_planner_obstacles_safe_distance;
}
