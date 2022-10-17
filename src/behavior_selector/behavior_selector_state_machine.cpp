#include <vector>
#include <carmen/collision_detection.h>
#include <carmen/global_graphics.h>
#include <carmen/rddf_util.h>
#include <carmen/rddf_interface.h>
#include <carmen/task_manager_interface.h>

#include "behavior_selector.h"

#define NO_NARROW_LANE_SIGN_AHEAD	0
#define NARROW_LANE_BEGIN			1
#define NARROW_LANE_END				2

#define NO_ENGINE_BRAKE_SIGN_AHEAD	0
#define ENGINE_BRAKE_ON				1
#define ENGINE_BRAKE_OFF			2

#define NO_TURN_LEFT_INDICATOR_SIGN_AHEAD	0
#define TURN_LEFT_INDICATOR_ON				1
#define TURN_LEFT_INDICATOR_OFF				2

#define NO_TURN_RIGHT_INDICATOR_SIGN_AHEAD	0
#define TURN_RIGHT_INDICATOR_ON				1
#define TURN_RIGHT_INDICATOR_OFF			2

#define NO_SET_MAX_GEAR_SIGN_AHEAD	0
#define SET_MAX_GEAR_1	1
#define SET_MAX_GEAR_2	2
#define SET_MAX_GEAR_3	3
#define SET_MAX_GEAR_4	4
#define SET_MAX_GEAR_5	5
#define SET_MAX_GEAR_6	6
#define SET_MAX_GEAR_7	7
#define SET_MAX_GEAR_8	8
#define SET_MAX_GEAR_9	9

using namespace std;

extern bool wait_start_moving;
extern bool autonomous;
extern bool all_paths_has_collision_and_goal_is_not_an_annotation;

extern carmen_rddf_annotation_message last_rddf_annotation_message;
extern carmen_robot_ackerman_config_t robot_config;

static double wait_for_given_seconds_start_time = 0.0;

carmen_ford_escape_signals_message signals_msg = {};

bool
forward_waypoint_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_robot_and_trailers_traj_point_t *nearest_forward_waypoint_ahead = get_nearest_forward_waypoint_ahead();

	if (nearest_forward_waypoint_ahead == NULL)
		return (false);

	double distance_to_forward_waypoint = DIST2D_P(nearest_forward_waypoint_ahead, &current_robot_pose_v_and_phi);
	double distance_to_act = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_forward_waypoint);

	if (distance_to_act >= distance_to_forward_waypoint)
		return (true);
	else
		return (false);
}


double
distance_to_forward_waypoint(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_robot_and_trailers_traj_point_t *nearest_forward_waypoint_ahead = get_nearest_forward_waypoint_ahead();

	if (nearest_forward_waypoint_ahead == NULL)
		return (1000.0);

	double distance_to_forward_waypoint = DIST2D_P(nearest_forward_waypoint_ahead, &current_robot_pose_v_and_phi);

	return (distance_to_forward_waypoint);
}


bool
reverse_waypoint_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_robot_and_trailers_traj_point_t *nearest_reverse_waypoint_ahead = get_nearest_reverse_waypoint_ahead();

	if (nearest_reverse_waypoint_ahead == NULL)
		return (false);

	double distance_to_reverse_waypoint = DIST2D_P(nearest_reverse_waypoint_ahead, &current_robot_pose_v_and_phi);
	double distance_to_act = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_reverse_waypoint);

	if (distance_to_act >= distance_to_reverse_waypoint)
		return (true);
	else
		return (false);
}


bool
pedestrian_near_pose_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	static double last_pedestrian_near_pose_ahead_timestamp = 0.0;

	carmen_robot_and_trailers_traj_point_t *waypoint_near_to_nearest_pedestrian_ahead = get_waypoint_near_to_nearest_pedestrian_ahead();

	if (waypoint_near_to_nearest_pedestrian_ahead == NULL)
		return (false);

	double distance_to_waypoint_near_to_nearest_pedestrian_ahead = DIST2D_P(waypoint_near_to_nearest_pedestrian_ahead, &current_robot_pose_v_and_phi);
	double distance_to_act = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_waypoint_near_to_nearest_pedestrian_ahead);

	if (distance_to_act >= distance_to_waypoint_near_to_nearest_pedestrian_ahead)
		last_pedestrian_near_pose_ahead_timestamp = timestamp;

	if (timestamp - last_pedestrian_near_pose_ahead_timestamp < 1.5)
		return (true);
	else
		return (false);
}


bool
path_final_pose_reached(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_robot_and_trailers_traj_point_t *path_final_pose = get_path_final_pose();

	if (path_final_pose == NULL)
		return (false);

	double distance_to_path_final_pose = DIST2D_P(path_final_pose, &current_robot_pose_v_and_phi);

	if ((distance_to_path_final_pose < robot_config.distance_between_front_and_rear_axles) &&
		(fabs(current_robot_pose_v_and_phi.v) < 0.4) &&
		((distance_to_path_final_pose < 0.5) || nearest_pose_is_the_final_pose(current_robot_pose_v_and_phi)))
		return (true);
	else
		return (false);
}


double
distance_to_reverse_waypoint(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_robot_and_trailers_traj_point_t *nearest_reverse_waypoint_ahead = get_nearest_reverse_waypoint_ahead();

	if (nearest_reverse_waypoint_ahead == NULL)
		return (1000.0);

	double distance_to_reverse_waypoint = DIST2D_P(nearest_reverse_waypoint_ahead, &current_robot_pose_v_and_phi);

	return (distance_to_reverse_waypoint);
}


double
distance_to_waypoint_near_to_nearest_pedestrian_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_robot_and_trailers_traj_point_t *waypoint_near_to_nearest_pedestrian_ahead = get_waypoint_near_to_nearest_pedestrian_ahead();

	if (waypoint_near_to_nearest_pedestrian_ahead == NULL)
		return (false);

	double distance_to_waypoint_near_to_nearest_pedestrian_ahead = DIST2D_P(waypoint_near_to_nearest_pedestrian_ahead, &current_robot_pose_v_and_phi);

	return (distance_to_waypoint_near_to_nearest_pedestrian_ahead);
}


bool
queue_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_queue_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_QUEUE,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_queue_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_queue_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_robot_and_trailers_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	if ((distance_to_act_on_annotation >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_queue_annotation->annotation_point))
		return (true);
	else
		return (false);
}


bool
stop_sign_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_stop_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_STOP,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_stop_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_stop_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_robot_and_trailers_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	if ((distance_to_act_on_annotation >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_stop_annotation->annotation_point))
		return (true);
	else
		return (false);
}


int
narrow_lane_sign_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_narrow_lane_sign_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_NARROW_LANE,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_narrow_lane_sign_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_narrow_lane_sign_annotation->annotation_point, current_robot_pose_v_and_phi);

	int last_goal_list_size;
	carmen_robot_and_trailers_traj_point_t *goal_list = behavior_selector_get_last_goal_list(last_goal_list_size);
	double distance_to_first_goal = distance_to_annotation;
	if (last_goal_list_size)
		distance_to_first_goal = DIST2D(current_robot_pose_v_and_phi, goal_list[0]);

	if ((distance_to_first_goal >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(current_robot_pose_v_and_phi, nearest_narrow_lane_sign_annotation->annotation_point))
	{
		if (nearest_narrow_lane_sign_annotation->annotation_code == RDDF_ANNOTATION_CODE_NARROW_LANE_BEGIN)
			return (NARROW_LANE_BEGIN);
		else
			return (NARROW_LANE_END);
	}
	else
		return (NO_NARROW_LANE_SIGN_AHEAD);
}


int
engine_brake_sign_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_engine_brake_sign_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_RETARDER_BRAKE,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_engine_brake_sign_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_engine_brake_sign_annotation->annotation_point, current_robot_pose_v_and_phi);

	int last_goal_list_size;
	carmen_robot_and_trailers_traj_point_t *goal_list = behavior_selector_get_last_goal_list(last_goal_list_size);
	double distance_to_first_goal = distance_to_annotation;
	if (last_goal_list_size)
		distance_to_first_goal = DIST2D(current_robot_pose_v_and_phi, goal_list[0]);

	if ((distance_to_first_goal >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(current_robot_pose_v_and_phi, nearest_engine_brake_sign_annotation->annotation_point))
	{
		if (nearest_engine_brake_sign_annotation->annotation_code == RDDF_ANNOTATION_CODE_RETARDER_BRAKE_ON)
			return (ENGINE_BRAKE_ON);
		else
			return (ENGINE_BRAKE_OFF);
	}
	else
		return (NO_ENGINE_BRAKE_SIGN_AHEAD);
}


int
turn_left_indicator_sign_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_turn_left_indicator_sign_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_TURN_LEFT_INDICATOR,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_turn_left_indicator_sign_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_turn_left_indicator_sign_annotation->annotation_point, current_robot_pose_v_and_phi);

	int last_goal_list_size;
	carmen_robot_and_trailers_traj_point_t *goal_list = behavior_selector_get_last_goal_list(last_goal_list_size);
	double distance_to_first_goal = distance_to_annotation;
	if (last_goal_list_size)
		distance_to_first_goal = DIST2D(current_robot_pose_v_and_phi, goal_list[0]);

	if ((distance_to_first_goal >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(current_robot_pose_v_and_phi, nearest_turn_left_indicator_sign_annotation->annotation_point))
	{
		if (nearest_turn_left_indicator_sign_annotation->annotation_code == RDDF_ANNOTATION_CODE_TURN_LEFT_INDICATOR_ON)
			return (TURN_LEFT_INDICATOR_ON);
		else
			return (TURN_LEFT_INDICATOR_OFF);
	}
	else
		return (NO_TURN_LEFT_INDICATOR_SIGN_AHEAD);
}


int
turn_right_indicator_sign_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_turn_right_indicator_sign_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_TURN_RIGHT_INDICATOR,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_turn_right_indicator_sign_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_turn_right_indicator_sign_annotation->annotation_point, current_robot_pose_v_and_phi);

	int last_goal_list_size;
	carmen_robot_and_trailers_traj_point_t *goal_list = behavior_selector_get_last_goal_list(last_goal_list_size);
	double distance_to_first_goal = distance_to_annotation;
	if (last_goal_list_size)
		distance_to_first_goal = DIST2D(current_robot_pose_v_and_phi, goal_list[0]);

	if ((distance_to_first_goal >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(current_robot_pose_v_and_phi, nearest_turn_right_indicator_sign_annotation->annotation_point))
	{
		if (nearest_turn_right_indicator_sign_annotation->annotation_code == RDDF_ANNOTATION_CODE_TURN_RIGHT_INDICATOR_ON)
			return (TURN_RIGHT_INDICATOR_ON);
		else
			return (TURN_RIGHT_INDICATOR_OFF);
	}
	else
		return (NO_TURN_RIGHT_INDICATOR_SIGN_AHEAD);
}


int
set_max_gear_sign_ahead(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_set_max_gear_sign_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_GEAR,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_set_max_gear_sign_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_set_max_gear_sign_annotation->annotation_point, current_robot_pose_v_and_phi);

	int last_goal_list_size;
	carmen_robot_and_trailers_traj_point_t *goal_list = behavior_selector_get_last_goal_list(last_goal_list_size);
	double distance_to_first_goal = distance_to_annotation;
	if (last_goal_list_size)
		distance_to_first_goal = DIST2D(current_robot_pose_v_and_phi, goal_list[0]);

	if ((distance_to_first_goal >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(current_robot_pose_v_and_phi, nearest_set_max_gear_sign_annotation->annotation_point))
	{
		switch(nearest_set_max_gear_sign_annotation->annotation_code)
		{
			case RDDF_ANNOTATION_CODE_GEAR_1:
				return (SET_MAX_GEAR_1);
				break;

			case RDDF_ANNOTATION_CODE_GEAR_2:
				return (SET_MAX_GEAR_2);
				break;
			
			case RDDF_ANNOTATION_CODE_GEAR_3:
				return (SET_MAX_GEAR_3);
				break;
			
			case RDDF_ANNOTATION_CODE_GEAR_4:
				return (SET_MAX_GEAR_4);
				break;
			
			case RDDF_ANNOTATION_CODE_GEAR_5:
				return (SET_MAX_GEAR_5);
				break;
			
			case RDDF_ANNOTATION_CODE_GEAR_6:
				return (SET_MAX_GEAR_6);
				break;
			
			case RDDF_ANNOTATION_CODE_GEAR_7:
				return (SET_MAX_GEAR_7);
				break;
			
			case RDDF_ANNOTATION_CODE_GEAR_8:
				return (SET_MAX_GEAR_8);
				break;
			
			case RDDF_ANNOTATION_CODE_GEAR_9:
				return (SET_MAX_GEAR_9);
				break;
		}
	}
	else
		return (NO_SET_MAX_GEAR_SIGN_AHEAD);
}


double
distance_to_stop_sign(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP)
		return (distance_to_annotation);
	else
		return (1000.0);
}


bool
wait_for_given_seconds(double seconds)
{
	if (wait_for_given_seconds_start_time == 0.0)
		wait_for_given_seconds_start_time = carmen_get_time();

	double t = carmen_get_time();
	if (t - wait_for_given_seconds_start_time < seconds)
	{
		return (false);
	}
	else
	{
		wait_for_given_seconds_start_time = 0.0;
		return (true);
	}
}


void
clear_wait_for_given_seconds()
{
	wait_for_given_seconds_start_time = 0.0;
}


double
distance_to_red_traffic_light(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp) &&
		(nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP))
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_traffic_light_stop(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, false);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP)
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_busy_pedestrian_track(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp) &&
		(nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP))
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_must_yield(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi,
		path_collision_info_t path_collision_info, double timestamp)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp) &&
		(nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_YIELD))
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_yield(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_YIELD)
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_pedestrian_track_stop(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, false);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP)
		return (distance_to_annotation);
	else
		return (1000.0);
}


int
perform_state_action(carmen_behavior_selector_state_message *decision_making_state_msg)
{
	switch (decision_making_state_msg->low_level_state)
	{
		case Initializing:
			break;
		case Stopped:
			break;

		case Free_Running:
			break;
		case Free_Reverse_Running:
			break;


		case Stopping_At_Red_Traffic_Light:
			break;
		case Stopped_At_Red_Traffic_Light_S0:
//			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Red_Traffic_Light_S1:
			break;
		case Stopped_At_Red_Traffic_Light_S2:
//			carmen_navigator_ackerman_go();
			break;


		case Stopping_At_Busy_Pedestrian_Track:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S0:
//			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Busy_Pedestrian_Track_S1:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S2:
//			carmen_navigator_ackerman_go();
			break;

		
		case Stopping_At_Busy_Queue:
			break;
		case Stopped_At_Busy_Queue_S0:
			break;
		case Stopped_At_Busy_Queue_S1:
			break;
		case Stopped_At_Busy_Queue_S2:
			break;


		case Stopping_At_Yield:
			break;
		case Stopped_At_Yield_S0:
//			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Yield_S1:
			break;
		case Stopped_At_Yield_S2:
//			carmen_navigator_ackerman_go();
			break;


		case Stopping_At_Stop_Sign:
			break;
		case Stopped_At_Stop_Sign_S0:
//			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Stop_Sign_S1:
			break;
		case Stopped_At_Stop_Sign_S2:
			break;


		case Stopping_To_Reverse:
			break;
		case Stopped_At_Reverse_S0:
			break;
		case Stopped_At_Reverse_S1:
			break;
		case Stopped_At_Reverse_S2:
			break;


		case Stopping_To_Pedestrian:
			break;
		case Stopped_At_Pedestrian_S0:
			break;
		case Stopped_At_Pedestrian_S1:
			break;
		case Stopped_At_Pedestrian_S2:
			break;


		case Stopping_To_Go_Forward:
			break;
		case Stopped_At_Go_Forward_S0:
			break;
		case Stopped_At_Go_Forward_S1:
			break;
		case Stopped_At_Go_Forward_S2:
			break;


		case Stopping_At_Unavoidable_Obstacle:
			break;
		case Stopped_At_Unavoidable_Obstacle_S0:
			break;


		case End_Of_Path_Reached:
			break;
		case End_Of_Path_Reached2:
			break;


		case Recovering_From_Error:
			break;


		default:
			printf("Error: Unknown state in perform_state_action()\n");
			return (1);
	}

	return (0);
}


bool
robot_reached_non_return_point(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_pedestrian_track_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_pedestrian_track_annotation == NULL)
		return (false);

	if (DIST2D(current_robot_pose_v_and_phi, nearest_pedestrian_track_annotation->annotation_point) < 
		(MIN_DISTANCE_TO_CONSIDER_CROSSWALK + robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels))
	{
		// printf("non return\n");
		return (true);
	}
	return (false);
}


bool
within_narrow_passage(carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi,
		carmen_robot_and_trailers_traj_point_t *last_valid_goal, carmen_behavior_selector_state_message *decision_making_state_msg __attribute__((unused)))
{
	carmen_annotation_t *barrier_annotation = carmen_behavior_selector_get_nearest_specified_annotation(RDDF_ANNOTATION_TYPE_BARRIER, last_rddf_annotation_message,
			&current_robot_pose_v_and_phi);

	if (barrier_annotation)
	{
		double size_front;
		double size_back;
		carmen_rddf_get_barrier_alignment_segments_sizes(barrier_annotation, &size_front, &size_back);
		if ((size_front == 0.0) && (size_back == 0.0))
			return (false);

		vector<carmen_robot_and_trailers_traj_point_t> rectilinear_route_segment =
				carmen_rddf_compute_rectilinear_route_segment(*barrier_annotation, size_front, size_back, 0.2);

		int index = carmen_rddf_index_of_point_within_rectlinear_route_segment(rectilinear_route_segment, current_robot_pose_v_and_phi);
		int index2 = -1;
		if (last_valid_goal)
			index2 = carmen_rddf_index_of_point_within_rectlinear_route_segment(rectilinear_route_segment, *last_valid_goal);
		if ((index != -1) || (index2 != -1))
			return (true);
		else
			return (false);
	}
//	else if (decision_making_state_msg->task == BEHAVIOR_SELECTOR_MOVE_TO_ENGAGE_POSE)
//		return (true);
	else
		return (false);
}


bool
could_not_compute_the_route(carmen_route_planner_state_t route_planner_state)
{
	if (route_planner_state == COULD_NOT_COMPUTE_THE_ROUTE)
		return (true);
	else
		return(false);
}


bool
route_was_recomputed(carmen_route_planner_state_t route_planner_state)
{
	if (route_planner_state == ROUTE_RECOMPUTED)
		return (true);
	else
		return(false);
}


bool
still_in_route(carmen_route_planner_state_t route_planner_state)
{
	if ((route_planner_state == EXECUTING_OFFROAD_PLAN) ||
		(route_planner_state == PUBLISHING_ROUTE) ||
		(route_planner_state == IN_RECTLINEAR_ROUTE_SEGMENT))
		return (true);
	else
		return(false);
}


int
perform_state_transition(carmen_behavior_selector_state_message *decision_making_state_msg,
		carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi,
		path_collision_info_t path_collision_info, double timestamp)
{
	switch (decision_making_state_msg->low_level_state)
	{
		case Initializing:
			decision_making_state_msg->low_level_state = Stopped;
			break;
		case Stopped:
			decision_making_state_msg->low_level_state_flags &= ~CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;

			if (autonomous && (!path_final_pose_reached(current_robot_pose_v_and_phi) || still_in_route(decision_making_state_msg->route_planner_state)))
			{
				if (going_forward())
					decision_making_state_msg->low_level_state_flags &= ~CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;
				else
					decision_making_state_msg->low_level_state_flags |= CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;

				if (wait_for_given_seconds(2.0))
				{
					if (going_forward())
						decision_making_state_msg->low_level_state = Free_Running;
					else
						decision_making_state_msg->low_level_state = Free_Reverse_Running;
				}
			}
			break;


		case Free_Running:
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			else if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Red_Traffic_Light;
			else if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp) && !robot_reached_non_return_point(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Busy_Pedestrian_Track;
			else if (busy_queue_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Busy_Queue;
			else if (must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Yield;
			else if (stop_sign_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Stop_Sign;
			else if (pedestrian_near_pose_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_To_Pedestrian;
			else if (reverse_waypoint_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_To_Reverse;
			else if (path_final_pose_reached(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = End_Of_Path_Reached;
			else if (all_paths_has_collision_and_goal_is_not_an_annotation == true)
				decision_making_state_msg->low_level_state = Stopping_At_Unavoidable_Obstacle;

			decision_making_state_msg->low_level_state_flags &= ~CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;
			break;


		case Free_Reverse_Running:
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			else if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Red_Traffic_Light;
			else if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp) && !robot_reached_non_return_point(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Busy_Pedestrian_Track;
			else if (busy_queue_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Busy_Queue;
			else if (must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Yield;
			else if (stop_sign_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Stop_Sign;
			else if (forward_waypoint_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_To_Go_Forward;
			else if (path_final_pose_reached(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = End_Of_Path_Reached;

			decision_making_state_msg->low_level_state_flags |= CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS;
			break;


		case End_Of_Path_Reached:
			if (wait_for_given_seconds(1.0))// && all_paths_has_collision_and_goal_is_not_an_annotation == false)
				decision_making_state_msg->low_level_state = End_Of_Path_Reached2;
			// else if(path_final_pose_reached(current_robot_pose_v_and_phi) == false)
			// 	decision_making_state_msg->low_level_state = Stopping_At_Unavoidable_Obstacle;
			break;


		case End_Of_Path_Reached2:
			if (wait_for_given_seconds(1.0))// &&	all_paths_has_collision_and_goal_is_not_an_annotation == false)
				decision_making_state_msg->low_level_state = Stopped;
			// else if(path_final_pose_reached(current_robot_pose_v_and_phi) == false)
			// 	decision_making_state_msg->low_level_state = Stopping_At_Unavoidable_Obstacle;
			break;


		case Stopping_At_Unavoidable_Obstacle:
			if (current_robot_pose_v_and_phi.v < 0.15)
				decision_making_state_msg->low_level_state = Stopped_At_Unavoidable_Obstacle_S0;
			break;

		case Stopped_At_Unavoidable_Obstacle_S0:
			if (route_was_recomputed(decision_making_state_msg->route_planner_state) && all_paths_has_collision_and_goal_is_not_an_annotation == false)
				decision_making_state_msg->low_level_state = Free_Running;
			else if (wait_for_given_seconds(2.0) && still_in_route(decision_making_state_msg->route_planner_state))
				decision_making_state_msg->low_level_state = Free_Running;
			// else if (could_not_compute_the_route(decision_making_state_msg->route_planner_state))
			// {
			// 	if(wait_for_given_seconds(1.0))
			// 		decision_making_state_msg->low_level_state = Stopped;
			// }

			break;



		case Stopping_At_Red_Traffic_Light:
			if ((current_robot_pose_v_and_phi.v < 0.15) &&
				((distance_to_red_traffic_light(current_robot_pose_v_and_phi, timestamp) < 2.0) ||
				 (distance_to_red_traffic_light(current_robot_pose_v_and_phi, timestamp) == 1000.0)))
				decision_making_state_msg->low_level_state = Stopped_At_Red_Traffic_Light_S0;
			else if (!red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Stopped_At_Red_Traffic_Light_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Red_Traffic_Light_S1;
			break;
		case Stopped_At_Red_Traffic_Light_S1:
			{
				static int steps2 = 0;

				if (steps2 > 3)
				{
					if (!red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp) || autonomous)
					{
						steps2 = 0;
						decision_making_state_msg->low_level_state = Stopped_At_Red_Traffic_Light_S2;
					}
				}
				else
					steps2++;
			}
			break;
		case Stopped_At_Red_Traffic_Light_S2:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5) && (distance_to_traffic_light_stop(current_robot_pose_v_and_phi) > 2.0))
				decision_making_state_msg->low_level_state = Free_Running;
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			break;


		case Stopping_At_Busy_Pedestrian_Track:
			if ((current_robot_pose_v_and_phi.v < 0.15) &&
				((distance_to_busy_pedestrian_track(current_robot_pose_v_and_phi, timestamp) < 2.0) ||
				 (distance_to_busy_pedestrian_track(current_robot_pose_v_and_phi, timestamp) == 1000.0)))
				decision_making_state_msg->low_level_state = Stopped_At_Busy_Pedestrian_Track_S0;
			else if (!busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Stopped_At_Busy_Pedestrian_Track_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Busy_Pedestrian_Track_S1;
			break;
		case Stopped_At_Busy_Pedestrian_Track_S1:
			{
				static int steps2 = 0;

				if (steps2 > 3)
				{
					if (!busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
					{
						steps2 = 0;
						decision_making_state_msg->low_level_state = Stopped_At_Busy_Pedestrian_Track_S2;
					}
				}
				else
					steps2++;
			}
			break;
		case Stopped_At_Busy_Pedestrian_Track_S2:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5) && (distance_to_pedestrian_track_stop(current_robot_pose_v_and_phi) > 2.0))
				decision_making_state_msg->low_level_state = Free_Running;
			if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopped_At_Busy_Pedestrian_Track_S0;
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			break;
		

		case Stopping_At_Busy_Queue:
			// printf("Stopping_At_Busy_Queue\n");
			// if ((current_robot_pose_v_and_phi.v < 0.15))// &&
				// ((distance_to_busy_pedestrian_track(current_robot_pose_v_and_phi, timestamp) < 2.0) ||
				//  (distance_to_busy_pedestrian_track(current_robot_pose_v_and_phi, timestamp) == 1000.0)))
				decision_making_state_msg->low_level_state = Stopped_At_Busy_Queue_S0;
			// else 
			if (!busy_queue_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Stopped_At_Busy_Queue_S0:
			// printf("Stopped_At_Busy_Queue_S0\n");
			decision_making_state_msg->low_level_state = Stopped_At_Busy_Queue_S1;
			break;
		case Stopped_At_Busy_Queue_S1:
			{
				// printf("Stopped_At_Busy_Queue_S1\n");
				static int steps2 = 0;

				if (steps2 > 3)
				{
					if (!busy_queue_ahead(current_robot_pose_v_and_phi, timestamp))
					{
						steps2 = 0;
						decision_making_state_msg->low_level_state = Stopped_At_Busy_Queue_S2;
					}
				}
				else
					steps2++;
			}
			break;
		case Stopped_At_Busy_Queue_S2:
			// printf("Stopped_At_Busy_Queue_S2\n");
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5))// && (distance_to_pedestrian_track_stop(current_robot_pose_v_and_phi) > 2.0))
				decision_making_state_msg->low_level_state = Free_Running;
			if (busy_queue_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopped_At_Busy_Queue_S0;
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			break;


		case Stopping_At_Yield:
			if ((current_robot_pose_v_and_phi.v < 0.15) &&
				((distance_to_yield(current_robot_pose_v_and_phi) < 2.0) ||
				 (distance_to_yield(current_robot_pose_v_and_phi) == 1000.0)))
				decision_making_state_msg->low_level_state = Stopped_At_Yield_S0;
			else if (!must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Stopped_At_Yield_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Yield_S1;
			break;
		case Stopped_At_Yield_S1:
			{
				static int steps2 = 0;

				if (steps2 > 3)
				{
					if (!must_yield(path_collision_info, timestamp))
					{
						steps2 = 0;
						decision_making_state_msg->low_level_state = Stopped_At_Yield_S2;
					}
				}
				else
					steps2++;
			}
			break;
		case Stopped_At_Yield_S2:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5) && (distance_to_yield(current_robot_pose_v_and_phi) > 2.0))
				decision_making_state_msg->low_level_state = Free_Running;
			if (must_yield(path_collision_info, timestamp))
				decision_making_state_msg->low_level_state = Stopped_At_Yield_S0;
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			break;


		case Stopping_At_Stop_Sign:
			if ((fabs(current_robot_pose_v_and_phi.v) < 0.5) && (distance_to_stop_sign(current_robot_pose_v_and_phi) < 4.0))
				decision_making_state_msg->low_level_state = Stopped_At_Stop_Sign_S0;
			if (!stop_sign_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Stopped_At_Stop_Sign_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Stop_Sign_S1;
			break;
		case Stopped_At_Stop_Sign_S1:
			{
				static bool wait_stopped = true;

				if (wait_stopped)
				{
					if (wait_for_given_seconds(5.0))
						wait_stopped = false;
				}
				else
				{
					if (!must_yield(path_collision_info, timestamp))
					{
						wait_stopped = true;
						decision_making_state_msg->low_level_state = Stopped_At_Stop_Sign_S2;
					}
				}
			}
			break;
		case Stopped_At_Stop_Sign_S2:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5) && (distance_to_stop_sign(current_robot_pose_v_and_phi) > 1.0))
				decision_making_state_msg->low_level_state = Free_Running;
			if (must_yield(path_collision_info, timestamp))
				decision_making_state_msg->low_level_state = Stopped_At_Stop_Sign_S0;
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			break;


		// Reverse handling
		case Stopping_To_Reverse:
			if ((fabs(current_robot_pose_v_and_phi.v) < 0.5) &&	(distance_to_reverse_waypoint(current_robot_pose_v_and_phi) < 2.0))
			{
				clear_wait_for_given_seconds();
				decision_making_state_msg->low_level_state = Stopped_At_Reverse_S0;
			}
			else if ((fabs(current_robot_pose_v_and_phi.v) < 0.02) && wait_for_given_seconds(60.0))
				decision_making_state_msg->low_level_state = Recovering_From_Error;
			break;
		case Stopped_At_Reverse_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Reverse_S1;
			break;
		case Stopped_At_Reverse_S1:
			{
				static bool wait_stopped = true;

				if (wait_stopped)
				{
					if (wait_for_given_seconds(4.0))
						wait_stopped = false;
				}
				else
				{
					if (autonomous && (fabs(current_robot_pose_v_and_phi.v) < 0.01))
					{
						wait_stopped = true;
						clear_wait_for_given_seconds();
						decision_making_state_msg->low_level_state = Stopped_At_Reverse_S2;
					}
					else if (!autonomous)
					{
						wait_stopped = true;
						clear_wait_for_given_seconds();
						decision_making_state_msg->low_level_state = Stopped;
					}
					else if (wait_for_given_seconds(3.0))
					{
						wait_stopped = true;
						decision_making_state_msg->low_level_state = Recovering_From_Error;
					}
				}
			}
			break;
		case Stopped_At_Reverse_S2:
			if (autonomous && ((current_robot_pose_v_and_phi.v < -0.5) || (distance_to_reverse_waypoint(current_robot_pose_v_and_phi) > 1.0)))
				decision_making_state_msg->low_level_state = Free_Reverse_Running;
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			break;


		// Reverse handling 2
		case Stopping_To_Go_Forward:
			if ((fabs(current_robot_pose_v_and_phi.v) < 0.5) && (distance_to_forward_waypoint(current_robot_pose_v_and_phi) < 2.0))
			{
				clear_wait_for_given_seconds();
				decision_making_state_msg->low_level_state = Stopped_At_Go_Forward_S0;
			}
			else if ((fabs(current_robot_pose_v_and_phi.v) < 0.02) && wait_for_given_seconds(60.0))
				decision_making_state_msg->low_level_state = Recovering_From_Error;
			break;
		case Stopped_At_Go_Forward_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Go_Forward_S1;
			break;
		case Stopped_At_Go_Forward_S1:
			{
				static bool wait_stopped = true;

				if (wait_stopped)
				{
					if (wait_for_given_seconds(4.0))
						wait_stopped = false;
				}
				else
				{
					if (autonomous && (fabs(current_robot_pose_v_and_phi.v) < 0.01))
					{
						wait_stopped = true;
						clear_wait_for_given_seconds();
						decision_making_state_msg->low_level_state = Stopped_At_Go_Forward_S2;
					}
					else if (!autonomous)
					{
						wait_stopped = true;
						clear_wait_for_given_seconds();
						decision_making_state_msg->low_level_state = Stopped;
					}
					else if (wait_for_given_seconds(3.0))
					{
						wait_stopped = true;
						decision_making_state_msg->low_level_state = Recovering_From_Error;
					}
				}
			}
			break;
		case Stopped_At_Go_Forward_S2:
			if (autonomous && ((current_robot_pose_v_and_phi.v > 0.5) || (distance_to_forward_waypoint(current_robot_pose_v_and_phi) > 1.0)))
				decision_making_state_msg->low_level_state = Free_Running;
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			break;


		// Pedestrian near RDDF handling
		case Stopping_To_Pedestrian:
			if ((current_robot_pose_v_and_phi.v < 0.15) &&
				(distance_to_waypoint_near_to_nearest_pedestrian_ahead(current_robot_pose_v_and_phi) < 2.0))
				decision_making_state_msg->low_level_state = Stopped_At_Pedestrian_S0;
			else if (!pedestrian_near_pose_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Stopped_At_Pedestrian_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Pedestrian_S1;
			break;
		case Stopped_At_Pedestrian_S1:
			{
				static int steps2 = 0;

				if (steps2 > 3)
				{
					if (!pedestrian_near_pose_ahead(current_robot_pose_v_and_phi, timestamp))
					{
						steps2 = 0;
						decision_making_state_msg->low_level_state = Stopped_At_Pedestrian_S2;
					}
				}
				else
					steps2++;
			}
			break;
		case Stopped_At_Pedestrian_S2:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5))
				decision_making_state_msg->low_level_state = Free_Running;
			if (pedestrian_near_pose_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopped_At_Pedestrian_S0;
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			break;


		case Recovering_From_Error:
			if (wait_for_given_seconds(4.0))
			{
				carmen_navigator_ackerman_go();
				decision_making_state_msg->low_level_state = Stopped;
			}
			else
				carmen_navigator_ackerman_stop();
			break;


		default:
			printf("Error: Unknown state in perform_state_transition()\n");
			return (2);
	}

	return (0);
}


int
run_decision_making_state_machine(carmen_behavior_selector_state_message *decision_making_state_msg,
		carmen_robot_and_trailers_traj_point_t current_robot_pose_v_and_phi, path_collision_info_t path_collision_info,
		carmen_robot_and_trailers_traj_point_t *last_valid_goal, double timestamp)
{
	int error;

	error = perform_state_transition(decision_making_state_msg, current_robot_pose_v_and_phi, path_collision_info, timestamp);
	if (error != 0)
		return (error);

	error = perform_state_action(decision_making_state_msg);
	if (error != 0)
		return (error);

	static int counter = 0;
	if (decision_making_state_msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_WITHIN_NARROW_PASSAGE)
		counter++;
	if ((counter == 0) || (counter > 40))
	{
		if (within_narrow_passage(current_robot_pose_v_and_phi, last_valid_goal, decision_making_state_msg))
			decision_making_state_msg->low_level_state_flags |= CARMEN_BEHAVIOR_SELECTOR_WITHIN_NARROW_PASSAGE;
		else
			decision_making_state_msg->low_level_state_flags &= ~CARMEN_BEHAVIOR_SELECTOR_WITHIN_NARROW_PASSAGE;
		counter = 0;
	}

	int narrow_lane_sign = narrow_lane_sign_ahead(current_robot_pose_v_and_phi);
	if (narrow_lane_sign == NARROW_LANE_BEGIN)
		carmen_task_manager_publish_set_collision_geometry_message(ENGAGE_GEOMETRY, timestamp);
	else if (narrow_lane_sign == NARROW_LANE_END)
		carmen_task_manager_publish_set_collision_geometry_message(DEFAULT_GEOMETRY, timestamp);

	int engine_brake_sign = engine_brake_sign_ahead(current_robot_pose_v_and_phi);
	if (engine_brake_sign == ENGINE_BRAKE_ON)
	{
		signals_msg.headlight_status = HEADLIGHT_ON;
		signals_msg.high_beams = 1;
		carmen_ford_escape_publish_signals_message(&signals_msg, carmen_get_time());
	}
	else if (engine_brake_sign == ENGINE_BRAKE_OFF)
	{
		signals_msg.headlight_status = HEADLIGHT_OFF;
		signals_msg.high_beams = 0;
		carmen_ford_escape_publish_signals_message(&signals_msg, carmen_get_time());
	}

	int turn_left_indicator_sign = turn_left_indicator_sign_ahead(current_robot_pose_v_and_phi);
	if (turn_left_indicator_sign == TURN_LEFT_INDICATOR_ON)
	{
		signals_msg.turn_signal = SIGNAL_LEFT;
		carmen_ford_escape_publish_signals_message(&signals_msg, carmen_get_time());
	}
	else if (turn_left_indicator_sign == TURN_LEFT_INDICATOR_OFF)
	{
		signals_msg.turn_signal = SIGNAL_OFF;
		carmen_ford_escape_publish_signals_message(&signals_msg, carmen_get_time());
	}

	int turn_right_indicator_sign = turn_right_indicator_sign_ahead(current_robot_pose_v_and_phi);
	if (turn_right_indicator_sign == TURN_RIGHT_INDICATOR_ON)
	{
		signals_msg.turn_signal = SIGNAL_RIGHT;
		carmen_ford_escape_publish_signals_message(&signals_msg, carmen_get_time());
	}
	else if (turn_right_indicator_sign == TURN_RIGHT_INDICATOR_OFF)
	{
		signals_msg.turn_signal = SIGNAL_OFF;
		carmen_ford_escape_publish_signals_message(&signals_msg, carmen_get_time());
	}

	int set_max_gear_sign = set_max_gear_sign_ahead(current_robot_pose_v_and_phi);
	if (set_max_gear_sign != NO_SET_MAX_GEAR_SIGN_AHEAD)
	{
		signals_msg.horn_status = 8 * set_max_gear_sign;  // usa a partir do bit 3. Assume-se que não será publicada ao mesmo tempo que a do task manager (tomada de força e basculamento)
		carmen_ford_escape_publish_signals_message(&signals_msg, carmen_get_time());
	}

	return (0);
}
