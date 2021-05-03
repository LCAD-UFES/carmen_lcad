#include <carmen/collision_detection.h>
#include <carmen/global_graphics.h>
#include <carmen/rddf_util.h>

#include "behavior_selector.h"


extern bool wait_start_moving;
extern bool autonomous;

extern carmen_rddf_annotation_message last_rddf_annotation_message;
extern carmen_robot_ackerman_config_t robot_config;

static double wait_for_given_seconds_start_time = 0.0;


bool
forward_waypoint_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_ackerman_traj_point_t *nearest_forward_waypoint_ahead = get_nearest_forward_waypoint_ahead();

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
distance_to_forward_waypoint(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_ackerman_traj_point_t *nearest_forward_waypoint_ahead = get_nearest_forward_waypoint_ahead();

	if (nearest_forward_waypoint_ahead == NULL)
		return (1000.0);

	double distance_to_forward_waypoint = DIST2D_P(nearest_forward_waypoint_ahead, &current_robot_pose_v_and_phi);

	return (distance_to_forward_waypoint);
}


bool
reverse_waypoint_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_ackerman_traj_point_t *nearest_reverse_waypoint_ahead = get_nearest_reverse_waypoint_ahead();

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
path_final_pose_reached(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_ackerman_traj_point_t *path_final_pose = get_path_final_pose();

	if (path_final_pose == NULL)
		return (false);

	double distance_to_path_final_pose = DIST2D_P(path_final_pose, &current_robot_pose_v_and_phi);

	if ((distance_to_path_final_pose < robot_config.distance_between_front_and_rear_axles) &&
		(fabs(current_robot_pose_v_and_phi.v) < 0.25) &&
		((distance_to_path_final_pose < 1.5) || nearest_pose_is_the_final_pose(current_robot_pose_v_and_phi)))
		return (true);
	else
		return (false);
}


double
distance_to_reverse_waypoint(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_ackerman_traj_point_t *nearest_reverse_waypoint_ahead = get_nearest_reverse_waypoint_ahead();

	if (nearest_reverse_waypoint_ahead == NULL)
		return (1000.0);

	double distance_to_reverse_waypoint = DIST2D_P(nearest_reverse_waypoint_ahead, &current_robot_pose_v_and_phi);

	return (distance_to_reverse_waypoint);
}


bool
stop_sign_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_stop_annotation = get_nearest_specified_annotation(RDDF_ANNOTATION_TYPE_STOP,
			last_rddf_annotation_message, &current_robot_pose_v_and_phi);

	if (nearest_stop_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_stop_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	if ((distance_to_act_on_annotation >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_stop_annotation->annotation_point))
		return (true);
	else
		return (false);
}


double
distance_to_stop_sign(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
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
distance_to_red_traffic_light(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
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
distance_to_traffic_light_stop(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
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
distance_to_busy_pedestrian_track(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
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
distance_to_must_yield(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi,
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
distance_to_yield(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
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
distance_to_pedestrian_track_stop(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
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


		case Stopping_To_Go_Forward:
			break;
		case Stopped_At_Go_Forward_S0:
			break;
		case Stopped_At_Go_Forward_S1:
			break;
		case Stopped_At_Go_Forward_S2:
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
robot_reached_non_return_point(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_pedestrian_track_annotation = get_nearest_specified_annotation(RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK,
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


int
perform_state_transition(carmen_behavior_selector_state_message *decision_making_state_msg,
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi,
		path_collision_info_t path_collision_info, double timestamp)
{
	switch (decision_making_state_msg->low_level_state)
	{
		case Initializing:
			decision_making_state_msg->low_level_state = Stopped;
			break;
		case Stopped:
			if (autonomous)
			{
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
			else if (must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Yield;
			else if (stop_sign_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Stop_Sign;
			else if (reverse_waypoint_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_To_Reverse;
			else if (path_final_pose_reached(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = End_Of_Path_Reached;
			break;


		case Free_Reverse_Running:
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			else if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Red_Traffic_Light;
			else if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp) && !robot_reached_non_return_point(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Busy_Pedestrian_Track;
			else if (must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Yield;
			else if (stop_sign_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Stop_Sign;
			else if (forward_waypoint_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_To_Go_Forward;
			else if (path_final_pose_reached(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = End_Of_Path_Reached;
			break;


		case End_Of_Path_Reached:
			if (wait_for_given_seconds(1.0))
				decision_making_state_msg->low_level_state = End_Of_Path_Reached2;
			break;


		case End_Of_Path_Reached2:
			if (wait_for_given_seconds(1.0))
				decision_making_state_msg->low_level_state = Stopped;
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
					if (!busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp) || autonomous)
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
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, path_collision_info_t path_collision_info, double timestamp)
{
	int error;

	error = perform_state_transition(decision_making_state_msg, current_robot_pose_v_and_phi, path_collision_info, timestamp);
	if (error != 0)
		return (error);

	error = perform_state_action(decision_making_state_msg);
	if (error != 0)
		return (error);

	return (0);
}
