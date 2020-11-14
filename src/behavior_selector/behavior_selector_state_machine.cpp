#include <carmen/collision_detection.h>
#include <carmen/global_graphics.h>
#include <carmen/rddf_util.h>

#include "behavior_selector.h"


extern bool wait_start_moving;
extern bool autonomous;

extern carmen_rddf_annotation_message last_rddf_annotation_message;


bool
stop_sign_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	if ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
		(distance_to_act_on_annotation >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_velocity_related_annotation->annotation_point))
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


void
clear_state_output(carmen_behavior_selector_state_message *decision_making_state_msg)
{
    decision_making_state_msg->behaviour_seletor_mode = none;
}


int
perform_state_action(carmen_behavior_selector_state_message *decision_making_state_msg, carmen_ackerman_traj_point_t *goal __attribute__ ((unused)),
		double timestamp __attribute__ ((unused)))
{
	switch (decision_making_state_msg->low_level_state)
	{
		case Initializing:
			break;
		case Stopped:
			break;
		case Free_Running:
			break;


		case Following_Moving_Object:
			break;

		case Stopping_At_Red_Traffic_Light:
			break;
		case Stopped_At_Red_Traffic_Light_S0:
			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Red_Traffic_Light_S1:
			break;
		case Stopped_At_Red_Traffic_Light_S2:
			carmen_navigator_ackerman_go();
			break;


		case Stopping_At_Busy_Pedestrian_Track:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S0:
			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Busy_Pedestrian_Track_S1:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S2:
			carmen_navigator_ackerman_go();
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
			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Stop_Sign_S1:
			break;
		default:
			printf("Error: Unknown state in perform_state_action()\n");
			return (1);
	}

	return (0);
}


int
perform_state_transition(carmen_behavior_selector_state_message *decision_making_state_msg,
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, int goal_type __attribute__ ((unused)),
		path_collision_info_t path_collision_info, double timestamp)
{
	switch (decision_making_state_msg->low_level_state)
	{
		case Initializing:
			decision_making_state_msg->low_level_state = Stopped;
			break;
		case Stopped:
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			else
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Free_Running:
			if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Red_Traffic_Light;
			else if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Busy_Pedestrian_Track;
			else if (must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Yield;
			else if (stop_sign_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Stop_Sign;
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
			break;


		case Stopping_At_Yield:
			if ((current_robot_pose_v_and_phi.v < 0.15) &&
				((distance_to_yield(current_robot_pose_v_and_phi) < 2.0) ||
				 (distance_to_yield(current_robot_pose_v_and_phi) == 1000.0)))
				decision_making_state_msg->low_level_state = Stopped_At_Yield_S0;
			break;
		case Stopped_At_Yield_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Yield_S1;
			break;
		case Stopped_At_Yield_S1:
			{
				static int steps2 = 0;

				if (steps2 > 3)
				{
					if (!must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
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
			if (must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopped_At_Yield_S0;
			break;


		case Stopping_At_Stop_Sign:
			if ((fabs(current_robot_pose_v_and_phi.v) < 0.01) && (distance_to_stop_sign(current_robot_pose_v_and_phi) < 4.0))
				decision_making_state_msg->low_level_state = Stopped_At_Stop_Sign_S0;
			break;
		case Stopped_At_Stop_Sign_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Stop_Sign_S1;
			break;
		case Stopped_At_Stop_Sign_S1:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		default:
			printf("Error: Unknown state in perform_state_transition()\n");
			return (2);
	}
	return (0);
}


int
run_decision_making_state_machine(carmen_behavior_selector_state_message *decision_making_state_msg,
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, carmen_ackerman_traj_point_t *goal, int goal_type,
		path_collision_info_t path_collision_info, double timestamp)
{
	int error;

	error = perform_state_transition(decision_making_state_msg, current_robot_pose_v_and_phi, goal_type, path_collision_info, timestamp);
	if (error != 0)
		return (error);

	error = perform_state_action(decision_making_state_msg, goal, timestamp);
	if (error != 0)
		return (error);

	clear_state_output(decision_making_state_msg);

	return (0);
}
