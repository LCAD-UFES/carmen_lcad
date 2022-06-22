#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/rddf_messages.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_util.h>
#include <carmen/udatmo.h>
#include <carmen/rddf_interface.h>
#include <carmen/task_manager_interface.h>
#include "behavior_selector.h"

extern bool wait_start_moving;

extern carmen_rddf_annotation_message last_rddf_annotation_message;
extern carmen_rddf_road_profile_message *last_rddf_message;
extern bool last_rddf_annotation_message_valid;

extern carmen_robot_ackerman_config_t robot_config;

extern bool autonomous;

extern double last_speed_limit;

extern double robot_max_centripetal_acceleration;
extern double distance_to_moving_object_with_v_multiplier;
extern bool keep_speed_limit;
extern int behavior_selector_reverse_driving;
extern carmen_route_planner_road_network_message *road_network_message;
extern double parking_speed_limit;
extern double move_to_engage_pose_speed_limit;

extern double annotation_velocity_bump;
extern double annotation_velocity_pedestrian_track_stop;
extern double annotation_velocity_yield;
extern double annotation_velocity_barrier;

extern carmen_moving_objects_point_clouds_message *pedestrians_tracked;
extern int behavior_selector_check_pedestrian_near_path;
extern double behavior_pedestrian_near_path_min_lateral_distance;
extern double behavior_selector_pedestrian_near_path_min_longitudinal_distance;


carmen_robot_and_trailer_traj_point_t
displace_pose(carmen_robot_and_trailer_traj_point_t robot_pose, double displacement)
{
	carmen_robot_and_trailer_pose_t displaced_robot_position = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&robot_pose, displacement);

	carmen_robot_and_trailer_traj_point_t displaced_robot_pose = robot_pose;
	displaced_robot_pose.x = displaced_robot_position.x;
	displaced_robot_pose.y = displaced_robot_position.y;

	return (displaced_robot_pose);
}


bool
going_forward()
{
	if (last_rddf_message->number_of_poses > 0)
	{
		if (last_rddf_message->poses[last_rddf_message->number_of_poses / 2].v >= 0.0)
			return (true);
		else
			return (false);
	}
	else
		return (true);
}


carmen_robot_and_trailer_traj_point_t *
get_nearest_forward_waypoint_ahead()
{
	// Ja esta em trecho com velocidade positiva
	if (last_rddf_message->poses[0].v >= 0.0)
		return (NULL);

	// Encontra a primeira velocidade positiva a frente no tempo
	int i;
	for (i = 0; (i < last_rddf_message->number_of_poses) && (last_rddf_message->poses[i].v < 0.0); i++)
		;

	if (i < last_rddf_message->number_of_poses)
		return (&(last_rddf_message->poses[i]));
	else
		return (NULL);
}


carmen_robot_and_trailer_traj_point_t *
get_nearest_reverse_waypoint_ahead()
{
	// Ja esta em trecho com velocidade negativa
	if (last_rddf_message->poses[0].v < 0.0)
		return (NULL);

	// Encontra a primeira velocidade negativa a frente no tempo
	int i;
	for (i = 0; (i < last_rddf_message->number_of_poses) && (last_rddf_message->poses[i].v >= 0.0); i++)
		;

	if (i < last_rddf_message->number_of_poses)
		return (&(last_rddf_message->poses[i]));
	else
		return (NULL);
}


carmen_robot_and_trailer_traj_point_t *
get_waypoint_near_to_nearest_pedestrian_ahead()
{
	if (!pedestrians_tracked || !behavior_selector_check_pedestrian_near_path)
		return (NULL);

	double distance_to_nearest_pedestrian_ahead = 100000.0;
	int index_of_waypoint = -1;
	for (int i = 0; i < pedestrians_tracked->num_point_clouds; i++)
	{
		if (strcmp(pedestrians_tracked->point_clouds[i].model_features.model_name, "pedestrian") == 0)
		{
			for (int j = 0; j < last_rddf_message->number_of_poses; j++)
			{
				double distance = DIST2D(pedestrians_tracked->point_clouds[i].object_pose, last_rddf_message->poses[j]);
				if ((distance < behavior_pedestrian_near_path_min_lateral_distance) && (distance < distance_to_nearest_pedestrian_ahead))
				{
					distance_to_nearest_pedestrian_ahead = distance;
					index_of_waypoint = j;
				}
			}
		}
	}

	if (index_of_waypoint != -1)
	{
		double safe_distance = robot_config.distance_between_front_and_rear_axles +
							   robot_config.distance_between_front_car_and_front_wheels +
							   behavior_selector_pedestrian_near_path_min_longitudinal_distance;
		double distance = 0.0;
		while ((distance < safe_distance) && (index_of_waypoint > 0))
		{
			distance += DIST2D(last_rddf_message->poses[index_of_waypoint], last_rddf_message->poses[index_of_waypoint - 1]);
			index_of_waypoint--;
		}

		return (&(last_rddf_message->poses[index_of_waypoint]));
	}
	else
		return (NULL);
}


int
get_index_of_waypoint_near_to_nearest_pedestrian_ahead()
{
	if (!pedestrians_tracked)
		return (-1);

	double distance_to_nearest_pedestrian_ahead = 100000.0;
	int index_of_waypoint = -1;
	for (int i = 0; i < pedestrians_tracked->num_point_clouds; i++)
	{
		if (strcmp(pedestrians_tracked->point_clouds[i].model_features.model_name, "pedestrian") == 0)
		{
			for (int j = 0; j < last_rddf_message->number_of_poses; j++)
			{
				double distance = DIST2D(pedestrians_tracked->point_clouds[i].object_pose, last_rddf_message->poses[j]);
				if ((distance < behavior_pedestrian_near_path_min_lateral_distance) && (distance < distance_to_nearest_pedestrian_ahead))
				{
					distance_to_nearest_pedestrian_ahead = distance;
					index_of_waypoint = j;
				}
			}
		}
	}

	if (index_of_waypoint != -1)
	{
		double safe_distance = robot_config.distance_between_front_and_rear_axles +
							   robot_config.distance_between_front_car_and_front_wheels +
							   behavior_selector_pedestrian_near_path_min_longitudinal_distance;
		double distance = 0.0;
		while ((distance < safe_distance) && (index_of_waypoint > 0))
		{
			distance += DIST2D(last_rddf_message->poses[index_of_waypoint], last_rddf_message->poses[index_of_waypoint - 1]);
			index_of_waypoint--;
		}

		return (index_of_waypoint);
	}
	else
		return (-1);
}


carmen_robot_and_trailer_traj_point_t *
get_path_final_pose()
{
	if (!last_rddf_message)
		return (NULL);

	return (&(last_rddf_message->poses[last_rddf_message->number_of_poses - 1]));
}


bool
nearest_pose_is_the_final_pose(carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi)
{
	double nearest_distance = DIST2D(current_robot_pose_v_and_phi, last_rddf_message->poses[0]);
	int nearest_index = 0;
	for (int i = 1; i < last_rddf_message->number_of_poses; i++)
	{
		double distance = DIST2D(current_robot_pose_v_and_phi, last_rddf_message->poses[i]);
		if (distance < nearest_distance)
		{
			nearest_distance = distance;
			nearest_index = i;
		}
	}
	if (nearest_index == (last_rddf_message->number_of_poses - 1))
		return (true);
	else
		return (false);
}


carmen_annotation_t *
get_nearest_specified_annotation_in_front(int annotation, carmen_rddf_annotation_message annotation_message, carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi)
{
	int nearest_annotation_index = -1;
	double distance_to_nearest_annotation = 1000.0;

	for (int i = 0; i < annotation_message.num_annotations; i++)
	{
		double distance_to_annotation = DIST2D_P(&annotation_message.annotations[i].annotation_point, current_robot_pose_v_and_phi);

		if ((annotation_message.annotations[i].annotation_type == annotation) &&
			(distance_to_annotation < distance_to_nearest_annotation) &&
			carmen_rddf_play_annotation_is_forward(*current_robot_pose_v_and_phi, annotation_message.annotations[i].annotation_point))
		{
			distance_to_nearest_annotation = distance_to_annotation;
			nearest_annotation_index = i;
		}
	}

	if (nearest_annotation_index != -1)
		return (&(annotation_message.annotations[nearest_annotation_index]));
	else
		return (NULL);
}


carmen_annotation_t *
get_nearest_velocity_related_annotation(carmen_rddf_annotation_message annotation_message,
		carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi, bool wait_start_moving)
{
	int nearest_annotation_index = -1;
	double distance_to_nearest_annotation = 1000.0;

	for (int i = 0; i < annotation_message.num_annotations; i++)
	{
		double distance_to_annotation = DIST2D_P(&annotation_message.annotations[i].annotation_point, current_robot_pose_v_and_phi);
		if (((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BARRIER) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_STOP) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_YIELD) ||
			 ((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_DYNAMIC) &&
			  (annotation_message.annotations[i].annotation_code == RDDF_ANNOTATION_CODE_DYNAMIC_STOP) && !wait_start_moving) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BUMP)) &&
			carmen_rddf_play_annotation_is_forward(*current_robot_pose_v_and_phi, annotation_message.annotations[i].annotation_point) &&
			 (distance_to_annotation < distance_to_nearest_annotation))
		{
			distance_to_nearest_annotation = distance_to_annotation;
			nearest_annotation_index = i;
		}
	}

	if (nearest_annotation_index != -1)
		return (&(annotation_message.annotations[nearest_annotation_index]));
	else
		return (NULL);
}


bool
busy_pedestrian_track_ahead(carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
//	return (false);

	static double last_pedestrian_track_busy_timestamp = 0.0;

//	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
//				&current_robot_pose_v_and_phi, false);
//
//	if (nearest_velocity_related_annotation == NULL)
//		return (false);
//
//	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
//	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_robot_and_trailer_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	carmen_annotation_t *nearest_pedestrian_track_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK,
			last_rddf_annotation_message, &displaced_robot_pose);

	if (nearest_pedestrian_track_annotation == NULL)
		return (false);

	if ((nearest_pedestrian_track_annotation->annotation_code == RDDF_ANNOTATION_CODE_PEDESTRIAN_TRACK_BUSY))// &&
//		(nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) &&
//		(distance_to_act_on_annotation >= distance_to_annotation) &&
//		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_velocity_related_annotation->annotation_point))
		last_pedestrian_track_busy_timestamp = timestamp;

	if (timestamp - last_pedestrian_track_busy_timestamp < 1.5)
		return (true);
	else
		return (false);
}


bool
must_yield(path_collision_info_t path_collision_info, double timestamp)
{
	static double last_must_yield_timestamp = 0.0;

	if (path_collision_info.valid && path_collision_info.mo_in_front)
		last_must_yield_timestamp = timestamp;

	if (timestamp - last_must_yield_timestamp < 0.5)
		return (true);
	else
		return (false);
}


bool
must_yield_ahead(path_collision_info_t path_collision_info, carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi,
		double timestamp)
{
	static double last_must_yield_timestamp = 0.0;

	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_robot_and_trailer_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	if ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_YIELD) &&
		(distance_to_act_on_annotation >= distance_to_annotation) &&
		(path_collision_info.valid && path_collision_info.mo_in_front) &&
		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_velocity_related_annotation->annotation_point))
		last_must_yield_timestamp = timestamp;

	if (timestamp - last_must_yield_timestamp < 0.5)
		return (true);
	else
		return (false);
}


carmen_robot_and_trailer_traj_point_t *
get_final_goal()
{
	carmen_robot_and_trailer_traj_point_t *final_goal = NULL;

	int goal_list_size;
	int *goal_type;
	carmen_robot_and_trailer_traj_point_t *goal_list = behavior_selector_get_last_goals_and_types(goal_type, goal_list_size);
	for (int i = 0; i < goal_list_size; i++)
	{
		if (goal_type[i] == FINAL_GOAL)
		{
			final_goal = &(goal_list[i]);
			break;
		}
	}

	return (final_goal);
}


double
get_distance_to_act_on_annotation(double v0, double va, double distance_to_annotation)
{
	// @@@ Alberto: rever pois esta fazendo a IARA lesmar quando esta lenta e longe de uma anotacao
	// va = v0 + a * t
	// a*t = v - v0
	// t = (va - v0) / a
	// da = va * t + 0.5 * a * t * t

	double a;
	if (v0 > 0.0)
		a = -get_robot_config()->desired_decelaration_forward; // Desaceleracao para anotacoes
	else
		a = -get_robot_config()->desired_decelaration_reverse; // Desaceleracao para reh

	v0 = fabs(v0); //a distancia para reagir a anotacao independe do sinal, sinal soh indica orientacao do movimento.

	double t = (va - v0) / a;
	double t_deceleration = ((t - 0.5) > 0.0)? (t - 0.5): 0.0;
	double daa = v0 * t + 0.5 * a * t_deceleration * t_deceleration;

	if (daa > distance_to_annotation)
		return (daa);// + 3.0 * get_robot_config()->distance_between_front_and_rear_axles);
	else
		return (distance_to_annotation);
}


bool
red_traffic_light_ahead(carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	static double last_red_timestamp = 0.0;

	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, false);

	if (nearest_velocity_related_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_robot_and_trailer_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	carmen_annotation_t *nearest_traffic_light_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT,
			last_rddf_annotation_message, &displaced_robot_pose);

	if (nearest_traffic_light_annotation == NULL)
		return (false);

	if (nearest_traffic_light_annotation->annotation_code == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN)
		return (false);
	else if ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) &&
		(distance_to_act_on_annotation >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_velocity_related_annotation->annotation_point))
		last_red_timestamp = timestamp;

	if (timestamp - last_red_timestamp < 3.0)
		return (true);

	return (false);
}


double
get_velocity_at_next_annotation(carmen_annotation_t *annotation, carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi,
		path_collision_info_t path_collision_info, carmen_behavior_selector_state_message behavior_selector_state_message,
		double timestamp)
{
	double v = get_max_v();

	if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) &&
		red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
		v = 0.0;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) &&
			 busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
		v = 0.0;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) &&
			 busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp) &&
			 (DIST2D(current_robot_pose_v_and_phi, annotation->annotation_point) > (1.5 + robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels)))
		v = 0.0;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP)
		v = annotation_velocity_pedestrian_track_stop;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)
		v = annotation_velocity_pedestrian_track_stop;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_YIELD) &&
			 must_yield_ahead(path_collision_info, current_robot_pose_v_and_phi, timestamp))
		v = 0.0;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
			 (behavior_selector_state_message.low_level_state != Stopped_At_Stop_Sign_S2))
		v = 0.0;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_DYNAMIC) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_DYNAMIC_STOP))
		v = 0.0;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_BUMP)
		v = annotation_velocity_bump;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_YIELD)
		v = annotation_velocity_yield;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_BARRIER)
		v = annotation_velocity_barrier;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_0))
		v = 0.0;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_5))
		v = 5.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_10))
		v = 10.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_15))
		v = 15.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_20))
		v = 20.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_30))
		v = 30.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_40))
		v = 40.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_60))
		v = 60.0 / 3.6;

	return (v);
}


double
distance_to_moving_obstacle_annotation(carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_DYNAMIC) &&
		(nearest_velocity_related_annotation->annotation_code == RDDF_ANNOTATION_CODE_DYNAMIC_STOP))
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
get_velocity_at_goal(double v0, double va, double dg, double da)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dg*(g-v)%2Fa%2B(v-g)*((g-v)%2F(2*a)))+for+a
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*((g-v)%2Fa)%2B0.5*a*((g-v)%2Fa)%5E2+for+g
	// http://www.physicsclassroom.com/class/1DKin/Lesson-6/Kinematic-Equations

//	double a = -get_robot_config()->maximum_acceleration_forward * 2.5;
	double a = (va * va - v0 * v0) / (2.0 * da);
	// TODO: @@@ Alberto: nao deveria ser 2.0 ao inves de 1.0 abaixo? Com 2.0 freia esponencialmente nos quebra molas...
	double sqrt_val = get_robot_config()->behaviour_selector_goal_velocity_tuning_factor * a * dg + v0 * v0;
	double vg = va;
	if (sqrt_val > 0.0)
		vg = sqrt(sqrt_val);
	if (vg < va)
		vg = va;

//	static double first_time = 0.0;
//	double t = carmen_get_time();
//	if (first_time == 0.0)
//		first_time = t;
	//printf("t %.3lf, v0 %.1lf, va %.1lf, a %.3lf, vg %.2lf, dg %.1lf, da %.1lf\n", t - first_time, v0, va, a, vg, dg, da);
//	printf("t %.3lf, v0 %.1lf, a %.3lf, vg %.2lf, dg %.1lf, tt %.3lf\n", t - first_time, v0, a, vg, dg, (vg - v0) / a);

	return (vg);
}


double
compute_distance_within_rddf(carmen_vector_3D_t annotation_point, carmen_robot_and_trailer_traj_point_t  current_robot_pose_v_and_phi)
{
	carmen_rddf_road_profile_message *rddf = last_rddf_message;

	double distance_to_annotation = 1000.0;
	double distance_to_car = 1000.0;
	int index_car_pose = 0;
	int index_annotation = 0;
	for (int i = 0; i < rddf->number_of_poses; i++)
	{
		double distance = DIST2D(annotation_point, rddf->poses[i]);
		if (distance < distance_to_annotation)
		{
			distance_to_annotation = distance;
			index_annotation = i;
		}
		distance = DIST2D(current_robot_pose_v_and_phi, rddf->poses[i]);
		if (distance < distance_to_car)
		{
			distance_to_car = distance;
			index_car_pose = i;
		}
	}

	double distance_within_rddf = 0.0;
	for (int i = index_car_pose; i < index_annotation; i++)
		distance_within_rddf += DIST2D(rddf->poses[i], rddf->poses[i + 1]);

	double min_distance = DIST2D(annotation_point, current_robot_pose_v_and_phi);

	return ((distance_within_rddf < min_distance)? min_distance: distance_within_rddf);
}


double
effective_distance_to_annotation(carmen_annotation_t *annotation, carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi)
{
	double distance = DIST2D(annotation->annotation_point, *current_robot_pose_v_and_phi);

	if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_BARRIER)
	{
		double size_front;
		double size_back;

		carmen_rddf_get_barrier_alignment_segments_sizes(annotation, &size_front, &size_back);

		carmen_vector_2D_t effective_annotation_point;
		effective_annotation_point.x = annotation->annotation_point.x + size_back * cos(carmen_normalize_theta(annotation->annotation_orientation + M_PI));
		effective_annotation_point.y = annotation->annotation_point.y + size_back * sin(carmen_normalize_theta(annotation->annotation_orientation + M_PI));
		double distance2 = DIST2D(effective_annotation_point, *current_robot_pose_v_and_phi);
		if (distance2 < distance)
			distance = distance2;

		effective_annotation_point.x = annotation->annotation_point.x + size_back * cos(carmen_normalize_theta(annotation->annotation_orientation));
		effective_annotation_point.y = annotation->annotation_point.y + size_back * sin(carmen_normalize_theta(annotation->annotation_orientation));
		distance2 = DIST2D(annotation->annotation_point, *current_robot_pose_v_and_phi);
		if (distance2 < distance)
			distance = distance2;
	}

	return (distance);
}


double
set_goal_velocity_according_to_annotation(carmen_robot_and_trailer_traj_point_t *goal, int goal_type,
		carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi, path_collision_info_t path_collision_info,
		carmen_behavior_selector_state_message behavior_selector_state_message, double timestamp)
{
	static bool clearing_annotation = false;
	static carmen_vector_3D_t previous_annotation_point = {0.0, 0.0, 0.0};

	if (!autonomous)
		clearing_annotation = false;

	carmen_annotation_t *nearest_velocity_related_annotation;
	//TODO Soh pega as anotacoes para frente. Precisamos tratar essas mesmas anotacao para tras?
	if (goal_type == ANNOTATION_GOAL_STOP)
		nearest_velocity_related_annotation = get_nearest_specified_annotation_in_front(RDDF_ANNOTATION_TYPE_STOP, last_rddf_annotation_message,
			current_robot_pose_v_and_phi);
	else
		nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
			current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation != NULL)
	{
//		carmen_robot_and_trailer_traj_point_t displaced_robot_pose = displace_pose(*current_robot_pose_v_and_phi,
//				get_robot_config()->distance_between_front_and_rear_axles +
//				get_robot_config()->distance_between_front_car_and_front_wheels);

		double distance_to_annotation = effective_distance_to_annotation(nearest_velocity_related_annotation, current_robot_pose_v_and_phi);
//		double distance_to_annotation = compute_distance_within_rddf(nearest_velocity_related_annotation->annotation_point, *current_robot_pose_v_and_phi);
//		FILE *caco13 = fopen("caco13.txt", "a");
//		fprintf(caco13, "%.2lf %.2lf\n", distance_to_annotation, DIST2D(nearest_velocity_related_annotation->annotation_point, *current_robot_pose_v_and_phi));
//		fflush(caco13);
//		fclose(caco13);

		double velocity_at_next_annotation = get_velocity_at_next_annotation(nearest_velocity_related_annotation, *current_robot_pose_v_and_phi, path_collision_info,
				behavior_selector_state_message, timestamp);

		double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi->v, velocity_at_next_annotation,
				distance_to_annotation);

		//TODO Depende da direcao do movimento do carro.
		bool annotation_ahead = carmen_rddf_play_annotation_is_forward(*current_robot_pose_v_and_phi, nearest_velocity_related_annotation->annotation_point);

		double distance_to_goal = carmen_distance_ackerman_traj(current_robot_pose_v_and_phi, goal);

		if (last_rddf_annotation_message_valid &&
			(clearing_annotation ||
			 (((distance_to_annotation < distance_to_act_on_annotation) ||
			   (distance_to_annotation < (distance_to_goal + robot_config.distance_between_front_and_rear_axles / 4.0))) && annotation_ahead) ||
			   ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
				((goal_type == ANNOTATION_GOAL2) || (goal_type == ANNOTATION_GOAL_STOP)))))
		{
			if (!clearing_annotation)
				previous_annotation_point = nearest_velocity_related_annotation->annotation_point;

			clearing_annotation = true;
			//TODO tem que Certificar de que ou ta tudo negativo ou positivo (acho que deveria tratar tudo positivo)
			goal->v = carmen_fmin(
					get_velocity_at_goal(current_robot_pose_v_and_phi->v, velocity_at_next_annotation, distance_to_goal, distance_to_annotation),
					goal->v);

			if (((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
					((goal_type == ANNOTATION_GOAL2) || (goal_type == ANNOTATION_GOAL_STOP))))
				goal->v = get_velocity_at_next_annotation(nearest_velocity_related_annotation, *current_robot_pose_v_and_phi,
						path_collision_info, behavior_selector_state_message, timestamp);
		}

		if (!annotation_ahead || (DIST2D(previous_annotation_point, nearest_velocity_related_annotation->annotation_point) > 0.0))
			clearing_annotation = false;

		if (annotation_ahead && (nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
				(distance_to_annotation < 10.0))
			last_speed_limit = velocity_at_next_annotation;

//		FILE *caco = fopen("caco4.txt", "a");
//		fprintf(caco, "ca %d, aa %d, daann %.1lf, dann %.1lf, v %.1lf, vg %.1lf, aif %d, dg %.1lf, av %.1lf, ts %lf\n", clearing_annotation, annotation_ahead,
//				distance_to_act_on_annotation, distance_to_annotation, current_robot_pose_v_and_phi->v,
//				goal->v,
//				carmen_rddf_play_annotation_is_forward(get_robot_pose(), nearest_velocity_related_annotation->annotation_point),
//				distance_to_goal, velocity_at_next_annotation, carmen_get_time());
//		fflush(caco);
//		fclose(caco);
	}

	return (goal->v);
}


double
set_goal_velocity_according_to_final_goal(carmen_robot_and_trailer_traj_point_t *goal, int goal_type,
		carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi)
{
	static bool clearing_final_goal = false;
	static carmen_robot_and_trailer_traj_point_t previous_final_goal_point = {};

	if (!autonomous)
		clearing_final_goal = false;

	carmen_robot_and_trailer_traj_point_t *final_goal = get_final_goal();

	if (final_goal != NULL)
	{
		double distance_to_final_goal = DIST2D_P(final_goal, current_robot_pose_v_and_phi);
		double velocity_at_final_goal = 0.0;
		double distance_to_act_on_final_goal = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi->v, velocity_at_final_goal,
				distance_to_final_goal);
		double distance_to_goal = carmen_distance_ackerman_traj(current_robot_pose_v_and_phi, goal);

		if (clearing_final_goal ||
			(distance_to_final_goal < distance_to_act_on_final_goal) ||
			(distance_to_final_goal < distance_to_goal))
		{
			if (!clearing_final_goal)
				previous_final_goal_point = *final_goal;

			clearing_final_goal = true;
			//TODO tem que Certificar de que ou ta tudo negativo ou positivo (acho que deveria tratar tudo positivo)
			goal->v = carmen_fmin(
					get_velocity_at_goal(current_robot_pose_v_and_phi->v, velocity_at_final_goal, distance_to_goal, distance_to_final_goal),
					goal->v);

			if (goal_type == FINAL_GOAL)
				goal->v = 0.0;
		}

		if (DIST2D(previous_final_goal_point, *final_goal) > 0.0)
			clearing_final_goal = false;
	}

	return (goal->v);
}


double
set_goal_velocity_according_to_state_machine(carmen_robot_and_trailer_traj_point_t *goal,
		carmen_behavior_selector_state_message behavior_selector_state_message)
{
	if (
		(behavior_selector_state_message.low_level_state == Stopping_At_Yield) ||
		(behavior_selector_state_message.low_level_state == Stopped_At_Yield_S0) ||
		(behavior_selector_state_message.low_level_state == Stopped_At_Yield_S1) ||

		(behavior_selector_state_message.low_level_state == Stopped_At_Reverse_S0) ||
		(behavior_selector_state_message.low_level_state == Stopped_At_Reverse_S1) ||

		(behavior_selector_state_message.low_level_state == Stopping_At_Busy_Pedestrian_Track) ||
		(behavior_selector_state_message.low_level_state == Stopped_At_Busy_Pedestrian_Track_S0) ||
		(behavior_selector_state_message.low_level_state == Stopped_At_Busy_Pedestrian_Track_S1) ||

		(behavior_selector_state_message.low_level_state == Stopping_To_Pedestrian) ||
		(behavior_selector_state_message.low_level_state == Stopped_At_Pedestrian_S0) ||
		(behavior_selector_state_message.low_level_state == Stopped_At_Pedestrian_S1) ||

		(behavior_selector_state_message.low_level_state == Stopped_At_Go_Forward_S0) ||
		(behavior_selector_state_message.low_level_state == Stopped_At_Go_Forward_S1)
	   )
		return (0.0);
	else
		return (goal->v);
}


double
set_goal_velocity_according_to_obstacle_distance(carmen_robot_and_trailer_traj_point_t *goal, carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi)
{
	double distance_to_obstacle = DIST2D_P(current_robot_pose_v_and_phi, goal);

	double tmp = carmen_fmin(
				get_velocity_at_goal(current_robot_pose_v_and_phi->v, 0.0, distance_to_obstacle, distance_to_obstacle),
				fabs(goal->v));
	if (goal->v < 0.0)
		goal->v = -tmp;
	else
		goal->v = tmp;

	return (goal->v);
}


double
limit_maximum_velocity_according_to_centripetal_acceleration(double target_v, double current_v, carmen_robot_and_trailer_traj_point_t *goal,
		carmen_robot_and_trailer_traj_point_t *poses_ahead, int number_of_poses)
{
	if (number_of_poses < 6)
		return (target_v);

	carmen_robot_and_trailer_traj_point_t *path = (carmen_robot_and_trailer_traj_point_t *) malloc(number_of_poses * sizeof(carmen_robot_and_trailer_traj_point_t));
	memcpy(path, poses_ahead, number_of_poses * sizeof(carmen_robot_and_trailer_traj_point_t));

	carmen_robot_and_trailer_traj_point_t *original_path_copy = path;
	path = &(path[1]);
	number_of_poses -= 1;
	for (int i = 0; i < ((number_of_poses / 4) - 1); i++)
		path[i] = path[i * 4];
	number_of_poses /= 4;
//	plot_state(path, number_of_poses, NULL, 0, true);

	double L = get_robot_config()->distance_between_front_and_rear_axles;
	for (int i = 0; i < (number_of_poses - 1); i++)
	{
		double delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i + 1]);
		path[i].phi = atan(L * (delta_theta / l));
	}

	double dist_walked = 0.0;
	double max_centripetal_acceleration = 0.0;
	double dist_to_max_curvature = 0.0;
	double max_path_phi = 0.0;
	for (int i = 1; i < (number_of_poses - 1); i++)
	{
		double l = DIST2D(path[i], path[i - 1]);
		dist_walked += l;
		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
		if (fabs(path[i].phi) > 0.001)
		{
			double radius_of_curvature = L / fabs(tan(path[i].phi));
			double centripetal_acceleration = (target_v * target_v) / radius_of_curvature;
			if (centripetal_acceleration > max_centripetal_acceleration)
			{
				dist_to_max_curvature = dist_walked;
				max_path_phi = path[i].phi;
				max_centripetal_acceleration = centripetal_acceleration;
			}
		}
	}
//	printf("max_centripetal_acceleration %4.2lf, dist_to_max_curvature %5.2lf\n", max_centripetal_acceleration, dist_to_max_curvature);

	double limited_target_v = target_v;
	if (max_centripetal_acceleration > robot_max_centripetal_acceleration)
	{
		double radius_of_curvature = L / fabs(tan(max_path_phi));
		double desired_v = sqrt(robot_max_centripetal_acceleration * radius_of_curvature);
		if (target_v < 0.0)
			desired_v = -desired_v;

		if (fabs(desired_v) < fabs(target_v))
		{
			carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi = get_robot_pose();
			double dist_to_goal = carmen_distance_ackerman_traj(&current_robot_pose_v_and_phi, goal);
			double velocity_at_goal = get_velocity_at_goal(current_v, desired_v, dist_to_goal, dist_to_max_curvature);
			if (fabs(velocity_at_goal) < fabs(target_v))
				limited_target_v = velocity_at_goal;
		}
	}

	free(original_path_copy);

	return (limited_target_v);
}


double
limit_maximum_velocity_according_to_maximum_steering_command_rate(carmen_robot_and_trailer_traj_point_t *goal,
		carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi,
		carmen_robot_and_trailer_traj_point_t *poses_ahead, int number_of_poses)
{
	double target_v = goal->v;
	if (number_of_poses < 6)
		return (target_v);

	carmen_robot_and_trailer_traj_point_t *path = (carmen_robot_and_trailer_traj_point_t *) malloc(number_of_poses * sizeof(carmen_robot_and_trailer_traj_point_t));
	memcpy(path, poses_ahead, number_of_poses * sizeof(carmen_robot_and_trailer_traj_point_t));

	carmen_robot_and_trailer_traj_point_t *original_path_copy = path;
	path = &(path[1]);
	number_of_poses -= 1;
	for (int i = 0; i < ((number_of_poses / 4) - 1); i++)
		path[i] = path[i * 4];
	number_of_poses /= 4;
//	plot_state(path, number_of_poses, NULL, 0, true);

	double L = get_robot_config()->distance_between_front_and_rear_axles;
	for (int i = 0; i < (number_of_poses - 1); i++)
	{
		double delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i + 1]);
		path[i].phi = atan(L * (delta_theta / l));
	}

	double previous_curvature = fabs(current_robot_pose_v_and_phi->phi) / L;
	double limited_target_v = target_v;
	double max_allowed_v = fabs(target_v);
	double total_s = 0.0;
	for (int i = 1; i < (number_of_poses - 1); i++)
	{
		double s = DIST2D(path[i], path[i - 1]);
		total_s += s;
		if (total_s > (1.3 * DIST2D(path[i], *goal)))
			break;
		double delta_t = s / fabs(target_v);
//		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
		double curvature = fabs(tan(path[i].phi)) / L;
		double delta_curvature = fabs(curvature - previous_curvature);
		double steering_command_rate = delta_curvature / delta_t;
		if (steering_command_rate > (robot_config.maximum_steering_command_rate * 0.7))
		{
			delta_t = delta_curvature / (robot_config.maximum_steering_command_rate * 0.7);
			double v = s / delta_t;
			if (v < max_allowed_v)
				max_allowed_v = v;
			if (target_v > 0.0)
				limited_target_v = max_allowed_v;
			else
				limited_target_v = -max_allowed_v;
//			printf("  limited_target_v %lf, target_v %lf\n", limited_target_v, target_v);
		}
		previous_curvature = curvature;
	}
//	printf("* limited_target_v %lf, target_v %lf\n", limited_target_v, target_v);

	free(original_path_copy);

	return (limited_target_v);
}

extern SampleFilter filter2;

//TODO @@@Vinicius Acho que current_robot_pose_v_and_phi nao pode ser negativo aqui, bem como o goal->v
//Acho que essa funcao nao faz sentido para dar reh se ela soh considerar os obstaculos moveis para frente. Vou usar o fabs para mante-la funcionando
double
set_goal_velocity_according_to_moving_obstacle(carmen_robot_and_trailer_traj_point_t *goal, carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi,
		int goal_type, double timestamp __attribute__ ((unused)))
{
	double car_pose_to_car_front = get_robot_config()->distance_between_front_and_rear_axles + get_robot_config()->distance_between_front_car_and_front_wheels;
	// um carro de tamanho para cada 10 milhas/h (4.4705 m/s) -> ver "The DARPA Urban Challenge" book, pg. 36.
	double min_dist_according_to_car_v = get_robot_config()->length * (current_robot_pose_v_and_phi->v / 4.4704) + car_pose_to_car_front;
	double desired_distance;
	desired_distance = carmen_fmax(distance_to_moving_object_with_v_multiplier * min_dist_according_to_car_v, car_pose_to_car_front + get_robot_config()->distance_between_front_and_rear_axles);

	double distance = datmo_get_moving_obstacle_distance(*current_robot_pose_v_and_phi, get_robot_config());
	double moving_obj_v = datmo_speed_front();

	// ver "The DARPA Urban Challenge" book, pg. 36.
	double Kgap = 0.1;
	double new_goal_v;
	if (goal->v > moving_obj_v)
		new_goal_v = moving_obj_v + Kgap * (distance - desired_distance);
	else
		new_goal_v = goal->v;
	SampleFilter_put(&filter2, new_goal_v);
	new_goal_v = SampleFilter_get(&filter2);
	if (new_goal_v < 0.0)
		new_goal_v = 0.0;

	if ((goal_type == MOVING_OBSTACLE_GOAL1) || (goal_type == MOVING_OBSTACLE_GOAL2))
		goal->v = carmen_fmin(new_goal_v, goal->v);

//	FILE *caco = fopen("caco.txt", "a");
//	fprintf(caco, "%lf %lf %lf %lf %lf %d %d %lf %lf %lf %d\n", moving_obj_v, goal->v, current_robot_pose_v_and_phi->v, distance,
//			desired_distance, autonomous, goal_type,
//			udatmo_speed_left(), udatmo_speed_right(), udatmo_speed_center(), udatmo_obstacle_detected(timestamp));
//	fflush(caco);
//	fclose(caco);

	return (goal->v);
}


double
compute_s_range(carmen_robot_and_trailer_traj_point_t *poses_ahead, int pose_index, int num_poses)
{
	double s_range = 0.0;
	for (int i = 0; (i < (num_poses - 1)) && (i < pose_index); i++)
		s_range += DIST2D(poses_ahead[i], poses_ahead[i + 1]);

	return (s_range);
}


double
compute_dist_walked_from_robot_to_goal(carmen_robot_and_trailer_traj_point_t *poses_ahead, carmen_robot_and_trailer_traj_point_t *goal_pose, int num_poses)
{
	double s_range = 0.0;
	for (int i = 0; i < (num_poses - 1); i++)
	{
		if (poses_ahead[i].x == goal_pose->x && poses_ahead[i].y == goal_pose->y)
			break;

		s_range += DIST2D(poses_ahead[i], poses_ahead[i + 1]);
	}

	return (s_range);
}


double
set_goal_velocity_according_to_general_moving_obstacle(carmen_robot_and_trailer_traj_point_t *goal,
		carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi __attribute__ ((unused)),
		int goal_type, carmen_rddf_road_profile_message *rddf __attribute__ ((unused)),
		path_collision_info_t path_collision_info __attribute__ ((unused)), double timestamp __attribute__ ((unused)))
{
	if (goal_type == MOVING_OBSTACLE_GOAL3)
	{
//		double car_pose_to_car_front = get_robot_config()->distance_between_front_and_rear_axles + get_robot_config()->distance_between_front_car_and_front_wheels;
//		// um carro de tamanho para cada 10 milhas/h (4.4705 m/s) -> ver "The DARPA Urban Challenge" book, pg. 36.
//		double min_dist_according_to_car_v = get_robot_config()->length * (current_robot_pose_v_and_phi->v / 4.4704) + car_pose_to_car_front;
//		double desired_distance;
//		desired_distance = carmen_fmax(distance_to_moving_object_with_v_multiplier * min_dist_according_to_car_v, car_pose_to_car_front + get_robot_config()->distance_between_front_and_rear_axles);
//
//		double distance;
//		double moving_obj_v;
//		if (path_collision_info.possible_collision_mo_pose_index < path_collision_info.possible_collision_mo_in_parallel_lane_pose_index)
//		{
//			distance = compute_s_range(rddf->poses, path_collision_info.possible_collision_mo_pose_index, rddf->number_of_poses) / 3.0;
//			moving_obj_v = path_collision_info.possible_collision_mo_sv;
//		}
//		else
//		{
//			distance = compute_s_range(rddf->poses, path_collision_info.possible_collision_mo_in_parallel_lane_pose_index, rddf->number_of_poses) / 3.0;
//			moving_obj_v = path_collision_info.possible_collision_mo_in_parallel_lane_sv;
//		}
//		distance = 0.0;
//		// ver "The DARPA Urban Challenge" book, pg. 36.
//		double Kgap = 0.1;
//		double new_goal_v;
//		if (goal->v > moving_obj_v)
//			new_goal_v = moving_obj_v + Kgap * (distance - desired_distance);
//		else
//			new_goal_v = goal->v;
//		if (new_goal_v < 0.0)
//			new_goal_v = 0.0;
//
		goal->v = 0.0;//carmen_fmin(new_goal_v, goal->v);
	}

	return (goal->v);
}


double
set_goal_velocity_according_to_last_speed_limit_annotation(carmen_robot_and_trailer_traj_point_t *goal)
{
	goal->v = carmen_fmin(last_speed_limit, goal->v);

	return (goal->v);
}


double
compute_max_v_using_torricelli(double v_init, double aceleration, double distance)
{
	double max_v = sqrt((v_init * v_init) + (2.0 * aceleration * distance));

	if (max_v > parking_speed_limit)
		max_v = parking_speed_limit;

	return (max_v);
}


int
set_goal_velocity(carmen_robot_and_trailer_traj_point_t *goal, carmen_robot_and_trailer_traj_point_t *current_robot_pose_v_and_phi,
		int goal_type, carmen_rddf_road_profile_message *rddf, path_collision_info_t path_collision_info,
		carmen_behavior_selector_state_message behavior_selector_state_message, double timestamp)
{
	double previous_v;
	int reversing_driving = 0;

//	double static activate_intermediate_velocity = 0;
//	static double initial_dist = 0.0;
//	static double intermediate_velocity = 0.0;
//	double path_dist = 0.0;

	if (behavior_selector_reverse_driving && goal->v < 0.0)
	{
		previous_v = goal->v = get_max_v_reverse();
		reversing_driving = 1;
	}
	else
		previous_v = goal->v = get_max_v();
	int who_set_the_goal_v = NONE;

	if (goal_type == OBSTACLE_GOAL)//@@@Vinicius aqui vai tudo ao quadrado, exceto pelo fmin, (mas passei fabs por garantia) goal_v negativo aqui atrapalha, tem que tratar.
		goal->v = set_goal_velocity_according_to_obstacle_distance(goal, current_robot_pose_v_and_phi);
	if (previous_v != goal->v)
		who_set_the_goal_v = OBSTACLE;

	previous_v = goal->v;//@@@Vinicius verificar para que o sinal velocidade atual nao atrapalhe na distancia de seguranca. Porem tem que ver o tratamento de objetos moveis para reh
	goal->v = set_goal_velocity_according_to_moving_obstacle(goal, current_robot_pose_v_and_phi, goal_type, timestamp);
	if (previous_v != goal->v)
		who_set_the_goal_v = MOVING_OBSTACLE;

	previous_v = goal->v; //@@@Vinicius Nao fiz nada, apenas coloca o goal pra zero
	goal->v = set_goal_velocity_according_to_general_moving_obstacle(goal, current_robot_pose_v_and_phi, goal_type,
			rddf, path_collision_info, timestamp);
	if (previous_v != goal->v)
		who_set_the_goal_v = MOVING_OBSTACLE;

	previous_v = goal->v; //Limita a velocidade quando o offroad eh acionado, tanto para frente quanto para reh
	if (((road_network_message != NULL) && (road_network_message->route_planner_state == EXECUTING_OFFROAD_PLAN) &&
		 (((reversing_driving == 0) && (goal->v > parking_speed_limit)) || ((reversing_driving == 1) && (goal->v < parking_speed_limit)))) ||
		 (behavior_selector_get_task() == BEHAVIOR_SELECTOR_PARK) ||
		 (behavior_selector_get_task() == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) ||
		 (behavior_selector_get_task() == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER) ||
	 	(behavior_selector_get_task() == BEHAVIOR_SELECTOR_MOVE_TO_ENGAGE_POSE))
	 	goal->v = (reversing_driving == 1)? -parking_speed_limit : parking_speed_limit;
	if (previous_v != goal->v)
		who_set_the_goal_v = PARKING_MANOUVER;

	previous_v = goal->v;
	if (keep_speed_limit) //@@@Vinicius Aqui gol_v nao pode ser negativo fmin
		goal->v = set_goal_velocity_according_to_last_speed_limit_annotation(goal);
	if (previous_v != goal->v)
		who_set_the_goal_v = KEEP_SPEED_LIMIT;

	previous_v = goal->v;
	if (((road_network_message != NULL) && (road_network_message->route_planner_state == IN_RECTLINEAR_ROUTE_SEGMENT) &&
		 (goal->v > move_to_engage_pose_speed_limit)) ||
	 	(behavior_selector_get_task() == BEHAVIOR_SELECTOR_MOVE_TO_ENGAGE_POSE))
	 	goal->v = (reversing_driving == 1)? -move_to_engage_pose_speed_limit : move_to_engage_pose_speed_limit;
	if (previous_v != goal->v)
		who_set_the_goal_v = MOVING_TO_ENGAGE_POSE_MANOUVER;

	previous_v = goal->v; //@@@Vinicius target_v nao pode ser negativo esse robot_pose eh diferente do current..pose_v? tratar ele para garantir de nao ser usado errado
	goal->v = limit_maximum_velocity_according_to_centripetal_acceleration(goal->v, get_robot_pose().v, goal,
			last_rddf_message->poses, last_rddf_message->number_of_poses);
	if (previous_v != goal->v)
		who_set_the_goal_v = CENTRIPETAL_ACCELERATION;

	previous_v = goal->v; //@@@Vinicius Aqui tem que tratar as anotacoes para frente dependendo da direcao que o carro ta indo e alguns Fmin (Tratado)
	goal->v = set_goal_velocity_according_to_annotation(goal, goal_type, current_robot_pose_v_and_phi, path_collision_info,
			behavior_selector_state_message, timestamp);
	if (previous_v != goal->v)
		who_set_the_goal_v = ANNOTATION;

	previous_v = goal->v;
	if (goal_type == SWITCH_VELOCITY_SIGNAL_GOAL)
		goal->v = 0.0;
	if (previous_v != goal->v)
		who_set_the_goal_v = STOP_AT_SWITCH_VELOCITY_SIGNAL;

	previous_v = goal->v;
	if (goal_type == FINAL_GOAL)
		goal->v = 0.0;
	if (previous_v != goal->v)
		who_set_the_goal_v = STOP_AT_FINAL_GOAL;

	previous_v = goal->v;
	if ((fabs(goal->v) < 1.0) && (fabs(current_robot_pose_v_and_phi->v) < 0.5) &&
//		(DIST2D_P(current_robot_pose_v_and_phi, goal) < distance_between_waypoints_and_goals()) &&
		(DIST2D_P(current_robot_pose_v_and_phi, goal) > 0.5))
	{
		double path_dist = compute_dist_walked_from_robot_to_goal(rddf->poses, goal, rddf->number_of_poses);
		double intermediate_velocity = compute_max_v_using_torricelli(current_robot_pose_v_and_phi->v, get_robot_config()->maximum_acceleration_forward, path_dist / 2.0);
		if (reversing_driving)
			goal->v = -intermediate_velocity;
		else
			goal->v = intermediate_velocity;
	}
	if (previous_v != goal->v)
		who_set_the_goal_v = TORRICHELLI;

	previous_v = goal->v;
	goal->v = limit_maximum_velocity_according_to_maximum_steering_command_rate(goal, current_robot_pose_v_and_phi, last_rddf_message->poses, last_rddf_message->number_of_poses);
	if (previous_v != goal->v)
		who_set_the_goal_v = STEERING_COMMAND_RATE;

	previous_v = goal->v;
	goal->v = set_goal_velocity_according_to_state_machine(goal, behavior_selector_state_message);
	if (previous_v != goal->v)
		who_set_the_goal_v = STATE_MACHINE;

//	printf("timestamp %lf, goal->v %lf, who_set_the_goal_v %d, bs_state %d, rp_state %d\n",
//			behavior_selector_state_message.timestamp, goal->v, who_set_the_goal_v, behavior_selector_state_message.low_level_state, behavior_selector_state_message.route_planner_state);

	return (who_set_the_goal_v);
}
