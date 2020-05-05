#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/rddf_messages.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_util.h>
#include <carmen/udatmo.h>
#include "behavior_selector.h"

extern bool wait_start_moving;

extern carmen_rddf_annotation_message last_rddf_annotation_message;
extern carmen_rddf_road_profile_message *last_rddf_message;
extern bool last_rddf_annotation_message_valid;

extern bool autonomous;

extern double last_speed_limit;

extern double robot_max_centripetal_acceleration;
extern double distance_to_moving_object_with_v_multiplier;
extern bool keep_speed_limit;


carmen_ackerman_traj_point_t
displace_pose(carmen_ackerman_traj_point_t robot_pose, double displacement)
{
	carmen_point_t displaced_robot_position = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&robot_pose, displacement);

	carmen_ackerman_traj_point_t displaced_robot_pose = robot_pose;
	displaced_robot_pose.x = displaced_robot_position.x;
	displaced_robot_pose.y = displaced_robot_position.y;

	return (displaced_robot_pose);
}


carmen_annotation_t *
get_nearest_specified_annotation(int annotation, carmen_rddf_annotation_message annotation_message,
		carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi)
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


bool
busy_pedestrian_track_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	static double last_pedestrian_track_busy_timestamp = 0.0;

//	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
//				&current_robot_pose_v_and_phi, false);
//
//	if (nearest_velocity_related_annotation == NULL)
//		return (false);
//
//	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
//	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	carmen_annotation_t *nearest_pedestrian_track_annotation = get_nearest_specified_annotation(RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK,
			last_rddf_annotation_message, &displaced_robot_pose);

	if (nearest_pedestrian_track_annotation == NULL)
		return (false);

	if ((nearest_pedestrian_track_annotation->annotation_code == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_BUSY))// &&
//		(nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) &&
//		(distance_to_act_on_annotation >= distance_to_annotation) &&
//		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_velocity_related_annotation->annotation_point))
		last_pedestrian_track_busy_timestamp = timestamp;

	if (timestamp - last_pedestrian_track_busy_timestamp < 1.5)
		return (true);

	return (false);
}


carmen_annotation_t *
get_nearest_velocity_related_annotation(carmen_rddf_annotation_message annotation_message,
		carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi, bool wait_start_moving)
{
	int nearest_annotation_index = -1;
	double distance_to_nearest_annotation = 1000.0;

	for (int i = 0; i < annotation_message.num_annotations; i++)
	{
		double distance_to_annotation = DIST2D_P(&annotation_message.annotations[i].annotation_point, current_robot_pose_v_and_phi);
		if (((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BARRIER) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) ||
			 ((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_STOP) && !wait_start_moving) ||
			 ((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) && !wait_start_moving) ||
			 ((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) && busy_pedestrian_track_ahead(*current_robot_pose_v_and_phi, carmen_get_time()) && !wait_start_moving) ||
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


double
get_distance_to_act_on_annotation(double v0, double va, double distance_to_annotation)
{
	// @@@ Alberto: rever pois esta fazendo a IARA lesmar quando esta lenta e longe de uma anotacao
	// va = v0 + a * t
	// a*t = v - v0
	// t = (va - v0) / a
	// da = va * t + 0.5 * a * t * t

	double a = -get_robot_config()->maximum_acceleration_forward * 1.1;
	double t = (va - v0) / a;
	double daa = v0 * t + 0.5 * a * t * t;

	//printf("t %.2lf, v0 %.1lf, va %.1lf, a %.2lf, tt %.2lf, daa %.1lf, da %.1lf\n", carmen_get_time(), v0, va, a, t, daa, distance_to_annotation);
	if (daa > distance_to_annotation)
		return (daa);// + 3.0 * get_robot_config()->distance_between_front_and_rear_axles);
	else
		return (distance_to_annotation);
}


bool
red_traffic_light_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	static double last_red_timestamp = 0.0;

	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, false);

	if (nearest_velocity_related_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	carmen_annotation_t *nearest_traffic_light_annotation = get_nearest_specified_annotation(RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT,
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
get_velocity_at_next_annotation(carmen_annotation_t *annotation, carmen_ackerman_traj_point_t current_robot_pose_v_and_phi,
		double timestamp)
{
	double v = 60.0 / 3.6;

	if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) &&
		red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
		v = 0.08;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) &&
			 busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
		v = 0.08;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) &&
			 busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
		v = 0.08;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP)
		v = 0.08;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_DYNAMIC) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_DYNAMIC_STOP))
		v = 0.09;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_BUMP)
		v = 2.5;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)
		v = 2.5;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_BARRIER)
		v = 0.6; //1.2 //2.0 reduzido pois as cancelas estao mais lentas para abrir
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
distance_to_moving_obstacle_annotation(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
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
compute_distance_within_rddf(carmen_vector_3D_t annotation_point, carmen_ackerman_traj_point_t  current_robot_pose_v_and_phi)
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
set_goal_velocity_according_to_annotation(carmen_ackerman_traj_point_t *goal, int goal_type,
		carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi, double timestamp)
{
	static bool clearing_annotation = false;
	static carmen_vector_3D_t previous_annotation_point = {0.0, 0.0, 0.0};

	if (!autonomous)
		clearing_annotation = false;

	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
			current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation != NULL)
	{
//		carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(*current_robot_pose_v_and_phi,
//				get_robot_config()->distance_between_front_and_rear_axles +
//				get_robot_config()->distance_between_front_car_and_front_wheels);

		double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, *current_robot_pose_v_and_phi);
//		double distance_to_annotation = compute_distance_within_rddf(nearest_velocity_related_annotation->annotation_point, *current_robot_pose_v_and_phi);
//		FILE *caco13 = fopen("caco13.txt", "a");
//		fprintf(caco13, "%.2lf %.2lf\n", distance_to_annotation, DIST2D(nearest_velocity_related_annotation->annotation_point, *current_robot_pose_v_and_phi));
//		fflush(caco13);
//		fclose(caco13);

		double velocity_at_next_annotation = get_velocity_at_next_annotation(nearest_velocity_related_annotation, *current_robot_pose_v_and_phi, timestamp);

		double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi->v, velocity_at_next_annotation,
				distance_to_annotation);
		bool annotation_ahead = carmen_rddf_play_annotation_is_forward(*current_robot_pose_v_and_phi, nearest_velocity_related_annotation->annotation_point);

		double distance_to_goal = carmen_distance_ackerman_traj(current_robot_pose_v_and_phi, goal);

		if (last_rddf_annotation_message_valid &&
			(clearing_annotation ||
			 (((distance_to_annotation < distance_to_act_on_annotation) ||
			   (distance_to_annotation < distance_to_goal)) && annotation_ahead) ||
			   ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
				((goal_type == ANNOTATION_GOAL2) || (goal_type == ANNOTATION_GOAL2)))))
		{
			if (!clearing_annotation)
				previous_annotation_point = nearest_velocity_related_annotation->annotation_point;

			clearing_annotation = true;
			goal->v = carmen_fmin(
					get_velocity_at_goal(current_robot_pose_v_and_phi->v, velocity_at_next_annotation, distance_to_goal, distance_to_annotation),
					goal->v);

			if (((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
					((goal_type == ANNOTATION_GOAL2) || (goal_type == ANNOTATION_GOAL2))))
				goal->v = get_velocity_at_next_annotation(nearest_velocity_related_annotation, *current_robot_pose_v_and_phi,
						timestamp);
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
set_goal_velocity_according_to_obstacle_distance(carmen_ackerman_traj_point_t *goal, carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi)
{
	double distance_to_obstacle = DIST2D_P(current_robot_pose_v_and_phi, goal);

	goal->v = carmen_fmin(
				get_velocity_at_goal(current_robot_pose_v_and_phi->v, 0.0, distance_to_obstacle, distance_to_obstacle),
				goal->v);

	return (goal->v);
}


double
limit_maximum_velocity_according_to_centripetal_acceleration(double target_v, double current_v, carmen_ackerman_traj_point_t *goal,
		carmen_ackerman_traj_point_t *path, int number_of_poses)
{
	if (number_of_poses == 0)
		return (target_v);

	double desired_v = 0.0;
	double max_centripetal_acceleration = 0.0;
	double dist_walked = 0.0;
	double dist_to_max_curvature = 0.0;
	double max_path_phi = 0.0;
	double L = get_robot_config()->distance_between_front_and_rear_axles;

	for (int i = 0; i < (number_of_poses - 1); i++)
	{
		double delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i + 1]);
		dist_walked += l;
		if (l < 0.01)
		{
			path[i].phi = 0.0;
			continue;
		}
		path[i].phi = atan((L * delta_theta) / l);
	}

	for (int i = 1; i < (number_of_poses - 1); i++)
	{
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

	double limited_target_v = target_v;
	if (max_centripetal_acceleration > robot_max_centripetal_acceleration)
	{
		double radius_of_curvature = L / fabs(tan(max_path_phi));
		desired_v = sqrt(robot_max_centripetal_acceleration * radius_of_curvature);
		if (desired_v < target_v)
		{
			carmen_ackerman_traj_point_t current_robot_pose_v_and_phi = get_robot_pose();
			double dist_to_goal = carmen_distance_ackerman_traj(&current_robot_pose_v_and_phi, goal);
			double velocity_at_goal = get_velocity_at_goal(current_v, desired_v, dist_to_goal, dist_to_max_curvature);
			if (velocity_at_goal < target_v)
				limited_target_v = velocity_at_goal;
		}
	}

	return (carmen_fmin(limited_target_v, target_v));
}

extern SampleFilter filter2;


double
set_goal_velocity_according_to_moving_obstacle(carmen_ackerman_traj_point_t *goal, carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi,
		int goal_type, double timestamp __attribute__ ((unused)))
{
	double car_pose_to_car_front = get_robot_config()->distance_between_front_and_rear_axles + get_robot_config()->distance_between_front_car_and_front_wheels;
	// um carro de tamanho para cada 10 milhas/h (4.4705 m/s) -> ver "The DARPA Urban Challenge" book, pg. 36.
	double min_dist_according_to_car_v = get_robot_config()->length * (current_robot_pose_v_and_phi->v / 4.4704) + car_pose_to_car_front;
	double desired_distance;
	desired_distance = carmen_fmax(distance_to_moving_object_with_v_multiplier * min_dist_according_to_car_v, car_pose_to_car_front + 2.5);

	double distance = udatmo_get_moving_obstacle_distance(*current_robot_pose_v_and_phi, get_robot_config());
	double moving_obj_v = udatmo_speed_front();

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

	if ((goal_type == MOVING_OBSTACLE_GOAL1) || (goal_type == MOVING_OBSTACLE_GOAL2))//udatmo_obstacle_detected(timestamp))// && (current_robot_pose_v_and_phi->v > moving_obj_v))
		goal->v = carmen_fmin(new_goal_v, goal->v);

	FILE *caco = fopen("caco.txt", "a");
	fprintf(caco, "%lf %lf %lf %lf %lf %d %d %lf %lf %lf %d\n", moving_obj_v, goal->v, current_robot_pose_v_and_phi->v, distance,
			desired_distance, autonomous, goal_type,
			udatmo_speed_left(), udatmo_speed_right(), udatmo_speed_center(), udatmo_obstacle_detected(timestamp));
	fflush(caco);
	fclose(caco);

	return (goal->v);
}


double
set_goal_velocity_according_to_last_speed_limit_annotation(carmen_ackerman_traj_point_t *goal)
{
	goal->v = carmen_fmin(last_speed_limit, goal->v);

	return (goal->v);
}


int
set_goal_velocity(carmen_ackerman_traj_point_t *goal, carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi,
		int goal_type, double timestamp)
{
	int who_set_the_goal_v = NONE;
	double previous_v = goal->v = get_max_v();
	if (goal_type == OBSTACLE_GOAL)
		goal->v = set_goal_velocity_according_to_obstacle_distance(goal, current_robot_pose_v_and_phi);
	if (previous_v != goal->v)
		who_set_the_goal_v = OBSTACLE;

	previous_v = goal->v;
	goal->v = set_goal_velocity_according_to_moving_obstacle(goal, current_robot_pose_v_and_phi, goal_type, timestamp);
	if (previous_v != goal->v)
		who_set_the_goal_v = MOVING_OBSTACLE;

	previous_v = goal->v;
	goal->v = limit_maximum_velocity_according_to_centripetal_acceleration(goal->v, get_robot_pose().v, goal,
			last_rddf_message->poses, last_rddf_message->number_of_poses);
	if (previous_v != goal->v)
		who_set_the_goal_v = CENTRIPETAL_ACCELERATION;

	previous_v = goal->v;
	goal->v = set_goal_velocity_according_to_annotation(goal, goal_type, current_robot_pose_v_and_phi, timestamp);
	if (previous_v != goal->v)
		who_set_the_goal_v = ANNOTATION;

	previous_v = goal->v;
	if (keep_speed_limit)
		goal->v = set_goal_velocity_according_to_last_speed_limit_annotation(goal);
	if (previous_v != goal->v)
		who_set_the_goal_v = KEEP_SPEED_LIMIT;

	return (who_set_the_goal_v);
}
