#include <vector>
#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/rddf_messages.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/collision_detection.h>
#include "behavior_selector.h"

using namespace std;


extern carmen_frenet_path_planner_set_of_paths *current_set_of_paths;
extern carmen_moving_objects_point_clouds_message *current_moving_objects;

extern carmen_obstacle_distance_mapper_map_message distance_map;

extern int frenet_path_planner_num_paths;
extern double frenet_path_planner_paths_displacement;
extern double frenet_path_planner_time_horizon;
extern double frenet_path_planner_delta_t;

extern int use_unity_simulator;
extern double in_lane_longitudinal_safety_magin_multiplier;
extern double in_path_longitudinal_safety_magin_multiplier;

extern carmen_mapper_virtual_laser_message virtual_laser_message;


double
compute_s_range(carmen_ackerman_traj_point_t *poses_ahead, int num_poses)
{
	double s_range = 0.0;
	for (int i = 0; i < num_poses; i++)
		s_range += DIST2D(poses_ahead[i], poses_ahead[i + 1]);

	return (s_range);
}


void
get_s_and_d_values(carmen_ackerman_traj_point_t *poses, int nearest_pose, t_point_cloud_struct *moving_object,
		double &estimated_s_v, double &estimated_d_t_0, double &estimated_d_v)
{
	double distance_moving_object_to_nearest_path_pose = DIST2D(poses[nearest_pose], moving_object->object_pose);
	double angle = ANGLE2D(poses[nearest_pose], moving_object->object_pose);
	double angle_with_respect_to_nearest_path_pose = carmen_normalize_theta(angle - poses[nearest_pose].theta);

	estimated_d_t_0 = distance_moving_object_to_nearest_path_pose * sin(angle_with_respect_to_nearest_path_pose);

	double angle_difference = carmen_normalize_theta(moving_object->orientation - poses[nearest_pose].theta);
	estimated_s_v = moving_object->linear_velocity * cos(angle_difference);
	estimated_d_v = moving_object->linear_velocity * sin(angle_difference);
}


double
get_s_displacement_for_a_given_sample(int s, double v, double delta_t)
{
	double t = (double) s * delta_t;
	double a = 0.0; // get_robot_config()->maximum_acceleration_forward;
	v = v + a * t;
	if (v > get_max_v())
		v = get_max_v();
	double s_displacement = v * t;

	return (s_displacement);
}


carmen_point_t
get_car_pose_at_s_displacement(double s_displacement, carmen_ackerman_traj_point_t *path, int &i)
{
	double s_range = 0.0;
	for (i = 0; (s_range < s_displacement) && (i < current_set_of_paths->number_of_poses - 1); i++)
		s_range += DIST2D(path[i], path[i + 1]);

	carmen_point_t car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&(path[i]),
			s_displacement - s_range);

	return (car_pose);
}

//extern int add_virtual_laser_points;


double
collision_s_distance_to_static_object(carmen_frenet_path_planner_set_of_paths *current_set_of_paths, int path_id,
		double delta_t, int num_samples)//, carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_ackerman_traj_point_t *path = &(current_set_of_paths->set_of_paths[path_id * current_set_of_paths->number_of_poses]);

	double min_collision_s_distance = (double) num_samples * delta_t;
	for (int s = 0; s < num_samples; s++)
	{
		double v = get_max_v();
//		if (current_robot_pose_v_and_phi.v < 5.0)
//			v = 5.0;
//		else
//			v = current_robot_pose_v_and_phi.v;
		double s_displacement = get_s_displacement_for_a_given_sample(s, v, delta_t);
		int index_in_path;
		carmen_point_t car_pose = get_car_pose_at_s_displacement(s_displacement, path, index_in_path);
		if (index_in_path == current_set_of_paths->number_of_poses - 1)
			break;

		carmen_ackerman_traj_point_t cp = {car_pose.x, car_pose.y, car_pose.theta, 0.0, 0.0};
		if (trajectory_pose_hit_obstacle(cp, get_robot_config()->model_predictive_planner_obstacles_safe_distance, &distance_map, NULL) == 1)
		{
				virtual_laser_message.positions[virtual_laser_message.num_positions].x = car_pose.x;
				virtual_laser_message.positions[virtual_laser_message.num_positions].y = car_pose.y;
				virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_RED;
				virtual_laser_message.num_positions++;

			if (((double) s * delta_t) < min_collision_s_distance)
				min_collision_s_distance = (double) s * delta_t;

			break;
		}
	}

	return (min_collision_s_distance);
}


double
collision_s_distance_to_moving_object(carmen_frenet_path_planner_set_of_paths *current_set_of_paths, int path_id,
		vector<vector <moving_object_pose_info_t> > moving_objects_poses, double delta_t, int num_samples,
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	if (!current_moving_objects)
		return ((double) num_samples * delta_t);

	carmen_ackerman_traj_point_t *path = &(current_set_of_paths->set_of_paths[path_id * current_set_of_paths->number_of_poses]);

	double min_collision_s_distance = (double) num_samples * delta_t;
	for (int s = 0; s < num_samples; s++)
	{
		double s_displacement = get_s_displacement_for_a_given_sample(s, current_robot_pose_v_and_phi.v, delta_t);
		int index_in_path;
		carmen_point_t car_pose = get_car_pose_at_s_displacement(s_displacement, path, index_in_path);
		if (index_in_path == current_set_of_paths->number_of_poses - 1)
			break;

 		for (int i = 0; i < current_moving_objects->num_point_clouds; i++)
		{
// 			double longitudinal_safety_magin;
// 			if (moving_objects_poses[s][i].behind && (fabs(moving_objects_poses[s][i].d - current_d) < 0.8)) // moving objects atras do carro
// 				longitudinal_safety_magin = 0.5;
// 			else
 			double longitudinal_safety_magin = in_path_longitudinal_safety_magin_multiplier * current_moving_objects->point_clouds[i].linear_velocity;
			double lateral_safety_margin = get_robot_config()->model_predictive_planner_obstacles_safe_distance;// +
//					0.1 * current_moving_objects->point_clouds[i].linear_velocity *
//					(current_moving_objects->point_clouds[i].width / current_moving_objects->point_clouds[i].length);
			carmen_point_t moving_objects_pose = {moving_objects_poses[s][i].pose.x, moving_objects_poses[s][i].pose.y, moving_objects_poses[s][i].pose.theta};

//			if ((s == 50) && (DIST2D(car_pose, moving_objects_pose) < 20.0) && (virtual_laser_message.num_positions < 5000))
//				add_virtual_laser_points = 0;
//			else
//				add_virtual_laser_points = 0;
			if (carmen_obstacle_avoider_car_collides_with_moving_object(car_pose, moving_objects_pose,
					&(current_moving_objects->point_clouds[i]), longitudinal_safety_magin, lateral_safety_margin))//,
//					i, moving_objects_poses[s][i].s, moving_objects_poses[s][i].d))
			{
				virtual_laser_message.positions[virtual_laser_message.num_positions].x = car_pose.x;
				virtual_laser_message.positions[virtual_laser_message.num_positions].y = car_pose.y;
				virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_ORANGE;
				virtual_laser_message.num_positions++;

				if (((double) s * delta_t) < min_collision_s_distance)
					min_collision_s_distance = (double) s * delta_t;

				break;
			}
		}
	}

	return (min_collision_s_distance);
}


carmen_ackerman_traj_point_t *
get_rddf_displaced(carmen_ackerman_traj_point_t *rddf_poses_ahead, int number_of_poses, double path_displacement)
{
	carmen_ackerman_traj_point_t *displaced_rddf = (carmen_ackerman_traj_point_t *) malloc(number_of_poses * sizeof(carmen_ackerman_traj_point_t));
	for (int i = 0; i < number_of_poses; i++)
	{
		displaced_rddf[i] = rddf_poses_ahead[i]; // para copiar os outros campos de carmen_ackerman_traj_point_t
		displaced_rddf[i].x = rddf_poses_ahead[i].x + path_displacement * cos(rddf_poses_ahead[i].theta + (M_PI / 2.0));
		displaced_rddf[i].y = rddf_poses_ahead[i].y + path_displacement * sin(rddf_poses_ahead[i].theta + (M_PI / 2.0));
	}

	return (displaced_rddf);
}


double
collision_s_distance_to_moving_object_in_parallel_lane(carmen_frenet_path_planner_set_of_paths *current_set_of_paths, int path_id,
		vector<vector <moving_object_pose_info_t> > moving_objects_poses, double delta_t, int num_samples,
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	if (!current_moving_objects)
		return ((double) num_samples * delta_t);

	double disp = frenet_path_planner_paths_displacement;
	double path_displacement = disp * ((frenet_path_planner_num_paths / 2) - path_id);
	carmen_ackerman_traj_point_t *path = get_rddf_displaced(current_set_of_paths->rddf_poses_ahead, current_set_of_paths->number_of_poses, path_displacement);

	double min_s_distance = (double) num_samples * delta_t;
	for (int s = 0; s < num_samples; s++)
	{
		double s_displacement = get_s_displacement_for_a_given_sample(s, current_robot_pose_v_and_phi.v, delta_t);
		int index_in_path;
		carmen_point_t car_pose = get_car_pose_at_s_displacement(s_displacement, path, index_in_path);
		if (index_in_path == current_set_of_paths->number_of_poses - 1)
			break;

 		for (int i = 0; i < current_moving_objects->num_point_clouds; i++)
		{
 			double longitudinal_safety_magin = in_lane_longitudinal_safety_magin_multiplier * current_moving_objects->point_clouds[i].linear_velocity;
			double lateral_safety_margin = get_robot_config()->model_predictive_planner_obstacles_safe_distance;
			carmen_point_t moving_objects_pose = {moving_objects_poses[0][i].pose.x, moving_objects_poses[0][i].pose.y, moving_objects_poses[0][i].pose.theta};
			if (carmen_obstacle_avoider_car_collides_with_moving_object(car_pose, moving_objects_pose,
					&(current_moving_objects->point_clouds[i]), longitudinal_safety_magin, lateral_safety_margin))//, 0, 0.0, 0.0))
			{
				virtual_laser_message.positions[virtual_laser_message.num_positions].x = car_pose.x;
				virtual_laser_message.positions[virtual_laser_message.num_positions].y = car_pose.y;
				virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_GREEN;
				virtual_laser_message.num_positions++;

				if (((double) s * delta_t) < min_s_distance)
					min_s_distance = (double) s * delta_t;

				break;
			}
		}
	}

	free(path);

	return (min_s_distance);
}


int
find_nearest_pose(carmen_ackerman_traj_point_t *path, int path_size, carmen_vector_3D_t object_pose)
{
	int i = 0;
	int nearest_pose = i;
	double min_distance = DIST2D(path[i], object_pose);
	for (i = 1; i < path_size; i++)
	{
		double distance = DIST2D(path[i], object_pose);
		if (distance < min_distance)
		{
			nearest_pose = i;
			min_distance = distance;
		}
	}

	return (nearest_pose);
}


moving_object_pose_info_t
simulate_moving_object_movement(t_point_cloud_struct *moving_object, carmen_ackerman_traj_point_t *path, int path_size,
		carmen_ackerman_traj_point_t *poses_back, int number_of_poses_back, double t)
{
	carmen_point_t moving_object_pose;

	double estimated_d_t_0 = 0.0;	// mais proximo da pose do moving_object e ajustar para o ponto intermediario entre duas poses
									// do poses_back ou path para maior precisao.
	double estimated_s_v = moving_object->linear_velocity;  // Pegar a diferenca entre o angulo do path (ou da poses_back) e o angulo
	double estimated_d_v = 0.0;								// da velocidade do moving_object, e calcular a velocidade longitudinal e
															// transversal...

	int nearest_pose_front = find_nearest_pose(path, path_size, moving_object->object_pose);
	int nearest_pose_back = find_nearest_pose(poses_back, number_of_poses_back, moving_object->object_pose);
	if (nearest_pose_front > nearest_pose_back)
		get_s_and_d_values(path, nearest_pose_front, moving_object, estimated_s_v, estimated_d_t_0, estimated_d_v);
	else
		get_s_and_d_values(poses_back, nearest_pose_back, moving_object, estimated_s_v, estimated_d_t_0, estimated_d_v);

	double s_displacement = estimated_s_v * t;
	double d_displacement = estimated_d_t_0;// + estimated_d_v * t;

	double s_range = 0.0;
	int i;
	carmen_ackerman_traj_point_t movin_object_pose_at_t;
	bool behind;
	if (nearest_pose_front >= nearest_pose_back)
	{
		behind = false;

		for (i = nearest_pose_front; (s_range < s_displacement) && (i < path_size - 1); i++)
			s_range += DIST2D(path[i], path[i + 1]);

		movin_object_pose_at_t = path[i];
	}
	else
	{
		behind = true;

		for (i = nearest_pose_back; (s_range < s_displacement) && (i > 0); i--)
			s_range += DIST2D(poses_back[i], poses_back[i - 1]);

		movin_object_pose_at_t = poses_back[i];

		if ((i == 0) && (s_range < s_displacement)) // o moving_object vai ultrapassar o carro
		{
			for ( ; (s_range < s_displacement) && (i < path_size - 1); i++)
				s_range += DIST2D(path[i], path[i + 1]);

			movin_object_pose_at_t = path[i];
		}
	}

	moving_object_pose = carmen_collision_detection_displace_car_on_its_frenet_frame(&movin_object_pose_at_t, s_displacement - s_range, d_displacement);

	moving_object_pose_info_t moving_object_pose_info;
	moving_object_pose_info.pose = moving_object_pose;
	moving_object_pose_info.s = s_range - (s_range - s_displacement);
	moving_object_pose_info.d = d_displacement;
	moving_object_pose_info.behind = behind;

	return (moving_object_pose_info);
}


void
compute_moving_objects_future_poses(vector<vector <moving_object_pose_info_t> > &moving_objects_poses, double delta_t, int num_poses,
		carmen_frenet_path_planner_set_of_paths *current_set_of_paths)
{
	if (!current_moving_objects)
		return;

	carmen_ackerman_traj_point_t *path = current_set_of_paths->rddf_poses_ahead;

	for (int i = 0; i < current_moving_objects->num_point_clouds; i++)
	{
		for (int s = 0; s < num_poses; s++)
		{
			moving_object_pose_info_t moving_object_pose_info = simulate_moving_object_movement(&(current_moving_objects->point_clouds[i]),
					path, current_set_of_paths->number_of_poses, current_set_of_paths->rddf_poses_back,
					current_set_of_paths->number_of_poses_back, delta_t * (double) s);

			moving_objects_poses[s][i] = moving_object_pose_info;
//			if (DIST2D(current_moving_objects->point_clouds[i].object_pose, path[0]) < 25.0)
//			{
//				virtual_laser_message.positions[virtual_laser_message.num_positions].x = moving_object_pose_info.pose.x;
//				virtual_laser_message.positions[virtual_laser_message.num_positions].y = moving_object_pose_info.pose.y;
//				virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_LIGHT_GREEN;
//				virtual_laser_message.num_positions++;
//			}
		}
	}
}


bool
path_with_collision_in_between(int path_id, vector<path_collision_info_t> path_collision_info_v, int current_path)
{
	for (int i = current_path + 1; i < path_id; i++)
		if (path_collision_info_v[i].s_distance_to_moving_object_in_parallel_lane < 0.15)
			return (true);

	for (int i = path_id + 1; i < current_path; i++)
		if (path_collision_info_v[i].s_distance_to_moving_object_in_parallel_lane < 0.15)
			return (true);

	return (false);
}


vector<path_collision_info_t>
get_paths_collision_info(carmen_frenet_path_planner_set_of_paths *current_set_of_paths,
		vector<vector <moving_object_pose_info_t> > moving_objects_data, carmen_ackerman_traj_point_t current_robot_pose_v_and_phi,
		double delta_t, int num_samples)
{
	int number_of_paths = current_set_of_paths->set_of_paths_size / current_set_of_paths->number_of_poses;
	vector<path_collision_info_t> path_collision_info_v;

	for (int path_id = 0; path_id < number_of_paths; path_id++)
	{
		path_collision_info_t path_collision_info;

		path_collision_info.path_id = path_id;
		path_collision_info.s_distance_without_collision_with_moving_object = collision_s_distance_to_moving_object(current_set_of_paths,
				path_id, moving_objects_data, delta_t, num_samples, current_robot_pose_v_and_phi);

		path_collision_info.s_distance_to_moving_object_in_parallel_lane = collision_s_distance_to_moving_object_in_parallel_lane(current_set_of_paths, path_id,
				moving_objects_data, delta_t, num_samples, current_robot_pose_v_and_phi);

		path_collision_info.s_distance_without_collision_with_static_object = collision_s_distance_to_static_object(current_set_of_paths,
				path_id, delta_t, num_samples);//, current_robot_pose_v_and_phi);

		if (use_unity_simulator)
			path_collision_info.s_distance_without_collision_with_static_object = (double) num_samples * delta_t;

		path_collision_info_v.push_back(path_collision_info);
	}

	for (int path_id = 0; path_id < number_of_paths; path_id++)
	{
		if ((path_collision_info_v[path_id].s_distance_without_collision_with_static_object == (double) num_samples * delta_t) &&
			(path_collision_info_v[path_id].s_distance_without_collision_with_moving_object == (double) num_samples * delta_t) &&
			(path_collision_info_v[path_id].s_distance_to_moving_object_in_parallel_lane != 0.0) &&
			!path_with_collision_in_between(path_id, path_collision_info_v, current_set_of_paths->selected_path))
			path_collision_info_v[path_id].path_has_no_collision = true;
		else
			path_collision_info_v[path_id].path_has_no_collision = false;
	}

	return (path_collision_info_v);
}


int
get_path_better_than_the_current_path(vector<path_collision_info_t> paths,
		carmen_frenet_path_planner_set_of_paths *current_set_of_paths, double *path_temporal_value)
{
	int current_path = current_set_of_paths->selected_path;
	double best_path_value = -1000.0;
	int best_path = -1;
	for (unsigned int i = 0; i < paths.size(); i++)
	{
		double lateral_value;
		double longitudinal_value;
		if (use_unity_simulator)
		{
			lateral_value = paths.size() - abs((int) i - current_path);
			longitudinal_value = 100.0  * paths[i].s_distance_without_collision_with_moving_object +
								 100.0  * paths[i].s_distance_to_moving_object_in_parallel_lane;
		}
		else
		{
			lateral_value = 10.0 * (paths.size() - abs((int) i - ((int) paths.size() / 2)));
			longitudinal_value = 1000.0 * paths[i].s_distance_without_collision_with_static_object +
								 100.0  * paths[i].s_distance_without_collision_with_moving_object +
								 100.0  * paths[i].s_distance_to_moving_object_in_parallel_lane;
		}
		double path_value = longitudinal_value + lateral_value + path_temporal_value[i];
		printf("\npath %d, v %5.1lf, lv %5.1lf (%5.3lf, %5.3lf, %5.3lf), lat_value %3.1lf, t_value %5.1lf",
				i, path_value, longitudinal_value,
				paths[i].s_distance_without_collision_with_static_object,
				paths[i].s_distance_without_collision_with_moving_object,
				paths[i].s_distance_to_moving_object_in_parallel_lane,
				lateral_value, path_temporal_value[i]);
		if (path_value > best_path_value)
		{
			best_path_value = path_value;
			best_path = i;
		}
	}

	printf("  -> best_path %d (%d)\n", (paths[best_path].path_has_no_collision)? best_path: -1, current_path);
	fflush(stdout);
	if (paths[best_path].path_has_no_collision)
		return (best_path);
	else
		return (-1);
}


bool
free_to_run(int who_set_the_goal_v)
{
	if ((who_set_the_goal_v == MOVING_OBSTACLE) || (who_set_the_goal_v == OBSTACLE))
		return (false);
	else
		return (true);
}


void
update_current_path(int best_path, int number_of_paths, carmen_frenet_path_planner_set_of_paths *current_set_of_paths, double *path_temporal_value,
		int who_set_the_goal_v, double &last_update_timestamp, double timestamp)
{
	bool update_path;
	if (use_unity_simulator)
		update_path = !free_to_run(who_set_the_goal_v) && (best_path != -1);
	else
		update_path = (best_path != current_set_of_paths->selected_path) && (best_path != -1);

	if (update_path)
	{
		for (int i = 0; i < number_of_paths; i++)
			path_temporal_value[i] = 0.0;
		path_temporal_value[best_path] = 100.0;

		last_update_timestamp = timestamp;

		publish_new_best_path(best_path, timestamp);
	}
}


double *
init_path_temporal_value(int number_of_paths)
{
	static double *path_temporal_value = NULL;
	if (!path_temporal_value)
	{
		path_temporal_value = (double*) (malloc(number_of_paths * sizeof(double)));
		for (int i = 0; i < number_of_paths; i++)
			path_temporal_value[i] = 0.0;
	}
	return (path_temporal_value);
}


void
set_optimum_path(carmen_frenet_path_planner_set_of_paths *current_set_of_paths,
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, int who_set_the_goal_v, double timestamp)
{
	if (!current_set_of_paths)
		return;

	double time_horizon = frenet_path_planner_time_horizon;
	double delta_t = frenet_path_planner_delta_t;
	int num_samples = (int) (time_horizon / delta_t);
	int number_of_paths = current_set_of_paths->set_of_paths_size / current_set_of_paths->number_of_poses;

	double *path_temporal_value = init_path_temporal_value(number_of_paths);
	static double last_update_timestamp = timestamp;

	int num_moving_objects;
	if (current_moving_objects != NULL)
		num_moving_objects = current_moving_objects->num_point_clouds;
	else
		num_moving_objects = 0;

	vector<vector <moving_object_pose_info_t> > moving_objects_data(num_samples, vector <moving_object_pose_info_t>(num_moving_objects));
	compute_moving_objects_future_poses(moving_objects_data, delta_t, num_samples, current_set_of_paths);

//	virtual_laser_message.num_positions = 0;

	vector<path_collision_info_t> paths = get_paths_collision_info(current_set_of_paths, moving_objects_data,
			current_robot_pose_v_and_phi, delta_t, num_samples);

	int best_path = get_path_better_than_the_current_path(paths, current_set_of_paths, path_temporal_value);
	update_current_path(best_path, number_of_paths, current_set_of_paths, path_temporal_value,
			who_set_the_goal_v, last_update_timestamp, timestamp);

	for (int i = 0; i < number_of_paths; i++)
		path_temporal_value[i] *= exp(-0.05 * (timestamp - last_update_timestamp));

//	publish_simulated_objects();
}
