#include <vector>
#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/rddf_messages.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/collision_detection.h>
#include "behavior_selector.h"

using namespace std;

typedef struct
{
	carmen_robot_and_trailer_pose_t car_pose;
	int index_in_path;
} car_poses_and_indexes_in_path_t;


extern carmen_frenet_path_planner_set_of_paths *current_set_of_paths;

extern carmen_obstacle_distance_mapper_map_message distance_map_free_of_moving_objects;

extern int frenet_path_planner_num_paths;
extern double frenet_path_planner_paths_displacement;
extern double frenet_path_planner_time_horizon;
extern double frenet_path_planner_delta_t;

extern int use_unity_simulator;
extern double in_lane_longitudinal_safety_margin;
extern double in_lane_longitudinal_safety_margin_with_v_multiplier;
extern double in_path_longitudinal_safety_margin;
extern double in_path_longitudinal_safety_margin_with_v_multiplier;

extern carmen_mapper_virtual_laser_message virtual_laser_message;
//extern int add_virtual_laser_points;

extern int behavior_selector_use_symotha;

extern double distance_car_pose_car_front;

bool print_path = false;


void
get_s_and_d_values(carmen_robot_and_trailer_traj_point_t *poses, int nearest_pose, t_point_cloud_struct *moving_object,
		double &estimated_s_v, double &estimated_d_t_0, double &estimated_d_v)
{
	double distance_moving_object_to_nearest_path_pose = DIST2D(poses[nearest_pose], moving_object->object_pose);
	double angle = ANGLE2D(poses[nearest_pose], moving_object->object_pose);
	double angle_with_respect_to_nearest_path_pose = carmen_normalize_theta(angle - poses[nearest_pose].theta);

	estimated_d_t_0 = distance_moving_object_to_nearest_path_pose * sin(angle_with_respect_to_nearest_path_pose);

	estimated_s_v = moving_object->linear_velocity;
	estimated_d_v = moving_object->lateral_velocity;
}


double
get_s_displacement_for_a_given_sample(int s, double v0, double a, double delta_t)
{
	double t = (double) s * delta_t;

//	double v = v0 + a * t;
	if (v0 > get_max_v())
		v0 = get_max_v();
	if (v0 < 0.0)
		v0 = 0.0;

	double s_displacement = v0 * t + 0.5 * a * t * t;
	if (s_displacement < 0.0)
		s_displacement = 0.0;

	return (s_displacement);
}


inline bool
car_fits_in_path(int index, carmen_robot_and_trailer_traj_point_t *path, int size, double distance_added_due_to_map_update_delay,
		double imperfection_of_robot_modeling_via_circles)
{
	double s_range = 0.0;
	for (int i = index; i < size - 1; i++)
		s_range += DIST2D(path[i], path[i + 1]);

	if (s_range > (distance_car_pose_car_front + imperfection_of_robot_modeling_via_circles + distance_added_due_to_map_update_delay))
		return (true);
	else
		return (false);
}


int
last_path_pose_that_the_car_can_occupy(carmen_robot_and_trailer_traj_point_t *path, int size, double car_v)
{
	double distance_added_due_to_map_update_delay = fabs(current_set_of_paths->timestamp - distance_map_free_of_moving_objects.timestamp) * fabs(car_v);
	double imperfection_of_robot_modeling_via_circles = get_robot_config()->width / 2.0;
	int path_pose = -1;
	for (int i = size - 2; i >= 0; i--)
	{
		if (car_fits_in_path(i, path, size, distance_added_due_to_map_update_delay, imperfection_of_robot_modeling_via_circles))
		{
			path_pose = i;
			break;
		}
	}

	return (path_pose);
}


carmen_robot_and_trailer_pose_t
get_car_pose_at_s_displacement(double s_displacement, carmen_robot_and_trailer_traj_point_t *path, int &i, int last_path_pose)
{
	double s_range = 0.0;
	for (i = 0; (s_range < s_displacement) && (i < (last_path_pose - 1)); i++)
		s_range += DIST2D(path[i], path[i + 1]);

	carmen_robot_and_trailer_pose_t car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&(path[i]),
			s_displacement - s_range);

	return (car_pose);
}


void
collision_s_distance_to_static_object(path_collision_info_t &path_collision_info,
		carmen_frenet_path_planner_set_of_paths *current_set_of_paths, int path_id,
		double delta_t, int num_samples, carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi,
		double robot_acc, double distance_to_goal)
{
	carmen_robot_and_trailer_traj_point_t *path = &(current_set_of_paths->set_of_paths[path_id * current_set_of_paths->number_of_poses]);

	double min_collision_s_distance = (double) num_samples * delta_t;
	int index_in_path = 0;
	int last_path_pose = last_path_pose_that_the_car_can_occupy(path, current_set_of_paths->number_of_poses, current_robot_pose_v_and_phi.v);
	double max_distance_to_static_object_considered =
			1.15 * ((distance_between_waypoints_and_goals() + distance_to_goal) / 2.0) + distance_car_pose_car_front;
	if (last_path_pose != -1)
	{
		double v = get_max_v();
		int last_s = 0;
		vector<car_poses_and_indexes_in_path_t> car_poses_and_indexes_in_path;
		double s_range = 0.0;
		for (int s = 0; s < num_samples; s++)
		{
			double s_displacement = get_s_displacement_for_a_given_sample(s, v, robot_acc, delta_t);
			if (s_displacement > max_distance_to_static_object_considered)
				break;

			///////////////////////////////
			for (; (s_range < s_displacement) && (index_in_path < (last_path_pose - 1)); index_in_path++)
				s_range += DIST2D(path[index_in_path], path[index_in_path + 1]);

			carmen_robot_and_trailer_pose_t car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&(path[index_in_path]),
					s_displacement - s_range);
			///////////////////////////////

			if (index_in_path >= last_path_pose - 1)
				break;
			car_poses_and_indexes_in_path_t cp_and_i = {car_pose, index_in_path};
			car_poses_and_indexes_in_path.push_back(cp_and_i);
			last_s = s;
		}
		
		for (int s = 0; s < last_s; s++)
		{
			carmen_robot_and_trailer_pose_t car_pose = car_poses_and_indexes_in_path[s].car_pose;
			carmen_robot_and_trailer_traj_point_t cp = {car_pose.x, car_pose.y, car_pose.theta, car_pose.beta, 0.0, 0.0};
			index_in_path = car_poses_and_indexes_in_path[s].index_in_path;
			// Incluir um loop para testar colisão com objetos móveis lentos o suficiente, como em collision_s_distance_to_moving_object(), só que com o teste ao contrário.
			if (trajectory_pose_hit_obstacle(cp, get_robot_config()->model_predictive_planner_obstacles_safe_distance,
					&distance_map_free_of_moving_objects, NULL) == 1)
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
	}

	path_collision_info.s_distance_without_collision_with_static_object = min_collision_s_distance;
	if (min_collision_s_distance == (double) num_samples * delta_t)
		path_collision_info.static_object_pose_index = current_set_of_paths->number_of_poses;
	else
		path_collision_info.static_object_pose_index = index_in_path;
}


double
update_path_collision_info(path_collision_info_t &path_collision_info, double min_collision_s_distance,
		int s, double delta_t, int mo, int index_in_path, vector<vector<moving_object_pose_info_t> > moving_objects_poses,
		carmen_moving_objects_point_clouds_message *current_moving_objects)
{
	if (((double) (s) * delta_t) < min_collision_s_distance)
	{
		min_collision_s_distance = (double) (s) * delta_t;
		path_collision_info.s_distance_without_collision_with_moving_object = min_collision_s_distance;
		path_collision_info.possible_collision_mo_pose_index = index_in_path;
		if (current_moving_objects->point_clouds[mo].in_front)
			path_collision_info.mo_in_front = true; // Posível colisão com objeto móvel à frente.

		path_collision_info.moving_object_id = current_moving_objects->point_clouds[mo].num_associated;
		path_collision_info.possible_collision_mo_sv = moving_objects_poses[0][mo].sv;
		path_collision_info.possible_collision_mo_dv = moving_objects_poses[0][mo].dv;
	}
	else if (((double) (s) * delta_t) == min_collision_s_distance) // Outro objeto móvel à mesma distância. Troca para este se ele oferecer mais risco
	{
		if (path_collision_info.mo_in_front == false) // Se o anterior não estava na frente, já troca por este
		{
			if (current_moving_objects->point_clouds[mo].in_front)
				path_collision_info.mo_in_front = true; // Posível colisão com objeto móvel à frente.

			path_collision_info.moving_object_id = current_moving_objects->point_clouds[mo].num_associated;
			path_collision_info.possible_collision_mo_sv = moving_objects_poses[0][mo].sv;
			path_collision_info.possible_collision_mo_dv = moving_objects_poses[0][mo].dv;
		}
		else if (current_moving_objects->point_clouds[mo].in_front && (path_collision_info.possible_collision_mo_sv < moving_objects_poses[0][mo].sv)) // Se este estiver mais rápido, troca por este
		{
			path_collision_info.moving_object_id = current_moving_objects->point_clouds[mo].num_associated;
			path_collision_info.possible_collision_mo_sv = moving_objects_poses[0][mo].sv;
			path_collision_info.possible_collision_mo_dv = moving_objects_poses[0][mo].dv;
		}
	}

	return (min_collision_s_distance);
}


void
collision_s_distance_to_moving_object(path_collision_info_t &path_collision_info,
		carmen_frenet_path_planner_set_of_paths *current_set_of_paths,
		carmen_moving_objects_point_clouds_message *current_moving_objects, int path_id,
		vector<vector <moving_object_pose_info_t> > moving_objects_poses, double delta_t, int num_samples,
		carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi, double robot_acc)
{
	path_collision_info.mo_in_front = false;	// Estar atrás é bom. Não precisamos cuidar. Colisão por trás é culpa do moving object.
	path_collision_info.moving_object_id = -1;

	if (!current_moving_objects || (current_moving_objects->num_point_clouds == 0))
	{
		path_collision_info.s_distance_without_collision_with_moving_object = (double) num_samples * delta_t;
		path_collision_info.possible_collision_mo_pose_index = current_set_of_paths->number_of_poses;
		path_collision_info.possible_collision_mo_sv = current_robot_pose_v_and_phi.v;
		path_collision_info.possible_collision_mo_dv = 0.0;

		return;
	}

	carmen_robot_and_trailer_traj_point_t *path = &(current_set_of_paths->set_of_paths[path_id * current_set_of_paths->number_of_poses]);

	double min_collision_s_distance = (double) num_samples * delta_t;
	int index_in_path = current_set_of_paths->number_of_poses;
	int last_path_pose = current_set_of_paths->number_of_poses;
	for (int s = 0; s < num_samples; s++)
	{
		double s_displacement = get_s_displacement_for_a_given_sample(s, current_robot_pose_v_and_phi.v, robot_acc, delta_t);
		carmen_robot_and_trailer_pose_t car_pose = get_car_pose_at_s_displacement(s_displacement, path, index_in_path, last_path_pose);
		if (index_in_path >= last_path_pose - 1)
			break;

 		for (int mo = 0; mo < current_moving_objects->num_point_clouds; mo++)
		{
 			if (current_moving_objects->point_clouds[mo].index_in_poses_ahead != -1)
 				continue; // ignora moving objects que estao em poses_ahead

 			double longitudinal_safety_margin = in_path_longitudinal_safety_margin + in_path_longitudinal_safety_margin_with_v_multiplier * fabs(current_moving_objects->point_clouds[mo].linear_velocity);
			double lateral_safety_margin = get_robot_config()->obstacle_avoider_obstacles_safe_distance;// +
			carmen_point_t moving_objects_pose = {moving_objects_poses[s][mo].pose.x, moving_objects_poses[s][mo].pose.y, moving_objects_poses[s][mo].pose.theta};

			if (carmen_obstacle_avoider_car_collides_with_moving_object(car_pose, moving_objects_pose,
					&(current_moving_objects->point_clouds[mo]), longitudinal_safety_margin, lateral_safety_margin))//,
//					i, moving_objects_poses[s][i].s, moving_objects_poses[s][i].d))
			{
				if (path_id == current_set_of_paths->selected_path && current_moving_objects->point_clouds[mo].in_front)
				{
					virtual_laser_message.positions[virtual_laser_message.num_positions].x = car_pose.x;
					virtual_laser_message.positions[virtual_laser_message.num_positions].y = car_pose.y;
					virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_ORANGE;
					virtual_laser_message.num_positions++;
				}

				min_collision_s_distance = update_path_collision_info(path_collision_info, min_collision_s_distance,
						s, delta_t, mo, index_in_path, moving_objects_poses, current_moving_objects);
			}
		}
	}

	if (min_collision_s_distance == (double) num_samples * delta_t)
	{
		path_collision_info.s_distance_without_collision_with_moving_object = min_collision_s_distance;
		path_collision_info.possible_collision_mo_pose_index = current_set_of_paths->number_of_poses;
	}
}


carmen_robot_and_trailer_traj_point_t *
get_rddf_displaced(carmen_robot_and_trailer_traj_point_t *rddf_poses_ahead, int number_of_poses, double path_displacement)
{
	carmen_robot_and_trailer_traj_point_t *displaced_rddf = (carmen_robot_and_trailer_traj_point_t *) malloc(number_of_poses * sizeof(carmen_robot_and_trailer_traj_point_t));
	for (int i = 0; i < number_of_poses; i++)
	{
		displaced_rddf[i] = rddf_poses_ahead[i]; // para copiar os outros campos de carmen_robot_and_trailer_traj_point_t
		displaced_rddf[i].x = rddf_poses_ahead[i].x + path_displacement * cos(rddf_poses_ahead[i].theta + (M_PI / 2.0));
		displaced_rddf[i].y = rddf_poses_ahead[i].y + path_displacement * sin(rddf_poses_ahead[i].theta + (M_PI / 2.0));
	}

	return (displaced_rddf);
}


void
collision_s_distance_to_moving_object_in_parallel_lane(path_collision_info_t &path_collision_info,
		carmen_frenet_path_planner_set_of_paths *current_set_of_paths,
		carmen_moving_objects_point_clouds_message *current_moving_objects, int path_id,
		vector<vector <moving_object_pose_info_t> > moving_objects_poses, double delta_t, int num_samples,
		carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi, double robot_acc)
{
	if (1)//current_moving_objects->num_point_clouds == 0)
	{
		path_collision_info.s_distance_to_moving_object_in_parallel_lane = (double) num_samples * delta_t;
		path_collision_info.possible_collision_mo_in_parallel_lane_pose_index = current_set_of_paths->number_of_poses;
		path_collision_info.possible_collision_mo_in_parallel_lane_sv = current_robot_pose_v_and_phi.v;
		path_collision_info.possible_collision_mo_in_parallel_lane_dv = 0.0;
		return;
	}

	double disp = frenet_path_planner_paths_displacement;
	double path_displacement = disp * ((frenet_path_planner_num_paths / 2) - path_id);
	carmen_robot_and_trailer_traj_point_t *path = get_rddf_displaced(current_set_of_paths->rddf_poses_ahead, current_set_of_paths->number_of_poses, path_displacement);

	double min_s_distance = (double) num_samples * delta_t;
	int index_in_path;
	int mo = 0;
	int last_path_pose = current_set_of_paths->number_of_poses;
	for (int s = 0; s < num_samples; s++)
	{
		double s_displacement = get_s_displacement_for_a_given_sample(s, current_robot_pose_v_and_phi.v, robot_acc, delta_t);
		carmen_robot_and_trailer_pose_t car_pose = get_car_pose_at_s_displacement(s_displacement, path, index_in_path, last_path_pose);
		if (index_in_path >= last_path_pose - 1)
			break;

 		for (mo = 0; mo < current_moving_objects->num_point_clouds; mo++)
		{
// 			if (fabs(current_moving_objects->point_clouds[i].linear_velocity) > fabs(current_robot_pose_v_and_phi.v) / 5.0)
// 				continue;

 			double longitudinal_safety_margin = in_lane_longitudinal_safety_margin + in_lane_longitudinal_safety_margin_with_v_multiplier * fabs(current_moving_objects->point_clouds[mo].linear_velocity);
			double lateral_safety_margin = get_robot_config()->obstacle_avoider_obstacles_safe_distance;
			carmen_point_t moving_objects_pose = {moving_objects_poses[0][mo].pose.x, moving_objects_poses[0][mo].pose.y, moving_objects_poses[0][mo].pose.theta};
			if (carmen_obstacle_avoider_car_collides_with_moving_object(car_pose, moving_objects_pose,
					&(current_moving_objects->point_clouds[mo]), longitudinal_safety_margin, lateral_safety_margin))//, 0, 0.0, 0.0))
			{
				if (path_id == current_set_of_paths->selected_path)
				{
					virtual_laser_message.positions[virtual_laser_message.num_positions].x = car_pose.x;
					virtual_laser_message.positions[virtual_laser_message.num_positions].y = car_pose.y;
					virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_GREEN;
					virtual_laser_message.num_positions++;
				}
				if (((double) s * delta_t) < min_s_distance)
				{
					min_s_distance = (double) s * delta_t;
					path_collision_info.s_distance_to_moving_object_in_parallel_lane = min_s_distance;
					path_collision_info.possible_collision_mo_in_parallel_lane_pose_index = index_in_path;
					path_collision_info.possible_collision_mo_in_parallel_lane_sv = moving_objects_poses[0][mo].sv;
					path_collision_info.possible_collision_mo_in_parallel_lane_dv = moving_objects_poses[0][mo].dv;
				}

				break;
			}
		}
	}

	if (min_s_distance == (double) num_samples * delta_t)
	{
		path_collision_info.s_distance_to_moving_object_in_parallel_lane = min_s_distance;
		path_collision_info.possible_collision_mo_in_parallel_lane_pose_index = current_set_of_paths->number_of_poses;
	}

	free(path);
}


int
find_nearest_pose(carmen_robot_and_trailer_traj_point_t *path, int path_size, carmen_vector_3D_t object_pose)
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


double
find_distance_to_near_pose(carmen_robot_and_trailer_traj_point_t *path, int path_size, carmen_point_t object_pose)
{
	int i = 0;
	double min_distance = DIST2D(path[i], object_pose);
	for (i = 1; i < path_size; i++)
	{
		double distance = DIST2D(path[i], object_pose);
		if (distance < min_distance)
			min_distance = distance;
	}

	return (min_distance);
}


carmen_robot_and_trailer_traj_point_t *
get_moving_object_lane(int &lane_size, double &d_distance_to_lane, double &s_distance_to_lane,
		t_point_cloud_struct *moving_object, carmen_frenet_path_planner_set_of_paths *set_of_paths)
{
	int lane_id = moving_object->lane_id;
	bool not_found = true;
	int j;
	for (j = 0; j < set_of_paths->number_of_nearby_lanes; j++)
	{
		if (lane_id == set_of_paths->nearby_lanes_ids[j])
		{
			not_found = false;
			break;
		}
	}

	if (not_found)
		return (NULL);

	lane_size = set_of_paths->nearby_lanes_sizes[j];
	if (lane_size <= moving_object->lane_index)
		return (NULL);

	carmen_robot_and_trailer_traj_point_t *moving_object_lane = &(set_of_paths->nearby_lanes[set_of_paths->nearby_lanes_indexes[j]]);

	int status;
	int mo_index = moving_object->lane_index;
	carmen_robot_and_trailer_traj_point_t mo_pose = {moving_object->object_pose.x, moving_object->object_pose.y, moving_object->orientation, 0.0, 0.0, 0.0};
	carmen_robot_and_trailer_traj_point_t mo_pose_projection_into_lane;
	if (mo_index < lane_size - 1)
		mo_pose_projection_into_lane = carmen_get_point_nearest_to_trajectory(&status, moving_object_lane[mo_index], moving_object_lane[mo_index + 1], mo_pose, 0.1);
	else
		mo_pose_projection_into_lane = moving_object_lane[mo_index];

	if (status != POINT_BEFORE_SEGMENT)
		s_distance_to_lane = DIST2D(mo_pose_projection_into_lane, moving_object_lane[mo_index]);
	else
		s_distance_to_lane = -DIST2D(mo_pose_projection_into_lane, moving_object_lane[mo_index]);

	double angle_that_tells_the_size_of_the_mo = ANGLE2D(mo_pose_projection_into_lane, mo_pose) - moving_object_lane[mo_index].theta;
	d_distance_to_lane = DIST2D(mo_pose, mo_pose_projection_into_lane) * sin(angle_that_tells_the_size_of_the_mo);

	moving_object_lane = &(moving_object_lane[mo_index]);

	return (moving_object_lane);
}


carmen_robot_and_trailer_pose_t
get_moving_obeject_pose_at_s_and_d_displacements(double s_displacement, double d_displacement,
		double d_distance_to_lane, double s_distance_to_lane, carmen_robot_and_trailer_traj_point_t *movin_object_lane,
		int lane_size, int lane_index)
{
	double s_range = s_distance_to_lane;
	int i;
	carmen_robot_and_trailer_pose_t car_pose;
	if (s_displacement >= 0.0)
	{
		for (i = 0; (s_range < s_displacement) && (i < (lane_size - lane_index - 1)); i++)
			s_range += DIST2D(movin_object_lane[i], movin_object_lane[i + 1]);

		car_pose = carmen_collision_detection_displace_car_on_its_frenet_frame(&(movin_object_lane[i]),
				s_displacement - s_range, d_displacement + d_distance_to_lane);
	}
	else
	{
		for (i = 0; (s_range < -s_displacement) && (lane_index + i > 0); i--)
			s_range += DIST2D(movin_object_lane[i], movin_object_lane[i - 1]);

		car_pose = carmen_collision_detection_displace_car_on_its_frenet_frame(&(movin_object_lane[i]),
				s_displacement + s_range, d_displacement + d_distance_to_lane);
	}

	return (car_pose);
}


moving_object_pose_info_t
simulate_moving_object_movement(t_point_cloud_struct *moving_object, carmen_robot_and_trailer_traj_point_t *movin_object_lane,
		int lane_size, double d_distance_to_lane, double s_distance_to_lane, double t)
{
	carmen_robot_and_trailer_traj_point_t movin_object_pose = {moving_object->object_pose.x, moving_object->object_pose.y,
			moving_object->orientation, 0.0, moving_object->linear_velocity, 0.0};

	double s_displacement = moving_object->linear_velocity * t;
	double d_displacement = moving_object->lateral_velocity * t;
	carmen_robot_and_trailer_pose_t moving_object_pose_at_t;
	if (movin_object_lane)
		moving_object_pose_at_t = get_moving_obeject_pose_at_s_and_d_displacements(s_displacement, d_displacement,
				d_distance_to_lane, s_distance_to_lane, movin_object_lane, lane_size, moving_object->lane_index);
	else
		moving_object_pose_at_t = carmen_collision_detection_displace_car_on_its_frenet_frame(&movin_object_pose, s_displacement, d_displacement);

	moving_object_pose_info_t moving_object_pose_info;
	moving_object_pose_info.pose = {moving_object_pose_at_t.x, moving_object_pose_at_t.y, moving_object_pose_at_t.theta};
	moving_object_pose_info.s = s_displacement;
	moving_object_pose_info.d = d_displacement;
	moving_object_pose_info.sv = moving_object->linear_velocity;
	moving_object_pose_info.dv = moving_object->lateral_velocity;

	return (moving_object_pose_info);
}


void
compute_moving_objects_future_poses(vector<vector <moving_object_pose_info_t> > &moving_objects_poses,
		carmen_frenet_path_planner_set_of_paths *current_set_of_paths, double delta_t, int num_poses,
		carmen_moving_objects_point_clouds_message *current_moving_objects)
{
	if (!current_moving_objects)
		return;

	for (int i = 0; i < current_moving_objects->num_point_clouds; i++)
	{
		int lane_size;
		double d_distance_to_lane, s_distance_to_lane;
		t_point_cloud_struct *moving_object = &(current_moving_objects->point_clouds[i]);
		carmen_robot_and_trailer_traj_point_t *movin_object_lane = get_moving_object_lane(lane_size, d_distance_to_lane, s_distance_to_lane,
				moving_object, current_set_of_paths);
		for (int s = 0; s < num_poses; s++)
		{
			moving_object_pose_info_t moving_object_pose_info = simulate_moving_object_movement(
					moving_object, movin_object_lane, lane_size, d_distance_to_lane, s_distance_to_lane,
					delta_t * (double) s);

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
		carmen_moving_objects_point_clouds_message *current_moving_objects,
		vector<vector <moving_object_pose_info_t> > moving_objects_data, carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi,
		double robot_acc, double delta_t, int num_samples)
{
	int number_of_paths = current_set_of_paths->set_of_paths_size / current_set_of_paths->number_of_poses;
	vector<path_collision_info_t> path_collision_info_v;

	int last_goal_list_size;
	double distance_to_goal;
	carmen_robot_and_trailer_traj_point_t *goal_list = behavior_selector_get_last_goal_list(last_goal_list_size);
	if (last_goal_list_size == 0)
		distance_to_goal = 0.0;
	else
		distance_to_goal = DIST2D(current_robot_pose_v_and_phi, goal_list[0]);

	for (int path_id = 0; path_id < number_of_paths; path_id++)
	{
		path_collision_info_t path_collision_info;

		path_collision_info.valid = 1;
		path_collision_info.path_id = path_id;
		collision_s_distance_to_moving_object(path_collision_info, current_set_of_paths,
				current_moving_objects, path_id, moving_objects_data, delta_t, num_samples, current_robot_pose_v_and_phi, robot_acc);

		collision_s_distance_to_moving_object_in_parallel_lane(path_collision_info, current_set_of_paths,
				current_moving_objects, path_id, moving_objects_data, delta_t, num_samples, current_robot_pose_v_and_phi, 0.0);

		collision_s_distance_to_static_object(path_collision_info, current_set_of_paths,
				path_id, delta_t, num_samples, current_robot_pose_v_and_phi, 0.0, distance_to_goal);

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
			lateral_value = 75.0 * (paths.size() - abs((int) i - ((int) paths.size() / 2)));
			if (behavior_selector_use_symotha)
				longitudinal_value = 1000.0 * paths[i].s_distance_without_collision_with_static_object;
			else
				longitudinal_value = 1000.0 * paths[i].s_distance_without_collision_with_static_object;// +
//									 100.0  * paths[i].s_distance_without_collision_with_moving_object +
//									 100.0  * paths[i].s_distance_to_moving_object_in_parallel_lane;

		}

		double path_value = longitudinal_value + lateral_value + path_temporal_value[i];
		if (path_value > best_path_value)
		{
			best_path_value = path_value;
			best_path = i;
		}

		if (print_path)
			printf("\npath %d, v %5.1lf, lv %5.1lf (%5.3lf, %5.3lf, %5.3lf), lat_value %3.1lf, t_value %5.1lf",
				i, path_value, longitudinal_value,
				paths[i].s_distance_without_collision_with_static_object,
				paths[i].s_distance_without_collision_with_moving_object,
				paths[i].s_distance_to_moving_object_in_parallel_lane,
				lateral_value, path_temporal_value[i]);
	}

	if (print_path)
	{
		printf("  -> best_path %d (%d) %lf\n", (paths[best_path].path_has_no_collision)? best_path: -1, current_path, current_set_of_paths->timestamp);
		fflush(stdout);
	}

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
		int who_set_the_goal_v, double &last_update_timestamp, carmen_behavior_selector_state_message behavior_selector_state_message,
		double timestamp)
{
	bool update_path;
	if (use_unity_simulator)
		update_path = !free_to_run(who_set_the_goal_v) && (best_path != -1);
	else
		update_path = (best_path != current_set_of_paths->selected_path) && (best_path != -1);

	if (behavior_selector_state_message.low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_WITHIN_NARROW_PASSAGE)
	{
		update_path = true;
		best_path = number_of_paths / 2;
		current_set_of_paths->selected_path = best_path;
	}

	if (update_path)
	{
		for (int i = 0; i < number_of_paths; i++)
			path_temporal_value[i] = 0.0;
		path_temporal_value[best_path] = 350.0;

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
print_mo(carmen_moving_objects_point_clouds_message *current_moving_objects, double robot_acc)
{
	if (!current_moving_objects)
		return;

	printf("%lf\n", current_moving_objects->timestamp);

	for (int j = 0; j < current_moving_objects->num_point_clouds; j++)
	{
		printf("id %d, in_front %d, lane_id %d, lane_index %d, index_in_pa %d, num_samples %d, l %0.2lf, w %0.2lf, vs_s %0.2lf, vd_s %0.2lf   -   points %d, r_acc %lf\n",
				current_moving_objects->point_clouds[j].num_associated,
				current_moving_objects->point_clouds[j].in_front,
				current_moving_objects->point_clouds[j].lane_id,
				current_moving_objects->point_clouds[j].lane_index,
				current_moving_objects->point_clouds[j].index_in_poses_ahead,
				current_moving_objects->point_clouds[j].num_valid_samples,
				current_moving_objects->point_clouds[j].length,
				current_moving_objects->point_clouds[j].width,
				current_moving_objects->point_clouds[j].linear_velocity,
				current_moving_objects->point_clouds[j].lateral_velocity,
				current_moving_objects->point_clouds[j].point_size,
				robot_acc);
	}
}


double
get_robot_acc_from_pose_and_goal(double &requested_acc, carmen_robot_and_trailer_traj_point_t *pose,
		carmen_behavior_selector_state_message behavior_selector_state_message)
{
	int last_goal_list_size;
	carmen_robot_and_trailer_traj_point_t *goal_list = behavior_selector_get_last_goal_list(last_goal_list_size);
	if (last_goal_list_size == 0)
	{
		requested_acc = 0.0;
		return (0.0);
	}

	carmen_robot_and_trailer_traj_point_t *goal;
	goal = &(goal_list[0]);
	double S = DIST2D_P(pose, goal);
	if (S < 0.1)
		requested_acc = 0.0;
	else
		requested_acc = ((goal->v * goal->v) - (pose->v * pose->v)) / (2.0 * S);
	if (requested_acc > get_robot_config()->maximum_acceleration_forward)
		requested_acc = get_robot_config()->maximum_acceleration_forward;

	if ((last_goal_list_size > 1) &&
		((behavior_selector_get_last_goal_type() == MOVING_OBSTACLE_GOAL3) ||
		 (behavior_selector_state_message.low_level_state == Stopping_At_Yield) ||
		 (behavior_selector_state_message.low_level_state == Stopped_At_Yield_S0) ||
		 (behavior_selector_state_message.low_level_state == Stopped_At_Yield_S1) ||
		 (behavior_selector_state_message.low_level_state == Stopping_At_Stop_Sign) ||
		 (behavior_selector_state_message.low_level_state == Stopped_At_Stop_Sign_S0) ||
		 (behavior_selector_state_message.low_level_state == Stopped_At_Stop_Sign_S1)))
		goal = &(goal_list[1]);
	else
		goal = &(goal_list[0]);

	S = DIST2D_P(pose, goal);
	if (S < 0.1)
		return (0.0);

	double acc = ((goal->v * goal->v) - (pose->v * pose->v)) / (2.0 * S);
	if (acc > get_robot_config()->maximum_acceleration_forward)
		acc = get_robot_config()->maximum_acceleration_forward;

//	printf("last_goal_list_size %d, %s, goal_list[0].v %lf, goal_list[1].v %lf, pose.v %lf, S %lf, goal_type %d\n",
//			last_goal_list_size,
//			get_low_level_state_name(behavior_selector_state_message.low_level_state),
//			goal_list[0].v, goal_list[1].v, pose->v, S, behavior_selector_get_last_goal_type());

	return (acc);
}

FILE *gnuplot_pipe;

void
plot(double used_acc, double requested_acc, double v)
{
	static bool first_time = true;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipe, "set yrange [-3:6]\n");

		FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");
		fclose(gnuplot_data_file);
	}

	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "a");
	fprintf(gnuplot_data_file, "%lf %lf %lf\n", used_acc, requested_acc, v);
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' u 1 w l lt rgb 'blue'    t 'used_acc',"
			"'./gnuplot_data.txt' u 2 w l lt rgb 'red'    t 'requested_acc',"
			"'./gnuplot_data.txt' u 3 w l lt rgb 'green'  t 'v'\n");

	fflush(gnuplot_pipe);
}


vector<path_collision_info_t>
set_optimum_path(carmen_frenet_path_planner_set_of_paths *current_set_of_paths,
		carmen_moving_objects_point_clouds_message *current_moving_objects,
		carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi,
		int who_set_the_goal_v, carmen_behavior_selector_state_message behavior_selector_state_message, double timestamp)
{
	if (!current_set_of_paths || behavior_selector_use_symotha)
	{
		vector<path_collision_info_t> no_collision_info = {};
		return (no_collision_info);
	}

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
	compute_moving_objects_future_poses(moving_objects_data, current_set_of_paths, delta_t, num_samples, current_moving_objects);

//	virtual_laser_message.num_positions = 0;

	static double robot_acc = 0.0;
	double requested_acc;
	double instantaneous_acc = get_robot_acc_from_pose_and_goal(requested_acc, &current_robot_pose_v_and_phi, behavior_selector_state_message);
//	if (instantaneous_acc > robot_acc)
//		robot_acc = instantaneous_acc;
//	else
		robot_acc += 0.1 * (instantaneous_acc - robot_acc);

	// plot(robot_acc, requested_acc, current_robot_pose_v_and_phi.v);

	vector<path_collision_info_t> paths_collision_info = get_paths_collision_info(current_set_of_paths, current_moving_objects,
			moving_objects_data, current_robot_pose_v_and_phi, robot_acc, delta_t, num_samples);

	int best_path = get_path_better_than_the_current_path(paths_collision_info, current_set_of_paths, path_temporal_value);
	update_current_path(best_path, number_of_paths, current_set_of_paths, path_temporal_value,
			who_set_the_goal_v, last_update_timestamp, behavior_selector_state_message, timestamp);

	for (int i = 0; i < number_of_paths; i++)
		path_temporal_value[i] *= exp(-0.005 * (timestamp - last_update_timestamp));

//	print_mo(current_moving_objects, robot_acc);
//	print_path = true;

	return (paths_collision_info);
}