#include <termios.h>
#include <unistd.h>
#include <list>
#include <carmen/global_planning_interface.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>
#include <carmen/task_manager_interface.h>

#include <carmen/offroad_planner_interface.h>
#include "nlp_mat_planner.h"
#include "nlp_mat_planner_path_planner_astar.h"

// #define SO_GOAL

#define LANE_LEFT_WIDTH (4.0 / 2.0)			 // 2.4
#define LANE_RIGHT_WIDTH ((4.0 / 2.0) - 0.5) // 2.4

#define STDIN_FILENO 0
#define NB_ENABLE 1
#define NB_DISABLE 0

#define MAX_U1 (M_PI - 0.01)
#define MAX_U2 (2.0 * M_PI - 0.01)
#define MAX_K (1.0 - 0.01)

double g_u1 = MAX_U1 / 2.0;
double g_u2 = MAX_U2 / 2.0;
double g_k = MAX_K / 2.0;
int g_c = 1;
double g_c2 = 1.0;

extern int grid_state_map_x_size;
extern int grid_state_map_y_size;

static int x_size, y_size;

carmen_robot_ackerman_config_t robot_config;
carmen_robot_ackerman_config_t GlobalState::robot_config;

carmen_semi_trailers_config_t semi_trailer_config;
carmen_semi_trailers_config_t GlobalState::semi_trailer_config;

carmen_path_planner_astar_t astar_config;

static carmen_navigator_config_t nav_config;

static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;
static carmen_obstacle_distance_mapper_map_message distance_map_struct;

static carmen_obstacle_distance_mapper_map_message print_map_struct;

carmen_obstacle_distance_mapper_map_message *GlobalState::distance_map = NULL;

carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map = NULL;

carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map_print_map = NULL;

double path_smoothing_obstacles_safe_distance;

carmen_route_planner_road_network_message *road_network_message = NULL;

carmen_behavior_selector_task_t *behavior_selector_task = NULL;

carmen_rddf_end_point_message *final_goal = NULL;

carmen_robot_and_trailers_traj_point_t robot_position;

double *goal_distance_map = NULL;

double *nonholonomic_heuristic_cost_map;
int nonholonomic_heuristic_cost_map_loaded = 0;
int nonholonomic_heuristic_cost_map_trailer_loaded = 0;

carmen_rddf_annotation_message *last_rddf_annotation_message = NULL;

double original_model_predictive_planner_obstacles_safe_distance;

int number_of_trailers_for_path_planning;

char *cache_filename = (char *)"offroad_planner_cache.txt";
int cache_writing = 1;
int time_mult = 0;
int param_ignore_check_ipc = 0;

static int argc_global;
static char **argv_global;

double GlobalState::w1;
double GlobalState::w2;
double GlobalState::w3;
double GlobalState::w4;
double GlobalState::w5;
double GlobalState::w6;
double GlobalState::w7 = 0.0;				  // look_ahead_based_on_stanley_method
double GlobalState::look_ahead_horizon = 5.0; // fraction of the path (in meters it is about N*0.10m) based_on_stanley_method

int GlobalState::behavior_selector_task;
double GlobalState::param_parking_speed_limit;
double GlobalState::distance_between_waypoints;
int GlobalState::route_planner_state;
double GlobalState::param_max_vel;
double GlobalState::param_max_vel_reverse;

static double param_offroad_min_resolution_path = -1.0;
double param_offroad_max_reed_shepp_distance = -1.0;

double max_phi_multiplier;

int semi_trailer_solver_type = 1; // 0=AMPL, 1=CASADI

double distance_map_huge_distance = 200.0;

int offroad_planner_obstacle_distance_map_on = 1;

int kbhit()
{
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds); // STDIN_FILENO is 0
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
	return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(int state)
{
	struct termios ttystate;

	// get the terminal state
	tcgetattr(STDIN_FILENO, &ttystate);

	if (state == NB_ENABLE)
	{
		// turn off canonical mode
		ttystate.c_lflag &= ~ICANON;
		// minimum of number input read.
		ttystate.c_cc[VMIN] = 1;
	}
	else if (state == NB_DISABLE)
	{
		// turn on canonical mode
		ttystate.c_lflag |= ICANON;
	}
	// set the terminal attributes.
	tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

double
get_offroad_min_resolution_path_param()
{
	return (param_offroad_min_resolution_path);
}

bool update_goal_distance_map(carmen_robot_and_trailers_traj_point_t *requested_goal)
{
	if (!obstacle_distance_grid_map)
		return (false);

	grid_state_map_x_size = round((obstacle_distance_grid_map->config.x_size * obstacle_distance_grid_map->config.resolution) / astar_config.state_map_resolution);
	grid_state_map_y_size = round((obstacle_distance_grid_map->config.y_size * obstacle_distance_grid_map->config.resolution) / astar_config.state_map_resolution);

	int x = get_grid_state_map_x(requested_goal->x, obstacle_distance_grid_map);
	int y = get_grid_state_map_y(requested_goal->y, obstacle_distance_grid_map);

	if (x >= grid_state_map_x_size || x < 0 || y >= grid_state_map_y_size || y < 0)
		return (false);

	carmen_point_t final_goal = {requested_goal->x, requested_goal->y, requested_goal->theta};
	goal_distance_map = get_goal_distance_map(goal_distance_map, &final_goal, obstacle_distance_grid_map);

	return (true);
}

void save_obstacle_distance_map(carmen_obstacle_distance_mapper_map_message *obstacle_distance_map)
{
	FILE *map_file = fopen("obstacle_distance_map.map", "w");

	fwrite(&(obstacle_distance_map->config), sizeof(carmen_map_config_t), 1, map_file);
	fwrite(&(obstacle_distance_map->size), sizeof(int), 1, map_file);
	fwrite(obstacle_distance_map->complete_x_offset, sizeof(short int), obstacle_distance_map->size, map_file);
	fwrite(obstacle_distance_map->complete_y_offset, sizeof(short int), obstacle_distance_map->size, map_file);

	fclose(map_file);

	carmen_map_t out_map;

	out_map.config = obstacle_distance_map->config;
	out_map.complete_map = (double *)malloc(sizeof(double) * out_map.config.x_size * out_map.config.y_size);
	carmen_test_alloc(out_map.complete_map);
	out_map.map = (double **)malloc(sizeof(double *) * out_map.config.x_size);
	carmen_test_alloc(out_map.map);
	for (int i = 0; i < out_map.config.x_size * out_map.config.y_size; i++)
		out_map.complete_map[i] = sqrt(obstacle_distance_map->complete_x_offset[i] * obstacle_distance_map->complete_x_offset[i] +
									   obstacle_distance_map->complete_y_offset[i] * obstacle_distance_map->complete_y_offset[i]);
	for (int x = 0; x < out_map.config.x_size; x++)
		out_map.map[x] = &(out_map.complete_map[x * out_map.config.y_size]);

	carmen_grid_mapping_save_map((char *)"visible_obstacle_distance_map.map", &out_map);

	free(out_map.map);
	free(out_map.complete_map);
}

double
get_goal_distance(carmen_robot_and_trailers_traj_point_t point)
{
	if (!goal_distance_map)
		return (grid_state_map_x_size * M_SQRT2 * astar_config.state_map_resolution); // Max dist.

	int x = get_grid_state_map_x(point.x, obstacle_distance_grid_map);
	int y = get_grid_state_map_y(point.y, obstacle_distance_grid_map);

	if (x >= grid_state_map_x_size || x < 0 || y >= grid_state_map_y_size || y < 0)
		return (grid_state_map_x_size * M_SQRT2 * astar_config.state_map_resolution); // Max dist.

	double distance_in_meters = goal_distance_map[(int)(x * grid_state_map_y_size + y)] * astar_config.state_map_resolution; // No carmen o mapa eh organizado em colunas (x trocado com o y)

	return (distance_in_meters);
}

bool find_the_nearest_pose_in_current_route(int &merge_pose_in_current_route_index,
											carmen_route_planner_road_network_message *road_network_message)
{
	double smallest_distance = 10000000.0;

	for (int pose_index = 0; pose_index < road_network_message->number_of_poses; pose_index++)
	{
		double distance = get_goal_distance(road_network_message->poses[pose_index]);
		if (distance < smallest_distance)
		{
			smallest_distance = distance;
			merge_pose_in_current_route_index = pose_index;
		}
	}

	if (smallest_distance < 10000000.0)
		return (true);
	else
		return (false);
}

bool find_the_nearest_pose_to_goal_in_current_route(int &start_pose_in_current_route_index, carmen_route_planner_road_network_message *road_network_message)
{
	double smallest_distance = 10000000.0;

	for (int pose_index = 0; pose_index < road_network_message->number_of_poses; pose_index++)
	{
		double distance = get_goal_distance(road_network_message->poses[pose_index]);
		if (distance < smallest_distance)
		{
			smallest_distance = distance;
			start_pose_in_current_route_index = pose_index;
		}
	}

	if (smallest_distance < 10000000.0)
		return (true);
	else
		return (false);
}

int is_obstacle(carmen_robot_and_trailers_traj_point_t point)
{
	int hit = trajectory_pose_hit_obstacle(point, robot_config.model_predictive_planner_obstacles_safe_distance, obstacle_distance_grid_map, &robot_config);
	if (hit)
		return (TRUE);
	else
		return (FALSE);
}

carmen_robot_and_trailers_traj_point_t
move_in_current_route_upto_a_suitable_goal_pose(int &merge_pose_in_current_route_index,
												carmen_route_planner_road_network_message *road_network_message)
{
	double distance_without_obstacles = 0.0;
	carmen_robot_and_trailers_traj_point_t goal_pose;
	while ((distance_without_obstacles < robot_config.distance_between_front_and_rear_axles * 8.0) &&
		   (merge_pose_in_current_route_index < (road_network_message->number_of_poses - 1)))
	{
		double delta_distance = DIST2D(road_network_message->poses[merge_pose_in_current_route_index], road_network_message->poses[merge_pose_in_current_route_index + 1]);
		distance_without_obstacles += delta_distance;
		merge_pose_in_current_route_index++;
		goal_pose = road_network_message->poses[merge_pose_in_current_route_index];
		if (is_obstacle(goal_pose))
			distance_without_obstacles = 0.0;
	}

	return (goal_pose);
}

carmen_robot_and_trailers_traj_point_t
move_back_in_current_route_upto_a_suitable_start_pose(int &start_pose_in_current_route_index, carmen_route_planner_road_network_message *road_network_message)
{
	if (semi_trailer_config.num_semi_trailers != 0)
		return (road_network_message->poses[road_network_message->number_of_poses - 1]);

	double distance_behind = 0.0;
	while ((distance_behind < robot_config.distance_between_front_and_rear_axles * 3.0) &&
		   (start_pose_in_current_route_index > 0))
	{
		distance_behind += DIST2D(road_network_message->poses[start_pose_in_current_route_index], road_network_message->poses[start_pose_in_current_route_index - 1]);
		start_pose_in_current_route_index--;
	}
	carmen_robot_and_trailers_traj_point_t start_pose = road_network_message->poses[start_pose_in_current_route_index];

	return (start_pose);
}

bool find_suitable_goal_pose_in_current_route(carmen_robot_and_trailers_traj_point_t &goal_pose,
											  int &merge_pose_in_current_route_index, carmen_route_planner_road_network_message *road_network_message)
{
	if (update_goal_distance_map(&robot_position))
	{
		//		system("cp /home/alberto/carmen/bin/obstacle_heuristic.map /home/alberto/carmen/bin/obstacle_heuristicx.map");
		if (find_the_nearest_pose_in_current_route(merge_pose_in_current_route_index, road_network_message))
		{
			goal_pose = move_in_current_route_upto_a_suitable_goal_pose(merge_pose_in_current_route_index, road_network_message);

			return (true);
		}
	}

	return (false);
}

bool find_suitable_start_pose_in_current_route(carmen_robot_and_trailers_traj_point_t &start_pose, int &start_pose_in_current_route_index,
											   carmen_robot_and_trailers_traj_point_t goal_pose, carmen_route_planner_road_network_message *road_network_message)
{
	if (update_goal_distance_map(&goal_pose))
	{
		if (find_the_nearest_pose_to_goal_in_current_route(start_pose_in_current_route_index, road_network_message))
		{
			start_pose = move_back_in_current_route_upto_a_suitable_start_pose(start_pose_in_current_route_index, road_network_message);

			return (true);
		}
	}

	return (false);
}

void save_plan(offroad_planner_plan_t plan, char *plan_file_name)
{
	FILE *plan_file = fopen(plan_file_name, "w");

	fwrite(&plan, sizeof(offroad_planner_plan_t), 1, plan_file);
	fwrite(plan.path.points, sizeof(carmen_robot_and_trailers_traj_point_t), plan.path.length, plan_file);

	fclose(plan_file);
}

void load_plan(offroad_planner_plan_t &plan, char *plan_file_name)
{
	FILE *plan_file = fopen(plan_file_name, "r");

	fread(&plan, sizeof(offroad_planner_plan_t), 1, plan_file);
	plan.path.points = (carmen_robot_and_trailers_traj_point_t *)malloc(sizeof(carmen_robot_and_trailers_traj_point_t) * plan.path.length);
	fread(plan.path.points, sizeof(carmen_robot_and_trailers_traj_point_t), plan.path.length, plan_file);

	fclose(plan_file);
}

int nearest_pose_in_current_route(carmen_robot_and_trailers_traj_point_t pose, carmen_route_planner_road_network_message *road_network_message)
{
	double smallest_distance = 10000000.0;
	int nearest_pose_index = 0;

	for (int pose_index = 0; pose_index < road_network_message->number_of_poses; pose_index++)
	{
		double distance = DIST2D(road_network_message->poses[pose_index], pose);
		if (distance < smallest_distance)
		{
			nearest_pose_index = pose_index;
			smallest_distance = distance;
		}
	}

	return (nearest_pose_index);
}

int remove_unecessary_points_at_the_end(std::vector<carmen_robot_and_trailers_traj_point_t> &astar_path,
										carmen_route_planner_road_network_message *road_network_message)
{
	unsigned int i = 0;
	int pose_index = 0;
	for (; i < astar_path.size(); i++)
	{
		pose_index = nearest_pose_in_current_route(astar_path[i], road_network_message);
		int point_is;
		carmen_robot_and_trailers_traj_point_t pose = {};

		if (pose_index < (road_network_message->number_of_poses - 1))
			pose = carmen_get_point_nearest_to_trajectory(&point_is,
														 road_network_message->poses[pose_index], road_network_message->poses[pose_index + 1], astar_path[i], 0.1);

		if (point_is != POINT_WITHIN_SEGMENT)
		{
			if (pose_index > 0)
				pose = carmen_get_point_nearest_to_trajectory(&point_is,
															 road_network_message->poses[pose_index - 1], road_network_message->poses[pose_index], astar_path[i], 0.1);
			else
				pose = road_network_message->poses[pose_index];
		}

		if ((DIST2D(astar_path[i], pose) < 0.1) &&
			(fabs(carmen_radians_to_degrees(carmen_normalize_theta(astar_path[i].theta - pose.theta))) < 20.0))
		{
			astar_path[i] = pose;
			break;
		}
	}

	if (i < astar_path.size() - 1)
		astar_path.erase(astar_path.begin() + i + 1, astar_path.end());

	return (pose_index);
}

int remove_unecessary_points_at_the_beginning(std::vector<carmen_robot_and_trailers_traj_point_t> &astar_path,
											  carmen_route_planner_road_network_message *road_network_message, int start_pose_in_current_route_index)
{
	int good_pose_index = 0;
	//	carmen_robot_and_trailer_traj_point_t good_pose;
	for (unsigned int i = 0; i < astar_path.size(); i++)
	{
		int pose_index = nearest_pose_in_current_route(astar_path[i], road_network_message);
		int point_is;
		carmen_robot_and_trailers_traj_point_t pose = {};

		if (pose_index < (road_network_message->number_of_poses - 1))
			pose = carmen_get_point_nearest_to_trajectory(&point_is,
														 road_network_message->poses[pose_index], road_network_message->poses[pose_index + 1], astar_path[i], 0.1);

		if (point_is != POINT_WITHIN_SEGMENT)
		{
			if (pose_index > 0)
				pose = carmen_get_point_nearest_to_trajectory(&point_is,
															 road_network_message->poses[pose_index - 1], road_network_message->poses[pose_index], astar_path[i], 0.1);
			else
				pose = road_network_message->poses[pose_index];
		}

		if ((DIST2D(astar_path[i], pose) < 0.1) &&
			(fabs(carmen_radians_to_degrees(carmen_normalize_theta(astar_path[i].theta - pose.theta))) < 35.0))
		{
			start_pose_in_current_route_index = pose_index;
			good_pose_index = i;
			//			good_pose = pose;
		}
	}

	if (good_pose_index > 0)
	{
		//		astar_path[good_pose_index] = good_pose;
		astar_path.erase(astar_path.begin(), astar_path.begin() + good_pose_index);
	}

	return (start_pose_in_current_route_index);
}

// static void
// print_poses(std::vector<carmen_robot_and_trailer_traj_point_t> poses, int number_of_poses, char *filename)
//{
//	FILE *arq = fopen(filename, "w");
//	for (int i = 0; i < 10000 && i < number_of_poses; i++)
//		fprintf(arq, "%lf %lf %lf %lf %lf\n",
//				poses[i].x, poses[i].y, poses[i].theta, poses[i].phi, poses[i].v);
//	fclose(arq);
// }

void print_path(carmen_robot_and_trailers_traj_point_t *path, int size)
{
	for (int i = 0; i < size; i++)
	{
		printf(" i %2d, x %.2f, y %.2f, phi %.2f, theta %.3f (%.2f), v[i] %lf, dist_2D [i]->[i+1] %.2f\n", i,
			   path[i].x, path[i].y, path[i].phi, // carmen_radians_to_degrees(path[i].phi),
			   path[i].theta, carmen_radians_to_degrees(path[i].theta),
			   path[i].v,
			   (i < size - 1) ? DIST2D(path[i], path[i + 1]) : 0.0); // O displacement eh o deslocamento para chegar ao proximo ponto.
	}
	printf("################ end path\n");
	fflush(stdout);
}

void save_plan_into_offroad_planner_cache(offroad_planner_plan_t plan)
{
	//	FILE *offroad_planner_cache = fopen("offroad_planner_cache.txt", "a+");
	FILE *offroad_planner_cache = fopen(cache_filename, "a+");

	fprintf(offroad_planner_cache, "robot %lf, %lf, %lf, %lf\n", plan.robot.x, plan.robot.y, plan.robot.theta, plan.robot.trailer_theta[0]);
	fprintf(offroad_planner_cache, "goal %lf, %lf, %lf, %lf\n", plan.goal.x, plan.goal.y, plan.goal.theta, plan.goal.trailer_theta[0]);
	fprintf(offroad_planner_cache, "length %d\n", plan.path.length);
	for (int i = 0; i < plan.path.length; i++)
		fprintf(offroad_planner_cache, "%lf, %lf, %lf, %lf, %lf, %lf\n",
				plan.path.points[i].x, plan.path.points[i].y, plan.path.points[i].theta, plan.path.points[i].trailer_theta[0],
				plan.path.points[i].v, plan.path.points[i].phi);
	fclose(offroad_planner_cache);
}

bool get_a_plan_from_offroad_planner_cache(FILE *offroad_planner_cache, offroad_planner_plan_t &plan)
{
	int num_itens;
	num_itens = fscanf(offroad_planner_cache, "robot %lf, %lf, %lf, %lf\n", &plan.robot.x, &plan.robot.y, &plan.robot.theta, &plan.robot.trailer_theta[0]);
	if (num_itens != 4)
		return (false);
	num_itens = fscanf(offroad_planner_cache, "goal %lf, %lf, %lf, %lf\n", &plan.goal.x, &plan.goal.y, &plan.goal.theta, &plan.goal.trailer_theta[0]);
	if (num_itens != 4)
		return (false);

	num_itens = fscanf(offroad_planner_cache, "length %d\n", &plan.path.length);
	if (num_itens != 1)
		return (false);
	plan.path.capacity = plan.path.length;
	plan.path.points = (carmen_robot_and_trailers_traj_point_t *)malloc(plan.path.length * sizeof(carmen_robot_and_trailers_traj_point_t));
	for (int i = 0; i < plan.path.length; i++)
	{
		num_itens = fscanf(offroad_planner_cache, "%lf, %lf, %lf, %lf, %lf, %lf\n",
						   &plan.path.points[i].x, &plan.path.points[i].y, &plan.path.points[i].theta, &plan.path.points[i].trailer_theta[0],
						   &plan.path.points[i].v, &plan.path.points[i].phi);

		if (num_itens != 6)
		{
			free(plan.path.points);
			return (false);
		}
	}

	plan.goal_set = 1;

	return (true);
}

bool plan_has_collision(offroad_planner_plan_t plan)
{
	bool path_has_collision = false;

	for (int i = 0; i < plan.path.length; i++)
	{
		if (trajectory_pose_hit_obstacle(plan.path.points[i], robot_config.model_predictive_planner_obstacles_safe_distance, obstacle_distance_grid_map, &robot_config))
		{
			printf("trajectory: %lf %lf %lf %lf %lf %lf %lf %lf\n", plan.path.points[i].x, plan.path.points[i].y, plan.path.points[i].theta, plan.path.points[i].trailer_theta[0], plan.path.points[i].trailer_theta[1], plan.path.points[i].trailer_theta[2], plan.path.points[i].trailer_theta[3], plan.path.points[i].trailer_theta[4]);
			path_has_collision = true;
			break;
		}
	}

	return (path_has_collision);
}

bool get_plan_from_offroad_planner_cache_old(offroad_planner_plan_t &plan, carmen_robot_and_trailers_traj_point_t robot, carmen_robot_and_trailers_traj_point_t goal)
{
	//	FILE *offroad_planner_cache = fopen("offroad_planner_cache.txt", "r");
	FILE *offroad_planner_cache = fopen(cache_filename, "r");

	if (!offroad_planner_cache)
		return (false);

	while (get_a_plan_from_offroad_planner_cache(offroad_planner_cache, plan))
	{
		if (near_enough(DIST2D(goal, plan.goal), THETA_DIFF(goal, plan.goal), THETA0_DIFF(goal, plan.goal)) &&
			near_enough(DIST2D(robot, plan.robot), THETA_DIFF(robot, plan.robot), THETA0_DIFF(robot, plan.robot)))
		{
			if (!plan_has_collision(plan))
			{
				fclose(offroad_planner_cache);

				return (true);
			}
			else
				free(plan.path.points);
		}
		else
			free(plan.path.points);
	}

	fclose(offroad_planner_cache);

	return (false);
}

bool get_plan_from_offroad_planner_cache(offroad_planner_plan_t &plan, carmen_robot_and_trailers_traj_point_t robot, carmen_robot_and_trailers_traj_point_t goal)
{

	std::string line;
	std::ifstream logfile(cache_filename);

	if (!logfile.is_open())
	{
		return (false);
	}
	else
	{
		bool reading_path_state = false;
		int lines_already_seen = 0;

		while (std::getline(logfile, line))
		{
			line.erase(std::remove(line.begin(), line.end(), ','), line.end()); // Removendo as possíveis vírgulas do cache
			carmen_line_content current_content = create_carmen_line_content(line);

			if (reading_path_state)
			{
				if (lines_already_seen < plan.path.length)
				{
					if (current_content.size == 6)
					{
						plan.path.points[lines_already_seen].x = std::stod(get_string_from_carmen_line_content(current_content, 0));
						plan.path.points[lines_already_seen].y = std::stod(get_string_from_carmen_line_content(current_content, 1));
						plan.path.points[lines_already_seen].theta = std::stod(get_string_from_carmen_line_content(current_content, 2));
						plan.path.points[lines_already_seen].trailer_theta[0] = std::stod(get_string_from_carmen_line_content(current_content, 3));
						plan.path.points[lines_already_seen].v = std::stod(get_string_from_carmen_line_content(current_content, 4));
						plan.path.points[lines_already_seen].phi = std::stod(get_string_from_carmen_line_content(current_content, 5));
					}
					else if (current_content.size == 11)
					{
						plan.path.points[lines_already_seen].x = std::stod(get_string_from_carmen_line_content(current_content, 0));
						plan.path.points[lines_already_seen].y = std::stod(get_string_from_carmen_line_content(current_content, 1));
						plan.path.points[lines_already_seen].theta = std::stod(get_string_from_carmen_line_content(current_content, 2));
						plan.path.points[lines_already_seen].num_trailers = std::stoi(get_string_from_carmen_line_content(current_content, 3));
						plan.path.points[lines_already_seen].trailer_theta[0] = std::stod(get_string_from_carmen_line_content(current_content, 4));
						plan.path.points[lines_already_seen].trailer_theta[1] = std::stod(get_string_from_carmen_line_content(current_content, 5));
						plan.path.points[lines_already_seen].trailer_theta[2] = std::stod(get_string_from_carmen_line_content(current_content, 6));
						plan.path.points[lines_already_seen].trailer_theta[3] = std::stod(get_string_from_carmen_line_content(current_content, 7));
						plan.path.points[lines_already_seen].trailer_theta[4] = std::stod(get_string_from_carmen_line_content(current_content, 8));
						plan.path.points[lines_already_seen].v = std::stod(get_string_from_carmen_line_content(current_content, 9));
						plan.path.points[lines_already_seen].phi = std::stod(get_string_from_carmen_line_content(current_content, 10));
					}
					else
						return (false);
				}

				if (lines_already_seen == (plan.path.length - 1))
				{
					plan.goal_set = 1;
					if (!plan_has_collision(plan))
					{
						return (true);
					}
					else
					{
						reading_path_state = false;
						plan.path.length = -1;
						lines_already_seen = 0;
						free(plan.path.points);
					}
				}
				else
					lines_already_seen++;
			}
			else
			{
				std::string tag = get_string_from_carmen_line_content(current_content, 0);

				if (tag.compare("robot") == 0)
				{
					if (current_content.size == 5)
					{
						reading_path_state = false;
						plan.path.length = -1;
						lines_already_seen = 0;

						plan.robot.x = std::stod(get_string_from_carmen_line_content(current_content, 1));
						plan.robot.y = std::stod(get_string_from_carmen_line_content(current_content, 2));
						plan.robot.theta = std::stod(get_string_from_carmen_line_content(current_content, 3));
						plan.robot.trailer_theta[0] = std::stod(get_string_from_carmen_line_content(current_content, 4));
					}
					else if (current_content.size == 10)
					{
						plan.robot.x = std::stod(get_string_from_carmen_line_content(current_content, 1));
						plan.robot.y = std::stod(get_string_from_carmen_line_content(current_content, 2));
						plan.robot.theta = std::stod(get_string_from_carmen_line_content(current_content, 3));
						plan.robot.num_trailers = std::stoi(get_string_from_carmen_line_content(current_content, 4));
						plan.robot.trailer_theta[0] = std::stod(get_string_from_carmen_line_content(current_content, 5));
						plan.robot.trailer_theta[1] = std::stod(get_string_from_carmen_line_content(current_content, 6));
						plan.robot.trailer_theta[2] = std::stod(get_string_from_carmen_line_content(current_content, 7));
						plan.robot.trailer_theta[3] = std::stod(get_string_from_carmen_line_content(current_content, 8));
						plan.robot.trailer_theta[4] = std::stod(get_string_from_carmen_line_content(current_content, 9));
					}
					else
						return (false);
				}
				else if (tag.compare("goal") == 0)
				{
					if (current_content.size == 5)
					{
						plan.goal.x = std::stod(get_string_from_carmen_line_content(current_content, 1));
						plan.goal.y = std::stod(get_string_from_carmen_line_content(current_content, 2));
						plan.goal.theta = std::stod(get_string_from_carmen_line_content(current_content, 3));
						plan.goal.trailer_theta[0] = std::stod(get_string_from_carmen_line_content(current_content, 4));
					}
					else if (current_content.size == 10)
					{
						plan.goal.x = std::stod(get_string_from_carmen_line_content(current_content, 1));
						plan.goal.y = std::stod(get_string_from_carmen_line_content(current_content, 2));
						plan.goal.theta = std::stod(get_string_from_carmen_line_content(current_content, 3));
						plan.goal.num_trailers = std::stoi(get_string_from_carmen_line_content(current_content, 4));
						plan.goal.trailer_theta[0] = std::stod(get_string_from_carmen_line_content(current_content, 5));
						plan.goal.trailer_theta[1] = std::stod(get_string_from_carmen_line_content(current_content, 6));
						plan.goal.trailer_theta[2] = std::stod(get_string_from_carmen_line_content(current_content, 7));
						plan.goal.trailer_theta[3] = std::stod(get_string_from_carmen_line_content(current_content, 8));
						plan.goal.trailer_theta[4] = std::stod(get_string_from_carmen_line_content(current_content, 9));
					}
					else
						return (false);
				}
				else if (tag.compare("length") == 0)
				{
					if (current_content.size == 2)
					{
						if (near_enough(DIST2D(goal, plan.goal), THETA_DIFF(goal, plan.goal), THETA0_DIFF(goal, plan.goal)) &&
							near_enough(DIST2D(robot, plan.robot), THETA_DIFF(robot, plan.robot), THETA0_DIFF(robot, plan.robot)))
						{
							reading_path_state = true;
							plan.path.length = std::stoi(get_string_from_carmen_line_content(current_content, 1));
							plan.path.capacity = plan.path.length;
							plan.path.points = (carmen_robot_and_trailers_traj_point_t *)malloc(plan.path.length * sizeof(carmen_robot_and_trailers_traj_point_t));
						}
					}
					else
						return (false);
				}
				else
					continue; // Está passando por um path que não é relevante por estar longe do goal e do origin.
			}
		}
	}

	return (false);
}

carmen_offroad_planner_feedback_t
carmen_offroad_planner_plan(offroad_planner_plan_t &plan, carmen_robot_and_trailers_traj_point_t &start_pose,
						   carmen_robot_and_trailers_traj_point_t goal_pose,
						   int &merge_pose_in_current_route_index,
						   carmen_route_planner_road_network_message *road_network_message = NULL)
{
#ifdef ENABLE_CACHING
	if (get_plan_from_offroad_planner_cache(plan, start_pose, goal_pose))
		return (PLAN_OK);
#endif

	if (!update_goal_distance_map(&goal_pose))
	{
		plan = {};

		return (COULD_NOT_COMPUTE_GOAL_DISTANCE_MAP);
	}

	//	pose_node initial_pose = {start_pose.x, start_pose.y, start_pose.theta, start_pose.trailer_theta[0], start_pose.phi, Forward};
	//	pose_node final_pose = {goal_pose.x, goal_pose.y, goal_pose.theta, goal_pose.trailer_theta[0], goal_pose.phi, Forward};
	carmen_offroad_planner_feedback_t feedback;
	double time = carmen_get_time();
	std::vector<carmen_robot_and_trailers_traj_point_t> astar_path = carmen_path_planner_astar_search(feedback, &start_pose, &goal_pose, obstacle_distance_grid_map, goal_distance_map, nonholonomic_heuristic_cost_map, road_network_message);

	printf("time: %f\n", carmen_get_time() - time);

	if (road_network_message)
	{
		if ((road_network_message->offroad_planner_request == PLAN_FROM_POSE_TO_LANE) && (semi_trailer_config.num_semi_trailers == 0))
		{
			merge_pose_in_current_route_index = remove_unecessary_points_at_the_end(astar_path, road_network_message);
			goal_pose = road_network_message->poses[merge_pose_in_current_route_index];
		}
		//		if (road_network_message->offroad_planner_request == PLAN_FROM_LANE_TO_FINAL_POSE)
		//		{
		//			merge_pose_in_current_route_index = remove_unecessary_points_at_the_beginning(astar_path, road_network_message, merge_pose_in_current_route_index);
		//			start_pose = road_network_message->poses[merge_pose_in_current_route_index];
		//		}
	}

	if (astar_path.size() > 1)
	{
		// Consertar o v 0.0 do initial pose e final pose
		astar_path[0].v = astar_path[1].v;
		astar_path[astar_path.size() - 1].v = astar_path[astar_path.size() - 2].v;

		astar_path = increase_path_resolution(astar_path);

		plan.robot = start_pose;
		plan.goal = goal_pose;
		plan.goal_set = 1;

		plan.path.length = astar_path.size();
		plan.path.capacity = plan.path.length;
		plan.path.points = (carmen_robot_and_trailers_traj_point_t *)malloc(plan.path.length * sizeof(carmen_robot_and_trailers_traj_point_t));
		for (int i = 0; i < plan.path.length; i++)
			plan.path.points[i] = astar_path[i];

#ifdef ENABLE_CACHING
		if (cache_writing)
			save_plan_into_offroad_planner_cache(plan);
#endif
	}
	else
		plan = {};

	//	save_plan(plan, (char *) "plan_saved.plan");
	//	free(plan.path.points);
	//	load_plan(plan, (char *) "plan_saved.plan");

	return (feedback);
}

int get_index_of_point_within_route(carmen_robot_and_trailers_traj_point_t point, carmen_route_planner_road_network_message *route)
{
	int j;

	for (j = 0; j < route->number_of_poses - 1; j++)
	{
		int point_in_trajectory_is;
		carmen_get_point_nearest_to_trajectory(&point_in_trajectory_is, route->poses[j], route->poses[j + 1],
											  point, 0.001);
		if (point_in_trajectory_is == POINT_WITHIN_SEGMENT)
			break;
	}

	if (j == route->number_of_poses - 1)
		return (-1);
	else
		return (j);
}

// VERIFICAR SE IRÁ FICAR AQUI

void compute_rectilinear_route_half_segment(vector<carmen_robot_and_trailers_traj_point_t> &rectilinear_route_segment,
											double size, carmen_annotation_t annotation, double theta, double step_size, bool reverse)
{
	if (reverse)
	{
		double distance = size;
		while (distance >= 0.0)
		{
			carmen_robot_and_trailers_traj_point_t point = {};
			double theta = carmen_normalize_theta(annotation.annotation_orientation + M_PI);
			point.x = annotation.annotation_point.x + distance * cos(theta);
			point.y = annotation.annotation_point.y + distance * sin(theta);
			point.theta = annotation.annotation_orientation;
			point.v = 1.0;
			rectilinear_route_segment.push_back(point);

			distance -= step_size;
		}
	}
	else
	{
		double distance = 0.0;
		while (distance < size)
		{
			carmen_robot_and_trailers_traj_point_t point = {};
			point.x = annotation.annotation_point.x + distance * cos(theta);
			point.y = annotation.annotation_point.y + distance * sin(theta);
			point.theta = annotation.annotation_orientation;
			point.v = 1.0;
			rectilinear_route_segment.push_back(point);

			distance += step_size;
		}
	}
}

vector<carmen_robot_and_trailers_traj_point_t>
carmen_rddf_compute_rectilinear_route_segment(carmen_annotation_t annotation, double size_front, double size_back, double step_size)
{
	vector<carmen_robot_and_trailers_traj_point_t> rectilinear_route_segment;

	double theta = annotation.annotation_orientation;
	compute_rectilinear_route_half_segment(rectilinear_route_segment, size_back, annotation, theta, step_size, true);
	compute_rectilinear_route_half_segment(rectilinear_route_segment, size_front, annotation, theta, step_size, false);

	return (rectilinear_route_segment);
}

void carmen_rddf_get_barrier_alignment_segments_sizes(carmen_annotation_t *annotation, double *size_front, double *size_back)
{
	switch (annotation->annotation_code)
	{
	case RDDF_ANNOTATION_CODE_BARRIER_25_25:
		*size_front = 25.0;
		*size_back = 25.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_25_20:
		*size_front = 25.0;
		*size_back = 20.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_25_15:
		*size_front = 25.0;
		*size_back = 15.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_25_10:
		*size_front = 25.0;
		*size_back = 10.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_20_25:
		*size_front = 20.0;
		*size_back = 25.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_20_20:
		*size_front = 20.0;
		*size_back = 20.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_20_15:
		*size_front = 20.0;
		*size_back = 15.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_20_10:
		*size_front = 20.0;
		*size_back = 10.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_15_25:
		*size_front = 15.0;
		*size_back = 25.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_15_20:
		*size_front = 15.0;
		*size_back = 20.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_15_15:
		*size_front = 15.0;
		*size_back = 15.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_15_10:
		*size_front = 15.0;
		*size_back = 10.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_10_25:
		*size_front = 10.0;
		*size_back = 25.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_10_20:
		*size_front = 10.0;
		*size_back = 20.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_10_15:
		*size_front = 10.0;
		*size_back = 15.0;
		break;

	case RDDF_ANNOTATION_CODE_BARRIER_10_10:
		*size_front = 10.0;
		*size_back = 10.0;
		break;

	default:
		*size_front = 0.0;
		*size_back = 0.0;
	}
}

/////////////////////////////

bool get_annotation_rectilinear_segment_first_pose(carmen_robot_and_trailers_traj_point_t &goal_pose, int &merge_pose_in_current_route_index,
												   carmen_route_planner_road_network_message *route)
{
	if (!last_rddf_annotation_message)
		return (false);

	carmen_annotation_t *barrier_annotation = carmen_behavior_selector_get_nearest_specified_annotation(RDDF_ANNOTATION_TYPE_BARRIER, *last_rddf_annotation_message,
																									  &robot_position);

	if (barrier_annotation)
	{
		double size_front;
		double size_back;
		carmen_rddf_get_barrier_alignment_segments_sizes(barrier_annotation, &size_front, &size_back);
		if ((size_front == 0.0) && (size_back == 0.0))
			return (false);

		vector<carmen_robot_and_trailers_traj_point_t> rectilinear_route_segment =
			carmen_rddf_compute_rectilinear_route_segment(*barrier_annotation, size_front, size_back, 0.5);

		if (rectilinear_route_segment.size() > 1)
		{
			goal_pose = rectilinear_route_segment[1];
			merge_pose_in_current_route_index = get_index_of_point_within_route(goal_pose, route);
			if (merge_pose_in_current_route_index != -1)
				return (true);
			else
				return (false);
		}
		else
			return (false);
	}
	else
		return (false);
}

vector<carmen_robot_and_trailers_traj_point_t>
get_rectilinear_segment(carmen_robot_and_trailers_traj_point_t &goal_pose)
{
	double size_front = 0.0;
	double size_back = (double)(final_goal->half_meters_to_final_goal) / 2.0;
	carmen_annotation_t annotation;
	annotation.annotation_orientation = final_goal->point.theta;
	annotation.annotation_point.x = final_goal->point.x;
	annotation.annotation_point.y = final_goal->point.y;

	vector<carmen_robot_and_trailers_traj_point_t> rectilinear_route_segment =
		carmen_rddf_compute_rectilinear_route_segment(annotation, size_front, size_back, 0.5);
	goal_pose = rectilinear_route_segment[0];

	return (rectilinear_route_segment);
}

vector<carmen_robot_and_trailers_traj_point_t>
get_rectilinear_segment(carmen_robot_and_trailers_traj_point_t robot_pose, carmen_robot_and_trailers_traj_point_t goal_pose)
{
	vector<carmen_robot_and_trailers_traj_point_t> rectilinear_route_segment;

	double size = DIST2D(robot_pose, goal_pose);
	double theta = ANGLE2D(robot_pose, goal_pose);
	double diff_theta = fabs(carmen_normalize_theta(theta - robot_pose.theta));
	double v = 1.0;
	double route_segment_theta = theta;

	if (diff_theta > (M_PI / 2.0))
	{
		v = -1.0;
		route_segment_theta = carmen_normalize_theta(route_segment_theta + M_PI);
	}

	double distance = 0.0;
	double delta_distance = size / round(size / 0.5);
	//	printf("delta_distance %lf\n", delta_distance);
	//	carmen_robot_and_trailer_traj_point_t pose = robot_pose;
	//	printf("robot_pose, x %.2f, y %.2f, phi %.2f, theta %.3f (%.2f), v[i] %lf\n",
	//			pose.x, pose.y, pose.phi, //carmen_radians_to_degrees(pose.phi),
	//			pose.theta, carmen_radians_to_degrees(pose.theta),
	//			pose.v);
	//	pose = goal_pose;
	//		printf("goal_pose , x %.2f, y %.2f, phi %.2f, theta %.3f (%.2f), v[i] %lf\n",
	//				pose.x, pose.y, pose.phi, //carmen_radians_to_degrees(pose.phi),
	//				pose.theta, carmen_radians_to_degrees(pose.theta),
	//				pose.v);
	for (int i = 0; i <= round(size / 0.5); i++)
	{
		carmen_robot_and_trailers_traj_point_t point = {};
		point.x = robot_pose.x + distance * cos(theta);
		point.y = robot_pose.y + distance * sin(theta);
		point.theta = route_segment_theta;
		point.v = v;
		rectilinear_route_segment.push_back(point);

		distance += delta_distance;
	}

	return (rectilinear_route_segment);
}

void clear_obstacle_distance_grid_map(carmen_obstacle_distance_mapper_map_message *distance_map, double distance_map_huge_distance)
{
	for (int i = 0; i < distance_map->size; i++)
		distance_map->complete_x_offset[i] = distance_map->complete_y_offset[i] = distance_map_huge_distance;
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
publish_offroad_plan(carmen_robot_and_trailers_traj_point_t *offroad_plan, int merged_plan_size,
					 carmen_offroad_planner_feedback_t feedback, int merge_pose_in_current_route_index,
					 carmen_robot_and_trailers_traj_point_t transition_pose, double time_to_plan = -1.0)
{
	carmen_offroad_planner_plan_message plan;

	plan.offroad_planner_feedback = feedback;
#ifdef TEST_RUSSO
	if (merged_plan_size > 300)
	{
		printf("merged_plan_size muito grande e = %d\n", merged_plan_size);
		merged_plan_size = 300;
	}
#endif

#ifdef SO_GOAL
	if (merged_plan_size >= 3)
	{
		if (offroad_plan)
		{
			offroad_plan[2] = offroad_plan[merged_plan_size - 1];
			offroad_plan[2].trailer_theta[0] = 0.0;
			offroad_plan[1] = offroad_plan[merged_plan_size - 2];
			offroad_plan[1].trailer_theta[0] = 0.0;
		}
		merged_plan_size = 3;
	}
#endif

	plan.number_of_poses = merged_plan_size;
	plan.poses = offroad_plan;
	plan.pose_id = merge_pose_in_current_route_index;
	plan.transition_pose = transition_pose;
	plan.time_to_plan = time_to_plan;
	if (offroad_plan)
		plan.goal_pose = offroad_plan[merged_plan_size - 1];
	else
		plan.goal_pose = transition_pose;

	carmen_offroad_planner_publish_plan(&plan);
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	robot_position = {msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta, msg->num_trailers, {0.0}, msg->v, msg->phi};

	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		robot_position.trailer_theta[z] = msg->trailer_theta[z];

	if (msg->semi_trailer_type != semi_trailer_config.num_semi_trailers)
	{
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc_global, argv_global, msg->semi_trailer_type);
		carmen_collision_detection_set_semi_trailer_type(semi_trailer_config.num_semi_trailers);
	}
}

static void
carmen_route_planner_road_network_message_handler(carmen_route_planner_road_network_message *msg)
{
	static int current_request = NO_REQUEST;

	if (compact_distance_map == NULL)
	{
		publish_offroad_plan(NULL, 0, DEACTIVATE, 0, robot_position);

		return;
	}

	road_network_message = msg;

	if (current_request == NO_REQUEST)
	{
		if (msg->offroad_planner_request == PLAN_FROM_POSE_TO_LANE)
		{
			current_request = PLAN_FROM_POSE_TO_LANE;
			carmen_robot_and_trailers_traj_point_t goal_pose;
			int merge_pose_in_current_route_index;
			if (find_suitable_goal_pose_in_current_route(goal_pose, merge_pose_in_current_route_index, msg))
			{
				clock_t start, end;
				double cpu_time_used;
				offroad_planner_plan_t plan;
				start = clock();
				carmen_offroad_planner_feedback_t feedback = carmen_offroad_planner_plan(plan, robot_position, goal_pose, merge_pose_in_current_route_index, msg);
				end = clock();
				cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
				publish_offroad_plan(plan.path.points, plan.path.length, feedback, merge_pose_in_current_route_index, robot_position, cpu_time_used);
				free(plan.path.points);
			}
			else
			{
				publish_offroad_plan(NULL, 0, COULD_NOT_FIND_SUITABLE_GOAL_POSE, 0, robot_position);
				return;
			}
		}
		else if (msg->offroad_planner_request == PLAN_FROM_LANE_TO_FINAL_POSE)
		{
			current_request = PLAN_FROM_LANE_TO_FINAL_POSE;
			if (!final_goal)
			{
				publish_offroad_plan(NULL, 0, GOAL_NOT_SET, -1, robot_position);

				return;
			}

			carmen_robot_and_trailers_traj_point_t goal_pose =
				{
					final_goal->point.x,
					final_goal->point.y,
					final_goal->point.theta,
					final_goal->point.num_trailers,
					{0.0},
					0.0,
					0.0};

			for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
				goal_pose.trailer_theta[z] = final_goal->point.trailer_theta[z];

			carmen_robot_and_trailers_traj_point_t start_pose = robot_position;
			int start_pose_in_current_route_index;
			if (find_suitable_start_pose_in_current_route(start_pose, start_pose_in_current_route_index, goal_pose, msg))
			{
				clock_t start, end;
				double cpu_time_used;
				offroad_planner_plan_t plan;
				int not_used = 0;
				start = clock();
				carmen_offroad_planner_feedback_t feedback = carmen_offroad_planner_plan(plan, start_pose, goal_pose, not_used, msg);
				end = clock();
				cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
				printf("PLAN_FROM_LANE_TO_FINAL_POSE cpu_time_used: %lf\n", cpu_time_used);
				//				carmen_offroad_planner_feedback_t feedback = carmen_offroad_planner_plan(plan, start_pose, goal_pose, start_pose_in_current_route_index, msg);
				publish_offroad_plan(plan.path.points, plan.path.length, feedback, start_pose_in_current_route_index, start_pose, cpu_time_used);
				free(plan.path.points);
			}
			else
			{
				publish_offroad_plan(NULL, 0, COULD_NOT_FIND_SUITABLE_GOAL_POSE, 0, robot_position);

				return;
			}
		}
		else if (msg->offroad_planner_request == PLAN_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT)
		{
			current_request = PLAN_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT;
			carmen_robot_and_trailers_traj_point_t goal_pose;
			int merge_pose_in_current_route_index;
			if (!get_annotation_rectilinear_segment_first_pose(goal_pose, merge_pose_in_current_route_index, msg))
			{
				publish_offroad_plan(NULL, 0, COULD_NOT_FIND_SUITABLE_GOAL_POSE, -1, robot_position);

				return;
			}

			carmen_robot_and_trailers_traj_point_t start_pose = robot_position;
			int start_pose_in_current_route_index;
			if (find_suitable_start_pose_in_current_route(start_pose, start_pose_in_current_route_index, goal_pose, msg))
			{
				clock_t start, end;
				double cpu_time_used;
				offroad_planner_plan_t plan;
				int not_used = 0;
				start = clock();
				carmen_offroad_planner_feedback_t feedback = carmen_offroad_planner_plan(plan, start_pose, goal_pose, not_used, msg);
				end = clock();
				cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
				publish_offroad_plan(plan.path.points, plan.path.length, feedback, start_pose_in_current_route_index, start_pose, cpu_time_used);
				free(plan.path.points);
			}
			else
			{
				publish_offroad_plan(NULL, 0, COULD_NOT_FIND_SUITABLE_GOAL_POSE, 0, robot_position);

				return;
			}
		}
		else if (msg->offroad_planner_request == PLAN_FROM_CURRENT_POSE_TO_RECTLINEAR_ROUTE_SEGMENT)
		{
			if (!final_goal)
			{
				publish_offroad_plan(NULL, 0, GOAL_NOT_SET, -1, robot_position);

				return;
			}
			current_request = PLAN_FROM_CURRENT_POSE_TO_RECTLINEAR_ROUTE_SEGMENT;
			carmen_robot_and_trailers_traj_point_t goal_pose;
			vector<carmen_robot_and_trailers_traj_point_t> rectilinear_route_segment = get_rectilinear_segment(goal_pose);
			if (rectilinear_route_segment.size() == 0)
			{
				publish_offroad_plan(NULL, 0, COULD_NOT_FIND_SUITABLE_GOAL_POSE, 0, robot_position);

				return;
			}

			carmen_robot_and_trailers_traj_point_t start_pose = robot_position;
			offroad_planner_plan_t plan;
			int not_used = 0;
			carmen_offroad_planner_feedback_t feedback = carmen_offroad_planner_plan(plan, start_pose, goal_pose, not_used, msg);

			plan.path.points = (carmen_robot_and_trailers_traj_point_t *)realloc(plan.path.points, sizeof(carmen_robot_and_trailers_traj_point_t) * (plan.path.length + rectilinear_route_segment.size()));
			for (unsigned int i = 0; i < rectilinear_route_segment.size(); i++)
				plan.path.points[i + plan.path.length] = rectilinear_route_segment[i];
			plan.path.length += rectilinear_route_segment.size();

			publish_offroad_plan(plan.path.points, plan.path.length, feedback, 0, start_pose);
			//			print_path(plan.path.points, plan.path.length);
			free(plan.path.points);
		}
		else if (msg->offroad_planner_request == PLAN_FROM_CURRENT_POSE_TO_ENGAGE_POSE)
		{
			current_request = PLAN_FROM_CURRENT_POSE_TO_ENGAGE_POSE;
			if (!final_goal)
			{
				publish_offroad_plan(NULL, 0, GOAL_NOT_SET, -1, robot_position);

				return;
			}
			carmen_robot_and_trailers_traj_point_t goal_pose =
				{
					final_goal->point.x,
					final_goal->point.y,
					final_goal->point.theta,
					final_goal->point.num_trailers,
					{0.0},
					0.0,
					0.0};

			for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
				goal_pose.trailer_theta[z] = final_goal->point.trailer_theta[z];

			vector<carmen_robot_and_trailers_traj_point_t> rectilinear_route_segment = get_rectilinear_segment(robot_position, goal_pose);
			if (rectilinear_route_segment.size() == 0)
			{
				publish_offroad_plan(NULL, 0, GOAL_NOT_SET, -1, robot_position);

				return;
			}

			offroad_planner_plan_t plan;
			plan.path.length = rectilinear_route_segment.size();
			plan.path.points = (carmen_robot_and_trailers_traj_point_t *)malloc(sizeof(carmen_robot_and_trailers_traj_point_t) * plan.path.length);
			for (int i = 0; i < plan.path.length; i++)
				plan.path.points[i] = rectilinear_route_segment[i];

			//			print_path(plan.path.points, plan.path.length);
			publish_offroad_plan(plan.path.points, plan.path.length, PLAN_OK, -1, robot_position);
			free(plan.path.points);
		}
		else if (msg->offroad_planner_request == PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE)
		{
			clock_t start, end;
			double cpu_time_used;
			current_request = PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE;
			if (!final_goal)
			{
				publish_offroad_plan(NULL, 0, GOAL_NOT_SET, -1, robot_position);

				return;
			}
			carmen_robot_and_trailers_traj_point_t goal_pose =
				{
					final_goal->point.x,
					final_goal->point.y,
					final_goal->point.theta,
					final_goal->point.num_trailers,
					{0.0},
					0.0,
					0.0};

			for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
				goal_pose.trailer_theta[z] = final_goal->point.trailer_theta[z];

			offroad_planner_plan_t plan;
			int not_used = 0;
			start = clock();
			carmen_offroad_planner_feedback_t feedback = carmen_offroad_planner_plan(plan, robot_position, goal_pose, not_used, msg);
			end = clock();
			cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
			//			print_path(plan.path.points, plan.path.length);
			publish_offroad_plan(plan.path.points, plan.path.length, feedback, -1, robot_position, cpu_time_used);
			free(plan.path.points);
		}
	}
	else if (msg->offroad_planner_request == NO_REQUEST)
	{
		current_request = NO_REQUEST;
	}
}

static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (compact_distance_map == NULL)
	{
		GlobalState::distance_map = obstacle_distance_grid_map = &distance_map_struct;
		obstacle_distance_grid_map_print_map = &print_map_struct;
		//carmen_obstacle_distance_mapper_create_new_map(obstacle_distance_grid_map, distance_map_huge_distance, message->config, message->host, message->timestamp);
		//carmen_obstacle_distance_mapper_create_new_map(obstacle_distance_grid_map_print_map, distance_map_huge_distance, message->config, message->host, message->timestamp);
		carmen_obstacle_distance_mapper_create_new_map(obstacle_distance_grid_map , message->config, message->host, message->timestamp);
		carmen_obstacle_distance_mapper_create_new_map(obstacle_distance_grid_map_print_map , message->config, message->host, message->timestamp);
		compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *)(calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(obstacle_distance_grid_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(obstacle_distance_grid_map_print_map, message);

	}
	else
	{
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(obstacle_distance_grid_map, compact_distance_map, distance_map_huge_distance);
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(obstacle_distance_grid_map_print_map, compact_distance_map, distance_map_huge_distance);
		carmen_obstacle_distance_mapper_free_compact_distance_map(compact_distance_map);
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(obstacle_distance_grid_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(obstacle_distance_grid_map_print_map, message);

	}


	if (offroad_planner_obstacle_distance_map_on == 0)
		clear_obstacle_distance_grid_map(obstacle_distance_grid_map, distance_map_huge_distance);


	obstacle_distance_grid_map_print_map->config = message->config;
	obstacle_distance_grid_map->config = message->config;
	x_size = obstacle_distance_grid_map->config.x_size;
	y_size = obstacle_distance_grid_map->config.y_size;
}

static void
carmen_behaviour_selector_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (obstacle_distance_grid_map)
		carmen_obstacle_distance_mapper_overwrite_distance_map_message_with_compact_distance_map(obstacle_distance_grid_map, message);
}

static void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	final_goal = rddf_end_point_message;

	//	if (!behavior_selector_task)
	//		return;
	//	if (*behavior_selector_task != BEHAVIOR_SELECTOR_PARK)
	//		return;
	//
	//	carmen_robot_and_trailer_traj_point_t point =
	//	{
	//		rddf_end_point_message->point.x,
	//		rddf_end_point_message->point.y,
	//		rddf_end_point_message->point.theta,
	//		0.0,
	//		0.0
	//	};
	//
	//	carmen_offroad_planner_feedback_t planner_feedback = astar.carmen_offroad_planner_update_goal(&point, obstacle_distance_grid_map, path_smoothing_obstacles_safe_distance);
	//	printf("%s\n", print_offroad_planner_feedback(planner_feedback));
	//	astar.carmen_offroad_planner_update_goal(&point, obstacle_distance_grid_map, path_smoothing_obstacles_safe_distance);
}

static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	behavior_selector_task = &(msg->task);

	if ((msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_WITHIN_NARROW_PASSAGE) ||
		(msg->task == BEHAVIOR_SELECTOR_MOVE_TO_ENGAGE_POSE))
		robot_config.model_predictive_planner_obstacles_safe_distance = astar_config.oa_obstacles_safe_distance;
	else
		robot_config.model_predictive_planner_obstacles_safe_distance = original_model_predictive_planner_obstacles_safe_distance;

	//carmen_collision_detection_set_robot_collision_config(msg->geometry_model);
}

static void
behavior_selector_set_task_message_handler(carmen_behavior_selector_set_task_message *msg)
{
	behavior_selector_task = &(msg->task);
}

static void
behavior_selector_set_algorithm_message_handler(carmen_behavior_selector_set_algorithm_message *msg)
{
	behavior_selector_task = &(msg->task);
}

static void
rddf_annotation_message_handler(carmen_rddf_annotation_message *message)
{
	last_rddf_annotation_message = message;
}

static void
offroad_planner_shutdown(int signal)
{
	static int done = 0;

	if (!done)
	{
		//		save_obstacle_distance_map(obstacle_distance_grid_map);
		carmen_ipc_disconnect();
		printf("Disconnected from IPC. signal = %d\n", signal);
		done = 1;
	}
	exit(0);
}

void timer_handler()
{
	if (kbhit() != 0)
	{
		char c = fgetc(stdin);

		switch (c)
		{
		case 'q':
			g_u1 += 0.1;
			if (g_u1 > MAX_U1)
				g_u1 = MAX_U1;
			break;
		case 'a':
			g_u1 -= 0.1;
			if (g_u1 < 0.01)
				g_u1 = 0.01;
			break;
		case 'w':
			g_u2 += 0.1;
			if (g_u2 > MAX_U2)
				g_u2 = MAX_U2;
			break;
		case 's':
			g_u2 -= 0.1;
			if (g_u2 < 0.01)
				g_u2 = 0.01;
			break;
		case 'e':
			g_k += 0.1;
			if (g_k > MAX_K)
				g_k = MAX_K;
			break;
		case 'd':
			g_k -= 0.1;
			if (g_k < 0.01)
				g_k = 0.01;
			break;
		case 'r':
			g_c += 1;
			if (g_c > 2)
				g_c = 2;
			break;
		case 'f':
			g_c -= 1;
			if (g_c < 1)
				g_c = 1;
			break;
		case 't':
			g_c2 += 0.1;
			break;
		case 'g':
			g_c2 -= 0.1;
			break;
		}

		carmen_robot_and_trailers_traj_point_t goal_pose = robot_position;
		goal_pose.x += 10.0;
		int merge_pose_in_current_route_index = 0;
		offroad_planner_plan_t plan;
		carmen_offroad_planner_feedback_t feedback = carmen_offroad_planner_plan(plan, robot_position, goal_pose, merge_pose_in_current_route_index, NULL);
		publish_offroad_plan(plan.path.points, plan.path.length, feedback, merge_pose_in_current_route_index, robot_position);
		free(plan.path.points);

		printf("%lf %lf %lf %d\n", g_u1, g_u2, g_k, g_c);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

void subscribe_messages()
{
	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t)carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behaviour_selector_subscribe_compact_lane_contents_message(NULL, (carmen_handler_t)carmen_behaviour_selector_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t)(carmen_localize_ackerman_globalpos_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t)(carmen_rddf_play_end_point_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_route_planner_subscribe_road_network_message(NULL, (carmen_handler_t)carmen_route_planner_road_network_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t)behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_subscribe_message((char *)CARMEN_BEHAVIOR_SELECTOR_SET_TASK_NAME, (char *)CARMEN_BEHAVIOR_SELECTOR_SET_TASK_FMT,
							NULL, sizeof(carmen_behavior_selector_set_task_message), (carmen_handler_t)behavior_selector_set_task_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_subscribe_message((char *)CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME, (char *)CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_FMT,
							NULL, sizeof(carmen_behavior_selector_set_algorithm_message), (carmen_handler_t)behavior_selector_set_algorithm_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t)rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);
}

static void
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
		{
			{(char *)"robot", (char *)"max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
			{(char *)"robot", (char *)"max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
			{(char *)"robot", (char *)"min_approach_dist", CARMEN_PARAM_DOUBLE, &robot_config.approach_dist, 1, NULL},
			{(char *)"robot", (char *)"min_side_dist", CARMEN_PARAM_DOUBLE, &robot_config.side_dist, 1, NULL},
			{(char *)"robot", (char *)"length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
			{(char *)"robot", (char *)"width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
			{(char *)"robot", (char *)"maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
			{(char *)"robot", (char *)"maximum_acceleration_reverse", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_reverse, 1, NULL},
			{(char *)"robot", (char *)"maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward, 1, NULL},
			{(char *)"robot", (char *)"maximum_deceleration_reverse", CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_reverse, 1, NULL},
			{(char *)"robot", (char *)"reaction_time", CARMEN_PARAM_DOUBLE, &robot_config.reaction_time, 0, NULL},
			{(char *)"robot", (char *)"distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
			{(char *)"robot", (char *)"maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate, 1, NULL},
			{(char *)"robot", (char *)"distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
			{(char *)"robot", (char *)"distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels, 1, NULL},
			{(char *)"robot", (char *)"distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_wheels, 1, NULL},
			{(char *)"robot", (char *)"desired_decelaration_forward", CARMEN_PARAM_DOUBLE, &robot_config.desired_decelaration_forward, 1, NULL},
			{(char *)"robot", (char *)"desired_decelaration_reverse", CARMEN_PARAM_DOUBLE, &robot_config.desired_decelaration_reverse, 1, NULL},
			{(char *)"robot", (char *)"desired_acceleration", CARMEN_PARAM_DOUBLE, &robot_config.desired_acceleration, 1, NULL},
			{(char *)"robot", (char *)"desired_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.desired_steering_command_rate, 1, NULL},
			{(char *)"robot", (char *)"understeer_coeficient", CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient, 1, NULL},
			//			{(char *) "robot",				(char *) "max_centripetal_acceleration", CARMEN_PARAM_DOUBLE, &robot_max_centripetal_acceleration, 1, NULL},
			//			{(char *) "robot",				(char *) "max_velocity_reverse",		 CARMEN_PARAM_DOUBLE, &param_max_vel_reverse, 1, NULL},
			{(char *)"semi_trailer", (char *)"initial_type", CARMEN_PARAM_INT, &(semi_trailer_config.num_semi_trailers), 0, NULL},
			{(char *)"navigator", (char *)"goal_size", CARMEN_PARAM_DOUBLE, &nav_config.goal_size, 1, NULL},
			{(char *)"navigator", (char *)"waypoint_tolerance", CARMEN_PARAM_DOUBLE, &nav_config.waypoint_tolerance, 1, NULL},
			{(char *)"navigator", (char *)"goal_theta_tolerance", CARMEN_PARAM_DOUBLE, &nav_config.goal_theta_tolerance, 1, NULL},
			{(char *)"navigator", (char *)"map_update_radius", CARMEN_PARAM_DOUBLE, &nav_config.map_update_radius, 1, NULL},
			{(char *)"navigator", (char *)"map_update_num_laser_beams", CARMEN_PARAM_INT, &nav_config.num_lasers_to_use, 1, NULL},
			{(char *)"navigator", (char *)"map_update_obstacles", CARMEN_PARAM_ONOFF, &nav_config.map_update_obstacles, 1, NULL},
			{(char *)"navigator", (char *)"map_update_freespace", CARMEN_PARAM_ONOFF, &nav_config.map_update_freespace, 1, NULL},
			{(char *)"navigator", (char *)"replan_frequency", CARMEN_PARAM_DOUBLE, &nav_config.replan_frequency, 1, NULL},
			{(char *)"navigator", (char *)"dont_integrate_odometry", CARMEN_PARAM_ONOFF, &nav_config.dont_integrate_odometry, 1, NULL},
			{(char *)"navigator", (char *)"plan_to_nearest_free_point", CARMEN_PARAM_ONOFF, &nav_config.plan_to_nearest_free_point, 1, NULL},
			{(char *)"offroad", (char *)"planner_state_map_resolution", CARMEN_PARAM_DOUBLE, &astar_config.state_map_resolution, 1, NULL},
			{(char *)"offroad", (char *)"planner_state_map_theta_resolution", CARMEN_PARAM_INT, &astar_config.state_map_theta_resolution, 1, NULL},
			{(char *)"offroad", (char *)"planner_state_map_beta_resolution", CARMEN_PARAM_INT, &astar_config.state_map_beta_resolution, 1, NULL},
			{(char *)"offroad", (char *)"planner_precomputed_cost_size", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_size, 1, NULL},
			{(char *)"offroad", (char *)"planner_precomputed_cost_theta_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_theta_size, 1, NULL},
			{(char *)"offroad", (char *)"planner_precomputed_cost_resolution", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_resolution, 1, NULL},
			{(char *)"offroad", (char *)"planner_precomputed_cost_file_name", CARMEN_PARAM_STRING, &astar_config.precomputed_cost_file_name, 1, NULL},
			{(char *)"offroad", (char *)"planner_use_matrix_cost_heuristic", CARMEN_PARAM_ONOFF, &astar_config.use_matrix_cost_heuristic, 1, NULL},

			{(char *)"offroad", (char *)"planner_precomputed_cost_trailer_size", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_trailer_size, 1, NULL},
			{(char *)"offroad", (char *)"planner_precomputed_cost_trailer_theta_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_trailer_theta_size, 1, NULL},
			{(char *)"offroad", (char *)"planner_precomputed_cost_trailer_beta_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_trailer_beta_size, 1, NULL},
			{(char *)"offroad", (char *)"planner_precomputed_cost_trailer_resolution", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_trailer_resolution, 1, NULL},
			{(char *)"offroad", (char *)"planner_precomputed_cost_trailer_file_name", CARMEN_PARAM_STRING, &astar_config.precomputed_cost_trailer_file_name, 1, NULL},
			{(char *)"offroad", (char *)"planner_use_matrix_cost_trailer_heuristic", CARMEN_PARAM_ONOFF, &astar_config.use_matrix_cost_trailer_heuristic, 1, NULL},

			{(char *)"offroad", (char *)"planner_min_dist_motion_change", CARMEN_PARAM_DOUBLE, &astar_config.min_dist_motion_change, 1, NULL},
			{(char *)"offroad", (char *)"planner_path_smoothing_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &astar_config.path_smoothing_obstacles_safe_distance, 1, NULL},
			{(char *)"offroad", (char *)"planner_max_phi_multiplier", CARMEN_PARAM_DOUBLE, &max_phi_multiplier, 1, NULL},
			{(char *)"offroad", (char *)"planner_smooth_path", CARMEN_PARAM_ONOFF, &astar_config.smooth_path, 1, NULL},
			{(char *)"offroad", (char *)"planner_penalties_w1", CARMEN_PARAM_DOUBLE, &astar_config.penalties_w1, 1, NULL},
			{(char *)"offroad", (char *)"planner_penalties_w2", CARMEN_PARAM_DOUBLE, &astar_config.penalties_w2, 1, NULL},
			{(char *)"offroad", (char *)"planner_penalties_w3", CARMEN_PARAM_DOUBLE, &astar_config.penalties_w3, 1, NULL},
			{(char *)"offroad", (char *)"planner_penalties_w4", CARMEN_PARAM_DOUBLE, &astar_config.penalties_w4, 1, NULL},
			{(char *)"offroad", (char *)"planner_penalties_w5", CARMEN_PARAM_DOUBLE, &astar_config.penalties_w5, 1, NULL},
			{(char *)"offroad", (char *)"planner_min_distance_to_lane", CARMEN_PARAM_DOUBLE, &astar_config.min_distance_to_lane, 1, NULL},
			{(char *)"offroad", (char *)"planner_use_rs_with_trailer", CARMEN_PARAM_ONOFF, &astar_config.use_rs_with_trailer, 1, NULL},
			{(char *)"offroad", (char *)"planner_smoothness_w1", CARMEN_PARAM_DOUBLE, &GlobalState::w1, 1, NULL},
			{(char *)"offroad", (char *)"planner_smoothness_w2", CARMEN_PARAM_DOUBLE, &GlobalState::w2, 1, NULL},
			{(char *)"offroad", (char *)"planner_smoothness_w3", CARMEN_PARAM_DOUBLE, &GlobalState::w3, 1, NULL},
			{(char *)"offroad", (char *)"planner_smoothness_w4", CARMEN_PARAM_DOUBLE, &GlobalState::w4, 1, NULL},
			{(char *)"offroad", (char *)"planner_smoothness_w5", CARMEN_PARAM_DOUBLE, &GlobalState::w5, 1, NULL},
			{(char *)"offroad", (char *)"planner_eliminate_path_follower", CARMEN_PARAM_ONOFF, &GlobalState::eliminate_path_follower, 1, NULL},
			{(char *)"obstacle_avoider", (char *)"obstacles_safe_distance", CARMEN_PARAM_ONOFF, &astar_config.oa_obstacles_safe_distance, 1, NULL},
			{(char *)"model", (char *)"predictive_planner_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &robot_config.model_predictive_planner_obstacles_safe_distance, 1, NULL},
			{(char *)"behavior_selector", (char *)"distance_between_waypoints", CARMEN_PARAM_DOUBLE, &GlobalState::distance_between_waypoints, 1, NULL},

			{(char *)"obstacle", (char *)"distance_mapper_distance_map_huge_distance", CARMEN_PARAM_DOUBLE, &distance_map_huge_distance, 1, NULL},
		};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	astar_config.goal_constraints.goal_achieved_distance = MAX_POLY_DISTANCE;
	astar_config.goal_constraints.goal_achieved_theta = MAX_POLY_THETA_DIFF;
	astar_config.goal_constraints.goal_achieved_trailer_theta = MAX_POLY_BETA_DIFF;
	astar_config.max_change_direction = -1; // -1 to without limits
	astar_config.max_reed_shepp_change_direction = -1;
	astar_config.use_reed_shepp = 1;
	astar_config.max_reed_shepp_distance_backwards = -1.0;
	astar_config.radius_circle_to_ignore_obstacles_from_final_goal = -1.0;

	astar_config.expansion_phi_resolution = 3;
	astar_config.expansion_v_step = EXPAND_NODES_V;

	carmen_param_allow_unfound_variables(1);

	double semi_trailer_d_multiplier = 1.0;

	carmen_param_t param_optional_list[] =
		{
			{(char *)"offroad", (char *)"planner_semi_trailer_solver_type", CARMEN_PARAM_INT, &semi_trailer_solver_type, 0, NULL},
			{(char *)"offroad", (char *)"planner_min_resolution_path", CARMEN_PARAM_DOUBLE, &param_offroad_min_resolution_path, 0, NULL},
			{(char *)"offroad", (char *)"planner_max_reed_shepp_distance", CARMEN_PARAM_DOUBLE, &param_offroad_max_reed_shepp_distance, 0, NULL},
			{(char *)"offroad", (char *)"planner_max_reed_shepp_distance_backwards", CARMEN_PARAM_DOUBLE, &astar_config.max_reed_shepp_distance_backwards, 0, NULL},
			{(char *)"offroad", (char *)"planner_goal_achieved_distance", CARMEN_PARAM_DOUBLE, &astar_config.goal_constraints.goal_achieved_distance, 0, NULL},
			{(char *)"offroad", (char *)"planner_goal_achieved_theta", CARMEN_PARAM_DOUBLE, &astar_config.goal_constraints.goal_achieved_theta, 0, NULL},
			{(char *)"offroad", (char *)"planner_goal_achieved_trailer_theta", CARMEN_PARAM_DOUBLE, &astar_config.goal_constraints.goal_achieved_trailer_theta, 0, NULL},
			{(char *)"offroad", (char *)"planner_max_change_direction", CARMEN_PARAM_INT, &astar_config.max_change_direction, 0, NULL},
			{(char *)"offroad", (char *)"planner_max_reed_shepp_change_direction", CARMEN_PARAM_INT, &astar_config.max_reed_shepp_change_direction, 0, NULL},
			{(char *)"offroad", (char *)"planner_use_reed_shepp", CARMEN_PARAM_INT, &astar_config.use_reed_shepp, 0, NULL},
			{(char *)"offroad", (char *)"planner_obstacle_distance_map_on", CARMEN_PARAM_INT, &offroad_planner_obstacle_distance_map_on, 1, NULL},
			{(char *)"offroad", (char *)"planner_semi_traile_d_multiplier", CARMEN_PARAM_DOUBLE, &semi_trailer_d_multiplier, 1, NULL},
			// {(char *) "offroad",	 (char *) "planner_expansion_phi_resolution", CARMEN_PARAM_INT, &astar_config.expansion_phi_resolution, 0, NULL},
			// {(char *) "offroad",	 (char *) "planner_expansion_v_step", CARMEN_PARAM_DOUBLE, &astar_config.expansion_v_step, 0, NULL},
			{(char *)"offroad", (char *)"planner_radius_circle_to_ignore_obstacles_from_final_goal", CARMEN_PARAM_DOUBLE, &astar_config.radius_circle_to_ignore_obstacles_from_final_goal, 0, NULL},
			{(char *)"commandline", (char *)"cache_writing", CARMEN_PARAM_ONOFF, &cache_writing, 0, NULL},
			{(char *)"commandline", (char *)"cache_filename", CARMEN_PARAM_STRING, &cache_filename, 0, NULL},
			{(char *)"commandline", (char *)"time_mult", CARMEN_PARAM_INT, &time_mult, 0, NULL},
			{(char *)"commandline", (char *)"ignore_check_ipc", CARMEN_PARAM_ONOFF, &param_ignore_check_ipc, 0, NULL},
		};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	GlobalState::robot_config = robot_config;
	robot_config.max_phi *= max_phi_multiplier;

	if (astar_config.use_matrix_cost_heuristic)
		alloc_cost_map();
	if (astar_config.use_matrix_cost_trailer_heuristic)
		alloc_trailer_cost_map();

	if (semi_trailer_config.num_semi_trailers > 0)
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc, argv, semi_trailer_config.num_semi_trailers);
	else
		astar_config.state_map_beta_resolution = 1;

	robot_config.model_predictive_planner_obstacles_safe_distance *= 1.2;
	original_model_predictive_planner_obstacles_safe_distance = robot_config.model_predictive_planner_obstacles_safe_distance;

	// Precisa ser trocado quando passarmos a mandar a quantidade de trailers usado em alguma mensagem, mas precisamos identificar o que exatamente vai informar o número de trailers.
	number_of_trailers_for_path_planning = semi_trailer_config.num_semi_trailers;

	if (astar_config.goal_constraints.goal_achieved_theta != MAX_POLY_THETA_DIFF)
	{
		astar_config.goal_constraints.goal_achieved_theta = carmen_degrees_to_radians(astar_config.goal_constraints.goal_achieved_theta);
	}

	if (astar_config.goal_constraints.goal_achieved_trailer_theta != MAX_POLY_BETA_DIFF)
	{
		astar_config.goal_constraints.goal_achieved_trailer_theta = carmen_degrees_to_radians(astar_config.goal_constraints.goal_achieved_trailer_theta);
	}

	// Can't allow this functionalitie with trailer. Can't guarantee the trailers not colliding
	//astar_config.radius_circle_to_ignore_obstacles_from_final_goal = behavior_selector_radius_circle_to_ignore_obstacles_from_final_goal_sanity_checkage(astar_config.radius_circle_to_ignore_obstacles_from_final_goal, semi_trailer_config.num_semi_trailers);

	if (astar_config.radius_circle_to_ignore_obstacles_from_final_goal > robot_config.distance_between_front_and_rear_axles)
	{
		carmen_die("param: offroad_planner_radius_circle_to_ignore_obstacles_from_final_goal is too high (%lf > %lf), it could be dangerous using it like this\nYou can change it in [%s:%d]\n", astar_config.radius_circle_to_ignore_obstacles_from_final_goal, robot_config.distance_between_front_and_rear_axles, __FILE__, __LINE__);
	}

	if (semi_trailer_config.num_semi_trailers > 0)
		semi_trailer_config.semi_trailers[0].d *= semi_trailer_d_multiplier;
}

int check_singular_connection(char *module_name_param)
{
	char buf[512];
	snprintf(buf, sizeof(buf), "pidof -s %s", module_name_param);
	FILE *cmd_pipe = popen(buf, "r");
	fgets(buf, 512, cmd_pipe);
	pid_t pid = strtoul(buf, NULL, 10);
	pclose(cmd_pipe);

	char module_name[2000];
	snprintf(module_name, sizeof(module_name), "%s-%d", module_name_param, pid);

	int ipc_returned = IPC_isModuleConnected(module_name);

	return (ipc_returned);
}

void check_ipc_connections(void)
{
	if (check_singular_connection((char *)"route_planner") == 0)
	{
		carmen_die("\nThe module '%sroute_planner%s' is not on! Start '%sroute_planner%s' before initializing offroad_planner!\nIf you are sure this is a mistake, you can start offroad with flag ignore_check_ipc (./offroad_planner -ignore_check_ipc on)\n", carmen_red_code, carmen_normal_code, carmen_red_code, carmen_normal_code);
	}
}

int main(int argc, char **argv)
{
	argc_global = argc;
	argv_global = argv;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	subscribe_messages();
	signal(SIGINT, offroad_planner_shutdown);
	if (param_ignore_check_ipc == 0)
	{
		sleep(2);
		check_ipc_connections();
	}

#ifdef TEST_RUSSO
	nonblock(NB_ENABLE);
	double timer_period = 0.1;
	carmen_ipc_addPeriodicTimer(timer_period, (TIMER_HANDLER_TYPE)timer_handler, NULL);
#endif
	carmen_ipc_dispatch();

#ifdef TEST_RUSSO
	nonblock(NB_DISABLE);
#endif

	return (0);
}