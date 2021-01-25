#include "path_planner_astar.h"

extern carmen_robot_ackerman_config_t robot_config;
extern carmen_path_planner_astar_t astar_config;

extern carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map;
extern carmen_map_p map_occupancy;

extern nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map;
extern int use_nonholonomic_heuristic_cost_map;
extern int heuristic_number;

extern int grid_state_map_x_size;
extern int grid_state_map_y_size;

cv::Mat map_image;
int expanded_nodes_by_astar;

char expansion_tree_file_name[] = "Expansion_illustration_0.png";


static int
sign(double a)
{
	if (a >= 0.0)
		return 1;
	else
		return -1;
}


void
override_initial_and_goal_poses(carmen_point_t &initial_pose, carmen_point_t &goal_pose)
{
	if (RUN_EXPERIMENT > 4 || RUN_EXPERIMENT < 0)
	{
		printf("Invalid RUN_EXPERIMENT value. Error in line %d of the file %s\n", __LINE__, __FILE__);
		exit(1);
	}

	initial_pose = experiments_ICRA[RUN_EXPERIMENT -1][0];
	goal_pose = experiments_ICRA[RUN_EXPERIMENT -1][1];
}


void
draw_map(carmen_obstacle_distance_mapper_map_message *distance_map, cv::Mat &map_image)
{
	if (distance_map->complete_x_offset == NULL)
		return;

	unsigned int width = distance_map->config.x_size;
	unsigned int height = distance_map->config.y_size;
	unsigned int size = width * height;
	unsigned char map[3 * size];

	for (unsigned int i = 0; i < size; ++i)
	{
		unsigned int row = (height - 1) - i % height;

		unsigned int col = i / height;

		unsigned int index = row * width + col;

		if (0.0 == distance_map->complete_x_offset[i] && 0.0 == distance_map->complete_y_offset[i])
		{
			map[index*3] = 0;
			map[index*3+1] = 0;
			map[index*3+2] = 0;
		}
		else
		{
			map[index*3] = 255;
			map[index*3+1] = 255;
			map[index*3+2] = 255;
		}
	}

    cv::Mat img(width, height, CV_8UC3, map);
    map_image = img.clone();
}


void
draw_state_in_opencv_image(state_node* current, carmen_map_config_t config, cv::Scalar color, cv::Mat map_image)
{
	if (current->parent == NULL)
		return;

	int img_x = (double) (current->pose.x - config.x_origin) / config.resolution;
	int img_y = (double) (current->pose.y - config.y_origin) / config.resolution;
	int parent_x = (double) (current->parent->pose.x - config.x_origin) / config.resolution;
	int parent_y = (double) (current->parent->pose.y - config.y_origin) / config.resolution;

	cv::line(map_image, cv::Point(img_x, config.y_size - 1 - img_y), cv::Point(parent_x, config.y_size - 1 - parent_y), color, 1, cv::LINE_8);
}


void
draw_point_on_map_img(double x, double y, carmen_map_config_t config, cv::Scalar color, cv::Mat map_image)
{
	int img_x = (double) (x - config.x_origin) / config.resolution;
	int img_y = (double) (y - config.y_origin) / config.resolution;
	cv::circle(map_image, cv::Point(img_x, config.y_size - 1 - img_y), 3, color, -1, 8);
}


void
draw_point_in_opencv_image(carmen_ackerman_traj_point_t current, carmen_ackerman_traj_point_t parent, carmen_map_config_t config, cv::Scalar color, int size = 1)
{
		int img_x = (double) (current.x - config.x_origin) / config.resolution;
		int img_y = (double) (current.y - config.y_origin) / config.resolution;
		int parent_x = (double) (parent.x - config.x_origin) / config.resolution;
		int parent_y = (double) (parent.y - config.y_origin) / config.resolution;

		cv::line(map_image, cv::Point(img_x, config.y_size - 1 - img_y), cv::Point(parent_x, config.y_size - 1 - parent_y), color, size, cv::LINE_8);
}


int
get_index_of_nearest_pose_in_path(carmen_ackerman_traj_point_t *path, carmen_point_t globalpos, int path_length)
{
	int nearest_pose_index = 0;
	double min_dist = DIST2D(path[nearest_pose_index], globalpos);
	for (int i = 1; i < path_length; i++)
	{
		double distance = DIST2D(path[i], globalpos);
		if (distance < min_dist)
		{
			min_dist = distance;
			nearest_pose_index = i;
		}
	}

	return (nearest_pose_index);
}


carmen_ackerman_traj_point_t *
get_poses_back(carmen_ackerman_traj_point_t *path, int nearest_pose_index)
{
	carmen_ackerman_traj_point_t *poses_back = (carmen_ackerman_traj_point_t *) malloc((nearest_pose_index + 1) * sizeof(carmen_ackerman_traj_point_t));
	for (int i = 0; i < (nearest_pose_index + 1); i++)
		poses_back[i] = path[nearest_pose_index - i];

	return (poses_back);
}


void
calculate_phi_ahead(std::vector<carmen_ackerman_traj_point_t> &path, int num_poses)
{
	double L = robot_config.distance_between_front_and_rear_axles;

	for (int i = 0; i < (num_poses - 1); i++)
	{
//		if (sign(path[i].v) == sign(path[i+1].v))
		double delta_theta;
		if (path[i].v >= 0)
			delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
		else
			delta_theta = carmen_normalize_theta(path[i].theta - path[i + 1].theta);

			double l = DIST2D(path[i], path[i + 1]);
			if (l < 0.01)
			{
				path[i].phi = 0.0;
				continue;
			}
			path[i].phi = atan(L * (delta_theta / l));

	}

	for (int i = 1; i < (num_poses - 1); i++)
	{
		path[i].phi = carmen_clamp(-0.5227, ((path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0), 0.5227);
	}
}


void
compute_theta(std::vector<carmen_ackerman_traj_point_t> &path, int num_poses)
{
	for (int i = 1; i < (num_poses - 1); i++)
	{
		// Sem essa condição, os thetas das poses que mudam a direção de movimento ficam errados
		if (sign(path[i].v) == path[i-1].v)
		{
			if (path[i-1].v >= 0.0)
				path[i-1].theta = carmen_normalize_theta(atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x));
			else
				path[i-1].theta = carmen_normalize_theta(atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x)+ M_PI);
		}

	}

	if (num_poses > 1)
		path[num_poses - 1].theta = path[num_poses - 2].theta;
}


void
calculate_theta_and_phi(std::vector<carmen_ackerman_traj_point_t> &poses_ahead, int num_poses_ahead)
{
	compute_theta(poses_ahead, num_poses_ahead);
	calculate_phi_ahead(poses_ahead, num_poses_ahead);
}


int
set_anchor(param_t *params)
{
	int cont = 4;
	params->anchor_points[0] = 1;
	params->anchor_points[params->path_size - 1] = 1;
	for (int i = 1; i < (params->path_size - 1); i++)
	{
		if (sign(params->points[i].v) != sign(params->points[i+1].v))
			params->anchor_points[i] = 1;
		else
		{
			params->anchor_points[i] = 0;
			cont+=2;
		}
	}
	return cont;
}


double
my_f(const gsl_vector *v, void *params)
{

	double dmax = 1.0; // escolher um valor melhor
	double kmax = 0.178571429;

	double obstacle_cost = 0.0;
	double curvature_cost = 0.0;
	double smoothness_cost = 0.0;
	double x, y, obst_x, obst_y, distance, delta_phi;
	double x_i, y_i, x_next, y_next, x_prev, y_prev, displacement;

	double curvature_term;

	param_t *param = (param_t*) params;

	int j = 0;
	for (int i = 0; i < param->path_size; i++)
	{
		if (!param->anchor_points[i])
		{
			param->points[i].x = gsl_vector_get(v, j++);
			param->points[i].y = gsl_vector_get(v, j++);
		}
	}

	for (int i = 1; i <(param->path_size - 1); i++)
	{
		if (!param->anchor_points[i])
		{

			x_i = param->points[i].x;
			y_i = param->points[i].y;
			x_next = param->points[i+1].x;
			y_next = param->points[i+1].y;
			x_prev = param->points[i-1].x;
			y_prev = param->points[i-1].y;

			carmen_ackerman_traj_point_t delta_i = DELTA2D(param->points[i], param->points[i - 1]);
			carmen_ackerman_traj_point_t delta_i_1 = DELTA2D(param->points[i+1], param->points[i]);
			carmen_ackerman_traj_point_t delta = DELTA2D(delta_i_1, delta_i);
			smoothness_cost +=  DOT2D(delta, delta);

			carmen_ackerman_traj_point_t current;
			current.x = x_i;
			current.y = y_i;

			if (param->points[i].v >= 0)
				current.theta = carmen_normalize_theta(atan2(y_next - y_i, x_next - x_i));
			else
				current.theta = carmen_normalize_theta(atan2(y_next - y_i, x_next - x_i) + M_PI);

			distance = carmen_obstacle_avoider_car_distance_to_nearest_obstacle(current, obstacle_distance_grid_map);

			if (distance <= dmax)
				obstacle_cost += (dmax - distance) * (dmax - distance) * (dmax - distance);

			if (param->points[i].v >= 0)
				delta_phi = abs(carmen_normalize_theta(atan2(y_next - y_i, x_next - x_i)) - carmen_normalize_theta(atan2(y_i - y_prev, x_i - x_prev)));
			else
				delta_phi = abs(carmen_normalize_theta(atan2(y_i - y_next, x_i - x_next)) - carmen_normalize_theta(atan2(y_prev - y_i, x_prev - x_i)));

//			delta_phi = abs(atan2(y_next - y_i, x_next - x_i) - atan2(y_i - y_prev, x_i - x_prev));

			if (delta_phi > 0)
				delta_phi = delta_phi / sqrt(DOT2D(delta_i, delta_i));

			if (delta_phi > kmax)
			{
				curvature_cost += pow(delta_phi - kmax, 2) * (delta_phi - kmax);
			}

		}
	}


	obstacle_cost = OBSTACLE_WEIGHT * obstacle_cost;
	curvature_cost = CURVATURE_WEIGHT * curvature_cost;
	smoothness_cost = SMOOTHNESS_WEIGHT * smoothness_cost;

	return obstacle_cost + curvature_cost + smoothness_cost;
}


double
single_point_my_f(carmen_ackerman_traj_point_t i, carmen_ackerman_traj_point_t i_prev, carmen_ackerman_traj_point_t i_next)
{

	double dmax = 1.0;
	double kmax = 0.178571429;

	double obstacle_cost = 0.0;
	double curvature_cost = 0.0;
	double smoothness_cost = 0.0;
	double distance, delta_phi;

	double curvature_term;
	carmen_ackerman_traj_point_t delta_i = DELTA2D(i, i_prev);
	carmen_ackerman_traj_point_t delta_i_1 = DELTA2D(i_next, i);
	carmen_ackerman_traj_point_t delta = DELTA2D(delta_i_1, delta_i);
	smoothness_cost +=  DOT2D(delta, delta);

	if (i.v >= 0)
		i.theta = carmen_normalize_theta(atan2(i_next.y - i.y, i_next.x - i.x));
	else
		i.theta = carmen_normalize_theta(atan2(i_next.y - i.y, i_next.x - i.x) + M_PI);

	distance = carmen_obstacle_avoider_car_distance_to_nearest_obstacle(i, obstacle_distance_grid_map);

	if (distance <= dmax)
		obstacle_cost += (dmax - distance) * (dmax - distance) * (dmax - distance);

	if (i.v >= 0)
	{
		double theta_1 = (atan2(i_next.y - i.y, i_next.x - i.x));
		double theta_2 = (atan2(i.y - i_prev.y, i.x - i_prev.x));
		delta_phi = abs(theta_1 - theta_2);
		delta_phi = abs(carmen_normalize_theta(atan2(i_next.y - i.y, i_next.x - i.x)) - carmen_normalize_theta(atan2(i.y - i_prev.y, i.x - i_prev.x)));
	}
	else
	{
		double theta_1 = (atan2(i.y - i_next.y, i.x - i_next.x));
		double theta_2 = (atan2(i_prev.y - i.y, i_prev.x - i.x));
		delta_phi = abs(theta_1 - theta_2);
	}


	if (delta_phi > 0)
		delta_phi = delta_phi / sqrt(DOT2D(delta_i, delta_i));

	if (delta_phi > kmax)
		curvature_cost += pow(delta_phi - kmax, 2) * (delta_phi - kmax);

	obstacle_cost = OBSTACLE_WEIGHT * obstacle_cost;
	curvature_cost = CURVATURE_WEIGHT * curvature_cost;
	smoothness_cost = SMOOTHNESS_WEIGHT * smoothness_cost;

	return obstacle_cost + curvature_cost + smoothness_cost;
}


void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double df_dx, df_dy, x, y, obst_x, obst_y, distance;
	double h = 0.00005;
	double f_x;
	param_t *param = (param_t *) params;
	gsl_vector *x_h = gsl_vector_alloc(param->problem_size);
	gsl_vector_memcpy(x_h, v);

	carmen_ackerman_traj_point_t temp_i;
	carmen_ackerman_traj_point_t temp_i_next;
	carmen_ackerman_traj_point_t temp_i_prev;


	int j = 0;
	for (int i = 0; i < param->path_size; i++)
	{
		if (!param->anchor_points[i])
		{
			param->points[i].x = gsl_vector_get(v, j++);
			param->points[i].y = gsl_vector_get(v, j++);
		}
	}

	j = 0;
	for (int i = 1; i < param->path_size - 1; i++)
	{
		if (!param->anchor_points[i])
		{

			//x
			temp_i = param->points[i];
			temp_i_prev = param->points[i-1];
			temp_i_next = param->points[i+1];

			f_x = single_point_my_f(temp_i, temp_i_prev, temp_i_next);
			temp_i.x += h;
			double f_x_h = single_point_my_f(temp_i, temp_i_prev, temp_i_next);
			double d_f_x_h = (f_x_h - f_x)/ h;
			gsl_vector_set(df, j, d_f_x_h);
			++j;

			//y
			temp_i = param->points[i];
			f_x = single_point_my_f(temp_i, temp_i_prev, temp_i_next);
			temp_i.y += h;
			f_x_h = single_point_my_f(temp_i, temp_i_prev, temp_i_next);
			d_f_x_h = (f_x_h - f_x)/ h;
			gsl_vector_set(df, j, d_f_x_h);
			++j;

		}
	}
	gsl_vector_free(x_h);
}


void
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


int
smooth_rddf_using_conjugate_gradient(std::vector<carmen_ackerman_traj_point_t> &astar_path)
{

	int iter = 0;
	int status, i = 0, j = 0;
	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;
	gsl_vector *v;
	gsl_multimin_function_fdf my_func;

	int size = astar_path.size();

	if (size < 5)
		return (0);

	int *anchor_points = (int*)malloc(sizeof(int)*size);
	for (int i = 0; i < size; i++)
	{
		anchor_points[i] = 0;

	}

	param_t *param =(param_t*) malloc(sizeof(param_t));
	param->points = &(astar_path[0]);
	param->anchor_points = anchor_points;
	param->path_size = size;
	param->problem_size = set_anchor(param);

	my_func.n = param->problem_size;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
//	my_func.params = &path;
	my_func.params = param;
	v = gsl_vector_alloc (param->problem_size);
	static int count = 0;
	++count;

	for (i = 0, j = 0; i < (size); i++)
	{
		if (!param->anchor_points[i])
		{
//			printf("poses_ahead = %f %f \n",poses_ahead[i].x, poses_ahead[i].y );
			gsl_vector_set (v, j++, astar_path[i].x);
			gsl_vector_set (v, j++, astar_path[i].y);
		}
	}

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc (T, param->problem_size);
	gsl_multimin_fdfminimizer_set (s, &my_func, v, 0.01, 0.0001);  //(function_fdf, gsl_vector, step_size, tol)
	do
	{
		++iter;
		status = gsl_multimin_fdfminimizer_iterate (s);
		if ((status != GSL_SUCCESS) && (status != GSL_ENOPROG) && (status != GSL_CONTINUE))
		{
			printf("Minimizer failed, code = %d iter = %d\n", status, iter);
			return (2);
		}

		status = gsl_multimin_test_gradient (s->gradient, 0.001); //(gsl_vector, epsabs) and  |g| < epsabs
		// status == 0 (GSL_SUCCESS), if a minimum has been found
	} while ((status != GSL_SUCCESS) && (status != GSL_ENOPROG) && (iter < 250));

	for (i = 0, j = 0; i < (size); i++)
	{
		if (!param->anchor_points[i])
		{
			astar_path[i].x = gsl_vector_get (s->x, j++);
			astar_path[i].y = gsl_vector_get (s->x, j++);
//			printf("poses_ahead after smoothing = %f %f \n",poses_ahead[i].x, poses_ahead[i].y);
		}
	}

	calculate_theta_and_phi(astar_path, size);

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);
//	printf("Terminou a suavização\n");
#if DRAW_EXPANSION_TREE && 0
	for (int i = 1; i < size; i++)
		draw_point_in_opencv_image(astar_path[i], astar_path[i-1], obstacle_distance_grid_map->config, cv::Scalar(0,0,255));

	imwrite("Expansion_illustration.png", map_image);
#endif

	free(param->anchor_points);
	free(param);
//	cout << "Smoothing running time is "<<time_count.get_since()<<" seconds\n";
//	exit(3);
	return (1);
}


void
add_lanes(carmen_route_planner_road_network_message &route_planner_road_network_message,
		carmen_ackerman_traj_point_t *path_copy)
{
	int num_lanes = NUM_LANES;
	if (path_copy)
		++num_lanes;

    route_planner_road_network_message.number_of_nearby_lanes = num_lanes;
    route_planner_road_network_message.nearby_lanes_ids =  (int *) malloc(route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_indexes = (int *) malloc(route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_sizes = (int *) malloc(route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_size = num_lanes *
    		(route_planner_road_network_message.number_of_poses_back + route_planner_road_network_message.number_of_poses - 1);	// a primeira pose do poses e poses back eh igual
    route_planner_road_network_message.nearby_lanes = (carmen_ackerman_traj_point_t *) malloc(route_planner_road_network_message.nearby_lanes_size * sizeof(carmen_ackerman_traj_point_t));
    route_planner_road_network_message.traffic_restrictions =  (int *) malloc(route_planner_road_network_message.nearby_lanes_size * sizeof(int));

    // Coloca o rddf como a lane 0, isto eh, a rota escolhida
    route_planner_road_network_message.nearby_lanes_ids[0] = 0;
    route_planner_road_network_message.nearby_lanes_indexes[0] = 0;
    route_planner_road_network_message.nearby_lanes_sizes[0] = route_planner_road_network_message.number_of_poses_back + route_planner_road_network_message.number_of_poses - 1; // a primeira pose do poses e poses back eh igual
    int pose_i = 0;
    for (int i = route_planner_road_network_message.number_of_poses_back - 1; i > 0; i--)
	{
    	route_planner_road_network_message.nearby_lanes[pose_i] = route_planner_road_network_message.poses_back[i];
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_LEFT_WIDTH(0, LANE_WIDTH);
    	++pose_i;
	}
    for (int i = 0; i < route_planner_road_network_message.number_of_poses; i++)
	{
    	route_planner_road_network_message.nearby_lanes[pose_i] = route_planner_road_network_message.poses[i];
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_LEFT_WIDTH(0, LANE_WIDTH);
    	++pose_i;
	}
    if (path_copy)
    {
        route_planner_road_network_message.nearby_lanes_ids[1] = 1;
        route_planner_road_network_message.nearby_lanes_indexes[1] = pose_i;
        route_planner_road_network_message.nearby_lanes_sizes[1] = route_planner_road_network_message.number_of_poses_back + route_planner_road_network_message.number_of_poses - 1; // a primeira pose do poses e poses back eh igual
        for (int i = 0; i < pose_i; i++)
    	{
        	route_planner_road_network_message.nearby_lanes[pose_i + i] = path_copy[i];
        	route_planner_road_network_message.traffic_restrictions[pose_i + i] = ROUTE_PLANNER_SET_LANE_LEFT_WIDTH(0, LANE_WIDTH);
    	}
		free(path_copy);
   }
}


void
free_lanes(carmen_route_planner_road_network_message route_planner_road_network_message)
{
    free(route_planner_road_network_message.nearby_lanes_indexes);
    free(route_planner_road_network_message.nearby_lanes_sizes);
    free(route_planner_road_network_message.nearby_lanes_ids);
    free(route_planner_road_network_message.nearby_lanes);
    free(route_planner_road_network_message.traffic_restrictions);
}


int
get_astar_map_theta(double theta, double map_theta_size)
{
	theta = theta < 0 ? (2 * M_PI + theta) : theta;
	int resolution = (int) round(360/map_theta_size);

	return  (int)round((carmen_radians_to_degrees(theta) / resolution)) % (int)round(360 / resolution);
}


int
get_grid_state_map_x(double x, carmen_obstacle_distance_mapper_map_message* obstacle_distance_grid_map)
{
	return round((double) ((x - obstacle_distance_grid_map->config.x_origin) / astar_config.state_map_resolution));
}


int
get_grid_state_map_y(double y, carmen_obstacle_distance_mapper_map_message* obstacle_distance_grid_map)
{
	return round((double) ((y - obstacle_distance_grid_map->config.y_origin) / astar_config.state_map_resolution));
}


double
get_distance_map_x(int x, carmen_obstacle_distance_mapper_map_message* obstacle_distance_grid_map)
{
	return (double) (x * astar_config.state_map_resolution) + obstacle_distance_grid_map->config.x_origin;
}


double
get_distance_map_y(int y, carmen_obstacle_distance_mapper_map_message* obstacle_distance_grid_map)
{
	return (double) (y * astar_config.state_map_resolution) + obstacle_distance_grid_map->config.y_origin;
}


void
get_current_pos(state_node* current_node, int &x, int &y, int &theta, int &direction, carmen_obstacle_distance_mapper_map_message* obstacle_distance_grid_map)
{
	x = get_grid_state_map_x(current_node->pose.x, obstacle_distance_grid_map);
	y = get_grid_state_map_y(current_node->pose.y, obstacle_distance_grid_map);
	theta = get_astar_map_theta(current_node->pose.theta, astar_config.state_map_theta_resolution);
	direction = current_node->pose.r;
}


double
obstacle_distance(double x, double y, carmen_obstacle_distance_mapper_map_message* obstacle_distance_grid_map)
{
	if ( NULL == obstacle_distance_grid_map)
		exit(1);

    carmen_point_t p;
    p.x = x;
    p.y = y;
    return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&p, obstacle_distance_grid_map));
}


carmen_map_t *
copy_grid_mapping_to_map(carmen_map_t *map, carmen_mapper_map_message *grid_map)
{
	int i;

	if (!map)
	{
		map = (carmen_map_t *) malloc(sizeof(carmen_map_t));
		map->map = (double **) malloc(grid_map->config.x_size * sizeof(double *));
	}

	map->config = grid_map->config;
	map->complete_map = grid_map->complete_map;
	for (i = 0; i < map->config.x_size; i++)
		map->map[i] = map->complete_map + i * map->config.y_size;

	return (map);
}


int
is_valid_grid_value(int x, int y, double current_resolution)
{
	double map_x = (double) (x * current_resolution) + map_occupancy->config.x_origin;
	double map_y = (double) (y * current_resolution) + map_occupancy->config.y_origin;
//	printf("map_x = %f map_y = %f x_origin = %f y_origin = %f\n", map_x, map_y, map_occupancy->config.x_origin, map_occupancy->config.y_origin);
	carmen_point_t global_point_in_map_coords;
	// Move global path point coordinates to map coordinates
	global_point_in_map_coords.x = (map_x - map_occupancy->config.x_origin) / map_occupancy->config.resolution;
	global_point_in_map_coords.y = (map_y - map_occupancy->config.y_origin) / map_occupancy->config.resolution;
	// Transform coordinates to integer indexes
	int x_map_cell = (int) round(global_point_in_map_coords.x);
	int y_map_cell = (int) round(global_point_in_map_coords.y);
	// Os mapas de carmen sao orientados a colunas, logo a equacao eh como abaixo
	int index = y_map_cell + map_occupancy->config.y_size * x_map_cell;
	if (map_occupancy->complete_map[index] == -1)
	{
		 return 0;
	}
	return 1;
}


void
save_map(char *file_name, double *map, int x_size, int y_size)
{
	carmen_map_t out_map;

	double *map_copy = (double *) malloc(x_size * y_size * sizeof(double));
	memcpy((void *) map_copy, (void *) map, x_size * y_size);

	double min_value, max_value;
	min_value = 100000000.0;
	max_value = -100000000.0;
	for (int i = 0; i < x_size * y_size; i++)
	{
		double value = map[i];
		if (value != 10000.0)
		{
//			printf("Value = %f\n", value);
			if (value < min_value)
				min_value = value;
			if (value > max_value)
				max_value = value;
		}
//		else
//			printf("x %d, y %d, value %lf, x_size %d, y_size %d\n", i % x_size, i / x_size, value, x_size, y_size);
	}

//	printf("map name %s, min_value %lf, max_value %lf\n", file_name, min_value, max_value);
	for (int i = 0; i < x_size * y_size; i++)
	{
		double value = map[i];
//		if (value != -1.0)
//		{
//			value = (value - min_value) / (max_value - min_value);
//			if (value < 0.8)
//				value = 0.8;
//			value = (value - 0.8) / (1.0 - 0.8);
//		}
		map_copy[i] = value;
	}
	out_map.complete_map = map_copy;
	out_map.config.x_size = x_size;
	out_map.config.y_size = y_size;
	out_map.config.resolution = 0.2;
	out_map.config.map_name = (char *) "test";
	out_map.config.x_origin = 0.0;
	out_map.config.y_origin = 0.0;

	out_map.map = (double **) malloc(sizeof(double *) * x_size);
	carmen_test_alloc(out_map.map);
	for (int x = 0; x < out_map.config.x_size; x++)
		out_map.map[x] = &(out_map.complete_map[x * out_map.config.y_size]);

	carmen_grid_mapping_save_map(file_name, &out_map);

	free(out_map.map);
	free(map_copy);
}


void
copy_map(double *cost_map, std::vector<std::vector<double> > map, int x_size, int y_size)
{
	for (int x = 0; x < x_size; x++)
		for (int y = 0; y < y_size; y++)
			cost_map[x + y * x_size] = map[x][y];
}


void
copy_map(std::vector<std::vector<double> > &map, double *cost_map, int x_size, int y_size)
{
	for (int x = 0; x < x_size; x++)
		for (int y = 0; y < y_size; y++)
			map[x][y] = cost_map[x + y * x_size];
}


double *
get_goal_distance_map(carmen_point_t goal_pose, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	printf("Carregando mapa da heurística com obstáculos\n");
	grid_state_map_x_size = round((obstacle_distance_grid_map->config.x_size * obstacle_distance_grid_map->config.resolution) / astar_config.state_map_resolution);
	grid_state_map_y_size = round((obstacle_distance_grid_map->config.y_size * obstacle_distance_grid_map->config.resolution)/ astar_config.state_map_resolution);

	int x_size = grid_state_map_x_size;
	int y_size = grid_state_map_y_size;
	double *utility_map = (double *) calloc(x_size * y_size, sizeof(double));
	double *cost_map = (double *) calloc(x_size * y_size, sizeof(double));
	std::fill_n(cost_map, x_size *y_size, -1.0);
	double pos_x = 0.0;
	double pos_y = 0.0;
	for (int x = 0; x < x_size; x++)
	{
		for (int y = 0; y < y_size; y++)
		{
			pos_x = get_distance_map_x(x, obstacle_distance_grid_map);
			pos_y = get_distance_map_y(y, obstacle_distance_grid_map);
			if (is_valid_grid_value(x, y, astar_config.state_map_resolution) == 0 || obstacle_distance(pos_x, pos_y, obstacle_distance_grid_map) <= (astar_config.state_map_resolution + 0.1)){
				cost_map[y + x * y_size] = 1.0; // Espaco ocupado eh representado como 1.0
			}
		}
	}
	std::vector<std::vector<double> > map(x_size, std::vector <double>(y_size));
	copy_map(map, cost_map, x_size, y_size);
	Planning exact_euclidean_distance_to_goal;
	exact_euclidean_distance_to_goal.setMap(map);
	exact_euclidean_distance_to_goal.expandObstacles(0.5);
	copy_map(cost_map, exact_euclidean_distance_to_goal.getExpandedMap(), x_size, y_size);
	int goal_x = get_grid_state_map_x(goal_pose.x, obstacle_distance_grid_map);
	int goal_y = get_grid_state_map_y(goal_pose.y, obstacle_distance_grid_map);
	copy_map(utility_map, exact_euclidean_distance_to_goal.pathDR(goal_y, goal_x),x_size, y_size);

	for (int i = 0; i < x_size * y_size; i++)
		if (utility_map[i] >= 50000.0) // O infinito de distacia eh representado como 50000.0, assim como o espaco ocupado.
			utility_map[i] = 1000.0;

	save_map((char *) "obstacle_heuristic.map", utility_map, x_size, y_size);
	printf("Mapa da heurística com obstáculos carregado!\n");
	free(cost_map);
	return utility_map;
}


grid_state_p ****
alloc_grid_state_map()
{
	grid_state_p ****grid_state_map;
	grid_state_map_x_size = round((obstacle_distance_grid_map->config.x_size * obstacle_distance_grid_map->config.resolution) / astar_config.state_map_resolution);
	grid_state_map_y_size = round((obstacle_distance_grid_map->config.y_size * obstacle_distance_grid_map->config.resolution)/ astar_config.state_map_resolution);
	printf("Distance map origin: %f %f\n", obstacle_distance_grid_map->config.x_origin, obstacle_distance_grid_map->config.y_origin);
	int theta_size = astar_config.state_map_theta_resolution;
	grid_state_map = (grid_state_p ****)calloc(grid_state_map_x_size, sizeof(grid_state_p***));
	carmen_test_alloc(grid_state_map);

	for (int i = 0; i < grid_state_map_x_size; i++)
	{
		grid_state_map[i] = (grid_state_p ***)calloc(grid_state_map_y_size, sizeof(grid_state_p**));
		carmen_test_alloc(grid_state_map[i]);

		for (int j = 0; j < grid_state_map_y_size; j++)
		{
			grid_state_map[i][j] = (grid_state_p**)calloc(theta_size, sizeof(grid_state_p*));
			carmen_test_alloc(grid_state_map[i][j]);

			for (int k = 0; k < theta_size; k++)
			{
				grid_state_map[i][j][k]= (grid_state_p*) calloc(2, sizeof(grid_state_p));
				carmen_test_alloc(grid_state_map[i][j][k]);

				for (int l = 0; l < 2 ; l++)
				{
					grid_state_map[i][j][k][l]= (grid_state_p) malloc(sizeof(grid_state));
					carmen_test_alloc(grid_state_map[i][j][k][l]);

					grid_state_map[i][j][k][l]->state = Not_visited;
					grid_state_map[i][j][k][l]->g = 0;
				}
			}
		}
	}
	return grid_state_map;
}

void
clear_grid_state_map(grid_state_p ****astar_map)
{
	int theta_size = astar_config.state_map_theta_resolution;
	for (int i = 0; i < grid_state_map_x_size; i++)
	{
		for (int j = 0; j < grid_state_map_y_size; j++)
		{
			for (int k = 0; k < theta_size; k++)
			{
				for (int l = 0; l < 2; l++)
					free(astar_map[i][j][k][l]);
				free(astar_map[i][j][k]);
			}
			free(astar_map[i][j]);
		}
		free(astar_map[i]);
	}
	free(astar_map);
}


static int
open_cost_map()
{
	FILE *fp;
	int result;

	fp = fopen (astar_config.precomputed_cost_file_name, "rw");
	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir a matriz. Usando o Reed-Shepp como heurística.\n");
		use_nonholonomic_heuristic_cost_map = 0;
		return 1;
	}

	for (int i = 0; i < astar_config.precomputed_cost_size/astar_config.precomputed_cost_resolution; i++)
	{
		for (int j = 0; j < astar_config.precomputed_cost_size/astar_config.precomputed_cost_resolution; j++)
		{
			for (int k = 0; k < astar_config.precomputed_cost_theta_size; k++)
			{
				result = fscanf(fp, "%lf ", &nonholonomic_heuristic_cost_map[i][j][k]->h);
//				printf("aqui %d %d %d\n", i, j, k);
				if (result == EOF)
				{
					printf("result == EOF\n");
					break;
				}
			}
		}
	}
	fclose (fp);
	use_nonholonomic_heuristic_cost_map = 1;
	printf("Mapa da heurística sem obstáculos carregado!\n");
	return 0;
}


void
alloc_cost_map()
{
	printf("Carregando mapa da heurística sem obstáculos\n");
	int x_size = round(astar_config.precomputed_cost_size  / astar_config.precomputed_cost_resolution);
	int y_size = round(astar_config.precomputed_cost_size / astar_config.precomputed_cost_resolution);
	int theta_size = astar_config.precomputed_cost_theta_size;
	nonholonomic_heuristic_cost_map = (nonholonomic_heuristic_cost_p ***) calloc(x_size, sizeof(nonholonomic_heuristic_cost_p **));
	carmen_test_alloc(nonholonomic_heuristic_cost_map);

	for (int i = 0; i < x_size; i++)
	{
		nonholonomic_heuristic_cost_map[i] = (nonholonomic_heuristic_cost_p **)calloc(y_size, sizeof(nonholonomic_heuristic_cost_p*));
		carmen_test_alloc(nonholonomic_heuristic_cost_map[i]);

		for (int j = 0; j < y_size; j++)
		{
			nonholonomic_heuristic_cost_map[i][j] = (nonholonomic_heuristic_cost_p*)calloc(astar_config.precomputed_cost_theta_size, sizeof(nonholonomic_heuristic_cost_p));
			carmen_test_alloc(nonholonomic_heuristic_cost_map[i][j]);

			for (int z = 0; z < theta_size; z++)
			{
				nonholonomic_heuristic_cost_map[i][j][z]= (nonholonomic_heuristic_cost_p) malloc(sizeof(nonholonomic_heuristic_cost));
				carmen_test_alloc(nonholonomic_heuristic_cost_map[i][j][z]);
			}
		}
	}

	open_cost_map();

}


void
clear_astar_search(boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> &FH, grid_state_p ****astar_map, state_node* goal_node)
{
	for (int i = 0; i < FH.size(); i++)
	{
		state_node *temp = FH.top();
		FH.pop();
		free(temp);
	}
	FH.clear();

	clear_grid_state_map(astar_map);
	free(goal_node);

}


state_node*
new_state_node(double pose_x, double pose_y, double pose_theta, motion_direction pose_r, double g, double h, double f, state_node* parent)
{
	state_node *new_state = (state_node*) malloc(sizeof(state_node));

	new_state->pose.x = pose_x;
	new_state->pose.y = pose_y;
	new_state->pose.theta = pose_theta;
	new_state->pose.r = pose_r;
	new_state->g = g;
	new_state->h = h;
	new_state->f = f;
	new_state->parent = parent;

	new_state->expanded_nodes = 0;
	new_state->movement = 0;

	return (new_state);

}


double
carmen_compute_abs_angular_distance(double theta_1, double theta_2)
{
	return (carmen_normalize_theta(abs(theta_1 - theta_2)));
}


std::vector<carmen_ackerman_traj_point_t>
reed_shepp_path(carmen_ackerman_traj_point_t current, carmen_ackerman_traj_point_t goal_state, int &reed_shepp_collision, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	int rs_pathl;
	int rs_numero;
	double tr;
	double ur;
	double vr;
	double distance_traveled = 0.0;
	double distance_traveled_old = 0.0;
	carmen_ackerman_traj_point_t rs_points[6]; // Por alguma razão, com o valor 5 acontece stack smashing às vezes quando o rs_pathl == 5
	double v_step;
	double step_weight;
	double path_cost = 0.0;
	std::vector<carmen_ackerman_traj_point_t> rs_path_nodes;

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);
	double rs_length = reed_shepp(current, goal_state, &rs_numero, &tr, &ur, &vr);

	rs_pathl = constRS(rs_numero, tr, ur, vr, current, rs_points);

	double ant_direction = 0;

	for (int i = rs_pathl; i > 0; i--)
	{
		carmen_ackerman_traj_point_t point = rs_points[i];
		if (rs_points[i].v < 0.0)
		{
			v_step = EXPAND_NODES_V;
			step_weight = 1.0;
		}
		else
		{
			v_step = -EXPAND_NODES_V;
			step_weight = 1.0;
		}
		while (DIST2D(point, rs_points[i-1]) > 0.2 || (abs(carmen_compute_abs_angular_distance(point.theta, rs_points[i-1].theta)) > 0.0472665))
		{
			distance_traveled_old = distance_traveled;
			point = carmen_libcarmodel_recalc_pos_ackerman(point, v_step, rs_points[i].phi,
					0.1, &distance_traveled, DELTA_T, robot_config);
			path_cost += step_weight * (distance_traveled - distance_traveled_old);
//			carmen_ackerman_traj_point_p new_state = (carmen_ackerman_traj_point_p) malloc(sizeof(carmen_ackerman_traj_point_t));
			carmen_ackerman_traj_point_t new_state;
			new_state = point;
			//Como o Reed Shepp realiza o caminho do goal para um ponto, ele está andando de ré. Por isso precisa-se inverter o sinal de v
			new_state.v = -new_state.v;
			new_state.theta = carmen_normalize_theta(new_state.theta);
//			printf("Reed-Shepp: %f %f %f %f\n", new_state.x, new_state.y, new_state.theta, new_state.v);
			if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(new_state, obstacle_distance_grid_map) < OBSTACLE_DISTANCE_MIN || ( ant_direction != 0 && sign(new_state.v) != sign(ant_direction)) /*|| (path_cost > 10 && new_state.v < 0)*/)
			{
				reed_shepp_collision = 1;
				return rs_path_nodes;
			}

			rs_path_nodes.push_back(new_state);
			ant_direction = new_state.v;
		}
	}

	std::reverse(rs_path_nodes.begin(), rs_path_nodes.end());

	/*
	state_node_p ant_state = (state_node_p) malloc(sizeof(state_node));
	ant_state = current;
	for (int i = 0; i<rs_path_nodes.size(); i++)
	{
		rs_path_nodes[i]->parent = ant_state;
		ant_state = rs_path_nodes[i];
	}
*/
	return rs_path_nodes;
}


double
h(state_node *current_pose, state_node *goal_pose, grid_state_p**** grid_state_map, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, double* goal_distance_map, nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map)
{
	double ho = -1;
	double nh = -1;

	int x_c;
	int y_c;
	int theta_c;
	int direction_c;
	get_current_pos(current_pose, x_c, y_c, theta_c, direction_c, obstacle_distance_grid_map);

	ho = goal_distance_map[y_c + x_c * grid_state_map_y_size];

	if (astar_config.use_matrix_cost_heuristic && use_nonholonomic_heuristic_cost_map)
	{
		int x = ((current_pose->pose.x - goal_pose->pose.x) * cos(current_pose->pose.theta) - (current_pose->pose.y - goal_pose->pose.y) * sin(current_pose->pose.theta))/astar_config.precomputed_cost_resolution;
		int y = ((current_pose->pose.x - goal_pose->pose.x) * sin(current_pose->pose.theta) + (current_pose->pose.y - goal_pose->pose.y) * cos(current_pose->pose.theta))/astar_config.precomputed_cost_resolution;
		int theta;

		if ((x <= 0 && y >= 0) || (x >= 0 && y <= 0))
			theta = get_astar_map_theta(carmen_normalize_theta(-(goal_pose->pose.theta - current_pose->pose.theta)), astar_config.precomputed_cost_theta_size);
		else
			theta = get_astar_map_theta(carmen_normalize_theta(goal_pose->pose.theta - current_pose->pose.theta), astar_config.precomputed_cost_theta_size);


		int half_map = round(((astar_config.precomputed_cost_size)/astar_config.precomputed_cost_resolution) / 2);

		if (x < half_map && y < half_map && x > -half_map && y > -half_map)
		{
			nh =  nonholonomic_heuristic_cost_map[x + half_map][y + half_map][theta]->h;
		}
	}

//	printf("NH = %f HO = %f\n", nh, ho);

	double return_h;

	#if COMPARE_HEURISTIC

	switch(heuristic_number){
		case 0:
			return_h = std::max(nh, ho);
			break;
		case 1:
			return_h = ho;
			break;
		case 2:
			return_h = std::max(nh, DIST2D(current_pose->pose, goal_pose->pose));
			break;
		case 3:
			return_h = DIST2D(current_pose->pose, goal_pose->pose);
			break;
	}

	#else
	return_h = std::max(nh, ho);
	#endif

	return return_h;
}


void
analytic_expansion(state_node** n, state_node* goal_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	int reed_shepp_collision = 0;

	carmen_ackerman_traj_point_t trajectory_current_pose;
	trajectory_current_pose.x = (*n)->pose.x;
	trajectory_current_pose.y = (*n)->pose.y;
	trajectory_current_pose.theta = (*n)->pose.theta;
	trajectory_current_pose.v = 0;
	trajectory_current_pose.phi = 0;

	carmen_ackerman_traj_point_t trajectory_goal_pose;
	trajectory_goal_pose.x = goal_node->pose.x;
	trajectory_goal_pose.y = goal_node->pose.y;
	trajectory_goal_pose.theta = goal_node->pose.theta;
	trajectory_goal_pose.v = 0;
	trajectory_goal_pose.phi = 0;

	std::vector<carmen_ackerman_traj_point_t> rs_path = reed_shepp_path(trajectory_current_pose, trajectory_goal_pose, reed_shepp_collision, obstacle_distance_grid_map);
	if (reed_shepp_collision == 0)
	{
		state_node_p ant_state = (state_node_p) malloc(sizeof(state_node));
		ant_state = (*n);

		for (int i = 0; i < rs_path.size(); i++)
		{
			state_node_p current_state = (state_node_p) malloc(sizeof(state_node));

			current_state->pose.x = rs_path[i].x;
			current_state->pose.y = rs_path[i].y;
			current_state->pose.theta = rs_path[i].theta;

			if (rs_path[i].v < 0)
				current_state->pose.r = Backward;
			else
				current_state->pose.r = Forward;

			//Extended
			current_state->expanded_nodes = -1;
			current_state->movement = -1;
			//

			current_state->parent = ant_state;

//			printf("Rs_node = %f %f %f %d\n", current_state->pose.x, current_state->pose.y, current_state->pose.theta, current_state->pose.r);
//			printf("Rs_path = %f %f %f %f\n", rs_path[i].x, rs_path[i].y, rs_path[i].theta, rs_path[i].v);

			*n = current_state;
			ant_state = current_state;

		}

//		free(ant_state);
//		exit(1);
	}
}


int
is_goal(state_node** current_node, state_node* goal_node, int cont_rs_nodes)
{
	if (DIST2D((*current_node)->pose, goal_node->pose) < 0.5 && (abs(carmen_radians_to_degrees((*current_node)->pose.theta) - carmen_radians_to_degrees(goal_node->pose.theta)) < 5))
		return 1;

	else if (cont_rs_nodes % int((*current_node)->h + 1) == 0)
	{
		analytic_expansion(current_node, goal_node, obstacle_distance_grid_map);

		if (DIST2D((*current_node)->pose, goal_node->pose) < 0.5 && (abs(carmen_radians_to_degrees((*current_node)->pose.theta) - carmen_radians_to_degrees(goal_node->pose.theta)) < 5))
			return 1;
	}

	return 0;
}


double
movement_cost(state_node* current_node, state_node* new_node)
{
	return DIST2D(current_node->pose, new_node->pose);
}


double
penalties(state_node* current_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	carmen_ackerman_traj_point_t trajectory_pose;
	trajectory_pose.x = current_node->pose.x;
	trajectory_pose.y = current_node->pose.y;
	trajectory_pose.theta = current_node->pose.theta;
	trajectory_pose.v = 0;
	trajectory_pose.phi = 0;

	double distance_to_nearest_obstacle = carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map);

	return (1/distance_to_nearest_obstacle) +
			PENALTIES_W1 * current_node->pose.r * movement_cost(current_node, current_node->parent) +
			PENALTIES_W2 * (current_node->pose.r != current_node->parent->pose.r);
}


int
is_valid_state(state_node *state, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	if (state->pose.x < obstacle_distance_grid_map->config.x_origin || state->pose.y < obstacle_distance_grid_map->config.y_origin)
		return 0;

	int x;
	int y;
	int theta;
	int direction;
	get_current_pos(state, x, y, theta, direction, obstacle_distance_grid_map);

	carmen_ackerman_traj_point_t trajectory_pose;
	trajectory_pose.x = state->pose.x;
	trajectory_pose.y = state->pose.y;
	trajectory_pose.theta = state->pose.theta;
	trajectory_pose.v = 0;
	trajectory_pose.phi = 0;


	if (x >= grid_state_map_x_size || y >= grid_state_map_y_size || x < 0 || y < 0 || carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map) < OBSTACLE_DISTANCE_MIN)
		return 0;

	return 1;
}


void
get_astar_path(state_node *n, std::vector<carmen_ackerman_traj_point_t> &path_result)
{
	if (n == NULL)
		return;

	carmen_ackerman_traj_point_t trajectory_pose;
	carmen_ackerman_traj_point_t last_state;

	trajectory_pose.x = n->pose.x;
	trajectory_pose.y = n->pose.y;
	trajectory_pose.theta = n->pose.theta;

	if (n->pose.r == Forward)
		trajectory_pose.v = 1.0;
	else
		trajectory_pose.v = -1.0;

	trajectory_pose.phi = 0;

	path_result.push_back(trajectory_pose);
	last_state = trajectory_pose;
	n = n->parent;

	while (n != NULL)
	{
		//Extended
//		printf("[get_astar_path] %d %d\n", n->expanded_nodes, n->movement);
		//

		trajectory_pose.x = n->pose.x;
		trajectory_pose.y = n->pose.y;
		trajectory_pose.theta = n->pose.theta;

		if (n->pose.r == Forward)
			trajectory_pose.v = 1.0;
		else
			trajectory_pose.v = -1.0;

		trajectory_pose.phi = 0;

		if ((DIST2D(trajectory_pose, last_state) >= 0.5 && DIST2D(trajectory_pose, last_state) <= 1.0 )|| (sign(trajectory_pose.v) != sign(last_state.v)))
		{
//			printf("[get_astar_path] %f %f %f\n", trajectory_pose.x, trajectory_pose.y, trajectory_pose.theta);
			path_result.push_back(trajectory_pose);
			last_state = trajectory_pose;
		}
		else if (DIST2D(trajectory_pose, last_state) > 1.0)
		{
			carmen_ackerman_traj_point_t trajectory_pose_2;
			trajectory_pose_2.x = (trajectory_pose.x + last_state.x) / 2.0;
			trajectory_pose_2.y = (trajectory_pose.y + last_state.y) / 2.0;
			trajectory_pose_2.theta = carmen_normalize_theta((trajectory_pose.theta + last_state.theta) / 2.0);
			trajectory_pose_2.v = trajectory_pose.v;
			trajectory_pose_2.phi = 0;

			path_result.push_back(trajectory_pose_2);
			path_result.push_back(trajectory_pose);
			last_state = trajectory_pose;
		}
		n = n->parent;

	}
	std::reverse(path_result.begin(), path_result.end());
}


void
check_initial_nodes(state_node *initial_node, state_node *goal_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, int &failed)
{
	failed = 0;

	carmen_ackerman_traj_point_t trajectory_pose;
	trajectory_pose.x = initial_node->pose.x;
	trajectory_pose.y = initial_node->pose.y;
	trajectory_pose.theta = initial_node->pose.theta;
	trajectory_pose.v = 0;
	trajectory_pose.phi = 0;
	printf("Initial_pose = %f %f %f\n", initial_node->pose.x, initial_node->pose.y, initial_node->pose.theta);

	if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map) < OBSTACLE_DISTANCE_MIN)
	{
		printf("Robot_pose next to obstacle: %f\n", carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map));
		failed = 1;
	}

	trajectory_pose.x = goal_node->pose.x;
	trajectory_pose.y = goal_node->pose.y;
	trajectory_pose.theta = goal_node->pose.theta;
	trajectory_pose.v = 0;
	trajectory_pose.phi = 0;

	if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map) < OBSTACLE_DISTANCE_MIN)
	{
		printf("Goal_pose next to obstacle: %f\n", carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map));
		failed = 1;
	}
	printf("Goal_pose = %f %f %f\n", goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta);
}


offroad_planner_plan_t
astar_mount_offroad_planner_plan(carmen_point_t *robot_pose, carmen_point_t *goal_pose, std::vector<carmen_ackerman_traj_point_t> path_result)
{
	carmen_ackerman_traj_point_t robot;
	carmen_ackerman_traj_point_t goal;
	robot.x = robot_pose->x;
	robot.y = robot_pose->y;
	robot.theta = robot_pose->theta;
	robot.v = 0;
	robot.phi = 0;

	goal.x = goal_pose->x;
	goal.y = goal_pose->y;
	goal.theta = goal_pose->theta;
	goal.v = 0;
	goal.phi = 0;

	offroad_planner_plan_t plan;
	plan.robot = robot;
	plan.goal = goal;
	plan.goal_set = 0;

	/*
	// Para enviar um path que não muda de direção
	if (SEND_MESSAGE_IN_PARTS)
	{
		if (carmen_astar_path_poses.size() > 1 && last_index_poses < carmen_astar_path_poses.size()){
			current_astar_path_poses_till_reverse_direction.clear();
		//	last_index_poses++;
	//		printf("last_index_poses = %d\n", last_index_poses);
			current_astar_path_poses_till_reverse_direction.push_back(carmen_astar_path_poses[last_index_poses]);
	//current pose
			robot.x = current_astar_path_poses_till_reverse_direction[0].x;
			robot.y = current_astar_path_poses_till_reverse_direction[0].y;
			robot.theta = current_astar_path_poses_till_reverse_direction[0].theta;
			robot.v = current_astar_path_poses_till_reverse_direction[0].v;
			robot.phi = current_astar_path_poses_till_reverse_direction[0].phi;

	//current goal
			goal.x = current_astar_path_poses_till_reverse_direction[0].x;
			goal.y = current_astar_path_poses_till_reverse_direction[0].y;
			goal.theta = current_astar_path_poses_till_reverse_direction[0].theta;
			goal.v = current_astar_path_poses_till_reverse_direction[0].v;
			goal.phi = current_astar_path_poses_till_reverse_direction[0].phi;

			int current_path_size = 1;
			int find_absolute_value = 0;
			double old_v = current_astar_path_poses_till_reverse_direction[find_absolute_value].v;
			while(old_v == 0.0 && (find_absolute_value+1 < carmen_astar_path_poses.size()))
			{
				old_v = carmen_astar_path_poses[find_absolute_value++].v;
				if (find_absolute_value > 2000)
				{
					printf("failsafe 2085\n");
					exit(1);
				}
			}

			printf("\n");
			for (int i = last_index_poses + 1; i < carmen_astar_path_poses.size() ; i++)
			{
	//			printf("old_v = %f %f\n", old_v, carmen_astar_path_poses[i].v);
				if (old_v == carmen_astar_path_poses[i].v)
				{
					current_astar_path_poses_till_reverse_direction.push_back(carmen_astar_path_poses[i]);
					printf("current_astar = %f %f %f %f %f %d\n",carmen_astar_path_poses[i].x, carmen_astar_path_poses[i].y, carmen_astar_path_poses[i].theta, carmen_astar_path_poses[i].v, carmen_astar_path_poses[i].phi, last_index_poses );
					last_index_poses = i;
					current_path_size++;
					goal.x = carmen_astar_path_poses[i].x;
					goal.y = carmen_astar_path_poses[i].y;
					goal.theta = carmen_astar_path_poses[i].theta;
					goal.v = carmen_astar_path_poses[i].v;
					goal.phi = carmen_astar_path_poses[i].phi;
				}
				else
				{
					break;
				}
		//		printf("i = %d\n", i);
			}

			last_index_poses++;
			plan.robot = robot;
			plan.goal = goal;
			plan.path.points = &(current_astar_path_poses_till_reverse_direction[0]);
			plan.path.length = current_path_size;
		}

	}
	else
	{
	*/
		plan.path.points = &(path_result[0]);
		plan.path.length = path_result.size();
	//}

	plan.path.capacity = 0;

	return plan;
}


pose_node
carmen_path_planner_astar_ackerman_kinematic(pose_node point, double lenght, double phi, double v)
{
	point.theta += v * (tan(phi) / lenght);
	point.theta = carmen_normalize_theta(point.theta);
	point.x += v * cos(point.theta);
	point.y += v * sin(point.theta);

	return point;
}


std::vector<state_node*>
expand_node(state_node* n, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
    std::vector<state_node*> neighbors;
    double target_phi[3] = {-robot_config.max_phi, 0.0, robot_config.max_phi};
    double target_v[2]   = {EXPAND_NODES_V, -EXPAND_NODES_V};

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
        	state_node_p new_state = (state_node_p) malloc(sizeof(state_node));
        	carmen_test_alloc(new_state);
        	new_state->pose = carmen_path_planner_astar_ackerman_kinematic(n->pose, robot_config.distance_between_front_and_rear_axles, target_phi[j], target_v[i]);
        	if (target_v[i] < 0)
        	{
				new_state->pose.r = Backward;
				new_state->movement = j + 3;
        	}
			else
			{
				new_state->pose.r = Forward;
				new_state->movement = j;
			}

        	if (is_valid_state(new_state, obstacle_distance_grid_map) == 1)
			{
				neighbors.push_back(new_state);
			}
			else
				free(new_state);
        }
    }



    return neighbors;
}


std::vector<carmen_ackerman_traj_point_t>
carmen_path_planner_astar_search(pose_node *initial_pose, pose_node *goal_pose,
		carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, double *goal_distance_map,
		nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map)
{

#if DRAW_EXPANSION_TREE
		draw_map(obstacle_distance_grid_map, map_image);
#endif

	expanded_nodes_by_astar = 0;
	std::vector<carmen_ackerman_traj_point_t> path_result;

	grid_state_p**** grid_state_map = alloc_grid_state_map();
	state_node* initial_node = new_state_node(initial_pose->x, initial_pose->y, initial_pose->theta, initial_pose->r, 0, 0, 0, NULL);
	state_node* goal_node = new_state_node(goal_pose->x, goal_pose->y, goal_pose->theta, goal_pose->r, 0, 0, 0, NULL);

	int initial_nodes_failed;
	check_initial_nodes(initial_node, goal_node, obstacle_distance_grid_map, initial_nodes_failed);
	if (initial_nodes_failed == 1)
		return path_result;

	boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> FH;
	FH.push(initial_node);

	int x, y, theta, direction;
	get_current_pos(initial_node, x, y, theta, direction, obstacle_distance_grid_map);
	grid_state_map[x][y][theta][direction]->state = Open;
	grid_state_map[x][y][theta][direction]->g = 0;

	state_node* n;
	int cont_rs_nodes = 0;

	while (!FH.empty())
	{
		n = FH.top();
		FH.pop();
		get_current_pos(n, x, y, theta, direction, obstacle_distance_grid_map);
		if (grid_state_map[x][y][theta][direction]->state != Closed)
		{
//			printf("Current_node = %f %f %f %d\n", n->pose.x, n->pose.y, n->pose.theta, n->pose.r);
			grid_state_map[x][y][theta][direction]->state = Closed;

			if (is_goal(&n, goal_node, cont_rs_nodes))
			{
				printf("Path encontrado!\n");
				printf("Current_node = %f %f %f %d\n", n->pose.x, n->pose.y, n->pose.theta, n->pose.r);
				printf("Initial node: %f %f %f \n", initial_pose->x, initial_pose->y, initial_pose->theta);
				printf("Goal node: %f %f %f \n", goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta);

				get_astar_path(n, path_result);

#if DRAW_EXPANSION_TREE
				draw_point_on_map_img(initial_node->pose.x, initial_node->pose.y, obstacle_distance_grid_map->config, cv::Scalar(130, 0, 0), map_image);
				draw_point_on_map_img(goal_node->pose.x, goal_node->pose.y, obstacle_distance_grid_map->config, cv::Scalar(0, 0, 130), map_image);
				for (int i = 1; i < path_result.size(); i++)
					draw_point_in_opencv_image(path_result[i], path_result[i-1], obstacle_distance_grid_map->config, cv::Scalar(0,0,255), 2);
				expansion_tree_file_name[strlen(expansion_tree_file_name) - 5] = heuristic_number + '0';
				printf("Expansion_tree_file_name = %s\n", expansion_tree_file_name);
				imwrite(expansion_tree_file_name, map_image);
#endif

				clear_astar_search(FH, grid_state_map, goal_node);
				printf("---------------------- Expanded_nodes = %d\n", expanded_nodes_by_astar);
				state_node *temp;
				while (n != NULL)
				{
					temp = n;
					n = n->parent;
					free(temp);
				}

				return path_result;
			}
			else
			{
				expanded_nodes_by_astar++;
				std::vector<state_node*> N = expand_node(n, obstacle_distance_grid_map);

				for (int i = 0; i < N.size(); i++)
				{
					get_current_pos(N[i], x, y, theta, direction, obstacle_distance_grid_map);

					if (grid_state_map[x][y][theta][direction]->state != Closed)
					{
						N[i]->g = n->g + movement_cost(n, N[i]);

						if (grid_state_map[x][y][theta][direction]->state == Not_visited || (grid_state_map[x][y][theta][direction]->state == Open &&
								grid_state_map[x][y][theta][direction]->g > N[i]->g))
						{
							N[i]->h = h(N[i], goal_node, grid_state_map, obstacle_distance_grid_map, goal_distance_map, nonholonomic_heuristic_cost_map);
							N[i]->parent = n;
							N[i]->f = N[i]->g + N[i]->h + penalties(N[i], obstacle_distance_grid_map);

							grid_state_map[x][y][theta][direction]->state = Open;
							grid_state_map[x][y][theta][direction]->g = N[i]->g;

							N[i]->expanded_nodes = expanded_nodes_by_astar;


							FH.push(N[i]);
						}
						else
							free(N[i]);
					}
				}
#if DRAW_EXPANSION_TREE
				int r = 0, g = 0, b = 0;
				int dist_to_goal = int(DIST2D(n->pose, goal_node->pose));
				int dist_to_initial = int(DIST2D(n->pose, initial_node->pose));
				g = carmen_clamp(0, dist_to_goal, 255);
				b = carmen_clamp(0, dist_to_initial, 255);
				draw_state_in_opencv_image(n, obstacle_distance_grid_map->config, cv::Scalar(b, g, r), map_image);
#endif
			}
		}
		else
			free(n);

		cont_rs_nodes++;


	}

	printf("Path não encontrado!\n");
	clear_astar_search(FH, grid_state_map, goal_node);
	return path_result;
}
