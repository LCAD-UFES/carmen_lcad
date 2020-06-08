#include <list>
#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>
#include <carmen/rddf_interface.h>
#include <carmen/route_planner_interface.h>

#include "path_planner_astar.h"


static carmen_robot_ackerman_config_t robot_config;
static carmen_path_planner_astar_t astar_config;

static carmen_navigator_config_t nav_config;

carmen_behavior_selector_state_t current_state = BEHAVIOR_SELECTOR_PARKING;
int steering_model = 1;

double distance_between_front_and_rear_axles;

static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;
static carmen_obstacle_distance_mapper_map_message distance_map_struct;
static carmen_obstacle_distance_mapper_map_message *distance_map = NULL;

static carmen_map_p map_occupancy = NULL;

carmen_mapper_virtual_laser_message virtual_laser_message;

int final_goal_received = 0;
int astar_path_sended = 0;
int astar_map_x_size;
int astar_map_y_size;
carmen_point_t *final_goal = NULL;
carmen_localize_ackerman_globalpos_message current_globalpos_msg;
cost_heuristic_node_p ***heuristic_without_obstacle_map;
std::vector<carmen_ackerman_traj_point_t> carmen_astar_path_poses;
carmen_route_planner_road_network_message route_planner_road_network_message;
offroad_planner_plan_t plan_path_poses;
int astar_path_poses_size = 0;


#define LANE_WIDTH 	2.4
#define NUM_LANES	1
#define MAX_VIRTUAL_LASER_SAMPLES 100000
#define SQRT2 sqrt(2.0)
#define DIST2D_D_P(x1,x2) (sqrt(((x1).x - (x2)->x) * ((x1).x - (x2)->x) + \
							((x1).y - (x2)->y) * ((x1).y - (x2)->y)))
#define USE_SMOOTH 1


double
obstacle_distance(double x, double y)
{
	if( NULL == distance_map)
		exit(1);

    carmen_point_t p;
    p.x = x;
    p.y = y;
    return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&p, distance_map));
}


carmen_position_t
nearest_obstacle_cell(double x, double y)
{
	if( NULL == distance_map)
		exit(1);

    carmen_point_t p;
    p.x = x;
    p.y = y;
    return (carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(&p, distance_map));
}


void
calculate_phi_ahead(carmen_ackerman_traj_point_t *path, int num_poses)
{
	double L = robot_config.distance_between_front_and_rear_axles;

	for (int i = 0; i < (num_poses - 1); i++)
	{
		double delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
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
		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
	}
}


void
compute_theta(carmen_ackerman_traj_point_t *path, int num_poses)
{
	for (int i = 0; i < (num_poses - 1); i++)
		path[i].theta = atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
	if (num_poses > 1)
		path[num_poses - 1].theta = path[num_poses - 2].theta;
}


void
calculate_theta_and_phi(carmen_ackerman_traj_point_t *poses_ahead, int num_poses_ahead)
{
	compute_theta(poses_ahead, num_poses_ahead);
	calculate_phi_ahead(poses_ahead, num_poses_ahead);
}


double
my_f(const gsl_vector *v, void *params)
{
	double wo = 1.0;
	double wk = 1.0;
	double ws = 1.0;
	double dmax = 5.0; // escolher um valor melhor
	double kmax = robot_config.distance_between_front_and_rear_axles / tan(robot_config.max_phi);

	// double *obstacles = (double *) params;
	param_t *param = (param_t*) params;
	int j = 0;
	for (int i = 0; i < param->path_size; i++)
	{
		if (!param->anchor_points[i])
		{
			param->points[i].x = gsl_vector_get(v, j++);
			param->points[i].y = gsl_vector_get(v, j++);
//			printf("x = %f y = %f i = %d\n",param->points[i].x, param->points[i].y, i);
		}
	}

	double obstacle_cost = 0.0;
	double x, y, obst_x, obst_y, distance;
/*
	for (int i = 2; i < v->size; i += 2) {
		x = gsl_vector_get(v, i);
		y = gsl_vector_get(v, i + 1);
		//obst_x = obstacles[i];
		//obst_y = obstacles[i + 1];
		//distance = sqrt((x - obst_x) * (x - obst_x) + (y - obst_y) * (y - obst_y));
		distance = obstacle_distance(x, y);
		obstacle_cost += wo * (distance - dmax) * (distance - dmax) ;
	}
*/
	double curvature_cost = 0.0;
	double delta_phi;


	double x_i, y_i, x_next, y_next, x_prev, y_prev, displacement;
/*
	for (int i = 2; i < v->size - 2; i += 2) {
		x_i = gsl_vector_get(v, i);
		y_i = gsl_vector_get(v, i + 1);
		x_next = gsl_vector_get(v, i + 2);
		y_next = gsl_vector_get(v, i + 3);
		x_prev = gsl_vector_get(v, i - 2);
		y_prev = gsl_vector_get(v, i - 1);
		delta_phi = abs(atan2(y_next - y_i, x_next - x_i) - atan2(y_i - y_prev, x_i - x_prev));
		displacement = sqrt((x_i - x_prev) * (x_i - x_prev) + (y_i - y_prev) * (y_i - y_prev));
		curvature_cost += wk * (delta_phi / displacement - kmax) * (delta_phi / displacement - kmax);
	}
*/
	double smoothness_cost = 0.0;
	double square_displacement;
/*
	for (int i = 2; i < v->size - 2; i += 2) {
		x_i = gsl_vector_get(v, i);
		y_i = gsl_vector_get(v, i + 1);
		x_next = gsl_vector_get(v, i + 2);
		y_next = gsl_vector_get(v, i + 3);
		x_prev = gsl_vector_get(v, i - 2);
		y_prev = gsl_vector_get(v, i - 1);
		square_displacement = ((x_next - x_i) - (x_i - x_prev)) * ((x_next - x_i) - (x_i - x_prev)) + ((y_next - y_i) - (y_i - y_prev)) * ((y_next - y_i) - (y_i - y_prev));
		smoothness_cost = ws * square_displacement;
	}

*/
/*	for (int i = 1; i <(param->path_size); i++)
	{
		x = param->points[i].x;
		y = param->points[i].y;
		//obst_x = obstacles[i];
		//obst_y = obstacles[i + 1];
		//distance = sqrt((x - obst_x) * (x - obst_x) + (y - obst_y) * (y - obst_y));
		distance = obstacle_distance(x, y);
//		if(distance <= dmax)
//		{
			obstacle_cost += pow(dmax - distance  , 2) * (dmax - distance );
//			printf("Entrou no obstacle_cost = %f %f\n", obstacle_cost, distance);
//		}
	}
*/
	double curvature_term;
	for (int i = 1; i <(param->path_size - 1); i++)
	{
		x_i = param->points[i].x;
		y_i = param->points[i].y;
		x_next = param->points[i+1].x;
		y_next = param->points[i+1].y;
		x_prev = param->points[i-1].x;
		y_prev = param->points[i-1].y;

		distance = obstacle_distance(x_i, y_i);
		if(distance <= dmax)
			obstacle_cost += abs(pow(distance - dmax, 2) * (distance - dmax));

		square_displacement = ((x_next - x_i) - (x_i - x_prev)) * ((x_next - x_i) - (x_i - x_prev)) + ((y_next - y_i) - (y_i - y_prev)) * ((y_next - y_i) - (y_i - y_prev));
//		square_displacement = pow(sqrt((x_next - x_i) * (x_next - x_i) + (y_next - y_i) * (y_next - y_i)) - sqrt((x_i - x_prev) * (x_i - x_prev) + (y_i - y_prev) * (y_i - y_prev)), 2);
		smoothness_cost +=  square_displacement;

		delta_phi = abs(atan2(y_next - y_i, x_next - x_i) - atan2(y_i - y_prev, x_i - x_prev));
		displacement = sqrt((x_i - x_prev) * (x_i - x_prev) + (y_i - y_prev) * (y_i - y_prev));
		if(abs(displacement) > 0.001)
		{
			curvature_term = (delta_phi / displacement);
			if(curvature_term > kmax)
			{
				curvature_term = (delta_phi / displacement) - kmax;
				curvature_cost += pow(curvature_term, 2) * curvature_term;
	//			printf("teste = %d %f %f %f\n",i, obstacle_cost, curvature_cost, smoothness_cost);
			}
		}

	}

	distance = obstacle_distance(param->points[param->path_size - 1].x, param->points[param->path_size - 1].y);
	if(distance < dmax)
		obstacle_cost += abs(pow(distance - dmax , 2) * (distance - dmax ));

	obstacle_cost = wo * obstacle_cost;
	curvature_cost = wk * curvature_cost;
	smoothness_cost = ws * smoothness_cost;
//	exit(1);
//	printf("costs= %f %f %f \n", obstacle_cost, curvature_cost, smoothness_cost);

	return obstacle_cost + curvature_cost + smoothness_cost;
//	return smoothness_cost;
}


void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double wo = 1.0;
	double wk = 1.0;
	double ws = 1.0;
	double dmax = 20.0; // escolher um valor melhor
	double kmax = 10.0; // colocar o valor certo

	double *obstacles = (double *) params;
	double df_dx, df_dy, x, y, obst_x, obst_y, distance;
	double curvature_cost = 0;
	double delta_phi;
	double x_i, y_i, x_next, y_next, x_prev, y_prev, displacement;
	double smoothness_cost = 0;
	double square_displacement;


	double h = 0.00005;
	double f_x = my_f(v, params);
	param_t *param = (param_t *) params;

	gsl_vector *x_h = gsl_vector_alloc(param->path_size*2);
	gsl_vector_memcpy(x_h, v);

	for (int i = 0; i < param->path_size; i++)
	{
		if (!param->anchor_points[i])
		{
			gsl_vector_set(x_h, i, gsl_vector_get(v, i) + h);
			double f_x_h = my_f(x_h, params);
			double d_f_x_h = (f_x_h - f_x)/ h;
			gsl_vector_set(df, i, d_f_x_h);
			gsl_vector_set(x_h, i, gsl_vector_get(v, i));
		}
	}
	gsl_vector_free(x_h);
/*
	for (int i = 2; i < v->size; i += 2) {
		x = gsl_vector_get(v, i);
		y = gsl_vector_get(v, i + 1);
		obst_x = obstacles[i];
		obst_y = obstacles[i + 1];
		distance = sqrt((x - obst_x) * (x - obst_x) + (y - obst_y) * (y - obst_y));
		df_dx = wo * 2 * (distance - dmax) * (x - obst_x) / distance;
		df_dy = wo * 2 * (distance - dmax) * (y - obst_y) / distance;

		// derivada da segunda parte
		// df_dx += alguma coisa;
		// df_dy += alguma coisa;

		// derivada da terceira parte
		// df_dx += alguma coisa;
		// df_dy += alguma coisa;

		gsl_vector_set(df, i, df_dx);
		gsl_vector_set(df, i + 1, df_dy);
	}

*/
}


void
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


static int
sign (double a)
{
	if (a >= 0.0)
		return 1;
	else
		return -1;
}


void
set_anchor(param_t *params)
{
	params->anchor_points[0] = 1;
	params->anchor_points[params->path_size - 1] = 1;
	for (int i = 0; i < (params->path_size - 1); i++)
	{
		if (sign(params->points[i].v) != sign(params->points[i+1].v))
			params->anchor_points[i] = 1;
		else
			params->anchor_points[i] = 0;
	}
}



int
smooth_rddf_using_conjugate_gradient(carmen_ackerman_traj_point_t *poses_ahead, int size)
{
	int iter = 0;
	int status, i = 0, j = 0;
	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;
	gsl_vector *v;
	gsl_multimin_function_fdf my_func;



	if (size < 5)
		return (0);

	int *anchor_points = (int*)malloc(sizeof(int)*size);
	for (int i = 0; i < size; i++)
	{
		anchor_points[i] = 0;

	}

	param_t *param =(param_t*) malloc(sizeof(param_t));
	param->points = poses_ahead;
	param->anchor_points = anchor_points;
	param->path_size = size-2;

	set_anchor(param);

	my_func.n = (2 * size) - 4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
//	my_func.params = &path;
	my_func.params = param;

	v = gsl_vector_alloc ((2 * size) - 4);

	static int count = 0;
	count++;

	for (i = 0, j = 0; i < (size - 2); i++)
	{

		gsl_vector_set (v, j++, poses_ahead[i].x);
		gsl_vector_set (v, j++, poses_ahead[i].y);
	}


	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc (T, (2 * size) - 4);
	gsl_multimin_fdfminimizer_set (s, &my_func, v, 0.1, 0.0001);  //(function_fdf, gsl_vector, step_size, tol)
	//Algumas vezes o código não realiza a otimização porque dá erro no do-while abaixo

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate (s);
		if ((status != GSL_SUCCESS) && (status != GSL_ENOPROG) && (status != GSL_CONTINUE))
		{
			printf("Otimização falhou code = %d iter = %d\n", status, iter);
			return (2);
		}

		status = gsl_multimin_test_gradient (s->gradient, 0.01); //(gsl_vector, epsabs) and  |g| < epsabs
		// status == 0 (GSL_SUCCESS), if a minimum has been found
	} while (status == GSL_CONTINUE && iter < 500);


	for (i = 0, j = 0; i < (size - 2); i++)
	{
		poses_ahead[i].x = gsl_vector_get (s->x, j++);
		poses_ahead[i].y = gsl_vector_get (s->x, j++);
	}

	calculate_theta_and_phi(poses_ahead, size);

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);
	printf("Terminou a otimização\n");

	for (int i = 1; i<size; i++)
	{
		printf("%f %f %f %f %f\n", poses_ahead[i].x, poses_ahead[i].y, poses_ahead[i].theta, poses_ahead[i].v, poses_ahead[i].phi);
	}

	return (1);
}


void
add_lanes(carmen_route_planner_road_network_message &route_planner_road_network_message,
		carmen_ackerman_traj_point_t *path_copy)
{
	int num_lanes = NUM_LANES;
	if (path_copy)
		num_lanes++;

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
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_WIDTH(0, LANE_WIDTH);
    	pose_i++;
	}
    for (int i = 0; i < route_planner_road_network_message.number_of_poses; i++)
	{
    	route_planner_road_network_message.nearby_lanes[pose_i] = route_planner_road_network_message.poses[i];
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_WIDTH(0, LANE_WIDTH);
    	pose_i++;
	}
    if (path_copy)
    {
        route_planner_road_network_message.nearby_lanes_ids[1] = 1;
        route_planner_road_network_message.nearby_lanes_indexes[1] = pose_i;
        route_planner_road_network_message.nearby_lanes_sizes[1] = route_planner_road_network_message.number_of_poses_back + route_planner_road_network_message.number_of_poses - 1; // a primeira pose do poses e poses back eh igual
        for (int i = 0; i < pose_i; i++)
    	{
        	route_planner_road_network_message.nearby_lanes[pose_i + i] = path_copy[i];
        	route_planner_road_network_message.traffic_restrictions[pose_i + i] = ROUTE_PLANNER_SET_LANE_WIDTH(0, LANE_WIDTH);
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
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_plan(offroad_planner_path_t path, carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	int *annotations = (int *) calloc (path.length, sizeof(int));
	int *annotations_codes = (int *) calloc (path.length, sizeof(int));
	for (int i = 0; i < path.length; i++)
	{
		annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
		annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
	}
	int nearest_pose_index = get_index_of_nearest_pose_in_path(path.points, globalpos_message->globalpos, path.length);
	carmen_ackerman_traj_point_t *path_copy = NULL;
	bool path_somoothed = false;
	if (nav_config.smooth_path)
	{
		path_copy = (carmen_ackerman_traj_point_t *) malloc(path.length * sizeof(carmen_ackerman_traj_point_t));
		memcpy((void *) path_copy, (void *) path.points, path.length * sizeof(carmen_ackerman_traj_point_t));
		//Estava dando erro na linha abaixo
//		path_somoothed = smooth_path_using_conjugate_gradient(path.points, path.length);
		if (!path_somoothed)
		{
			free(path_copy);
			path_copy = NULL;
			printf("Could not smooth path\n");
		}
		else
		{
			printf("Path smoothed!\n");
		}
	}
//	carmen_route_planner_road_network_message route_planner_road_network_message;
	route_planner_road_network_message.poses = &(path.points[nearest_pose_index]);
	route_planner_road_network_message.poses_back = get_poses_back(path.points, nearest_pose_index);
	route_planner_road_network_message.number_of_poses = path.length - nearest_pose_index;
	route_planner_road_network_message.number_of_poses_back = nearest_pose_index + 1;	// tem que ter pelo menos uma pose_back que eh igual aa primeira poses
	route_planner_road_network_message.annotations = annotations;
	route_planner_road_network_message.annotations_codes = annotations_codes;
	add_lanes(route_planner_road_network_message, path_copy);
	route_planner_road_network_message.timestamp = globalpos_message->timestamp;
	route_planner_road_network_message.host = carmen_get_host();
	carmen_route_planner_publish_road_network_message(&route_planner_road_network_message);
//	printf("route_planner print %f %f %f \n", route_planner_road_network_message.poses->x, route_planner_road_network_message.poses->y, route_planner_road_network_message.poses->theta);

	//free_lanes(route_planner_road_network_message);
	//free(annotations);
	//free(annotations_codes);
	//free(route_planner_road_network_message.poses_back);
	// Estava dando erro de corruption na linha abaixo e ainda não encontrei o motivo
//	free(path.points);
}



void
publish_astar_draw()
{
	virtual_laser_message.host = carmen_get_host();
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//	virtual_laser_message.num_positions = 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////


//Astar

void
draw_astar_object(carmen_ackerman_traj_point_t *object_pose, int color)
{
	/*

	  CARMEN_RED = 0,
	  CARMEN_BLUE = 1,
	  CARMEN_WHITE = 2,
	  CARMEN_YELLOW = 3,
	  CARMEN_GREEN = 4,
	  CARMEN_LIGHT_BLUE = 5,
	  CARMEN_BLACK = 6,
	  CARMEN_ORANGE = 7,
	  CARMEN_GREY = 8,
	  CARMEN_LIGHT_GREY = 9,
	  CARMEN_PURPLE = 10,

	 */
	virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x;
	virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y;
	virtual_laser_message.colors[virtual_laser_message.num_positions] = color;
	virtual_laser_message.num_positions++;
}



static carmen_map_t *
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
is_valid_grid_value(double x, double y, double current_resolution)
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
	if(map_occupancy->complete_map[index] == -1)
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

	printf("map name %s, min_value %lf, max_value %lf\n", file_name, min_value, max_value);
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


double*
get_obstacle_heuristic_map(carmen_point_t *goal_pose)
{
	printf("Carregando mapa da heurística com obstáculos\n");
	int x_size = distance_map->config.x_size;
	int y_size = distance_map->config.y_size;
	double *utility_map = (double *) calloc(x_size * y_size, sizeof(double));
	double *cost_map = (double *) calloc(x_size * y_size, sizeof(double));
	std::fill_n(cost_map, x_size *y_size, -1.0);

	for (int x = 0; x < x_size; x++)
	{
		for(int y = 0; y < y_size; y++)
		{
			if(obstacle_distance(distance_map->config.x_origin + (x * distance_map->config.resolution), distance_map->config.y_origin + (y * distance_map->config.resolution)) < 2.0
					|| is_valid_grid_value(x, y, distance_map->config.resolution) == 0)
			{
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

	int goal_x = round((goal_pose->x - distance_map->config.x_origin)/distance_map->config.resolution);
	int goal_y = round((goal_pose->y - distance_map->config.y_origin)/distance_map->config.resolution);
	copy_map(utility_map, exact_euclidean_distance_to_goal.pathDR(goal_y, goal_x),x_size, y_size);

	for (int i = 0; i < x_size * y_size; i++)
		if (utility_map[i] >= 50000.0) // O infinito de distacia eh representado como 50000.0, assim como o espaco ocupado.
			utility_map[i] = 1000.0;

	save_map((char *) "obstacle_heuristic.map", utility_map, x_size, y_size);
	printf("Mapa da heurística com obstáculos carregado!\n");
	free(cost_map);
	return utility_map;
}


double
get_distance_map_x(int x)
{
	return (double) (x * astar_config.state_map_resolution) + distance_map->config.x_origin;
}


double
get_distance_map_y(int y)
{
	return (double) (y * astar_config.state_map_resolution) + distance_map->config.y_origin;
}


map_node_p ***
alloc_astar_map()
{
	map_node_p ***astar_map;
	astar_map_x_size = round((distance_map->config.x_size * distance_map->config.resolution) / astar_config.state_map_resolution);
	astar_map_y_size = round((distance_map->config.y_size * distance_map->config.resolution)/ astar_config.state_map_resolution);
	int theta_size = astar_config.state_map_theta_resolution;
	double pos_x = 0.0;
	double pos_y = 0.0;
	astar_map = (map_node_p ***)calloc(astar_map_x_size, sizeof(map_node_p**));
	carmen_test_alloc(astar_map);

	for (int i = 0; i < astar_map_x_size; i++)
	{
		astar_map[i] = (map_node_p **)calloc(astar_map_y_size, sizeof(map_node_p*));
		carmen_test_alloc(astar_map[i]);

		for (int j = 0; j < astar_map_y_size; j++)
		{
			astar_map[i][j] = (map_node_p*)calloc(theta_size, sizeof(map_node_p));
			carmen_test_alloc(astar_map[i][j]);

			for (int z = 0; z < theta_size; z++)
			{
				astar_map[i][j][z]= (map_node_p) malloc(sizeof(map_node));
				carmen_test_alloc(astar_map[i][j][z]);
				pos_x = get_distance_map_x(i);
				pos_y = get_distance_map_y(j);
				if(is_valid_grid_value(i, j, astar_config.state_map_resolution) == 1)
				{
					astar_map[i][j][z]->obstacle_distance = obstacle_distance(pos_x, pos_y);
				}
				else
					astar_map[i][j][z]->obstacle_distance = 0;

				astar_map[i][j][z]->is_closed = 0;
				astar_map[i][j][z]->is_open = 0;

			}
		}
	}
	return astar_map;
}


void
clear_astar_map(map_node_p ***astar_map)
{
	int theta_size = astar_config.state_map_theta_resolution;
	for (int i = 0; i < astar_map_x_size; i++)
		for (int j = 0; j < astar_map_y_size; j++)
			for (int k = 0; k < theta_size; k++)
				astar_map[i][j][k] = NULL;
}


static int
open_cost_map()
{
	FILE *fp;
	int result;

	fp = fopen (astar_config.precomputed_cost_file_name, "rw");
	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir o arquivo.\n");
		return 1;
	}

	for (int i = 0; i < astar_config.precomputed_cost_size/astar_config.precomputed_cost_resolution; i++)
	{
		for (int j = 0; j < astar_config.precomputed_cost_size/astar_config.precomputed_cost_resolution; j++)
		{
			for (int k = 0; k < astar_config.precomputed_cost_theta_size; k++)
			{
				result = fscanf(fp, "%lf ", &heuristic_without_obstacle_map[i][j][k]->h);
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
	heuristic_without_obstacle_map = (cost_heuristic_node_p ***)calloc(x_size, sizeof(cost_heuristic_node_p**));
	carmen_test_alloc(heuristic_without_obstacle_map);

	for (int i = 0; i < x_size; i++)
	{
		heuristic_without_obstacle_map[i] = (cost_heuristic_node_p **)calloc(y_size, sizeof(cost_heuristic_node_p*));
		carmen_test_alloc(heuristic_without_obstacle_map[i]);

		for (int j = 0; j < y_size; j++)
		{
			heuristic_without_obstacle_map[i][j] = (cost_heuristic_node_p*)calloc(astar_config.precomputed_cost_theta_size, sizeof(cost_heuristic_node_p));
			carmen_test_alloc(heuristic_without_obstacle_map[i][j]);

			for (int z = 0; z < theta_size; z++)
			{
				heuristic_without_obstacle_map[i][j][z]= (cost_heuristic_node_p) malloc(sizeof(cost_heuristic_node));
				carmen_test_alloc(heuristic_without_obstacle_map[i][j][z]);
			}
		}
	}

	open_cost_map();
}



void
clear_cost_map()
{
	int x_size = round(astar_config.precomputed_cost_size  / astar_config.precomputed_cost_resolution);
	int y_size = round(astar_config.precomputed_cost_size / astar_config.precomputed_cost_resolution);
	int theta_size = astar_config.precomputed_cost_theta_size;

	for (int i = 0; i < x_size; i++)
		for (int j = 0; j < y_size; j++)
			for (int z = 0; z < theta_size; z++)
				heuristic_without_obstacle_map[i][j][z] = NULL;
}


state_node*
create_state_node(double x, double y, double theta, double v, double phi, double g, double h, double f, state_node *parent, double dtg)
{
	state_node *new_state = (state_node*) malloc(sizeof(state_node));
	new_state->state.x = x;
	new_state->state.y = y;
	new_state->state.theta = theta;
	new_state->state.v = v;
	new_state->state.phi = phi;
	new_state->g = g;
	new_state->h = h;
	new_state->f = f;
	new_state->parent = parent;
	new_state->distance_traveled_g = dtg;

	return (new_state);
}


int
get_astar_map_theta(double theta, double map_theta_size)
{
	theta = theta < 0 ? (2 * M_PI + theta) : theta;
	int resolution = (int) round(360/map_theta_size);

	return  (int)round((carmen_radians_to_degrees(theta) / resolution)) % (int)round(360 / resolution);
}


int
get_astar_map_x(double x)
{
	return round((double) (x - distance_map->config.x_origin) / astar_config.state_map_resolution);
}


int
get_astar_map_y(double y)
{
	return round((double) (y - distance_map->config.y_origin) / astar_config.state_map_resolution);
}


discrete_pos_node*
get_current_pos(state_node* current_state)
{
	discrete_pos_node *current_pos = (discrete_pos_node*) malloc(sizeof(discrete_pos_node));
	current_pos->x = get_astar_map_x(current_state->state.x);
	current_pos->y = get_astar_map_y(current_state->state.y);
	current_pos->theta = get_astar_map_theta(current_state->state.theta, astar_config.state_map_theta_resolution);

	return current_pos;
}


int
is_goal(state_node* current_state, state_node* goal_state)
{
	if(DIST2D(current_state->state, goal_state->state) < 0.5 && abs(carmen_radians_to_degrees(current_state->state.theta) - carmen_radians_to_degrees(goal_state->state.theta)) < 5)
		return 1;
	else
		return 0;
}

void
astar_map_close_node(map_node_p ***astar_map, discrete_pos_node *current_pos)
{
	astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_closed = 1;
	astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_open = 0;
}


void
astar_map_open_node(map_node_p ***astar_map, discrete_pos_node *current_pos)
{
	astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_closed = 0;
	astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_open = 1;
}


int
is_valid_state(state_node *state, map_node_p ***astar_map)
{
	discrete_pos_node *current_pos = get_current_pos(state);
	if(current_pos->x >= astar_map_x_size || current_pos->y >= astar_map_y_size || current_pos->x <= 0 || current_pos->y <= 0 || astar_map[current_pos->x][current_pos->y][current_pos->theta]->obstacle_distance < 2.0)
	{
		free(current_pos);
		return 0;
	}
	free(current_pos);
	return 1;
}


carmen_ackerman_traj_point_t
carmen_conventional_astar_ackerman_kinematic_3(carmen_ackerman_traj_point_t point, double lenght, double phi, double v)
{
	double	radcurv = lenght / tan(fabs(phi));

	if(phi == 0)
	{
		point.x += v * cos(point.theta);
		point.y += v * sin(point.theta);
		point.theta = carmen_normalize_theta(point.theta);
		point.phi = phi;
		point.v = v;
	}
	else
	{
		double temp_v = fabs(v) / radcurv;
		int direction_signal = phi >= 0 ? -1 : 1;
		double center_x = point.x + radcurv * sin(point.theta) * direction_signal;
		double center_y = point.y - radcurv * cos(point.theta) * direction_signal;
		double va1 = carmen_normalize_theta(point.theta + 1.5707963268 * direction_signal);
		double va2;

		if (v >= 0)
			va2 = va1 - temp_v * direction_signal;
		else
			va2 = va1 + temp_v * direction_signal;

		point.x = center_x + radcurv * cos(va2);
		point.y = center_y + radcurv * sin(va2);
		point.theta = point.theta - v / radcurv * direction_signal;
		point.theta = carmen_normalize_theta(point.theta);
		point.v = v;
		point.phi = phi;

	}

	return point;
}


std::vector<state_node>
build_state_path(state_node *node)
{
	std::vector<state_node> state_path;
	int i = 0;

    while (node != NULL)
    {
        state_path.push_back(*node);
        node = node->parent;
        i++;
    }

    return(state_path);
}


std::vector<carmen_ackerman_traj_point_t>
build_rddf_poses(state_node *current_state)
{
	std::vector<state_node> path;
	path = build_state_path(current_state);
	std::reverse(path.begin(), path.end());
	std::vector<carmen_ackerman_traj_point_t> temp_rddf_poses_from_path;

	for (int i = 0; i < path.size(); i++)
	{
//		path[i].state.v = abs(path[i].state.v);
		temp_rddf_poses_from_path.push_back(path[i].state);
//		printf("[build_rddf_poses] %f %f %f %f %f\n", path[i].state.x, path[i].state.y, path[i].state.theta, path[i].state.v, path[i].state.phi);
//		draw_astar_object(&path[i].state, CARMEN_GREEN);
	}

	return temp_rddf_poses_from_path;
}


void
astar_mount_path_message(state_node *current_state)
{
	//std::vector<carmen_ackerman_traj_point_t> temp_rddf_poses_from_path = build_rddf_poses(current_state);
	//carmen_astar_path_poses = temp_rddf_poses_from_path[0];
	//std::copy(temp_rddf_poses_from_path.begin(), temp_rddf_poses_from_path.end(), carmen_astar_path_poses);
	carmen_astar_path_poses = build_rddf_poses(current_state);
	astar_path_poses_size = carmen_astar_path_poses.size();


	for (int i = 0; i < astar_path_poses_size; i++)
	{
//		draw_astar_object(&carmen_astar_path_poses[i], CARMEN_GREEN);
	}
//	publish_astar_draw();


	//printf("%f\n",DIST2D(current_globalpos_msg.globalpos, goal_state->state));
//	printf("Chegou ao fim do path!\n");
}


std::vector<state_node*>
expansion(state_node *current, state_node *goal_state, map_node_p ***astar_map)
{
    std::vector<state_node*> neighbor;
    double target_phi;
    double steering_acceleration[3] = {0.0, -0.4, 0.4};
    double target_v[2]   = {2.0, -2.0};
    double time_lenght;
    int size_for;

    for (int i = 0; i < 2; i++)
    {
    	for (int j = 0; j < 3; j++)
    	{
        	state_node_p new_state = (state_node_p) malloc(sizeof(state_node));
        	carmen_test_alloc(new_state);
        	target_phi = carmen_clamp(-robot_config.max_phi, (current->state.phi + steering_acceleration[j]), robot_config.max_phi);
//        	target_phi = steering_acceleration[j];
        	new_state->state = carmen_conventional_astar_ackerman_kinematic_3(current->state, SQRT2, target_phi, target_v[i]);
			new_state->distance_traveled_g = SQRT2;

			if(is_valid_state(new_state, astar_map) == 0)
			{
				free(new_state);
			}
			else
			{
//				draw_astar_object(&new_state->state, CARMEN_RED);
				neighbor.push_back(new_state);
			}
    	}
    }

//    publish_astar_draw();
    return neighbor;
}


double
g(state_node *current)
{
	return current->g;
}


double
movementcost(state_node *current, state_node *neighbor)
{
	return (neighbor->distance_traveled_g);
}


double
carmen_compute_abs_angular_distance(double theta_1, double theta_2)
{
	return (carmen_normalize_theta(abs(theta_1 - theta_2)));
}


std::vector<state_node*>
reed_shepp_path(state_node *current, state_node *goal_state)
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
	std::vector<state_node*> rs_path_nodes;

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);
	double rs_length = reed_shepp(current->state, goal_state->state, &rs_numero, &tr, &ur, &vr);

	rs_pathl = constRS(rs_numero, tr, ur, vr, current->state, rs_points);

	for (int i = rs_pathl; i > 0; i--)
	{
		carmen_ackerman_traj_point_t point = rs_points[i];
		if (rs_points[i].v < 0.0)
		{
			v_step = 2.0;
			step_weight = 2.0;
		}
		else
		{
			v_step = -2.0;
			step_weight = 1.0;
		}
		while (DIST2D(point, rs_points[i-1]) > 0.2 || (abs(carmen_compute_abs_angular_distance(point.theta, rs_points[i-1].theta)) > 0.0872665))
		{

			distance_traveled_old = distance_traveled;
			point = carmen_libcarmodel_recalc_pos_ackerman(point, v_step, rs_points[i].phi,
					0.1, &distance_traveled, DELTA_T, robot_config);
			path_cost += step_weight * (distance_traveled - distance_traveled_old);
			state_node_p new_state = (state_node_p) malloc(sizeof(state_node));
			new_state->state = point;
			//Como o Reed Shepp realiza o caminho do goal para um ponto, ele está andando de ré. Por isso precisa-se inverter o sinal de v
			new_state->state.v = -new_state->state.v;
			new_state->f = path_cost;
			rs_path_nodes.push_back(new_state);
		}
	}

	std::reverse(rs_path_nodes.begin(), rs_path_nodes.end());
	state_node_p ant_state = (state_node_p) malloc(sizeof(state_node));
	ant_state = current;
	for (int i = 0; i<rs_path_nodes.size(); i++)
	{
		rs_path_nodes[i]->parent = ant_state;
		ant_state = rs_path_nodes[i];
	}
	return rs_path_nodes;
}


double
reed_shepp_cost(carmen_ackerman_traj_point_t current, carmen_ackerman_traj_point_t goal)
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
	carmen_ackerman_traj_point_t point_old = {0, 0, 0, 0, 0};

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);
	double rs_length = reed_shepp(current, goal, &rs_numero, &tr, &ur, &vr);

	rs_pathl = constRS(rs_numero, tr, ur, vr, current, rs_points);
	for (int i = rs_pathl; i > 0; i--)
	{
		carmen_ackerman_traj_point_t point = rs_points[i];
		if (rs_points[i].v < 0.0)
		{
			v_step = 2.0;
			step_weight = 1.0;
		}
		else
		{
			v_step = -2.0;
			step_weight = 1.0;
		}
		while (DIST2D(point, rs_points[i-1]) > 0.2 || (abs(carmen_compute_abs_angular_distance(point.theta, rs_points[i-1].theta)) > 0.0872665))
		{
			distance_traveled_old = distance_traveled;
			point_old = point;
			point = carmen_libcarmodel_recalc_pos_ackerman(point, v_step, rs_points[i].phi,
					0.1, &distance_traveled, DELTA_T, robot_config);
			path_cost += step_weight * (distance_traveled - distance_traveled_old);

		}
	}

	return path_cost;
}


int
hitObstacle(std::vector<state_node*> path, map_node_p ***astar_map )
{
	for(int i = 0; i < path.size(); i++)
	{
		discrete_pos_node *current_pos = get_current_pos(path[i]);
		if(astar_map[current_pos->x][current_pos->y][current_pos->theta]->obstacle_distance < 2.0)
		{
			free(current_pos);
			return 1;
		}
		free(current_pos);
	}
	return 0;
}


double
h(map_node_p ***astar_map, double* heuristic_obstacle_map, state_node *current, state_node *goal)
{
	double ho = -DBL_MAX;
	double rs = -DBL_MAX;
	discrete_pos_node *current_pos = get_current_pos(current);

	int current_x = round((current->state.x - distance_map->config.x_origin)/distance_map->config.resolution);
	int current_y = round((current->state.y - distance_map->config.y_origin)/distance_map->config.resolution);
	//Multiplicar o ho pela resolução do mapa porque parece que ele considera cada célula com o tamanho de 1, em vez de 0.2
	//então o valor do ho fica praticamente sempre maior que o da heuristica sem obstáculos
	ho = (heuristic_obstacle_map[current_y + current_x * distance_map->config.y_size] * distance_map->config.resolution) ;

	if(astar_config.use_matrix_cost_heuristic)
	{
		int x = ((current->state.x - goal->state.x) * cos(current->state.theta) - (current->state.y - goal->state.y) * sin(current->state.theta))/astar_config.precomputed_cost_resolution;
		int y = ((current->state.x - goal->state.x) * sin(current->state.theta) + (current->state.y - goal->state.y) * cos(current->state.theta))/astar_config.precomputed_cost_resolution;
		int theta;

		if ((x <= 0 && y >= 0) || (x >= 0 && y <= 0))
			theta = get_astar_map_theta(carmen_normalize_theta(-(goal->state.theta - current->state.theta)), astar_config.precomputed_cost_theta_size);
		else
			theta = get_astar_map_theta(carmen_normalize_theta(goal->state.theta - current->state.theta), astar_config.precomputed_cost_theta_size);


		int half_map = round(((astar_config.precomputed_cost_size)/astar_config.precomputed_cost_resolution) / 2);
	//	printf("Real x = %d y= %d theta = %d\n", x + half_map, y + half_map, theta);
		if(x < half_map && y < half_map && x > -half_map && y > -half_map)
		{
			rs =  heuristic_without_obstacle_map[x + half_map][y + half_map][theta]->h;
		}
	}
	else
	{
		double rs_cost = reed_shepp_cost(current->state, goal->state);
		rs = rs_cost;

	}
//	printf("[h]rs = %f\tho = %f\n", rs, ho);
	if(rs == -1)
		ho+=10;

	int returned_h = std::max(rs, ho);
	if(returned_h > 0)
		returned_h = returned_h - (astar_map[current_pos->x][current_pos->y][current_pos->theta]->obstacle_distance * 2.0);

	free(current_pos);
	return returned_h;
}


void
update_neighbors(map_node_p ***astar_map, double* heuristic_obstacle_map ,state_node *current, state_node *goal_state, boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> &open)
{

	std::vector<state_node*> neighbor_expansion = expansion(current, goal_state, astar_map);

	int it_neighbor_number = 0;
	double current_node_cost = 0.0;

	while(it_neighbor_number < neighbor_expansion.size())
	{
		current_node_cost = g(current) + movementcost(current, neighbor_expansion[it_neighbor_number]);
		discrete_pos_node *current_pos = get_current_pos(neighbor_expansion[it_neighbor_number]);

		if(astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_open == 1 && astar_map[current_pos->x][current_pos->y][current_pos->theta]->g > neighbor_expansion[it_neighbor_number]->g)
		{
			astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_open = 0;

		}

		if(astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_closed == 0 && astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_open == 0)
		{
			neighbor_expansion[it_neighbor_number]->g = current_node_cost;
			neighbor_expansion[it_neighbor_number]->h = h(astar_map, heuristic_obstacle_map ,neighbor_expansion[it_neighbor_number], goal_state);
			neighbor_expansion[it_neighbor_number]->f = neighbor_expansion[it_neighbor_number]->g + neighbor_expansion[it_neighbor_number]->h;
			neighbor_expansion[it_neighbor_number]->parent = current;

			//Penalidades
			if(neighbor_expansion[it_neighbor_number]->state.v < 0)
			{
				neighbor_expansion[it_neighbor_number]->g = (2.0 * neighbor_expansion[it_neighbor_number]->distance_traveled_g) + current->g;
				neighbor_expansion[it_neighbor_number]->f = neighbor_expansion[it_neighbor_number]->g + neighbor_expansion[it_neighbor_number]->h;
			}

			if(neighbor_expansion[it_neighbor_number]->state.v != current->state.v)
			{
				neighbor_expansion[it_neighbor_number]->g +=SQRT2+2;
				neighbor_expansion[it_neighbor_number]->f = neighbor_expansion[it_neighbor_number]->g + neighbor_expansion[it_neighbor_number]->h;
			}

			if(neighbor_expansion[it_neighbor_number]->state.phi != 0.0)
			{
				neighbor_expansion[it_neighbor_number]->g +=SQRT2+1;
				neighbor_expansion[it_neighbor_number]->f = neighbor_expansion[it_neighbor_number]->g + neighbor_expansion[it_neighbor_number]->h;
			}

			if(neighbor_expansion[it_neighbor_number]->state.phi != current->state.phi)
			{
				neighbor_expansion[it_neighbor_number]->g +=SQRT2;
				neighbor_expansion[it_neighbor_number]->f = neighbor_expansion[it_neighbor_number]->g + neighbor_expansion[it_neighbor_number]->h;
			}

			astar_map_open_node(astar_map, current_pos);
			astar_map[current_pos->x][current_pos->y][current_pos->theta]->g = neighbor_expansion[it_neighbor_number]->g;
			open.push(neighbor_expansion[it_neighbor_number]);
			free(current_pos);
		}
		it_neighbor_number++;
	}
}

offroad_planner_plan_t
astar_mount_offroad_planner_plan(carmen_point_t *robot_pose, carmen_point_t *goal_pose)
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
	plan.path.points = &(carmen_astar_path_poses[0]);
	plan.path.length = astar_path_poses_size;
	plan.path.capacity = 0;

	return plan;
}


int
carmen_path_planner_astar_get_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose)
{

	printf("Robot Pose : %f %f %f\n", robot_pose->x, robot_pose->y, robot_pose->theta);
	printf("Goal Pose : %f %f %f\n", goal_pose->x, goal_pose->y, goal_pose->theta);

	virtual_laser_message.num_positions = 0;
	std::vector<state_node*> rs_path;
	std::vector<state_node*> neighbor_expansion;
	state_node *start_state, *goal_state, *current;

	int cont_rs_nodes = 0;

	double* heuristic_obstacle_map = get_obstacle_heuristic_map(goal_pose);
	map_node_p ***astar_map = alloc_astar_map();

	boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> open;
	start_state = create_state_node(robot_pose->x, robot_pose->y, robot_pose->theta, 2.0, 0.0, 0.0, DBL_MAX, DBL_MAX, NULL, 0);
	goal_state = create_state_node(goal_pose->x, goal_pose->y, goal_pose->theta, 0.0, 0.0, 0.0, 0.0, 0.0, NULL, 0);

	// Following the code from: http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html
	open.push(start_state);
	discrete_pos_node *current_pos = get_current_pos(start_state);
	astar_map_open_node(astar_map, current_pos);
	free(current_pos);

	while (!open.empty())
	{
		current = open.top();
		open.pop();

//		Apenas para fazer uma verificação no método que obtém a célula com obstáculo mais próximo
//		carmen_position_t temp = nearest_obstacle_cell(current->state.x, current->state.y);
//		printf("current cell = %f %f\n", current->state.x, current->state.y);
//		printf("nearest_obstacle_cell = %f %f\n",temp.x, temp.y);

		if(is_goal(current, goal_state) == 1)
		{
			printf("Goal encontrado\n");
			break;
		}

		discrete_pos_node *current_pos = get_current_pos(current);

		if(astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_closed == 1)
			continue;

		astar_map_close_node(astar_map, current_pos);
		free(current_pos);

		if(cont_rs_nodes%17==0)
		{
			rs_path = reed_shepp_path(current, goal_state);
			if(hitObstacle(rs_path, astar_map) == 0 && rs_path.front()->f < current->f )
			{
				rs_path.front()->parent = current;
				printf("Current_state: %f %f %f \n", current->state.x, current->state.y, current->state.theta);
				current = rs_path.back();
				current->g = 0;
				current->h = 0;
				current->distance_traveled_g = 0;
				current->f = 0;
				open.push(current);
				printf("Reed Shepp encontrou o caminho \n");
//				printf("Goal_state: %f %f %f \n", goal_state->state.x, goal_state->state.y, goal_state->state.theta);
				break;
			}
		}

		update_neighbors(astar_map, heuristic_obstacle_map, current, goal_state, open);
		neighbor_expansion.clear();
		cont_rs_nodes++;
	}
//	offroad_planner_plan_t plan;

	if(open.empty())
	{
		printf("Path Planner Astar não encontrou o caminho\n");
		return 0;
	}
	else
	{
		current = open.top();
		astar_mount_path_message(current);
//		plan = astar_mount_offroad_planner_plan(robot_pose, goal_pose);
		open.clear();
		if (USE_SMOOTH)
		{
			int res = smooth_rddf_using_conjugate_gradient(&(carmen_astar_path_poses[0]), astar_path_poses_size);

			if(res != 1)
				printf("Otimização falhou, retornou o valor %d\n", res);

		}

	}

	clear_astar_map(astar_map);
	free(heuristic_obstacle_map);
	astar_path_sended = 1;
	return(1);

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
	carmen_point_t robot_position = {msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta};
	current_globalpos_msg = *msg;
	if (final_goal_received != 0)
	{
//		printf("Pos_message_handler: %lf %lf %lf\n", robot_position.x, robot_position.y, robot_position.theta);
		int result_planner = carmen_path_planner_astar_get_path(&robot_position, final_goal);

		if(result_planner)
			plan_path_poses = astar_mount_offroad_planner_plan(&robot_position, final_goal);
		//printf("Primeiro print %f %f %f \n", plan_path_poses.path.points->x, plan_path_poses.path.points->y, plan_path_poses.path.points->theta);

		if(astar_path_sended)
		{
			publish_plan(plan_path_poses.path, msg);

		}

		final_goal_received = 0;

	}

	if(astar_path_sended && DIST2D(robot_position, *route_planner_road_network_message.poses) > 2.0)
	{
//		printf("route_planner print %f %f %f \n", route_planner_road_network_message.poses->x, route_planner_road_network_message.poses->y, route_planner_road_network_message.poses->theta);
//		printf("plan_path print %f %f %f \n", plan_path_poses.path.points->x, plan_path_poses.path.points->y, plan_path_poses.path.points->theta);

		int nearest_pose_index = get_index_of_nearest_pose_in_path(route_planner_road_network_message.poses, robot_position, route_planner_road_network_message.number_of_poses);
		route_planner_road_network_message.poses_back = get_poses_back(route_planner_road_network_message.poses, nearest_pose_index);
		route_planner_road_network_message.poses = &(route_planner_road_network_message.poses[nearest_pose_index]);
		route_planner_road_network_message.number_of_poses = route_planner_road_network_message.number_of_poses - nearest_pose_index;
		route_planner_road_network_message.number_of_poses_back = nearest_pose_index + 1;
		carmen_route_planner_publish_road_network_message(&route_planner_road_network_message);
//		printf("poses reenviadas %f %f %f \n", route_planner_road_network_message.poses->x, route_planner_road_network_message.poses->y, route_planner_road_network_message.poses->theta);
	}

	if(astar_path_sended && DIST2D_P(&robot_position, final_goal) < 6.0){
		printf("Chegou ao destino\n");
		astar_path_sended = 0;
		free_lanes(route_planner_road_network_message);
		free(route_planner_road_network_message.poses_back);
		carmen_astar_path_poses.clear();
	}

	//	if (astar.carmen_offroad_planner_update_robot_pose(&robot_position) == 0)
//		return;

}


static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (compact_distance_map == NULL)
	{
		distance_map = &distance_map_struct;
		carmen_obstacle_distance_mapper_create_new_map(distance_map, message->config, message->host, message->timestamp);
		compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(distance_map, message);
	}
	else
	{
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(distance_map, compact_distance_map, DISTANCE_MAP_HUGE_DISTANCE);
		carmen_obstacle_distance_mapper_free_compact_distance_map(compact_distance_map);
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(distance_map, message);
	}

	distance_map->config = message->config;
//	astar.carmen_offroad_planner_set_obstacle_distance_map(distance_map);
}


void
carmen_path_planner_astar_mapper_handler(carmen_mapper_map_message *message)
{
	if(final_goal_received)
		return;

	static double last_time_stamp = 0.0;

	if (message->size <= 0)
		return;

	// @@@ Alberto: codigo adicionado para atualizar o mapa a uma taxa maxima de 10Hz
	if ((carmen_get_time() - last_time_stamp) > 0.1)
		last_time_stamp = carmen_get_time();
	else
		return;

	if (map_occupancy && (message->config.x_size != map_occupancy->config.x_size || message->config.y_size != map_occupancy->config.y_size))
		carmen_map_destroy(&map_occupancy);

	map_occupancy = copy_grid_mapping_to_map(map_occupancy, message);
	}


static void
carmen_behaviour_selector_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (distance_map)
		carmen_obstacle_distance_mapper_overwrite_distance_map_message_with_compact_distance_map(distance_map, message);
}


static void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	final_goal = &(rddf_end_point_message->point);
	final_goal_received = 1;
}


static void
offroad_planner_shutdown(int signal)
{
	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("Disconnected from IPC. signal = %d\n", signal);
		done = 1;
	}
	exit(0);
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behaviour_selector_subscribe_compact_lane_contents_message(NULL, (carmen_handler_t) carmen_behaviour_selector_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) (carmen_localize_ackerman_globalpos_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) (carmen_rddf_play_end_point_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) carmen_path_planner_astar_mapper_handler, CARMEN_SUBSCRIBE_LATEST);

}


void
define_messages()
{
	carmen_route_planner_define_messages();
}


static void
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = 
	{
			{(char *)"robot",				(char *)"max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},//todo add max_v and max_phi in carmen.ini
			{(char *)"robot",				(char *)"min_approach_dist", CARMEN_PARAM_DOUBLE, &robot_config.approach_dist, 1, NULL},
			{(char *)"robot",				(char *)"min_side_dist", CARMEN_PARAM_DOUBLE, &robot_config.side_dist, 1, NULL},
			{(char *)"robot",				(char *)"length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
			{(char *)"robot",				(char *)"width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
			{(char *)"robot",				(char *)"maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
			{(char *)"robot",				(char *)"reaction_time", CARMEN_PARAM_DOUBLE,	&robot_config.reaction_time, 0, NULL},
			{(char *)"robot",				(char *)"distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
			{(char *)"robot",				(char *)"maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate, 1, NULL},
			{(char *)"robot",				(char *)"distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
			{(char *)"navigator",			(char *)"goal_size", CARMEN_PARAM_DOUBLE, &nav_config.goal_size, 1, NULL},
			{(char *)"navigator",			(char *)"waypoint_tolerance", CARMEN_PARAM_DOUBLE, &nav_config.waypoint_tolerance, 1, NULL},
			{(char *)"navigator",			(char *)"goal_theta_tolerance", CARMEN_PARAM_DOUBLE, &nav_config.goal_theta_tolerance, 1, NULL},
			{(char *)"navigator",			(char *)"map_update_radius", CARMEN_PARAM_DOUBLE,	&nav_config.map_update_radius, 1, NULL},
			{(char *)"navigator",			(char *)"map_update_num_laser_beams", CARMEN_PARAM_INT, &nav_config.num_lasers_to_use, 1, NULL},
			{(char *)"navigator",			(char *)"map_update_obstacles", CARMEN_PARAM_ONOFF, &nav_config.map_update_obstacles, 1, NULL},
			{(char *)"navigator",			(char *)"map_update_freespace", CARMEN_PARAM_ONOFF, &nav_config.map_update_freespace, 1, NULL},
			{(char *)"navigator",			(char *)"replan_frequency", CARMEN_PARAM_DOUBLE, &nav_config.replan_frequency, 1, NULL},
			{(char *)"navigator",			(char *)"dont_integrate_odometry", CARMEN_PARAM_ONOFF, &nav_config.dont_integrate_odometry, 1, NULL},
			{(char *)"navigator",			(char *)"plan_to_nearest_free_point", CARMEN_PARAM_ONOFF,	&nav_config.plan_to_nearest_free_point, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
			{(char *)"path_planner_astar",		(char *)"state_map_resolution", CARMEN_PARAM_DOUBLE, &astar_config.state_map_resolution, 1, NULL},
			{(char *)"path_planner_astar",		(char *)"state_map_theta_resolution", CARMEN_PARAM_INT, &astar_config.state_map_theta_resolution, 1, NULL},
			{(char *)"path_planner_astar",		(char *)"precomputed_cost_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_size, 1, NULL},
			{(char *)"path_planner_astar",		(char *)"precomputed_cost_theta_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_theta_size, 1, NULL},
			{(char *)"path_planner_astar",		(char *)"precomputed_cost_resolution", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_resolution, 1, NULL},
			{(char *)"path_planner_astar",		(char *)"precomputed_cost_file_name", CARMEN_PARAM_STRING, &astar_config.precomputed_cost_file_name, 1, NULL},
			{(char *)"path_planner_astar",		(char *)"use_matrix_cost_heuristic", CARMEN_PARAM_ONOFF, &astar_config.use_matrix_cost_heuristic, 1, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	if(astar_config.use_matrix_cost_heuristic)
		alloc_cost_map();

	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
	virtual_laser_message.host = carmen_get_host();

}


int 
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	define_messages();
	subscribe_messages();
	printf("Aguardando Final Goal\n");

	signal(SIGINT, offroad_planner_shutdown);

	carmen_ipc_dispatch();

	return (0);
}
