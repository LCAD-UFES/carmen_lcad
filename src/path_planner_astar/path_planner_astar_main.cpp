#include <list>
#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>
#include <carmen/rddf_interface.h>
#include <carmen/route_planner_interface.h>

#include "path_planner_astar.h"

// evg-thin

#include <float.h>
#include <math.h>
#include "evg-thin/datatypes.hh"
#include "evg-thin/fileio.hh"
#include "evg-thin/evg-thin.hh"
#include "evg-thin/utils.hh"
#include <string.h>
#include <stdlib.h>
#include <iostream>


#define PRINT_VORONOI 1
char* outfile = (char*) "voronoi_edges.ppm";
unsigned int unknown_min=127;
unsigned int unknown_max=128;
bool pruning = false;
bool robot_close = false;
float distance_min=1.0;
float distance_max=FLT_MAX;
int robot_locx=-1;
int robot_locy=-1;

/////////////////

Time time_count;

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
int first_published = 0;
int last_index_poses = 0;

int astar_map_x_size;
int astar_map_y_size;
carmen_point_t *final_goal = NULL;
carmen_localize_ackerman_globalpos_message current_globalpos_msg;
cost_heuristic_node_p ***heuristic_without_obstacle_map;
std::vector<carmen_ackerman_traj_point_t> carmen_astar_path_poses;
std::vector<carmen_ackerman_traj_point_t> current_astar_path_poses_till_reverse_direction;
carmen_route_planner_road_network_message route_planner_road_network_message;

offroad_planner_plan_t plan_path_poses;
int astar_path_poses_size = 0;

std::vector<voronoi_node> voronoi_points;

voronoi_BFS_node_p **voronoi_map;
int voronoi_visited_it = 0;
//cv::Mat temp_voronoi;

int expansion_number = 0;
int cache_exit_edge;

int reed_shepp_collision = 0;

#define LANE_WIDTH 	2.4
#define NUM_LANES	1
#define MAX_VIRTUAL_LASER_SAMPLES 100000
#define SQRT2 1.414
#define DELTA2D(x1,x2) ((carmen_ackerman_traj_point_t){x1.x - x2.x, x1.y - x2.y, 0.0, 0.0, 0.0})

#define SMOOTHNESS_WEIGHT 1.0
#define OBSTACLE_WEIGHT 1.0
#define CURVATURE_WEIGHT 1.0
#define	VORONOI_WEIGHT 0.0

#define USE_SMOOTH 1
#define USE_NEW_EXPANSION 0
//#define USE_NOBSTACLE_HEURISTIC 1
#define EXPANSION_VELOCITY 1.0

//#define OBSTACLE_DISTANCE_MIN 1.0
#define OBSTACLE_DISTANCE_MIN 0.5
#define SEND_MESSAGE_IN_PARTS 0

int teste_edge = 0;

int use_matrix_heuristic = 0;
using namespace cv;


void
evg_thin_on_map(map_node_p ***astar_map, double *heuristic_obstacle_map)
{
	column_type col(astar_map_y_size,Unknown);
	grid_type curr_grid(astar_map_x_size,col);

	for (int i = 0; i < astar_map_x_size; i++)
	{
		for (int j = 0; j < astar_map_y_size; j++)
		{
			double cell= astar_map[i][j][0]->obstacle_distance;
			if (cell <= 1.5)
				curr_grid[i][j]=Occupied;
			else
				curr_grid[i][j]=Free;
		}
	}

	if (curr_grid.empty() || curr_grid[0].empty())
	{
		cout << "Read grid has no dimensions\n";
		exit(-1);
	}

	if (robot_locx < 0 || robot_locx >= int(curr_grid.size()) ||
		robot_locy < 0 || robot_locy >= int(curr_grid[0].size()))
	{
		robot_locx=curr_grid.size()/2;
		robot_locy=curr_grid[0].size()/2;
	}
/*
	cout << "Pruning: ";
	if (pruning)
		cout << "On\n";
	else
		cout << "Off\n";
	cout << "Robot close: ";
	if (robot_close)
		cout << "On\n";
	else
		cout << "Off\n";
	cout << "Minimum distance: " << distance_min << endl;
	cout << "Maximum distance: " << distance_max << endl;
	cout << "Robot location: "<<robot_locx<<", "<<robot_locy<<endl<<endl;
*/
	evg_thin thin(curr_grid,distance_min,distance_max,pruning,robot_close,robot_locx,robot_locy);

	Time t;
	skeleton_type skel=thin.generate_skeleton();
//	cout << "Skeleton created in "<<t.get_since()<<" seconds\n";
	carmen_position_t foo_value;
	foo_value.x = -2;
	foo_value.y = -2;
	if (PRINT_VORONOI)
	{
		Mat imagem = Mat(astar_map_x_size, astar_map_y_size, CV_8UC3);
		for (int i = 0; i < astar_map_x_size; i++)
		{
			for (int j = 0; j < astar_map_y_size; j++)
			{
				if (astar_map[i][j][0]->obstacle_distance < OBSTACLE_DISTANCE_MIN)
				{
					imagem.at<Vec3b>(i, j)[0] = 0;
					imagem.at<Vec3b>(i, j)[1] = 0;
					imagem.at<Vec3b>(i, j)[2] = 0;
				}
				else
				{
					imagem.at<Vec3b>(i, j)[0] = 255;
					imagem.at<Vec3b>(i, j)[1] = 255;
					imagem.at<Vec3b>(i, j)[2] = 255;
				}
			}
		}

		for (unsigned int i=0;i<skel.size();i++)
		{
			imagem.at<Vec3b>(skel[i].x,skel[i].y)[0] = 0;
			imagem.at<Vec3b>(skel[i].x,skel[i].y)[1] = 0;
			imagem.at<Vec3b>(skel[i].x,skel[i].y)[2] = 255;
//			circle(imagem, Point(skel[i].y,skel[i].x), skel[i].radius, Scalar(255,0,0));
//			printf("Point Skeleton: %d %d \n", skel[i].x,skel[i].y);
			voronoi_map[skel[i].x][skel[i].y]->is_edge = 1;
			voronoi_map[skel[i].x][skel[i].y]->nearest_edge = foo_value;

			voronoi_node temp;
			temp.x = skel[i].x;
			temp.y = skel[i].y;
			temp.h = heuristic_obstacle_map[skel[i].y + skel[i].x * astar_map_y_size];
			temp.already_expanded = 0;
			voronoi_points.push_back(temp);
		}
//		temp_voronoi = imagem.clone();
		imwrite(outfile, imagem);
	}
	else
	{
		for (unsigned int i=0;i<skel.size();i++)
		{

			voronoi_map[skel[i].x][skel[i].y]->is_edge = 1;
			voronoi_map[skel[i].x][skel[i].y]->nearest_edge = foo_value;

			voronoi_node temp;
			temp.x = skel[i].x;
			temp.y = skel[i].y;
			temp.h = heuristic_obstacle_map[skel[i].y + skel[i].x * astar_map_y_size];
			temp.already_expanded = 0;
			voronoi_points.push_back(temp);
		}
	}
}


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
	return round((double) ((x - distance_map->config.x_origin) / astar_config.state_map_resolution));
}


int
get_astar_map_y(double y)
{
	return round((double) ((y - distance_map->config.y_origin) / astar_config.state_map_resolution));
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


void
get_current_pos(state_node* current_state, int &x, int &y, int &theta, int &direction)
{
	x = get_astar_map_x(current_state->state.x);
	y = get_astar_map_y(current_state->state.y);
	theta = get_astar_map_theta(current_state->state.theta, astar_config.state_map_theta_resolution);

	if(current_state->state.v >= 0)
		direction = 0;
	else
		direction = 1;
}



carmen_position_t
nearest_edge_cell(int x_map, int y_map)
{
	carmen_position_t current;
	int expand[3] = {-1,0,1};
	++voronoi_visited_it;
	current.x = x_map;
	current.y = y_map;
	std::vector<carmen_position_t> fila;
	fila.push_back(current);
	while(!fila.empty())
	{
		current = fila.back();
		fila.pop_back();
//		temp_voronoi.at<Vec3b>((int)current.x, (int)current.y)[1] = 0;
//		temp_voronoi.at<Vec3b>((int)current.x, (int)current.y)[2] = 0;
		if(voronoi_map[(int)current.x][(int)current.y]->is_edge == 1)
			return current;
		else
		{
			for(int i = 0; i < 3 ; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					int new_x = current.x + expand[i];
					int new_y = current.y + expand[j];
					if(new_x >= astar_map_x_size || new_x < 0 || new_y >= astar_map_y_size || new_y <  0 ||
					voronoi_map[new_x][new_y]->is_obstacle == 1 || voronoi_map[new_x][new_y]->visited_iteration == voronoi_visited_it)
						continue;
					carmen_position_t new_point;
					new_point.x = new_x;
					new_point.y = new_y;
					fila.insert(fila.begin(), new_point);
					voronoi_map[new_x][new_y]->visited_iteration = voronoi_visited_it;

				}
			}
		}
	}

	carmen_position_t foo;
	foo.x = -3;
	foo.y = -3;
	return foo;
}


static int
sign(double a)
{
	if (a >= 0.0)
		return 1;
	else
		return -1;
}


void
calculate_phi_ahead(carmen_ackerman_traj_point_t *path, int num_poses)
{
	double L = robot_config.distance_between_front_and_rear_axles;

	for (int i = 0; i < (num_poses - 1); i++)
	{
//		if(sign(path[i].v) == sign(path[i+1].v))
		double delta_theta;
		if(path[i].v >= 0)
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
//		if(sign(path[i].v) == sign(path[i+1].v))
//		{
//			path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
			//As vezes ele enviava um phi maior que o maximo
			path[i].phi = carmen_clamp(-0.5227, ((path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0), 0.5227);
//		}
	}
}


void
compute_theta(carmen_ackerman_traj_point_t *path, int num_poses)
{
	for (int i = 0; i < (num_poses - 1); i++)
	{
		if(sign(path[i].v) == sign(path[i+1].v))
		{
			if(path[i].v >= 0.0)
				path[i].theta = carmen_normalize_theta(atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x));
			else
				path[i].theta = carmen_normalize_theta(atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x)+ M_PI);
//				path[i].theta = carmen_normalize_theta(atan2(path[i].y - path[i+1].y, path[i].x - path[i+1].x));
		}
		else
		{
			if(path[i].v >= 0.0)
				path[i].theta = carmen_normalize_theta(atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x));
			else
				path[i].theta = carmen_normalize_theta(atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x) + M_PI);
		}

	}
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
safeAcos(double x)
{
	if (x < -1.0) x = -1.0 ;
	else if (x > 1.0) x = 1.0 ;
	return acos (x) ;
}


double
distance_to_nearest_edge(double x, double y)
{
	int x_map = get_astar_map_x(x);
	int y_map = get_astar_map_y(y);

	carmen_position_t real_position;
	real_position.x = x;
	real_position.y = y;
	if(voronoi_map[x_map][y_map]->nearest_edge.x > 0)
		return DIST2D(real_position, voronoi_map[x_map][y_map]->nearest_edge);

	carmen_position_t edge_cell = nearest_edge_cell(x_map, y_map);
	edge_cell.x = get_distance_map_x(edge_cell.x);
	edge_cell.y = get_distance_map_y(edge_cell.y);
	if(edge_cell.x == -3)
		return 0.0;

	double distance = DIST2D(real_position, edge_cell);
//	printf("Distance_to_nearest_edge = %f Cell = %f %f Edge_cell = %f %f\n", distance, current_cell.x, current_cell.y, edge_cell.x, edge_cell.y);
//	line(temp_voronoi, Point(y_map, x_map), Point(edge_cell.y, edge_cell.x), Scalar(0,255,0));
	voronoi_map[x_map][y_map]->nearest_edge = edge_cell;
	return distance;
}


double
my_f(const gsl_vector *v, void *params)
{

	double dmax = 1.0; // escolher um valor melhor
//	double kmax = robot_config.distance_between_front_and_rear_axles / tan(robot_config.max_phi);
//	double kmax = 0.22; // Encontrar outra forma de obter esse valor
//	double kmax = 0.178571429;
	double kmax = 0.108571429;

	double obstacle_cost = 0.0;
	double curvature_cost = 0.0;
	double smoothness_cost = 0.0;
	double voronoi_cost = 0.0;
	double x, y, obst_x, obst_y, distance, delta_phi;
	double x_i, y_i, x_next, y_next, x_prev, y_prev, displacement;

	double curvature_term, distance_voronoi;
	double voronoi_a = 1.0;
	double voronoi_max_distance = 2.0;

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
		if(!param->anchor_points[i]){

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

			if(param->points[i].v >=0)
				current.theta = atan2(y_next - y_i, x_next - x_i);
			else
				current.theta = atan2(y_i - y_next, x_i - x_next);

			distance = carmen_obstacle_avoider_car_distance_to_nearest_obstacle(current, distance_map);

//			if(distance > 0)
//				obstacle_cost -= distance;

			if(distance <= dmax)
				obstacle_cost += (dmax - distance) * (dmax - distance) * (dmax - distance);

			/*
			if(distance <= voronoi_max_distance)
			{
				distance_voronoi = distance_to_nearest_edge(x_i, y_i);
				voronoi_cost += (voronoi_a / (voronoi_a + distance)) * (distance_voronoi / (distance + distance_voronoi)) *
						(pow(distance - voronoi_max_distance, 2) / pow(voronoi_max_distance, 2));
			}
				*/


			//Utilizando a segunda definição de delta_phi
//			delta_phi = DOT2D(delta_i, delta_i_1)/ (sqrt(DOT2D(delta_i, delta_i)) * sqrt(DOT2D(delta_i_1, delta_i_1)));
//			delta_phi = safeAcos(delta_phi);
//			delta_phi = fabs(delta_phi) / sqrt(DOT2D(delta_i, delta_i));
			delta_phi = abs(atan2(y_next - y_i, x_next - x_i) - atan2(y_i - y_prev, x_i - x_prev));
			delta_phi = delta_phi / sqrt(DOT2D(delta_i, delta_i));

			if(delta_phi > kmax)
			{
				curvature_cost += pow(delta_phi - kmax, 2) * (delta_phi - kmax);
			}

		}
	}


	obstacle_cost = OBSTACLE_WEIGHT * obstacle_cost;
	curvature_cost = CURVATURE_WEIGHT * curvature_cost;
	smoothness_cost = SMOOTHNESS_WEIGHT * smoothness_cost;
	voronoi_cost = VORONOI_WEIGHT * voronoi_cost;
//	printf("costs= %f %f %f %f\n", obstacle_cost, curvature_cost, smoothness_cost, voronoi_cost);
	return obstacle_cost + curvature_cost + smoothness_cost + voronoi_cost;
}


double
single_point_my_f(carmen_ackerman_traj_point_t i, carmen_ackerman_traj_point_t i_prev, carmen_ackerman_traj_point_t i_next)
{

	double dmax = 1.0; // escolher um valor melhor
//	double kmax = robot_config.distance_between_front_and_rear_axles / tan(robot_config.max_phi);
//	double kmax = 0.22; // Encontrar outra forma de obter esse valor
//	double kmax = 0.178571429;
	double kmax = 0.108571429;

	double voronoi_a = 1.0;
	double voronoi_max_distance = 2.0;

	double obstacle_cost = 0.0;
	double curvature_cost = 0.0;
	double smoothness_cost = 0.0;
	double voronoi_cost = 0.0;
	double distance, delta_phi;

	double curvature_term, distance_voronoi;
	carmen_ackerman_traj_point_t delta_i = DELTA2D(i, i_prev);
	carmen_ackerman_traj_point_t delta_i_1 = DELTA2D(i_next, i);
	carmen_ackerman_traj_point_t delta = DELTA2D(delta_i_1, delta_i);
	smoothness_cost +=  DOT2D(delta, delta);

	if(i.v >=0)
		i.theta = atan2(i_next.y - i.y, i_next.x - i.x);
	else
		i.theta = atan2(i.y - i_next.y, i.x - i_next.x);

	distance = carmen_obstacle_avoider_car_distance_to_nearest_obstacle(i, distance_map);
//	if(distance > 0)
//		obstacle_cost -= distance;

	if(distance <= dmax)
		obstacle_cost += (dmax - distance) * (dmax - distance) * (dmax - distance);
	/*
	if(distance <= voronoi_max_distance)
	{
		distance_voronoi = distance_to_nearest_edge(i.x, i.y);

		voronoi_cost += (voronoi_a / (voronoi_a + distance)) * (distance_voronoi / (distance + distance_voronoi)) *
				(pow(distance - voronoi_max_distance, 2) / pow(voronoi_max_distance, 2));
	}
	 */

//	delta_phi = DOT2D(delta_i, delta_i_1)/ (sqrt(DOT2D(delta_i, delta_i)) * sqrt(DOT2D(delta_i_1, delta_i_1)));
//	delta_phi = safeAcos(delta_phi);
//	delta_phi = fabs(delta_phi) / sqrt(DOT2D(delta_i, delta_i));

	delta_phi = abs(atan2(i_next.y - i.y, i_next.x - i.x) - atan2(i.y - i_prev.y, i.x - i_prev.x));
	delta_phi = delta_phi / sqrt(DOT2D(delta_i, delta_i));
//	printf("delta_phi = %f %f\n", delta_phi, kmax);
	if(delta_phi > kmax)
	{
		curvature_cost += pow(delta_phi - kmax, 2) * (delta_phi - kmax);
	}

	obstacle_cost = OBSTACLE_WEIGHT * obstacle_cost;
	curvature_cost = CURVATURE_WEIGHT * curvature_cost;
	smoothness_cost = SMOOTHNESS_WEIGHT * smoothness_cost;
	voronoi_cost = VORONOI_WEIGHT * voronoi_cost;
//	printf("distance = %f %f %f\n", distance, distance_voronoi, delta_phi);
//	printf("costs = %f %f %f %f\n", obstacle_cost, curvature_cost, smoothness_cost, voronoi_cost);

	return obstacle_cost + curvature_cost + smoothness_cost + voronoi_cost;
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



int
smooth_rddf_using_conjugate_gradient(carmen_ackerman_traj_point_t *poses_ahead, int size)
{
//	time_count.reset();
//	printf("Suavização iniciada\n");
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
			gsl_vector_set (v, j++, poses_ahead[i].x);
			gsl_vector_set (v, j++, poses_ahead[i].y);
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
		if(!param->anchor_points[i])
		{
			poses_ahead[i].x = gsl_vector_get (s->x, j++);
			poses_ahead[i].y = gsl_vector_get (s->x, j++);
//			printf("poses_ahead after smoothing = %f %f \n",poses_ahead[i].x, poses_ahead[i].y);
		}
	}

	calculate_theta_and_phi(poses_ahead, size);

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);
//	printf("Terminou a suavização\n");

	/*
	for (int i = 1; i < size; i++)
	{
		printf("%f %f %f %f %f Ponto Ancora=%d\n", poses_ahead[i].x, poses_ahead[i].y, poses_ahead[i].theta, poses_ahead[i].v, poses_ahead[i].phi, param->anchor_points[i]);
	}
*/
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
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_WIDTH(0, LANE_WIDTH);
    	++pose_i;
	}
    for (int i = 0; i < route_planner_road_network_message.number_of_poses; i++)
	{
    	route_planner_road_network_message.nearby_lanes[pose_i] = route_planner_road_network_message.poses[i];
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_WIDTH(0, LANE_WIDTH);
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
get_index_of_nearest_pose_in_current_path(std::vector<carmen_ackerman_traj_point_t> path, carmen_point_t globalpos, int path_length)
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
//	printf("Alo %d %d\n", nearest_pose_index, path_length);
	return (nearest_pose_index);
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

//	carmen_ackerman_traj_point_t *poses_back = (carmen_ackerman_traj_point_t *) malloc((1) * sizeof(carmen_ackerman_traj_point_t));
//	poses_back[0] = path[nearest_pose_index];

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

//	carmen_route_planner_road_network_message route_planner_road_network_message;
	route_planner_road_network_message.poses = &(path.points[nearest_pose_index]);
	route_planner_road_network_message.poses_back = get_poses_back(path.points, nearest_pose_index);
	route_planner_road_network_message.number_of_poses = path.length - nearest_pose_index;
	route_planner_road_network_message.number_of_poses_back = nearest_pose_index + 1;	// tem que ter pelo menos uma pose_back que eh igual aa primeira poses
//	route_planner_road_network_message.number_of_poses_back = 1;
	route_planner_road_network_message.annotations = annotations;
	route_planner_road_network_message.annotations_codes = annotations_codes;
	add_lanes(route_planner_road_network_message, path_copy);
	route_planner_road_network_message.timestamp = globalpos_message->timestamp;
	route_planner_road_network_message.host = carmen_get_host();
	first_published = 0;

	// Change in route planner message //////////////////////////////////////////////////////////////////////
    static int *nearby_lanes_node_ids = NULL;	// Size == nearby_lanes_size. Ids dos nós (poses) de todas as lanes.

	static int *nearby_lanes_merges_indexes = NULL;	// Size == number_of_nearby_lanes. O ponto em nearby_lanes_merges onde começam os merges de cada lane.
	static int *nearby_lanes_merges_sizes = NULL;		// Size == number_of_nearby_lanes. O número de merges de cada lane.

	static int *nearby_lanes_forks_indexes = NULL;	// Size == number_of_nearby_lanes. O ponto em nearby_lanes_forks onde começam os forks de cada lane.
	static int *nearby_lanes_forks_sizes = NULL;		// Size == number_of_nearby_lanes. O número de forks de cada lane.

    nearby_lanes_node_ids = (int *) realloc(nearby_lanes_node_ids, route_planner_road_network_message.nearby_lanes_size * sizeof(int));
    route_planner_road_network_message.nearby_lanes_node_ids = nearby_lanes_node_ids;

    nearby_lanes_merges_indexes = (int *) realloc(nearby_lanes_merges_indexes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_merges_indexes = nearby_lanes_merges_indexes;
    nearby_lanes_merges_sizes = (int *) realloc(nearby_lanes_merges_sizes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_merges_sizes = nearby_lanes_merges_sizes;

    nearby_lanes_forks_indexes = (int *) realloc(nearby_lanes_forks_indexes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_forks_indexes = nearby_lanes_forks_indexes;
    nearby_lanes_forks_sizes = (int *) realloc(nearby_lanes_forks_sizes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_forks_sizes = nearby_lanes_forks_sizes;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	carmen_route_planner_publish_road_network_message(&route_planner_road_network_message);

	/*
	int i;
	time_count.reset();
	for(i = 0; i < route_planner_road_network_message.number_of_poses; i++){
//		printf("route_planner print %f %f %f %f %f\n", route_planner_road_network_message.poses[i].x, route_planner_road_network_message.poses[i].y, route_planner_road_network_message.poses[i].theta, route_planner_road_network_message.poses[i].v, route_planner_road_network_message.poses[i].phi);
//		printf("Collision check: %f \n",carmen_obstacle_avoider_car_distance_to_nearest_obstacle(route_planner_road_network_message.poses[i] ,distance_map));
		carmen_obstacle_avoider_car_distance_to_nearest_obstacle(route_planner_road_network_message.poses[i] ,distance_map);
	}
	printf("Collision check time is %f seconds for %d poses\n", time_count.get_since(), i+1);
*/

	free_lanes(route_planner_road_network_message);
	free(annotations);
	free(annotations_codes);
	free(route_planner_road_network_message.poses_back);
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
	++virtual_laser_message.num_positions;
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


double*
get_obstacle_heuristic_map(carmen_point_t *goal_pose, map_node_p**** astar_map)
{
	printf("Carregando mapa da heurística com obstáculos\n");
	int x_size = astar_map_x_size;
	int y_size = astar_map_y_size;
	double *utility_map = (double *) calloc(x_size * y_size, sizeof(double));
	double *cost_map = (double *) calloc(x_size * y_size, sizeof(double));
	std::fill_n(cost_map, x_size *y_size, -1.0);

	for (int x = 0; x < x_size; x++)
	{
		for(int y = 0; y < y_size; y++)
		{
			if(astar_map[x][y][0][0]->obstacle_distance <= OBSTACLE_DISTANCE_MIN)
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

//	int goal_x = round((goal_pose->x - distance_map->config.x_origin)/distance_map->config.resolution);
//	int goal_y = round((goal_pose->y - distance_map->config.y_origin)/distance_map->config.resolution);
	int goal_x = get_astar_map_x(goal_pose->x);
	int goal_y = get_astar_map_y(goal_pose->y);
	copy_map(utility_map, exact_euclidean_distance_to_goal.pathDR(goal_y, goal_x),x_size, y_size);

	for (int i = 0; i < x_size * y_size; i++)
		if (utility_map[i] >= 50000.0) // O infinito de distacia eh representado como 50000.0, assim como o espaco ocupado.
			utility_map[i] = 1000.0;

	save_map((char *) "obstacle_heuristic.map", utility_map, x_size, y_size);
	printf("Mapa da heurística com obstáculos carregado!\n");
	free(cost_map);
	return utility_map;
}


map_node_p ****
alloc_astar_map()
{
	map_node_p ****astar_map;
	astar_map_x_size = round((distance_map->config.x_size * distance_map->config.resolution) / astar_config.state_map_resolution);
	astar_map_y_size = round((distance_map->config.y_size * distance_map->config.resolution)/ astar_config.state_map_resolution);
	printf("Distance map origin: %f %f\n", distance_map->config.x_origin, distance_map->config.y_origin);
	//exit(1);
	int theta_size = astar_config.state_map_theta_resolution;
	double pos_x = 0.0;
	double pos_y = 0.0;
	astar_map = (map_node_p ****)calloc(astar_map_x_size, sizeof(map_node_p***));
	carmen_test_alloc(astar_map);
	voronoi_map = (voronoi_BFS_node_p **)calloc(astar_map_x_size, sizeof(voronoi_BFS_node_p*));
	carmen_test_alloc(voronoi_map);

	carmen_position_t foo_value;
	foo_value.x = -1;
	foo_value.y = -1;

	for (int i = 0; i < astar_map_x_size; i++)
	{
		astar_map[i] = (map_node_p ***)calloc(astar_map_y_size, sizeof(map_node_p**));
		carmen_test_alloc(astar_map[i]);
		voronoi_map[i] = (voronoi_BFS_node_p*)calloc(astar_map_y_size, sizeof(voronoi_BFS_node_p));
		carmen_test_alloc(voronoi_map[i]);

		for (int j = 0; j < astar_map_y_size; j++)
		{
			astar_map[i][j] = (map_node_p**)calloc(theta_size, sizeof(map_node_p*));
			carmen_test_alloc(astar_map[i][j]);
			voronoi_map[i][j] = (voronoi_BFS_node_p)calloc(astar_map_y_size, sizeof(voronoi_BFS_node));
			carmen_test_alloc(voronoi_map[i][j]);
			pos_x = get_distance_map_x(i);
			pos_y = get_distance_map_y(j);
			for (int z = 0; z < theta_size; z++)
			{
				astar_map[i][j][z]= (map_node_p*) calloc(2, sizeof(map_node_p));
				carmen_test_alloc(astar_map[i][j][z]);

				for (int a = 0; a < 2 ; a++)
				{
					astar_map[i][j][z][a]= (map_node_p) malloc(sizeof(map_node));
					if(is_valid_grid_value(i, j, astar_config.state_map_resolution) == 1)
					{
						astar_map[i][j][z][a]->obstacle_distance = obstacle_distance(pos_x, pos_y);
					}
					else
						astar_map[i][j][z][a]->obstacle_distance = 0;

					astar_map[i][j][z][a]->is_closed = 0;
					astar_map[i][j][z][a]->is_open = 0;
				}
			}
			voronoi_map[i][j]->is_edge = 0;
			voronoi_map[i][j]->nearest_edge = foo_value;
			voronoi_map[i][j]->visited_iteration = 0;
			if(is_valid_grid_value(i, j, astar_config.state_map_resolution) == 1 && obstacle_distance(pos_x, pos_y) > OBSTACLE_DISTANCE_MIN)
				voronoi_map[i][j]->is_obstacle = 0;
			else
				voronoi_map[i][j]->is_obstacle = 1;

		}
	}
	return astar_map;
}


void
clear_astar_map(map_node_p ****astar_map)
{
	int theta_size = astar_config.state_map_theta_resolution;
	for (int i = 0; i < astar_map_x_size; i++)
	{
		for (int j = 0; j < astar_map_y_size; j++)
		{
			for (int k = 0; k < theta_size; k++)
			{
				for(int l = 0; l < 2; l++)
					free(astar_map[i][j][k][l]);
				free(astar_map[i][j][k]);
			}
			free(astar_map[i][j]);
			free(voronoi_map[i][j]);
		}
		free(astar_map[i]);
		free(voronoi_map[i]);
	}
	free(astar_map);
	free(voronoi_map);
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
		use_matrix_heuristic = 0;
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
	use_matrix_heuristic = 1;
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


void
clear_list(std::vector<state_node*> lista)
{
	if(lista.size() != 0)
	{
		for(int i = 0; i < lista.size(); i++)
		{
			state_node *temp = lista.back();
			lista.pop_back();
			free(temp);
		}
		lista.clear();

	}
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
	new_state->total_distance_traveled = 0.0;

	return (new_state);
}


int
is_goal(state_node* current_state, state_node* goal_state)
{
	if(DIST2D(current_state->state, goal_state->state) < 0.9 && abs(carmen_radians_to_degrees(current_state->state.theta) - carmen_radians_to_degrees(goal_state->state.theta)) < 5)
		return 1;
	else
		return 0;
}


void
astar_map_close_node(map_node_p ****astar_map, int x, int y, int theta, int direction)
{
	astar_map[x][y][theta][direction]->is_closed = 1;
	astar_map[x][y][theta][direction]->is_open = 0;
}


void
astar_map_open_node(map_node_p ****astar_map, int x, int y, int theta, int direction)
{
	astar_map[x][y][theta][direction]->is_closed = 0;
	astar_map[x][y][theta][direction]->is_open = 1;
}


int
is_valid_state(state_node *state, map_node_p ****astar_map)
{
	if(state->state.x < distance_map->config.x_origin || state->state.y < distance_map->config.y_origin)
		return 0;

	int x;
	int y;
	int theta;
	int direction;
	get_current_pos(state, x, y, theta, direction);
	if(x >= astar_map_x_size || y >= astar_map_y_size || x < 0 || y < 0 || carmen_obstacle_avoider_car_distance_to_nearest_obstacle(state->state, distance_map) < OBSTACLE_DISTANCE_MIN)//astar_map[x][y][theta]->obstacle_distance < 1.5)
	{
		return 0;
	}
//	printf("Current_pos = %d %d\n", x, y);
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


int
get_edge_in_vision(state_node *current)
{
	// Esse método serve para obter a edge (que é obtida pelo método do voronoi) com menor valor de h(n) (o que significa que é a edge mais próxima do goal)
	// no qual faz uma linha reta com o ponto atual do carro que não bate em um obstáculo
	int x_c;
	int y_c;
	int theta_c;
	int direction_c;
	get_current_pos(current, x_c, y_c, theta_c, direction_c);
	int index_voronoi_point = -1;
	int x, y;
	double min_h = DBL_MAX;
	carmen_vector_2D_t min_h_point;
	min_h_point.x = -1;
	min_h_point.y = -1;
	carmen_bresenham_param_t params;
	int free_line;

	for(int i = 0; i < voronoi_points.size(); i++)
	{
		carmen_get_bresenham_parameters(x_c, y_c, voronoi_points[i].x, voronoi_points[i].y, &params);
		carmen_get_current_point(&params, &x, &y);
		free_line = 1;

		while (carmen_get_next_point(&params))
		{
			carmen_get_current_point(&params, &x, &y);
			if(voronoi_map[x][y]->is_obstacle == 1)
			{
				free_line = 0;
				break;
			}
		}

		if(free_line && voronoi_points[i].h < min_h)
		{
			min_h = voronoi_points[i].h;
			min_h_point.x = voronoi_points[i].x;
			min_h_point.y = voronoi_points[i].y;
			index_voronoi_point = i;
		}


	}
	return index_voronoi_point;

}


std::vector<state_node>
build_state_path(state_node *node)
{
	std::vector<state_node> state_path;
	int i = 0;
    while (node != NULL)
    {
//    	printf("state_path = %d %f %f\n",i ,node->state.x, node->state.y);
        state_path.push_back(*node);
//        printf("before failsafe %d %f %f %f\n", i, node->state.x, node->state.y, node->state.theta);
        node = node->parent;
        ++i;

        if(i>5000)
        {
        	printf("Failsafe %f %f %f\n", node->state.x, node->state.y, node->state.theta);
        	break;
        }
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
	carmen_ackerman_traj_point_t last_state;
	temp_rddf_poses_from_path.push_back(path[0].state);
	last_state = path[0].state;
	for (int i = 1; i < path.size(); i++)
	{
		//&& DIST2D(path[i].state, last_state) <= 0.5)
		if((DIST2D(path[i].state, last_state) >= 0.5 )|| (sign(path[i].state.v) != sign(last_state.v)))
		{
			temp_rddf_poses_from_path.push_back(path[i].state);
			last_state = path[i].state;
			printf("[build_rddf_poses] %f %f %f %f %f\n", path[i].state.x, path[i].state.y, path[i].state.theta, path[i].state.v, path[i].state.phi);
		}
		/*
		else if(DIST2D(path[i].state, last_state) > 0.5)
		{
			carmen_ackerman_traj_point_t middle_state;
			middle_state.x = (path[i].state.x + last_state.x)/2.0;
			middle_state.y = (path[i].state.y + last_state.y)/2.0;
			middle_state.theta = (path[i].state.theta + last_state.theta)/2.0;
			middle_state.phi = (path[i].state.phi + last_state.phi)/2.0;
			middle_state.v = path[i].state.v;
			temp_rddf_poses_from_path.push_back(middle_state);
			printf("[middle_state____] %f %f %f %f %f\n", middle_state.x, middle_state.y, middle_state.theta, middle_state.v, middle_state.phi);
			temp_rddf_poses_from_path.push_back(path[i].state);
			last_state = path[i].state;
			printf("[build_rddf_poses] %f %f %f %f %f\n", path[i].state.x, path[i].state.y, path[i].state.theta, path[i].state.v, path[i].state.phi);

		}
		*/

//		distance_to_nearest_edge(path[i].state.x, path[i].state.y);
//		printf("[build_rddf_poses] %f %f %f %f %f\n", path[i].state.x, path[i].state.y, path[i].state.theta, path[i].state.v, path[i].state.phi);
//		draw_astar_object(&path[i].state, CARMEN_GREEN);
	}
//	imwrite("voronoi_lines.ppm", temp_voronoi);
	return temp_rddf_poses_from_path;
}


void
astar_mount_path_message(state_node *current_state)
{
	//std::vector<carmen_ackerman_traj_point_t> temp_rddf_poses_from_path = build_rddf_poses(current_state);
	//carmen_astar_path_poses = temp_rddf_poses_from_path[0];
	//std::copy(temp_rddf_poses_from_path.begin(), temp_rddf_poses_from_path.end(), carmen_astar_path_poses);
//	carmen_astar_path_poses.clear();
	std::vector<carmen_ackerman_traj_point_t>().swap(carmen_astar_path_poses);
	carmen_astar_path_poses = build_rddf_poses(current_state);
	astar_path_poses_size = carmen_astar_path_poses.size();


//	printf("%f\n",DIST2D(current_globalpos_msg.globalpos, goal_state->state));
//	printf("Chegou ao fim do path!\n");
}


double
carmen_compute_abs_angular_distance(double theta_1, double theta_2)
{
	return (carmen_normalize_theta(abs(theta_1 - theta_2)));
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
			v_step = EXPANSION_VELOCITY;
			step_weight = 1.0;
		}
		else
		{
			v_step = -EXPANSION_VELOCITY;
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


double
h(map_node_p ****astar_map, double* heuristic_obstacle_map, state_node *current, state_node *goal)
{
	double ho = -1;
	double rs = -1;

	int x_c;
	int y_c;
	int theta_c;
	int direction_c;
	get_current_pos(current, x_c, y_c, theta_c, direction_c);

	ho = heuristic_obstacle_map[y_c + x_c * astar_map_y_size];
	if(astar_config.use_matrix_cost_heuristic)
	{
		if(use_matrix_heuristic)
	//	if(0)
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
	}
//	printf("[h]rs = %f\tho = %f\n", rs, ho);

	double returned_h = std::max(rs, ho);

	return returned_h;
}


void
exit_expansion(state_node *current, double edge_in_vision_theta, int edge_in_vision, map_node_p ****astar_map, boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> &open, double* heuristic_obstacle_map, state_node *goal_state)
{
	int first_fail = 0;
	int count_expansions = 0;
	double distance_traveled = 0.0;
	double target_phi;
	double time_lenght;
	int size_for;
	state_node_p old_state = (state_node_p) malloc(sizeof(state_node));
	carmen_test_alloc(old_state);
	target_phi = carmen_clamp(-robot_config.max_phi, (current->state.phi + (edge_in_vision_theta - current->state.theta)), robot_config.max_phi);
	old_state->state = carmen_conventional_astar_ackerman_kinematic_3(current->state, robot_config.distance_between_front_and_rear_axles, target_phi, EXPANSION_VELOCITY);
	int x_c;
	int y_c;
	int theta_c;
	int direction_c;
	get_current_pos(old_state, x_c, y_c, theta_c, direction_c);

	if(is_valid_state(old_state, astar_map) == 1 && sqrt(carmen_square(voronoi_points[edge_in_vision].x - x_c)+carmen_square(voronoi_points[edge_in_vision].y - y_c)) > 2 && DIST2D(old_state->state, goal_state->state) > 5)
	{
		old_state->distance_traveled_g = DIST2D(old_state->state, current->state);
		old_state->parent = current;
		old_state->g = current->g + old_state->distance_traveled_g;
//		astar_map_open_node(astar_map, x_c, y_c, theta_c);
//		astar_map[x_c][y_c][theta_c]->g = old_state->g;
//		open.push(old_state);
	}
	else
	{
		free(old_state);
		first_fail = 1;
	}

	if(first_fail == 0)
	{

		int x_c;
		int y_c;
		int theta_c;
		int direction_c;
		get_current_pos(old_state, x_c, y_c, theta_c, direction_c);
		while(sqrt(carmen_square(voronoi_points[edge_in_vision].x - x_c)+carmen_square(voronoi_points[edge_in_vision].y - y_c)) > 2 && DIST2D(old_state->state, goal_state->state) > 5)
		{
			if(fabs(edge_in_vision_theta - old_state->state.theta) > 0.02)
				target_phi = carmen_clamp(-robot_config.max_phi, (old_state->state.phi + (edge_in_vision_theta - old_state->state.theta)), robot_config.max_phi);
			else
				target_phi = 0.0;

			state_node_p new_state = (state_node_p) malloc(sizeof(state_node));
			carmen_test_alloc(new_state);
//			new_state->state = carmen_conventional_astar_ackerman_kinematic_3(old_state->state, robot_config.distance_between_front_and_rear_axles, target_phi, EXPANSION_VELOCITY);
			new_state->state = carmen_conventional_astar_ackerman_kinematic_3(old_state->state, SQRT2, target_phi, EXPANSION_VELOCITY);

			if(is_valid_state(new_state, astar_map) == 1)
			{
				count_expansions++;
				new_state->distance_traveled_g = DIST2D(new_state->state, old_state->state);
				new_state->parent = old_state;
				new_state->g = old_state->g + new_state->distance_traveled_g;
				new_state->g = new_state->g * 0.5;
				new_state->h = h(astar_map, heuristic_obstacle_map ,new_state, goal_state);
				new_state->f = (new_state->g + new_state->h) * 0.5;
				new_state->state.v = 1.0;
//				printf("Push old_state =\t%f\t%f\t%f\n", old_state->state.x, old_state->state.y, old_state->state.theta);
//				printf("Push parent =    \t%f\t%f\t%f\n", new_state->parent->state.x, new_state->parent->state.y, new_state->parent->state.theta);
//				printf("Push new_state =\t%f\t%f\t%f\t%f\n", new_state->state.x, new_state->state.y, new_state->state.theta, new_state->f);
//				printf("Push  = %f %f %f\n", sqrt(carmen_square(edge_in_vision.x - x_c)+carmen_square(edge_in_vision.y - y_c)), edge_in_vision.x, edge_in_vision.y );
				old_state = new_state;
//				open.push(new_state);
				get_current_pos(old_state, x_c, y_c, theta_c, direction_c);
//				astar_map_open_node(astar_map, x_c, y_c, theta_c);
//				astar_map[x_c][y_c][theta_c]->g = new_state->g;
	//			printf("Current_pos = %d %d\n", x_c, y_c);
				teste_edge = 1;
			}
			else
			{
				free(new_state);
				break;
			}
		}
//		printf("Count_expansions = %d\n",count_expansions);
		open.push(old_state);
		get_current_pos(old_state, x_c, y_c, theta_c, direction_c);
		astar_map_open_node(astar_map, x_c, y_c, theta_c, direction_c);
		astar_map[x_c][y_c][theta_c][direction_c]->g = old_state->g;


	}
}


std::vector<state_node*>
expansion(state_node *current, state_node *goal_state, map_node_p ****astar_map)
{
    std::vector<state_node*> neighbor;
    double distance_traveled = 0.0;
    double target_phi;
//    double steering_acceleration[3] = {0.4, -0.4, 0.0};
    double steering_acceleration[3] = {robot_config.max_phi, -robot_config.max_phi, 0.0};
    double target_v[2]   = {EXPANSION_VELOCITY, -EXPANSION_VELOCITY};
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
        	new_state->state = carmen_conventional_astar_ackerman_kinematic_3(current->state, robot_config.distance_between_front_and_rear_axles, target_phi, target_v[i]);
//        	new_state->state = carmen_libcarmodel_recalc_pos_ackerman(current->state, target_v[i], target_phi, 1.0 , &distance_traveled, DELTA_T, robot_config);

//			new_state->distance_traveled_g = SQRT2 * astar_config.state_map_resolution;
			if(is_valid_state(new_state, astar_map) == 1)
			{
				new_state->distance_traveled_g = DIST2D(new_state->state, current->state);
				neighbor.push_back(new_state);
			}
			else
				free(new_state);

    	}
    }

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
			v_step = EXPANSION_VELOCITY;
			step_weight = 1.0;
		}
		else
		{
			v_step = -EXPANSION_VELOCITY;
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
			new_state->state.theta = carmen_normalize_theta(new_state->state.theta);
			new_state->f = path_cost;
//			printf("Step weight = %f %f \n", step_weight, new_state->state.v);
			rs_path_nodes.push_back(new_state);
			if(carmen_obstacle_avoider_car_distance_to_nearest_obstacle(new_state->state, distance_map) < OBSTACLE_DISTANCE_MIN)
			{
				reed_shepp_collision = 1;
				break;
			}
		}
	}
	if(reed_shepp_collision == 1)
	{
		return rs_path_nodes;
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


int
hitObstacle(std::vector<state_node*> path, map_node_p ****astar_map )
{
	for(int i = 0; i < path.size(); i++)
	{
		if(carmen_obstacle_avoider_car_distance_to_nearest_obstacle(path[i]->state, distance_map) < OBSTACLE_DISTANCE_MIN || sign(path[i]->state.v) != sign(path[i]->parent->state.v))
			return 1;
	}
	return 0;
}


void
update_neighbors(map_node_p ****astar_map, double* heuristic_obstacle_map ,state_node *current, state_node *start_state, state_node *goal_state, boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> &open)
{

	int x;
	int y;
	int theta;
	int direction;
	std::vector<state_node*> neighbor_expansion = expansion(current, goal_state, astar_map);
	++expansion_number;

	int it_neighbor_number = 0;
	double current_node_cost = 0.0;

	while(it_neighbor_number < neighbor_expansion.size())
	{
		current_node_cost = g(current) + movementcost(current, neighbor_expansion[it_neighbor_number]);
		get_current_pos(neighbor_expansion[it_neighbor_number], x, y, theta, direction);

		if(astar_map[x][y][theta][direction]->is_open == 1 && astar_map[x][y][theta][direction]->g > neighbor_expansion[it_neighbor_number]->g)
			astar_map[x][y][theta][direction]->is_open = 0;

		if(astar_map[x][y][theta][direction]->is_closed == 0 && astar_map[x][y][theta][direction]->is_open == 0)
		{
			neighbor_expansion[it_neighbor_number]->g = current_node_cost;
			neighbor_expansion[it_neighbor_number]->h = h(astar_map, heuristic_obstacle_map ,neighbor_expansion[it_neighbor_number], goal_state);
			neighbor_expansion[it_neighbor_number]->parent = current;

			//Penalidades
			if(neighbor_expansion[it_neighbor_number]->state.v < 0)
				neighbor_expansion[it_neighbor_number]->g += (1.5 * neighbor_expansion[it_neighbor_number]->distance_traveled_g);

			if(neighbor_expansion[it_neighbor_number]->state.v != current->state.v)
				neighbor_expansion[it_neighbor_number]->g +=5;

			if(neighbor_expansion[it_neighbor_number]->state.phi != current->state.phi)
				neighbor_expansion[it_neighbor_number]->g +=1;

			//Abaixo é uma punição para ele não criar um estado invertido sozinho
			if(current != start_state)
			{
				int first_v = current->parent->state.v;
				int sec_v = current->state.v;
				int thi_v = neighbor_expansion[it_neighbor_number]->state.v;
				if(first_v == thi_v && first_v != sec_v)
					neighbor_expansion[it_neighbor_number]->g +=20;
			}


			neighbor_expansion[it_neighbor_number]->f = neighbor_expansion[it_neighbor_number]->g + neighbor_expansion[it_neighbor_number]->h;

			astar_map_open_node(astar_map, x, y, theta, direction);
			astar_map[x][y][theta][direction]->g = neighbor_expansion[it_neighbor_number]->g;

			neighbor_expansion[it_neighbor_number]->total_distance_traveled = current->total_distance_traveled + neighbor_expansion[it_neighbor_number]->distance_traveled_g;
			open.push(neighbor_expansion[it_neighbor_number]);
		}
		++it_neighbor_number;
	}

/*
	if(USE_NEW_EXPANSION && !(current->state.v < 0) && (cache_exit_edge == -1 || voronoi_points[cache_exit_edge].h > current->h))
	{
		int edge_in_vision = get_edge_in_vision(current);
//		printf("edge_in_vision = %d %d %f %d\n", voronoi_points[edge_in_vision].x, voronoi_points[edge_in_vision].y, voronoi_points[edge_in_vision].h, voronoi_points[edge_in_vision].already_expanded);

		if(edge_in_vision > 0 && voronoi_points[edge_in_vision].already_expanded == 0)
		{
			int x_c;
			int y_c;
			int theta_c;
			int direction_c;
			get_current_pos(current, x_c, y_c, theta_c, direction_c);
	//		printf("current = %d %d\n", x_c, y_c);
			double edge_in_vision_theta = carmen_normalize_theta(atan2((voronoi_points[edge_in_vision].y - y_c), (voronoi_points[edge_in_vision].x - x_c)));
		//	printf("Angle: %f, %f\n", edge_in_vision_theta, edge_in_vision_theta - current->state.theta);

			if(((edge_in_vision_theta - current->state.theta) > -0.4) && ((edge_in_vision_theta - current->state.theta) < 0.4))
			{
				exit_expansion(current, edge_in_vision_theta, edge_in_vision, astar_map, open, heuristic_obstacle_map, goal_state);
				voronoi_points[edge_in_vision].already_expanded = 1;
				cache_exit_edge = edge_in_vision;
			}
		}
	}
*/

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

	// Para enviar um path que não muda de direção
	if(SEND_MESSAGE_IN_PARTS)
	{
		if(carmen_astar_path_poses.size() > 1 && last_index_poses < carmen_astar_path_poses.size()){
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
				if(find_absolute_value > 2000)
				{
					printf("failsafe 2085\n");
					exit(1);
				}
			}

			printf("\n");
			for(int i = last_index_poses + 1; i < carmen_astar_path_poses.size() ; i++)
			{
	//			printf("old_v = %f %f\n", old_v, carmen_astar_path_poses[i].v);
				if(old_v == carmen_astar_path_poses[i].v)
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
		printf("SEND_MESSAGE_IN_PARTS está desligado, a mensagem está enviando o path inteiro\n");
		plan.path.points = &(carmen_astar_path_poses[0]);
		plan.path.length = carmen_astar_path_poses.size();
	}

	plan.path.capacity = 0;

	return plan;
}


int
carmen_path_planner_astar_get_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose)
{
/*
	//Primeiro teste
	robot_pose->x = 7757870.320648;
	robot_pose->y = -363567.511582;
	robot_pose->theta= -0.713516;

	goal_pose->x = 7757917.200000;
	goal_pose->y = -363591.200000;
	goal_pose->theta= -2.245537;

	//Segundo teste

	robot_pose->x = 7757870.318257;
	robot_pose->y = -363567.510643;
	robot_pose->theta= -0.713137;

	goal_pose->x = 7757915.200000;
	goal_pose->y = -363592.800000;
	goal_pose->theta= 0.841942;

	//Terceiro teste
	robot_pose->x = 7757871.909156;
	robot_pose->y = -363568.708179;
	robot_pose->theta= -0.704927;

	goal_pose->x = 7757928.000000;
	goal_pose->y = -363577.400000;
	goal_pose->theta= 2.457696;
*/

	printf("Robot Pose : %f %f %f\n", robot_pose->x, robot_pose->y, robot_pose->theta);
	printf("Goal Pose : %f %f %f\n", goal_pose->x, goal_pose->y, goal_pose->theta);
	teste_edge = 0;
	cache_exit_edge = -1;
	expansion_number = 0;
	time_count.reset();
	virtual_laser_message.num_positions = 0;
	std::vector<state_node*> rs_path;
	state_node *start_state, *goal_state, *current;

	int cont_rs_nodes = 0;
	int rs_found = 0;
	map_node_p ****astar_map = alloc_astar_map();

	if(DIST2D_P(robot_pose, goal_pose) > astar_map_x_size)
	{
		printf("Distância entre robot_pose e goal é maior que o tamanho do mapa local\n");
		return 0;
	}


	double* heuristic_obstacle_map = get_obstacle_heuristic_map(goal_pose, astar_map);
//	printf("Heuristic map time is %f seconds\n\n", time_count.get_since());
//	evg_thin_on_map(astar_map, heuristic_obstacle_map);

	boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> open;
	start_state = create_state_node(robot_pose->x, robot_pose->y, robot_pose->theta, 0.0, 0.0, 0.0, 9999, 9999, NULL, 0);
	goal_state = create_state_node(goal_pose->x, goal_pose->y, goal_pose->theta, 0.0, 0.0, 0.0, 0.0, 0.0, NULL, 0);

	if(carmen_obstacle_avoider_car_distance_to_nearest_obstacle(start_state->state, distance_map) < OBSTACLE_DISTANCE_MIN)
	{
		printf("Robot_pose next to obstacle: %f\n", carmen_obstacle_avoider_car_distance_to_nearest_obstacle(start_state->state, distance_map));
		return 0;
	}

//	if(carmen_obstacle_avoider_car_distance_to_nearest_obstacle(goal_state->state, distance_map) < OBSTACLE_DISTANCE_MIN)
	if(carmen_obstacle_avoider_car_distance_to_nearest_obstacle(goal_state->state, distance_map) < 0.1)
	{
		printf("Goal_pose next to obstacle: %f\n", carmen_obstacle_avoider_car_distance_to_nearest_obstacle(goal_state->state, distance_map));
		return 0;
	}

	// Following the code from: http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html
	open.push(start_state);
	int x;
	int y;
	int theta;
	int direction;
	get_current_pos(start_state, x, y, theta, direction);
	astar_map_open_node(astar_map, x, y, theta, direction);
//	time_count.reset();
	while (!open.empty())
	{
		current = open.top();
		open.pop();
//		Apenas para fazer uma verificação no método que obtém a célula com obstáculo mais próximo
//		carmen_position_t temp = nearest_obstacle_cell(current->state.x, current->state.y);
//		printf("current cell = %f %f %f %f %f %f\n", current->state.x, current->state.y, current->state.theta, current->state.phi, current->f, current->total_distance_traveled);
//		printf("nearest_obstacle_cell = %f %f\n",temp.x, temp.y);

		if(is_goal(current, goal_state) == 1)
		{
			printf("Goal encontrado %f\n", DIST2D(current->state, goal_state->state));
			printf("Current_state: %f %f %f \n", current->state.x, current->state.y, current->state.theta);
			break;
		}

		get_current_pos(current, x, y, theta, direction);

		if(astar_map[x][y][theta][direction]->is_closed == 1)
		{
			free(current);
			continue;
		}

		astar_map_close_node(astar_map, x, y, theta, direction);

//		if(cont_rs_nodes%3==0)
		if(cont_rs_nodes % int(current->h + 1) == 0)
		{
			reed_shepp_collision = 0;
			rs_path = reed_shepp_path(current, goal_state);
//			if(hitObstacle(rs_path, astar_map) == 0)//&& rs_path.front()->f > (current->h) )
			if(reed_shepp_collision == 0)
			{
				rs_path.front()->parent = current;
				current = rs_path.back();
				current->total_distance_traveled = rs_path.front()->parent->total_distance_traveled + rs_path.front()->f;
//				open.push(current);
				rs_found = 1;
//				continue;
				break;
			}

			clear_list(rs_path);

		}

		update_neighbors(astar_map, heuristic_obstacle_map, current, start_state, goal_state, open);
		++cont_rs_nodes;
	}

	free(heuristic_obstacle_map);
//	printf("=====Current after loop %f %f %f\n", current->state.x, current->state.y, current->state.theta);
	if(!open.empty() || rs_found)
	{

//		current = open.top();
		printf("Open List = %ld\n", open.size());
		printf("Length of the path is %f \n", current->total_distance_traveled);
		astar_mount_path_message(current);
		printf("A* Planning time is %f seconds\n", time_count.get_since());

		if (USE_SMOOTH)
		{
			int res = smooth_rddf_using_conjugate_gradient(&(carmen_astar_path_poses[0]), astar_path_poses_size);
			if(res != 1)
				printf("Otimização falhou, retornou o valor %d\n", res);
		}

		printf("Planning time is %f seconds\n", time_count.get_since());
		if(expansion_number > 0){
			printf("Expansion_number = %d \n", expansion_number);
		}

		for(int i = 0; i < open.size(); i++)
		{
			state_node *temp = open.top();
			open.pop();
			free(temp);
		}
		open.clear();
		voronoi_points.clear();
//		carmen_astar_path_poses.erase(carmen_astar_path_poses.begin());
		astar_path_sended = 1;
		last_index_poses = 0;

	}

	clear_astar_map(astar_map);
	free(goal_state);

	if (astar_path_sended == 1)
	{
		return 1;
	}

	else
	{
		printf("Astar não conseguiu encontrar o caminho\n");
		return 0;
	}

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
		int result_planner = carmen_path_planner_astar_get_path(&robot_position, final_goal);

		if(result_planner)
			plan_path_poses = astar_mount_offroad_planner_plan(&robot_position, final_goal);
		if(astar_path_sended)
		{
			publish_plan(plan_path_poses.path, msg);
		}
		final_goal_received = 0;

	}

	if(astar_path_sended && first_published == 0)
	{
		first_published = 1;
	}

	if(astar_path_sended && DIST2D_P(&robot_position, final_goal) < 6.0 && abs(carmen_compute_abs_angular_distance(robot_position.theta, final_goal->theta)) < 0.2617995)
	{
		printf("Chegou ao destino\n");
		astar_path_sended = 0;
		carmen_astar_path_poses.clear();
	}

	else if(astar_path_sended && DIST2D(robot_position, *route_planner_road_network_message.poses) > 4.0)
	{
		publish_plan(plan_path_poses.path, msg);
	}

	else if(astar_path_sended && SEND_MESSAGE_IN_PARTS && msg->v < 0.01 && (DIST2D(robot_position, current_astar_path_poses_till_reverse_direction[current_astar_path_poses_till_reverse_direction.size()-1]) <= 3.0) && abs(carmen_compute_abs_angular_distance(robot_position.theta, current_astar_path_poses_till_reverse_direction[current_astar_path_poses_till_reverse_direction.size()-1].theta)) < (0.2617995*4))//get_index_of_nearest_pose_in_current_path(current_astar_path_poses_till_reverse_direction, robot_position, current_astar_path_poses_till_reverse_direction.size()) > current_astar_path_poses_till_reverse_direction.size() -  3)
	{
		if(last_index_poses < carmen_astar_path_poses.size() - 5){
			plan_path_poses = astar_mount_offroad_planner_plan(&robot_position, final_goal);
			publish_plan(plan_path_poses.path, msg);
		}

	}

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
	{
		carmen_map_destroy(&map_occupancy);
	}

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

void
print_parameters_astar()
{
	printf("Max steering angle = %f\n", robot_config.max_phi );
	printf("State map resolution = %f\n", astar_config.state_map_resolution );
	printf("State map theta resolution = %d\n", astar_config.state_map_theta_resolution );
	printf("Precomputed cost size = %d\n", astar_config.precomputed_cost_size );
	printf("Precomputed cost theta size = %d\n", astar_config.precomputed_cost_theta_size );
	printf("Precomputed cost resolution = %f\n", astar_config.precomputed_cost_resolution );
	printf("Precomputed cost file name = %s\n", astar_config.precomputed_cost_file_name );
	printf("Use matrix cost heuristic = %d\n", astar_config.use_matrix_cost_heuristic );
}


static void
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = 
	{
//			{(char *)"robot", 				(char *) "max_steering_angle",CARMEN_PARAM_DOUBLE, &robot_config.max_phi,1, NULL},
			{(char *)"robot",				(char *)"max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
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
			{(char *)"path_planner_astar",	(char *)"state_map_resolution", CARMEN_PARAM_DOUBLE, &astar_config.state_map_resolution, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"state_map_theta_resolution", CARMEN_PARAM_INT, &astar_config.state_map_theta_resolution, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"precomputed_cost_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_size, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"precomputed_cost_theta_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_theta_size, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"precomputed_cost_resolution", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_resolution, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"precomputed_cost_file_name", CARMEN_PARAM_STRING, &astar_config.precomputed_cost_file_name, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"use_matrix_cost_heuristic", CARMEN_PARAM_ONOFF, &astar_config.use_matrix_cost_heuristic, 1, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	if(astar_config.use_matrix_cost_heuristic)
//	if(0)
		alloc_cost_map();
/*
	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
	virtual_laser_message.host = carmen_get_host();
*/
}


int 
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	define_messages();
	subscribe_messages();
	print_parameters_astar();
	printf("Aguardando Final Goal\n");

	signal(SIGINT, offroad_planner_shutdown);

	carmen_ipc_dispatch();

	return (0);
}
