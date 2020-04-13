#include <list>
#include <carmen/carmen.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>
#include <carmen/collision_detection.h>
#include "g2o/types/slam2d/se2.h"

#include "rddf_util.h"
#include "rddf_index.h"

using namespace std;
using namespace g2o;


extern double distance_between_front_and_rear_axles;
extern double rddf_min_distance_between_waypoints;
extern double traffic_sign_curvature;
extern int traffic_sign_is_on;
extern int state_traffic_sign_code;
extern int traffic_sign_code;
extern double state_traffic_sign_curvature;
extern vector<carmen_annotation_t> annotation_read_from_file;
extern double distance_between_front_car_and_front_wheels;
extern double default_search_radius;
extern double maximum_curvature;

static int state_traffic_sign_is_on = false;
static carmen_point_t state_traffic_sign_pose = {0.0, 0.0, 0.0};
extern int dynamic_plot_state;


bool
is_in_the_zone_of_influence(carmen_point_t robot_pose, carmen_point_t annotation_point, double max_delta_angle, double max_distance)
{
	double delta_angle = fabs(robot_pose.theta - annotation_point.theta);
	double distance = DIST2D(robot_pose, annotation_point);
	bool delta_angle_ok = (carmen_radians_to_degrees(carmen_normalize_theta(delta_angle)) < max_delta_angle);
	bool distance_ok  = (distance < max_distance);
	bool in_the_zone_of_influence = (delta_angle_ok && distance_ok);

	return (in_the_zone_of_influence);
}


bool
is_in_the_zone_of_influence(carmen_point_t robot_pose, carmen_point_t annotation_point, double max_delta_angle, double max_distance_across, double max_distance_aligned)
{
	double delta_angle = fabs(robot_pose.theta - annotation_point.theta);
	double distance = DIST2D(robot_pose, annotation_point);
	double distance_across  = fabs(distance * sin(delta_angle));
	double distance_aligned = fabs(distance * cos(delta_angle));
	bool delta_angle_ok = (carmen_radians_to_degrees(carmen_normalize_theta(delta_angle)) < max_delta_angle);
	bool distance_across_ok  = (distance_across < max_distance_across);
	bool distance_aligned_ok = (distance_aligned < max_distance_aligned);
	bool in_the_zone_of_influence = (delta_angle_ok && distance_across_ok && distance_aligned_ok);

	return (in_the_zone_of_influence);
}


void
find_direction_traffic_sign(carmen_point_t robot_pose, bool ahead)
{
	size_t annotation_index;

	for (annotation_index = 0; annotation_index < annotation_read_from_file.size(); annotation_index++)
	{
		if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN)
		{
			carmen_ackerman_traj_point_t annotation_point;
			annotation_point.x = annotation_read_from_file[annotation_index].annotation_point.x;
			annotation_point.y = annotation_read_from_file[annotation_index].annotation_point.y;
			annotation_point.theta = annotation_read_from_file[annotation_index].annotation_orientation;
			double distance_car_pose_car_front = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
			carmen_point_t actual_annotation_point = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&annotation_point, distance_car_pose_car_front);
			if (is_in_the_zone_of_influence(robot_pose, actual_annotation_point, 70.0, default_search_radius  /*, 1.5 * rddf_min_distance_between_waypoints */ ))
				break;
		}
	}

	if (annotation_index < annotation_read_from_file.size())
	{
		double curvature = annotation_read_from_file[annotation_index].annotation_point.z;
		traffic_sign_curvature = MIN(fabs(curvature), fabs(maximum_curvature)) * sign(curvature);
		traffic_sign_code = annotation_read_from_file[annotation_index].annotation_code;

		if (ahead)
			traffic_sign_is_on = (traffic_sign_code != RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF);
		else
			traffic_sign_is_on = (traffic_sign_code == RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF);
	}
}


void
calculate_phi_ahead(carmen_ackerman_traj_point_t *path, int num_poses)
{
	double L = distance_between_front_and_rear_axles;

	for (int i = 0; i < (num_poses - 1); i++)
	{
		double delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i + 1]);
		if (l < 0.01)
		{
			path[i].phi = 0.0;
			continue;
		}
		path[i].phi = L * atan(delta_theta / l);
	}

	for (int i = 1; i < (num_poses - 1); i++)
	{
		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
	}
}


void
calculate_phi_back(carmen_ackerman_traj_point_t *path, int num_poses)
{
	double L = distance_between_front_and_rear_axles;

	for (int i = (num_poses - 1); i > 0; i--)
	{
		double delta_theta = carmen_normalize_theta(path[i - 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i - 1]);
		if (l < 0.01)
		{
			path[i].phi = 0.0;
			continue;
		}
		path[i].phi = L * atan(delta_theta / l);
	}

	for (int i = (num_poses - 2); i > 0; i--)
	{
		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
	}
}


void
calculate_theta_ahead(carmen_ackerman_traj_point_t *path, int num_poses)
{
	for (int i = 0; i < (num_poses - 1); i++)
		path[i].theta = atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
	if (num_poses > 1)
		path[num_poses - 1].theta = path[num_poses - 2].theta;
}


void
calculate_theta_back(carmen_ackerman_traj_point_t *path, int num_poses)
{
	for (int i = 1; i < num_poses; i++)
//		path[i].theta = atan2(path[i - 1].y - path[i].y, path[i - 1].x - path[i].x);
		path[i].theta = carmen_normalize_theta(atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x) + M_PI);
}


void
calculate_theta_and_phi(carmen_ackerman_traj_point_t *poses_ahead, int num_poses_ahead,
		carmen_ackerman_traj_point_t *poses_back, int num_poses_back)
{
	calculate_theta_ahead(poses_ahead, num_poses_ahead);
	poses_back[0].theta = poses_ahead[0].theta;
	calculate_theta_back(poses_back, num_poses_back);

	calculate_phi_ahead(poses_ahead, num_poses_ahead);
	poses_back[0].phi = poses_ahead[0].phi;
	calculate_phi_back(poses_back, num_poses_back);
}


//////////////////////////////////////////////////////////////////////////////////

/* GSL - GNU Scientific Library
 * Multidimensional Minimization
 * https://www.gnu.org/software/gsl/doc/html/multimin.html
 *
 * Sebastian Thrun
 */


//Function to be minimized summation[x(i+1)-2x(i)+x(i-1)]
double
my_f(const gsl_vector *v, void *params)
{
	list<carmen_ackerman_traj_point_t> *p = (list<carmen_ackerman_traj_point_t> *) params;
	int i, j, size = (p->size() - 2);           //we have to discount the first and last point that wont be optimized
	double a = 0.0, b = 0.0, sum = 0.0;

	double x_prev = p->front().x;				//x(i-1)
	double x      = gsl_vector_get(v, 0);		//x(i)
	double x_next = gsl_vector_get(v, 1);		//x(i+1)

	double y_prev = p->front().y;
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);

	for (i = 2, j = (size+2); i < size; i++, j++)
	{
		a = x_next - (2*x) + x_prev;
		b = y_next - (2*y) + y_prev;
		sum += (a*a + b*b);

		x_prev = x;
		x      = x_next;
		x_next = gsl_vector_get(v, i);

		y_prev = y;
		y      = y_next;
		y_next = gsl_vector_get(v, j);
	}

	x_prev = x;
	x      = x_next;
	x_next = p->back().x;

	y_prev = y;
	y      = y_next;
	y_next = p->back().y;

	a = x_next - (2*x) + x_prev;
	b = y_next - (2*y) + y_prev;
	sum += (a*a + b*b);

	return (sum);
}


//The gradient of f, df = (df/dx, df/dy)
//derivative in each point [2x(i-2)-8x(i-1)+12x(i)-8x(i+1)+2x(i+2)]
void
my_df (const gsl_vector *v, void *params, gsl_vector *df)
{
	list<carmen_ackerman_traj_point_t> *p = (list<carmen_ackerman_traj_point_t> *) params;
	int i, j, size =(p->size() - 2);

	double x_prev2= 0;
	double x_prev = p->front().x;
	double x      = gsl_vector_get(v, 0);
	double x_next = gsl_vector_get(v, 1);
	double x_next2= gsl_vector_get(v, 2);
	double sum_x  =  (10*x) - (8*x_next) + (2*x_next2) - (4*x_prev);
	gsl_vector_set(df, 0, sum_x);

	double y_prev2= 0;
	double y_prev = p->front().y;
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);
	double y_next2= gsl_vector_get(v, size+2);
	double sum_y  = (10*y) - (8*y_next) + (2*y_next2) - (4*y_prev);
	gsl_vector_set(df, size, sum_y);

	for (i = 3, j = (size+3); i < size; i++, j++)
	{
		x_prev2= x_prev;
		x_prev = x;
		x      = x_next;
		x_next = x_next2;
		x_next2= gsl_vector_get(v, i);
		sum_x = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
		gsl_vector_set(df, (i-2), sum_x);

		y_prev2= y_prev;
		y_prev = y;
		y      = y_next;
		y_next = y_next2;
		y_next2= gsl_vector_get(v, j);
		sum_y = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
		gsl_vector_set(df, (j-2), sum_y);
	}

	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	x_next2= p->back().x;
	sum_x  = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
	gsl_vector_set(df, size-2, sum_x);

	y_prev2= y_prev;
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	y_next2= p->back().y;
	sum_y  = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
	gsl_vector_set(df, (2*size)-2, sum_y);

	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	sum_x  = (2*x_prev2) - (8*x_prev) + (10*x) - (4*x_next);
	gsl_vector_set(df, size-1, sum_x);

	y_prev2= y_prev;
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	sum_y  = (2*y_prev2) - (8*y_prev) + (10*y) - (4*y_next);
	gsl_vector_set(df, (2*size)-1, sum_y);
}


// Compute both f and df together
void
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


int
smooth_rddf_using_conjugate_gradient(carmen_ackerman_traj_point_t *poses_ahead, int num_poses_ahead,
		carmen_ackerman_traj_point_t *poses_back, int num_poses_back)
{
	int iter = 0;
	int status, i = 0, j = 0, size;

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;
	gsl_vector *v;
	gsl_multimin_function_fdf my_func;

	list<carmen_ackerman_traj_point_t>::iterator it;
	list<carmen_ackerman_traj_point_t> path;

	for (i = (num_poses_back - 1); i > 0; i--) // skip poses_back[0], because it is equal to poses_ahead[0]
		path.push_back(poses_back[i]);

	for (i = 0; i < num_poses_ahead; i++)
		path.push_back(poses_ahead[i]);

	if (path.size() < 5)
		return (1);

	size = path.size();

	my_func.n = (2 * size) - 4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &path;

	v = gsl_vector_alloc ((2 * size) - 4);

	static int count = 0;
	count++;
//	FILE *plot = fopen("gnuplot_smooth_lane.m", "w");

//	fprintf(plot, "a%d = [\n", count);
	it = path.begin();
//	fprintf(plot, "%f %f\n", it->x, it->y);

	it++; // skip the first pose
	for (i = 0, j = (size - 2); i < (size - 2); i++, j++, it++)
	{
//		fprintf(plot, "%f %f\n", it->x, it->y);

		gsl_vector_set (v, i, it->x);
		gsl_vector_set (v, j, it->y);
	}

//	fprintf(plot, "%f %f]\n\n", it->x, it->y);

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc (T, (2 * size) - 4);

	gsl_multimin_fdfminimizer_set (s, &my_func, v, 0.1, 0.01);  //(function_fdf, gsl_vector, step_size, tol)

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate (s);
		if (status) // error code
			return (0);

		status = gsl_multimin_test_gradient (s->gradient, 0.2);   //(gsl_vector, epsabs) and  |g| < epsabs
		// status == 0 (GSL_SUCCESS), if a minimum has been found
	} while (status == GSL_CONTINUE && iter < 999);

//	printf("status %d, iter %d\n", status, iter);
//	fflush(stdout);
	it = path.begin();

//	fprintf(plot, "b%d = [   \n%f %f\n", count, it->x, it->y);

	it++; // skip the first pose
	for (i = 0, j = (size - 2); i < (size - 2); i++, j++, it++)
	{
		it->x = gsl_vector_get (s->x, i);
		it->y = gsl_vector_get (s->x, j);

//		fprintf(plot, "%f %f\n", it->x, it->y);
	}

//	fprintf(plot, "%f %f]\n\n", it->x, it->y);
//	fprintf(plot, "\nplot (a%d(:,1), a%d(:,2), b%d(:,1), b%d(:,2)); \nstr = input (\"a   :\");\n\n", count, count, count, count);
//	fclose(plot);

	it = path.begin();
	it++;
	for (i = (num_poses_back - 2); i > 0; i--, it++) // skip first and last poses
		poses_back[i] = *it;
	poses_back[0] = *it;

	for (i = 0; i < num_poses_ahead - 1; i++, it++) // skip last pose
		poses_ahead[i] = *it;

	calculate_theta_and_phi(poses_ahead, num_poses_ahead, poses_back, num_poses_back);

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);

	return (1);
}


void
plot_state(carmen_ackerman_traj_point_t *path, int num_points, carmen_ackerman_traj_point_t *path2, int num_points2, bool display)
{
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;

	if (first_time)
	{
		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xlabel 'x'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'y'\n");
		fprintf(gnuplot_pipeMP, "set tics out\n");
		first_time = false;
	}

	if (display)
	{
		system("cp gnuplot_data_lane.txt gnuplot_data_lane_.txt");
		system("cp gnuplot_data_lane2.txt gnuplot_data_lane2_.txt");
	}

	FILE *gnuplot_data_lane  = fopen("gnuplot_data_lane.txt", "w");
	FILE *gnuplot_data_lane2 = fopen("gnuplot_data_lane2.txt", "w");

	for (int i = 0; i < num_points; i++)
	{
		fprintf(gnuplot_data_lane, "%lf %lf %lf %lf %lf %lf\n", path[i].x, path[i].y,
				0.8 * cos(path[i].theta), 0.8 * sin(path[i].theta), path[i].theta, path[i].phi);
	}
	for (int i = 0; i < num_points2; i++)
	{
		fprintf(gnuplot_data_lane2, "%lf %lf %lf %lf %lf %lf\n", path2[i].x, path2[i].y,
				0.8 * cos(path2[i].theta), 0.8 * sin(path2[i].theta), path2[i].theta, path2[i].phi);
	}
	fclose(gnuplot_data_lane);
	fclose(gnuplot_data_lane2);

	if (display)
	{
		fprintf(gnuplot_pipeMP, "plot "
				"'./gnuplot_data_lane_.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane Ahead normal'"
				", './gnuplot_data_lane2_.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane Back normal'"
				", './gnuplot_data_lane.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane Ahead smooth'"
				", './gnuplot_data_lane2.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane Back smooth' axes x1y1\n");
		fflush(gnuplot_pipeMP);
		dynamic_plot_state = abs(dynamic_plot_state) - 1;
//		fprintf(gnuplot_pipeMP, "plot "
////				"'./gnuplot_data_lane_.txt' using 1:2 w l title 'Lane Ahead normal'"
////				", './gnuplot_data_lane2_.txt' using 1:2 w l title 'Lane Back normal'"
//				"'./gnuplot_data_lane.txt' using 1:2 w l title 'Lane Ahead smooth'"
//				", './gnuplot_data_lane2.txt' using 1:2 w l title 'Lane Back smooth' axes x1y1\n");
//		fflush(gnuplot_pipeMP);
	}
}


double
get_lane_prob(carmen_point_t pose, carmen_map_p road_map)
{
	int x = round((pose.x - road_map->config.x_origin) / road_map->config.resolution);
	int y = round((pose.y - road_map->config.y_origin) / road_map->config.resolution);
	if (x < 0 || x >= road_map->config.x_size || y < 0 || y >= road_map->config.y_size)
		return (-1.0);

	double cell = road_map->map[x][y];
	road_prob *cell_prob = (road_prob *) &cell;
//	double prob = (double) (cell_prob->lane_center - cell_prob->off_road - cell_prob->broken_marking - cell_prob->solid_marking) / MAX_PROB;
	double prob = (double) (cell_prob->lane_center) / MAX_PROB;

	return ((prob > 0.0) ? prob : 0.0);
}


carmen_point_t
add_distance_to_pose(carmen_point_t pose, double distance)
{
	carmen_point_t next_pose = pose;
	next_pose.x += distance * cos(pose.theta);
	next_pose.y += distance * sin(pose.theta);

	return (next_pose);
}


carmen_point_t
add_orthogonal_distance_to_pose(carmen_point_t pose, double distance)
{
	carmen_point_t next_pose = pose;
	double orthogonal_theta;
	if (distance >= 0.0)
		orthogonal_theta = carmen_normalize_theta(pose.theta + (M_PI / 2.0));
	else
		orthogonal_theta = carmen_normalize_theta(pose.theta - (M_PI / 2.0));
	next_pose.x += fabs(distance) * cos(orthogonal_theta);
	next_pose.y += fabs(distance) * sin(orthogonal_theta);

	return (next_pose);
}


carmen_point_t
find_pose_by_annotation(carmen_point_t previous_pose, bool ahead)
{
	carmen_point_t rddf_pose = previous_pose;
	double delta_theta = 2.0 * asin(0.5 * rddf_min_distance_between_waypoints * traffic_sign_curvature);

	if (ahead)
	{
		rddf_pose.theta += delta_theta;
		rddf_pose = add_distance_to_pose(rddf_pose, rddf_min_distance_between_waypoints);
	}
	else
	{
		rddf_pose.theta -= delta_theta;
		rddf_pose = add_distance_to_pose(rddf_pose, -rddf_min_distance_between_waypoints);
	}

	return (rddf_pose);
}


carmen_point_t
find_nearest_pose_by_road_map(carmen_point_t rddf_pose_candidate, carmen_map_p road_map)
{
	carmen_point_t rddf_pose = rddf_pose_candidate; // Keep candidate's theta
	double max_lane_prob = get_lane_prob(rddf_pose_candidate, road_map);
	double min_delta_pose = 0.0;

	double step = road_map->config.resolution / 4.0;
	double lane_expected_width = 1.0;
	double left_limit = lane_expected_width / 2.0;
	double right_limit = -lane_expected_width / 2.0;

	for (double delta_pose = right_limit; delta_pose <= left_limit; delta_pose += step)
 	{
		carmen_point_t lane_pose = add_orthogonal_distance_to_pose(rddf_pose_candidate, delta_pose);
 		double lane_prob = get_lane_prob(lane_pose, road_map);
 		if (lane_prob > max_lane_prob ||
 			(lane_prob == max_lane_prob && fabs(delta_pose) < min_delta_pose))
 		{
 			max_lane_prob = lane_prob;
 			min_delta_pose = fabs(delta_pose);
 			rddf_pose.x = lane_pose.x;
 			rddf_pose.y = lane_pose.y;
 		}
 	}

 	return (rddf_pose);
}


double
average_theta(carmen_ackerman_traj_point_t *poses, int curr_index, int num_poses_avg)
{
	double sum_theta_x = 0.0, sum_theta_y = 0.0;
	int num_poses = ((curr_index + 1) >= num_poses_avg) ? num_poses_avg : (curr_index + 1);

	for (int i = 0, index = curr_index; i < num_poses; i++, index--)
	{
		sum_theta_x += cos(poses[index].theta);
		sum_theta_y += sin(poses[index].theta);
	}

	if (sum_theta_x == 0.0 && sum_theta_y == 0.0)
		return (0.0);

	double avg_theta = atan2(sum_theta_y, sum_theta_x);

	return (avg_theta);
}


void
save_traffic_sign_state()
{
	state_traffic_sign_is_on = traffic_sign_is_on;
	state_traffic_sign_code = traffic_sign_code;
	state_traffic_sign_curvature = traffic_sign_curvature;
}


void
restore_traffic_sign_state()
{
	traffic_sign_is_on = state_traffic_sign_is_on;
	traffic_sign_code = state_traffic_sign_code;
	traffic_sign_curvature = state_traffic_sign_curvature;
}


void
carmen_rddf_play_check_reset_traffic_sign_state(carmen_point_t new_pose)
{
	if (DIST2D(new_pose, state_traffic_sign_pose) > 5.0)
	{
		state_traffic_sign_is_on = traffic_sign_is_on = false;
		state_traffic_sign_code = traffic_sign_code = RDDF_ANNOTATION_CODE_NONE;
		state_traffic_sign_curvature = traffic_sign_curvature = 0.0;
	}
	state_traffic_sign_pose = new_pose;
}


int
pose_out_of_map_coordinates(carmen_point_t pose, carmen_map_p map)
{
	double x_min = map->config.x_origin;
	double x_max = map->config.x_origin + map->config.x_size * map->config.resolution;
	double y_min = map->config.y_origin;
	double y_max = map->config.y_origin + map->config.y_size * map->config.resolution;
	int out_of_map = (pose.x < x_min || pose.x >= x_max || pose.y < y_min || pose.y >= y_max);

	return (out_of_map);
}


int
fill_in_poses_ahead_by_road_map(carmen_point_t initial_pose, carmen_map_p road_map,
		carmen_ackerman_traj_point_t *poses_ahead, int num_poses_desired)
{
	carmen_point_t rddf_pose, previous_pose, rddf_pose_candidate;
	int num_poses = 0;
	double velocity = 10.0, phi = 0.0;
	int num_poses_avg = 5;

//	rddf_log = fopen("/home/rcarneiro/carmen_lcad/src/rddf/log.txt", "w");
//	g_num_pose = 0;

	find_direction_traffic_sign(initial_pose, true);
	save_traffic_sign_state();
	if (traffic_sign_is_on)
	{
		previous_pose = rddf_pose = initial_pose;
		rddf_pose_candidate = find_pose_by_annotation(previous_pose, true);
	}
	else
	{
		previous_pose = rddf_pose = find_nearest_pose_by_road_map(initial_pose, road_map);
		rddf_pose_candidate = add_distance_to_pose(previous_pose, rddf_min_distance_between_waypoints);
	}

	poses_ahead[0] = create_ackerman_traj_point_struct(rddf_pose.x, rddf_pose.y, velocity, phi, rddf_pose.theta);
	num_poses = 1;
	do
	{
//		g_num_pose = num_poses;
		find_direction_traffic_sign(rddf_pose_candidate, true);
		if (traffic_sign_is_on)
		{
			previous_pose = rddf_pose = rddf_pose_candidate;
			rddf_pose_candidate = find_pose_by_annotation(previous_pose, true);
		}
		else
		{
			rddf_pose = find_nearest_pose_by_road_map(rddf_pose_candidate, road_map);
			rddf_pose.theta = atan2(rddf_pose.y - previous_pose.y, rddf_pose.x - previous_pose.x);
			previous_pose = rddf_pose;
			previous_pose.theta = average_theta(poses_ahead, num_poses, num_poses_avg);
			rddf_pose_candidate = add_distance_to_pose(previous_pose, rddf_min_distance_between_waypoints);
		}
		if (pose_out_of_map_coordinates(rddf_pose, road_map))
			break;

		poses_ahead[num_poses] = create_ackerman_traj_point_struct(rddf_pose.x, rddf_pose.y, velocity, phi, rddf_pose.theta);
		num_poses++;
	} while (num_poses < num_poses_desired);

//	for (int i = 0; i < num_poses; i++)
//		fprintf(rddf_log, "%d\t%lf\t%lf\t%lf\n", i, poses_ahead[i].x, poses_ahead[i].y, poses_ahead[i].theta);

	calculate_theta_ahead(poses_ahead, num_poses);
	calculate_phi_ahead(poses_ahead, num_poses);
	restore_traffic_sign_state();

	return (num_poses);
}


int
fill_in_poses_back_by_road_map(carmen_point_t initial_pose, carmen_map_p road_map,
		carmen_ackerman_traj_point_t *poses_back, int num_poses_desired)
{
	carmen_point_t rddf_pose, previous_pose, rddf_pose_candidate;
	int num_poses = 0;
	double velocity = 10.0, phi = 0.0;
	int num_poses_avg = 5;

	find_direction_traffic_sign(initial_pose, false);
	if (traffic_sign_is_on)
	{
		previous_pose = rddf_pose = initial_pose;
		rddf_pose_candidate = find_pose_by_annotation(previous_pose, false);
	}
	else
	{
		previous_pose = rddf_pose = initial_pose;
		rddf_pose_candidate = add_distance_to_pose(previous_pose, -rddf_min_distance_between_waypoints);
	}

	poses_back[0] = create_ackerman_traj_point_struct(rddf_pose.x, rddf_pose.y, velocity, phi, rddf_pose.theta);
	num_poses = 1;
	do
	{
//		g_num_pose = num_poses;
		find_direction_traffic_sign(rddf_pose_candidate, false);
		if (traffic_sign_is_on)
		{
			previous_pose = rddf_pose = rddf_pose_candidate;
			rddf_pose_candidate = find_pose_by_annotation(previous_pose, false);
		}
		else
		{
			rddf_pose = find_nearest_pose_by_road_map(rddf_pose_candidate, road_map);
			rddf_pose.theta = atan2(previous_pose.y - rddf_pose.y, previous_pose.x - rddf_pose.x);
			previous_pose = rddf_pose;
			previous_pose.theta = average_theta(poses_back, num_poses, num_poses_avg);
			rddf_pose_candidate = add_distance_to_pose(previous_pose, -rddf_min_distance_between_waypoints);
		}
//		if (pose_out_of_map_coordinates(rddf_pose, road_map))
			break;

		poses_back[num_poses] = create_ackerman_traj_point_struct(rddf_pose.x, rddf_pose.y, velocity, phi, rddf_pose.theta);
		num_poses++;
	} while (num_poses < num_poses_desired);

//	for (int i = 0; i < num_poses; i++)
//		fprintf(rddf_log, "%d\t%lf\t%lf\t%lf\n", i, poses_back[i].x, poses_back[i].y, poses_back[i].theta);
//	fclose(rddf_log);

	calculate_theta_back(poses_back, num_poses);
	calculate_phi_back(poses_back, num_poses);
	restore_traffic_sign_state();

	return (num_poses);
}


int
carmen_rddf_play_find_nearest_poses_by_road_map(carmen_point_t initial_pose, carmen_map_p road_map,
		carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int *num_poses_back, int num_poses_ahead_max)
{
	static int num_poses_ahead;

	num_poses_ahead = fill_in_poses_ahead_by_road_map(initial_pose, road_map, poses_ahead, num_poses_ahead_max);

	initial_pose.x = poses_ahead[0].x;
	initial_pose.y = poses_ahead[0].y;
	initial_pose.theta = poses_ahead[0].theta;
	(*num_poses_back) = fill_in_poses_back_by_road_map(initial_pose, road_map, poses_back, num_poses_ahead_max / 3);
	poses_back[0].phi = poses_ahead[0].phi;

	if (dynamic_plot_state)
		plot_state(poses_ahead, num_poses_ahead, poses_back, *num_poses_back, false);

	smooth_rddf_using_conjugate_gradient(poses_ahead, num_poses_ahead, poses_back, *num_poses_back);

	if (dynamic_plot_state)
		plot_state(poses_ahead, num_poses_ahead, poses_back, *num_poses_back, true);

	return (num_poses_ahead);
}
