/*
 * rddf_by_road_map_image.cpp
 *
 *  Created on: 8 de mar. de 2026
 *      Author: Raphael Carneiro
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <dirent.h>
#include <errno.h>
#include <opencv2/opencv.hpp>

typedef struct
{
	double x;
	double y;
	double theta;
} carmen_point_t, *carmen_point_p;

#define M_PI	3.14159265358979323846

double
carmen_normalize_theta(double theta)
{
  double multiplier;

  if (theta >= -M_PI && theta < M_PI)
    return theta;

  multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
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

int
get_class(const cv::Vec3b& pixel)
{
    if(pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0)
    {
        /* BGR: [0, 0, 0] - classe 0 - cor preta - Nao e pista */
        return(0);
    }
    else if(pixel[0] == 255 && pixel[1] == 0 && pixel[2] == 82)
    {
        /* BGR: [255, 0, 82] - classe 1 - cor roxa - Faixa solida */
        return(1);
    }
    else if(pixel[0] == 255 && pixel[1] == 255 && pixel[2] == 255)
    {
        /* BGR: [255, 255, 255] - classe 2 - cor branca - Faixa tracejada */
        return(2);
    }
    else if(pixel[0] == 0 && pixel[1] == 255 && pixel[2] == 0)
    {
        /* BGR: [0, 255, 0] - classe 3 - cor verde - Centro da pista */
        return(3);
    }
    else if(pixel[0] == 0 && pixel[1] == 200 && pixel[2] == 255)
    {
        /* BGR: [0, 200, 255] - classe 4 - cor amarela - Entre o centro e a borda da pista */
        return(4);
    }
    else if(pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 139)
    {
        /* BGR: [0, 0, 139] - classe 5 - cor vermelha - Borda da pista */
        return(5);
    }

    return(-1);
}

double
get_lane_prob(const cv::Mat& road_map, double x, double y, double x_center, double y_center, double resolution, int x_size, int y_size)
{
    double map_min_x;
    double map_max_x;
    double map_min_y;
    double map_max_y;
    double local_x;
    double local_y;
    int ix;
    int iy;
    int lane_class;
    cv::Vec3b pixel;

    if(road_map.empty() || road_map.cols != x_size || road_map.rows != y_size || road_map.type() != CV_8UC3)
    {
        return(-1.0);
    }

    map_min_x = x_center - ((double)x_size * resolution) / 2.0;
    map_max_x = x_center + ((double)x_size * resolution) / 2.0;
    map_min_y = y_center - ((double)y_size * resolution) / 2.0;
    map_max_y = y_center + ((double)y_size * resolution) / 2.0;

    if(x < map_min_x || x >= map_max_x || y < map_min_y || y >= map_max_y)
    {
        return(-1.0);
    }

    local_x = x - map_min_x;
    local_y = map_max_y - y;

    ix = (int)(local_x / resolution);
    iy = (int)(local_y / resolution);

    if(ix < 0 || ix >= x_size || iy < 0 || iy >= y_size)
    {
        return(-1.0);
    }

    pixel = road_map.at<cv::Vec3b>(iy, ix);
    lane_class = get_class(pixel);

    switch(lane_class)
    {
        case 0:
            return(0.0);

        case 1:
            return(0.2);

        case 2:
            return(0.4);

        case 3:
            return(1.0);

        case 4:
            return(0.8);

        case 5:
            return(0.6);

        default:
            return(0.0);
    }
}

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
	list<carmen_robot_and_trailers_traj_point_t> *p = (list<carmen_robot_and_trailers_traj_point_t> *) params;
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
	list<carmen_point_t> *p = (list<carmen_point_t> *) params;
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
smooth_rddf_using_conjugate_gradient(carmen_point_t *poses_ahead, int num_poses_ahead)
{
	int iter = 0;
	int status, i = 0, j = 0, size;

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;
	gsl_vector *v;
	gsl_multimin_function_fdf my_func;

	list<carmen_point_t>::iterator it;
	list<carmen_point_t> path;

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

	if (poses_back)
		poses_back[0] = *it;

	for (i = 0; i < num_poses_ahead - 1; i++, it++) // skip last pose
		poses_ahead[i] = *it;

	calculate_theta_and_phi(poses_ahead, num_poses_ahead, poses_back, num_poses_back);

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);

	return (1);
}

int
is_point_inside_map(double x, double y, double x_center, double y_center, double resolution, int x_size, int y_size)
{
    double map_min_x;
    double map_max_x;
    double map_min_y;
    double map_max_y;

    map_min_x = x_center - ((double)x_size * resolution) / 2.0;
    map_max_x = x_center + ((double)x_size * resolution) / 2.0;
    map_min_y = y_center - ((double)y_size * resolution) / 2.0;
    map_max_y = y_center + ((double)y_size * resolution) / 2.0;

    if(x < map_min_x || x >= map_max_x || y < map_min_y || y >= map_max_y)
    {
        return(0);
    }

    return(1);
}

double
average_theta(carmen_point_t *poses, int curr_index, int num_poses_avg)
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
compute_theta(carmen_point_t *path, int num_poses)
{
	for (int i = 0; i < (num_poses - 1); i++)
		path[i].theta = atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
	if (num_poses > 1)
		path[num_poses - 1].theta = path[num_poses - 2].theta;
}

carmen_point_t
find_nearest_pose_by_road_map(carmen_point_t rddf_pose_candidate, const cv::Mat& road_map, double x_center, double y_center, double resolution)
{
	carmen_point_t rddf_pose = rddf_pose_candidate; // Keep candidate's theta
	double max_lane_prob = get_lane_prob(road_map, rddf_pose_candidate.x, rddf_pose_candidate.y, x_center, y_center, resolution, road_map.cols, road_map.rows);

	double step = resolution / 4.0;
	double lane_expected_width = 3.0;
	double left_limit = lane_expected_width / 2.0;
	double right_limit = -lane_expected_width / 2.0;
	double min_delta_pose = fmax(fabs(left_limit), fabs(right_limit));

	for (double delta_pose = right_limit; delta_pose <= left_limit; delta_pose += step)
 	{
		carmen_point_t lane_pose = add_orthogonal_distance_to_pose(rddf_pose_candidate, delta_pose);
 		double lane_prob = get_lane_prob(road_map, lane_pose.x, lane_pose.y, x_center, y_center, resolution, road_map.cols, road_map.rows);
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

int
fill_in_poses_ahead_by_road_map(carmen_point_t initial_pose, const cv::Mat& road_map, double x_center, double y_center, double resolution,
		carmen_point_t *poses_ahead, int num_poses_desired)
{
	carmen_point_t rddf_pose, previous_pose, rddf_pose_candidate;
	int num_poses = 0;
	int num_poses_avg = 5;
	double rddf_min_distance_between_waypoints = 0.5;
	previous_pose = rddf_pose = find_nearest_pose_by_road_map(initial_pose, road_map, x_center, y_center, resolution);
	rddf_pose_candidate = add_distance_to_pose(previous_pose, rddf_min_distance_between_waypoints);

	if(is_point_inside_map(rddf_pose.x, rddf_pose.y, x_center, y_center, resolution, road_map.cols, road_map.rows))
	{
		poses_ahead[0] = rddf_pose;
		num_poses = 1;
		do
		{
			rddf_pose = find_nearest_pose_by_road_map(rddf_pose_candidate, road_map, x_center, y_center, resolution);
			rddf_pose.theta = atan2(rddf_pose.y - previous_pose.y, rddf_pose.x - previous_pose.x);
			previous_pose = rddf_pose;
			previous_pose.theta = average_theta(poses_ahead, num_poses, num_poses_avg);
			rddf_pose_candidate = add_distance_to_pose(previous_pose, rddf_min_distance_between_waypoints);
			if(!is_point_inside_map(rddf_pose.x, rddf_pose.y, x_center, y_center, resolution, road_map.cols, road_map.rows))
				break;

			poses_ahead[num_poses] = rddf_pose;
			num_poses++;
		} while (num_poses < num_poses_desired);

		compute_theta(poses_ahead, num_poses);
		smooth_rddf_using_conjugate_gradient(poses_ahead, num_poses);
	}

	return (num_poses);
}


const char*
get_file_basename(const char* filepath1)
{
    const char* p1;
    const char* p2;

    p1 = strrchr(filepath1, '/');
    p2 = strrchr(filepath1, '\\');

    if(p1 == NULL && p2 == NULL)
    {
        return(filepath1);
    }
    if(p1 == NULL)
    {
        return(p2 + 1);
    }
    if(p2 == NULL)
    {
        return(p1 + 1);
    }

    return((p1 > p2 ? p1 : p2) + 1);
}

int
parse_centers_from_filename(const char* filename, double* x_center, double* y_center)
{
    const char* base_name;
    char tmp[1024];
    char* token;
    char* saveptr;
    char* endptr;
    double values[2] = {0.0, 0.0};
    int count;

    base_name = get_file_basename(filename);

    if(strlen(base_name) >= sizeof(tmp))
    {
        return(0);
    }

    strcpy(tmp, base_name);

    count = 0;
    token = strtok_r(tmp, "_", &saveptr);

    while(token != NULL)
    {
        if(count == 0)
        {
            if(token[0] != 'i')
            {
                return(0);
            }

            values[0] = strtod(token + 1, &endptr);
            if(endptr == token + 1 || *endptr != '\0')
            {
                return(0);
            }
        }
        else if(count == 1)
        {
            values[1] = strtod(token, &endptr);
            if(endptr == token || *endptr != '\0')
            {
                return(0);
            }
        }

        count++;
        token = strtok_r(NULL, "_", &saveptr);
    }

    if(count < 5)
    {
        return(0);
    }

    *x_center = values[0];
    *y_center = values[1];

    return(1);
}

void
count_cell_probs(const cv::Mat& road_map, double x_center, double y_center, double resolution, int x_size, int y_size)
{
    int row;
    int col;
    double x;
    double y;
    double prob;
    int count_neg1;
    int count_00;
    int count_02;
    int count_04;
    int count_06;
    int count_08;
    int count_10;

    count_neg1 = 0;
    count_00 = 0;
    count_02 = 0;
    count_04 = 0;
    count_06 = 0;
    count_08 = 0;
    count_10 = 0;

    for(row = 0; row < y_size; row++)
    {
        for(col = 0; col < x_size; col++)
        {
            x = x_center + (((double)col + 0.5) - ((double)x_size / 2.0)) * resolution;
            y = y_center + (((double)y_size / 2.0) - ((double)row + 0.5)) * resolution;

            prob = get_lane_prob(road_map, x, y, x_center, y_center, resolution, x_size, y_size);

            if(prob == -1.0)
            {
                count_neg1++;
            }
            else if(prob == 0.0)
            {
                count_00++;
            }
            else if(prob == 0.2)
            {
                count_02++;
            }
            else if(prob == 0.4)
            {
                count_04++;
            }
            else if(prob == 0.6)
            {
                count_06++;
            }
            else if(prob == 0.8)
            {
                count_08++;
            }
            else if(prob == 1.0)
            {
                count_10++;
            }
            else
            {
                printf("Aviso: probabilidade inesperada em row=%d col=%d: %.17g\n", row, col, prob);
            }
        }
    }

    printf("Contagem por probabilidade:\n");
    printf("-1.0 : %d\n", count_neg1);
    printf(" 0.0 : %d\n", count_00);
    printf(" 0.2 : %d\n", count_02);
    printf(" 0.4 : %d\n", count_04);
    printf(" 0.6 : %d\n", count_06);
    printf(" 0.8 : %d\n", count_08);
    printf(" 1.0 : %d\n", count_10);
}


int
world_to_map_indices(double x, double y, double x_center, double y_center, double resolution, int x_size, int y_size, int* ix, int* iy)
{
    double map_min_x;
    double map_max_x;
    double map_min_y;
    double map_max_y;
    double local_x;
    double local_y;

    map_min_x = x_center - ((double)x_size * resolution) / 2.0;
    map_max_x = x_center + ((double)x_size * resolution) / 2.0;
    map_min_y = y_center - ((double)y_size * resolution) / 2.0;
    map_max_y = y_center + ((double)y_size * resolution) / 2.0;

    if(x < map_min_x || x >= map_max_x || y < map_min_y || y >= map_max_y)
    {
        return(0);
    }

    local_x = x - map_min_x;
    local_y = map_max_y - y;

    *ix = (int)(local_x / resolution);
    *iy = (int)(local_y / resolution);

    if(*ix < 0 || *ix >= x_size || *iy < 0 || *iy >= y_size)
    {
        return(0);
    }

    return(1);
}

int
rectangles_intersect(double min_x1, double max_x1, double min_y1, double max_y1, double min_x2, double max_x2, double min_y2, double max_y2)
{
    if(max_x1 <= min_x2 || max_x2 <= min_x1)
    {
        return(0);
    }

    if(max_y1 <= min_y2 || max_y2 <= min_y1)
    {
        return(0);
    }

    return(1);
}

int
has_png_extension(const char* filename)
{
    const char* dot;

    dot = strrchr(filename, '.');

    if(dot == NULL)
    {
        return(0);
    }

    if(strcmp(dot, ".png") == 0)
    {
        return(1);
    }

    if(strcmp(dot, ".PNG") == 0)
    {
        return(1);
    }

    return(0);
}

void
compute_map_bounds(double x_center, double y_center, double resolution, int x_size, int y_size, double* map_min_x, double* map_max_x, double* map_min_y, double* map_max_y)
{
    *map_min_x = x_center - ((double)x_size * resolution) / 2.0;
    *map_max_x = x_center + ((double)x_size * resolution) / 2.0;
    *map_min_y = y_center - ((double)y_size * resolution) / 2.0;
    *map_max_y = y_center + ((double)y_size * resolution) / 2.0;
}

void
copy_road_map_into_big_map(cv::Mat& big_road_map, const cv::Mat& road_map, double src_x_center, double src_y_center, double dst_x_center, double dst_y_center, double resolution)
{
    int src_x_size;
    int src_y_size;
    int dst_x_size;
    int dst_y_size;
    int src_row;
    int src_col;
    double x;
    double y;
    int dst_ix;
    int dst_iy;
    cv::Vec3b pixel;

    src_x_size = road_map.cols;
    src_y_size = road_map.rows;
    dst_x_size = big_road_map.cols;
    dst_y_size = big_road_map.rows;

    for(src_row = 0; src_row < src_y_size; src_row++)
    {
        for(src_col = 0; src_col < src_x_size; src_col++)
        {
            x = src_x_center + (((double)src_col + 0.5) - ((double)src_x_size / 2.0)) * resolution;
            y = src_y_center + (((double)src_y_size / 2.0) - ((double)src_row + 0.5)) * resolution;

            if(!world_to_map_indices(x, y, dst_x_center, dst_y_center, resolution, dst_x_size, dst_y_size, &dst_ix, &dst_iy))
            {
                continue;
            }

            pixel = road_map.at<cv::Vec3b>(src_row, src_col);

            if(get_class(pixel) == 0)
            {
                continue;
            }

            big_road_map.at<cv::Vec3b>(dst_iy, dst_ix) = pixel;
        }
    }
}

int
save_big_road_map(const cv::Mat& big_road_map, double x_center, double y_center)
{
    char output_filename[2048];

    if(snprintf(output_filename, sizeof(output_filename), "./big_road_map_%.17g_%.17g.png", x_center, y_center) >= (int)sizeof(output_filename))
    {
        return(0);
    }

    if(!cv::imwrite(output_filename, big_road_map))
    {
        return(0);
    }

    printf("Imagem salva em: %s\n", output_filename);

    return(1);
}

void
plot_matched_point(cv::Mat& plot_map, double x, double y, double x_center, double y_center, double resolution, int x_size, int y_size)
{
    int ix;
    int iy;

    if(!world_to_map_indices(x, y, x_center, y_center, resolution, x_size, y_size, &ix, &iy))
    {
        return;
    }

    plot_map.at<cv::Vec3b>(iy, ix)[0] = 64;
    plot_map.at<cv::Vec3b>(iy, ix)[1] = 64;
    plot_map.at<cv::Vec3b>(iy, ix)[2] = 64;
}

void
show_plot_map(const cv::Mat& plot_map)
{
    cv::namedWindow("matched_points", cv::WINDOW_AUTOSIZE);
    cv::imshow("matched_points", plot_map);
    cv::waitKey(0);
}

int
save_plot_map(const cv::Mat& plot_map, const char* png_filename)
{
    const char* base_name;
    char output_filename[2048];
    char name_without_ext[1024];
    const char* dot;
    size_t base_len;
    size_t stem_len;

    base_name = get_file_basename(png_filename);
    base_len = strlen(base_name);

    if(base_len >= sizeof(name_without_ext))
    {
        return(0);
    }

    strcpy(name_without_ext, base_name);

    dot = strrchr(name_without_ext, '.');
    if(dot != NULL)
    {
        stem_len = (size_t)(dot - name_without_ext);
        name_without_ext[stem_len] = '\0';
    }

    if(snprintf(output_filename, sizeof(output_filename), "./%s_with_rddf.png", name_without_ext) >= (int)sizeof(output_filename))
    {
        return(0);
    }

    if(!cv::imwrite(output_filename, plot_map))
    {
        return(0);
    }

    printf("Imagem com RDDF salva em: %s\n", output_filename);

    return(1);
}

int
main(int argc, char** argv)
{
    const char* dir_img;
    DIR* dirp;
    struct dirent* entry;
    char filepath[4096];
    cv::Mat road_map;
    cv::Mat big_road_map;
    double big_x_center;
    double big_y_center;
    double resolution;
    int big_x_size;
    int big_y_size;
    double big_map_min_x;
    double big_map_max_x;
    double big_map_min_y;
    double big_map_max_y;
    double img_x_center;
    double img_y_center;
    double img_map_min_x;
    double img_map_max_x;
    double img_map_min_y;
    double img_map_max_y;
    int used_images;
    int skipped_images;
    int total_png_files;

    if(argc != 4)
    {
        printf("Uso: %s <dir_img> <x> <y>\n", argv[0]);
        return(1);
    }

    dir_img = argv[1];
    big_x_center = strtod(argv[2], NULL);
    big_y_center = strtod(argv[3], NULL);
    resolution = 0.2;

    big_x_size = (int)(160.0 / resolution);
    big_y_size = (int)(160.0 / resolution);

    big_road_map = cv::Mat(big_y_size, big_x_size, CV_8UC3, cv::Scalar(0, 0, 0));

    compute_map_bounds(big_x_center, big_y_center, resolution, big_x_size, big_y_size, &big_map_min_x, &big_map_max_x, &big_map_min_y, &big_map_max_y);

    dirp = opendir(dir_img);

    if(dirp == NULL)
    {
        printf("Erro: nao foi possivel abrir o diretorio: %s\n", dir_img);
        return(1);
    }

    used_images = 0;
    skipped_images = 0;
    total_png_files = 0;

    while(1)
    {
        entry = readdir(dirp);

        if(entry == NULL)
        {
            break;
        }

        if(!has_png_extension(entry->d_name))
        {
            continue;
        }

        total_png_files++;

        if(snprintf(filepath, sizeof(filepath), "%s/%s", dir_img, entry->d_name) >= (int)sizeof(filepath))
        {
            printf("Aviso: caminho muito longo, ignorando arquivo: %s\n", entry->d_name);
            skipped_images++;
            continue;
        }

        if(!parse_centers_from_filename(entry->d_name, &img_x_center, &img_y_center))
        {
            printf("Aviso: nome fora do padrao esperado, ignorando arquivo: %s\n", entry->d_name);
            skipped_images++;
            continue;
        }

        road_map = cv::imread(filepath, cv::IMREAD_COLOR);

        if(road_map.empty())
        {
            printf("Aviso: nao foi possivel ler a imagem: %s\n", filepath);
            skipped_images++;
            continue;
        }

        compute_map_bounds(img_x_center, img_y_center, resolution, road_map.cols, road_map.rows, &img_map_min_x, &img_map_max_x, &img_map_min_y, &img_map_max_y);

        if(!rectangles_intersect(big_map_min_x, big_map_max_x, big_map_min_y, big_map_max_y, img_map_min_x, img_map_max_x, img_map_min_y, img_map_max_y))
        {
            continue;
        }

        copy_road_map_into_big_map(big_road_map, road_map, img_x_center, img_y_center, big_x_center, big_y_center, resolution);
        used_images++;
    }

    closedir(dirp);

    printf("Diretorio de imagens: %s\n", dir_img);
    printf("Centro do big_road_map: x=%.6f y=%.6f\n", big_x_center, big_y_center);
    printf("Dimensoes do big_road_map: %d x %d celulas\n", big_x_size, big_y_size);
    printf("Resolucao: %.6f m/celula\n", resolution);
    printf("Arquivos PNG encontrados: %d\n", total_png_files);
    printf("Imagens usadas no mosaico: %d\n", used_images);
    printf("Imagens ignoradas por erro: %d\n", skipped_images);

    if(!save_big_road_map(big_road_map, big_x_center, big_y_center))
    {
        printf("Erro: nao foi possivel salvar a imagem final\n");
        return(1);
    }

    return(0);
}
