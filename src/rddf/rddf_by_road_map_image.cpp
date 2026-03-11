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
#include <list>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>

typedef struct
{
	double x;          // path waypoint coordinate in meters
	double y;          // path waypoint coordinate in meters
	double theta;      // path waypoint heading orientation in radians, x-axis: 0, y-axis: pi/2
} waypoint_t, *waypoint_p;

typedef struct
{
	cv::Mat img;       // map image in pixels x pixels
	double x_center;   // map center coordinate in meters
	double y_center;   // map center coordinate in meters
	double resolution; // map cell unit size in meters x meters
} map_t, *map_p;

#define M_PI	3.14159265358979323846


double
carmen_normalize_theta(double theta)
{
    double multiplier;

    if (theta >= -M_PI && theta < M_PI)
    {
    	return(theta);
    }

    multiplier = floor(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;

    if (theta >= M_PI)
    {
    	theta -= 2*M_PI;
    }

    if (theta < -M_PI)
    {
    	theta += 2*M_PI;
    }

    return(theta);
}


double
average_theta(waypoint_t *poses, int curr_index, int num_poses_avg)
{
	double sum_theta_x = 0.0;
	double sum_theta_y = 0.0;
	int num_poses = fmin((curr_index + 1), num_poses_avg);

	for (int i = 0, index = curr_index; i < num_poses; i++, index--)
	{
		sum_theta_x += cos(poses[index].theta);
		sum_theta_y += sin(poses[index].theta);
	}

	if (sum_theta_x == 0.0 && sum_theta_y == 0.0)
	{
		return (0.0);
	}

	double avg_theta = atan2(sum_theta_y, sum_theta_x);

	return(avg_theta);
}


void
compute_theta(waypoint_t *path, int num_poses)
{
	for (int i = 0; i < (num_poses - 1); i++)
	{
		path[i].theta = atan2((path[i + 1].y - path[i].y), (path[i + 1].x - path[i].x));
	}

	if (num_poses > 1)
	{
		path[num_poses - 1].theta = path[num_poses - 2].theta;
	}
}


waypoint_t
add_distance_to_pose(waypoint_t pose, double distance)
{
	waypoint_t next_pose = pose;

	next_pose.x += distance * cos(pose.theta);
	next_pose.y += distance * sin(pose.theta);

	return(next_pose);
}


waypoint_t
add_orthogonal_distance_to_pose(waypoint_t pose, double distance)
{
	waypoint_t next_pose = pose;
	double orthogonal_theta;

	if (distance >= 0.0)
	{
		orthogonal_theta = carmen_normalize_theta(pose.theta + (M_PI / 2.0));
	}
	else
	{
		orthogonal_theta = carmen_normalize_theta(pose.theta - (M_PI / 2.0));
	}

	next_pose.x += fabs(distance) * cos(orthogonal_theta);
	next_pose.y += fabs(distance) * sin(orthogonal_theta);

	return(next_pose);
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
get_lane_prob(const waypoint_t coord, const map_t road_map)
{
    if(road_map.img.empty() || road_map.img.type() != CV_8UC3)
    {
        return(-1.0);
    }

    double map_min_x = road_map.x_center - ((double)road_map.img.cols * road_map.resolution) / 2.0;
    double map_max_x = road_map.x_center + ((double)road_map.img.cols * road_map.resolution) / 2.0;
    double map_min_y = road_map.y_center - ((double)road_map.img.rows * road_map.resolution) / 2.0;
    double map_max_y = road_map.y_center + ((double)road_map.img.rows * road_map.resolution) / 2.0;

    if(coord.x < map_min_x || coord.x >= map_max_x || coord.y < map_min_y || coord.y >= map_max_y)
    {
        return(-1.0);
    }

    double local_x = coord.x - map_min_x;
    double local_y = map_max_y - coord.y;

    int ix = (int)(local_x / road_map.resolution);
    int iy = (int)(local_y / road_map.resolution);

    if(ix < 0 || ix >= road_map.img.cols || iy < 0 || iy >= road_map.img.rows)
    {
        return(-1.0);
    }

    cv::Vec3b img_pixel = road_map.img.at<cv::Vec3b>(iy, ix);
    int lane_class = get_class(img_pixel);

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
	std::list<waypoint_t> *p = (std::list<waypoint_t> *) params;
	int i, j, size = (p->size() - 2);           //we have to discount the first and last point that won't be optimized
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
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	std::list<waypoint_t> *p = (std::list<waypoint_t> *) params;
	int i, j, size = (p->size() - 2);

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
smooth_rddf_using_conjugate_gradient(waypoint_t *poses_ahead, int num_poses_ahead)
{
	int iter = 0;
	int status, i = 0, j = 0, size;

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;
	gsl_vector *v;
	gsl_multimin_function_fdf my_func;

	std::list<waypoint_t>::iterator it;
	std::list<waypoint_t> path;

	for (i = 0; i < num_poses_ahead; i++)
	{
		path.push_back(poses_ahead[i]);
	}

	if (path.size() < 5)
	{
		return (1);
	}

	size = path.size();

	my_func.n = (2 * size) - 4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &path;

	v = gsl_vector_alloc ((2 * size) - 4);

	static int count = 0;
	count++;
	it = path.begin();
	it++; // skip the first pose

	for (i = 0, j = (size - 2); i < (size - 2); i++, j++, it++)
	{
		gsl_vector_set (v, i, it->x);
		gsl_vector_set (v, j, it->y);
	}

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc (T, (2 * size) - 4);

	gsl_multimin_fdfminimizer_set(s, &my_func, v, 0.1, 0.01);  //(function_fdf, gsl_vector, step_size, tol)

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate (s);
		if (status) // error code
		{
			return(0);
		}

		status = gsl_multimin_test_gradient (s->gradient, 0.2);   //(gsl_vector, epsabs) and  |g| < epsabs
		// status == 0 (GSL_SUCCESS), if a minimum has been found
	} while (status == GSL_CONTINUE && iter < 999);

	it = path.begin();
	it++; // skip the first pose
	for (i = 0, j = (size - 2); i < (size - 2); i++, j++, it++)
	{
		it->x = gsl_vector_get (s->x, i);
		it->y = gsl_vector_get (s->x, j);
	}

	it = path.begin();
	it++;
	for (i = 0; i < num_poses_ahead - 1; i++, it++) // skip last pose
	{
		poses_ahead[i] = *it;
	}

	compute_theta(poses_ahead, num_poses_ahead);

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);

	return(1);
}


int
is_point_inside_map(const waypoint_t coord, const map_t map)
{
    double map_min_x = map.x_center - ((double)map.img.cols * map.resolution) / 2.0;
    double map_max_x = map.x_center + ((double)map.img.cols * map.resolution) / 2.0;
    double map_min_y = map.y_center - ((double)map.img.rows * map.resolution) / 2.0;
    double map_max_y = map.y_center + ((double)map.img.rows * map.resolution) / 2.0;

    if(coord.x < map_min_x || coord.x >= map_max_x || coord.y < map_min_y || coord.y >= map_max_y)
    {
        return(0);
    }

    return(1);
}


waypoint_t
find_nearest_pose_by_road_map(const waypoint_t rddf_pose_candidate, const map_t road_map)
{
	waypoint_t rddf_pose = rddf_pose_candidate; // Keep candidate's theta
	double max_lane_prob = get_lane_prob(rddf_pose_candidate, road_map);
	double step = road_map.resolution / 4.0;
	double lane_expected_width = 3.0;
	double left_limit = lane_expected_width / 2.0;
	double right_limit = -lane_expected_width / 2.0;
	double min_delta_pose = fmax(fabs(left_limit), fabs(right_limit));

	for (double delta_pose = right_limit; delta_pose <= left_limit; delta_pose += step)
 	{
		waypoint_t lane_pose = add_orthogonal_distance_to_pose(rddf_pose_candidate, delta_pose);
 		double lane_prob = get_lane_prob(lane_pose, road_map);
 		if (lane_prob > max_lane_prob ||
 			(lane_prob == max_lane_prob && fabs(delta_pose) < min_delta_pose))
 		{
 			max_lane_prob = lane_prob;
 			min_delta_pose = fabs(delta_pose);
 			rddf_pose = lane_pose;
 		}
 	}

 	return(rddf_pose);
}


int
fill_in_poses_ahead_by_road_map(const waypoint_t initial_pose, const map_t road_map, waypoint_t *poses_ahead, const int num_poses_desired)
{
	waypoint_t rddf_pose, previous_pose, rddf_pose_candidate;
	int num_poses = 0;
	int num_poses_avg = 5;
	double rddf_min_distance_between_waypoints = 0.5;
	previous_pose = rddf_pose = find_nearest_pose_by_road_map(initial_pose, road_map);
	rddf_pose_candidate = add_distance_to_pose(previous_pose, rddf_min_distance_between_waypoints);

	if(is_point_inside_map(rddf_pose, road_map))
	{
		poses_ahead[0] = rddf_pose;
		num_poses = 1;
		do
		{
			rddf_pose = find_nearest_pose_by_road_map(rddf_pose_candidate, road_map);
			rddf_pose.theta = atan2(rddf_pose.y - previous_pose.y, rddf_pose.x - previous_pose.x);
			previous_pose = rddf_pose;
			previous_pose.theta = average_theta(poses_ahead, num_poses, num_poses_avg);
			rddf_pose_candidate = add_distance_to_pose(previous_pose, rddf_min_distance_between_waypoints);
			if(!is_point_inside_map(rddf_pose, road_map))
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
get_file_basename(const char* filepath)
{
    const char* p1;
    const char* p2;

    p1 = strrchr(filepath, '/');
    p2 = strrchr(filepath, '\\');

    if(p1 == NULL && p2 == NULL)
    {
        return(filepath);
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
    map_t map = {road_map, x_center, y_center, resolution};
    waypoint_t coord;
    double prob;

    int count_neg1 = 0;
    int count_00 = 0;
    int count_02 = 0;
    int count_04 = 0;
    int count_06 = 0;
    int count_08 = 0;
    int count_10 = 0;

    for(int row = 0; row < y_size; row++)
    {
        for(int col = 0; col < x_size; col++)
        {
            coord.x = x_center + (((double)col + 0.5) - ((double)x_size / 2.0)) * resolution;
            coord.y = y_center + (((double)y_size / 2.0) - ((double)row + 0.5)) * resolution;

            prob = get_lane_prob(coord, map);

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


waypoint_t*
infer_rddf_from_img(const waypoint_t *waypoints, int waypoint_index, const map_t road_map, int num_poses_desired, int *num_poses_filled)
{
	waypoint_t initial_pose;
	waypoint_t *poses_ahead;

	if(waypoints == NULL)
	{
		printf("waypoints passed as a null pointer to infer_rddf_from_img().\n");
		return(NULL);
	}

	if(waypoint_index < 0)
	{
		printf("waypoint_index passed as a negative number to infer_rddf_from_img().\n");
		return(NULL);
	}

	if(num_poses_filled == NULL)
	{
		printf("num_poses_filled passed as a null pointer to infer_rddf_from_img().\n");
		return(NULL);
	}

	poses_ahead = (waypoint_t *) malloc(num_poses_desired * sizeof(waypoint_t));
	if(poses_ahead == NULL)
	{
		printf("Unable to alloc memory for poses_ahead in infer_rddf_from_img().\n");
		return(NULL);
	}

	initial_pose = waypoints[waypoint_index];

	*num_poses_filled = fill_in_poses_ahead_by_road_map(initial_pose, road_map, poses_ahead, num_poses_desired);

	if(*num_poses_filled <= 0)
	{
		free(poses_ahead);
		return(NULL);
	}

	return(poses_ahead);
}


int
find_first_waypoint_inside_map(waypoint_t *waypoints, int num_waypoints, double map_min_x, double map_max_x, double map_min_y, double map_max_y)
{
    int i;

    for(i = 0; i < num_waypoints; i++)
    {
        if(waypoints[i].x >= map_min_x && waypoints[i].x < map_max_x &&
           waypoints[i].y >= map_min_y && waypoints[i].y < map_max_y)
        {
            return(i);
        }
    }

    return(-1);
}


double
distance_point_to_segment(double px, double py, double ax, double ay, double bx, double by)
{
	double abx;
	double aby;
	double apx;
	double apy;
	double ab_len_sq;
	double t;
	double qx;
	double qy;
	double dx;
	double dy;

	abx = bx - ax;
	aby = by - ay;
	apx = px - ax;
	apy = py - ay;

	ab_len_sq = abx * abx + aby * aby;

	if(ab_len_sq == 0.0)
	{
		dx = px - ax;
		dy = py - ay;
		return(sqrt(dx * dx + dy * dy));
	}

	t = (apx * abx + apy * aby) / ab_len_sq;

	if(t < 0.0)
	{
		t = 0.0;
	}
	else if(t > 1.0)
	{
		t = 1.0;
	}

	qx = ax + t * abx;
	qy = ay + t * aby;

	dx = px - qx;
	dy = py - qy;

	return(sqrt(dx * dx + dy * dy));
}


double
distance_to_rddf(const waypoint_t waypoint, const waypoint_t *rddf, int num_rddf, double *dtheta)
{
	int i;
	double min_distance;
	double best_theta_segment;
	double ax;
	double ay;
	double bx;
	double by;
	double abx;
	double aby;
	double apx;
	double apy;
	double ab_len_sq;
	double t;
	double qx;
	double qy;
	double dx;
	double dy;
	double distance;

	if(dtheta != NULL)
	{
		*dtheta = 0.0;
	}

	if(rddf == NULL)
	{
		return(-1.0);
	}

	if(num_rddf <= 0)
	{
		return(-1.0);
	}

	if(num_rddf == 1)
	{
		dx = waypoint.x - rddf[0].x;
		dy = waypoint.y - rddf[0].y;

		if(dtheta != NULL)
		{
			*dtheta = carmen_normalize_theta(waypoint.theta - rddf[0].theta);
		}

		return(sqrt(dx * dx + dy * dy));
	}

	min_distance = -1.0;
	best_theta_segment = 0.0;

	for(i = 0; i < (num_rddf - 1); i++)
	{
		ax = rddf[i].x;
		ay = rddf[i].y;
		bx = rddf[i + 1].x;
		by = rddf[i + 1].y;

		abx = bx - ax;
		aby = by - ay;
		apx = waypoint.x - ax;
		apy = waypoint.y - ay;

		ab_len_sq = abx * abx + aby * aby;

		if(ab_len_sq == 0.0)
		{
			qx = ax;
			qy = ay;
		}
		else
		{
			t = (apx * abx + apy * aby) / ab_len_sq;

			if(t < 0.0)
			{
				t = 0.0;
			}
			else if(t > 1.0)
			{
				t = 1.0;
			}

			qx = ax + t * abx;
			qy = ay + t * aby;
		}

		dx = waypoint.x - qx;
		dy = waypoint.y - qy;
		distance = sqrt(dx * dx + dy * dy);

		if(min_distance < 0.0 || distance < min_distance)
		{
			min_distance = distance;

			if(ab_len_sq == 0.0)
			{
				best_theta_segment = rddf[i].theta;
			}
			else
			{
				best_theta_segment = atan2(aby, abx);
			}
		}
	}

	if(dtheta != NULL)
	{
		*dtheta = carmen_normalize_theta(waypoint.theta - best_theta_segment);
	}

	return(min_distance);
}


int
ends_with(const char* str, const char* suffix)
{
    size_t str_len;
    size_t suffix_len;

    if(str == NULL || suffix == NULL)
    {
        return(0);
    }

    str_len = strlen(str);
    suffix_len = strlen(suffix);

    if(str_len < suffix_len)
    {
        return(0);
    }

    if(strcmp(str + (str_len - suffix_len), suffix) == 0)
    {
        return(1);
    }

    return(0);
}


int
build_pred_filename_from_gt(const char* gt_filename, char* pred_filename, size_t pred_filename_size)
{
    size_t gt_len;
    size_t stem_len;

    if(gt_filename == NULL || pred_filename == NULL || pred_filename_size <= 0)
    {
        return(0);
    }

    if(!ends_with(gt_filename, "_gt.png"))
    {
        return(0);
    }

    gt_len = strlen(gt_filename);
    stem_len = gt_len - strlen("_gt.png");

    if((stem_len + strlen("_pred.png") + 1) > pred_filename_size)
    {
        return(0);
    }

    memcpy(pred_filename, gt_filename, stem_len);
    strcpy(pred_filename + stem_len, "_pred.png");

    return(1);
}


void
compare_rddfs(const waypoint_t *inferred_rddf_gt,   int num_inferred_gt,
              const waypoint_t *inferred_rddf_pred, int num_inferred_pred,
              const char *gt_filename, int verbose)
{
    int i;
    int n;
    double dtheta;
    double abs_dtheta;
    double dist;
    double sum_dist;
    double sum_sq_dist;
    double max_dist;
    double mean_dist;
    double var_dist;
    double std_dev_dist;
    double sum_abs_dtheta;
    double sum_sq_abs_dtheta;
    double max_abs_dtheta;
    double mean_abs_dtheta;
    double var_abs_dtheta;
    double std_dev_abs_dtheta;

    if(inferred_rddf_gt == NULL || inferred_rddf_pred == NULL)
    {
        printf("Aviso: compare_rddfs recebeu ponteiro nulo para arquivo %s\n", gt_filename);
        return;
    }

    n = num_inferred_pred;

    if(n <= 0)
    {
        printf("Aviso: nenhum waypoint inferido comparavel para arquivo %s\n", gt_filename);
        return;
    }

    sum_dist = 0.0;
    sum_sq_dist = 0.0;
    max_dist = 0.0;

    sum_abs_dtheta = 0.0;
    sum_sq_abs_dtheta = 0.0;
    max_abs_dtheta = 0.0;

    for(i = 0; i < n; i++)
    {
        dist = distance_to_rddf(inferred_rddf_pred[i], inferred_rddf_gt, num_inferred_gt, &dtheta);
        abs_dtheta = fabs(dtheta);

        sum_dist += dist;
        sum_sq_dist += dist * dist;

        if(dist > max_dist)
        {
            max_dist = dist;
        }

        sum_abs_dtheta += abs_dtheta;
        sum_sq_abs_dtheta += abs_dtheta * abs_dtheta;

        if(abs_dtheta > max_abs_dtheta)
        {
            max_abs_dtheta = abs_dtheta;
        }

        if(verbose)
        {
            printf("%s waypoint %d: dist=%.17g dtheta=%.17g abs_dtheta=%.17g\n",
                   gt_filename, i, dist, dtheta, abs_dtheta);
        }
    }

    mean_dist = sum_dist / (double)n;
    var_dist = (sum_sq_dist / (double)n) - (mean_dist * mean_dist);
    if(var_dist < 0.0)
    {
        var_dist = 0.0;
    }
    std_dev_dist = sqrt(var_dist);

    mean_abs_dtheta = sum_abs_dtheta / (double)n;
    var_abs_dtheta = (sum_sq_abs_dtheta / (double)n) - (mean_abs_dtheta * mean_abs_dtheta);
    if(var_abs_dtheta < 0.0)
    {
        var_abs_dtheta = 0.0;
    }
    std_dev_abs_dtheta = sqrt(var_abs_dtheta);

    printf("%s: num_gt=%d num_pred=%d comparados=%d media_dist=%.17g max_dist=%.17g std_dev_dist=%.17g media_abs_dtheta=%.17g max_abs_dtheta=%.17g std_dev_abs_dtheta=%.17g\n",
           gt_filename,
           num_inferred_gt,
           num_inferred_pred,
           n,
           mean_dist,
           max_dist,
           std_dev_dist,
           mean_abs_dtheta,
           max_abs_dtheta,
           std_dev_abs_dtheta);
}


void
read_images(const char* dir_img_gt, const char* dir_img_pred, const waypoint_t* rddf, int num_rddf, int verbose)
{
    DIR* dirp_gt;
    struct dirent* entry;
    char gt_filepath[4096];
    char pred_filename[1024];
    char pred_filepath[4096];
    cv::Mat road_map_img_gt;
    cv::Mat road_map_img_pred;
    map_t road_map_gt;
    map_t road_map_pred;
    double x_center;
    double y_center;
    int total_gt_png_files;
    int matched_pairs;
    int skipped_files;
    int waypoint_index;
    waypoint_t *inferred_rddf_gt;
    waypoint_t *inferred_rddf_pred;
    int num_inferred_gt;
    int num_inferred_pred;
    int num_poses_desired = 150;

    if(dir_img_gt == NULL || dir_img_pred == NULL || rddf == NULL || num_rddf <= 0)
    {
        printf("Erro: parametros invalidos em read_images().\n");
        return;
    }

    dirp_gt = opendir(dir_img_gt);

    if(dirp_gt == NULL)
    {
        printf("Erro: nao foi possivel abrir o diretorio de ground-truth: %s\n", dir_img_gt);
        return;
    }

    total_gt_png_files = 0;
    matched_pairs = 0;
    skipped_files = 0;

    while(1)
    {
        entry = readdir(dirp_gt);

        if(entry == NULL)
        {
            break;
        }

        if(!has_png_extension(entry->d_name))
        {
            continue;
        }

        total_gt_png_files++;

        if(!ends_with(entry->d_name, "_gt.png"))
        {
            if(verbose)
            {
                printf("Aviso: arquivo ignorado por nao seguir padrao *_gt.png: %s\n", entry->d_name);
            }
            skipped_files++;
            continue;
        }

        if(!parse_centers_from_filename(entry->d_name, &x_center, &y_center))
        {
            if(verbose)
            {
                printf("Aviso: nao foi possivel extrair centros do nome: %s\n", entry->d_name);
            }
            skipped_files++;
            continue;
        }

        if(!build_pred_filename_from_gt(entry->d_name, pred_filename, sizeof(pred_filename)))
        {
            if(verbose)
            {
                printf("Aviso: nao foi possivel montar nome predito para: %s\n", entry->d_name);
            }
            skipped_files++;
            continue;
        }

        if(snprintf(gt_filepath, sizeof(gt_filepath), "%s/%s", dir_img_gt, entry->d_name) >= (int)sizeof(gt_filepath))
        {
            printf("Aviso: caminho muito longo, ignorando GT: %s\n", entry->d_name);
            skipped_files++;
            continue;
        }

        if(snprintf(pred_filepath, sizeof(pred_filepath), "%s/%s", dir_img_pred, pred_filename) >= (int)sizeof(pred_filepath))
        {
            printf("Aviso: caminho muito longo, ignorando PRED correspondente a: %s\n", entry->d_name);
            skipped_files++;
            continue;
        }

        road_map_img_gt = cv::imread(gt_filepath, cv::IMREAD_COLOR);
        if(road_map_img_gt.empty())
        {
            printf("Aviso: nao foi possivel ler imagem GT: %s\n", gt_filepath);
            skipped_files++;
            continue;
        }

        road_map_img_pred = cv::imread(pred_filepath, cv::IMREAD_COLOR);
        if(road_map_img_pred.empty())
        {
            printf("Aviso: nao foi possivel ler imagem PRED correspondente: %s\n", pred_filepath);
            skipped_files++;
            continue;
        }

        road_map_gt.img = road_map_img_gt;
        road_map_gt.x_center = x_center;
        road_map_gt.y_center = y_center;
        road_map_gt.resolution = 0.2;

        road_map_pred.img = road_map_img_pred;
        road_map_pred.x_center = x_center;
        road_map_pred.y_center = y_center;
        road_map_pred.resolution = 0.2;

        waypoint_index = 0;

        while(waypoint_index < num_rddf)
        {
            if(is_point_inside_map(rddf[waypoint_index], road_map_gt))
            {
                break;
            }

            waypoint_index++;
        }

        if(waypoint_index >= num_rddf)
        {
            if(verbose)
            {
                printf("Aviso: nenhum waypoint da RDDF caiu dentro do mapa para %s\n", entry->d_name);
            }
            skipped_files++;
            continue;
        }

        inferred_rddf_gt   = infer_rddf_from_img(rddf, waypoint_index, road_map_gt,   num_poses_desired, &num_inferred_gt);
        inferred_rddf_pred = infer_rddf_from_img(rddf, waypoint_index, road_map_pred, num_poses_desired, &num_inferred_pred);

        if(inferred_rddf_gt == NULL || inferred_rddf_pred == NULL)
        {
            if(verbose)
            {
                if(inferred_rddf_gt == NULL)
                {
                	printf("Aviso: falha ao inferir RDDF para %s\n", gt_filepath);
                }

                if(inferred_rddf_pred == NULL)
                {
                	printf("Aviso: falha ao inferir RDDF para %s\n", pred_filepath);
                }
            }

            if(inferred_rddf_gt != NULL)
            {
                free(inferred_rddf_gt);
            }

            if(inferred_rddf_pred != NULL)
            {
                free(inferred_rddf_pred);
            }

            skipped_files++;
            continue;
        }

        compare_rddfs(inferred_rddf_gt,   num_inferred_gt,
                      inferred_rddf_pred, num_inferred_pred,
                      entry->d_name, verbose);

        free(inferred_rddf_gt);
        free(inferred_rddf_pred);

        matched_pairs++;
    }

    closedir(dirp_gt);

    printf("Diretorio GT: %s\n", dir_img_gt);
    printf("Diretorio PRED: %s\n", dir_img_pred);
    printf("Arquivos PNG encontrados em GT: %d\n", total_gt_png_files);
    printf("Pares processados com sucesso: %d\n", matched_pairs);
    printf("Arquivos ignorados: %d\n", skipped_files);
}


waypoint_t*
get_rddf_from_file(const char* txt_filename, int* num_waypoints)
{
	FILE *f;
	char line[4096];
	waypoint_t *waypoints;
	int capacity;
	int count;
	double x;
	double y;
	double theta;
	int n;

	if(num_waypoints == NULL)
	{
		return(NULL);
	}

	*num_waypoints = 0;

	if(txt_filename == NULL)
	{
		return(NULL);
	}

	f = fopen(txt_filename, "r");
	if(f == NULL)
	{
		return(NULL);
	}

	capacity = 1024;
	count = 0;
	waypoints = (waypoint_t *) malloc(capacity * sizeof(waypoint_t));

	if(waypoints == NULL)
	{
        printf("Erro: malloc não conseguiu alocar memoria em get_rddf_from_file()\n");
		fclose(f);
		return(NULL);
	}

	while(fgets(line, sizeof(line), f) != NULL)
	{
		n = sscanf(line, "%lf %lf %lf", &x, &y, &theta);

		if(n < 3)
		{
			continue;
		}

		if(count >= capacity)
		{
			waypoint_t *new_waypoints;

			capacity *= 2;
			new_waypoints = (waypoint_t *) realloc(waypoints, capacity * sizeof(waypoint_t));

			if(new_waypoints == NULL)
			{
		        printf("Erro: realloc não conseguiu alocar memoria em get_rddf_from_file()\n");
				free(waypoints);
				fclose(f);
				return(NULL);
			}

			waypoints = new_waypoints;
		}

		waypoints[count].x = x;
		waypoints[count].y = y;
		waypoints[count].theta = theta;
		count++;
	}

	fclose(f);

	if(count == 0)
	{
        printf("Alerta: nenhum waypoint valido encontrado em get_rddf_from_file()\n");
		free(waypoints);
		return(NULL);
	}

	{
		waypoint_t *new_waypoints;

		new_waypoints = (waypoint_t *) realloc(waypoints, count * sizeof(waypoint_t));
		if(new_waypoints != NULL)
		{
			waypoints = new_waypoints;
		}
	}

	*num_waypoints = count;

	return(waypoints);
}


int
main(int argc, char** argv)
{
    const char* dir_img_gt;
    const char* dir_img_pred;
    const char* rddf_filename;
    waypoint_t *rddf;
    int num_rddf;
    int verbose;

    if(argc != 4 && argc != 5)
    {
        printf("Uso: %s <dir_img_gt> <dir_img_pred> <arquivo_rddf.txt> [-verbose]\n", argv[0]);
        return(1);
    }

    dir_img_gt = argv[1];
    dir_img_pred = argv[2];
    rddf_filename = argv[3];
    verbose = 0;

    if(argc == 5)
    {
        if(strcmp(argv[4], "-verbose") == 0)
        {
            verbose = 1;
        }
        else
        {
            printf("Erro: parametro opcional invalido: %s\n", argv[4]);
            printf("Uso: %s <dir_img_gt> <dir_img_pred> <arquivo_rddf.txt> [-verbose]\n", argv[0]);
            return(1);
        }
    }

    rddf = get_rddf_from_file(rddf_filename, &num_rddf);
    if(rddf == NULL)
    {
        printf("Erro: nao foi possivel ler o arquivo RDDF: %s\n", rddf_filename);
        return(1);
    }

    if(verbose)
    {
        printf("Arquivo RDDF: %s\n", rddf_filename);
        printf("Waypoints lidos: %d\n", num_rddf);
    }

    read_images(dir_img_gt, dir_img_pred, rddf, num_rddf, verbose);

    free(rddf);

    return(0);
}
