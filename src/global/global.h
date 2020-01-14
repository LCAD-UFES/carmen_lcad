/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

/** @addtogroup global libglobal **/
// @{

/** \file global.h
 * \brief Library global. Contains a huge amount of useful things.
 *
 * Library global. Contains a huge amount of useful things.
 **/


#ifndef CARMEN_GLOBAL_H
#define CARMEN_GLOBAL_H


/* to satisfy gcc >= 4.3 */


#if (!defined(__cplusplus)) && (defined(__GNUC_GNU_INLINE__) || defined(__GNUC_STDC_INLINE__))
#define carmen_inline inline __attribute__ ((gnu_inline))
#else
#define carmen_inline inline
#endif


#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <ctype.h>
#ifdef	EXTERNAL_IPC
#include <ipc.h>
#else
#include <carmen/ipc.h>
#endif

#define CARMEN_MAJOR_VERSION 0
#define CARMEN_MINOR_VERSION 8
#define CARMEN_REVISION 0

/* Useful macros */

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef MIN
#define MIN(x,y) (x < y ? x : y)
#endif

#ifndef MIN3
#define MIN3(x,y,z) MIN(MIN(x,y),z)
#endif

#ifndef MAX
#define MAX(x,y) (x > y ? x : y)
#endif

#ifndef MAX3
#define MAX3(x,y,z) MAX(MAX(x,y),z)
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846  // PI
#endif


#define PERP2D(u,v)  ((u).x * (v).y - (u).y * (v).x)

#define DOT2D(x1,x2) ((x1).x * (x2).x + (x1).y * (x2).y)

#define DOT2D_P(x1,x2) ((x1)->x * (x2)->x + (x1)->y * (x2)->y)

#define DIST2D(x1,x2) (sqrt(((x1).x - (x2).x) * ((x1).x - (x2).x) + \
							((x1).y - (x2).y) * ((x1).y - (x2).y)))

#define ANGLE2D(x1,x2) (atan2(((x2).y - (x1).y), ((x2).x - (x1).x)))

#define DIST2D_P(x1,x2) (sqrt(((x1)->x - (x2)->x) * ((x1)->x - (x2)->x) + \
							((x1)->y - (x2)->y) * ((x1)->y - (x2)->y)))

#define DOT3D(x1,x2) ((x1).x * (x2).x + (x1).y * (x2).y + (x1).z * (x2).z)

#define DOT3DP_P(x1,x2) ((x1)->x * (x2)->x + (x1)->y * (x2)->y + (x1)->z * (x2)->z)

#define DIST3D(x1,x2) (sqrt(((x1).x - (x2).x) * ((x1).x - (x2).x) + \
							((x1).y - (x2).y) * ((x1).y - (x2).y) + \
							((x1).z - (x2).z) * ((x1).z - (x2).z)))

#define DIST3D_P(x1,x2) (sqrt(((x1)->x - (x2)->x) * ((x1)->x - (x2)->x) + \
							((x1)->y - (x2)->y) * ((x1)->y - (x2)->y) + \
							((x1)->z - (x2)->z) * ((x1)->z - (x2)->z)))

#define TRANSLATE3D(p,offset) p.x += offset.x; \
							  p.y += offset.y; \
							  p.z += offset.z;

//TODO Verificar se esta funcionando
#define ROTATE3DZ(p,theta) p.x = p.x * cos(theta) - p.y * sin(theta); \
						   p.y = p.x * sin(theta) + p.y * cos(theta);


#define	POINT_WITHIN_SEGMENT		0
#define	SEGMENT_TOO_SHORT			1
#define	POINT_BEFORE_SEGMENT		2
#define	POINT_AFTER_SEGMENT			3

typedef struct {
	double x;
	double y;
	double z;
} carmen_vector_3D_t, *carmen_vector_3D_p;

typedef struct
{
	double horizontal_angle;
	double vertical_angle;
	double length;
}carmen_sphere_coord_t, *carmen_sphere_coord_p;


typedef struct {
	double x;
	double y;
} carmen_vector_2D_t, *carmen_vector_2D_p;


typedef struct {
	double roll;
	double pitch;
	double yaw;
} carmen_orientation_3D_t, *carmen_orientation_3D_p;


typedef struct {
	double q0;
	double q1;
	double q2;
	double q3;
} carmen_quaternion_t, *carmen_quaternion_3D_p;

typedef struct carmen_pose_3D {
	carmen_vector_3D_t position;
	carmen_orientation_3D_t orientation;
} carmen_pose_3D_t, *carmen_pose_3D_p;

typedef struct {
	int x, y;
}carmen_image_coord_t;

typedef struct {
	carmen_image_coord_t coordinates;
	carmen_vector_3D_t pose;
} carmen_saliency_t, *carmen_saliency_p;

typedef struct {
	double x;
	double y;
} carmen_position_t, *carmen_position_p;


typedef struct {
	double x;
	double y;
	double theta;
} carmen_point_t, *carmen_point_p;


typedef struct {
	double x;
	double y;
	double theta;
	double t_vel;
	double r_vel;
} carmen_traj_point_t, *carmen_traj_point_p;


typedef struct {
	double x;
	double y;
	double theta;
	double v;
	double phi;
} carmen_ackerman_traj_point_t, *carmen_ackerman_traj_point_p;


typedef struct {
	double x;
	double y;
	double theta;
	double v;
	double phi;
	double time;
} carmen_ackerman_path_point_t, *carmen_ackerman_path_point_p;


typedef struct { // Mudado para incluir x, y, e theta para testar novo mpc de posicao. Este tipo deve ser mudado para o tipo acima se der certo o novo mpc.
	double x;
	double y;
	double theta;
	double v;
	double phi;
	double time;
} carmen_ackerman_motion_command_t, *carmen_ackerman_motion_command_p;


typedef struct {
	double max_r_vel;
	double max_t_vel;
	double acceleration;
	double deceleration;
	double approach_dist;
	double side_dist;
	double length;
	double width;
	double reaction_time;
	int allow_rear_motion;
	int rectangular;
	int interpolate_odometry;
} carmen_robot_config_t;


typedef struct {
	double distance_between_rear_wheels;
	double distance_between_front_and_rear_axles;
	double distance_between_rear_car_and_rear_wheels;
	double distance_between_front_car_and_front_wheels;
} carmen_car_config_t;

typedef struct {
	double max_phi;
	double max_v;
	double distance_between_rear_wheels;
	double distance_between_front_and_rear_axles;
	double distance_between_rear_car_and_rear_wheels;
	double distance_between_front_car_and_front_wheels;
	double distance_between_rearview;
	double approach_dist;
	double side_dist;
	double length;
	double width;
	double reaction_time;
	int allow_rear_motion;
	int rectangular;
	int interpolate_odometry;
	double maximum_acceleration_forward;
	double maximum_deceleration_forward;
	double maximum_deceleration_reverse;
	double maximum_acceleration_reverse;
	double maximum_steering_command_rate;
	double understeer_coeficient;

	double obstacle_avoider_obstacles_safe_distance;
	double model_predictive_planner_obstacles_safe_distance; /*< distance used for car collision radius */
	double model_predictive_planner_obstacles_safe_length_distance; /* distance from rear to front for moving obstacle collision check */

	double behaviour_selector_main_central_lane_obstacles_safe_distance;
	double behaviour_selector_central_lane_obstacles_safe_distance;
	double behaviour_selector_lateral_lane_obstacles_safe_distance;
	double behaviour_selector_lateral_lane_displacement;

	double behaviour_selector_goal_velocity_tuning_factor; // Correct imperfections of the velocity control by modifying the braking starting point
} carmen_robot_ackerman_config_t;

typedef struct {
    double x;
    double y;
    double theta;
    double length;
    double width;
} carmen_rectangle_t;


typedef struct {
    carmen_position_t P0;
    carmen_position_t P1;
} carmen_line_segment_t;


typedef struct _spherical_point_cloud
{
	int num_points;
	carmen_sphere_coord_t *sphere_points;
	double timestamp;
} spherical_point_cloud;


typedef struct _cell_coords_t
{
	int x, y;
} cell_coords_t;


typedef struct {
	int ipx;
	int ipy;
	carmen_sphere_coord_t laser_polar;
} carmen_velodyne_points_in_cam_t, *carmen_velodyne_points_in_cam_p;


typedef enum { CARMEN_MOTOR, CARMEN_SERVO } carmen_arm_joint_t;

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} carmen_bresenham_param_t;

typedef void (*carmen_usage_func)(char *fmt, ...);

typedef struct {
	int length;
	int capacity;
	int entry_size;
	void *list;
} carmen_list_t;

typedef struct {
	double timestamp;
	char *host;
} carmen_default_message;

#define CARMEN_DEFAULT_MESSAGE_FMT "{double,string}"

typedef struct {
	char *module_name;
	int pid;
	double timestamp;
	char *hostname;
} carmen_heartbeat_message;

#define CARMEN_HEARTBEAT_NAME "carmen_heartbeat"
#define CARMEN_HEARTBEAT_FMT "{string, int, double, string}"

#define carmen_red_code "[31;1m"
#define carmen_blue_code "[34;1m"
#define carmen_normal_code "[0m"

#define carmen_time_code(code, str) { double time_code_t1, time_code_t2; time_code_t1 = carmen_get_time(); code; time_code_t2 = carmen_get_time(); fprintf(stderr, "%-20s : %.2f ms.\n", str, (time_code_t2 - time_code_t1) * 1000.0); }

void carmen_test_ipc(IPC_RETURN_TYPE err, const char *err_msg, const char *ipc_msg);

#define carmen_test_alloc(X) do {if ((void *)(X) == NULL) carmen_die("Out of memory in %s, (%s, line %d).\n", __FUNCTION__, __FILE__, __LINE__); } while (0)

#define carmen_test_ipc_return(ERR, ERR_MSG, IPC_MSG) do {carmen_test_ipc((ERR), (ERR_MSG), (IPC_MSG)); if ((ERR) != IPC_OK) return; } while (0)

#define carmen_test_ipc_return_int(ERR, ERR_MSG, IPC_MSG) do {carmen_test_ipc((ERR), (ERR_MSG), (IPC_MSG)); if ((ERR) != IPC_OK) return -1; } while (0)

#define carmen_test_ipc_return_null(ERR, ERR_MSG, IPC_MSG) do {carmen_test_ipc((ERR), (ERR_MSG), (IPC_MSG)); if ((ERR) != IPC_OK) return NULL; } while (0)

#define carmen_test_ipc_exit(ERR, ERR_MSG, IPC_MSG) do {carmen_test_ipc((ERR), (ERR_MSG), (IPC_MSG)); if ((ERR) != IPC_OK) {fprintf(stderr, "This is a fatal error. Exiting.\n"); exit(-1);} } while (0)

int carmen_find_param(const char *lvalue);

int carmen_find_param_pair(const char *lvalue);

char *carmen_find_robot_name(int argc, char **argv);

char *carmen_param_pair(const char *lvalue);

char *carmen_param_pair_and_remove(const char *lvalue);

int carmen_num_params(void);

char *carmen_get_param_by_num(int param_index);

int carmen_read_commandline_parameters(int argc, char **argv);

int carmen_process_param_int(char *lvalue, carmen_usage_func usage, int *return_value);

double carmen_process_param_double(char *lvalue, carmen_usage_func usage, double *return_value);

int carmen_process_param_onoff(char *lvalue, carmen_usage_func usage, int *return_value);

char *carmen_process_param_string(char *lvalue, carmen_usage_func usage);

char *carmen_process_param_file(char *lvalue, carmen_usage_func usage);

char *carmen_process_param_directory(char *lvalue, carmen_usage_func usage);

char *carmen_extract_filename(char *path);

void carmen_perror(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
void carmen_verbose(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void carmen_warn(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
void carmen_die(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
void carmen_die_syserror(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
void carmen_carp_set_verbose(int verbosity);
int carmen_carp_get_verbose(void);
void carmen_carp_set_output(FILE *output);

extern carmen_inline double carmen_get_time(void)
{
	struct timeval tv;
	double t;

	if (gettimeofday(&tv, NULL) < 0)
		carmen_warn("carmen_get_time encountered error in gettimeofday : %s\n",
				strerror(errno));
	t = tv.tv_sec + tv.tv_usec/1000000.0;
	return t;
}

char *carmen_get_host(void);

carmen_default_message *carmen_default_message_create(void);

#define AVERAGE_LENGTH 20

typedef struct {
	double data[AVERAGE_LENGTH];
	int index;
	double sum;
	int count;
} carmen_running_average_t;

void carmen_running_average_clear(carmen_running_average_t *average);
void carmen_running_average_add(carmen_running_average_t *average, double x);
double carmen_running_average_report(carmen_running_average_t *average);

/* This weirdness of extern carmen_inline is to allow the function to be carmen_inlined
   outside the library. Guess what! There's an exact copy of this function in
   global.c as well. <sigh> */

extern carmen_inline int carmen_round(double X)
{
	if (X >= 0)
		return (int)(X + 0.5);
	else
		return (int)(X - 0.5);
}

extern carmen_inline double carmen_clamp(double X, double Y, double Z)
{
	if (Y < X)
		return X;
	else if (Y > Z)
		return Z;
	return Y;
}

extern carmen_inline int carmen_trunc(double X)
{
	return (int)(X);
}

extern carmen_inline void carmen_erase_structure(void* ptr, int size_of_struture)
{
	memset(ptr, 0, size_of_struture);
}


extern carmen_inline double carmen_normalize_theta(double theta)
{
	int multiplier;

	if (theta >= -M_PI && theta < M_PI)
		return theta;

	multiplier = (int)(theta / (2*M_PI));
	theta = theta - multiplier*2*M_PI;
	if (theta >= M_PI)
		theta -= 2*M_PI;
	if (theta < -M_PI)
		theta += 2*M_PI;

	return theta;
}

extern carmen_inline double carmen_knots_to_meters_per_second(double knots)
{
	/// KNOTS_TO_METERS_PER_SECOND 0.5148
	return (0.5148 * knots);
}

extern carmen_inline double carmen_radians_to_degrees(double theta)
{
	return (theta * 180.0 / M_PI);
}

extern carmen_inline double carmen_degrees_to_radians(double theta)
{
	return (theta * M_PI / 180.0);
}

extern carmen_inline int carmen_imin(int val1, int val2)
{
	if (val2 < val1)
		return val2;
	return val1;
}

extern carmen_inline int carmen_imax(int val1, int val2)
{
	if (val2 > val1)
		return val2;
	return val1;
}

extern carmen_inline double carmen_fmin(double val1, double val2)
{
	if (val2 < val1)
		return val2;
	return val1;
}

extern carmen_inline double carmen_fmax(double val1, double val2)
{
	if (val2 > val1)
		return val2;
	return val1;
}

extern carmen_inline double carmen_square(double val)
{
	return (val*val);
}

double carmen_global_convert_degmin_to_double(double dm_format);

extern carmen_inline double carmen_distance_traj(carmen_traj_point_p p1, carmen_traj_point_p p2)
{
	return sqrt((p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y));
}

extern carmen_inline double carmen_distance_ackerman_traj(carmen_ackerman_traj_point_p p1, carmen_ackerman_traj_point_p p2)
{
	return sqrt((p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y));
}

extern carmen_inline double carmen_ackerman_traj_distance2(carmen_ackerman_traj_point_p p1, carmen_ackerman_traj_point_p p2)
{
	return (p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y);
}

extern carmen_inline double carmen_angle_between(carmen_traj_point_p p1, carmen_traj_point_p p2)
{
	return atan2(p2->y - p1->y, p2->x - p1->x);
}

extern carmen_inline double carmen_distance(carmen_point_p p1, carmen_point_p p2)
{
	return sqrt((p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y));
}

void carmen_get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y,
		carmen_bresenham_param_t *params);
extern carmen_inline void carmen_get_current_point(carmen_bresenham_param_t *params, int *x, int *y)
{
	if (params->UsingYIndex)
	{
		*y = params->XIndex;
		*x = params->YIndex;
		if (params->Flipped)
			*x = -*x;
	}
	else
	{
		*x = params->XIndex;
		*y = params->YIndex;
		if (params->Flipped)
			*y = -*y;
	}
}

extern carmen_inline int carmen_get_next_point(carmen_bresenham_param_t *params)
{
	if (params->XIndex == params->X2)
	{
		return 0;
	}
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else
	{
		params->DTerm += params->IncrNE;
		params->YIndex += params->Increment;
	}
	return 1;
}

int carmen_sign(double num);

void carmen_rect_to_polar(double x, double y, double *r, double *theta);

void carmen_rotate_2d(double *x, double *y, double theta);

unsigned int carmen_generate_random_seed(void);
unsigned int carmen_randomize(int *argc, char ***argv);
void carmen_set_random_seed(unsigned int seed);

int carmen_int_random(int max);
double carmen_double_random(double max);
double carmen_uniform_random(double min, double max);
double carmen_gaussian_random(double mean, double std);
double carmen_normal_distribution(double std);

int carmen_file_exists(const char *filename);
char *carmen_file_extension(const char *filename);
char *carmen_file_find(const char *filename);
char **carmen_get_search_path(int *num_paths);

void carmen_global_start_progress_bar(char *label);
void carmen_global_end_progress_bar(void);
void carmen_global_update_progress_bar(int count, int size);

int carmen_strcasecmp (const char *s1, const char *s2);
int carmen_strncasecmp (const char *s1, const char *s2, size_t n);

char *carmen_new_string(const char *fmt, ...);
//char *carmen_new_stringv(const char *fmt, va_list ap); FIXME Python wrapper SWIG does not know how to handle va_list

void carmen_print_version(void);

int carmen_parse_sonar_offsets(char *offset_string, carmen_point_p offsets,
		int num_sonars);

int carmen_parse_bumper_offsets(char *offset_string, carmen_position_p offsets, int num_bumpers);

int carmen_parse_arm_joint_types(char *joint_string, carmen_arm_joint_t *joint_types,
		int num_joints);

int carmen_terminal_cbreak(int blocking);
int carmen_terminal_restore(void);

carmen_list_t *carmen_list_create(int entry_size, int initial_capacity);
carmen_list_t *carmen_list_create_from_data(int entry_size, int num_elements,
		void *data);
carmen_list_t *carmen_list_duplicate(carmen_list_t *list);
void carmen_list_add(carmen_list_t *list, void *entry);
void carmen_list_insert(carmen_list_t *list, void *entry, int i);
void carmen_list_delete(carmen_list_t *list, int entry_num);
void *carmen_list_get(carmen_list_t *list, int entry_num);
void carmen_list_set(carmen_list_t *list, int entry_num, void *entry);
int carmen_list_length(carmen_list_t *list);
void carmen_list_destroy(carmen_list_t **list);

void carmen_eigs_to_covariance(double theta, double major, double minor,
		double *vx, double *vxy, double *vy);

double carmen_get_curvature_from_phi(double phi, double v, double understeer_coeficient, double distance_between_front_and_rear_axles);
double carmen_get_phi_from_curvature(double curvature, double v, double understeer_coeficient, double distance_between_front_and_rear_axles);

carmen_vector_3D_t carmen_covert_sphere_to_cartesian_coord(carmen_sphere_coord_t sphere_point);
void carmen_alloc_spherical_point_cloud(spherical_point_cloud *velodyne_points, int num_points, int spherical_point_cloud_index);
void carmen_add_bias_and_multiplier_to_v_and_phi(double *odometry_v, double *odometry_phi, double raw_v, double raw_phi, 
					    double v_bias, double v_multiplier, double phi_bias, double phi_multiplier);
carmen_ackerman_traj_point_t
carmen_get_point_nearest_to_trajectory(int *point_in_trajectory_is,
		carmen_ackerman_traj_point_t v,
		carmen_ackerman_traj_point_t w,
		carmen_ackerman_traj_point_t p, double min_segment_size);

int
carmen_line_to_point_crossed_rectangle(carmen_position_t *intersection, carmen_position_t origin, carmen_position_t point, carmen_rectangle_t rectangle);

int
carmen_line_to_line_intersection(carmen_position_t *I0, carmen_position_t s1_p0, carmen_position_t s1_p1, carmen_position_t s2_p0, carmen_position_t s2_p1);

extern carmen_inline char *carmen_next_word(char *str)
{
	char *mark = str;

	if(str == NULL)
		return NULL;
	while(*mark != '\0' && !(*mark == ' ' || *mark == '\t'))
		mark++;
	while(*mark != '\0' &&  (*mark == ' ' || *mark == '\t'))
		mark++;
	return mark;
}

extern carmen_inline char *carmen_next_n_words(char *str, int n)
{
	int i;
	char *result;

	result = str;
	for(i = 0; i < n; i++)
		result = carmen_next_word(result);
	return result;
}

#ifdef __cplusplus
}
#endif

#endif
// @}
