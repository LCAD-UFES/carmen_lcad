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

#include <carmen/carmen.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/xsens_interface.h>
#include <carmen/car_model.h>
#include <carmen/task_manager_interface.h>
#include <carmen/task_manager_messages.h>

#include <prob_measurement_model.h>
#include <prob_map.h>

#include <gsl/gsl_fit.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_errno.h>

#include "localize_ackerman_core.h"
#include "localize_ackerman_messages.h"
#include "localize_ackerman_interface.h"
#include "localize_ackerman_velodyne.h"
#include "localize_ackerman_beta_particle_filter.h"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#define PREDICT_BETA_GSL_ERROR_CODE 100.0
#define MIN_DISTANCE_BETWEEN_POINTS	(semi_trailer_config.beta_correct_max_distance / 100.0)
#define MIN_CLUSTER_SIZE			10

static int necessary_maps_available = 0;

#define BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE 50
static carmen_base_ackerman_odometry_message base_ackerman_odometry_vector[BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE];
static int base_ackerman_odometry_index = -1;

#define FUSED_ODOMETRY_VECTOR_SIZE 50
static carmen_fused_odometry_message fused_odometry_vector[FUSED_ODOMETRY_VECTOR_SIZE];
static int g_fused_odometry_index = -1;

/* global variables */
carmen_map_t *new_map = NULL;
carmen_localize_ackerman_map_t localize_map;
carmen_localize_ackerman_particle_filter_p filter;
carmen_localize_ackerman_particle_filter_p beta_filter;
carmen_localize_ackerman_summary_t summary;

carmen_map_t local_map;
carmen_map_t local_sum_remission_map;
carmen_map_t local_mean_remission_map;
carmen_map_t local_variance_remission_map;
carmen_map_t local_sum_sqr_remission_map;
carmen_map_t local_count_remission_map;

carmen_compact_map_t local_compacted_map;
carmen_compact_map_t local_compacted_mean_remission_map;
carmen_compact_map_t local_compacted_variance_remission_map;
carmen_localize_ackerman_binary_map_t binary_map;

carmen_robot_ackerman_laser_message front_laser;

carmen_xsens_global_quat_message *xsens_global_quat_message = NULL;

// Variables read via read_parameters()
extern carmen_robot_ackerman_config_t 	car_config;
extern carmen_semi_trailer_config_t 	semi_trailer_config;

extern int robot_publish_odometry;

extern int number_of_sensors;
extern sensor_parameters_t *spherical_sensor_params;
extern sensor_data_t *spherical_sensor_data;

extern int use_raw_laser;
extern int mapping_mode;

extern char *save_globalpos_file;
extern double save_globalpos_timestamp;
FILE *globalpos_file = NULL;

carmen_localize_ackerman_globalpos_message globalpos = {};

//extern double laser_ranges[10000];

cell_coords_t **map_cells_hit_by_each_rays = NULL;

carmen_point_t g_std;
int g_reinitiaze_particles = 10;
bool global_localization_requested = false;

static carmen_velodyne_partial_scan_message *last_velodyne_message = NULL;

carmen_behavior_selector_path_goals_and_annotations_message *behavior_selector_path_goals_and_annotations_message = NULL;


static int
get_fused_odometry_index_by_timestamp(double timestamp)
{
	double min_diff, diff;
	int min_index;

	min_diff = DBL_MAX;
	min_index = -1;

	for (int i = 0; i < FUSED_ODOMETRY_VECTOR_SIZE; i++)
	{
		diff = fabs(fused_odometry_vector[i].timestamp - timestamp);

		if (diff < min_diff)
		{
			min_diff = diff;
			min_index = i;
		}
	}

	return min_index;
}


static int
get_base_ackerman_odometry_index_by_timestamp(double timestamp)
{
	double min_diff, diff;
	int min_index;

	min_diff = DBL_MAX;
	min_index = -1;

	for (int i = 0; i < BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE; i++)
	{
		diff = fabs(base_ackerman_odometry_vector[i].timestamp - timestamp);

		if (diff < min_diff)
		{
			min_diff = diff;
			min_index = i;
		}
	}

	return min_index;
}


static void
publish_particles_name(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary,
			char *message_name, double timestamp)
{
	static carmen_localize_ackerman_particle_message pmsg;
	IPC_RETURN_TYPE err;

	pmsg.timestamp = timestamp;
	pmsg.host = carmen_get_host();
	pmsg.globalpos = summary->mean;
	pmsg.globalpos_std = summary->std;
	pmsg.num_particles = filter->param->num_particles;
	pmsg.particles = filter->particles;

	err = IPC_publishData(message_name, &pmsg);
	carmen_test_ipc_exit(err, "Could not publish", message_name);
}

FILE *gnuplot_pipe = NULL;


void
plot_graph(carmen_vector_3D_t *points_position_with_respect_to_car,
		carmen_vector_3D_t *points_position_with_respect_to_car_estimated, int size)
{
	static bool first_time = true;

	if (first_time)
	{
		first_time = false;
		gnuplot_pipe = popen("taskset -c 0 gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [-5:5]\n");
		fprintf(gnuplot_pipe, "set yrange [0:7]\n");
		fprintf(gnuplot_pipe, "set size square\n");
		fprintf(gnuplot_pipe, "set size ratio -1\n");
	}

	FILE *graph_file;

	graph_file = fopen("caco_localize.txt", "w");
	for (int i = 0; i < size; i++)
	{
		fprintf(graph_file, "%lf %lf %lf %lf %d\n",
				points_position_with_respect_to_car[i].x, points_position_with_respect_to_car[i].y,
				points_position_with_respect_to_car_estimated[i].x, points_position_with_respect_to_car_estimated[i].y, i);
	}
	fclose(graph_file);

	fprintf(gnuplot_pipe, "plot 'caco_localize.txt' u 1:2 t 'points', 'caco_localize.txt' u 3:4 t 'estimated'\n");

	fflush(gnuplot_pipe);
}


static carmen_vector_3D_t
get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range,
		rotation_matrix *velodyne_to_board_matrix, rotation_matrix *board_to_car_matrix,
		carmen_vector_3D_t velodyne_pose_position, carmen_vector_3D_t sensor_board_1_pose_position)
{
    double cos_rot_angle = cos(rot_angle);
    double sin_rot_angle = sin(rot_angle);

    double cos_vert_angle = cos(vert_angle);
    double sin_vert_angle = sin(vert_angle);

    double xy_distance = range * cos_vert_angle;

    carmen_vector_3D_t velodyne_reference;

    velodyne_reference.x = (xy_distance * cos_rot_angle);
    velodyne_reference.y = (xy_distance * sin_rot_angle);
    velodyne_reference.z = (range * sin_vert_angle);

    carmen_vector_3D_t board_reference = multiply_matrix_vector(velodyne_to_board_matrix, velodyne_reference);
    board_reference = add_vectors(board_reference, velodyne_pose_position);

    carmen_vector_3D_t car_reference = multiply_matrix_vector(board_to_car_matrix, board_reference);
    car_reference = add_vectors(car_reference, sensor_board_1_pose_position);

    return (car_reference);
}


int
compute_points_with_respect_to_king_pin(carmen_vector_3D_t *points_position_with_respect_to_car,
		carmen_velodyne_partial_scan_message *velodyne_message, double *vertical_correction,
		rotation_matrix *velodyne_to_board_matrix, rotation_matrix *board_to_car_matrix,
		carmen_vector_3D_t velodyne_pose_position, carmen_vector_3D_t sensor_board_1_pose_position)
{
	int j = spherical_sensor_params[0].ray_order[semi_trailer_config.beta_correct_velodyne_ray];	// raio d interesse
	int num_valid_points = 0;
	for (int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		if (velodyne_message->partial_scan[i].distance[j] != 0)
		{
			points_position_with_respect_to_car[num_valid_points] = get_velodyne_point_car_reference(
					-carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle + ((semi_trailer_config.beta_correct_max_distance < 0.0)? 180.0: 0.0)),
					carmen_degrees_to_radians(vertical_correction[j]), (double) velodyne_message->partial_scan[i].distance[j] / 500.0,
					velodyne_to_board_matrix, board_to_car_matrix, velodyne_pose_position, sensor_board_1_pose_position);

			points_position_with_respect_to_car[num_valid_points].x += semi_trailer_config.M;
			num_valid_points++;
		}
	}

	return (num_valid_points);
}


int
compare_x(const void *a, const void *b)
{
	carmen_vector_3D_t *arg1 = (carmen_vector_3D_t *) a;
	carmen_vector_3D_t *arg2 = (carmen_vector_3D_t *) b;

	double delta_x = arg1->x - arg2->x;
	if (delta_x < 0.0)
		return (-1);
	if (delta_x > 0.0)
		return (1);

	return (0);
}

#include <carmen/dbscan.h>


dbscan::Cluster
generate_cluster_with_all_points(carmen_vector_3D_t *points, int size)
{
	dbscan::Cluster cluster;

	for (int i = 0; i < size; i++)
	{
		carmen_point_t point = {points[i].x, points[i].y, 0.0};
		cluster.push_back(point);
	}

	return (cluster);
}


int
remove_small_clusters_of_points(int num_filtered_points, carmen_vector_3D_t *points_position_with_respect_to_car)
{
	dbscan::Cluster single_cluster = generate_cluster_with_all_points(points_position_with_respect_to_car, num_filtered_points);
	dbscan::Clusters clusters = dbscan::dbscan(MIN_DISTANCE_BETWEEN_POINTS, MIN_CLUSTER_SIZE, single_cluster);
	if (clusters.size() > 0)
	{
		int largest_cluster = 0;
		unsigned int largest_cluster_size = clusters[0].size();
		for (unsigned int i = 1; i < clusters.size(); i++)
		{
			if (clusters[i].size() > largest_cluster_size)
			{
				largest_cluster_size = clusters[i].size();
				largest_cluster = i;
			}
		}

//		printf("num_c %d, largest_c %d, lcs %d\n", clusters.size(), largest_cluster, clusters[largest_cluster].size());
		for (unsigned int i = 0; i < clusters[largest_cluster].size(); i++)
		{
			points_position_with_respect_to_car[i].x = clusters[largest_cluster][i].x;
			points_position_with_respect_to_car[i].y = clusters[largest_cluster][i].y;
		}
		num_filtered_points = clusters[largest_cluster].size();
	}
	else
		num_filtered_points = 0;

	return (num_filtered_points);
}


int
compute_points_position_with_respect_to_car(carmen_vector_3D_t *points_position_with_respect_to_car)
{
	int num_valid_points = compute_points_with_respect_to_king_pin(points_position_with_respect_to_car,
			last_velodyne_message, spherical_sensor_params[0].vertical_correction,
			spherical_sensor_params[0].sensor_to_support_matrix, spherical_sensor_params[0].support_to_car_matrix,
			spherical_sensor_params[0].pose.position, spherical_sensor_params[0].sensor_support_pose.position);

	if (num_valid_points == 0)
		return (0);

	int num_filtered_points = 0;
	for (int i = 0; i < num_valid_points; i++)
	{
		double angle = atan2(points_position_with_respect_to_car[i].y, points_position_with_respect_to_car[i].x);
		double distance_to_king_pin = sqrt(DOT2D(points_position_with_respect_to_car[i], points_position_with_respect_to_car[i]));
		if ((angle > (-semi_trailer_config.beta_correct_angle_factor * semi_trailer_config.max_beta)) &&
			(angle < (semi_trailer_config.beta_correct_angle_factor * semi_trailer_config.max_beta)) &&
			(distance_to_king_pin < fabs(semi_trailer_config.beta_correct_max_distance)))
		{
			double x = points_position_with_respect_to_car[i].x * cos(M_PI / 2.0) - points_position_with_respect_to_car[i].y * sin(M_PI / 2.0);
			double y = points_position_with_respect_to_car[i].x * sin(M_PI / 2.0) + points_position_with_respect_to_car[i].y * cos(M_PI / 2.0);
			points_position_with_respect_to_car[num_filtered_points].x = x;
			points_position_with_respect_to_car[num_filtered_points].y = y;
			num_filtered_points++;
		}
	}
	qsort((void *) (points_position_with_respect_to_car), (size_t) num_filtered_points, sizeof(carmen_vector_3D_t), compare_x);

	num_filtered_points = remove_small_clusters_of_points(num_filtered_points, points_position_with_respect_to_car);

	return (num_filtered_points);
}


//double
//compute_new_beta(carmen_vector_3D_t *points_position_with_respect_to_car, int size)
//{
//	// https://www.gnu.org/software/gsl/doc/html/lls.html
//
//	const double *x = (double *) points_position_with_respect_to_car;
//	const double *y = (double *) points_position_with_respect_to_car;
//	y = &(y[1]);
//
//	double c0, c1, cov00, cov01, cov11, sumsq;
//	gsl_fit_linear(x, 3, y, 3, size, &c0, &c1, &cov00, &cov01, &cov11, &sumsq);
//
//	return (-atan(c1));
//}


int
dofit(const gsl_multifit_robust_type *T, const gsl_matrix *X, const gsl_vector *y, gsl_vector *c, gsl_matrix *cov)
{
	int s;
	gsl_multifit_robust_workspace *work = gsl_multifit_robust_alloc(T, X->size1, X->size2);

	s = gsl_multifit_robust(X, y, c, cov, work);
	gsl_multifit_robust_free(work);

	return (s);
}


double
compute_new_beta(carmen_vector_3D_t *points_position_with_respect_to_car,
		carmen_vector_3D_t *points_position_with_respect_to_car_estimated, int size)
{
	size_t n = size;
	const size_t p = 2; /* linear fit */
	gsl_matrix *X, *cov;
	gsl_vector *x, *y, *c, *c_ols;

	gsl_set_error_handler_off(); // We will have to read the status and handle the errors codes

	X = gsl_matrix_alloc(n, p);
	x = gsl_vector_alloc(n);
	y = gsl_vector_alloc(n);

	c = gsl_vector_alloc(p);
	c_ols = gsl_vector_alloc(p);
	cov = gsl_matrix_alloc(p, p);

	for (size_t i = 0; i < n; i++)
	{
		double xi = points_position_with_respect_to_car[i].x;
		double yi = points_position_with_respect_to_car[i].y;

		gsl_vector_set(x, i, xi);
		gsl_vector_set(y, i, yi);
	}

	/* construct design matrix X for linear fit */
	for (size_t i = 0; i < n; ++i)
	{
		double xi = gsl_vector_get(x, i);

		gsl_matrix_set(X, i, 0, 1.0);
		gsl_matrix_set(X, i, 1, xi);
	}

	int status = dofit(gsl_multifit_robust_bisquare, X, y, c, cov);

	if (status)
	{
		if (status != GSL_EMAXITER)
		{
			printf("Failed, gsl_errno = %d. This error isn't handle yet, please check it !!\n", status);
			printf("Discarding beta, using the default prediction one. size n = %d\n", (int) n);

			return (PREDICT_BETA_GSL_ERROR_CODE);
		}
	}

	/* output data and model */
	for (size_t i = 0; i < n; ++i)
	{
		double xi = gsl_vector_get(x, i);
		gsl_vector_view v = gsl_matrix_row(X, i);
		double y_rob, y_err;

		gsl_multifit_robust_est(&v.vector, c, cov, &y_rob, &y_err);
		points_position_with_respect_to_car_estimated[i].x = xi;
		points_position_with_respect_to_car_estimated[i].y = y_rob;
	}

	double estimated_beta = -atan(gsl_vector_get(c, 1));

	gsl_matrix_free(X);
	gsl_vector_free(x);
	gsl_vector_free(y);
	gsl_vector_free(c);
	gsl_vector_free(c_ols);
	gsl_matrix_free(cov);

	return (estimated_beta);
}


double
compute_semi_trailer_beta_using_velodyne(carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point, double dt,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
{
	if (semi_trailer_config.type == 0)
		return (0.0);

	double predicted_beta = compute_semi_trailer_beta(robot_and_trailer_traj_point, dt, robot_config, semi_trailer_config);

	if (!last_velodyne_message ||
		!spherical_sensor_params[0].sensor_to_support_matrix ||
		!spherical_sensor_params[0].support_to_car_matrix)
		return (predicted_beta);

	carmen_vector_3D_t *points_position_with_respect_to_car = (carmen_vector_3D_t *) malloc(last_velodyne_message->number_of_32_laser_shots * sizeof(carmen_vector_3D_t));
	carmen_vector_3D_t *points_position_with_respect_to_car_estimated = (carmen_vector_3D_t *) malloc(last_velodyne_message->number_of_32_laser_shots * sizeof(carmen_vector_3D_t));

	int size = compute_points_position_with_respect_to_car(points_position_with_respect_to_car);
	if (size < MIN_CLUSTER_SIZE)
	{
		free(points_position_with_respect_to_car);
		free(points_position_with_respect_to_car_estimated);

		return (predicted_beta);
	}

	double beta = compute_new_beta(points_position_with_respect_to_car, points_position_with_respect_to_car_estimated, size);

	plot_graph(points_position_with_respect_to_car, points_position_with_respect_to_car_estimated, size);

	free(points_position_with_respect_to_car);
	free(points_position_with_respect_to_car_estimated);

	if (beta == PREDICT_BETA_GSL_ERROR_CODE)
		return (predicted_beta);
	else
		return (carmen_normalize_theta(beta - semi_trailer_config.beta_correct_beta_bias));
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_globalpos(carmen_localize_ackerman_summary_p summary, double v, double phi, double timestamp)
{
	if (!global_localization_requested)
		return;

	if (g_reinitiaze_particles)
	{
		g_reinitiaze_particles--;
//		globalpos.beta = 0.0;
//		return;
	}
		
	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();
	globalpos.globalpos = summary->mean;
	globalpos.globalpos_std = summary->std;
	globalpos.odometrypos = summary->odometry_pos;
	globalpos.globalpos_xy_cov = summary->xy_cov;
	globalpos.v = v;
	globalpos.phi = phi;
	globalpos.converged = summary->converged;


	static double last_timestamp = 0.0;
	if (last_timestamp == 0.0)
		last_timestamp = timestamp;
	if (semi_trailer_config.type > 0)
	{
		globalpos.semi_trailer_engaged = 1;
		carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point =
		{
				globalpos.globalpos.x,
				globalpos.globalpos.y,
				globalpos.globalpos.theta,
				globalpos.beta,
				globalpos.v,
				globalpos.phi
		};
		double delta_t = globalpos.timestamp - last_timestamp;
//		globalpos.beta = compute_semi_trailer_beta(robot_and_trailer_traj_point, delta_t, car_config, semi_trailer_config);
		globalpos.beta = compute_semi_trailer_beta_using_velodyne(robot_and_trailer_traj_point, delta_t, car_config, semi_trailer_config);
	}
	else
	{
		globalpos.semi_trailer_engaged = 0;
		globalpos.beta = 0.0;
	}
	globalpos.semi_trailer_type = semi_trailer_config.type;
	last_timestamp = timestamp;

	if (g_fused_odometry_index == -1)
	{
		globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = globalpos.pose.position.z = 0.0;
		globalpos.velocity.x = globalpos.velocity.y = globalpos.velocity.z = 0.0;
	}
	else
	{	// Aproveita alguns dados da fused_odometry. 
		// Os valores referentes aa globalpos corrente sao escritos abaixo.
		globalpos.pose = fused_odometry_vector[get_fused_odometry_index_by_timestamp(timestamp)].pose; 	
		globalpos.velocity = fused_odometry_vector[get_fused_odometry_index_by_timestamp(timestamp)].velocity; 	
	}
	globalpos.pose.orientation.yaw = globalpos.globalpos.theta;

	globalpos.pose.position.x = globalpos.globalpos.x;
	globalpos.pose.position.y = globalpos.globalpos.y;
	globalpos.pose.position.z = 0;
	globalpos.velocity.x = v;
	
	//globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = 0.0;

	if (save_globalpos_file)
	{
		if (globalpos_file == NULL)
			globalpos_file = fopen(save_globalpos_file, "w");
		if (globalpos_file && (timestamp >= save_globalpos_timestamp))
			fprintf(globalpos_file, "%lf %lf %lf %lf %lf %lf %lf\n",
					globalpos.pose.position.x, globalpos.pose.position.y, globalpos.pose.position.z,
					globalpos.pose.orientation.yaw, v, phi, timestamp);
	}
	carmen_localize_ackerman_publish_globalpos_message(&globalpos);
}


void
publish_particles_prediction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, double timestamp)
{
	publish_particles_name(filter, summary, (char *) CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME, timestamp);
//	FILE *caco = fopen("cacoxx.txt", "a");
//	for (int i = 0; i < filter->param->num_particles; i++)
//		fprintf(caco, "%03d %2.10lf\n", i, filter->particles[i].weight);
//	fprintf(caco, "++++++++++++++++++++++++++++++\n");
//	fclose(caco);

}


void
publish_particles_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, double timestamp)
{
	publish_particles_name(filter, summary, (char *) CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME, timestamp);
}


static void
publish_sensor(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, int num_readings,
		double *range, carmen_laser_laser_config_t laser_config, int front, double timestamp)
{
	static carmen_localize_ackerman_sensor_message sensor;
	IPC_RETURN_TYPE err;

	sensor.timestamp = timestamp;
	sensor.host = carmen_get_host();
	if (front)
	{
		sensor.pose.x = summary->mean.x + filter->param->front_laser_offset * cos(summary->mean.theta);
		sensor.pose.y = summary->mean.y + filter->param->front_laser_offset * sin(summary->mean.theta);
		sensor.pose.theta = summary->mean.theta;
		sensor.num_laser = 1;
	}
	else
	{
		sensor.pose.x = summary->mean.x + filter->param->rear_laser_offset * cos(summary->mean.theta + M_PI);
		sensor.pose.y = summary->mean.y + filter->param->rear_laser_offset * sin(summary->mean.theta + M_PI);
		sensor.pose.theta = summary->mean.theta + M_PI;
		sensor.num_laser = 2;
	}
	sensor.num_readings = num_readings;
	sensor.laser_skip = filter->param->laser_skip;
	sensor.config = laser_config;
	sensor.range = range;
	sensor.mask = filter->laser_mask;
	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, &sensor);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);
}


static void
publish_first_globalpos(carmen_localize_ackerman_initialize_message *initialize_msg)
{
	carmen_localize_ackerman_globalpos_message globalpos_ackerman_message = {};
	globalpos_ackerman_message.globalpos = *initialize_msg->mean;
	globalpos_ackerman_message.globalpos_std = *initialize_msg->std;
	globalpos_ackerman_message.odometrypos = *initialize_msg->std;

	globalpos_ackerman_message.timestamp = initialize_msg->timestamp;
	globalpos_ackerman_message.host = carmen_get_host();

	globalpos_ackerman_message.converged = 0;
	globalpos_ackerman_message.globalpos_xy_cov = 0.0;
	globalpos_ackerman_message.phi = 0.0;
	globalpos_ackerman_message.v = 0.0;

	globalpos_ackerman_message.beta = initialize_msg->beta;
	globalpos.beta = initialize_msg->beta;
	
	globalpos_ackerman_message.semi_trailer_engaged = globalpos.semi_trailer_engaged;
	globalpos_ackerman_message.semi_trailer_type = globalpos.semi_trailer_type;

	carmen_localize_ackerman_publish_globalpos_message(&globalpos_ackerman_message);
}


static void
publish_globalpos_on_mapping_mode(carmen_fused_odometry_message *msg, double timestamp)
{
	if (g_fused_odometry_index != -1)
	{
		IPC_RETURN_TYPE err;
		carmen_pose_3D robot_pose = msg->pose;
		double dt = timestamp - msg->timestamp;
		// TODO @@@ Alberto: se dt for grande nao era o caso de nao publicar?
		robot_pose = carmen_ackerman_interpolated_robot_position_at_time(robot_pose, dt, msg->velocity.x, msg->phi,
				car_config.distance_between_front_and_rear_axles);

		globalpos.timestamp = timestamp;
		globalpos.host = carmen_get_host();
		globalpos.v = msg->velocity.x;
		globalpos.phi = msg->phi;
		globalpos.pose = robot_pose;
		globalpos.velocity = msg->velocity;
		globalpos.globalpos.x = globalpos.pose.position.x;
		globalpos.globalpos.y = globalpos.pose.position.y;
		globalpos.globalpos.theta = globalpos.pose.orientation.yaw;

		if (save_globalpos_file)
		{
			if (globalpos_file == NULL)
				globalpos_file = fopen(save_globalpos_file, "w");
			if (globalpos_file && (timestamp >= save_globalpos_timestamp))
				fprintf(globalpos_file, "%lf %lf %lf %lf %lf %lf %lf\n",
						globalpos.globalpos.x, globalpos.globalpos.y, 0.0,
						globalpos.globalpos.theta, globalpos.v, globalpos.phi, timestamp);
		}

		err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
		carmen_test_ipc_exit(err, "Could not publish",	CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
	}
}


static void
publish_carmen_base_ackerman_odometry()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_base_ackerman_odometry_message base_ackerman_odometry;

	base_ackerman_odometry.x = filter->particles[0].x;
	base_ackerman_odometry.y = filter->particles[0].y;
	base_ackerman_odometry.theta = filter->particles[0].theta;
	base_ackerman_odometry.v = filter->particles[0].v;
	base_ackerman_odometry.phi = filter->particles[0].phi;
	base_ackerman_odometry.timestamp = carmen_get_time();
	base_ackerman_odometry.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &base_ackerman_odometry);
	carmen_test_ipc(err, "Could not publish carmen_base_ackerman_odometry_message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
}
///////////////////////////////////////////////////////////////////////////////////////////////


static void
velodyne_variable_scan_localize(carmen_velodyne_variable_scan_message *message, int sensor)
{	// For cameras and other sensors (not Velodyne)
	int odometry_index, fused_odometry_index;
	int velodyne_initilized;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(message->timestamp);
	fused_odometry_index = get_fused_odometry_index_by_timestamp(message->timestamp);

	if (mapping_mode)
	{
		publish_globalpos_on_mapping_mode(&fused_odometry_vector[fused_odometry_index], message->timestamp);
		return;
	}

	if (!necessary_maps_available || !global_localization_requested ||
		((base_ackerman_odometry_index < 0) && (filter->param->prediction_type != 2)))
		return;

	velodyne_initilized = localize_ackerman_velodyne_variable_scan_build_instanteneous_maps(message, &spherical_sensor_params[sensor], 
			&spherical_sensor_data[sensor], base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi);
	if (!velodyne_initilized)
		return;

	carmen_localize_ackerman_velodyne_prediction(filter, &base_ackerman_odometry_vector[odometry_index],
			xsens_global_quat_message,
			message->timestamp, car_config.distance_between_front_and_rear_axles);

	publish_particles_prediction(filter, &summary, message->timestamp);

	carmen_localize_ackerman_velodyne_correction(filter, &localize_map, &local_compacted_map, &local_compacted_mean_remission_map,
			&local_compacted_variance_remission_map, &binary_map);

	publish_particles_correction(filter, &summary, message->timestamp);

//	if (fabs(base_ackerman_odometry_vector[odometry_index].v) > 0.2)
	{
		carmen_localize_ackerman_velodyne_resample(filter);
	}

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);

		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v,
				base_ackerman_odometry_vector[odometry_index].phi, message->timestamp);

		if ((filter->param->prediction_type == 2) && !robot_publish_odometry)
			publish_carmen_base_ackerman_odometry();
	}

	if (g_reinitiaze_particles)
		carmen_localize_ackerman_initialize_particles_gaussians(filter, 1, &(summary.mean), &g_std);
}


////////////////////////////////////// RANIK ///////////////////////////////////////////////////////

double*
convert_map_to_array(carmen_map_p map)
{
    unsigned int width = map->config.x_size;
    unsigned int height = map->config.y_size;
    unsigned int size = width * height;
    unsigned int row = 0, col = 0, index = 0;
    double *array_map = (double *) malloc (size * sizeof(double));

    for (unsigned int i = 0; i < size; ++i)
    {
        row = (height - 1) - i % height;

        col = i / height;

        index = row * width + col;

        array_map[index] =  1 - map->complete_map[i];

//    	if (map->complete_map[i] != 0.0)
//    		printf("%lf\n", map->complete_map[i]);
    }
    return (array_map);

    exit(0);
}


void
cv_draw_map(carmen_map_p map)
{
	double *inverted_map = convert_map_to_array(map);

	Mat image_localize_map = Mat(map->config.x_size, map->config.y_size, CV_64FC1 , inverted_map, 0);

	imshow("Localize Map", image_localize_map);
	waitKey(1);
	free(inverted_map);
}


void
cv_draw_map_rotated(carmen_map_p map, double x, double y, double rotation_angle)
{
	double *inverted_map = convert_map_to_array(map);

	unsigned int height = map->config.y_size;
	double inverse_resolution = 1.0 / map->config.resolution;

	unsigned int row = height - floor((y - map->config.y_origin) * inverse_resolution + 0.5);
	unsigned int col = floor((x - map->config.x_origin) * inverse_resolution + 0.5);

	Mat image_localize_map = Mat(map->config.x_size, map->config.y_size, CV_64FC1 , inverted_map, 0);

//	//printf("%lf %lf %d %d\n", x, y, (int)x - 125, (int)y - 125);
//	if (row  > (unsigned int)map->config.x_size || col > (unsigned int) map->config.x_size)
//	{
//		printf("Out of map\n");
//		return;
//	}

	Mat roteted_map;
	//Mat rotation_matrix = getRotationMatrix2D(Point(map->config.x_size / 2, map->config.y_size / 2), 90 + rotation_angle, 1.0);
	Mat rotation_matrix = getRotationMatrix2D(Point(col, row), -rotation_angle, 1.0);
	warpAffine(image_localize_map, roteted_map, rotation_matrix, Size(image_localize_map.cols, image_localize_map.rows), INTER_NEAREST);


	//Rect myROI(col - 125, row - 125, 250, 250);
	//image_localize_map = image_localize_map(myROI);
	Rect myROI(col - 150, row - 150, 300, 300);
	roteted_map = roteted_map(myROI);

	//circle(image_localize_map, Point(x, y), 1, cvScalar(0, 0, 255), 25, 8, 0);

//	imshow("Localize Map", roteted_map);
	imshow("Localize Map", image_localize_map);
	waitKey(1);
	free(inverted_map);
}


void
cv_draw_compact_map(carmen_compact_map_t *compact_local_map)
{
	carmen_map_t local_map;
	carmen_grid_mapping_create_new_map(&local_map, compact_local_map->config.x_size, compact_local_map->config.y_size, compact_local_map->config.resolution, 'c');
	memset(local_map.complete_map, 0, local_map.config.x_size * local_map.config.y_size * sizeof(double));
	carmen_prob_models_uncompress_compact_map(&local_map, compact_local_map);

	double *inverted_map = convert_map_to_array(&local_map);

	Mat image_local_map = Mat(local_map.config.x_size, local_map.config.y_size, CV_64F , inverted_map, 0);

//	Rect myROI((local_map.config.x_size / 2) - 150, (local_map.config.y_size / 2) - 150, 300, 300);
//	image_local_map = image_local_map(myROI);

	imshow("Local Map", image_local_map);
	waitKey(1);

	free (local_map.complete_map);
	free (local_map.map);
	free(inverted_map);
}

double*
get_unknown_region(carmen_map_p map)
{
    unsigned int width = map->config.x_size;
    unsigned int height = map->config.y_size;
    unsigned int size = width * height;
    unsigned int row = 0, col = 0, index = 0;
    double *array_map = (double *) malloc (size * sizeof(double));

    for (unsigned int i = 0; i < size; ++i)
    {
        row = (height - 1) - i % height;

        col = i / height;

        index = row * width + col;

        if (map->complete_map[i] < 0.0)
        	array_map[index] =  0.0;
        else
        	array_map[index] =  1.0;

//    	if (map->complete_map[i] != 0.0)
//    		printf("%lf\n", map->complete_map[i]);
    }
    return (array_map);

    exit(0);
}


void
cv_draw_map_unknown_region(carmen_map_p map, double x, double y, double rotation_angle)
{
	double *inverted_map = get_unknown_region(map);

	unsigned int height = map->config.y_size;
	double inverse_resolution = 1.0 / map->config.resolution;

	unsigned int row = height - floor((y - map->config.y_origin) * inverse_resolution + 0.5);
	unsigned int col = floor((x - map->config.x_origin) * inverse_resolution + 0.5);

	Mat image_localize_map = Mat(map->config.x_size, map->config.y_size, CV_64FC1 , inverted_map, 0);

//	printf("%lf %lf %d %d\n", x, y, (int)x - 125, (int)y - 125);
	if (row  > (unsigned int)map->config.x_size || col > (unsigned int) map->config.x_size)
	{
		printf("Out of map\n");
		return;
	}

	Mat roteted_map;
	//Mat rotation_matrix = getRotationMatrix2D(Point(map->config.x_size / 2, map->config.y_size / 2), 90 + rotation_angle, 1.0);
	Mat rotation_matrix = getRotationMatrix2D(Point(col, row), -rotation_angle, 1.0);
	warpAffine(image_localize_map, roteted_map, rotation_matrix, Size(image_localize_map.cols, image_localize_map.rows), INTER_NEAREST);


	//Rect myROI(col - 125, row - 125, 250, 250);
	//image_localize_map = image_localize_map(myROI);
	Rect myROI(col - 150, row - 150, 300, 300);
	roteted_map = roteted_map(myROI);

	//circle(image_localize_map, Point(x, y), 1, cvScalar(0, 0, 255), 25, 8, 0);

	imshow("Unknown Map", roteted_map);
	waitKey(1);
	free(inverted_map);
}


void
display_maps()
{
	cv_draw_map(&localize_map.carmen_map);
	cv_draw_compact_map(&local_compacted_map);
	//cv_draw_map_unknown_region(&localize_map.carmen_map, summary.mean.x, summary.mean.y, carmen_radians_to_degrees(summary.mean.theta));
}
///////////////////////////// END RANIK ///////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{	// Used by Velodyne.
	int odometry_index, fused_odometry_index;
	int velodyne_initilized;

	last_velodyne_message = velodyne_message;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(velodyne_message->timestamp);
	fused_odometry_index = get_fused_odometry_index_by_timestamp(velodyne_message->timestamp);

	if (mapping_mode)
	{
		publish_globalpos_on_mapping_mode(&fused_odometry_vector[fused_odometry_index], velodyne_message->timestamp);
		return;
	}

	if (!necessary_maps_available || !global_localization_requested ||
		((base_ackerman_odometry_index < 0) && (filter->param->prediction_type != 2)))
		return;

	carmen_current_semi_trailer_data_t semi_trailer_data =
	{
			globalpos.semi_trailer_engaged,
			globalpos.semi_trailer_type,
			semi_trailer_config.d,
			semi_trailer_config.M,
			globalpos.beta
	};

	velodyne_initilized = localize_ackerman_velodyne_partial_scan_build_instanteneous_maps(velodyne_message, &spherical_sensor_params[0], 
			&spherical_sensor_data[0], base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi, semi_trailer_data);
	if (!velodyne_initilized)
		return;

	carmen_localize_ackerman_velodyne_prediction(filter,
			&base_ackerman_odometry_vector[odometry_index], xsens_global_quat_message,
			velodyne_message->timestamp, car_config.distance_between_front_and_rear_axles);

//	carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point =
//	{
//			globalpos.globalpos.x,
//			globalpos.globalpos.y,
//			globalpos.globalpos.theta,
//			globalpos.beta,
//			globalpos.v,
//			globalpos.phi
//	};
//
//	carmen_localize_ackerman_beta_prediction(beta_filter, robot_and_trailer_traj_point, car_config, semi_trailer_config, velodyne_message->timestamp - beta_filter->last_timestamp);

	publish_particles_prediction(filter, &summary, velodyne_message->timestamp);

	carmen_localize_ackerman_velodyne_correction(filter,
			&localize_map, &local_compacted_map, &local_compacted_mean_remission_map, &local_compacted_variance_remission_map, &binary_map);

//	carmen_localize_ackerman_beta_correction(beta_filter,
//			&localize_map, &local_compacted_map, &local_compacted_mean_remission_map, &local_compacted_variance_remission_map, &binary_map);

	publish_particles_correction(filter, &summary, velodyne_message->timestamp);

	if (filter->initialized)
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);

//	if (beta_filter->initialized)
//		carmen_localize_ackerman_summarize_beta(filter, &summary);

	// if (fabs(base_ackerman_odometry_vector[odometry_index].v) > 0.2)
	{
		carmen_localize_ackerman_velodyne_resample(filter);

//		carmen_localize_ackerman_beta_resample(beta_filter);
	}

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);
		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi,
				velodyne_message->timestamp);

		if ((filter->param->prediction_type == 2) && !robot_publish_odometry)
			publish_carmen_base_ackerman_odometry();

//		static bool ft = true;
//		static double init_t = 0.0;
//		if (ft)
//		{
//			init_t = globalpos.timestamp;
//			ft = false;
//		}
//
//		FILE *caco = fopen("caco_gpos.txt", "a");
//		fprintf(caco, "%lf %lf %lf %lf %lf\n", globalpos.timestamp - init_t, velodyne_message->timestamp - init_t,
//				base_ackerman_odometry_vector[odometry_index].timestamp - init_t,
//				base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi);
//		fflush(caco);
//		fclose(caco);
	}

	if (g_reinitiaze_particles)
		carmen_localize_ackerman_initialize_particles_gaussians(filter, 1, &(summary.mean), &g_std);

	//display_maps();             // Ranik Display a crop of local and offline maps
}


static void
velodyne_variable_scan_message_handler1(carmen_velodyne_variable_scan_message *message)
{	// Variable scan because you can have more or less than 32 vertical rays. Used by stereo cameras or kinect.
	velodyne_variable_scan_localize(message, 1);
}


static void
velodyne_variable_scan_message_handler2(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 2);
}


static void
velodyne_variable_scan_message_handler3(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 3);
}


static void
velodyne_variable_scan_message_handler4(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 4);
}


static void
velodyne_variable_scan_message_handler5(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 5);
}


static void
velodyne_variable_scan_message_handler6(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 6);
}


static void
velodyne_variable_scan_message_handler7(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 7);
}


static void
velodyne_variable_scan_message_handler8(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 8);
}


static void
velodyne_variable_scan_message_handler9(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 9);
}


static void
localize_using_lidar(int sensor_number, carmen_velodyne_variable_scan_message *msg)
{
	int odometry_index, fused_odometry_index, instanteneous_maps_ok = 0;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(msg->timestamp);
	fused_odometry_index = get_fused_odometry_index_by_timestamp(msg->timestamp);

	if (mapping_mode)
	{
		publish_globalpos_on_mapping_mode(&fused_odometry_vector[fused_odometry_index], msg->timestamp);
		return;
	}

	if (!necessary_maps_available || !global_localization_requested || ((base_ackerman_odometry_index < 0) && (filter->param->prediction_type != 2)))
		return;

	carmen_current_semi_trailer_data_t semi_trailer_data =
	{
			globalpos.semi_trailer_engaged,
			globalpos.semi_trailer_type,
			semi_trailer_config.d,
			semi_trailer_config.M,
			globalpos.beta
	};

	instanteneous_maps_ok = localize_ackerman_variable_scan_build_instanteneous_maps(msg, &spherical_sensor_params[sensor_number], 
			&spherical_sensor_data[sensor_number], base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi, semi_trailer_data);
	
	if (!instanteneous_maps_ok)
		return;

	// TUDO the filter should be one for each lidar?
	carmen_localize_ackerman_velodyne_prediction(filter, &base_ackerman_odometry_vector[odometry_index], xsens_global_quat_message,
			msg->timestamp, car_config.distance_between_front_and_rear_axles);

	publish_particles_prediction(filter, &summary, msg->timestamp);

	carmen_localize_ackerman_velodyne_correction(filter, &localize_map, &local_compacted_map, &local_compacted_mean_remission_map,
			&local_compacted_variance_remission_map, &binary_map);

	publish_particles_correction(filter, &summary, msg->timestamp);

	if (filter->initialized)
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);

	// if (fabs(base_ackerman_odometry_vector[odometry_index].v) > 0.2)
	{
		carmen_localize_ackerman_velodyne_resample(filter);
	}

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);
		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi,	msg->timestamp);

		if ((filter->param->prediction_type == 2) && !robot_publish_odometry)
			publish_carmen_base_ackerman_odometry();
	}

	if (g_reinitiaze_particles)
		carmen_localize_ackerman_initialize_particles_gaussians(filter, 1, &(summary.mean), &g_std);
}

void
variable_scan_message_handler_0(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(10, message);
}

void
variable_scan_message_handler_1(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(11, message);
}

void
variable_scan_message_handler_2(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(12, message);
}

void
variable_scan_message_handler_3(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(13, message);
}

void
variable_scan_message_handler_4(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(14, message);
}

void
variable_scan_message_handler_5(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(15, message);
}

void
variable_scan_message_handler_6(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(16, message);
}

void
variable_scan_message_handler_7(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(17, message);
}

void
variable_scan_message_handler_8(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(18, message);
}

void
variable_scan_message_handler_9(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(19, message);
}

void
variable_scan_message_handler_10(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(20, message);
}

void
variable_scan_message_handler_11(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(21, message);
}

void
variable_scan_message_handler_12(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(22, message);
}

void
variable_scan_message_handler_13(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(23, message);
}

void
variable_scan_message_handler_14(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(24, message);
}

void
variable_scan_message_handler_15(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(25, message);
}


static void
robot_ackerman_frontlaser_handler(carmen_robot_ackerman_laser_message *flaser)
{
	if (!necessary_maps_available)
		return;

	carmen_localize_ackerman_run(filter, &localize_map, flaser, filter->param->front_laser_offset, 0,
			&base_ackerman_odometry_vector[base_ackerman_odometry_index], car_config.distance_between_front_and_rear_axles);

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize(filter, &summary, &localize_map, flaser->num_readings,
				flaser->range, filter->param->front_laser_offset,
				flaser->config.angular_resolution,
				flaser->config.start_angle, 0);
		publish_globalpos(&summary, flaser->v, flaser->phi, flaser->timestamp);
		publish_particles_correction(filter, &summary, flaser->timestamp);
		publish_sensor(filter, &summary, flaser->num_readings, flaser->range, flaser->config, 1, flaser->timestamp);
	}
}


static void
raw_laser_handler(carmen_laser_laser_message *laser)
{
	int odometry_index;

	if (!necessary_maps_available || base_ackerman_odometry_index < 0)
		return;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(laser->timestamp);

	carmen_localize_ackerman_run_with_raw_laser(filter, &localize_map,
			laser, &base_ackerman_odometry_vector[odometry_index],
			filter->param->front_laser_offset, car_config.distance_between_front_and_rear_axles);

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize(filter,
				&summary, &localize_map, laser->num_readings,
				laser->range, filter->param->front_laser_offset,
				laser->config.angular_resolution,
				laser->config.start_angle, 0);
		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi,
				laser->timestamp);
		publish_particles_correction(filter, &summary, laser->timestamp);
		publish_sensor(filter, &summary, laser->num_readings, laser->range, laser->config, 1, laser->timestamp);
	}
}


static void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	base_ackerman_odometry_index = (base_ackerman_odometry_index + 1) % BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE;
	base_ackerman_odometry_vector[base_ackerman_odometry_index] = *msg;

//	localize_using_map_set_robot_pose_into_the_map(msg->v, msg->phi, msg->timestamp);
}


static void
fused_odometry_handler(carmen_fused_odometry_message *msg)
{
	static int is_first_fused_odometry_message = 1;

	if (is_first_fused_odometry_message)
	{
		is_first_fused_odometry_message = 0;
		return;
	}

	g_fused_odometry_index = (g_fused_odometry_index + 1) % FUSED_ODOMETRY_VECTOR_SIZE;
	fused_odometry_vector[g_fused_odometry_index] = *msg;
}


static void
carmen_xsens_subscribe_xsens_global_quat_message_handler(carmen_xsens_global_quat_message *message)
{
	xsens_global_quat_message = message;
}


static void
carmen_localize_ackerman_initialize_message_handler(carmen_localize_ackerman_initialize_message *initialize_msg)
{
	if (initialize_msg->distribution == CARMEN_INITIALIZE_GAUSSIAN)
	{
		carmen_localize_ackerman_initialize_particles_gaussians(filter,
			initialize_msg->num_modes, initialize_msg->mean, initialize_msg->std);

		g_std = initialize_msg->std[0];
//		g_std = {0.0, 0.0, 0.0};
		g_reinitiaze_particles = 10;

		filter->last_timestamp = initialize_msg->timestamp;

		publish_first_globalpos(initialize_msg); // Alberto: se publicar pode sujar o mapa devido a inicializacao.
	}
	else if (initialize_msg->distribution == CARMEN_INITIALIZE_UNIFORM)
	{
		//todo pode dar problema aqui se o mapa nao estiver inicializado
		carmen_localize_ackerman_initialize_particles_uniform(filter, &front_laser, &localize_map);
		publish_particles_correction(filter, &summary, initialize_msg->timestamp);
	}

	global_localization_requested = true;
}


static void
localize_map_update_handler(carmen_map_server_localize_map_message *message)
{
	carmen_map_server_localize_map_message_to_localize_map(message, &localize_map);

//	x_origin = message->config.x_origin;
//	y_origin = message->config.y_origin;

	necessary_maps_available = 1;
}


static void
globalpos_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_globalpos_message globalpos;
	//  FORMATTER_PTR formatter;

	if (!necessary_maps_available)
		return;

	/* formatter = */IPC_msgInstanceFormatter(msgRef);
	IPC_freeByteArray(callData);

	globalpos.timestamp = carmen_get_time();
	globalpos.host = carmen_get_host();
	globalpos.globalpos = summary.mean;
	globalpos.globalpos_std = summary.std;
	globalpos.globalpos_xy_cov = summary.xy_cov;
	globalpos.odometrypos = summary.odometry_pos;
	globalpos.converged = summary.converged;

	err = IPC_respondData(msgRef, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


static void 
map_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_map_query_message msg;
	carmen_localize_ackerman_map_message response;

#ifndef NO_ZLIB
	unsigned long compress_buf_size;
	int compress_return;
	unsigned char *compressed_map;
#endif

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &msg,
			sizeof(carmen_localize_ackerman_map_query_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall",
			IPC_msgInstanceName(msgRef));

	response.config = localize_map.config;

	if (msg.map_is_global_likelihood)
	{
		response.map_is_global_likelihood = 1;
		response.data = (unsigned char *) localize_map.complete_gprob;
		response.size = localize_map.config.x_size * localize_map.config.y_size * sizeof(double);
	}
	else
	{
		response.map_is_global_likelihood = 0;
		response.data = (unsigned char *) localize_map.complete_prob;
		response.size = localize_map.config.x_size * localize_map.config.y_size * sizeof(double);
	}

#ifndef NO_ZLIB
	compress_buf_size = response.size * 1.01 + 12;
	compressed_map = (unsigned char *) calloc(compress_buf_size, sizeof(unsigned char));
	carmen_test_alloc(compressed_map);
	compress_return = carmen_compress(
			(unsigned char *)compressed_map,
			(unsigned long *)&compress_buf_size,
			(unsigned char *)response.data,
			(unsigned long)response.size,
			Z_DEFAULT_COMPRESSION);
	if (compress_return != Z_OK) {
		free(compressed_map);
		response.compressed = 0;
	} else {
		response.size = compress_buf_size;
		response.data = compressed_map;
		response.compressed = 1;
	}
#else
	response.compressed = 0;
#endif

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_LOCALIZE_ACKERMAN_MAP_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_LOCALIZE_ACKERMAN_MAP_NAME);
}


static void
carmen_task_manager_set_semi_trailer_type_and_beta_message_handler(carmen_task_manager_set_semi_trailer_type_and_beta_message *message)
{
	if (semi_trailer_config.type != message->semi_trailer_type)
	{
		char *fake_module_name = (char *) "carmen_task_manager_set_semi_trailer_type_and_beta_message_handler()";
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, 1, &fake_module_name, message->semi_trailer_type);
		globalpos.beta = message->beta;
	}
}


static void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *msg)
{
	behavior_selector_path_goals_and_annotations_message = msg;
}


static void
shutdown_localize(int x)
{
	if (x == SIGINT)
	{
		if (globalpos_file)
			fclose(globalpos_file);

		carmen_verbose("Disconnecting from IPC network.\n");
		exit(1);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


int 
define_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);

	/* register map request message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_MAP_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_NAME);

	/* register globalpos request message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, IPC_VARIABLE_LENGTH, CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);

	carmen_velodyne_define_messages();

	return 0;
}


static void
subscribe_to_ipc_message()
{
	IPC_RETURN_TYPE err;

	carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) carmen_localize_ackerman_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler,	CARMEN_SUBSCRIBE_LATEST);

	if (!mapping_mode)
	{
		if (use_raw_laser)
		{
			carmen_laser_subscribe_laser1_message(NULL, (carmen_handler_t) raw_laser_handler, CARMEN_SUBSCRIBE_LATEST);
			carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		else
		{
			carmen_robot_ackerman_subscribe_frontlaser_message(&front_laser, (carmen_handler_t) robot_ackerman_frontlaser_handler, CARMEN_SUBSCRIBE_LATEST);
		}

		carmen_map_server_subscribe_localize_map_message(NULL, (carmen_handler_t) localize_map_update_handler, CARMEN_SUBSCRIBE_LATEST);

		/* subscribe to map request message */
		// TODO: create proper subscribe messages. Check with Alberto if there is a reason for doing these subscribes with the low level API.
		err = IPC_subscribe(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, map_query_handler, NULL);
		carmen_test_ipc(err, "Could not subscribe", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);
		IPC_setMsgQueueLength(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, 1);

		err = IPC_subscribe(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, globalpos_query_handler, NULL);
		carmen_test_ipc(err, "Could not subscribe", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME);
		IPC_setMsgQueueLength(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, 1);

		// stereo velodyne (cameras stereo)
		if ((number_of_sensors > 0) && spherical_sensor_params[0].alive)
			carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 1) && spherical_sensor_params[1].alive)
			carmen_stereo_velodyne_subscribe_scan_message(1, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler1, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 2) && spherical_sensor_params[2].alive)
			carmen_stereo_velodyne_subscribe_scan_message(2, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler2, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 3) && spherical_sensor_params[3].alive)
			carmen_stereo_velodyne_subscribe_scan_message(3, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler3, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 4) && spherical_sensor_params[4].alive)
			carmen_stereo_velodyne_subscribe_scan_message(4, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler4, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 5) && spherical_sensor_params[5].alive)
			carmen_stereo_velodyne_subscribe_scan_message(5, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler5, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 6) && spherical_sensor_params[6].alive)
			carmen_stereo_velodyne_subscribe_scan_message(6, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler6, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 7) && spherical_sensor_params[7].alive)
			carmen_stereo_velodyne_subscribe_scan_message(7, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler7, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 8) && spherical_sensor_params[8].alive)
			carmen_stereo_velodyne_subscribe_scan_message(8, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler8, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 9) && spherical_sensor_params[9].alive)
			carmen_stereo_velodyne_subscribe_scan_message(9, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler9, CARMEN_SUBSCRIBE_LATEST);

		// lidars
		if ((number_of_sensors > 10) && spherical_sensor_params[10].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_0, CARMEN_SUBSCRIBE_LATEST, 0);

		if ((number_of_sensors > 11) && spherical_sensor_params[11].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_1, CARMEN_SUBSCRIBE_LATEST, 1);

		if ((number_of_sensors > 12) && spherical_sensor_params[12].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_2, CARMEN_SUBSCRIBE_LATEST, 2);

		if ((number_of_sensors > 13) && spherical_sensor_params[13].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_3, CARMEN_SUBSCRIBE_LATEST, 3);

		if ((number_of_sensors > 14) && spherical_sensor_params[14].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_4, CARMEN_SUBSCRIBE_LATEST, 4);

		if ((number_of_sensors > 15) && spherical_sensor_params[15].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_5, CARMEN_SUBSCRIBE_LATEST, 5);

		if ((number_of_sensors > 16) && spherical_sensor_params[16].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_6, CARMEN_SUBSCRIBE_LATEST, 6);

		if ((number_of_sensors > 17) && spherical_sensor_params[17].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_7, CARMEN_SUBSCRIBE_LATEST, 7);

		if ((number_of_sensors > 18) && spherical_sensor_params[18].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_8, CARMEN_SUBSCRIBE_LATEST, 8);

		if ((number_of_sensors > 19) && spherical_sensor_params[19].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_9, CARMEN_SUBSCRIBE_LATEST, 9);

		if ((number_of_sensors > 20) && spherical_sensor_params[20].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_10, CARMEN_SUBSCRIBE_LATEST, 10);

		if ((number_of_sensors > 21) && spherical_sensor_params[21].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_11, CARMEN_SUBSCRIBE_LATEST, 11);

		if ((number_of_sensors > 22) && spherical_sensor_params[22].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_12, CARMEN_SUBSCRIBE_LATEST, 12);

		if ((number_of_sensors > 23) && spherical_sensor_params[23].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_13, CARMEN_SUBSCRIBE_LATEST, 13);

		if ((number_of_sensors > 24) && spherical_sensor_params[24].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_14, CARMEN_SUBSCRIBE_LATEST, 14);

		if ((number_of_sensors > 25) && spherical_sensor_params[25].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_15, CARMEN_SUBSCRIBE_LATEST, 15);

		// IMU
		if (filter->param->prediction_type == 2) // use IMU based prediction
			carmen_xsens_subscribe_xsens_global_quat_message(NULL, (carmen_handler_t) carmen_xsens_subscribe_xsens_global_quat_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		if ((number_of_sensors > 0) && spherical_sensor_params[0].alive)
			carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

		// stereo velodyne camera 3
		if ((number_of_sensors > 3) && spherical_sensor_params[3].alive)
			carmen_stereo_velodyne_subscribe_scan_message(3, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler3, CARMEN_SUBSCRIBE_LATEST);

		// lidar0
		if ((number_of_sensors > 10) && spherical_sensor_params[10].alive)
			carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_0, CARMEN_SUBSCRIBE_LATEST, 0);
	}

	carmen_task_manager_subscribe_set_semi_trailer_type_and_beta_message(NULL, (carmen_handler_t) carmen_task_manager_set_semi_trailer_type_and_beta_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(NULL, (carmen_handler_t) path_goals_and_annotations_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
init_local_maps(ProbabilisticMapParams map_params)
{
	init_probabilistic_grid_map_model(&map_params, NULL);

	init_carmen_map(&map_params, &local_map);
	init_carmen_map(&map_params, &local_mean_remission_map);
	init_carmen_map(&map_params, &local_sum_remission_map);
	init_carmen_map(&map_params, &local_sum_sqr_remission_map);
	init_carmen_map(&map_params, &local_count_remission_map);
	init_carmen_map(&map_params, &local_variance_remission_map);

	local_compacted_map.coord_x = NULL;
	local_compacted_map.coord_y = NULL;
	local_compacted_map.value = NULL;
	local_compacted_map.config.map_name = NULL;
	local_compacted_map.number_of_known_points_on_the_map = 0;

	local_compacted_mean_remission_map.coord_x = NULL;
	local_compacted_mean_remission_map.coord_y = NULL;
	local_compacted_mean_remission_map.value = NULL;
	local_compacted_mean_remission_map.config.map_name = NULL;
	local_compacted_mean_remission_map.number_of_known_points_on_the_map = 0;

	local_compacted_variance_remission_map.coord_x = NULL;
	local_compacted_variance_remission_map.coord_y = NULL;
	local_compacted_variance_remission_map.value = NULL;
	local_compacted_variance_remission_map.config.map_name = NULL;
	local_compacted_variance_remission_map.number_of_known_points_on_the_map = 0;

	memset(&binary_map, 0, sizeof(carmen_localize_ackerman_binary_map_t));
}


static void
init_localize_map()
{
	localize_map.carmen_map.complete_map = NULL;
	localize_map.complete_distance = NULL;
	localize_map.complete_gprob = NULL;
	localize_map.complete_prob = NULL;
	localize_map.complete_x_offset = NULL;
	localize_map.complete_y_offset = NULL;

	localize_map.carmen_map.map = NULL;
	localize_map.distance = NULL;
	localize_map.gprob = NULL;
	localize_map.prob = NULL;
	localize_map.x_offset = NULL;
	localize_map.y_offset = NULL;
}


void
nonblock(int state)
{
#define NB_ENABLE 		1
#define NB_DISABLE 		0

	struct termios ttystate;

	//get the terminal state
	tcgetattr(STDIN_FILENO, &ttystate);

	if (state == NB_ENABLE)
	{
		//turn off canonical mode
		ttystate.c_lflag &= ~ICANON;
		//minimum of number input read.
		ttystate.c_cc[VMIN] = 1;
	}
	else if (state == NB_DISABLE)
	{
		//turn on canonical mode
		ttystate.c_lflag |= ICANON;
	}
	//set the terminal attributes.
	tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

}


int
kbhit()
{
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
	return FD_ISSET(STDIN_FILENO, &fds);
}


void
timer_handler()
{
	if (kbhit() != 0)
	{
        char c = fgetc(stdin);

        switch (c)
        {
        case '1':
           	semi_trailer_config.beta_correct_velodyne_ray = 0;
            break;

        case '2':
            // increase Velodyne single ray
        	semi_trailer_config.beta_correct_velodyne_ray++;
            if (semi_trailer_config.beta_correct_velodyne_ray > 31)
            	semi_trailer_config.beta_correct_velodyne_ray = 0;
            if (semi_trailer_config.beta_correct_velodyne_ray < 0)
            	semi_trailer_config.beta_correct_velodyne_ray = 31;
            break;
        case '3':
            // decrease Velodyne single ray
        	semi_trailer_config.beta_correct_velodyne_ray--;
            if (semi_trailer_config.beta_correct_velodyne_ray > 31)
            	semi_trailer_config.beta_correct_velodyne_ray = 0;
            if (semi_trailer_config.beta_correct_velodyne_ray < 0)
            	semi_trailer_config.beta_correct_velodyne_ray = 31;
            break;
        }
        printf("semi_trailer_config.beta_correct_velodyne_ray %d\n", semi_trailer_config.beta_correct_velodyne_ray);
	}
}


int 
main(int argc, char **argv) 
{ 
	carmen_localize_ackerman_param_t param;
	ProbabilisticMapParams map_params;

	/* initialize carmen */
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	/* Setup exit handler */
	signal(SIGINT, shutdown_localize);

	/* Initialize all the relevant parameters */
	carmen_localize_ackerman_read_parameters(argc, argv, &param, &map_params);

#ifndef OLD_MOTION_MODEL
	param.motion_model = carmen_localize_ackerman_motion_initialize(argc, argv);
#endif

	/* Allocate memory for the particle filter */
	filter = carmen_localize_ackerman_particle_filter_initialize(&param);
//	beta_filter = carmen_localize_ackerman_particle_filter_initialize(&param);

	init_localize_map();
	init_local_maps(map_params);

	define_ipc_messages();
	subscribe_to_ipc_message();

	nonblock(NB_ENABLE);
//	double timer_period = 0.1;
//	carmen_ipc_addPeriodicTimer(timer_period, (TIMER_HANDLER_TYPE) timer_handler, NULL);
	carmen_ipc_dispatch();

	nonblock(NB_DISABLE);

	carmen_ipc_dispatch();

	return (0);
}
