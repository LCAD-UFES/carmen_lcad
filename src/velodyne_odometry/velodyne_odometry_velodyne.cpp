#include <carmen/carmen.h>
#include <prob_map.h>
#include "velodyne_odometry_velodyne.h"

typedef struct
{
	double max_range;
	double variance;
	double prob_of_random_reading;
	double prob_of_random_max;
	int num_lasers;
	double offset;
	double side_offset;
	double angular_offset;
	double angular_resolution;
	double fov;
	double start_angle;
	int id;
} localize_ackerman_velodyne_laser_config_t;


extern carmen_pose_3D_t sensor_board_1_pose;
extern rotation_matrix *sensor_board_1_to_car_matrix;

extern double robot_wheel_radius;
extern carmen_robot_ackerman_config_t car_config;

static localize_ackerman_velodyne_laser_config_t front_laser_config;
static carmen_laser_laser_message flaser_message;


static void
localize_ackerman_velodyne_laser_initialize()
{
	flaser_message.host = carmen_get_host();
	flaser_message.num_readings = front_laser_config.num_lasers;
	flaser_message.range = (double *) calloc(front_laser_config.num_lasers, sizeof(double));
	carmen_test_alloc(flaser_message.range);

	flaser_message.num_remissions = 0;
	flaser_message.remission = 0;

	flaser_message.id = front_laser_config.id;
	flaser_message.num_readings = front_laser_config.num_lasers;

	flaser_message.config.maximum_range       = front_laser_config.max_range;
	flaser_message.config.fov                 = front_laser_config.fov;
	flaser_message.config.start_angle         = front_laser_config.start_angle;
	flaser_message.config.angular_resolution  = front_laser_config.angular_resolution;

	//this was placed here because compiling with the old motion model
	//did't work, check this if this breaks something
	flaser_message.config.remission_mode      = REMISSION_NONE;
}


static void
fill_laser_config_data(localize_ackerman_velodyne_laser_config_t *lasercfg)
{
	lasercfg->num_lasers = 1 + carmen_round(lasercfg->fov / lasercfg->angular_resolution);
	lasercfg->start_angle = -0.5 * lasercfg->fov;

	/* give a warning if it is not a standard configuration */

	if (fabs(lasercfg->fov - M_PI) > 1e-6 &&
			fabs(lasercfg->fov - 100.0/180.0 * M_PI) > 1e-6 &&
			fabs(lasercfg->fov -  90.0/180.0 * M_PI) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (fov=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->fov));

	if (fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(1.0)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.5)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.25)) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (res=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->angular_resolution));

}


void
localize_ackerman_velodyne_laser_read_parameters(int argc, char **argv)
{
	static char frontlaser_fov_string[256];
	static char frontlaser_res_string[256];

	carmen_param_t param_list[] =
	{
		{(char *) "robot", (char *) "frontlaser_id", CARMEN_PARAM_INT, &(front_laser_config.id), 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	sprintf(frontlaser_fov_string, (char*)"laser%d_fov", front_laser_config.id);
	sprintf(frontlaser_res_string, (char*)"laser%d_resolution", front_laser_config.id);

	carmen_param_t param_list_front_laser[] =
	{
		{(char *) "simulator", (char *) "frontlaser_maxrange", CARMEN_PARAM_DOUBLE, &(front_laser_config.max_range), 1, NULL},
		{(char *) "robot", (char *) "frontlaser_offset", CARMEN_PARAM_DOUBLE, &(front_laser_config.offset), 1, NULL},
		{(char *) "robot", (char *) "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, &(front_laser_config.side_offset), 1, NULL},
		{(char *) "robot", (char *) "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(front_laser_config.angular_offset), 1, NULL},
		{(char *) "laser", frontlaser_fov_string, CARMEN_PARAM_DOUBLE, &(front_laser_config.fov), 0, NULL},
		{(char *) "laser", frontlaser_res_string, CARMEN_PARAM_DOUBLE, &(front_laser_config.angular_resolution), 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list_front_laser, sizeof(param_list_front_laser) / sizeof(param_list_front_laser[0]));

	front_laser_config.angular_resolution =	carmen_degrees_to_radians(front_laser_config.angular_resolution);

	front_laser_config.fov = carmen_degrees_to_radians(front_laser_config.fov);

	fill_laser_config_data(&front_laser_config);

	localize_ackerman_velodyne_laser_initialize();
}


static void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points, int max_point_buffer,
		int use_remission)
{
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points && use_remission)
		intensity[*point_cloud_index] = (unsigned char *) realloc((void *) intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


static void
compute_laser_rays_targets(sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data,
		rotation_matrix *r_matrix_car_to_global, carmen_pose_3D_t *robot_pose, double x_origin, double y_origin, int point_cloud_index,
		double v, double phi, int use_remission)
{
	spherical_point_cloud v_zt = velodyne_data->points[point_cloud_index];
	int N = v_zt.num_points / velodyne_params->vertical_resolution;

	double dt = velodyne_params->time_spent_by_each_scan;
	double dt1 = -(double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = *robot_pose;

	int jump = 1;
	for (int j = 0; j < N; j += jump)
	{
		double dt2 = j * dt;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(*robot_pose, dt1 + dt2, v, phi,
				car_config.distance_between_front_and_rear_axles);
		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(velodyne_data, velodyne_params,
				j * velodyne_params->vertical_resolution, robot_interpolated_position.position,
				sensor_board_1_pose, r_matrix_car_to_global, sensor_board_1_to_car_matrix,
				robot_wheel_radius, x_origin, y_origin, &car_config, 0, 0, use_remission);
	}
}


void
pid_plot_velocity(double *intensity, double *angle, double *previous_intensity, double *previous_angle, int size)
{
	static bool first_time = true;
	static FILE *gnuplot_pipe;

	if (first_time)
	{
		first_time = false;
		gnuplot_pipe = popen("gnuplot", "w"); //("gnuplot -persist", "w") to keep last plot after program closes
		fprintf(gnuplot_pipe, "set yrange [0:1]\n");
	}

	FILE *gnuplot_data_file = fopen("gnuplot_velocity_data.txt", "w");

	for (int i = 0; i < size; i++)
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf\n", intensity[i], angle[i], previous_intensity[i], previous_angle[i]);

	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_velocity_data.txt' using 2:1 with lines title '%s', "
			"'./gnuplot_velocity_data.txt' using 4:3 with lines title '%s'\n",
			"intensity", "previous_intensity");

	fflush(gnuplot_pipe);
}


static void
compute_v_and_phi(sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data)
{
	int current_point_cloud = velodyne_data->point_cloud_index;
	int previous_point_cloud = current_point_cloud - 1;
	if (previous_point_cloud < 0)
		previous_point_cloud = NUM_VELODYNE_POINT_CLOUDS - 1;

	spherical_point_cloud velodyne_points = velodyne_data->points[current_point_cloud];
	unsigned char *velodyne_intensities = velodyne_data->intensity[current_point_cloud];

	spherical_point_cloud previous_velodyne_points = velodyne_data->points[previous_point_cloud];
	unsigned char *previous_velodyne_intensities = velodyne_data->intensity[previous_point_cloud];

	int N = velodyne_points.num_points / velodyne_params->vertical_resolution;
	int previous_N = previous_velodyne_points.num_points / velodyne_params->vertical_resolution;

	if (previous_N < N)
		N = previous_N;
	if (N < 1000)
		return;

	double *intensity = (double *) malloc(N * sizeof(double));
	double *angle = (double *) malloc(N * sizeof(double));

	double *previous_intensity = (double *) malloc(N * sizeof(double));
	double *previous_angle = (double *) malloc(N * sizeof(double));

	for (int j = 0; j < N; j += 1)
	{
		int ray = j * velodyne_params->vertical_resolution + 10;

		intensity[j] = velodyne_intensities[ray] / 255.0;
		angle[j] = velodyne_points.sphere_points[ray].horizontal_angle;

		previous_intensity[j] = previous_velodyne_intensities[ray] / 255.0;
		previous_angle[j] = previous_velodyne_points.sphere_points[ray].horizontal_angle;
	}
	pid_plot_velocity(intensity, angle, previous_intensity, previous_angle, N);

	free(intensity);
	free(angle);

	free(previous_intensity);
	free(previous_angle);
}


int
velodyne_odometry_compute_odometry(carmen_velodyne_partial_scan_message *velodyne_message,
		sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data, double v, double phi)
{
	velodyne_data->current_timestamp = velodyne_message->timestamp;

	int num_points = velodyne_message->number_of_32_laser_shots * velodyne_params->vertical_resolution;
	build_sensor_point_cloud(&(velodyne_data->points), velodyne_data->intensity, &(velodyne_data->point_cloud_index), num_points,
			NUM_VELODYNE_POINT_CLOUDS, velodyne_params->use_remission);

	carmen_velodyne_partial_scan_update_points_with_remission_check(velodyne_message, velodyne_params->vertical_resolution,
			&(velodyne_data->points[velodyne_data->point_cloud_index]), velodyne_data->intensity[velodyne_data->point_cloud_index],
			velodyne_params->ray_order,	velodyne_params->vertical_correction, velodyne_params->range_max, velodyne_message->timestamp,
			velodyne_params->use_remission);

	carmen_pose_3D_t local_pose;

	local_pose.position.x = 0.0;
	local_pose.position.y = 0.0;
	local_pose.position.z = 0.0;
	local_pose.orientation.pitch = local_pose.orientation.roll = local_pose.orientation.yaw = 0.0;

	static rotation_matrix *r_matrix_car_to_global = NULL;
	r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, local_pose.orientation);

	compute_laser_rays_targets(velodyne_params, velodyne_data, r_matrix_car_to_global, &local_pose,
			0.0, 0.0, velodyne_data->point_cloud_index, v, phi, velodyne_params->use_remission);

	compute_v_and_phi(velodyne_params, velodyne_data);

	velodyne_data->last_timestamp = velodyne_message->timestamp;

	return (1);
}
