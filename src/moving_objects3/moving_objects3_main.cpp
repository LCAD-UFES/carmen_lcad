#include <vector>

#include <carmen/carmen.h>
#include <carmen/laser_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <prob_motion_model.h>
#include <prob_measurement_model.h>

#include <carmen/moving_objects3_interface.h>
#include "moving_objects3_particle_filter.h"

using namespace std;

typedef struct
{
	int num_spheres;
	int num_particles;
	int num_points_to_store;
	int num_angular_sections;

}PolarSlamParams;

// parameters of the module
PolarSlamParams polar_slam_params;

// slam
OdometryMotionModelParams odometry_model_params;
BeanRangeFinderMeasurementModelParams laser_model_params;

// particle filter
carmen_moving_objects3_particles_message particles_message;
std::vector<moving_objects3_particle_t> particle_set;

double previous_timestamp;

void
arrange_velodyne_vertical_angles_to_true_position(carmen_velodyne_partial_scan_message *velodyne_message)
{
	const int column_correspondence[32] =
	{
		0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
		24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
	};

	int i, j;
	unsigned short original_distances[32];

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		memcpy(original_distances, velodyne_message->partial_scan[i].distance, 32 * sizeof(unsigned short));

		for (j = 0; j < 32; j++)
		{
			velodyne_message->partial_scan[i].distance[column_correspondence[j]] = original_distances[j];
		}
	}
}


void
build_particles_message(carmen_moving_objects3_particles_message *particles_message,
		std::vector<moving_objects3_particle_t> particle_set)
{
	particles_message->num_particles = NUM_OF_PARTICLES;

	if(particles_message->particles == NULL)
		particles_message->particles = (moving_objects3_particle_t*) malloc(particles_message->num_particles * sizeof(moving_objects3_particle_t));

	for(int i = 0; i < particles_message->num_particles; i++)
	{
		particles_message->particles[i].pose.x = particle_set[i].pose.x;
		particles_message->particles[i].pose.y = particle_set[i].pose.y;
		particles_message->particles[i].pose.theta = particle_set[i].pose.theta;
		particles_message->particles[i].geometry.length = particle_set[i].geometry.length;
		particles_message->particles[i].geometry.width = particle_set[i].geometry.width;
	}

	particles_message->host = carmen_get_host();
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


// ********************************************************************************
// TODO: refactor: separar a funcao de projecao no chao e de publicacao da projecao
// ********************************************************************************
void
generate_2D_map_from_velodyne_pointcloud(carmen_velodyne_partial_scan_message *velodyne_message, double *range, double *angles)
{
	const static double sorted_vertical_angles[32] =
	{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
	};

	double delta_time = velodyne_message->timestamp - previous_timestamp;

	arrange_velodyne_vertical_angles_to_true_position(velodyne_message);

	int i, j;
	double min_ground_range;

	// *********************
	// ** TODO: ler do ini
	// *********************
	double CAR_HEIGHT = 1.725;
	double MAX_RANGE = laser_model_params.max_range;
	double MIN_RANGE = 3.0;

	carmen_velodyne_projected_on_ground_message message;
	message.angles = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));
	message.ranges = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));
	message.intensity = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		// os angulos do velodyne crescem na direção horária.
		double hor_angle = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
		min_ground_range = MAX_RANGE;

		message.angles[i] = hor_angle;
		message.ranges[i] = MAX_RANGE;
		message.intensity[i] = 0.0;

		// for (j = 0; j < (32 - 1); j++)
		for (j = 0; j < 23; j++) // the 22nd angle is the 0.0. The next angles are higher (trees, etc)
		{
			double range_0 = (((double) velodyne_message->partial_scan[i].distance[j]) / 500.0);
			double range_1 = (((double) velodyne_message->partial_scan[i].distance[j + 1]) / 500.0);

			// **********************************************************
			// TODO: checar se essa eh a melhor forma de tratar max_range
			// **********************************************************
			if ((range_0 >= MAX_RANGE) || (range_0 <= MIN_RANGE))
				continue;

			if ((range_1 >= MAX_RANGE) || (range_1 <= MIN_RANGE))
				continue;

			double angle_0 = carmen_degrees_to_radians(sorted_vertical_angles[j]);
			double angle_1 = carmen_degrees_to_radians(sorted_vertical_angles[j + 1]);

			double cos_vert_angle0 = cos(angle_0);
			double cos_vert_angle1 = cos(angle_1);

			double xy_distance0 = range_0 * cos_vert_angle0;
			double xy_distance1 = range_1 * cos_vert_angle1;

			double delta_ray = xy_distance1 - xy_distance0;
			double next_ray_angle = -carmen_normalize_theta(angle_1 - angle_0) + atan(CAR_HEIGHT / xy_distance0);
			double expected_delta_ray = (CAR_HEIGHT - xy_distance0 * tan(next_ray_angle)) / tan(next_ray_angle);

			if (delta_ray < expected_delta_ray)
			{
				if ((delta_ray / expected_delta_ray) < 0.5)
				{
					message.intensity[i] = 1.0;

					if ((xy_distance1 > MIN_RANGE) && (xy_distance1 < MAX_RANGE) && (xy_distance1 < min_ground_range))
						min_ground_range = xy_distance1;
				}
			}
		}

		message.ranges[i] = min_ground_range;
		range[i] = min_ground_range;
		angles[i] = hor_angle;
	}

	message.num_rays = velodyne_message->number_of_32_laser_shots;
	message.host = carmen_get_host();
	message.timestamp = velodyne_message->timestamp;

	carmen_publish_velodyne_projected_message(&message);



	static int first = 1;

	if (first == 1)
	{
		for(int i = 0; i < NUM_OF_PARTICLES; i++)
		{
			moving_objects3_particle_t particle;
			particle.pose.x = carmen_uniform_random(-10.0, 10.0);
			particle.pose.y = carmen_uniform_random(-3.0, 3.0);
			particle.pose.theta = carmen_uniform_random(-M_PI/4.0, M_PI/4.0);
			particle.geometry.length = 4.5;
			particle.geometry.width = 1.60;
			particle.velocity = carmen_uniform_random(0.0, 1.0);

			particle_set.push_back(particle);
		}
		first = 0;
		delta_time = 0.05;
	}
	else
	{
		particle_set = algorithm_particle_filter(particle_set, message, delta_time);
	}

	build_particles_message(&particles_message, particle_set);

	previous_timestamp = particles_message.timestamp = velodyne_message->timestamp;

	carmen_publish_moving_objects3_particles_message(&particles_message);

	free(message.angles);
	free(message.ranges);
	free(message.intensity);

}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_velodyne_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	int num_rays = velodyne_message->number_of_32_laser_shots;

	double *angles = (double*) calloc (sizeof(double), num_rays);
	double *range = (double*) calloc (sizeof(double), num_rays);

	generate_2D_map_from_velodyne_pointcloud(velodyne_message, range, angles);

	free(angles);
	free(range);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();

		printf("Moving Objects: disconnected.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
	carmen_velodyne_subscribe_partial_scan_message(NULL,
			(carmen_handler_t) carmen_velodyne_handler,
			CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters(int argc, char *argv[])
{
	carmen_param_t basic_param_list[] =
	{
			{(char *) "polar_slam", (char *) "odom_a1", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha1), 1, NULL},
			{(char *) "polar_slam", (char *) "odom_a2", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha2), 1, NULL},
			{(char *) "polar_slam", (char *) "odom_a3", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha3), 1, NULL},
			{(char *) "polar_slam", (char *) "odom_a4", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha4), 1, NULL},
	};

	carmen_param_t laser_param_list[] =
	{
			{(char *) "polar_slam", (char *) "laser_sampling_step", CARMEN_PARAM_INT, 	&(laser_model_params.sampling_step), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_num_beams", CARMEN_PARAM_INT, 		&(laser_model_params.laser_beams), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_fov_range", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.fov_range), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_max_range", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.max_range), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_lambda_short", CARMEN_PARAM_DOUBLE, &(laser_model_params.lambda_short), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_sigma_zhit", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.sigma_zhit), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_zhit", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zhit), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_zmax", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zmax), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_zrand", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zrand), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_zshort", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zshort), 0, NULL},
			{(char *) "robot", (char *) "frontlaser_offset", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.front_offset), 0, NULL},
			{(char *) "robot", (char *) "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.side_offset), 0, NULL},
			{(char *) "robot", (char *) "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(laser_model_params.angular_offset), 0, NULL},
	};

	carmen_param_t polar_slam_param_list[] =
	{
			{(char *) "polar_slam", (char *) "num_spheres", 			CARMEN_PARAM_INT, &(polar_slam_params.num_spheres), 0, NULL},
			{(char *) "polar_slam", (char *) "num_particles", 			CARMEN_PARAM_INT, &(polar_slam_params.num_particles), 0, NULL},
			{(char *) "polar_slam", (char *) "num_points_to_store", 	CARMEN_PARAM_INT, &(polar_slam_params.num_points_to_store), 0, NULL},
			{(char *) "polar_slam", (char *) "num_angular_sections", 	CARMEN_PARAM_INT, &(polar_slam_params.num_angular_sections), 0, NULL}
	};

	carmen_param_install_params(argc, argv, basic_param_list, sizeof(basic_param_list) / sizeof(basic_param_list[0]));
	carmen_param_install_params(argc, argv, laser_param_list, sizeof(laser_param_list) / sizeof(laser_param_list[0]));
	carmen_param_install_params(argc, argv, polar_slam_param_list, sizeof(polar_slam_param_list) / sizeof(polar_slam_param_list[0]));
}


void
define_messages()
{
	carmen_moving_objects3_define_messages();
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	define_messages();
	read_parameters(argc, argv);
	subscribe_messages();
	carmen_ipc_dispatch();

	return 0;
}

