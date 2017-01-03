#include <vector>

#include <carmen/carmen.h>
#include <carmen/laser_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <prob_motion_model.h>
#include <prob_measurement_model.h>

#include <carmen/moving_objects3_interface.h>
#include "moving_objects3_particle_filter.h"

#define NUM_OF_RAYS 360

using namespace std;

double range_max = 70.0;

// particle filter
carmen_moving_objects3_particles_message particles_message;
std::vector<moving_objects3_particle_t> particle_set;

//
carmen_velodyne_projected_on_ground_message current_message;
carmen_velodyne_projected_on_ground_message previous_message;

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


void
generate_virtual_scan(double *virtual_scan, carmen_velodyne_partial_scan_message *velodyne_message)
{
	const static double sorted_vertical_angles[32] =
	{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
	};

	arrange_velodyne_vertical_angles_to_true_position(velodyne_message);

	int i, j, index;
	double min_ground_range;

	double CAR_HEIGHT = 1.725;
	double MAX_RANGE = range_max;
	double MIN_RANGE = 3.0;

	double virtual_scan_resolution = carmen_degrees_to_radians(360.0/NUM_OF_RAYS);

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		// os angulos do velodyne crescem na direção horária.
		double hor_angle = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
		min_ground_range = MAX_RANGE;

		// find index of virtual scan
		index = (int) ((hor_angle + carmen_degrees_to_radians(180.0)) / virtual_scan_resolution);

		virtual_scan[index] = MAX_RANGE;

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
					if ((xy_distance1 > MIN_RANGE) && (xy_distance1 < MAX_RANGE) && (xy_distance1 < min_ground_range))
						min_ground_range = xy_distance1;
				}
			}
		}
		virtual_scan[index] = min_ground_range;
	}
}


void
build_message_from_virtual_scan(carmen_velodyne_projected_on_ground_message *message, double *virtual_scan, double timestamp)
{
	message->angles = (double *) malloc (NUM_OF_RAYS * sizeof(double));
	message->ranges = (double *) malloc (NUM_OF_RAYS * sizeof(double));
	message->intensity = (double *) malloc (NUM_OF_RAYS * sizeof(double));
	message->num_rays = NUM_OF_RAYS;

	double virtual_scan_resolution = carmen_degrees_to_radians(360.0/NUM_OF_RAYS);

	for (int i = 0; i < message->num_rays; i++)
	{
		message->angles[i] = (((double) i) * virtual_scan_resolution) - carmen_degrees_to_radians(180.0);
		message->ranges[i] = virtual_scan[i];
		message->intensity[i] = 1.0;
	}

	message->host = carmen_get_host();
	message->timestamp = timestamp;
	carmen_publish_velodyne_projected_message(message);

	free(message->angles);
	free(message->ranges);
	free(message->intensity);
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
	double MAX_RANGE = range_max;
	double MIN_RANGE = 3.0;

	current_message.angles = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));
	current_message.ranges = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));
	current_message.intensity = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		// os angulos do velodyne crescem na direção horária.
		double hor_angle = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
		min_ground_range = MAX_RANGE;

		current_message.angles[i] = hor_angle;
		current_message.ranges[i] = MAX_RANGE;
		current_message.intensity[i] = 0.0;

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
					current_message.intensity[i] = 1.0;

					if ((xy_distance1 > MIN_RANGE) && (xy_distance1 < MAX_RANGE) && (xy_distance1 < min_ground_range))
						min_ground_range = xy_distance1;
				}
			}
		}

		current_message.ranges[i] = min_ground_range;
		range[i] = min_ground_range;
		angles[i] = hor_angle;
	}

	current_message.num_rays = velodyne_message->number_of_32_laser_shots;
	current_message.host = carmen_get_host();
	current_message.timestamp = velodyne_message->timestamp;

	carmen_publish_velodyne_projected_message(&current_message);



	static int first = 1;

	if (first == 1)
	{
		for(int i = 0; i < NUM_OF_PARTICLES; i++)
		{
			moving_objects3_particle_t particle;
			particle.pose.x = carmen_uniform_random(-15.0, 15.0);
			particle.pose.y = carmen_uniform_random(-15.0, 15.0);
			particle.pose.theta = carmen_uniform_random(-M_PI, M_PI);
			particle.geometry.length = 4.5;
			particle.geometry.width = 1.60;
			particle.velocity = carmen_uniform_random(-25.0, 25.0);

			particle_set.push_back(particle);
		}
		first = 0;
		delta_time = 0.05;
	}
	else
	{
		particle_set = algorithm_particle_filter(particle_set, current_message, delta_time);
	}

	printf("num points: %d\n", current_message.num_rays);
	build_particles_message(&particles_message, particle_set);

	previous_timestamp = particles_message.timestamp = velodyne_message->timestamp;

	carmen_publish_moving_objects3_particles_message(&particles_message);

	free(current_message.angles);
	free(current_message.ranges);
	free(current_message.intensity);

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

	double virtual_scan[NUM_OF_RAYS];
//	int num_rays = velodyne_message->number_of_32_laser_shots;

//	double *angles = (double*) calloc (sizeof(double), num_rays);
//	double *range = (double*) calloc (sizeof(double), num_rays);

//	generate_2D_map_from_velodyne_pointcloud(velodyne_message, range, angles);

	generate_virtual_scan(virtual_scan, velodyne_message);
	build_message_from_virtual_scan(&current_message, virtual_scan, velodyne_message->timestamp);

//	free(angles);
//	free(range);
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
	carmen_param_t param_list[] =
	{
			{(char *) "localize_ackerman", (char *) "velodyne_range_max", CARMEN_PARAM_DOUBLE, &(range_max), 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
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

