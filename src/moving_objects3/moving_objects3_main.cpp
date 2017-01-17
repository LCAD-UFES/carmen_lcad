#include <vector>

#include <carmen/carmen.h>

#include <carmen/laser_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/mapper_interface.h>

#include <prob_motion_model.h>
#include <prob_measurement_model.h>

#include <carmen/moving_objects3_interface.h>
#include "moving_objects3_particle_filter.h"

#define NUM_OF_RAYS 1080

using namespace std;

double range_max = 70.0;

int use_map = 1;

// particle filter
carmen_moving_objects3_particles_message particles_message;
std::vector<moving_objects3_particle_t> particle_set;

//
carmen_velodyne_projected_on_ground_message current_message;
carmen_velodyne_projected_on_ground_message previous_message;

double previous_timestamp;

carmen_point_t globalpos;

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

	double inv_scan_resolution = NUM_OF_RAYS/(2*M_PI);

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		// os angulos do velodyne crescem na direção horária.
		double hor_angle = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
		min_ground_range = MAX_RANGE;

		// find index of virtual scan
		index = (int) ( (hor_angle + M_PI) * inv_scan_resolution );

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
generate_virtual_scan_from_map(double *virtual_scan, carmen_mapper_map_message *map_message)
{
	int i, j, index;
	double x, y, range, angle, delta_x, delta_y;
	double inv_scan_resolution = NUM_OF_RAYS/(2*M_PI);

	for (i = 0; i < map_message->config.x_size; i++)
	{
		for (j = 0; j < map_message->config.y_size; j++)
		{
			index = j + i * map_message->config.y_size;
			if (map_message->complete_map[index] > 0.5)
			{
				// find x and y
				x = i * map_message->config.resolution + map_message->config.x_origin;
				y = j * map_message->config.resolution + map_message->config.y_origin;

				// find range and angle
				delta_x = x - globalpos.x;
				delta_y = y - globalpos.y;

				range = sqrt(delta_x*delta_x + delta_y*delta_y);
				angle = atan2(delta_y, delta_x);

				index = (int) ( (angle + M_PI) * inv_scan_resolution );

				virtual_scan[index] = virtual_scan[index] > range ? range : virtual_scan[index];

			}

		}
	}
}


void
scan_differencing(double *current_virtual_scan, double *last_virtual_scan)
{
	(void) current_virtual_scan;
	(void) last_virtual_scan;
}


void
build_message_from_virtual_scan(carmen_velodyne_projected_on_ground_message *message, double *virtual_scan, double timestamp)
{

	if(NUM_OF_RAYS != message->num_rays)
	{
		message->angles    = (double *) realloc (message->angles,    NUM_OF_RAYS * sizeof(double));
		message->ranges    = (double *) realloc (message->ranges,    NUM_OF_RAYS * sizeof(double));
		message->intensity = (double *) realloc (message->intensity, NUM_OF_RAYS * sizeof(double));
		message->num_rays  = NUM_OF_RAYS;
	}

	double virtual_scan_resolution = carmen_degrees_to_radians(360.0/NUM_OF_RAYS);

	for (int i = 0; i < message->num_rays; i++)
	{
		message->angles[i] = (((double) i) * virtual_scan_resolution) - carmen_degrees_to_radians(180.0);
		message->ranges[i] = virtual_scan[i];
		message->intensity[i] = 1.0;
	}

	message->host = carmen_get_host();
	message->timestamp = timestamp;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_virtual_scan_message(double *virtual_scan, double timestamp, int num_of_rays)
{
	carmen_moving_objects3_virtual_scan_message virtual_scan_message;

	virtual_scan_message.num_rays = num_of_rays;
	virtual_scan_message.virtual_scan = virtual_scan;
	virtual_scan_message.timestamp = timestamp;
	virtual_scan_message.host = carmen_get_host();

	carmen_publish_moving_objects3_virtual_scan_message(&virtual_scan_message);
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
	for (int i = 0; i < NUM_OF_RAYS; i++)
	{
		virtual_scan[i] = range_max;
	}
	generate_virtual_scan(virtual_scan, velodyne_message);
	publish_virtual_scan_message(virtual_scan, velodyne_message->timestamp, NUM_OF_RAYS);
}


void
carmen_mapper_handler(carmen_mapper_map_message *map_message)
{
	double virtual_scan[NUM_OF_RAYS];
	for (int i = 0; i < NUM_OF_RAYS; i++)
	{
		virtual_scan[i] = range_max;
	}
	generate_virtual_scan_from_map(virtual_scan, map_message);
	publish_virtual_scan_message(virtual_scan, map_message->timestamp, NUM_OF_RAYS);
}


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	globalpos.theta = globalpos_message->globalpos.theta;
	globalpos.x = globalpos_message->globalpos.x;
	globalpos.y = globalpos_message->globalpos.y;
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
	if (use_map)
	{
		carmen_mapper_subscribe_message(NULL,
				(carmen_handler_t) carmen_mapper_handler,
				CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		carmen_velodyne_subscribe_partial_scan_message(NULL,
				(carmen_handler_t) carmen_velodyne_handler,
				CARMEN_SUBSCRIBE_LATEST);
	}

	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
			(carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler,
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

	if (argc > 1)
	{
		use_map = atoi(argv[1]);
	}

	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	define_messages();
	read_parameters(argc, argv);
	subscribe_messages();
	carmen_ipc_dispatch();

	return 0;
}

