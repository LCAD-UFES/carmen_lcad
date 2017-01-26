#include <vector>

#include <carmen/carmen.h>

#include <carmen/laser_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/virtual_scan_interface.h>

#include <prob_motion_model.h>
#include <prob_measurement_model.h>

#include <carmen/moving_objects3_interface.h>
#include "moving_objects3_particle_filter.h"

#define NUM_OF_RAYS 1080

using namespace std;

double range_max = 70.0;

int scan_type = 2;

// particle filter
carmen_moving_objects3_particles_message particles_message;

std::vector<track_info> objects_tracks;

//
carmen_velodyne_projected_on_ground_message current_message;
carmen_velodyne_projected_on_ground_message previous_message;

double previous_timestamp;

carmen_point_t globalpos;

carmen_map_server_offline_map_message offline_map_message;

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
		std::vector<track_info> objects_tracks)
{
	particles_message->num_particles = objects_tracks.size() * NUM_OF_PARTICLES;

	particles_message->particles = (moving_objects3_particle_t*) malloc(particles_message->num_particles * sizeof(moving_objects3_particle_t));

	for (unsigned int i = 0; i < objects_tracks.size(); i++)
	{
		for (int j = 0; j < NUM_OF_PARTICLES; j++)
		{
			int index = i+j*objects_tracks.size();
			particles_message->particles[index].pose.x = objects_tracks[i].particles[j].pose.x;
			particles_message->particles[index].pose.y = objects_tracks[i].particles[j].pose.y;
			particles_message->particles[index].pose.theta = objects_tracks[i].particles[j].pose.theta;
			particles_message->particles[index].geometry.length = objects_tracks[i].particles[j].geometry.length;
			particles_message->particles[index].geometry.width = objects_tracks[i].particles[j].geometry.width;
		}
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
				angle = carmen_normalize_theta(atan2(delta_y, delta_x) - globalpos.theta);

				index = (int) ( (angle + M_PI) * inv_scan_resolution );

				virtual_scan[index] = virtual_scan[index] > range ? range : virtual_scan[index];

			}

		}
	}
}


void
generate_virtual_scan_from_compact_map(double *virtual_scan, carmen_map_server_compact_cost_map_message *compact_map_message)
{
	int i, index;
	double x, y, range, angle, delta_x, delta_y;
	double inv_scan_resolution = NUM_OF_RAYS/(2*M_PI);

	for (i = 0; i < compact_map_message->size; i++)
	{
		if (compact_map_message->value[i] > 0.5)
		{
			// find x and y
			x = compact_map_message->coord_x[i] * compact_map_message->config.resolution + compact_map_message->config.x_origin;
			y = compact_map_message->coord_y[i] * compact_map_message->config.resolution + compact_map_message->config.y_origin;

			// find range and angle
			delta_x = x - globalpos.x;
			delta_y = y - globalpos.y;

			range = sqrt(delta_x*delta_x + delta_y*delta_y);
			angle = carmen_normalize_theta(atan2(delta_y, delta_x) - globalpos.theta);

			index = (int) ( (angle + M_PI) * inv_scan_resolution );

			virtual_scan[index] = virtual_scan[index] > range ? range : virtual_scan[index];
		}
	}
}


void
generate_virtual_scan_from_message(double *virtual_scan, carmen_virtual_scan_message *message)
{
	int i, index;
	double inv_scan_resolution = NUM_OF_RAYS/(2*M_PI);

	for (i = 0; i < message->num_rays; i++)
	{
		if (message->intensity[i] > 0.0)
		{
			index = (int) ( (message->angles[i] + M_PI) * inv_scan_resolution );
			virtual_scan[index] = virtual_scan[index] > message->ranges[i] ? message->ranges[i] : virtual_scan[index];
		}
	}

}


void
scan_differencing(double *current_virtual_scan, double *last_virtual_scan, int num_of_rays)
{
	int ind = 0;
	for (int i = 0; i < num_of_rays; i++)
	{
		track_info track_t;
		if ( current_virtual_scan[i] > last_virtual_scan[i] )
		{
			if (ind + 5 < i)
			{
				track_t.index = i;
				objects_tracks.push_back(track_t);
			}
			ind = i;
		}
	}

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


void
remove_static_points(double *virtual_scan, double *offline_virtual_scan, int num_of_rays)
{
	for (int i = 0; i < num_of_rays; i++)
	{
		virtual_scan[i] = virtual_scan[i] < (offline_virtual_scan[i] - 1.0) ? virtual_scan[i] : range_max;
	}
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
	static int frames = 0;
	carmen_moving_objects3_particles_message particles_message;

	double virtual_scan[NUM_OF_RAYS];
	static double last_virtual_scan[NUM_OF_RAYS];
	double offline_virtual_scan[NUM_OF_RAYS];
	for (int i = 0; i < NUM_OF_RAYS; i++)
	{
		offline_virtual_scan[i] = virtual_scan[i] = range_max;
	}
	generate_virtual_scan(virtual_scan, velodyne_message);

	// remove points in offline map
	generate_virtual_scan_from_map(offline_virtual_scan, &offline_map_message);
	remove_static_points(virtual_scan, offline_virtual_scan, NUM_OF_RAYS);

	double delta_time = 0.0;
	if (frames == 1)
	{
		scan_differencing(virtual_scan, last_virtual_scan, NUM_OF_RAYS);

		printf("size %ld\n", objects_tracks.size());
		for (unsigned int i = 0; i < objects_tracks.size(); i++)
		{
			objects_tracks[i].particles = importance_sampling(virtual_scan, NUM_OF_RAYS, objects_tracks[i].index, NUM_OF_PARTICLES);
		}
	}
	if (frames > 1)
	{
		delta_time = velodyne_message->timestamp - previous_timestamp;
		for (unsigned int i = 0; i < objects_tracks.size(); i++)
		{
			objects_tracks[i].particles = algorithm_particle_filter(objects_tracks[i].particles, virtual_scan, NUM_OF_RAYS, delta_time);
		}
	}

	frames++;

	publish_virtual_scan_message(virtual_scan, velodyne_message->timestamp, NUM_OF_RAYS);

	memcpy(last_virtual_scan, virtual_scan, NUM_OF_RAYS * sizeof(double));
	previous_timestamp = velodyne_message->timestamp;


	build_particles_message(&particles_message, objects_tracks);
	carmen_publish_moving_objects3_particles_message(&particles_message);
	free (particles_message.particles);
}


void
carmen_mapper_handler(carmen_mapper_map_message *map_message)
{
	double virtual_scan[NUM_OF_RAYS];
	double offline_virtual_scan[NUM_OF_RAYS];
	for (int i = 0; i < NUM_OF_RAYS; i++)
	{
		offline_virtual_scan[i] = virtual_scan[i] = range_max;
	}

	generate_virtual_scan_from_map(virtual_scan, map_message);

	// remove points in offline map
	generate_virtual_scan_from_map(offline_virtual_scan, &offline_map_message);
	remove_static_points(virtual_scan, offline_virtual_scan, NUM_OF_RAYS);

	publish_virtual_scan_message(virtual_scan, map_message->timestamp, NUM_OF_RAYS);
}


void
carmen_compact_map_handler(carmen_map_server_compact_cost_map_message *compact_map_message)
{
	double virtual_scan[NUM_OF_RAYS];
	for (int i = 0; i < NUM_OF_RAYS; i++)
	{
		virtual_scan[i] = range_max;
	}
	generate_virtual_scan_from_compact_map(virtual_scan, compact_map_message);
	publish_virtual_scan_message(virtual_scan, compact_map_message->timestamp, NUM_OF_RAYS);
}


void
carmen_virtual_scan_handler(carmen_virtual_scan_message *message)
{
	double virtual_scan[NUM_OF_RAYS];
	for (int i = 0; i < NUM_OF_RAYS; i++)
	{
		virtual_scan[i] = range_max;
	}
	generate_virtual_scan_from_message(virtual_scan, message);
	publish_virtual_scan_message(virtual_scan, message->timestamp, NUM_OF_RAYS);
}


void
carmen_offline_map_handler()
{

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
	switch (scan_type)
	{
		case 0:
			carmen_velodyne_subscribe_partial_scan_message(NULL,
					(carmen_handler_t) carmen_velodyne_handler,
					CARMEN_SUBSCRIBE_LATEST);
			break;
		case 1:
			carmen_mapper_subscribe_message(NULL,
					(carmen_handler_t) carmen_mapper_handler,
					CARMEN_SUBSCRIBE_LATEST);
			break;
		case 2:
			carmen_virtual_scan_subscribe_message(NULL,
					(carmen_handler_t) carmen_virtual_scan_handler,
					CARMEN_SUBSCRIBE_LATEST);
			break;
		case 3:
			carmen_map_server_subscribe_compact_cost_map(NULL,
					(carmen_handler_t) carmen_compact_map_handler,
					CARMEN_SUBSCRIBE_LATEST);
			break;
		default:
			break;

	}

	carmen_map_server_subscribe_offline_map(&offline_map_message,
			(carmen_handler_t) carmen_offline_map_handler,
			CARMEN_SUBSCRIBE_LATEST);

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
		scan_type = atoi(argv[1]);
	}

	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	define_messages();
	read_parameters(argc, argv);
	subscribe_messages();
	carmen_ipc_dispatch();

	return 0;
}

