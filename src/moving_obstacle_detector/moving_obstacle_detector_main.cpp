#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <carmen/carmen.h>

#include <carmen/behavior_selector_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/global_graphics.h>

// open cv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "moving_obstacle_detector.h"

carmen_point_t globalpos;
double range_max = 70.0;

char *current_map;
char *previous_map;

carmen_map_config_t current_config;
carmen_map_config_t previous_config;

std::vector<moving_obstacle_t> moving_obstacle_list;

carmen_behavior_selector_road_profile_message *road_profile_message;

#define HISTORY_SIZE 40
#define MAX_ASSOCIATION_DISTANCE 2.77

//#define USE_OPEN_CV

void
subtract_map(char *subtracted_map, char *current_map, char *previous_map, carmen_map_config_t current_config)
{
	int size = current_config.x_size * current_config.y_size;

	for (int i = 0; i < size; i++)
	{
		subtracted_map[i] = current_map[i] - previous_map[i];
	}
}


void
show_map(char *map, carmen_map_config_t config)
{
	unsigned int size = config.x_size * config.y_size;
	unsigned char *map_char = (unsigned char *) malloc (size * sizeof(unsigned char));
	unsigned int width = config.x_size;
	unsigned int height = config.y_size;

	for (unsigned int i = 0; i < size; ++i)
	{
		// get the current row
		unsigned int row = (height - 1) - i % height;
		// get the current col
		unsigned int col = i / height;
		// get the current index
		unsigned int index = row * width + col;

		map_char[index] = (1 - map[i]) * 255;
	}

	// create a new opencv image
	cv::Mat img(width, height, CV_8UC1, map_char);
	cv::Mat dst;

	cv::resize(img, dst, cv::Size(600,600));

	cv::imshow("Map", dst);
	cv::waitKey(1);

	free (map_char);
}


void
flood_fill(char *map, carmen_map_config_t *config, moving_obstacle_observation_t *observation, int i, int j)
{
	if (i >= 0 && j >= 0 && i < config->x_size && j < config->y_size)
	{
		int index = j + i * config->y_size;
		if (map[index] == 1)
		{
			map[index] = 0;
			carmen_position_t cell;
			cell.x = i * config->resolution + config->x_origin;
			cell.y = j * config->resolution + config->y_origin;
			observation->cell_vector.push_back(cell);

			flood_fill(map, config, observation, i, j - 1);
			flood_fill(map, config, observation, i + 1, j);
			flood_fill(map, config, observation, i, j + 1);
			flood_fill(map, config, observation, i - 1, j);

			flood_fill(map, config, observation, i - 1, j - 1);
			flood_fill(map, config, observation, i + 1, j - 1);
			flood_fill(map, config, observation, i + 1, j + 1);
			flood_fill(map, config, observation, i - 1, j + 1);
		}
	}
}


void
compute_centroid(moving_obstacle_observation_t *observation)
{
	unsigned int i = 0;
	double sum_x = 0.0, sum_y = 0.0;

	for (i = 0; i < observation->cell_vector.size(); i++)
	{
		sum_x += observation->cell_vector[i].x;
		sum_y += observation->cell_vector[i].y;
	}

	observation->centroid.x = sum_x / ((double) i);
	observation->centroid.y = sum_y / ((double) i);
}


double
distance(carmen_position_t p1, carmen_position_t p2)
{
	return sqrt( pow( (p2.x - p1.x), 2) + pow( (p2.y - p1.y), 2));
}


int
compare_distances(const void *a, const void *b)
{
	distance_t arg1 = *((distance_t *) a);
	distance_t arg2 = *((distance_t *) b);

	if (arg1.distance < arg2.distance)
		return -1;
	if (arg1.distance > arg2.distance)
		return 1;
	return 0;
}


int
associate_observations(moving_obstacle_observation_t observation)
{
	int i = 0;
	int size = moving_obstacle_list.size();

	if (size > 0)
	{
		distance_t *distance_array = (distance_t *) malloc(size * sizeof(distance_t));

		for (i = 0; i < size; i++)
		{
			distance_array[i].distance = distance(moving_obstacle_list[i].observations.front().centroid, observation.centroid);
			distance_array[i].index = i;
		}

		std::qsort(distance_array, size, sizeof(distance_t), compare_distances);

		int index = distance_array[0].index;

		free(distance_array);

		return (index);
	}
	return -1;
}


double
distace_to_lane(carmen_position_t position)
{
	double min_dist = DBL_MAX;
	double dist;
	carmen_position_t rddf_pos;
	for (int i = 0; i < road_profile_message->number_of_poses; i++)
	{
		rddf_pos.x = road_profile_message->poses[i].x;
		rddf_pos.y = road_profile_message->poses[i].y;

		dist = distance(position, rddf_pos);
		if (dist < min_dist)
		{
			min_dist = dist;
		}
	}

	return min_dist;

}


void
detect_obstacles(char *subtracted_map, char *current_map, carmen_map_config_t config, double timestamp)
{
	int i, j, index;
	int map_size = config.x_size * config.y_size;

	// copy the current map because it will be modified
	char *map = (char *) malloc(map_size * sizeof(char));
	memcpy(map, current_map, map_size * sizeof(char));

	for (index = 0; index < map_size; index++)
	{
		if (subtracted_map[index] == 1)
		{
			i = index/config.y_size;
			j = index % config.y_size;

			carmen_position_t position;
			position.x = i * config.resolution + config.x_origin;
			position.y = j * config.resolution + config.y_origin;

			if (distace_to_lane(position) > 7.0)
				continue;

			moving_obstacle_observation_t observation;

			flood_fill(map, &config, &observation, i, j);

			if (observation.cell_vector.size() > 0 && observation.cell_vector.size() < 500)
			{
				//observation.centroid = position;

				compute_centroid(&observation);
				int val = associate_observations(observation);
				observation.timestamp = timestamp;

				if (val >= 0 && distance(moving_obstacle_list[val].observations.front().centroid, observation.centroid) <= MAX_ASSOCIATION_DISTANCE)
				{
					moving_obstacle_list[val].observations.push_front(observation);
					moving_obstacle_list[val].associated = 1;
					moving_obstacle_list[val].age++;
					if (moving_obstacle_list[val].observations.size() > HISTORY_SIZE)
					{
						moving_obstacle_list[val].observations.pop_back();
					}
				}
				else
				{
					moving_obstacle_t obstacle;
					obstacle.color = index % 10;
					obstacle.associated = 0;
					obstacle.age = 0;
					obstacle.observations.push_front(observation);
					if (obstacle.observations.size() > HISTORY_SIZE)
					{
						obstacle.observations.pop_back();
					}
					moving_obstacle_list.push_back(obstacle);
				}
			}
		}
	}

	free(map);
}


void
remove_obstacles(double timestamp)
{
	carmen_position_t globalposition;

	globalposition.x = globalpos.x;
	globalposition.y = globalpos.y;

	double dist;
	double timediff;

	std::vector<moving_obstacle_t>::iterator iter = moving_obstacle_list.begin();
	while (iter != moving_obstacle_list.end())
	{
		dist = distance(globalposition, iter->observations[0].centroid);
		timediff = timestamp - iter->observations[0].timestamp;

		if(dist > 70 || (iter->age > 0 && iter->associated == 0) || timediff > 0.3)
		{
			iter = moving_obstacle_list.erase(iter);
		}
		else
		{
			++iter;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_subtracted_points(char *subtracted_map, carmen_map_config_t config)
{
	int i, j, k, index, num_positions;
	int map_size = config.x_size * config.y_size;
	double x, y;
	carmen_mapper_virtual_laser_message virtual_laser_message;

	virtual_laser_message.colors = (char *) (malloc(map_size * sizeof(char)));
	virtual_laser_message.positions = (carmen_position_t *) (malloc(map_size * sizeof(carmen_position_t)) );

	k = 0;
	num_positions = 0;

	for (i = 0; i < config.x_size; i++)
	{
		for (j = 0; j < config.y_size; j++)
		{
			index = j + i * config.y_size;
			if (subtracted_map[index] == 1)
			{
				num_positions++;

				// find x and y
				x = i * config.resolution + config.x_origin;
				y = j * config.resolution + config.y_origin;

				virtual_laser_message.colors[k] = CARMEN_RED;
				virtual_laser_message.positions[k].x = x;
				virtual_laser_message.positions[k].y = y;

				k++;
			}
		}
	}

	if (num_positions > 0)
	{
		virtual_laser_message.num_positions = num_positions;
		virtual_laser_message.host = carmen_get_host();
		carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
	}
	free(virtual_laser_message.colors);
	free(virtual_laser_message.positions);
}


void
publish_moving_obstacles()
{
	int num_points = 0, k = 0;
	carmen_mapper_virtual_laser_message virtual_laser_message;

	for (unsigned int i = 0; i < moving_obstacle_list.size(); i++)
	{
		num_points += moving_obstacle_list[i].observations[0].cell_vector.size();
	}

	virtual_laser_message.colors = (char *) (malloc(num_points * sizeof(char)));
	virtual_laser_message.positions = (carmen_position_t *) (malloc(num_points * sizeof(carmen_position_t)) );


	for (unsigned int i = 0; i < moving_obstacle_list.size(); i++)
	{
		int cell_size = moving_obstacle_list[i].observations[0].cell_vector.size();

		for (int j = 0; j < cell_size; j++)
		{
			virtual_laser_message.colors[k] = moving_obstacle_list[i].color;
			virtual_laser_message.positions[k] = moving_obstacle_list[i].observations[0].cell_vector[j];
			k++;
		}
	}

	virtual_laser_message.num_positions = num_points;
	virtual_laser_message.host = carmen_get_host();
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());

	free(virtual_laser_message.colors);
	free(virtual_laser_message.positions);
}

///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_compact_cost_map_handler(carmen_map_server_compact_cost_map_message *compact_map_message)
{

	static int first = 1;

	// build complete map
	int map_size = compact_map_message->config.x_size * compact_map_message->config.y_size;

	if (first == 1)
	{
		current_map = (char *) malloc(map_size * sizeof(char));
		previous_map = (char *) malloc(map_size * sizeof(char));
	}

	memset(current_map, 0, map_size * sizeof(char));

	current_config = compact_map_message->config;
	for (int i = 0; i < compact_map_message->size; i++)
	{
		int index = compact_map_message->coord_y[i] + compact_map_message->coord_x[i] * compact_map_message->config.y_size;
		current_map[index] = compact_map_message->value[i] > 0.67 ? 1 : 0;
	}

	#ifdef USE_OPEN_CV
	show_map(current_map, compact_map_message->config);
	#endif

	if (first == 0)
	{
		char *subtracted_map = (char *) malloc(map_size * sizeof(char));

		if (current_config.x_origin == previous_config.x_origin && current_config.y_origin == previous_config.y_origin)
		{
			subtract_map(subtracted_map, current_map, previous_map, current_config);
			detect_obstacles(subtracted_map, current_map, current_config, compact_map_message->timestamp);
		}

		remove_obstacles(compact_map_message->timestamp);

		#ifdef USE_OPEN_CV
			//show_map(subtracted_map, compact_map_message->config);
		#endif

		publish_moving_obstacles();

		free(subtracted_map);
	}

	memcpy(previous_map, current_map, map_size * sizeof(char));
	previous_config = current_config;

	first = 0;

}


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	globalpos.theta = globalpos_message->globalpos.theta;
	globalpos.x = globalpos_message->globalpos.x;
	globalpos.y = globalpos_message->globalpos.y;
}


void
carmen_lane_message_handler(carmen_behavior_selector_road_profile_message *message)
{
	road_profile_message = message;
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{

		free(current_map);
		free(previous_map);

		cv::destroyAllWindows();

		carmen_ipc_disconnect();

		printf("Moving obstacle detector: disconnected.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
	carmen_map_server_subscribe_compact_cost_map(NULL,
		(carmen_handler_t) carmen_compact_cost_map_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
		(carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME,
		(char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
		NULL, sizeof (carmen_behavior_selector_road_profile_message),
		(carmen_handler_t) carmen_lane_message_handler, CARMEN_SUBSCRIBE_LATEST);
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


int
main(int argc, char **argv)
{

#ifdef USE_OPEN_CV
	cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);
	std::cout<<"OpenCV Version used:"<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<std::endl;
#endif
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	read_parameters(argc, argv);
	subscribe_messages();
	carmen_ipc_dispatch();

	return 0;
}

