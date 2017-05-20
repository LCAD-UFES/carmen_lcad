#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <carmen/carmen.h>

#include <carmen/behavior_selector_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/global_graphics.h>

// open cv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "moving_obstacle_detector.h"

carmen_point_t globalpos;
double range_max = 70.0;
int rddf_ready = 0;

char *current_map;
char *previous_map;
char *offline_map;

carmen_map_config_t current_config;
carmen_map_config_t previous_config;
carmen_map_config_t offline_config;

std::vector<moving_obstacle_t> moving_obstacle_list;

carmen_behavior_selector_road_profile_message *road_profile_message;

#define HISTORY_SIZE 20
#define MAX_ASSOCIATION_DISTANCE 2.5

#define USE_OPEN_CV


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
show_map(char *map, int map_id, carmen_map_config_t config)
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

	char map_name[10];
	sprintf(map_name, "Map %d", map_id);

	cv::imshow(map_name, dst);
	cv::waitKey(1);

	free (map_char);
}


void
dilate_map(char *current_map, carmen_map_config_t *config)
{
	int i, j, index;
	int map_size = config->x_size * config->y_size;

	char *map = (char *) malloc(map_size * sizeof(char));
	memcpy(map, current_map, map_size * sizeof(char));

	for (i = 1; i < (config->x_size - 1); i++)
	{
		for (j = 1; j < (config->y_size - 1); j++)
		{
			index = j + i * previous_config.y_size;
			if (map[index] == 1)
			{
				index = (j - 1) + (i - 1) * previous_config.y_size;
				current_map[index] = 1;
				index = (j - 1) + (i) * previous_config.y_size;
				current_map[index] = 1;
				index = (j - 1) + (i + 1) * previous_config.y_size;
				current_map[index] = 1;

				index = (j) + (i - 1) * previous_config.y_size;
				current_map[index] = 1;
				index = (j) + (i + 1) * previous_config.y_size;
				current_map[index] = 1;

				index = (j + 1) + (i - 1) * previous_config.y_size;
				current_map[index] = 1;
				index = (j + 1) + (i) * previous_config.y_size;
				current_map[index] = 1;
				index = (j + 1) + (i + 1) * previous_config.y_size;
				current_map[index] = 1;
			}
		}
	}
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
fast_flood_fill(char *map, carmen_map_config_t *config, moving_obstacle_observation_t *observation, int i, int j)
{
	std::queue<cell_coords_t> cell_queue;

	cell_coords_t cell;
	cell.x = i;
	cell.y = j;

	static const int di[8] = {0, 1, 1, 1, 0, -1, -1, -1}; // relative neighbor i coordinates
	static const int dj[8] = {-1, -1, 0, 1, 1, 1, 0, -1}; // relative neighbor j coordinates

	cell_queue.push(cell);

	while (!cell_queue.empty())
	{
		cell = cell_queue.front();
		cell_queue.pop();
		i = cell.x;
		j = cell.y;
		int index = j + i * config->y_size;

		if (map[index] == 1)
		{
			map[index] = 0;

			carmen_position_t cell_position;
			cell_position.x = i * config->resolution + config->x_origin;
			cell_position.y = j * config->resolution + config->y_origin;
			observation->cell_vector.push_back(cell_position);

			for (int k = 0; k < 8; k++)
			{
				int ni = i + di[k];
				int nj = j + dj[k];

				if (ni >= 0 && nj >= 0 && ni < config->x_size && nj < config->y_size)
				{
					cell.x = ni;
					cell.y = nj;
					cell_queue.push(cell);
				}
			}
		}
	}
}


double
distance(carmen_position_t p1, carmen_position_t p2)
{
	return sqrt( pow( (p2.x - p1.x), 2) + pow( (p2.y - p1.y), 2));
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


void
compute_velocity_and_orientation(moving_obstacle_t *obstacle)
{
	double delta_dist = 0.0;
	double delta_time = 0.0;
	double velocity = 0.0;
	double sum_velocity = 0.0;
	int num = 0;

	for (unsigned int i = 1; i < obstacle->observations.size(); i++)
	{
		delta_dist = distance(obstacle->observations[i].centroid,obstacle->observations[i-1].centroid);
		delta_time = obstacle->observations[i-1].timestamp - obstacle->observations[i].timestamp;

		if (delta_time > 0.0)
			velocity = delta_dist / delta_time;
		if (velocity < 35.0)
		{
			sum_velocity += velocity;
			num++;
		}
	}

	if (num != 0)
		obstacle->velocity = sum_velocity / (double) num;
	obstacle->orientation = globalpos.theta; //atan2(sum_delta_y, sum_delta_x);

}


void
predict_position(moving_obstacle_t *obstacle, double timestamp)
{
	carmen_position_t position = obstacle->observations.front().centroid;

	double delta_t = timestamp - obstacle->observations.front().timestamp;

	obstacle->position.x = position.x + delta_t * obstacle->velocity * cos(obstacle->orientation);
	obstacle->position.y = position.y + delta_t * obstacle->velocity * sin(obstacle->orientation);
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
			distance_array[i].distance = distance(moving_obstacle_list[i].position, observation.centroid);
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
distance_to_lane(carmen_position_t position)
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

	for (int i = 0; i < road_profile_message->number_of_poses_back; i++)
	{
		rddf_pos.x = road_profile_message->poses_back[i].x;
		rddf_pos.y = road_profile_message->poses_back[i].y;

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
	static int current_id = 0;

	// copy the current map because it will be modified
	char *map = (char *) malloc(map_size * sizeof(char));
	memcpy(map, current_map, map_size * sizeof(char));

	// predict objects locations based on velocity
	for (unsigned int i = 0; i < moving_obstacle_list.size(); i++)
	{
		predict_position( &moving_obstacle_list[i], timestamp);
	}

	for (index = 0; index < map_size; index++)
	{
		if (subtracted_map[index] == 1)
		{
			i = index/config.y_size;
			j = index % config.y_size;

			carmen_position_t position;
			position.x = i * config.resolution + config.x_origin;
			position.y = j * config.resolution + config.y_origin;

			if (distance_to_lane(position) > 4.0)
				continue;

			moving_obstacle_observation_t observation;

			fast_flood_fill(map, &config, &observation, i, j);

			if (observation.cell_vector.size() > 16 && observation.cell_vector.size() < 500)
			{
				compute_centroid(&observation);
				int val = associate_observations(observation);
				observation.timestamp = timestamp;

				if (val >= 0 && distance(moving_obstacle_list[val].position, observation.centroid) <= MAX_ASSOCIATION_DISTANCE)
				{
					moving_obstacle_list[val].observations.push_front(observation);
					moving_obstacle_list[val].associated = 1;
					moving_obstacle_list[val].age++;
					if (moving_obstacle_list[val].observations.size() > HISTORY_SIZE)
					{
						moving_obstacle_list[val].observations.pop_back();
					}
					moving_obstacle_list[val].position = observation.centroid;
					compute_velocity_and_orientation(&moving_obstacle_list[val]);
				}
				else
				{
					moving_obstacle_t obstacle;
					obstacle.id = current_id;
					obstacle.color = current_id % 10;
					current_id++;
					obstacle.associated = 0;
					obstacle.age = 0;
					obstacle.observations.push_front(observation);
					obstacle.velocity = 4.0;
					obstacle.orientation = globalpos.theta;
					obstacle.position = observation.centroid;
					moving_obstacle_list.push_back(obstacle);
				}
			}
		}
	}

	free(map);
}


void
translate_maps(char *previous_map, carmen_map_config_t previous_config, carmen_map_config_t current_config)
{
	int i, j, new_index, old_index;
	cell_coords_t map_cell;

	for (i = 0; i < previous_config.x_size; i++)
	{
		for (j = 0; j < previous_config.y_size; j++)
		{
			old_index = j + i * previous_config.y_size;
			map_cell = carmen_obstacle_distance_mapper_get_map_cell_from_configs(current_config, previous_config, i, j);
			if (map_cell.x >= 0 && map_cell.x < current_config.x_size &&
					map_cell.y >= 0 && map_cell.y < current_config.y_size)
			{
				new_index = map_cell.y + map_cell.x * current_config.y_size;
				previous_map[new_index] = previous_map[old_index];
			}
		}
	}

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
		dist = distance(globalposition, iter->position);
		timediff = timestamp - iter->observations[0].timestamp;

		if(dist > 70.0 || (iter->age > 1 && iter->associated == 0) || timediff > 0.3)
		{
			iter = moving_obstacle_list.erase(iter);
		}
		else
		{
			++iter;
		}
	}
}


void
plot_obstacles()
{
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w");
		fprintf(gnuplot_pipeMP, "set size ratio -1\n");
		fprintf(gnuplot_pipeMP, "set key outside\n");
		fprintf(gnuplot_pipeMP, "set xlabel 'x'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'y'\n");
	}

	FILE *gnuplot_tracked_points = fopen("gnuplot_tracked_points.txt", "w");

	double x_plot, y_plot;
	double first_x = 7757859.3;
	double first_y = -363559.8;

	for (unsigned int j = 0; j < moving_obstacle_list.size(); j++)
	{
		for (unsigned int i = 0; i < moving_obstacle_list[j].observations.size(); i++)
		{
			x_plot = moving_obstacle_list[j].observations[i].centroid.x - first_x;
			y_plot = moving_obstacle_list[j].observations[i].centroid.y - first_y;
			fprintf(gnuplot_tracked_points, "%lf %lf\n", x_plot, y_plot);
		}
	}

	fclose(gnuplot_tracked_points);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_tracked_points.txt' using 1:2 title 'tracked_points'\n");

	fflush(gnuplot_pipeMP);
}


cv::Scalar
compute_color(int color)
{
	switch (color)
	{
		case CARMEN_RED:
			return cv::Scalar(0,0,255);
		case CARMEN_BLUE:
			return cv::Scalar(255,0,0);
		case CARMEN_WHITE:
			return cv::Scalar(255,0,127);
		case CARMEN_YELLOW:
			return cv::Scalar(0,127,127);
		case CARMEN_GREEN:
			return cv::Scalar(0,127,0);
		case CARMEN_LIGHT_BLUE:
			return cv::Scalar(255,255,0);
		case CARMEN_BLACK:
			return cv::Scalar(0,0,0);
		case CARMEN_ORANGE:
			return cv::Scalar(0,127,255);
		case CARMEN_GREY:
			return cv::Scalar(66,66,66);
		case CARMEN_LIGHT_GREY:
			return cv::Scalar(127,127,127);
		default:
			return cv::Scalar(0,0,0);
	}
}


void
show_tracks(char *map, int map_id, carmen_map_config_t config)
{
	unsigned int size = config.x_size * config.y_size;
	unsigned char *map_char = (unsigned char *) malloc (3*size * sizeof(unsigned char));
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

		map_char[3*index] = (1 - map[i]) * 255;
		map_char[3*index + 1] = (1 - map[i]) * 255;
		map_char[3*index + 2] = (1 - map[i]) * 255;
	}


	// create a new opencv image
	cv::Mat img(width, height, CV_8UC3, map_char);
	cv::Mat dst;


	cv::Scalar color;

	int x, y;
	for (unsigned int i = 0; i < moving_obstacle_list.size(); i++)
	{
		color = compute_color(moving_obstacle_list[i].color);
		for (unsigned int j = 0; j < moving_obstacle_list[i].observations.size(); j++)
		{
			x = (moving_obstacle_list[i].observations[j].centroid.x - config.x_origin) / config.resolution;
			y = (moving_obstacle_list[i].observations[j].centroid.y - config.y_origin) / config.resolution;

			if (x >= 0 && x < config.x_size && y >= 0 && y < config.y_size)
			{
				// get the current row
				unsigned int row = (height - 1) - y;
				// get the current col
				unsigned int col = x;

				cv::circle(img, cv::Point(col, row), 4, color, 1);

			}
		}
	}

	x = (globalpos.x - config.x_origin) / config.resolution;
	y = (height - 1) - ((globalpos.y - config.y_origin) / config.resolution);

	cv::circle(img, cv::Point(x, y), 6, cv::Scalar(255, 0, 255), -1);

	cv::resize(img, dst, cv::Size(600,600));

	char map_name[10];
	sprintf(map_name, "Map %d", map_id);

	cv::imshow(map_name, dst);
	cv::waitKey(1);

	free (map_char);
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
publish_moving_objects(double timestamp)
{
	carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;
	moving_objects_point_clouds_message.num_point_clouds = moving_obstacle_list.size();
	moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(moving_objects_point_clouds_message.num_point_clouds * sizeof(t_point_cloud_struct)));

	for (int i = 0; i < moving_objects_point_clouds_message.num_point_clouds; i++)
	{
		moving_objects_point_clouds_message.point_clouds[i].r = 1.0;
		moving_objects_point_clouds_message.point_clouds[i].g = 1.0;
		moving_objects_point_clouds_message.point_clouds[i].b = 1.0;
		moving_objects_point_clouds_message.point_clouds[i].point_size = 0;
		moving_objects_point_clouds_message.point_clouds[i].linear_velocity = moving_obstacle_list[i].velocity;
		moving_objects_point_clouds_message.point_clouds[i].orientation = carmen_normalize_theta(moving_obstacle_list[i].orientation);
		moving_objects_point_clouds_message.point_clouds[i].object_pose.x = moving_obstacle_list[i].position.x;
		moving_objects_point_clouds_message.point_clouds[i].object_pose.y = moving_obstacle_list[i].position.y;
		moving_objects_point_clouds_message.point_clouds[i].object_pose.z = 0.0;
		moving_objects_point_clouds_message.point_clouds[i].height = 1.6;
		moving_objects_point_clouds_message.point_clouds[i].length = 1.6;
		moving_objects_point_clouds_message.point_clouds[i].width = 1.6;
		moving_objects_point_clouds_message.point_clouds[i].geometric_model = 0;
		moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.height = 1.6;
		moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.length = 1.6;
		moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.width = 1.6;
		moving_objects_point_clouds_message.point_clouds[i].model_features.red = 1.0;
		moving_objects_point_clouds_message.point_clouds[i].model_features.green = 0.0;
		moving_objects_point_clouds_message.point_clouds[i].model_features.blue = 0.8;
		moving_objects_point_clouds_message.point_clouds[i].model_features.model_name = (char *) "car";
		moving_objects_point_clouds_message.point_clouds[i].num_associated = moving_obstacle_list[i].id;

	}

	moving_objects_point_clouds_message.timestamp = timestamp;
	moving_objects_point_clouds_message.host = carmen_get_host();

	carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);
	free(moving_objects_point_clouds_message.point_clouds);
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
carmen_map_handler(carmen_mapper_map_message *map_message)
{
	static int first_map = 1;

	int map_size = map_message->size;

	if (first_map == 1)
	{
		current_map = (char *) malloc(map_size * sizeof(char));
		previous_map = (char *) malloc(map_size * sizeof(char));
	}

	memset(current_map, 0, map_size * sizeof(char));
	current_config = map_message->config;

	for (int i = 0; i < map_size; i++)
	{
		current_map[i] = map_message->complete_map[i] > 0.35 ? 1 : 0;
	}

	dilate_map(current_map, &current_config);

	if (!rddf_ready)
		return;

	if (first_map == 0)
	{
		char *subtracted_map = (char *) malloc(map_size * sizeof(char));

		if (current_config.x_origin == previous_config.x_origin && current_config.y_origin == previous_config.y_origin)
		{
			subtract_map(subtracted_map, current_map, previous_map, current_config);
		}
		else
		{
			translate_maps(previous_map, previous_config, current_config);
			subtract_map(subtracted_map, current_map, previous_map, current_config);
		}
		detect_obstacles(subtracted_map, current_map, current_config, map_message->timestamp);
		remove_obstacles(map_message->timestamp);
		free(subtracted_map);
	}

	publish_moving_objects(map_message->timestamp);

	#ifdef USE_OPEN_CV
		show_tracks(current_map, 1, current_config);
	#endif

	memcpy(previous_map, current_map, map_size * sizeof(char));
	previous_config = current_config;

	first_map = 0;
}


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
		current_map[index] = compact_map_message->value[i] > 0.5 ? 1 : 0;
	}

//	if (current_config.x_origin == offline_config.x_origin && current_config.y_origin == offline_config.y_origin)
//	{
//		subtract_map(current_map, current_map, offline_map, current_config);
//	}


	#ifdef USE_OPEN_CV
//		show_map(current_map, 2, compact_map_message->config);
//		show_map(offline_map, 1, offline_config);
	#endif

	if (!rddf_ready)
		return;

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
			//show_map(subtracted_map, 3, compact_map_message->config);
		#endif

		publish_moving_obstacles();

		free(subtracted_map);
	}

	#ifdef USE_OPEN_CV
		show_tracks(current_map, 1, current_config);
	#endif

	memcpy(previous_map, current_map, map_size * sizeof(char));
	previous_config = current_config;

	first = 0;

}


void
carmen_map_server_offline_map_message_handler(carmen_map_server_offline_map_message *offline_map_message)
{
	static int first_map = 1;

	int map_size = offline_map_message->size;

	if (first_map)
	{
		offline_map = (char *) malloc(map_size * sizeof(char));
		first_map = 0;
	}

	for (int i = 0; i < map_size; i++)
	{
		offline_map[i] = offline_map_message->complete_map[i] > 0.5 ? 1 : 0;
	}

	offline_config = offline_map_message->config;
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
	rddf_ready = 1;
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
//	carmen_map_server_subscribe_compact_cost_map(NULL,
//		(carmen_handler_t) carmen_compact_cost_map_handler,
//		CARMEN_SUBSCRIBE_LATEST);

//	carmen_map_server_subscribe_offline_map(NULL,
//		(carmen_handler_t) carmen_map_server_offline_map_message_handler,
//		CARMEN_SUBSCRIBE_LATEST);

	carmen_mapper_subscribe_map_message(NULL,
		(carmen_handler_t) carmen_map_handler,
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
	cv::namedWindow("Map 1", cv::WINDOW_AUTOSIZE);
//	cv::namedWindow("Map 2", cv::WINDOW_AUTOSIZE);
//	cv::namedWindow("Map 3", cv::WINDOW_AUTOSIZE);
	std::cout<<"OpenCV Version used:"<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<std::endl;
#endif
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	read_parameters(argc, argv);
	subscribe_messages();
	carmen_ipc_dispatch();

	return 0;
}

