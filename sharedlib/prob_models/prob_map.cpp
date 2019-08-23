#include <carmen/global.h>
#include <carmen/carmen.h>
#include <locale.h>
#include "prob_measurement_model.h"
#include "prob_map.h"
#include <vector>

#define MAX_LOG_ODDS_POSSIBLE	37.0


static ProbabilisticMapParams map_params;


// Convert from map to real-world coord
double
grid_to_map_x(int x)
{
	return map_params.grid_res * ((double) x);
}


double
grid_to_map_y(int y)
{
	return map_params.grid_res * ((double) y);
}


// Convert from real-world to map coord
int
map_to_grid_x(carmen_map_t *map, double x)
{
	int mx = round((x - map->config.x_origin) / map->config.resolution);

	return mx;
}


// Convert from real-world to map coord
int
map_to_grid_x(double x)
{
	int mx = round(x / map_params.grid_res);

	return mx;
}


int
map_to_grid_y(carmen_map_t *map, double y)
{
	int my = round((y - map->config.y_origin) / map->config.resolution);

	return my;
}


int
map_to_grid_y(double y)
{
	int my = round(y / map_params.grid_res);

	return my;
}


// Check that map coords are inside the array
int
map_grid_is_valid(carmen_map_t *map, int x, int y)
{
	if ((x >= 0) && (x < map->config.x_size) && (y >= 0) && (y < map->config.y_size))
		return 1;

	return 0;
}


// Check that map coords are inside the array
int
map_grid_is_valid(carmen_compact_map_t *map, int x, int y)
{
	if ((x >= 0) && (x < map->config.x_size) && (y >= 0) && (y < map->config.y_size))
		return 1;

	return 0;
}


// Check that map coords are inside the array
int
map_grid_is_valid(int x, int y)
{
	if ((x >= 0) && (x < map_params.grid_sx) && (y >= 0) && (y < map_params.grid_sy))
		return 1;

	return 0;
}


void
set_log_odds_map_cell(const ProbabilisticMap *map, int x, int y, double value)
{
	map->log_odds_map[x][y] = value;
}


double
carmen_prob_models_log_odds_to_probabilistic(double lt_i)
{
	return (1.0L - (1.0L / (1.0L + expl((long double) lt_i))));
}


double
carmen_prob_models_probabilistic_to_log_odds(double p_mi)
{
	double lt_i;
	static double min_log_odds_as_probability = carmen_prob_models_log_odds_to_probabilistic(-MAX_LOG_ODDS_POSSIBLE);
	static double max_log_odds_as_probability = carmen_prob_models_log_odds_to_probabilistic(MAX_LOG_ODDS_POSSIBLE);

	if (p_mi < 0.0) // unknown
		p_mi = 0.5;

	if (p_mi < min_log_odds_as_probability)
		lt_i = -MAX_LOG_ODDS_POSSIBLE;
	else if (p_mi < max_log_odds_as_probability)
		lt_i = logl((long double) p_mi / (1.0L - (long double) p_mi));
	else
		lt_i = MAX_LOG_ODDS_POSSIBLE;

	return (lt_i);
}


double
get_log_odds(double p_mi)
{
	double lt_i;
	static double min_log_odds_as_probability = carmen_prob_models_log_odds_to_probabilistic(-MAX_LOG_ODDS_POSSIBLE);
	static double max_log_odds_as_probability = carmen_prob_models_log_odds_to_probabilistic(MAX_LOG_ODDS_POSSIBLE);

	if (p_mi < 0.0) // unknown
		p_mi = 0.5;

	if (p_mi < min_log_odds_as_probability)
		lt_i = -MAX_LOG_ODDS_POSSIBLE;
	else if (p_mi < max_log_odds_as_probability)
		lt_i = logl((long double) p_mi / (1.0L - (long double) p_mi));
	else
		lt_i = MAX_LOG_ODDS_POSSIBLE;

	return (lt_i);
}


double
get_log_odds_map_cell(const carmen_map_t *map, int x, int y)
{
	return get_log_odds(map->map[x][y]);
}


int
count_number_of_known_point_on_the_map(carmen_map_t *map, double value)
{
	int i;
	int count = 0;
	int number_of_cells = map->config.x_size * map->config.y_size;

	for (i = 0; i < number_of_cells; i++)
	{
		if (map->complete_map[i] != value)
			count++;
	}

	return (count);
}


void
carmen_prob_models_create_compact_map_old(carmen_compact_map_t *cmap, carmen_map_t *map, double value)
{
	int i, k, N;
	int number_of_cells = map->config.x_size * map->config.y_size;

	N = count_number_of_known_point_on_the_map(map, value);

	cmap->config = map->config;
	cmap->number_of_known_points_on_the_map = N;

	if (N == 0)
		return;

	if (map->config.map_name != NULL)
	{
		cmap->config.map_name = (char *) calloc(strlen(map->config.map_name) + 1, sizeof(char));
		strcpy(cmap->config.map_name, map->config.map_name);
	}

	cmap->coord_x = (int *) malloc(cmap->number_of_known_points_on_the_map * sizeof(int));
	cmap->coord_y = (int *) malloc(cmap->number_of_known_points_on_the_map * sizeof(int));
	cmap->value = (double *) malloc(cmap->number_of_known_points_on_the_map * sizeof(double));

	for (i = 0, k = 0; i < number_of_cells; i++)
	{
		if (map->complete_map[i] != value)
		{
			cmap->coord_x[k] = i / map->config.x_size;
			cmap->coord_y[k] = i % map->config.y_size;
			cmap->value[k] = map->complete_map[i];
			k++;
		}
	}
}


void
carmen_prob_models_create_compact_map(carmen_compact_map_t *cmap, carmen_map_t *map, double value)
{
	int i, k;
	int number_of_cells = map->config.x_size * map->config.y_size;

	cmap->config = map->config;

	if (map->config.map_name != NULL)
	{
		cmap->config.map_name = (char *) calloc(strlen(map->config.map_name) + 1, sizeof(char));
		strcpy(cmap->config.map_name, map->config.map_name);
	}

	cmap->coord_x = (int *) malloc(number_of_cells * sizeof(int));
	cmap->coord_y = (int *) malloc(number_of_cells * sizeof(int));
	cmap->value = (double *) malloc(number_of_cells * sizeof(double));

	for (i = 0, k = 0; i < number_of_cells; i++)
	{
		if (map->complete_map[i] != value)
		{
			cmap->coord_x[k] = i / map->config.x_size;
			cmap->coord_y[k] = i % map->config.y_size; // nao deveria ser x_size??
			cmap->value[k] = map->complete_map[i];
			k++;
		}
	}
	cmap->number_of_known_points_on_the_map = k;
}


void
carmen_prob_models_initialize_cost_map(carmen_map_t *cost_map, carmen_map_config_t config, double resolution)
{
	cost_map->config.resolution = resolution;
	cost_map->config.x_origin = config.x_origin;
	cost_map->config.y_origin = config.y_origin;

	if (((cost_map->config.x_size * cost_map->config.resolution) != (config.x_size * config.resolution)) ||
		((cost_map->config.y_size * cost_map->config.resolution) != (config.y_size * config.resolution)))
	{
		if (cost_map->complete_map != NULL)
			free(cost_map->complete_map);

		if (cost_map->map != NULL)
			free(cost_map->map);

		cost_map->complete_map = NULL;
		cost_map->map = NULL;

		cost_map->config = config;
		cost_map->config.resolution = resolution;
		cost_map->config.x_size = (config.x_size * config.resolution) / cost_map->config.resolution;
		cost_map->config.y_size = (config.y_size * config.resolution) / cost_map->config.resolution;
	}

	if (cost_map->complete_map == NULL)
	{
		cost_map->complete_map = (double *) calloc(sizeof(double), cost_map->config.x_size * cost_map->config.y_size);
		carmen_test_alloc(cost_map->complete_map);
		cost_map->map = (double **) calloc(cost_map->config.x_size, sizeof(double *));
		carmen_test_alloc(cost_map->map);
		for (int i = 0; i < cost_map->config.x_size; i++)
			cost_map->map[i] = cost_map->complete_map + i * cost_map->config.y_size;
	}
}


void
carmen_prob_models_build_obstacle_cost_map(carmen_map_t *cost_map, carmen_map_t *map, double resolution, double obstacle_cost_distance, double obstacle_probability_threshold)
{
	cost_map->config.resolution = resolution;
	carmen_prob_models_initialize_cost_map(cost_map, map->config, resolution);
	carmen_prob_models_convert_obstacles_map_to_cost_map(cost_map, map, obstacle_probability_threshold, obstacle_cost_distance, 0);
}


void
carmen_prob_models_clear_carmen_map_using_compact_map(carmen_map_t *map, carmen_compact_map_t *cmap, double value)
{
	for (int i = 0; i < cmap->number_of_known_points_on_the_map; i++)
		map->map[cmap->coord_x[i]][cmap->coord_y[i]] = value;
}


void
carmen_prob_models_uncompress_compact_map(carmen_map_t *map, carmen_compact_map_t *cmap)
{
	for (int i = 0; i < cmap->number_of_known_points_on_the_map; i++)
		map->map[cmap->coord_x[i]][cmap->coord_y[i]] = cmap->value[i];
}


void
carmen_prob_models_free_compact_map(carmen_compact_map_t *map)
{
	if (map->coord_x != NULL)
		free(map->coord_x);

	if (map->coord_y != NULL)
		free(map->coord_y);

	if (map->value != NULL)
		free(map->value);

	if (map->config.map_name != NULL)
		free(map->config.map_name);

	map->value = NULL;
	map->coord_x = NULL;
	map->coord_y = NULL;
	map->config.map_name = NULL;
	map->number_of_known_points_on_the_map = 0;
}


#define INF 1E20
#define square(x) ((x)*(x))

/* dt of 1d function using squared distance */
static float *
dt(float *f, int n)
{
	float *d = new float[n];
	int *v = new int[n];
	float *z = new float[n + 1];
	int k = 0;

	v[0] = 0;
	z[0] = -INF;
	z[1] = +INF;
	for (int q = 1; q <= n - 1; q++)
	{
		float s = ((f[q] + square(q)) - (f[v[k]] + square(v[k]))) / (2 * q - 2 * v[k]);
		while (s <= z[k])
		{
			k--;
			s = ((f[q] + square(q)) - (f[v[k]] + square(v[k]))) / (2 * q - 2 * v[k]);
		}
		k++;
		v[k] = q;
		z[k] = s;
		z[k + 1] = +INF;
	}

	k = 0;
	for (int q = 0; q <= n - 1; q++)
	{
		while (z[k + 1] < q)
			k++;
		d[q] = square(q - v[k]) + f[v[k]];
	}

	delete[] v;
	delete[] z;

	return d;
}


/* dt of 2d function using squared distance */
// http://cs.brown.edu/~pff/dt/
void
carmen_prob_models_convert_obstacles_map_to_cost_map(carmen_map_t *cost_map, carmen_map_t *map, double occupancy_threshold,
		double distance_for_zero_cost_in_meters, int invert_map)
{
	int width = cost_map->config.x_size;
	int height = cost_map->config.y_size;

	float *f = new float[std::max(width, height)];
	float distance_for_zero_cost_in_pixels = distance_for_zero_cost_in_meters / cost_map->config.resolution;

	// transform along columns
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			int xi = carmen_clamp(0, round((x / map->config.resolution) * cost_map->config.resolution), map->config.x_size - 1);
			int yi = carmen_clamp(0, round((y / map->config.resolution) * cost_map->config.resolution), map->config.y_size - 1);
			f[y] = (map->map[xi][yi] > occupancy_threshold)? 0.0: INF;
		}
		float *d = dt(f, height);
		for (int y = 0; y < height; y++)
			cost_map->map[x][y] = d[y];

		delete[] d;
	}

	// transform along rows
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
			f[x] = cost_map->map[x][y];

		float *d = dt(f, width);
		for (int x = 0; x < width; x++)
		{
			float distance = sqrt(d[x]);
			if (invert_map)
				cost_map->map[x][y] = (distance < distance_for_zero_cost_in_pixels)? (distance / distance_for_zero_cost_in_pixels): 1.0;
			else
				cost_map->map[x][y] = (distance > distance_for_zero_cost_in_pixels)? 0.0: 1.0 - (distance / distance_for_zero_cost_in_pixels);
		}
		delete[] d;
	}

	delete[] f;
}


void
carmen_prob_models_log_odds_occupancy_grid_mapping(carmen_map_t *map, int xi, int yi, double inverse_sensor_model_value)
{
	double lt_i;

	lt_i = get_log_odds(map->map[xi][yi]);

	lt_i = lt_i + inverse_sensor_model_value;

	map->map[xi][yi] = carmen_prob_models_log_odds_to_probabilistic(lt_i);
}


void
carmen_prob_models_log_odds_occupancy_grid_mapping(carmen_map_t *map, int i, double inverse_sensor_model_log_odds_value)
{
	double lt_i;

	lt_i = get_log_odds(map->complete_map[i]);

	lt_i = lt_i + inverse_sensor_model_log_odds_value;

	map->complete_map[i] = carmen_prob_models_log_odds_to_probabilistic(lt_i);
}


void
carmen_prob_models_occupancy_grid_mapping_log_odds_only(carmen_map_t *log_odds_map, int xi, int yi, double inverse_sensor_model_log_odds_value)
{
	double lt_i;

	lt_i = log_odds_map->map[xi][yi];

	lt_i = lt_i + inverse_sensor_model_log_odds_value;

	log_odds_map->map[xi][yi] = lt_i;
}


void
carmen_prob_models_clean_carmen_map(carmen_map_t *map)
{
	int i;

	for (i = 0; i < map->config.x_size * map->config.y_size; i++)
	{
		map->complete_map[i] = -1.0;
	}
}

void
carmen_prob_models_update_log_odds_of_nearest_target(carmen_map_t *map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, int thread_id)
{
	cell_coords_t cell_hit_by_nearest_ray;
	cell_hit_by_nearest_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / map->config.resolution);
	cell_hit_by_nearest_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / map->config.resolution);

	if (sensor_data->obstacle_height[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]] < 0.5)
		return;

	if (map_grid_is_valid(map, cell_hit_by_nearest_ray.x, cell_hit_by_nearest_ray.y))
	{
		if (!sensor_data->maxed[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]] &&
			!sensor_data->ray_hit_the_robot[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]] &&
			!(carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]], highest_sensor, safe_range_above_sensors)))
			carmen_prob_models_log_odds_occupancy_grid_mapping(map, cell_hit_by_nearest_ray.x, cell_hit_by_nearest_ray.y, 2.0 * sensor_params->log_odds.log_odds_occ);
	}
}


void
carmen_prob_models_update_log_odds_of_cells_hit_by_rays(carmen_map_t *log_odds_map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, int thread_id, double safe_height_from_ground)
{
	int i;
	cell_coords_t cell_hit_by_ray, cell_hit_by_nearest_ray;
	double log_odds_of_the_cell_hit_by_the_ray_that_hit_the_nearest_target = 0.0;

	cell_hit_by_nearest_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / log_odds_map->config.resolution);
	cell_hit_by_nearest_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / log_odds_map->config.resolution);

	if (map_grid_is_valid(log_odds_map, cell_hit_by_nearest_ray.x, cell_hit_by_nearest_ray.y))
		log_odds_of_the_cell_hit_by_the_ray_that_hit_the_nearest_target = log_odds_map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y];
		
	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (i != sensor_data->ray_that_hit_the_nearest_target[thread_id])
		{
			cell_hit_by_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][i].x / log_odds_map->config.resolution);
			cell_hit_by_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][i].y / log_odds_map->config.resolution);
			if (map_grid_is_valid(log_odds_map, cell_hit_by_ray.x, cell_hit_by_ray.y))
				if (sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] != sensor_params->log_odds.log_odds_l0)
					carmen_prob_models_occupancy_grid_mapping_log_odds_only(log_odds_map, cell_hit_by_ray.x, cell_hit_by_ray.y, sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i]);
		}
	}

	if (map_grid_is_valid(log_odds_map, cell_hit_by_nearest_ray.x, cell_hit_by_nearest_ray.y))
	{
		log_odds_map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y] = log_odds_of_the_cell_hit_by_the_ray_that_hit_the_nearest_target;

		if (!sensor_data->maxed[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]] &&
			!sensor_data->ray_hit_the_robot[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]] &&
			!(carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]], highest_sensor, safe_range_above_sensors, safe_height_from_ground)))
			carmen_prob_models_occupancy_grid_mapping_log_odds_only(log_odds_map, cell_hit_by_nearest_ray.x, cell_hit_by_nearest_ray.y, 2.0 * sensor_params->log_odds.log_odds_occ);
	}
}


void
carmen_prob_models_update_sum_and_count_of_cells_hit_by_rays(carmen_map_t *map, carmen_map_t *sum_occupancy_map, carmen_map_t *count_occupancy_map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, int thread_id, double safe_height_from_ground)
{
	int i;
	cell_coords_t cell_hit_by_ray, cell_hit_by_nearest_ray;
	double log_odds_of_the_cell_hit_by_the_ray_that_hit_the_nearest_target = 0.0;

	cell_hit_by_nearest_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / map->config.resolution);
	cell_hit_by_nearest_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / map->config.resolution);

	if (map_grid_is_valid(map, cell_hit_by_nearest_ray.x, cell_hit_by_nearest_ray.y))
		log_odds_of_the_cell_hit_by_the_ray_that_hit_the_nearest_target = map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y];

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (i != sensor_data->ray_that_hit_the_nearest_target[thread_id])
		{
			cell_hit_by_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][i].x / map->config.resolution);
			cell_hit_by_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][i].y / map->config.resolution);
			if (map_grid_is_valid(map, cell_hit_by_ray.x, cell_hit_by_ray.y))
				if (sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] > sensor_params->log_odds.log_odds_l0)
				{
					if(count_occupancy_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] == -1.0)
					{
						count_occupancy_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] = 0.0;
						sum_occupancy_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] = 0.0;
					}
					carmen_prob_models_log_odds_occupancy_grid_mapping(map, cell_hit_by_ray.x, cell_hit_by_ray.y, sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i]);

					sum_occupancy_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] += carmen_prob_models_log_odds_to_probabilistic(sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i]); //map->map[cell_hit_by_ray.x][cell_hit_by_ray.y];
					count_occupancy_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] += 1.0;
				}
		}
	}

	if (map_grid_is_valid(map, cell_hit_by_nearest_ray.x, cell_hit_by_nearest_ray.y))
	{
		map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y] = log_odds_of_the_cell_hit_by_the_ray_that_hit_the_nearest_target;

		if (!sensor_data->maxed[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]] &&
			!sensor_data->ray_hit_the_robot[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]] &&
			!(carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]], highest_sensor, safe_range_above_sensors, safe_height_from_ground)))
		{
			if (count_occupancy_map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y] == -1.0)
			{
				count_occupancy_map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y] = 0.0;
				sum_occupancy_map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y] = 0.0;
			}
			carmen_prob_models_log_odds_occupancy_grid_mapping(map, cell_hit_by_nearest_ray.x, cell_hit_by_nearest_ray.y, 2.0 * sensor_params->log_odds.log_odds_occ);

			sum_occupancy_map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y] += carmen_prob_models_log_odds_to_probabilistic(2.0 * sensor_params->log_odds.log_odds_occ); //map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y];
			count_occupancy_map->map[cell_hit_by_nearest_ray.x][cell_hit_by_nearest_ray.y] += 1.0;
		}
	}
}


void
carmen_prob_models_get_maximum_probability_of_cells_hit_by_rays(carmen_map_t *map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id)
{
	int i;
	cell_coords_t cell_hit_by_ray;
	double probability_from_log_odds;

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] > sensor_params->log_odds.log_odds_l0)
		{
			cell_hit_by_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][i].x / map->config.resolution);
			cell_hit_by_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][i].y / map->config.resolution);

			if (map_grid_is_valid(map, cell_hit_by_ray.x, cell_hit_by_ray.y))
			{
				probability_from_log_odds = carmen_prob_models_log_odds_to_probabilistic(sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i]);

				if (map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] < probability_from_log_odds)
					map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] = probability_from_log_odds;
			}
		}
	}
}


void
carmen_prob_models_update_log_odds_of_cells_hit_by_ldmrs_rays(carmen_map_t *log_odds_map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id)
{
	int i;
	cell_coords_t cell_hit_by_ray;

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		cell_hit_by_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][i].x / log_odds_map->config.resolution);
		cell_hit_by_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][i].y / log_odds_map->config.resolution);

		if (map_grid_is_valid(log_odds_map, cell_hit_by_ray.x, cell_hit_by_ray.y) && !sensor_data->maxed[thread_id][i])// &&
//			(sensor_data->obstacle_height[thread_id][i] > 0.0) && (sensor_data->obstacle_height[thread_id][i] < 4.0) &&
//			(sensor_data->ray_size_in_the_floor[thread_id][i] > 13.0))
			carmen_prob_models_occupancy_grid_mapping_log_odds_only(log_odds_map, cell_hit_by_ray.x, cell_hit_by_ray.y, sensor_params->log_odds.log_odds_occ);
	}
}


static int
get_distance_index(double distance)
{	// Gera um indice para cada faixa de distancias. As faixas crescem de ~50% a cada intervalo. Indices de zero a 9 permitem distancias de zero a ~70 metros
	if (distance < 3.5)
		return (0);

	int distance_index = (int) ((log(distance - 3.5 + 1.0) / log(1.45)) +  0.5);
	if (distance_index > 9)
		return (9);

	return (distance_index);
}


float ***
load_calibration_table(char *calibration_file)
{
	FILE *calibration_file_bin = fopen(calibration_file, "r");
	if (!calibration_file_bin)
		return (NULL);

	float ***table;
	table = (float ***) calloc(32, sizeof(float **));
	for (int i = 0; i < 32; i++)
	{
		table[i] = (float **) calloc(10, sizeof(float *));
		for (int j = 0; j < 10; j++)
			table[i][j] = (float *) calloc(256, sizeof(float));
	}
	int laser, ray_size, intensity;
	long accumulated_intennsity, count;
	float val, max_val = 0.0, min_val = 255.0;
	while (fscanf(calibration_file_bin, "%d %d %d %f %ld %ld", &laser, &ray_size, &intensity, &val, &accumulated_intennsity, &count) == 6)
	{
		table[laser][ray_size][intensity] = val;
		if (val > max_val)
			max_val = val;
		if (val < min_val)
			min_val = val;
	}

	for (int i = 0; i < 32; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			for (int k = 0; k < 256; k++)
			{
				val = table[i][j][k];
				val = (val - min_val) / (max_val - min_val);
				if (val > 1.0)
					val = 1.0;
				if (val < 0.0)
					val = 0.0;
				table[i][j][k] = val;
			}
		}
	}
	fclose(calibration_file_bin);

	return (table);
}


static void
update_intensity_of_cells(carmen_map_t *sum_remission_map,
		carmen_map_t *sum_sqr_remission_map, carmen_map_t *count_remission_map,
		sensor_data_t *sensor_data, sensor_parameters_t *sensor_params,
		double highest_sensor, double safe_range_above_sensors, int ray, cell_coords_t *map_cells_hit_by_rays, int thread_id, double safe_height_from_ground = 0.0)
{
	cell_coords_t cell_hit_by_ray;

	cell_hit_by_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][ray].x / sum_remission_map->config.resolution);
	cell_hit_by_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][ray].y / sum_remission_map->config.resolution);

	if (map_grid_is_valid(sum_remission_map, cell_hit_by_ray.x, cell_hit_by_ray.y) &&
		!sensor_data->maxed[thread_id][ray] && !sensor_data->ray_hit_the_robot[thread_id][ray] &&
		!(carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][ray], highest_sensor, safe_range_above_sensors, safe_height_from_ground)))
	{
		if (map_cells_hit_by_rays != NULL)
		{
			map_cells_hit_by_rays->x = cell_hit_by_ray.x;
			map_cells_hit_by_rays->y = cell_hit_by_ray.y;
		}

		int distance_index = get_distance_index(sensor_data->ray_size_in_the_floor[thread_id][ray]);
		int intensity = (int) round(sensor_data->processed_intensity[thread_id][ray] * 255.0);
		if (sensor_params->save_calibration_file)
			fprintf(sensor_params->save_calibration_file, "%ld %ld %04d %04d %02d %d %03d\n",
					(long int) sum_remission_map->config.x_origin, (long int) sum_remission_map->config.y_origin,
					cell_hit_by_ray.x, cell_hit_by_ray.y,
					ray, distance_index, intensity);

		double calibrated_intensity;
		if (sensor_params->calibration_table)
		{
			calibrated_intensity = sensor_params->calibration_table[ray][distance_index][intensity];
			if (calibrated_intensity == 0.0) // invalid table entry
				return;
		}
		else
			calibrated_intensity = sensor_data->processed_intensity[thread_id][ray];

		if (sum_remission_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] < 0.0)
		{
			sum_sqr_remission_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] = calibrated_intensity * calibrated_intensity;
			sum_remission_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] = calibrated_intensity;
			count_remission_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] = 1.0;
		}
		else
		{
			sum_sqr_remission_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] += calibrated_intensity * calibrated_intensity;
			sum_remission_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] += calibrated_intensity;
			count_remission_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] += 1.0;
		}
	}
}


static void
update_intensity_of_cells(carmen_compact_map_t *mean_remission_compact_map,
		sensor_data_t *sensor_data, sensor_parameters_t *sensor_params,
		double highest_sensor, double safe_range_above_sensors, int ray, cell_coords_t *map_cells_hit_by_rays, int thread_id, double safe_height_from_ground = 0.0)
{
	cell_coords_t cell_hit_by_ray;

	cell_hit_by_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][ray].x / mean_remission_compact_map->config.resolution);
	cell_hit_by_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][ray].y / mean_remission_compact_map->config.resolution);

	if (map_grid_is_valid(mean_remission_compact_map, cell_hit_by_ray.x, cell_hit_by_ray.y) &&
		!sensor_data->maxed[thread_id][ray] && !sensor_data->ray_hit_the_robot[thread_id][ray] &&
		!(carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][ray], highest_sensor, safe_range_above_sensors, safe_height_from_ground)))
	{
		if (map_cells_hit_by_rays != NULL)
		{
			map_cells_hit_by_rays->x = cell_hit_by_ray.x;
			map_cells_hit_by_rays->y = cell_hit_by_ray.y;
		}

		int distance_index = get_distance_index(sensor_data->ray_size_in_the_floor[thread_id][ray]);
		int intensity = (int) round(sensor_data->processed_intensity[thread_id][ray] * 255.0);
		if (sensor_params->save_calibration_file)
			fprintf(sensor_params->save_calibration_file, "%ld %ld %04d %04d %02d %d %03d\n",
					(long int) mean_remission_compact_map->config.x_origin, (long int) mean_remission_compact_map->config.y_origin,
					cell_hit_by_ray.x, cell_hit_by_ray.y,
					ray, distance_index, intensity);

		double calibrated_intensity;
		if (sensor_params->calibration_table)
		{
			calibrated_intensity = sensor_params->calibration_table[ray][distance_index][intensity];
			if (calibrated_intensity == 0.0) // invalid table entry
				return;
		}
		else
			calibrated_intensity = sensor_data->processed_intensity[thread_id][ray];

		if (mean_remission_compact_map->number_of_known_points_on_the_map > 1)
		{
			if ((mean_remission_compact_map->coord_x[mean_remission_compact_map->number_of_known_points_on_the_map - 1] == cell_hit_by_ray.x) &&
				(mean_remission_compact_map->coord_y[mean_remission_compact_map->number_of_known_points_on_the_map - 1] == cell_hit_by_ray.y))
				mean_remission_compact_map->number_of_known_points_on_the_map--; // Sobreescreve celulas revisitadas em seguida
		}
		mean_remission_compact_map->coord_x[mean_remission_compact_map->number_of_known_points_on_the_map] = cell_hit_by_ray.x;
		mean_remission_compact_map->coord_y[mean_remission_compact_map->number_of_known_points_on_the_map] = cell_hit_by_ray.y;
		mean_remission_compact_map->value[mean_remission_compact_map->number_of_known_points_on_the_map] = calibrated_intensity;
		mean_remission_compact_map->number_of_known_points_on_the_map++;
	}
}


carmen_map_t *
carmen_prob_models_check_if_new_snapshot_map_allocation_is_needed(carmen_map_t *snapshot_map, carmen_map_t *current_map)
{
	if (snapshot_map == NULL)
	{
		snapshot_map = carmen_map_interface_create_new_empty_map(&(current_map->config));
	}
	else if ((snapshot_map->config.x_size != current_map->config.x_size) ||
		 (snapshot_map->config.y_size != current_map->config.y_size))
	{
		carmen_map_destroy(&snapshot_map);
		snapshot_map = carmen_map_interface_create_new_empty_map(&(current_map->config));
	}
	return (snapshot_map);
}


carmen_map_t *
carmen_prob_models_check_if_new_log_odds_snapshot_map_allocation_is_needed(carmen_map_t *log_odds_snapshot_map, carmen_map_t *current_map)
{
	if (log_odds_snapshot_map == NULL)
	{
		log_odds_snapshot_map = carmen_map_interface_create_new_empty_log_odds_map(&(current_map->config));
	}
	else if ((log_odds_snapshot_map->config.x_size != current_map->config.x_size) ||
		 (log_odds_snapshot_map->config.y_size != current_map->config.y_size))
	{
		carmen_map_destroy(&log_odds_snapshot_map);
		log_odds_snapshot_map = carmen_map_interface_create_new_empty_log_odds_map(&(current_map->config));
	}
	return (log_odds_snapshot_map);
}


void
carmen_prob_models_update_intensity_of_cells_hit_by_rays(carmen_map_t *sum_remission_map, carmen_map_t *sum_sqr_remission_map,
		carmen_map_t *count_remission_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor,
		double safe_range_above_sensors, cell_coords_t *map_cells_hit_by_each_rays, int thread_id, double safe_height_from_ground)
{
	int i;

	if (map_cells_hit_by_each_rays != NULL)
		memset(map_cells_hit_by_each_rays, 0, sensor_params->vertical_resolution * sizeof(cell_coords_t));

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (map_cells_hit_by_each_rays != NULL)
			update_intensity_of_cells(sum_remission_map, sum_sqr_remission_map, count_remission_map, sensor_data, sensor_params, highest_sensor, safe_range_above_sensors, i, &map_cells_hit_by_each_rays[i], thread_id, safe_height_from_ground);
		else
			update_intensity_of_cells(sum_remission_map, sum_sqr_remission_map, count_remission_map, sensor_data, sensor_params, highest_sensor, safe_range_above_sensors, i, NULL, thread_id, safe_height_from_ground);
	}
}


void
carmen_prob_models_add_intensity_of_cells_hit_by_rays(carmen_compact_map_t *mean_remission_compact_map,
		sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor,
		double safe_range_above_sensors, cell_coords_t *map_cells_hit_by_each_rays, int thread_id)
{
	int i;

	if (map_cells_hit_by_each_rays != NULL)
		memset(map_cells_hit_by_each_rays, 0, sensor_params->vertical_resolution * sizeof(cell_coords_t));

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (map_cells_hit_by_each_rays != NULL)
			update_intensity_of_cells(mean_remission_compact_map, sensor_data, sensor_params, highest_sensor, safe_range_above_sensors, i, &map_cells_hit_by_each_rays[i], thread_id);
		else
			update_intensity_of_cells(mean_remission_compact_map, sensor_data, sensor_params, highest_sensor, safe_range_above_sensors, i, NULL, thread_id);
	}
}


void
carmen_prob_models_update_intensity_of_cells_hit_by_rays_for_calibration(carmen_map_t *sum_remission_map,
		carmen_map_t *sum_sqr_remission_map, carmen_map_t *count_remission_map, carmen_map_t *remission_map,
		sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, int thread_id)
{
	int i, j;
	cell_coords_t cell_hit_by_ray;

	for(j = 0; j < sensor_params->vertical_resolution; j++)
	{

		for (i = 0; i < sensor_params->vertical_resolution; i++)
		{
			if (i != j)
				update_intensity_of_cells(&sum_remission_map[j], &sum_sqr_remission_map[j], &count_remission_map[j], sensor_data, sensor_params, highest_sensor, safe_range_above_sensors, i, NULL, thread_id);
		}
		cell_hit_by_ray.x = round(sensor_data->ray_position_in_the_floor[thread_id][j].x / sum_remission_map->config.resolution);
		cell_hit_by_ray.y = round(sensor_data->ray_position_in_the_floor[thread_id][j].y / sum_remission_map->config.resolution);

		if (map_grid_is_valid(sum_remission_map, cell_hit_by_ray.x, cell_hit_by_ray.y) &&
				!sensor_data->maxed[thread_id][j] && !sensor_data->ray_hit_the_robot[thread_id][j] &&
				!(carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][j], highest_sensor, safe_range_above_sensors)))
		{
			remission_map[j].map[cell_hit_by_ray.x][cell_hit_by_ray.y] = 255.0 * sensor_data->processed_intensity[thread_id][j];
		}
	}
}


void
carmen_prob_models_calc_mean_and_variance_remission_map(carmen_map_t *mean_remission_map, carmen_map_t *variance_remission_map, carmen_map_t *sum_remission_map, carmen_map_t *sum_sqr_remission_map __attribute__ ((unused)), carmen_map_t *count_remission_map)
{
	int i = 0;
	double mean = 0.0, variance = 0.0;

	mean_remission_map->config = sum_remission_map->config;
	variance_remission_map->config = sum_remission_map->config;

	for (i = 0; i < sum_remission_map->config.x_size * sum_remission_map->config.y_size; i++)
	{
		if (sum_remission_map->complete_map[i] > 0.0)
		{
			mean = sum_remission_map->complete_map[i] / count_remission_map->complete_map[i];
			mean_remission_map->complete_map[i] = mean;

			// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
			variance = sum_sqr_remission_map->complete_map[i] - ((sum_remission_map->complete_map[i] * sum_remission_map->complete_map[i]) / count_remission_map->complete_map[i]);
			variance /= count_remission_map->complete_map[i];
			variance_remission_map->complete_map[i] = variance;
		}
		else
		{
			mean_remission_map->complete_map[i] = -1.0;
			variance_remission_map->complete_map[i] = -1.0;
		}
	}
}


void
carmen_prob_models_calc_mean_remission_map(carmen_map_t *mean_remission_map, carmen_map_t *sum_remission_map, carmen_map_t *count_remission_map)
{
	int i = 0;
	double mean = 0.0;

	mean_remission_map->config = sum_remission_map->config;

	for (i = 0; i < sum_remission_map->config.x_size * sum_remission_map->config.y_size; i++)
	{
		if (sum_remission_map->complete_map[i] > 0.0)
		{
			mean = sum_remission_map->complete_map[i] / count_remission_map->complete_map[i];
			mean_remission_map->complete_map[i] = mean;
		}
		else
			mean_remission_map->complete_map[i] = -1.0;
	}
}


void
carmen_prob_models_update_cells_crossed_by_ray(carmen_map_t *map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id)
{
	int j;
	cell_coords_t a, b;
	double dx, dy, dr;
	int step_count;
	int nx, ny;
//	int ray_start_occupied = 0;
	
//	if (sensor_data->maxed[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]])
//		return;

	a.x = round(sensor_data->ray_origin_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / map->config.resolution);
	a.y = round(sensor_data->ray_origin_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / map->config.resolution);
	b.x = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / map->config.resolution);
	b.y = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / map->config.resolution);

	// Compute line parameters
	dx = (double) (b.x - a.x);
	dy = (double) (b.y - a.y);
	dr = sqrt(dx * dx + dy * dy);

	dx /= dr;
	dy /= dr;
	step_count = (int) round(dr);

	// Walk the line and update the grid
	for (j = 0; j < step_count - 2; j++)
	{
		nx = (int) round(a.x + dx * (double) j);
		ny = (int) round(a.y + dy * (double) j);

		if (map_grid_is_valid(map, nx, ny) && !((nx == b.x) && (ny == b.y)))
		{
//			if ((j < 8) && (map->map[nx][ny] > 0.85)) // Alberto: estes numeros sao bem ad hoc...
//				ray_start_occupied = 1;
//			if (ray_start_occupied && (map->map[nx][ny] <= 0.85))
//				ray_start_occupied = 0;
//			if (ray_start_occupied == 0)
				carmen_prob_models_log_odds_occupancy_grid_mapping(map, nx, ny, sensor_params->log_odds.log_odds_free);
			if (map->map[nx][ny] >= 0.5)
				break;	// do not cross obstacles until they are cleared
		}
	}
}


void
carmen_prob_models_update_log_odds_of_cells_crossed_by_ray(carmen_map_t *log_odds_map, sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, int thread_id)
{
	int j;
	cell_coords_t a, b;
	double dx, dy, dr;
	int step_count;
	int nx, ny;
//	int ray_start_occupied = 0;

//	if (sensor_data->maxed[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]])
//		return;

	a.x = round(sensor_data->ray_origin_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / log_odds_map->config.resolution);
	a.y = round(sensor_data->ray_origin_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / log_odds_map->config.resolution);
	b.x = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / log_odds_map->config.resolution);
	b.y = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / log_odds_map->config.resolution);

	// Compute line parameters
	dx = (double) (b.x - a.x);
	dy = (double) (b.y - a.y);
	dr = sqrt(dx * dx + dy * dy);

	dx /= dr;
	dy /= dr;
	step_count = (int) round(dr);

	// Walk the line and update the grid
	for (j = 0; j < step_count - 2; j++)
	{
		nx = (int) round(a.x + dx * (double) j);
		ny = (int) round(a.y + dy * (double) j);

		if (map_grid_is_valid(log_odds_map, nx, ny) && !((nx == b.x) && (ny == b.y)))
		{
//			if ((j < 8) && (map->map[nx][ny] > 0.85)) // Alberto: estes numeros sao bem ad hoc...
//				ray_start_occupied = 1;
//			if (ray_start_occupied && (log_odds_map->log_odds_map[nx][ny] <= 0.85))
//				ray_start_occupied = 0;
//			if (ray_start_occupied == 0)
			carmen_prob_models_occupancy_grid_mapping_log_odds_only(log_odds_map, nx, ny, sensor_params->log_odds.log_odds_free);
			if (log_odds_map->map[nx][ny] >= sensor_params->log_odds.log_odds_l0)
				break;	// do not cross obstacles until they are cleared
		}
	}
}


void
carmen_prob_models_update_sum_and_count_cells_crossed_by_ray(carmen_map_t *map, carmen_map_t *sum_occupancy_map, carmen_map_t *count_occupancy_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id)
{
	int j;
	cell_coords_t a, b;
	double dx, dy, dr;
	int step_count;
	int nx, ny;
//	int ray_start_occupied = 0;

//	if (sensor_data->maxed[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]])
//		return;

	a.x = round(sensor_data->ray_origin_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / map->config.resolution);
	a.y = round(sensor_data->ray_origin_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / map->config.resolution);
	b.x = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].x / map->config.resolution);
	b.y = round(sensor_data->ray_position_in_the_floor[thread_id][sensor_data->ray_that_hit_the_nearest_target[thread_id]].y / map->config.resolution);

	// Compute line parameters
	dx = (double) (b.x - a.x);
	dy = (double) (b.y - a.y);
	dr = sqrt(dx * dx + dy * dy);

	dx /= dr;
	dy /= dr;
	step_count = (int) round(dr);

	// Walk the line and update the grid
	for (j = 0; j < step_count - 2; j++)
	{
		nx = (int) round(a.x + dx * (double) j);
		ny = (int) round(a.y + dy * (double) j);

		if (map_grid_is_valid(map, nx, ny) && !((nx == b.x) && (ny == b.y)))
		{
//			if ((j < 8) && (map->map[nx][ny] > 0.85)) // Alberto: estes numeros sao bem ad hoc...
//				ray_start_occupied = 1;
//			if (ray_start_occupied && (map->map[nx][ny] <= 0.85))
//				ray_start_occupied = 0;
//			if (ray_start_occupied == 0)
			if (count_occupancy_map->map[nx][ny] == -1.0)
			{
				count_occupancy_map->map[nx][ny] = 0.0;
				sum_occupancy_map->map[nx][ny] = 0.0;
			}
			carmen_prob_models_log_odds_occupancy_grid_mapping(map, nx, ny, sensor_params->log_odds.log_odds_free);
			sum_occupancy_map->map[nx][ny] += carmen_prob_models_log_odds_to_probabilistic(sensor_params->log_odds.log_odds_free);
			count_occupancy_map->map[nx][ny] += 1.0;
			if (map->map[nx][ny] >= 0.5)
				break;	// do not cross obstacles until they are cleared
		}
	}
}


void
carmen_prob_models_updade_cells_bellow_robot(carmen_point_t pose, carmen_map_t *map, double prob, carmen_robot_ackerman_config_t *car_config)
{
	double delta_vertical_x, delta_vertical_y, delta_horizontal_x, delta_horizontal_y;
	carmen_point_t vertical_pose, horizontal_pose[2];
	int vertical_size, horizontal_size;

	if (map == NULL)
		return;

	vertical_size = ceil(((car_config->length) / map->config.resolution));
	horizontal_size = ceil((((car_config->width)/2.0) / map->config.resolution));

	delta_vertical_x = cos(pose.theta);
	delta_vertical_y = sin(pose.theta);

	delta_horizontal_x = cos(M_PI/2 - pose.theta);
	delta_horizontal_y = sin(M_PI/2 - pose.theta);

	vertical_pose.theta = pose.theta;
	vertical_pose.x = (pose.x - map->config.x_origin) / map->config.resolution;
	vertical_pose.y = (pose.y - map->config.y_origin) /map->config.resolution;

	vertical_pose.x -= car_config->distance_between_rear_car_and_rear_wheels / map->config.resolution * cos(vertical_pose.theta);
	vertical_pose.y -= car_config->distance_between_rear_car_and_rear_wheels / map->config.resolution * sin(vertical_pose.theta);

	for (int v = 0; v <= vertical_size; v++)
	{
		horizontal_pose[0] = vertical_pose;
		horizontal_pose[1] = vertical_pose;

		for (int h = 0; h <= horizontal_size; h++)
		{
			for (int i = 0; i < 2; i++)
			{
				if (horizontal_pose[i].x >= 0 && horizontal_pose[i].x < map->config.x_size &&
						horizontal_pose[i].y >= 0 && horizontal_pose[i].y < map->config.y_size)
				{
					map->complete_map[(int)horizontal_pose[i].x * map->config.y_size + (int)horizontal_pose[i].y] = prob;
				}
			}

			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;

			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
		}

		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}
}


void
carmen_prob_models_alloc_sensor_data(sensor_data_t *sensor_data, int vertical_resolution, int number_of_threads)
{

	sensor_data->vectors_size = vertical_resolution;

	for (int i = 0; i < number_of_threads; i++)
	{
		sensor_data->ray_position_in_the_floor[i] = (carmen_vector_2D_t *) malloc(vertical_resolution * sizeof(carmen_vector_2D_t));
		carmen_test_alloc((sensor_data->ray_position_in_the_floor[i]));

		sensor_data->ray_origin_in_the_floor[i] = (carmen_vector_2D_t *) malloc(vertical_resolution * sizeof(carmen_vector_2D_t));
		carmen_test_alloc((sensor_data->ray_origin_in_the_floor[i]));

		sensor_data->ray_size_in_the_floor[i] = (double *) malloc(vertical_resolution * sizeof(double));
		carmen_test_alloc((sensor_data->ray_size_in_the_floor[i]));

		sensor_data->obstacle_height[i] = (double *) malloc(vertical_resolution * sizeof(double));
		carmen_test_alloc((sensor_data->obstacle_height[i]));

		sensor_data->maxed[i] = (int *) malloc(vertical_resolution * sizeof(int));
		carmen_test_alloc((sensor_data->maxed[i]));

		sensor_data->processed_intensity[i] = (double *) malloc(vertical_resolution * sizeof(double));
		carmen_test_alloc((sensor_data->processed_intensity[i]));

		sensor_data->occupancy_log_odds_of_each_ray_target[i] = (double *) malloc(vertical_resolution * sizeof(double));
		carmen_test_alloc((sensor_data->occupancy_log_odds_of_each_ray_target[i]));

		sensor_data->ray_hit_the_robot[i] = (int *) malloc(vertical_resolution * sizeof(int));
		carmen_test_alloc((sensor_data->ray_hit_the_robot[i]));
	}
}


void
set_log_odds_map_cell(carmen_map_t *map, int x, int y, double lt_i)
{
	map->map[x][y] = carmen_prob_models_log_odds_to_probabilistic(lt_i);
}


void
set_image_map_cell(const carmen_map_t *map, int x, int y, double value)
{
	map->map[x][y] = value;
}


void
set_image_map_cell(const ProbabilisticMap *map, int x, int y, int value)
{
	map->image_map[x][y] = value;
}


double
get_image_map_cell(const carmen_map_t *map, int x, int y)
{
	return (map->map[x][y]);
}


int
get_image_map_cell(const ProbabilisticMap *map, int x, int y)
{
	return (map->image_map[x][y]);
}


int
get_log_odds_map_cell(const ProbabilisticMap *map, int x, int y)
{
	return map->log_odds_map[x][y];
}


int
carmen_prob_models_unaceptable_height(double obstacle_height, double highest_sensor, double safe_range_above_sensors, double safe_height_from_ground)
{
	if ((obstacle_height > highest_sensor + safe_range_above_sensors) || (obstacle_height < safe_height_from_ground))	// obstaculo bateria no carro pois esta dentro de sua altura
	{
		return (1);
	}

	return (0);
}


double
carmen_prob_models_compute_expected_delta_ray_old(double ray_size, int ray_index, double *vertical_correction, double sensor_height)
{
	double expected_delta_ray, next_ray_angle;

	next_ray_angle = -carmen_degrees_to_radians(vertical_correction[ray_index] - vertical_correction[ray_index-1]) + atan(sensor_height / ray_size);
	if (next_ray_angle <= 0.0)
		return (0.0001);

	expected_delta_ray = (sensor_height - ray_size * tan(next_ray_angle)) / tan(next_ray_angle);

	return (expected_delta_ray);
}


double
carmen_prob_models_compute_expected_delta_ray(double ray_length, int ray_index, double *vertical_correction, double sensor_height)
{
	double expected_delta_ray, alpha_plus_theta, theta, ray_size_on_the_floor, beta;
	
	theta = carmen_degrees_to_radians(vertical_correction[ray_index] - vertical_correction[ray_index-1]);
	alpha_plus_theta = carmen_degrees_to_radians(vertical_correction[ray_index] + 90.0);	
	ray_size_on_the_floor = sqrt((sensor_height * sensor_height + ray_length * ray_length) - (2.0 * sensor_height * ray_length * cos(alpha_plus_theta)));
	beta = asin((sin(alpha_plus_theta) * sensor_height) / ray_size_on_the_floor);
	
	expected_delta_ray = (ray_length * sin(theta)) / sin(theta + beta);

	return (expected_delta_ray);
}


double
carmen_prob_models_compute_expected_delta_ray(double h, double r1, double theta)
{
	double a = r1 * sin(acos(h / r1));
	double alpha = asin(a / r1);
	double expected_delta_ray = ((sin(alpha + theta) * h) / sin(carmen_degrees_to_radians(90.0) + alpha + theta)) - a;

	return (expected_delta_ray);
}


double
get_log_odds_via_unexpeted_delta_range_old(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int ray_index, int scan_index,
		bool reduce_sensitivity, int thread_id)
{
	int previous_ray_index;
	double ray_size1, ray_size2, delta_ray, expected_delta_ray; //, expected_delta_ray_old;
	double log_odds;
	double ray_length;
	double obstacle_evidence, p_obstacle;
	double p_0;
	double sigma;

	previous_ray_index = ray_index - 1;
#if (DISCOUNT_RAY_19 == 1)
	// Tratamento do raio que falta/apagado
	if (previous_ray_index == 19)
		previous_ray_index = 18;
#endif
	if (previous_ray_index < 0)
		return (sensor_params->log_odds.log_odds_l0);

	if (sensor_data->maxed[thread_id][previous_ray_index] || sensor_data->maxed[thread_id][ray_index])
		return (sensor_params->log_odds.log_odds_l0);

	if (sensor_data->ray_hit_the_robot[thread_id][previous_ray_index] || sensor_data->ray_hit_the_robot[thread_id][ray_index])
		return (sensor_params->log_odds.log_odds_l0);

	if ((sensor_data->obstacle_height[thread_id][previous_ray_index] < -2.0) || (sensor_data->obstacle_height[thread_id][ray_index] < -2.0))
		return (sensor_params->log_odds.log_odds_l0);

	ray_length = sensor_data->points[sensor_data->point_cloud_index].sphere_points[scan_index + ray_index].length;

	ray_size1 = sensor_data->ray_size_in_the_floor[thread_id][previous_ray_index];
	ray_size2 = sensor_data->ray_size_in_the_floor[thread_id][ray_index];

	delta_ray = ray_size2 - ray_size1;
	expected_delta_ray = carmen_prob_models_compute_expected_delta_ray(ray_length, ray_index, sensor_params->vertical_correction, sensor_params->height);
//	expected_delta_ray_old = carmen_prob_models_compute_expected_delta_ray_old(ray_size1, ray_index, sensor_params->vertical_correction, sensor_params->height);

	obstacle_evidence = (expected_delta_ray - delta_ray) / expected_delta_ray;

//	printf("%lf %lf %lf %lf\n", sensor_data->range[ray_index], expected_delta_ray, expected_delta_ray_old, obstacle_evidence);

	// Testa se tem um obstaculo com um buraco em baixo
	obstacle_evidence = (obstacle_evidence > 1.0)? 1.0: obstacle_evidence;

	if (reduce_sensitivity)
	{
		if (delta_ray > expected_delta_ray) // @@@ Alberto: nao trata buraco?
			return (sensor_params->log_odds.log_odds_free);
		sigma = -sensor_params->unexpeted_delta_range_sigma;
	}
	else
	{
		if (delta_ray > expected_delta_ray) // @@@ Alberto: nao trata buraco?
			return (sensor_params->log_odds.log_odds_l0);
//		two_times_sigma = (2.0 * sensor_params->unexpeted_delta_range_sigma * sensor_params->unexpeted_delta_range_sigma);
		sigma = sensor_params->unexpeted_delta_range_sigma;
	}

	// valor da exponencial com evidencia zero (antigo)
	//	p_0 = exp(-1.0 / two_times_sigma);
	//	p_obstacle = (exp(-((obstacle_evidence - 1.0) * (obstacle_evidence - 1.0)) / two_times_sigma) - p_0) / (1.0 - p_0);

	//a bom é entre [0.1 e 1.0]
	//two_times_sigma = 2*a*a;
	//plot [0:1] (1.0 / exp(-x / (2*a*a)) - 1.0) / (1.0 / exp(-1.0 / (2*a*a)) - 1.0)

	p_0 = (1.0 / exp(1.0 / sigma) - 1.0);
	p_obstacle = (1.0 / exp(obstacle_evidence / sigma) - 1.0) / p_0;

//	printf("%lf\n", p_obstacle);
	log_odds = log(p_obstacle / (1.0 - p_obstacle));

	return (log_odds);
}


//extern FILE *plot_data;

double
get_log_odds_via_unexpeted_delta_range(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int ray_index, int scan_index,
		bool reduce_sensitivity, int thread_id)
{
	int previous_ray_index;
	double ray_size1, ray_size2, delta_ray, expected_delta_ray; //, expected_delta_ray_old;
	double log_odds;
	double obstacle_evidence, p_obstacle;
//	double p_0;
	double sigma;

	previous_ray_index = ray_index - 1;
	if (previous_ray_index < 0)
		return (sensor_params->log_odds.log_odds_l0);

//	fprintf(plot_data, " > ri %d  pm %d  cm %d  ap %lf  ac %lf ", ray_index, sensor_data->maxed[thread_id][previous_ray_index], sensor_data->maxed[thread_id][ray_index],
//			sensor_params->vertical_correction[previous_ray_index], sensor_params->vertical_correction[ray_index]);

	if (sensor_data->maxed[thread_id][previous_ray_index] || sensor_data->maxed[thread_id][ray_index])
		return (sensor_params->log_odds.log_odds_l0);
//	if (sensor_data->maxed[thread_id][ray_index])
//		return (sensor_params->log_odds.log_odds_l0);

	if (sensor_data->ray_hit_the_robot[thread_id][previous_ray_index] || sensor_data->ray_hit_the_robot[thread_id][ray_index])
		return (sensor_params->log_odds.log_odds_l0);

	if ((sensor_data->obstacle_height[thread_id][previous_ray_index] < -2.0) || (sensor_data->obstacle_height[thread_id][ray_index] < -2.0))
		return (sensor_params->log_odds.log_odds_l0);

//	double ray_length = sensor_data->points[sensor_data->point_cloud_index].sphere_points[scan_index + ray_index].length;
	double previous_ray_length = sensor_data->points[sensor_data->point_cloud_index].sphere_points[scan_index + previous_ray_index].length;

//	if (sensor_data->maxed[thread_id][previous_ray_index])
//		ray_size1 = sensor_params->height / tan(-carmen_degrees_to_radians(sensor_params->vertical_correction[previous_ray_index]));
//	else
		ray_size1 = sensor_data->ray_size_in_the_floor[thread_id][previous_ray_index];
	ray_size2 = sensor_data->ray_size_in_the_floor[thread_id][ray_index];

	delta_ray = ray_size2 - ray_size1;
	expected_delta_ray = carmen_prob_models_compute_expected_delta_ray(sensor_params->height, previous_ray_length,
			carmen_degrees_to_radians(sensor_params->vertical_correction[ray_index] - sensor_params->vertical_correction[previous_ray_index]));
//	expected_delta_ray = carmen_prob_models_compute_expected_delta_ray(ray_length, ray_index, sensor_params->vertical_correction, sensor_params->height);
//	expected_delta_ray_old = carmen_prob_models_compute_expected_delta_ray_old(ray_size1, ray_index, sensor_params->vertical_correction, sensor_params->height);
//	printf("r1 %lf, r2 %lf, dr %lf, edr %lf\n", ray_size1, ray_size2, delta_ray, expected_delta_ray);
	obstacle_evidence = (expected_delta_ray - delta_ray) / expected_delta_ray;
	
	// Testa se tem um obstaculo com um buraco em baixo
	obstacle_evidence = (obstacle_evidence > 1.0)? 1.0: obstacle_evidence;
	
//	fprintf(plot_data, ": r_1 %lf  r1 %lf  r2 %lf  d %lf  ed %lf ",
//			sensor_params->height / tan(-carmen_degrees_to_radians(sensor_params->vertical_correction[previous_ray_index])),
//			ray_size1, ray_size2, delta_ray, expected_delta_ray);

	if (reduce_sensitivity)
	{
		if (delta_ray > expected_delta_ray) // @@@ Alberto: nao trata buraco?
			return (sensor_params->log_odds.log_odds_free);
		sigma = 0.15;
	}
	else
	{
		if (delta_ray > expected_delta_ray) // @@@ Alberto: nao trata buraco?
			return (sensor_params->log_odds.log_odds_l0);
		sigma = sensor_params->unexpeted_delta_range_sigma;
	}

	double p_0 = exp(-1.0 / sigma);

	p_obstacle = (exp(-((1.0 - obstacle_evidence) * (1.0 - obstacle_evidence)) / sigma) - p_0) / (1.0 - p_0);

	if (p_obstacle >= 1.0)
		log_odds = MAX_LOG_ODDS_POSSIBLE;
	else
		log_odds = log(p_obstacle / (1.0 - p_obstacle));

	if (log_odds > MAX_LOG_ODDS_POSSIBLE)
		log_odds = MAX_LOG_ODDS_POSSIBLE;

//	fprintf(plot_data, " p0 %lf  po %lf  lo %lf; ", p_0, p_obstacle, log_odds);

	return (log_odds);
}

double
get_log_odds_via_unexpeted_delta_range_jose(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data,
		int ray_index __attribute__ ((unused)), int scan_index __attribute__ ((unused)),
		bool reduce_sensitivity __attribute__ ((unused)), int thread_id)
{
	int previous_ray_index;
	double ray_size1, ray_size2, delta_ray, line_angle; //, expected_delta_ray_old;
//	double log_odds;

	previous_ray_index = ray_index - 1;
	if (previous_ray_index < 0)
		return (sensor_params->log_odds.log_odds_l0);

	if (sensor_data->maxed[thread_id][previous_ray_index] || sensor_data->maxed[thread_id][ray_index])
		return (sensor_params->log_odds.log_odds_l0);

	if (sensor_data->ray_hit_the_robot[thread_id][previous_ray_index] || sensor_data->ray_hit_the_robot[thread_id][ray_index])
		return (sensor_params->log_odds.log_odds_l0);

	if ((sensor_data->obstacle_height[thread_id][previous_ray_index] < -2.0) || (sensor_data->obstacle_height[thread_id][ray_index] < -2.0))
		return (sensor_params->log_odds.log_odds_l0);

	ray_size1 = sensor_data->ray_size_in_the_floor[thread_id][previous_ray_index];
	ray_size2 = sensor_data->ray_size_in_the_floor[thread_id][ray_index];

	delta_ray = ray_size2 - ray_size1;
	line_angle = atan2((sensor_data->obstacle_height[thread_id][ray_index] - sensor_data->obstacle_height[thread_id][previous_ray_index]), delta_ray);
	if (abs(line_angle) > M_PI / 2)
	{
		if (line_angle > 0)
			line_angle = M_PI - line_angle;
		else
			line_angle = -M_PI - line_angle;
	}

	double obstacle_probability = 1.0 - exp(-pow(line_angle / (M_PI/16.0),2));
	if (line_angle < 0.000001)
		return sensor_params->log_odds.log_odds_free;

//	if (expected_delta_ray > M_PI/32)
//		log_odds = sensor_params->log_odds.log_odds_occ;
//	else
//		log_odds = sensor_params->log_odds.log_odds_free;

	return log(obstacle_probability / (1.0 - obstacle_probability));
}


double
get_log_odds_via_unexpeted_delta_range_reverse(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int ray_index, int scan_index,
		bool reduce_sensitivity, int thread_id)
{
	int next_ray_index;
	double ray_length, ray_size1, ray_size2, delta_ray, expected_delta_ray; //, expected_delta_ray_old;
	double log_odds;
	double obstacle_evidence, p_obstacle;
	double p_0;
	double two_times_sigma;

	next_ray_index = ray_index + 1;
	if (next_ray_index >= 31)
		return (sensor_params->log_odds.log_odds_l0);

	if (sensor_data->maxed[thread_id][next_ray_index] || sensor_data->maxed[thread_id][ray_index])
		return (sensor_params->log_odds.log_odds_l0);

	if (sensor_data->ray_hit_the_robot[thread_id][next_ray_index] || sensor_data->ray_hit_the_robot[thread_id][ray_index])
		return (sensor_params->log_odds.log_odds_l0);

	if ((sensor_data->obstacle_height[thread_id][next_ray_index] < -2.0) || (sensor_data->obstacle_height[thread_id][ray_index] < -2.0))
		return (sensor_params->log_odds.log_odds_l0);

	ray_length = sensor_data->points[sensor_data->point_cloud_index].sphere_points[scan_index + next_ray_index].length;

	ray_size2 = sensor_data->ray_size_in_the_floor[thread_id][next_ray_index];
	ray_size1 = sensor_data->ray_size_in_the_floor[thread_id][ray_index];

	delta_ray = ray_size2 - ray_size1;
	expected_delta_ray = carmen_prob_models_compute_expected_delta_ray(ray_length, next_ray_index, sensor_params->vertical_correction, sensor_params->height);

	if (delta_ray < expected_delta_ray)
		return (sensor_params->log_odds.log_odds_l0);

	obstacle_evidence = (delta_ray - expected_delta_ray) / expected_delta_ray;
	
	if (obstacle_evidence > 1.0)
		return (sensor_params->log_odds.log_odds_l0);
	
	if (reduce_sensitivity)
		two_times_sigma = (2.0 * (sensor_params->unexpeted_delta_range_sigma / 4.0) * (sensor_params->unexpeted_delta_range_sigma / 4.0));
	else
		two_times_sigma = (2.0 * sensor_params->unexpeted_delta_range_sigma * sensor_params->unexpeted_delta_range_sigma);
	// valor da exponencial com evidencia zero
	p_0 = exp(-1.0 / two_times_sigma);
	p_obstacle = (exp(-((obstacle_evidence - 1.0) * (obstacle_evidence - 1.0)) / two_times_sigma) - p_0) / (1.0 - p_0);
	log_odds = log(p_obstacle / (1.0 - p_obstacle));

	return (log_odds);
}


void
carmen_prob_models_get_occuppancy_log_odds_by_height(sensor_data_t *sensor_data, sensor_parameters_t *sensor_params, int scan_index,
		double highest_sensor, double safe_range_above_sensors, int reduce_sensitivity, int thread_id)
{
	int i;
	double min_ray_size = 10000.0;
	int min_ray_size_index = sensor_params->vertical_resolution - 1;

//	for (i = sensor_params->vertical_resolution-2; i >= 0; i--)
	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][i], highest_sensor, safe_range_above_sensors))
			sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] = sensor_params->log_odds.log_odds_l0;
		else
		{
			sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] = get_log_odds_via_unexpeted_delta_range(sensor_params, sensor_data, i, scan_index, reduce_sensitivity, thread_id);// +
			//get_log_odds_via_unexpeted_delta_range_reverse(sensor_params, sensor_data, i, scan_index, reduce_sensitivity, thread_id);

			if (sensor_data->obstacle_height[thread_id][i] < 0.5)
				sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] = sensor_params->log_odds.log_odds_l0;

			if (sensor_data->obstacle_height[thread_id][i] > sensor_params->unsafe_height_above_ground)
				if (!sensor_data->maxed[thread_id][i] &&
					!sensor_data->ray_hit_the_robot[thread_id][i] &&
					!(carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][i], highest_sensor, safe_range_above_sensors)))
				{
					sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] = sensor_params->log_odds.log_odds_occ;
				}
		}
		if ((sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] > sensor_params->log_odds.log_odds_l0) && (min_ray_size > sensor_data->ray_size_in_the_floor[thread_id][i]))
		{
			min_ray_size = sensor_data->ray_size_in_the_floor[thread_id][i];
			min_ray_size_index = i;
		}
	}
	sensor_data->ray_that_hit_the_nearest_target[thread_id] = min_ray_size_index;
}


void
carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data_t *sensor_data, sensor_parameters_t *sensor_params, int scan_index,
		double highest_sensor, double safe_range_above_sensors, int reduce_sensitivity, int thread_id, double safe_height_from_ground)
{
	int i;
	double min_ray_size = 10000.0;
	int min_ray_size_index = sensor_params->vertical_resolution - 1;

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][i], highest_sensor, safe_range_above_sensors, safe_height_from_ground))
			sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] = sensor_params->log_odds.log_odds_l0;
		else
		{
			sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] = get_log_odds_via_unexpeted_delta_range(sensor_params, sensor_data, i, scan_index, reduce_sensitivity, thread_id);// +
			//get_log_odds_via_unexpeted_delta_range_reverse(sensor_params, sensor_data, i, scan_index, reduce_sensitivity, thread_id);

//			fprintf(plot_data, " oh %lf  uhag %lf # ", sensor_data->obstacle_height[thread_id][i], sensor_params->unsafe_height_above_ground);
			if (sensor_data->obstacle_height[thread_id][i] > sensor_params->unsafe_height_above_ground)
			{
				if (!sensor_data->maxed[thread_id][i] && !sensor_data->ray_hit_the_robot[thread_id][i] && !(carmen_prob_models_unaceptable_height(sensor_data->obstacle_height[thread_id][i], highest_sensor, safe_range_above_sensors, safe_height_from_ground)))
				{
					sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] = sensor_params->log_odds.log_odds_occ;
				}
			}
		}
		if ((sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i] > sensor_params->log_odds.log_odds_l0) &&
			(min_ray_size > sensor_data->ray_size_in_the_floor[thread_id][i]))
		{
			min_ray_size = sensor_data->ray_size_in_the_floor[thread_id][i];
			min_ray_size_index = i;
		}
	}
//	fprintf(plot_data, "log odds %lf  ray %d ", sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][min_ray_size_index], min_ray_size_index);

	sensor_data->ray_that_hit_the_nearest_target[thread_id] = min_ray_size_index;
}


double
carmen_prob_models_compute_expected_delta_ray2(double ray_size, int ray_index, double *vertical_correction, double sensor_height)
{
	double angle_between_rays, expected_delta_ray;

	angle_between_rays = carmen_degrees_to_radians(vertical_correction[ray_index] - vertical_correction[ray_index-1]);
	expected_delta_ray = (ray_size * sin(angle_between_rays)) / (sin(asin(sensor_height / ray_size) - angle_between_rays));

	return (expected_delta_ray);
}


int
carmen_prob_models_ray_hit_the_robot_old(double distance_between_rear_robot_and_rear_wheels, double robot_length, double robot_width, double x, double y)
{
	/* car corners position
	 	 front
	       x^
	        |
	x1,y1 --|-- x1,y0
	      | | |
	 y    | | |
	 <----|-. |
	      |   |
	x0,y1 ----- x0,y0
		  rear
	 */

	robot_width += 0.8;
	//distance_between_rear_robot_and_rear_wheels += 5.0;

	carmen_vector_2D_t car_corners[2];
	car_corners[0].y = -robot_width / 2;
	car_corners[0].x = -(distance_between_rear_robot_and_rear_wheels + 3.5); // @@@ Alberto: Tem que melhorar esta funcao para evitar este + 3.5
	car_corners[1].y = (robot_width / 2);
	car_corners[1].x = robot_length - distance_between_rear_robot_and_rear_wheels;


	if ((x > car_corners[0].x) && (y > car_corners[0].y) && (x < car_corners[1].x) && (y < car_corners[1].y))
		return 1;

	return 0;
}


static int
carmen_parse_collision_file(double **polygon)
{
	FILE *poly;
	int n_points, h_lvl;
	char *poly_file;

	carmen_param_allow_unfound_variables(0);
	carmen_param_t param_list[] =
	{
			{ (char *) "robot", (char *) "collision_file", CARMEN_PARAM_STRING, &poly_file, 1, NULL }
	};
	carmen_param_install_params(0, NULL, param_list, sizeof(param_list) / sizeof(param_list[0]));

	poly = fopen(poly_file, "r");
	setlocale(LC_NUMERIC, "C");

	if (poly == NULL)
		printf("Can not load Col File\n");

	fscanf(poly, "%d\n", &n_points);
	fscanf(poly, "%d\n", &h_lvl);
	*polygon = (double *) malloc(n_points * 4 * sizeof(double));
	for (int i = 0; i < n_points; i++)
	{
		fscanf(poly, "%lf %lf %lf %lf\n", *polygon + (4 * i), *polygon + (4 * i + 1), *polygon + (4 * i + 2), *polygon + (4 * i + 3));
		printf("%lf %lf %lf %lf\n", (*polygon)[4 * i], (*polygon)[4 * i + 1], (*polygon)[4 * i + 2], (*polygon)[4 * i + 3]);
	}
	fclose(poly);

	return n_points;
}


int
carmen_prob_models_ray_hit_the_robot(double distance_between_rear_robot_and_rear_wheels __attribute__ ((unused)),
		double robot_length __attribute__ ((unused)), double robot_width __attribute__ ((unused)),
		double x, double y)
{
	static int load_polygon = 1;
	static double *collision_model_circles;
	static int col_n_points = 0;

	if (load_polygon)
	{
		col_n_points = carmen_parse_collision_file(&collision_model_circles);
		load_polygon = 0;
		printf("Col Loaded\n");
	}

	for (int i = 0; i < col_n_points; i++)
	{
		double center_x = collision_model_circles[i * 4];
		double center_y = collision_model_circles[i * 4 + 1];
		double radius = collision_model_circles[i * 4 + 2];

		double distance = sqrt((x - center_x) * (x - center_x) + (y - center_y) * (y - center_y));
		if (distance < (radius + 0.8))
			return (1);
	}

	return (0);
}


int
get_ray_origin_a_target_b_and_target_height(double *ax, double *ay, double *bx, double *by, float *obstacle_z, int *ray_hit_the_car, carmen_sphere_coord_t sphere_point,
		carmen_vector_3D_t robot_position, carmen_vector_3D_t sensor_robot_reference, carmen_pose_3D_t sensor_pose, carmen_pose_3D_t sensor_board_pose,
		rotation_matrix *sensor_to_board_matrix, double range_max, rotation_matrix *r_matrix_robot_to_global,
		rotation_matrix* board_to_robot_matrix, double robot_wheel_radius, double x_origin, double y_origin, carmen_robot_ackerman_config_t *car_config)
{
	int maxed;

	if (sphere_point.length >= range_max || sphere_point.length <= 0.0)
	{
		sphere_point.length = range_max;
		maxed = 1;
	}
	else
		maxed = 0;

	carmen_vector_3D_t sensor_position_in_the_world = carmen_change_sensor_reference(robot_position, sensor_robot_reference, r_matrix_robot_to_global);

	carmen_vector_3D_t point_position_in_the_robot = carmen_get_sensor_sphere_point_in_robot_cartesian_reference(sphere_point, sensor_pose, sensor_board_pose,
				sensor_to_board_matrix, board_to_robot_matrix);

	carmen_vector_3D_t global_point_position_in_the_world = carmen_change_sensor_reference(robot_position, point_position_in_the_robot, r_matrix_robot_to_global);


	*ax = sensor_position_in_the_world.x - x_origin;
	*ay = sensor_position_in_the_world.y - y_origin;

	*ray_hit_the_car = carmen_prob_models_ray_hit_the_robot(car_config->distance_between_rear_car_and_rear_wheels, car_config->length, car_config->width, point_position_in_the_robot.x, point_position_in_the_robot.y);

	*bx = global_point_position_in_the_world.x - x_origin;
	*by = global_point_position_in_the_world.y - y_origin;

	*obstacle_z = global_point_position_in_the_world.z - (robot_position.z - robot_wheel_radius);

	return (maxed);
}


void
carmen_prob_models_compute_relevant_map_coordinates(sensor_data_t *sensor_data, sensor_parameters_t *sensor_params, int scan_index,
		carmen_vector_3D_t robot_position, carmen_pose_3D_t sensor_board_pose, rotation_matrix *r_matrix_robot_to_global, rotation_matrix *board_to_robot_matrix,
		double robot_wheel_radius, double x_origin, double y_origin, carmen_robot_ackerman_config_t *car_config,
		int overwrite_blind_spots_around_the_robot, int thread_id)
{
	int i;
	double ax, ay, bx, by;
	float obstacle_z;
	double closest_ray = 10000.0;
	carmen_vector_2D_t ray_origin = {0.0, 0.0};
	bool first = true;

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		sensor_data->maxed[thread_id][i] = get_ray_origin_a_target_b_and_target_height(&ax, &ay, &bx, &by, &obstacle_z, &sensor_data->ray_hit_the_robot[thread_id][i], sensor_data->points[sensor_data->point_cloud_index].sphere_points[scan_index + i],
				robot_position,	sensor_params->sensor_robot_reference, sensor_params->pose, sensor_board_pose, sensor_params->sensor_to_support_matrix, 
				sensor_params->current_range_max, r_matrix_robot_to_global, board_to_robot_matrix, robot_wheel_radius, x_origin, y_origin, car_config);

		sensor_data->ray_position_in_the_floor[thread_id][i].x = bx;
		sensor_data->ray_position_in_the_floor[thread_id][i].y = by;
		sensor_data->ray_size_in_the_floor[thread_id][i] = sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
		sensor_data->obstacle_height[thread_id][i] = obstacle_z;
		sensor_data->processed_intensity[thread_id][i] = (double) (sensor_data->intensity[sensor_data->point_cloud_index][scan_index + i]) / 255.0;

		if (first)
		{
			ray_origin.x = ax;
			ray_origin.y = ay;
			first = false;
		}

		if (!sensor_data->ray_hit_the_robot[thread_id][i])
		{
			if (sensor_data->ray_size_in_the_floor[thread_id][i] < closest_ray)
			{
				ray_origin.x = bx;
				ray_origin.y = by;
				closest_ray = sensor_data->ray_size_in_the_floor[thread_id][i];
			}
		}
	}

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (overwrite_blind_spots_around_the_robot)
		{
			sensor_data->ray_origin_in_the_floor[thread_id][i].x = ax;
			sensor_data->ray_origin_in_the_floor[thread_id][i].y = ay;
		}
		else
		{
			sensor_data->ray_origin_in_the_floor[thread_id][i].x = ray_origin.x;
			sensor_data->ray_origin_in_the_floor[thread_id][i].y = ray_origin.y;
		}
	}
}


void
carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data_t *sensor_data, sensor_parameters_t *sensor_params, int scan_index,
		carmen_vector_3D_t robot_position, carmen_pose_3D_t sensor_board_pose, rotation_matrix *r_matrix_robot_to_global, rotation_matrix *board_to_robot_matrix,
		double robot_wheel_radius, double x_origin, double y_origin, carmen_robot_ackerman_config_t *car_config,
		int overwrite_blind_spots_around_the_robot, int thread_id, int use_remission)
{
	int i;
	double ax, ay, bx, by;
	float obstacle_z;
	double closest_ray = 10000.0;
	carmen_vector_2D_t ray_origin = {0.0, 0.0};
	bool first = true;

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		sensor_data->maxed[thread_id][i] = get_ray_origin_a_target_b_and_target_height(&ax, &ay, &bx, &by, &obstacle_z, &sensor_data->ray_hit_the_robot[thread_id][i], sensor_data->points[sensor_data->point_cloud_index].sphere_points[scan_index + i],
				robot_position,	sensor_params->sensor_robot_reference, sensor_params->pose, sensor_board_pose, sensor_params->sensor_to_support_matrix,
				sensor_params->current_range_max, r_matrix_robot_to_global, board_to_robot_matrix, robot_wheel_radius, x_origin, y_origin, car_config);

		sensor_data->ray_position_in_the_floor[thread_id][i].x = bx;
		sensor_data->ray_position_in_the_floor[thread_id][i].y = by;
		sensor_data->ray_size_in_the_floor[thread_id][i] = sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
		sensor_data->obstacle_height[thread_id][i] = obstacle_z;
		if (use_remission)
			sensor_data->processed_intensity[thread_id][i] = (double) (sensor_data->intensity[sensor_data->point_cloud_index][scan_index + i]) / 255.0;

		if (first)
		{
			ray_origin.x = ax;
			ray_origin.y = ay;
			first = false;
		}

		if (!sensor_data->ray_hit_the_robot[thread_id][i])
		{
			if (sensor_data->ray_size_in_the_floor[thread_id][i] < closest_ray)
			{
				ray_origin.x = bx;
				ray_origin.y = by;
				closest_ray = sensor_data->ray_size_in_the_floor[thread_id][i];
			}
		}
	}

	for (i = 0; i < sensor_params->vertical_resolution; i++)
	{
		if (overwrite_blind_spots_around_the_robot)
		{
			sensor_data->ray_origin_in_the_floor[thread_id][i].x = ax;
			sensor_data->ray_origin_in_the_floor[thread_id][i].y = ay;
		}
		else
		{
			sensor_data->ray_origin_in_the_floor[thread_id][i].x = ray_origin.x;
			sensor_data->ray_origin_in_the_floor[thread_id][i].y = ray_origin.y;
		}
	}
}


void
init_carmen_map(const ProbabilisticMapParams *params, carmen_map_t *carmen_map)
{
	int i;

	carmen_map->config.x_size = params->grid_sx;
	carmen_map->config.y_size = params->grid_sy;
	carmen_map->config.resolution = params->grid_res;
	strcpy(carmen_map->config.origin, "from_mapping");
	carmen_map->config.map_name = (char *) malloc((strlen("occupancy grid") + 1) * sizeof(char));
	strcpy(carmen_map->config.map_name, "occupancy grid");
	carmen_map->config.x_origin = 0.0;
	carmen_map->config.y_origin = 0.0;

	carmen_map->complete_map = (double *) malloc(carmen_map->config.x_size * carmen_map->config.y_size * sizeof(double));
	carmen_test_alloc(carmen_map->complete_map);
	for (i = 0; i < carmen_map->config.x_size * carmen_map->config.y_size; i++)
		carmen_map->complete_map[i] = -1.0; // unknown

	carmen_map->map = (double **) calloc(carmen_map->config.x_size, sizeof(double *));
	carmen_test_alloc(carmen_map->map);
	for (int x_index = 0; x_index < carmen_map->config.x_size; x_index++)
		carmen_map->map[x_index] = carmen_map->complete_map + x_index * carmen_map->config.y_size;
}


void
free_probabilistic_map(ProbabilisticMapParams *params, ProbabilisticMap *map, int num_particles)
{
	for (int m = 0; m < num_particles; m++)
	{
		if (map[m].image_map)
		{
			for(int x = 0; x < params->grid_sx; x++)
			{
				if (map[m].image_map[x])
					free(map[m].image_map[x]);

			}
			free(map[m].image_map);
		}
		if (map[m].log_odds_map)
		{
			for (int x = 0; x < params->grid_sx; x++)
			{
				if (map[m].log_odds_map[x])
					free(map[m].log_odds_map[x]);

			}
			free(map[m].log_odds_map);
		}
	}
}


void
compute_initial_range_for_each_laser_ray_index(carmen_map_t *map_with_robot_only)
{
	(void)map_with_robot_only;
	/*	carmen_point_t xt;
	carmen_point_t zt_pose;
	double ztk_star, ztk;
	double p;
	double q = 1.0;

	xt.x = xt.y = xt.theta = 0.0;
	transform_robot_pose_to_laser_pose(&zt_pose, &xt);

	for (int k = 0; k < params.laser_beams; k += measurement_model_params.sampling_step)
	{
		ztk = zt[k];
		ztk_star = carmen_ray_cast(zt_pose, k, map, measurement_model_params.max_range, measurement_model_params.start_angle, measurement_model_params.angle_step);
		if (ztk > measurement_model_params.max_range)
			ztk = measurement_model_params.max_range;
		if (ztk_star > measurement_model_params.max_range)
			ztk_star = measurement_model_params.max_range;

		p = beam_range_finder_model_probability(ztk_star, ztk);
		q = q * p;
	}
	return q;
	 */
}


void
set_map_as_fully_occupied(carmen_map_t *carmen_map)
{
	int i;

	for (i = 0; i < carmen_map->config.x_size * carmen_map->config.y_size; i++)
		carmen_map->complete_map[i] = 1.0;
}


void
carmen_update_cells_below_robot(carmen_map_t *map, carmen_point_t xt)
{	// @@@ Alberto: Nao ee propria para robos retangulares
	int j;
	double r, dx, dy, dr;
	double ax, ay, bx, by, px, py;
	int step_count;
	int nx, ny;
	int angle;

	ax = xt.x;
	ay = xt.y;


	if (map_params.robot_length > map_params.robot_width)
		r = map_params.robot_length;
	else
		r = map_params.robot_width;

	/*
	http://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles
	for(int y=-radius; y<=radius; y++)
	    for(int x=-radius; x<=radius; x++)
        	if(x*x+y*y <= radius*radius)
        	    setpixel(origin.x+x, origin.y+y);
	*/

	// Ray-trace the grid
	for (angle = 0; angle < 360; angle++)
	{
		// Calculate the actual coords
		bx = r * cos(carmen_degrees_to_radians(angle)) + ax;
		by = r * sin(carmen_degrees_to_radians(angle)) + ay;

		// Compute line parameters
		dx = bx - ax;
		dy = by - ay;
		dr = sqrt(dx * dx + dy * dy);

		dx /= dr;
		dy /= dr;
		step_count = (int) round(dr);

		// Walk the line and update the grid
		for (j = 0; j < step_count - 2; j++)
		{
			px = ax + dx * (double) j;
			py = ay + dy * (double) j;

			nx = map_to_grid_x(map, px);
			ny = map_to_grid_y(map, py);

			if (map_grid_is_valid(map, nx, ny))
				set_image_map_cell(map, nx, ny, 0.0); // free the cell
		}
	}
}


void
init_probabilistic_grid_map_model(ProbabilisticMapParams *params, carmen_map_t *carmen_map)
{
	carmen_map_t *map_with_robot_only;
	carmen_point_t robot_pose;

	map_params = *params;
	if (carmen_map != NULL)
	{
		map_params.grid_res = params->grid_res = carmen_map->config.resolution;
		map_params.grid_sx = params->grid_sx = carmen_map->config.x_size;
		map_params.grid_sy = params->grid_sy = carmen_map->config.y_size;
		map_params.grid_size = params->grid_size = carmen_map->config.x_size * carmen_map->config.y_size;

		map_params.width = params->width = carmen_map->config.x_size * carmen_map->config.resolution;
		map_params.height = params->height = carmen_map->config.y_size * carmen_map->config.resolution;

		map_with_robot_only = carmen_map_clone(carmen_map);
		set_map_as_fully_occupied(map_with_robot_only);

		robot_pose.x = grid_to_map_x(map_params.grid_sx / 2);
		robot_pose.y = grid_to_map_y(map_params.grid_sy / 2);
		robot_pose.theta = 0.0;
		carmen_update_cells_below_robot(map_with_robot_only, robot_pose);

		compute_initial_range_for_each_laser_ray_index(map_with_robot_only);

		carmen_map_destroy(&map_with_robot_only);
	}
}


void
init_probabilistic_map(ProbabilisticMapParams *params, carmen_map_t *carmen_map,
		ProbabilisticMap *map, int num_particles)
{
	init_probabilistic_grid_map_model(params, carmen_map);

	free_probabilistic_map(params, map, num_particles);

	for (int m = 0; m < num_particles; m++)
	{
		map[m].image_map = (int **) calloc(params->grid_sx, sizeof(int *));
		map[m].log_odds_map = (float **) calloc(params->grid_sx, sizeof(float *));
		for (int x = 0; x<params->grid_sx; x++)
		{
			map[m].image_map[x] = (int *) calloc(params->grid_sy, sizeof(int));
			map[m].log_odds_map[x] = (float *) calloc(params->grid_sy, sizeof(float));

			for (int y = 0; y < params->grid_sy; y++)
			{
				set_image_map_cell(&map[m], x, y, PMC_UNKNOWN_AREA);
				set_log_odds_map_cell(&map[m], x, y, (float)params->log_odds_bias);
			}
		}
	}
}


/** \brief Copy the complete_map to map
 *
 *  Copy the complete_map to map (both in carmen_map_t).
 *  @param c1 the carmen_map, with carmen_map.map allocated
 * 	@param c2 the config map information
 */

void
copy_complete_map_to_map(carmen_map_p carmen_map, carmen_map_config_t config)
{
	if (!carmen_map || !carmen_map->map)
	{
		fprintf(stderr, "Map not allocated\n");
		return;
	}

	int i = 0;
	for (i = 0; i < config.x_size; i++)
		carmen_map->map[i] = carmen_map->complete_map + i * config.y_size;
}


/**
 * Perform a ray cast from (ax, ay) to (bx, by). If color != MapColor.WHITE, paint the ray with the color.
 * @param ax X coordinate of point a.
 * @param ay Y coordinate of point a.
 * @param bx X coordinate of point b.
 * @param by Y coordinate of point b.
 * @param color color.
 * @returns Size of the ray (in meters).
 */
double
ray_cast_between_coordinates(double ax, double ay, double bx, double by, const ProbabilisticMap *map, ProbabilisticMapColor color)
{
	double j;
	double dx, dy, dr;
	double px, py;
	double step_count;
	int nx, ny;
	int occ;

	// Compute line parameters
	dx = bx - ax;
	dy = by - ay;
	dr = sqrt(dx * dx + dy * dy);
	if (dr < map_params.grid_res)
		return 0.0;

	step_count = dr / map_params.grid_res;
	dx /= step_count;
	dy /= step_count;

	// Just to keep the compiler happy ...
	occ = (int)PMC_UNKNOWN_AREA;

	// Walk the line (ray)
	px = py = 0; // Just to keep the compiler happy ...
	for (j = 1; j <= step_count; j = j + 1.0)
	{
		px = ax + dx * j;
		py = ay + dy * j;

		nx = map_to_grid_x(px);
		ny = map_to_grid_y(py);

		if (map_grid_is_valid(nx, ny))
		{
			occ = get_image_map_cell(map, nx, ny);
			if (color == PMC_UNKNOWN_AREA)
			{
				if (occ == (int) PMC_UNKNOWN_AREA)
					break;
				else if (occ < (int) PMC_OBSTACLE_COLOR_LIMIT) // found an obstacle
					return map_params.range_max;
			}
			else
			{
				if (color != PMC_WHITE)
					set_image_map_cell(map, nx, ny, (int)color);

				if (occ < (int) PMC_OBSTACLE_COLOR_LIMIT) // found an obstacle
					break;
			}
		}
		else
			break; // TODO outside the map @@ Alberto: checar se isso é o correto!
		//return _range_max; // outside the map
	}
	dx = px - ax;
	dy = py - ay;
	return sqrt(dx * dx + dy * dy);
}


/**
 * Perform a ray cast from (ax, ay) to (bx, by).
 * @param ax X coordinate of point a.
 * @param ay Y coordinate of point a.
 * @param bx X coordinate of point b.
 * @param by Y coordinate of point b.
 * @param by map.
 * @returns Size of the ray (in meters).
 */
double
ray_cast_between_coordinates(double ax, double ay, double bx, double by, carmen_map_t *map, double range_max)
{
	double j;
	double dx, dy, dr;
	double px, py;
	double step_count;
	int nx, ny;
	float occ;

	// Compute line parameters
	dx = bx - ax;
	dy = by - ay;
	dr = sqrt(dx * dx + dy * dy);
	if (dr < map->config.resolution)
		return 0.0;

	step_count = dr / map->config.resolution;
	dx /= step_count;
	dy /= step_count;

	// Just to keep the compiler happy ...
	occ = (int) PMC_UNKNOWN_AREA;

	// Walk the line (ray)
	px = py = 0; // Just to keep the compiler happy ...
	for (j = 1; j <= step_count; j = j + 1.0)
	{
		px = ax + dx * j;
		py = ay + dy * j;

		nx = map_to_grid_x(map, px);
		ny = map_to_grid_y(map, py);

		if (map_grid_is_valid(map, nx, ny))
		{
			occ = get_image_map_cell(map, nx, ny);
			if (occ > 0.5) // found an obstacle
				break;
		}
		else
			return (range_max); // outside the map
	}
	dx = px - ax;
	dy = py - ay;

	return (sqrt(dx * dx + dy * dy));
}


double 
ray_cast_unknown_between_coordinates(double ax, double ay, double bx, double by, const ProbabilisticMap *map)
{
	double j;
	double dx, dy, dr;
	double px, py;
	double step_count;
	int nx, ny;
	int occ;

	// Compute line parameters
	dx = bx - ax;
	dy = by - ay;
	dr = sqrt(dx * dx + dy * dy);
	if (dr < map_params.grid_res)
		return 0.0;

	step_count = dr / map_params.grid_res;
	dx /= step_count;
	dy /= step_count;

	// Just to keep the compiler happy ...
	occ = (int) PMC_UNKNOWN_AREA;

	// Walk the line (ray)
	px = py = 0; // Just to keep the compiler happy ...
	for (j = 1; j <= step_count; j = j + 1.0)
	{
		px = ax + dx * j;
		py = ay + dy * j;

		nx = map_to_grid_x(px);
		ny = map_to_grid_y(py);

		if (map_grid_is_valid(nx, ny))
		{
			occ = get_image_map_cell(map, nx, ny);
			if (occ == (int) PMC_UNKNOWN_AREA)
				break;
			else if (occ < (int) PMC_OBSTACLE_COLOR_LIMIT) // found an obstacle
				return -1;
		}
		else
			return -1; // outside the map
	}
	dx = px - ax;
	dy = py - ay;
	return sqrt(dx * dx + dy * dy);
}


/**
 * Returns bearing
 * @param i ray index
 * @param theta robot's orientation in radians
 */
double
bearing(int i, double theta)
{
	double angle;
	double theta_in_degrees;

	theta_in_degrees = carmen_radians_to_degrees(theta);

	// Compute range end-point
	angle = map_params.range_start + i * map_params.range_step;
	// Now adjust for the robot's orientation
	angle += theta_in_degrees;
	// Convert to Radians
	angle = carmen_degrees_to_radians(angle);
	return angle;
}


double
carmen_ray_cast(carmen_point_t xt, int sample_index, carmen_map_t *map, double range_max, double start_angle, double angle_step)
{
	double r;
	double ax, ay, bx, by;
	double angle;

	ax = xt.x;
	ay = xt.y;

	r = range_max;

	angle = carmen_degrees_to_radians(sample_index * angle_step + start_angle) + xt.theta;

	// Calculate the actual coords
	// Note that r is relative to the robot
	bx = r * cos(angle) + ax;
	by = r * sin(angle) + ay;

	return ray_cast_between_coordinates(ax, ay, bx, by, map, range_max);
}


double
ray_cast(carmen_point_t xt, int sample_index, const ProbabilisticMap *map, ProbabilisticMapColor color)
{
	int i;
	double r;
	double ax, ay, bx, by;
	double angle;

	i = sample_index;

	ax = xt.x;
	ay = xt.y;

	r = map_params.range_max;

	angle = bearing(i, xt.theta);

	// Calculate the actual coords
	// Note that r is relative to the robot
	bx = r * cos(angle) + ax;
	by = r * sin(angle) + ay;

	return ray_cast_between_coordinates(ax, ay, bx, by, map, color);
}


double 
ray_cast_unknown(carmen_point_t xt, const ProbabilisticMap *map, double max_distance)
{
	double ax, ay, bx, by;

	ax = xt.x;
	ay = xt.y;

	// Calculate the actual coords
	// Note that r is relative to the robot
	bx = max_distance * cos(xt.theta) + ax;
	by = max_distance * sin(xt.theta) + ay;

	double distance = ray_cast_unknown_between_coordinates(ax, ay, bx, by, map);
	if (distance < 0)
		return max_distance;
	else
		return distance;
}


void 
draw_ray_cast(carmen_point_t xt, double distance, double direction, const ProbabilisticMap *map, ProbabilisticMapColor color)
{
	double r;
	double ax, ay, bx, by;
	double angle;
	double theta;

	ax = xt.x;
	ay = xt.y;
	theta = xt.theta;

	r = distance;

	angle = theta + direction;

	bx = r * cos(angle) + ax;
	by = r * sin(angle) + ay;

	draw_ray(ax, ay, bx, by, map, color);
}


void 
draw_ray(double ax, double ay, double bx, double by, const ProbabilisticMap *map, ProbabilisticMapColor color)
{
	double j;
	double dx, dy, dr;
	double px, py;
	double step_count;
	int nx, ny;

	// Compute line parameters
	dx = bx - ax;
	dy = by - ay;
	dr = sqrt(dx * dx + dy * dy);
	if (dr < map_params.grid_res)
		return;

	step_count = dr / map_params.grid_res;
	dx /= step_count;
	dy /= step_count;

	// Walk the line (ray)
	px = py = 0; // Just to keep the compiler happy ...
	for (j = 1; j <= step_count; j = j + 1.0)
	{
		px = ax + dx * j;
		py = ay + dy * j;

		nx = map_to_grid_x(px);
		ny = map_to_grid_y(py);

		if (map_grid_is_valid(nx, ny))
		{
			set_image_map_cell(map, nx, ny, (int)color);
		}
		else
			break; // outside the map @@ Alberto: checar se isso \E9 o correto!
	}
}

// Equation 9.6 of the book Probabilistic Robotics
double 
occupancy_grid_probability(double lt_i)
{
	return (1.0 - (1.0 / (1.0 + exp(lt_i))));
}


/**
 * Occupancy grid mapping algorithm for map cell i (xi, yi) in the LRS perceptual field.
 * @See Table 9.1 of the Probabilistic Robotics book.
 * @param xi Coordinate x of map cell i.
 * @param yi Coordinate y of map cell i.
 * @param map Map.
 * @param inverse_sensor_model_value Inverse sensor model value.
 */
void
occupancy_grid_mapping(int xi, int yi, const ProbabilisticMap *map, int inverse_sensor_model_value)
{
	double lt_i;
	double p_mi;

	lt_i = get_log_odds_map_cell(map, xi, yi);

	lt_i = lt_i + inverse_sensor_model_value - map_params.l0;

	// Check if current value has reached the limits or if it is OK.
	if (lt_i > map_params.log_odds_max)
		set_log_odds_map_cell(map, xi, yi, (float)map_params.log_odds_max);
	else if (lt_i < map_params.log_odds_min)
		set_log_odds_map_cell(map, xi, yi, (float)map_params.log_odds_min);
	else
		set_log_odds_map_cell(map, xi, yi, (float)lt_i);

	// Fill in the real map with the probability already converted to a visual value.
	p_mi = occupancy_grid_probability(lt_i - (double)(map_params.log_odds_bias));

	set_image_map_cell(map, xi, yi, (int)(p_mi * 255.0));
}


/**
 * Occupancy grid mapping algorithm for map cell i (xi, yi) in the LRS perceptual field.
 * @See Table 9.1 of the Probabilistic Robotics book.
 * @param xi Coordinate x of map cell i.
 * @param yi Coordinate y of map cell i.
 * @param map Map.
 * @param inverse_sensor_model_value Inverse sensor model value.
 */
void
occupancy_grid_mapping(carmen_map_t *map, int xi, int yi, BeanRangeFinderMeasurementModelParams* laser_params, int inverse_sensor_model_value)
{
	double lt_i;

	lt_i = get_log_odds_map_cell(map, xi, yi);

	lt_i = lt_i + inverse_sensor_model_value - laser_params->l0;

	// Check if current value has reached the limits or if it is OK.
	if (lt_i > MAX_LOG_ODDS_POSSIBLE)
		set_log_odds_map_cell(map, xi, yi, MAX_LOG_ODDS_POSSIBLE);
	else if (lt_i < -MAX_LOG_ODDS_POSSIBLE)
		set_log_odds_map_cell(map, xi, yi, -MAX_LOG_ODDS_POSSIBLE);
	else
		set_log_odds_map_cell(map, xi, yi, (float) lt_i);
}


/**
 * Inverse range sensor model for occupancy grid mapping algorithm.
 * @See Table 9.2 of the Probabilistic Robotics book.
 * @param xi Coordinate x of map cell i.
 * @param yi Coordinate y of map cell i.
 * @param map Map.
 */
void 
inverse_range_sensor_model()
{
	//TODO
}


void
update_cells_above_robot(const ProbabilisticMap *map, carmen_point_t xt,
		double robot_length, double robot_width,
		ProbabilisticMapColor color)
{
	int j;
	double r, dx, dy, dr;
	double ax, ay, bx, by, px, py;
	int step_count;
	int nx, ny;
	int angle;

	ax = xt.x;
	ay = xt.y;

	if (robot_length > robot_width)
		r = robot_length;
	else
		r = robot_width;

	// Ray-trace the grid
	for (angle = 0; angle < 360; angle++)
	{
		// Calculate the actual coords
		bx = r * cos(carmen_degrees_to_radians(angle)) + ax;
		by = r * sin(carmen_degrees_to_radians(angle)) + ay;

		// Compute line parameters
		dx = bx - ax;
		dy = by - ay;
		dr = sqrt(dx * dx + dy * dy);

		dx /= dr;
		dy /= dr;
		step_count = (int) round(dr);

		// Walk the line and update the grid
		for (j = 0; j < step_count - 2; j++)
		{
			px = ax + dx * (double) j;
			py = ay + dy * (double) j;

			nx = map_to_grid_x(px);
			ny = map_to_grid_y(py);

			if (map_grid_is_valid(nx, ny))
				set_image_map_cell(map, nx, ny, (int)color);
		}
	}
}


void
carmen_update_cells_in_the_sensor_perceptual_field(carmen_map_t *map, carmen_point_t xt, const double *zt, sensor_parameters_t *sensor_params)
{
	BeanRangeFinderMeasurementModelParams laser_params;

	laser_params.start_angle = -0.5 * sensor_params->fov_range;
	laser_params.fov_range = sensor_params->fov_range;
	laser_params.angle_step = sensor_params->fov_range / (double) (sensor_params->laser_beams - 1);
	laser_params.angular_offset = sensor_params->angular_offset;
	laser_params.front_offset = sensor_params->front_offset;
	laser_params.side_offset = sensor_params->side_offset;
	laser_params.lambda_short = sensor_params->lambda_short;
	laser_params.max_range = sensor_params->range_max;
	laser_params.sampling_step = sensor_params->sampling_step;
	laser_params.sigma_zhit = sensor_params->sigma_zhit;
	laser_params.zhit = sensor_params->zhit;
	laser_params.zmax = sensor_params->zmax;
	laser_params.zshort = sensor_params->zshort;
	laser_params.zrand = sensor_params->zrand;
	laser_params.laser_beams = sensor_params->laser_beams;
	laser_params.l0 = sensor_params->log_odds.log_odds_l0;
	laser_params.lfree = sensor_params->log_odds.log_odds_free;
	laser_params.locc = sensor_params->log_odds.log_odds_occ;

	carmen_update_cells_in_the_laser_perceptual_field(map, xt, zt, &laser_params);
}


void
carmen_update_cells_in_the_laser_perceptual_field(carmen_map_t *map, carmen_point_t xt, const double *zt, BeanRangeFinderMeasurementModelParams *laser_params)
{
	int i, j;
	double r, dx, dy, dr;
	double ax, ay, bx, by, px, py;
	int step_count;
	int nx, ny;
	int maxed;
	double angle;
	double angle_step, start_angle;

	start_angle = laser_params->start_angle;
	angle_step = laser_params->fov_range / (double) (laser_params->laser_beams - 1);

	ax = xt.x;
	ay = xt.y;

	// Ray-trace the grid
	for (i = 0; i < laser_params->laser_beams; i ++)
	{
		r = zt[i];

		// There was a fudge here due to a bug in MRDS that returns
		// zero instead of max range for a laser miss
		if (r > laser_params->max_range)
		{
			r = laser_params->max_range;
			maxed = 1;
		}
		else
			maxed = 0;

		angle = carmen_degrees_to_radians(i * angle_step + start_angle) + xt.theta;

		// Calculate the actual coords
		// Note that r is relative to the robot
		bx = r * cos(angle) + ax;
		by = r * sin(angle) + ay;

		// Compute line parameters
		dx = bx - ax;
		dy = by - ay;
		dr = sqrt(dx * dx + dy * dy);

		dx /= dr;
		dy /= dr;
		step_count = (int) round(dr);

		// Walk the line and update the grid
		for (j = 0; j < step_count - 2; j++)
		{
			px = ax + dx * (double) j;
			py = ay + dy * (double) j;

			nx = map_to_grid_x(map, px);
			ny = map_to_grid_y(map, py);

			if (map_grid_is_valid(map, nx, ny))
			{
				occupancy_grid_mapping(map, nx, ny, laser_params, laser_params->lfree);

				if (get_image_map_cell(map, nx, ny) >= 0.5)
					break;	// do not cross obstacles until they are cleared
			}
			else
				break; // off map
		}

		// Place an obstacle at the end of the ray if the scan
		// was not maxed out (a "miss" returns the max range value)
		if (!maxed)
		{
			px = ax + dx * (double) step_count;
			py = ay + dy * (double) step_count;

			nx = map_to_grid_x(map, px);
			ny = map_to_grid_y(map, py);

			if (map_grid_is_valid(map, nx, ny))
				occupancy_grid_mapping(map, nx, ny, laser_params, laser_params->locc);
		}
	}
}


void
save_probabilistic_map(const ProbabilisticMap *map, carmen_point_t pose)
{
	static int map_number = 0;
	char map_file_name[1000];
	FILE *map_file;

	sprintf(map_file_name, "map_file%d.pnm", map_number);
	map_file = fopen(map_file_name, "w");

	int robot_center_x = map_to_grid_x(pose.x);
	int robot_center_y = map_to_grid_y(pose.y);

	// PNM file header
	fprintf(map_file, "P3\n#PNM criado por Alberto\n");
	fprintf(map_file, "%d %d\n255\n", map_params.grid_sx, map_params.grid_sy);

	for (int y = map_params.grid_sy-1; y >= 0; y--)
	{
		for (int x = 0; x < map_params.grid_sx; x++)
		{
			int gray_value = get_image_map_cell(map, x, y);

			if ( (abs(x - robot_center_x) <= 2) && (abs(y - robot_center_y) <= 2) )
			{
				fprintf(map_file, "%d\n%d\n%d\n", 255, 0, 0);
			}
			else
			{
				fprintf(map_file, "%d\n%d\n%d\n", gray_value, gray_value, gray_value);
			}
		}
	}

	fclose(map_file);
	map_number++;
}


/**
 * copy image and probabilistic info from map to map
 */
void 
copy_probabilistic_map(ProbabilisticMap *dst, const ProbabilisticMap *src)
{
	for(int x = 0; x < map_params.grid_sx; x++)
	{
		for(int y = 0; y < map_params.grid_sy; y++)
		{
			dst->image_map[x][y] = src->image_map[x][y];
			dst->log_odds_map[x][y] = src->log_odds_map[x][y];
		}
	}
}


void 
translate_map(const ProbabilisticMap *map, ProbabilisticMapParams map_config, double dx, double dy)
{
	// copy information from the current map
	float *log_odds_map_copy = (float*)malloc(map_config.grid_sx * map_config.grid_sy * sizeof(float*));
	for(int x = 0; x < map_config.grid_sx; x++)
	{
		for(int y = 0; y < map_config.grid_sy; y++)
		{
			log_odds_map_copy[y * map_config.grid_sx + x] = map->log_odds_map[x][y];
		}
	}

	for (double yt = 0.0; yt < map_config.height; yt += map_config.grid_res)
	{
		double yt_1 = yt - dy;
		int nyt_1 = round(yt_1 / map_config.grid_res);
		int nyt = round(yt / map_config.grid_res);

		for (double xt = 0.0; xt < map_config.width; xt += map_config.grid_res)
		{
			double xt_1 = xt - dx;
			int nxt_1 = round(xt_1 / map_config.grid_res);
			int nxt = round(xt / map_config.grid_res);

			if (nxt >= 0 && nxt < map_config.grid_sx && nyt >= 0 && nyt < map_config.grid_sy)
			{
				map->log_odds_map[nxt][nyt] = nxt_1 >= 0 && nxt_1 < map_config.grid_sx && nyt_1 >= 0 && nyt_1 < map_config.grid_sy ?
						log_odds_map_copy[nyt_1 * map_config.grid_sx + nxt_1] : // copy old region to the translated map
						(float)map_config.log_odds_bias; // init the new region with default values
			}
		}
	}

	free(log_odds_map_copy);
}


void 
update_cells_in_the_camera_perceptual_field(const carmen_point_t Xt, const ProbabilisticMap *global_map,
		ProbabilisticMapParams global_map_params, ProbabilisticMapParams stereo_map_params,
		double y_offset, float *zt)
{
	for (double _y = 0.0; _y < stereo_map_params.height; _y += stereo_map_params.grid_res)
	{
		for (double _x = 0.0; _x < stereo_map_params.width; _x += stereo_map_params.grid_res)
		{
			// get instantaneous map grid cell
			int _nx = round(_x / stereo_map_params.grid_res);
			int _ny = round(_y / stereo_map_params.grid_res);

			float observation = zt[_ny * stereo_map_params.grid_sx + _nx];
			if (observation < 0) //cell is outside perceptual field
				continue;

			int inverse_sensor_model_value = (observation == 0.0) ? stereo_map_params.lfree : stereo_map_params.locc;

			// rotate by Xt.theta and translate by (Xt.x,Xt.y)
			double x = _x * cos(Xt.theta) - (_y - y_offset) * sin(Xt.theta) + Xt.x;
			double y = _x * sin(Xt.theta) + (_y - y_offset) * cos(Xt.theta) + Xt.y;

			// get global map grid cell
			int nx = round(x / global_map_params.grid_res);
			int ny = round(y / global_map_params.grid_res);

			if (nx >= 0 && nx < global_map_params.grid_sx && ny >= 0 && ny < global_map_params.grid_sy)
				occupancy_grid_mapping(nx, ny, global_map, inverse_sensor_model_value);
		}
	}
}


void 
copy_probabilistic_map_to_image_buffer(ProbabilisticMapParams map_config, ProbabilisticMap *probabilistic_map, unsigned char *dst, int n_channels)
{
	for (int j = 0; j < map_config.grid_sy; j++)
	{
		for (int i = 0; i < map_config.grid_sx; i++)
		{
			float log_odds = get_log_odds_map_cell(probabilistic_map, i, j);
			float value;
			if (log_odds == (float)map_config.log_odds_min)
			{
				value = 1.0;
			}
			else if (log_odds == (float)map_config.log_odds_max)
			{
				value = 0.0;
			}
			else if (log_odds == (float)map_config.log_odds_bias)
			{
				value = 0.5;
			}
			else
			{
				double p_mi = 1.0 - (1.0 / (1.0 + exp((double)(log_odds - map_params.log_odds_bias))));
				value = 1.0 - p_mi;
			}

			for (int n = 0; n < n_channels; n++)
			{
				int offset = n_channels * (j * map_config.grid_sx + i);
				dst[offset + n] = (unsigned char)MIN(round(255.0 * value), 255);
			}
		}
	}
}


void
carmen_prob_models_update_current_map_with_snapshot_map_and_clear_snapshot_map(carmen_map_t *current_map, carmen_map_t *snapshot_map)
{	// Os dois mapas tem que ser probabilisiticos
	#pragma omp for
	for (int i = 0; i < current_map->config.x_size * current_map->config.y_size; i++)
	{
		if (snapshot_map->complete_map[i] > 0.0)
			carmen_prob_models_log_odds_occupancy_grid_mapping(current_map, i, get_log_odds(snapshot_map->complete_map[i]));

		snapshot_map->complete_map[i] = -1.0;
	}
}


void
carmen_prob_models_update_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(carmen_map_t *current_map,
		carmen_map_t *log_odds_snapshot_map, double log_odds_l0)
{	// O current_map eh probabilistico e o snapshot_map log_odds
	#pragma omp for
	for (int i = 0; i < current_map->config.x_size * current_map->config.y_size; i++)
	{
		if (log_odds_snapshot_map->complete_map[i] != log_odds_l0)
			carmen_prob_models_log_odds_occupancy_grid_mapping(current_map, i, log_odds_snapshot_map->complete_map[i]);

		log_odds_snapshot_map->complete_map[i] = log_odds_l0;
	}
}


void
carmen_prob_models_clear_cells_hit_by_single_ray(carmen_map_t *log_odds_snapshot_map, double log_odds_occ, double log_odds_l0)
{
	int width = log_odds_snapshot_map->config.x_size;
	int height = log_odds_snapshot_map->config.y_size;
	double **map = log_odds_snapshot_map->map;

	for (int x = 1; x < width - 1; x++)
	{
		for (int y = 1; y < height - 1; y++)
		{
			if (map[x][y] == log_odds_occ)
			{
				if ((map[x-1][y-1] == log_odds_l0) &&
					(map[x-1][y] == log_odds_l0) &&
					(map[x-1][y+1] == log_odds_l0) &&
					(map[x][y-1] == log_odds_l0) &&
					(map[x][y+1] == log_odds_l0) &&
					(map[x+1][y-1] == log_odds_l0) &&
					(map[x+1][y] == log_odds_l0) &&
					(map[x+1][y+1] == log_odds_l0))
					map[x][y] = log_odds_l0;
			}
		}
	}
}


void
carmen_prob_models_overwrite_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(carmen_map_t *current_map,
		carmen_map_t *log_odds_snapshot_map, double log_odds_l0)
{	// O current_map eh probabilistico e o snapshot_map log_odds
	#pragma omp for
	for (int i = 0; i < current_map->config.x_size * current_map->config.y_size; i++)
	{
		if (log_odds_snapshot_map->complete_map[i] != log_odds_l0)
			current_map->complete_map[i] = carmen_prob_models_log_odds_to_probabilistic(log_odds_snapshot_map->complete_map[i]);

		log_odds_snapshot_map->complete_map[i] = log_odds_l0;
	}
}


void
carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(carmen_map_t *current_map, carmen_map_t *snapshot_map)
{	// @@@ Alberto: Se o snapshot_map for de log_odds esta funcao esta errada: log_odds podem ser menores que zero e iguais a -1
	// Para esta funcao funcionar os dois mapas tem que ser probabilisticos
	int xi, yi;
	
	for (xi = 0; xi < current_map->config.x_size; xi++)
	{
		for (yi = 0; yi < current_map->config.y_size; yi++)
		{
			if (snapshot_map->map[xi][yi] > 0.0)
				current_map->map[xi][yi] = snapshot_map->map[xi][yi];

			snapshot_map->map[xi][yi] = -1.0;
		}
	}
}


inline void
compute_intermediate_pixel_distance(int x, int y,
		double **distance, short int **x_offset, short int **y_offset)
{
	for (int i = -1; i < 2; i++)
		for (int j = -1; j < 2; j++)
		{
			double v = distance[x + i][y + j] + ((i * j != 0) ? 1.414213562 : 1.0);
			if (v < distance[x][y])
			{
				int xpi = x + i;
				int ypj = y + j;
				distance[x][y] = v;
				x_offset[x][y] = x_offset[xpi][ypj] + i;
				y_offset[x][y] = y_offset[xpi][ypj] + j;
			}
		}
}


void
carmen_prob_models_initialize_distance_map(carmen_prob_models_distance_map *lmap, carmen_map_config_t config)
{
	int i;

	if (lmap->complete_distance == NULL)
	{
		/* copy map parameters from carmen map */
		lmap->config = config;

		/* allocate distance map */
		lmap->complete_distance = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_distance);

		lmap->distance = (double **) calloc(lmap->config.x_size,
				sizeof(double *));
		carmen_test_alloc(lmap->distance);

		/* allocate x offset map */
		lmap->complete_x_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_x_offset);
		lmap->x_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->x_offset);
		/* allocate y offset map */
		lmap->complete_y_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_y_offset);
		lmap->y_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->y_offset);
	}
	else
	{
		/* copy map parameters from carmen map */
		lmap->config = config;

		memset(lmap->complete_distance, 0,
				lmap->config.x_size * lmap->config.y_size * sizeof(double));
		memset(lmap->distance, 0, lmap->config.x_size * sizeof(double *));
		memset(lmap->complete_x_offset, 0,
				lmap->config.x_size * lmap->config.y_size * sizeof(short int));
		memset(lmap->x_offset, 0, lmap->config.x_size * sizeof(short int *));
		memset(lmap->complete_y_offset, 0,
				lmap->config.x_size * lmap->config.y_size * sizeof(short int));
		memset(lmap->y_offset, 0, lmap->config.x_size * sizeof(short int *));
	}

	for (i = 0; i < lmap->config.x_size; i++)
	{
		lmap->distance[i] = lmap->complete_distance + i * lmap->config.y_size;
		lmap->x_offset[i] = lmap->complete_x_offset + i * lmap->config.y_size;
		lmap->y_offset[i] = lmap->complete_y_offset + i * lmap->config.y_size;
	}
}


void
initialize_distance_measurements(int x_size, int y_size,
		double minimum_occupied_prob, carmen_prob_models_distance_map* lmap,
		double** cmap_map, double** distance, short int** x_offset,
		short int** y_offset)
{
	int x, y;

	int total_size = x_size * y_size;
	std::fill_n(lmap->complete_distance, total_size, DISTANCE_MAP_HUGE_DISTANCE);
	std::fill_n(lmap->complete_x_offset, total_size, DISTANCE_MAP_HUGE_DISTANCE);
	std::fill_n(lmap->complete_y_offset, total_size, DISTANCE_MAP_HUGE_DISTANCE);

	for (x = 0; x < x_size; x++)
	{
		for (y = 0; y < y_size; y++)
		{
			if (cmap_map[x][y] > minimum_occupied_prob)
			{
				distance[x][y] = 0.0;
				x_offset[x][y] = 0;
				y_offset[x][y] = 0;
			}
		}
	}
}


void
carmen_prob_models_create_distance_map(carmen_prob_models_distance_map *lmap, carmen_map_p map, double minimum_occupied_prob)
{
	int x, y;

	lmap->config = map->config;

	double **cmap_map = map->map;
	double **distance = lmap->distance;
	short int **x_offset = lmap->x_offset;
	short int **y_offset = lmap->y_offset;

	int x_size = lmap->config.x_size;
	int y_size = lmap->config.y_size;

	initialize_distance_measurements(x_size, y_size, minimum_occupied_prob,
			lmap, cmap_map, distance, x_offset, y_offset);

	// Use dynamic programming to estimate the minimum distance from
	// every map cell to an occupied map cell

	// pass 1
	for (x = 1; x < x_size - 1; x++)
		for (y = 1; y < y_size - 1; y++)
			compute_intermediate_pixel_distance(x, y, distance, x_offset, y_offset);

	// pass 2
	for (x = x_size - 2; x >= 1; x--)
		for (y = y_size - 2; y >= 1; y--)
			compute_intermediate_pixel_distance(x, y, distance, x_offset, y_offset);
}


void
carmen_prob_models_create_masked_distance_map(carmen_prob_models_distance_map *lmap, carmen_map_p map,
		double minimum_occupied_prob, carmen_point_p robot_position, carmen_point_p goal_position)
{
#define DIST_SQR(x1,y1,x2,y2) ((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

	int x, y;

	lmap->config = map->config;

	double **cmap_map = map->map;
	double **distance = lmap->distance;
	short int **x_offset = lmap->x_offset;
	short int **y_offset = lmap->y_offset;

	int x_size = lmap->config.x_size;
	int y_size = lmap->config.y_size;

	int px = (robot_position->x - map->config.x_origin) / map->config.resolution;
	int py = (robot_position->y - map->config.y_origin) / map->config.resolution;
	int gx = (goal_position->x - map->config.x_origin) / map->config.resolution;
	int gy = (goal_position->y - map->config.y_origin) / map->config.resolution;

	int margin = 15.0 / map->config.resolution;

	int sqr_d = DIST_SQR(px,py,gx,gy) + margin * margin;

	initialize_distance_measurements(x_size, y_size, minimum_occupied_prob,
									lmap, cmap_map, distance, x_offset, y_offset);

	// Use dynamic programming to estimate the minimum distance from
	// every map cell to an occupied map cell

	// pass 1
	for (x = 1; x < x_size - 1; x++)
	{
		for (y = 1; y < y_size - 1; y++)
		{
			if ((DIST_SQR(px,py,x,y) < sqr_d) && (DIST_SQR(gx,gy,x,y) < sqr_d))
				compute_intermediate_pixel_distance(x, y, distance, x_offset, y_offset);
			else
			{
				distance[x][y] = HUGE_DISTANCE;
				x_offset[x][y] = carmen_imax((x_size-1) - x, x - 0);
				y_offset[x][y] = carmen_imax((y_size-1) - y, y - 0);
			}
		}
	}

	// pass 2
	for (x = x_size - 2; x >= 1; x--)
	{
		for (y = y_size - 2; y >= 1; y--)
		{
			if ((DIST_SQR(px,py,x,y) < sqr_d) && (DIST_SQR(gx,gy,x,y) < sqr_d))
				compute_intermediate_pixel_distance(x, y, distance, x_offset, y_offset);
			else
			{
				distance[x][y] = HUGE_DISTANCE;
				x_offset[x][y] = carmen_imax((x_size-1) - x, x - 0);
				y_offset[x][y] = carmen_imax((y_size-1) - y, y - 0);
			}
		}
	}
}


int
is_inside_ellipse(int x, int y, int f1x, int f1y, int f2x, int f2y, double major_axis)
{
    int xmf1x_sqd = carmen_square(x - f1x);
    int ymf1y_sqd = carmen_square(y - f1y);

    int xmf2x_sqd = carmen_square(x - f2x);
    int ymf2y_sqd = carmen_square(y - f2y);

    return sqrt(xmf1x_sqd + ymf1y_sqd) + sqrt(xmf2x_sqd + ymf2y_sqd) < major_axis;
}

/* compute minimum distance to all occupied cells */
void carmen_prob_models_create_masked_distance_map_old(carmen_prob_models_distance_map *lmap,
        carmen_map_p map, double minimum_occupied_prob, carmen_point_p robot_position,
        carmen_point_p goal_position)
{
//	double time_now = carmen_get_time();
    int x, y;

    lmap->config = map->config;

    double **cmap_map = map->map;
    double **distance = lmap->distance;
    short int **x_offset = lmap->x_offset;
    short int **y_offset = lmap->y_offset;

    int x_size = lmap->config.x_size;
    int y_size = lmap->config.y_size;

    /* Initialize the distance measurements before dynamic programming */
    	initialize_distance_measurements(x_size, y_size, minimum_occupied_prob,
    			lmap, cmap_map, distance, x_offset, y_offset);

    /* Use dynamic programming to estimate the minimum distance from
     every map cell to an occupied map cell */
    if (NULL != robot_position && NULL != goal_position)
    {
        double inverse_resolution = 1.0/map->config.resolution;

        /* get the robot position in the map coordinates */
        int rx = floor((robot_position->x - map->config.x_origin) * inverse_resolution + 0.5);
        int ry = floor((robot_position->y - map->config.y_origin) * inverse_resolution + 0.5);

        /* get the goal position in the map coordinates */
        int gx = floor((goal_position->x - map->config.x_origin) * inverse_resolution + 0.5);
        int gy = floor((goal_position->y - map->config.y_origin) * inverse_resolution + 0.5);

        int major_axis = sqrt(carmen_square(ry - gy) + carmen_square(rx - gx)) + 15 * inverse_resolution;


        /* pass 1 */
        for (x = 1; x < x_size - 1; x++)
            for (y = 1; y < y_size - 1; y++)
                if (is_inside_ellipse(x, y, rx, ry, gx, gy, major_axis))
                    compute_intermediate_pixel_distance(x, y, distance, x_offset, y_offset);
                else
                {
//                	x_far_far_away = (x > ((x_size-1)/2) ? 1 : (x_size - 2));
//                	y_far_far_away = (y > ((y_size-1)/2) ? 1 : (x_size - 2));
                    distance[x][y] = HUGE_DISTANCE;
                    x_offset[x][y] = (x > ((x_size-1)/2)) ? 1 : (x_size - 2);
                    y_offset[x][y] = (y > ((y_size-1)/2)) ? 1 : (x_size - 2);
                }

        /* pass 2 */
        for (x = x_size - 2; x >= 1; x--)
            for (y = y_size - 2; y >= 1; y--)
                if (is_inside_ellipse(x, y, rx, ry, gx, gy, major_axis))
                    compute_intermediate_pixel_distance(x, y, distance, x_offset, y_offset);
                else
                {
                    distance[x][y] = HUGE_DISTANCE;
                    x_offset[x][y] = (x > ((x_size-1)/2)) ? 1 : (x_size - 2);
                    y_offset[x][y] = (y > ((y_size-1)/2)) ? 1 : (x_size - 2);
                }

    }
}
