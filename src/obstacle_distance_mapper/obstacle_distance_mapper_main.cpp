#include <carmen/carmen.h>
#include "obstacle_distance_mapper_interface.h"
#include <carmen/grid_mapping_interface.h>

#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <iostream>
#include <string.h>

#define      HUGE_DISTANCE     32000


double obstacle_probability_threshold 	= 0.5;
double obstacle_cost_distance 			= 1.0;

carmen_map_t 						 map;
carmen_map_t 						 cost_map;
carmen_grid_mapping_distance_map 	 distance_map;

using namespace std;

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
carmen_mapper_initialize_distance_map(carmen_grid_mapping_distance_map *lmap, carmen_map_p cmap)
{
	int i;

	if (lmap->complete_distance == NULL)
	{
		/* copy map parameters from carmen map */
		lmap->config = cmap->config;

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
		lmap->config = cmap->config;

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

/* compute minimum distance to all occupied cells */
void
carmen_mapper_create_distance_map(carmen_grid_mapping_distance_map *lmap, carmen_map_p map,
		double minimum_occupied_prob)
{
	int x, y;

	lmap->config = map->config;

	double **cmap_map = map->map;
	double **distance = lmap->distance;
	short int **x_offset = lmap->x_offset;
	short int **y_offset = lmap->y_offset;

	int x_size = lmap->config.x_size;
	int y_size = lmap->config.y_size;

	int total_size = x_size * y_size;
	std::fill_n(lmap->complete_distance, total_size, HUGE_DISTANCE);
	std::fill_n(lmap->complete_x_offset, total_size, HUGE_DISTANCE);
	std::fill_n(lmap->complete_y_offset, total_size, HUGE_DISTANCE);

	/* Initialize the distance measurements before dynamic programming */
	for (x = 0; x < x_size; x++)
	{
		for (y = 0; y < y_size; y++)
		{
			if (cmap_map[x][y] > minimum_occupied_prob)
			{
				distance[x][y] = 0.0;
				x_offset[x][y] = 0.0;
				y_offset[x][y] = 0.0;
			}
		}
	}

	/* Use dynamic programming to estimate the minimum distance from
     every map cell to an occupied map cell */

	/* pass 1 */
	for (x = 1; x < x_size - 1; x++)
		for (y = 1; y < y_size - 1; y++)
			compute_intermediate_pixel_distance(x, y, distance, x_offset, y_offset);

	/* pass 2 */
	for (x = x_size - 2; x >= 1; x--)
		for (y = y_size - 2; y >= 1; y--)
			compute_intermediate_pixel_distance(x, y, distance, x_offset, y_offset);
}

void
carmen_mapper_build_obstacle_cost_map(carmen_map_t *cost_map, carmen_map_t *map, carmen_grid_mapping_distance_map *distance_map, double distance_for_zero_cost_in_pixels)
{
	carmen_prob_models_initialize_cost_map(cost_map, map, map->config.resolution);

	double resolution = distance_map->config.resolution;
	for (int x = 0; x < distance_map->config.x_size; x++)
	{
		for (int y = 0; y < distance_map->config.y_size; y++)
		{
			double distance = distance_map->distance[x][y] * resolution;
			cost_map->map[x][y] = (distance > distance_for_zero_cost_in_pixels)? 0.0: 1.0 - (distance / distance_for_zero_cost_in_pixels);
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
mapper_publish_distance_map(double timestamp, double obstacle_probability_threshold)
{
	if (distance_map.complete_distance == NULL)
		carmen_mapper_initialize_distance_map(&distance_map, &map);

	carmen_mapper_create_distance_map(&distance_map, &map, obstacle_probability_threshold);
	carmen_grid_mapping_publish_distance_map_message(&distance_map, timestamp);
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void
carmen_grid_mapping_map_handler(carmen_grid_mapping_message *msg)
{
	carmen_compact_map_t compacted_cost_map;

	carmen_grid_mapping_copy_map_from_message(&map, msg);

	mapper_publish_distance_map(msg->timestamp, obstacle_probability_threshold);
	carmen_mapper_build_obstacle_cost_map(&cost_map, &map, &distance_map, obstacle_cost_distance);
	carmen_prob_models_create_compact_map(&compacted_cost_map, &cost_map, 0.0);

	if (compacted_cost_map.number_of_known_points_on_the_map > 0)
	{
		carmen_map_server_publish_compact_cost_map_message(&compacted_cost_map,	msg->timestamp);
		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, &compacted_cost_map, 0.0);
		carmen_prob_models_free_compact_map(&compacted_cost_map);
	}

//	static double last_timestamp = 0.0;
//	double timestamp = carmen_get_time();
//	printf("delta_t %lf\n", timestamp - last_timestamp);
//	last_timestamp = timestamp;
}


void 
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("obstacle_distance_mapper: disconnected.\n");

    exit(0);
  }
}


static int
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *)"rrt",	(char *)"obstacle_cost_distance",	CARMEN_PARAM_DOUBLE,	&obstacle_cost_distance,	1, NULL},
			{(char *)"rrt",	(char *)"obstacle_probability_threshold",	CARMEN_PARAM_DOUBLE,	&obstacle_probability_threshold,	1, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

  
int 
main(int argc, char **argv) 
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  /* Read parameters */
  read_parameters(argc, argv);

  /* Subscribe to mapper messages */
  carmen_grid_mapping_subscribe_message(NULL,
  			       (carmen_handler_t) carmen_grid_mapping_map_handler,
  			       CARMEN_SUBSCRIBE_LATEST);

  /* Loop forever waiting for messages */
  carmen_ipc_dispatch();

  return (0);
}
