#include <carmen/carmen.h>
#include <car_model.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include "collision_detection.h"
#include "obstacle_avoider.h"

static int current_map = 0;
static carmen_map_p map_vector[NUM_MAPS];

static int current_pose = 0;
static carmen_robot_and_trailers_traj_point_t pose_vector[NUM_POSES];

static carmen_robot_and_trailers_traj_point_t trajectory_vector_of_points[MAX_TRAJECTORY_VECTOR_OF_POINTS_SIZE];

static carmen_obstacle_distance_mapper_map_message *obstacle_distance_map = NULL;


void
add_map_to_map_vector(carmen_mapper_map_message *message)
{
	if (current_map < NUM_MAPS - 1)
		current_map++;
	else
		current_map = 0;

	copy_grid_mapping_to_map_vector(message, current_map);
}


void
add_cost_map_to_map_vector(carmen_map_t *cost_map)
{
	if (current_map < NUM_MAPS - 1)
		current_map++;
	else
		current_map = 0;

	copy_cost_map_to_map_vector(cost_map, current_map);
}


void
obstacle_avoider_update_map(carmen_obstacle_distance_mapper_map_message *map)
{
	obstacle_distance_map = map;
}


carmen_obstacle_distance_mapper_map_message *
get_current_map()
{
	return (obstacle_distance_map);
}


carmen_robot_and_trailers_traj_point_t
get_current_pose()
{
	return(pose_vector[current_pose]);
}


void
add_pose_to_pose_vector(carmen_robot_and_trailers_traj_point_t pose)
{
	if (current_pose < NUM_POSES - 1)
	{
		current_pose++;
	}
	else
	{
		current_pose = 0;
	}

	pose_vector[current_pose] = pose;
}


void
initialize_map_vector(int number_of_maps)
{
	int i;

	for (i = 0; i < number_of_maps; i++)
	{
		map_vector[i] = NULL;
	}
}


//static void
//print_path(carmen_robot_and_trailer_motion_command_t *path, int size)
//{
//	FILE *caco = fopen("caco.txt", "a");
//
//	printf("\n** size %d, s %.3lf, timestamp %lf\n", size, DIST2D(path[0], path[size - 1]), carmen_get_time());
//	fprintf(caco, "\n** size %d, s %.3lf, timestamp %lf\n", size, DIST2D(path[0], path[size - 1]), carmen_get_time());
//	for (int i = 0; i < size; i++)
//	{
//		if (i < ((size > 10)? 10: size))
//			printf(" i %2d, x %.2f, y %.2f, phi %.2f, theta %.2f, v[i] %lf, dist_2D [i]->[i+1] %.2f, dt %.3lf\n", i,
//				path[i].x, path[i].y, path[i].phi, //carmen_radians_to_degrees(path[i].phi),
//				carmen_radians_to_degrees(path[i].theta),
//				path[i].v,
//				(i < size - 1)? DIST2D(path[i], path[i + 1]): 0.0, path[i].time); // O displacement eh o deslocamento para chegar ao proximo ponto.
//
//		fprintf(caco, " i %2d, x %.2f, y %.2f, phi %.2f, theta %.2f, v[i] %lf, dist_2D [i]->[i+1] %.2f, dt %.3lf\n", i,
//				path[i].x, path[i].y, path[i].phi, //carmen_radians_to_degrees(path[i].phi),
//				carmen_radians_to_degrees(path[i].theta),
//				path[i].v,
//				(i < size - 1)? DIST2D(path[i], path[i + 1]): 0.0, path[i].time);
//	}
//	fclose(caco);
//}


static int
build_predicted_trajectory(carmen_robot_and_trailers_motion_command_t *motion_commands_vector, int *num_motion_commands,
		carmen_robot_and_trailers_traj_point_t initial_pose, carmen_robot_ackerman_config_t *carmen_robot_ackerman_config, carmen_semi_trailer_config_t *carmen_semi_trailer_config)
{
	int i, trajectory_vector_of_points_size = 0;
	carmen_robot_and_trailers_traj_point_t pose;

	pose = initial_pose;

//	print_path(motion_commands_vector, *num_motion_commands);

	for (i = 0; i < *num_motion_commands; i++)
	{
		double distance_traveled = 0.0;
		pose = carmen_libcarmodel_recalc_pos_ackerman(pose, motion_commands_vector[i].v, motion_commands_vector[i].phi,
				motion_commands_vector[i].time, &distance_traveled, motion_commands_vector[i].time,
				*carmen_robot_ackerman_config, *carmen_semi_trailer_config);

		motion_commands_vector[i].x = pose.x;
		motion_commands_vector[i].y = pose.y;
		motion_commands_vector[i].theta = pose.theta;
		motion_commands_vector[i].num_trailers = pose.num_trailers;
		motion_commands_vector[i].trailer_theta[0] = pose.trailer_theta[0];
		motion_commands_vector[i].trailer_theta[1] = pose.trailer_theta[1];
		motion_commands_vector[i].trailer_theta[2] = pose.trailer_theta[2];
		motion_commands_vector[i].trailer_theta[3] = pose.trailer_theta[3];
		motion_commands_vector[i].trailer_theta[4] = pose.trailer_theta[4];

		trajectory_vector_of_points[trajectory_vector_of_points_size] = pose;
		trajectory_vector_of_points_size++;
		if (trajectory_vector_of_points_size >= (MAX_TRAJECTORY_VECTOR_OF_POINTS_SIZE - 2))
			break;
	}
	*num_motion_commands = trajectory_vector_of_points_size;

	return (trajectory_vector_of_points_size);
}


carmen_navigator_ackerman_plan_message
build_navigator_ackerman_plan_message(carmen_robot_and_trailers_motion_command_t *motion_commands_vector, int *num_motion_commands,
			carmen_robot_ackerman_config_t *carmen_robot_ackerman_config, carmen_semi_trailer_config_t *carmen_semi_trailer_config, double timestamp)
{
	int i, trajectory_vector_of_points_size;
	carmen_navigator_ackerman_plan_message predicted_trajectory_message;

	trajectory_vector_of_points_size = build_predicted_trajectory(motion_commands_vector, num_motion_commands, pose_vector[current_pose], carmen_robot_ackerman_config, carmen_semi_trailer_config);

	predicted_trajectory_message.path_length = trajectory_vector_of_points_size;
	predicted_trajectory_message.path = (carmen_robot_and_trailers_traj_point_t *) malloc(sizeof(carmen_robot_and_trailers_traj_point_t) * (trajectory_vector_of_points_size));

	for (i = 0; i < trajectory_vector_of_points_size; i++)
	{
		predicted_trajectory_message.path[i].x 		= trajectory_vector_of_points[i].x;
		predicted_trajectory_message.path[i].y 		= trajectory_vector_of_points[i].y;
		predicted_trajectory_message.path[i].theta 	= trajectory_vector_of_points[i].theta;
		predicted_trajectory_message.path[i].num_trailers 	= trajectory_vector_of_points[i].num_trailers;
		predicted_trajectory_message.path[i].trailer_theta[0] 	= trajectory_vector_of_points[i].trailer_theta[0];
		predicted_trajectory_message.path[i].trailer_theta[1] 	= trajectory_vector_of_points[i].trailer_theta[1];
		predicted_trajectory_message.path[i].trailer_theta[2] 	= trajectory_vector_of_points[i].trailer_theta[2];
		predicted_trajectory_message.path[i].trailer_theta[3] 	= trajectory_vector_of_points[i].trailer_theta[3];
		predicted_trajectory_message.path[i].trailer_theta[4] 	= trajectory_vector_of_points[i].trailer_theta[4];
		predicted_trajectory_message.path[i].v 		= trajectory_vector_of_points[i].v;
		predicted_trajectory_message.path[i].phi 	= trajectory_vector_of_points[i].phi;
	}

	predicted_trajectory_message.timestamp = timestamp;
	predicted_trajectory_message.host = carmen_get_host();

	return (predicted_trajectory_message);
}


void
copy_grid_mapping_to_map_vector(carmen_mapper_map_message *grid_map, int position)
{
	int i;

	if (map_vector[position] == NULL)
	{
		map_vector[position] = (carmen_map_t *) malloc (sizeof(carmen_map_t));
		carmen_test_alloc(map_vector[position]);
		map_vector[position]->complete_map = (double *) malloc(sizeof(double) * grid_map->size);
		carmen_test_alloc(map_vector[position]->complete_map);
		map_vector[position]->map = (double **)calloc(grid_map->config.x_size, sizeof(double *));
		carmen_test_alloc(map_vector[position]->map);
	}
	else if ((grid_map->size != (map_vector[position]->config.x_size * map_vector[position]->config.y_size)) || // o novo mapa pode ser de tamanho diferente...
			(grid_map->config.x_size != map_vector[position]->config.x_size))
	{
		free(map_vector[position]->map);
		free(map_vector[position]->complete_map);

		map_vector[position]->complete_map = (double *) malloc(sizeof(double) * grid_map->size);
		carmen_test_alloc(map_vector[position]->complete_map);
		map_vector[position]->map = (double **)calloc(grid_map->config.x_size, sizeof(double *));
		carmen_test_alloc(map_vector[position]->map);
	}

	map_vector[position]->config = grid_map->config;

	memcpy(map_vector[position]->complete_map, grid_map->complete_map, sizeof(double) * grid_map->size);

	for (i = 0; i < map_vector[position]->config.x_size; i++)
		map_vector[position]->map[i] = map_vector[position]->complete_map + i * map_vector[position]->config.y_size;
}


void
copy_cost_map_to_map_vector(carmen_map_t *cost_map, int position)
{
	int i;

	if (map_vector[position] == NULL)
	{
		map_vector[position] = (carmen_map_t *) malloc (sizeof(carmen_map_t));
		carmen_test_alloc(map_vector[position]);
		map_vector[position]->complete_map = (double *) malloc(sizeof(double) * cost_map->config.x_size * cost_map->config.y_size);
		carmen_test_alloc(map_vector[position]->complete_map);
		map_vector[position]->map = (double **)calloc(cost_map->config.x_size, sizeof(double *));
		carmen_test_alloc(map_vector[position]->map);
	}
	else if (((cost_map->config.x_size * cost_map->config.y_size) != (map_vector[position]->config.x_size * map_vector[position]->config.y_size)) || // o novo mapa pode ser de tamanho diferente...
			(cost_map->config.x_size != map_vector[position]->config.x_size))
	{
		free(map_vector[position]->map);
		free(map_vector[position]->complete_map);

		map_vector[position]->complete_map = (double *) malloc(sizeof(double) * cost_map->config.x_size * cost_map->config.y_size);
		carmen_test_alloc(map_vector[position]->complete_map);
		map_vector[position]->map = (double **)calloc(cost_map->config.x_size, sizeof(double *));
		carmen_test_alloc(map_vector[position]->map);
	}

	map_vector[position]->config = cost_map->config;

	memcpy(map_vector[position]->complete_map, cost_map->complete_map, sizeof(double) * cost_map->config.x_size * cost_map->config.y_size);

	for (i = 0; i < map_vector[position]->config.x_size; i++)
		map_vector[position]->map[i] = map_vector[position]->complete_map + i * map_vector[position]->config.y_size;
}


double
get_last_motion_command_total_time(carmen_robot_and_trailers_motion_command_t *motion_command_vector, int num_motion_commands)
{
	double motion_command_total_time;
	int i;

	motion_command_total_time = 0.0;

	for (i = 0; i < num_motion_commands; i++)
		motion_command_total_time += motion_command_vector[i].time;

	return motion_command_total_time;
}


/***************************************************************************
		   --- Obstacle Avoider Core ---
 ***************************************************************************/

static int
identify_unstoppable_colision(double delta_time, double delta_velocity, carmen_robot_ackerman_config_t *carmen_robot_ackerman_config)
{
	double deceleration = delta_velocity / delta_time;

	return (deceleration > carmen_robot_ackerman_config->maximum_deceleration_forward);
}


static void
velocity_recalculate(carmen_robot_and_trailers_motion_command_t *motion_commands_vector, int num_motion_commands)
{
	int i;
	double reduction_factor;

	reduction_factor = 0.95;
	for (i = 0; i < num_motion_commands; i++)
	{
		if (i != 0)
			reduction_factor *= 0.95;
		motion_commands_vector[i].v *= reduction_factor;
		motion_commands_vector[i].time *= 1.0 / reduction_factor;
		
		if (fabs(motion_commands_vector[i].v) < 0.05)
		{
			motion_commands_vector[i].time = 1.0;
			motion_commands_vector[i].v = 0.0;
			break;
		}
	}
	for ( ; i < num_motion_commands; i++)
	{
		motion_commands_vector[i].time = 1.0;
		motion_commands_vector[i].v = 0.0;
	}
}


int
obstacle_avoider(carmen_robot_and_trailers_motion_command_t *motion_commands_vector, int *num_motion_commands, carmen_robot_ackerman_config_t *carmen_robot_ackerman_config, carmen_semi_trailer_config_t *carmen_semi_trailer_config)
{
	int pose_index = current_pose;
	int hit_obstacle = 0;
	
	if (obstacle_distance_map != NULL)
	{
		int iter = 0;
		do
		{
			hit_obstacle = 0;
			*num_motion_commands = build_predicted_trajectory(motion_commands_vector, num_motion_commands, pose_vector[pose_index], carmen_robot_ackerman_config, carmen_semi_trailer_config);

			for (int i = 0; i < *num_motion_commands; i++)
			{
				if (trajectory_pose_hit_obstacle(trajectory_vector_of_points[i], carmen_robot_ackerman_config->obstacle_avoider_obstacles_safe_distance,
						obstacle_distance_map, carmen_robot_ackerman_config))
				{
					if (identify_unstoppable_colision(motion_commands_vector[0].time * (*num_motion_commands), motion_commands_vector[0].v, carmen_robot_ackerman_config))
					{
						// Deveria gerar um alerta...
					}
					velocity_recalculate(motion_commands_vector, *num_motion_commands);
					hit_obstacle = 1;
					break;
				}
			}
			iter++;
		} while (hit_obstacle && (fabs(motion_commands_vector[0].v) > 0.001) && (iter < 500));
	}
	else
	{
		publish_base_ackerman_motion_command_message_to_stop_robot("Error: no map available in function obstacle_avoider()\n");
	}

	return (hit_obstacle);
}
