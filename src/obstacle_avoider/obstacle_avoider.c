

/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include "collision_detection.h"
#include "obstacle_avoider.h"

static int current_map = 0;
static carmen_map_p map_vector[NUM_MAPS];

static int current_pose = 0;
static carmen_ackerman_traj_point_t pose_vector[NUM_POSES];

static carmen_ackerman_traj_point_t trajectory_vector_of_points[MAX_TRAJECTORY_VECTOR_OF_POINTS_SIZE];



void
add_map_to_map_vector(carmen_grid_mapping_message *message)
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
add_pose_to_pose_vector(carmen_ackerman_traj_point_t pose)
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


static int
build_predicted_trajectory(carmen_ackerman_motion_command_p motion_commands_vector, int num_motion_commands,
		carmen_ackerman_traj_point_t initial_pose, carmen_robot_ackerman_config_t *carmen_robot_ackerman_config)
{
	int i, trajectory_vector_of_points_size = 0;
	carmen_ackerman_traj_point_t pose;

	pose = initial_pose;

	for (i = 0; i < num_motion_commands; i++)
	{
		pose = predict_new_robot_position(pose, motion_commands_vector[i].v, motion_commands_vector[i].phi, motion_commands_vector[i].time, carmen_robot_ackerman_config);
		trajectory_vector_of_points[trajectory_vector_of_points_size] = pose;
		trajectory_vector_of_points_size++;
		if (trajectory_vector_of_points_size >= (MAX_TRAJECTORY_VECTOR_OF_POINTS_SIZE - 2))
			break;
	}
	return (trajectory_vector_of_points_size);
}


carmen_navigator_ackerman_plan_message
build_navigator_ackerman_plan_message(carmen_ackerman_motion_command_p motion_commands_vector, int num_motion_commands,
			carmen_robot_ackerman_config_t *carmen_robot_ackerman_config)
{
	int i, trajectory_vector_of_points_size;
	carmen_navigator_ackerman_plan_message predicted_trajectory_message;

	trajectory_vector_of_points_size = build_predicted_trajectory(motion_commands_vector, num_motion_commands, pose_vector[current_pose], carmen_robot_ackerman_config);

	predicted_trajectory_message.path_length = trajectory_vector_of_points_size;
	predicted_trajectory_message.path = (carmen_ackerman_traj_point_t *) malloc(sizeof(carmen_ackerman_traj_point_t) * (trajectory_vector_of_points_size));

	for (i = 0; i < trajectory_vector_of_points_size; i++)
	{
		predicted_trajectory_message.path[i].x 		= trajectory_vector_of_points[i].x;
		predicted_trajectory_message.path[i].y 		= trajectory_vector_of_points[i].y;
		predicted_trajectory_message.path[i].theta 	= trajectory_vector_of_points[i].theta;
		predicted_trajectory_message.path[i].v 		= trajectory_vector_of_points[i].v;
		predicted_trajectory_message.path[i].phi 	= trajectory_vector_of_points[i].phi;
	}

	predicted_trajectory_message.timestamp = carmen_get_time();
	predicted_trajectory_message.host = carmen_get_host();

	return predicted_trajectory_message;
}


void
copy_grid_mapping_to_map_vector(carmen_grid_mapping_message *grid_map, int position)
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
get_last_motion_command_total_time(carmen_ackerman_motion_command_p motion_command_vector, int num_motion_commands)
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
velocity_recalculate(carmen_ackerman_motion_command_t *motion_commands_vector, int num_motion_commands)
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
obstacle_avoider(carmen_ackerman_motion_command_t *motion_commands_vector, int num_motion_commands, carmen_robot_ackerman_config_t *carmen_robot_ackerman_config)
{
	int map_index = current_map;
	int pose_index = current_pose;
	int hit_obstacle = 0;
	
	if (map_vector[map_index] != NULL)
	{
		do
		{
			hit_obstacle = 0;
			int trajectory_lenght = build_predicted_trajectory(motion_commands_vector, num_motion_commands, pose_vector[pose_index], carmen_robot_ackerman_config);

			for (int i = 0; i < trajectory_lenght; i++)
			{
				if (obstacle_avoider_pose_hit_obstacle(to_carmen_point_t(&(trajectory_vector_of_points[i])), map_vector[map_index], carmen_robot_ackerman_config))
				{
					if (identify_unstoppable_colision(motion_commands_vector[0].time * num_motion_commands, motion_commands_vector[0].v, carmen_robot_ackerman_config))
					{
						// Deveria gerar um alerta...
					}
					velocity_recalculate(motion_commands_vector, num_motion_commands);
					hit_obstacle = 1;
					break;
				}
			}
		} while (hit_obstacle && (fabs(motion_commands_vector[0].v) > 0.001));
	}
	else
	{
		publish_base_ackerman_motion_command_message_to_stop_robot("Error: no map available in function obstacle_avoider()\n");
	}

	return (hit_obstacle);
}
