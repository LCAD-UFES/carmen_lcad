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

/****************************************
 * library to create and control random *
 * two-legged people                    *
 ****************************************/

#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/mapper_interface.h>
#include <carmen/route_planner_interface.h>

#include "simulator_ackerman.h"
#include "objects_ackerman.h"

#define MAX_STEP  0.40  //m

static int num_objects = 0;
static int list_capacity = 0;
static carmen_traj_point_t *traj_object_list = NULL;
static carmen_object_ackerman_t *object_list = NULL;

static double leg_width = .1;
static double min_dist_from_robot = .4;
static double person_speed = .3;

extern carmen_route_planner_road_network_message *road_network_message;
extern int autonomous;

static void update_other_robot(carmen_object_ackerman_t *object);


static void check_list_capacity(void)
{
	if (traj_object_list == NULL)
	{
		list_capacity = 10;
		traj_object_list = (carmen_traj_point_t *) calloc(10, sizeof(carmen_traj_point_t));
		carmen_test_alloc(traj_object_list);
		object_list = (carmen_object_ackerman_t *) calloc(10, sizeof(carmen_object_ackerman_t));
		carmen_test_alloc(object_list);
	}
	else if (num_objects >= list_capacity)
	{
		list_capacity += 10;

		traj_object_list = (carmen_traj_point_t *) realloc(traj_object_list, sizeof(carmen_traj_point_t) * list_capacity);
		carmen_test_alloc(traj_object_list);

		object_list = (carmen_object_ackerman_t *) realloc(object_list, sizeof(carmen_object_ackerman_t) * list_capacity);
		carmen_test_alloc(object_list);
	}
}

void carmen_simulator_ackerman_create_object(double x, double y, double theta, carmen_simulator_ackerman_object_t type, double speed)
{
	check_list_capacity();

	if (speed == -1)
		speed = fabs(carmen_gaussian_random(0.0, person_speed / 2.0) + person_speed / 2.0);

	object_list[num_objects].type = type;
	object_list[num_objects].is_robot = 0;

	object_list[num_objects].x1 = x + cos(theta + M_PI / 2) * MAX_STEP / 4;
	object_list[num_objects].y1 = y + sin(theta + M_PI / 2) * MAX_STEP / 4;
	object_list[num_objects].x2 = x - cos(theta + M_PI / 2) * MAX_STEP / 4;
	object_list[num_objects].y2 = y - sin(theta + M_PI / 2) * MAX_STEP / 4;

	object_list[num_objects].theta = theta;
	object_list[num_objects].width = leg_width;

	object_list[num_objects].tv = speed;
	object_list[num_objects].rv = 0;

	object_list[num_objects].lane_id = 0;

	traj_object_list[num_objects].t_vel = speed;
	traj_object_list[num_objects].r_vel = speed;
	traj_object_list[num_objects].x = x;
	traj_object_list[num_objects].y = y;
	traj_object_list[num_objects].theta = theta;

	num_objects++;
}

static int in_map(double x, double y, carmen_map_p map)
{
	int map_x, map_y;
	double occupancy;

	map_x = carmen_trunc((x - map->config.x_origin) / map->config.resolution);
	map_y = carmen_trunc((y - map->config.y_origin) / map->config.resolution);

	if (map_x < 0 || map_x > map->config.x_size || map_y < 0 || map_y > map->config.y_size)
	{
		return 0;
	}

	occupancy = map->map[map_x][map_y];

	if (occupancy > 0.5 || occupancy < 0)
		return 0;

	return 1;
}

carmen_ackerman_traj_point_t *search_old(carmen_ackerman_traj_point_t key, carmen_ackerman_traj_point_t *lane, int size)
{
	int begin = 0;
	int end = size - 1;

	int count = 0;
	while (begin != end)
	{
		double dist_begin = DIST2D(key, lane[begin]);
		double dist_end = DIST2D(key, lane[end]);
		if (dist_begin > dist_end)
			begin = (begin + end) / 2;
		else
			end = (begin + end) / 2;

		count++;
		if (count > size)
			break;
	}

	return (&(lane[begin]));
}

carmen_ackerman_traj_point_t *
find_nearest_pose_in_lane(carmen_ackerman_traj_point_t key, carmen_ackerman_traj_point_t *lane, int size, int *index_in_lane)
{
	double min_distance = DIST2D(key, lane[0]);
	int index = 0;
	for (int i = 1; i < size - 1; i++)
	{
		double distance = DIST2D(key, lane[i]);
		if (distance < min_distance)
		{
			index = i;
			min_distance = distance;
		}
	}

	*index_in_lane = index;

	return (&(lane[index]));
}


carmen_ackerman_traj_point_t *
find_nearest_pose_in_the_nearest_lane(carmen_object_ackerman_t *object, carmen_ackerman_traj_point_t pose,
		int *lane_size, int *index_in_lane)
{
	carmen_ackerman_traj_point_t *nearest_pose_in_the_nearest_lane = NULL;
	double nearest_distance = 100000000000.0;
	if (object->lane_id == 0)
	{
		int lane_index = 0;
		for (int i = 0; i < road_network_message->number_of_nearby_lanes; i++)
		{
			carmen_ackerman_traj_point_t *lane = &(road_network_message->nearby_lanes[road_network_message->nearby_lanes_indexes[i]]);
			int size = road_network_message->nearby_lanes_sizes[i];
			int index;
			carmen_ackerman_traj_point_t *nearest_pose_in_the_lane = find_nearest_pose_in_lane(pose, lane, size, &index);
			double distance = DIST2D(pose, *nearest_pose_in_the_lane);
			if (distance < nearest_distance)
			{
				lane_index = i;
				nearest_distance = distance;
				nearest_pose_in_the_nearest_lane = nearest_pose_in_the_lane;
				*lane_size = size;
				*index_in_lane = index;
			}
		}

		object->lane_id = road_network_message->nearby_lanes_ids[lane_index];
	}
	else
	{
		for (int i = 0; i < road_network_message->number_of_nearby_lanes; i++)
		{
			if (object->lane_id == road_network_message->nearby_lanes_ids[i])
			{
				carmen_ackerman_traj_point_t *lane = &(road_network_message->nearby_lanes[road_network_message->nearby_lanes_indexes[i]]);
				*lane_size = road_network_message->nearby_lanes_sizes[i];
				nearest_pose_in_the_nearest_lane = find_nearest_pose_in_lane(pose, lane, *lane_size, index_in_lane);
			}
		}
	}

	return (nearest_pose_in_the_nearest_lane);
}

static void update_object_in_lane(int i, carmen_simulator_ackerman_config_t *simulator_config)
{
	double dt = simulator_config->delta_t;
	carmen_object_ackerman_t new_object = object_list[i];
	double dx = new_object.tv * dt * cos(new_object.theta);
	double dy = new_object.tv * dt * sin(new_object.theta);
//	printf(" v %lf, dt %lf, x %lf, y %lf\n", new_object.tv, dt, new_object.x1, new_object.y1);
	new_object.x1 += dx;
	new_object.y1 += dy;

	carmen_ackerman_traj_point_t pose_ahead;
	pose_ahead.x = new_object.x1;
	pose_ahead.y = new_object.y1;
	int lane_size;
	int index_in_lane;
	carmen_ackerman_traj_point_t *pose_in_lane = find_nearest_pose_in_the_nearest_lane(&new_object, pose_ahead, &lane_size, &index_in_lane);
	int status;
	carmen_ackerman_traj_point_t next_pose;
	if (index_in_lane == 0)
		next_pose = carmen_get_point_nearest_to_trajectory(&status, pose_in_lane[0], pose_in_lane[1], pose_ahead, 0.1);
	else if (index_in_lane >= (lane_size - 1))
		next_pose = carmen_get_point_nearest_to_trajectory(&status, pose_in_lane[-1], pose_in_lane[0], pose_ahead, 0.1);
	else
		next_pose = carmen_get_point_nearest_to_trajectory(&status, pose_in_lane[-1], pose_in_lane[1], pose_ahead, 0.1);
	new_object.x1 = next_pose.x;
	new_object.y1 = next_pose.y;
	new_object.theta = next_pose.theta;
//	printf("*v %lf, dt %lf, x %lf, y %lf\n", new_object.tv, dt, new_object.x1, new_object.y1);
//	new_object.tv = new_object.tv + 0.05 * (simulator_config->v - new_object.tv);
	new_object.rv = 0.0;

	object_list[i] = new_object;
	object_list[i].time_of_last_update = carmen_get_time();
}

/* randomly updates the person's position based on its velocity */
static void update_random_object(int i, carmen_simulator_ackerman_config_t *simulator_config)
{
	carmen_object_ackerman_t new_object;
	double vx, vy;
	double separation;
	double dist;

	double mean_x, mean_y, delta_x, delta_y;

	new_object = object_list[i];
	if (!new_object.is_robot)
	{
		separation = hypot(new_object.x1 - new_object.x2, new_object.y1 - new_object.y2);
		if (separation > MAX_STEP)
		{
			mean_x = (new_object.x1 + new_object.x2) / 2;
			mean_y = (new_object.y1 + new_object.y2) / 2;

			delta_x = fabs((new_object.x1 - new_object.x2) / 2);
			delta_y = fabs((new_object.y1 - new_object.y2) / 2);

			new_object.x1 = carmen_gaussian_random(mean_x, delta_x);
			new_object.x2 = carmen_gaussian_random(mean_x, delta_x);
			new_object.y1 = carmen_gaussian_random(mean_y, delta_y);
			new_object.y2 = carmen_gaussian_random(mean_y, delta_y);
		}
		else
		{
			vx = new_object.tv * cos(new_object.theta) * simulator_config->delta_t;
			vy = new_object.tv * sin(new_object.theta) * simulator_config->delta_t;
			new_object.x1 += carmen_gaussian_random(vx, vx / 10.0);
			new_object.x2 += carmen_gaussian_random(vx, vx / 10.0);
			new_object.y1 += carmen_gaussian_random(vy, vy / 10.0);
			new_object.y2 += carmen_gaussian_random(vy, vy / 10.0);

		}

		dist = hypot((new_object.x1 + new_object.x2) / 2.0 - simulator_config->true_pose.x,
				(new_object.y1 + new_object.y2) / 2.0 - simulator_config->true_pose.y);

		if (!in_map(new_object.x1, new_object.y1, &(simulator_config->map)) || !in_map(new_object.x2, new_object.y2, &(simulator_config->map))
				|| dist < min_dist_from_robot || carmen_simulator_object_too_close(new_object.x1, new_object.y1, i)
				|| carmen_simulator_object_too_close(new_object.x2, new_object.y2, i))
		{
			object_list[i].theta = carmen_normalize_theta(carmen_gaussian_random(M_PI + object_list[i].theta, M_PI / 4));
			return;
		}
	}
	else
	{
		vx = new_object.tv * cos(new_object.theta) * simulator_config->delta_t;
		vy = new_object.tv * sin(new_object.theta) * simulator_config->delta_t;
		new_object.x1 += carmen_gaussian_random(vx, vx / 10.0);
		new_object.y1 += carmen_gaussian_random(vy, vy / 10.0);

		dist = hypot(new_object.x1 - simulator_config->true_pose.x, new_object.y1 - simulator_config->true_pose.y);

		if (!in_map(new_object.x1, new_object.y1, &(simulator_config->map)) || dist < min_dist_from_robot
				|| carmen_simulator_object_too_close(new_object.x1, new_object.y1, i))
		{
			object_list[i].theta = carmen_normalize_theta(carmen_gaussian_random(M_PI + object_list[i].theta, M_PI / 4));

			return;
		}
	}
	object_list[i] = new_object;
	object_list[i].time_of_last_update = carmen_get_time();
}

static void update_line_follower(carmen_object_ackerman_t * object, carmen_simulator_ackerman_config_t *simulator_config)
{
	carmen_object_ackerman_t new_object;
	double vx, vy;
	double separation;
	double dist = 0;

	double mean_x, mean_y, delta_x, delta_y;

	new_object = *object;

	if (!object->is_robot)
	{
		separation = hypot(new_object.x1 - new_object.x2, new_object.y1 - new_object.y2);

		if (separation > MAX_STEP)
		{
			mean_x = (new_object.x1 + new_object.x2) / 2;
			mean_y = (new_object.y1 + new_object.y2) / 2;

			delta_x = fabs((new_object.x1 - new_object.x2) / 2);
			delta_y = fabs((new_object.y1 - new_object.y2) / 2);

			new_object.x1 = carmen_gaussian_random(mean_x, delta_x);
			new_object.x2 = carmen_gaussian_random(mean_x, delta_x);
			new_object.y1 = carmen_gaussian_random(mean_y, delta_y);
			new_object.y2 = carmen_gaussian_random(mean_y, delta_y);
		}
		else
		{
			vx = new_object.tv * cos(new_object.theta) * simulator_config->delta_t;
			vy = new_object.tv * sin(new_object.theta) * simulator_config->delta_t;
			new_object.x1 += carmen_gaussian_random(vx, vx / 10.0);
			new_object.x2 += carmen_gaussian_random(vx, vx / 10.0);
			new_object.y1 += carmen_gaussian_random(vy, vy / 10.0);
			new_object.y2 += carmen_gaussian_random(vy, vy / 10.0);

		}

		dist = hypot((object->x1 + object->x2) / 2.0 - simulator_config->true_pose.x, (object->y1 + object->y2) / 2.0 - simulator_config->true_pose.y);

		if (!in_map(new_object.x1, new_object.y1, &(simulator_config->map)) || !in_map(new_object.x2, new_object.y2, &(simulator_config->map))
				|| dist < min_dist_from_robot)
		{
			return;
		}
	}
	else
	{
		vx = new_object.tv * cos(new_object.theta) * simulator_config->delta_t;
		vy = new_object.tv * sin(new_object.theta) * simulator_config->delta_t;
		new_object.x1 += carmen_gaussian_random(vx, vx / 10.0);
		new_object.y1 += carmen_gaussian_random(vy, vy / 10.0);

		dist = hypot(object->x1 - simulator_config->true_pose.x, object->y1 - simulator_config->true_pose.y);

		if (!in_map(new_object.x1, new_object.y1, &(simulator_config->map)) || dist < min_dist_from_robot)
		{
			return;
		}
	}
	*object = new_object;
	object->time_of_last_update = carmen_get_time();
}

static void update_traj_object(int index)
{
	if (object_list[index].type == CARMEN_SIMULATOR_ACKERMAN_OTHER_ROBOT)
	{
		traj_object_list[index].x = object_list[index].x1;
		traj_object_list[index].y = object_list[index].y1;
		traj_object_list[index].theta = object_list[index].theta;
		traj_object_list[index].t_vel = object_list[index].tv;
		traj_object_list[index].r_vel = object_list[index].rv;
	}
	else
	{
		traj_object_list[index].x = (object_list[index].x1 + object_list[index].x2) / 2.0;
		traj_object_list[index].y = (object_list[index].y1 + object_list[index].y2) / 2.0;
		traj_object_list[index].theta = object_list[index].theta;
		traj_object_list[index].t_vel = object_list[index].tv;
	}
}

void
add_rectangle_to_map(carmen_mapper_virtual_laser_message *virtual_laser_message, double width, double length, int index)
{
	double logitudinal_disp = -length / 2.0;
	double lateral_disp = -width / 2.0;
	for ( ; lateral_disp < width / 2.0; lateral_disp = lateral_disp + 0.2)
	{
		virtual_laser_message->positions[virtual_laser_message->num_positions].x = object_list[index].x1 +
				lateral_disp * cos(object_list[index].theta + M_PI / 2.0) +
				logitudinal_disp * cos(object_list[index].theta);
		virtual_laser_message->positions[virtual_laser_message->num_positions].y = object_list[index].y1 +
				lateral_disp * sin(object_list[index].theta + M_PI / 2.0) +
				logitudinal_disp * sin(object_list[index].theta);
		virtual_laser_message->colors[virtual_laser_message->num_positions] = CARMEN_PURPLE;
		virtual_laser_message->num_positions++;

		virtual_laser_message->positions[virtual_laser_message->num_positions].x = object_list[index].x1 +
				lateral_disp * cos(object_list[index].theta - M_PI / 2.0) +
				logitudinal_disp * cos(object_list[index].theta);
		virtual_laser_message->positions[virtual_laser_message->num_positions].y = object_list[index].y1 +
				lateral_disp * sin(object_list[index].theta - M_PI / 2.0) +
				logitudinal_disp * sin(object_list[index].theta);
		virtual_laser_message->colors[virtual_laser_message->num_positions] = CARMEN_PURPLE;
		virtual_laser_message->num_positions++;
	}

	lateral_disp = width / 2.0;
	for ( ; logitudinal_disp < length / 2.0; logitudinal_disp = logitudinal_disp + 0.2)
	{
		virtual_laser_message->positions[virtual_laser_message->num_positions].x = object_list[index].x1 +
				lateral_disp * cos(object_list[index].theta + M_PI / 2.0) +
				logitudinal_disp * cos(object_list[index].theta);
		virtual_laser_message->positions[virtual_laser_message->num_positions].y = object_list[index].y1 +
				lateral_disp * sin(object_list[index].theta + M_PI / 2.0) +
				logitudinal_disp * sin(object_list[index].theta);
		virtual_laser_message->colors[virtual_laser_message->num_positions] = CARMEN_PURPLE;
		virtual_laser_message->num_positions++;

		virtual_laser_message->positions[virtual_laser_message->num_positions].x = object_list[index].x1 +
				lateral_disp * cos(object_list[index].theta - M_PI / 2.0) +
				logitudinal_disp * cos(object_list[index].theta);
		virtual_laser_message->positions[virtual_laser_message->num_positions].y = object_list[index].y1 +
				lateral_disp * sin(object_list[index].theta - M_PI / 2.0) +
				logitudinal_disp * sin(object_list[index].theta);
		virtual_laser_message->colors[virtual_laser_message->num_positions] = CARMEN_PURPLE;
		virtual_laser_message->num_positions++;
	}

	for (lateral_disp = -width / 2.0; lateral_disp < width / 2.0; lateral_disp = lateral_disp + 0.2)
	{
		virtual_laser_message->positions[virtual_laser_message->num_positions].x = object_list[index].x1 +
				lateral_disp * cos(object_list[index].theta + M_PI / 2.0) +
				logitudinal_disp * cos(object_list[index].theta);
		virtual_laser_message->positions[virtual_laser_message->num_positions].y = object_list[index].y1 +
				lateral_disp * sin(object_list[index].theta + M_PI / 2.0) +
				logitudinal_disp * sin(object_list[index].theta);
		virtual_laser_message->colors[virtual_laser_message->num_positions] = CARMEN_PURPLE;
		virtual_laser_message->num_positions++;

		virtual_laser_message->positions[virtual_laser_message->num_positions].x = object_list[index].x1 +
				lateral_disp * cos(object_list[index].theta - M_PI / 2.0) +
				logitudinal_disp * cos(object_list[index].theta);
		virtual_laser_message->positions[virtual_laser_message->num_positions].y = object_list[index].y1 +
				lateral_disp * sin(object_list[index].theta - M_PI / 2.0) +
				logitudinal_disp * sin(object_list[index].theta);
		virtual_laser_message->colors[virtual_laser_message->num_positions] = CARMEN_PURPLE;
		virtual_laser_message->num_positions++;
	}
}

void
add_objects_to_map()
{
	carmen_mapper_virtual_laser_message virtual_laser_message;
	virtual_laser_message.positions = (carmen_position_t *) malloc(num_objects * 1000 * sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) malloc(num_objects * 1000 * sizeof(char));
	virtual_laser_message.num_positions = 0;
	for (int index = 0; index < num_objects; index++)
	{
		double width = 1.7;
		double length = 4.1;
		switch (object_list[index].type)
		{
		case CARMEN_SIMULATOR_ACKERMAN_RANDOM_OBJECT:
		case CARMEN_SIMULATOR_ACKERMAN_LINE_FOLLOWER:
		case CARMEN_SIMULATOR_ACKERMAN_OTHER_ROBOT:
			break;

		case CARMEN_SIMULATOR_ACKERMAN_PERSON:
			width = 0.5;
			length = 0.5;
			break;

		case CARMEN_SIMULATOR_ACKERMAN_BIKE:
			width = 0.5;
			length = 1.5;
			break;

		case CARMEN_SIMULATOR_ACKERMAN_CAR:
			width = 1.7;
			length = 4.1;
			break;

		case CARMEN_SIMULATOR_ACKERMAN_TRUCK:
			width = 2.2;
			length = 15.0;
			break;
		}
		add_rectangle_to_map(&virtual_laser_message, width, length, index);
	}

	virtual_laser_message.host = carmen_get_host();
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());

	free(virtual_laser_message.positions);
	free(virtual_laser_message.colors);
}

/* updates all objects */
void carmen_simulator_ackerman_update_objects(carmen_simulator_ackerman_config_t *simulator_config)
{
	if (!road_network_message || !autonomous)
		return;

	for (int index = 0; index < num_objects; index++)
	{
		if (object_list[index].type == CARMEN_SIMULATOR_ACKERMAN_RANDOM_OBJECT)
			update_random_object(index, simulator_config);
		else if (object_list[index].type == CARMEN_SIMULATOR_ACKERMAN_LINE_FOLLOWER)
			update_line_follower(object_list + index, simulator_config);
		else if (object_list[index].type == CARMEN_SIMULATOR_ACKERMAN_OTHER_ROBOT)
			update_other_robot(object_list + index);
		else if (object_list[index].type == CARMEN_SIMULATOR_ACKERMAN_PERSON)
			update_object_in_lane(index, simulator_config);
		else if (object_list[index].type == CARMEN_SIMULATOR_ACKERMAN_BIKE)
			update_object_in_lane(index, simulator_config);
		else if (object_list[index].type == CARMEN_SIMULATOR_ACKERMAN_CAR)
			update_object_in_lane(index, simulator_config);
		else if (object_list[index].type == CARMEN_SIMULATOR_ACKERMAN_TRUCK)
			update_object_in_lane(index, simulator_config);
		update_traj_object(index);
	}

//	add_objects_to_map();
}

static void add_object_to_laser(double object_x, double object_y, double width, carmen_laser_laser_message *laser,
		carmen_simulator_ackerman_config_t *simulator_config, int is_rear)
{
	double phi, angle_diff;
	int index, i;
	double separation = M_PI / laser->num_readings;
	double lwidth;
	double dist;
	carmen_point_t robot;
	carmen_simulator_ackerman_laser_config_t *laser_config;

	robot = simulator_config->true_pose;

	if (is_rear)
	{
		laser_config = &(simulator_config->rear_laser_config);
	}
	else
	{
		laser_config = &(simulator_config->front_laser_config);
	}

	robot.x = simulator_config->true_pose.x + laser_config->offset * cos(simulator_config->true_pose.theta)
			- laser_config->side_offset * sin(simulator_config->true_pose.theta);

	robot.y = simulator_config->true_pose.y + laser_config->offset * sin(simulator_config->true_pose.theta)
			+ laser_config->side_offset * cos(simulator_config->true_pose.theta);

	robot.theta = carmen_normalize_theta(simulator_config->true_pose.theta + laser_config->angular_offset);

	phi = atan2(object_y - robot.y, object_x - robot.x);
	angle_diff = carmen_normalize_theta(phi - robot.theta);

	if (fabs(angle_diff) >= 0.5 * laser_config->fov)
		return;

	index = carmen_normalize_theta(angle_diff + M_PI / 2) / separation;
	dist = hypot(object_x - robot.x, object_y - robot.y);

	if (dist >= laser_config->max_range)
		return;

	lwidth = width / (double) (dist) / M_PI * laser->num_readings / 2;
	i = carmen_fmax(index - lwidth, 0);

	for (; i < index + lwidth && i < laser->num_readings; i++)
	{
		if (laser->range[i] > dist)
			laser->range[i] = dist;
	}
}

/* modifies the laser reading to acount for objects near the robot */
void carmen_simulator_ackerman_add_objects_to_laser(carmen_laser_laser_message *laser, carmen_simulator_ackerman_config_t *simulator_config, int is_rear)
{
	int index;
	for (index = 0; index < num_objects; index++)
	{
		add_object_to_laser(object_list[index].x1, object_list[index].y1, object_list[index].width, laser, simulator_config, is_rear);
		if (!object_list[index].is_robot)
			add_object_to_laser(object_list[index].x2, object_list[index].y2, object_list[index].width, laser, simulator_config, is_rear);
	}
}

/* frees all objects */
void carmen_simulator_objects_clear_objects(void)
{
	num_objects = 0;
}

/* gets the possitions of all the objects */
void carmen_simulator_ackerman_get_object_poses(int * num, carmen_traj_point_t **coordinates)
{
	*num = num_objects;
	*coordinates = traj_object_list;
}

void carmen_simulator_ackerman_get_objects(int *num, carmen_simulator_ackerman_objects_t **objects)
{
	static carmen_simulator_ackerman_objects_t *current_objects = NULL;

	current_objects = (carmen_simulator_ackerman_objects_t *) realloc(current_objects, num_objects * sizeof(carmen_simulator_ackerman_objects_t));
	for (int i = 0; i < num_objects; i++)
	{
		current_objects[i].type = object_list[i].type;
		current_objects[i].x = object_list[i].x1;
		current_objects[i].y = object_list[i].y1;
		current_objects[i].theta = object_list[i].theta;
		current_objects[i].v = object_list[i].tv;
	}
	*num = num_objects;
	*objects = current_objects;
}

void carmen_simulator_ackerman_initialize_object_model(int argc, char *argv[])
{
	carmen_param_t param_list[] =
	{
		{ "simulator", "person_leg_width", CARMEN_PARAM_DOUBLE, &leg_width, 1, NULL },
		{ "simulator", "person_dist_from_robot", CARMEN_PARAM_DOUBLE, &min_dist_from_robot, 1, NULL },
		{ "simulator", "person_speed", CARMEN_PARAM_DOUBLE, &person_speed, 1, NULL }
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	// Read moving objects
	FILE *moving_objects_file = fopen("moving_objects_file.txt", "r");
	if (moving_objects_file)
	{
		char line[2048];
		while (fgets(line, 2047, moving_objects_file))
		{
			if (line[0] == '#')
				continue;

			int type;
			double x, y, theta, speed;
			if (sscanf(line, "%d %lf %lf %lf %lf\n", &type, &x, &y, &theta, &speed) == 5)
				carmen_simulator_ackerman_create_object(x, y, theta, type, speed);
		}
		fclose(moving_objects_file);
	}
}

static carmen_object_ackerman_t *
find_object(IPC_CONTEXT_PTR context)
{
	int i;

	for (i = 0; i < num_objects; i++)
	{
		if (object_list[i].type == CARMEN_SIMULATOR_ACKERMAN_OTHER_ROBOT && object_list[i].context == context)
			return object_list + i;
	}
	return NULL;
}

static void globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	IPC_CONTEXT_PTR context;
	carmen_object_ackerman_t *object;

	context = IPC_getContext();
	object = find_object(context);
	if (object == NULL)
		return;
	if (carmen_get_time() - object->time_of_last_update > 10)
	{
		object->x1 = msg->globalpos.x;
		object->y1 = msg->globalpos.y;
		object->theta = msg->globalpos.theta;
		object->time_of_last_update = carmen_get_time();
	}
}

static void simulator_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err = IPC_OK;
	FORMATTER_PTR formatter;
	carmen_simulator_ackerman_truepos_message msg;
	IPC_CONTEXT_PTR context;
	carmen_object_ackerman_t *object;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &msg, sizeof(carmen_simulator_ackerman_truepos_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

	context = IPC_getContext();
	object = find_object(context);
	if (object == NULL)
		return;

	object->x1 = msg.truepose.x;
	object->y1 = msg.truepose.y;
	object->theta = msg.truepose.theta;
	object->time_of_last_update = carmen_get_time();
}

void carmen_simulator_ackerman_add_robot(char *program_name, char *robot_central)
{
	char buffer[1024];
	IPC_CONTEXT_PTR current_context;

	current_context = IPC_getContext();

	check_list_capacity();

	sprintf(buffer, "%s_%d", program_name, getpid());
	IPC_connectModule(program_name, robot_central);

	object_list[num_objects].context = IPC_getContext();

	if (IPC_isMsgDefined(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME))
	{
		IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, simulator_handler, NULL);
		IPC_setMsgQueueLength(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, 1);
	}
	else
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_param_set_module(NULL);
	carmen_param_get_double("robot_width", &(object_list[num_objects].width), NULL);

	object_list[num_objects].type = CARMEN_SIMULATOR_ACKERMAN_OTHER_ROBOT;
	object_list[num_objects].is_robot = 1;

	num_objects++;

	IPC_setContext(current_context);
}

static void update_other_robot(carmen_object_ackerman_t *object)
{
	IPC_CONTEXT_PTR current_context;

	current_context = IPC_getContext();
	IPC_setContext(object->context);

	IPC_listenClear(0);
	IPC_setContext(current_context);
}

int carmen_simulator_object_too_close(double x, double y, int skip)
{
	int i;

	for (i = 0; i < num_objects; i++)
	{
		if (i == skip)
			continue;
		if (hypot(x - object_list[i].x1, y - object_list[i].y1) < min_dist_from_robot)
		{
			return 1;
		}
		if (!object_list[i].is_robot)
		{
			if (hypot(x - object_list[i].x2, y - object_list[i].y2) < min_dist_from_robot)
			{
				return 1;
			}
		}
	}

	return 0;
}
