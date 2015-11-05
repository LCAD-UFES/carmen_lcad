#include <carmen/carmen.h>
#include "collision_detection.h"
#include "obstacle_avoider_messages.h"

#define SECURITY_VELOCITY_PERCENT 0.5



carmen_point_t
to_carmen_point_t (carmen_ackerman_traj_point_t *p)
{
	carmen_point_t point;

	point.x = p->x;
	point.y = p->y;
	point.theta = p->theta;

	return (point);
}


carmen_point_t 
to_map_pose(carmen_point_t world_pose, carmen_map_config_t *map_config) 
{
	carmen_point_t p;

	p.theta = world_pose.theta;
	p.x = (world_pose.x - map_config->x_origin) / map_config->resolution;
	p.y = (world_pose.y - map_config->y_origin) / map_config->resolution;

	return p;
}


carmen_point_t
to_world_pose(carmen_point_t map_pose, carmen_map_config_t *map_config)
{
	carmen_point_t wp;

	wp.theta = map_pose.theta;
	wp.x = (map_pose.x * map_config->resolution) + map_config->x_origin;
	wp.y = (map_pose.y * map_config->resolution) + map_config->y_origin;

	return wp;
}


int 
trajectory_pose_hit_obstacle(carmen_ackerman_traj_point_t trajectory_pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config) 
{
	carmen_point_t pose;

	if (map == NULL)
	{
		printf("map == NULL in trajectory_pose_hit_obstacle()\n");
		return (1);
	}

	pose.x = trajectory_pose.x;
	pose.y = trajectory_pose.y;
	pose.theta = trajectory_pose.theta;

	return (pose_hit_obstacle(pose, map, car_config));
}


int
colision_detection_is_valid_position(int x, int y, carmen_map_t *map)
{
	return ((x >= 0) && (x < map->config.x_size) && (y >= 0) && (y < map->config.y_size));
}


void
rotate_xy_around_robot_pose(int *rx, int *ry, double x, double y, carmen_point_t car_pose)
{
	double sin_theta = sin(car_pose.theta);
	double cos_theta = cos(car_pose.theta);

	*rx = round(x * cos_theta - y * sin_theta + car_pose.x);
	*ry = round(x * sin_theta + y * cos_theta + car_pose.y);
}


int
compute_robot_border_coordinates(double *robot_border_x_coordinates, double *robot_border_y_coordinates,
		double horizontal_size, double vertical_size, double distance_between_rear_car_and_rear_wheels)
{
	int i = 0;

	// a borda do carro eh computada com ele de lado (apontando para 0 graus)
	for (double j = 0.0; j < horizontal_size; j += 1.0)
	{
		// Parte lateral superior
		robot_border_x_coordinates[i] = j - distance_between_rear_car_and_rear_wheels;
		robot_border_y_coordinates[i] = vertical_size / 2.0;
		i++;
	}
	for (double j = 0.0; j < horizontal_size; j += 1.0)
	{
		// Parte lateral inferior
		robot_border_x_coordinates[i] = j - distance_between_rear_car_and_rear_wheels;
		robot_border_y_coordinates[i] = -vertical_size / 2.0;
		i++;
	}
	for (double j = 0.0; j < vertical_size; j += 1.0)
	{
		// Trazeira
		robot_border_x_coordinates[i] = -distance_between_rear_car_and_rear_wheels;
		robot_border_y_coordinates[i] = j - vertical_size / 2.0;
		i++;
	}
	for (double
			j = 0.0; j < vertical_size; j += 1.0)
	{
		// Dianteira
		robot_border_x_coordinates[i] = horizontal_size - distance_between_rear_car_and_rear_wheels;
		robot_border_y_coordinates[i] = j - vertical_size / 2.0;
		i++;
	}

	return (i);
}


double
carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border_old(const carmen_point_t *pose, carmen_map_t *map,
		double car_length, double car_width, double distance_between_rear_car_and_rear_wheels)
{
	static double *robot_border_x_coordinates = NULL;
	static double *robot_border_y_coordinates = NULL;
	static double vertical_size;
	static double horizontal_size;
	static int num_coordinates;
	double max_occupancy = 0.0;

	if (robot_border_x_coordinates == NULL)
	{
		horizontal_size = car_length / map->config.resolution;
		vertical_size = car_width / map->config.resolution;

		robot_border_x_coordinates = (double *) malloc((ceil(vertical_size) * 2.0 + ceil(horizontal_size) * 2.0) * sizeof(double));
		robot_border_y_coordinates = (double *) malloc((ceil(vertical_size) * 2.0 + ceil(horizontal_size) * 2.0) * sizeof(double));

		num_coordinates = compute_robot_border_coordinates(robot_border_x_coordinates, robot_border_y_coordinates,
				horizontal_size, vertical_size, distance_between_rear_car_and_rear_wheels / map->config.resolution);
	}

	carmen_point_t car_pose = to_map_pose(*pose, &map->config);
	for (int i = 0; i < num_coordinates; i++)
	{
		double x = robot_border_x_coordinates[i];
		double y = robot_border_y_coordinates[i];

		int rx, ry;
		rotate_xy_around_robot_pose(&rx, &ry, x, y, car_pose);
//		printf("x = %d, y = %d\n", rx, ry);
		if (!colision_detection_is_valid_position(rx, ry, map))
			return (1.0);

		double map_value = map->map[rx][ry];
		if (map_value > max_occupancy)
			max_occupancy = map_value;
	}
//	printf("nova\n");
	return (max_occupancy);
}


double
carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(const carmen_point_t *pose, carmen_map_t *map,
		double car_length, double car_width, double distance_between_rear_car_and_rear_wheels)
{
	double max_occupancy = 0.0;

	int vertical_size = ceil(car_length / map->config.resolution);
	int horizontal_size = ceil((car_width / 2.0) / map->config.resolution);

	double delta_vertical_x = cos(pose->theta);
	double delta_vertical_y = sin(pose->theta);
	double delta_horizontal_x = cos(M_PI / 2.0 - pose->theta);
	double delta_horizontal_y = sin(M_PI / 2.0 - pose->theta);
	double delta_horizontal_x_2 = delta_horizontal_x * horizontal_size;
	double delta_horizontal_y_2 = delta_horizontal_y * horizontal_size;

	carmen_point_t vertical_pose = to_map_pose(*pose, &map->config);
	vertical_pose.x -= distance_between_rear_car_and_rear_wheels / map->config.resolution * cos(vertical_pose.theta);
	vertical_pose.y -= distance_between_rear_car_and_rear_wheels / map->config.resolution * sin(vertical_pose.theta);
	for (int v = 0; v <= vertical_size; v++)
	{
		carmen_point_t horizontal_pose[2];
		horizontal_pose[0] = vertical_pose;
		horizontal_pose[1] = vertical_pose;
		if ((v == 0) || (v == vertical_size))
		{
			for (int h = 0; h <= horizontal_size; h++)
			{
				for (int i = 0; i < 2; i++)
				{
					if (!colision_detection_is_valid_position(round(horizontal_pose[i].x), round(horizontal_pose[i].y), map))
						return 1.0;

//					printf("x = %d, y = %d\n", (int) round(horizontal_pose[i].x), (int) round(horizontal_pose[i].y));
					double value = map->complete_map[(int) round(horizontal_pose[i].x) * map->config.y_size + (int) round(horizontal_pose[i].y)];
					if (value > max_occupancy)
						max_occupancy = value;

					if (h == 0)
						break;
				}
				horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
				horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;
				horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
				horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
			}
		}
		else
		{
			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x_2;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y_2;
			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x_2;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y_2;
			for (int i = 0; i < 2; i++)
			{
				if (!colision_detection_is_valid_position(round(horizontal_pose[i].x), round(horizontal_pose[i].y), map))
					return 1.0;

//				printf("x = %d, y = %d\n", (int) round(horizontal_pose[i].x), (int) round(horizontal_pose[i].y));
				double value = map->complete_map[(int) round(horizontal_pose[i].x) * map->config.y_size + (int) round(horizontal_pose[i].y)];
				if (value > max_occupancy)
					max_occupancy = value;
			}
		}
		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}
//	printf("nova\n");
	return (max_occupancy);
}


double
carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot(const carmen_point_t *pose, carmen_map_t *map,
		double car_length, double car_width, double distance_between_rear_car_and_rear_wheels)
{
	double max_occupancy = 0.0;

	int vertical_size = ceil(car_length / map->config.resolution);
	int horizontal_size = ceil((car_width / 2.0) / map->config.resolution);

	double delta_vertical_x = cos(pose->theta);
	double delta_vertical_y = sin(pose->theta);
	double delta_horizontal_x = cos(M_PI / 2.0 - pose->theta);
	double delta_horizontal_y = sin(M_PI / 2.0 - pose->theta);

	carmen_point_t vertical_pose = to_map_pose(*pose, &map->config);
	vertical_pose.x -= distance_between_rear_car_and_rear_wheels / map->config.resolution * cos(vertical_pose.theta);
	vertical_pose.y -= distance_between_rear_car_and_rear_wheels / map->config.resolution * sin(vertical_pose.theta);
	for (int v = 0; v <= vertical_size; v++)
	{
		carmen_point_t horizontal_pose[2];
		horizontal_pose[0] = vertical_pose;
		horizontal_pose[1] = vertical_pose;
		for (int h = 0; h <= horizontal_size; h++)
		{
			for (int i = 0; i < 2; i++)
			{
				if (!colision_detection_is_valid_position(round(horizontal_pose[i].x), round(horizontal_pose[i].y), map))
					return 1.0;

//				printf("x = %d, y = %d\n", (int) round(horizontal_pose[i].x), (int) round(horizontal_pose[i].y));
				double value = map->complete_map[(int) round(horizontal_pose[i].x) * map->config.y_size + (int) round(horizontal_pose[i].y)];
				if (value > max_occupancy)
					max_occupancy = value;

				if (h == 0)
					break;
			}
			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;
			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
		}
		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}
//	printf("velha\n");
	return (max_occupancy);
}


int
obstacle_avoider_pose_hit_obstacle(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config)
{
	if (carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(&pose, map,
			car_config->length, car_config->width, car_config->distance_between_rear_car_and_rear_wheels) > 0.5)
		return 1;
	else
		return 0;
}


int
pose_hit_obstacle(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config)
{
	if (carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(&pose, map,
			car_config->length, car_config->width, car_config->distance_between_rear_car_and_rear_wheels) > 0.5)
		return 1;
	else
		return 0;
}

/* do the same of pose_hit_obstacle but it considers the map's blue region as obstacle. */
int
pose_hit_obstacle_ultrasonic(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config)
{
	int vertical_size, horizontal_size;
	carmen_point_t vertical_pose, horizontal_pose[2];
	double delta_vertical_x, delta_vertical_y, delta_horizontal_x, delta_horizontal_y, value;

	vertical_size = ceil(car_config->length / map->config.resolution);
	horizontal_size = ceil((car_config->width / 2.0) / map->config.resolution);

	delta_vertical_x = cos(pose.theta);
	delta_vertical_y = sin(pose.theta);

	delta_horizontal_x = cos(M_PI/2 - pose.theta);
	delta_horizontal_y = sin(M_PI/2 - pose.theta);

	vertical_pose = to_map_pose(pose, &map->config);

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

				if (!colision_detection_is_valid_position(horizontal_pose[i].x, horizontal_pose[i].y, map))
					return 1;

				value = map->complete_map[(int)horizontal_pose[i].x * map->config.y_size + (int)horizontal_pose[i].y];

				if (value > 0.5 || value < 0.0)
					return 1;

			}

			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;

			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
		}

		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}

	return 0;
}


carmen_ackerman_traj_point_t
predict_new_robot_position(carmen_ackerman_traj_point_t current_robot_position, double v, double phi, double time, carmen_robot_ackerman_config_t *car_config)
{
	carmen_ackerman_traj_point_t new_robot_position;
	int i;
	double delta_time;

	new_robot_position = current_robot_position;
	delta_time = time / 1.0;
	for (i = 0; i < 1; i++)
	{
		new_robot_position.x = new_robot_position.x + v * cos(new_robot_position.theta) * delta_time;
		new_robot_position.y = new_robot_position.y + v * sin(new_robot_position.theta) * delta_time;
		new_robot_position.theta = new_robot_position.theta + (v * tan(phi) / car_config->distance_between_front_and_rear_axles) * delta_time;
	}

	new_robot_position.v = v;
	new_robot_position.phi = phi;
	
	return (new_robot_position);
}
