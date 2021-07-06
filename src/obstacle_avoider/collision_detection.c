#define _GNU_SOURCE
#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/mapper_messages.h>
#include "collision_detection.h"
#include "obstacle_avoider_messages.h"
#include <math.h>
#include <float.h>
#include <locale.h>

#define SECURITY_VELOCITY_PERCENT 0.5

carmen_collision_config_t global_collision_config;

//carmen_mapper_virtual_laser_message virtual_laser_message;

carmen_point_t
to_carmen_point_t (carmen_robot_and_trailer_traj_point_t *p)
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


//inline carmen_point_t
carmen_point_t
carmen_collision_detection_displace_point_in_car_coordinate_frame(const carmen_point_t point, carmen_point_t *localizer_pose, double displacement)
{
	carmen_point_t path_point_in_map_coords;
	double coss, sine;

	//calcula o displacement em relação a orientação atual do carro
	sincos(point.theta, &sine, &coss);
	double x_disp = point.x + displacement * coss;
	double y_disp = point.y + displacement * sine;

	//coloca o displacement no sistema de coordenadas do mundo
	sincos(localizer_pose->theta, &sine, &coss);
	path_point_in_map_coords.x = localizer_pose->x + x_disp * coss - y_disp * sine;
	path_point_in_map_coords.y = localizer_pose->y + x_disp * sine + y_disp * coss;

	return (path_point_in_map_coords);
}

carmen_position_t
carmen_collision_detection_in_car_coordinate_frame(const carmen_robot_and_trailer_pose_t point,
		carmen_robot_and_trailer_pose_t *localizer_pose, double x, double y)
{
	carmen_position_t path_point_in_map_coords;
	double coss, sine;

	sincos(point.theta, &sine, &coss);
	double x_disp = point.x + x * coss - y * sine;
	double y_disp = point.y + x * sine + y * coss;

	sincos(localizer_pose->theta, &sine, &coss);
	path_point_in_map_coords.x = (localizer_pose->x + x_disp * coss - y_disp * sine);
	path_point_in_map_coords.y = (localizer_pose->y + x_disp * sine + y_disp * coss);

	return (path_point_in_map_coords);
}

carmen_robot_and_trailer_pose_t
carmen_collision_detection_displace_car_pose_according_to_car_orientation(carmen_robot_and_trailer_traj_point_t *car_pose, double displace)
{
	carmen_robot_and_trailer_pose_t displaced_car_pose;
	double coss, sine;

	sincos(car_pose->theta, &sine, &coss);
	displaced_car_pose.x = car_pose->x + displace * coss;
	displaced_car_pose.y = car_pose->y + displace * sine;

	displaced_car_pose.theta = car_pose->theta;
	displaced_car_pose.beta = car_pose->beta;

	return (displaced_car_pose);
}

carmen_position_t
carmen_collision_detection_displaced_pose_according_to_car_orientation(carmen_robot_and_trailer_traj_point_t *car_pose, double x, double y)
{
	carmen_position_t displaced_car_pose;
	double coss, sine;

	sincos(car_pose->theta, &sine, &coss);
	displaced_car_pose.x = car_pose->x + x * coss - y * sine;
	displaced_car_pose.y = car_pose->y + x * sine + y * coss;

	return (displaced_car_pose);
}

carmen_robot_and_trailer_pose_t
carmen_collision_detection_displace_car_on_its_frenet_frame(carmen_robot_and_trailer_traj_point_t *car_pose, double s, double d)
{
	carmen_robot_and_trailer_pose_t displaced_car_pose;
	double coss, sine;

	double displacement = s;
	sincos(car_pose->theta, &sine, &coss);
	displaced_car_pose.x = car_pose->x + displacement * coss;
	displaced_car_pose.y = car_pose->y + displacement * sine;

	displacement = d;
	sincos(carmen_normalize_theta(car_pose->theta + M_PI / 2.0), &sine, &coss);
	displaced_car_pose.x = displaced_car_pose.x + displacement * coss;
	displaced_car_pose.y = displaced_car_pose.y + displacement * sine;

	displaced_car_pose.theta = car_pose->theta;
	displaced_car_pose.beta = car_pose->beta;

	return (displaced_car_pose);
}

carmen_position_t
carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(carmen_point_t *global_point, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	carmen_point_t global_point_in_map_coords;
	// Move global path point coordinates to map coordinates
	global_point_in_map_coords.x = (global_point->x - distance_map->config.x_origin) / distance_map->config.resolution;
	global_point_in_map_coords.y = (global_point->y - distance_map->config.y_origin) / distance_map->config.resolution;

	// Transform coordinates to integer indexes
	int x_map_cell = (int) round(global_point_in_map_coords.x);
	int y_map_cell = (int) round(global_point_in_map_coords.y);

	carmen_position_t cell;
	cell.x = -1.0;
	cell.y = -1.0;
	if ((x_map_cell < 0 || x_map_cell >= distance_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= distance_map->config.y_size))
		return (cell);

	// Os mapas de carmen sao orientados a colunas, logo a equacao eh como abaixo
	int index = y_map_cell + distance_map->config.y_size * x_map_cell;

	cell.x = (double) distance_map->complete_x_offset[index] + (double) global_point_in_map_coords.x;
	cell.y = (double) distance_map->complete_y_offset[index] + (double) global_point_in_map_coords.y;

	// convert to global coordinates system
	cell.x = distance_map->config.x_origin + cell.x * distance_map->config.resolution;
	cell.y = distance_map->config.y_origin + cell.y * distance_map->config.resolution;

	return (cell);
}


double
carmen_obstacle_avoider_distance_from_global_point_to_obstacle(carmen_position_t *global_point, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	carmen_point_t global_point_in_map_coords;

	// Move global path point coordinates to map coordinates
	global_point_in_map_coords.x = (global_point->x - distance_map->config.x_origin) / distance_map->config.resolution;
	global_point_in_map_coords.y = (global_point->y - distance_map->config.y_origin) / distance_map->config.resolution;

	// Transform coordinates to integer indexes
	int x_map_cell = (int) round(global_point_in_map_coords.x);
	int y_map_cell = (int) round(global_point_in_map_coords.y);
	if ((x_map_cell < 0 || x_map_cell >= distance_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= distance_map->config.y_size))
		return (-1000.0);

	// Os mapas de carmen sao orientados a colunas, logo a equacao eh como abaixo
	int index = y_map_cell + distance_map->config.y_size * x_map_cell;

	double dx = ((double) distance_map->complete_x_offset[index] + (double) x_map_cell) - global_point_in_map_coords.x;
	double dy = ((double) distance_map->complete_y_offset[index] + (double) y_map_cell) - global_point_in_map_coords.y;

//	virtual_laser_message.positions[virtual_laser_message.num_positions].x = global_point->x + dx * distance_map->config.resolution;
//	virtual_laser_message.positions[virtual_laser_message.num_positions].y = global_point->y + dy * distance_map->config.resolution;
//	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_RED;
//	virtual_laser_message.num_positions++;
//	virtual_laser_message.positions[virtual_laser_message.num_positions].x = global_point->x;
//	virtual_laser_message.positions[virtual_laser_message.num_positions].y = global_point->y;
//	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_YELLOW;
//	virtual_laser_message.num_positions++;

	double distance_in_map_coordinates = sqrt(dx * dx + dy * dy);
	double distance = distance_in_map_coordinates * distance_map->config.resolution;

	return (distance);
}


void
semi_trailer_collision_config_initialization(int semi_trailer_type)
{
	global_collision_config.semi_trailer_type = semi_trailer_type;

	char semi_trailer_string[2048];
	sprintf(semi_trailer_string, "%s%d", "semi_trailer", semi_trailer_type);

	char *semi_trailer_collision_file;
	carmen_param_t param_list[] =
	{
		{semi_trailer_string, "d", CARMEN_PARAM_DOUBLE, &(global_collision_config.semi_trailer_d), 0, NULL},
		{semi_trailer_string, "M", CARMEN_PARAM_DOUBLE, &(global_collision_config.semi_trailer_M), 0, NULL},
		{semi_trailer_string, "max_beta", CARMEN_PARAM_DOUBLE, &(global_collision_config.semi_trailer_max_beta), 0, NULL},
		{semi_trailer_string, "collision_file", CARMEN_PARAM_STRING, &semi_trailer_collision_file, 0, NULL},
	};
	carmen_param_install_params(0, NULL, param_list, sizeof(param_list) / sizeof(param_list[0]));
	global_collision_config.semi_trailer_max_beta = carmen_degrees_to_radians(global_collision_config.semi_trailer_max_beta);

	char *carmen_home = getenv("CARMEN_HOME");
	char collision_file_[2048];
	strcpy(collision_file_, carmen_home);
	strcat(collision_file_, "/bin/");
	strcat(collision_file_, semi_trailer_collision_file);

	FILE *collision_file_pointer = fopen(collision_file_, "r");
	setlocale(LC_NUMERIC, "C");
	fscanf(collision_file_pointer, "%d", &(global_collision_config.n_semi_trailer_markers));
	int max_h_level;
	fscanf(collision_file_pointer, "%d", &max_h_level);	// Para compatibilidade multi height
	global_collision_config.semi_trailer_markers = (carmen_collision_marker_t *) malloc(global_collision_config.n_semi_trailer_markers * sizeof(carmen_collision_marker_t));

	for (int i = 0; i < global_collision_config.n_semi_trailer_markers; i++)
		fscanf(collision_file_pointer,"%lf %lf %lf %d", &(global_collision_config.semi_trailer_markers[i].x) , &(global_collision_config.semi_trailer_markers[i].y),
				&(global_collision_config.semi_trailer_markers[i].radius), &(global_collision_config.semi_trailer_markers[i].height_level));

	fclose(collision_file_pointer);
}


void
check_collision_config_initialization()
{
	static int collision_config_initialized = 0;

	if (collision_config_initialized)
		return;
	collision_config_initialized = 1;

	char *collision_file;
	carmen_param_t param_list[] =
	{
		{"robot", "collision_file", 		CARMEN_PARAM_STRING, 	&collision_file, 								1, NULL},
		{"semi",  "trailer_initial_type", 	CARMEN_PARAM_INT, 		&(global_collision_config.semi_trailer_type), 	1, NULL},
	};
	carmen_param_install_params(0, NULL, param_list, sizeof(param_list) / sizeof(param_list[0]));

	char *carmen_home = getenv("CARMEN_HOME");

	if (carmen_home == NULL)
		exit(printf("Could not get environment variable $CARMEN_HOME in check_collision_config_initialization()\n"));

	char collision_file_[2048];
	sprintf(collision_file_, "%s/bin/%s", carmen_home, collision_file);

	FILE *collision_file_pointer = fopen(collision_file_, "r");
	setlocale(LC_NUMERIC, "C");
	fscanf(collision_file_pointer, "%d", &(global_collision_config.n_markers));
	int max_h_level;
	fscanf(collision_file_pointer, "%d", &max_h_level);	// Para compatibilidade multi height
	global_collision_config.markers = (carmen_collision_marker_t *) malloc(global_collision_config.n_markers * sizeof(carmen_collision_marker_t));

	for (int i = 0; i < global_collision_config.n_markers; i++)
		fscanf(collision_file_pointer,"%lf %lf %lf %d", &global_collision_config.markers[i].x , &global_collision_config.markers[i].y,
				&global_collision_config.markers[i].radius, &global_collision_config.markers[i].height_level);

	fclose(collision_file_pointer);

	if (global_collision_config.semi_trailer_type != 0)
		semi_trailer_collision_config_initialization(global_collision_config.semi_trailer_type);
}


void
get_initial_displacement_and_displacement_inc(double *initial_displacement, double *displacement_inc, double circle_radius, double number_of_point,
		carmen_robot_ackerman_config_t *robot_config)
{
//	0.8 ee um espacco de guarda aa frente e atras
	double car_lenght = robot_config->distance_between_front_and_rear_axles + robot_config->distance_between_rear_car_and_rear_wheels +
			robot_config->distance_between_front_car_and_front_wheels + 2.0 * (0.8 - circle_radius) + 0.8;
	*displacement_inc = car_lenght / number_of_point;
	*initial_displacement = circle_radius - (robot_config->distance_between_rear_car_and_rear_wheels + 0.8);
}


carmen_position_t
move_semi_trailer_marker_to_robot_coordinate_frame(double x, double y, double beta)
{
	carmen_position_t displaced_marker;

	beta = -beta;
	displaced_marker.x = (x - global_collision_config.semi_trailer_d) * cos(beta) - y * sin(beta) - global_collision_config.semi_trailer_M;
	displaced_marker.y = (x - global_collision_config.semi_trailer_d) * sin(beta) + y * cos(beta);

	return displaced_marker;
}


//This Function expected the points to check in local coordinates, if you need use global coordinates, just set localizer_pose as 0.0
double
carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles(carmen_robot_and_trailer_pose_t *localizer_pose,
		carmen_robot_and_trailer_pose_t local_point_to_check, carmen_obstacle_distance_mapper_map_message *distance_map, double safety_distance)
{
	check_collision_config_initialization();

	double proximity_to_obstacles = 0.0;

	for (int i = 0; i < global_collision_config.n_markers; i++)
	{
		// Pega local_point_to_check e coloca no sistema de coordenadas definido por localizer_pose
		carmen_position_t displaced_point = carmen_collision_detection_in_car_coordinate_frame(local_point_to_check, localizer_pose,
				global_collision_config.markers[i].x, global_collision_config.markers[i].y);
		double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
		// distance equals to -1000.0 when the coordinates are outside of map
		if (distance != -1000.0)
		{
			double delta = distance - (global_collision_config.markers[i].radius + safety_distance);
			if (delta < 0.0)
				proximity_to_obstacles += delta * delta;
		}
	}

	if (global_collision_config.semi_trailer_type > 0)
	{
		if (fabs(local_point_to_check.beta) > global_collision_config.semi_trailer_max_beta)
			return (pow(2.0, global_collision_config.semi_trailer_markers[0].radius + safety_distance));
		else
		{
			for (int i = 0; i < global_collision_config.n_semi_trailer_markers; i++)
			{
				carmen_position_t displaced_marker = move_semi_trailer_marker_to_robot_coordinate_frame(
						global_collision_config.semi_trailer_markers[i].x, global_collision_config.semi_trailer_markers[i].y, local_point_to_check.beta);

				// Pega local_point_to_check e coloca no sistema de coordenadas definido por localizer_pose
				carmen_position_t displaced_point = carmen_collision_detection_in_car_coordinate_frame(local_point_to_check, localizer_pose,
						displaced_marker.x, displaced_marker.y);
				double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
				// distance equals to -1000.0 when the coordinates are outside of map
				if (distance != -1000.0)
				{
					double delta = distance - (global_collision_config.semi_trailer_markers[i].radius + safety_distance);
					if (delta < 0.0)
						proximity_to_obstacles += delta * delta;
				}
			}
		}
	}

	return (proximity_to_obstacles);
}

carmen_mapper_virtual_laser_message virtual_laser_message;
int add_virtual_laser_points = 0;

void
add_circle(carmen_point_t center, double radius, int color)
{
	double coss, sine;

	virtual_laser_message.positions[virtual_laser_message.num_positions].x = center.x;
	virtual_laser_message.positions[virtual_laser_message.num_positions].y = center.y;
	virtual_laser_message.colors[virtual_laser_message.num_positions] = color;
	virtual_laser_message.num_positions++;

	for (double theta = 0.0; theta < 2.0 * M_PI; theta += (2.0 * M_PI) / 12.0)
	{
		sincos(theta, &sine, &coss);
		double x = center.x + radius * coss - radius * sine;
		double y = center.y + radius * sine + radius * coss;

		virtual_laser_message.positions[virtual_laser_message.num_positions].x = x;
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = y;
		virtual_laser_message.colors[virtual_laser_message.num_positions] = color;
		virtual_laser_message.num_positions++;
	}
}


void
add_line(carmen_point_t point1, carmen_point_t point2, int color)
{
	double theta = atan2(point2.y - point1.y, point2.x - point1.x);
	double coss, sine;
	sincos(theta, &sine, &coss);

	for (double disp = 0.0; disp <= DIST2D(point1, point2); disp += DIST2D(point1, point2) / 10.0)
	{
		double x = point1.x + disp * coss;
		double y = point1.y + disp * sine;

		virtual_laser_message.positions[virtual_laser_message.num_positions].x = x;
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = y;
		virtual_laser_message.colors[virtual_laser_message.num_positions] = color;
		virtual_laser_message.num_positions++;
	}
}


int
compute_mo_points_old(carmen_position_t *mo_points, double width, double length, double x, double y, double theta)
{
	double points_displacement = 0.4;

	double logitudinal_disp = -length / 2.0;
	double lateral_disp = -width / 2.0;
	int mo_points_size = 0;
	double sin, cos;
	sincos(theta, &sin, &cos);
	double sin_plus, cos_plus;
	sincos(theta + M_PI / 2.0, &sin_plus, &cos_plus);
	double sin_minus, cos_minus;
	sincos(theta - M_PI / 2.0, &sin_minus, &cos_minus);
	for (; lateral_disp < width / 2.0; lateral_disp = lateral_disp + points_displacement)
	{
		mo_points[mo_points_size].x = x + lateral_disp * cos_plus + logitudinal_disp * cos;
		mo_points[mo_points_size].y = y + lateral_disp * sin_plus + logitudinal_disp * sin;
		mo_points_size++;

		mo_points[mo_points_size].x = x + lateral_disp * cos_minus + logitudinal_disp * cos;
		mo_points[mo_points_size].y = y + lateral_disp * sin_minus + logitudinal_disp * sin;
		mo_points_size++;
	}

	lateral_disp = width / 2.0;
	for (; logitudinal_disp < length / 2.0; logitudinal_disp = logitudinal_disp + points_displacement)
	{
		mo_points[mo_points_size].x = x + lateral_disp * cos_plus + logitudinal_disp * cos;
		mo_points[mo_points_size].y = y + lateral_disp * sin_plus + logitudinal_disp * sin;
		mo_points_size++;

		mo_points[mo_points_size].x = x + lateral_disp * cos_minus + logitudinal_disp * cos;
		mo_points[mo_points_size].y = y + lateral_disp * sin_minus + logitudinal_disp * sin;
		mo_points_size++;
	}

	for (lateral_disp = -width / 2.0; lateral_disp < width / 2.0; lateral_disp = lateral_disp + points_displacement)
	{
		mo_points[mo_points_size].x = x + lateral_disp * cos_plus + logitudinal_disp * cos;
		mo_points[mo_points_size].y = y + lateral_disp * sin_plus + logitudinal_disp * sin;
		mo_points_size++;

		mo_points[mo_points_size].x = x + lateral_disp * cos_minus + logitudinal_disp * cos;
		mo_points[mo_points_size].y = y + lateral_disp * sin_minus + logitudinal_disp * sin;
		mo_points_size++;
	}

	return (mo_points_size);
}


int
compute_mo_points(carmen_position_t *mo_points, double width, double length, double x, double y, double theta)
{
	double points_displacement = width / 2.0;
	int mo_points_size = 0;
	double sin, cos;
	sincos(theta, &sin, &cos);

	for (double logitudinal_disp = -length / 2.0; logitudinal_disp < length / 2.0; logitudinal_disp = logitudinal_disp + points_displacement)
	{
		mo_points[mo_points_size].x = x + logitudinal_disp * cos;
		mo_points[mo_points_size].y = y + logitudinal_disp * sin;
		mo_points_size++;
	}

	return (mo_points_size);
}


int
carmen_obstacle_avoider_car_collides_with_moving_object(carmen_robot_and_trailer_pose_t car_pose, carmen_point_t moving_object_pose,
		t_point_cloud_struct *moving_object, double longitudinal_safety_magin, double lateral_safety_margin)
{
	check_collision_config_initialization();

	carmen_robot_and_trailer_traj_point_t cp = {car_pose.x, car_pose.y, car_pose.theta, car_pose.beta, 0.0, 0.0};
	carmen_position_t mo_points[1000];
	int mo_points_size = compute_mo_points(mo_points, moving_object->width, moving_object->length, moving_object_pose.x, moving_object_pose.y, moving_object_pose.theta);
//	printf("id %d, mo_points_size %d\n", moving_object->num_associated, mo_points_size);
	double mo_radius_plus_safety_margin = moving_object->width / 2.0 + lateral_safety_margin;
	carmen_collision_marker_t *markers = global_collision_config.markers;
	carmen_collision_marker_t *semi_trailer_markers = global_collision_config.semi_trailer_markers;
	int n_markers = global_collision_config.n_markers;
	for (double displacement = -longitudinal_safety_magin; displacement <= longitudinal_safety_magin; displacement += 0.5)
	{
		carmen_robot_and_trailer_pose_t ldcp = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&cp, displacement);
		carmen_robot_and_trailer_traj_point_t ldcp2 = {ldcp.x, ldcp.y, ldcp.theta, car_pose.beta, 0.0, 0.0};
		for (int i = 0; i < n_markers; i++)
		{
			double radius = markers[i].radius + mo_radius_plus_safety_margin;
			double radius_sq = radius * radius;
			carmen_position_t displaced_marker = carmen_collision_detection_displaced_pose_according_to_car_orientation(&ldcp2,
					markers[i].x, markers[i].y);
			double x = displaced_marker.x;
			double y = displaced_marker.y;
			for (int j = 0; j < mo_points_size; j++)
			{
				double mo_p_x = mo_points[j].x;
				double mo_p_y = mo_points[j].y;
				double distance_sq = (x - mo_p_x) * (x - mo_p_x) + (y - mo_p_y) * (y - mo_p_y);

				if (distance_sq <= radius_sq)
					return (1);
			}
		}

		if (global_collision_config.semi_trailer_type > 0)
		{
			for (int i = 0; i < global_collision_config.n_semi_trailer_markers; i++)
			{
				carmen_position_t displaced_marker = move_semi_trailer_marker_to_robot_coordinate_frame(
						semi_trailer_markers[i].x, semi_trailer_markers[i].y, ldcp2.beta);

				double radius = semi_trailer_markers[i].radius + mo_radius_plus_safety_margin;
				double radius_sq = radius * radius;
				carmen_position_t displaced_marker_ = carmen_collision_detection_displaced_pose_according_to_car_orientation(&ldcp2,
						displaced_marker.x, displaced_marker.y);
				double x = displaced_marker_.x;
				double y = displaced_marker_.y;
				for (int j = 0; j < mo_points_size; j++)
				{
					double mo_p_x = mo_points[j].x;
					double mo_p_y = mo_points[j].y;
					double distance_sq = (x - mo_p_x) * (x - mo_p_x) + (y - mo_p_y) * (y - mo_p_y);

					if (distance_sq <= radius_sq)
						return (1);
				}
			}
		}
	}

	return (0);
}


double
carmen_obstacle_avoider_car_distance_to_nearest_obstacle(carmen_robot_and_trailer_traj_point_t trajectory_pose,
carmen_obstacle_distance_mapper_map_message *distance_map)
{
	check_collision_config_initialization();

	if (distance_map == NULL)
	{
		printf("distance_map == NULL in carmen_obstacle_avoider_car_distance_to_nearest_obstacle()\n");
		return (0.0);
	}

	double min_distance = 1000000000.0;
	for (int i = 0; i < global_collision_config.n_markers; i++)
	{
		carmen_position_t displaced_point = carmen_collision_detection_displaced_pose_according_to_car_orientation(&trajectory_pose,
				global_collision_config.markers[i].x, global_collision_config.markers[i].y);
		double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
		//distance equals to -1000.0 when the coordinates are outside of map
		if (distance != -1000.0)
		{	// A fucao retorna valor negativo se o carro encobrir um obstaculo.
			distance = distance - global_collision_config.markers[i].radius;
			if (distance < min_distance)
				min_distance = distance;
		}
	}

	if (global_collision_config.semi_trailer_type > 0)
	{
		if (fabs(trajectory_pose.beta) > global_collision_config.semi_trailer_max_beta)
			return (0.0);
		else
		{
			for (int i = 0; i < global_collision_config.n_semi_trailer_markers; i++)
			{
				carmen_position_t displaced_marker = move_semi_trailer_marker_to_robot_coordinate_frame(
						global_collision_config.semi_trailer_markers[i].x, global_collision_config.semi_trailer_markers[i].y, trajectory_pose.beta);

				carmen_position_t displaced_point = carmen_collision_detection_displaced_pose_according_to_car_orientation(&trajectory_pose,
						displaced_marker.x, displaced_marker.y);
				double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
				//distance equals to -1000.0 when the coordinates are outside of map
				if (distance != -1000.0)
				{	// A fucao retorna valor negativo se o carro encobrir um obstaculo.
					distance = distance - global_collision_config.semi_trailer_markers[i].radius;
					if (distance < min_distance)
						min_distance = distance;
				}
			}
		}
	}

	return (min_distance);
}


int
trajectory_pose_hit_obstacle(carmen_robot_and_trailer_traj_point_t trajectory_pose, double safety_distance,
carmen_obstacle_distance_mapper_map_message *distance_map, carmen_robot_ackerman_config_t *robot_config __attribute__ ((unused)))
{
	check_collision_config_initialization();

	if (distance_map == NULL)
	{
		printf("distance_map == NULL in trajectory_pose_hit_obstacle()\n");
		return (1);
	}

	for (int i = 0; i < global_collision_config.n_markers; i++)
	{
		carmen_position_t displaced_point = carmen_collision_detection_displaced_pose_according_to_car_orientation(&trajectory_pose,
				global_collision_config.markers[i].x, global_collision_config.markers[i].y);
		double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
		//distance equals to -1000.0 when the coordinates are outside of map
		if (distance != -1000.0)
		{
			if (distance < global_collision_config.markers[i].radius + safety_distance)
			{
//				virtual_laser_message.positions[virtual_laser_message.num_positions].x = displaced_point.x;
//				virtual_laser_message.positions[virtual_laser_message.num_positions].y = displaced_point.y;
//				virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_BLUE;
//				virtual_laser_message.num_positions++;
				return (1);
			}
		}
		else
			return (2);
	}

	if (global_collision_config.semi_trailer_type > 0)
	{
		if (fabs(trajectory_pose.beta) > global_collision_config.semi_trailer_max_beta)
			return (1);
		else
		{
			for (int i = 0; i < global_collision_config.n_semi_trailer_markers; i++)
			{
				carmen_position_t displaced_marker = move_semi_trailer_marker_to_robot_coordinate_frame(
						global_collision_config.semi_trailer_markers[i].x, global_collision_config.semi_trailer_markers[i].y, trajectory_pose.beta);

				carmen_position_t displaced_point = carmen_collision_detection_displaced_pose_according_to_car_orientation(&trajectory_pose,
						displaced_marker.x, displaced_marker.y);
				double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
				//distance equals to -1000.0 when the coordinates are outside of map
				if (distance != -1000.0)
				{
					if (distance < global_collision_config.semi_trailer_markers[i].radius + safety_distance)
					{
//						virtual_laser_message.positions[virtual_laser_message.num_positions].x = displaced_point.x;
//						virtual_laser_message.positions[virtual_laser_message.num_positions].y = displaced_point.y;
//						virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_BLUE;
//						virtual_laser_message.num_positions++;
						return (1);
					}
				}
				else
					return (2);
			}
		}
	}

	return (0);
}


double
carmen_obstacle_avoider_compute_closest_car_distance_to_colliding_point(carmen_robot_and_trailer_traj_point_t *car_pose, carmen_position_t point_to_check,
		carmen_robot_ackerman_config_t robot_config, double circle_radius)
{
	int number_of_point = 4;
	double displacement_inc = robot_config.distance_between_front_and_rear_axles / (number_of_point - 2);
	double displacement = 0.0;
	double proximity_to_colliding_point = circle_radius;

	for (int i = -1; i < number_of_point; i++)
	{
		displacement = displacement_inc * i;

		if (i < 0)
			displacement = -robot_config.distance_between_rear_car_and_rear_wheels;

		if (i == number_of_point - 1)
			displacement = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;

		carmen_robot_and_trailer_pose_t displaced_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(car_pose, displacement);
		double distance = sqrt((displaced_car_pose.x - point_to_check.x) * (displaced_car_pose.x - point_to_check.x) +
							   (displaced_car_pose.y - point_to_check.y) * (displaced_car_pose.y - point_to_check.y));
		double delta = distance - circle_radius;
		if (delta < 0.0)
		{
			if (-delta < proximity_to_colliding_point)
				proximity_to_colliding_point = -delta;
		}
	}

	return (proximity_to_colliding_point);
}


carmen_collision_config_t *
carmen_get_global_collision_config()
{
	check_collision_config_initialization();

	return (&global_collision_config);
}
