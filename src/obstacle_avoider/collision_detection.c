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
int global_max_height_level;

//carmen_mapper_virtual_laser_message virtual_laser_message;

/**
 * @brief Computes unit vector representing local y-axis of the obb
 *
 * @details To better understand what unit local axis mean refer to Huynh, Johnny. "Separating Axis Theorem
 *          for Oriented Bounding Boxes.", figure at page 13, where Ay is the local y-axis of obb A and
 *          By is local y-axis of obb B.
 *
 * @param obb Oriented bounding box
 * @return 2D vector normalized representing the local y-axis of the box
 */
carmen_vector_2D_t
get_unit_local_axis_y(const carmen_oriented_bounding_box obb)
{
	carmen_vector_2D_t axis;

//	axis.x = cos(obb.orientation);
//	axis.y = sin(obb.orientation);
	sincos(obb.orientation,&axis.y, &axis.x);

	return axis;
}

/**
 * @brief Computes unit vector representing local x-axis of the obb
 *
 * @details To better understand what unit local axis mean refer to Huynh, Johnny. "Separating Axis Theorem
 *          for Oriented Bounding Boxes.", figure at page 13, where Ax is the local x-axis of obb A and
 *          By is local x-axis of obb B. Don't use this function if you already have the unit vector at the axis Y,
 *          just use the fact that the y-axis and x-axis are perpendicular (Ax.x = -Ay.x; Ax.y = Ay.x)
 *
 * @param obb Oriented bounding box
 *
 * @return 2D vector normalized representing the local x-axis of the box
 */
carmen_vector_2D_t
get_unit_local_axis_x(const carmen_oriented_bounding_box obb)
{
	carmen_vector_2D_t axis;
	axis = get_unit_local_axis_y(obb);
	double temp = axis.x;
	axis.x = -axis.y;
	axis.y = temp;
	return axis;
}

/**
 * @brief Project vector T onto some arbitrary axis
 *
 * @param T Vector to be projected
 * @param Axis Vector representing the axis
 *
 * @return size of the projection of T at the axis
 */
double
project_T_at_axis(const carmen_vector_2D_t T, const carmen_vector_2D_t Axis)
{
	return fabs(DOT2D(T,Axis));
}


/**
 * @brief Project specific dimensions of an obb at an arbritary axis
 *
 * @details This function implements the operations c + |(d1*V2) dot V1| + |(d2*V3) dot V1|, where dot represent the
 *          scalar product between the two vectors. This is used during the test cases to the collision between two obb
 *          using the Separation axis theorem. For more information, check Huynh, Johnny. "Separating Axis Theorem
 *          for Oriented Bounding Boxes.", in the section "Separating Axis Theorem and Rectangles in 2D Space"
 *
 * @param V1 Axis of the first obb where the projection will occur
 * @param V2 unit x-axis of the second obb
 * @param V3 unit y-axis of the second obb
 * @param d1 half width of the second obb
 * @param d2 half length of the second obb
 * @param c half width/length of the first obb (depends on the test)
 * @return size of the projection of the dimensions of the two obb at the axis V1
 */
double
project_dimensions_at_axis(const carmen_vector_2D_t V1, carmen_vector_2D_t V2, carmen_vector_2D_t V3, double d1, double d2, double c)
{
	V2.x = d1 * V2.x;
	V2.y = d1 * V2.y;
	V3.x = d2 * V3.x;
	V3.y = d2 * V3.y;
	return c + fabs(DOT2D(V2,V1)) + fabs(DOT2D(V3, V1));
}

/**
 * @brief Compute collision between two obb's using the Separation axis theorem
 *
 * @details This function implements the scheme for obb collision as described in Huynh, Johnny. "Separating Axis Theorem
 *          for Oriented Bounding Boxes."
 *
 * @param obb1 first oriented bouding box
 * @param obb2 second oriented bouding box
 * @return 0.0 if no collision occurs between the two boxes, penetration distance of obb2 into obb1 otherwise.
 */
double
compute_collision_obb_obb(const carmen_oriented_bounding_box obb1, const carmen_oriented_bounding_box obb2)
{
	carmen_vector_2D_t T; // T is a vector extends from center of obb1 to center of obb2
	T.x = obb2.object_pose.x - obb1.object_pose.x;
	T.y = obb2.object_pose.y - obb1.object_pose.y;

    /* Calculating unit local axis from the two obb's. Here A refers to obb1 and B refers to obb2 */
	carmen_vector_2D_t Ay = get_unit_local_axis_y(obb1);
	carmen_vector_2D_t Ax;
	Ax.x = Ay.y; Ax.y = -Ay.x; // Ax is perpendicular to Ay

	carmen_vector_2D_t By = get_unit_local_axis_y(obb2);
	carmen_vector_2D_t Bx;
	Bx.x = By.y; Bx.y = -By.x; // Bx is perpendicular to By

    /* When computing the first two tests, the results are stored so we can later use it to calculate
     * the penetration distance of obb2 into obb1. */

    //Projection at Ay
    double d1 = project_dimensions_at_axis(Ay, Bx, By, obb2.width*0.5, obb2.length*0.5, obb1.length*0.5);
    double d2 = project_T_at_axis(T, Ay);
    if(d2 > d1)
        return 0;

    //Projection at Ax
    double d3 = project_dimensions_at_axis(Ax, Bx, By, obb2.width*0.5, obb2.length*0.5, obb1.width*0.5);
    double d4 = project_T_at_axis(T, Ax);
    if(d4 > d3)
        return 0;

    //Projection at By
    if(project_T_at_axis(T, By) > project_dimensions_at_axis(By, Ax, Ay, obb1.width*0.5, obb1.length*0.5, obb2.width*0.5))
        return 0;

    //Projection at Bx
    if(project_T_at_axis(T, Bx) > project_dimensions_at_axis(Bx, Ax, Ay, obb1.width*0.5, obb1.length*0.5, obb2.width*0.5))
        return 0;

    // If the code reaches here the two boxes have an interception.
	double penetration_distance = carmen_square(d1 - d2) + carmen_square(d3 - d4);

	return penetration_distance;

}

/**
 * @brief      Determines if it has collision between lines.
 *
 * @param[in]  line1  First line in point-angle form
 * @param[in]  line2  Second line in point-angle form
 * 
 * @return     Different from zero if has collision between lines, zero otherwise.
 */
int
has_collision_between_lines(carmen_point_t line1, carmen_point_t line2)
{
	// utilize as linhas em seu formato ponto-angulo, isto eh, contendo um ponto por onde a linha 
    // atravesse e seu respectivo angulo.
    // Por exemplo, para uma linha da forma y = a * x + b, escolhemos um valor de x, 
    // como x = 1.0, e encontramos y = a * (1.0) + b = a + b. Além disso, sabemos que a inclinação
    // dessa linha eh dado pelo coeficiente 'a'. Assim, teremos a representação:
    // carmen_point_t line;
    // line.x = 1.0;
    // line.y = a + b;
    // line.theta = a;

	// Tratando linhas como caixas com largura zero e comprimento infinito, o codigo de colisao
	// implementado eh bem robusto em relacao a casos degenerados como esse.
	carmen_oriented_bounding_box obb1;
	obb1.object_pose.x = line1.x;
	obb1.object_pose.y = line1.y;
	obb1.length = DBL_MAX;
	obb1.width = 0.0;
	obb1.orientation = line1.theta;
	obb1.linear_velocity = 0.0;

	carmen_oriented_bounding_box obb2;
	obb2.object_pose.x = line2.x;
	obb2.object_pose.y = line2.y;
	obb2.length = DBL_MAX;
	obb2.width = 0.0;
	obb2.orientation = line2.theta;
	obb2.linear_velocity = 0.0;

	return (int) (compute_collision_obb_obb(obb1, obb2) + 0.5);
}

/**
 * @brief Get index number of set bit, a number 
 * 
 * @param x
 * @return int32_t log_2(v), i.e. n such that 2 ∧ n = v
 */
uint32_t
GetBitIndex(uint32_t x)
{
  double input = (double)x;
  double answer = log2(input);
  uint32_t converted = (uint32_t)floorf(answer);

  return converted;
}

int
clamp(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  return value;
}

/**
 * @brief Insert object defined by OBB pObject into grid.
 * 
 * @param pObject OBB that define the object
 * @param object_index relative number of the object into the grid. (N+1 if the grid already has N objects)
 * @param grid 
 */
void
InsertObjectIntoGrid(carmen_oriented_bounding_box pObject,
                     int object_index,
                     carmen_uniform_collision_grid* grid)
{
  carmen_test_alloc(grid); // sanity check

  // Compute the extent of grid cells the bounding sphere of A overlaps.
  // Test assumes objects have been inserted in all rows/columns overlapped
  float ooCellWidth = 1.0f / grid->cell_width;
  double radius = pObject.length + pObject.width;
  
  int x1 = (int)floorf((pObject.object_pose.x - radius) * ooCellWidth + grid->grid_width/2);
  int x2 = (int)floorf((pObject.object_pose.x + radius) * ooCellWidth + grid->grid_width/2);
  int y1 = (int)floorf((pObject.object_pose.y - radius) * ooCellWidth + grid->grid_height/2);
  int y2 = (int)floorf((pObject.object_pose.y + radius) * ooCellWidth + grid->grid_height/2);

  x1 = clamp(x1, 0, grid->grid_width-1);
  x2 = clamp(x2, 0, grid->grid_width-1);
  y1 = clamp(y1, 0, grid->grid_height-1);
  y2 = clamp(y2, 0, grid->grid_height-1);

  int i = object_index / 32;
  // Compute the merged (bitwise-or) bit array of all overlapped grid rows.
  // Ditto for all overlapped grid columns
  for (int y = y1; y <= y2; y++)
    grid->rowBitArray[y][i] |= (1 << object_index);
  for (int x = x1; x <= x2; x++)
    grid->columnBitArray[x][i] |= (1 << object_index);
}

/**
 * @brief Construct an uniform collision grid and insert an list of objects into it.
 * 
 * @param num_objects 
 * @param objects
 * @param grid_width 
 * @param grid_height 
 * @param cell_width 
 * @return carmen_uniform_collision_grid 
 */
carmen_uniform_collision_grid
construct_uniform_collision_grid(int num_objects, carmen_oriented_bounding_box *objects, int grid_width, int grid_height, double cell_width)
{
	carmen_uniform_collision_grid grid;

	grid.num_objects = num_objects;
	grid.cell_width = cell_width;
	grid.grid_width = grid_width;
	grid.grid_height = grid_height;

	grid.num_objects_div_32 = (num_objects + 31) / 32;

	grid.rowBitArray = (int32_t **)malloc(grid_height * sizeof(int32_t *));
	for (int i = 0; i < grid_height; i++)
		grid.rowBitArray[i] = (int32_t *)calloc(grid.num_objects_div_32, sizeof(int32_t));

	grid.columnBitArray = (int32_t **)malloc(grid_width * sizeof(int32_t *));
	for (int i = 0; i < grid_width; i++)
		grid.columnBitArray[i] = (int32_t *)calloc(grid.num_objects_div_32, sizeof(int32_t));

  grid.objects = (carmen_oriented_bounding_box *)malloc(num_objects * sizeof(carmen_oriented_bounding_box));
  for (int i = 0; i < num_objects; i++) 
  {
		grid.objects[i] = objects[i];
		InsertObjectIntoGrid(objects[i], i, &grid);
	}

	return grid;
}

/**
 * @brief Check the object pObject against the grid and return the sum of the penetration distance between pObject and all of the objects inside the grid. Implementation based on (Real Time Collision Detection - 2004)
 *
 * @param pObject 
 * @param grid 
 * @return double 
 */
double
TestObjectAgainstGrid(carmen_oriented_bounding_box pObject, carmen_uniform_collision_grid *grid)
{

	// Allocate temporary bit arrays for all objects and clear them
	int32_t *mergedRowArray = (int32_t *)calloc(grid->num_objects_div_32, sizeof(int32_t));
	int32_t *mergedColumnArray = (int32_t *)calloc(grid->num_objects_div_32, sizeof(int32_t));

	// Compute the extent of grid cells the bounding sphere of A overlaps.
	// Test assumes objects have been inserted in all rows/columns overlapped
	float ooCellWidth = 1.0f / grid->cell_width;
	double radius = pObject.length + pObject.width;
	
	int x1 = (int)floorf((pObject.object_pose.x - radius) * ooCellWidth + grid->grid_width/2);
	int x2 = (int)floorf((pObject.object_pose.x + radius) * ooCellWidth + grid->grid_width/2);
	int y1 = (int)floorf((pObject.object_pose.y - radius) * ooCellWidth + grid->grid_height/2);
	int y2 = (int)floorf((pObject.object_pose.y + radius) * ooCellWidth + grid->grid_height/2);

	x1 = clamp(x1, 0, grid->grid_width-1);
	x2 = clamp(x2, 0, grid->grid_width-1);
	y1 = clamp(y1, 0, grid->grid_height-1);
	y2 = clamp(y2, 0, grid->grid_height-1);

	// Compute the merged (bitwise-or’ed) bit array of all overlapped grid rows.
	// Ditto for all overlapped grid columns
	for (int y = y1; y <= y2; y++)
		for (int i = 0; i < grid->num_objects_div_32; i++)
			mergedRowArray[i] |= grid->rowBitArray[y][i];
	for (int x = x1; x <= x2; x++)
		for (int i = 0; i < grid->num_objects_div_32; i++)
			mergedColumnArray[i] |= grid->columnBitArray[x][i];

	// Now go through the intersection of the merged bit arrays and collision test
	// those objects having their corresponding bit set
	double p_distance = 0;
	for (int i = 0; i < grid->num_objects_div_32; i++)
	{
		int32_t objectsMask = mergedRowArray[i] & mergedColumnArray[i];
		while (objectsMask)
		{
			// Clears all but lowest bit set (eg. 01101010 -> 00000010)
			int32_t less = objectsMask - 1;
			int32_t singleObjectMask = (less | objectsMask) ^ less;

			// Get index number of set bit, test against corresponding object
			// (GetBitIndex(v) returns log_2(v), i.e. n such that 2 ∧ n = v)
      		uint32_t objectIndex = GetBitIndex(singleObjectMask) + i * 32;

			p_distance += compute_collision_obb_obb(pObject, grid->objects[objectIndex]);
			// Mask out tested object, and continue with any remaining objects
			objectsMask ^= singleObjectMask;
		}
	}

	return p_distance;
}

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
			car_config->length, car_config->width, car_config->distance_between_rear_car_and_rear_wheels) > 0.5) //0.3
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
	path_point_in_map_coords.x = (localizer_pose->x + x_disp * coss - y_disp * sine);
	path_point_in_map_coords.y = (localizer_pose->y + x_disp * sine + y_disp * coss);

	return (path_point_in_map_coords);
}

carmen_point_t
carmen_collision_detection_in_car_coordinate_frame(const carmen_point_t point, carmen_point_t *localizer_pose, double x, double y)
{
	carmen_point_t path_point_in_map_coords;
	double coss, sine;

	sincos(point.theta, &sine, &coss);
	double x_disp = point.x + x * coss + y * sine;
	double y_disp = point.y + x * sine + y * coss;

	sincos(localizer_pose->theta, &sine, &coss);
	path_point_in_map_coords.x = (localizer_pose->x + x_disp * coss - y_disp * sine);
	path_point_in_map_coords.y = (localizer_pose->y + x_disp * sine + y_disp * coss);

	return (path_point_in_map_coords);
}

carmen_point_t
carmen_collision_detection_displace_car_pose_according_to_car_orientation(carmen_ackerman_traj_point_t *car_pose, double displace)
{
	carmen_point_t displaced_car_pose;
	double coss, sine;

	sincos(car_pose->theta, &sine, &coss);
	displaced_car_pose.x = car_pose->x + displace * coss;
	displaced_car_pose.y = car_pose->y + displace * sine;

	displaced_car_pose.theta = car_pose->theta;

	return (displaced_car_pose);
}

carmen_point_t
carmen_collision_detection_pose_according_to_car_orientation(carmen_ackerman_traj_point_t *car_pose, double x, double y)
{
	carmen_point_t displaced_car_pose;
	double coss, sine;

	sincos(car_pose->theta, &sine, &coss);
	displaced_car_pose.x = car_pose->x + x * coss + y * sine;
	displaced_car_pose.y = car_pose->y + x * sine + y * coss;

	displaced_car_pose.theta = car_pose->theta;

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
	// Os mapas de carmen sao orientados a colunas, logo a equacao eh como abaixo
	int index = y_map_cell + distance_map->config.y_size * x_map_cell;
	if (index < 0 || index >= distance_map->size)
		return (cell);

	cell.x = (double) distance_map->complete_x_offset[index] + (double) global_point_in_map_coords.x;
	cell.y = (double) distance_map->complete_y_offset[index] + (double) global_point_in_map_coords.y;

	// convert to global coordinates system
	cell.x = distance_map->config.x_origin + cell.x * distance_map->config.resolution;
	cell.y = distance_map->config.y_origin + cell.y * distance_map->config.resolution;

	return (cell);
}


double
carmen_obstacle_avoider_distance_from_global_point_to_obstacle(carmen_point_t *global_point, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	carmen_point_t global_point_in_map_coords;

	// Move global path point coordinates to map coordinates
	global_point_in_map_coords.x = (global_point->x - distance_map->config.x_origin) / distance_map->config.resolution;
	global_point_in_map_coords.y = (global_point->y - distance_map->config.y_origin) / distance_map->config.resolution;

	// Transform coordinates to integer indexes
	int x_map_cell = (int) round(global_point_in_map_coords.x);
	int y_map_cell = (int) round(global_point_in_map_coords.y);

	// Os mapas de carmen sao orientados a colunas, logo a equacao eh como abaixo
	int index = y_map_cell + distance_map->config.y_size * x_map_cell;
	if (index < 0 || index >= distance_map->size)
		return (-1.0);

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


/*A funcao abaixo foi substituida pela funcao acima para atender casos mais geras*/
//double
//distance_from_traj_point_to_obstacle(carmen_point_t point,  carmen_point_t *localizer_pose,
//		carmen_obstacle_distance_mapper_message *distance_map, double displacement, double min_dist)
//{
//	// Move path point to map coordinates
//	carmen_ackerman_path_point_t path_point_in_map_coords =	move_path_point_to_map_coordinates(point, localizer_pose, distance_map, displacement);
//	int x_map_cell = (int) round(path_point_in_map_coords.x);
//	int y_map_cell = (int) round(path_point_in_map_coords.y);
//
//	// Os mapas de carmen sao orientados a colunas, logo a equacao eh como abaixo
//	int index = y_map_cell + distance_map->config.y_size * x_map_cell;
//	if (index < 0 || index >= distance_map->size)
//		return (min_dist);
//
//	double dx = (double) distance_map->complete_x_offset[index] + (double) x_map_cell - path_point_in_map_coords.x;
//	double dy = (double) distance_map->complete_y_offset[index] + (double) y_map_cell - path_point_in_map_coords.y;
//
//	double distance_in_map_coordinates = sqrt(dx * dx + dy * dy);
//	double distance = distance_in_map_coordinates * distance_map->config.resolution;
//
//	return (distance);
//}

void
check_collision_config_initialization(){

	static int collision_config_initialized = 0;
	char* collision_file;
	int i;
	FILE *collision_file_pointer;

	if (collision_config_initialized)
		return;

	carmen_param_allow_unfound_variables(0);
	carmen_param_t param_list[] =
	{
		{"robot", "collision_file", CARMEN_PARAM_STRING, &collision_file, 1, NULL},
	};
	carmen_param_install_params(0, NULL, param_list, sizeof(param_list)/sizeof(param_list[0]));

	collision_file_pointer = fopen(collision_file, "r");
	setlocale(LC_NUMERIC, "C");
	fscanf(collision_file_pointer,"%d", &(global_collision_config.n_markers));
	global_collision_config.markers = (carmen_collision_marker_t*) malloc(global_collision_config.n_markers*sizeof(carmen_collision_marker_t));

	for (i=0; i<global_collision_config.n_markers; i++)
		fscanf(collision_file_pointer,"%lf %lf %lf %lf", &global_collision_config.markers[i].x , &global_collision_config.markers[i].y,
				&global_collision_config.markers[i].radius, &global_collision_config.markers[i].hight);

	fclose(collision_file_pointer);
	collision_config_initialized = 1;
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

//This Function expected the points to check in local coordinates, if you need use global coordinates, just set localizer_pose as 0.0
double
carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles(carmen_point_t *localizer_pose, carmen_point_t local_point_to_check, // point_to_check_in_respect_to_car,
carmen_robot_ackerman_config_t robot_config, carmen_obstacle_distance_mapper_map_message *distance_map, double safety_distance)
{
	check_collision_config_initialization();

	double proximity_to_obstacles = 0.0;

	for (int i = 0; i < global_collision_config.n_markers; i++)
	{
		carmen_point_t displaced_point = carmen_collision_detection_in_car_coordinate_frame(local_point_to_check, localizer_pose,
				global_collision_config.markers[i].x, global_collision_config.markers[i].y);
		double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
		//distance equals to -1.0 when the coordinates are outside of map
		if (distance != -1.0)
		{
			double delta = distance - (global_collision_config.markers[i].radius + safety_distance);
			if (delta < 0.0)
				proximity_to_obstacles += delta * delta;
		}
	}
	return (proximity_to_obstacles);
}


double
carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles_old(carmen_point_t *localizer_pose, carmen_point_t local_point_to_check, // point_to_check_in_respect_to_car,
		carmen_robot_ackerman_config_t robot_config, carmen_obstacle_distance_mapper_map_message *distance_map, double circle_radius)
{
	double initial_displacement, displacement_inc;
	double number_of_point = 5.0;
	get_initial_displacement_and_displacement_inc(&initial_displacement, &displacement_inc, circle_radius, number_of_point, &robot_config);

	double proximity_to_obstacles = 0.0;
	for (double i = 0; i < number_of_point; i += 1.0)
	{
		double displacement = initial_displacement + i * displacement_inc;
		carmen_point_t displaced_point = carmen_collision_detection_displace_point_in_car_coordinate_frame(local_point_to_check, localizer_pose, displacement);
		double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
		//distance equals to -1.0 when the coordinates are outside of map
		if (distance != -1.0)
		{
			double delta = distance - circle_radius;
			if (delta < 0.0)
				proximity_to_obstacles += delta * delta;
		}
	}

	return (proximity_to_obstacles);
}


int
trajectory_pose_hit_obstacle(carmen_ackerman_traj_point_t trajectory_pose, double safety_distance,
carmen_obstacle_distance_mapper_map_message *distance_map, carmen_robot_ackerman_config_t *robot_config)
{
	check_collision_config_initialization();

	if (distance_map == NULL)
	{
		printf("distance_map == NULL in trajectory_pose_hit_obstacle()\n");
		return (1);
	}

	for (int i = 0; i < global_collision_config.n_markers; i++)
	{
		carmen_point_t displaced_point = carmen_collision_detection_pose_according_to_car_orientation(&trajectory_pose,
				global_collision_config.markers[i].x, global_collision_config.markers[i].y);
		double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
		//distance equals to -1.0 when the coordinates are outside of map
		if (distance != -1.0)
		{
			if (distance < global_collision_config.markers[i].radius + safety_distance)
				return (1);
		}
		else
			return (2);
	}
	return (0);
}

int
trajectory_pose_hit_obstacle_old(carmen_ackerman_traj_point_t trajectory_pose, double circle_radius,
		carmen_obstacle_distance_mapper_map_message *distance_map, carmen_robot_ackerman_config_t *robot_config)
{
	if (distance_map == NULL)
	{
		printf("distance_map == NULL in trajectory_pose_hit_obstacle()\n");
		return (1);
	}

	double initial_displacement, displacement_inc;
	double number_of_point = 5.0;
	get_initial_displacement_and_displacement_inc(&initial_displacement, &displacement_inc, circle_radius, number_of_point, robot_config);

	for (double i = 0; i < number_of_point; i += 1.0)
	{
		double displacement = initial_displacement + i * displacement_inc;
		carmen_point_t displaced_point = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&trajectory_pose, displacement);
		double distance = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&displaced_point, distance_map);
		//distance equals to -1.0 when the coordinates are outside of map
		if (distance != -1.0)
		{
			if (distance < circle_radius)
				return (1);
		}
		else
			return (2);
	}

	return (0);
}

double
carmen_obstacle_avoider_compute_closest_car_distance_to_colliding_point(carmen_ackerman_traj_point_t *car_pose, carmen_position_t point_to_check,
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

		carmen_point_t displaced_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(car_pose, displacement);
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


carmen_collision_config_t*
carmen_get_global_collision_config(){
	check_collision_config_initialization();
	return &global_collision_config;
}
