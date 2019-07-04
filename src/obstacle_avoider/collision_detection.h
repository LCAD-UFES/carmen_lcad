
#ifndef COLISION_DETECTION_H
#define COLISION_DETECTION_H

#include <carmen/carmen.h>
#include <carmen/ipc_wrapper.h>
#include <carmen/obstacle_distance_mapper_messages.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _carmen_oriented_bounding_box
{
	carmen_vector_2D_t object_pose; /*< Center of the bounding box*/
	double length;
	double width;
	double orientation;
	double linear_velocity;
} carmen_oriented_bounding_box;

typedef struct _carmen_uniform_collision_grid
{
	int num_objects;
	double cell_width;
	int grid_width;
	int grid_height;
	int num_objects_div_32;
	int32_t **rowBitArray;
	int32_t **columnBitArray;
	carmen_oriented_bounding_box *objects;
} carmen_uniform_collision_grid;

typedef struct
{
	double 	x;
	double 	y;
	double 	radius;
	double 	hight;
} carmen_collision_marker_t;

typedef struct
{
	int n_markers;
	carmen_collision_marker_t* markers;
} carmen_collision_config_t;

double compute_collision_obb_obb(const carmen_oriented_bounding_box carmen_oriented_bounding_box1,
								 const carmen_oriented_bounding_box carmen_oriented_bounding_box2);

int has_collision_between_lines(carmen_point_t line1, carmen_point_t line2);

uint32_t GetBitIndex(uint32_t x);

void InsertObjectIntoGrid(carmen_oriented_bounding_box pObject, int object_index, carmen_uniform_collision_grid* grid);

carmen_uniform_collision_grid construct_uniform_collision_grid( int num_objects, carmen_oriented_bounding_box* objects,
  																int grid_width,int grid_height,double cell_width);

double TestObjectAgainstGrid(carmen_oriented_bounding_box pObject, carmen_uniform_collision_grid* grid);

  void set_variable_map_config(double map_config);

carmen_point_t to_carmen_point_t (carmen_ackerman_traj_point_t *p);
carmen_point_t to_map_pose(carmen_point_t world_pose, carmen_map_config_t *map_config);

int colision_detection_is_valid_position(int x, int y, carmen_map_t *map);
double carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot(const carmen_point_t *pose, carmen_map_t *map,
		double car_length, double car_width, double distance_between_rear_car_and_rear_wheels);
double carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(const carmen_point_t *pose, carmen_map_t *map,
		double car_length, double car_width, double distance_between_rear_car_and_rear_wheels);

int pose_hit_obstacle(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config);

int obstacle_avoider_pose_hit_obstacle(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config);

int pose_hit_obstacle_ultrasonic(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config);

int trajectory_pose_hit_obstacle(carmen_ackerman_traj_point_t trajectory_pose, double safety_distance,
		carmen_obstacle_distance_mapper_map_message *distance_map, carmen_robot_ackerman_config_t *robot_config);

double
road_velocity_percentual(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config);

carmen_point_t
carmen_collision_detection_move_path_point_to_world_coordinates(const carmen_point_t point, carmen_point_t *localizer_pose, double displacement);

double
carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles(carmen_point_t *localizer_pose, carmen_point_t local_point_to_check,
		carmen_robot_ackerman_config_t robot_config, carmen_obstacle_distance_mapper_map_message *distance_map, double safety_distance);

double
carmen_obstacle_avoider_distance_from_global_point_to_obstacle(carmen_point_t *global_point, carmen_obstacle_distance_mapper_map_message *distance_map);

carmen_point_t
carmen_collision_detection_displace_car_pose_according_to_car_orientation(carmen_ackerman_traj_point_t *car_pose, double displacement);

carmen_position_t
carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(carmen_point_t *global_point, carmen_obstacle_distance_mapper_map_message *distance_map);

double
carmen_obstacle_avoider_compute_closest_car_distance_to_colliding_point(carmen_ackerman_traj_point_t *car_pose, carmen_position_t point_to_check,
		carmen_robot_ackerman_config_t robot_config, double circle_radius);

carmen_collision_config_t*
carmen_get_global_collision_config();

#ifdef __cplusplus
}
#endif

#endif
