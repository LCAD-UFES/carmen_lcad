
#ifndef COLISION_DETECTION_H
#define COLISION_DETECTION_H

#include <carmen/carmen.h>
#include <carmen/ipc_wrapper.h>
#include <carmen/obstacle_distance_mapper_messages.h>
#include <carmen/moving_objects_messages.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_GEOMETRY	0
#define ENGAGE_GEOMETRY 	1


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
	int 	height_level;
} carmen_collision_marker_t;

typedef struct
{
	int geometry;

	int n_markers;
	carmen_collision_marker_t *markers;

	int semi_trailer_type;
	double semi_trailer_d[MAX_NUM_TRAILERS];
	double semi_trailer_M[MAX_NUM_TRAILERS];
	double semi_trailer_max_beta[MAX_NUM_TRAILERS];

	int n_semi_trailer_markers[MAX_NUM_TRAILERS];
	carmen_collision_marker_t *semi_trailer_markers[MAX_NUM_TRAILERS];
} carmen_collision_config_t;


carmen_point_t to_carmen_point_t (carmen_robot_and_trailers_traj_point_t *p);
carmen_point_t to_map_pose(carmen_point_t world_pose, carmen_map_config_t *map_config);

int colision_detection_is_valid_position(int x, int y, carmen_map_t *map);
double carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot(const carmen_point_t *pose, carmen_map_t *map,
		double car_length, double car_width, double distance_between_rear_car_and_rear_wheels);
double carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(const carmen_point_t *pose, carmen_map_t *map,
		double car_length, double car_width, double distance_between_rear_car_and_rear_wheels);

int pose_hit_obstacle_ultrasonic(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config);

int trajectory_pose_hit_obstacle(carmen_robot_and_trailers_traj_point_t trajectory_pose, double safety_distance,
		carmen_obstacle_distance_mapper_map_message *distance_map, carmen_robot_ackerman_config_t *robot_config);

double
carmen_obstacle_avoider_car_distance_to_nearest_obstacle(carmen_robot_and_trailers_traj_point_t trajectory_pose,
carmen_obstacle_distance_mapper_map_message *distance_map);

double
road_velocity_percentual(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config);

carmen_point_t
carmen_collision_detection_move_path_point_to_world_coordinates(const carmen_point_t point, carmen_point_t *localizer_pose, double displacement);

double
carmen_obstacle_avoider_proximity_to_obstacles(carmen_robot_and_trailers_pose_t *localizer_pose,
		carmen_robot_and_trailers_pose_t local_point_to_check, carmen_obstacle_distance_mapper_map_message *distance_map, double safety_distance);

double
carmen_obstacle_avoider_distance_from_global_point_to_obstacle(carmen_position_t *global_point, carmen_obstacle_distance_mapper_map_message *distance_map);

carmen_robot_and_trailers_pose_t
carmen_collision_detection_displace_car_pose_according_to_car_orientation(carmen_robot_and_trailers_traj_point_t *car_pose, double displacement);

carmen_position_t
carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(carmen_point_t *global_point, carmen_obstacle_distance_mapper_map_message *distance_map);

double
carmen_obstacle_avoider_compute_closest_car_distance_to_colliding_point(carmen_robot_and_trailers_traj_point_t *car_pose, carmen_position_t point_to_check,
		carmen_robot_ackerman_config_t robot_config, double circle_radius);

carmen_position_t
carmen_collision_detection_displaced_pose_according_to_car_orientation(carmen_robot_and_trailers_traj_point_t *car_pose, double x, double y);

carmen_collision_config_t*
carmen_collision_detection_get_global_collision_config();

void
carmen_collision_detection_set_robot_collision_config(int collision_geometry);

int
carmen_obstacle_avoider_car_collides_with_moving_object(carmen_robot_and_trailers_pose_t car_pose, carmen_point_t moving_object_pose,
		t_point_cloud_struct *moving_object, double longitudinal_safety_magin, double lateral_safety_margin);//,
//		int obj_id, double obj_s, double obj_d);

carmen_position_t
carmen_collision_detection_in_car_coordinate_frame(const carmen_robot_and_trailers_pose_t point,
		carmen_robot_and_trailers_pose_t *localizer_pose, double x, double y);

carmen_robot_and_trailers_pose_t
carmen_collision_detection_displace_car_on_its_frenet_frame(carmen_robot_and_trailers_traj_point_t *car_pose, double s, double d);

void
carmen_collision_detection_set_semi_trailer_type(int semi_trailer_type);

#ifdef __cplusplus
}
#endif

#endif
