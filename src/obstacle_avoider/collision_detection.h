#ifndef COLISION_DETECTION_H
#define COLISION_DETECTION_H


#include <carmen/carmen.h>
#include <carmen/ipc_wrapper.h>

#ifdef __cplusplus
extern "C" {
#endif

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
int trajectory_pose_hit_obstacle(carmen_ackerman_traj_point_t trajectory_pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config);

double
road_velocity_percentual(carmen_point_t pose, carmen_map_t *map, carmen_robot_ackerman_config_t *car_config);

carmen_ackerman_traj_point_t predict_new_robot_position(carmen_ackerman_traj_point_t current_robot_position, double v, double phi, double time, carmen_robot_ackerman_config_t *car_config);

#ifdef __cplusplus
}
#endif

#endif
