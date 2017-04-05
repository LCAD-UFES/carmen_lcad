#ifndef UDATMO_API_H
#define UDATMO_API_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "udatmo_messages.h"

#include <carmen/obstacle_distance_mapper_messages.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/rddf_messages.h>


void carmen_udatmo_init(const carmen_robot_ackerman_config_t *robot_config);


void carmen_udatmo_setup(int argc, char *argv[]);


carmen_udatmo_moving_obstacles_message *carmen_udatmo_detect_moving_obstacles(void);


carmen_udatmo_moving_obstacle *carmen_udatmo_find_front_moving_obstacle(const carmen_udatmo_moving_obstacles_message *message);


carmen_udatmo_moving_obstacle *carmen_udatmo_detect_front_moving_obstacle(void);


carmen_udatmo_moving_obstacles_message *carmen_udatmo_get_moving_obstacles(void);


int carmen_udatmo_front_obstacle_detected(void);


double carmen_udatmo_front_obstacle_speed(carmen_ackerman_traj_point_t *robot_pose);


double carmen_udatmo_front_obstacle_distance(carmen_ackerman_traj_point_t *robot_pose);


carmen_ackerman_traj_point_t carmen_udatmo_front_obstacle_position(void);


void carmen_udatmo_update_robot_pose(const carmen_ackerman_traj_point_t *robot_pose);


void carmen_udatmo_update_robot_pose_with_globalpos(carmen_localize_ackerman_globalpos_message *message);


void carmen_udatmo_update_robot_pose_with_truepos(carmen_simulator_ackerman_truepos_message *message);


void carmen_udatmo_update_distance_map(carmen_obstacle_distance_mapper_message *message);


void carmen_udatmo_update_offline_map(carmen_mapper_map_message *grid);


void carmen_udatmo_update_rddf(carmen_rddf_road_profile_message *message);


#ifdef __cplusplus
}
#endif

#endif
