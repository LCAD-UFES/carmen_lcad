#ifndef UDATMO_H
#define UDATMO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "udatmo_messages.h"

#include <carmen/obstacle_distance_mapper_messages.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/rddf_messages.h>

void carmen_udatmo_init(carmen_robot_ackerman_config_t *robot_config, int min_poses_ahead, int max_poses_ahead);

void carmen_udatmo_setup(int argc, char *argv[]);

carmen_udatmo_moving_obstacles_message *carmen_udatmo_detect_moving_obstacles(void);

carmen_udatmo_moving_obstacles_message *carmen_udatmo_get_moving_obstacles(void);

int carmen_udatmo_front_obstacle_detected();

double carmen_udatmo_front_obstacle_distance(carmen_ackerman_traj_point_t *robot_pose);

carmen_ackerman_traj_point_t carmen_udatmo_front_obstacle_position(void);

double carmen_udatmo_front_obstacle_speed(carmen_ackerman_traj_point_t *robot_pose);

void carmen_udatmo_update_distance_map(carmen_obstacle_distance_mapper_message *message);

void carmen_udatmo_update_robot_pose_with_globalpos(carmen_localize_ackerman_globalpos_message *message);

void carmen_udatmo_update_robot_pose_with_truepos(carmen_simulator_ackerman_truepos_message *message);

void carmen_udatmo_update_rddf(carmen_rddf_road_profile_message *message);


#ifdef __cplusplus
}
#endif

#endif
