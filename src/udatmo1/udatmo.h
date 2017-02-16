#ifndef UDATMO_H
#define UDATMO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/rddf_messages.h>

void udatmo_init(const carmen_robot_ackerman_config_t robot_config);

int udatmo_obstacle_detected(void);

void udatmo_clear_detected(void);

void udatmo_shift_history(void);

int udatmo_detect_obstacle_index(carmen_obstacle_distance_mapper_message *current_map,
							carmen_rddf_road_profile_message *rddf,
							int goal_index,
							int rddf_pose_index,
							carmen_ackerman_traj_point_t robot_pose,
							double timestamp);

double udatmo_speed_front(void);

#ifdef __cplusplus
}
#endif

#endif
