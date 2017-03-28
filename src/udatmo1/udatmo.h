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

bool udatmo_obstacle_detected(double timestamp);

void udatmo_clear_detected(void);

void udatmo_shift_history(void);

int udatmo_detect_obstacle_index(carmen_obstacle_distance_mapper_message *current_map,
							carmen_rddf_road_profile_message *rddf,
							int goal_index,
							int rddf_pose_index,
							carmen_ackerman_traj_point_t robot_pose,
							double timestamp);

double udatmo_speed_front(void);

carmen_ackerman_traj_point_t udatmo_get_moving_obstacle_position(void);

double udatmo_get_moving_obstacle_distance(carmen_ackerman_traj_point_t robot_pose, carmen_robot_ackerman_config_t *robot_config);


#ifdef __cplusplus
}
#endif

#endif
