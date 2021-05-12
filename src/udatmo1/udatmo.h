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

int udatmo_detect_obstacle_index(carmen_obstacle_distance_mapper_map_message *current_map,
							carmen_rddf_road_profile_message *rddf,
							int goal_index,
							int rddf_pose_index,
							carmen_robot_and_trailer_traj_point_t robot_pose,
							double timestamp);

double udatmo_speed_front(void);
double udatmo_speed_left(void);
double udatmo_speed_right(void);
double udatmo_speed_center(void);

carmen_ackerman_traj_point_t udatmo_get_moving_obstacle_position(void);

carmen_ackerman_traj_point_t udatmo_get_moving_obstacle_position_left(void);

carmen_ackerman_traj_point_t udatmo_get_moving_obstacle_position_right(void);

double udatmo_get_moving_obstacle_distance(carmen_robot_and_trailer_traj_point_t robot_pose, carmen_robot_ackerman_config_t *robot_config);

void udatmo_set_behaviour_selector_central_lane_obstacles_safe_distance(double behaviour_selector_central_lane_obstacles_safe_distance);
void udatmo_set_model_predictive_planner_obstacles_safe_distance(double model_predictive_planner_obstacles_safe_distance);

#ifdef __cplusplus
}
#endif

#endif
