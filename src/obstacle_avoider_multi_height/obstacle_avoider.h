#ifndef CARMEN_OBSTACLE_AVOIDER_MAIN_H
#define CARMEN_OBSTACLE_AVOIDER_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include "collision_detection.h"

#define NUM_MOTION_COMMANDS_VECTORS	10
#define	NUM_MOTION_COMMANDS_PER_VECTOR	500
#define NUM_POSES 20
#define NUM_MAPS 5
#define MAX_TRAJECTORY_VECTOR_OF_POINTS_SIZE 500


double carmen_obstacle_avoider_initialize(int argc, char **argv);
void carmen_obstacle_avoider_stop_robot(char *reason);
void carmen_obstacle_avoider_shutdown(int x);
void carmen_obstacle_avoider_usage(char *progname, char *fmt, ...);
void print_motion_command_vector(carmen_ackerman_motion_command_p motion_commands_vector, int num_motion_commands);
void obstacle_avoider_timer_handler(void);
void check_message_absence_timeout_timer_handler(void);

carmen_navigator_ackerman_plan_message build_navigator_ackerman_plan_message(carmen_ackerman_motion_command_p motion_commands_vector, int num_motion_commands, carmen_robot_ackerman_config_t *carmen_robot_ackerman_config, double timestamp);
void copy_grid_mapping_to_map_vector(carmen_mapper_map_message *grid_map, int position);
void copy_cost_map_to_map_vector(carmen_map_t *cost_map, int position);
double get_last_motion_command_total_time(carmen_ackerman_motion_command_p motion_command_vector, int num_motion_commands);
int obstacle_avoider(carmen_ackerman_motion_command_t *motion_commands_vector, int num_motion_commands, carmen_robot_ackerman_config_t *carmen_robot_ackerman_config);
void publish_base_ackerman_motion_command_message_to_stop_robot();
void add_map_to_map_vector(carmen_mapper_map_message *message);
void add_cost_map_to_map_vector(carmen_map_t *cost_map);
void initialize_map_vector(int number_of_maps);
void add_pose_to_pose_vector(carmen_ackerman_traj_point_t pose);
carmen_ackerman_traj_point_t get_current_pose();
void obstacle_avoider_update_map(carmen_obstacle_distance_mapper_map_message *map);
void obstacle_avoider_update_map_level1(carmen_obstacle_distance_mapper_map_message *map);
carmen_obstacle_distance_mapper_map_message *get_current_map();

#ifdef __cplusplus
}
#endif

#endif
