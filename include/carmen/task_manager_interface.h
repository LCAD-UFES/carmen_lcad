#ifndef TASK_MANAGER_INTERFACE_H
#define TASK_MANAGER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include "task_manager_messages.h"


void
carmen_task_manager_subscribe_state_message(carmen_task_manager_state_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_subscribe_set_collision_geometry_message(carmen_task_manager_set_collision_geometry_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_subscribe_desired_engage_state_message(carmen_task_manager_desired_engage_state_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_subscribe_set_semi_trailer_type_and_beta_message(carmen_task_manager_set_semi_trailer_type_and_beta_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);


void
carmen_task_manager_publish_state_message(carmen_task_manager_mission_level_state_t mission_state, char* mission_id, char *mission_filename,
											carmen_robot_and_trailer_pose_t pose, char *info);

void
carmen_task_manager_publish_set_collision_geometry_message(int geometry, double timestamp);

void
carmen_task_manager_publish_desired_engage_state_message(int desired_engage_state, double timestamp);

void
carmen_task_manager_publish_set_semi_trailer_type_and_beta_message(int semi_trailer_type, double beta, double timestamp);

void
carmen_task_manager_read_semi_trailer_parameters(carmen_semi_trailer_config_t *semi_trailer_config, int argc, char **argv, int semi_trailer_type);

#ifdef __cplusplus
}
#endif

#endif
