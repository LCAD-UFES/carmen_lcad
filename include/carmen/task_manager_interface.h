#ifndef TASK_MANAGER_INTERFACE_H
#define TASK_MANAGER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include "task_manager_messages.h"


void
carmen_task_manager_subscribe_mission_state_message(carmen_task_manager_mission_state_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_subscribe_task_state_message(carmen_task_manager_task_state_message *msg,
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
carmen_task_manager_publish_mission_state_message(carmen_task_manager_mission_level_state_t mission_state, char* mission_id, char *mission_filename,
											carmen_robot_and_trailers_pose_t pose, char *info);

void
carmen_task_manager_publish_task_state_message(carmen_task_manager_task_state_message msg);

void
carmen_task_manager_publish_set_collision_geometry_message(int geometry, double timestamp);

void
carmen_task_manager_publish_desired_engage_state_message(int desired_engage_state, double timestamp);

void
carmen_task_manager_publish_set_semi_trailer_type_and_beta_message(int semi_trailer_type, double beta, double timestamp);

void
carmen_task_manager_read_semi_trailer_parameters(carmen_semi_trailers_config_t *semi_trailer_config, int argc, char **argv, int semi_trailer_type);


void
carmen_task_manager_set_constant_speed_subscribe_message(carmen_task_manager_set_constant_speed_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_set_constant_speed_unsubscribe_message(carmen_handler_t handler);

void
carmen_task_manager_publish_set_constant_speed_message(carmen_task_manager_set_constant_speed_message *message);

void
carmen_task_manager_set_constant_speed_define_message();

void
carmen_task_manager_set_maximum_throttle_subscribe_message(carmen_task_manager_set_maximum_throttle_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_set_maximum_throttle_unsubscribe_message(carmen_handler_t handler);

void
carmen_task_manager_publish_set_maximum_throttle_message(carmen_task_manager_set_maximum_throttle_message *message);

void
carmen_task_manager_set_maximum_throttle_define_message();

void
carmen_task_manager_set_minimum_throttle_subscribe_message(carmen_task_manager_set_minimum_throttle_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_set_minimum_throttle_unsubscribe_message(carmen_handler_t handler);

void
carmen_task_manager_publish_set_minimum_throttle_message(carmen_task_manager_set_minimum_throttle_message *message);

void
carmen_task_manager_set_minimum_throttle_define_message();

void
carmen_task_manager_fast_stop_at_distance_of_region_of_interest_subscribe_message(carmen_task_manager_fast_stop_at_distance_of_region_of_interest_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_fast_stop_at_distance_of_region_of_interest_unsubscribe_message(carmen_handler_t handler);

void
carmen_task_manager_publish_fast_stop_at_distance_of_region_of_interest_message(carmen_task_manager_fast_stop_at_distance_of_region_of_interest_message *message);

void
carmen_task_manager_fast_stop_at_distance_of_region_of_interest_define_message();

void
carmen_task_manager_fast_stop_subscribe_message(carmen_task_manager_fast_stop_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_fast_stop_unsubscribe_message(carmen_handler_t handler);

void
carmen_task_manager_publish_fast_stop_message(carmen_task_manager_fast_stop_message *message);

void
carmen_task_manager_fast_stop_define_message();

void
carmen_task_manager_distance_to_region_of_interest_subscribe_message(carmen_task_manager_distance_to_region_of_interest_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_distance_to_region_of_interest_unsubscribe_message(carmen_handler_t handler);

void
carmen_task_manager_publish_distance_to_region_of_interest_message(carmen_task_manager_distance_to_region_of_interest_message *message);

void
carmen_task_manager_distance_to_region_of_interest_define_message();

void
carmen_task_manager_turn_engine_on_off_subscribe_message(carmen_task_manager_turn_engine_on_off_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_turn_engine_on_off_unsubscribe_message(carmen_handler_t handler);

void
carmen_task_manager_publish_turn_engine_on_off_message(carmen_task_manager_turn_engine_on_off_message *message);

void
carmen_task_manager_turn_engine_on_off_define_message();


#ifdef __cplusplus
}
#endif

#endif
