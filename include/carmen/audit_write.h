#ifndef AUDIT_WRITE_H
#define AUDIT_WRITE_H

#include <carmen/proccontrol_messages.h>
#include <carmen/model_predictive_planner_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/task_manager_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/extra_keys_interface.h>

#include <prob_map.h>
#include "audit_messages.h"
#include "audit_interface.h"

#define AUDIT_FILE_HEADER	"### Lume System Audit File ###"

#define SUBDIR_TIME_INTERVAL	(100 * 100)
#define AUDIT_TIME_INTERVAL		(100)

#define	XGV_RED_BUTTON_FLAG	0x4000000


typedef char *(*audit_conv_t)(char *, void *);


typedef struct
{
	const char *param;
	int param_value;
	const char *message_name;
	const char *ipc_message_name;
	const char *message_format;
	size_t message_size;
	audit_conv_t conv_func;
	void *message_data;
	int interpreted;

}	audit_config_t, *audit_config_p;


void audit_write_header(carmen_FILE *outfile, char *audit_id, char *audit_starttime_str);
void audit_write_all_message_formats(carmen_FILE *outfile, const char *audit_message_formats);
void audit_write_all_params(carmen_FILE *outfile, double audit_starttime);
void audit_write_proccontrol_output(carmen_proccontrol_output_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_model_predictive_planner_message(carmen_model_predictive_planner_motion_plan_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_obstacle_avoider_message(carmen_navigator_ackerman_plan_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_behavior_selector_current_state_message(carmen_behavior_selector_state_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_path_goals_and_annotations_message(carmen_behavior_selector_path_goals_and_annotations_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_route_planner_road_network_message(carmen_route_planner_road_network_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_offroad_planner_plan_message(carmen_offroad_planner_plan_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_rddf_play_end_point_message(carmen_rddf_end_point_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_frenet_path_planner_message(carmen_frenet_path_planner_set_of_paths *msg, carmen_FILE *outfile, double timestamp);
void audit_write_route_planner_destination_message(carmen_route_planner_destination_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_task_manager_set_collision_geometry_message(carmen_task_manager_set_collision_geometry_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_task_manager_desired_engage_state_message(carmen_task_manager_desired_engage_state_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_task_manager_set_semi_trailer_type_and_beta_message(carmen_task_manager_set_semi_trailer_type_and_beta_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_mapper_map_message(carmen_mapper_map_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_mapper_map_level1_message(carmen_mapper_map_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_offline_map_message(carmen_mapper_map_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_map_server_compact_cost_map_message(carmen_map_server_compact_cost_map_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_obstacle_distance_mapper_compact_map_message(carmen_obstacle_distance_mapper_compact_map_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_robot_ackerman_motion_command_message(carmen_robot_ackerman_motion_command_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_ford_escape_error_message(carmen_ford_escape_error_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_extra_keys_message(carmen_extra_keys_message_t *msg, carmen_FILE *outfile, double timestamp);
void audit_write_ford_escape_velocity_pid_data_message(velocity_pid_data_message *msg, carmen_FILE *outfile, double timestamp);
void audit_write_ford_escape_steering_pid_data_message(steering_pid_data_message *msg, carmen_FILE *outfile, double timestamp);
#endif
