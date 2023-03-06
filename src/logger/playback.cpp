/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <fcntl.h>
#include <carmen/carmen.h>
#include <carmen/playback_interface.h>
#include <carmen/carmen_gps.h>
#include <carmen/Gdc_To_Utm_Converter.h>
#include <carmen/readlog.h>
#include <carmen/logger.h>
#include <carmen/writelog.h>
#include <carmen/proccontrol_interface.h>
#include <carmen/proccontrol_messages.h>
#include <carmen/model_predictive_planner_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/task_manager_interface.h>
#include <carmen/audit_interface.h>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <carmen/util_io.h>


#define        MAX_LINE_LENGTH           (5*4000000)

char *log_filename = NULL;
carmen_FILE *logfile = NULL;
carmen_logfile_index_p logfile_index = NULL;

double playback_starttime = 0.0;
double last_logfile_time = 0.0;
double playback_speed = 1.0;

int current_position = 0;
int stop_position = INT_MAX;
bool recur = false;
double recur_play_time = 0.0;
double stop_time = DBL_MAX;
double stop_x = 0.0;
double stop_y = 0.0;
double search_radius = 10.0;
int killall_after_finish = 0;

int offset = 0;
int autostart = 0;
int paused = 1;
int fast = 0;
int advance_frame = 0;
int rewind_frame = 0;
int basic_messages = 0;
char *ignore_list = NULL;

int g_publish_odometry = 1;
int point_cloud_odometry_using_fake_gps = 0;


double timestamp_last_message_published = 0.0;

double playback_timestamp;
int playback_timestamp_is_updated = 0;

double playback_pose_x;
double playback_pose_y;
int playback_pose_is_updated = 0;

int audit_version = 0;

carmen_base_ackerman_odometry_message odometry_ackerman;
carmen_robot_ackerman_velocity_message velocity_ackerman;

carmen_can_dump_can_line_message can_dump;

carmen_visual_odometry_pose6d_message visual_odometry;
carmen_simulator_ackerman_truepos_message truepos_ackerman;
carmen_robot_ackerman_laser_message laser_ackerman1, laser_ackerman2, laser_ackerman3, laser_ackerman4, laser_ackerman5;
carmen_laser_laser_message rawlaser1, rawlaser2, rawlaser3, rawlaser4, rawlaser5;
carmen_laser_ldmrs_message laser_ldmrs;
carmen_laser_ldmrs_new_message laser_ldmrs_new;
carmen_laser_ldmrs_objects_message laser_ldmrs_objects;
carmen_laser_ldmrs_objects_data_message laser_ldmrs_objects_data;

carmen_imu_message imu;
carmen_gps_gpgga_message gpsgga;
carmen_gps_gphdt_message gpshdt;
carmen_gps_gprmc_message gpsrmc;

carmen_kinect_depth_message raw_depth_kinect_0, raw_depth_kinect_1;
carmen_kinect_video_message raw_video_kinect_0, raw_video_kinect_1;

carmen_velodyne_variable_scan_message velodyne_variable_scan, velodyne_variable_scan0, velodyne_variable_scan1, velodyne_variable_scan2, velodyne_variable_scan3, velodyne_variable_scan4, velodyne_variable_scan5, velodyne_variable_scan6, velodyne_variable_scan7, velodyne_variable_scan8, velodyne_variable_scan9,
                                      velodyne_variable_scan10, velodyne_variable_scan11, velodyne_variable_scan12, velodyne_variable_scan13, velodyne_variable_scan14, velodyne_variable_scan15;

carmen_velodyne_partial_scan_message velodyne_partial_scan;
carmen_velodyne_gps_message velodyne_gps;

carmen_xsens_global_euler_message xsens_euler;
carmen_xsens_global_quat_message xsens_quat;
carmen_xsens_global_matrix_message xsens_matrix;
carmen_xsens_mtig_message xsens_mtig;
carmen_bumblebee_basic_stereoimage_message bumblebee_basic_stereoimage1, bumblebee_basic_stereoimage2, bumblebee_basic_stereoimage3, bumblebee_basic_stereoimage4, bumblebee_basic_stereoimage5, bumblebee_basic_stereoimage6, bumblebee_basic_stereoimage7, bumblebee_basic_stereoimage8, bumblebee_basic_stereoimage9, bumblebee_basic_stereoimage10, bumblebee_basic_stereoimage11, bumblebee_basic_stereoimage12, bumblebee_basic_stereoimage13;
camera_message camera1_message, camera2_message, camera3_message, camera4_message, camera5_message, camera6_message, camera7_message, camera8_message, camera9_message, camera10_message, camera11_message, camera12_message, camera13_message, camera14_message, camera15_message, camera16_message, camera17_message, camera18_message, camera19_message, camera20_message;

carmen_web_cam_message web_cam_message;

carmen_pi_imu_message_t pi_imu_message;

carmen_base_ackerman_motion_command_message ackerman_motion_message;
carmen_ultrasonic_sonar_sensor_message ultrasonic_message;

carmen_ford_escape_status_message ford_escape_status;

carmen_localize_ackerman_globalpos_message globalpos;

carmen_model_predictive_planner_motion_plan_message model_predictive_planner_motion_plan_message;
carmen_navigator_ackerman_plan_message navigator_ackerman_plan_message;
carmen_behavior_selector_state_message behavior_selector_state_message;
carmen_behavior_selector_path_goals_and_annotations_message behavior_selector_path_goals_and_annotations_message;
carmen_route_planner_road_network_message route_planner_road_network_message;
carmen_offroad_planner_plan_message offroad_planner_plan_message;
carmen_rddf_end_point_message rddf_end_point_message;
carmen_frenet_path_planner_set_of_paths frenet_path_planner_set_of_paths;
carmen_route_planner_destination_message route_planner_destination_message;
carmen_task_manager_set_collision_geometry_message task_manager_set_collision_geometry_message;
carmen_task_manager_desired_engage_state_message task_manager_desired_engage_state_message;
carmen_task_manager_set_semi_trailer_type_and_beta_message task_manager_set_semi_trailer_type_and_beta_message;
carmen_audit_status_message audit_status_message;


char*
carmen_string_to_model_predictive_planner_motion_plan_message(char* string, carmen_model_predictive_planner_motion_plan_message *msg)
{
	char *current_pos = string;
	int current_vector_size;
	if (strncmp(current_pos, "MODEL_PREDICTIVE_PLANNER", 24) == 0)
		current_pos += 24;

	current_vector_size = CLF_READ_INT(&current_pos);
	if (msg->plan == NULL)
	{
		msg->plan_length = current_vector_size;
		msg->plan = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->plan_length, sizeof(carmen_robot_and_trailers_traj_point_t));
	}
	else if (msg->plan_length != current_vector_size)
	{
		msg->plan_length = current_vector_size;
		msg->plan = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->plan, msg->plan_length * sizeof(carmen_robot_and_trailers_traj_point_t));
	}


	for (int i = 0; i < msg->plan_length; i++)
	{
		msg->plan[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->plan[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->plan[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->plan[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->plan[i].num_trailers; ii++)
				msg->plan[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->plan[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);

		msg->plan[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->plan[i].phi = 		CLF_READ_DOUBLE(&current_pos);
	}

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char*
carmen_string_to_navigator_ackerman_plan_message(char* string, carmen_navigator_ackerman_plan_message *msg)
{
	int current_vector_size;

	char *current_pos = string;

	if (strncmp(current_pos, "OBSTACLE_AVOIDER", 16) == 0)
		current_pos += 16;

	current_vector_size = CLF_READ_INT(&current_pos);
	if (msg->path == NULL)
	{
		msg->path_length = current_vector_size;
		msg->path = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->path_length, sizeof(carmen_robot_and_trailers_traj_point_t));
	}
	else if (msg->path_length != current_vector_size)
	{
		msg->path_length = current_vector_size;
		msg->path = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->path, msg->path_length * sizeof(carmen_robot_and_trailers_traj_point_t));
	}


	for (int i = 0; i < msg->path_length; i++)
	{
		msg->path[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->path[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->path[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->path[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->path[i].num_trailers; ii++)
				msg->path[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->path[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->path[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->path[i].phi = 		CLF_READ_DOUBLE(&current_pos);
	}

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char*
carmen_string_to_behavior_selector_state_message(char* string, carmen_behavior_selector_state_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "BEHAVIOR_SELECTOR_STATE", 23) == 0)
		current_pos += 23;

	msg->task = (carmen_behavior_selector_task_t) CLF_READ_INT(&current_pos);
	msg->algorithm = (carmen_behavior_selector_algorithm_t) CLF_READ_INT(&current_pos);
	msg->low_level_state = (carmen_behavior_selector_low_level_state_t) CLF_READ_INT(&current_pos);
	msg->low_level_state_flags = CLF_READ_INT(&current_pos);
	msg->route_planner_state = (carmen_route_planner_state_t) CLF_READ_INT(&current_pos);
	msg->offroad_planner_request = (offroad_planner_request_t) CLF_READ_INT(&current_pos);
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);

	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char*
carmen_string_to_behavior_selector_path_goals_and_annotations_message(char* string, carmen_behavior_selector_path_goals_and_annotations_message *msg)
{
	int current_vector_size;
	int current_vector_size_back;

	char *current_pos = string;

	if (strncmp(current_pos, "BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS", 44) == 0)
		current_pos += 44;

	current_vector_size = CLF_READ_INT(&current_pos);
	current_vector_size_back = CLF_READ_INT(&current_pos);

	if (msg->poses == NULL)
	{
		msg->number_of_poses = current_vector_size;
		msg->poses = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->number_of_poses, sizeof(carmen_robot_and_trailers_traj_point_t));

		msg->annotations = (int *) calloc(msg->number_of_poses, sizeof(int));
		msg->annotations_codes = (int *) calloc(msg->number_of_poses, sizeof(int));
	}
	else if (msg->number_of_poses != current_vector_size)
	{
		msg->number_of_poses = current_vector_size;
		msg->poses = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->poses, msg->number_of_poses * sizeof(carmen_robot_and_trailers_traj_point_t));

		msg->annotations = (int *) realloc(msg->annotations, msg->number_of_poses * sizeof(int));
		msg->annotations_codes = (int *) realloc(msg->annotations_codes, msg->number_of_poses * sizeof(int));
	}

	if (msg->poses_back == NULL)
	{
		msg->number_of_poses_back = current_vector_size_back;
		msg->poses_back = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->number_of_poses_back, sizeof(carmen_robot_and_trailers_traj_point_t));
	}
	else if (msg->number_of_poses_back != current_vector_size_back)
	{
		msg->number_of_poses_back = current_vector_size_back;
		msg->poses_back = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->poses_back, msg->number_of_poses_back * sizeof(carmen_robot_and_trailers_traj_point_t));
	}

	for (int i = 0; i < msg->number_of_poses; i++)
	{
		msg->poses[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->poses[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->poses[i].num_trailers; ii++)
				msg->poses[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->poses[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].phi = 	CLF_READ_DOUBLE(&current_pos);
	}

	for (int i = 0; i < msg->number_of_poses_back; i++)
	{
		msg->poses_back[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->poses_back[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->poses_back[i].num_trailers; ii++)
				msg->poses_back[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->poses_back[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].phi = 	CLF_READ_DOUBLE(&current_pos);
	}

	for (int i = 0; i < msg->number_of_poses; i++)
		msg->annotations[i] = CLF_READ_INT(&current_pos);

	for (int i = 0; i < msg->number_of_poses; i++)
		msg->annotations_codes[i] = CLF_READ_INT(&current_pos);

	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->goal_list == NULL)
	{
		msg->goal_list_size = current_vector_size;
		msg->goal_list = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->goal_list_size, sizeof(carmen_robot_and_trailers_traj_point_t));
	}
	else if (msg->goal_list_size != current_vector_size)
	{
		msg->goal_list_size = current_vector_size;
		msg->goal_list = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->goal_list, msg->goal_list_size * sizeof(carmen_robot_and_trailers_traj_point_t));
	}

	for (int i = 0; i < msg->goal_list_size; i++)
	{
		msg->goal_list[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->goal_list[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->goal_list[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->goal_list[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->goal_list[i].num_trailers; ii++)
				msg->goal_list[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->goal_list[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->goal_list[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->goal_list[i].phi = 	CLF_READ_DOUBLE(&current_pos);
	}

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char*
carmen_string_to_route_planner_road_network_message(char* string, carmen_route_planner_road_network_message *msg)
{
	char *current_pos = string;

	int current_vector_size;
	int current_vector_size_back;

	if (strncmp(current_pos, "ROUTE_PLANNER_ROAD_NETWORK", 26) == 0)
		current_pos += 26;

	current_vector_size = CLF_READ_INT(&current_pos);
	current_vector_size_back = CLF_READ_INT(&current_pos);

	if (msg->poses == NULL)
	{
		msg->number_of_poses = current_vector_size;
		msg->poses = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->number_of_poses, sizeof(carmen_robot_and_trailers_traj_point_t));

		msg->annotations = (int *) calloc(msg->number_of_poses, sizeof(int));
		msg->annotations_codes = (int *) calloc(msg->number_of_poses, sizeof(int));
	}
	else if (msg->number_of_poses != current_vector_size)
	{
		msg->number_of_poses = current_vector_size;
		msg->poses = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->poses, msg->number_of_poses * sizeof(carmen_robot_and_trailers_traj_point_t));

		msg->annotations = (int *) realloc(msg->annotations, msg->number_of_poses * sizeof(int));
		msg->annotations_codes = (int *) realloc(msg->annotations_codes, msg->number_of_poses * sizeof(int));
	}

	if (msg->poses_back == NULL)
	{
		msg->number_of_poses_back = current_vector_size_back;
		msg->poses_back = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->number_of_poses_back, sizeof(carmen_robot_and_trailers_traj_point_t));
	}
	else if (msg->number_of_poses_back != current_vector_size_back)
	{
		msg->number_of_poses_back = current_vector_size_back;
		msg->poses_back = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->poses_back, msg->number_of_poses_back * sizeof(carmen_robot_and_trailers_traj_point_t));
	}

	for (int i = 0; i < msg->number_of_poses; i++)
	{
		msg->poses[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->poses[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->poses[i].num_trailers; ii++)
				msg->poses[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->poses[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].phi = 	CLF_READ_DOUBLE(&current_pos);
	}

	for (int i = 0; i < msg->number_of_poses_back; i++)
	{
		msg->poses_back[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->poses_back[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->poses_back[i].num_trailers; ii++)
				msg->poses_back[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->poses_back[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].phi = 	CLF_READ_DOUBLE(&current_pos);
	}

	for (int i = 0; i < msg->number_of_poses; i++)
		msg->annotations[i] = CLF_READ_INT(&current_pos);

	for (int i = 0; i < msg->number_of_poses; i++)
		msg->annotations_codes[i] = CLF_READ_INT(&current_pos);

	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->nearby_lanes_indexes == NULL)
	{
		msg->number_of_nearby_lanes = current_vector_size;
		msg->nearby_lanes_indexes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
		msg->nearby_lanes_sizes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
		msg->nearby_lanes_ids = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));

		msg->nearby_lanes_merges_indexes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
		msg->nearby_lanes_merges_sizes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));

		msg->nearby_lanes_forks_indexes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
		msg->nearby_lanes_forks_sizes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));

		msg->nearby_lanes_crossroads_indexes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
		msg->nearby_lanes_crossroads_sizes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));

		msg->nearby_lanes_node_ids = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
	}
	else if (msg->number_of_nearby_lanes != current_vector_size)
	{
		msg->number_of_nearby_lanes = current_vector_size;
		msg->nearby_lanes_indexes = (int *) realloc(msg->nearby_lanes_indexes, msg->number_of_nearby_lanes * sizeof(int));
		msg->nearby_lanes_sizes = (int *) realloc(msg->nearby_lanes_sizes, msg->number_of_nearby_lanes * sizeof(int));
		msg->nearby_lanes_ids = (int *) realloc(msg->nearby_lanes_ids, msg->number_of_nearby_lanes * sizeof(int));

		msg->nearby_lanes_merges_indexes =(int *) realloc(msg->nearby_lanes_merges_indexes, msg->number_of_nearby_lanes * sizeof(int));
		msg->nearby_lanes_merges_sizes = (int *) realloc(msg->nearby_lanes_merges_sizes, msg->number_of_nearby_lanes * sizeof(int));

		msg->nearby_lanes_forks_indexes = (int *) realloc(msg->nearby_lanes_forks_indexes, msg->number_of_nearby_lanes * sizeof(int));
		msg->nearby_lanes_forks_sizes = (int *) realloc(msg->nearby_lanes_forks_sizes, msg->number_of_nearby_lanes * sizeof(int));

		msg->nearby_lanes_crossroads_indexes = (int *) realloc(msg->nearby_lanes_crossroads_indexes, msg->number_of_nearby_lanes * sizeof(int));
		msg->nearby_lanes_crossroads_sizes = (int *) realloc(msg->nearby_lanes_crossroads_sizes, msg->number_of_nearby_lanes * sizeof(int));

		msg->nearby_lanes_node_ids = (int *) realloc(msg->nearby_lanes_node_ids, msg->number_of_nearby_lanes * sizeof(int));
	}

	for (int i = 0; i < msg->number_of_nearby_lanes; i++)
	{
		msg->nearby_lanes_indexes[i] = 	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_sizes[i] =	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_ids[i] =		CLF_READ_INT(&current_pos);
	}

	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->nearby_lanes == NULL)
	{
		msg->nearby_lanes_size = current_vector_size;
		msg->nearby_lanes = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->nearby_lanes_size, sizeof(carmen_robot_and_trailers_traj_point_t));
		msg->traffic_restrictions = (int *) calloc(msg->nearby_lanes_size, sizeof(int));
	}
	else if (msg->nearby_lanes_size != current_vector_size)
	{
		msg->nearby_lanes_size = current_vector_size;
		msg->nearby_lanes = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->nearby_lanes, msg->nearby_lanes_size * sizeof(carmen_robot_and_trailers_traj_point_t));
		msg->traffic_restrictions = (int *) realloc(msg->traffic_restrictions, msg->nearby_lanes_size * sizeof(int));
	}

	for (int i = 0; i < msg->nearby_lanes_size; i++)
	{
		msg->nearby_lanes[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->nearby_lanes[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->nearby_lanes[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->nearby_lanes[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->nearby_lanes[i].num_trailers; ii++)
				msg->nearby_lanes[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->nearby_lanes[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->nearby_lanes[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->nearby_lanes[i].phi = 	CLF_READ_DOUBLE(&current_pos);
	}

	for (int i = 0; i < msg->nearby_lanes_size; i++)
		msg->traffic_restrictions[i] = CLF_READ_INT(&current_pos);

	for (int i = 0; i < msg->number_of_nearby_lanes; i++)
	{
		msg->nearby_lanes_merges_indexes[i] = 	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_merges_sizes[i] =	CLF_READ_INT(&current_pos);
	}

	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->nearby_lanes_merges == NULL)
	{
		msg->nearby_lanes_merges_size = current_vector_size;
		msg->nearby_lanes_merges = (carmen_route_planner_junction_t *) calloc(msg->nearby_lanes_merges_size, sizeof(carmen_route_planner_junction_t));
	}
	else if (msg->nearby_lanes_merges_size != current_vector_size)
	{
		msg->nearby_lanes_merges_size = current_vector_size;
		msg->nearby_lanes_merges = (carmen_route_planner_junction_t *) realloc(msg->nearby_lanes_merges, msg->nearby_lanes_merges_size * sizeof(carmen_route_planner_junction_t));
	}

	for (int i = 0; i < msg->nearby_lanes_merges_size; i++)
	{
		msg->nearby_lanes_merges[i].node_id = 		CLF_READ_INT(&current_pos);
		msg->nearby_lanes_merges[i].index_of_node_in_current_lane = 		CLF_READ_INT(&current_pos);
		msg->nearby_lanes_merges[i].target_node_index_in_nearby_lane = 	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_merges[i].target_lane_id = 	CLF_READ_INT(&current_pos);
	}

	for (int i = 0; i < msg->number_of_nearby_lanes; i++)
	{
		msg->nearby_lanes_forks_indexes[i] = 	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_forks_sizes[i] =	CLF_READ_INT(&current_pos);
	}

	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->nearby_lanes_forks == NULL)
	{
		msg->nearby_lanes_forks_size = current_vector_size;
		msg->nearby_lanes_forks = (carmen_route_planner_junction_t *) calloc(msg->nearby_lanes_forks_size, sizeof(carmen_route_planner_junction_t));
	}
	else if (msg->nearby_lanes_forks_size != current_vector_size)
	{
		msg->nearby_lanes_forks_size = current_vector_size;
		msg->nearby_lanes_forks = (carmen_route_planner_junction_t *) realloc(msg->nearby_lanes_forks, msg->nearby_lanes_forks_size * sizeof(carmen_route_planner_junction_t));
	}

	for (int i = 0; i < msg->nearby_lanes_forks_size; i++)
	{
		msg->nearby_lanes_forks[i].node_id = 		CLF_READ_INT(&current_pos);
		msg->nearby_lanes_forks[i].index_of_node_in_current_lane = 		CLF_READ_INT(&current_pos);
		msg->nearby_lanes_forks[i].target_node_index_in_nearby_lane = 	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_forks[i].target_lane_id = 	CLF_READ_INT(&current_pos);
	}

	for (int i = 0; i < msg->number_of_nearby_lanes; i++)
	{
		msg->nearby_lanes_crossroads_indexes[i] = 	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_crossroads_indexes[i] =	CLF_READ_INT(&current_pos);
	}

	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->nearby_lanes_crossroads == NULL)
	{
		msg->nearby_lanes_crossroads_size = current_vector_size;
		msg->nearby_lanes_crossroads = (carmen_route_planner_junction_t *) calloc(msg->nearby_lanes_crossroads_size, sizeof(carmen_route_planner_junction_t));
	}
	else if (msg->nearby_lanes_crossroads_size != current_vector_size)
	{
		msg->nearby_lanes_crossroads_size = current_vector_size;
		msg->nearby_lanes_crossroads = (carmen_route_planner_junction_t *) realloc(msg->nearby_lanes_crossroads, msg->nearby_lanes_crossroads_size * sizeof(carmen_route_planner_junction_t));
	}

	for (int i = 0; i < msg->nearby_lanes_crossroads_size; i++)
	{
		msg->nearby_lanes_crossroads[i].node_id = 		CLF_READ_INT(&current_pos);
		msg->nearby_lanes_crossroads[i].index_of_node_in_current_lane = 		CLF_READ_INT(&current_pos);
		msg->nearby_lanes_crossroads[i].target_node_index_in_nearby_lane = 	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_crossroads[i].target_lane_id = 	CLF_READ_INT(&current_pos);
	}

	for (int i = 0; i < msg->number_of_nearby_lanes; i++)
	{
		msg->nearby_lanes_node_ids[i] = 	CLF_READ_INT(&current_pos);
	}


	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->route == NULL)
	{
		msg->route_size = current_vector_size_back;
		msg->route = (carmen_route_planner_route_t *) calloc(msg->route_size, sizeof(carmen_route_planner_route_t));
	}
	else if (msg->route_size != current_vector_size_back)
	{
		msg->route_size = current_vector_size_back;
		msg->route = (carmen_route_planner_route_t *) realloc(msg->route, msg->route_size * sizeof(carmen_route_planner_route_t));
	}

	for (int i = 0; i < msg->route_size; i++)
	{
		msg->route[i].pose.x = 		CLF_READ_DOUBLE(&current_pos);
		msg->route[i].pose.y = 		CLF_READ_DOUBLE(&current_pos);
		msg->route[i].pose.theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->route[i].pose.num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->route[i].pose.num_trailers; ii++)
				msg->route[i].pose.trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->route[i].pose.trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->route[i].pose.v = 		CLF_READ_DOUBLE(&current_pos);
		msg->route[i].pose.phi = 	CLF_READ_DOUBLE(&current_pos);
		msg->route[i].node_id = 	CLF_READ_INT(&current_pos);
		msg->route[i].lane_id = 	CLF_READ_INT(&current_pos);
	}

	msg->offroad_planner_request = (offroad_planner_request_t) CLF_READ_INT(&current_pos);
	msg->route_planner_state = (carmen_route_planner_state_t) CLF_READ_INT(&current_pos);


	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char*
carmen_string_to_route_planner_destination_message(char* string, carmen_route_planner_destination_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "ROUTE_PLANNER_DESTINATION", 25) == 0)
		current_pos += 25;

	static char can_line[1024];
	CLF_READ_STRING(can_line, &current_pos);

	msg->destination = can_line;
	msg->destination_point.x = CLF_READ_DOUBLE(&current_pos);
	msg->destination_point.y = CLF_READ_DOUBLE(&current_pos);
	msg->destination_point.theta = CLF_READ_DOUBLE(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char*
carmen_string_to_offroad_planner_plan_message(char* string, carmen_offroad_planner_plan_message *msg)
{
	int current_vector_size;

	char *current_pos = string;

	if (strncmp(current_pos, "OFFROAD_PLANNER_PLAN", 20) == 0)
		current_pos += 20;

	msg->offroad_planner_feedback = (carmen_offroad_planner_feedback_t) CLF_READ_INT(&current_pos);
	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->poses == NULL)
	{
		msg->number_of_poses = current_vector_size;
		msg->poses = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->number_of_poses, sizeof(carmen_robot_and_trailers_traj_point_t));
	}
	else if (msg->number_of_poses != current_vector_size)
	{
		msg->number_of_poses = current_vector_size;
		msg->poses = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->poses, msg->number_of_poses * sizeof(carmen_robot_and_trailers_traj_point_t));
	}

	for (int i = 0; i < msg->number_of_poses; i++)
	{
		msg->poses[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->poses[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->poses[i].num_trailers; ii++)
				msg->poses[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->poses[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses[i].phi = 	CLF_READ_DOUBLE(&current_pos);
	}

	msg->pose_id = CLF_READ_INT(&current_pos);

	msg->transition_pose.x = 		CLF_READ_DOUBLE(&current_pos);
	msg->transition_pose.y = 		CLF_READ_DOUBLE(&current_pos);
	msg->transition_pose.theta = 	CLF_READ_DOUBLE(&current_pos);
	if (audit_version == 1)
	{
		msg->transition_pose.num_trailers = CLF_READ_INT(&current_pos);
		for (int ii = 0; ii < msg->transition_pose.num_trailers; ii++)
			msg->transition_pose.trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
	}
	else
		msg->transition_pose.trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
	msg->transition_pose.v = 		CLF_READ_DOUBLE(&current_pos);
	msg->transition_pose.phi = 		CLF_READ_DOUBLE(&current_pos);

	msg->goal_pose.x = 				CLF_READ_DOUBLE(&current_pos);
	msg->goal_pose.y = 				CLF_READ_DOUBLE(&current_pos);
	msg->goal_pose.theta = 			CLF_READ_DOUBLE(&current_pos);
	if (audit_version == 1)
	{
		msg->goal_pose.num_trailers = CLF_READ_INT(&current_pos);
		for (int ii = 0; ii < msg->goal_pose.num_trailers; ii++)
			msg->goal_pose.trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
	}
	else
		msg->goal_pose.trailer_theta[0] = 			CLF_READ_DOUBLE(&current_pos);
	msg->goal_pose.v = 				CLF_READ_DOUBLE(&current_pos);
	msg->goal_pose.phi = 			CLF_READ_DOUBLE(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char*
carmen_string_to_rddf_end_point_message(char* string, carmen_rddf_end_point_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "RDDF_PLAY_END_POINT", 19) == 0)
		current_pos += 19;
	msg->half_meters_to_final_goal = CLF_READ_INT(&current_pos);
	msg->point.x =		CLF_READ_DOUBLE(&current_pos);
	msg->point.y = 		CLF_READ_DOUBLE(&current_pos);
	msg->point.theta = 	CLF_READ_DOUBLE(&current_pos);
	if (audit_version == 1)
	{
		msg->point.num_trailers = CLF_READ_INT(&current_pos);
		for (int ii = 0; ii < msg->point.num_trailers; ii++)
			msg->point.trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
	}
	else
		msg->point.trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char*
carmen_string_to_frenet_path_planner_set_of_paths(char* string, carmen_frenet_path_planner_set_of_paths *msg)
{
	int current_vector_size, current_vector_size2;

	char *current_pos = string;

	if (strncmp(current_pos, "FRENET_PATH_PLANNER", 19) == 0)
		current_pos += 19;

	current_vector_size = CLF_READ_INT(&current_pos);
	current_vector_size2 = CLF_READ_INT(&current_pos);

	if (msg->set_of_paths == NULL)
	{
		msg->set_of_paths_size = current_vector_size;
		msg->set_of_paths = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->set_of_paths_size, sizeof(carmen_robot_and_trailers_traj_point_t));
	}
	else if (msg->set_of_paths_size != current_vector_size)
	{
		msg->set_of_paths_size = current_vector_size;
		msg->set_of_paths = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->set_of_paths, msg->set_of_paths_size * sizeof(carmen_robot_and_trailers_traj_point_t));
	}

	for (int i = 0; i < msg->set_of_paths_size; i++)
	{
		msg->set_of_paths[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->set_of_paths[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->set_of_paths[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->set_of_paths[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->set_of_paths[i].num_trailers; ii++)
				msg->set_of_paths[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->set_of_paths[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->set_of_paths[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->set_of_paths[i].phi = 		CLF_READ_DOUBLE(&current_pos);
	}
	msg->selected_path = CLF_READ_INT(&current_pos);
	current_vector_size = current_vector_size2;
	current_vector_size2 = CLF_READ_INT(&current_pos);

	if (msg->poses_back == NULL)
	{
		msg->number_of_poses_back = current_vector_size2;
		msg->poses_back = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->number_of_poses_back, sizeof(carmen_robot_and_trailers_traj_point_t));
		msg->rddf_poses_back = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->number_of_poses_back, sizeof(carmen_robot_and_trailers_traj_point_t));

	}
	else if (msg->number_of_poses_back != current_vector_size2)
	{
		msg->number_of_poses_back = current_vector_size2;
		msg->poses_back = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->poses_back, msg->number_of_poses_back * sizeof(carmen_robot_and_trailers_traj_point_t));
		msg->rddf_poses_back = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->rddf_poses_back, msg->number_of_poses_back * sizeof(carmen_robot_and_trailers_traj_point_t));

	}

	for (int i = 0; i < msg->number_of_poses_back; i++)
	{
		msg->poses_back[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->poses_back[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->poses_back[i].num_trailers; ii++)
				msg->poses_back[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->poses_back[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->poses_back[i].phi = 	CLF_READ_DOUBLE(&current_pos);
	}

	if (msg->rddf_poses_ahead == NULL)
	{
		msg->number_of_poses = current_vector_size;
		msg->rddf_poses_ahead = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->number_of_poses, sizeof(carmen_robot_and_trailers_traj_point_t));

		msg->annotations = (int *) calloc(msg->number_of_poses, sizeof(int));
		msg->annotations_codes = (int *) calloc(msg->number_of_poses, sizeof(int));
	}
	else if (msg->number_of_poses != current_vector_size)
	{
		msg->number_of_poses = current_vector_size;
		msg->rddf_poses_ahead = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->rddf_poses_ahead, msg->number_of_poses * sizeof(carmen_robot_and_trailers_traj_point_t));

		msg->annotations = (int *) realloc(msg->annotations, msg->number_of_poses * sizeof(int));
		msg->annotations_codes = (int *) realloc(msg->annotations_codes, msg->number_of_poses * sizeof(int));
	}

	for (int i = 0; i < msg->number_of_poses; i++)
	{
		msg->rddf_poses_ahead[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->rddf_poses_ahead[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->rddf_poses_ahead[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->rddf_poses_ahead[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->rddf_poses_ahead[i].num_trailers; ii++)
				msg->rddf_poses_ahead[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->rddf_poses_ahead[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->rddf_poses_ahead[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->rddf_poses_ahead[i].phi = 		CLF_READ_DOUBLE(&current_pos);
	}

	for (int i = 0; i < msg->number_of_poses_back; i++)
	{
		msg->rddf_poses_back[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->rddf_poses_back[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->rddf_poses_back[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->rddf_poses_back[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->rddf_poses_back[i].num_trailers; ii++)
				msg->rddf_poses_back[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->rddf_poses_back[i].trailer_theta[0] = 		CLF_READ_DOUBLE(&current_pos);
		msg->rddf_poses_back[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->rddf_poses_back[i].phi = 		CLF_READ_DOUBLE(&current_pos);
	}

	for (int i = 0; i < msg->number_of_poses; i++)
		msg->annotations[i] = CLF_READ_INT(&current_pos);

	for (int i = 0; i < msg->number_of_poses; i++)
		msg->annotations_codes[i] = CLF_READ_INT(&current_pos);

	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->nearby_lanes_indexes == NULL)
	{
		msg->number_of_nearby_lanes = current_vector_size;
		msg->nearby_lanes_indexes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
		msg->nearby_lanes_sizes = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
		msg->nearby_lanes_ids = (int *) calloc(msg->number_of_nearby_lanes, sizeof(int));
	}
	else if (msg->number_of_nearby_lanes != current_vector_size)
	{
		msg->number_of_nearby_lanes = current_vector_size;
		msg->nearby_lanes_indexes = (int *) realloc(msg->nearby_lanes_indexes, msg->number_of_nearby_lanes * sizeof(int));
		msg->nearby_lanes_sizes = (int *) realloc(msg->nearby_lanes_sizes, msg->number_of_nearby_lanes * sizeof(int));
		msg->nearby_lanes_ids = (int *) realloc(msg->nearby_lanes_ids, msg->number_of_nearby_lanes * sizeof(int));
	}

	for (int i = 0; i < msg->number_of_nearby_lanes; i++)
	{
		msg->nearby_lanes_indexes[i] = 	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_sizes[i] =	CLF_READ_INT(&current_pos);
		msg->nearby_lanes_ids[i] =		CLF_READ_INT(&current_pos);
	}

	current_vector_size = CLF_READ_INT(&current_pos);

	if (msg->nearby_lanes == NULL)
	{
		msg->nearby_lanes_size = current_vector_size;
		msg->nearby_lanes = (carmen_robot_and_trailers_traj_point_t *) calloc(msg->nearby_lanes_size, sizeof(carmen_robot_and_trailers_traj_point_t));
		msg->traffic_restrictions = (int *) calloc(msg->nearby_lanes_size, sizeof(int));
	}
	else if (msg->nearby_lanes_size != current_vector_size)
	{
		msg->nearby_lanes_size = current_vector_size;
		msg->nearby_lanes = (carmen_robot_and_trailers_traj_point_t *) realloc(msg->nearby_lanes, msg->nearby_lanes_size * sizeof(carmen_robot_and_trailers_traj_point_t));
		msg->traffic_restrictions = (int *) realloc(msg->traffic_restrictions, msg->nearby_lanes_size * sizeof(int));
	}

	for (int i = 0; i < msg->nearby_lanes_size; i++)
	{
		msg->nearby_lanes[i].x = 		CLF_READ_DOUBLE(&current_pos);
		msg->nearby_lanes[i].y = 		CLF_READ_DOUBLE(&current_pos);
		msg->nearby_lanes[i].theta = 	CLF_READ_DOUBLE(&current_pos);
		if (audit_version == 1)
		{
			msg->nearby_lanes[i].num_trailers = CLF_READ_INT(&current_pos);
			for (int ii = 0; ii < msg->nearby_lanes[i].num_trailers; ii++)
				msg->nearby_lanes[i].trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
		}
		else
			msg->nearby_lanes[i].trailer_theta[0] = 	CLF_READ_DOUBLE(&current_pos);
		msg->nearby_lanes[i].v = 		CLF_READ_DOUBLE(&current_pos);
		msg->nearby_lanes[i].phi = 		CLF_READ_DOUBLE(&current_pos);
	}

	for (int i = 0; i < msg->nearby_lanes_size; i++)
		msg->traffic_restrictions[i] = CLF_READ_INT(&current_pos);


	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char*
carmen_string_to_task_manager_set_collision_geometry_message(char* string, carmen_task_manager_set_collision_geometry_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "TASK_MANAGER_SET_GEOMETRY", 25) == 0)
		current_pos += 25;

	msg->geometry = CLF_READ_INT(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char*
carmen_string_to_task_manager_desired_engage_state_message(char* string, carmen_task_manager_desired_engage_state_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "TASK_MANAGER_DESIRED_ENGAGE_STATE", 34) == 0)
		current_pos += 34;

	msg->desired_engage_state = CLF_READ_INT(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char*
carmen_string_to_task_manager_set_semi_trailer_type_and_beta_message(char* string, carmen_task_manager_set_semi_trailer_type_and_beta_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "TASK_MANAGER_SET_SEMI_TRAILER_TYPE_AND_BETA", 43) == 0)
		current_pos += 43;

	msg->semi_trailer_type = CLF_READ_INT(&current_pos);
	if (audit_version == 1)
	{
		msg->num_trailers = CLF_READ_INT(&current_pos);
		for (int ii = 0; ii < msg->num_trailers; ii++)
			msg->trailer_theta[ii] = CLF_READ_DOUBLE(&current_pos);
	}
	else
		msg->trailer_theta[0] = CLF_READ_DOUBLE(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char*
carmen_string_to_audit_status_message(char* string, carmen_audit_status_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "AUDIT_STATUS_MESSAGE", 20) == 0)
		current_pos += 20;

	msg->decision_making_status = 	CLF_READ_INT(&current_pos);
	msg->basic_perception_status = 	CLF_READ_INT(&current_pos);
	msg->lidar_status = 			CLF_READ_INT(&current_pos);
	msg->camera_status = 			CLF_READ_INT(&current_pos);
	msg->other_sensors_status = 	CLF_READ_INT(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


typedef char *(*converter_func)(char *, void *);

typedef struct {
	char *logger_message_name;
	char *ipc_message_name;
	converter_func conv_func;
	void *message_data;
	int interpreted;
} logger_callback_t;

static logger_callback_t logger_callbacks[] =
{
	{(char *) "LASER_LDMRS", (char *) CARMEN_LASER_LDMRS_NAME, (converter_func) carmen_string_to_laser_ldmrs_message, &laser_ldmrs, 0},
	{(char *) "LASER_LDMRS_NEW", (char *) CARMEN_LASER_LDMRS_NEW_NAME, (converter_func) carmen_string_to_laser_ldmrs_new_message, &laser_ldmrs_new, 0},
	{(char *) "LASER_LDMRS_OBJECTS", (char *) CARMEN_LASER_LDMRS_OBJECTS_NAME, (converter_func) carmen_string_to_laser_ldmrs_objects_message, &laser_ldmrs_objects, 0},
	{(char *) "LASER_LDMRS_OBJECTS_DATA", (char *) CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME, (converter_func) carmen_string_to_laser_ldmrs_objects_data_message, &laser_ldmrs_objects_data, 0},
	{(char *) "RAWLASER1", (char *) CARMEN_LASER_FRONTLASER_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser1, 0},
	{(char *) "RAWLASER2", (char *) CARMEN_LASER_REARLASER_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser2, 0},
	{(char *) "RAWLASER3", (char *) CARMEN_LASER_LASER3_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser3, 0},
	{(char *) "RAWLASER4", (char *) CARMEN_LASER_LASER4_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser4, 0},
	{(char *) "RAWLASER5", (char *) CARMEN_LASER_LASER5_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser5, 0},
	{(char *) "ROBOTLASER_ACK1", (char *) CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman1, 0},
	{(char *) "ROBOTLASER_ACK2", (char *) CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman2, 0},
	{(char *) "ROBOTLASER_ACK3", (char *) CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman3, 0},
	{(char *) "ROBOTLASER_ACK4", (char *) CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman4, 0},
	{(char *) "ROBOTLASER_ACK5", (char *) CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman5, 0},
	{(char *) "ODOM_ACK", (char *) CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, (converter_func) carmen_string_to_base_ackerman_odometry_message, &odometry_ackerman, 0},
	{(char *) "ROBOTVELOCITY_ACK", (char *) CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, (converter_func) carmen_string_to_robot_ackerman_velocity_message, &velocity_ackerman, 0},
	{(char *) "CAN_DUMP_CAN_LINE", (char *) CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_NAME, (converter_func) carmen_string_to_carmen_can_dump_can_line_message, &can_dump, 0},
	{(char *) "VISUAL_ODOMETRY", (char *) CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME, (converter_func) carmen_string_to_visual_odometry_message, &visual_odometry, 0},
	{(char *) "TRUEPOS_ACK", (char *) CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, (converter_func) carmen_string_to_simulator_ackerman_truepos_message, &truepos_ackerman, 0},
	{(char *) "IMU", (char *) CARMEN_IMU_MESSAGE_NAME, (converter_func) carmen_string_to_imu_message, &imu, 0},
	{(char *) "NMEAGGA", (char *) CARMEN_GPS_GPGGA_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gpgga_message, &gpsgga, 0},
	{(char *) "NMEAHDT", (char *) CARMEN_GPS_GPHDT_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gphdt_message, &gpshdt, 0},
	{(char *) "NMEARMC", (char *) CARMEN_GPS_GPRMC_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gprmc_message, &gpsrmc, 0},
	{(char *) "RAW_KINECT_DEPTH0", (char *) CARMEN_KINECT_DEPTH_MSG_0_NAME, (converter_func) carmen_string_to_kinect_depth_message, &raw_depth_kinect_0, 0},
	{(char *) "RAW_KINECT_DEPTH1", (char *) CARMEN_KINECT_DEPTH_MSG_1_NAME, (converter_func) carmen_string_to_kinect_depth_message, &raw_depth_kinect_1, 0},
	{(char *) "RAW_KINECT_VIDEO0", (char *) CARMEN_KINECT_VIDEO_MSG_0_NAME, (converter_func) carmen_string_to_kinect_video_message, &raw_video_kinect_0, 0},
	{(char *) "RAW_KINECT_VIDEO1", (char *) CARMEN_KINECT_VIDEO_MSG_1_NAME, (converter_func) carmen_string_to_kinect_video_message, &raw_video_kinect_1, 0},
	{(char *) "VELODYNE_PARTIAL_SCAN", (char *) CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, (converter_func) carmen_string_to_velodyne_partial_scan_message, &velodyne_partial_scan, 0},
	{(char *) "VELODYNE_PARTIAL_SCAN_IN_FILE", (char *) CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, (converter_func) carmen_string_and_file_to_velodyne_partial_scan_message, &velodyne_partial_scan, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN", (char *) "carmen_stereo_velodyne_scan_message8", (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan, 0},
	{(char *) "VELODYNE_GPS", (char *) CARMEN_VELODYNE_GPS_MESSAGE_NAME, (converter_func) carmen_string_to_velodyne_gps_message, &velodyne_gps, 0},
	{(char *) "XSENS_EULER", (char *) CARMEN_XSENS_GLOBAL_EULER_NAME, (converter_func) carmen_string_to_xsens_euler_message, &xsens_euler, 0},
	{(char *) "XSENS_QUAT", (char *) CARMEN_XSENS_GLOBAL_QUAT_NAME, (converter_func) carmen_string_to_xsens_quat_message, &xsens_quat, 0},
	{(char *) "PI_IMU", (char *) CARMEN_PI_IMU_NAME, (converter_func) carmen_string_to_pi_imu_message, &pi_imu_message, 0},
	{(char *) "XSENS_MATRIX", (char *) CARMEN_XSENS_GLOBAL_MATRIX_NAME, (converter_func) carmen_string_to_xsens_matrix_message, &xsens_matrix, 0},
	{(char *) "XSENS_MTIG", (char *) CARMEN_XSENS_MTIG_NAME, (converter_func) carmen_string_to_xsens_mtig_message, &xsens_mtig, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE1", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage1, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE2", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage2, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE3", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage3, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE4", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage4, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE5", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage5, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE6", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage6, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE7", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage7, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE8", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage8, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE9", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage9, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE10", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage10, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE11", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE11_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage11, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE12", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE12_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage12, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE13", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE13_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage13, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE1", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage1, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE2", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage2, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage3, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE4", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage4, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE5", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage5, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE6", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage6, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE7", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage7, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE8", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage8, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE9", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage9, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE10", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage10, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE11", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE11_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage11, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE12", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE12_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage12, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE13", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE13_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage13, 0},
	{(char *) "CAMERA1_MESSAGE", (char *) CAMERA1_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera1_message, 0},
	{(char *) "CAMERA2_MESSAGE", (char *) CAMERA2_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera2_message, 0},
	{(char *) "CAMERA3_MESSAGE", (char *) CAMERA3_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera3_message, 0},
	{(char *) "CAMERA4_MESSAGE", (char *) CAMERA4_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera4_message, 0},
	{(char *) "CAMERA5_MESSAGE", (char *) CAMERA5_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera5_message, 0},
	{(char *) "CAMERA6_MESSAGE", (char *) CAMERA6_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera6_message, 0},
	{(char *) "CAMERA7_MESSAGE", (char *) CAMERA7_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera7_message, 0},
	{(char *) "CAMERA8_MESSAGE", (char *) CAMERA8_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera8_message, 0},
	{(char *) "CAMERA9_MESSAGE", (char *) CAMERA9_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera9_message, 0},
	{(char *) "CAMERA10_MESSAGE", (char *) CAMERA10_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera10_message, 0},
	{(char *) "CAMERA11_MESSAGE", (char *) CAMERA11_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera11_message, 0},
	{(char *) "CAMERA12_MESSAGE", (char *) CAMERA12_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera12_message, 0},
	{(char *) "CAMERA13_MESSAGE", (char *) CAMERA13_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera13_message, 0},
	{(char *) "CAMERA14_MESSAGE", (char *) CAMERA14_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera14_message, 0},
	{(char *) "CAMERA15_MESSAGE", (char *) CAMERA15_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera15_message, 0},
	{(char *) "CAMERA16_MESSAGE", (char *) CAMERA16_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera16_message, 0},
	{(char *) "CAMERA17_MESSAGE", (char *) CAMERA17_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera17_message, 0},
	{(char *) "CAMERA18_MESSAGE", (char *) CAMERA18_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera18_message, 0},
	{(char *) "CAMERA19_MESSAGE", (char *) CAMERA19_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera19_message, 0},
	{(char *) "CAMERA20_MESSAGE", (char *) CAMERA20_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera20_message, 0},
	{(char *) "WEB_CAM_IMAGE", (char *) CARMEN_WEB_CAM_MESSAGE_NAME, (converter_func) carmen_string_to_web_cam_message, &web_cam_message, 0},
	{(char *) "BASEMOTION_ACK", (char *) CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, (converter_func) carmen_string_to_base_ackerman_motion_message, &ackerman_motion_message, 0},
	{(char *) "ULTRASONIC_SONAR_SENSOR", (char *) CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, (converter_func) carmen_string_to_ultrasonic_message, &ultrasonic_message, 0},
	{(char *) "FORD_ESCAPE_STATUS", (char *) CARMEN_FORD_ESCAPE_STATUS_NAME, (converter_func) carmen_string_to_ford_escape_estatus_message, &ford_escape_status, 0},
	{(char *) "GLOBALPOS_ACK", (char *) CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, (converter_func) carmen_string_to_globalpos_message, &globalpos, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN0", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE0_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan0, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN1", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan1, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN2", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan2, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN3", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan3, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN4", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE4_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan4, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN5", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE5_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan5, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN6", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE6_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan6, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN7", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE7_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan7, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN8", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE8_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan8, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN9", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE9_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan9, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN10", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE10_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan10, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN11", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE11_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan11, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN12", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE12_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan12, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN13", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE13_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan13, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN14", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE14_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan14, 0},
	{(char *) "VARIABLE_VELODYNE_SCAN15", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE15_NAME, (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan15, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE0", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE0_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan0, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE1", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan1, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE2", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan2, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE3", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan3, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE4", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE4_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan4, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE5", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE5_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan5, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE6", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE6_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan6, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE7", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE7_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan7, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE8", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE8_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan8, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE9", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE9_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan9, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE10", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE10_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan10, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE11", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE11_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan11, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE12", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE12_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan12, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE13", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE13_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan13, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE14", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE14_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan14, 0},
	{(char *) "VELODYNE_VARIABLE_SCAN_IN_FILE15", (char *) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE15_NAME, (converter_func) carmen_string_and_file_to_variable_velodyne_scan_message, &velodyne_variable_scan15, 0},
	{(char *) "MODEL_PREDICTIVE_PLANNER", 						(char *) CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME , 		(converter_func) carmen_string_to_model_predictive_planner_motion_plan_message, &model_predictive_planner_motion_plan_message, 0},
	{(char *) "OBSTACLE_AVOIDER", 								(char *) CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME, 								(converter_func) carmen_string_to_navigator_ackerman_plan_message, &navigator_ackerman_plan_message, 0},
	{(char *) "BEHAVIOR_SELECTOR_STATE", 						(char *) CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, 						(converter_func) carmen_string_to_behavior_selector_state_message, &behavior_selector_state_message, 0},
	{(char *) "BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS", 	(char *) CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_NAME, 	(converter_func) carmen_string_to_behavior_selector_path_goals_and_annotations_message, &behavior_selector_path_goals_and_annotations_message, 0},
	{(char *) "ROUTE_PLANNER_ROAD_NETWORK", 					(char *) CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME, 					(converter_func) carmen_string_to_route_planner_road_network_message, &route_planner_road_network_message, 0},
	{(char *) "ROUTE_PLANNER_DESTINATION", 						(char *) CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME, 					(converter_func) carmen_string_to_route_planner_destination_message, &route_planner_destination_message, 0},
	{(char *) "OFFROAD_PLANNER_PLAN", 							(char *) CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME, 							(converter_func) carmen_string_to_offroad_planner_plan_message, &offroad_planner_plan_message, 0},
	{(char *) "RDDF_PLAY_END_POINT", 							(char *) CARMEN_RDDF_END_POINT_MESSAGE_NAME, 								(converter_func) carmen_string_to_rddf_end_point_message, &rddf_end_point_message, 0},
	{(char *) "FRENET_PATH_PLANNER", 							(char *) CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME, 				(converter_func) carmen_string_to_frenet_path_planner_set_of_paths, &frenet_path_planner_set_of_paths, 0},
	{(char *) "TASK_MANAGER_SET_COLLISION_GEOMETRY", 			(char *) CARMEN_TASK_MANAGER_SET_GEOMETRY_MESSAGE_NAME, 					(converter_func) carmen_string_to_task_manager_set_collision_geometry_message, &task_manager_set_collision_geometry_message, 0},
	{(char *) "TASK_MANAGER_DESIRED_ENGAGE_STATE", 				(char *) CARMEN_TASK_MANAGER_DESIRED_ENGAGE_STATE_MESSAGE_NAME, 			(converter_func) carmen_string_to_task_manager_desired_engage_state_message, &task_manager_desired_engage_state_message, 0},
	{(char *) "TASK_MANAGER_SET_SEMI_TRAILER_TYPE_AND_BETA", 	(char *) CARMEN_TASK_MANAGER_SET_SEMI_TRAILER_TYPE_AND_BETA_MESSAGE_NAME, 	(converter_func) carmen_string_to_task_manager_set_semi_trailer_type_and_beta_message, &task_manager_set_semi_trailer_type_and_beta_message, 0},
	{(char *) "AUDIT_STATUS_MESSAGE", 							(char *) CARMEN_AUDIT_STATUS_MESSAGE_NAME, 									(converter_func) carmen_string_to_audit_status_message, &audit_status_message, 0},

};


void
publish_info_message()
{
	IPC_RETURN_TYPE err;
	carmen_playback_info_message playback_info_message;

	playback_info_message.message_number = current_position;
	playback_info_message.message_timestamp = playback_timestamp;
	playback_info_message.message_timestamp_difference = playback_starttime + playback_timestamp;
	playback_info_message.playback_speed = playback_speed;

	err = IPC_publishData (CARMEN_PLAYBACK_INFO_MESSAGE_NAME, &playback_info_message);
	carmen_test_ipc (err, "Could not publish", CARMEN_PLAYBACK_INFO_MESSAGE_NAME);

	timestamp_last_message_published = playback_timestamp;
}


void print_playback_status(void)
{
	char str[100];

	if(paused)
		sprintf(str, "PAUSED ");
	else
		sprintf(str, "PLAYING");
	fprintf(stderr, "\rSTATUS:    %s   TIME:    %f    Current message: %d                ",
			str, playback_timestamp, current_position);
}


// ts is in logfile time
void wait_for_timestamp(double playback_ts)
{
	double current_time, ts; // in logfile time
	struct timeval tv;

	// playback_starttime is offset between file-start and playback-start
	ts = carmen_get_time();
	if(playback_starttime == 0.0)
		playback_starttime = ts - playback_ts;
	current_time = (ts - playback_starttime) * playback_speed;
	if(!fast && !paused && playback_ts > current_time)
	{
		double towait = (playback_ts - current_time) / playback_speed;
		tv.tv_sec = (int)floor(towait);
		tv.tv_usec = (towait - tv.tv_sec) * 1e6;
		select(0, NULL, NULL, NULL, &tv);
	}
}


void
update_playback_pose(char *message_name, void *message_data)
{
	double latitude, longitude, elevation = 0.0;

	if (strcmp(message_name, "NMEAGGA") == 0)
	{
		carmen_gps_gpgga_message *gps = (carmen_gps_gpgga_message *) message_data;
		latitude = (gps->lat_orient == 'S') ? - gps->latitude : gps->latitude;
		longitude = (gps->long_orient == 'W') ? - gps->longitude : gps->longitude;
		elevation = gps->sea_level;
	}
	else if (strcmp(message_name, "XSENS_MTIG") == 0)
	{
		carmen_xsens_mtig_message *gps = (carmen_xsens_mtig_message *) message_data;
		latitude = gps->latitude;
		longitude = gps->longitude;
		elevation = gps->height;
	}
	else if (strcmp(message_name, "VELODYNE_GPS") == 0)
	{
		carmen_velodyne_gps_message *gps = (carmen_velodyne_gps_message *) message_data;
		latitude = carmen_global_convert_degmin_to_double(gps->latitude);
		longitude = carmen_global_convert_degmin_to_double(gps->longitude);
		latitude = (gps->latitude_hemisphere == 'S') ? - latitude : latitude;
		longitude = (gps->longitude_hemisphere == 'W') ? - longitude : longitude;
	}
	else
		return;

	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, elevation);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc, utm);

	playback_pose_x = utm.y;
	playback_pose_y = - utm.x;
	playback_pose_is_updated = 1;
}


bool
check_in_file_message(char *logger_message_name, char *logger_message_line)
{
	bool error = false;

	error |= ((strcmp(logger_message_name, "VELODYNE_PARTIAL_SCAN_IN_FILE") == 0) && (velodyne_partial_scan.number_of_32_laser_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE0") == 0) && (velodyne_variable_scan0.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE1") == 0) && (velodyne_variable_scan1.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE2") == 0) && (velodyne_variable_scan2.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE3") == 0) && (velodyne_variable_scan3.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE4") == 0) && (velodyne_variable_scan4.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE5") == 0) && (velodyne_variable_scan5.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE6") == 0) && (velodyne_variable_scan6.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE7") == 0) && (velodyne_variable_scan7.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE8") == 0) && (velodyne_variable_scan8.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE9") == 0) && (velodyne_variable_scan9.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE10") == 0) && (velodyne_variable_scan10.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE11") == 0) && (velodyne_variable_scan11.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE12") == 0) && (velodyne_variable_scan12.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE13") == 0) && (velodyne_variable_scan13.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE14") == 0) && (velodyne_variable_scan14.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "VELODYNE_VARIABLE_SCAN_IN_FILE15") == 0) && (velodyne_variable_scan15.number_of_shots <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE1") == 0) && (bumblebee_basic_stereoimage1.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE2") == 0) && (bumblebee_basic_stereoimage2.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3") == 0) && (bumblebee_basic_stereoimage3.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE4") == 0) && (bumblebee_basic_stereoimage4.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE5") == 0) && (bumblebee_basic_stereoimage5.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE6") == 0) && (bumblebee_basic_stereoimage6.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE7") == 0) && (bumblebee_basic_stereoimage7.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE8") == 0) && (bumblebee_basic_stereoimage8.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE9") == 0) && (bumblebee_basic_stereoimage9.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE10") == 0) && (bumblebee_basic_stereoimage10.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE11") == 0) && (bumblebee_basic_stereoimage11.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE12") == 0) && (bumblebee_basic_stereoimage12.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE13") == 0) && (bumblebee_basic_stereoimage13.image_size <= 0));

	if (!error)
		return false;

	static double last_update = 0.0;
	double current_time = carmen_get_time();

	if (current_time - last_update > 2.0)
	{
		fprintf(stderr, "\nFILE NOT FOUND: %s\n", logger_message_line);
		last_update = current_time;
	}

	return true;
}


int
wildcard_strcmp(const char *wildcard, const char *str)
{
	const char *cp = NULL, *mp = NULL;

	while ((*str != 0) && (*wildcard != '*'))
	{
		if ((*wildcard != *str) && (*wildcard != '?'))
			return (1);

		wildcard++;
		str++;
	}

	while (*str != 0)
	{
		if (*wildcard == '*')
		{
		  wildcard++;
		  if (*wildcard == 0)
			return (0);

		  mp = wildcard;
		  cp = str + 1;
		}
		else if ((*wildcard == *str) || (*wildcard == '?'))
		{
		  wildcard++;
		  str++;
		}
		else
		{
		  wildcard = mp;
		  str = cp;
		  cp++;
		}
	}

	while (*wildcard == '*')
		wildcard++;

	return (*wildcard != 0); // Zero if got a match
}


bool
check_ignore_list(char *message_name)
{
	static char ignore_message[100], *start;

	for (char *p = ignore_list; p != NULL && *p != '\0'; p++)
	{
		start = p;
		p = strchr(p, ',');
		if (p == NULL)
			p = start + strlen(start);
		strncpy(ignore_message, start, p - start);
		ignore_message[p - start] = '\0';

		if (wildcard_strcmp(ignore_message, message_name) == 0)
			return true;
	}

	return false;
}


int
read_message(int message_num, int publish, int no_wait)
{
	static char line[MAX_LINE_LENGTH];
	char *current_pos;
	int i, j;
	char command[100];
	static double last_update = 0;
	double current_time;

	line[0] = 0;
	message_num = (message_num < 0) ? 0 : (message_num >= logfile_index->num_messages) ? (logfile_index->num_messages - 1) : message_num;
	carmen_logfile_read_line(logfile_index, logfile, message_num, MAX_LINE_LENGTH, line);
	current_pos = carmen_next_word(line);
	playback_timestamp_is_updated = 0;
	playback_pose_is_updated = 0;

	for (i = 0; i < (int) (sizeof(logger_callbacks) / sizeof(logger_callback_t)); i++)
	{
		for (j = 0; line[j] != ' ' && line[j] != 0; j++)
			command[j] = line[j];
		command[j] = 0;

		if (strncmp(command, logger_callbacks[i].logger_message_name, j) == 0)
		{
			if (!basic_messages || !logger_callbacks[i].interpreted)
			{
				current_pos = logger_callbacks[i].conv_func(current_pos, logger_callbacks[i].message_data);
				if (*current_pos != '\n' && *current_pos != '\0')
				{
					playback_timestamp = atof(current_pos);
					playback_timestamp_is_updated = 1;
					update_playback_pose(logger_callbacks[i].logger_message_name, logger_callbacks[i].message_data);
					//printf("command = %s, playback_timestamp = %lf\n", command, playback_timestamp);
					if (publish)
					{
						current_time = carmen_get_time();
						if (current_time - last_update > 0.2)
						{
							print_playback_status();
							last_update = current_time;
						}
						if (!no_wait)
							wait_for_timestamp(playback_timestamp);

						int do_not_publish = !g_publish_odometry && (strcmp(logger_callbacks[i].ipc_message_name, CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME) == 0);
						do_not_publish |= check_ignore_list(command) || check_in_file_message(command, line);
						if (!do_not_publish)
							IPC_publishData(logger_callbacks[i].ipc_message_name, logger_callbacks[i].message_data);
					}
				}

				int is_front_laser_message = (strcmp(command, "FLASER") == 0);
				return (is_front_laser_message);
			}
			break;
		}
	}

	return 0;
}


int
find_next_position_with_timestamp(int start_msg, int step)
{
	int msg;

	msg = std::max(0, start_msg);
	msg = std::min(msg, logfile_index->num_messages - 1);
	read_message(msg, 0, 1);

	while (playback_timestamp_is_updated == 0 && step != 0)
	{
		if ((step > 0 && (msg + step) >= logfile_index->num_messages) || (step < 0 && (msg + step) < 0))
			break;

		msg += step;
		read_message(msg, 0, 1);
	}

	return (msg);
}


void
find_current_position_by_timestamp(double timestamp)
{
	int msg1, msg2;
	double dist1, dist2, dist;

	msg1 = find_next_position_with_timestamp(0, 1);
	dist1 = (timestamp - playback_timestamp);

	msg2 = find_next_position_with_timestamp(logfile_index->num_messages - 1, -1);
	dist2 = dist = (timestamp - playback_timestamp);

	if (dist1 <= 0.0)
		msg2 = msg1;

	if (dist2 >= 0.0)
		msg1 = msg2;

    while ((msg2 - msg1) > 1 && dist != 0.0)
	{
        int msg_mid = round(((msg1 + 1) * fabs(dist2) + (msg2 - 1) * fabs(dist1)) / (fabs(dist1) + fabs(dist2)));
    	int msg = find_next_position_with_timestamp(msg_mid, -1);
    	if (msg == msg1)
        	msg = find_next_position_with_timestamp(msg_mid + 1, 1);
    	if (msg == msg2)
    		break;

    	dist = (timestamp - playback_timestamp);
    	if (dist > 0.0)
    	{
    		msg1 = msg;
    		dist1 = dist;
    	}
    	else
    	{
    		msg2 = msg;
    		dist2 = dist;
    	}
	}
	read_message(msg2, 0, 1);
	current_position = logfile_index->current_position - 1;
}


int
find_next_position_with_pose(int start_msg, int step)
{
	int msg = start_msg;
	read_message(msg, 0, 1);

	while (playback_pose_is_updated == 0 && step != 0)
	{
		if ((step > 0 && (msg + step) >= logfile_index->num_messages) || (step < 0 && (msg + step) < 0))
			break;

		msg += step;
		read_message(msg, 0, 1);
	}

	return (msg);
}


void
find_current_position_by_pose(double x, double y)
{
	carmen_point_t target_pose = {x, y, 0.0};

	for (int msg = 0; msg < logfile_index->num_messages; msg += 100)
	{
		msg = find_next_position_with_pose(msg, 1);
		carmen_point_t pose = {playback_pose_x, playback_pose_y, 0.0};
		double dist = DIST2D(pose, target_pose);
		if (dist <= search_radius)
			break;
	}
	current_position = logfile_index->current_position - 1;
}


void
playback_command_set_message(char *message)
{
	int msg1 = -1, msg2 = -1;
	double t1 = -1.0, t2 = -1.0, r1 = -1.0, r2 = -1.0, x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, r = -1.0;

    if (!carmen_playback_is_valid_message(message, &msg1, &msg2, &t1, &t2, &r1, &r2, &x1, &y1, &x2, &y2, &r))
    	return;

    recur = false;
	recur_play_time = 0.0;
	stop_position = INT_MAX;
	stop_time = DBL_MAX;
	stop_x = stop_y = 0.0;

    if (msg1 >= 0 || msg2 >= 0)
    {
		if (msg1 >= 0)
		{
			playback_timestamp = 0.0;
	    	find_next_position_with_timestamp(msg1, -1);
			read_message(msg1, 0, 1);
			current_position = logfile_index->current_position - 1;
		}
		if (msg2 >= 0)
			stop_position = msg2;
    }
    else if (t1 >= 0.0 || t2 >= 0.0)
    {
    	if (t1 >= 0.0)
    		find_current_position_by_timestamp(t1);
    	if (t2 >= 0.0)
    		stop_time = t2;
    }
    else if (r1 >= 0.0 || r2 >= 0.0)
    {
    	recur = true;
    	if (r1 >= 0.0)
    	{
    		recur_play_time = r1;
    		find_current_position_by_timestamp(r1);
    	}
    	if (r2 >= 0.0)
    		stop_time = r2;
    }
    else
    {
    	if (r >= 0.0)
    		search_radius = r;
    	if (x1 != 0.0 && y1 != 0.0)
    		find_current_position_by_pose(x1, y1);
    	if (x2 != 0.0 && y2 != 0.0)
    		stop_x = x2, stop_y = y2;
    }
}


void
check_audit_version()
{
	// Essa função serve apenas para checar a versão do audit usada, que foi inserida após a mudança multi_trailer do carmen. Por causa dessa função, os audits antigos continuam funcionando

	std::string line;
	std::ifstream logfile_s(log_filename);

	if (!logfile_s.is_open())
	{
		std::cerr << "Unable to open the input file: " << log_filename << std::endl;
		exit(-1);
	}

	while (std::getline(logfile_s, line))
	{
		carmen_line_content current_content = create_carmen_line_content(line);
		std::string tag = get_string_from_carmen_line_content(current_content, 0);
		if (tag.at(0) == '#')
		{
			std::string tag_1 = get_string_from_carmen_line_content(current_content, 1);
			if (tag_1.compare("audit") == 0 && get_string_from_carmen_line_content(current_content, 2).compare("id:") == 0)
			{
				std::string tag_2 = get_string_from_carmen_line_content(current_content, 3);
				audit_version = (tag_2.compare("(null)") == 0) ? 0 : std::stoi(tag_2);
				break;
			}
		}
		else
			break;

	}
	logfile_s.close();
}


void
playback_command_handler(carmen_playback_command_message *command)
{
	switch (command->cmd)
	{
		case CARMEN_PLAYBACK_COMMAND_PLAY:
			offset = 0;
			if (paused)
			{
				playback_starttime = 0.0;
				paused = 0;
				//      fprintf(stderr, " PLAY ");
				print_playback_status();
			}
			break;

		case CARMEN_PLAYBACK_COMMAND_STOP:
			offset = 0;
			if(!paused)
			{
				paused = 1;
				//      fprintf(stderr, " STOP ");
				print_playback_status();
			}
			break;

		case CARMEN_PLAYBACK_COMMAND_RESET:
			offset = 0;
			if (!paused)
				paused = 1;
			current_position = 0;
			playback_starttime = 0.0;
			//    fprintf(stderr, "\nRESET ");
			playback_timestamp = 0;
			playback_timestamp_is_updated = 0;
			playback_pose_is_updated = 0;
			print_playback_status();

			break;

		case CARMEN_PLAYBACK_COMMAND_FORWARD:
			offset = command->offset;
			advance_frame = 1;
			break;

		case CARMEN_PLAYBACK_COMMAND_REWIND:
			offset = -1 * command->offset;
			rewind_frame = 1;
			break;

		case CARMEN_PLAYBACK_COMMAND_FWD_SINGLE:
			offset = 0;
			advance_frame = 1;
			break;

		case CARMEN_PLAYBACK_COMMAND_RWD_SINGLE:
			offset = 0;
			rewind_frame = 1;
			break;

		case CARMEN_PLAYBACK_COMMAND_SET_SPEED:
			break;

		case CARMEN_PLAYBACK_COMMAND_SET_MESSAGE:
			offset = 0;
			if (!paused)
				paused = 1;
			playback_command_set_message(command->message);
			//printf("\nspeed = %f, playback_speed = %f\n", command->speed, playback_speed);
			print_playback_status();
			publish_info_message();
			break;
	}
	if (fabs(command->speed - playback_speed) > 0.001)
	{
		playback_starttime = 0.0;
		playback_speed = command->speed;
		print_playback_status();
		publish_info_message();
	}
}

void
define_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_VELOCITY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);

	err = IPC_defineMsg(CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, IPC_VARIABLE_LENGTH, CARMEN_ULTRASONIC_SONAR_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ULTRASONIC_SONAR_SENSOR_NAME);

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_FRONTLASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_REARLASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_REARLASER_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER3_NAME, IPC_VARIABLE_LENGTH, CARMEN_LASER_LASER3_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER3_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER4_NAME, IPC_VARIABLE_LENGTH, CARMEN_LASER_LASER4_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER4_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER5_NAME, IPC_VARIABLE_LENGTH, CARMEN_LASER_LASER5_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER5_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPGGA_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_GPS_GPGGA_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_GPGGA_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPRMC_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_GPS_GPRMC_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_GPRMC_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_KINECT_DEPTH_MSG_0_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_DEPTH_MSG_0_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_DEPTH_MSG_0_NAME);

	err = IPC_defineMsg(CARMEN_KINECT_VIDEO_MSG_0_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_VIDEO_MSG_0_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_VIDEO_MSG_0_NAME);

	err = IPC_defineMsg(CARMEN_KINECT_DEPTH_MSG_1_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_DEPTH_MSG_1_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_DEPTH_MSG_1_NAME);

	err = IPC_defineMsg(CARMEN_KINECT_VIDEO_MSG_1_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_VIDEO_MSG_1_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_VIDEO_MSG_1_NAME);

	err = IPC_defineMsg(CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME);

	err = IPC_defineMsg("carmen_stereo_velodyne_scan_message8", IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", "carmen_stereo_velodyne_scan_message8");

	carmen_velodyne_define_messages();

	err = IPC_defineMsg(CARMEN_VELODYNE_GPS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_GPS_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_GPS_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_MATRIX_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_MATRIX_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_MATRIX_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_EULER_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_EULER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_EULER_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_MTIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_NAME);

	err = IPC_defineMsg(CARMEN_PI_IMU_NAME, IPC_VARIABLE_LENGTH, CARMEN_PI_IMU_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_PI_IMU_NAME);

	for (int camera = 1; camera <= 13; camera++)
		carmen_bumblebee_basic_define_messages(camera);

	for (int camera_number = 1; camera_number <= 20; camera_number++)
		camera_drivers_define_message(camera_number);

	err = IPC_defineMsg(CARMEN_WEB_CAM_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_WEB_CAM_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_WEB_CAM_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);

	err = IPC_defineMsg(CARMEN_FORD_ESCAPE_STATUS_NAME, IPC_VARIABLE_LENGTH, CARMEN_FORD_ESCAPE_STATUS_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_FORD_ESCAPE_STATUS_NAME);

	carmen_subscribe_message((char *) CARMEN_PLAYBACK_COMMAND_NAME, (char *) CARMEN_PLAYBACK_COMMAND_FMT,
			NULL, sizeof(carmen_playback_command_message), (carmen_handler_t) playback_command_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
main_playback_loop(void)
{
	print_playback_status();

	while (1)
	{
		// eu faco esse teste para evitar uma sobrecarga de mensagens sobre o central e o
		// playback_control. vale ressaltar que essa mensagem so possuir carater informativo
		if (fabs(timestamp_last_message_published - playback_timestamp) > 0.05)
			publish_info_message();

		if (advance_frame || rewind_frame)
			paused = 1;

		if (offset != 0)
		{
			playback_starttime = 0.0;
			current_position += offset;
			if(current_position < 0)
				current_position = 0;
			if(current_position >= logfile_index->num_messages - 1)
				current_position = logfile_index->num_messages - 2;
			offset = 0;
		}

		if (!paused && current_position >= logfile_index->num_messages - 1)
		{
			paused = 1;
			current_position = 0;
			playback_starttime = 0.0;
			playback_timestamp = 0;
			playback_timestamp_is_updated = 0;
			playback_pose_is_updated = 0;
			print_playback_status();

			if (killall_after_finish)
			{
				char buf[512];
				FILE *cmd_pipe = popen("pidof -s proccontrol", "r");
				fgets(buf, 512, cmd_pipe);
				pid_t pid = strtoul(buf, NULL, 10);
				pclose( cmd_pipe );
				kill(pid, SIGINT);
			}
		}
		else if (!paused && current_position < logfile_index->num_messages - 1)
		{
			read_message(current_position, 1, 0);
			current_position++;
			carmen_point_t current_pose = {playback_pose_x, playback_pose_y, 0.0}, stop_pose = {stop_x, stop_y, 0.0};
			if ((current_position > stop_position) ||
				(playback_timestamp_is_updated && playback_timestamp >= stop_time) ||
				(playback_pose_is_updated && DIST2D(current_pose, stop_pose) <= search_radius))
			{
				if (recur)
				{
					playback_starttime = 0.0;
					find_current_position_by_timestamp(recur_play_time);
					print_playback_status();
				}
				else
				{
					offset = 0;
					paused = 1;
					print_playback_status();
					stop_position = INT_MAX;
					stop_time = DBL_MAX;
					stop_x = stop_y = 0.0;
				}
			}
		}
		else if (paused && advance_frame)
		{
			//      laser = 0;
			//      while(current_position < logfile_index->num_messages - 1 && !laser) {
			//	laser = read_message(current_position, 1);
			//	current_position++;
			//      }
			if (current_position < logfile_index->num_messages - 1)
			{
				read_message(current_position, 1, 0);
				current_position++;
				publish_info_message();
			}
			print_playback_status();
			advance_frame = 0;
		}
		else if (paused && rewind_frame)
		{
			//      laser = 0;
			//      while(current_position > 0 && !laser) {
			//	current_position--;
			//	laser = read_message(current_position, 0);
			//      }
			//      laser = 0;
			//      while(current_position > 0 && !laser) {
			//	current_position--;
			//	laser = read_message(current_position, 0);
			//      }
			//      read_message(current_position, 1);
			//      current_position++;
			if (current_position > 0)
			{
				current_position--;
				read_message(current_position, 1, 0);
				publish_info_message();
			}
			print_playback_status();
			rewind_frame = 0;
		}
		if (paused)
		{
			if (killall_after_finish)
			{
				char buf[512];
				FILE *cmd_pipe = popen("pidof -s proccontrol", "r");
				fgets(buf, 512, cmd_pipe);
				pid_t pid = strtoul(buf, NULL, 10);
				pclose( cmd_pipe );
				kill(pid, SIGINT);
			}
			carmen_ipc_sleep(0.01);
		}
		if (fast)
			carmen_ipc_sleep(0.000001);
		else
			carmen_ipc_sleep(0.0001);
	}
}


void
usage(char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	fprintf(stderr, "Usage: playback <log_filename> [args]\n");
	fprintf(stderr, "[args]: -fast {on|off} -autostart {on|off} -basic {on|off}\n");
	fprintf(stderr, "        -ignore <comma-separated log message list with wildcards>\n");
	fprintf(stderr, "        -play_message     <num>   -stop_message     <num>\n");
	fprintf(stderr, "        -play_time        <num>   -stop_time        <num>\n");
	fprintf(stderr, "        -recur_play_time  <num>   -recur_stop_time  <num>\n");
	fprintf(stderr, "        -play_pose_x      <num>   -stop_pose_x      <num>\n");
	fprintf(stderr, "        -play_pose_y      <num>   -stop_pose_y      <num>\n");
	fprintf(stderr, "        -search_radius    <num>\n");
	fprintf(stderr, "        -killall_after_finish {on|off}\n");
	exit(-1);
}

void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "robot",			(char *) "publish_odometry", CARMEN_PARAM_ONOFF,		&(g_publish_odometry),	0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));


	carmen_param_allow_unfound_variables(1);
	carmen_param_t param_list_[] =
	{
		{(char *) "point",			(char *) "cloud_odometry_using_fake_gps", CARMEN_PARAM_INT,		&(point_cloud_odometry_using_fake_gps),	0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list_, sizeof(param_list_) / sizeof(param_list_[0]));
	carmen_param_allow_unfound_variables(0);

	double current_time = 0.0, recur_stop_time = DBL_MAX, current_x = 0.0, current_y = 0.0;

	carmen_param_t param_list2[] =
	{
		{(char *) "commandline",	(char *) "fast",				CARMEN_PARAM_ONOFF,		&(fast),				0, NULL},
		{(char *) "commandline",	(char *) "autostart",			CARMEN_PARAM_ONOFF, 	&(autostart),			0, NULL},
		{(char *) "commandline",	(char *) "basic",				CARMEN_PARAM_ONOFF, 	&(basic_messages),		0, NULL},
		{(char *) "commandline",	(char *) "ignore",				CARMEN_PARAM_STRING, 	&(ignore_list),			0, NULL},
		{(char *) "commandline",	(char *) "play_message",		CARMEN_PARAM_INT, 		&(current_position),	0, NULL},
		{(char *) "commandline",	(char *) "stop_message",		CARMEN_PARAM_INT, 		&(stop_position),		0, NULL},
		{(char *) "commandline",	(char *) "play_time",			CARMEN_PARAM_DOUBLE,	&(current_time),		0, NULL},
		{(char *) "commandline",	(char *) "stop_time",			CARMEN_PARAM_DOUBLE,	&(stop_time),			0, NULL},
		{(char *) "commandline",	(char *) "recur_play_time",		CARMEN_PARAM_DOUBLE,	&(recur_play_time),		0, NULL},
		{(char *) "commandline",	(char *) "recur_stop_time",		CARMEN_PARAM_DOUBLE,	&(recur_stop_time),		0, NULL},
		{(char *) "commandline",	(char *) "play_pose_x",			CARMEN_PARAM_DOUBLE,	&(current_x),			0, NULL},
		{(char *) "commandline",	(char *) "play_pose_y",			CARMEN_PARAM_DOUBLE,	&(current_y),			0, NULL},
		{(char *) "commandline",	(char *) "stop_pose_x",			CARMEN_PARAM_DOUBLE,	&(stop_x),				0, NULL},
		{(char *) "commandline",	(char *) "stop_pose_y",			CARMEN_PARAM_DOUBLE,	&(stop_y),				0, NULL},
		{(char *) "commandline",	(char *) "search_radius",		CARMEN_PARAM_DOUBLE,	&(search_radius),		0, NULL},
		{(char *) "commandline",	(char *) "killall_after_finish",	CARMEN_PARAM_ONOFF,		&(killall_after_finish),0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_list2, sizeof(param_list2) / sizeof(param_list2[0]));

	paused = !(autostart);

	char message[2000];
	message[0] = 0;

    if (current_position > 0 || stop_position < INT_MAX)
    	sprintf(message, "%d:%d", current_position, stop_position);
    else if (current_time > 0.0 || stop_time < DBL_MAX)
		sprintf(message, "t %lf:%lf", current_time, stop_time);
    else if (recur_play_time > 0.0 || recur_stop_time < DBL_MAX)
		sprintf(message, "r %lf:%lf", recur_play_time, recur_stop_time);
    else if (current_x != 0.0 || current_y != 0.0)
    	sprintf(message, "p %lf %lf : %lf %lf", current_x, current_y, stop_x, stop_y);
    else if (stop_x != 0.0 || stop_y != 0.0)
    	sprintf(message, "p : %lf %lf", stop_x, stop_y);

    if (message[0] != 0)
		playback_command_set_message(message);
}


void 
shutdown_playback_module(int sig)
{
	if(sig == SIGINT) {
		fprintf(stderr, "\n");
		exit(1);
	}
}


carmen_logfile_index_p 
load_logindex_file(char *index_file_name)
{
	FILE *index_file;
	carmen_logfile_index_p logfile_index;

	index_file = fopen(index_file_name, "r");
	if (index_file == NULL)
		carmen_die("Error: could not open file %s for reading.\n", index_file_name);

	logfile_index = (carmen_logfile_index_p) malloc(sizeof(carmen_logfile_index_t));
	carmen_test_alloc(logfile_index);
	fread(logfile_index, sizeof(carmen_logfile_index_t), 1, index_file);

	// carmen_logfile_index_messages() (in readlog.c) set file size as last offset
	// so, offset array contains one element more than messages
	// it is required by carmen_logfile_read_line to read the last line
	logfile_index->offset = (off_t *) malloc((logfile_index->num_messages + 1) * sizeof(off_t));
	fread(logfile_index->offset, sizeof(off_t), logfile_index->num_messages + 1, index_file);
	logfile_index->current_position = 0;

	fclose(index_file);
	return (logfile_index);
}


void 
save_logindex_file(carmen_logfile_index_p logfile_index, char *index_file_name)
{
	FILE *index_file;

	index_file = fopen(index_file_name, "w");
	if (index_file == NULL)
		carmen_die("Error: could not open file %s for writing.\n", index_file_name);

	fwrite(logfile_index, sizeof(carmen_logfile_index_t), 1, index_file);

	// carmen_logfile_index_messages() (in readlog.c) set file size as last offset
	// so, offset array contains one element more than messages
	// it is required by carmen_logfile_read_line to read the last line
	fwrite(logfile_index->offset, sizeof(off_t), logfile_index->num_messages+1, index_file);

	fclose(index_file);
}


int 
index_file_older_than_log_file(FILE *index_file, carmen_FILE *logfile)
{
	struct stat stat_buf;
	time_t last_log_modification, last_index_modification;

	fstat(fileno(index_file), &stat_buf);
	last_index_modification = stat_buf.st_mtime;

	fstat(fileno(logfile->fp), &stat_buf);
	last_log_modification = stat_buf.st_mtime;

	if (last_log_modification > last_index_modification)
		return (1);
	else
		return (0);
}


void
set_messages()
{
	memset(&odometry_ackerman, 0, sizeof(odometry_ackerman));
	memset(&velocity_ackerman, 0, sizeof(velocity_ackerman));
	memset(&can_dump, 0, sizeof(can_dump));
	memset(&visual_odometry, 0, sizeof(visual_odometry));
	memset(&imu, 0, sizeof(imu));
	memset(&truepos_ackerman, 0, sizeof(truepos_ackerman));
	memset(&laser_ackerman1, 0, sizeof(laser_ackerman1));
	memset(&laser_ackerman2, 0, sizeof(laser_ackerman2));
	memset(&laser_ackerman3, 0, sizeof(laser_ackerman3));
	memset(&laser_ackerman4, 0, sizeof(laser_ackerman4));
	memset(&laser_ackerman5, 0, sizeof(laser_ackerman5));
	memset(&laser_ldmrs, 0, sizeof(laser_ldmrs));
	memset(&laser_ldmrs_new, 0, sizeof(laser_ldmrs_new));
	memset(&laser_ldmrs_objects, 0, sizeof(laser_ldmrs_objects));
	memset(&laser_ldmrs_objects_data, 0, sizeof(laser_ldmrs_objects_data));
	memset(&rawlaser1, 0, sizeof(rawlaser1));
	memset(&rawlaser2, 0, sizeof(rawlaser2));
	memset(&rawlaser3, 0, sizeof(rawlaser3));
	memset(&rawlaser4, 0, sizeof(rawlaser4));
	memset(&rawlaser5, 0, sizeof(rawlaser5));
	memset(&gpsgga, 0, sizeof(gpsgga));
	memset(&gpshdt, 0, sizeof(gpshdt));
	memset(&gpsrmc, 0, sizeof(gpsrmc));
	memset(&raw_depth_kinect_0, 0, sizeof(raw_depth_kinect_0));
	memset(&raw_depth_kinect_1, 0, sizeof(raw_depth_kinect_1));
	memset(&raw_video_kinect_0, 0, sizeof(raw_video_kinect_0));
	memset(&raw_video_kinect_1, 0, sizeof(raw_video_kinect_1));
	memset(&velodyne_partial_scan, 0, sizeof(velodyne_partial_scan));
	memset(&velodyne_variable_scan, 0, sizeof(velodyne_variable_scan));
	memset(&velodyne_variable_scan0, 0, sizeof(velodyne_variable_scan0));
	memset(&velodyne_variable_scan1, 0, sizeof(velodyne_variable_scan1));
	memset(&velodyne_variable_scan2, 0, sizeof(velodyne_variable_scan2));
	memset(&velodyne_variable_scan3, 0, sizeof(velodyne_variable_scan3));
	memset(&velodyne_variable_scan4, 0, sizeof(velodyne_variable_scan4));
	memset(&velodyne_variable_scan5, 0, sizeof(velodyne_variable_scan5));
	memset(&velodyne_variable_scan6, 0, sizeof(velodyne_variable_scan6));
	memset(&velodyne_variable_scan7, 0, sizeof(velodyne_variable_scan7));
	memset(&velodyne_variable_scan8, 0, sizeof(velodyne_variable_scan8));
	memset(&velodyne_variable_scan9, 0, sizeof(velodyne_variable_scan9));
	memset(&velodyne_variable_scan10, 0, sizeof(velodyne_variable_scan10));
	memset(&velodyne_variable_scan11, 0, sizeof(velodyne_variable_scan11));
	memset(&velodyne_variable_scan12, 0, sizeof(velodyne_variable_scan12));
	memset(&velodyne_variable_scan13, 0, sizeof(velodyne_variable_scan13));
	memset(&velodyne_variable_scan14, 0, sizeof(velodyne_variable_scan14));
	memset(&velodyne_variable_scan15, 0, sizeof(velodyne_variable_scan15));
	memset(&velodyne_gps, 0, sizeof(velodyne_gps));
	memset(&xsens_euler, 0, sizeof(xsens_euler));
	memset(&xsens_quat, 0, sizeof(xsens_quat));
	memset(&xsens_matrix, 0, sizeof(xsens_matrix));
	memset(&xsens_mtig, 0, sizeof(xsens_mtig));
	memset(&pi_imu_message, 0, sizeof(pi_imu_message));
	memset(&bumblebee_basic_stereoimage1, 0, sizeof(bumblebee_basic_stereoimage1));
	memset(&bumblebee_basic_stereoimage2, 0, sizeof(bumblebee_basic_stereoimage2));
	memset(&bumblebee_basic_stereoimage3, 0, sizeof(bumblebee_basic_stereoimage3));
	memset(&bumblebee_basic_stereoimage4, 0, sizeof(bumblebee_basic_stereoimage4));
	memset(&bumblebee_basic_stereoimage5, 0, sizeof(bumblebee_basic_stereoimage5));
	memset(&bumblebee_basic_stereoimage6, 0, sizeof(bumblebee_basic_stereoimage6));
	memset(&bumblebee_basic_stereoimage7, 0, sizeof(bumblebee_basic_stereoimage7));
	memset(&bumblebee_basic_stereoimage8, 0, sizeof(bumblebee_basic_stereoimage8));
	memset(&bumblebee_basic_stereoimage9, 0, sizeof(bumblebee_basic_stereoimage9));
	memset(&bumblebee_basic_stereoimage10, 0, sizeof(bumblebee_basic_stereoimage10));
	memset(&bumblebee_basic_stereoimage11, 0, sizeof(bumblebee_basic_stereoimage11));
	memset(&bumblebee_basic_stereoimage12, 0, sizeof(bumblebee_basic_stereoimage12));
	memset(&bumblebee_basic_stereoimage13, 0, sizeof(bumblebee_basic_stereoimage13));
	memset(&camera1_message, 0, sizeof(camera_message));
	memset(&camera2_message, 0, sizeof(camera_message));
	memset(&camera3_message, 0, sizeof(camera_message));
	memset(&camera4_message, 0, sizeof(camera_message));
	memset(&camera5_message, 0, sizeof(camera_message));
	memset(&camera6_message, 0, sizeof(camera_message));
	memset(&camera7_message, 0, sizeof(camera_message));
	memset(&camera8_message, 0, sizeof(camera_message));
	memset(&camera9_message, 0, sizeof(camera_message));
	memset(&camera10_message, 0, sizeof(camera_message));
	memset(&camera11_message, 0, sizeof(camera_message));
	memset(&camera12_message, 0, sizeof(camera_message));
	memset(&camera13_message, 0, sizeof(camera_message));
	memset(&camera14_message, 0, sizeof(camera_message));
	memset(&camera15_message, 0, sizeof(camera_message));
	memset(&camera16_message, 0, sizeof(camera_message));
	memset(&camera17_message, 0, sizeof(camera_message));
	memset(&camera18_message, 0, sizeof(camera_message));
	memset(&camera19_message, 0, sizeof(camera_message));
	memset(&camera20_message, 0, sizeof(camera_message));
	memset(&web_cam_message, 0, sizeof(web_cam_message));
	memset(&ackerman_motion_message, 0, sizeof(ackerman_motion_message));
	memset(&ultrasonic_message, 0, sizeof(ultrasonic_message));
	memset(&ford_escape_status, 0, sizeof(ford_escape_status));
	memset(&globalpos, 0, sizeof(globalpos));

	memset(&model_predictive_planner_motion_plan_message, 0, sizeof(model_predictive_planner_motion_plan_message));
	memset(&navigator_ackerman_plan_message, 0, sizeof(navigator_ackerman_plan_message));
	memset(&behavior_selector_state_message, 0, sizeof(behavior_selector_state_message));
	memset(&behavior_selector_path_goals_and_annotations_message, 0, sizeof(behavior_selector_path_goals_and_annotations_message));
	memset(&route_planner_road_network_message, 0, sizeof(route_planner_road_network_message));
	memset(&offroad_planner_plan_message, 0, sizeof(offroad_planner_plan_message));
	memset(&rddf_end_point_message, 0, sizeof(rddf_end_point_message));
	memset(&frenet_path_planner_set_of_paths, 0, sizeof(frenet_path_planner_set_of_paths));
	memset(&route_planner_destination_message, 0, sizeof(route_planner_destination_message));
	memset(&task_manager_set_collision_geometry_message, 0, sizeof(task_manager_set_collision_geometry_message));
	memset(&task_manager_desired_engage_state_message, 0, sizeof(task_manager_desired_engage_state_message));
	memset(&task_manager_set_semi_trailer_type_and_beta_message, 0, sizeof(task_manager_set_semi_trailer_type_and_beta_message));
}


int
main(int argc, char **argv)
{
	FILE *index_file;
	char index_file_name[2000];

	if (argc < 2 || strcmp(argv[1], "-h") == 0)
		usage(NULL);

	set_messages();

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	log_filename = argv[1];
	logfile = carmen_fopen(log_filename, "r");
	if (logfile == NULL)
		carmen_die("Error: could not open file %s for reading.\n", log_filename);

	int fadvise_error = posix_fadvise(fileno(logfile->fp), 0, 0, POSIX_FADV_SEQUENTIAL);
	if (fadvise_error)
		carmen_die("Could not advise POSIX_FADV_SEQUENTIAL on playback.\n");

	strcpy(index_file_name, log_filename);
	strcat(index_file_name, ".index");
	index_file = fopen(index_file_name, "r");
	if (index_file == NULL)
	{
		logfile_index = carmen_logfile_index_messages(logfile);
		save_logindex_file(logfile_index, index_file_name);
	}
	else if (index_file_older_than_log_file(index_file, logfile))
	{
		fclose(index_file);
		logfile_index = carmen_logfile_index_messages(logfile);
		save_logindex_file(logfile_index, index_file_name);
	}
	else
	{
		logfile_index = load_logindex_file(index_file_name);
	}

	read_parameters (argc, argv);
	define_ipc_messages();
	carmen_playback_define_messages();

	signal(SIGINT, shutdown_playback_module);

	// Método para realizar a checagem da versão do audit para manter a retrocompatibilidade. Só é relevante ao usar o audit.
	check_audit_version();

	main_playback_loop();
	return 0;
}
