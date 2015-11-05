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

#define        MAX_LINE_LENGTH           (5*4000000)

carmen_FILE *logfile = NULL;
carmen_logfile_index_p logfile_index = NULL;

double playback_starttime = 0.0;
double last_logfile_time = 0.0;
double playback_speed = 1.0;

int current_position = 0;
int offset = 0;
int paused = 1;
int fast = 0;
int advance_frame = 0;
int rewind_frame = 0;
int basic_messages = 0;

double timestamp_last_message_published = 0;

double playback_timestamp;

carmen_base_ackerman_odometry_message odometry_ackerman;
carmen_robot_ackerman_velocity_message velocity_ackerman;

carmen_visual_odometry_pose6d_message visual_odometry;
carmen_simulator_ackerman_truepos_message truepos_ackerman;
carmen_robot_ackerman_laser_message laser_ackerman1, laser_ackerman2, laser_ackerman3, laser_ackerman4, laser_ackerman5;
carmen_laser_laser_message rawlaser1, rawlaser2, rawlaser3, rawlaser4, rawlaser5;

carmen_imu_message imu;
carmen_gps_gpgga_message gpsgga;
carmen_gps_gprmc_message gpsrmc;

carmen_kinect_depth_message raw_depth_kinect_0, raw_depth_kinect_1;
carmen_kinect_video_message raw_video_kinect_0, raw_video_kinect_1;

carmen_velodyne_partial_scan_message velodyne_partial_scan;
carmen_velodyne_gps_message velodyne_gps;

carmen_xsens_global_euler_message xsens_euler;
carmen_xsens_global_quat_message xsens_quat;
carmen_xsens_global_matrix_message xsens_matrix;
carmen_xsens_mtig_message xsens_mtig;
carmen_bumblebee_basic_stereoimage_message bumblebee_basic_stereoimage1, bumblebee_basic_stereoimage2, bumblebee_basic_stereoimage3, bumblebee_basic_stereoimage4, bumblebee_basic_stereoimage5, bumblebee_basic_stereoimage6, bumblebee_basic_stereoimage7, bumblebee_basic_stereoimage8, bumblebee_basic_stereoimage9;

carmen_web_cam_message web_cam_message;

carmen_base_ackerman_motion_command_message ackerman_motion_message;
carmen_ultrasonic_sonar_sensor_message ultrasonic_message;

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

void playback_command_handler(carmen_playback_command_message *command)
{
	switch(command->cmd) {
	case CARMEN_PLAYBACK_COMMAND_PLAY:
		offset = 0;
		if(paused) {
			playback_starttime = 0.0;
			paused = 0;
			//      fprintf(stderr, " PLAY ");
			print_playback_status();
		}
		break;
	case CARMEN_PLAYBACK_COMMAND_STOP:
		offset = 0;
		if(!paused) {
			paused = 1;
			//      fprintf(stderr, " STOP ");
			print_playback_status();
		}
		break;
	case CARMEN_PLAYBACK_COMMAND_RESET:
		offset = 0;
		if(!paused)
			paused = 1;
		current_position = 0;
		playback_starttime = 0.0;
		//    fprintf(stderr, "\nRESET ");
		playback_timestamp = 0;
		print_playback_status();
		break;
	case CARMEN_PLAYBACK_COMMAND_FORWARD:
		offset = command->arg;
		advance_frame = 1;
		break;
	case CARMEN_PLAYBACK_COMMAND_REWIND:
		offset = -1 * command->arg;
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
	case CARMEN_PLAYBACK_COMMAND_SET_INITIAL_TIME:
		offset = 0;
		if(!paused)
			paused = 1;
		current_position = command->arg;
		//printf("\nspeed = %f, playback_speed = %f\n", command->speed, playback_speed);
		print_playback_status();
		break;
	}
	if(fabs(command->speed - playback_speed) > 0.001) {
		playback_starttime = 0.0;
		playback_speed = command->speed;
		print_playback_status();
	}
}

void register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_ROBOT_ACKERMAN_VELOCITY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);

	err = IPC_defineMsg(CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_ULTRASONIC_SONAR_SENSOR_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_ULTRASONIC_SONAR_SENSOR_NAME);

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_ROBOT_ACKERMAN_FRONTLASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_ROBOT_ACKERMAN_REARLASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_REARLASER_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER3_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LASER_LASER3_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER3_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER4_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LASER_LASER4_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER4_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LASER5_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LASER_LASER5_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER5_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPGGA_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_GPS_GPGGA_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_GPS_GPGGA_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPRMC_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_GPS_GPRMC_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_GPS_GPRMC_MESSAGE_NAME);

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

	err = IPC_defineMsg(CARMEN_VELODYNE_GPS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_GPS_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_GPS_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_MATRIX_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_XSENS_GLOBAL_MATRIX_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_XSENS_GLOBAL_MATRIX_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_EULER_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_XSENS_GLOBAL_EULER_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_XSENS_GLOBAL_EULER_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_XSENS_GLOBAL_QUAT_FMT);
	carmen_test_ipc_exit(err, "Could not define",
			CARMEN_XSENS_GLOBAL_QUAT_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_MTIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_NAME);

	int camera;
	for (camera = 1; camera <= 9; camera++)
		carmen_bumblebee_basic_define_messages(camera);

	err = IPC_defineMsg(CARMEN_WEB_CAM_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_WEB_CAM_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_WEB_CAM_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);

	carmen_subscribe_message(CARMEN_PLAYBACK_COMMAND_NAME,
			CARMEN_PLAYBACK_COMMAND_FMT,
			NULL, sizeof(carmen_playback_command_message),
			(carmen_handler_t)playback_command_handler,
			CARMEN_SUBSCRIBE_LATEST);
}

// ts is in logfile time
void wait_for_timestamp(double playback_timestamp)
{
	double current_time; // in logfile time
	struct timeval tv;

	//printf("playback_timestamp = %lf, playback_starttime = %lf, carmen_get_time = %lf\n", playback_timestamp, playback_starttime, carmen_get_time());
	// playback_starttime is offset between file-start and playback-start
	if(playback_starttime == 0.0)
		playback_starttime = (carmen_get_time() - playback_timestamp / playback_speed);
	current_time = (carmen_get_time() - playback_starttime) * playback_speed;
	if(!fast && !paused && playback_timestamp > current_time) {
		double towait = (playback_timestamp - current_time) / playback_speed;
		tv.tv_sec = (int)floor(towait);
		tv.tv_usec = (towait - tv.tv_sec) * 1e6;
		select(0, NULL, NULL, NULL, &tv);
	}
}

typedef char *(*converter_func)(char *, void *);

typedef struct {
	char *logger_message_name;
	char *ipc_message_name;
	converter_func conv_func;
	void *message_data;
	int interpreted;
} logger_callback_t;

logger_callback_t logger_callbacks[] =
	{
		{"RAWLASER1", CARMEN_LASER_FRONTLASER_NAME, (converter_func)carmen_string_to_laser_laser_message, &rawlaser1, 0},
		{"RAWLASER2", CARMEN_LASER_REARLASER_NAME, (converter_func)carmen_string_to_laser_laser_message, &rawlaser2, 0},
		{"RAWLASER3", CARMEN_LASER_LASER3_NAME, (converter_func)carmen_string_to_laser_laser_message, &rawlaser3, 0},
		{"RAWLASER4", CARMEN_LASER_LASER4_NAME, (converter_func)carmen_string_to_laser_laser_message, &rawlaser4, 0},
		{"RAWLASER5", CARMEN_LASER_LASER5_NAME, (converter_func)carmen_string_to_laser_laser_message, &rawlaser5, 0},
		{"ROBOTLASER_ACK1", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func)carmen_string_to_robot_ackerman_laser_message, &laser_ackerman1, 0},
		{"ROBOTLASER_ACK2", CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, (converter_func)carmen_string_to_robot_ackerman_laser_message, &laser_ackerman2, 0},
		{"ROBOTLASER_ACK3", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func)carmen_string_to_robot_ackerman_laser_message, &laser_ackerman3, 0},
		{"ROBOTLASER_ACK4", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func)carmen_string_to_robot_ackerman_laser_message, &laser_ackerman4, 0},
		{"ROBOTLASER_ACK5", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func)carmen_string_to_robot_ackerman_laser_message, &laser_ackerman5, 0},
		
		{"ODOM_ACK", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, (converter_func) carmen_string_to_base_ackerman_odometry_message, &odometry_ackerman, 0},
		{"ROBOTVELOCITY_ACK", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, (converter_func) carmen_string_to_robot_ackerman_velocity_message, &velocity_ackerman, 0},
		
		{"VISUAL_ODOMETRY", CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME, (converter_func) carmen_string_to_visual_odometry_message, &visual_odometry, 0},
		{"TRUEPOS_ACK", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, (converter_func)carmen_string_to_simulator_ackerman_truepos_message, &truepos_ackerman, 0},
		{"IMU", CARMEN_IMU_MESSAGE_NAME, (converter_func) carmen_string_to_imu_message, &imu, 0},
		{"NMEAGGA", CARMEN_GPS_GPGGA_MESSAGE_NAME, (converter_func)carmen_string_to_gps_gpgga_message, &gpsgga, 0},
		{"NMEARMC", CARMEN_GPS_GPRMC_MESSAGE_NAME, (converter_func)carmen_string_to_gps_gprmc_message, &gpsrmc, 0},
		{"RAW_KINECT_DEPTH0", CARMEN_KINECT_DEPTH_MSG_0_NAME, (converter_func)carmen_string_to_kinect_depth_message, &raw_depth_kinect_0, 0},
		{"RAW_KINECT_DEPTH1", CARMEN_KINECT_DEPTH_MSG_1_NAME, (converter_func)carmen_string_to_kinect_depth_message, &raw_depth_kinect_1, 0},
		{"RAW_KINECT_VIDEO0", CARMEN_KINECT_VIDEO_MSG_0_NAME, (converter_func)carmen_string_to_kinect_video_message, &raw_video_kinect_0, 0},
		{"RAW_KINECT_VIDEO1", CARMEN_KINECT_VIDEO_MSG_1_NAME, (converter_func)carmen_string_to_kinect_video_message, &raw_video_kinect_1, 0},
		{"VELODYNE_PARTIAL_SCAN", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, (converter_func)carmen_string_to_velodyne_partial_scan_message, &velodyne_partial_scan, 0},
		{"VELODYNE_GPS", CARMEN_VELODYNE_GPS_MESSAGE_NAME, (converter_func)carmen_string_to_velodyne_gps_message, &velodyne_gps, 0},
		{"XSENS_EULER", CARMEN_XSENS_GLOBAL_EULER_NAME, (converter_func)carmen_string_to_xsens_euler_message, &xsens_euler, 0},
		{"XSENS_QUAT", CARMEN_XSENS_GLOBAL_QUAT_NAME, (converter_func)carmen_string_to_xsens_quat_message, &xsens_quat, 0},
		{"XSENS_MATRIX", CARMEN_XSENS_GLOBAL_MATRIX_NAME, (converter_func)carmen_string_to_xsens_matrix_message, &xsens_matrix, 0},
		{"XSENS_MTIG", CARMEN_XSENS_MTIG_NAME, (converter_func)carmen_string_to_xsens_mtig_message, &xsens_mtig, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE1", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage1, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE2", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage2, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE3", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage3, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE4", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage4, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE5", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage5, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE6", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage6, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE7", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage7, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE8", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage8, 0},
		{"BUMBLEBEE_BASIC_STEREOIMAGE9", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME, (converter_func)carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage9, 0},
		{"WEB_CAM_IMAGE", CARMEN_WEB_CAM_MESSAGE_NAME, (converter_func) carmen_string_to_web_cam_message, &web_cam_message, 0},
		{"BASEMOTION_ACK", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, (converter_func) carmen_string_to_base_ackerman_motion_message, &ackerman_motion_message, 0},
		{"ULTRASONIC_SONAR_SENSOR", CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, (converter_func) carmen_string_to_ultrasonic_message, &ultrasonic_message, 0},
	};

int read_message(int message_num, int publish, int no_wait)
{
	//  char *line[MAX_LINE_LENGTH];
	char *line;
	char *current_pos;
	int i, j;
	char command[100];
	static double last_update = 0;
	double current_time;

	line = (char *) malloc(MAX_LINE_LENGTH * sizeof(char));
	if (line == NULL)
		carmen_die("Could not alloc memory in playback.c:read_message()\n");

	carmen_logfile_read_line(logfile_index, logfile, message_num,
			MAX_LINE_LENGTH, line);
	current_pos = carmen_next_word(line);

	for(i = 0; i < (int)(sizeof(logger_callbacks) /
			sizeof(logger_callback_t)); i++) {
		/* copy the command over */
		j = 0;
		while(line[j] != ' ') {
			command[j] = line[j];
			j++;
		}
		command[j] = '\0';
		if(strncmp(command, logger_callbacks[i].logger_message_name, j) == 0) {
			if(!basic_messages || !logger_callbacks[i].interpreted) {
				current_pos =
					logger_callbacks[i].conv_func(current_pos,
							logger_callbacks[i].message_data);
				playback_timestamp = atof(current_pos);
				//printf("command = %s, playback_timestamp = %lf\n", command, playback_timestamp);
				if(publish) {
					current_time = carmen_get_time();
					if(current_time - last_update > 0.2) {
						print_playback_status();
						last_update = current_time;
					}
					if (!no_wait)
						wait_for_timestamp(playback_timestamp);
					IPC_publishData(logger_callbacks[i].ipc_message_name,
							logger_callbacks[i].message_data);
				}
				/* return 1 if it is a front laser message */
				free(line);
				return (strcmp(command, "FLASER") == 0);
			}
		}
	}

	free(line);
	return 0;
}


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


void main_playback_loop(void)
{
	print_playback_status();

	while (1)
	{
		// eu faco esse teste para evitar uma sobrecarga de mensagens sobre o central e o
		// playback_control. vale ressaltar que essa mensagem so possuir carater informativo
		if (fabs(timestamp_last_message_published - playback_timestamp) > 0.05)
			publish_info_message();

		if(advance_frame || rewind_frame)
			paused = 1;

		if(offset != 0) {
			playback_starttime = 0.0;
			current_position += offset;
			if(current_position < 0)
				current_position = 0;
			if(current_position >= logfile_index->num_messages - 1)
				current_position = logfile_index->num_messages - 2;
			offset = 0;
		}

		if(!paused && current_position >= logfile_index->num_messages - 1) {
			paused = 1;
			current_position = 0;
			playback_starttime = 0.0;
			playback_timestamp = 0;
			print_playback_status();
		}
		else if(!paused && current_position < logfile_index->num_messages - 1) {
			read_message(current_position, 1, 0);
			current_position++;
		}
		else if(paused && advance_frame) {
			//      laser = 0;
			//      while(current_position < logfile_index->num_messages - 1 && !laser) {
			//	laser = read_message(current_position, 1);
			//	current_position++;
			//      }
			if (current_position < logfile_index->num_messages - 1) {
				read_message(current_position, 1, 0);
				current_position++;
				publish_info_message();
			}
			print_playback_status();
			advance_frame = 0;
		}
		else if(paused && rewind_frame) {
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
			if (current_position > 0) {
				current_position--;
				read_message(current_position, 1, 0);
				publish_info_message();
			}
			print_playback_status();
			rewind_frame = 0;
		}
		if(paused)
			carmen_ipc_sleep(0.01);
		if(fast)
			carmen_ipc_sleep(0.000001);
		else
			carmen_ipc_sleep(0.0001);
	}
}


void usage(char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	fprintf(stderr, "Usage: playback filename <args>\n");
	fprintf(stderr, "\t-fast         - ignore timestamps.\n");
	exit(-1);
}

void read_parameters(int argc, char **argv)
{
	int index;

	if(argc < 2)
		usage("Needs at least one argument.\n");

	for(index = 0; index < argc; index++) {
		if(strncmp(argv[index], "-h", 2) == 0 ||
				strncmp(argv[index], "--help", 6) == 0)
			usage(NULL);
		if(strncmp(argv[index], "-fast", 5) == 0)
			fast = 1;
		if(strncmp(argv[index], "-autostart", 5) == 0)
			paused = 0;
		if(strncmp(argv[index], "-basic", 6) == 0)
			basic_messages = 1;
	}
}

void shutdown_playback_module(int sig)
{
	if(sig == SIGINT) {
		fprintf(stderr, "\n");
		exit(1);
	}
}


carmen_logfile_index_p load_logindex_file(char *index_file_name)
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


void save_logindex_file(carmen_logfile_index_p logfile_index, char *index_file_name)
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


int index_file_older_than_log_file(FILE *index_file, carmen_FILE *logfile)
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


int main(int argc, char **argv)
{
	FILE *index_file;
	char index_file_name[2000];

	memset(&odometry_ackerman, 0, sizeof(odometry_ackerman));
	memset(&velocity_ackerman, 0, sizeof(velocity_ackerman));
	
	memset(&visual_odometry, 0, sizeof(visual_odometry));
	memset(&imu, 0, sizeof(imu));
	memset(&truepos_ackerman, 0, sizeof(truepos_ackerman));
	memset(&laser_ackerman1, 0, sizeof(laser_ackerman1));
	memset(&laser_ackerman2, 0, sizeof(laser_ackerman2));
	memset(&laser_ackerman3, 0, sizeof(laser_ackerman3));
	memset(&laser_ackerman4, 0, sizeof(laser_ackerman4));
	memset(&laser_ackerman5, 0, sizeof(laser_ackerman5));
	memset(&rawlaser1, 0, sizeof(rawlaser1));
	memset(&rawlaser2, 0, sizeof(rawlaser2));
	memset(&rawlaser3, 0, sizeof(rawlaser3));
	memset(&rawlaser4, 0, sizeof(rawlaser4));
	memset(&rawlaser5, 0, sizeof(rawlaser5));
	memset(&gpsgga, 0, sizeof(gpsgga));
	memset(&gpsrmc, 0, sizeof(gpsrmc));
	memset(&raw_depth_kinect_0, 0, sizeof(raw_depth_kinect_0));
	memset(&raw_depth_kinect_1, 0, sizeof(raw_depth_kinect_1));
	memset(&raw_video_kinect_0, 0, sizeof(raw_video_kinect_0));
	memset(&raw_video_kinect_1, 0, sizeof(raw_video_kinect_1));
	memset(&velodyne_partial_scan, 0, sizeof(velodyne_partial_scan));
	memset(&velodyne_gps, 0, sizeof(velodyne_gps));
	memset(&xsens_euler, 0, sizeof(xsens_euler));
	memset(&xsens_quat, 0, sizeof(xsens_quat));
	memset(&xsens_matrix, 0, sizeof(xsens_matrix));
	memset(&xsens_mtig, 0, sizeof(xsens_mtig));
	memset(&bumblebee_basic_stereoimage1, 0, sizeof(bumblebee_basic_stereoimage1));
	memset(&bumblebee_basic_stereoimage2, 0, sizeof(bumblebee_basic_stereoimage2));
	memset(&bumblebee_basic_stereoimage3, 0, sizeof(bumblebee_basic_stereoimage3));
	memset(&bumblebee_basic_stereoimage4, 0, sizeof(bumblebee_basic_stereoimage4));
	memset(&bumblebee_basic_stereoimage5, 0, sizeof(bumblebee_basic_stereoimage5));
	memset(&bumblebee_basic_stereoimage6, 0, sizeof(bumblebee_basic_stereoimage6));
	memset(&bumblebee_basic_stereoimage7, 0, sizeof(bumblebee_basic_stereoimage7));
	memset(&bumblebee_basic_stereoimage8, 0, sizeof(bumblebee_basic_stereoimage8));
	memset(&bumblebee_basic_stereoimage9, 0, sizeof(bumblebee_basic_stereoimage9));
	memset(&web_cam_message, 0, sizeof(web_cam_message));
	memset(&ackerman_motion_message, 0, sizeof(ackerman_motion_message));
	memset(&ultrasonic_message, 0, sizeof(ultrasonic_message));

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	register_ipc_messages ();
	read_parameters (argc, argv);
	carmen_playback_define_messages ();
	signal(SIGINT, shutdown_playback_module);

	logfile = carmen_fopen(argv[1], "r");
	if (logfile == NULL)
		carmen_die("Error: could not open file %s for reading.\n", argv[1]);

	int fadvise_error = posix_fadvise(fileno(logfile->fp), 0, 0, POSIX_FADV_SEQUENTIAL);
	if (fadvise_error)
	{
		printf("Could not advise POSIX_FADV_SEQUENTIAL on playback\n");
		exit(1);
	}

	strcpy(index_file_name, argv[1]);
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

	main_playback_loop();
	return 0;
}

