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

#define        MAX_LINE_LENGTH           (5*4000000)

carmen_FILE *logfile = NULL;
carmen_logfile_index_p logfile_index = NULL;

double playback_starttime = 0.0;
double last_logfile_time = 0.0;
double playback_speed = 1.0;

int current_position = 0;
int stop_position = INT_MAX;
double stop_time = DBL_MAX;
double stop_x = 0.0;
double stop_y = 0.0;
double search_radius = 10.0;

int offset = 0;
int autostart = 0;
int paused = 1;
int fast = 0;
int advance_frame = 0;
int rewind_frame = 0;
int basic_messages = 0;

int g_publish_odometry = 1;

double timestamp_last_message_published = 0.0;

double playback_timestamp;
int playback_timestamp_is_updated = 0;

double playback_pose_x;
double playback_pose_y;
int playback_pose_is_updated = 0;

carmen_base_ackerman_odometry_message odometry_ackerman;
carmen_robot_ackerman_velocity_message velocity_ackerman;

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

carmen_velodyne_variable_scan_message velodyne_variable_scan;
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

carmen_ford_escape_status_message ford_escape_status;

carmen_localize_ackerman_globalpos_message globalpos;


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
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE1", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage1, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE2", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage2, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage3, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE4", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage4, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE5", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage5, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE6", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage6, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE7", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage7, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE8", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage8, 0},
	{(char *) "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE9", (char *) CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage9, 0},
	{(char *) "WEB_CAM_IMAGE", (char *) CARMEN_WEB_CAM_MESSAGE_NAME, (converter_func) carmen_string_to_web_cam_message, &web_cam_message, 0},
	{(char *) "BASEMOTION_ACK", (char *) CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, (converter_func) carmen_string_to_base_ackerman_motion_message, &ackerman_motion_message, 0},
	{(char *) "ULTRASONIC_SONAR_SENSOR", (char *) CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, (converter_func) carmen_string_to_ultrasonic_message, &ultrasonic_message, 0},
	{(char *) "FORD_ESCAPE_STATUS", (char *) CARMEN_FORD_ESCAPE_STATUS_NAME, (converter_func) carmen_string_to_ford_escape_estatus_message, &ford_escape_status, 0},
	{(char *) "GLOBALPOS_ACK", (char *) CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, (converter_func) carmen_string_to_globalpos_message, &globalpos, 0},
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
void wait_for_timestamp(double playback_timestamp)
{
	double current_time; // in logfile time
	struct timeval tv;

	//printf("playback_timestamp = %lf, playback_starttime = %lf, carmen_get_time = %lf\n", playback_timestamp, playback_starttime, carmen_get_time());
	// playback_starttime is offset between file-start and playback-start
	if(playback_starttime == 0.0)
		playback_starttime = (carmen_get_time() - playback_timestamp / playback_speed);
	current_time = (carmen_get_time() - playback_starttime) * playback_speed;
	if(!fast && !paused && playback_timestamp > current_time)
	{
		double towait = (playback_timestamp - current_time) / playback_speed;
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
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE1") == 0) && (bumblebee_basic_stereoimage1.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE2") == 0) && (bumblebee_basic_stereoimage2.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3") == 0) && (bumblebee_basic_stereoimage3.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE4") == 0) && (bumblebee_basic_stereoimage4.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE5") == 0) && (bumblebee_basic_stereoimage5.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE6") == 0) && (bumblebee_basic_stereoimage6.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE7") == 0) && (bumblebee_basic_stereoimage7.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE8") == 0) && (bumblebee_basic_stereoimage8.image_size <= 0));
	error |= ((strcmp(logger_message_name, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE9") == 0) && (bumblebee_basic_stereoimage9.image_size <= 0));

	if (!error)
		return true;

	static double last_update = 0.0;
	double current_time = carmen_get_time();

	if (current_time - last_update > 2.0)
	{
		fprintf(stderr, "\nFILE NOT FOUND: %s\n", logger_message_line);
		last_update = current_time;
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
					do_not_publish |= !check_in_file_message(command, line);
					if (!do_not_publish)
						IPC_publishData(logger_callbacks[i].ipc_message_name, logger_callbacks[i].message_data);
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
	int msg = start_msg;
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
	double t1 = -1.0, t2 = -1.0, x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, r = -1.0;

    if (!carmen_playback_is_valid_message(message, &msg1, &msg2, &t1, &t2, &x1, &y1, &x2, &y2, &r))
    	return;

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
		{
			stop_position = msg2;
			stop_time = DBL_MAX;
			stop_x = stop_y = 0.0;
		}
    }
    else if (t1 >= 0.0 || t2 >= 0.0)
    {
    	if (t1 >= 0.0)
    		find_current_position_by_timestamp(t1);
    	if (t2 >= 0.0)
    	{
    		stop_time = t2;
			stop_position = INT_MAX;
			stop_x = stop_y = 0.0;
    	}
    }
    else
    {
    	if (r >= 0.0)
    		search_radius = r;
    	if (x1 != 0.0 && y1 != 0.0)
    		find_current_position_by_pose(x1, y1);
    	if (x2 != 0.0 && y2 != 0.0)
    	{
    		stop_x = x2, stop_y = y2;
			stop_position = INT_MAX;
			stop_time = DBL_MAX;
    	}
    }
}


void playback_command_handler(carmen_playback_command_message *command)
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

void define_ipc_messages(void)
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

	for (int camera = 1; camera <= 9; camera++)
		carmen_bumblebee_basic_define_messages(camera);

	err = IPC_defineMsg(CARMEN_WEB_CAM_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_WEB_CAM_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_WEB_CAM_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);

	err = IPC_defineMsg(CARMEN_FORD_ESCAPE_STATUS_NAME, IPC_VARIABLE_LENGTH, CARMEN_FORD_ESCAPE_STATUS_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_FORD_ESCAPE_STATUS_NAME);

	carmen_subscribe_message((char *) CARMEN_PLAYBACK_COMMAND_NAME, (char *) CARMEN_PLAYBACK_COMMAND_FMT,
			NULL, sizeof(carmen_playback_command_message), (carmen_handler_t) playback_command_handler, CARMEN_SUBSCRIBE_LATEST);
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
				offset = 0;
				paused = 1;
				print_playback_status();
				stop_position = INT_MAX;
				stop_time = DBL_MAX;
				stop_x = stop_y = 0.0;
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
			carmen_ipc_sleep(0.01);
		if (fast)
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

	fprintf(stderr, "Usage: playback <log_file_name> [args]\n");
	fprintf(stderr, "[args]: -fast {on|off} -autostart {on|off} -basic {on|off}\n");
	fprintf(stderr, "        -play_message  <num>   -stop_message <num>\n");
	fprintf(stderr, "        -play_time     <num>   -stop_time    <num>\n");
	fprintf(stderr, "        -play_pose_x   <num>   -stop_pose_x  <num>\n");
	fprintf(stderr, "        -play_pose_y   <num>   -stop_pose_y  <num>\n");
	fprintf(stderr, "        -search_radius <num>\n");
	exit(-1);
}

void read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "robot",			(char *) "publish_odometry",CARMEN_PARAM_ONOFF,		&(g_publish_odometry),	0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	double current_time = 0.0, current_x = 0.0, current_y = 0.0;

	carmen_param_t param_list2[] =
	{
		{(char *) "commandline",	(char *) "fast",			CARMEN_PARAM_ONOFF,		&(fast),				0, NULL},
		{(char *) "commandline",	(char *) "autostart",		CARMEN_PARAM_ONOFF, 	&(autostart),			0, NULL},
		{(char *) "commandline",	(char *) "basic",			CARMEN_PARAM_ONOFF, 	&(basic_messages),		0, NULL},
		{(char *) "commandline",	(char *) "play_message",	CARMEN_PARAM_INT, 		&(current_position),	0, NULL},
		{(char *) "commandline",	(char *) "stop_message",	CARMEN_PARAM_INT, 		&(stop_position),		0, NULL},
		{(char *) "commandline",	(char *) "play_time",		CARMEN_PARAM_DOUBLE,	&(current_time),		0, NULL},
		{(char *) "commandline",	(char *) "stop_time",		CARMEN_PARAM_DOUBLE,	&(stop_time),			0, NULL},
		{(char *) "commandline",	(char *) "play_pose_x",		CARMEN_PARAM_DOUBLE,	&(current_x),			0, NULL},
		{(char *) "commandline",	(char *) "play_pose_y",		CARMEN_PARAM_DOUBLE,	&(current_y),			0, NULL},
		{(char *) "commandline",	(char *) "stop_pose_x",		CARMEN_PARAM_DOUBLE,	&(stop_x),				0, NULL},
		{(char *) "commandline",	(char *) "stop_pose_y",		CARMEN_PARAM_DOUBLE,	&(stop_y),				0, NULL},
		{(char *) "commandline",	(char *) "search_radius",	CARMEN_PARAM_DOUBLE,	&(search_radius),		0, NULL},
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
    else if (current_x != 0.0 || current_y != 0.0)
    	sprintf(message, "p %lf %lf : %lf %lf", current_x, current_y, stop_x, stop_y);
    else if (stop_x != 0.0 || stop_y != 0.0)
    	sprintf(message, "p : %lf %lf", stop_x, stop_y);

    if (message[0] != 0)
		playback_command_set_message(message);
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
	char *log_file_name;

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
	memset(&ford_escape_status, 0, sizeof(ford_escape_status));
	memset(&globalpos, 0, sizeof(globalpos));

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	if (argc < 2)
		carmen_die("Error: wrong number of parameters: program requires 1 parameter and received %d parameter(s).\n"
				"Usage:\n %s <log_file_name>\n", argc - 1, argv[0]);

	if (strcmp(argv[1], "-h") == 0)
		usage(NULL);

    log_file_name = argv[1];
	logfile = carmen_fopen(log_file_name, "r");
	if (logfile == NULL)
		carmen_die("Error: could not open file %s for reading.\n", log_file_name);

	int fadvise_error = posix_fadvise(fileno(logfile->fp), 0, 0, POSIX_FADV_SEQUENTIAL);
	if (fadvise_error)
		carmen_die("Could not advise POSIX_FADV_SEQUENTIAL on playback.\n");

	strcpy(index_file_name, log_file_name);
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

	main_playback_loop();
	return 0;
}
