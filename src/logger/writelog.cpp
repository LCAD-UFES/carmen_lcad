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

#include <carmen/carmen.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>

//byte numbers
#define GET_LOW_ORDER_NIBBLE(x) (int_to_nibble_hex[x & 0xf])
#define GET_HIGH_ORDER_NIBBLE(x) (int_to_nibble_hex[(x >> 4) & 0xf])

//short numbers
#define GET_SHORT_FIRST_NIBBLE(x) (int_to_nibble_hex[x & 0xf])
#define GET_SHORT_SECOND_NIBBLE(x) (int_to_nibble_hex[(x >> 4) & 0xf])
#define GET_SHORT_THIRD_NIBBLE(x) (int_to_nibble_hex[(x >> 8) & 0xf])
#define GET_SHORT_FOURTH_NIBBLE(x) (int_to_nibble_hex[(x >> 12) & 0xf])

char *hex_char_image = NULL; // Stores the image as a string of nibbles, i.e., a hexadecimal digit ([0-9a-fA-F])
char *hex_char_image_kinect = NULL;
char *hex_char_depth_kinect = NULL;
char int_to_nibble_hex[16];
static int frame_number = 1;

void carmen_logwrite_write_robot_name(char *robot_name, carmen_FILE *outfile)
{
	carmen_fprintf(outfile, "# robot: %s\n", robot_name);
}

void carmen_logwrite_write_header(carmen_FILE *outfile)
{
	carmen_fprintf(outfile, "%s\n", CARMEN_LOGFILE_HEADER);
	carmen_fprintf(outfile, "# file format is one message per line\n");
	carmen_fprintf(outfile,
			"# message_name [message contents] ipc_timestamp ipc_hostname logger_timestamp\n");
	carmen_fprintf(outfile,
			"# message formats defined: PARAM SYNC ODOM RAWLASER1 RAWLASER2 RAWLASER3 RAWLASER4 ROBOTLASER1 ROBOTLASER2 FLASER RLASER LASER3 LASER4\n");
	carmen_fprintf(outfile, "# PARAM param_name param_value\n");
	carmen_fprintf(outfile, "# COMMENT text \n");
	carmen_fprintf(outfile, "# SYNC tagname\n");
	carmen_fprintf(outfile, "# ODOM x y theta tv rv accel\n");
	carmen_fprintf(outfile,
			"# TRUEPOS true_x true_y true_theta odom_x odom_y odom_theta\n");
	carmen_fprintf(outfile,
			"# RAWLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]\n");
	carmen_fprintf(outfile,
			"# RAWLASER2 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]\n");
	carmen_fprintf(outfile,
			"# RAWLASER3 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]\n");
	carmen_fprintf(outfile,
			"# RAWLASER4 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]\n");
	carmen_fprintf(outfile,
			"# POSITIONLASER laserid x y z phi(roll) theta(pitch) psi(yaw) \n");
	carmen_fprintf(outfile,
			"# ROBOTLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values] laser_pose_x laser_pose_y laser_pose_theta robot_pose_x robot_pose_y robot_pose_theta laser_tv laser_rv forward_safety_dist side_safty_dist turn_axis\n");
	carmen_fprintf(outfile,
			"# ROBOTLASER2 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values] laser_pose_x laser_pose_y laser_pose_theta robot_pose_x robot_pose_y robot_pose_theta laser_tv laser_rv forward_safety_dist side_safty_dist turn_axis\n");
	carmen_fprintf(outfile,
			"# NMEAGGA gpsnr utc latitude_dm lat_orient longitude_dm long_orient gps_quality num_satellites hdop sea_level alititude geo_sea_level geo_sep data_age\n");
	carmen_fprintf(outfile,
			"# NMEARMC gpsnr validity utc latitude_dm lat_orient longitude_dm long_orient speed course variation var_dir date\n");
	carmen_fprintf(outfile,
			"# SONAR cone_angle num_sonars [sonar_reading] [sonar_offsets x y theta]\n");
	carmen_fprintf(outfile,
			"# BUMPER num_bumpers [bumper_reading] [bumper_offsets x y]\n");
	carmen_fprintf(outfile, "# SCANMARK start_stop_indicator laserID \n");
	carmen_fprintf(outfile,
			"# IMU accelerationX accelerationY accelerationZ quaternion_q0 quaternion_q1 quaternion_q2 quaternion_q3 magneticfieldX magneticfieldY magneticfieldZ gyroX gyroY gyroZ\n");
	carmen_fprintf(outfile,
			"# XSENS_EULER accelerationX accelerationY accelerationZ pitch roll yaw magneticfieldX magneticfieldY magneticfieldZ gyroX gyroY gyroZ\n");
	carmen_fprintf(outfile,
			"# XSENS_QUAT accelerationX accelerationY accelerationZ quaternion_q0 quaternion_q1 quaternion_q2 quaternion_q3 magneticfieldX magneticfieldY magneticfieldZ gyroX gyroY gyroZ\n");
	carmen_fprintf(outfile,
			"# XSENS_MATRIX accelerationX accelerationY accelerationZ matrix_0-0 matrix_0-1 matrix_0-2 matrix_1-0 matrix_1-1 matrix_1-2 matrix_2-0 matrix_2-1 matrix_2-2s magneticfieldX magneticfieldY magneticfieldZ gyroX gyroY gyroZ\n");
	carmen_fprintf(outfile,
			"# XSENS_MTIG quaternion_q0 quaternion_q1 quaternion_q2 quaternion_q3 accelerationX accelerationY accelerationZ gyroX gyroY gyroZ magneticfieldX magneticfieldY magneticfieldZ velocityX velocityY velocityZ latitude longitude height gps_fix xkf_valid sensor_ID\n");
	carmen_fprintf(outfile, "# VECTORMOVE distance theta\n");
	carmen_fprintf(outfile, "# ROBOTVELOCITY tv rv \n");
	carmen_fprintf(outfile, "# VECTORMOVE distance theta\n");
	carmen_fprintf(outfile, "# ROBOTVELOCITY tv rv \n");
	carmen_fprintf(outfile,
			"# FOLLOWTRAJECTORY x y theta tv rv num readings [trajectory points: x y theta tv rv]\n");
	carmen_fprintf(outfile, "# BASEVELOCITY tv rv \n");
	carmen_fprintf(outfile, "# \n");
	carmen_fprintf(outfile, "# OLD LOG MESSAGES: \n");
	carmen_fprintf(outfile,
			"# (old) # FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta\n");
	carmen_fprintf(outfile,
			"# (old) # RLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta\n");
	carmen_fprintf(outfile, "# (old) # LASER3 num_readings [range_readings]\n");
	carmen_fprintf(outfile, "# (old) # LASER4 num_readings [range_readings]\n");
	carmen_fprintf(outfile,
			"# (old) # REMISSIONFLASER num_readings [range_readings remission_value]\n");
	carmen_fprintf(outfile,
			"# (old) # REMISSIONRLASER num_readings [range_readings remission_value]\n");
	carmen_fprintf(outfile,
			"# (old) # REMISSIONLASER3 num_readings [range_readings remission_value]\n");
	carmen_fprintf(outfile,
			"# (old) # REMISSIONLASER4 num_readings [range_readings remission_value]\n");
}

void carmen_logwrite_write_odometry_ackerman(
		carmen_base_ackerman_odometry_message *odometry, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile, "ODOM_ACK %f %f %f %f %f %f %s %f\n", odometry->x,
			odometry->y, odometry->theta, odometry->v, odometry->phi,
			odometry->timestamp, odometry->host, timestamp);
}

void carmen_logwrite_write_visual_odometry(
		carmen_visual_odometry_pose6d_message *odometry, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile,
			"VISUAL_ODOMETRY %f %f %f %f %f %f %f %f %f %s %f\n",
			odometry->pose_6d.x, odometry->pose_6d.y, odometry->pose_6d.z,
			odometry->pose_6d.roll, odometry->pose_6d.pitch,
			odometry->pose_6d.yaw, odometry->v, odometry->phi,
			odometry->timestamp, odometry->host, timestamp);
}

void carmen_logwrite_write_ackerman_truepos(
		carmen_simulator_ackerman_truepos_message *truepos,
		carmen_FILE *outfile, double timestamp)
{
	carmen_fprintf(outfile, "TRUEPOS_ACK %f %f %f %f %f %f %f %f %f %s %f\n",
			truepos->truepose.x, truepos->truepose.y, truepos->truepose.theta,
			truepos->odometrypose.x, truepos->odometrypose.y,
			truepos->odometrypose.theta, truepos->v, truepos->phi,
			truepos->timestamp, truepos->host, timestamp);
}

void carmen_logwrite_write_laser_laser(carmen_laser_laser_message *laser,
		int laser_num, carmen_FILE *outfile, double timestamp)
{
	int i;

	carmen_fprintf(outfile, "RAWLASER%d ", laser_num);
	carmen_fprintf(outfile, "%d %f %f %f %f %f %d ", laser->config.laser_type,
			laser->config.start_angle, laser->config.fov,
			laser->config.angular_resolution, laser->config.maximum_range,
			laser->config.accuracy, laser->config.remission_mode);
	carmen_fprintf(outfile, "%d ", laser->num_readings);
	for (i = 0; i < laser->num_readings; i++)
		carmen_fprintf(outfile, "%.3f ", laser->range[i]);
	carmen_fprintf(outfile, "%d ", laser->num_remissions);
	for (i = 0; i < laser->num_remissions; i++)
		carmen_fprintf(outfile, "%f ", laser->remission[i]);
	carmen_fprintf(outfile, "%f %s %f\n", laser->timestamp, laser->host,
			timestamp);
}

void carmen_logwrite_write_laser_ldmrs(carmen_laser_ldmrs_message *laser,
		int laser_num, carmen_FILE *outfile, double timestamp)
{
	int i;
	(void) laser_num;
	carmen_fprintf(outfile, "LASER_LDMRS ");
	carmen_fprintf(outfile, "%d %f %f %d %f %f %d ", laser->scan_number,
			laser->scan_start_time, laser->scan_end_time,
			laser->angle_ticks_per_rotation, laser->start_angle,
			laser->end_angle, laser->scan_points);

	for (i = 0; i < laser->scan_points; i++)
	{
		carmen_fprintf(outfile, "%f %f %f %d ",
				laser->arraypoints[i].horizontal_angle,
				laser->arraypoints[i].vertical_angle,
				laser->arraypoints[i].radial_distance,
				laser->arraypoints[i].flags);
	}

	carmen_fprintf(outfile, "%f %s %f\n", laser->timestamp, laser->host,
			timestamp);
}

void carmen_logwrite_write_laser_ldmrs_new(
		carmen_laser_ldmrs_new_message *laser, int laser_num,
		carmen_FILE *outfile, double timestamp)
{
	int i;
	(void) laser_num;
	carmen_fprintf(outfile, "LASER_LDMRS_NEW ");
	carmen_fprintf(outfile, "%d %d %d %f %f %d %f %f %d %d ",
			laser->scan_number, laser->scanner_status, laser->sync_phase_offset,
			laser->scan_start_time, laser->scan_end_time,
			laser->angle_ticks_per_rotation, laser->start_angle,
			laser->end_angle, laser->scan_points, laser->flags);

	for (i = 0; i < laser->scan_points; i++)
	{
		carmen_fprintf(outfile, "%f %f %f %d %d %d ",
				laser->arraypoints[i].horizontal_angle,
				laser->arraypoints[i].vertical_angle,
				laser->arraypoints[i].radial_distance,
				laser->arraypoints[i].layer, laser->arraypoints[i].echo,
				laser->arraypoints[i].flags);
	}

	carmen_fprintf(outfile, "%f %s %f\n", laser->timestamp, laser->host,
			timestamp);
}

void carmen_logwrite_write_laser_ldmrs_objects(
		carmen_laser_ldmrs_objects_message *laser, int laser_num,
		carmen_FILE *outfile, double timestamp)
{
	int i;
	(void) laser_num;
	carmen_fprintf(outfile, "LASER_LDMRS_OBJECTS ");
	carmen_fprintf(outfile, "%d ", laser->num_objects);
	for (i = 0; i < laser->num_objects; i++)
	{
		carmen_fprintf(outfile, "%d %f %f %f %f %f %f %d ",
				laser->objects_list[i].id, laser->objects_list[i].x,
				laser->objects_list[i].y, laser->objects_list[i].lenght,
				laser->objects_list[i].width, laser->objects_list[i].velocity,
				laser->objects_list[i].orientation,
				laser->objects_list[i].classId);
	}

	carmen_fprintf(outfile, "%f %s %f\n", laser->timestamp, laser->host,
			timestamp);
}

void carmen_logwrite_write_laser_ldmrs_objects_data(
		carmen_laser_ldmrs_objects_data_message *laser, int laser_num,
		carmen_FILE *outfile, double timestamp)
{
	int i;
	(void) laser_num;
	carmen_fprintf(outfile, "LASER_LDMRS_OBJECTS_DATA ");
	carmen_fprintf(outfile, "%d ", laser->num_objects);
	for (i = 0; i < laser->num_objects; i++)
	{
		carmen_fprintf(outfile,
				"%d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d ",
				laser->objects_data_list[i].object_id,
				laser->objects_data_list[i].object_age,
				laser->objects_data_list[i].object_prediction_age,
				laser->objects_data_list[i].reference_point_x,
				laser->objects_data_list[i].reference_point_y,
				laser->objects_data_list[i].reference_point_sigma_x,
				laser->objects_data_list[i].reference_point_sigma_y,
				laser->objects_data_list[i].closest_point_x,
				laser->objects_data_list[i].closest_point_y,
				laser->objects_data_list[i].bounding_box_center_x,
				laser->objects_data_list[i].bounding_box_center_y,
				laser->objects_data_list[i].bounding_box_length,
				laser->objects_data_list[i].bounding_box_width,
				laser->objects_data_list[i].object_box_center_x,
				laser->objects_data_list[i].object_box_center_y,
				laser->objects_data_list[i].object_box_lenght,
				laser->objects_data_list[i].object_box_width,
				laser->objects_data_list[i].object_box_orientation,
				laser->objects_data_list[i].abs_velocity_x,
				laser->objects_data_list[i].abs_velocity_y,
				laser->objects_data_list[i].abs_velocity_sigma_x,
				laser->objects_data_list[i].abs_velocity_sigma_y,
				laser->objects_data_list[i].relative_velocity_x,
				laser->objects_data_list[i].relative_velocity_y,
				laser->objects_data_list[i].class_id);
	}

	carmen_fprintf(outfile, "%f %s %f\n", laser->timestamp, laser->host,
			timestamp);
}

void carmen_logwrite_write_robot_ackerman_laser(
		carmen_robot_ackerman_laser_message *laser, int laser_num,
		carmen_FILE *outfile, double timestamp)
{
	int i;

	carmen_fprintf(outfile, "ROBOTLASER_ACK%d ", laser_num);
	carmen_fprintf(outfile, "%d %f %f %f %f %f %d ", laser->config.laser_type,
			laser->config.start_angle, laser->config.fov,
			laser->config.angular_resolution, laser->config.maximum_range,
			laser->config.accuracy, laser->config.remission_mode);
	carmen_fprintf(outfile, "%d ", laser->num_readings);
	for (i = 0; i < laser->num_readings; i++)
		carmen_fprintf(outfile, "%.3f ", laser->range[i]);
	carmen_fprintf(outfile, "%d ", laser->num_remissions);
	for (i = 0; i < laser->num_remissions; i++)
		carmen_fprintf(outfile, "%f ", laser->remission[i]);
	carmen_fprintf(outfile, "%f %f %f %f %f %f ", laser->laser_pose.x,
			laser->laser_pose.y, laser->laser_pose.theta, laser->robot_pose.x,
			laser->robot_pose.y, laser->robot_pose.theta);
	carmen_fprintf(outfile, "%f %f %f %f %f ", laser->v, laser->phi,
			laser->forward_safety_dist, laser->side_safety_dist,
			laser->turn_axis);
	carmen_fprintf(outfile, "%f %s %f\n", laser->timestamp, laser->host,
			timestamp);
}

void carmen_logwrite_write_param(char *module, char *variable, char *value,
		double ipc_time, char *hostname, carmen_FILE *outfile, double timestamp)
{
	carmen_fprintf(outfile, "PARAM %s_%s %s %f %s %f\n", module, variable,
			value, ipc_time, hostname, timestamp);
}

void carmen_logwrite_write_sync(carmen_logger_sync_message *sync_message,
		carmen_FILE *outfile)
{
	carmen_fprintf(outfile, "SYNC %s %f %s %f\n", sync_message->tag,
			sync_message->timestamp, sync_message->host, carmen_get_time());
}

void carmen_logwrite_write_localize_ackerman(
		carmen_localize_ackerman_globalpos_message *msg, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile,
			"GLOBALPOS_ACK %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %s %f\n",
			msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta,
			msg->globalpos_std.x, msg->globalpos_std.y,
			msg->globalpos_std.theta, msg->odometrypos.x, msg->odometrypos.y, msg->odometrypos.theta,
			msg->pose.orientation.pitch, msg->pose.orientation.roll, msg->pose.orientation.yaw,
			msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
			msg->velocity.x, msg->velocity.y, msg->velocity.z,
			msg->v, msg->phi, msg->globalpos_xy_cov,
			msg->converged, msg->timestamp, msg->host, timestamp);
}

void carmen_logger_write_gps_gpgga(carmen_gps_gpgga_message *gps_msg,
		carmen_FILE *outfile, double timestamp)
{
	char lat_o = gps_msg->lat_orient;
	char long_o = gps_msg->long_orient;

	if (lat_o == '\0')
		lat_o = 'N';

	if (long_o == '\0')
		long_o = 'E';

	carmen_fprintf(outfile,
			"NMEAGGA %d %lf %lf %c %lf %c %d %d %lf %lf %lf %lf %lf %d %lf %s %lf\n",
			gps_msg->nr, gps_msg->utc, gps_msg->latitude_dm, lat_o,
			gps_msg->longitude_dm, long_o, gps_msg->gps_quality,
			gps_msg->num_satellites, gps_msg->hdop, gps_msg->sea_level,
			gps_msg->altitude, gps_msg->geo_sea_level, gps_msg->geo_sep,
			gps_msg->data_age, gps_msg->timestamp, gps_msg->host, timestamp);
}

void carmen_logger_write_gps_gphdt(carmen_gps_gphdt_message *gps_msg,
		carmen_FILE *outfile, double timestamp)
{
	carmen_fprintf(outfile, "NMEAHDT %d %lf %d %lf %s %lf\n", gps_msg->nr,
			gps_msg->heading, gps_msg->valid, gps_msg->timestamp, gps_msg->host,
			timestamp);
}

void carmen_logger_write_gps_gprmc(carmen_gps_gprmc_message *gps_msg,
		carmen_FILE *outfile, double timestamp)
{
	char lat_o = gps_msg->lat_orient;
	char long_o = gps_msg->long_orient;

	char vardir = gps_msg->var_dir;

	if (lat_o == '\0')
		lat_o = 'N';

	if (long_o == '\0')
		long_o = 'E';

	if (vardir == '\0')
		vardir = 'E';

	carmen_fprintf(outfile,
			"NMEARMC %d %d %lf %lf %c %lf %c %lf %lf %lf %c %d %lf %s %lf\n",
			gps_msg->nr, gps_msg->validity, gps_msg->utc, gps_msg->latitude_dm,
			lat_o, gps_msg->longitude_dm, long_o, gps_msg->speed,
			gps_msg->true_course, gps_msg->variation, vardir, gps_msg->date,
			gps_msg->timestamp, gps_msg->host, timestamp);
}

void carmen_logwrite_write_ultrasonic_sonar_sensor(
		carmen_ultrasonic_sonar_sensor_message *sonar, carmen_FILE *outfile,
		double timestamp)
{
	int i;

	carmen_fprintf(outfile, "ULTRASONIC_SONAR_SENSOR %d %d %lf %lf %lf %lf ",
			sonar->number_of_sonars, sonar->sonar_beans, sonar->fov,
			sonar->angle_step, sonar->start_angle, sonar->max_range);
	for (i = 0; i < 4; i++)
		carmen_fprintf(outfile, "%.2lf ", sonar->sensor[i]);
	carmen_fprintf(outfile, "%lf %s %lf\n", sonar->timestamp, sonar->host,
			timestamp);
}

void carmen_logwrite_write_pantilt_scanmark(
		carmen_pantilt_scanmark_message *scanmark, carmen_FILE *outfile,
		double timestamp)
{

	carmen_fprintf(outfile, "SCANMARK ");

	carmen_fprintf(outfile, "%d ", scanmark->type);
	carmen_fprintf(outfile, "%d ", scanmark->laserid);

	carmen_fprintf(outfile, "%lf %s %lf\n", scanmark->timestamp, scanmark->host,
			timestamp);
}

void carmen_logwrite_write_pantilt_laserpos(
		carmen_pantilt_laserpos_message *laserpos, carmen_FILE *outfile,
		double timestamp)
{

	carmen_fprintf(outfile, "POSITIONLASER %d ", laserpos->id);

	carmen_fprintf(outfile, "%f %f %f ", laserpos->x, laserpos->y, laserpos->z);
	carmen_fprintf(outfile, "%f %f %f ", laserpos->phi, laserpos->theta,
			laserpos->psi);

	carmen_fprintf(outfile, "%lf %s %lf\n", laserpos->timestamp, laserpos->host,
			timestamp);
}

void carmen_logwrite_write_imu(carmen_imu_message *msg, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile, "IMU ");
	carmen_fprintf(outfile, "%lf ", msg->accX);
	carmen_fprintf(outfile, "%lf ", msg->accY);
	carmen_fprintf(outfile, "%lf ", msg->accZ);

	carmen_fprintf(outfile, "%lf ", msg->q0);
	carmen_fprintf(outfile, "%lf ", msg->q1);
	carmen_fprintf(outfile, "%lf ", msg->q2);
	carmen_fprintf(outfile, "%lf ", msg->q3);

	carmen_fprintf(outfile, "%lf ", msg->magX);
	carmen_fprintf(outfile, "%lf ", msg->magY);
	carmen_fprintf(outfile, "%lf ", msg->magZ);

	carmen_fprintf(outfile, "%lf ", msg->gyroX);
	carmen_fprintf(outfile, "%lf ", msg->gyroY);
	carmen_fprintf(outfile, "%lf ", msg->gyroZ);

	carmen_fprintf(outfile, "%lf %s %lf\n", msg->timestamp, msg->host,
			timestamp);
}

void carmen_logwrite_write_xsens_euler(carmen_xsens_global_euler_message* msg,
		carmen_FILE *outfile, double timestamp)
{
	carmen_fprintf(outfile, "XSENS_EULER ");
	carmen_fprintf(outfile, "%lf ", msg->m_acc.x);
	carmen_fprintf(outfile, "%lf ", msg->m_acc.y);
	carmen_fprintf(outfile, "%lf ", msg->m_acc.z);

	carmen_fprintf(outfile, "%lf ", msg->euler_data.m_pitch);
	carmen_fprintf(outfile, "%lf ", msg->euler_data.m_roll);
	carmen_fprintf(outfile, "%lf ", msg->euler_data.m_yaw);

	carmen_fprintf(outfile, "%lf ", msg->m_mag.x);
	carmen_fprintf(outfile, "%lf ", msg->m_mag.y);
	carmen_fprintf(outfile, "%lf ", msg->m_mag.z);

	carmen_fprintf(outfile, "%lf ", msg->m_gyr.x);
	carmen_fprintf(outfile, "%lf ", msg->m_gyr.y);
	carmen_fprintf(outfile, "%lf ", msg->m_gyr.z);

	carmen_fprintf(outfile, "%lf %hu %lf %s %lf\n", msg->m_temp, msg->m_count,
			msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_xsens_quat(carmen_xsens_global_quat_message* msg,
		carmen_FILE *outfile, double timestamp)
{
	carmen_fprintf(outfile, "XSENS_QUAT ");
	carmen_fprintf(outfile, "%lf ", msg->m_acc.x);
	carmen_fprintf(outfile, "%lf ", msg->m_acc.y);
	carmen_fprintf(outfile, "%lf ", msg->m_acc.z);

	carmen_fprintf(outfile, "%lf ", msg->quat_data.m_data[0]);
	carmen_fprintf(outfile, "%lf ", msg->quat_data.m_data[1]);
	carmen_fprintf(outfile, "%lf ", msg->quat_data.m_data[2]);
	carmen_fprintf(outfile, "%lf ", msg->quat_data.m_data[3]);

	carmen_fprintf(outfile, "%lf ", msg->m_mag.x);
	carmen_fprintf(outfile, "%lf ", msg->m_mag.y);
	carmen_fprintf(outfile, "%lf ", msg->m_mag.z);

	carmen_fprintf(outfile, "%lf ", msg->m_gyr.x);
	carmen_fprintf(outfile, "%lf ", msg->m_gyr.y);
	carmen_fprintf(outfile, "%lf ", msg->m_gyr.z);

	carmen_fprintf(outfile, "%lf %hu %lf %s %lf\n", msg->m_temp, msg->m_count,
			msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_pi_imu(carmen_pi_imu_message_t* msg,
			       carmen_FILE *outfile,
			       double timestamp)
{
	carmen_fprintf(outfile, "PI_IMU ");
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.accel.x);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.accel.y);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.accel.z);

	carmen_fprintf(outfile, "%lf ", msg->imu_vector.magnetometer.x);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.magnetometer.y);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.magnetometer.z);

	carmen_fprintf(outfile, "%lf ", msg->imu_vector.gyro.x);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.gyro.y);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.gyro.z);

	carmen_fprintf(outfile, "%lf ", msg->imu_vector.quat_data.m_data[0]);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.quat_data.m_data[1]);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.quat_data.m_data[2]);
	carmen_fprintf(outfile, "%lf ", msg->imu_vector.quat_data.m_data[3]);


	carmen_fprintf(outfile, "%lf %s %lf\n", msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_xsens_matrix(carmen_xsens_global_matrix_message* msg,
		carmen_FILE *outfile, double timestamp)
{
	carmen_fprintf(outfile, "XSENS_MATRIX ");
	carmen_fprintf(outfile, "%lf ", msg->m_acc.x);
	carmen_fprintf(outfile, "%lf ", msg->m_acc.y);
	carmen_fprintf(outfile, "%lf ", msg->m_acc.z);

	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[0][0]);
	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[0][1]);
	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[0][2]);

	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[1][0]);
	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[1][1]);
	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[1][2]);

	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[2][0]);
	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[2][1]);
	carmen_fprintf(outfile, "%lf ", msg->matrix_data.m_data[2][2]);

	carmen_fprintf(outfile, "%lf ", msg->m_mag.x);
	carmen_fprintf(outfile, "%lf ", msg->m_mag.y);
	carmen_fprintf(outfile, "%lf ", msg->m_mag.z);

	carmen_fprintf(outfile, "%lf ", msg->m_gyr.x);
	carmen_fprintf(outfile, "%lf ", msg->m_gyr.y);
	carmen_fprintf(outfile, "%lf ", msg->m_gyr.z);

	carmen_fprintf(outfile, "%lf %hu %lf %s %lf\n", msg->m_temp, msg->m_count,
			msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_xsens_mtig(carmen_xsens_mtig_message* msg,
		carmen_FILE *outfile, double timestamp)
{
	carmen_fprintf(outfile, "XSENS_MTIG ");

	carmen_fprintf(outfile, "%lf ", msg->quat.q0);
	carmen_fprintf(outfile, "%lf ", msg->quat.q1);
	carmen_fprintf(outfile, "%lf ", msg->quat.q2);
	carmen_fprintf(outfile, "%lf ", msg->quat.q3);

	carmen_fprintf(outfile, "%lf ", msg->acc.x);
	carmen_fprintf(outfile, "%lf ", msg->acc.y);
	carmen_fprintf(outfile, "%lf ", msg->acc.z);

	carmen_fprintf(outfile, "%lf ", msg->gyr.x);
	carmen_fprintf(outfile, "%lf ", msg->gyr.y);
	carmen_fprintf(outfile, "%lf ", msg->gyr.z);

	carmen_fprintf(outfile, "%lf ", msg->mag.x);
	carmen_fprintf(outfile, "%lf ", msg->mag.y);
	carmen_fprintf(outfile, "%lf ", msg->mag.z);

	carmen_fprintf(outfile, "%lf ", msg->velocity.x);
	carmen_fprintf(outfile, "%lf ", msg->velocity.y);
	carmen_fprintf(outfile, "%lf ", msg->velocity.z);

	carmen_fprintf(outfile, "%lf ", msg->latitude);
	carmen_fprintf(outfile, "%lf ", msg->longitude);
	carmen_fprintf(outfile, "%lf ", msg->height);

	carmen_fprintf(outfile, "%d ", msg->gps_fix);
	carmen_fprintf(outfile, "%d ", msg->xkf_valid);
	carmen_fprintf(outfile, "%d ", msg->sensor_ID);

	carmen_fprintf(outfile, "%lf %s %lf\n", msg->timestamp, msg->host,
			timestamp);
}

void carmen_logwrite_write_robot_ackerman_vector_move(
		carmen_robot_ackerman_vector_move_message *msg, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile, "VECTORMOVE_ACK %f %f %f %s %f\n", msg->distance,
			msg->theta, msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_ford_escape_status_message(
		carmen_ford_escape_status_message *msg, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile,
			"FORD_ESCAPE_STATUS %lf %lf %lf %u %d %d %d %d %d %d %d %lf %s %lf\n",
			msg->g_XGV_throttle, msg->g_XGV_steering, msg->g_XGV_brakes,
			msg->g_XGV_component_status, msg->g_XGV_main_propulsion,
			msg->g_XGV_main_fuel_supply, msg->g_XGV_parking_brake,
			msg->g_XGV_gear, msg->g_XGV_turn_signal, msg->g_XGV_horn_status,
			msg->g_XGV_headlights_status, msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_carmen_can_dump_can_line_message(
		carmen_can_dump_can_line_message *msg, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile,
			"CAN_DUMP_CAN_LINE_MESSADE %s %lf %s %lf\n",
			msg->can_line, msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_robot_ackerman_velocity(
		carmen_robot_ackerman_velocity_message *msg, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile, "ROBOTVELOCITY_ACK %f %f %f %s %f\n", msg->v,
			msg->phi, msg->timestamp, msg->host, timestamp);

}

void carmen_logwrite_write_robot_ackerman_follow_trajectory(
		carmen_robot_ackerman_follow_trajectory_message *msg,
		carmen_FILE *outfile, double timestamp)
{
	carmen_fprintf(outfile, "FOLLOWTRAJECTORY_ACK %lf %lf %lf %lf %lf ",
			msg->robot_position.x, msg->robot_position.y,
			msg->robot_position.theta, msg->robot_position.v,
			msg->robot_position.phi);

	carmen_fprintf(outfile, "%d ", msg->trajectory_length);

	int i;
	for (i = 0; i < msg->trajectory_length; i++)
		carmen_fprintf(outfile, "%lf %lf %lf %lf %lf ", msg->trajectory[i].x,
				msg->trajectory[i].y, msg->trajectory[i].theta,
				msg->trajectory[i].v, msg->trajectory[i].phi);

	carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, timestamp);

}

void carmen_logwrite_write_base_ackerman_velocity(
		carmen_base_ackerman_velocity_message *msg, carmen_FILE *outfile,
		double timestamp)
{
	carmen_fprintf(outfile, "BASEVELOCITY_ACK %f %f %f %s %f\n", msg->v,
			msg->phi, msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_base_ackerman_motion(
		carmen_base_ackerman_motion_command_message *msg, carmen_FILE *outfile,
		double timestamp)
{
	int i;

	carmen_fprintf(outfile, "BASEMOTION_ACK %d", msg->num_motion_commands);
	for (i = 0; i < msg->num_motion_commands; i++)
		carmen_fprintf(outfile, " %f %f %f", msg->motion_command[i].v,
				msg->motion_command[i].phi, msg->motion_command[i].time);

	carmen_fprintf(outfile, " %f %s %f\n", msg->timestamp, msg->host,
			timestamp);
}

void carmen_logwrite_write_logger_comment(carmen_logger_comment_message *msg,
		carmen_FILE *outfile, double timestamp)
{
	unsigned int l = strlen(msg->text);
	char buffer[l + 1];
	strcpy(buffer, msg->text);
	unsigned int x;
	for (x = 0; x <= l; x++)
	{
		if (msg->text[x] == ' ')
			msg->text[x] = '_';
	}
	carmen_fprintf(outfile, "COMMENT %s %f %s %f\n", msg->text, msg->timestamp,
			msg->host, timestamp);
}

void carmen_logwrite_write_kinect_depth(carmen_kinect_depth_message *kinect,
		int kinect_num, carmen_FILE *outfile, double timestamp)
{
	int i, j;
	unsigned short depth_value;

	carmen_fprintf(outfile, "RAW_KINECT_DEPTH%d ", kinect_num);
	carmen_fprintf(outfile, "%d ", kinect->width);
	carmen_fprintf(outfile, "%d ", kinect->height);
	carmen_fprintf(outfile, "%d ", kinect->size);

	if (hex_char_depth_kinect == NULL)
	{
		hex_char_depth_kinect = (char *) malloc(
				(4 * kinect->size) * sizeof(char));
		for (i = 0; i < 16; i++)
		{
			if (i <= 9)
				int_to_nibble_hex[i] = '0' + i;
			else
				int_to_nibble_hex[i] = 'a' + i - 10;
		}
	}

	for (i = j = 0; i < (kinect->size); i++, j += 4)
	{
		depth_value = convert_kinect_depth_meters_to_raw(kinect->depth[i]);

		hex_char_depth_kinect[j] = GET_SHORT_FIRST_NIBBLE(depth_value);
		hex_char_depth_kinect[j + 1] = GET_SHORT_SECOND_NIBBLE(depth_value);
		hex_char_depth_kinect[j + 2] = GET_SHORT_THIRD_NIBBLE(depth_value);
		hex_char_depth_kinect[j + 3] = GET_SHORT_FOURTH_NIBBLE(depth_value);
	}

	carmen_fwrite(hex_char_depth_kinect, (4 * kinect->size), 1, outfile);
	carmen_fprintf(outfile, "%f %s %f\n", kinect->timestamp, kinect->host,
			timestamp);
}

void carmen_logwrite_write_kinect_video(carmen_kinect_video_message *kinect,
		int kinect_num, carmen_FILE *outfile, double timestamp)
{
	int i, j;

	carmen_fprintf(outfile, "RAW_KINECT_VIDEO%d ", kinect_num);
	carmen_fprintf(outfile, "%d ", kinect->width);
	carmen_fprintf(outfile, "%d ", kinect->height);
	carmen_fprintf(outfile, "%d ", kinect->size);

	if (hex_char_image_kinect == NULL)
	{

		hex_char_image_kinect = (char *) malloc(
				(2 * kinect->size) * sizeof(char)); // Twice the number of bytes

		for (i = 0; i < 16; i++)
		{
			if (i <= 9)
				int_to_nibble_hex[i] = '0' + i;
			else
				int_to_nibble_hex[i] = 'a' + i - 10;
		}
	}

	for (i = j = 0; i < (kinect->size); i++, j += 2)
	{
		hex_char_image_kinect[j] = GET_HIGH_ORDER_NIBBLE(kinect->video[i]);
		hex_char_image_kinect[j + 1] = GET_LOW_ORDER_NIBBLE(kinect->video[i]);
	}

	carmen_fwrite(hex_char_image_kinect, (2 * kinect->size), 1, outfile);
	carmen_fprintf(outfile, "%f %s %f\n", kinect->timestamp, kinect->host,
			timestamp);
}

char *hex_char_distance_and_intensity;

void carmen_logwrite_write_velodyne_partial_scan(
		carmen_velodyne_partial_scan_message* msg, carmen_FILE* outfile,
		double timestamp)
{
	int i, j, k, angle;

	carmen_fprintf(outfile, "VELODYNE_PARTIAL_SCAN ");
	carmen_fprintf(outfile, "%d ", msg->number_of_32_laser_shots);

	if (hex_char_distance_and_intensity == NULL)
	{
		hex_char_distance_and_intensity = (char *) malloc(
				(64 + 128 + 1) * sizeof(char)); // 2 * 32 laser intensities and 4 * 32 laser distances
		for (i = 0; i < 16; i++)
		{
			if (i <= 9)
				int_to_nibble_hex[i] = '0' + i;
			else
				int_to_nibble_hex[i] = 'a' + i - 10;
		}
	}

	for (i = 0; i < msg->number_of_32_laser_shots; i++)
	{
		angle = (int) (msg->partial_scan[i].angle * 100);
		carmen_fprintf(outfile, "%d ", angle);

		for (j = k = 0; j < 32; j += 1, k += 6)
		{
			hex_char_distance_and_intensity[k] = GET_SHORT_FIRST_NIBBLE(
					msg->partial_scan[i].distance[j]);
			hex_char_distance_and_intensity[k + 1] = GET_SHORT_SECOND_NIBBLE(
					msg->partial_scan[i].distance[j]);
			hex_char_distance_and_intensity[k + 2] = GET_SHORT_THIRD_NIBBLE(
					msg->partial_scan[i].distance[j]);
			hex_char_distance_and_intensity[k + 3] = GET_SHORT_FOURTH_NIBBLE(
					msg->partial_scan[i].distance[j]);

			hex_char_distance_and_intensity[k + 4] = GET_LOW_ORDER_NIBBLE(
					msg->partial_scan[i].intensity[j]);
			hex_char_distance_and_intensity[k + 5] = GET_HIGH_ORDER_NIBBLE(
					msg->partial_scan[i].intensity[j]);
		}

		hex_char_distance_and_intensity[k] = ' ';

		carmen_fwrite(hex_char_distance_and_intensity, (64 + 128 + 1), 1,
				outfile);
	}

	carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_to_file_velodyne(
		carmen_velodyne_partial_scan_message* msg, carmen_FILE *outfile,
		double timestamp, char *log_filename)
{
	const double HIGH_LEVEL_SUBDIR_TIME = 100.0 * 100.0; // new each 100 x 100 seconds
	const double LOW_LEVEL_SUBDIR_TIME = 100.0; // new each 100 seconds

	int high_level_subdir = ((int) (msg->timestamp / HIGH_LEVEL_SUBDIR_TIME))
			* HIGH_LEVEL_SUBDIR_TIME;
	int low_level_subdir = ((int) (msg->timestamp / LOW_LEVEL_SUBDIR_TIME))
			* LOW_LEVEL_SUBDIR_TIME;

	int i;
	static char directory[1024];
	static char subdir[1024];
	static char path[1024];

	/**
	 * TODO: @Filipe: Check if the mkdir call is time consuming.
	 */
	sprintf(directory, "%s_velodyne", log_filename);
	mkdir(directory, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(subdir, "%s/%d", directory, high_level_subdir);
	mkdir(subdir, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(subdir, "%s/%d/%d", directory, high_level_subdir, low_level_subdir);
	mkdir(subdir, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(path, "%s/%lf.pointcloud", subdir, msg->timestamp);

	FILE *image_file = fopen(path, "wb");

	for (i = 0; i < msg->number_of_32_laser_shots; i++)
	{
		fwrite(&(msg->partial_scan[i].angle), sizeof(double), 1, image_file);
		fwrite(msg->partial_scan[i].distance, sizeof(short), 32, image_file);
		fwrite(msg->partial_scan[i].intensity, sizeof(char), 32, image_file);
	}

	fclose(image_file);

	carmen_fprintf(outfile, "VELODYNE_PARTIAL_SCAN_IN_FILE %s %d ", path,
			msg->number_of_32_laser_shots);
	carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_to_file_velodyne_variable(
		carmen_velodyne_variable_scan_message* msg, int velodyne_number, carmen_FILE *outfile,
		double timestamp, char *log_filename)
{
	const double HIGH_LEVEL_SUBDIR_TIME = 100.0 * 100.0; // new each 100 x 100 seconds
	const double LOW_LEVEL_SUBDIR_TIME = 100.0; // new each 100 seconds

	int high_level_subdir = ((int) (msg->timestamp / HIGH_LEVEL_SUBDIR_TIME))
			* HIGH_LEVEL_SUBDIR_TIME;
	int low_level_subdir = ((int) (msg->timestamp / LOW_LEVEL_SUBDIR_TIME))
			* LOW_LEVEL_SUBDIR_TIME;

	int i;
	static char directory[1024];
	static char subdir[1024];
	static char path[1024];

	/**
	 * TODO: @Filipe: Check if the mkdir call is time consuming.
	 */
	sprintf(directory, "%s_velodyne%d", log_filename, velodyne_number);
	mkdir(directory, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(subdir, "%s/%d", directory, high_level_subdir);
	mkdir(subdir, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(subdir, "%s/%d/%d", directory, high_level_subdir, low_level_subdir);
	mkdir(subdir, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(path, "%s/%lf.pointcloud", subdir, msg->timestamp);

	FILE *image_file = fopen(path, "wb");

	for (i = 0; i < msg->number_of_shots; i++)
	{
		fwrite(&(msg->partial_scan[i].angle), sizeof(double), 1, image_file);
		fwrite(msg->partial_scan[i].distance, sizeof(short), msg->partial_scan->shot_size, image_file);
		fwrite(msg->partial_scan[i].intensity, sizeof(char), msg->partial_scan->shot_size, image_file);
	}

	fclose(image_file);

	//novo arquivo de log do velodyne
	//para salvar o .xyz do velodyne ao mesmo tempo que gera o log
	/*
	static char path2[1024];

	sprintf(path2, "%s/%lf.xyz", subdir, msg->timestamp);

	std::ofstream o(path2);

	static double vertical_correction[32];
	static double vc_16[32] = {-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static double vc_32[32] = {-30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0,
			-20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67};
	if(msg->partial_scan->shot_size == 16)
		memcpy(vertical_correction, vc_16, sizeof(vc_16));
	else if (msg->partial_scan->shot_size == 32)
		memcpy(vertical_correction, vc_32, sizeof(vc_32));

	for (int i = 0; i < msg->number_of_shots; i++)
	{
		for (int j = 0; j < msg->partial_scan->shot_size; j++)
		{
			double v_angle = (carmen_degrees_to_radians(vertical_correction[j]));

			double range = (double) msg->partial_scan[i].distance[j];

			double h_angle = (carmen_degrees_to_radians(msg->partial_scan[i].angle));

			double x, y, z;
			x = range * cos(v_angle) * cos(h_angle);
			y = range * cos(v_angle) * sin(h_angle);
			z = range * sin(v_angle);

			o << x << "\t" << y << "\t" << z << std::endl;
		}
		o << std::endl;
	}

	o.close();
	*/
	//fim do novo arquivo

	carmen_fprintf(outfile, "VELODYNE_VARIABLE_SCAN_IN_FILE%d %s %d %d %d ", velodyne_number, path,
			velodyne_number, msg->partial_scan->shot_size, msg->number_of_shots);
	carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, timestamp);
}

char *hex_char_distance_and_intensity_variable;

void carmen_logwrite_write_variable_velodyne_scan(
		carmen_velodyne_variable_scan_message* msg, int velodyne_number, carmen_FILE* outfile,
		double timestamp)
{
	int i, j, k, angle;

	carmen_fprintf(outfile, "VARIABLE_VELODYNE_SCAN%d ", velodyne_number);
	carmen_fprintf(outfile, "%d ", msg->number_of_shots);
	carmen_fprintf(outfile, "%d ", msg->partial_scan[0].shot_size);

	int vector_size = (2 * msg->partial_scan[0].shot_size
			+ 4 * msg->partial_scan[0].shot_size + 1);

	if (hex_char_distance_and_intensity_variable == NULL)
	{
		hex_char_distance_and_intensity_variable = (char *) malloc(
				vector_size * sizeof(char)); // 2 * 32 laser intensities and 4 * 32 laser distances

		for (i = 0; i < 16; i++)
		{
			if (i <= 9)
				int_to_nibble_hex[i] = '0' + i;
			else
				int_to_nibble_hex[i] = 'a' + i - 10;
		}
	}

	for (i = 0; i < msg->number_of_shots; i++)
	{
		angle = (int) (msg->partial_scan[i].angle * 100);
		carmen_fprintf(outfile, "%d ", angle);

		for (j = k = 0; j < msg->partial_scan[0].shot_size; j += 1, k += 6)
		{
			hex_char_distance_and_intensity_variable[k] =
					GET_SHORT_FIRST_NIBBLE(msg->partial_scan[i].distance[j]);
			hex_char_distance_and_intensity_variable[k + 1] =
					GET_SHORT_SECOND_NIBBLE(msg->partial_scan[i].distance[j]);
			hex_char_distance_and_intensity_variable[k + 2] =
					GET_SHORT_THIRD_NIBBLE(msg->partial_scan[i].distance[j]);
			hex_char_distance_and_intensity_variable[k + 3] =
					GET_SHORT_FOURTH_NIBBLE(msg->partial_scan[i].distance[j]);

			hex_char_distance_and_intensity_variable[k + 4] =
					GET_LOW_ORDER_NIBBLE(msg->partial_scan[i].intensity[j]);
			hex_char_distance_and_intensity_variable[k + 5] =
					GET_HIGH_ORDER_NIBBLE(msg->partial_scan[i].intensity[j]);
		}

		hex_char_distance_and_intensity_variable[k] = ' ';

		carmen_fwrite(hex_char_distance_and_intensity_variable, vector_size, 1,
				outfile);
	}

	carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_velodyne_gps(carmen_velodyne_gps_message* msg,
		carmen_FILE* outfile, double timestamp)
{
	carmen_fprintf(outfile, "VELODYNE_GPS ");

	carmen_fprintf(outfile, "%f ", msg->gyro1);
	carmen_fprintf(outfile, "%f ", msg->gyro2);
	carmen_fprintf(outfile, "%f ", msg->gyro3);

	carmen_fprintf(outfile, "%f ", msg->temp1);
	carmen_fprintf(outfile, "%f ", msg->temp2);
	carmen_fprintf(outfile, "%f ", msg->temp3);

	carmen_fprintf(outfile, "%f ", msg->accel1_x);
	carmen_fprintf(outfile, "%f ", msg->accel2_x);
	carmen_fprintf(outfile, "%f ", msg->accel3_x);

	carmen_fprintf(outfile, "%f ", msg->accel1_y);
	carmen_fprintf(outfile, "%f ", msg->accel2_y);
	carmen_fprintf(outfile, "%f ", msg->accel3_y);

	carmen_fprintf(outfile, "%d ", msg->utc_time);
	carmen_fprintf(outfile, "%c ", msg->status);
	carmen_fprintf(outfile, "%f ", msg->latitude);
	carmen_fprintf(outfile, "%c ", msg->latitude_hemisphere);
	carmen_fprintf(outfile, "%f ", msg->longitude);
	carmen_fprintf(outfile, "%c ", msg->longitude_hemisphere);
	carmen_fprintf(outfile, "%f ", msg->speed_over_ground);
	carmen_fprintf(outfile, "%f ", msg->course_over_ground);
	carmen_fprintf(outfile, "%d ", msg->utc_date);
	carmen_fprintf(outfile, "%f ", msg->magnetic_variation_course);
	carmen_fprintf(outfile, "%f ", msg->magnetic_variation_direction);
	carmen_fprintf(outfile, "%c ", msg->mode_indication);

	carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_bumblebee_basic_steroimage_old(
		carmen_bumblebee_basic_stereoimage_message* msg, int bumblebee_num,
		carmen_FILE *outfile, double timestamp)
{
	int i;

	carmen_fprintf(outfile, "BUMBLEBEE_BASIC_STEREOIMAGE%d ", bumblebee_num);
	carmen_fprintf(outfile, "%d ", msg->width);
	carmen_fprintf(outfile, "%d ", msg->height);
	carmen_fprintf(outfile, "%d ", msg->image_size);
	carmen_fprintf(outfile, "%d ", msg->isRectified);

	for (i = 0; i < (msg->image_size); i++)
		carmen_fprintf(outfile, "%d ", (int) msg->raw_right[i]);

	for (i = 0; i < (msg->image_size); i++)
		carmen_fprintf(outfile, "%d ", (int) msg->raw_left[i]);

	carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_bumblebee_basic_steroimage(
		carmen_bumblebee_basic_stereoimage_message* msg, int bumblebee_num,
		carmen_FILE *outfile, double timestamp, int frequency)
{

	int i, j;

	if ((frame_number % frequency) == 0)
	{
		if (hex_char_image == NULL)
		{
			hex_char_image = (char *) malloc(
					(2 * msg->image_size + 1) * sizeof(char)); // Twice the number of bytes plus 1 for a space at the end
			for (i = 0; i < 16; i++)
			{
				if (i <= 9)
					int_to_nibble_hex[i] = '0' + i;
				else
					int_to_nibble_hex[i] = 'a' + i - 10;
			}
		}

		carmen_fprintf(outfile, "BUMBLEBEE_BASIC_STEREOIMAGE%d ",
				bumblebee_num);
		carmen_fprintf(outfile, "%d ", msg->width);
		carmen_fprintf(outfile, "%d ", msg->height);
		carmen_fprintf(outfile, "%d ", msg->image_size);
		carmen_fprintf(outfile, "%d ", msg->isRectified);

		for (i = j = 0; i < (msg->image_size); i++, j += 2)
		{
			hex_char_image[j] = GET_HIGH_ORDER_NIBBLE(msg->raw_right[i]);
			hex_char_image[j + 1] = GET_LOW_ORDER_NIBBLE(msg->raw_right[i]);
		}
		hex_char_image[j] = ' ';
		carmen_fwrite(hex_char_image, 2 * msg->image_size + 1, 1, outfile);

		for (i = j = 0; i < (msg->image_size); i++, j += 2)
		{
			hex_char_image[j] = GET_HIGH_ORDER_NIBBLE(msg->raw_left[i]);
			hex_char_image[j + 1] = GET_LOW_ORDER_NIBBLE(msg->raw_left[i]);
		}
		hex_char_image[j] = ' ';
		carmen_fwrite(hex_char_image, 2 * msg->image_size + 1, 1, outfile);

		carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host,
				timestamp);
		frame_number = 0;
	}
	frame_number++;
}

unsigned char* read_raw_image(const char* filename)
{
    //int i;
    FILE* f = fopen(filename, "rb");
    //unsigned char info[54];
    //fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

    // extract image height and width from header
    int width = 1280;//*(int*)&info[18];
    int height = 960*2; //*(int*)&info[22];

    int size = 3 * width * height;
    unsigned char* data = new unsigned char[size]; // allocate 3 bytes per pixel
    fread(data, sizeof(unsigned char), size, f); // read the rest of the data at once
    fclose(f);

    /*for(i = 0; i < size; i += 3)
    {
            unsigned char tmp = data[i];
            data[i] = data[i+2];
            data[i+2] = tmp;
    }*/

    return data;
}

void carmen_logwrite_write_to_file_bumblebee_basic_steroimage(
		carmen_bumblebee_basic_stereoimage_message* msg, int bumblebee_num,
		carmen_FILE *outfile, double timestamp, int frequency, char *log_filename)
{
	const double HIGH_LEVEL_SUBDIR_TIME = 100.0 * 100.0; // new each 100 x 100 seconds
	const double LOW_LEVEL_SUBDIR_TIME = 100.0; // new each 100 seconds

	int high_level_subdir = ((int) (msg->timestamp / HIGH_LEVEL_SUBDIR_TIME))
			* HIGH_LEVEL_SUBDIR_TIME;
	int low_level_subdir = ((int) (msg->timestamp / LOW_LEVEL_SUBDIR_TIME))
			* LOW_LEVEL_SUBDIR_TIME;

	static char directory[1024];
	static char subdir[1024];
	static char path[1024];

	/**
	 * TODO: @Filipe: Check if the mkdir call is time consuming.
	 */
	sprintf(directory, "%s_bumblebee", log_filename);
	mkdir(directory, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(subdir, "%s/%d", directory, high_level_subdir);
	mkdir(subdir, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(subdir, "%s/%d/%d", directory, high_level_subdir, low_level_subdir);
	mkdir(subdir, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	// DEBUG:
	//printf("%lf %d %d %s\n", msg->timestamp, high_level_subdir, low_level_subdir, subdir);

	if ((frame_number % frequency) == 0)
	{
		if (0)//(bumblebee_num == 4) // ZED Camera
		{
//			  int width;                    /**<The x dimension of the image in pixels. */
//			  int height;                   /**<The y dimension of the image in pixels. */
//			  int image_size;              /**<width*height*bytes_per_pixel. */
//			  int isRectified;
//			  unsigned char *raw_left;
//			  unsigned char *raw_right;
//			  double timestamp;
//			  char *host;


			sprintf(path, "%s/%lf.bb%d.png", subdir, msg->timestamp, bumblebee_num);
			static cv::Mat dest;

			cv::Mat left = cv::Mat(cv::Size(msg->width, msg->height), CV_8UC3, msg->raw_left);
			cv::Mat right = cv::Mat(cv::Size(msg->width, msg->height), CV_8UC3, msg->raw_right);

			cv::hconcat(left, right, dest);
			cv::imwrite(path, dest);

		}
		else
		{
			sprintf(path, "%s/%lf.bb%d.image", subdir, msg->timestamp,
					bumblebee_num);

			FILE *image_file = fopen(path, "wb");

			fwrite(msg->raw_left, msg->image_size, sizeof(unsigned char),
					image_file);
			fwrite(msg->raw_right, msg->image_size, sizeof(unsigned char),
					image_file);

			fclose(image_file);
		}

		carmen_fprintf(outfile,
				"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE%d %s %d %d %d %d ",
				bumblebee_num, path, msg->width, msg->height, msg->image_size,
				msg->isRectified);

		carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host,
				timestamp);

		frame_number = 0;
	}

	frame_number++;
}


void carmen_logwrite_write_web_cam_message(carmen_web_cam_message* msg,
		carmen_FILE *outfile, double timestamp)
{
	int i;

	carmen_fprintf(outfile, "WEB_CAM_IMAGE ");
	carmen_fprintf(outfile, "%d ", msg->width);
	carmen_fprintf(outfile, "%d ", msg->height);
	carmen_fprintf(outfile, "%d ", msg->image_size);

	for (i = 0; i < (msg->image_size); i++)
	{
		carmen_fprintf(outfile, "%d ", msg->img_data[i]);
	}

	carmen_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, timestamp);
}
