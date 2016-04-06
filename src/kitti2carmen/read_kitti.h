
#ifndef SRC_KITTI2CARMEN_READ_KITTI_H_
#define SRC_KITTI2CARMEN_READ_KITTI_H_

#include <carmen/velodyne_messages.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/stereo_velodyne_messages.h>
#include <carmen/gps_nmea_messages.h>
#include <carmen/xsens_messages.h>

carmen_velodyne_variable_scan_message
read_velodyne(char *dir, int file_id, double timestamp);

carmen_bumblebee_basic_stereoimage_message
read_camera(char *dir_left, char *dir_right, int file_id, double timestamp);

void
read_gps(char *dir_gps, int file_id, double timestamp, carmen_gps_gpgga_message *gps_msg,
		carmen_xsens_global_quat_message *xsens_msg);

#endif
