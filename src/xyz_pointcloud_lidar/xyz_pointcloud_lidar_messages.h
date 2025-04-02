#ifndef XYZ_POINTCLOUD_LIDAR_MESSAGES_H_
#define XYZ_POINTCLOUD_LIDAR_MESSAGES_H_

#include "carmen/global.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct carmen_xyz_pointcloud_lidar_message {
    int num_of_points;
    carmen_vector_3D_t *points;
    double time_spent_by_each_point;
    double timestamp;
    char* host;
} carmen_xyz_pointcloud_lidar_message;

typedef struct
{
	char *model;
	int id;
	char *ip;
	char *port;
	double max_range;               // Maximum sensing distance in metters
	double time_between_points;     // Given by the spinning frequency
	carmen_pose_3D_t pose;          // x, y, z, roll, pitch, yaw
	int sensor_reference;			// If it is in reference in relation to the sensorboard (0), to the front_bullbar (1), or the rear_bullbar (2)
}carmen_xyz_lidar_config;

#define		CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME		"carmen_xyz_pointcloud_lidar"
#define		CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_FMT			"{int,<{double,double,double}:1>,double,double,string}"

#define      CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE0_NAME       "carmen_xyz_pointcloud_lidar0"
#define      CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE1_NAME       "carmen_xyz_pointcloud_lidar1"
#define      CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE2_NAME       "carmen_xyz_pointcloud_lidar2"
#define      CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE3_NAME       "carmen_xyz_pointcloud_lidar3"

#ifdef __cplusplus
}
#endif

#endif
