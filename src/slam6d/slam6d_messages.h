#ifndef CARMEN_SLAM6D_MESSAGES_H
#define CARMEN_SLAM6D_MESSAGES_H

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int width;
	int height;
	int is_keyframe;
	int depth_size;
	int image_size;
	double position[3];
	double rotation[9];
	unsigned short *depth;
	unsigned char* image;
	double timestamp;
	char* host;
} carmen_slam6d_pointcloud_message;

#define      CARMEN_SLAM6D_POINTCLOUD_MESSAGE_NAME       "carmen_slam6d_pointcloud"
#define      CARMEN_SLAM6D_POINTCLOUD_MESSAGE_FMT        "{int, int, int, int, int, [double:3], [double:9], <ushort:4>, <ubyte:5>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
