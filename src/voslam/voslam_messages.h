#ifndef CARMEN_VOSLAM_MESSAGES_H
#define CARMEN_VOSLAM_MESSAGES_H

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int is_keyframe;
	int stereo_size;
	int is_loop_closure;
	int loop_partner_index;
	double position[3];
	double rotation[9];
	double position_gl[3];
	double rotation_gl[9];
	unsigned short* depth;
	unsigned char* image;
	double timestamp;
	char* host;
} carmen_voslam_pointcloud_message;

#define      CARMEN_VOSLAM_POINTCLOUD_MESSAGE_NAME       "carmen_voslam_pointcloud_message"
#define      CARMEN_VOSLAM_POINTCLOUD_MESSAGE_FMT        "{int, int, int, int, [double:3], [double:9], [double:3], [double:9], <ushort:2>, <ubyte:2>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
