#ifndef CARMEN_KINECT_MESSAGES_H
#define CARMEN_KINECT_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int id;
	int width;
	int height;
	int size;
	float *depth;
	double timestamp;
	char *host;
} carmen_kinect_depth_message;

#define CARMEN_KINECT_DEPTH_FMT  "{int,int,int,int,<float:4>,double,string}"

#define CARMEN_KINECT_DEPTH_MSG_0_FMT	CARMEN_KINECT_DEPTH_FMT
#define CARMEN_KINECT_DEPTH_MSG_1_FMT	CARMEN_KINECT_DEPTH_FMT

#define CARMEN_KINECT_DEPTH_MSG_0_NAME "carmen_kinect_depth_msg_0"
#define CARMEN_KINECT_DEPTH_MSG_1_NAME "carmen_kinect_depth_msg_1"

typedef struct {
    int id;
	int width;
	int height;
	int size;
	unsigned char* video;
	double timestamp;
	char *host;
} carmen_kinect_video_message;

#define CARMEN_KINECT_VIDEO_FMT  "{int,int,int,int,<ubyte:4>,double,string}"

#define CARMEN_KINECT_VIDEO_MSG_0_FMT	CARMEN_KINECT_VIDEO_FMT
#define CARMEN_KINECT_VIDEO_MSG_1_FMT	CARMEN_KINECT_VIDEO_FMT

#define CARMEN_KINECT_VIDEO_MSG_0_NAME "carmen_kinect_video_msg_0"
#define CARMEN_KINECT_VIDEO_MSG_1_NAME "carmen_kinect_video_msg_1"

typedef struct {
	int id;
	int width;
	int height;
	int size;
	short depth[307200];
	double timestamp;
	char *host;
} carmen_kinect_depth_mrds_message;

#define CARMEN_KINECT_DEPTH_MRDS_FMT  "{int,int,int,int,[short:307200],double,string}"

#define CARMEN_KINECT_DEPTH_MRDS_MSG_0_FMT	CARMEN_KINECT_DEPTH_MRDS_FMT
#define CARMEN_KINECT_DEPTH_MRDS_MSG_1_FMT	CARMEN_KINECT_DEPTH_MRDS_FMT

#define CARMEN_KINECT_DEPTH_MRDS_MSG_0_NAME "carmen_kinect_depth_mrds_msg_0"
#define CARMEN_KINECT_DEPTH_MRDS_MSG_1_NAME "carmen_kinect_depth_mrds_msg_1"

typedef struct {
    int id;
	int width;
	int height;
	int size;
	unsigned char video[921600];
	double timestamp;
	char *host;
} carmen_kinect_video_mrds_message;

#define CARMEN_KINECT_VIDEO_MRDS_FMT  "{int,int,int,int,[ubyte:921600],double,string}"

#define CARMEN_KINECT_VIDEO_MRDS_MSG_0_FMT	CARMEN_KINECT_VIDEO_MRDS_FMT
#define CARMEN_KINECT_VIDEO_MRDS_MSG_1_FMT	CARMEN_KINECT_VIDEO_MRDS_FMT

#define CARMEN_KINECT_VIDEO_MRDS_MSG_0_NAME "carmen_kinect_video_mrds_msg_0"
#define CARMEN_KINECT_VIDEO_MRDS_MSG_1_NAME "carmen_kinect_video_mrds_msg_1"

#ifdef __cplusplus
}
#endif

#endif
