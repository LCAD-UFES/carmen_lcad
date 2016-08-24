/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef CARMEN_ZED_CAMERA_SENSOR_MESSAGES_H
#define CARMEN_ZED_CAMERA_SENSOR_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int width;
    int height;
    int image_size;
    int isRectified;
    unsigned char *raw_left;
    unsigned char *raw_right;
    double timestamp;
    char *host;
} carmen_zed_camera_sensor_stereoimage_message;

typedef struct {
    int width;
    int height;
    int image_size;
    unsigned char *raw_image;
    double timestamp;
    char *host;
} carmen_zed_camera_sensor_depthmap_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_NAME       "carmen_zed_camera_sensor_stereoimage"
#define      CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_NAME       "carmen_zed_camera_sensor_depthmap"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_FMT        "{int,int,int,int,<ubyte:3>,<ubyte:3>,double,string}"
#define      CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_FMT        "{int,int,int,<ubyte:3>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
