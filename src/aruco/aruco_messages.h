#include <carmen/carmen.h>
#ifndef CARMEN_ARUCO_POSE_H
#define CARMEN_ARUCO_POSE_H

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct
{
    int n_markers_detected;
    int *ids_markers_detected;
    double rvec[3];
    double tvec[3];
} carmen_aruco_pose;

typedef struct
{
	int n_poses;
    carmen_aruco_pose *poses;
	double timestamp;
	char *host;
} carmen_aruco_message;

#define      CARMEN_ARUCO1_NAME          "carmen_aruco_message1"
#define      CARMEN_ARUCO2_NAME          "carmen_aruco_message2"
#define      CARMEN_ARUCO3_NAME          "carmen_aruco_message3"
#define      CARMEN_ARUCO4_NAME          "carmen_aruco_message4"
#define      CARMEN_ARUCO5_NAME          "carmen_aruco_message5"
#define      CARMEN_ARUCO6_NAME          "carmen_aruco_message6"
#define      CARMEN_ARUCO7_NAME          "carmen_aruco_message7"
#define      CARMEN_ARUCO8_NAME          "carmen_aruco_message8"
#define      CARMEN_ARUCO9_NAME          "carmen_aruco_message9"
#define      CARMEN_ARUCO10_NAME         "carmen_aruco_message10"
#define      CARMEN_ARUCO11_NAME         "carmen_aruco_message11"
#define      CARMEN_ARUCO12_NAME         "carmen_aruco_message12"
#define      CARMEN_ARUCO13_NAME         "carmen_aruco_message13"
#define      CARMEN_ARUCO14_NAME         "carmen_aruco_message14"
#define      CARMEN_ARUCO15_NAME         "carmen_aruco_message15"
#define      CARMEN_ARUCO16_NAME         "carmen_aruco_message16"
#define      CARMEN_ARUCO17_NAME         "carmen_aruco_message17"
#define      CARMEN_ARUCO18_NAME         "carmen_aruco_message18"
#define      CARMEN_ARUCO19_NAME         "carmen_aruco_message19"
#define      CARMEN_ARUCO20_NAME         "carmen_aruco_message20"

#define      CARMEN_ARUCO_MESSAGE_FMT   "{int,<{int,<int:1>,[double:3],[double:3]}:1>,double,string}}"


#ifdef __cplusplus
}
#endif

#endif
