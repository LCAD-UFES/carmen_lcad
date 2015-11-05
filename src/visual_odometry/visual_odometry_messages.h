#ifndef CARMEN_VISUAL_ODOMETRY_MESSAGES_H
#define CARMEN_VISUAL_ODOMETRY_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int x, y;
	int valido;
} gdk_position;

typedef struct {
	double x, y, z;
	double yaw, pitch, roll;
} carmen_6d_point;

typedef carmen_6d_point carmen_6d_velocity;

typedef struct {
  carmen_6d_point pose_6d;
  carmen_6d_velocity velocity_6d;
  double v;
  double phi;
  double timestamp;
  char *host;
} carmen_visual_odometry_pose6d_message;

#define      CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME       "carmen_visual_odometry_pose6d_message"
#define      CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_FMT        "{{double, double, double, double, double, double}, {double, double, double, double, double, double}, double, double, double, string}"

typedef struct {
	int image_size;
	unsigned char* left_image;
	int inliers_size;
	int* inliers;
	carmen_6d_point pose_6d;
	double timestamp;
	char* host;
} carmen_visual_odometry_image_message;

#define      CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_NAME       "carmen_visual_odometry_image_message"
#define      CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_FMT        "{int, <byte:1>, int, <int:3>,{double, double, double, double, double, double}, double, string}"
#ifdef __cplusplus
}
#endif

#endif

// @}
