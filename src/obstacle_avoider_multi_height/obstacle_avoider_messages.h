#include <carmen/carmen.h>

#ifndef CARMEN_OBSTACLE_AVOIDER_MESSAGES_H
#define CARMEN_OBSTACLE_AVOIDER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	carmen_point_t left_near_obstacle;
	carmen_point_t right_near_obstacle;
	double timestamp;
	char *host;
} carmen_robot_ackerman_road_velocity_control_message;

#define      CARMEN_ROBOT_ACKERMAN_ROAD_VELOCITY_CONTROL_NAME         "carmen_robot_ackerman_road_velocity_control"
#define      CARMEN_ROBOT_ACKERMAN_ROAD_VELOCITY_CONTROL_FMT          "{{double, double, double}, {double,double,double}, double, string}"


typedef struct
{
	int robot_will_hit_obstacle;
	double timestamp;
	char *host;
} carmen_obstacle_avoider_robot_will_hit_obstacle_message;

#define      CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_NAME         "carmen_obstacle_avoider_robot_hit_obstacle"
#define      CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_FMT          "{int, double, string}"

#ifdef __cplusplus
}
#endif

#endif

