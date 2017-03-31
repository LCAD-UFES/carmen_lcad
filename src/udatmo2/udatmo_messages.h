#ifndef UDATMO_MESSAGES_H
#define UDATMO_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif


#include <carmen/carmen.h>


typedef struct
{
	int rddf_index;

	int rddf_lane;

	double x;

	double y;

	double theta;

	double v;
}
carmen_udatmo_moving_obstacle;


typedef struct
{
	/** @brief Name of originating host. */
	char *host;

	/** @brief Time of reading, in seconds since the epoch. */
	double timestamp;

	/** @brief Number of obstacles in this message. */
	int num_obstacles;

	/** @brief Array of moving obstacles. */
	carmen_udatmo_moving_obstacle *obstacles;
}
carmen_udatmo_moving_obstacles_message;


#define CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME "carmen_udatmo_moving_obstacles_message"

#define CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_FMT "{string, double, int, <{int, double, double, double, double}:1>}"


#ifdef __cplusplus
}
#endif

#endif
