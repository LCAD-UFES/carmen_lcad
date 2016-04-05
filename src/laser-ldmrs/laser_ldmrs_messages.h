/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

 *********************************************************/

#ifndef CARMEN_LASER_LDMRS_MESSAGES_H
#define CARMEN_LASER_LDMRS_MESSAGES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint8_t layer;
	uint8_t echo;
	uint8_t flags;
	float horizontal_angle;
	float radial_distance;
	float pulse_width;
} carmen_laser_ldmrs_point;

/* Message Struct Example */
typedef struct {                                     
	uint16_t scan_number;
	uint16_t scanner_status;
	uint16_t sync_phase_offset;
//	struct timespec scan_start_time;
//	struct timespec scan_end_time;
	uint16_t angle_ticks_per_rotation;
	float start_angle;
	float end_angle;
	uint16_t scan_points;
	float mount_yaw;
	float mount_pitch;
	float mount_roll;
	float mount_x;
	float mount_y;
	float mount_z;
	uint16_t flags;
	carmen_laser_ldmrs_point *points;
	double timestamp; 		/* !!! mandatory !!! */
	char *host; 			/* !!! mandatory !!! */
} carmen_laser_ldmrs_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_LASER_LDMRS_NAME       "carmen_laser_ldmrs"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_LASER_LDMRS_FMT        "{short,short,short,short,float,float,short,float,float,float,float,float,float,short,<{byte,byte,byte,float,float,float}:7>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
