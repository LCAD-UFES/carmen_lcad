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
	double horizontal_angle;
	double vertical_angle;
	double radial_distance;
} carmen_laser_ldmrs_point;

/* Message Struct Example */
typedef struct {                                     
	uint16_t scan_number;
	double scan_start_time;
	double scan_end_time;
	uint16_t angle_ticks_per_rotation;
	double start_angle;
	double end_angle;
	uint16_t scan_points;
	carmen_laser_ldmrs_point *arraypoints;
	double timestamp; 		/* !!! mandatory !!! */
	char *host; 			/* !!! mandatory !!! */
} carmen_laser_ldmrs_message;

typedef struct {
	unsigned short id;
	double x;
	double y;
	double lenght;
	double width;
	double velocity;
	double orientation;
} carmen_laser_ldmrs_object;

typedef struct {
	unsigned short num_objects;
	carmen_laser_ldmrs_object *objects_list;
	double timestamp;
	char *host;
} carmen_laser_ldmrs_objects_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_LASER_LDMRS_NAME       "carmen_laser_ldmrs"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_LASER_LDMRS_FMT        "{short,double,double,short,double,double,short,<{double,double,double}:7>,double,string}"

#define		 CARMEN_LASER_LDMRS_OBJECTS_NAME "carmen_laser_ldmrs_objects"

#define		 CARMEN_LASER_LDMRS_OBJECTS_FMT  "{short,<{short,double,double,double,double,double,double}:1>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
