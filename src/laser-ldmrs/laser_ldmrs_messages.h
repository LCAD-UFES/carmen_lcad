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
	unsigned short flags;
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
	unsigned short classId;
} carmen_laser_ldmrs_object;

typedef struct {
	unsigned short num_objects;
	carmen_laser_ldmrs_object *objects_list;
	double timestamp;
	char *host;
} carmen_laser_ldmrs_objects_message;


typedef struct {
	unsigned short object_id;
	unsigned short object_age;
	unsigned short object_prediction_age;
	double reference_point_x;
	double reference_point_y;
	double reference_point_sigma_x;
	double reference_point_sigma_y;
	double closest_point_x;
	double closest_point_y;
	double bounding_box_center_x;
	double bounding_box_center_y;
	double bounding_box_length;
	double bounding_box_width;
	double object_box_center_x;
	double object_box_center_y;
	double object_box_lenght;
	double object_box_width;
	double object_box_orientation;
	double abs_velocity_x;
	double abs_velocity_y;
	double abs_velocity_sigma_x;
	double abs_velocity_sigma_y;
	double relative_velocity_x;
	double relative_velocity_y;
	unsigned short class_id;
} carmen_laser_ldmrs_object_data;

typedef struct {
	unsigned short num_objects;
	carmen_laser_ldmrs_object_data *objects_data_list;
	double timestamp;
	char *host;
} carmen_laser_ldmrs_objects_data_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_LASER_LDMRS_NAME       "carmen_laser_ldmrs"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_LASER_LDMRS_FMT        "{short,double,double,short,double,double,short,<{double,double,double,short}:7>,double,string}"

#define		 CARMEN_LASER_LDMRS_OBJECTS_NAME "carmen_laser_ldmrs_objects"

#define		 CARMEN_LASER_LDMRS_OBJECTS_FMT  "{short,<{short,double,double,double,double,double,double,short}:1>,double,string}"

#define		 CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME "carmen_laser_ldmrs_objects_data"

#define		 CARMEN_LASER_LDMRS_OBJECTS_DATA_FMT  "{short,<{short,short,short,double,double,double,double,double,double,double,double,double,double,double,double,double,double,double,double,double,double,double,double,double,short}:1>,double,string}"


#ifdef __cplusplus
}
#endif

#endif

// @}
