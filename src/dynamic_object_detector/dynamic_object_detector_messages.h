/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef CARMEN_DYNAMIC_OBJECT_DETECTOR_MESSAGES_H
#define CARMEN_DYNAMIC_OBJECT_DETECTOR_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  
	int x,y;
}carmen_dynamic_object_detector_cartesian_points;

typedef struct {
	carmen_dynamic_object_detector_cartesian_points* points;
	int points_count;
} carmen_dynamic_object_detector_object;

/* Message Struct Example */
typedef struct {
	double* objects_map;
	int map_size;
	carmen_dynamic_object_detector_object* objects_list;
	int obj_count;
	carmen_map_config_t config;
	double timestamp; 		/* !!! obrigatory !!! */
	char *host; 			/* !!! obrigatory !!! */
} carmen_dynamic_object_detector_clustered_objects_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_NAME       "carmen_dynamic_object_detector_clustered_objects"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_FMT       "{<double:2>, int,<{<{int, int}:2>, int}:4>, int, {int, int, double, [byte:64], string, double, double}, double, string}"
//#define      CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_FMT       "{<double:2>, int, {int, int, double, [byte:64], string, double, double}, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
