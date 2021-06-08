#include <carmen/grid_mapping.h>
#include <carmen/map_io.h>
#include <carmen/map_interface.h>
#include <carmen/map_server_interface.h>

#ifndef CARMEN_MOVING_OBJECTS_MAP_MESSAGES_H
#define CARMEN_MOVING_OBJECTS_MAP_MESSAGES_H


/********************
 * laser - velodyne *
 ********************/
#ifndef CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGES_H
#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define	PEDESTRIAN	'P'
#define	BIKE		'b' // Width: 1,20 m; Length: 2,20 m
#define	CAR			'C' // Width: 1,8 m to 2,1; Length: 3,9 m to 5,3 m
#define	BUS			'B' // Width: 2,4 m to 2,6 m; Length: 10 m to 14 m;
#define	TRUCK		'T'
#define	UNKNOWN		'U'


/************
 * grid map *
 ************/
typedef struct {
	double *complete_map;
	int size;
	carmen_map_config_t config;
	double timestamp;			/* !!! obligatory !!! */
	char *host;					/* !!! obligatory !!! */
} carmen_moving_objects_map_message;

#define CARMEN_MOVING_OBJECTS_MAP_MESSAGE_NAME "carmen_moving_objects_map_message_name"
#define CARMEN_MOVING_OBJECTS_MAP_MESSAGE_FMT		"{<double:2>, int, {int, int, double, [byte:64], string, double, double}, double, string}"


/*********
 * laser *
 *********/
typedef struct {
	double x;
	double y;
	int label;  // 1: moving object; 0: static object
	int cluster; // is core cluster
	double range;
} cartesian_points;

typedef struct {
	cartesian_points *points;
	int num_points;
	int num_clusters;
} carmen_objects_cartesian_points;

typedef struct {
	double *range;
	carmen_pose_3D_t car_pose;
	carmen_vector_3D_t *points;
	carmen_laser_laser_config_t config;
	carmen_vector_3D_t velocity;
	double pose_time;
	double phi;
	int num_points;
	int num_readings;
} carmen_laser_cartesian_points;


/***************************
 * velodyne - point clouds *
 ***************************/
struct _object_geometry
{
	double length;
	double width;
	double height;
};

typedef struct _object_geometry object_geometry_t;

struct _object_model_features {
	int model_id;		// Car, Bike, etc.
	char *model_name; 	// short description of the type of model
	object_geometry_t geometry;
	double red, green, blue;
};

typedef struct _object_model_features object_model_features_t;

struct _particle_print {
	carmen_point_t pose;
	double velocity;
	int class_id;
	object_geometry_t geometry;
};

typedef struct _particle_print particle_print_t;

typedef struct {
	int point_size;
	int num_valid_samples;
	int in_front;
	int lane_id;
	int lane_index;
	int index_in_poses_ahead;
	double r, g, b;
	double linear_velocity;
	double linear_velocity_std;
	double lateral_velocity;
	double lateral_velocity_std;
	double orientation;
	double length;
	double length_std;
	double width;
	double width_std;
	double height;
	double height_std;
	int geometric_model;
	object_model_features_t model_features;
	int	num_associated;	// The moving object id
	carmen_vector_3D_t object_pose;
	carmen_vector_3D_t *points;
	// particle_print_t *particulas; // para a visualização das partículas
} t_point_cloud_struct;

typedef struct {
	int num_point_clouds;
	t_point_cloud_struct *point_clouds;
	double timestamp;
	char *host;
} carmen_moving_objects_point_clouds_message;


struct _moving_objects_tracking
{
	carmen_pose_3D_t moving_objects_pose;
	double linear_velocity;
	double length;
	double width;
	double height;
	int geometric_model;
	object_model_features_t model_features;
	int	num_associated;
	double timestamp;
	//particle_print_t *particulas; // para a visualização das partículas
};

typedef struct _moving_objects_tracking moving_objects_tracking_t;


#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_NAME 	"carmen_moving_objects_point_clouds_message_name"

#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_FMT		"{int, <{int, int, int, int, int, int, double, double, double, double, double, double, double, double, double, double, double, double, double, double, int, {int, string, {double, double, double}, double, double, double}, int, {double, double, double}, <{double, double, double}:1>}:1>, double, string}"

#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_0_NAME 	"carmen_moving_objects_point_clouds_message_0_name"
#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_1_NAME 	"carmen_moving_objects_point_clouds_message_1_name"
#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_2_NAME 	"carmen_moving_objects_point_clouds_message_2_name"
#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_3_NAME 	"carmen_moving_objects_point_clouds_message_3_name"
#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_4_NAME 	"carmen_moving_objects_point_clouds_message_4_name"
//#define CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_FMT		"{int, <{int, double, double, double, double, double, double, double, double, int, {int, string, {double, double, double}, double, double, double}, int, {double, double, double}, <{double, double, double}:1>,<{{double, double, double}, double, int, {double, double, double} }:1>}:1>, double, string}"


#ifdef __cplusplus
}
#endif

#endif
#endif
