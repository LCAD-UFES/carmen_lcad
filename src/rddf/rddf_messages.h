
#ifndef __CARMEN_RDDF_MESSAGES_H__
#define __CARMEN_RDDF_MESSAGES_H__

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C"
{
#endif

//    typedef enum
//    {
//        RDDF_ANNOTATION_NONE = 0,
//        RDDF_ANNOTATION_END_POINT_AREA = 1,
//        RDDF_ANNOTATION_HUMAN_INTERVENTION = 2
//    } RDDF_ANNOTATION;

	#define RDDF_ANNOTATION_NONE 0
	#define RDDF_ANNOTATION_END_POINT_AREA 1
	#define RDDF_ANNOTATION_HUMAN_INTERVENTION 2
    #define RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK 3
    #define RDDF_ANNOTATION_TYPE_STOP 4
    #define RDDF_ANNOTATION_TYPE_BARRIER 5
    #define RDDF_ANNOTATION_TYPE_BUMP 6
    #define RDDF_ANNOTATION_TYPE_SPEED_LIMIT 7
	#define RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT 8
	#define RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN 9

    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_0 1
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_5 2
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_10 3
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_15 4
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_20 5
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_30 6
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_40 7
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_60 8
    #define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_BUMP 9
	#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_SPEED_20 10
	#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_SPEED_30 11

    typedef struct
    {
        int number_of_poses;
        int number_of_poses_back;
        carmen_ackerman_traj_point_t *poses;
        carmen_ackerman_traj_point_t *poses_back;
        // int *signals_annotations;
        int *annotations;
        double timestamp;
        char *host;
    } carmen_rddf_road_profile_message;

	#define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME "carmen_rddf_road_profile_message"
	// #define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT "{int, int, <{double, double, double, double, double}:1>, <{double, double, double, double, double}:2>, <int:1>, <int:1>, double, string}"
	#define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT "{int, int, <{double, double, double, double, double}:1>, <{double, double, double, double, double}:2>, <int:1>, double, string}"


    typedef struct
    {
        // trocar para number_of_waypoints
        int number_of_poses;
        carmen_point_t point;
        double timestamp;
        char *host;
    } carmen_rddf_end_point_message;

	#define CARMEN_RDDF_END_POINT_MESSAGE_NAME "carmen_rddf_end_point_message"
	#define CARMEN_RDDF_END_POINT_MESSAGE_FMT "{int, {double, double, double}, double, string}"


    typedef carmen_rddf_end_point_message carmen_rddf_nearest_waypoint_message;

	#define CARMEN_RDDF_NEAREST_WAYPOINT_MESSAGE_NAME "carmen_rddf_nearest_waypoint_message"
	#define CARMEN_RDDF_NEAREST_WAYPOINT_MESSAGE_FMT "{int, {double, double, double}, int, double, string}"


    typedef struct
    {
        int number_of_poses;
        carmen_ackerman_traj_point_t *poses;
        double timestamp;
        char *host;
    } carmen_rddf_waypoints_around_end_point_message;

	#define CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME "carmen_rddf_waypoints_around_end_point_message"
	#define CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_FMT "{int, <{double,double,double,double,double}:1>, double, string}"


    typedef carmen_rddf_end_point_message carmen_rddf_nearest_waypoint_confirmation_message;

	#define CARMEN_RDDF_NEAREST_WAYPOINT_CONFIRMATION_MESSAGE_NAME "carmen_rddf_nearest_waypoint_confirmation_message"
	#define CARMEN_RDDF_NEAREST_WAYPOINT_CONFIRMATION_MESSAGE_FMT "{int, {double, double, double}, int, double, string}"

    typedef struct
    {
		carmen_vector_3D_t annotation_point;
		double annotation_orientation;
		char *annotation_description;
		int annotation_type;
		int annotation_code;
    } carmen_annotation_t;

    // TODO: update this message to use carmen_annotation_t.
    typedef struct
	{
		carmen_vector_3D_t annotation_point;
		double annotation_orientation;
		char *annotation_description;
		int annotation_type;
		int annotation_code;
		double timestamp;
		char *host;
	} carmen_rddf_add_annotation_message;

    #define CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME "carmen_rddf_add_annotation_message"
    #define CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_FMT "{{double,double,double},double,string,int,int,double,string}"

	/*
	typedef carmen_rddf_add_annotation_message carmen_rddf_annotation_message;

	#define CARMEN_RDDF_ANNOTATION_MESSAGE_NAME "carmen_rddf_annotation_message"
	#define CARMEN_RDDF_ANNOTATION_MESSAGE_FMT "{{double,double,double},double,string,int,int,double,string}"
	 */

    typedef struct
	{
    	int num_annotations;
    	carmen_annotation_t *annotations;
		double timestamp;
		char *host;
	} carmen_rddf_annotation_message;

	#define CARMEN_RDDF_ANNOTATION_MESSAGE_NAME "carmen_rddf_annotation_message"
	#define CARMEN_RDDF_ANNOTATION_MESSAGE_FMT "{int, <{{double,double,double},double,string,int,int}:1>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
