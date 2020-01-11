
#ifndef __CARMEN_RDDF_MESSAGES_H__
#define __CARMEN_RDDF_MESSAGES_H__

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C"
{
#endif

	#define NUM_RDDF_ANNOTATION_TYPES						14

	#define RDDF_ANNOTATION_TYPE_NONE 						0
	#define RDDF_ANNOTATION_TYPE_END_POINT_AREA 			1
	#define RDDF_ANNOTATION_TYPE_HUMAN_INTERVENTION 		2
    #define RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK 			3
    #define RDDF_ANNOTATION_TYPE_STOP 						4
    #define RDDF_ANNOTATION_TYPE_BARRIER 					5
    #define RDDF_ANNOTATION_TYPE_BUMP 						6
    #define RDDF_ANNOTATION_TYPE_SPEED_LIMIT 				7
	#define RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT 				8
	#define RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN 				9
	#define RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP			10
	#define RDDF_ANNOTATION_TYPE_DYNAMIC					11
	#define RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP		12
	#define RDDF_ANNOTATION_TYPE_PLACE_OF_INTEREST			13


	#define NUM_RDDF_ANNOTATION_CODES						22

	#define RDDF_ANNOTATION_CODE_NONE		 				0

	#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_0 				1
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_5 				2
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_10 			3
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_15 			4
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_20 			5
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_30 			6
    #define RDDF_ANNOTATION_CODE_SPEED_LIMIT_40 			7
	#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_60 			8
	#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_80 			9
	#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_100 			10
	#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_110 			11

	#define RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED 			12
	#define RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN 		13
	#define RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW 		14
	#define RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF 			15
	#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_RIGHT 	16
	#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_LEFT		17

	#define RDDF_ANNOTATION_CODE_DYNAMIC_STOP				18

	#define RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_BUSY		19

	#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_GO_STRAIGHT	20
	#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF			21


	typedef struct
    {
        int number_of_poses;
        int number_of_poses_back;
        carmen_ackerman_traj_point_t *poses;
        carmen_ackerman_traj_point_t *poses_back;
        int *annotations;
        int *annotations_codes;
        double timestamp;
        char *host;
    } carmen_rddf_road_profile_message;

	#define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME "carmen_rddf_road_profile_message"
	#define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT "{int, int, <{double, double, double, double, double}:1>, <{double, double, double, double, double}:2>, <int:1>, <int:1>, double, string}"


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


    typedef struct
    {
        int number_of_poses;
        carmen_ackerman_traj_point_t *poses;
        double timestamp;
        char *host;
    } carmen_rddf_waypoints_around_end_point_message;

	#define CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME "carmen_rddf_waypoints_around_end_point_message"
	#define CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_FMT "{int, <{double,double,double,double,double}:1>, double, string}"


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


	typedef carmen_rddf_add_annotation_message carmen_rddf_dynamic_annotation_message;

	#define CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME "carmen_rddf_dynamic_annotation_message"
	#define CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_FMT "{{double,double,double},double,string,int,int,double,string}"


    typedef struct
    {
		carmen_vector_3D_t annotation_point;
		double annotation_orientation;
		char *annotation_description;
		int annotation_type;
		int annotation_code;
    } carmen_annotation_t;

    typedef struct
	{
    	int num_annotations;
    	carmen_annotation_t *annotations;
		double timestamp;
		char *host;
	} carmen_rddf_annotation_message;

	#define CARMEN_RDDF_ANNOTATION_MESSAGE_NAME "carmen_rddf_annotation_message"
	#define CARMEN_RDDF_ANNOTATION_MESSAGE_FMT "{int, <{{double,double,double},double,string,int,int}:1>,double,string}"

    typedef struct
	{
    	int traffic_sign_state;
    	double traffic_sign_data;
		double timestamp;
		char *host;
	} carmen_rddf_traffic_sign_message;

	#define CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME "carmen_rddf_traffic_sign_message"
	#define CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_FMT "{int,double,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
