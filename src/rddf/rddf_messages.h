
#ifndef __CARMEN_RDDF_MESSAGES_H__
#define __CARMEN_RDDF_MESSAGES_H__

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Atencao!!! Ao adicionar um novo tipo de anotacao, incrementar o NUM_RDDF_ANNOTATION_TYPES
// (note que comecca de zero; assim, NUM_RDDF_ANNOTATION_TYPES eh igual ao ultimo codigo mais 1)
	#define NUM_RDDF_ANNOTATION_TYPES						24

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
	#define RDDF_ANNOTATION_TYPE_YIELD 						14
	#define RDDF_ANNOTATION_TYPE_PREDEFINED_ROUTE			15
	#define RDDF_ANNOTATION_TYPE_AMV						16
	#define RDDF_ANNOTATION_TYPE_NARROW_LANE				17
	#define RDDF_ANNOTATION_TYPE_RETARDER_BRAKE				18
	#define RDDF_ANNOTATION_TYPE_TURN_LEFT_INDICATOR		19
	#define RDDF_ANNOTATION_TYPE_TURN_RIGHT_INDICATOR		20
	#define RDDF_ANNOTATION_TYPE_GEAR						21
	#define RDDF_ANNOTATION_TYPE_QUEUE						22
	#define RDDF_ANNOTATION_TYPE_SINGLE_TRAFFIC_AREA		23


// Atencao!!! Ao adicionar um novo tipo de CODE de anotacao, incrementar o NUM_RDDF_ANNOTATION_CODES
// (note que comecca de zero; assim, NUM_RDDF_ANNOTATION_CODES eh igual ao ultimo codigo mais 1)
	#define NUM_RDDF_ANNOTATION_CODES						94

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

	#define RDDF_ANNOTATION_CODE_PEDESTRIAN_TRACK_BUSY		19

	#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_GO_STRAIGHT	20
	#define RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF			21

	#define RDDF_ANNOTATION_CODE_PREDEFINED_ROUTE_CYCLIC	22

	#define RDDF_ANNOTATION_CODE_BARRIER_25_25				23
	#define RDDF_ANNOTATION_CODE_BARRIER_25_20				24
	#define RDDF_ANNOTATION_CODE_BARRIER_25_15				25
	#define RDDF_ANNOTATION_CODE_BARRIER_25_10				26
	#define RDDF_ANNOTATION_CODE_BARRIER_20_25				27
	#define RDDF_ANNOTATION_CODE_BARRIER_20_20				28
	#define RDDF_ANNOTATION_CODE_BARRIER_20_15				29
	#define RDDF_ANNOTATION_CODE_BARRIER_20_10				30
	#define RDDF_ANNOTATION_CODE_BARRIER_15_25				31
	#define RDDF_ANNOTATION_CODE_BARRIER_15_20				32
	#define RDDF_ANNOTATION_CODE_BARRIER_15_15				33
	#define RDDF_ANNOTATION_CODE_BARRIER_15_10				34
	#define RDDF_ANNOTATION_CODE_BARRIER_10_25				35
	#define RDDF_ANNOTATION_CODE_BARRIER_10_20				36
	#define RDDF_ANNOTATION_CODE_BARRIER_10_15				37
	#define RDDF_ANNOTATION_CODE_BARRIER_10_10				38

	#define RDDF_ANNOTATION_CODE_NARROW_LANE_BEGIN			39
	#define RDDF_ANNOTATION_CODE_NARROW_LANE_END			40

	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_ON			41
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_OFF			42

	#define RDDF_ANNOTATION_CODE_TURN_LEFT_INDICATOR_ON		43
	#define RDDF_ANNOTATION_CODE_TURN_LEFT_INDICATOR_OFF	44

	#define RDDF_ANNOTATION_CODE_TURN_RIGHT_INDICATOR_ON	45
	#define RDDF_ANNOTATION_CODE_TURN_RIGHT_INDICATOR_OFF	46

	#define RDDF_ANNOTATION_CODE_GEAR_1 					47
	#define RDDF_ANNOTATION_CODE_GEAR_2 					48
	#define RDDF_ANNOTATION_CODE_GEAR_3 					49
	#define RDDF_ANNOTATION_CODE_GEAR_4 					50
	#define RDDF_ANNOTATION_CODE_GEAR_5 					51
	#define RDDF_ANNOTATION_CODE_GEAR_6 					52
	#define RDDF_ANNOTATION_CODE_GEAR_7 					53
	#define RDDF_ANNOTATION_CODE_GEAR_8 					54
	#define RDDF_ANNOTATION_CODE_GEAR_9 					55
	#define RDDF_ANNOTATION_CODE_GEAR_10 					56
	#define RDDF_ANNOTATION_CODE_GEAR_11 					57
	#define RDDF_ANNOTATION_CODE_GEAR_12 					58
	#define RDDF_ANNOTATION_CODE_GEAR_13 					59
	#define RDDF_ANNOTATION_CODE_GEAR_14 					60
	#define RDDF_ANNOTATION_CODE_GEAR_15 					61
	#define RDDF_ANNOTATION_CODE_GEAR_16 					62
	#define RDDF_ANNOTATION_CODE_GEAR_17 					63
	#define RDDF_ANNOTATION_CODE_GEAR_18 					64
	#define RDDF_ANNOTATION_CODE_GEAR_19 					65
	#define RDDF_ANNOTATION_CODE_GEAR_20 					66
	#define RDDF_ANNOTATION_CODE_GEAR_21 					67
	#define RDDF_ANNOTATION_CODE_GEAR_22 					68
	#define RDDF_ANNOTATION_CODE_GEAR_23 					69
	#define RDDF_ANNOTATION_CODE_GEAR_24 					70
	#define RDDF_ANNOTATION_CODE_GEAR_25 					71
	#define RDDF_ANNOTATION_CODE_GEAR_26 					72
	#define RDDF_ANNOTATION_CODE_GEAR_27 					73
	#define RDDF_ANNOTATION_CODE_GEAR_28 					74
	#define RDDF_ANNOTATION_CODE_GEAR_29 					75
	#define RDDF_ANNOTATION_CODE_GEAR_30 					76
	#define RDDF_ANNOTATION_CODE_GEAR_31 					77
	#define RDDF_ANNOTATION_CODE_GEAR_32 					78
	#define RDDF_ANNOTATION_CODE_GEAR_RW_1 					79
	#define RDDF_ANNOTATION_CODE_GEAR_RW_2 					80
	#define RDDF_ANNOTATION_CODE_GEAR_NEUTRAL 				81

	#define RDDF_ANNOTATION_CODE_SPEED_LIMIT_25 			82

	#define RDDF_ANNOTATION_CODE_QUEUE_BUSY					83

	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_1		84
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_2		85
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_3		86
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_4		87
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_5		88
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_6		89
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_7		90
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_8		91
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_9		92
	#define RDDF_ANNOTATION_CODE_RETARDER_BRAKE_LEVEL_10	93

	#define MIN_DISTANCE_TO_CONSIDER_CROSSWALK 1.0


	typedef struct
    {
        int number_of_poses;
        int number_of_poses_back;
        carmen_robot_and_trailers_traj_point_t *poses;
        carmen_robot_and_trailers_traj_point_t *poses_back;
        int *annotations;
        int *annotations_codes;
        double timestamp;
        char *host;
    } carmen_rddf_road_profile_message;

	#define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME "carmen_rddf_road_profile_message"
	#define CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT "{int, int, <{double, double, double, int, [double:5], double, double}:1>, <{double, double, double, int, [double:5], double, double}:2>, <int:1>, <int:1>, double, string}"


    typedef struct
    {
        int half_meters_to_final_goal;	// number of waypoints to consider near end_point (point)
        carmen_robot_and_trailers_pose_t point;
        double timestamp;
        char *host;
    } carmen_rddf_end_point_message;

	#define CARMEN_RDDF_END_POINT_MESSAGE_NAME "carmen_rddf_end_point_message"
	#define CARMEN_RDDF_END_POINT_MESSAGE_FMT "{int, {double, double, double, int, [double:5]}, double, string}"


    typedef struct
    {
        int number_of_poses;
        carmen_robot_and_trailers_traj_point_t *poses;
        double timestamp;
        char *host;
    } carmen_rddf_waypoints_around_end_point_message;

	#define CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME "carmen_rddf_waypoints_around_end_point_message"
	#define CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_FMT "{int, <{double,double,double,int, [double:5],double,double}:1>, double, string}"


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

	typedef enum {CREATE_ACTION, READ_ACTION, UPDATE_ACTION, DELETE_ACTION} crud_t;

	typedef struct
	{
		crud_t action;
		carmen_annotation_t old_annotation;
		carmen_annotation_t new_annotation;
		double timestamp;
		char *host;
	} carmen_rddf_update_annotation_message;

	#define CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_NAME "carmen_rddf_update_annotation_message"
	#define CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_FMT "{int,{{double,double,double},double,string,int,int},{{double,double,double},double,string,int,int},double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
