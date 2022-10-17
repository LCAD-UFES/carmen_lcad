
#ifndef TASK_MANAGER_MESSAGES_H
#define TASK_MANAGER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define ENGAGED		1
#define DISENGAGED	0


// current_task
typedef enum
{
	TASK_MANAGER_NONE,
	TASK_MANAGER_SET_FINAL_GOAL,
	TASK_MANAGER_SET_ROBOT_POSE,
	TASK_MANAGER_FOLLOW_ROUTE,
	TASK_MANAGER_PARK,
	TASK_MANAGER_PARK_TRUCK_AND_SEMI_TRAILER,
	TASK_MANAGER_GO_AFTER_SET_COURSE_TO,
	TASK_MANAGER_GO_AFTER_PARK_AT,
	TASK_MANAGER_GO_AFTER_PARK_TRUCK_AND_SEMI_TRAILER_AT,
	TASK_MANAGER_SET_ROUTE_PLANNER_GRAPH,
	TASK_MANAGER_SET_MAP_SERVER_MAP,
	TASK_MANAGER_GET_ENGAGE_POSE,
	TASK_MANAGER_SET_COLLISION_GEOMETRY_ENGAGE,
	TASK_MANAGER_SET_COLLISION_GEOMETRY_DEFAULT,
	TASK_MANAGER_DISENGAGE_SEMI_TRAILER,
	TASK_MANAGER_ENGAGE_SEMI_TRAILER,
	TASK_MANAGER_SET_MAXIMUM_SPEED,
	TASK_MANAGER_ANNOUNCEMENT,
	TASK_MANAGER_ABORT,
	TASK_MANAGER_PUBLISH_MISSION_STATE,
	TASK_MANAGER_BUTTON,
	TASK_MANAGER_WAIT_FOR_BUTTON,
	TASK_MANAGER_SET_LED_PATTERN,
	TASK_MANAGER_SET_POWER_TAKE_OFF_STATE,
	TASK_MANAGER_SET_TILT_CART_STATE,
	TASK_MANAGER_SOUND_HORN,
	TASK_MANAGER_WAIT_TIME,
	TASK_MANAGER_RESTART,
	TASK_MANAGER_REPEAT,
	TASK_MANAGER_GO,
	TASK_MANAGER_STOP,
	TASK_MANAGER_EXECUTE,
	TASK_MANAGER_INIT,
} carmen_task_manager_task_type;


#define print_task(x) ( \
	(x == TASK_MANAGER_NONE)? "NONE": \
	(x == TASK_MANAGER_SET_FINAL_GOAL)? "SET_FINAL_GOAL": \
	(x == TASK_MANAGER_SET_ROBOT_POSE)? "SET_ROBOT_POSE": \
	(x == TASK_MANAGER_FOLLOW_ROUTE)? "FOLLOW_ROUTE": \
	(x == TASK_MANAGER_PARK)? "PARK": \
	(x == TASK_MANAGER_PARK_TRUCK_AND_SEMI_TRAILER)? "PARK_TRUCK_AND_SEMI_TRAILER": \
	(x == TASK_MANAGER_GO_AFTER_SET_COURSE_TO)? "GO_AFTER_SET_COURSE_TO": \
	(x == TASK_MANAGER_GO_AFTER_PARK_AT)? "GO_AFTER_PARK_AT": \
	(x == TASK_MANAGER_GO_AFTER_PARK_TRUCK_AND_SEMI_TRAILER_AT)? "GO_AFTER_PARK_TRUCK_AND_SEMI_TRAILER_AT": \
	(x == TASK_MANAGER_SET_ROUTE_PLANNER_GRAPH)? "SET_ROUTE_PLANNER_GRAPH": \
	(x == TASK_MANAGER_SET_MAP_SERVER_MAP)? "SET_MAP_SERVER_MAP": \
	(x == TASK_MANAGER_SET_COLLISION_GEOMETRY_ENGAGE)? "SET_COLLISION_GEOMETRY_ENGAGE": \
	(x == TASK_MANAGER_SET_COLLISION_GEOMETRY_DEFAULT)? "SET_COLLISION_GEOMETRY_DEFAULT": \
	(x == TASK_MANAGER_DISENGAGE_SEMI_TRAILER)? "DISENGAGE_SEMI_TRAILER": \
	(x == TASK_MANAGER_ENGAGE_SEMI_TRAILER)? "ENGAGE_SEMI_TRAILER": \
	(x == TASK_MANAGER_ENGAGE_SEMI_TRAILER)? "SET_MAXIMUM_SPEED": \
	(x == TASK_MANAGER_ANNOUNCEMENT)? "ANNOUNCEMENT": \
	(x == TASK_MANAGER_ABORT)? "ABORT": \
	(x == TASK_MANAGER_PUBLISH_MISSION_STATE)? "PUBLISH_MISSION_STATE" : \
	(x == TASK_MANAGER_BUTTON)? "BUTTON" : \
	(x == TASK_MANAGER_WAIT_FOR_BUTTON)? "WAIT_FOR_BUTTON" : \
	(x == TASK_MANAGER_SET_LED_PATTERN)? "SET_LED_PATTERN" : \
	(x == TASK_MANAGER_SET_POWER_TAKE_OFF_STATE)? "SET_POWER_TAKE_OFF_STATE" : \
	(x == TASK_MANAGER_SET_TILT_CART_STATE)? "SET_TILT_CART_STATE" : \
	(x == TASK_MANAGER_SOUND_HORN)? "SOUND_HORN" : \
	(x == TASK_MANAGER_WAIT_TIME)? "WAIT_TIME" : \
	(x == TASK_MANAGER_RESTART)? "RESTART": \
	(x == TASK_MANAGER_REPEAT)? "REPEAT": \
	(x == TASK_MANAGER_GO)? "GO": \
	(x == TASK_MANAGER_STOP)? "STOP": \
	(x == TASK_MANAGER_EXECUTE)? "EXECUTE": \
	(x == TASK_MANAGER_INIT)? "INIT": "")


typedef enum
{
	STARTING_MISSION,
	MISSION_COMPLETED,
	MISSION_FAILED,
	MISSION_FILE_STARTED,
	MISSION_FILE_FINISHED,
	IDLE_MODE
} carmen_task_manager_mission_level_state_t;

#define print_mission_level_state(x) ( \
	(x == STARTING_MISSION) 	? "STARTING_MISSION": \
	(x == MISSION_COMPLETED)	? "MISSION_COMPLETED": \
	(x == MISSION_FAILED)		? "MISSION_FAILED": \
	(x == MISSION_FILE_STARTED)	? "MISSION_FILE_STARTED": \
	(x == MISSION_FILE_FINISHED)	? "MISSION_FILE_FINISHED": \
	(x == IDLE_MODE)		? "IDLE_MODE": "")

//Sugestao-pensar-sobre
//typedef enum
//{
//	NEW_MISSION_FILE_STARTED,
//	LOAD,
//	UNLOAD,
//	PARK
//} carmen_task_manager_mission_id_t;


typedef struct
{
	int geometry;
	double timestamp;
	char *host;
} carmen_task_manager_set_collision_geometry_message;

#define	CARMEN_TASK_MANAGER_SET_GEOMETRY_MESSAGE_NAME	"carmen_task_manager_set_collision_geometry"
#define	CARMEN_TASK_MANAGER_SET_GEOMETRY_MESSAGE_FMT	"{int,double,string}"

typedef struct
{
	int desired_engage_state;
	double timestamp;
	char *host;
} carmen_task_manager_desired_engage_state_message;

#define	CARMEN_TASK_MANAGER_DESIRED_ENGAGE_STATE_MESSAGE_NAME	"carmen_task_manager_desired_engage_state"
#define	CARMEN_TASK_MANAGER_DESIRED_ENGAGE_STATE_MESSAGE_FMT	"{int,double,string}"

typedef struct
{
	int semi_trailer_type;
	int num_trailers;
	double trailer_theta[MAX_NUM_TRAILERS];
	double timestamp;
	char *host;
} carmen_task_manager_set_semi_trailer_type_and_beta_message;

#define	CARMEN_TASK_MANAGER_SET_SEMI_TRAILER_TYPE_AND_BETA_MESSAGE_NAME	"carmen_task_manager_set_semi_trailer_type_and_beta"
#define	CARMEN_TASK_MANAGER_SET_SEMI_TRAILER_TYPE_AND_BETA_MESSAGE_FMT	"{int,double,double,string}"

typedef struct
{
	carmen_task_manager_mission_level_state_t mission_state;
	char *mission_id;
	char *mission_filename;
	carmen_robot_and_trailers_pose_t pose;
	char *info;
	double timestamp;
	char *host;
} carmen_task_manager_mission_state_message;

#define	CARMEN_TASK_MANAGER_MISSION_STATE_MESSAGE_NAME	"carmen_task_manager_mission_state_message"
#define	CARMEN_TASK_MANAGER_MISSION_STATE_MESSAGE_FMT "{int,string,string,{double,double,double,double},string,double,string}"


typedef struct
{
	carmen_task_manager_task_type current_task;
	double timestamp;
	char *host;
} carmen_task_manager_task_state_message;

#define	CARMEN_TASK_MANAGER_TASK_STATE_MESSAGE_NAME	"carmen_task_manager_task_state_message"
#define	CARMEN_TASK_MANAGER_TASK_STATE_MESSAGE_FMT "{int,double,string}"


#ifdef __cplusplus
}
#endif

#endif

