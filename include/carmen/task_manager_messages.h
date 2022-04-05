
#ifndef TASK_MANAGER_MESSAGES_H
#define TASK_MANAGER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define ENGAGED		1
#define DISENGAGED	0


typedef enum
{
	STARTING_MISSION,
	MISSION_COMPLETED,
	TASK_FAILED
} carmen_task_manager_mission_level_state_t;


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
	double beta;
	double timestamp;
	char *host;
} carmen_task_manager_set_semi_trailer_type_and_beta_message;

#define	CARMEN_TASK_MANAGER_SET_SEMI_TRAILER_TYPE_AND_BETA_MESSAGE_NAME	"carmen_task_manager_set_semi_trailer_type_and_beta"
#define	CARMEN_TASK_MANAGER_SET_SEMI_TRAILER_TYPE_AND_BETA_MESSAGE_FMT	"{int,double,double,string}"

typedef struct
{
	carmen_task_manager_mission_level_state_t mission_state;
	char *mission_filename;
	carmen_robot_and_trailer_pose_t final_goal;
	char *info;
	double timestamp;
	char *host;
} carmen_task_manager_state_message;

#define	CARMEN_TASK_MANAGER_STATE_MESSAGE_NAME	"carmen_task_manager_state_message"
#define	CARMEN_TASK_MANAGER_STATE_MESSAGE_FMT "{int,string,{double,double,double,double},string,double,string}"


#ifdef __cplusplus
}
#endif

#endif

