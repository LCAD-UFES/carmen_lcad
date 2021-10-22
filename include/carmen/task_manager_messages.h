
#ifndef TASK_MANAGER_MESSAGES_H
#define TASK_MANAGER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define ENGAGED		1
#define DISENGAGED	0


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

#ifdef __cplusplus
}
#endif

#endif

