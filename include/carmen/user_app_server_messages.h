#include <carmen/carmen.h>

#ifndef USER_APP_SERVER_MESSAGES_H
#define USER_APP_SERVER_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif

#define USER_APP_SERVER_MISSION_ABORT	1


typedef struct
{
	char *mission;
	double timestamp;
	char *host;
} carmen_user_app_server_execute_mission_message;

#define	CARMEN_USER_APP_SERVER_EXECUTE_MISSION_MESSAGE_NAME	"carmen_user_app_server_execute_mission"
#define	CARMEN_USER_APP_SERVER_EXECUTE_MISSION_MESSAGE_FMT	"{string,double,string}"

typedef struct
{
	int mission_uptate_code;
	double timestamp;
	char *host;
} carmen_user_app_server_update_mission_message;

#define	CARMEN_USER_APP_SERVER_UPDATE_MISSION_MESSAGE_NAME	"carmen_user_app_server_update_mission_message"
#define	CARMEN_USER_APP_SERVER_UPDATE_MISSION_MESSAGE_FMT	"{int,double,string}"


#ifdef __cplusplus
}
#endif

#endif
