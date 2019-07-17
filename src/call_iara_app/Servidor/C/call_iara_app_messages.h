#include <carmen/carmen.h>
#include <carmen/voice_interface_interface.h>
#ifndef CARMEN_APP_SOLICITATION_MESSAGES_H
#define CARMEN_APP_SOLICITATION_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif


#define MAXSIZE 1024

typedef struct
{
	int reqnumber;
	char origin[255];
	char destination[255];
	char ipclient[255];
} carmen_app_solicitation_message;


#define CARMEN_APP_SOLICITATION_MESSAGE_NAME	"carmen_app_solicitation_message"
#define CARMEN_APP_SOLICITATION_MESSAGE_FMT		"{int, string, string, string}"

#ifdef __cplusplus
}
#endif

#endif
