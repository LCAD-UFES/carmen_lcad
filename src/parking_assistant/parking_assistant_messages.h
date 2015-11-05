/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef CARMEN_PARKING_ASSISTANT_MESSAGES_H
#define CARMEN_PARKING_ASSISTANT_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct {                                     
	carmen_point_t pose;
	double timestamp; 		/* !!! obrigatory !!! */
	char *host; 			/* !!! obrigatory !!! */
} carmen_parking_assistant_goal_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_PARKING_ASSISTANT_GOAL_NAME       "carmen_parking_assistant_goal"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_PARKING_ASSISTANT_GOAL_FMT        "{{double, double, double},double,string}"

/* Message Struct Example */
typedef struct {
	double size,r;
	carmen_point_t p1,p2,p3,c1,c2;
	double timestamp; 		/* !!! obrigatory !!! */
	char *host; 			/* !!! obrigatory !!! */
} carmen_parking_assistant_parking_space_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_PARKING_ASSISTANT_PARKING_SPACE_NAME       "carmen_parking_assistant_parking_space"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_PARKING_ASSISTANT_PARKING_SPACE_FMT        "{double,double,{double, double, double},{double, double, double},{double, double, double},{double, double, double},{double, double, double},double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
