/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

 *********************************************************/

#ifndef CARMEN_FACIAL_GREETING_MESSAGES_H
#define CARMEN_FACIAL_GREETING_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct {
	double timestamp; 		/* !!! obligatory !!! */
	char *host; 			/* !!! obligatory !!! */
} carmen_facial_greeting_default_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_FACIAL_GREETING_DEFAULT_MESSAGE_NAME       "carmen_facial_greeting_default_message"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_FACIAL_GREETING_DEFAULT_MESSAGE_FMT        "{double,string}"

typedef struct {
	char *name;
	double timestamp; 		/* !!! obligatory !!! */
	char *host; 			/* !!! obligatory !!! */
} carmen_face_recog_message;

#define      CARMEN_FACE_RECOG_MESSAGE_NAME       "carmen_face_recog_message"
#define      CARMEN_FACE_RECOG_MESSAGE_FMT        "{string,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
