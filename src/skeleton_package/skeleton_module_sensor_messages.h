/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef CARMEN_SKELETON_MODULE_SENSOR_MESSAGES_H
#define CARMEN_SKELETON_MODULE_SENSOR_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct {                                     
  char caracter;
  double timestamp; 		/* !!! obrigatory !!! */
  char *host; 			/* !!! obrigatory !!! */
} carmen_skeleton_module_sensor_keymessage_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_NAME       "carmen_skeleton_module_sensor_keymessage"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_FMT        "{byte,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
