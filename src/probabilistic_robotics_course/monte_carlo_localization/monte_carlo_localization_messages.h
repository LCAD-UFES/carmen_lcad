/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef CARMEN_MONTE_CARLO_LOCALIZATION_MESSAGES_H
#define CARMEN_MONTE_CARLO_LOCALIZATION_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct {
  double x, y, theta;
  double timestamp; 		/* !!! obrigatory !!! */
  char *host; 			/* !!! obrigatory !!! */
} carmen_monte_carlo_localization_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_MONTE_CARLO_LOCALIZATION_NAME       "carmen_monte_carlo_localization_message"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_MONTE_CARLO_LOCALIZATION_MESSAGE_FMT        "{double,double,double,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
