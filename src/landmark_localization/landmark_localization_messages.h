#ifndef CARMEN_LANDMARK_LOCALIZATION_MESSAGES_H
#define CARMEN_LANDMARK_LOCALIZATION_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct {
  double timestamp; 		/* !!! obrigatory !!! */
  char *host; 			/* !!! obrigatory !!! */
} carmen_landmark_localization_state_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_LANDMARK_LOCALIZATION_STATE_MESSAGE_NAME       "carmen_landmark_localization_state_message"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_LANDMARK_LOCALIZATION_STATE_MESSAGE_FMT        "{double,string}"

#ifdef __cplusplus
}
#endif

#endif
