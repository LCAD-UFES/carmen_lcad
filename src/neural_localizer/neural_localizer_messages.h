#ifndef CARMEN_NEURAL_LOCALIZER_MESSAGES_H
#define CARMEN_NEURAL_LOCALIZER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

//* Message Struct Example */
typedef struct {
	int width;
	int heigth;
	int image_size;
	unsigned char* image;
  double timestamp; 		/* !!! obrigatory !!! */
  char *host; 			/* !!! obrigatory !!! */
} carmen_neural_localizer_place_command_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_NAME       "carmen_neural_localizer_place_command_message"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_FMT       "{int, int, int, <ubyte:3>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
