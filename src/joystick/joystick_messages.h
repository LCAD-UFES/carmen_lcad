
#ifndef JOYSTICK_MESSAGES_H_
#define JOYSTICK_MESSAGES_H_

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int nb_axes;
	int nb_buttons;
	int *axes;
	int *buttons;
	double timestamp;
	char* host;
} carmen_joystick_status_message;

#define      CARMEN_JOYSTICK_STATUS_MESSAGE_NAME       "carmen_joystick_status_message"
#define      CARMEN_JOYSTICK_STATUS_MESSAGE_FMT        "{int, int, <int:1>, <int:2>, double, string}"

#ifdef __cplusplus
}
#endif



#endif /* JOYSTICK_MESSAGES_H_ */
