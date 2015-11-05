#ifndef CARMEN_LASLAM_INTERFACE_H
#define CARMEN_LASLAM_INTERFACE_H

#include <carmen/laslam_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

void carmen_laslam_define_landmark_message();

void carmen_laslam_subscribe_landmark_message(
		carmen_laslam_landmark_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_laslam_unsubscribe_landmark_message(carmen_handler_t handler);

void carmen_laslam_publish_landmark_message(carmen_laslam_landmark_message* message);

#ifdef __cplusplus
}
#endif

#endif

