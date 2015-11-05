#ifndef CARMEN_KINECT_INTERFACE_H
#define CARMEN_KINECT_INTERFACE_H

#include "kinect_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_kinect_subscribe_depth_message(int kinect_id, carmen_kinect_depth_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_kinect_subscribe_depth_mrds_message0(carmen_kinect_depth_mrds_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_kinect_unsubscribe_depth_message(int kinect_id, carmen_handler_t handler);

void
carmen_kinect_unsubscribe_depth_mrds_message0(carmen_handler_t handler);

void
carmen_kinect_subscribe_video_message(int kinect_id, carmen_kinect_video_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_kinect_subscribe_video_mrds_message0(carmen_kinect_video_mrds_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_kinect_unsubscribe_video_message(int kinect_id, carmen_handler_t handler);

void
carmen_kinect_unsubscribe_video_mrds_message0(carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_kinect_define_depth_mrds_message0();

IPC_RETURN_TYPE
carmen_kinect_define_video_mrds_message0();

IPC_RETURN_TYPE
carmen_kinect_define_kinect_messages(int kinect_id);

IPC_RETURN_TYPE
carmen_kinect_define_kinect_mrds_messages(int kinect_id);

char* carmen_kinect_get_depth_messagename(int kinect_id);

char* carmen_kinect_get_video_messagename(int kinect_id);

#ifdef __cplusplus
}
#endif

#endif
