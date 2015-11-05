#ifndef CARMEN_STEREO_MAPPING_INTERFACE_H
#define CARMEN_STEREO_MAPPING_INTERFACE_H

#include "stereo_mapping_messages.h"

#define VIEW_MODE_BIRDS_EYE 'B'
#define VIEW_MODE_BIRDS_EYE_OPENCV 'O'
#define VIEW_MODE_ROAD_FINDING 'R'

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_stereo_mapping_subscribe_message(carmen_stereo_mapping_message *timestamp,
    carmen_handler_t handler,
    carmen_subscribe_t subscribe_how,
    int camera);

void
carmen_stereo_mapping_unsubscribe_message(carmen_handler_t handler, int camera);

IPC_RETURN_TYPE
carmen_stereo_mapping_define_messages(int camera);

char*
carmen_stereo_mapping_get_messagename(int camera);

#ifdef __cplusplus
}
#endif

#endif
