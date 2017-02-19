#ifndef UDATMO_DETECTOR_C_API_H
#define UDATMO_DETECTOR_C_API_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "udatmo_messages.h"

#include <carmen/obstacle_distance_mapper_messages.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/rddf_messages.h>

carmen_udatmo_moving_obstacles_message *carmen_udatmo_detector_detect(void);

void carmen_udatmo_detector_setup(int argc, char *argv[]);

void carmen_udatmo_detector_update_distance_map(carmen_obstacle_distance_mapper_message *message);

void carmen_udatmo_detector_update_globalpos(carmen_localize_ackerman_globalpos_message *message);

void carmen_udatmo_detector_update_rddf(carmen_rddf_road_profile_message *message);

#ifdef __cplusplus
}
#endif

#endif
