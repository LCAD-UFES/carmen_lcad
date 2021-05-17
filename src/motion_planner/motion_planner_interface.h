/*
 * motion_planner_interface.h
 *
 *  Created on: 20/09/2012
 *      Author: romulo
 */

#ifndef MOTION_PLANNER_INTERFACE_H_
#define MOTION_PLANNER_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <carmen/carmen.h>
#include "motion_planner_messages.h"

void carmen_motion_planner_subscribe_path_message(
		carmen_motion_planner_path_message *path,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_motion_planner_publish_path_message(
		carmen_robot_and_trailer_traj_point_t *path, int size, int algorithm);

void carmen_motion_planner_define_path_message();


#ifdef __cplusplus
}
#endif



#endif /* MOTION_PLANNER_INTERFACE_H_ */
