
#ifndef _CARMEN_FRENET_PATH_PLANNER_INTERFACE_H_
#define _CARMEN_FRENET_PATH_PLANNER_INTERFACE_H_

#include <carmen/carmen.h>
#include <carmen/route_planner_interface.h>
#include "frenet_path_planner_messages.h"


#ifdef __cplusplus
extern "C"
{
#endif

void
carmen_frenet_path_planner_define_messages();

void
carmen_frenet_path_planner_subscribe_set_of_paths_message(carmen_frenet_path_planner_set_of_paths *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_frenet_path_planner_unsubscribe_set_of_paths_message(carmen_handler_t handler);

void
carmen_frenet_path_planner_publish_set_of_paths_message(carmen_frenet_path_planner_set_of_paths *message);

void
carmen_frenet_path_planner_subscribe_selected_path_message(carmen_frenet_path_planner_selected_path *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);


void
carmen_frenet_path_planner_unsubscribe_selected_path_message(carmen_handler_t handler);

void
carmen_frenet_path_planner_publish_selected_path_message(carmen_frenet_path_planner_selected_path *message);

void
frenet_path_planner_get_parameters(int argc, char **argv);

carmen_frenet_path_planner_set_of_paths
frenet_path_planner_build_frenet_path_plan(carmen_robot_and_trailer_traj_point_t *poses_ahead,
		carmen_robot_and_trailer_traj_point_t *poses_back,
		int num_poses, int num_poses_back, double v, int *annotations, int *annotations_codes,
		carmen_route_planner_road_network_message *road_network_message, double timestamp);

#ifdef __cplusplus
}
#endif

#endif

// @}

