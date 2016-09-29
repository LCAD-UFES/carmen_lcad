#ifndef STEHS_PLANNER_H
#define STEHS_PLANNER_H


#ifdef __cplusplus
extern "C" {
#endif


#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/ford_escape_hybrid_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>


typedef struct
{
	carmen_point_t pose;
	double v;
	double phi;

	carmen_ackerman_traj_point_t goal;
	carmen_obstacle_distance_mapper_message *distance_map;

	carmen_robot_ackerman_config_t robot_config;

	bool active;
	unsigned int show_debug_info;
	unsigned int cheat;
	bool ready;

} stehs_planner_config_t, *stehs_planner_config_p;


#ifdef __cplusplus
}
#endif

#endif
