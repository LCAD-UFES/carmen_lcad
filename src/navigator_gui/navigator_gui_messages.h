/*
 * navigator_gui_messages.h
 *
 *  Created on: 29/01/2013
 *      Author: romulo
 */
#include <carmen/carmen.h>

#ifndef NAVIGATOR_GUI_MESSAGES_H_
#define NAVIGATOR_GUI_MESSAGES_H_


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {CARMEN_NAVIGATOR_MAP_v, CARMEN_NAVIGATOR_MAP_LEVEL1_v, CARMEN_NAVIGATOR_ENTROPY_v,
	      CARMEN_NAVIGATOR_COST_v, CARMEN_NAVIGATOR_UTILITY_v,
          CARMEN_LOCALIZE_LMAP_v, CARMEN_LOCALIZE_GMAP_v, CARMEN_LANE_MAP_v, CARMEN_COST_MAP_v,
          CARMEN_OFFLINE_MAP_v, CARMEN_NONE_v, CARMEN_COMPLETE_MAP_v, CARMEN_REMISSION_MAP_v,
          CARMEN_MOVING_OBJECTS_MAP_v, CARMEN_ROAD_MAP_v
}carmen_navigator_map_t;

typedef struct {
	int r;
	int g;
	int b;
} carmen_rgb;

typedef struct {
	carmen_robot_and_trailers_traj_point_t *path;
	int path_length;
	int path_id; //unsigned number that identifies the path
	carmen_rgb path_color;
	double timestamp;
	char *host;
} carmen_navigator_gui_path_message;


#define      CARMEN_NAVIGATOR_GUI_PATH_NAME       "carmen_navigator_gui_path"
#define      CARMEN_NAVIGATOR_GUI_PATH_FMT        "{<{double, double, double, int, [double:5], double, double}:2>,int,int,{int, int, int},double,string}"

#ifdef __cplusplus
}
#endif


#endif /* NAVIGATOR_GUI_MESSAGES_H_ */
