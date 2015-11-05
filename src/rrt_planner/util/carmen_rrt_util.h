/*
 * carmen_rrt_util.h
 *
 *  Created on: 12/11/2012
 *      Author: romulo
 */

#ifndef CARMEN_RRT_UTIL_H_
#define CARMEN_RRT_UTIL_H_

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

void carmen_rrt_load_lane_points(char *rddf_path);
void carmen_rrt_build_lane_map(carmen_map_t *lane_map, carmen_map_config_t config, double lane_width);


#ifdef __cplusplus
}
#endif



#endif /* CARMEN_RRT_UTIL_H_ */
