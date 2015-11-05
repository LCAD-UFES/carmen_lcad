/*
 * carmen_rrt_util.cpp
 *
 *  Created on: 12/11/2012
 *      Author: romulo
 */
#include "carmen_rrt_util.h"
#include "../model/global_state.h"
#include "lane.h"


void carmen_rrt_load_lane_points(char *rddf_path)
{
	GlobalState::lane_points = Lane::get_street(rddf_path);
}




