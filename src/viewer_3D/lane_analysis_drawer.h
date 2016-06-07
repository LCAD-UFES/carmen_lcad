#ifndef LANE_ANALYSIS_DRAWER_H_
#define LANE_ANALYSIS_DRAWER_H_

#include <carmen/carmen.h>
#include <carmen/lane_analysis_interface.h>

#include <opencv2/opencv.hpp>

#include "viewer_3D.h"

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "viewer_3D.h"

struct lane_analysis_drawer {
	std::vector<carmen_vector_3D_t> left, right;
	std::vector<carmen_vector_3D_t> left_ahead, right_ahead;
};

void draw_lane_analysis(lane_analysis_drawer * lane_drawer);
void add_to_trail(carmen_elas_lane_estimation_message * message, const carmen_pose_3D_t &car_pose, lane_analysis_drawer * lane_drawer);

lane_analysis_drawer* create_lane_analysis_drawer();
void destroy_lane_analysis_drawer(lane_analysis_drawer * lane_drawer);

#endif // LANE_ANALYSIS_DRAWER_H_
