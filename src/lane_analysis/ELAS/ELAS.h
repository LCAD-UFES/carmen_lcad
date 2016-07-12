#ifndef __ELAS_LANE_ANALYSIS_H
#define __ELAS_LANE_ANALYSIS_H

#include <memory>
#include <opencv2/opencv.hpp>

// utils
#include "utils/common.h"
#include "utils/HelperXML.h"
#include "utils/Viz.h"

// modules
#include "pre_processing.h"
#include "feature_map_generation.h"
#include "horizontal_signalisation.h"
#include "lane_estimation.h"
#include "lmt_classification.h"
#include "adjacent_lanes.h"

#define DISPLAY_PRE_PROCESSING 			false
#define DISPLAY_FEATURE_MAPS 			false
#define DISPLAY_CROSSWALK 				false
#define DISPLAY_ROAD_SIGNS 				false
#define DISPLAY_MARKINGS_REMOVAL 		false
#define DISPLAY_LANE_POSITION			false
#define DISPLAY_LANE_CENTER_DEVIATION	false
#define DISPLAY_LANE_MARKINGS_TYPE		false
#define DISPLAY_ADJACENT_LANES			false


namespace ELAS {

struct raw_elas_message {
	struct { vector<Point2d> left, right; } lane_position;
	struct { int left, right; } lmt;
	struct { int left, right; } adjacent_lanes;
	bool lane_change; // TODO: change for int, where -1 = to left, 1 = to right and 0 = none
	double lane_deviation;
	struct { Point2d point_bottom, point_top, direction; double width; } lane_base;
	int trustworthy_height;

	bool isKalmanNull;
	double execution_time;
	double car_position_x;
	vector<road_sign> pavement_signs;
	int idx_frame;
};

// methods
void run(const cv::Mat3b & original_frame);
void init(std::string &config_fname, string & config_xml_fname);
void setIPM(cv::Size& _origSize, cv::Size& _dstSize, std::vector<cv::Point2f>& _origPoints, std::vector<cv::Point2f>& _dstPoints);
ConfigXML * getConfigXML();
void finishProgram();
void finishRun();

raw_elas_message& get_raw_message();
frame_viz get_viz_data(raw_elas_message * message);
void display(const Mat3b& frame, raw_elas_message * message, Mat3b * out = NULL);

} // end namespace ELAS


#endif // __ELAS_LANE_ANALYSIS_H
