#ifndef __ELAS_VIZ_H
#define __ELAS_VIZ_H

#include <opencv2/opencv.hpp>

#include "IPM.h"
#include "../LMTDetector.h"
#include "../FaixasAdjacentes.h"

using namespace std;
using namespace cv;

struct frame_viz {
	struct { vector<Point2d> left, right; } lane_position;
	struct { int left, right; } lmt;
	struct { Point2d point_bottom, point_top, direction; double width; } lane_base;
	vector<viz_symbols> symbols;
	struct { int left, right; } adjacent_lanes;
	int lane_change;
	double lane_deviation;
	double execution_time;
	int frame_number;
	int trustworthy_height;
	double x_carro;
	bool isKalmanNull;
	int idx_frame;

	void set_lane_deviation(double x_carro, double x_base, double lane_width);
	void set_lane_base(double x_bottom, double x_top, double lane_width, const Rect &roi, IPM *ipm);
};
void render(frame_viz &data, const Mat3b &color_frame, ConfigXML *config);

#endif // __ELAS_VIZ_H