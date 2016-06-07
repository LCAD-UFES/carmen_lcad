#ifndef ELAS_LANE_ESTIMATION_H
#define ELAS_LANE_ESTIMATION_H

#include <opencv2/opencv.hpp>
#include "utils/common.h"
#include "utils/Helper.h"

#include "feature_map_generation.h"
#include "horizontal_signalisation.h"

// TODO: remove dependence of old modules
// old modules
#include "Houghs.h"
#include "AnaliseDasHoughs.h"
#include "FiltroDeParticulasHough.h"

#define DISPLAY_PARTICLE_FILTER true

#define HOUGH_ONLY false
#define BUFFER_HOUGHS_SIZE 10
#define BUFFER_MIN_LENGTH 50
#define NUM_PARTICLES 500
#define USE_TRUSTWORTHY_HEIGHT false


namespace ELAS {

	// structs
	struct lane_position {
		std::vector<cv::Point2d> left, right;
		HoughDoMeio * lane_base;
		bool is_hough_only;
		double center_deviation;
		int trustworthy_height;
		bool is_kalman_disabled;
	};
	struct raw_houghs { std::vector<HoughLine> ego_lane, adjacent_lanes; };
	struct lane_change { bool status; };
	struct LaneMeasurement { HoughLine left, right; };

	void lane_position_estimation(const pre_processed * _pre_processed, const feature_maps * _feature_maps, const road_signs * _road_signs, ConfigXML * _cfg, lane_position * _out_lane_position, lane_change * _out_lane_change, raw_houghs * _out_raw_houghs);
	void lane_estimation_init(ConfigXML * _cfg);
	bool buffer_mechanism(vector<HoughLine> & houghs_X, ConfigXML * _cfg);
	// void lane_measurement_generation(); // lane measurement generation
	void lane_center_deviation(lane_position * _lane_position, ConfigXML * _cfg);

	// viz
	void viz_lane_measurement_generation(const Mat3b & _colorFrame, HoughLine &esq, HoughLine &dir);
	
}


#endif // ELAS_LANE_ESTIMATION_H
