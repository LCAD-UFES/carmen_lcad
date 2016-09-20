#ifndef ELAS_ADJACENT_LANE_DETECTION_H
#define ELAS_ADJACENT_LANE_DETECTION_H

#include <opencv2/opencv.hpp>
#include "utils/common.h"
#include "utils/Helper.h"

#include "feature_map_generation.h"
#include "lane_estimation.h"
#include "lmt_classification.h"

#include "FaixasAdjacentes.h"

#define ADJACENT_LANES_BUFFER_SIZE 3


namespace ELAS {
	struct adjacent_lanes { int left, right; };
	void adjacent_lanes_detection(const feature_maps * _feature_maps, const lane_position * _lane_position, const raw_houghs * _raw_houghs, const lane_marking_type * _lmt, ConfigXML * _cfg, adjacent_lanes * out);
}


#endif // ELAS_ADJACENT_LANE_DETECTION_H
