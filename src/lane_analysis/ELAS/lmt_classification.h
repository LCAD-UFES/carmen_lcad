#ifndef ELAS_LMT_CLASSIFICATION_H
#define ELAS_LMT_CLASSIFICATION_H

#include <opencv2/opencv.hpp>
#include "utils/common.h"
#include "utils/Helper.h"

#include "pre_processing.h"
#include "feature_map_generation.h"
#include "lane_estimation.h"

#include "LMTDetector.h"


namespace ELAS {

	struct lane_marking_type { int left, right; };

	void lmt_classification_init(ConfigXML * _cfg);
	void lmt_classification(const pre_processed * _pre_processed, const feature_maps * _feature_maps, const lane_position * _lane_position, lane_marking_type * out);

}


#endif // ELAS_LMT_CLASSIFICATION_H
