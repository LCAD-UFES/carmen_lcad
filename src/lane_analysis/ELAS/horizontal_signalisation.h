#ifndef ELAS_HORIZONTAL_SIGNALISATION_H
#define ELAS_HORIZONTAL_SIGNALISATION_H

#include <opencv2/opencv.hpp>
#include "utils/common.h"
#include "feature_map_generation.h"

#include "SinalizacaoHorizontal.h"


namespace ELAS {

	struct crosswalks { bool status; cv::Mat1b mask; };
	struct road_sign { int id; std::vector<cv::Point> region; };
	struct road_signs { int n; std::vector<road_sign> signs; vector<SinalizacaoHorizontal::Blob> road_signs_blobs; };

	void crosswalk_detection(const feature_maps * _feature_maps, ConfigXML * _cfg, crosswalks * out);
	void road_signs_classification(const pre_processed * _pre_processed, const feature_maps * _feature_maps, HoughDoMeio * hough_anterior, ConfigXML * _cfg, road_signs * out);
	void pavement_markings_removal(feature_maps * _feature_maps, crosswalks * _crosswalks, road_signs * _road_signs, ConfigXML * _cfg);

}


#endif // ELAS_HORIZONTAL_SIGNALISATION_H
