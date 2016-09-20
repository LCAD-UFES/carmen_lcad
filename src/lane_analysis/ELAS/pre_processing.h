#ifndef ELAS_PRE_PROCESSING_H
#define ELAS_PRE_PROCESSING_H

#include <opencv2/opencv.hpp>
#include "utils/common.h"

#define MASK_IPM_ERODE_SIZE -8


namespace ELAS {

	struct pre_processed {
		cv::Mat1b grayFrame, grayFrameRoi, grayFrameRoiIPM;
		cv::Mat3b colorFrame, colorFrameRoiIPM;
		cv::Mat1b maskIPM;
	};

	void pre_processing(const cv::Mat3b & original_frame, const ConfigXML * _cfg, pre_processed * out);

	cv::Mat1b toGrayscale(const cv::Mat3b & frame);
	cv::Mat1b toIPM(const cv::Mat1b & frameRoi, IPM * ipm);
	cv::Mat3b toIPM(const cv::Mat3b & frameRoi, IPM * ipm);

	std::vector<cv::Point> get_vertices_ipm(const ConfigXML * _cfg);
	cv::Mat1b get_mask_ipm(int erode_size, const ConfigXML * _cfg);

}


#endif // ELAS_PRE_PROCESSING_H
