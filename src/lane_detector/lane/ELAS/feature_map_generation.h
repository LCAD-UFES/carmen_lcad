#ifndef ELAS_FEATURE_MAP_GENERATION_H
#define ELAS_FEATURE_MAP_GENERATION_H

#include <opencv2/opencv.hpp>
#include "utils/common.h"
#include "utils/Helper.h"
#include "pre_processing.h"

#define TAU_BEGIN 10
#define TAU_END 100
#define MAP_SRF_THRES 60

#define GAUSSIAN_SIZE_IN 2.0 // smaller
#define GAUSSIAN_SIZE_OUT 2.2
#define GAUSSIAN_THRES 0.3

#define SOBEL_THRES 60

#define INTENSITY_THRES 10 // TODO: check if it is needed


namespace ELAS {

	struct feature_maps { cv::Mat1b map_srf, map_srf_ipm, map_hdog_ipm, map_vad_ipm, map_inb_ipm; };
	void feature_map_generation(const pre_processed * _pre_processed, const ConfigXML * _cfg, feature_maps * out);

	cv::Mat1b get_map_srf(const cv::Mat1b &grayFrame, int tau_begin = TAU_BEGIN, int tau_end = TAU_END, int _threshold = MAP_SRF_THRES);
	cv::Mat1b get_map_hdog_ipm(const cv::Mat1b &grayFrameRoiIPM, const cv::Mat &mask_ipm);
	cv::Mat1b get_map_vad_ipm(const cv::Mat1b &grayFrameRoiIPM, const cv::Mat &mask_ipm);
	cv::Mat1b get_map_inb_ipm(const cv::Mat1b &grayFrameRoi, const cv::Mat1b &map_srf, const ConfigXML *_cfg);

	cv::Mat1b filtroNieto(cv::Mat1b &srcGRAY, int tau_begin = TAU_BEGIN, int tau_end = TAU_END);
	cv::Mat difference_of_gaussians(const cv::Mat &frame, double gaussian_size_in = GAUSSIAN_SIZE_IN, double gaussian_size_out = GAUSSIAN_SIZE_OUT, double thres = GAUSSIAN_THRES, bool y_direction = false);

}

#endif // ELAS_FEATURE_MAP_GENERATION_H
