#ifndef LASLAM_FEATURES_H_
#define LASLAM_FEATURES_H_

#include <opencv/cv.h>
#include <vector>

std::vector<cv::KeyPoint> extract_features(const IplImage* rightImg);
std::vector<cv::KeyPoint> extract_matched_features(const IplImage* leftImg, const IplImage* rightImg);

#endif /* LASLAM_FEATURES_H_ */
