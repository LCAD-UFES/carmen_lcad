#ifndef UDATMO_MUNKRES_H
#define UDATMO_MUNKRES_H


#include <opencv2/opencv.hpp>


namespace udatmo
{

extern const int STAR;

cv::Mat munkres(const cv::Mat &data, double roof);


} // namespace udatmo


#endif
