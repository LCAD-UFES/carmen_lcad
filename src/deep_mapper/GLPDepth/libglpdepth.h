#ifndef LIBGLPDEPTH_H
#define LIBGLPDEPTH_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void
initialize_glpdepth();

unsigned char*
libglpdepth_process_image(cv::Mat img, int cut_param, int down_param);

#endif
