#ifndef CAMERA_DRIVERS_PROCESS_IMAGE_H
#define CAMERA_DRIVERS_PROCESS_IMAGE_H


#ifdef __cplusplus

#include <carmen/carmen.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include "camera_drivers_messages.h"

extern "C"
{
#endif


struct tCrop { int x, y, width, height; };

void
load_undistort(char *camera_name, int undistort, int width, int height, int number_of_images, int &crop, std::vector<tCrop> &x_y_vector, cv::Mat &MapX, cv::Mat &MapY);

int
process_image(camera_message *message, int image_index, char*camera_name, int undistort, cv::Mat &src_image);


#ifdef __cplusplus
}
#endif

#endif