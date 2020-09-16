
#ifndef ROAD_MAPPER_UTILS_H_
#define ROAD_MAPPER_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/global_graphics.h>
#include <carmen/road_mapper.h>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

cv::Mat rotate(cv::Mat src, cv::Point pt, double angle);
void remission_map_to_image(carmen_map_p map, cv::Mat *remission_map_img, int channels);
void offline_map_to_image(carmen_map_p map, cv::Mat *offline_map_img, int channels);
void road_map_to_image(carmen_map_p map, cv::Mat *road_map_img);
void road_map_to_image_black_and_white(carmen_map_p map, cv::Mat *road_map_img, const int class_bits);
int global_pos_on_map_q4(carmen_point_t global_pos, carmen_map_p *maps, int maps_size);
int maps_has_same_origin(carmen_map_p map1, carmen_map_p map2);
carmen_map_p alloc_map_pointer(void);
void free_map_pointer(carmen_map_p map);

#ifdef __cplusplus
}
#endif

#endif /* ROAD_MAPPER_UTILS_H_ */

