
#ifndef ROAD_MAPPER_UTILS_H_
#define ROAD_MAPPER_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include "rddf_graph.h"

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

typedef struct rddf_graph_node
{
	int x;
	int y;
	rddf_graph_node *prox;

} rddf_graph_node;

using namespace std;

cv::Mat rotate(cv::Mat src, cv::Point pt, double angle);
void road_map_to_image(carmen_map_p map, cv::Mat *road_map_img);
void road_map_to_image_black_and_white(carmen_map_p map, cv::Mat *road_map_img, const int class_bits);
void road_map_find_center(carmen_map_p map);
int **alloc_matrix(int r, int c);
//t_list *create_list();
//t_list *insert_in_list (t_list *l, t_point p);
void print_map_in_terminal (carmen_map_p map);
void display_graph_in_image(carmen_map_p map, vector<rddf_graph_node*> &closed_set);
void print_list (rddf_graph_node *l);
bool road_center (carmen_map_p map, int x, int y, unsigned short *next_lane_center);
void print_open_set(std::vector<rddf_graph_node*> &open_set);
bool point_is_in_map(carmen_map_p map, int x, int y);
void expand_neighbours(carmen_map_p map, vector<rddf_graph_node*> &open_set, vector<rddf_graph_node*> &closed_set, int **already_visited);
void road_map_find_center(carmen_map_p map);

#ifdef __cplusplus
}
#endif

#endif /* ROAD_MAPPER_UTILS_H_ */

