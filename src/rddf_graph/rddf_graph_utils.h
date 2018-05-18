
#ifndef ROAD_MAPPER_UTILS_H_
#define ROAD_MAPPER_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <iostream>
#include <string>
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

extern double g_x_origin;
extern double g_y_origin;
extern char *g_road_map_folder;


typedef struct
{
	int size;
	int *point;
} rddf_graph_edges_t;


typedef struct
{
	int size;
	carmen_position_t *point;
	carmen_position_t *world_coordinate;
	rddf_graph_edges_t *edge;
} rddf_graph_t;

typedef struct
{
	int size;
	rddf_graph_t *graph;
} rddf_graphs_of_map_t;

using namespace std;

cv::Mat rotate(cv::Mat src, cv::Point pt, double angle);
void road_map_to_image(carmen_map_p map, cv::Mat *road_map_img);
void road_map_to_image_black_and_white(carmen_map_p map, cv::Mat *road_map_img, const int class_bits);
//void road_map_find_center(carmen_map_p map);
int **alloc_matrix(int r, int c);
void print_map_in_terminal (carmen_map_p map);
void set_world_origin_to_road_map(carmen_map_p road_map);
void show_road_map(carmen_map_p road_map, int x, int y);
//int count_graph_nodes(rddf_graph_node *graph);
//void display_graph_in_image(carmen_map_p map, vector<rddf_graph_node*> &closed_set);
int** alloc_matrix(int r, int c);
bool point_is_lane_center (carmen_map_p map, int x, int y);
//void print_open_set(std::vector<rddf_graph_node*> &open_set);
bool point_is_in_map(carmen_map_p map, int x, int y);
rddf_graph_t * A_star(int x, int y, carmen_map_p map, int **already_visited);
//void A_star(carmen_map_p map, rddf_graph_node* p, vector<rddf_graph_node*> &closed_set, int **already_visited, int *called_expand);
void generate_road_map_graph(carmen_map_p map, std::string str_road_map_filename);
void parse_world_origin_to_road_map(string road_map_filename);

#ifdef __cplusplus
}
#endif

#endif /* ROAD_MAPPER_UTILS_H_ */

