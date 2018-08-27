
#ifndef RDDF_GRAPH_UTILS_H_
#define RDDF_GRAPH_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <iostream>
#include <string>
#include <queue>
#include <string.h>
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

#define LOCAL_MAP_SIZE (210)
#define GLOBAL_MAP_SIZE (1800)


typedef struct
{
	carmen_position_t point;
	carmen_position_t point_world_origin;

} open_set_t;

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


using namespace std;

cv::Mat rotate(cv::Mat src, cv::Point pt, double angle);
void road_map_to_image(carmen_map_p map, cv::Mat *road_map_img);
void already_visited_to_image(carmen_map_p map, cv::Mat *road_map_img);
void draw_lines (carmen_map_p map, cv::Mat *image);
void show_road_map(carmen_map_p map, int x, int y);
void show_already_visited(carmen_map_p map);
bool point_is_lane_center (carmen_map_p map, int x, int y);
bool point_is_in_map(carmen_map_p map, int x, int y);
bool point_is_already_visited(carmen_map_p already_visited, int x, int y);
void get_new_currents_and_x_y(carmen_position_t *current, int *x, int *y);
void get_new_map_block(char *folder, char map_type, carmen_map_p map, carmen_point_t pose, int op);
bool check_limits_of_central_road_map(int x, int y);
bool map_is_not_in_queue(carmen_point_t map_origin, vector <carmen_point_t> &map_queue);
bool get_neighbour(carmen_position_t *neighbour, carmen_position_t *current, carmen_map_p already_visited, carmen_map_p map, vector<carmen_position_t> &open_set, vector <carmen_point_t> &map_queue, char *road_map_folder);
rddf_graph_t * add_point_to_graph_branch(carmen_map_p map, rddf_graph_t * graph, int x, int y, int branch_node);
rddf_graph_t * add_point_to_graph(carmen_map_p map, rddf_graph_t *graph, int x, int y);
rddf_graph_t * A_star(rddf_graph_t *graph, int x, int y, carmen_map_p map, carmen_map_p already_visited, vector <carmen_point_t> &map_queue, char *road_map_folder);
void write_graph_for_gnuplot (rddf_graph_t * graph);
void write_graph_on_file(rddf_graph_t *graph);
void generate_road_map_graph(carmen_map_p map, char *road_map_folder, bool view_graph_construction);

#ifdef __cplusplus
}
#endif

#endif /* RDDF_GRAPH_UTILS_H_ */

