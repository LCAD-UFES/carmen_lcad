#ifndef ROUTE_PLANNER_UTILS_H_
#define ROUTE_PLANNER_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/rddf_interface.h>
#include <iostream>
#include <algorithm>
#include <list>
#include <string>
#include <queue>
#include <list>
#include <utility>
#include <string.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <GL/glut.h>
//#include "../rddf/rddf_util.h"
#include <carmen/rddf_messages.h>
#include <carmen/rddf_util.h>
#include <carmen/road_network_generator_utils.h>
#include <carmen/carmen_gps.h>
#include <carmen/gps_xyz_interface.h>
#include <Python.h>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/flann/miniflann.hpp>
//#include <opencv2/tracking.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define new_node (t_adjacent_list*)malloc(sizeof(t_adjacent_list))

using namespace std;
using namespace cv;

struct type_adjacent_list{
    int vertex;
    double cost;
    struct type_adjacent_list *prox;
};
typedef struct type_adjacent_list t_adjacent_list;


struct type_a_star_utils{
    int *g_score;
    int *distancia;
    int *anterior;
    int *closed_set;
    int *open_set;
    int *fechado;
    int anterior_size;
    int path_g_n;
    int closed_set_size;
    int open_set_size;
};
typedef struct type_a_star_utils t_a_star_utils;


typedef struct{
    char v;
    int vertex;
    int x;
    int y;
}t_coords;

void call_osmnx_python_func (Gdc_Coord_3d origin_gdc, Gdc_Coord_3d destination_gdc);
int* alloc_array (int graph_size);
float* alloc_array_float (int graph_size);
int** alloc_matrix (int graph_size);
t_a_star_utils add_vertex_to_closed_set(t_a_star_utils r, int vertex_id);
t_a_star_utils add_vertex_to_open_set(t_a_star_utils r, int vertex_id);
float h_n_manh (int vertex_id, int vertex_end, t_graph graph);
float h_n_eucl (int vertex_id, int vertex_end, t_graph graph);
t_a_star_utils a_star(t_adjacent_list** adjacent_list, t_graph graph, int graph_size, int vertex_ini, int vertex_end);
t_adjacent_list **add_to_list_undir(t_adjacent_list **adjacent_list, int u, int v, int w);
t_adjacent_list **create_adjacent_list(t_adjacent_list ** adjacent_list, t_graph graph);
int find_closest_point_in_graph (t_graph graph, carmen_point_t point);
//carmen_point_t set_destination(vector<carmen_annotation_t> annotations);
carmen_point_t set_destination(vector<carmen_annotation_t> annotations, char *goal);
void get_annotation_from_rddf(char *carmen_annotation_filename, vector<carmen_annotation_t> &annotations);

#ifdef __cplusplus
}
#endif

#endif /* ROUTE_PLANNER_H_ */
