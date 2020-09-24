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
#include <time.h>
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

using namespace std;
using namespace cv;


struct adjacent_list
{
    int vertex;
    double cost;
    struct adjacent_list *prox;
};

typedef struct adjacent_list adjacent_list_t;


typedef struct
{
    double *g_score;
    int *distancia;
    int *anterior;
    int *closed_set;
    int *open_set;
    int *fechado;
    int anterior_size;
    int path_g_n;
    int closed_set_size;
    int open_set_size;
    bool found_path;
} a_star_utils_t;


typedef struct
{
    char v;
    int vertex;
    int x;
    int y;
} coords_t;


typedef struct
{
	int u;
	int v;
} osm_edges_t;;


typedef struct
{
	int id;
	double lat;
	double lon;
	carmen_point_t utm_point;
	vector <osm_edges_t> edges;
} osm_graph_t;


vector <int> read_osm_route_from_file(string route_filename);
vector <osm_graph_t> read_osm_graph_file(string graph_filename);
void call_osmnx_python_func (Gdc_Coord_3d origin_gdc, Gdc_Coord_3d destination_gdc);
int *alloc_array (int graph_size);
a_star_utils_t add_vertex_to_closed_set(a_star_utils_t r, int vertex_id);
//void add_vertex_to_open_set(a_star_utils_t r, int vertex_id);
float h_n_manh (int vertex_id, int vertex_end, graph_t graph);
float h_n_eucl (int vertex_id, int vertex_end, graph_t graph);
a_star_utils_t a_star_k_neighbours(graph_t graph, int k, int vertex_ini, int vertex_end);
int find_closest_point_in_graph (graph_t graph, carmen_point_t point);
//carmen_point_t set_destination(vector<carmen_annotation_t> annotations);
carmen_point_t set_destination(vector<carmen_annotation_t> annotations, char *goal);
void get_annotation_from_rddf(char *carmen_annotation_filename, vector<carmen_annotation_t> &annotations);
void get_graph_from_file(graph_t *graph, graph_t *lane_graph, char *filename);
void get_route_list(graph_t graph, carmen_position_t center, double range, int *number_of_routes, route_t *routes[]);

#ifdef __cplusplus
}
#endif

#endif /* ROUTE_PLANNER_H_ */
