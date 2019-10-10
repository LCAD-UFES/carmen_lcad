#ifndef ROAD_MAP_PATH_PLANNING_UTILS_H_
#define ROAD_MAP_PATH_PLANNING_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/carmen_gps.h>
#include <iostream>
#include <list>
#include <string>
#include <queue>
#include <list>
#include <utility>
#include <string.h>
#include <dirent.h>
#include <string>
#include <vector>
//#include <GL/glut.h>
#include "../rddf/rddf_util.h"

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

struct type_edge
{
	int u;
	int v;
};
typedef type_edge t_edge;


struct type_node
{
	int node_id;
	double lon;
	double lat;
	carmen_position_t point;
};
typedef type_node t_node;


struct type_graph
{
	int qtd_nodes;
	int qtd_edges;
	t_node* nodes;
	t_edge* edges;
};
typedef type_graph t_graph;


struct type_osmnx
{
	char *locale;
	char *city;
	char *country;
	char *graph_type;
	char *plot;
	string python_command;
};
typedef type_osmnx t_osmnx;


struct type_route
{
	int route_size;
	int *route;
};
typedef type_route t_route;


t_graph read_graph_file(char *graph_filename);
void process_graph (string python_command);

#ifdef __cplusplus
}
#endif

#endif
