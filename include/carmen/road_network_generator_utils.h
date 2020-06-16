#ifndef ROAD_NETWORK_UTILS_H_
#define ROAD_NETWORK_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
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
#include <GL/glut.h>
#include <carmen/rddf_util.h>
#include <carmen/carmen_gps.h>
#include <carmen/gps_xyz_interface.h>

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


typedef struct
{
	int p_ini;
	int p_sec;
	int p_penul;
	int p_last;
} neaby_lane_t;


typedef struct
{
	int u;
	int v;
	double cost;
	vector <carmen_rddf_waypoint> rddf_point; //optional. Used only for lane level graph
} edge_t;


typedef struct
{
	int id;
	char type;
	double lon;
	double lat;
	carmen_rddf_waypoint rddf_point;
	vector <edge_t> edges;
	vector<neaby_lane_t> nearby_lanes;
} node_t;


typedef struct
{
	vector <node_t> nodes;
	vector <edge_t> edges;
} graph_t;


float convert_world_coordinate_image_coordinate_to_image_coordinate(double world_point, double world_origin, double map_resolution);
vector<string> get_files_from_rddf_list(char *rddf_list);
void load_rddfs (vector<string> files, vector< vector<carmen_rddf_waypoint> > &rddfs);
graph_t build_nearby_lanes (graph_t graph, double nearby_lane_range, char *option);
graph_t build_lane_graph (graph_t lane_graph, graph_t graph, char *option);
double euclidean_distance(double x1, double y1, double x2, double y2);
void convert_utm_to_lat_long (carmen_point_t pose, Gdc_Coord_3d &lat_long_coordinate);
graph_t build_graph(graph_t graph, vector< vector<carmen_rddf_waypoint> > rddfs, char* option);
graph_t read_graph_file (char* graph_file);
void save_graph_to_file(graph_t graph, char* graph_file);
void save_graph_to_gpx (graph_t graph);


#ifdef __cplusplus
}
#endif

#endif /* GRAPH_UTILS_H_ */
