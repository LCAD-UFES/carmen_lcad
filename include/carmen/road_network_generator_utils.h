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


//struct type_lane{
//    //mandatory attributes:
//    int source_node;
//    int dest_node;
//    float road_meters;
//    float max_velocity;
//    float mean_velocity;
//    int qtd_lanes;
//};
//typedef type_lane t_lane;
//
//struct type_edge{
//    //mandatory attributes:
//    int edge_id;
//    int source_node;
//    int dest_node;
//    float road_meters;
//    float max_velocity;
//    float mean_velocity;
//    int qtd_lanes;
//    carmen_map_t maps_from_this_edge;
//    vector <t_lane> edge_lanes;
//    //variable_attributes:
//    vector <t_lane> overtake_lanes;
//    string edge_description;
//    vector <string> annotations;
//    vector <string> flow_restrictions;
//};
//typedef type_edge t_edge;
//
//struct type_node{
//    int node_id;
//    carmen_position_t utm_pos;
//    t_edge edge;
//    int cont_out;
//    int cont_in;
//    vector <t_edge> in_edges;
//    vector <t_edge> out_edges;
//    struct type_node *prox;
//};
//typedef type_node t_graph;


struct type_nearby_lane
{
	int p_ini;
	int p_sec;
	int p_penul;
	int p_last;
};
typedef struct type_nearby_lane t_neaby_lane;


struct type_edge
{
	int u;
	int v;
	double cost;
	vector <carmen_rddf_waypoint> rddf_point; //optional. Used only for lane level graph
};
typedef struct type_edge t_edge;




struct type_node
{
	int id;
	char type;
	double lon;
	double lat;
	carmen_rddf_waypoint rddf_point;
	vector <t_edge> edges;
	vector<t_neaby_lane> nearby_lanes;
};
typedef struct type_node t_node;


struct type_graph
{
	vector <t_node> nodes;
	vector <t_edge> edges;
};
typedef struct type_graph t_graph;


float convert_world_coordinate_image_coordinate_to_image_coordinate(double world_point, double world_origin, double map_resolution);
vector<string> get_files_from_rddf_list(char *rddf_list);
void load_rddfs (vector<string> files, vector< vector<carmen_rddf_waypoint> > &rddfs);
t_graph build_nearby_lanes (t_graph graph, double nearby_lane_range, char *option);
t_graph build_lane_graph (t_graph lane_graph, t_graph graph, char *option);
double euclidean_distance(double x1, double y1, double x2, double y2);
void convert_utm_to_lat_long (carmen_point_t pose, Gdc_Coord_3d &lat_long_coordinate);
t_graph build_graph(t_graph graph, vector< vector<carmen_rddf_waypoint> > rddfs, char* option);
t_graph read_graph_file (char* graph_file);
void save_graph_to_file(t_graph graph, char* graph_file);
void save_graph_to_gpx (t_graph graph);


#ifdef __cplusplus
}
#endif

#endif /* GRAPH_UTILS_H_ */
