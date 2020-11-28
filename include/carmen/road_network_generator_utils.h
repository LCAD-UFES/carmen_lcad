#ifndef ROAD_NETWORK_UTILS_H_
#define ROAD_NETWORK_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/route_planner_interface.h>
#include <string.h>
#include <dirent.h>
#include <GL/glut.h>
#include <carmen/rddf_util.h>
#include <carmen/carmen_gps.h>
#include <carmen/gps_xyz_interface.h>
#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <queue>
#include <list>
#include <utility>

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

using namespace std;
using namespace cv;


typedef struct
{
	int nearby_lane_id;
	int initial_node_id;
	int nearby_lane_size;
} nearby_lane_t;


#ifndef EDGE_TYPE_
#define EDGE_TYPE_
// These structs are needed in a C library (route_planner_interface.h) but this is a C++ library

typedef struct
{
	int u;
	int v;
	int u_ref;
	int v_ref;
	double cost;
} edge_t;

#define DISABLING_COST	1.0e100

typedef struct
{
	int id;
	carmen_position_t start;
	carmen_position_t end;
	edge_t edge;
	int status;		/* enabled:1 , disabled:0 */
} route_t;

#endif


typedef struct
{
	edge_t edge_a;
	edge_t edge_b;
} crossroad_t;


typedef struct
{
	int id;
	int id_ref;
	int lane_id;
	char type; //'m' for merge node, 'f' for fork node, 'e' for end of road node, 'i' begin of road node, 'n' for normal node
	double lon;
	double lat;
	carmen_rddf_waypoint rddf_point;
	vector <edge_t> edges;
	vector <edge_t> edges_in;
	vector <nearby_lane_t> nearby_lanes;
	vector <crossroad_t> nearby_crossroads;
	int traffic_restrictions; 	// LANE_LEFT_WIDTH | LANE_RIGHT_WIDTH | LEFT_MARKING | RIGHT_MARKING | LEVEL | YIELD | BIFURCATION
								//     6 bits      |      6 bits      |  3 bits enum |  3 bits enum  | 2 bits| 1 bit |   1 bit
	int back_joint_node_id;
	int next_joint_node_id;
} node_t;


typedef struct
{
	vector <node_t> nodes;
	vector <edge_t> edges;
} graph_t;


const double EPS = 1E-9;


struct point_t
{
    double x, y;

    bool operator<(const point_t& p) const
    {
        return x < p.x - EPS || (abs(x - p.x) < EPS && y < p.y - EPS);
    }
};
typedef struct point_t point_t;


struct line_t {
    double a, b, c;

    line_t() {}
    line_t(point_t p, point_t q)
    {
        a = p.y - q.y;
        b = q.x - p.x;
        c = -a * p.x - b * p.y;
        norm();
    }

    void norm()
    {
        double z = sqrt(a * a + b * b);
        if (abs(z) > EPS)
            a /= z, b /= z, c /= z;
    }

    double dist(point_t p) const { return a * p.x + b * p.y + c; }
};
typedef struct line_t line_t;


vector<string> get_files_from_rddf_list(char *rddf_list);
void load_rddfs (vector<string> files, vector< vector<carmen_rddf_waypoint> > &rddfs);
// void build_nearby_lanes(graph_t &graph, double nearby_lane_range, vector< vector<int> > nearby_indexes);
void process_crossroads_around_nodes(graph_t &graph, double nearby_lane_range);
void build_nearby_lanes(graph_t &graph, double nearby_lane_range, vector< vector<int> > nearby_indexes, unsigned int qtd_lanes);
double euclidean_distance(double x1, double y1, double x2, double y2);
void convert_utm_to_lat_long (carmen_point_t pose, Gdc_Coord_3d &lat_long_coordinate);
graph_t build_lane_graph (graph_t lane_graph, graph_t &graph);
vector<edge_t> build_graph_edges (vector<node_t> graph);
graph_t build_graph(vector<string> files, graph_t graph, vector< vector<carmen_rddf_waypoint> > rddfs, vector< vector<int> > &nearby_indexes, char* option);
FILE *open_graph_file(char* graph_file, string option);
graph_t read_graph_file (FILE *f_graph);
graph_t read_lane_graph_file (FILE *f_graph);
void save_graph_to_file(graph_t graph, FILE *f_graph);
void save_lane_graph_to_file(graph_t lane_graph, FILE *f_graph);
void save_graph_to_gpx (graph_t graph);


#endif /* GRAPH_UTILS_H_ */
