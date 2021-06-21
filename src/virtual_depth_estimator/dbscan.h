#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <carmen/velodyne_camera_calibration.h>

using namespace std;

vector<image_cartesian>
get_biggest_cluster(vector<vector<image_cartesian>> &clusters);

vector<vector<image_cartesian>>
dbscan_compute_clusters(double d2, size_t density, const vector<image_cartesian> &points);

vector<vector<image_cartesian>>
filter_object_points_using_dbscan(vector<vector<image_cartesian>> &points_lists);

#endif