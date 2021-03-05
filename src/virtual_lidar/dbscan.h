#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <carmen/velodyne_camera_calibration.h>

using namespace std;

vector<vector<image_cartesian>>
dbscan_compute_clusters(double d2, size_t density, const vector<image_cartesian> points);

#endif