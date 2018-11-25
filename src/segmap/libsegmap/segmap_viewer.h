
#ifndef _SEGMAP_LIBSEGMAP_SEGMAP_VIEWER_H_
#define _SEGMAP_LIBSEGMAP_SEGMAP_VIEWER_H_

#include <opencv/cv.hpp>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "segmap_grid_map.h"
#include "segmap_pose2d.h"
#include "segmap_util.h"
#include "segmap_particle_filter.h"

using namespace Eigen;
using namespace pcl;
using namespace std;
using namespace cv;

void
draw_poses(GridMap &map, Mat &map_img, vector<Matrix<double, 4, 4>> &poses);


void
draw_particle(Mat &map_img, Pose2d &p, GridMap &map, Scalar color);


void
draw_pointcloud(Mat &m, PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map, int highlight_points=1);


void
view(ParticleFilter &pf, GridMap &map, vector<Matrix<double, 4, 4>> &poses, Pose2d current_pose,
	PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr transformed_cloud);


#endif
