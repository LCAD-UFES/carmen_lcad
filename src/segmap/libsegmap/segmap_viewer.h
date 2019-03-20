
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
draw_pose(GridMap &map, Mat &map_img, Pose2d &p, Scalar color);

void
draw_pose(GridMap &map, Mat &map_img, Matrix<double, 4, 4> &pose, Scalar color);

void
draw_poses(GridMap &map, Mat &map_img, vector<Matrix<double, 4, 4>> &poses, Scalar color);

void
draw_particle(Mat &map_img, Pose2d &p, GridMap &map, Scalar color);

void
draw_pointcloud(Mat &m, PointCloud<PointXYZRGB>::Ptr transformed_cloud,
		GridMap &map, int paint_points = 1, Scalar color = Scalar(0,0,0));

void
view(ParticleFilter &pf, GridMap &map, Pose2d current_pose,
	PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr transformed_cloud, Matrix<double, 4, 4> *vel2car,
	double v, double phi, Mat *pf_view_img);

void
colorize_cloud_according_to_segmentation(PointCloud<PointXYZRGB>::Ptr cloud);

class PointCloudViewer
{
public:
	PointCloudViewer(float point_size = 1, float back_red = 0.5, float back_green = 0.5, float back_blue = 0.5);
	~PointCloudViewer();

	void show(PointCloud<PointXYZRGB>::Ptr cloud, double r = -1., double g = -1., double b = -1.);
	void show(Mat &img, char *name, int resize_to_width=-1);
	void loop();
	void clear();

protected:
	pcl::visualization::PCLVisualizer *_cloud_viewer;

	// background color
	float _br, _bg, _bb;
	int _point_size;
	int _n_clouds;
	bool _pause_viewer;
	bool _img_visible;

};

#endif
