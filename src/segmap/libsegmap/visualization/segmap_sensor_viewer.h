
#ifndef __POINTCLOUD_VIEWER_H__
#define __POINTCLOUD_VIEWER_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv/cv.hpp>


class PointCloudViewer
{
public:
	PointCloudViewer(float point_size = 1, float back_red = 0.5, float back_green = 0.5, float back_blue = 0.5);
	~PointCloudViewer();

	void set_step(int step_mode) { _pause_viewer = step_mode; }
	void show(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double r = -1., double g = -1., double b = -1.);
	void show(cv::Mat &img, const char *name, int resize_to_width=-1);
	void loop();
	void clear();
	void set_camera_pose(double x, double y);

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
