
#include <carmen/segmap_sensor_viewer.h>

using namespace pcl;
using namespace cv;


PointCloudViewer::PointCloudViewer(float point_size, float back_red, float back_green, float back_blue)
{
	// the viewer is initialized in the show method so
	// the pcl window is only created when there are
	// pointclouds to show.
	_cloud_viewer = NULL;

	_br = back_red;
	_bg = back_green;
	_bb = back_blue;

	_point_size = point_size;
	_n_clouds = 0;
	_img_visible = false;
	_pause_viewer = true;
}


PointCloudViewer::~PointCloudViewer()
{
	if (_cloud_viewer != NULL)
		delete(_cloud_viewer);
}


void
PointCloudViewer::show(PointCloud<PointXYZRGB>::Ptr cloud, double r, double g, double b)
{
	char cloud_name[64];

	if (_cloud_viewer == NULL)
	{
		_cloud_viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
		_cloud_viewer->setBackgroundColor(_br, _bg, _bb);
		//_cloud_viewer->addCoordinateSystem(2.);
		//_cloud_viewer->initCameraParameters();
	}

	sprintf(cloud_name, "cloud%d", _n_clouds);
	_n_clouds++;

	_cloud_viewer->addPointCloud(cloud, cloud_name);
	_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
																									_point_size,
																									cloud_name);

	if ((r >= 0) && (g >= 0) && (b >= 0))
	{
		_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
																										r, g, b,
																										cloud_name);
	}
}


void
PointCloudViewer::show(Mat &img, const char *name, int resize_to_width)
{
	if (resize_to_width > 0)
	{
		double factor = (double) resize_to_width / (double) img.cols;
		int w = img.cols * factor;
		int h = img.rows * factor;

		Mat resized(h, w, img.type());
		resize(img, resized, resized.size());
		imshow(name, resized);
	}
	else
		imshow(name, img);

	_img_visible = true;
}


void
PointCloudViewer::loop()
{
	if (_img_visible == 0)
		imshow("default_img", Mat::zeros(300, 300, CV_8UC3));

	char c = ' ';
	while (1)
	{
		if (_cloud_viewer)
			_cloud_viewer->spinOnce();

		c = waitKey(5);

		if (c == 's')
			_pause_viewer = !_pause_viewer;

		if (!_pause_viewer || (_pause_viewer && c == 'n'))
			break;
	}
}


void
PointCloudViewer::clear()
{
	if (_cloud_viewer)
		_cloud_viewer->removeAllPointClouds();

	_n_clouds = 0;
}


void
PointCloudViewer::set_camera_pose(double x, double y)
{
	if (_cloud_viewer)
	{
		_cloud_viewer->setCameraPosition(x - 5.0, y - 5.0, 10.0, 0, 0, 0, 0);
		_cloud_viewer->updateCamera();
	}
}
