

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>


using namespace pcl;
using namespace cv;
using namespace Eigen;


int
main()
{
	char *name = "/dados/logs/a.txt.00.txt"; //optimized_20180112-2.txt";
	FILE *f = fopen(name, "r");

	if (f == NULL) exit(printf("FILE %s not found.", name));

	double x, y, th, t, x0, y0;
	char dummy[256];

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(0., 0., 1.);
	viewer.removeAllPointClouds();

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);
	Matrix<double, 4, 4> transform;

	x0 = y0 = -1;

	while (!feof(f))
	{
		fscanf(f, "%lf %lf %lf %lf %s %s %s\n", &x, &y, &th, &t, dummy, dummy, dummy);

		if (x0 == -1)
		{
			x0 = x;
			y0 = y;
		}

		sprintf(dummy, "/dados/logs/log_volta_da_ufes-20180112-2.txt_velodyne_pcd/%lf.ply", t);
		printf("loading file %s\n", dummy);
		pcl::io::loadPLYFile(dummy, *cloud);

		char *cloud_name = (char *) calloc (128, sizeof(char));
		sprintf(cloud_name, "cloud_%lf", t);

		th = -th;

		transform << cos(th), -sin(th), 0, x - x0,
				sin(th), cos(th), 0, -(y - y0),
				0, 0, 1, 0,
				0, 0, 0, 1;

		transformPointCloud(*cloud, *transformed_cloud, transform);

		for (int i = 0; i < transformed_cloud->size(); i++)
		{
			transformed_cloud->at(i).r *= 10;
			transformed_cloud->at(i).g *= 10;
			transformed_cloud->at(i).b *= 10;
		}

		viewer.addPointCloud(transformed_cloud, cloud_name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);

		char c = ' ';
		while (c != 'n')
		{
			imshow("map", Mat::zeros(300, 300, CV_8UC3));
			viewer.spinOnce();
			c = waitKey(5);
		}
	}

	fclose(f);
	return 0;
}
