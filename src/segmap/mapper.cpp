
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <deque>
#include <string>
#include <random>
#include <iostream>
#include <Eigen/Geometry>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "libsegmap/segmap_car_config.h"
#include "libsegmap/segmap_grid_map.h"
#include "libsegmap/segmap_particle_filter.h"
#include "libsegmap/segmap_pose2d.h"
#include "libsegmap/segmap_util.h"
#include "libsegmap/segmap_dataset.h"
#include "libsegmap/segmap_viewer.h"

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;


void
create_map(GridMap &map, vector<Matrix<double, 4, 4>> &poses, PointCloud<PointXYZRGB>::Ptr cloud,
		PointCloud<PointXYZRGB>::Ptr transformed_cloud, DatasetInterface &dataset)
{
	//pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	//viewer.setBackgroundColor(.5, .5, .5);
	//viewer.removeAllPointClouds();
	//viewer.addCoordinateSystem(10);

	Matrix<double, 4, 4> vel2car = dataset.transform_vel2car();
	Matrix<double, 4, 4> car2world;

	deque<string> cloud_names;
	int step = 1;

	for (int i = 0; i < poses.size(); i += 1)
	{
		dataset.load_fused_pointcloud_and_camera(i, cloud, 1);

		car2world = poses[i] * vel2car;
		transformPointCloud(*cloud, *transformed_cloud, car2world);
		//transformed_cloud = cloud;

		map.reload(poses[i](0, 3), poses[i](1, 3));
		printf("car pose: %lf %lf\n", poses[i](0, 3), poses[i](1, 3));

		for (int i = 0; i < transformed_cloud->size(); i++)
			map.add_point(transformed_cloud->at(i));

		//char *cloud_name = (char *) calloc (32, sizeof(char));
		//sprintf(cloud_name, "cloud%d", i);
		////viewer.removeAllPointClouds();
		//viewer.addPointCloud(transformed_cloud, cloud_name);
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
		//cloud_names.push_back(cloud_name);

		//if (cloud_names.size() >= 10)
		//{
		//	viewer.removePointCloud(cloud_names[0]);
		//	cloud_names.pop_front();
		//}

		Mat map_img = map.to_image().clone();
		draw_pose(map, map_img, poses[i], Scalar(0, 255, 0));
		imshow("viewer", map_img);

		char c = ' ';
		while (1)
		{
			//viewer.spinOnce();
			c = waitKey(5);

			if (c == 's')
				step = !step;
			if (!step || (step && c == 'n'))

				break;
			if (c == 'r')
			{
				printf("Reinitializing\n");
				i = 0;
			}
		}
	}

	//waitKey(-1);
}


DatasetInterface*
create_dataset(char *dataset_name)
{
	DatasetInterface *dataset;

	if (!strcmp(dataset_name, "carmen"))
		//dataset = new DatasetCarmen(960, 1280, "/dados/data/data_log_estacionamentos-20181130.txt/", 0);
		dataset = new DatasetCarmen(480, 640, "/dados/data/data_20180112/", 1);
	else if (!strcmp(dataset_name, "kitti"))
		dataset = new DatasetKitti("/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/", 1);
	else
		exit(printf("Error: dataset '%s' not found.\n", dataset_name));

	return dataset;
}


int
main(int argc, char **argv)
{
	char *dataset_name = "carmen";

	if (argc > 1)
		dataset_name = argv[1];

	vector<double> times;
	vector<Matrix<double, 4, 4>> poses;
	vector<pair<double, double>> odom;
	DatasetInterface *dataset;

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);

	dataset = create_dataset(dataset_name);
	dataset->load_data(times, poses, odom);

	char *map_name = "/dados/maps/maps_20180112-2/";
	char cmd[256];

	sprintf(cmd, "rm -rf %s && mkdir %s", map_name, map_name);
	system(cmd);
	GridMap map(map_name, 50., 50., 0.2, GridMapTile::TYPE_VISUAL);
	create_map(map, poses, cloud, transformed_cloud, *dataset);
	printf("Done\n");

	return 0;
}


