
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
colorize_cloud_according_to_segmentation(PointCloud<PointXYZRGB>::Ptr cloud)
{
	CityScapesColorMap colormap;
	for (int i = 0; i < cloud->size(); i++)
	{
		Scalar color = colormap.color(cloud->at(i).r);
		cloud->at(i).r = color[2];
		cloud->at(i).g = color[1];
		cloud->at(i).b = color[0];
	}
}


void
create_map(GridMap &map, PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr transformed_cloud, DatasetInterface &dataset)
{
	//pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	//viewer.setBackgroundColor(.5, .5, .5);
	//viewer.removeAllPointClouds();
	//viewer.addCoordinateSystem(10);

	Matrix<double, 4, 4> vel2car = dataset.transform_vel2car();

	deque<string> cloud_names;
	int step = 1;

	for (int i = 300; i < dataset.data.size(); i += 1)
	{
		Pose2d pose = dataset.data[i].pose;

		printf("Step %d car pose: %lf %lf %lf\n", i, pose.x, pose.y, pose.th);

		dataset.load_fused_pointcloud_and_camera(i, cloud, 1);
		transform_pointcloud(cloud, transformed_cloud, pose, vel2car, dataset.data[i].v, dataset.data[i].phi);
		//transformed_cloud = cloud;

		map.reload(pose.x, pose.y);

		for (int j = 0; j < transformed_cloud->size(); j++)
			map.add_point(transformed_cloud->at(j));

		//if (map._map_type == GridMapTile::TYPE_SEMANTIC)
		//	colorize_cloud_according_to_segmentation(transformed_cloud);
        //
		//char *cloud_name = (char *) calloc (32, sizeof(char));
		//sprintf(cloud_name, "cloud%d", i);
		////viewer.removeAllPointClouds();
		//viewer.addPointCloud(transformed_cloud, cloud_name);
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
		//cloud_names.push_back(cloud_name);
		//
		//if (cloud_names.size() >= 50)
		//{
		//	viewer.removePointCloud(cloud_names[0]);
		//	cloud_names.pop_front();
		//}

		Mat map_img = map.to_image().clone();
		draw_pose(map, map_img, pose, Scalar(0, 255, 0));
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
        dataset = new DatasetCarmen("/dados/data/data_log_volta_da_ufes-20180112-2.txt/", 1);
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

	DatasetInterface *dataset;

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);

	dataset = create_dataset(dataset_name);

	char *map_name = "/dados/maps/maps_log_volta_da_ufes-20180112-2.txt";
	char cmd[256];

	sprintf(cmd, "rm -rf %s && mkdir %s", map_name, map_name);
	system(cmd);
	GridMap map(map_name, 50., 50., 0.2, GridMapTile::TYPE_SEMANTIC, 1);
	create_map(map, cloud, transformed_cloud, *dataset);
	printf("Done\n");

	return 0;
}


