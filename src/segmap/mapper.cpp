
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
create_map(GridMap &map, DatasetInterface &dataset)
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);

    /*
	pcl::visualization::PCLVisualizer viewer("CloudViewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.removeAllPointClouds();
	viewer.addCoordinateSystem(2);
    */

	Matrix<double, 4, 4> vel2car = dataset.transform_vel2car();

	deque<string> cloud_names;
	int pause_viewer = 1;
	int step = 1;

	for (int i = 0; i < dataset.data.size(); i += step)
	{
		if (fabs(dataset.data[i].v) < 0.1)
			continue;

		Pose2d pose = dataset.data[i].pose;

		//printf("Step %d car pose: %lf %lf %lf\n", i, pose.x, pose.y, pose.th);

		dataset.load_fused_pointcloud_and_camera(i, cloud, 1);
		transform_pointcloud(cloud, transformed_cloud, pose, vel2car, dataset.data[i].v, dataset.data[i].phi);
		//transformed_cloud = cloud;

		map.reload(pose.x, pose.y);

		for (int j = 0; j < transformed_cloud->size(); j++)
			map.add_point(transformed_cloud->at(j));

		if (map._map_type == GridMapTile::TYPE_SEMANTIC)
			colorize_cloud_according_to_segmentation(transformed_cloud);

        /*
		char *cloud_name = (char *) calloc (32, sizeof(char));
		sprintf(cloud_name, "cloud%d", i);
		//viewer.removeAllPointClouds();
		viewer.addPointCloud(transformed_cloud, cloud_name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
		cloud_names.push_back(cloud_name);

		if (cloud_names.size() >= 1000)
		{
			viewer.removePointCloud(cloud_names[0]);
			cloud_names.pop_front();
		}
		*/

		Mat map_img = map.to_image().clone();
		draw_pose(map, map_img, pose, Scalar(0, 255, 0));
		imshow("viewer", map_img);

		char c = ' ';
		while (1)
		{
			//viewer.spinOnce();
			c = waitKey(5);

			if (c == 's')
				pause_viewer = !pause_viewer;
			if (!pause_viewer || (pause_viewer && c == 'n'))

				break;
			if (c == 'r')
			{
				printf("Reinitializing\n");
				i = 0;
			}
			if (c == 'f')
				step *= 2;
			if (c == 'g')
			{
				step /= 2;
				if (step < 1) step = 1;
			}
		} 

//		if (i > 500 && i < dataset.data.size() - 1000)
//			i = dataset.data.size() - 1000;
	}

	//waitKey(-1);
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Error: Use %s <log data directory>\n", argv[0]));

	char dataset_name[256];
	char map_name[256];

	sprintf(dataset_name, "/dados/data/%s", argv[1]);
	sprintf(map_name, "/dados/maps/map_%s", argv[1]);
	printf("dataset_name: %s\n", dataset_name);
	printf("map_name: %s\n", map_name);

	DatasetInterface *dataset;
    dataset = new DatasetCarmen(dataset_name, 1);

	char cmd[256];
	sprintf(cmd, "rm -rf %s && mkdir %s", map_name, map_name);
	system(cmd);

	GridMap map(map_name, 50., 50., 0.2, GridMapTile::TYPE_SEMANTIC, 1);
	create_map(map, *dataset);

	printf("Done\n");
	return 0;
}


