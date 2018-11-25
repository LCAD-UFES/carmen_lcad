
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
print_poses(vector<Matrix<double, 4, 4>> &poses)
{
	Matrix<double, 4, 4> p;

	for (int i = 0; i < poses.size(); i++)
	{
		p = poses[i];

		printf("%.2lf %.2lf %.2lf\n",
				p(0, 3) / p(3, 3),
				p(1, 3) / p(3, 3),
				p(2, 3) / p(3, 3));
	}
}


Mat
segmented_image_view(Mat &m)
{
	CityScapesColorMap color_map;
	Mat view(m.rows, m.cols, CV_8UC3);

	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			int cl = m.data[3 * (i * m.cols + j)];
			Scalar color;

			if (cl < 20)
				color = color_map.color(cl);
			else
				color = Scalar(0, 0, 0);

			view.data[3 * (i * view.cols + j)] = color[0];
			view.data[3 * (i * view.cols + j) + 1] = color[1];
			view.data[3 * (i * view.cols + j) + 2] = color[2];
		}
	}

	return view;
}


void
load_fused_pointcloud_and_camera(int i, PointCloud<PointXYZRGB>::Ptr cloud, DatasetInterface &dataset, int is_segmented, int view=0)
{
	int p, x, y;
	Matrix<double, 3, 1> pixel;
	PointXYZRGB point;
	PointCloud<PointXYZRGB>::Ptr raw_cloud(new PointCloud<PointXYZRGB>);

	cloud->clear();
	dataset.load_pointcloud(i, raw_cloud);
	Mat img = dataset.load_image(i);

	Mat viewer_img;

	if (view)
	{
		if (is_segmented)
			viewer_img = segmented_image_view(img);
		else
			viewer_img = img.clone();
	}

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		point = raw_cloud->at(i);
    	pixel = dataset.transform_vel2cam(point);
    
		x = pixel(0, 0) / pixel(2, 0);
		y = pixel(1, 0) / pixel(2, 0);

		// to use fused camera and velodyne
		//if (0)
		if (point.x > 7 && x >= 0 && x < img.cols && y >= 0 && y < img.rows && point.x < 30. && point.y < 30.)
		{
			pcl::PointXYZRGB point2;

			point2.x = point.x;
			point2.y = point.y;
			point2.z = point.z;

			// colors
			p = 3 * (y * img.cols + x);
			point2.r = img.data[p + 2];
			point2.g = img.data[p + 1];
			point2.b = img.data[p + 0];

			if (view)
				circle(viewer_img, Point(x, y), 2, Scalar(0,0,255), -1);

			cloud->push_back(point2);
		}

		// to use remission
		else if (0)
		//else if (1) //point.z < 0.)
		{
			point.r *= 3;
			point.g *= 3;
			point.b *= 3;
			cloud->push_back(point);
		}
	}

	if (view)
		imshow("cam_vel_fused", viewer_img);
}


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
		load_fused_pointcloud_and_camera(i, cloud, dataset, map._map_type == GridMapTile::TYPE_SEMANTIC, 1);

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
		draw_poses(map, map_img, poses);
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


void
run_particle_filter(ParticleFilter &pf, GridMap &map, vector<Matrix<double, 4, 4>> &poses,
		vector<pair<double, double>> &odom, vector<double> &times, PointCloud<PointXYZRGB>::Ptr cloud,
		PointCloud<PointXYZRGB>::Ptr transformed_cloud, DatasetInterface &dataset)
{
	while (1)
	{
		pf.seed(time(NULL));
		pf.reset(0., 0., 0.);
		map.reload(0., 0.);

		printf("Initial particles\n");
		view(pf, map, poses, Pose2d::from_matrix(poses[0]), NULL, NULL);

		for (int i = 1; i < times.size(); i++)
		{
			printf("Step %d of %ld\n", i+1, times.size());
			Pose2d gps = Pose2d::from_matrix(poses[i]); // TODO: add noise

			printf("Prediction\n");
			pf.predict(odom[i].first, odom[i].second, times[i] - times[i-1]);
			//view(pf, map, poses, gps, NULL, NULL);

			//if (i % 4 == 0 && i > 0)
			//if (i > 16)
			if (1)
			{
				printf("Correction\n");
				load_fused_pointcloud_and_camera(i, cloud, dataset, map._map_type == GridMapTile::TYPE_SEMANTIC);
				pf.correct(gps, cloud, map, transformed_cloud);

				Pose2d mean = pf.mean();
				Pose2d mode = pf.mode();

				printf("True pose: %.2lf %.2lf %.2lf\n", gps.x, gps.y, radians_to_degrees(gps.th));
				printf("PF Mean: %.2lf %.2lf %.2lf Error: %lf\n", mean.x, mean.y, radians_to_degrees(mean.th),
						sqrt(pow(gps.x - mean.x, 2) + pow(gps.y - mean.y, 2)));
				printf("PF Mode: %.2lf %.2lf %.2lf Error: %lf\n", mode.x, mode.y, radians_to_degrees(mode.th),
						sqrt(pow(gps.x - mode.x, 2) + pow(gps.y - mode.y, 2)));

				view(pf, map, poses, gps, cloud, transformed_cloud);
				map.reload(mode.x, mode.y);
			}
		}
	}
}


DatasetInterface*
create_dataset(char *dataset_name)
{
	DatasetInterface *dataset;

	if (!strcmp(dataset_name, "carmen"))
		dataset = new DatasetCarmen(480, 640, "/dados/data_20180112-2/data/", 1);
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

	ParticleFilter pf(30, 0.5, 0.5, degrees_to_radians(10),
			0.5, degrees_to_radians(2.),
			0.05, 0.05, degrees_to_radians(2.),
			25.0, 25.0, degrees_to_radians(100),
			100., 100., 100.);

	//system("rm -rf /dados/maps/maps_20180112-2/*");
	GridMap map("/dados/maps/maps_20180112-2/", 50., 50., 0.2, GridMapTile::TYPE_SEMANTIC);

	//create_map(map, poses, cloud, transformed_cloud, *dataset);
	run_particle_filter(pf, map, poses, odom, times, cloud, transformed_cloud, *dataset);

	printf("Done\n");
	return 0;
}


