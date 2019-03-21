
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
#include "libsegmap/segmap_sensors.h"
#include <carmen/segmap_command_line.h>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;

#define VIEW 1
#define USE_NEW 1


PointCloud<PointXYZRGB>::Ptr
filter_pointcloud(PointCloud<PointXYZRGB>::Ptr raw_cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);
	cloud->clear();

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		if ((fabs(raw_cloud->at(i).x) > 6.0 || fabs(raw_cloud->at(i).y) > 3.0) // remove rays that hit car
		    && raw_cloud->at(i).x < 70.0  // remove max range
		    && raw_cloud->at(i).z < -0.  // remove tree tops
		     )
			cloud->push_back(raw_cloud->at(i));
	}

	return cloud;
}


void
increase_bightness(PointCloud<PointXYZRGB>::Ptr aligned)
{
	// /*
	for (int j = 0; j < aligned->size(); j++)
	{
		// int b = ((aligned->at(j).z + 5.0) / 10.) * 255;
		// if (b < 0) b = 0;
		// if (b > 255) b = 255;
		int mult = 3;

		int color = mult * (int) aligned->at(j).r;
		if (color > 255)
			color = 255;
		else if (color < 0)
			color = 0;

		aligned->at(j).r = (unsigned char) color;

		color = mult * (int) aligned->at(j).g;
		if (color > 255)
			color = 255;
		else if (color < 0)
			color = 0;

		aligned->at(j).g = (unsigned char) color;

		color = mult * (int) aligned->at(j).b;
		if (color > 255)
			color = 255;
		else if (color < 0)
			color = 0;

		aligned->at(j).b = (unsigned char) color;
	}
	// */
}


void
colorize(PointCloud<PointXYZRGB>::Ptr cloud, Matrix<double, 4, 4> &lidar2cam,
					Matrix<double, 3, 4> &projection, Mat &img,
					PointCloud<PointXYZRGB>::Ptr colored)
{
	Mat orig = img.clone();
	Point ppixel;
	int is_valid;

	for (int i = 0; i < cloud->size(); i++)
	{
		get_pixel_position(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z,
												lidar2cam, projection, img, &ppixel, &is_valid);

		if (is_valid)
		{
			circle(img, ppixel, 2, Scalar(0, 0, 255), -1);

			PointXYZRGB point = cloud->at(i);
			point.r = orig.data[3 * (ppixel.y * orig.cols + ppixel.x) + 2];
			point.g = orig.data[3 * (ppixel.y * orig.cols + ppixel.x) + 1];
			point.b = orig.data[3 * (ppixel.y * orig.cols + ppixel.x) + 0];
			colored->push_back(point);
		}
	}
}


#if USE_NEW
void
create_map(GridMap &map, const char *log_path, NewCarmenDataset *dataset,
						char path_save_maps[])
{
	DataSample *sample;
	PointCloudViewer viewer;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr colored(new PointCloud<PointXYZRGB>);

	Mat img;
	Matrix<double, 4, 4> lidar2car = dataset->vel2car();
	Matrix<double, 4, 4> lidar2cam = dataset->vel2cam();
	Matrix<double, 3, 4> projection = dataset->projection_matrix();

	SemanticSegmentationLoader sloader(log_path);
	Pose2d p0 = dataset->at(0)->pose;

	for (int i = 0; i < dataset->size(); i++)
	{
		sample = dataset->at(i);

		if (fabs(sample->v) < 1.0)
			continue;

		CarmenLidarLoader loader(sample->velodyne_path.c_str(),
															sample->n_laser_shots,
															dataset->intensity_calibration);

		Pose2d pose = sample->pose;
		pose.x -= p0.x;
		pose.y -= p0.y;

		load_as_pointcloud(&loader, cloud);
		cloud = filter_pointcloud(cloud);

		img = load_image(sample);
		//img = sloader.load(sample);

		colored->clear();
		colorize(cloud, lidar2cam, projection, img, colored);

		transformPointCloud(*colored, *transformed, (Pose2d::to_matrix(pose) * lidar2car).cast<float>());

		map.reload(pose.x, pose.y);
		for (int j = 0; j < transformed->size(); j++)
			map.add_point(transformed->at(j));

		Mat map_img = map.to_image().clone();
		draw_pose(map, map_img, pose, Scalar(0, 255, 0));

		// flip vertically.
		flip(map_img, map_img, 0);

		//viewer.show(transformed);
		viewer.show(map_img, "map", 640);
		viewer.show(img, "img", 640);
		viewer.loop();
	}
}

#else
void
create_map(GridMap &map, DatasetInterface &dataset, char path_save_maps[])
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud2(new PointCloud<PointXYZRGB>);

#if VIEW
	int pause_viewer = 1;

	pcl::visualization::PCLVisualizer viewer("CloudViewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.removeAllPointClouds();
	viewer.addCoordinateSystem(2);
#endif

	Matrix<double, 4, 4> vel2car = dataset.transform_vel2car();
	Mat img_view;
	char map_name[512];

	deque<string> cloud_names;
	int step = 1;

	Matrix<double, 3, 3> rot3d;
	Matrix<double, 4, 4> transf;

	for (int i = 0; i < dataset.data.size(); i += step)
	{
		if (fabs(dataset.data[i].v) < 0.1)
		continue;

		Pose2d pose = dataset.data[i].pose;

		cloud->clear();
		transformed_cloud->clear();
		dataset.load_fused_pointcloud_and_camera(i, cloud, dataset.data[i].v, dataset.data[i].phi, 1, &img_view);
		//pose.x = pose.y = 0.;
		//pose.th = normalize_theta(-pose.th - degrees_to_radians(18));

		//double roll, pitch, yaw;
		//Vector3d euler = dataset.data[i].xsens.toRotationMatrix().eulerAngles(2, 1, 0);
		//yaw = euler[0]; pitch = euler[1]; roll = euler[2];

		//printf("roll: %lf pitch: %lf yaw: %lf\n",
		//radians_to_degrees(roll), radians_to_degrees(pitch), radians_to_degrees(yaw));

		//Matrix<double, 4, 4> mat = pose6d_to_matrix(pose.x, pose.y, 0., roll, pitch, pose.th);
		Matrix<double, 4, 4> mat = pose6d_to_matrix(pose.x, pose.y, 0, 0, 0, pose.th);
		pcl::transformPointCloud(*cloud, *transformed_cloud, mat);

		map.reload(pose.x, pose.y);

		for (int j = 0; j < transformed_cloud->size(); j++)
		map.add_point(transformed_cloud->at(j));

		Mat map_img = map.to_image().clone();
		draw_pose(map, map_img, pose, Scalar(0, 255, 0));

		Mat concat;
		hconcat(map_img, img_view, concat);
		//sprintf(map_name, "%s/step_%010d.png", path_save_maps, i);
		//imwrite(map_name, concat);

#if VIEW

		/*
		 rot3d = dataset.data[i].xsens;
		 transf << rot3d(0, 0), rot3d(0, 1), rot3d(0, 2), 0,
		 rot3d(1, 0), rot3d(1, 1), rot3d(1, 2), 0,
		 rot3d(2, 0), rot3d(2, 1), rot3d(2, 2), 0,
		 0, 0, 0, 1;
		 pcl::transformPointCloud(*cloud, *transformed_cloud2, transf);
		 */

		if (map._map_type == GridMapTile::TYPE_SEMANTIC)
		colorize_cloud_according_to_segmentation(transformed_cloud);
		//increase_bightness(transformed_cloud);

		///*
		char *cloud_name = (char *) calloc (32, sizeof(char));
		sprintf(cloud_name, "cloud%d", i);

		//viewer.removeAllPointClouds();
		//for (int j = 0; j < transformed_cloud->size(); j++)
		//transformed_cloud->at(j).z = 0.;

		viewer.removeAllPointClouds();
		viewer.addPointCloud(transformed_cloud, cloud_name);
		//viewer.addPointCloud(transformed_cloud2, "xsens");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "xsens");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "xsens");
		//cloud_names.push_back(cloud_name);

		//if (cloud_names.size() >= 300)
		//{
		//	viewer.removePointCloud(cloud_names[0]);
		//	cloud_names.pop_front();
		//}
		//*/

		//view(pf, map, dataset.data[i].pose, cloud, transformed_cloud, &vel2car, dataset.data[i].v, dataset.data[i].phi);

		imshow("concat", concat);

		char c = ' ';
		while (1)
		{
			viewer.spinOnce();
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
		//*/
#endif

//		if (i > 500 && i < dataset.data.size() - 1000)
//			i = dataset.data.size() - 1000;
	}

	/*
	 for (int i = 0; i < 500; i++)
	 {
	 if (fabs(dataset.data[i].v) < 0.1)
	 continue;

	 Pose2d pose = dataset.data[i].pose;

	 cloud->clear();
	 transformed_cloud->clear();
	 dataset.load_fused_pointcloud_and_camera(i, cloud, dataset.data[i].v, dataset.data[i].phi, 1);
	 pcl::transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(pose));

	 for (int j = 0; j < transformed_cloud->size(); j++)
	 transformed_cloud->at(j).z += .7;

	 viewer.removePointCloud("bola");
	 viewer.addPointCloud(transformed_cloud, "bola");
	 viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "bola");
	 viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.5, 0, "bola");

	 char c = ' ';
	 while (1)
	 {
	 viewer.spinOnce();
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
	 }
	 */

	//waitKey(-1);
}
#endif


int
main(int argc, char **argv)
{
	string map_type, log_path, map_path;
	double resolution, tile_size;
	po::variables_map args;

	CommandLineArguments args_parser;

	args_parser.add_positional<string>("log_path", "Path of a log", 1);
	args_parser.add<string>("map_type,t", "Map type: [categorical | gaussian]", "gaussian");
	args_parser.add<double>("resolution,r", "Map resolution", 0.2);
	args_parser.add<double>("tile_size,s", "Map tiles size", 50);
	args_parser.add<string>("map_path,m", "Path to save the maps", "/tmp");
	args_parser.save_config_file("data/mapper_config.txt");
	args_parser.parse(argc, argv);

	map_type = args_parser.get<string>("map_type");
	resolution = args_parser.get<double>("resolution");
	tile_size = args_parser.get<double>("tile_size");
	log_path = args_parser.get<string>("log_path");
	map_path = args_parser.get<string>("map_path");

	printf("map path: %s\n", map_path.c_str());
	printf("tile size: %lf\n", tile_size);
	printf("resolution: %lf\n", resolution);

	/*
	 char path_save_maps[256];
	 char dataset_name[256];
	 char map_name[256];

	 sprintf(dataset_name, "/dados/data/%s", argv[1]);
	 sprintf(map_name, "/dados/maps/map_%s", argv[1]);
	 sprintf(path_save_maps, "/dados/map_imgs/%s", argv[1]);

	 char cmd[256];
	 sprintf(cmd, "rm -rf %s && mkdir %s", path_save_maps, path_save_maps);
	 system(cmd);

	 sprintf(cmd, "rm -rf %s && mkdir %s", map_name, map_name);
	 system(cmd);

	 printf("dataset_name: %s\n", dataset_name);
	 printf("map_name: %s\n", map_name);
	 printf("path to save maps: %s\n", path_save_maps);
	 */

	GridMap map(map_path, tile_size, tile_size, resolution, GridMapTile::TYPE_VISUAL, 1);

#if USE_NEW	

	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());

	NewCarmenDataset *dataset;
	dataset = new NewCarmenDataset(log_path, odom_calib_path, fused_odom_path);
	create_map(map, log_path.c_str(), dataset, "/tmp");

#else
	DatasetInterface *dataset = new DatasetCarmen(dataset_name, 0);
	create_map(map, *dataset, path_save_maps);
#endif

	printf("Done\n");
	return 0;
}

