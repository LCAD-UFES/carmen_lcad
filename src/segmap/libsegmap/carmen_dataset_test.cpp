
#include <cstdio>
#include <string>
#include "segmap_dataset.h"
#include "segmap_util.h"
#include "segmap_viewer.h"
#include "segmap_sensors.h"
#include <carmen/segmap_command_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;


#define enhance(x) ((x * 5 > 255) ? (255) : (x * 5))


PointCloud<PointXYZRGB>::Ptr
load_pointcloud_from_file(DataSample *sample)
{
	char path[512];

	PointCloud<PointXYZRGB>::Ptr raw(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	sprintf(path,
					"/dados/data/data_log-estacionamento-ambiental-20181208.txt/velodyne/%lf.ply",
					sample->velodyne_time);

	int success = pcl::io::loadPLYFile(path, *raw);

	if (success < 0 || raw->size() == 0)
		return cloud;

	double h, v, r;
	double x, y, z;

	for (int i = 0; i < raw->size(); i++)
	{
		cartersian2spherical(raw->at(i).x, raw->at(i).y, raw->at(i).z, &v, &h, &r);
		spherical2cartersian(v, -h, r, &x, &y, &z);

		PointXYZRGB p;

		p.x = (float) x;
		p.y = (float) y;
		p.z = (float) z;

		p.r = (unsigned char) raw->at(i).r;
		p.g = (unsigned char) raw->at(i).g;
		p.b = (unsigned char) raw->at(i).b;

		cloud->push_back(PointXYZRGB(p));
	}

	return cloud;
}


PointCloud<PointXYZRGB>::Ptr
load_pcl(string path, int n_shots, unsigned char ***calib)
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	CarmenLidarLoader loader(path.c_str(), n_shots, calib);
	load_as_pointcloud(&loader, cloud);
	return cloud;
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;
	args.add_positional<string>("log", "Path to a log", 1);
	args.parse(argc, argv);

	string log_path = args.get<string>("log");
	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());

	NewCarmenDataset dataset(log_path, odom_calib_path, fused_odom_path);

	DataSample* data_package;
	Pose2d gps0, dead_reckoning;
	PointCloudViewer viewer;
	PointCloud<PointXYZRGB>::Ptr cloud;
	PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);

	gps0 = dataset[0]->gps;
	dead_reckoning = Pose2d(0, 0, dataset.initial_angle());

	for (int i = 1; i < dataset.size(); i += 5)
	{
		data_package = dataset[i];

		if (i > 0)
			ackerman_motion_model(dead_reckoning, data_package->v, data_package->phi, data_package->image_time - dataset[i - 1]->image_time);

		if (fabs(data_package->v) < 1.0)
			continue;

		printf("gps: %lf %lf ", data_package->gps.x, data_package->gps.y);
		printf("odom: %lf %lf ", data_package->v, data_package->phi);
		printf("gps time: %lf ", data_package->gps_time);
		printf("image time: %lf ", data_package->image_time);
		printf("velodyne time: %lf ", data_package->velodyne_time);
		printf("odom time: %lf ", data_package->odom_time);
		printf("xsens time: %lf ", data_package->xsens_time);
		printf("fused odom: %lf %lf %lf\n", data_package->pose.x, data_package->pose.y, data_package->pose.th);
		printf("dead reckoning: %lf %lf %lf\n", dead_reckoning.x, dead_reckoning.y, dead_reckoning.th);

		cloud = load_pcl(data_package->velodyne_path,
								 data_package->n_laser_shots,
								 dataset.intensity_calibration);

		if (cloud->size() != (data_package->n_laser_shots * 32))
			exit(printf("Problem: %d\n", i));

		Pose2d p = data_package->pose;
		p.x -= gps0.x;
		p.y -= gps0.y;

		for (int j = 0; j < cloud->size(); j++)
		{
			cloud->at(j).r = (unsigned char) enhance(cloud->at(j).r);
			cloud->at(j).g = (unsigned char) enhance(cloud->at(j).g);
			cloud->at(j).b = (unsigned char) enhance(cloud->at(j).b);
		}

		Matrix<double, 4, 4> T = Pose2d::to_matrix(p);
		transformPointCloud(*cloud, *cloud2, T);

		Mat img = load_image(data_package);
		viewer.show(img, "img", 320);
		viewer.show(cloud2);
		viewer.loop();

		cloud2->clear();

		//char *name = new char[64];
		//sprintf(name, "%d", i);
		//viewer.addPointCloud(cloud2, name);
		//viewer.spinOnce();

		//viewer.clear();
	}

	printf("Ok\n");
	return 0;
}

