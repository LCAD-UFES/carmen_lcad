
#include <cstdio>
#include <string>

#include <Eigen/Core>
#include <opencv/cv.hpp>
#include <pcl/common/transforms.h>

#include <carmen/segmap_dataset.h>
#include <carmen/carmen_lidar_reader.h>
#include <carmen/carmen_image_reader.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/command_line.h>

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;

#define brighten(x) ((unsigned char) ((x * 5 > 255) ? (255) : (x * 5)))


void
print_sample_info(DataSample *data_package)
{
	printf("gps: %lf %lf ", data_package->gps.x, data_package->gps.y);
	printf("odom: %lf %lf ", data_package->v, data_package->phi);
	printf("gps time: %lf ", data_package->gps_time);
	printf("image time: %lf ", data_package->image_time);
	printf("velodyne time: %lf ", data_package->velodyne_time);
	printf("odom time: %lf ", data_package->odom_time);
	printf("xsens time: %lf ", data_package->xsens_time);
	printf("fused odom: %lf %lf %lf\n---\n", data_package->pose.x, data_package->pose.y, data_package->pose.th);
}


void
load_and_transform_pointcloud(DataSample *data_package,
															CarmenLidarLoader &vloader,
															PointCloud<PointXYZRGB>::Ptr cloud,
															Pose2d offset_pose)
{
	Pose2d pose;
	Matrix<double, 4, 4> T;

	vloader.reinitialize(data_package->velodyne_path, data_package->n_laser_shots);
	load_as_pointcloud(&vloader, cloud);

	for (int j = 0; j < cloud->size(); j++)
	{
		cloud->at(j).r =  brighten(cloud->at(j).r);
		cloud->at(j).g = (unsigned char) brighten(cloud->at(j).g);
		cloud->at(j).b = (unsigned char) brighten(cloud->at(j).b);
	}

	pose = data_package->pose;

	// to prevent numerical issues.
	pose.x -= offset_pose.x;
	pose.y -= offset_pose.y;

	T = Pose2d::to_matrix(pose);
	transformPointCloud(*cloud, *cloud, T);
}


NewCarmenDataset::SyncSensor
get_reference_sensor(CommandLineArguments &args)
{
	string ssensor = args.get<string>("sync_sensor");

	if (ssensor.compare("velodyne") == 0)
		return NewCarmenDataset::SYNC_BY_VELODYNE;
	else if (ssensor.compare("camera") == 0)
		return NewCarmenDataset::SYNC_BY_CAMERA;
	else
		exit(printf("Error: Invalid sync mode '%s'.\n.", ssensor.c_str()));
}


NewCarmenDataset::SyncMode
get_sync_mode(CommandLineArguments &args)
{
	string smode = args.get<string>("sync_mode");

	if (smode.compare("nearest") == 0)
		return NewCarmenDataset::SYNC_BY_NEAREST;
	else if (smode.compare("nearest_before") == 0)
		return NewCarmenDataset::SYNC_BY_NEAREST_BEFORE;
	else
		exit(printf("Error: Invalid sync mode '%s'\n.", smode.c_str()));
}


NewCarmenDataset *
create_dataset_from_args(CommandLineArguments &args)
{
	NewCarmenDataset *dataset;

	string log_path = args.get<string>("log");
	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());

	NewCarmenDataset::SyncMode mode = get_sync_mode(args);
	NewCarmenDataset::SyncSensor reference_sensor = get_reference_sensor(args);

	int gps_id = args.get<int>("gps_id");

	dataset = new NewCarmenDataset(log_path,
																 odom_calib_path,
																 fused_odom_path,
																 gps_id,
																 reference_sensor,
																 mode);

	printf("Dataset loaded! Number of data packages: %d.\n", dataset->size());

	return dataset;
}


CommandLineArguments
parse_command_line_args(int argc, char **argv)
{
	CommandLineArguments args;

	args.add_positional<string>("log", "Path to a log", 1);
	args.add<int>("step,s", "Number of messages to skip.", 1);
	args.add<int>("gps_id,g", "Id of the gps sensor", 1);
	args.add<string>("sync_sensor,r", "Synchronization sensor [velodyne | camera]", "camera");
	args.add<string>("sync_mode,m", "Synchronization mode [nearest | nearest_before]", "nearest");
	args.parse(argc, argv);

	return args;
}


int
main(int argc, char **argv)
{
	int step;
	Mat img;
	Pose2d offset;
	DataSample* data_package;
	PointCloudViewer viewer;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	CommandLineArguments args = parse_command_line_args(argc, argv);
	NewCarmenDataset *dataset = create_dataset_from_args(args);
	CarmenLidarLoader vloader;
	CarmenImageLoader iloader;

	step = args.get<int>("step");
	offset = dataset->at(0)->pose;

	for (int i = 1; i < dataset->size(); i += step)
	{
		data_package = dataset->at(i);

		// ignore packages when the car is stopped.
		if (fabs(data_package->v) < 1.0)
			continue;

		print_sample_info(data_package);
		load_and_transform_pointcloud(data_package, vloader, cloud, offset);
		img = iloader.load(data_package);

		viewer.show(img, "img", 640);
		viewer.show(cloud);
		viewer.loop();
	}

	printf("Log is done.\n");
	return 0;
}

