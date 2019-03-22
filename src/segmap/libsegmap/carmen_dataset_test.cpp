
#include <cstdio>
#include <string>
#include "segmap_dataset.h"
#include "segmap_util.h"
#include "segmap_viewer.h"
#include "segmap_sensors.h"
#include <carmen/segmap_command_line.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

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
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	gps0 = dataset[0]->gps;
	dead_reckoning = Pose2d(0, 0, dataset.initial_angle());

	for (int i = 1; i < dataset.size(); i++)
	{
		data_package = dataset[i];

		if (i > 0)
			ackerman_motion_model(dead_reckoning, data_package->v, data_package->phi, data_package->image_time - dataset[i - 1]->image_time);

		printf("gps: %lf %lf ", data_package->gps.x, data_package->gps.y);
		printf("odom: %lf %lf ", data_package->v, data_package->phi);
		printf("gps time: %lf ", data_package->gps_time);
		printf("image time: %lf ", data_package->image_time);
		printf("velodyne time: %lf ", data_package->velodyne_time);
		printf("odom time: %lf ", data_package->odom_time);
		printf("xsens time: %lf ", data_package->xsens_time);
		printf("fused odom: %lf %lf %lf\n", data_package->pose.x, data_package->pose.y, data_package->pose.th);
		printf("dead reckoning: %lf %lf %lf\n", dead_reckoning.x, dead_reckoning.y, dead_reckoning.th);

		CarmenLidarLoader loader(data_package->velodyne_path.c_str(),
		                         data_package->n_laser_shots,
		                         dataset.intensity_calibration);

		load_as_pointcloud(&loader, cloud);
		Mat img = load_image(data_package);

		viewer.show(img, "img", 320);
		viewer.show(cloud);
		viewer.loop();
		viewer.clear();
	}

	printf("Ok\n");
	return 0;
}
