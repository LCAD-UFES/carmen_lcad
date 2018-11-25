
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
run_particle_filter(ParticleFilter &pf, GridMap &map, vector<Matrix<double, 4, 4>> &poses,
		vector<pair<double, double>> &odom, vector<double> &times, PointCloud<PointXYZRGB>::Ptr cloud,
		PointCloud<PointXYZRGB>::Ptr transformed_cloud, DatasetInterface &dataset)
{
	Pose2d p0 = Pose2d::from_matrix(poses[0]);

	pf.seed(time(NULL));
	pf.reset(p0.x, p0.y, p0.th);
	map.reload(p0.x, p0.y);

	printf("Initial particles\n");
	view(pf, map, poses, p0, NULL, NULL);

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
			dataset.load_fused_pointcloud_and_camera(i, cloud, 1);
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


DatasetInterface*
create_dataset(char *dataset_name)
{
	DatasetInterface *dataset;

	if (!strcmp(dataset_name, "carmen"))
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

	ParticleFilter pf(30, 0.5, 0.5, degrees_to_radians(10),
			0.5, degrees_to_radians(2.),
			0.05, 0.05, degrees_to_radians(2.),
			25.0, 25.0, degrees_to_radians(100),
			100., 100., 100.);

	GridMap map("/dados/maps/maps_20180112-2/", 50., 50., 0.2, GridMapTile::TYPE_SEMANTIC);
	run_particle_filter(pf, map, poses, odom, times, cloud, transformed_cloud, *dataset);

	printf("Done\n");
	return 0;
}


