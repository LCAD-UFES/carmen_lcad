
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
	//printf("N poses: %ld\n", poses.size());
	//Pose2d p0 = dataset._gps[0];
	Pose2d p0 = Pose2d::from_matrix(poses[0]);

	pf.seed(time(NULL));
	pf.reset(p0.x, p0.y, p0.th);
	map.reload(p0.x, p0.y);

	int last_reload = 0;
	Matrix<double, 4, 4> vel2car = dataset.transform_vel2car();
	int step = 10;

	for (int i = step; i < times.size(); i += step)
	{
		//Pose2d gps = dataset._gps[i];
		Pose2d gt_pose = Pose2d::from_matrix(poses[i]); // TODO: add noise

		pf.predict(odom[i].first, odom[i].second, times[i] - times[i - step]);
		//view(pf, map, poses, gps, NULL, NULL);
		dataset.load_fused_pointcloud_and_camera(i, cloud, 0);

		//if (i % 1 == 0 && i > 0)
		//if (i > 16)
		//if (1)
		{
			pf.correct(cloud, map, transformed_cloud, vel2car);

			Pose2d mean = pf.mean();
			Pose2d mode = pf.mode();

			printf("Step: %d of %ld ", i + 1, times.size());
			printf("GT_pose: %lf %lf %lf ", gt_pose.x, gt_pose.y, gt_pose.th);
			printf("PF_Mean: %lf %lf %lf ", mean.x, mean.y, mean.th);
			printf("PF_Mode: %lf %lf %lf ", mode.x, mode.y, mode.th);
			printf("D_GT_MEAN: %lf ", dist2d(mean.x, mean.y, gt_pose.x, gt_pose.y));
			printf("D_GT_MODE: %lf ", dist2d(mode.x, mode.y, gt_pose.x, gt_pose.y));
			printf("O_GT_MEAN: %lf ", fabs(normalize_theta(mean.th - gt_pose.th)));
			printf("O_GT_MODE: %lf ", fabs(normalize_theta(mode.th - gt_pose.th)));
			printf("\n");
			fflush(stdout);

			if (odom[i].first > 0.1 && (i - last_reload > 10))
			{
				map.reload(mode.x, mode.y);
				last_reload = i;
			}
		}

		view(pf, map, poses, gt_pose, cloud, transformed_cloud, &vel2car);
	}
}


DatasetInterface*
create_dataset(char *dataset_name)
{
	DatasetInterface *dataset;

	if (!strcmp(dataset_name, "carmen"))
		dataset = new DatasetCarmen(960, 1280, "/dados/data/data_20180907-2/", 1);
		//dataset = new DatasetCarmen(480, 640, "/dados/data/data_20180112-2/", 1);
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
			0.1, degrees_to_radians(0.5),
			0.01, 0.01, degrees_to_radians(0.5),
			100., 100., 100.);

	GridMap map("/dados/maps/maps_20180112-2/", 50., 50., 0.2, GridMapTile::TYPE_SEMANTIC);
	run_particle_filter(pf, map, poses, odom, times, cloud, transformed_cloud, *dataset);

	printf("Done\n");
	return 0;
}


