
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
run_particle_filter(ParticleFilter &pf, GridMap &map, DatasetInterface &dataset)
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);

	Pose2d p0 = dataset.data[0].pose;

	pf.seed(time(NULL));
	pf.reset(p0.x, p0.y, p0.th);
	map.reload(p0.x, p0.y);

	int last_reload = 0;
	Matrix<double, 4, 4> vel2car = dataset.transform_vel2car();
	int step = 1;

	for (int i = step; i < dataset.data.size(); i += step)
	{
	    //if (fabs(dataset.data[i].v) < 1.0) continue;
		Pose2d gt_pose = dataset.data[i].pose;

		pf.predict(dataset.data[i].v, dataset.data[i].phi, dataset.data[i].image_time - dataset.data[i - step].image_time);
		//view(pf, map, poses, gps, NULL, NULL);
		dataset.load_fused_pointcloud_and_camera(i, cloud, 0);

        //printf("Prediction\n");
        //view(pf, map, gt_pose, cloud, transformed_cloud, &vel2car, dataset.data[i].v, dataset.data[i].phi);

		//if (i % 1 == 0 && i > 0)
		//if (i > 16)
		//if (1)
		{
			pf.correct(cloud, map, transformed_cloud, vel2car, dataset.data[i].v, dataset.data[i].phi);

			Pose2d mean = pf.mean();
			Pose2d mode = pf.mode();

			printf("Step: %d of %ld ", i + 1, dataset.data.size());
			printf("GT_pose: %lf %lf %lf ", gt_pose.x, gt_pose.y, gt_pose.th);
			printf("PF_Mean: %lf %lf %lf ", mean.x, mean.y, mean.th);
			printf("PF_Mode: %lf %lf %lf ", mode.x, mode.y, mode.th);
			printf("D_GT_MEAN: %lf ", dist2d(mean.x, mean.y, gt_pose.x, gt_pose.y));
			printf("D_GT_MODE: %lf ", dist2d(mode.x, mode.y, gt_pose.x, gt_pose.y));
			printf("O_GT_MEAN: %lf ", fabs(normalize_theta(mean.th - gt_pose.th)));
			printf("O_GT_MODE: %lf ", fabs(normalize_theta(mode.th - gt_pose.th)));
			printf("\n");
			fflush(stdout);

			if (dataset.data[i].v > 0.1 && (i - last_reload > 10))
			{
				map.reload(mode.x, mode.y);
				last_reload = i;
			}
		}

		//printf("Correction\n");
		//view(pf, map, gt_pose, cloud, transformed_cloud, &vel2car, dataset.data[i].v, dataset.data[i].phi);
	}
}


int
main(int argc, char **argv)
{
	if (argc < 3)
		exit(printf("Error: Use %s <test log data directory> <map log data directory>\n", argv[0]));

	char dataset_name[256];
	char map_name[256];

	sprintf(dataset_name, "/dados/data/%s", argv[1]);
	sprintf(map_name, "/dados/maps/map_%s", argv[2]);
	printf("dataset_name: %s\n", dataset_name);
	printf("map_name: %s\n", map_name);

	DatasetInterface *dataset;
	dataset = new DatasetCarmen(dataset_name, 1);

	ParticleFilter pf(30, 0.5, 0.5, degrees_to_radians(5),
			0.2, degrees_to_radians(.5),
			0.1, 0.1, degrees_to_radians(.5),
			100., 100., 100.);

	GridMap map(map_name, 50., 50., 0.2, GridMapTile::TYPE_SEMANTIC);
	run_particle_filter(pf, map, *dataset);

	printf("Done\n");
	return 0;
}


