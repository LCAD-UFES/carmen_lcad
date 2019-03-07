
#include <iostream>
#include <deque>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <carmen/carmen.h>
#include "gicp.h"
#include <carmen/segmap_util.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>

#include "g2o/types/slam2d/se2.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace g2o;
using namespace pcl;
using namespace Eigen;

pcl::visualization::PCLVisualizer *viewer = NULL;
pcl::visualization::PCLVisualizer *viewer2 = NULL;
deque<string> clouds_on_viewer;
int pause_viewer = 1;


PointCloud<PointXYZRGB>::Ptr 
filter_pointcloud(PointCloud<PointXYZRGB>::Ptr raw_cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);

	cloud->clear();

	///*
	for (int i = 0; i < raw_cloud->size(); i++)
	{
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) 
			&& raw_cloud->at(i).x < 70.0 
			//&& raw_cloud->at(i).z > -1.0
			//&& raw_cloud->at(i).z < -0.0
		)
			cloud->push_back(raw_cloud->at(i));
	}
	// */
	//copyPointCloud(*raw_cloud, *cloud);

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
		int mult = 2;

		int color = mult * (int) aligned->at(j).r;
		if (color > 255) color = 255;
		else if (color < 0) color = 0;
	
		aligned->at(j).r = (unsigned char) color;

		color = mult * (int) aligned->at(j).g;
		if (color > 255) color = 255;
		else if (color < 0) color = 0;

		aligned->at(j).g = (unsigned char) color;

		color = mult * (int) aligned->at(j).b;
		if (color > 255) color = 255;
		else if (color < 0) color = 0;

		aligned->at(j).b = (unsigned char) color;
	}
	// */
}


void
init_viewers()
{
	viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
	viewer2 = new pcl::visualization::PCLVisualizer("CloudViewer2");

	viewer->setBackgroundColor(.5, .5, .5);
	viewer->removeAllPointClouds();

	viewer2->setBackgroundColor(.5, .5, .5);
	viewer2->removeAllPointClouds();
}


void
draw_pose(Matrix<double, 4, 4> &pose)
{
    Eigen::Affine3f affine;
	affine = pose.cast<float>();
    viewer->addCoordinateSystem(1., affine);
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 3)
		exit(printf("Use %s <data-directory> <output_file>\n", argv[0]));

	DatasetCarmen dataset(argv[1], 0);

	FILE *out_file = fopen(argv[2], "w");

	if (out_file == NULL)
		exit(printf("Output file '%s' could not be open.\n", argv[2]));

	init_viewers();

	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);

	char cloud_name[256];

	Matrix<double, 4, 4> target_pose;
	Matrix<double, 4, 4> source_pose;
	Matrix<double, 4, 4> correction;
	int converged;

    dataset.load_pointcloud(0, target, dataset.data[0].v, dataset.data[0].phi);
	target = filter_pointcloud(target);
	target_pose = pose3d_to_matrix(0., 0., dataset.odom_calib.init_angle);
    draw_pose(target_pose);

    double time_last_icp = 0;

    for (int i = 1; i < dataset.data.size(); i++)
	{
		if (fabs(dataset.data[i].v) < 0.2 || dataset.data[i].velodyne_time - time_last_icp < 0.05) // || fabs(data[0].sync[i].phi) < carmen_degrees_to_radians(20.))
			continue;

        time_last_icp = dataset.data[i].velodyne_time;

		source->clear();
		aligned->clear();
		//load_pointcloud(argv[1], data[0].sync[i].cloud_time, source);
        dataset.load_pointcloud(i, source,  dataset.data[i].v, dataset.data[i].phi);
		source = filter_pointcloud(source);
		run_gicp(source, target, &correction, &converged, aligned);

		fprintf(out_file, "From: %d To: %d Converged: %d RelativePose: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
            i - 1, i, converged,
            correction(0, 0), correction(0, 1), correction(0, 2), correction(0, 3),
            correction(1, 0), correction(1, 1), correction(1, 2), correction(1, 3),
            correction(2, 0), correction(2, 1), correction(2, 2), correction(2, 3),
            correction(3, 0), correction(3, 1), correction(3, 2), correction(3, 3)
        );

///*
		viewer2->removeAllPointClouds();

		viewer2->addPointCloud(source, "source");
		viewer2->addPointCloud(target, "target");
		viewer2->addPointCloud(aligned, "transformed");
		
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");

		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "transformed");
//*/

		target->clear();
        copyPointCloud(*source, *target);

		if (converged)
		{
			/*
            correction = create_transformation_matrix(
                correction(0, 3) / correction(3, 3),
                correction(1, 3) / correction(3, 3),
                theta_from_matrix(correction)
            );
			*/

			target_pose = target_pose * correction;
			sprintf(cloud_name, "cloud%d", i);

	        pcl::transformPointCloud(
		        *source, 
		        *aligned, 
		        target_pose);

			increase_bightness(aligned);
            clouds_on_viewer.push_back(string(cloud_name));

            if (clouds_on_viewer.size() > DBL_MAX)
            {
                viewer->removePointCloud(clouds_on_viewer[0]);
                clouds_on_viewer.pop_front();
            }
    
			viewer->addPointCloud(aligned, cloud_name);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
			printf("target pose: %lf %lf %lf\n", 
				target_pose(0, 3) / target_pose(3, 3), 
				target_pose(1, 3) / target_pose(3, 3), 
				Pose2d::theta_from_matrix(target_pose));

			draw_pose(target_pose);
		}

		imshow("viewer", Mat::zeros(300, 300, CV_8UC3));

		char c = ' ';
		while (1)
		{
			viewer2->spinOnce();
			viewer->spinOnce();
			c = waitKey(5);

			if (c == 's')
				pause_viewer = !pause_viewer;

			if (!pause_viewer || (pause_viewer && c == 'n'))
				break;
		} 
	}

	fclose(out_file);

	return 0;
}
