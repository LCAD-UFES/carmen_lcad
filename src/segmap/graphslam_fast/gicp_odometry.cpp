
#include <iostream>
#include <deque>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <carmen/carmen.h>
#include "graphslam_util.h"
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


PointCloud<PointXYZRGB>::Ptr 
filter_pointcloud(PointCloud<PointXYZRGB>::Ptr raw_cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);

	cloud->clear();

	///*
	for (int i = 0; i < raw_cloud->size(); i++)
	{
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) && 
			 raw_cloud->at(i).x < 70.0) // || raw_cloud->at(i).z < 0.))
			cloud->push_back(raw_cloud->at(i));
	}
	// */
	//copyPointCloud(*raw_cloud, *cloud);

	return cloud;
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 2)
		exit(printf("Use %s <data-directories>\n", argv[0]));

	DatasetCarmen dataset(argv[1], 0);

	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);

    deque<string> clouds_on_viewer;
	pcl::visualization::PCLVisualizer viewer("CloudViewer");
	//pcl::visualization::PCLVisualizer viewer2("CloudViewer2");
	//viewer2.setBackgroundColor(1, 1, 1);

	viewer.setBackgroundColor(0, 0, 1);
	viewer.removeAllPointClouds();

	char cloud_name[256];
	int pause_viewer = 1;

	Matrix<double, 4, 4> target_pose;
	Matrix<double, 4, 4> source_pose;
	Matrix<double, 4, 4> correction;
	int converged;

	//load_pointcloud(argv[1], data[0].sync[0].cloud_time, target);
    dataset.load_fused_pointcloud_and_camera(0, target, 1);
	target = filter_pointcloud(target);
	target_pose = pose3d_to_matrix(0., 0., dataset.odom_calib.init_angle);
    
    Eigen::Affine3f affine;
	affine = target_pose.cast<float>();
    viewer.addCoordinateSystem(1., affine);

    double time_last_icp = 0;

	//for (int i = 1; i < data[0].sync.size(); i++)
    for (int i = 1; i < dataset.data.size(); i++)
	{
		if (fabs(dataset.data[i].v) < 0.2 || dataset.data[i].velodyne_time - time_last_icp < 0.05) // || fabs(data[0].sync[i].phi) < carmen_degrees_to_radians(20.))
			continue;

        time_last_icp = dataset.data[i].velodyne_time;

		source->clear();
		aligned->clear();
		//load_pointcloud(argv[1], data[0].sync[i].cloud_time, source);
        dataset.load_fused_pointcloud_and_camera(i, source, 1);
		source = filter_pointcloud(source);
		run_gicp(source, target, &correction, &converged, aligned);

		printf("Step: %d Converged: %d Correction: %lf %lf %lf %lf\n", 
            i, converged,
            correction(0, 3) / correction(3, 3),
            correction(1, 3) / correction(3, 3),
            correction(2, 3) / correction(3, 3),
            Pose2d::theta_from_matrix(correction)
        );

/*
		viewer2.removeAllPointClouds();

		viewer2.addPointCloud(source, "source");
		viewer2.addPointCloud(target, "target");
		viewer2.addPointCloud(aligned, "transformed");
		
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");

		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source");
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target");
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "transformed");
*/

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

            clouds_on_viewer.push_back(string(cloud_name));

            if (clouds_on_viewer.size() > 100)
            {
                viewer.removePointCloud(clouds_on_viewer[0]);
                clouds_on_viewer.pop_front();
            }
    
			viewer.addPointCloud(aligned, cloud_name);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
			printf("target pose: %lf %lf %lf\n", 
				target_pose(0, 3) / target_pose(3, 3), 
				target_pose(1, 3) / target_pose(3, 3), 
				Pose2d::theta_from_matrix(target_pose));

			//Matrix4f target_pose_float = target_pose.cast<float>();
			affine = target_pose.cast<float>();
            viewer.addCoordinateSystem(1., affine);
		}

		imshow("viewer", Mat::zeros(300, 300, CV_8UC3));

		char c = ' ';
		while (1)
		{
			//viewer2.spinOnce();
			viewer.spinOnce();
			c = waitKey(5);

			if (c == 's')
				pause_viewer = !pause_viewer;

			if (!pause_viewer || (pause_viewer && c == 'n'))
				break;
		} 
	}
	
	return 0;
}
