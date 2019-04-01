
#include <deque>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <carmen/segmap_util.h>
#include "gicp.h"

#include <opencv/cv.hpp>

using namespace cv;
using namespace std;
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
		if ((fabs(raw_cloud->at(i).x) > 2.0 || fabs(raw_cloud->at(i).y) > 2.0) && raw_cloud->at(i).x < 70.0 
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
		int mult = 3;

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
	//viewer2 = new pcl::visualization::PCLVisualizer("CloudViewer2");

	viewer->setBackgroundColor(.5, .5, .5);
	viewer->removeAllPointClouds();

	//viewer2->setBackgroundColor(.5, .5, .5);
	//viewer2->removeAllPointClouds();
}


void
draw_pose(Matrix<double, 4, 4> &pose)
{
    Eigen::Affine3f affine;
	affine = pose.cast<float>();
    viewer->addCoordinateSystem(1., affine);
}


vector<string>
read_velodynes(string data_dir)
{
	vector<string> path_clouds;
	string filename = (data_dir + "/velodyne.txt");
	FILE *f = fopen(filename.c_str(), "r");

	if (f == NULL)
		exit(printf("Error: file %s not found!\n", filename.c_str()));

	char cloud_path[256];
	
	while (!feof(f))
	{
		if (fscanf(f, "\n%s\n", cloud_path))
			path_clouds.push_back(data_dir + "/velodyne/" + string(cloud_path));
	}

	fclose(f);
	return path_clouds;
}


PointCloud<PointXYZRGB>::Ptr 
load_cloud(string path)
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	int success = pcl::io::loadPLYFile(path, *cloud);

	if (success < 0 || cloud->size() == 0)
		exit(printf("Cloud %s not found.\n", path.c_str()));

	return cloud;
}


PointCloud<PointXYZRGB>::Ptr 
leafize(PointCloud<PointXYZRGB>::Ptr cloud, double leaf_size)
{
	PointCloud<PointXYZRGB>::Ptr leafed(new PointCloud<PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	grid.setInputCloud(cloud);
	grid.filter(*leafed);

	return leafed;
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 3)
		exit(printf("Use %s <data-directory> <output_file>\n", argv[0]));

	char cloud_name[128];
	vector<string> path_clouds = read_velodynes(argv[1]);
	printf("Number of clouds: %ld\n", path_clouds.size());

	FILE *out_file = fopen(argv[2], "w");

	if (out_file == NULL)
		exit(printf("Output file '%s' could not be open.\n", argv[2]));

	printf("Output file %s created.\n", argv[2]);

	init_viewers();

	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);

	Matrix<double, 4, 4> target_pose;
	Matrix<double, 4, 4> source_pose;
	Matrix<double, 4, 4> correction;
	int converged;

	target = load_cloud(path_clouds[800]);
	target = filter_pointcloud(target);
	increase_bightness(target);
	target_pose = pose3d_to_matrix(0., 0., 0.);
    draw_pose(target_pose);

    for (int i = 801; i < path_clouds.size(); i++)
	{
		source->clear();
		aligned->clear();
        source = load_cloud(path_clouds[i]);
		source = filter_pointcloud(source);
		source = leafize(source, 0.15);
		increase_bightness(source);

		run_gicp(source, target, &correction, &converged, aligned, -1);

		fprintf(out_file, "From: %d To: %d Converged: %d RelativePose: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
            i - 1, i, converged,
            correction(0, 0), correction(0, 1), correction(0, 2), correction(0, 3),
            correction(1, 0), correction(1, 1), correction(1, 2), correction(1, 3),
            correction(2, 0), correction(2, 1), correction(2, 2), correction(2, 3),
            correction(3, 0), correction(3, 1), correction(3, 2), correction(3, 3)
        );
		fflush(out_file);

		fprintf(stdout, "From: %d To: %d Converged: %d RelativePose: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
            i - 1, i, converged,
            correction(0, 0), correction(0, 1), correction(0, 2), correction(0, 3),
            correction(1, 0), correction(1, 1), correction(1, 2), correction(1, 3),
            correction(2, 0), correction(2, 1), correction(2, 2), correction(2, 3),
            correction(3, 0), correction(3, 1), correction(3, 2), correction(3, 3)
        );

		/*
		viewer2->removeAllPointClouds();

		//viewer2->addPointCloud(source, "source");
		viewer2->addPointCloud(target, "target");
		viewer2->addPointCloud(aligned, "transformed");
		
		//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");

		//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source");
		//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target");
		//viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "transformed");
		*/

		//target->clear();
        //copyPointCloud(*source, *target);
		//copyPointCloud(*aligned, *target);
		(*target) += (*aligned);
		target = leafize(target, 0.15);

		viewer->removeAllPointClouds();
		viewer->addPointCloud(target, "lcad");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "lcad");

		if (converged)
		{
			target_pose = correction; // * target_pose;

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
			//viewer2->spinOnce();
			viewer->spinOnce();
			c = waitKey(5);

			if (c == 's')
				pause_viewer = !pause_viewer;

			if (!pause_viewer || (pause_viewer && c == 'n'))
				break;
		} 
	}

	fclose(out_file);
	printf("Done.\n");

	return 0;
}
