
#include <cstdio>
#include <cstdlib>
#include <carmen/segmap_dataset.h>
#include <Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;


PointCloud<PointXYZRGB>::Ptr 
filter_pointcloud(PointCloud<PointXYZRGB>::Ptr raw_cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);
	cloud->clear();

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) 
			&& raw_cloud->at(i).x < 70.0  // remove max range
			&& raw_cloud->at(i).z > -1.25  // remove ground
			//&& raw_cloud->at(i).z < -0.5  // remove tree tops
		)
			cloud->push_back(raw_cloud->at(i));
	}

	return cloud;
}


void
read_odom_data(char *filename, vector<pair<int,int>> &indices,
	vector<Matrix<double, 4, 4>> &relative_transform_vector,
	vector<int> &convergence_vector)
{
    FILE *f = fopen(filename, "r");

    if (f == NULL)
        exit(printf("Error: file '%s' not found.\n", filename));

    int from, to, converged, n;
    double data[16];
    Matrix<double, 4, 4> relative_pose;

    while (!feof(f))
    {
        n = fscanf(f, "%d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
            &from, &to, &converged,
            data, data + 1, data + 2, data + 3, 
            data + 4, data + 5, data + 6, data + 7,
            data + 8, data + 9, data + 10, data + 11,
            data + 12, data + 13, data + 14, data + 15);

        if (n != 19)
            continue;

        indices.push_back(pair<int, int>(from, to));
        convergence_vector.push_back(converged);

        relative_pose << data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11],
            data[12], data[13], data[14], data[15];
        
        relative_transform_vector.push_back(relative_pose);
    }

    printf("%ld relative poses loaded!\n", relative_transform_vector.size());

    fclose(f);
}


void
increase_bightness(PointCloud<PointXYZRGB>::Ptr aligned)
{
	for (int j = 0; j < aligned->size(); j++)
	{
		// int b = ((aligned->at(j).z + 5.0) / 10.) * 255;
		// if (b < 0) b = 0;
		// if (b > 255) b = 255;
		int mult = 4;

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
}


int 
main(int argc, char **argv)
{
    if (argc < 3)
        exit(printf("Error: Use %s <dataset_dir> <odom_file>\n", argv[0]));

    vector<pair<int,int>> indices;
	vector<Matrix<double, 4, 4>> relative_transform_vector;
	vector<int> convergence_vector;

    read_odom_data(argv[2], indices, relative_transform_vector, convergence_vector);
    DatasetCarmen dataset(argv[1], 0);

    int pause_viewer = 1;
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
	viewer->setBackgroundColor(.5, .5, .5);

    for (int i = 0; i < relative_transform_vector.size(); i++)
    {
        printf("Cloud %d of %ld Converged: %d\n", i, relative_transform_vector.size(), convergence_vector[i]);

        cout << relative_transform_vector[i] << endl << endl;

    	viewer->removeAllPointClouds();

	    PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	    PointCloud<PointXYZRGB>::Ptr source_moved(new PointCloud<PointXYZRGB>);
	    PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	    PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);

        dataset.load_pointcloud(indices[i].first, target, dataset.data[indices[i].first].v, dataset.data[indices[i].first].phi);
        dataset.load_pointcloud(indices[i].second, source, dataset.data[indices[i].second].v, dataset.data[indices[i].second].phi);

        target = filter_pointcloud(target);
        source = filter_pointcloud(source);

	    Pose2d pose_target = dataset.data[indices[i].first].pose;
	    Pose2d pose_source = dataset.data[indices[i].second].pose;

	    pose_source.x -= pose_target.x;
	    pose_source.y -= pose_target.y;
	    pose_target.x = 0.;
	    pose_target.y = 0.;

	    Matrix<double, 4, 4> guess = 
		    Pose2d::to_matrix(pose_target).inverse() *
		    Pose2d::to_matrix(pose_source);

        Matrix<double, 4, 4> relative_pose = guess * relative_transform_vector[i];

    	pcl::transformPointCloud(*source, *source_moved, guess);
    	pcl::transformPointCloud(*source, *aligned, relative_pose);

        //viewer->addPointCloud(source_moved, "source");
        viewer->addPointCloud(target, "target");
        viewer->addPointCloud(aligned, "aligned");

        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned");

        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source"); // red
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target"); // green
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "aligned");  // blue

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

    viewer->spin();
    return 0;
}
