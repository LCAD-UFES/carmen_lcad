
#include <cstdio>
#include <cstdlib>
#include <carmen/segmap_dataset.h>
#include <Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

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
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) && 
			 raw_cloud->at(i).x < 70.0 && raw_cloud->at(i).z < -1.)
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
draw_pose(pcl::visualization::PCLVisualizer *viewer, Matrix<double, 4, 4> &pose)
{
    Eigen::Affine3f affine;
	affine = pose.cast<float>();
    viewer->addCoordinateSystem(1., affine);
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
	viewer->removeAllPointClouds();

    char cloud_name[128];
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr moved(new PointCloud<PointXYZRGB>);

    Matrix<double, 4, 4> pose = pose3d_to_matrix(0., 0., dataset.odom_calib.init_angle);
    dataset.load_pointcloud(indices[0].first, cloud, dataset.data[indices[0].first].v, dataset.data[indices[0].first].phi);

    //sprintf(cloud_name, "%05d", indices[0].first);
    //viewer->addPointCloud(cloud, cloud_name);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    //draw_pose(viewer, pose);

    for (int i = 0; i < relative_transform_vector.size(); i++)
    {
        cloud->clear();
        dataset.load_pointcloud(indices[i].second, cloud, dataset.data[indices[i].second].v, dataset.data[indices[i].second].phi);
        cloud = filter_pointcloud(cloud);
		increase_bightness(cloud);

        ///*
	    Pose2d pose_tm1 = dataset.data[indices[i].first].pose;
	    Pose2d pose_t = dataset.data[indices[i].second].pose;

	    pose_t.x -= pose_tm1.x;
	    pose_t.y -= pose_tm1.y;
	    pose_tm1.x = 0.;
	    pose_tm1.y = 0.;

	    Matrix<double, 4, 4> step = 
		    Pose2d::to_matrix(pose_tm1).inverse() *
		    Pose2d::to_matrix(pose_t) * 
		    relative_transform_vector[i];

        //Pose2d util = Pose2d::from_matrix(step);
        //util.y = -util.y;
        //util.th = -util.th;
        //step = Pose2d::to_matrix(util);

        pose = pose * step;
        //*/
        //pose = Pose2d::to_matrix(dataset.data[indices[i].second].pose);

        moved->clear();
    	pcl::transformPointCloud(*cloud, *moved, pose);
        sprintf(cloud_name, "cloud_%05d", indices[i].second);
        viewer->addPointCloud(moved, string(cloud_name));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, string(cloud_name));
        draw_pose(viewer, pose);

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
