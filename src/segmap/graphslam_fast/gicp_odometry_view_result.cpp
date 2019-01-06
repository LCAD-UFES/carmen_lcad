
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

    Matrix<double, 4, 4> pose = pose3d_to_matrix(0, 0, 0);
    dataset.load_pointcloud(indices[0].first, cloud);

    sprintf(cloud_name, "%05d", indices[0].first);
    viewer->addPointCloud(cloud, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    draw_pose(viewer, pose);

    for (int i = 0; i < relative_transform_vector.size(); i++)
    {
        cloud->clear();
        dataset.load_pointcloud(indices[i].second, cloud);
        pose *= relative_transform_vector[i];

    	pcl::transformPointCloud(*cloud, *moved, pose);
        sprintf(cloud_name, "%05d", indices[i].second);
        viewer->addPointCloud(moved, cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
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