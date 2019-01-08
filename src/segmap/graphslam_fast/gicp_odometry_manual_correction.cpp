
#include <cstdio>
#include <cstdlib>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_pose2d.h>
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
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) && 
			raw_cloud->at(i).x < 70.0) // && raw_cloud->at(i).z > -1.)
			cloud->push_back(raw_cloud->at(i));
	}

	return cloud;
}


void
read_odom_data(char *filename, vector<pair<int,int>> &indices,
	vector<Pose2d> &relative_transform_vector,
	vector<int> &convergence_vector)
{
    FILE *f = fopen(filename, "r");

    if (f == NULL)
        exit(printf("Error: file '%s' not found.\n", filename));

    int from, to, converged, n;

    while (!feof(f))
    {
	    Pose2d relative_pose;
    
	    n = fscanf(f, "%d %d %d %lf %lf %lf\n", 
            &from, &to, &converged,
            &relative_pose.x, &relative_pose.y, &relative_pose.th);

        if (n != 6)
            continue;

        indices.push_back(pair<int, int>(from, to));
        convergence_vector.push_back(converged);
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


void
redraw_cloud(PointCloud<PointXYZRGB>::Ptr source, pcl::visualization::PCLVisualizer *viewer, Matrix<double, 4, 4> pose)
{
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);

	pcl::transformPointCloud(*source, *aligned, pose);

	viewer->removePointCloud("aligned");
	viewer->addPointCloud(aligned, "aligned");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "aligned");  // blue

}


void
update_file(char *filename, vector<Pose2d> &corrected_poses, vector<pair<int,int>> &indices, vector<int> &convergence_vector)
{
	FILE *f = fopen(filename, "w");

	for (int i = 0; i < corrected_poses.size(); i++)
	{
		fprintf(f, "%d %d %d %lf %lf %lf\n",
			indices[i].first, indices[i].second, convergence_vector[i],
			corrected_poses[i].x, corrected_poses[i].y, corrected_poses[i].th);
	}

	fclose(f);
}


int 
main(int argc, char **argv)
{
    if (argc < 3)
        exit(printf("Error: Use %s <dataset_dir> <odom_file>\n", argv[0]));

    vector<pair<int,int>> indices;
	vector<Pose2d> relative_transform_vector;
	vector<int> convergence_vector;

    read_odom_data(argv[2], indices, relative_transform_vector, convergence_vector);
    DatasetCarmen dataset(argv[1], 0);

    int pause_viewer = 1;
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
	viewer->setBackgroundColor(.5, .5, .5);

	int i;
	int direction = 0;

	vector<Pose2d> corrected_poses(relative_transform_vector.size());

	for (i = 0; i < relative_transform_vector.size(); i++)
		corrected_poses[i] = relative_transform_vector[i];

	i = 0;
	Pose2d mem(0., 0., 0.);

    while (1)
    {
        printf("Cloud %d of %ld Converged: %d\n", i, relative_transform_vector.size(), convergence_vector[i]);

    	viewer->removeAllPointClouds();

	    PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	    PointCloud<PointXYZRGB>::Ptr source_moved(new PointCloud<PointXYZRGB>);
	    PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);

        dataset.load_pointcloud(indices[i].first, target);
        dataset.load_pointcloud(indices[i].second, source);

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

    	pcl::transformPointCloud(*source, *source_moved, guess);

        viewer->addPointCloud(source_moved, "source");
        viewer->addPointCloud(target, "target");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source"); // red
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target"); // green

        imshow("viewer", Mat::zeros(300, 300, CV_8UC3));

		Pose2d corrected_pose = corrected_poses[i];
		redraw_cloud(source, viewer, Pose2d::to_matrix(corrected_pose));

		char c = 0;

		while (1)
		{
			//viewer2->spinOnce();
			viewer->spinOnce();
			c = waitKey(5);

			if (c == ' ')
				pause_viewer = !pause_viewer;

			if (c == 'u')
				update_file(argv[2], corrected_poses, indices, convergence_vector);

			if ((!pause_viewer && direction == 0) || (pause_viewer && c == 83)) // right arrow
			{
				i++;

				if (i >= relative_transform_vector.size())
					i = 0;

				direction = 0;
				break;
			}
			if ((!pause_viewer && direction != 0) || (pause_viewer && c == 81)) // left arrow
			{
				i--;

				if (i < 0)
					i = relative_transform_vector.size() - 1;

				direction = 1;
				break;
			}

			int updated = 0;

			if (c == '1')
			{
				mem.x = corrected_pose.x - relative_transform_vector[i].x;
				mem.y = corrected_pose.y - relative_transform_vector[i].y;
				mem.th = normalize_theta(corrected_pose.th - relative_transform_vector[i].th);
			}

			if (c == '0')
			{
				corrected_pose.x = mem.x + relative_transform_vector[i].x;
				corrected_pose.y = mem.y + relative_transform_vector[i].y;
				corrected_pose.th = normalize_theta(mem.th + relative_transform_vector[i].th);
				updated = 1;
			}

			// reset
			if (c == 'h')
			{
				corrected_pose = relative_transform_vector[i];
				updated = 1;
			}

			if (c == 'a') { corrected_pose.x -= 0.1; updated = 1; } 
			if (c == 'd') { corrected_pose.x += 0.1; updated = 1; } 
			if (c == 's') { corrected_pose.y -= 0.1; updated = 1; } 
			if (c == 'w') { corrected_pose.y += 0.1; updated = 1; } 
			if (c == 'q') { corrected_pose.th = normalize_theta(corrected_pose.th - degrees_to_radians(0.5)); updated = 1; } 
			if (c == 'e') { corrected_pose.th = normalize_theta(corrected_pose.th + degrees_to_radians(0.5)); updated = 1; } 

			if (updated)
			{
				corrected_poses[i] = corrected_pose;
				redraw_cloud(source, viewer, Pose2d::to_matrix(corrected_pose)); 
			}
		} 
    }

    viewer->spin();
    return 0;
}
