
#include <vector>
#include <string>
#include <algorithm>
#include <Eigen/Core>
#include "gicp.h"
#include <carmen/segmap_util.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_command_line.h>

using namespace std;
using namespace pcl;
using namespace Eigen;


PointCloud<PointXYZRGB>::Ptr
filter_pointcloud(PointCloud<PointXYZRGB>::Ptr raw_cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloud = PointCloud<PointXYZRGB>::Ptr(
	    new PointCloud<PointXYZRGB>);
	cloud->clear();

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0)
		    && raw_cloud->at(i).x < 70.0  // remove max range
		    && raw_cloud->at(i).z > -1.3  // remove ground
		    && raw_cloud->at(i).z < -0.  // remove tree tops
		        )
			cloud->push_back(raw_cloud->at(i));
	}

	return cloud;
}


PointCloud<PointXYZRGB>::Ptr
create_cloud(DatasetCarmen &dataset, int id, Pose2d &target_pose)
{
	PointCloud<PointXYZRGB>::Ptr raw_cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr raw_moved(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr out_cloud(new PointCloud<PointXYZRGB>);

	for (int i = -5; i < 6; i++)
	{
		dataset.load_pointcloud(id + i, raw_cloud, dataset.data[id + i].v,
		                        dataset.data[id + i].phi);

		raw_cloud = filter_pointcloud(raw_cloud);
		Pose2d pose = dataset.data[id + i].pose;
		pose.x -= target_pose.x;
		pose.y -= target_pose.y;

		Matrix<double, 4, 4>tr = pose3d_to_matrix(0., 0., target_pose.th).inverse()
		    * Pose2d::to_matrix(pose);

		pcl::transformPointCloud(*raw_cloud, *raw_moved, tr);
		(*out_cloud) += (*raw_moved);
	}

	return out_cloud;
}


void
run_icp_step(DatasetCarmen &dataset, int from, int to,
             Matrix<double, 4, 4>*relative_transform, int *convergence_flag)
{
	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);
	//PointCloud<PointXYZRGB>::Ptr aligned2(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr source_moved(new PointCloud<PointXYZRGB>);

	Pose2d target_pose = dataset.data[from].pose;

	target = create_cloud(dataset, from, target_pose);
	source = create_cloud(dataset, to, target_pose);

	run_gicp(source, target, relative_transform, convergence_flag, aligned, 0.1);
	//pcl::transformPointCloud(*source, *aligned2, ((*relative_transform) * guess).cast<float>());

	///*
	if (1)
	{
		static pcl::visualization::PCLVisualizer *viewer = NULL;

		if (viewer == NULL)
			viewer = new pcl::visualization::PCLVisualizer();

		viewer->removeAllPointClouds();
		viewer->setBackgroundColor(.5, .5, .5);
		viewer->addPointCloud(target, "target");
		viewer->addPointCloud(source, "source");
		viewer->addPointCloud(aligned, "aligned");
		//viewer->addPointCloud(aligned2, "aligned2d");
		viewer->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
		viewer->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		viewer->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned2d");
		viewer->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source");
		viewer->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target");
		viewer->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "aligned");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "aligned2d");
		viewer->spinOnce();
	}
	//*/
}


void
write_output(FILE *out_file, vector<pair<int, int>>&loop_closure_indices,
             vector<Matrix<double, 4, 4>>&relative_transform_vector,
             vector<int>&convergence_vector)
{
	for (int i = 0; i < loop_closure_indices.size(); i++)
	{
		fprintf(
		    out_file,
		    "%d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
		    loop_closure_indices[i].first, loop_closure_indices[i].second,
		    convergence_vector[i], relative_transform_vector[i](0, 0),
		    relative_transform_vector[i](0, 1), relative_transform_vector[i](0, 2),
		    relative_transform_vector[i](0, 3), relative_transform_vector[i](1, 0),
		    relative_transform_vector[i](1, 1), relative_transform_vector[i](1, 2),
		    relative_transform_vector[i](1, 3), relative_transform_vector[i](2, 0),
		    relative_transform_vector[i](2, 1), relative_transform_vector[i](2, 2),
		    relative_transform_vector[i](2, 3), relative_transform_vector[i](3, 0),
		    relative_transform_vector[i](3, 1), relative_transform_vector[i](3, 2),
		    relative_transform_vector[i](3, 3));
	}
}


void
write_output_to_graphslam(
    char *out_file, DatasetCarmen &dataset, vector<pair<int, int>>&indices,
    vector<Matrix<double, 4, 4>>&relative_transform_vector,
    vector<int>&convergence_vector)
{
	FILE *f = fopen(out_file, "w");

	for (int i = 0; i < indices.size(); i++)
	{
		Pose2d pose_target = dataset.data[indices[i].first].pose;
		Pose2d pose_source = dataset.data[indices[i].second].pose;

		pose_source.x -= pose_target.x;
		pose_source.y -= pose_target.y;
		//pose_source.x = 0;
		//pose_source.y = 0;
		pose_target.x = 0.;
		pose_target.y = 0.;

		Matrix<double, 4, 4>guess = Pose2d::to_matrix(pose_target).inverse()
		    * Pose2d::to_matrix(pose_source);

		Matrix<double, 4, 4>relative_pose = relative_transform_vector[i] * guess;
		Pose2d pose = Pose2d::from_matrix(relative_pose);

		fprintf(f, "%d %d %d %lf %lf %lf\n", indices[i].first, indices[i].second,
		        convergence_vector[i], pose.x, pose.y, pose.th);
	}

	fclose(f);
}


void
detect_loop_closures(NewCarmenDataset &dataset, vector<pair<int, int>> *loop_closure_indices,
                     double dist_threshold, double time_threshold, int step)
{
	for (int i = 0; i < dataset.data.size(); i += step)
	{
		double min_dist = DBL_MAX;
		int nn_id = -1;

		for (int j = i + 1; j < dataset.data.size(); j++)
		{
			double dx = dataset.data[i].pose.x - dataset.data[j].pose.x;
			double dy = dataset.data[i].pose.y - dataset.data[j].pose.y;
			double dt = fabs(dataset.data[i].velodyne_time - dataset.data[j].velodyne_time);

			double dist = sqrt(pow(dx, 2) + pow(dy, 2));

			// search for the nearest pose that obeys the distance and time constraints.
			if ((dist < min_dist) && (dist < dist_threshold) && (dt > time_threshold))
			{
				min_dist = dist;
				nn_id = j;
			}
		}

		if (nn_id >= 0)
			loop_closure_indices.push_back(pair<int, int>(i, nn_id));
	}

	printf("Num loop closures: %ld\n", loop_closure_indices.size());
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;

	args.add_positional<string>("log-path", "Path of a log", 1);
	args.add_positional<string>("output", "Path of the output file", 1);
	args.add<double>("voxel_size,v", "Size of voxels in voxel grid filter", 0.2);
	args.add<double>("loop_dist,d", "Maximum distance (in meters) to assume two poses form a loop closure", 2.0);
	args.add<double>("time_dist,t", "Minimum temporal difference (in seconds) to assume two poses form a loop closure (instead of being consecutive poses)", 60.0);
	args.add<int>("subsampling,s", "Number of data packages to jump when searching for loop closures (<= 1 for using all packages)", 0);
	args.save_config_file("loop_closures_config.txt");
	args.parse(argc, argv);

	string log_path;
	string odom_calib_path;

	const char *log_path = args.get<string>("log-path");
	vector<char*> splitted = string_split(log_path, "/");
	char *log_name = splitted[splitted.size() - 1];
	const char *odom_calib_path = (string("/data/data2/data_") + string(log_name) + string("/odom_calib.txt")).c_str();

	NewCarmenDataset dataset(log_path.c_str(), odom_calib_path);
	NewCarmenDataset dataset_for_iterating(log_path.c_str(), odom_calib_path);

	FILE *report_file = safe_fopen(args.get<string>("output").c_str(), "w");

	if (out_file == NULL)
		exit(printf("Output file '%s' could not be open.\n", argv[2]));

	int size = dataset.data.size() - 1;

	vector<Matrix<double, 4, 4>>relative_transform_vector(size);
	vector<int>convergence_vector(size);
	vector<pair<int, int>>loop_closure_indices;

	printf("Running.\n");
	detect_loop_closures(dataset, dataset_for_iterating,
	                     &loop_closure_indices,
	                     args.get<double>("loop_dist"),
	                     args.get<double>("time_dist"),
	                     args.get<int>("subsampling"));

	int n_processed_clouds = 0;
	int n = loop_closure_indices.size();
	// int n = 400;

	//#pragma omp parallel for default(none) private(i) shared(dataset, convergence_vector, relative_transform_vector, size, loop_closure_indices, n_processed_clouds, n)
	for (i = 0; i < n; i++)
	{
		run_icp_step(dataset, loop_closure_indices[i].first,
		             loop_closure_indices[i].second,
		             &(relative_transform_vector[i]), &(convergence_vector[i]));

		//#pragma omp critical
		{
			n_processed_clouds++;

			if (n_processed_clouds % 100 == 0)
				printf("%d processed clouds of %d\n", n_processed_clouds, n);
		}
	}

	write_output(out_file, loop_closure_indices, relative_transform_vector,
	             convergence_vector);
	write_output_to_graphslam(argv[3], dataset, loop_closure_indices,
	                          relative_transform_vector, convergence_vector);

	fclose (out_file);

	return 0;
}
