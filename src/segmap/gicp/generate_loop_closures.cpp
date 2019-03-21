
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
#include <carmen/segmap_sensors.h>
#include <carmen/segmap_viewer.h>

using namespace std;
using namespace pcl;
using namespace Eigen;


PointCloud<PointXYZRGB>::Ptr
filter_pointcloud(PointCloud<PointXYZRGB>::Ptr raw_cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);
	cloud->clear();

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) // remove rays that hit car
		    && raw_cloud->at(i).x < 70.0  // remove max range
		    && raw_cloud->at(i).z > -1.3  // remove ground
		    && raw_cloud->at(i).z < -0.  // remove tree tops
		        )
			cloud->push_back(raw_cloud->at(i));
	}

	return cloud;
}


Matrix<double, 4, 4>
compute_vel2target(Pose2d pose, Pose2d &target, Matrix<double, 4, 4> &vel2car)
{
	// to prevent numerical issues
	pose.x -= target.x;
	pose.y -= target.y;

	Matrix<double, 4, 4> world2target = pose3d_to_matrix(0., 0., target.th).inverse();
	Matrix<double, 4, 4> source2world = Pose2d::to_matrix(pose);
	Matrix<double, 4, 4> vel2target = world2target * source2world * vel2car;

	return vel2target;
}


PointCloud<PointXYZRGB>::Ptr
create_cloud(NewCarmenDataset &dataset, int id, Pose2d &target_pose, Matrix<double, 4, 4> &vel2car)
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr moved(new PointCloud<PointXYZRGB>);

	CarmenLidarLoader loader(dataset[id]->velodyne_path.c_str(),
													 dataset[id]->n_laser_shots,
													 dataset.intensity_calibration);

	load_as_pointcloud(&loader, cloud);
	cloud = filter_pointcloud(cloud);

	Pose2d pose = dataset[id]->pose;
	Matrix<double, 4, 4> vel2target = compute_vel2target(pose, target_pose, vel2car);

	pcl::transformPointCloud(*cloud, *moved, vel2target);

	return moved;
}


void
run_icp_step(NewCarmenDataset &dataset, int from, int to,
						 Matrix<double, 4, 4> &vel2car,
						 Matrix<double, 4, 4> *relative_transform,
						 int *convergence_flag,
						 bool view = false)
{
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);

	Pose2d target_pose = dataset[from]->pose;

	target = create_cloud(dataset, from, target_pose, vel2car);
	source = create_cloud(dataset, to, target_pose, vel2car);

	run_gicp(source, target, relative_transform, convergence_flag, aligned, 0.1);

	if (view)
	{
		//PointCloud<PointXYZRGB>::Ptr aligned2(new PointCloud<PointXYZRGB>);
		//PointCloud<PointXYZRGB>::Ptr source_moved(new PointCloud<PointXYZRGB>);

		static PointCloudViewer *viewer = NULL;

		if (viewer == NULL)
			viewer = new PointCloudViewer(4);

		viewer->clear();
		viewer->show(target, 0, 1, 0);
		viewer->show(source, 1, 0, 0);
		viewer->show(aligned, 0, 0, 1);
		viewer->loop();
	}
}


void
write_output(FILE *report_file, vector<pair<int, int>> &loop_closure_indices,
             vector<Matrix<double, 4, 4>> &relative_transform_vector,
             vector<int> &convergence_vector)
{
	for (int i = 0; i < loop_closure_indices.size(); i++)
	{
		fprintf(
				report_file,
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
write_output_to_graphslam(FILE *out_file,
													NewCarmenDataset &dataset, vector<pair<int, int>> &indices,
													vector<Matrix<double, 4, 4>> &relative_transform_vector,
													vector<int> &convergence_vector)
{
	Matrix<double, 4, 4> vel2target;
	Matrix<double, 4, 4> vel2car = dataset.vel2car();
	Matrix<double, 4, 4> corrected_pose;

	for (int i = 0; i < indices.size(); i++)
	{
		Pose2d pose_target = dataset[indices[i].first]->pose;
		Pose2d pose_source = dataset[indices[i].second]->pose;

		vel2target = compute_vel2target(pose_source, pose_target, vel2car);
		corrected_pose = relative_transform_vector[i] * vel2target;
		Pose2d pose = Pose2d::from_matrix(corrected_pose);

		fprintf(out_file, "%d %d %d %lf %lf %lf\n", indices[i].first, indices[i].second,
		        convergence_vector[i], pose.x, pose.y, pose.th);
	}
}


void
detect_loop_closures(NewCarmenDataset &dataset, vector<pair<int, int>> *loop_closure_indices,
                     double dist_threshold, double time_threshold, int step)
{
	printf("Detecting loop closures.\n");

	if (step <= 0)
		step = 1;

	for (int i = 0; i < dataset.size(); i += step)
	{
		double min_dist = DBL_MAX;
		int nn_id = -1;

		for (int j = i + 1; j < dataset.size(); j++)
		{
			double dx = dataset[i]->pose.x - dataset[j]->pose.x;
			double dy = dataset[i]->pose.y - dataset[j]->pose.y;
			double dt = fabs(dataset[i]->velodyne_time - dataset[j]->velodyne_time);

			double dist = sqrt(pow(dx, 2) + pow(dy, 2));

			// search for the nearest pose that obeys the distance and time constraints.
			if ((dist < min_dist) && (dist < dist_threshold) && (dt > time_threshold))
			{
				min_dist = dist;
				nn_id = j;
			}
		}

		if (nn_id >= 0)
			loop_closure_indices->push_back(pair<int, int>(i, nn_id));
	}

	printf("Num loop closures found: %ld\n", loop_closure_indices->size());
}


void
add_arguments_for_parsing(CommandLineArguments *args)
{
	args->add_positional<string>("log_path", "Path of a log", 1);
	args->add_positional<string>("output", "Path of the output file", 1);
	args->add<double>("voxel_size,x", "Size of voxels in voxel grid filter", 0.1);
	args->add<double>("loop_dist,d", "Maximum distance (in meters) to assume two poses form a loop closure", 2.0);
	args->add<double>("time_dist,t", "Minimum temporal difference (in seconds) to assume two poses form a loop closure (instead of being consecutive poses)", 60.0);
	args->add<int>("subsampling,s", "Number of data packages to skip when looking for loop closures (<= 1 for using all packages)", 0);
	args->add<string>("report_file,r", "Path to a file to save debug information", "/tmp/loop_closure_report.txt");
	args->add<bool>("view,v", "Boolean to turn visualization on or off", false);
	args->save_config_file("loop_closures_config.txt");
}


void
run_icps(NewCarmenDataset &dataset,
				 vector<pair<int, int>> &loop_closure_indices,
				 vector<Matrix<double, 4, 4>> *relative_transform_vector,
				 vector<int> *convergence_vector,
				 bool view)
{
	printf("Running ICPs.\n");

	int i;
	int n_processed_clouds = 0;
	int n = loop_closure_indices.size();

	Matrix<double, 4, 4> vel2car = dataset.vel2car();

#ifdef _OPENMP
	#pragma omp parallel for default(none) private(i) \
		shared(dataset, convergence_vector, relative_transform_vector, \
		loop_closure_indices, n_processed_clouds, n, vel2car, view)
#endif
	for (i = 0; i < n; i++)
	{
		run_icp_step(dataset, loop_closure_indices[i].first,
		             loop_closure_indices[i].second,
								 vel2car,
		             &(relative_transform_vector->at(i)),
								 &(convergence_vector->at(i)),
								 view);

#ifdef _OPENMP
		#pragma omp critical
#endif
		{
			n_processed_clouds++;

			if (n_processed_clouds % 100 == 0)
				printf("%d processed clouds of %d\n", n_processed_clouds, n);
		}
	}
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;
	add_arguments_for_parsing(&args);
	args.parse(argc, argv);

	bool view = args.get<bool>("view");
	string log_path = args.get<string>("log_path");
	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());

	FILE *out_file = safe_fopen(args.get<string>("output").c_str(), "w");
	FILE *report_file = safe_fopen(args.get<string>("report_file").c_str(), "w");

	NewCarmenDataset dataset(log_path, odom_calib_path, fused_odom_path);

	vector<pair<int, int>> loop_closure_indices;

	detect_loop_closures(dataset,
	                     &loop_closure_indices,
	                     args.get<double>("loop_dist"),
	                     args.get<double>("time_dist"),
	                     args.get<int>("subsampling"));

	int size = dataset.size();
	vector<Matrix<double, 4, 4>> relative_transform_vector(size);
	vector<int> convergence_vector(size);

#ifdef _OPENMP
	view = false;
#endif

	run_icps(dataset,
					 loop_closure_indices,
					 &relative_transform_vector,
					 &convergence_vector,
					 view);

	write_output(report_file, loop_closure_indices, relative_transform_vector,
	             convergence_vector);

	write_output_to_graphslam(out_file, dataset, loop_closure_indices,
	                          relative_transform_vector, convergence_vector);

	fclose (out_file);
	fclose (report_file);

	printf("Done.");
	return 0;
}
