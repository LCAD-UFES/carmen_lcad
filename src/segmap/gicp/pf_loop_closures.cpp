
#include <vector>
#include <string>
#include <algorithm>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <carmen/util_io.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/carmen_lidar_reader.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_constructors.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter.h>
#include <carmen/segmap_particle_filter_viewer.h>

#include <carmen/command_line.h>
#include "gicp.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace cv;


void
update_map(DataSample *sample, GridMap *map, SensorPreproc &preproc)
{
	preproc.reinitialize(sample);

	for (int i = 0; i < preproc.size(); i++)
	{
		vector<PointXYZRGB> points = preproc.next_points_in_world();

		for (int j = 0; j < points.size(); j++)
			map->add_point(points[j]);
	}
}


double
compute_percentage_of_points_that_hit_map(DataSample *sample, SensorPreproc &preproc, GridMap &map)
{
	std::vector<double> cell;

	double percentage_points_that_hit_map = 0.;
	int n_points_that_hit_map = 0;
	int n_points_total = 0;

	preproc.reinitialize(sample);

	for (int i = 0; i < preproc.size(); i++)
	{
		vector<PointXYZRGB> points = preproc.next_points_in_world();

		for (int j = 0; j < points.size(); j++)
		{
			std::vector<double> cell = map.read_cell(points[j]);

			if (cell[3] > 1)
				n_points_that_hit_map++;

			n_points_total++;
		}
	}

	if (n_points_total > 0)
		percentage_points_that_hit_map = ((double) n_points_that_hit_map / (double) n_points_total);

	return percentage_points_that_hit_map;
}


Pose2d
estimate_pose_with_particle_filter(DataSample *sample,
																	 SensorPreproc &preproc,
																	 GridMap &map,
																	 ParticleFilter &pf,
																	 double dt,
																	 int pf_reinit_required,
																	 Pose2d &pose_guess,
																	 Mat *pf_viewer_img,
																	 int view)
{
	int n_correction_steps = 0;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	if (pf_reinit_required)
	{
		pf.reset(pose_guess.x, pose_guess.y, pose_guess.th);
		n_correction_steps = 5;
	}
	else
	{
		pf.predict(sample->v, sample->phi, dt);
		n_correction_steps = 1;
	}

	preproc.reinitialize(sample);
	load_as_pointcloud(preproc, cloud, SensorPreproc::CAR_REFERENCE);

	for (int i = 0; i < n_correction_steps; i++)
		pf.correct(cloud, map, sample->gps);

	if (view)
		(*pf_viewer_img) = pf_view(pf, map, pose_guess, cloud, 1);

	return pf.mean();
}


void
run_loop_closure_estimation(NewCarmenDataset &dataset, SensorPreproc &preproc, GridMap &map,
														ParticleFilter &pf, CommandLineArguments &args,
														vector<pair<int, int>> *loop_closure_indices,
														vector<Matrix<double, 4, 4>> *relative_transform_vector,
														double intersection_threshold_for_loop_closure_detection,
														int view)
{
	Mat viewer_img;
	DataSample *sample;
	double percentage_points_that_hit_map;
	Pose2d estimate;

	PointCloudViewer viewer;
	Pose2d offset = dataset[0]->pose;
	int step = args.get<int>("step");
	double skip_velocity_threshold = args.get<double>("v_thresh");
	int pf_reinit_required = 1;

	vector<int> loop_indices;
	vector<Pose2d> estimated_poses;

	for (int i = step; i < dataset.size(); i += step)
	{
		sample = dataset[i];

		if (fabs(sample->v) < skip_velocity_threshold)
			continue;

		Pose2d current_pose = sample->pose;
		current_pose.x -= offset.x;
		current_pose.y -= offset.y;

		map.reload(current_pose.x, current_pose.y);

		// if there is enough intersection with the map, assume we detected a loop closure.
		// the displacement is estimated using the particle filter.
		percentage_points_that_hit_map =
				compute_percentage_of_points_that_hit_map(sample, preproc, map);

		printf("percentage_points_that_hit_map: %lf\n", percentage_points_that_hit_map);

		if (percentage_points_that_hit_map > intersection_threshold_for_loop_closure_detection)
		{
			double dt = sample->time - dataset[i - step]->time;

			estimate = estimate_pose_with_particle_filter(sample, preproc, map, pf, dt,
																										pf_reinit_required, current_pose,
																										&viewer_img, view);

			loop_indices.push_back(i);
			estimated_poses.push_back(estimate);

			printf("LOOP %d %lf %lf %lf\n", i, estimate.x, estimate.y, estimate.th);
			sample->pose = estimate;
			sample->pose.x += offset.x;
			sample->pose.y += offset.y;
			current_pose = estimate;

			pf_reinit_required = 0;
		}
		else
		{
			pf_reinit_required = 1;
		}

		update_map(sample, &map, preproc);

		if (pf_reinit_required && view)
		{
			viewer_img = map.to_image().clone();
			draw_pose(map, viewer_img, current_pose, Scalar(0, 255, 0));
		}

		if (view)
		{
			Mat flipped;
			flip(viewer_img, flipped, 0);
			viewer.show(flipped, "map", 640);
			viewer.loop();
		}
	}
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;

	add_default_slam_args(args);
	args.add_positional<std::string>("output", "Path to the output file", 1);
	args.add<std::string>("odom_calib,o", "Odometry calibration file", "");
	args.add<std::string>("fused_odom,f", "Fused odometry file (optimized using graphslam)", "");
	args.add<double>("inter_thresh", "Threshold for percentage of points that thit map for detecting a loop closure [0-1]", 0.5);
	args.add<int>("view,v", "Flag to indicate if viewer should be on (potentially reducing performance)", 1);

	add_default_sensor_preproc_args(args);
	add_default_mapper_args(args);
	add_default_localizer_args(args);

	args.save_config_file(default_data_dir() + "/pf_loop_closures_config.txt");
	args.parse(argc, argv);

	NewCarmenDataset dataset(args.get<string>("log_path"),
	                         args.get<string>("odom_calib"),
	                         args.get<string>("fused_odom"),
	                         args.get<int>("gps_id"));

	SensorPreproc preproc = create_sensor_preproc(args, &dataset, args.get<string>("log_path"));
	GridMap map = create_grid_map(args, 1);
	ParticleFilter pf = create_particle_filter(args);

	vector<pair<int, int>> loop_closure_indices;
	vector<Matrix<double, 4, 4>> relative_transform_vector;

	run_loop_closure_estimation(dataset, preproc, map, pf, args,
															&loop_closure_indices,
															&relative_transform_vector,
															args.get<double>("inter_thresh"),
															args.get<int>("view"));

	printf("Done.");
	return 0;
}
