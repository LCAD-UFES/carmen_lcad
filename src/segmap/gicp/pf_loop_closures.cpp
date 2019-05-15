
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
#include <carmen/util_math.h>

#include <carmen/command_line.h>
#include "gicp.h"
#include <carmen/segmap_loop_closures.h>


using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace cv;

enum LoopClosureDetectionMethod
{
	DETECT_USING_INTERSECTION_WITH_MAP = 0,
	DETECT_BY_DIST_AND_TIME,
};


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
																	 PointCloudViewer &viewer,
																	 int n_correction_steps_when_reinitializing,
																	 int view)
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	preproc.reinitialize(sample);
	load_as_pointcloud(preproc, cloud, SensorPreproc::CAR_REFERENCE);

	if (pf_reinit_required)
	{
		printf("* Reinitializing\n");
		//viewer.set_step(1);
		pf.reset(pose_guess.x, pose_guess.y, pose_guess.th);
		run_viewer_if_necessary(&pose_guess, map, pf, cloud, viewer, 1, 1, view);

		for (int i = 0; i < n_correction_steps_when_reinitializing; i++)
		{
			printf("** Step %d - Prediction\n", i);
			// the prediction is just to add a little bit of gaussian noise
			// in every correction step.
			pf.predict(0, 0, 0);
			run_viewer_if_necessary(&pose_guess, map, pf, cloud, viewer, 1, 1, view);

			printf("** Step %d - Correction\n", i);
			pf.correct(cloud, map, sample->gps);
			run_viewer_if_necessary(&pose_guess, map, pf, cloud, viewer, 1, 1, view);
		}

		printf("\n");
	}
	else
	{
		printf("* Prediction\n");
		pf.predict(sample->v, sample->phi, dt);
		run_viewer_if_necessary(&pose_guess, map, pf, cloud, viewer, 1, 1, view);

		printf("* Correction\n");
		pf.correct(cloud, map, sample->gps);
		run_viewer_if_necessary(&pose_guess, map, pf, cloud, viewer, 1, 1, view);

		printf("\n");
	}

	return pf.mean();
}


int
search_for_loop_closures(DataSample *sample, int sample_id,
												 NewCarmenDataset &dataset,
												 SensorPreproc &preproc, GridMap &map,
												 double intersection_threshold_for_loop_closure_detection,
												 LoopClosureDetectionMethod method,
												 double max_dist_threshold,
												 double min_time_threshold)
{
	int nn_id = -1;

	if (method == DETECT_USING_INTERSECTION_WITH_MAP)
	{
		double percentage_points_that_hit_map;

		// if there is enough intersection with the map, assume we detected a loop closure.
		percentage_points_that_hit_map =
				compute_percentage_of_points_that_hit_map(sample, preproc, map);

		printf("percentage_points_that_hit_map: %lf\n", percentage_points_that_hit_map);

		if (percentage_points_that_hit_map > intersection_threshold_for_loop_closure_detection)
		{
			// the loop closure constraint is created in relation to the nearest pose in previous
			// visits to the region. Different from the next case, we do not impose restrictions
			// on distance and time difference between poses.

			search_for_loop_closure_using_pose_dist(dataset, sample->pose, sample->time,
			                                        0, sample_id, DBL_MAX, -DBL_MAX, &nn_id);
		}
	}
	else if (method == DETECT_BY_DIST_AND_TIME)
	{
		search_for_loop_closure_using_pose_dist(dataset, sample->pose, sample->time,
		                                        0, sample_id, max_dist_threshold,
		                                        min_time_threshold, &nn_id);
	}
	else
		exit(printf("Error: Invalid detection method '%d'\n", method));

	return nn_id;
}


Pose2d
compute_relative_pose(Pose2d &reference, Pose2d &pose)
{
	Matrix<double, 4, 4> mat = Pose2d::to_matrix(reference).inverse() * Pose2d::to_matrix(pose);
	return Pose2d::from_matrix(mat);
}


void
run_loop_closure_estimation(NewCarmenDataset &dataset, SensorPreproc &preproc, GridMap &map,
														ParticleFilter &pf,
														vector<pair<int, int>> *loop_closure_indices,
														vector<Pose2d> *relative_transform_vector,
														double intersection_threshold_for_loop_closure_detection,
						                double max_dist_threshold,
						                double min_time_threshold,
														int n_corrections_when_reinit,
														int step,
														double v_thresh,
														int view,
														Pose2d offset)
{
	DataSample *sample;
	Pose2d estimate;

	int loop_id;
	PointCloudViewer viewer;
	int pf_reinit_required = 1;

	vector<int> loop_indices;
	vector<Pose2d> estimated_poses;

	if (step <= 0) step = 1;

	for (int i = step; i < dataset.size(); i += step)
	{
		sample = dataset[i];

		if (fabs(sample->v) < v_thresh)
			continue;

		Pose2d current_pose = sample->pose;
		current_pose.x -= offset.x;
		current_pose.y -= offset.y;

		map.reload(current_pose.x, current_pose.y);

		loop_id = search_for_loop_closures(sample, i, dataset, preproc, map,
																			 intersection_threshold_for_loop_closure_detection,
																			 DETECT_BY_DIST_AND_TIME,
												               max_dist_threshold,
												               min_time_threshold);

		if (loop_id >= 0)
		{
			double dt = sample->time - dataset[i - step]->time;

			estimate = estimate_pose_with_particle_filter(sample, preproc, map, pf, dt,
																										pf_reinit_required, current_pose,
																										viewer,
																										n_corrections_when_reinit,
																										view);

			Pose2d loop_pose = dataset[loop_id]->pose;
			loop_pose.x -= offset.x;
			loop_pose.y -= offset.y;

			loop_closure_indices->push_back(pair<int, int>(loop_id, i));
			relative_transform_vector->push_back(compute_relative_pose(loop_pose, estimate));

			pf_reinit_required = 0;
		}
		else
		{
			update_map(sample, &map, preproc);
			run_viewer_if_necessary(&current_pose, map, pf, NULL, viewer, 0, 1, view);
			pf_reinit_required = 1;
		}
	}
}


void
save_output(string path,
						vector<pair<int, int>> &loop_closure_indices,
						vector<Pose2d> &relative_transform_vector)
{
	FILE *f = safe_fopen(path.c_str(), "w");

	for (int i = 0; i < loop_closure_indices.size(); i++)
	{
		fprintf(f, "%d %d 1 %lf %lf %lf\n",
						loop_closure_indices[i].first,
						loop_closure_indices[i].second,
						relative_transform_vector[i].x,
						relative_transform_vector[i].y,
						relative_transform_vector[i].th);
	}

	fclose(f);
}


void
remove_previous_map(string path)
{
	if (boost::filesystem::exists(path))
		boost::filesystem::remove_all(path);
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;

	args.add_positional<std::string>("log_path", "Path to a log", 1);
	args.add_positional<string>("param_file", "Path to the carmen.ini file", 1);
	args.add<std::string>("odom_calib,o", "Odometry calibration file", "none");
	args.add<std::string>("fused_odom,f", "Fused odometry file (optimized using graphslam)", "none");
	args.add<double>("inter_thresh", "Threshold for percentage of points that thit map for detecting a loop closure [0-1]", 0.97);
	args.add<int>("n_corrections_when_reinit", "Number of correction steps when reinitializing particle filter", 5);

	add_default_sensor_preproc_args(args);
	add_default_mapper_args(args);
	add_default_localizer_args(args);
	add_default_gicp_args(args);

	args.save_config_file(default_data_dir() + "/pf_loop_closures_config.txt");
	args.parse(argc, argv);

	NewCarmenDataset *dataset = create_dataset(args.get<string>("log_path"), args, "fused_odom");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, args.get<string>("log_path"));
	remove_previous_map(args.get<string>("map_path"));
	GridMap map = create_grid_map(args, 1);
	ParticleFilter pf = create_particle_filter(args);

	vector<pair<int, int>> loop_closure_indices;
	vector<Pose2d> relative_transform_vector;

	Pose2d offset = Pose2d(args.get<double>("offset_x"),
												 args.get<double>("offset_y"), 0);

	run_loop_closure_estimation(*dataset, preproc, map, pf,
															&loop_closure_indices,
															&relative_transform_vector,
															args.get<double>("inter_thresh"),
															args.get<double>("loop_dist"),
															args.get<double>("time_dist"),
															args.get<int>("n_corrections_when_reinit"),
															args.get<int>("subsampling"),
															args.get<double>("v_thresh"),
															args.get<int>("view"),
															offset);

	save_output(args.get<string>("output"),
	            loop_closure_indices,
	            relative_transform_vector);

	printf("Done.");
	return 0;
}
