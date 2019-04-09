
#include <string>
#include <opencv/cv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <carmen/util_math.h>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_particle_filter.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_particle_filter_viewer.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_constructors.h>

using namespace cv;
using namespace std;
using namespace pcl;


void
viewer(DataSample *sample, ParticleFilter &pf, GridMap &map, int step, int n_total_steps,
			 PointCloud<PointXYZRGB>::Ptr cloud, Pose2d offset,
			 PointCloudViewer &s_viewer)
{
	Mat view_img;
	Mat pf_view_img;

	Pose2d gt_pose = sample->pose;

	//6979 400 7757735.655524 -363557.823545 0.668327;
	//gt_pose.x = 7757735.655524;
	//gt_pose.y = -363557.823545;
	//gt_pose.th = 0.668327;

	gt_pose.x -= offset.x;
	gt_pose.y -= offset.y;

	Pose2d mean = pf.mean();
	Pose2d mode = pf.mode();

	printf("Step: %d of %d ", step, n_total_steps);
	printf("GT_pose: %lf %lf %lf ", gt_pose.x, gt_pose.y, gt_pose.th);
	printf("PF_Mean: %lf %lf %lf ", mean.x, mean.y, mean.th);
	printf("PF_Mode: %lf %lf %lf ", mode.x, mode.y, mode.th);
	printf("D_GT_MEAN: %lf ", dist2d(mean.x, mean.y, gt_pose.x, gt_pose.y));
	printf("D_GT_MODE: %lf ", dist2d(mode.x, mode.y, gt_pose.x, gt_pose.y));
	printf("O_GT_MEAN: %lf ", fabs(normalize_theta(mean.th - gt_pose.th)));
	printf("O_GT_MODE: %lf ", fabs(normalize_theta(mode.th - gt_pose.th)));
	printf("\n");
	fflush(stdout);

	Mat pf_img = pf_view(pf, map, gt_pose, pf.mean(), cloud, 1);

	//Mat concat;
	//hconcat(pf_view_img, view_img, concat);
	////sprintf(img_name, "%s/step_%010d.png", path_save_maps, i);
	//char text[32];
	//sprintf(text, "DistGT: %.2lfm Vel: %.2lfm/s", dist2d(mean.x, mean.y, gt_pose.x, gt_pose.y), sample->v);
	//putText(concat, text, Point(780, 700), FONT_HERSHEY_PLAIN, 1.3, Scalar(255,255,255), 1);
	////imwrite(img_name, concat);
	Mat flipped;
	flip(pf_img, flipped, 0);

	s_viewer.show(flipped, "bla");
	s_viewer.loop();
}


void
run_particle_filter(ParticleFilter &pf, GridMap &map,
										NewCarmenDataset *dataset,
										SensorPreproc &preproc,
										int step,
										double skip_velocity_threshold,
										int correction_step,
										int steps_to_skip_map_reload,
										Pose2d offset)
{
	double dt;
	DataSample *sample;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloudViewer s_viewer;
	Pose2d pose;

	Pose2d p0 = dataset->at(0)->pose;
	p0.x -= offset.x;
	p0.y -= offset.y;

	pf.reset(p0.x, p0.y, p0.th);
	map.reload(p0.x, p0.y);

	int n = 0;
	int do_correction;
	int last_reload = 0;

	for (int i = step; i < dataset->size(); i += step)
	{
		sample = dataset->at(i);

		if (fabs(sample->v) < skip_velocity_threshold)
			continue;

		dt = sample->time - dataset->at(i - step)->time;
		pf.predict(sample->v, sample->phi, dt);

		preproc.reinitialize(sample);
		load_as_pointcloud(preproc, cloud, SensorPreproc::CAR_REFERENCE);

		do_correction = 0;

		if (correction_step <= 1)
			do_correction = 1;
		else if (n % correction_step == 0)
			do_correction = 1;

		if (do_correction)
			pf.correct(cloud, map, sample->gps);

		pose = pf.mean();

		// only reload the map if the car is moving, and after a while.
		// this prevents frequent (expensive) reloads when near to borders.
		if ((fabs(sample->v) > 0.) && (n - last_reload > steps_to_skip_map_reload))
		{
			map.reload(pose.x, pose.y);
			last_reload = i;
		}

		viewer(sample, pf, map, i, dataset->size(), cloud, offset, s_viewer);
		sample->pose = pf.mean();
		update_map(sample, &map, preproc);

		n++;
	}
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;
	add_default_slam_args(args);
	add_default_sensor_preproc_args(args);
	add_default_mapper_args(args);
	add_default_localizer_args(args);
	args.save_config_file(default_data_dir() + "/localizer_config.txt");
	args.parse(argc, argv);

	string log_path = args.get<string>("log_path");
	NewCarmenDataset* dataset = create_dataset(log_path, "graphslam_to_map");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);

	GridMap map = create_grid_map(args, 0);
	ParticleFilter pf = create_particle_filter(args);

	pf.seed(args.get<int>("seed"));
	Pose2d offset = Pose2d(args.get<double>("offset_x"),
												 args.get<double>("offset_y"), 0);

	run_particle_filter(pf, map, dataset, preproc,
											args.get<int>("step"),
											args.get<double>("v_thresh"),
											args.get<int>("correction_step"),
											args.get<int>("steps_to_skip_map_reload"),
											offset);

	printf("Done.\n");
	return 0;
}

