
#include <string>
#include <opencv/cv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <carmen/util_time.h>
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
			 PointCloud<PointXYZRGB>::Ptr cloud,
			 PointCloudViewer &s_viewer, double duration, int view_flag,
			 CarmenImageLoader &iloader)
{
	Pose2d gt = sample->pose;
	Pose2d mean = pf.mean();
	Pose2d mode = pf.mode();
	Pose2d std = pf.std();

	printf("%d of %d ", step, n_total_steps);
	printf("Mean: %lf %lf %lf ", mean.x, mean.y, mean.th);
	printf("Std: %lf %lf %lf ", std.x, std.y, std.th);
	printf("GT: %lf %lf %lf ", gt.x, gt.y, gt.th);
	printf("Mode: %lf %lf %lf ", mode.x, mode.y, mode.th);
	printf("Duration: %lf ", duration);
	printf("\n");
	fflush(stdout);

	if (view_flag)
	{
		Mat view_img;
		Mat pf_view_img;

		Mat pf_img = pf_view(pf, map, &gt, mean, cloud, 1);

		//Mat concat;
		//hconcat(pf_view_img, view_img, concat);
		////sprintf(img_name, "%s/step_%010d.png", path_save_maps, i);
		//char text[32];
		//sprintf(text, "DistGT: %.2lfm Vel: %.2lfm/s", dist2d(mean.x, mean.y, gt_pose.x, gt_pose.y), sample->v);
		//putText(concat, text, Point(780, 700), FONT_HERSHEY_PLAIN, 1.3, Scalar(255,255,255), 1);
		////imwrite(img_name, concat);
		Mat flipped;
		flip(pf_img, flipped, 0);

		Mat img = iloader.load(sample);
		s_viewer.show(img, "img", 400);
		s_viewer.show(flipped, "pf_viewer");
		s_viewer.loop();
	}
}


void
run_particle_filter(ParticleFilter &pf, GridMap &map,
										NewCarmenDataset *dataset,
										SensorPreproc &preproc,
										int step,
										double skip_velocity_threshold,
										int correction_step,
										int steps_to_skip_map_reload,
										int view_flag)
{
	double dt;
	DataSample *sample;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloudViewer s_viewer;
	CarmenImageLoader iloader;
	TimeCounter timer;

	Pose2d p0 = dataset->at(0)->gps;
	pf.reset(p0.x, p0.y, p0.th);
	map.reload(p0.x, p0.y);

	int n = 0;
	int do_correction;
	int last_reload = 0;

	s_viewer.set_step(0);

	for (int i = step; i < dataset->size(); i += step)
	{
		sample = dataset->at(i);

		if (fabs(sample->v) < skip_velocity_threshold)
			continue;

		timer.start();

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
			//pf.correct(sample, &map, preproc);

		//printf("* Pose: %lf %lf %lf v: %lf n: %d last_reload: %d steps: %d\n", pose.x, pose.y, pose.th,
		//fabs(sample->v), n, last_reload, steps_to_skip_map_reload);

		// only reload the map if the car is moving, and after a while.
		// this prevents frequent (expensive) reloads when near to borders.
		if ((fabs(sample->v) > 0.) && (n - last_reload > steps_to_skip_map_reload))
		{
			Pose2d mean = pf.mean();

			map.reload(mean.x, mean.y);
			last_reload = n;
		}

		//sample->pose = pf.mean();
		//update_map(sample, &map, preproc);
		//preproc.reinitialize(sample);
		//load_as_pointcloud(preproc, cloud, SensorPreproc::CAR_REFERENCE);
		viewer(sample, pf, map, i, dataset->size(), cloud, s_viewer, timer.ellapsed(), view_flag, iloader);

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
	args.add<int>("view,v", "Flag to set visualization on or off", 1);
	args.save_config_file(default_data_dir() + "/localizer_config.txt");
	args.parse(argc, argv);

	string log_path = args.get<string>("log_path");
	NewCarmenDataset* dataset = create_dataset(log_path, "graphslam_to_map");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);

	GridMap map = create_grid_map(args, 0);
	ParticleFilter pf = create_particle_filter(args);

	pf.seed(args.get<int>("seed"));

	run_particle_filter(pf, map, dataset, preproc,
											args.get<int>("step"),
											args.get<double>("v_thresh"),
											args.get<int>("correction_step"),
											args.get<int>("steps_to_skip_map_reload"),
											args.get<int>("view"));

	printf("Done.\n");
	return 0;
}

