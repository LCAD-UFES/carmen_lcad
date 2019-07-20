
#include <boost/filesystem.hpp>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>
#include <carmen/segmap_preproc.h>
#include <carmen/util_math.h>
#include <carmen/command_line.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_constructors.h>

#include <opencv/cv.hpp>
#include <carmen/util_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>
#include <map>

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;


void
write_points_to_file(FILE *fptr, SensorPreproc &preproc)
{
	SensorPreproc::CompletePointData point;
	vector<SensorPreproc::CompletePointData> shot;

	for (int i = 0; i < preproc.size(); i++)
	{
		shot = preproc.next_points();

		for (int j = 0; j < shot.size(); j++)
		{
			point = shot[j];

			if (point.valid)
				fprintf(fptr, "%d %d %lf %lf %lf %lf %lf %d %d\n",
								point.laser_id, point.raw_intensity, point.range, point.h_angle,
								point.world.x, point.world.y, point.world.z,
								(int) (point.world.x), (int)(point.world.y / 0.2));
		}
	}
}


void
save_points_from_log(NewCarmenDataset *dataset, SensorPreproc &preproc, CommandLineArguments &args)
{
	DataSample *sample;

	FILE *fptr = safe_fopen(args.get<string>("output").c_str(), "w");

	for (int i = 0; i < dataset->size(); i += args.get<int>("step"))
	{
		sample = dataset->at(i);

		if (i % 50 == 0)
			printf("%d of %d\n", i, dataset->size());

		if (fabs(sample->v) < args.get<double>("v_thresh"))
			continue;

		preproc.reinitialize(sample);

		write_points_to_file(fptr, preproc);
	}

	fclose(fptr);
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;

	add_default_slam_args(args);
	add_default_sensor_preproc_args(args);
	args.add_positional<string>("output", "path to the output file");

	args.parse(argc, argv);

	if (args.get<int>("use_calib"))
		exit(printf("Error: please, set 'use_calib' to false.\n"));

	string log_path = args.get<string>("log_path");
	NewCarmenDataset *dataset = create_dataset(log_path, args, "graphslam");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);

	printf("*** IMPORTANTE: This step will take ~15min and it requires ~16GB of data.\n");

	save_points_from_log(dataset, preproc, args);

	return 0;
}

