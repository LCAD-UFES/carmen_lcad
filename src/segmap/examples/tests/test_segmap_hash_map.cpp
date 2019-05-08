
#include <cstdio>
#include <string>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_preproc.h>
#include <carmen/command_line.h>
#include <carmen/segmap_constructors.h>
#include <carmen/segmap_hash_map.h>
#include <carmen/segmap_sensor_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>

#include <carmen/segmap_grayscale_cell.h>
#include <carmen/segmap_color_cell.h>
#include <carmen/segmap_height_cell.h>

using namespace std;
using namespace pcl;
using namespace cv;


template<class CellType> Mat
draw_map(HashGridMap<CellType> &map)
{
	typename std::map<int, std::map<int, CellType>>::iterator col_it;
	typename std::map<int, CellType>::iterator cell_it;
	Mat map_img(map._height, map._width, CV_8UC3, Scalar(128, 0, 0));

	for (col_it = map._cells.begin(); col_it != map._cells.end(); col_it++)
	{
		int y_coord = col_it->first;

		for (cell_it = col_it->second.begin(); cell_it != col_it->second.end(); cell_it++)
		{
			int x_coord = cell_it->first;
			cv::Scalar color = cell_it->second.get_color();

			map_img.data[3 * (y_coord * map_img.cols + x_coord)] = color[0];
			map_img.data[3 * (y_coord * map_img.cols + x_coord) + 1] = color[1];
			map_img.data[3 * (y_coord * map_img.cols + x_coord) + 2] = color[2];
		}
	}

	Mat flipped_map;
	flip(map_img, flipped_map, 0);
	return flipped_map;
}



int
main(int argc, char **argv)
{
	CommandLineArguments args;

	args.add_positional<string>("log", "Path to a log", 1);
	add_default_sensor_preproc_args(args);
	args.parse(argc, argv);

	string log_path = args.get<string>("log");
	NewCarmenDataset *dataset = create_dataset(log_path, args.get<double>("camera_latency"), "graphslam");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloudViewer viewer;

	HashGridMap<HeightCell> map(200, 200, 0.2,
														 dataset->at(0)->pose.y - 100.0,
														 dataset->at(0)->pose.x - 100.0);

	viewer.set_step(1);

	map.load("/tmp/test_map.bin");

	for (int i = 1; i < dataset->size(); i += 1)
	{
		DataSample* data_package = dataset->at(i);

		// ignore packages when the car is stopped.
		if (fabs(data_package->v) < 1.0)
			continue;

		printf("Time: %lf\n", data_package->velodyne_time);

		preproc.reinitialize(data_package);
		load_as_pointcloud(preproc, cloud, SensorPreproc::WORLD_REFERENCE);
		map.add(cloud);

		Mat map_img = draw_map(map);
		//img = preproc.get_sample_img();
		viewer.show(map_img, "map", 640);
		//viewer.show(cloud);
		viewer.loop();

		if (i % 50 == 0)
			map.save("/tmp/test_map.bin");
	}

	printf("Done.\n");
	return 0;
}

