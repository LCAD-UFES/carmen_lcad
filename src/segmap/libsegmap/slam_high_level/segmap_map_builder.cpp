

#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>
#include <carmen/util_time.h>
#include <carmen/util_math.h>

#include <boost/filesystem.hpp>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>
#include <carmen/segmap_particle_filter_viewer.h>
#include <carmen/segmap_map_builder.h>
#include <carmen/segmap_args.h>
#include <carmen/util_strings.h>
#include <carmen/util_io.h>


using namespace pcl;
using namespace std;
using namespace cv;


void
view_one_map(DataSample *sample, PointCloudViewer &viewer, GridMap *map, int img_width, const char *name)
{
	if (map == NULL)
		return;

	Mat map_img = map->to_image().clone();
	draw_pose(*map, map_img, sample->pose, Scalar(0, 255, 0));

	Mat vertically_inverted_img;
	flip(map_img, vertically_inverted_img, 0);

	viewer.show(vertically_inverted_img, name, img_width);
}


void
view_maps(DataSample *sample, SensorPreproc &preproc,
          PointCloudViewer &viewer, GridMap *visual_map,
          GridMap *reflectivity_map, GridMap *semantic_map,
          GridMap *occupancy_map, int img_width, int view_point_cloud,
          int view_img_with_points)
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	view_one_map(sample, viewer, visual_map, img_width, "visual_map");
	view_one_map(sample, viewer, semantic_map, img_width, "semantic_map");
	view_one_map(sample, viewer, occupancy_map, img_width, "occupancy_map");
	view_one_map(sample, viewer, reflectivity_map, img_width, "reflectivity_map");

	if (view_point_cloud)
	{
		viewer.clear();
		preproc.reinitialize(sample);
		load_as_pointcloud(preproc, cloud, SensorPreproc::WORLD_REFERENCE);
		viewer.show(cloud);
	}

	if (view_img_with_points)
	{
		Mat img = preproc._img_with_points;
		Mat semantic_img = preproc._semantic_img_with_points;

		if (img.rows)
			viewer.show(img, "img", img_width);

		if (semantic_img.rows)
			viewer.show(semantic_img, "semantic_img", img_width);
	}

	viewer.loop();
}


void
create_map(NewCarmenDataset *dataset,
					 SensorPreproc &preproc,
					 CommandLineArguments &args,
					 const std::string &map_path,
					 const std::vector<int> &samples_to_map)
{
	Timer timer;
	DataSample *sample;
	PointCloudViewer viewer(2, 0.0, 0.0, 1.0);
	vector<double> times;

	// Como evitar esse trecho feio de codigo e ao mesmo tempo evitar pagar o preco de buscar os valores dos parametros na hash?
	int view_flag = args.get<int>("view");
	int save_map = args.get<int>("save_maps");
	int img_width = args.get<int>("viewer_width");
	int tile_size = args.get<double>("tile_size");
	double resolution = args.get<double>("resolution");
	double skip_velocity_threshold = args.get<double>("v_thresh");
	int view_pointcloud = args.get<int>("view_pointcloud");
	int view_imgs = args.get<int>("view_imgs");

	string log_path = args.get<string>("log_path");
	string log_name = file_name_from_path(log_path);

	if (args.get<int>("clean_map"))
	{
		boost::filesystem::remove_all(map_path + "map_visual_" + log_name);
		boost::filesystem::remove_all(map_path + "map_semantic_" + log_name);
		boost::filesystem::remove_all(map_path + "map_occupancy_" + log_name);
		boost::filesystem::remove_all(map_path + "map_reflectivity_" + log_name);
	}


	GridMap *visual_map = NULL;
	GridMap *semantic_map = NULL;
	GridMap *occupancy_map = NULL;
	GridMap *reflectivity_map = NULL;


	if (args.get<int>("build_occupancy_map"))
		occupancy_map = new GridMap(map_path + "map_occupancy_" + log_name, tile_size, tile_size, resolution, GridMapTile::TYPE_OCCUPANCY, save_map);

	if (args.get<int>("build_visual_map"))
		visual_map = new GridMap(map_path + "map_visual_" + log_name, tile_size, tile_size, resolution, GridMapTile::TYPE_VISUAL, save_map);

	if (args.get<int>("build_semantic_map"))
		semantic_map = new GridMap(map_path + "map_semantic_" + log_name, tile_size, tile_size, resolution, GridMapTile::TYPE_SEMANTIC, save_map);

	if (args.get<int>("build_reflectivity_map"))
		reflectivity_map = new GridMap(map_path + "map_reflectivity_" + log_name, tile_size, tile_size, resolution, GridMapTile::TYPE_REFLECTIVITY, save_map);

	viewer.set_step(args.get<int>("start_paused"));

	if (args.get<int>("build_visual_map"))
		preproc.set_load_img_flag(1);
	
	if (args.get<int>("build_semantic_map"))
		preproc.set_load_semantic_img_flag(1);

	for (int i = 0; i < samples_to_map.size(); i++)
	{
		sample = dataset->at(samples_to_map[i]);

		if (fabs(sample->v) < skip_velocity_threshold)
			continue;

		timer.start();

		if (args.get<int>("build_occupancy_map"))
			occupancy_map->reload(sample->pose.x, sample->pose.y);

		if (args.get<int>("build_visual_map"))
			visual_map->reload(sample->pose.x, sample->pose.y);

		if (args.get<int>("build_semantic_map"))
			semantic_map->reload(sample->pose.x, sample->pose.y);

		if (args.get<int>("build_reflectivity_map"))
			reflectivity_map->reload(sample->pose.x, sample->pose.y);

		update_maps(sample, preproc, visual_map,
		            reflectivity_map,
								semantic_map,
								occupancy_map);

		times.push_back(timer.ellapsed());

		if (times.size() % 50 == 0)
			printf("Step %d of %ld AvgStepDuration: %lf LastStepDuration: %lf\n",
						 i, samples_to_map.size(), mean(times), times[times.size() - 1]);

		if (view_flag && i % 1 == 0)
		{
			view_maps(sample, preproc, viewer, visual_map,
			          reflectivity_map, 
			          semantic_map,
			          occupancy_map, img_width, view_pointcloud,
			          view_imgs);
		}
	}

	if (args.get<int>("build_occupancy_map"))
	{
		occupancy_map->save();
		delete(occupancy_map);
	}

	if (args.get<int>("build_visual_map"))
	{
		visual_map->save();
		delete(visual_map);
	}

	if (args.get<int>("build_semantic_map"))
	{
		semantic_map->save();
		delete(semantic_map);
	}

	if (args.get<int>("build_reflectivity_map"))
	{
		reflectivity_map->save();
		delete(reflectivity_map);
	}

	destroyAllWindows();
}


void
create_map(NewCarmenDataset *dataset,
					 SensorPreproc &preproc,
					 CommandLineArguments &args)
{
	vector<int> all_indices;
	int step = args.get<int>("step");

	for (int i = 0; i < dataset->size(); i += step)
		all_indices.push_back(i);

	create_map(dataset, preproc, args, "/dados/maps2/", all_indices);
}
