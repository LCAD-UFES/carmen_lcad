
#include <iostream>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include "g2o/types/slam2d/se2.h"

#include <carmen/util_io.h>
#include <carmen/util_math.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_loop_closures.h>
#include <carmen/segmap_particle_filter_viewer.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_constructors.h>
#include <carmen/segmap_map_builder.h>
#include <carmen/command_line.h>
#include "gicp.h"

#include <vector>
#include <algorithm>
#include <map>
#include <set>

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace Eigen;
using namespace std;
using namespace pcl;
using namespace cv;


void
show_flipped_img_in_viewer(PointCloudViewer &viewer, Mat &img)
{
	Mat flipped;
	flip(img, flipped, 0);
	viewer.show(flipped, "map", 640);
	viewer.loop();
}


void
run_viewer_if_necessary(Pose2d *pose,
												GridMap &map,
												ParticleFilter &pf,
												DataSample *sample,
												SensorPreproc &preproc,
												PointCloudViewer &viewer,
												int pf_was_updated,
												int show_particles,
												int view)
{
	if (view)
	{
		Mat img;

		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
		preproc.reinitialize(sample);
		load_as_pointcloud(preproc, cloud, SensorPreproc::CAR_REFERENCE);
		

		if (pf_was_updated)
			img = pf_view(pf, map, pose, pf.mean(), cloud, show_particles);
		else
		{
			img = map.to_image().clone();
			draw_pose(map, img, *pose, Scalar(0, 255, 0));
			transformPointCloud(*cloud, *cloud, Pose2d::to_matrix(*pose));
			draw_pointcloud(img, cloud, map, 1, Scalar(0, 255, 0));
		}

		//viewer.clear();
		//PointCloud<PointXYZRGB>::Ptr transformed(new PointCloud<PointXYZRGB>);
		//transformPointCloud(*cloud, *transformed, Pose2d::to_matrix(*pose));
		//viewer.show(cloud);
		//viewer.set_camera_pose(pose->x, pose->y);
		show_flipped_img_in_viewer(viewer, img);
	}
}


void
run_viewer_if_necessary(Pose2d *pose,
												GridMap &map,
												ParticleFilter &pf,
												PointCloud<PointXYZRGB>::Ptr cloud,
												PointCloudViewer &viewer,
												int pf_was_updated,
												int show_particles,
												int view)
{
	if (view)
	{
		Mat img;

		if (pf_was_updated)
			img = pf_view(pf, map, pose, pf.mean(), cloud, show_particles);
		else
		{
			img = map.to_image().clone();
			draw_pose(map, img, *pose, Scalar(0, 255, 0));
			transformPointCloud(*cloud, *cloud, Pose2d::to_matrix(*pose));
			draw_pointcloud(img, cloud, map, 1, Scalar(0, 255, 0));
		}

		//viewer.clear();
		//PointCloud<PointXYZRGB>::Ptr transformed(new PointCloud<PointXYZRGB>);
		//transformPointCloud(*cloud, *transformed, Pose2d::to_matrix(*pose));
		//viewer.show(cloud);
		//viewer.set_camera_pose(pose->x, pose->y);
		show_flipped_img_in_viewer(viewer, img);
	}
}


Matrix<double, 4, 4>
compute_source2target_transform(Pose2d target_pose, Pose2d source_pose)
{
	source_pose.x -= target_pose.x;
	source_pose.y -= target_pose.y;

	Matrix<double, 4, 4> world2target = pose3d_to_matrix(0., 0., target_pose.th).inverse();
	Matrix<double, 4, 4> source2world = Pose2d::to_matrix(source_pose);
	Matrix<double, 4, 4> source2target = world2target * source2world;

	return source2target;
}


void
find_dataset_indices_for_accumulating_data(NewCarmenDataset &target_dataset,
																					 int target_id,
																					 double dist_accumulate_target_cloud,
																					 int *start, int *end)
{
	int i;
	double d;

	// walk forward and backward in the dataset to find clouds around the target one.
	d = 0;
	for (i = target_id - 1; i >= 0 && d < dist_accumulate_target_cloud; i--)
		//d += dist2d(target_dataset[i]->pose.x, target_dataset[i]->pose.y, target_dataset[i+1]->pose.x, target_dataset[i+1]->pose.y);
		d = dist2d(target_dataset[i]->pose.x, target_dataset[i]->pose.y, target_dataset[target_id]->pose.x, target_dataset[target_id]->pose.y);

	(*start) = (i >= 0) ? i : 0;

	d = 0;
	for (i = target_id + 1; i < target_dataset.size() && d < dist_accumulate_target_cloud; i++)
		//d += dist2d(target_dataset[i]->pose.x, target_dataset[i]->pose.y, target_dataset[i-1]->pose.x, target_dataset[i-1]->pose.y);
		d = dist2d(target_dataset[i]->pose.x, target_dataset[i]->pose.y, target_dataset[target_id]->pose.x, target_dataset[target_id]->pose.y);

	(*end) = (i < target_dataset.size()) ? i : target_dataset.size();
}


void
create_target_accumulating_clouds(NewCarmenDataset &target_dataset,
                                  SensorPreproc &target_preproc,
                                  int target_id,
                                  double dist_accumulate_target_cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target)
{
	int i, st, end;
	Matrix<double, 4, 4> transform_to_target;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	target->clear();

	find_dataset_indices_for_accumulating_data(target_dataset, target_id,
																						 dist_accumulate_target_cloud,
																						 &st, &end);

	// load the clouds
	for (i = st; i < end; i++)
	{
		target_preproc.reinitialize(target_dataset[i]);
		load_as_pointcloud(target_preproc, cloud, SensorPreproc::CAR_REFERENCE);
		transform_to_target = compute_source2target_transform(target_dataset[target_id]->pose,
		                                                      target_dataset[i]->pose);
		pcl::transformPointCloud(*cloud, *cloud, transform_to_target);
		(*target) += (*cloud);
	}
}


void
search_for_loop_closure_using_pose_dist(NewCarmenDataset &dataset,
                                        Pose2d reference_pose,
                                        double reference_pose_time,
                                        int from,
                                        int to,
                                        double max_dist_threshold,
                                        double min_time_threshold,
                                        int *nn_id,
                                        double min_velocity)
{
	*nn_id = -1;
	double min_dist = DBL_MAX;

	for (int j = from; j < to; j++)
	{
		if (fabs(dataset[j]->v) < min_velocity)
			continue;

		double dx = reference_pose.x - dataset[j]->pose.x;
		double dy = reference_pose.y - dataset[j]->pose.y;
		double dt = fabs(reference_pose_time - dataset[j]->time);

		double dist = sqrt(pow(dx, 2) + pow(dy, 2));

		if ((dist < min_dist) && (dist < max_dist_threshold) && (dt > min_time_threshold))
		{
			min_dist = dist;
			(*nn_id) = j;
		}
	}
}


void
run_icp_step(NewCarmenDataset &target_dataset,
             NewCarmenDataset &source_dataset,
             int target_id,
             int source_id,
             Matrix<double, 4, 4> *relative_transform,
             int *convergence_flag,
             SensorPreproc &target_preproc,
             SensorPreproc &source_preproc,
             double voxel_grid_size,
             double dist_accumulate_target_cloud,
             bool view)
{
	Matrix<double, 4, 4> source2target;
	Matrix<double, 4, 4> correction;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>);

	create_target_accumulating_clouds(target_dataset, target_preproc,
	                                  target_id, dist_accumulate_target_cloud,
	                                  target);

	source_preproc.reinitialize(source_dataset[source_id]);
	load_as_pointcloud(source_preproc, source, SensorPreproc::CAR_REFERENCE);

	source2target = compute_source2target_transform(target_dataset[target_id]->pose,
	                                                source_dataset[source_id]->pose);

	pcl::transformPointCloud(*source, *source, source2target);
	run_gicp(source, target, &correction, convergence_flag, aligned, voxel_grid_size);

	(*relative_transform) = correction * source2target;

	if (view)
	{
		static PointCloudViewer *viewer = NULL;

		if (viewer == NULL)
			viewer = new PointCloudViewer(3);

		std::cout << "source2target" << std::endl << source2target << std::endl;
		std::cout << "correction" << std::endl << correction << std::endl;
		std::cout << "(*relative_transform)" << std::endl << (*relative_transform) << std::endl;

		Pose2d target_pose = target_dataset[target_id]->pose;
		Matrix<double, 4, 4> corrected_pose = Pose2d::to_matrix(target_pose) * (*relative_transform);
		Pose2d pose = Pose2d::from_matrix(corrected_pose);

		printf("target_pose %lf %lf %lf\n", target_dataset[target_id]->pose.x, target_dataset[target_id]->pose.y, target_dataset[target_id]->pose.th);
		printf("source_pose %lf %lf %lf\n", source_dataset[source_id]->pose.x, source_dataset[source_id]->pose.y, source_dataset[source_id]->pose.th);
		printf("target_id %d source_id %d x: %lf y: %lf z: %lf\n", target_id, source_id, pose.x, pose.y, pose.th);

		viewer->clear();
		viewer->show(target); //, 0, 1, 0);

		/*
		for (int i = 0; i < aligned->size(); i++)
		{
			aligned->at(i).z += 5.0;
			//aligned->at(i).b = 255;
		}
		*/
		//transformPointCloud(*source, *source, correction);

		viewer->show(source, 1, 0, 0);
		viewer->show(aligned, 0, 0, 1);

		viewer->loop();
	}
}


void
create_map_accumulating_points(NewCarmenDataset &target_dataset,
										            int target_id,
										            SensorPreproc &target_preproc,
																GridMap &map,
										            double dist_accumulate_target_cloud)
{
	int st, end;
	Matrix<double, 4, 4> transform_to_target;

	map.reload(0, 0);

	find_dataset_indices_for_accumulating_data(target_dataset, target_id,
																						 dist_accumulate_target_cloud,
																						 &st, &end);

	for (int i = st; i < end; i++)
	{
		target_preproc.reinitialize(target_dataset[i]);
		transform_to_target = compute_source2target_transform(target_dataset[target_id]->pose,
		                                                      target_dataset[i]->pose);

		for (int j = 0; j < target_preproc.size(); j++)
		{
			vector<PointXYZRGB> points = target_preproc.next_points_in_car();

			for (int j = 0; j < points.size(); j++)
			{
				PointXYZRGB p = points[j];
				p = transform_point(transform_to_target, p);
				map.add_point(p);
			}
		}
	}
}


void
run_pf_step(NewCarmenDataset &target_dataset,
            NewCarmenDataset &source_dataset,
            int target_id,
            int source_id,
            Matrix<double, 4, 4> *relative_transform,
            int *convergence_flag,
            SensorPreproc &target_preproc,
            SensorPreproc &source_preproc,
						ParticleFilter &pf,
						GridMap &map,
						PointCloudViewer &viewer,
            double dist_accumulate_target_cloud,
						int n_corrections_when_reinit,
            bool view)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	create_map_accumulating_points(target_dataset, target_id, target_preproc,
																 map, dist_accumulate_target_cloud);

	source_preproc.reinitialize(source_dataset[source_id]);
	load_as_pointcloud(source_preproc, cloud, SensorPreproc::CAR_REFERENCE);

	Matrix<double, 4, 4> smat = compute_source2target_transform(target_dataset[target_id]->pose,
																															source_dataset[source_id]->pose);

	Pose2d source_as_pose = Pose2d::from_matrix(smat);
	pf.seed(source_id);
	pf.reset(source_as_pose.x, source_as_pose.y, source_as_pose.th);

	//printf("Source: %lf %lf %lf\n", source_as_pose.x, source_as_pose.y, source_as_pose.th);
	//viewer.set_step(1);
	run_viewer_if_necessary(&source_as_pose, map, pf, cloud, viewer, 1, 0, view);

	for (int i = 0; i < n_corrections_when_reinit; i++)
	{
		pf.predict(0, 0, 0); // just to add a little noise
		run_viewer_if_necessary(&source_as_pose, map, pf, cloud, viewer, 1, 1, view);
		pf.correct(cloud, map, source_dataset[source_id]->gps);
		run_viewer_if_necessary(&source_as_pose, map, pf, cloud, viewer, 1, 1, view);
	}

	Pose2d estimate = pf.mean();
	// for compatibility issues, the relative transform should store a "correction" term
	// in relation to the source2target transform.
	// (*relative_transform) = Pose2d::to_matrix(estimate) * smat.inverse();
	(*relative_transform) = Pose2d::to_matrix(estimate);
	(*convergence_flag) = 1;

	//cout << (*relative_transform) << endl;
	//printf("Estimate: %lf %lf %lf\n\n", estimate.x, estimate.y, estimate.th);
	run_viewer_if_necessary(&source_as_pose, map, pf, cloud, viewer, 1, 1, view);
}


void
save_output(std::string path,
            NewCarmenDataset &reference_dataset,
            std::vector<std::pair<int, int>> &indices,
            std::vector<Eigen::Matrix<double, 4, 4>> &relative_transform_vector,
            std::vector<int> &convergence_vector,
            int project_to_world)
{
	Eigen::Matrix<double, 4, 4> source2target;
	Eigen::Matrix<double, 4, 4> corrected_pose;

	FILE *f = safe_fopen(path.c_str(), "w");

	for (int i = 0; i < indices.size(); i++)
	{
		corrected_pose = relative_transform_vector[i];

		if (project_to_world)
		{
			Pose2d target_pose = reference_dataset[indices[i].first]->pose;
			corrected_pose = Pose2d::to_matrix(target_pose) * corrected_pose;
		}

		Pose2d pose = Pose2d::from_matrix(corrected_pose);

		fprintf(f, "%d %d %d %lf %lf %lf\n", indices[i].first, indices[i].second,
		        convergence_vector[i], pose.x, pose.y, pose.th);
	}

	fclose(f);
}


void
save_report_file(std::string path, std::vector<std::pair<int, int>> &loop_closure_indices,
                 std::vector<Eigen::Matrix<double, 4, 4>> &relative_transform_vector,
                 std::vector<int> &convergence_vector)
{
	FILE *report_file = safe_fopen(path.c_str(), "w");

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

	fclose(report_file);
}


void
estimate_displacements_with_particle_filter(NewCarmenDataset &target_dataset,
																						NewCarmenDataset &dataset_to_adjust,
																						string target_dataset_path,
																						string dataset_to_adjust_path,
                                            vector<pair<int, int>> &loop_closure_indices,
                                            vector<Matrix<double, 4, 4>> *relative_transform_vector,
                                            vector<int> *convergence_vector,
																						int n_corrections_when_reinit,
                                            CommandLineArguments &args)
{
	printf("Running displacement estimation using particle filters.\n");

	int i;
	int view = args.get<int>("view");
	int n_processed_clouds = 0;
	int n = loop_closure_indices.size();

#ifdef _OPENMP
	if (view)
		omp_set_num_threads(1);
#endif

	string adj_name = file_name_from_path(dataset_to_adjust_path);
	PointCloudViewer viewer;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 5) default(none) private(i) \
		shared(target_dataset, dataset_to_adjust, convergence_vector, relative_transform_vector, \
					 loop_closure_indices, n_processed_clouds, n, view, args, \
					 n_corrections_when_reinit, viewer, target_dataset_path, dataset_to_adjust_path, adj_name)
#endif
	for (i = 0; i < n; i++)
	{
		SensorPreproc target_preproc = create_sensor_preproc(args, &target_dataset, target_dataset_path);
		SensorPreproc adj_preproc = create_sensor_preproc(args, &dataset_to_adjust, dataset_to_adjust_path);

		GridMap map(string("/tmp/map_") + adj_name + "_" + std::to_string(i),
								args.get<double>("tile_size"),
								args.get<double>("tile_size"),
								args.get<double>("resolution"),
								GridMapTile::TYPE_REFLECTIVITY, 1);

		ParticleFilter pf(args.get<int>("n_particles"),
											args.get<double>("gps_xy_std"),
											args.get<double>("gps_xy_std"),
											degrees_to_radians(args.get<double>("gps_h_std")),
											args.get<double>("v_std"),
											degrees_to_radians(args.get<double>("phi_std")),
											args.get<double>("odom_xy_std"),
											args.get<double>("odom_xy_std"),
											degrees_to_radians(args.get<double>("odom_h_std")),
											args.get<double>("color_red_std"),
											args.get<double>("color_green_std"),
											args.get<double>("color_blue_std"),
											args.get<double>("reflectivity_std")
											);

		pf.set_use_map_weight(1);

		run_pf_step(target_dataset,
		            dataset_to_adjust,
		            loop_closure_indices[i].first,
		            loop_closure_indices[i].second,
		            &(relative_transform_vector->at(i)),
		            &(convergence_vector->at(i)),
		            target_preproc,
		            adj_preproc,
								pf, map,
								viewer,
								args.get<double>("dist_to_accumulate"),
								n_corrections_when_reinit,
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


void
estimate_loop_closures_with_particle_filter_in_map(NewCarmenDataset &dataset,
																									string dataset_path,
																									vector<pair<int, int>> &loop_closure_indices,
																									vector<Matrix<double, 4, 4>> *relative_transform_vector,
																									vector<int> *convergence_vector,
																									int n_corrections_when_reinit,
																									CommandLineArguments &args)
{
	int view = args.get<int>("view");

	string name = file_name_from_path(dataset_path);
	string map_path = string("/dados/maps2/loop_map_") + name;

	if (boost::filesystem::exists(map_path))
		boost::filesystem::remove_all(map_path);

	GridMap map(map_path,
							args.get<double>("tile_size"),
							args.get<double>("tile_size"),
							args.get<double>("resolution"),
							GridMapTile::TYPE_REFLECTIVITY, 1);

	GridMap map_for_viewing(map_path,
													args.get<double>("tile_size"),
													args.get<double>("tile_size"),
													args.get<double>("resolution"),
													GridMapTile::TYPE_REFLECTIVITY, 1);

	ParticleFilter pf(args.get<int>("n_particles"),
										args.get<double>("gps_xy_std"),
										args.get<double>("gps_xy_std"),
										degrees_to_radians(args.get<double>("gps_h_std")),
										args.get<double>("v_std"),
										degrees_to_radians(args.get<double>("phi_std")),
										args.get<double>("odom_xy_std"),
										args.get<double>("odom_xy_std"),
										degrees_to_radians(args.get<double>("odom_h_std")),
										args.get<double>("color_red_std"),
										args.get<double>("color_green_std"),
										args.get<double>("color_blue_std"),
										args.get<double>("reflectivity_std")
										);

	pf.set_use_map_weight(1);

	cv::Mat pf_img;
	PointCloudViewer viewer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr aux_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	SensorPreproc preproc = create_sensor_preproc(args, &dataset, dataset_path);
	loop_closure_indices.clear();
	relative_transform_vector->clear();
	convergence_vector->clear();

	map.reload(dataset[0]->pose.x,
		         dataset[0]->pose.y);

	int nn_id;
	int is_init = 0;
	//int prev_id = 0;
	DataSample *sample;
	double dt;
	

	viewer.set_step(0);

	for (int i = 0; i < dataset.size(); i++)
	{
		sample = dataset[i];

		if (fabs(sample->v) < args.get<double>("v_thresh"))
			continue;

		search_for_loop_closure_using_pose_dist(dataset,
																						sample->pose,
																						sample->time,
																						0, i,
																						args.get<double>("loop_dist"),
																						args.get<double>("time_dist"),
																						&nn_id,
																						args.get<double>("v_thresh"));

		map.reload(sample->pose.x, sample->pose.y);
		map_for_viewing.reload(sample->pose.x, sample->pose.y);

		preproc.reinitialize(sample);
		load_as_pointcloud(preproc, cloud, SensorPreproc::CAR_REFERENCE);

		if (nn_id < 0)
		{
			// update_map(sample, &map, preproc);
			//update_map(sample, &map_for_viewing, preproc);
			update_maps(sample, preproc, NULL, &map, NULL, NULL);
			update_maps(sample, preproc, NULL, &map_for_viewing, NULL, NULL);

			if (view)
				run_viewer_if_necessary(&sample->pose, map_for_viewing, pf, cloud, viewer, 0, 0, view);
		}
		else
		{
			Pose2d pf_pose = pf.mean();
			double d = dist2d(pf_pose.x, pf_pose.y, sample->pose.x, sample->pose.y);

			// TODO: turn the value into a parameter
			if (!is_init || d > 10.0)
			{
				// initialize particle filter
				pf.reset(sample->pose.x,
				         sample->pose.y,
				         sample->pose.th);

				for (int k = 0; k < n_corrections_when_reinit; k++)
				{
					// just to add a little gaussian noise.
					pf.predict(0, 0, 0);

					if (view)
						run_viewer_if_necessary(&sample->pose, map_for_viewing, pf, cloud, viewer, 1, 1, view);

					pf.correct(cloud, map, sample->gps);

					if (view)
						run_viewer_if_necessary(&sample->pose, map_for_viewing, pf, cloud, viewer, 1, 1, view);
				}

				is_init = 1;
			}
			else
			{
				dt = sample->time - dataset.at(i - 1)->time;
				pf.predict(sample->v, sample->phi, dt);

				if (view)
					run_viewer_if_necessary(&sample->pose, map_for_viewing, pf, cloud, viewer, 1, 1, view);

				pf.correct(cloud, map, sample->gps);

				// uncomment for updating the map with the localization estimate.
				preproc.reinitialize(sample);
				load_as_pointcloud(preproc, aux_cloud, SensorPreproc::CAR_REFERENCE);
				transformPointCloud(*aux_cloud, *aux_cloud, Pose2d::to_matrix(pf.mean()));
				for (int j = 0; j < aux_cloud->size(); j++)
					map_for_viewing.add_point(aux_cloud->at(j));

				if (view)
					run_viewer_if_necessary(&sample->pose, map_for_viewing, pf, cloud, viewer, 1, 1, view);
			}

			Pose2d mean = pf.mean();
			loop_closure_indices.push_back(pair<int, int>(nn_id, i));

			// for compatibility issues, we have to specify the pose in relation to a sample in the target dataset.
			Matrix<double, 4, 4>  world2nn = Pose2d::to_matrix(dataset[nn_id]->pose).inverse();
			Matrix<double, 4, 4>  pose_in_nn = world2nn * Pose2d::to_matrix(mean);
			relative_transform_vector->push_back(pose_in_nn);
			convergence_vector->push_back(1);

			//prev_id = i;
		}
	}
}


void
expand_area_around_point(NewCarmenDataset &dataset, std::set<int> *poses_for_mapping, int idx, std::map<int, int> *loop_closures)
{
	// todo: turn this value into a command line argument
	const double SIZE_EXPANSION = 20.0;  // meters

	for (int i = 0; i < dataset.size(); i++)
	{
		double d = dist2d(dataset[idx]->pose.x, dataset[idx]->pose.y, dataset[i]->pose.x, dataset[i]->pose.y);

		if (d < SIZE_EXPANSION && loop_closures->find(i) == loop_closures->end())
			poses_for_mapping->insert(i);
	}

	/*
	// expand to the left
	double d = 0.0;

	for (int i = (idx - 1); i >= 0 && d < SIZE_EXPANSION; i--)
	{
		poses_for_mapping->insert(i);
		d += dist2d(dataset[i]->pose.x, dataset[i]->pose.y, dataset[i + 1]->pose.x, dataset[i + 1]->pose.y);
	}

	// expand to the right
	d = 0.0;

	for (int i = (idx + 1); i < dataset.size() && d < SIZE_EXPANSION; i++)
	{
		poses_for_mapping->insert(i);
		d += dist2d(dataset[i]->pose.x, dataset[i]->pose.y, dataset[i - 1]->pose.x, dataset[i - 1]->pose.y);
	}
	*/
}


void
grow_mapped_area(std::set<int> *poses_for_mapping, NewCarmenDataset &dataset, std::map<int, int> *loop_closures)
{
	// the set stores the values sorted. Because of that, the vector created
	// below is already sorted.
	vector<int> initial_set(poses_for_mapping->begin(), poses_for_mapping->end());

	for (int i = 0; i < initial_set.size(); i++)
	{
		int point_should_be_expanded = 0;
		int sample_idx = initial_set[i];
		int previous_idx = initial_set[i - 1];
		int next_idx = initial_set[i + 1];

		if ((i == 0) || (i == initial_set.size() - 1))
			point_should_be_expanded = 1;
		else if ((sample_idx - 1 != previous_idx) || (sample_idx + 1 != next_idx))
			point_should_be_expanded = 1;

		if (point_should_be_expanded)
			expand_area_around_point(dataset, poses_for_mapping, sample_idx, loop_closures);
	}
}


void
detect_loop_closures(NewCarmenDataset &dataset, CommandLineArguments &args,
                     std::map<int, int> *loop_closures,
                     std::set<int> *poses_for_mapping)
{
	double d, dt, loop_dist, time_dist, min_v;
	DataSample *sample_i, *sample_j;

	loop_dist = args.get<double>("loop_dist");
	time_dist = args.get<double>("time_dist");
	min_v = args.get<double>("v_thresh");

	for (int i = 0; i < dataset.size(); i++)
	{
		sample_i = dataset[i];

		if (fabs(sample_i->v) < min_v || sample_i->v < 0.0)
			continue;

		int nn = -1;
		double nn_d = DBL_MAX;

		for (int j = 0; j < i; j++)
		{
			sample_j = dataset[j];

			if (fabs(sample_j->v) < min_v || sample_j->v < 0.0)
				continue;

      d = dist2d(sample_i->pose.x, sample_i->pose.y, sample_j->pose.x, sample_j->pose.y);
      dt = fabs(sample_i->time - sample_j->time);

      if ((d < loop_dist) && (dt > time_dist) && (d < nn_d))
      {
      	nn_d = d;
				nn = j;
      }
		}

		// todo: try to use all loop closures instead of using only the nearest.
		if (nn != -1)
		{
			// if the pose is not a loop closure, add it to the set of poses
			// to be used for mapping.
			if (loop_closures->find(nn) == loop_closures->end())
				poses_for_mapping->insert(nn);

			loop_closures->insert(pair<int, int>(i, nn));
		}
	}

	grow_mapped_area(poses_for_mapping, dataset, loop_closures);
}


void
do_prediction_and_correction(ParticleFilter &pf, DataSample *sample, double v, double phi, double dt,
                             SensorPreproc &preproc, GridMap &map,
                             PointCloudViewer &viewer, int view)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	preproc.reinitialize(sample);
	load_as_pointcloud(preproc, cloud, SensorPreproc::CAR_REFERENCE); 

	pf.predict(v, phi, dt);

	if (view)
		run_viewer_if_necessary(&sample->pose, map, pf, cloud, viewer, 1, 1, view);

	pf.correct(sample, map, preproc);

	if (view)
		run_viewer_if_necessary(&sample->pose, map, pf, cloud, viewer, 1, 1, view);
}


void
reinitialize_particle_filter(ParticleFilter &pf, GridMap &map, SensorPreproc &preproc, PointCloudViewer &viewer, int view,
                             DataSample *sample, int n_corrections_when_reinit)
{
	// initialize particle filter
	pf.reset(sample->pose.x, sample->pose.y, sample->pose.th);

	for (int k = 0; k < n_corrections_when_reinit; k++)
		do_prediction_and_correction(pf, sample, 0, 0, 0, preproc, map, viewer, view);
}


Pose2d
update_particle_filter(ParticleFilter &pf, GridMap &map, SensorPreproc &preproc, PointCloudViewer &viewer, int view,
                       NewCarmenDataset &dataset, DataSample *sample, std::map<int, int>::iterator it, int is_init,
                       int n_corrections_when_reinit)
{
	// TODO: turn the value into a parameter
	const double DIST_FOR_JUMP_DETECTION = 10.0;

	Pose2d mean;

	map.reload(sample->pose.x, sample->pose.y);

	mean = pf.mean();
	double d = dist2d(mean.x, mean.y, sample->pose.x, sample->pose.y);

	if (is_init || d > DIST_FOR_JUMP_DETECTION)
		reinitialize_particle_filter(pf, map, preproc, viewer, view, sample, n_corrections_when_reinit);
	else
	{
		if (it->first > 0)
		{
			double dt = fabs(sample->time - dataset.at(it->first - 1)->time);
			do_prediction_and_correction(pf, sample, sample->v, sample->phi, dt, preproc, map, viewer, view);
		}
	}

	return pf.mean();
}


void
run_particle_filter(string map_path,
                    GridMapTile::MapType map_type,
                    string imode,
                    vector<pair<int, int>> &loop_closure_indices,
                		vector<Matrix<double, 4, 4>> *relative_transform_vector, vector<int> *convergence_vector,
                		int n_corrections_when_reinit,
                		CommandLineArguments &args,
                		NewCarmenDataset &tgt_dataset,
                		NewCarmenDataset &adj_dataset,
                		std::map<int, int> &loop_closures,
                		string &adj_dataset_path)
{
	SensorPreproc preproc = create_sensor_preproc(args, &adj_dataset, adj_dataset_path, imode);

	GridMap map(map_path, args.get<double>("tile_size"), args.get<double>("tile_size"), args.get<double>("resolution"), map_type, 0);

	ParticleFilter pf(args.get<int>("n_particles"),
	                  args.get<double>("gps_xy_std"), args.get<double>("gps_xy_std"), degrees_to_radians(args.get<double>("gps_h_std")),
	                  args.get<double>("v_std"), degrees_to_radians(args.get<double>("phi_std")),
	                  args.get<double>("odom_xy_std"), args.get<double>("odom_xy_std"), degrees_to_radians(args.get<double>("odom_h_std")),
	                  args.get<double>("color_red_std"), args.get<double>("color_green_std"), args.get<double>("color_blue_std"),
	                  args.get<double>("reflectivity_std")
	);

	pf.set_use_map_weight(1);

	Pose2d mean;
	DataSample *sample;
	PointCloudViewer viewer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	viewer.set_step(args.get<int>("start_paused"));

	int is_init = 1;
	std::map<int, int>::iterator it;

	for (it = loop_closures.begin(); it != loop_closures.end(); it++)
	{
		sample = adj_dataset[it->first];

		mean = update_particle_filter(pf, map, preproc, viewer, args.get<int>("view"), adj_dataset, sample, it, is_init,
		                              n_corrections_when_reinit);

		// for compatibility issues, we have to specify the pose in relation to a sample in the target dataset.
		Matrix<double, 4, 4>  world2nn = Pose2d::to_matrix(tgt_dataset[it->second]->pose).inverse();
		Matrix<double, 4, 4>  pose_in_nn = world2nn * Pose2d::to_matrix(mean);

		loop_closure_indices.push_back(pair<int, int>(it->second, it->first));
		relative_transform_vector->push_back(pose_in_nn);
		convergence_vector->push_back(1);

		is_init = 0;
	}
}


void
estimate_loop_closures_with_particle_filter_in_map_with_smart_loop_closure_detection(
		NewCarmenDataset &dataset, string dataset_path, vector<pair<int, int>> &loop_closure_indices,
		vector<Matrix<double, 4, 4>> *relative_transform_vector, vector<int> *convergence_vector,
		int n_corrections_when_reinit, CommandLineArguments &args)
{
	// IMPORTANT: these data structures sort the keys and the code below assumes it!!
	std::map<int, int> loop_closures;
	std::set<int> poses_for_mapping;

	detect_loop_closures(dataset, args, &loop_closures, &poses_for_mapping);
	SensorPreproc preproc = create_sensor_preproc(args, &dataset, dataset_path);

	string dir_to_save_maps = "/tmp/loop_closure_maps/";

	if (!boost::filesystem::exists(dir_to_save_maps))
	{
		boost::filesystem::create_directory(dir_to_save_maps);
		create_map(&dataset, preproc, args, dir_to_save_maps, vector<int>(poses_for_mapping.begin(), poses_for_mapping.end()));
	}

	string map_path;
	string log_name = file_name_from_path(dataset_path);

	loop_closure_indices.clear();
	relative_transform_vector->clear();
	convergence_vector->clear();

	map_path = dir_to_save_maps + "/map_occupancy_" + log_name;
	run_particle_filter(map_path,
	                    GridMapTile::TYPE_OCCUPANCY, "reflectivity",
	                    loop_closure_indices, relative_transform_vector, convergence_vector,
	                    n_corrections_when_reinit, args, dataset, dataset, loop_closures,
	                    dataset_path);

	map_path = dir_to_save_maps + "/map_reflectivity_" + log_name;
	run_particle_filter(map_path,
	                    GridMapTile::TYPE_REFLECTIVITY, "reflectivity",
	                    loop_closure_indices, relative_transform_vector, convergence_vector,
	                    n_corrections_when_reinit, args, dataset, dataset, loop_closures,
	                    dataset_path);

	map_path = dir_to_save_maps + "/map_visual_" + log_name;
	run_particle_filter(map_path,
	                    GridMapTile::TYPE_VISUAL, "colour",
	                    loop_closure_indices, relative_transform_vector, convergence_vector,
	                    n_corrections_when_reinit, args, dataset, dataset, loop_closures,
	                    dataset_path);

	map_path = dir_to_save_maps + "/map_semantic_" + log_name;
	run_particle_filter(map_path,
	                    GridMapTile::TYPE_SEMANTIC, "semantic",
	                    loop_closure_indices, relative_transform_vector, convergence_vector,
	                    n_corrections_when_reinit, args, dataset, dataset, loop_closures,
	                    dataset_path);
}


void
estimate_displacements_with_particle_filter_in_map(NewCarmenDataset &target_dataset,
                                                   NewCarmenDataset &dataset_to_adjust,
                                                   string target_dataset_path,
                                                   string dataset_to_adjust_path,
                                                   vector<pair<int, int>> &loop_closure_indices,
                                                   vector<Matrix<double, 4, 4>> *relative_transform_vector,
                                                   vector<int> *convergence_vector,
                                                   int n_corrections_when_reinit,
                                                   CommandLineArguments &args)
{
	string adj_name = file_name_from_path(dataset_to_adjust_path);
	string tgt_name = file_name_from_path(target_dataset_path);
	string dir_maps_are_saved = "/dados/maps2/";
	string map_path;

	//int map_has_to_be_created = 0;
	//if (!boost::filesystem::exists(map_path))
		//map_has_to_be_created = 1;
	assert(boost::filesystem::exists(map_path));

	std::map<int, int> loop_closures;

	for (int i = 0; i < dataset_to_adjust.size(); i++)
		loop_closures.insert(pair<int, int>(i, 0));

	map_path = dir_maps_are_saved + "/map_" + tgt_name + "_occupancy";
	run_particle_filter(map_path,
						GridMapTile::TYPE_OCCUPANCY, "reflectivity",
						loop_closure_indices, relative_transform_vector, convergence_vector,
						n_corrections_when_reinit, args, target_dataset, dataset_to_adjust, loop_closures,
						dataset_to_adjust_path);

	map_path = dir_maps_are_saved + "/map_" + tgt_name + "_reflectivity";
	run_particle_filter(map_path,
						GridMapTile::TYPE_REFLECTIVITY, "reflectivity",
						loop_closure_indices, relative_transform_vector, convergence_vector,
						n_corrections_when_reinit, args, target_dataset, dataset_to_adjust, loop_closures,
						dataset_to_adjust_path);

	map_path = dir_maps_are_saved + "/map_" + tgt_name + "_semantic";
	run_particle_filter(map_path,
						GridMapTile::TYPE_SEMANTIC, "semantic",
						loop_closure_indices, relative_transform_vector, convergence_vector,
						n_corrections_when_reinit, args, target_dataset, dataset_to_adjust, loop_closures,
						dataset_to_adjust_path);

	map_path = dir_maps_are_saved + "/map_" + tgt_name + "_visual";
	run_particle_filter(map_path,
						GridMapTile::TYPE_VISUAL, "colour",
						loop_closure_indices, relative_transform_vector, convergence_vector,
						n_corrections_when_reinit, args, target_dataset, dataset_to_adjust, loop_closures,
						dataset_to_adjust_path);

	/*
	if (map_has_to_be_created)
	{
		printf("Creating map. It may take a while.\n");
		SensorPreproc target_preproc = create_sensor_preproc(args, &target_dataset, target_dataset_path);
		create_map(map, &target_dataset, args.get<int>("step"), target_preproc, args.get<double>("v_thresh"), view, 640);
	}
	*/

	/*
	ParticleFilter pf(args.get<int>("n_particles"),
										args.get<double>("gps_xy_std"),
										args.get<double>("gps_xy_std"),
										degrees_to_radians(args.get<double>("gps_h_std")),
										args.get<double>("v_std"),
										degrees_to_radians(args.get<double>("phi_std")),
										args.get<double>("odom_xy_std"),
										args.get<double>("odom_xy_std"),
										degrees_to_radians(args.get<double>("odom_h_std")),
										args.get<double>("color_red_std"),
										args.get<double>("color_green_std"),
										args.get<double>("color_blue_std"),
										args.get<double>("reflectivity_std")
										);

	pf.set_use_map_weight(1);

	cv::Mat pf_img;
	PointCloudViewer viewer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	SensorPreproc adj_preproc = create_sensor_preproc(args, &dataset_to_adjust, dataset_to_adjust_path);
	loop_closure_indices.clear();
	relative_transform_vector->clear();
	convergence_vector->clear();

	Matrix<double, 4, 4>  world2origin = Pose2d::to_matrix(target_dataset[0]->pose).inverse();

	// initialize particle filter
	pf.reset(dataset_to_adjust[0]->pose.x,
	         dataset_to_adjust[0]->pose.y,
	         dataset_to_adjust[0]->pose.th);

	map.reload(dataset_to_adjust[0]->pose.x,
		         dataset_to_adjust[0]->pose.y);

	int is_first = 1;
	int prev_id = 0;
	DataSample *sample;
	double dt;

	viewer.set_step(0);

	// estimate localization with particle filter
	for (int i = 0; i < dataset_to_adjust.size(); i++)
	{
		sample = dataset_to_adjust[i];

		//if (fabs(sample->v) < 1.0)
			//continue;

		if (i % 50 == 0)
			printf("Cloud %d of %d\n", i, dataset_to_adjust.size());

		adj_preproc.reinitialize(sample);
		load_as_pointcloud(adj_preproc, cloud, SensorPreproc::CAR_REFERENCE);

		// reinitialize if the pose estimate get too far from the target path?
		if (is_first)
		{
			for (int k = 0; k < n_corrections_when_reinit; k++)
			{
				pf.predict(0, 0, 0);

				if (view)
					run_viewer_if_necessary(NULL, map, pf, cloud, viewer, 1, 1, view);

				pf.correct(cloud, map, sample->gps);

				if (view)
					run_viewer_if_necessary(NULL, map, pf, cloud, viewer, 1, 1, view);
			}

			is_first = 0;
		}
		else
		{
			dt = sample->time - dataset_to_adjust.at(prev_id)->time;
			pf.predict(sample->v, sample->phi, dt);

			if (view)
				run_viewer_if_necessary(NULL, map, pf, cloud, viewer, 1, 1, view);

			pf.correct(cloud, map, sample->gps);

			if (view)
				run_viewer_if_necessary(NULL, map, pf, cloud, viewer, 1, 1, view);
		}

		Pose2d mean = pf.mean();
		map.reload(mean.x, mean.y);

		loop_closure_indices.push_back(pair<int, int>(0, i));

		// for compatibility issues, we have to specify the pose in relation to a sample in the target dataset.
		Matrix<double, 4, 4>  pose_in_origin = world2origin * Pose2d::to_matrix(mean);
		relative_transform_vector->push_back(pose_in_origin);
		convergence_vector->push_back(1);

		prev_id = i;
	}
	*/
}


void
estimate_displacements_with_gicp(NewCarmenDataset &target_dataset,
																 NewCarmenDataset &dataset_to_adjust,
																 string target_dataset_path,
																 string dataset_to_adjust_path,
                                 vector<pair<int, int>> &loop_closure_indices,
                                 vector<Matrix<double, 4, 4>> *relative_transform_vector,
                                 vector<int> *convergence_vector,
                                 CommandLineArguments &args)
{
	printf("Running ICPs.\n");

	int i;
	int view = args.get<int>("view");
	int n_processed_clouds = 0;
	int n = loop_closure_indices.size();

#ifdef _OPENMP
	if (view)
		omp_set_num_threads(1);
#endif

	double dist_acc = args.get<double>("dist_to_accumulate");
	double voxel_size = args.get<double>("voxel_size");

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 5) default(none) private(i) \
		shared(target_dataset, dataset_to_adjust, convergence_vector, relative_transform_vector, \
		       loop_closure_indices, n_processed_clouds, n, view, args, dist_acc, voxel_size, \
					 target_dataset_path, dataset_to_adjust_path)
#endif
	for (i = 0; i < n; i++)
	{
		SensorPreproc target_preproc = create_sensor_preproc(args, &target_dataset, target_dataset_path);
		SensorPreproc adj_preproc = create_sensor_preproc(args, &dataset_to_adjust, dataset_to_adjust_path);

		run_icp_step(target_dataset,
		             dataset_to_adjust,
		             loop_closure_indices[i].first,
		             loop_closure_indices[i].second,
		             &(relative_transform_vector->at(i)),
		             &(convergence_vector->at(i)),
		             target_preproc,
		             adj_preproc,
		             voxel_size,
		             dist_acc,
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
