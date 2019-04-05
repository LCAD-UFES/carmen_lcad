
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
#include "gicp.h"

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
run_viewer_if_necessary(Pose2d pose,
												GridMap &map,
												ParticleFilter &pf,
												PointCloud<PointXYZRGB>::Ptr cloud,
												PointCloudViewer &viewer,
												int pf_was_updated,
												int view)
{
	if (view)
	{
		Mat img;

		if (pf_was_updated)
			img = pf_view(pf, map, pose, pf.mean(), cloud, 1);
		else
		{
			img = map.to_image().clone();
			draw_pose(map, img, pose, Scalar(0, 255, 0));
		}

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
create_target_accumulating_clouds(NewCarmenDataset &target_dataset,
                                  SensorPreproc &target_preproc,
                                  int target_id,
                                  double dist_accumulate_target_cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target)
{
	Matrix<double, 4, 4> transform_to_target;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	target->clear();

	double d;
	int i, st, end;

	// walk forward and backward in the dataset to find clouds around the target one.
	d = 0;
	for (i = target_id - 1; i >= 0 && d < dist_accumulate_target_cloud; i--)
		//d += dist2d(target_dataset[i]->pose.x, target_dataset[i]->pose.y, target_dataset[i+1]->pose.x, target_dataset[i+1]->pose.y);
		d = dist2d(target_dataset[i]->pose.x, target_dataset[i]->pose.y, target_dataset[target_id]->pose.x, target_dataset[target_id]->pose.y);

	st = (i >= 0) ? i : 0;

	d = 0;
	for (i = target_id + 1; i < target_dataset.size() && d < dist_accumulate_target_cloud; i++)
		//d += dist2d(target_dataset[i]->pose.x, target_dataset[i]->pose.y, target_dataset[i-1]->pose.x, target_dataset[i-1]->pose.y);
		d = dist2d(target_dataset[i]->pose.x, target_dataset[i]->pose.y, target_dataset[target_id]->pose.x, target_dataset[target_id]->pose.y);

	end = (i < target_dataset.size()) ? i : target_dataset.size();

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
                                        int *nn_id)
{
	*nn_id = -1;
	double min_dist = DBL_MAX;

	for (int j = from; j < to; j++)
	{
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
	run_gicp(source, target, relative_transform, convergence_flag, aligned, voxel_grid_size);

	if (view)
	{
		static PointCloudViewer *viewer = NULL;

		if (viewer == NULL)
			viewer = new PointCloudViewer(3);

		viewer->clear();
		viewer->show(target); //, 0, 1, 0);
		viewer->show(source, 1, 0, 0);
		viewer->show(aligned, 0, 0, 1);
		viewer->loop();
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

	create_target_accumulating_clouds(target_dataset, target_preproc,
	                                  target_id, dist_accumulate_target_cloud,
	                                  cloud);

	map.reload(0, 0);

	for (int i = 0; i < cloud->size(); i++)
		map.add_point(cloud->at(i));

	source_preproc.reinitialize(source_dataset[source_id]);
	load_as_pointcloud(source_preproc, cloud, SensorPreproc::CAR_REFERENCE);

	Matrix<double, 4, 4> smat = compute_source2target_transform(target_dataset[target_id]->pose,
																															source_dataset[source_id]->pose);

	Pose2d source_as_pose = Pose2d::from_matrix(smat);
	pf.reset(source_as_pose.x, source_as_pose.y, source_as_pose.th);

	printf("Reset\n");
	run_viewer_if_necessary(source_as_pose, map, pf, cloud, viewer, 1, view);

	for (int i = 0; i < n_corrections_when_reinit; i++)
	{
		pf.predict(0, 0, 0); // just to add a little noise

		printf("Prediction\n");
		run_viewer_if_necessary(source_as_pose, map, pf, cloud, viewer, 1, view);

		pf.correct(cloud, map, source_dataset[source_id]->gps);

		printf("Correction\n");
		run_viewer_if_necessary(source_as_pose, map, pf, cloud, viewer, 1, view);
	}

	Pose2d estimate = pf.mean();
	(*relative_transform) = Pose2d::to_matrix(estimate);
	(*convergence_flag) = 1;
}


void
save_output(std::string path,
            NewCarmenDataset &reference_dataset,
            NewCarmenDataset &dataset_to_be_adjusted,
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
		Pose2d target_pose = reference_dataset[indices[i].first]->pose;
		Pose2d source_pose = dataset_to_be_adjusted[indices[i].second]->pose;

		source2target = compute_source2target_transform(target_pose, source_pose);
		corrected_pose = relative_transform_vector[i] * source2target;

		if (project_to_world)
			corrected_pose = Pose2d::to_matrix(target_pose) * corrected_pose;

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
