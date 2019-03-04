
#include <algorithm>
#include <vector>
#include <Eigen/Core>
#include "gicp.h"
#include <carmen/segmap_util.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/visualization/pcl_visualizer.h>


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
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) 
			&& raw_cloud->at(i).x < 70.0  // remove max range
			&& raw_cloud->at(i).z > -1.5  // remove ground
			&& raw_cloud->at(i).z < -0.0  // remove tree tops
		)
			cloud->push_back(raw_cloud->at(i));
	}

	return cloud;
}


void
run_icp_step(DatasetCarmen &test_dataset, DatasetCarmen &mapping_dataset, int test_id, int mapping_id, 
	Matrix<double, 4, 4> *relative_transform, int *convergence_flag)
{
	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);
	//PointCloud<PointXYZRGB>::Ptr aligned2(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr source_moved(new PointCloud<PointXYZRGB>);

	mapping_dataset.load_pointcloud(mapping_id, target, mapping_dataset.data[mapping_id].v, mapping_dataset.data[mapping_id].phi);
	test_dataset.load_pointcloud(test_id, source, test_dataset.data[test_id].v, test_dataset.data[test_id].phi);

	source = filter_pointcloud(source);
	target = filter_pointcloud(target);

	Pose2d target_pose = mapping_dataset.data[mapping_id].pose;
	Pose2d source_pose = test_dataset.data[test_id].pose;

	source_pose.x -= target_pose.x;
	source_pose.y -= target_pose.y;
	target_pose.x = 0.;
	target_pose.y = 0.;
	
	Matrix<double, 4, 4> guess = 
		Pose2d::to_matrix(target_pose).inverse() *
		Pose2d::to_matrix(source_pose);
	
	pcl::transformPointCloud(*source, *source_moved, guess);
	run_gicp(source_moved, target, relative_transform, convergence_flag, aligned, 0.01);

	//pcl::transformPointCloud(*source, *aligned2, ((*relative_transform) * guess).cast<float>());
	/*
	if (1)
	{
		printf("target: %lf %lf %lf test: %lf %lf %lf\n",
			target_pose.x, target_pose.y, target_pose.th,
			source_pose.x, source_pose.y, source_pose.th);
		printf("converged: %d\n", *convergence_flag);

		static pcl::visualization::PCLVisualizer *viewer = NULL;

		if (viewer == NULL)
			viewer = new pcl::visualization::PCLVisualizer();

		viewer->removeAllPointClouds();
		viewer->setBackgroundColor(.5, .5, .5);
		viewer->addPointCloud(target, "target");
		viewer->addPointCloud(source_moved, "source");
		viewer->addPointCloud(aligned, "aligned");
		//viewer->addPointCloud(aligned2, "aligned2d");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned2d");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source"); 
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target"); 
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "aligned"); 
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "aligned2d");
		viewer->spinOnce();
		//viewer->spin();
	}
	*/
}


void
write_output(FILE *out_file, vector<pair<int, int>> &loop_closure_indices, vector<Matrix<double, 4, 4>> &relative_transform_vector, vector<int> &convergence_vector)
{
	for (int i = 0; i < loop_closure_indices.size(); i++)
	{
		fprintf(out_file, "%d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
            loop_closure_indices[i].first, loop_closure_indices[i].second, convergence_vector[i],
            relative_transform_vector[i](0, 0), relative_transform_vector[i](0, 1), relative_transform_vector[i](0, 2), relative_transform_vector[i](0, 3),
            relative_transform_vector[i](1, 0), relative_transform_vector[i](1, 1), relative_transform_vector[i](1, 2), relative_transform_vector[i](1, 3),
            relative_transform_vector[i](2, 0), relative_transform_vector[i](2, 1), relative_transform_vector[i](2, 2), relative_transform_vector[i](2, 3),
            relative_transform_vector[i](3, 0), relative_transform_vector[i](3, 1), relative_transform_vector[i](3, 2), relative_transform_vector[i](3, 3)
        ); 
	}
}


void
write_output_to_graphslam(char *out_file, DatasetCarmen &test_dataset, DatasetCarmen &mapping_dataset, 
	vector<pair<int, int>> &indices, vector<Matrix<double, 4, 4>> &relative_transform_vector, vector<int> &convergence_vector)
{
    FILE *f = fopen(out_file, "w");

    for (int i = 0; i < indices.size(); i++)
    {
	    Pose2d pose_source = test_dataset.data[indices[i].first].pose;
        Pose2d pose_target = mapping_dataset.data[indices[i].second].pose;

	    pose_source.x -= pose_target.x;
	    pose_source.y -= pose_target.y;
	    pose_target.x = 0.;
	    pose_target.y = 0.;

	    Matrix<double, 4, 4> guess = 
		    Pose2d::to_matrix(pose_target).inverse() *
		    Pose2d::to_matrix(pose_source);

        Matrix<double, 4, 4> relative_pose = Pose2d::to_matrix(pose_target) * (relative_transform_vector[i] * guess);
        Pose2d pose = Pose2d::from_matrix(relative_pose);

		pose.x += mapping_dataset.data[indices[i].second].pose.x;
		pose.y += mapping_dataset.data[indices[i].second].pose.y;
        
        fprintf(f, "%d %d %d %lf %lf %lf\n", indices[i].first, indices[i].second,
            convergence_vector[i], pose.x, pose.y, pose.th);
    }

    fclose(f);
}


void
find_all_loop_closure_poses(DatasetCarmen &test_dataset, DatasetCarmen &mapping_dataset, vector<pair<int, int>> &loop_closure_indices)
{
	for (int i = 0; i < test_dataset.data.size(); i++)
	{
		if (fabs(test_dataset.data[i].v) < 1.0)
			continue;

		double min_dist = DBL_MAX;
		int nn_id = -1;

		for (int j = 0; j < mapping_dataset.data.size(); j++)
		{
			double delta_x = test_dataset.data[i].pose.x - mapping_dataset.data[j].pose.x;
			double delta_y = test_dataset.data[i].pose.y - mapping_dataset.data[j].pose.y;
			double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

			if (dist < min_dist)
			{
				min_dist = dist;
				nn_id = j;
			}
		}

		//printf("%d %d %lf\n", i, nn_id, min_dist);

		if (min_dist < 5.0)
			loop_closure_indices.push_back(pair<int, int>(i, nn_id));
	}

	printf("Num loop closures: %ld\n", loop_closure_indices.size());
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 5)
		exit(printf("Use %s <data-directory-mapping-log> <data-directory-test-log> <output_file> <output_file_to_graphslam>\n", argv[0]));

	int i;
	DatasetCarmen mapping_dataset(argv[1], 0);
	DatasetCarmen test_dataset(argv[2], 0);

	FILE *out_file = fopen(argv[3], "w");
	if (out_file == NULL)
		exit(printf("Output file '%s' could not be open.\n", argv[2]));

	vector<pair<int, int>> loop_closure_indices;
	find_all_loop_closure_poses(test_dataset, mapping_dataset, loop_closure_indices);
	
	// remove the last elements of the vector to run tests more efficiently
	// loop_closure_indices.erase(loop_closure_indices.begin()+400, loop_closure_indices.end());

	int n_processed_clouds = 0;
	int n = loop_closure_indices.size();

	vector<Matrix<double, 4, 4>> relative_transform_vector(n);
	vector<int> convergence_vector(n);

    printf("Running.\n");

   	#pragma omp parallel for default(none) private(i) shared(test_dataset, mapping_dataset, convergence_vector, relative_transform_vector, loop_closure_indices, n_processed_clouds, n) 
    for (i = 0; i < n; i++)
	{
	    run_icp_step(test_dataset, mapping_dataset, 
			loop_closure_indices[i].first, loop_closure_indices[i].second, 
            &(relative_transform_vector[i]), &(convergence_vector[i]));

		#pragma omp critical
        {
            n_processed_clouds++;

            if (n_processed_clouds % 100 == 0)
    	    	printf("%d processed clouds of %d\n", n_processed_clouds, n);
        }
	}

	write_output(out_file, loop_closure_indices, relative_transform_vector, convergence_vector);
    write_output_to_graphslam(argv[4], test_dataset, mapping_dataset, loop_closure_indices, relative_transform_vector, convergence_vector);

	fclose(out_file);

	return 0;
}
