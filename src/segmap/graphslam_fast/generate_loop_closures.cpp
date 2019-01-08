
#include <algorithm>
#include <vector>
#include <Eigen/Core>
#include "graphslam_util.h"
#include <carmen/segmap_util.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


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
		if ((fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) && 
			 raw_cloud->at(i).x < 70.0) // && raw_cloud->at(i).z > -1.0)
			cloud->push_back(raw_cloud->at(i));
	}

	return cloud;
}


void
run_icp_step(DatasetCarmen &dataset, int from, int to, Matrix<double, 4, 4> *relative_transform, int *convergence_flag)
{
	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr source_moved(new PointCloud<PointXYZRGB>);

	dataset.load_pointcloud(from, target);
	dataset.load_pointcloud(to, source);

	source = filter_pointcloud(source);
	target = filter_pointcloud(target);

	Pose2d target_pose = dataset.data[from].pose;
	Pose2d source_pose = dataset.data[to].pose;

	source_pose.x -= target_pose.x;
	source_pose.y -= target_pose.y;
	target_pose.x = 0.;
	target_pose.y = 0.;
	
	Matrix<double, 4, 4> guess = 
		Pose2d::to_matrix(target_pose).inverse() *
		Pose2d::to_matrix(source_pose);
	
	pcl::transformPointCloud(*source, *source_moved, guess);

	run_gicp(source_moved, target, relative_transform, convergence_flag, aligned, 0.05);
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
write_output_to_graphslam(char *out_file, DatasetCarmen &dataset, vector<pair<int, int>> &indices, vector<Matrix<double, 4, 4>> &relative_transform_vector, vector<int> &convergence_vector)
{
    FILE *f = fopen(out_file, "w");

    for (int i = 0; i < indices.size(); i++)
    {
        Pose2d pose_target = dataset.data[indices[i].first].pose;
	    Pose2d pose_source = dataset.data[indices[i].second].pose;

	    pose_source.x -= pose_target.x;
	    pose_source.y -= pose_target.y;
	    pose_target.x = 0.;
	    pose_target.y = 0.;

	    Matrix<double, 4, 4> guess = 
		    Pose2d::to_matrix(pose_target).inverse() *
		    Pose2d::to_matrix(pose_source);

        Matrix<double, 4, 4> relative_pose = guess * relative_transform_vector[i];
        Pose2d pose = Pose2d::from_matrix(relative_pose);
        
        fprintf(f, "%d %d %d %lf %lf %lf\n", indices[i].first, indices[i].second,
            convergence_vector[i], pose.x, pose.y, pose.th);
    }

    fclose(f);
}


struct myclass 
{
    bool operator() (pair<double, int> i, pair<double, int> j) 
    { 
        return (i.first < j.first); 
    }
} myobject;


void
find_loop_closure_poses(DatasetCarmen &dataset, vector<pair<int, int>> &loop_closure_indices)
{
	for (int i = 0; i < dataset.data.size(); i++)
	{
        if (fabs(dataset.data[i].v) < 0.5)
            continue;

        vector<pair<double, int>> candidates;

		for (int j = i + 1; j < dataset.data.size(); j++)
		{
			double delta_x = dataset.data[i].pose.x - dataset.data[j].pose.x;
			double delta_y = dataset.data[i].pose.y - dataset.data[j].pose.y;
			double delta_t = dataset.data[i].velodyne_time - dataset.data[j].velodyne_time;

			double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

			if (dist < 5.0 && fabs(delta_t) > 20.)
                candidates.push_back(pair<double, int>(dist, j));
		}

        std::sort(candidates.begin(), candidates.end(), myobject);
        int n = (candidates.size() < 1) ? candidates.size() : 1;

        for (uint k = 0; k < n; k++)
		{
    		loop_closure_indices.push_back(pair<int, int>(i, candidates[k].second));

            /*
            printf("%d %d\n", i, candidates[k].second);
           	cv::Mat imgi = dataset.load_image(i);
           	cv::Mat imgj = dataset.load_image(candidates[k].second);
            cv::imshow("i", imgi);
            cv::imshow("j", imgj);
            cv::waitKey(1);
            */
        }
	}

	printf("Num loop closures: %ld\n", loop_closure_indices.size());
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 4)
		exit(printf("Use %s <data-directory> <output_file> <output_file_to_graphslam>\n", argv[0]));

	int i;
	DatasetCarmen dataset(argv[1], 0);

	FILE *out_file = fopen(argv[2], "w");

	if (out_file == NULL)
		exit(printf("Output file '%s' could not be open.\n", argv[2]));

    int size = dataset.data.size() - 1;
    //int size = 100;

	vector<Matrix<double, 4, 4>> relative_transform_vector(size);
	vector<int> convergence_vector(size);
	vector<pair<int, int>> loop_closure_indices;

    printf("Running.\n");
	find_loop_closure_poses(dataset, loop_closure_indices);

	int n_processed_clouds = 0;
	int n = loop_closure_indices.size();

   	#pragma omp parallel for default(none) private(i) shared(dataset, convergence_vector, relative_transform_vector, size, loop_closure_indices, n_processed_clouds, n) 
    for (i = 0; i < n; i++)
	{
	    run_icp_step(dataset, loop_closure_indices[i].first, loop_closure_indices[i].second, 
            &(relative_transform_vector[i]), &(convergence_vector[i]));

		#pragma omp critical
        {
            n_processed_clouds++;

            if (n_processed_clouds % 100 == 0)
    	    	printf("%d processed clouds of %d\n", n_processed_clouds, n);
        }
	}

	write_output(out_file, loop_closure_indices, relative_transform_vector, convergence_vector);
    write_output_to_graphslam(argv[3], dataset, loop_closure_indices, relative_transform_vector, convergence_vector);

	fclose(out_file);

	return 0;
}
