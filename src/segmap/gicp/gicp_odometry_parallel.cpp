

#include <vector>
#include <Eigen/Core>
#include "gicp.h"
#include <carmen/segmap_util.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>


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
			 raw_cloud->at(i).x < 70.0 && raw_cloud->at(i).z < 0 && raw_cloud->at(i).z > -1.5)
			cloud->push_back(raw_cloud->at(i));
	}

	return cloud;
}


void
run_icp_step(DatasetCarmen &dataset, int i, vector<Matrix<double, 4, 4>> &relative_transform_vector, vector<int> &convergence_vector)
{
	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr source_moved(new PointCloud<PointXYZRGB>);

	dataset.load_pointcloud(i - 1, target, dataset.data[i - 1].v, dataset.data[i - 1].phi);
	dataset.load_pointcloud(i, source, dataset.data[i].v, dataset.data[i].phi);

	source = filter_pointcloud(source);
	target = filter_pointcloud(target);

	Pose2d target_pose = dataset.data[i-1].pose;
	Pose2d source_pose = dataset.data[i].pose;

	source_pose.x -= target_pose.x;
	source_pose.y -= target_pose.y;
	target_pose.x = 0.;
	target_pose.y = 0.;
	
    //** TEST **//
	//source_pose = target_pose;

	Matrix<double, 4, 4> guess = 
		Pose2d::to_matrix(target_pose).inverse() *
		Pose2d::to_matrix(source_pose);
	
	pcl::transformPointCloud(*source, *source_moved, guess);

	Matrix<double, 4, 4> correction;
	run_gicp(source_moved, target, &correction, &(convergence_vector[i-1]), aligned, 0.0);

	relative_transform_vector[i-1] = correction;
}


void
write_output(FILE *out_file, vector<Matrix<double, 4, 4>> &relative_transform_vector, vector<int> &convergence_vector)
{
	for (int i = 0; i < relative_transform_vector.size(); i++)
	{
		fprintf(out_file, "%d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
            i, i + 1, convergence_vector[i],
            relative_transform_vector[i](0, 0), relative_transform_vector[i](0, 1), relative_transform_vector[i](0, 2), relative_transform_vector[i](0, 3),
            relative_transform_vector[i](1, 0), relative_transform_vector[i](1, 1), relative_transform_vector[i](1, 2), relative_transform_vector[i](1, 3),
            relative_transform_vector[i](2, 0), relative_transform_vector[i](2, 1), relative_transform_vector[i](2, 2), relative_transform_vector[i](2, 3),
            relative_transform_vector[i](3, 0), relative_transform_vector[i](3, 1), relative_transform_vector[i](3, 2), relative_transform_vector[i](3, 3)
        ); 
	}
}


void
write_output_to_graphslam(char *out_file, DatasetCarmen &dataset, vector<Matrix<double, 4, 4>> &relative_transform_vector, vector<int> &convergence_vector)
{
    FILE *f = fopen(out_file, "w");

    for (int i = 1; i < relative_transform_vector.size(); i++)
    {
        Pose2d pose_target = dataset.data[i-1].pose;
	    Pose2d pose_source = dataset.data[i].pose;

	    pose_source.x -= pose_target.x;
	    pose_source.y -= pose_target.y;
	    pose_target.x = 0.;
	    pose_target.y = 0.;

	    Matrix<double, 4, 4> guess = 
		    Pose2d::to_matrix(pose_target).inverse() *
		    Pose2d::to_matrix(pose_source);

        Matrix<double, 4, 4> relative_pose = guess * relative_transform_vector[i];
        Pose2d pose = Pose2d::from_matrix(relative_pose);
        
        fprintf(f, "%d %d %d %lf %lf %lf\n", i-1, i,
            convergence_vector[i], pose.x, pose.y, pose.th);
    }

    fclose(f);
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 4)
		exit(printf("Use %s <data-directory> <output_file> <output_file_graphslam>\n", argv[0]));

	int i;
	DatasetCarmen dataset(argv[1], 0);

	FILE *out_file = fopen(argv[2], "w");

	if (out_file == NULL)
		exit(printf("Output file '%s' could not be open.\n", argv[2]));

    int size = dataset.data.size() - 1;
    //int size = 100;
	vector<Matrix<double, 4, 4>> relative_transform_vector(size);
	vector<int> convergence_vector(size);

    printf("Running.\n");

	int n_processed_clouds = 0;

	#pragma omp parallel for default(none) shared(dataset, convergence_vector, relative_transform_vector, size, n_processed_clouds) private(i)
    for (i = 1; i < (size + 1); i++)
	{
		run_icp_step(dataset, i, relative_transform_vector, convergence_vector);

        #pragma omp critical
        {
            n_processed_clouds++;

            if (n_processed_clouds % 100 == 0)
    	    	printf("%d processed clouds of %d\n", n_processed_clouds, size);
        }
	}

	write_output(out_file, relative_transform_vector, convergence_vector);
    write_output_to_graphslam(argv[3], dataset, relative_transform_vector, convergence_vector);
	fclose(out_file);

	return 0;
}
