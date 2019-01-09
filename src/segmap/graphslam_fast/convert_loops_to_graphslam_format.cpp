
#include <cstdio>
#include <cstdlib>
#include <Eigen/Core>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>


void
read_odom_data(char *filename, vector<pair<int,int>> &indices,
	vector<Matrix<double, 4, 4>> &relative_transform_vector,
	vector<int> &convergence_vector)
{
    FILE *f = fopen(filename, "r");

    if (f == NULL)
        exit(printf("Error: file '%s' not found.\n", filename));

    int from, to, converged, n;
    double data[16];
    Matrix<double, 4, 4> relative_pose;

    while (!feof(f))
    {
        n = fscanf(f, "%d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
            &from, &to, &converged,
            data, data + 1, data + 2, data + 3, 
            data + 4, data + 5, data + 6, data + 7,
            data + 8, data + 9, data + 10, data + 11,
            data + 12, data + 13, data + 14, data + 15);

        if (n != 19)
            continue;

        indices.push_back(pair<int, int>(from, to));
        convergence_vector.push_back(converged);

        relative_pose << data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11],
            data[12], data[13], data[14], data[15];
        
        relative_transform_vector.push_back(relative_pose);
    }

    printf("%ld relative poses loaded!\n", relative_transform_vector.size());

    fclose(f);
}


int
main(int argc, char **argv)
{
    if (argc < 4)
        exit(printf("Error: Use %s <dataset_dir> <odom_file> <output_file>\n", argv[0]));

	DatasetCarmen dataset(argv[1], 0);

    vector<pair<int,int>> indices;
	vector<Matrix<double, 4, 4>> relative_transform_vector;
	vector<int> convergence_vector;
    read_odom_data(argv[2], indices, relative_transform_vector, convergence_vector);

    FILE *f = fopen(argv[3], "w");

    for (int i = 0; i < relative_transform_vector.size(); i++)
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

        Matrix<double, 4, 4> relative_pose = relative_transform_vector[i] * guess;
        Pose2d pose = Pose2d::from_matrix(relative_pose);

        fprintf(f, "%d %d %d %lf %lf %lf\n", 
            indices[i].first, indices[i].second,
            convergence_vector[i], pose.x, pose.y, pose.th
        );
    }

    fclose(f);
    return 0;
}

