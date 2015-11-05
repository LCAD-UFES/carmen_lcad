#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct
{
	double x, y, z;
	double yaw, pitch, roll;
}tPose;

typedef struct
{
	int index;
	char file_name[1024];
	tPose pose;
	int correspondence;
	double distance;
}tData;

tData *test_data_set = NULL;
tData *training_data_set = NULL;

int
read_training_data_set(char* path_name, int size_of_training_data)
{
	int i;
	FILE* fd;

	if(training_data_set == NULL)
		training_data_set = (tData *) malloc (size_of_training_data * sizeof(tData));

	if((fd = fopen(path_name, "r")) == NULL)
		return -1;

	for(i = 0; i < size_of_training_data; i++)
	{
		training_data_set[i].index = i;
		training_data_set[i].correspondence = 0;

		fscanf(fd, "\n%s", training_data_set[i].file_name);

		fscanf(fd, "%lf", &training_data_set[i].pose.x);
		fscanf(fd, "%lf", &training_data_set[i].pose.y);
		fscanf(fd, "%lf", &training_data_set[i].pose.z);

		fscanf(fd, "%lf", &training_data_set[i].pose.yaw);
		fscanf(fd, "%lf", &training_data_set[i].pose.pitch);
		fscanf(fd, "%lf", &training_data_set[i].pose.roll);
	}

	fclose(fd);
	return 0;
}

int
read_test_data_set(char* path_name, int size_of_test_data)
{
	int i;
	FILE* fd;

	if(test_data_set == NULL)
		test_data_set = (tData *) malloc (size_of_test_data * sizeof(tData));

	if((fd = fopen(path_name, "r")) == NULL)
			return -1;

	for(i = 0; i < size_of_test_data; i++)
	{
		test_data_set[i].index = i;
		test_data_set[i].correspondence = 0;

		fscanf(fd, "\n%s", test_data_set[i].file_name);

		fscanf(fd, "%lf", &test_data_set[i].pose.x);
		fscanf(fd, "%lf", &test_data_set[i].pose.y);
		fscanf(fd, "%lf", &test_data_set[i].pose.z);

		fscanf(fd, "%lf", &test_data_set[i].pose.yaw);
		fscanf(fd, "%lf", &test_data_set[i].pose.pitch);
		fscanf(fd, "%lf", &test_data_set[i].pose.roll);
	}

	fclose(fd);
	return 0;
}

int write_test_data_set(char* path_name, int size_of_test_data)
{
	FILE* fd;

	if((fd = fopen(path_name, "w")) == NULL)
		return -1;

	for(int i = 0; i < size_of_test_data; i++)
	{
		fprintf(fd, "%s ", test_data_set[i].file_name);
		fprintf(fd, "%f %f %f %f %f %f ", test_data_set[i].pose.x, test_data_set[i].pose.y, test_data_set[i].pose.z,
				test_data_set[i].pose.yaw, test_data_set[i].pose.pitch, test_data_set[i].pose.roll);
		fprintf(fd, "%d ", test_data_set[i].correspondence);
		fprintf(fd, "; %f ", test_data_set[i].distance);
		fprintf(fd, "\n");
	}

	fclose(fd);

	return 0;
}

double euclidean_distance_between_data_poses(tData test_data, tData training_data)
{
	return sqrt(((test_data.pose.x - training_data.pose.x) * (test_data.pose.x-training_data.pose.x))
			+((test_data.pose.y - training_data.pose.y) * (test_data.pose.y - training_data.pose.y))
			+((test_data.pose.z - training_data.pose.z) * (test_data.pose.z - training_data.pose.z)));
}

void
do_correspondence_between_datasets(int size_of_test_data, int size_of_training_data)
{
	int i, j;
	double distance, min_distance;

	for(i = 0; i < size_of_test_data; i++)
	{
		distance = 0.0;
		min_distance = 999999.0;

		for(j = 0; j < size_of_training_data; j++)
		{
			distance = euclidean_distance_between_data_poses(test_data_set[i], training_data_set[j]);

			if(distance < min_distance)
			{
				min_distance = distance;
				test_data_set[i].distance = min_distance;
				test_data_set[i].correspondence = j;
			}
		}
	}
}

int
main()
{
	read_training_data_set((char *) "/media/OS/Users/Lauro/Downloads/Log/datasets/2013_1.0/data_orig.txt", 2689);
	read_test_data_set((char *) "/media/OS/Users/Lauro/Downloads/Log/datasets/2012_1.0/data_orig.txt", 2485);

	do_correspondence_between_datasets(2485, 2689);

	write_test_data_set((char *) "/media/OS/Users/Lauro/Downloads/Log/datasets/2013_1.0/data_assoc.txt", 2485);

	return 0;
}
