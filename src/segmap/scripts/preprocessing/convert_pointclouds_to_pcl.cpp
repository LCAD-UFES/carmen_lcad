

#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <carmen/carmen.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>


using namespace std;
using namespace pcl;


double velodyne_vertical_angles[32] =
{
		-30.6700000, -29.3300000, -28.0000000, -26.6700000, -25.3300000, -24.0000000, -22.6700000, -21.3300000,
		-20.0000000, -18.6700000, -17.3300000, -16.0000000, -14.6700000, -13.3300000, -12.0000000, -10.6700000,
		-9.3299999, -8.0000000, -6.6700001, -5.3299999, -4.0000000, -2.6700001, -1.3300000, 0.0000000, 1.3300000,
		2.6700001, 4.0000000, 5.3299999, 6.6700001, 8.0000000, 9.3299999, 10.6700000
};

int velodyne_ray_order[32] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};


PointXYZRGB
compute_point_from_velodyne(double v_angle, double h_angle, double radius, unsigned char intensity)
{
	// build a new point
	PointXYZRGB point;

	double cos_rot_angle = cos(h_angle);
	double sin_rot_angle = sin(h_angle);

	double cos_vert_angle = cos(v_angle);
	double sin_vert_angle = sin(v_angle);

	double xy_distance = radius * cos_vert_angle;

	point.x = (xy_distance * cos_rot_angle);
	point.y = (xy_distance * sin_rot_angle);
	point.z = (radius * sin_vert_angle);

	point.r = intensity;
	point.g = intensity;
	point.b = intensity;

	return point;
}


void
load_pointcloud(char *path, int n_rays, PointCloud<PointXYZRGB>::Ptr cloud)
{
	FILE *f = fopen(path, "rb");

	if (f == NULL)
		exit(printf("File %s not found.\n", path));

	double h_angle, v_angle;
	unsigned short distances[32];
	unsigned char intensities[32];
	double range;

	cloud->clear();

	for(int i = 0; i < n_rays; i++)
	{
		fread(&h_angle, sizeof(double), 1, f);
		fread(distances, sizeof(unsigned short), 32, f);
		fread(intensities, sizeof(unsigned char), 32, f);

		h_angle = M_PI * h_angle / 180.;

		for (int j = 0; j < 32; j++)
		{
			range = (double) distances[velodyne_ray_order[j]] / 500.;
			v_angle = velodyne_vertical_angles[j];
			v_angle = M_PI * v_angle / 180.;

			PointXYZRGB point = compute_point_from_velodyne(v_angle, -h_angle, range, intensities[velodyne_ray_order[j]]);
			cloud->push_back(point);
		}
	}

	fclose(f);
}


int
main(int argc, char **argv)
{
	if (argc < 3)
	{
		printf("\nError: Use %s <file-list_velodynes> <output_dir>\n\n", argv[0]);
		exit(0);
	}

	int n_rays;
	double time;
	char str[256], dummy[256], path[256];
	sprintf(str, "rm -rf %s && mkdir %s", argv[2], argv[2]);
	printf("Command: %s\n", str);
	system(str);

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	FILE *f = fopen(argv[1], "r");

	if (f == NULL)
		exit(printf("Error: file '%s' not found!\n", argv[1]));

	int i = 0;
	int success = 0;

	while (!feof(f))
	{
		success = fscanf(f, "\n%s %s %d %lf %s %s\n", dummy, path, &n_rays, &time, dummy, dummy);

		if (success != 6)
			continue;

		load_pointcloud(path, n_rays, cloud);

		sprintf(str, "%s/%lf.ply", argv[2], time);
		pcl::io::savePLYFileBinary(str, *cloud);

		if (i % 100 == 0)
			printf("Saving pointcloud %d: %s\n", i+1, str);

		i++;
	}

	return 0;
}

