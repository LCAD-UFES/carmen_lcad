
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

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


PointCloud<PointXYZRGB>::Ptr
load_pointcloud(const char *path)
{
	int c;
	double h_angle, v_angle;
	unsigned short distances[32];
	unsigned char intensities[32];
	double range;

	FILE *f = fopen(path, "rb");

	if (f == NULL)
		exit(printf("File '%s' not found.\n", path));

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	while (!feof(f))
	{
		c = fread(&h_angle, sizeof(double), 1, f);
		if (c != 1) break;  // reading error

		c = fread(distances, sizeof(unsigned short), 32, f);
		if (c != 32) break;  // reading error

		fread(intensities, sizeof(unsigned char), 32, f);
		if (c != 32) break;  // reading error

		h_angle = M_PI * (-h_angle) / 180.;

		for (int j = 0; j < 32; j++)
		{
			range = (double) distances[velodyne_ray_order[j]] / 500.;
			v_angle = velodyne_vertical_angles[j];
			v_angle = M_PI * v_angle / 180.;

			PointXYZRGB point = compute_point_from_velodyne(v_angle, h_angle, range, intensities[velodyne_ray_order[j]]);
			cloud->push_back(point);
		}
	}

	fclose(f);
	return cloud;
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <input cloud .bin> [optional: output cloud path .ply]\n", argv[0]));

	char *input_cloud_path = argv[1];
	char *output_path = NULL;

	if (argc == 3)
		output_path = argv[2];
	
	PointCloud<PointXYZRGB>::Ptr cloud = load_pointcloud(input_cloud_path);

	if (output_path != NULL)
	{
		pcl::io::savePLYFileBinary(output_path, *cloud);
		printf("Cloud saved to '%s'\n", output_path);
	}

	pcl::visualization::PCLVisualizer viewer("CloudViewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(2);
	viewer.addPointCloud(cloud);
	viewer.spin();

	return 0;
}


