

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

const double TIME_SPENT_IN_EACH_SCAN = 0.000046091445;
const double distance_between_rear_wheels = 1.535;
const double distance_between_front_and_rear_axles = 2.625;
const double distance_between_front_car_and_front_wheels = 0.85;
const double distance_between_rear_car_and_rear_wheels = 0.96;
const double car_length = (distance_between_front_and_rear_axles +
						distance_between_rear_car_and_rear_wheels +
						distance_between_front_car_and_front_wheels);
const double car_width = distance_between_rear_wheels;


class Data
{
public:
	char cloud_path[128];
	int n_rays;
	double cloud_time;

	char image_path[128];
	int w, h, size, n;
	double image_time;

	double x, y, quality, gps_time;
	double angle, angle_quality, gps2_time;

	double v, phi, ack_time;
};


void
load_data(char *name, vector<Data> &lines)
{
	FILE *f = fopen(name, "r");
	char dummy[128];
	int idummy;

	while (!feof(f))
	{
		Data d;

		char c = fgetc(f);
		if (c != 'V')
			continue;

		fscanf(f, "\n%s %s %d %lf ",
				dummy, d.cloud_path, &d.n_rays, &d.cloud_time);

		fscanf(f, " %s %s %d %d %d %d %lf ",
				dummy, d.image_path, &d.w, &d.h, &d.size, &d.n, &d.image_time);

		fscanf(f, " %s %d %lf %lf %lf %lf ",
			dummy, &idummy, &d.x, &d.y, &d.quality, &d.gps_time);

		fscanf(f, " %s %d %lf %lf %lf ",
			dummy,  &idummy, &d.angle, &d.angle_quality, &d.gps2_time);

		fscanf(f, " %s %lf %lf %lf\n",
			dummy, &d.v, &d.phi, &d.ack_time);

		lines.push_back(d);
	}

	fclose(f);
}


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
load_pointcloud(Data &d, PointCloud<PointXYZRGB>::Ptr cloud)
{
	FILE *f = fopen(d.cloud_path, "rb");

	double th, ds;
	double h_angle, v_angle;
	unsigned short distances[32];
	unsigned char intensities[32];
	double range;

	cloud->clear();

	th = 0;

	for(int i = 0; i < d.n_rays; i++)
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

	    	PointXYZRGB point = compute_point_from_velodyne(v_angle, h_angle, range, intensities[velodyne_ray_order[j]]);

	    	if (range > 70.  || (fabs(point.x) < 4. && fabs(point.y) < 2.)) // || point.z < -1.5 || point.z > 1.5)
	    		continue;

	    	ds = d.v * (i * TIME_SPENT_IN_EACH_SCAN);
	    	point.x += ds * cos(th);
	    	point.y += ds * sin(th);
	    	th = carmen_normalize_theta(ds * (tan(d.phi) / distance_between_front_and_rear_axles));

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
		printf("\nError: Use %s <sync_file> <output_dir>\n\n", argv[0]);
		exit(0);
	}

	vector<Data> lines;
	char str[256];

	load_data(argv[1], lines);

	sprintf(str, "rm -rf %s && mkdir %s", argv[2], argv[2]);
	printf("Command: %s\n", str);
	system(str);

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	for (int i = 0; i < lines.size(); i++)
	{
		load_pointcloud(lines[i], cloud);
		sprintf(str, "%s/%lf.ply", argv[2], lines[i].cloud_time);
		printf("%d of %ld - Saving pointcloud %s\n", i+1, lines.size(), str);
		pcl::io::savePLYFileBinary(str, *cloud);
	}

	return 0;
}

