
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <deque>
#include <string>
#include <random>
#include <iostream>
#include <Eigen/Geometry>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "libsegmap/segmap_car_config.h"
#include "libsegmap/segmap_grid_map.h"
#include "libsegmap/segmap_particle_filter.h"
#include "libsegmap/segmap_pose2d.h"
#include "libsegmap/segmap_util.h"


using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;


vector<double>
load_timestamps()
{
	vector<double> times;
	char *time_name = "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/oxts/timestamps.txt";

	int dummy;
	double t;
	FILE *f = fopen(time_name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", time_name));

	while(fscanf(f, "%d-%d-%d %d:%d:%lf", &dummy, &dummy, &dummy, &dummy, &dummy, &t) == 6)
		times.push_back(t);

	fclose(f);
	return times;
}


vector<vector<double>>
load_oxts(vector<double> &times)
{
	vector<vector<double>> data;

	char *dir = "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/oxts/data";
	char name[1024];

	for (int i = 0; i < times.size(); i++)
	{
		sprintf(name, "%s/%010d.txt", dir, i);
		data.push_back(read_vector(name));
	}

	return data;
}


void
print_poses(vector<Matrix<double, 4, 4>> &poses)
{
	Matrix<double, 4, 4> p;

	for (int i = 0; i < poses.size(); i++)
	{
		p = poses[i];

		printf("%.2lf %.2lf %.2lf\n",
				p(0, 3) / p(3, 3),
				p(1, 3) / p(3, 3),
				p(2, 3) / p(3, 3));
	}
}


void
read_pointcloud_kitti(int i, PointCloud<PointXYZRGB>::Ptr cloud)
{
	// pointers
	static int num;
	static float *data;
	static int first = 1;

	float *px = data+0;
	float *py = data+1;
	float *pz = data+2;
	float *pr = data+3;

	char name[1024];
	sprintf(name, "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/velodyne_points/data/%010d.bin", i);

	num = 1000000;

	if (first)
	{
		data = (float*) malloc(num * sizeof(float));
		first = 0;
	}

	printf("loading pointcloud '%s'\n", name);

	// load point cloud
	FILE *stream;
	stream = fopen(name, "rb");
	num = fread(data, sizeof(float), num, stream) / 4;
	fclose(stream);

	for (int i = 0; i < num; i++)
	{
		px += 4; py += 4; pz += 4; pr += 4;

		pcl::PointXYZRGB point;

		point.x = *px;
		point.y = *py;
		point.z = *pz;
		point.r = *pr;
		point.g = *pr;
		point.b = *pr;

		cloud->push_back(point);
	}
}


void
read_pointcloud_carmen(int i, PointCloud<PointXYZRGB>::Ptr cloud)
{
	char name[1024];
	sprintf(name, "/dados/data_20180112-2/data/velodyne/%010d.ply", i);
	pcl::io::loadPLYFile(name, *cloud);
}


void
load_fused_pointcloud_and_camera(int i, PointCloud<PointXYZRGB>::Ptr cloud)
{
	cloud->clear();

	PointCloud<PointXYZRGB>::Ptr raw_cloud(new PointCloud<PointXYZRGB>);
	//read_pointcloud_kitti(i, raw_cloud);
	read_pointcloud_carmen(i, raw_cloud);

	char name[1024];
	//sprintf(name, "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/image_02/data/%010d.png", i);
	//sprintf(name, "/dados/imgs_kitti/%010d_trainval.png", i);
	//sprintf(name, "/dados/data_20180112-2/data/trainfine/%010d.png", i);
	sprintf(name, "/dados/data_20180112-2/data/bb3/%010d.png", i);

	//Mat img(375, 1242, CV_8UC3);
	//Mat raw_img = imread(name);
	//cv::resize(raw_img, img, img.size());
	Mat img = imread(name);

	printf("loading image '%s'\n", name);

	Mat view = img.clone();

	int p, x, y;
	Matrix<double, 4, 1> P;
	Matrix<double, 3, 1> pixel;
	PointXYZRGB point;

	//Matrix<double, 3, 4> vel2cam = kitti_velodyne_to_cam();
	Matrix<double, 3, 4> vel2cam = carmen_velodyne_to_cam3(img.cols, img.rows);

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		point = raw_cloud->at(i);

		P << point.x, point.y, point.z, 1.;
		pixel = vel2cam * P;

		x = pixel(0, 0) / pixel(2, 0);
		y = pixel(1, 0) / pixel(2, 0);
		y = img.rows - y - 1;
		x = img.cols - x - 1;

		//cout << pose << endl << endl;
		//cout << P << endl << endl;
		//cout << Pt << endl << endl;
		//cout << "P3d:" << endl;
		//cout << P << endl;
		//cout << "pixel:" << endl;
		//cout << pixel << endl;
		//cout << "x-y: " << x << " " << y << endl;
		//cout << endl;

		//if (0)
		if (point.x > 7 && x >= 0 && x < img.cols && y >= 0 && y < img.rows)
		{
			pcl::PointXYZRGB point2;

			point2.x = P(0, 0) / P(3, 0);
			point2.y = P(1, 0) / P(3, 0);
			point2.z = P(2, 0) / P(3, 0);

			// colors
			p = 3 * (y * img.cols + x);
			point2.r = img.data[p + 2];
			point2.g = img.data[p + 1];
			point2.b = img.data[p + 0];

			circle(view, Point(x, y), 2, Scalar(0,0,255), -1);
			cloud->push_back(point2);
		}

		else if (0)
		//else if (1) //point.z < 0.)
		{
			point.r *= 3;
			point.g *= 3;
			point.b *= 3;
			cloud->push_back(point);
		}
	}

	imshow("img", view);
}


vector<pair<double, double>>
estimate_v(vector<Matrix<double, 4, 4>> &poses, vector<double> &ts)
{
	vector<pair<double, double>> vs;

	double lx = 0, ly = 0, lt = 0;

	for (int i = 0; i < poses.size(); i++)
	{
		double x = poses[i](0, 3) / poses[i](3, 3);
		double y = poses[i](1, 3) / poses[i](3, 3);

		double ds = sqrt(pow(x - lx, 2) + pow(y - ly, 2));
		double dt = ts[i] - lt;

		lx = x;
		ly = y;
		lt = ts[i];

		if (i > 0)
			vs.push_back(pair<double, double>(ds / dt, 0));

		if (i == 1)
			vs.push_back(pair<double, double>(ds / dt, 0));
	}

	return vs;
}


void
draw_rectangle(Mat &img,
		double x, double y, double theta,
		double height, double width, Scalar color,
		double x_origin, double y_origin, double pixels_by_meter)
{
/*
	vector<Point2f> vertices;
	vector<Point> vertices_px;

	vertices.push_back(Point2f(-width / 2., -height / 2.));
	vertices.push_back(Point2f(-width / 2., height / 2.));
	vertices.push_back(Point2f(width / 2., height / 2.));
	vertices.push_back(Point2f(width / 2., 0.));
	vertices.push_back(Point2f(0., 0.));
	vertices.push_back(Point2f(width / 2., 0));
	vertices.push_back(Point2f(width / 2., -height / 2.));

	double v_radius, v_angle;

	// transform vertices
	for (int i = 0; i < vertices.size(); i++)
	{
		v_radius = sqrt(pow(vertices[i].x, 2.) + pow(vertices[i].y, 2.));
		v_angle = atan2(vertices[i].y, vertices[i].x);

		vertices[i].x = v_radius * cos(v_angle + theta) + x;
		vertices[i].y = v_radius * sin(v_angle + theta) + y;

		Point p;
		p.x = (int) ((vertices[i].x - x_origin) * pixels_by_meter);
		p.y = (int) ((vertices[i].y - y_origin) * pixels_by_meter);

		vertices_px.push_back(p);
	}

	for (int i = 0; i < vertices_px.size(); i++)
	{
		if (i == vertices_px.size() - 1)
			line(img, vertices_px[i], vertices_px[0], color, 1);
		else
			line(img, vertices_px[i], vertices_px[i + 1], color, 1);
	}
*/
}


void
draw_poses(GridMap &map, Mat &map_img, vector<Matrix<double, 4, 4>> &poses)
{
/*
	Point pixel;
	int radius = (0.25 * map._pixels_by_m);
	Pose2d p;
	double shift_x, shift_y;

	for (int i = 0; i < poses.size(); i += 1)
	{
		p = Pose2d::from_matrix(poses[i]);

		shift_x = center_to_rear_axis * cos(p.th);
		shift_y = center_to_rear_axis * sin(p.th);

		pixel.x = (p.x - map._xo) * map._pixels_by_m;
		pixel.y = (p.y - map._yo) * map._pixels_by_m;
		circle(map_img, pixel, radius, Scalar(0, 255, 0), -1);

		draw_rectangle(map_img,
				p.x + shift_x,
				p.y + shift_y,
				p.th,
				car_width, car_length, Scalar(0, 255, 0),
				map._xo,
				map._yo,
				map._pixels_by_m);
	}
*/
}


void
draw_particle(Mat &map_img, Pose2d &p, GridMap &map, int radius, Scalar color)
{
/*
	Point pixel;

	pixel.x = (p.x - map._xo) * map._pixels_by_m;
	pixel.y = (p.y - map._yo) * map._pixels_by_m;
	circle(map_img, pixel, radius, color, 1);

	Point p2;
	p2.x = ((2.0 * cos(p.th) + p.x) - map._xo) * map._pixels_by_m;
	p2.y = ((2.0 * sin(p.th) + p.y) - map._yo) * map._pixels_by_m;

	line(map_img, pixel, p2, color, 1);
*/
}


void
view(ParticleFilter &pf, GridMap &map, vector<Matrix<double, 4, 4>> &poses, Pose2d current_pose,
	PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr transformed_cloud)
{
	/*
	int i, j;
	Pose2d p;
	Point pixel;
	double shift_x, shift_y;

	Mat bkp;
	Pose2d mean = pf.mean();
	Pose2d mode = pf.mode();

	if (cloud != NULL)
	{
		bkp = map.to_image();
		transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(mode));
		for(int i = 0; i < transformed_cloud->size(); i++)
		{
			transformed_cloud->at(i).r = 255;
			transformed_cloud->at(i).g = 0;
			transformed_cloud->at(i).b = 0;
			map.add_point(transformed_cloud->at(i));
		}
	}

	Mat map_img = map.to_image();

	if (cloud != NULL)
		bkp.copyTo(*map._map);

	int radius = (0.25 * map._pixels_by_m);
	*/

	/*
	for (i = 0; i < poses.size(); i += 1)
	{
		p = Pose2d::from_matrix(poses[i]);

		shift_x = center_to_rear_axis * cos(p.th);
		shift_y = center_to_rear_axis * sin(p.th);

		pixel.x = (p.x - map._xo) * map._pixels_by_m;
		pixel.y = (p.y - map._yo) * map._pixels_by_m;
		circle(map_img, pixel, radius, Scalar(0, 255, 0), -1);

		// GPS error area
		circle(map_img, pixel, sqrt(pf._gps_var_x) * map._pixels_by_m, Scalar(0, 255, 0), 1);

		draw_rectangle(map_img,
				p.x + shift_x,
				p.y + shift_y,
				p.th,
				car_width, car_length, Scalar(0, 255, 0),
				map._xo,
				map._yo,
				map._pixels_by_m);
	}
	*/

	/*
	for (i = 0; i < pf._n; i++)
		draw_particle(map_img, pf._p[i], map, radius, Scalar(0, 0, 255));

	draw_particle(map_img, mean, map, radius, Scalar(0, 255, 255));
	draw_particle(map_img, mode, map, radius, Scalar(255, 0, 0));
	draw_particle(map_img, current_pose, map, radius, Scalar(0, 0, 0));

	//Mat resized_map(1000, 1000, CV_8UC3);
	//resize(map_img, resized_map, resized_map.size());
	imshow("map", map_img);
	waitKey(-1);
	*/
}


void
create_map(GridMap &map, vector<Matrix<double, 4, 4>> &poses, PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr transformed_cloud)
{
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(.5, .5, .5);
	viewer.removeAllPointClouds();
	viewer.addCoordinateSystem(10);

	Matrix<double, 4, 4> vel2car = carmen_vel2car();
	Matrix<double, 4, 4> car2world;

	deque<string> cloud_names;
	int step = 1;

	for (int i = 0; i < poses.size(); i += 1)
	{
		load_fused_pointcloud_and_camera(i, cloud);

		car2world = poses[i] * vel2car;
		transformPointCloud(*cloud, *transformed_cloud, car2world);
		//transformed_cloud = cloud;

		map.reload(poses[i](0, 3), poses[i](1, 3));
		printf("car pose: %lf %lf\n", poses[i](0, 3), poses[i](1, 3));

		for (int i = 0; i < transformed_cloud->size(); i++)
			map.add_point(transformed_cloud->at(i));

		char *cloud_name = (char *) calloc (32, sizeof(char));
		sprintf(cloud_name, "cloud%d", i);
		//viewer.removeAllPointClouds();
		viewer.addPointCloud(transformed_cloud, cloud_name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
		cloud_names.push_back(cloud_name);

		//if (cloud_names.size() >= 10)
		//{
		//	viewer.removePointCloud(cloud_names[0]);
		//	cloud_names.pop_front();
		//}

		Mat map_img = map.to_image().clone();
		draw_poses(map, map_img, poses);
		imshow("map", map_img);

		char c = ' ';
		while (1)
		{
			viewer.spinOnce();
			c = waitKey(5);

			if (c == 's')
				step = !step;
			if (!step || (step && c == 'n'))
				break;
			if (c == 'r')
			{
				printf("Reinitializing\n");
				i = 0;
			}
		}
	}

	waitKey(-1);
}


void
run_particle_filter(ParticleFilter &pf, GridMap &map, vector<Matrix<double, 4, 4>> &poses,
		vector<pair<double, double>> &odom, vector<double> &times, PointCloud<PointXYZRGB>::Ptr cloud,
		PointCloud<PointXYZRGB>::Ptr transformed_cloud)
{
	while (1)
	{
		pf.seed(time(NULL));
		pf.reset(0., 0., 0.);

		printf("Initial particles\n");
		view(pf, map, poses, Pose2d::from_matrix(poses[0]), NULL, NULL);

		for (int i = 1; i < times.size(); i++)
		{
			printf("Step %d of %ld\n", i+1, times.size());
			Pose2d gps = Pose2d::from_matrix(poses[i]); // TODO: add noise

			printf("Prediction\n");
			pf.predict(odom[i].first, odom[i].second, times[i] - times[i-1]);
			view(pf, map, poses, gps, NULL, NULL);

			//if (i % 4 == 0 && i > 0)
			//if (i > 16)
			if (1)
			{
				printf("Correction\n");
				load_fused_pointcloud_and_camera(i, cloud);
				pf.correct(gps, cloud, map, transformed_cloud);

				Pose2d mean = pf.mean();
				Pose2d mode = pf.mode();

				printf("True pose: %.2lf %.2lf %.2lf\n", gps.x, gps.y, radians_to_degrees(gps.th));
				printf("PF Mean: %.2lf %.2lf %.2lf Error: %lf\n", mean.x, mean.y, radians_to_degrees(mean.th),
						sqrt(pow(gps.x - mean.x, 2) + pow(gps.y - mean.y, 2)));
				printf("PF Mode: %.2lf %.2lf %.2lf Error: %lf\n", mode.x, mode.y, radians_to_degrees(mode.th),
						sqrt(pow(gps.x - mode.x, 2) + pow(gps.y - mode.y, 2)));

				view(pf, map, poses, gps, cloud, transformed_cloud);
			}
		}
	}
}


void
load_carmen_data(vector<double> &times,
		vector<Matrix<double, 4, 4>> &poses,
		vector<pair<double, double>> &odom)
{
	FILE *f = fopen("/dados/data_20180112-2/data/poses.txt", "r");

	char dummy[128];
	double x, y, th, t, v, phi;
	double x0, y0;
	int first = 1;

	double ds, lt, dt, odom_x, odom_y, odom_th;

	odom_x = odom_y = odom_th = 0.;

	while (!feof(f))
	{
		fscanf(f, "\n%s %lf %lf %lf %lf %s %lf %lf %s\n",
				dummy, &x, &y, &th, &t, dummy, &v, &phi, dummy);

		if (first)
		{
			x0 = x;
			y0 = y;
			first = 0;
		}
		else
		{
			dt = t - lt;
			ds = v * 1.006842 * dt;
			odom_x += ds * cos(odom_th);
			odom_y += ds * sin(odom_th);
			odom_th += ds * tan(phi * 0.861957 - 0.002372) / distance_between_front_and_rear_axles;
			odom_th = normalize_theta(odom_th);
		}
		lt = t;

		Pose2d pose(odom_x, odom_y, odom_th);
		// Pose2d pose(x - x0, y - y0, normalize_theta(th));

		poses.push_back(Pose2d::to_matrix(pose));
		times.push_back(t);
		odom.push_back(pair<double, double>(v, phi));
	}

	Matrix<double, 4, 4> p0_inv = Matrix<double, 4, 4>(poses[0]).inverse();

	for (int i = 0; i < poses.size(); i++)
	{
		poses[i] = p0_inv * poses[i];
		Pose2d p = Pose2d::from_matrix(poses[i]);
		p.y = -p.y;
		p.th = normalize_theta(-p.th);
		poses[i] = Pose2d::to_matrix(p);
	}

	fclose(f);
}


int
main()
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);

	ParticleFilter pf(50, 0.5, 0.5, degrees_to_radians(10),
			0.2, degrees_to_radians(1),
			0.1, 0.1, degrees_to_radians(2),
			25.0, 25.0, degrees_to_radians(100),
			100., 100., 100.);

	//GridMap map(-20, -2, 100, 200, 0.2);
	system("rm -rf /dados/maps/maps_20180112-2/*");
	GridMap map("/dados/maps/maps_20180112-2/", 75., 75., 0.4, GridMapTile::TYPE_VISUAL);

	// KITTI
	/*
	vector<double> times = load_timestamps();
	vector<vector<double>> data = load_oxts(times);
	vector<Matrix<double, 4, 4>> poses = oxts2Mercartor(data);
	vector<pair<double, double>> odom = estimate_v(poses, times); // assumes for simplicity that phi is zero.
	*/

	// Carmen
	vector<double> times;
	vector<Matrix<double, 4, 4>> poses;
	vector<pair<double, double>> odom;
	load_carmen_data(times, poses, odom);

	create_map(map, poses, cloud, transformed_cloud);
	run_particle_filter(pf, map, poses, odom, times, cloud, transformed_cloud);

	printf("Done\n");
	return 0;
}


