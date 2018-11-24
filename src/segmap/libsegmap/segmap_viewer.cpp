
#include <vector>
#include <opencv/cv.hpp>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "segmap_car_config.h"
#include "segmap_viewer.h"
#include "segmap_grid_map.h"
#include "segmap_pose2d.h"
#include "segmap_util.h"
#include "segmap_particle_filter.h"

using namespace Eigen;
using namespace pcl;
using namespace std;
using namespace cv;


void
draw_poses(GridMap &map, Mat &map_img, vector<Matrix<double, 4, 4>> &poses)
{
	Point pixel;
	int radius = (0.25 * map.pixels_by_m);
	Pose2d p;
	double shift_x, shift_y;

	for (int i = 0; i < poses.size(); i += 1)
	{
		p = Pose2d::from_matrix(poses[i]);

		shift_x = center_to_rear_axis * cos(p.th);
		shift_y = center_to_rear_axis * sin(p.th);

		pixel.x = (p.x - map.xo) * map.pixels_by_m;
		pixel.y = (p.y - map.yo) * map.pixels_by_m;
		circle(map_img, pixel, radius, Scalar(0, 255, 0), -1);

		draw_rectangle(map_img,
				p.x + shift_x,
				p.y + shift_y,
				p.th,
				car_width, car_length, Scalar(0, 255, 0),
				map.xo,
				map.yo,
				map.pixels_by_m);
	}
}


void
draw_particle(Mat &map_img, Pose2d &p, GridMap &map, Scalar color)
{
	int radius;
	Point pixel;

	radius = 0.25 * map.pixels_by_m;

	pixel.x = (p.x - map.xo) * map.pixels_by_m;
	pixel.y = (p.y - map.yo) * map.pixels_by_m;
	circle(map_img, pixel, radius, color, 1);

	Point p2;
	p2.x = ((2.0 * cos(p.th) + p.x) - map.xo) * map.pixels_by_m;
	p2.y = ((2.0 * sin(p.th) + p.y) - map.yo) * map.pixels_by_m;

	line(map_img, pixel, p2, color, 1);
}


void
draw_pointcloud(Mat &m, PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map, int highlight_points)
{
	int px, py;
	unsigned char r, g, b;
	PointXYZRGB point;

	for(int i = 0; i < transformed_cloud->size(); i++)
	{
		point = transformed_cloud->at(i);

		if (highlight_points)
		{
			r = 255;
			g = b = 0;
		}
		else
		{
			r = point.r;
			g = point.g;
			b = point.b;
		}

		px = (point.x - map.xo) * map.pixels_by_m;
		py = (point.y - map.yo) * map.pixels_by_m;

		circle(m, Point(px, py), 2, Scalar(b, g, r), -1);
	}
}


//void
//crop_and_resize_map(double viewer_zoom, Mat &img, GridMap &map, Mat &resized, Pose2d car_pose, double *pixel_by_m, double *origin_x, double origin_y)
//{
//	int resized_width = img.cols - viewer_zoom * img.cols;
//	int resized_height = img.rows - viewer_zoom * img.rows;
//
//	*origin_x = car_pose -
//
//	Rect roi;
//
//	roi.x = (viewer_zoom / 2.) * map_img.cols;
//	roi.y = (viewer_zoom / 2.) * map_img.rows;
//	roi.width = map_img.cols - (viewer_zoom / 2.) * map_img.cols;
//	roi.height = map_img.rows - (viewer_zoom / 2.) * map_img.rows;
//
//	double mult = (double) 800. / (double) map_img.rows;
//
//	Mat resized_map((int) (map_img.rows * mult), (int) (map_img.cols * mult), CV_8UC3);
//	resize(map_img(roi), resized_map, resized_map.size());
//
//}


void
view(ParticleFilter &pf, GridMap &map, vector<Matrix<double, 4, 4>> &poses, Pose2d current_pose,
	PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr transformed_cloud)
{
	static int step = 1;

	char c;
	int i, j;
	Pose2d p;
	Point pixel;
	double shift_x, shift_y;

	Pose2d mean = pf.mean();
	Pose2d mode = pf.mode();

	Mat map_img = map.to_image();

	if (cloud != NULL)
	{
		transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(mode));
		draw_pointcloud(map_img, transformed_cloud, map);
	}

	draw_poses(map, map_img, poses);

	for (i = 0; i < pf._n; i++)
		draw_particle(map_img, pf._p[i], map, Scalar(0, 0, 255));

	draw_particle(map_img, mean, map, Scalar(0, 255, 255));
	draw_particle(map_img, mode, map, Scalar(255, 0, 0));
	draw_particle(map_img, current_pose, map, Scalar(0, 0, 0));

	double mult = (double) 800. / (double) map_img.rows;
	Mat resized_map((int) (map_img.rows * mult), (int) (map_img.cols * mult), CV_8UC3);
	resize(map_img, resized_map, resized_map.size());
	imshow("viewer", resized_map);
	//imshow("viewer", map_img);

	c = waitKey(step ? -1 : 1);
	if (c == 'S' || c == 's')
		step = !step;
}


