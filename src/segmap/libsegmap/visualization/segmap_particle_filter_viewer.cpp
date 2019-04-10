
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter_viewer.h>
#include <carmen/segmap_definitions.h>
#include <pcl/common/transforms.h>

using namespace Eigen;
using namespace pcl;
using namespace std;
using namespace cv;



void
draw_rectangle(Mat &img,
							 double x, double y, double theta,
							 double height, double width, Scalar color,
							 double x_origin, double y_origin, double pixels_by_meter)
{
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
}


void
draw_pose(GridMap &map, Mat &map_img, Pose2d &p, Scalar color)
{
	int radius = (0.25 * map.pixels_by_m);
	double shift_x, shift_y;
	Point pixel;

	shift_x = center_to_rear_axis * cos(p.th);
	shift_y = center_to_rear_axis * sin(p.th);

	pixel.x = (p.x - map.xo) * map.pixels_by_m;
	pixel.y = (p.y - map.yo) * map.pixels_by_m;
	circle(map_img, pixel, radius, color, -1);

	draw_rectangle(map_img,
								 p.x + shift_x,
								 p.y + shift_y,
								 p.th,
								 car_width, car_length, color,
								 map.xo,
								 map.yo,
								 map.pixels_by_m);
}


void
draw_pose(GridMap &map, Mat &map_img, Matrix<double, 4, 4> &pose, Scalar color)
{
	Pose2d p = Pose2d::from_matrix(pose);
	draw_pose(map, map_img, p, color);
}


void
draw_poses(GridMap &map, Mat &map_img, vector<Matrix<double, 4, 4>> &poses, Scalar color)
{
	for (int i = 0; i < poses.size(); i += 1)
		draw_pose(map, map_img, poses[i], color);
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
draw_pointcloud(Mat &m, PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map, int paint_points, Scalar color)
{
	int px, py;
	unsigned char r, g, b;
	PointXYZRGB point;

	for(int i = 0; i < transformed_cloud->size(); i++)
	{
		point = transformed_cloud->at(i);

		if (paint_points)
		{
			r = color[2];
			g = color[1];
			b = color[0];
		}
		else
		{
			r = point.r;
			g = point.g;
			b = point.b;
		}

		px = (point.x - map.xo) * map.pixels_by_m;
		py = (point.y - map.yo) * map.pixels_by_m;


		if (px >= 0 && px < m.cols && py >= 0 && py < m.rows)
		{
			m.data[3 * (py * m.cols + px)] = b;
			m.data[3 * (py * m.cols + px) + 1] = g;
			m.data[3 * (py * m.cols + px) + 2] = r;
			//circle(m, Point(px, py), 1, Scalar(b, g, r), 1);
		}
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


Mat
pf_view(ParticleFilter &pf, GridMap &map,
		 Pose2d current_pose,
		 Pose2d pf_pose,
		 PointCloud<PointXYZRGB>::Ptr cloud,
		 int draw_particles)
{
	Point pixel;

	cv::Mat map_img = map.to_image();

	if (cloud != NULL)
	{
		PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);

		transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(current_pose));
		draw_pointcloud(map_img, transformed_cloud, map, 1, Scalar(0, 255, 0));
		/*
		for (int i = 0; i < transformed_cloud->size(); i++)
			transformed_cloud->at(i).g = 255;
		draw_pointcloud(map_img, transformed_cloud, map, 1);
		*/

		transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(pf_pose));

		// debug:
		for (int i = 0; i < transformed_cloud->size(); i++)
		{
			if (fabs(transformed_cloud->at(i).x - pf_pose.x) > 200
					|| fabs(transformed_cloud->at(i).y - pf_pose.y) > 200)
			{
				printf("i: %d Point: %lf %lf Transformed: %lf %lf pf_pose: %lf %lf %lf\n",
							 i, cloud->at(i).x, cloud->at(i).y,
							 transformed_cloud->at(i).x, transformed_cloud->at(i).y,
							 pf_pose.x, pf_pose.y, pf_pose.th
				);
			}
		}

		draw_pointcloud(map_img, transformed_cloud, map, 1, Scalar(0, 0, 255));
		/*
		for (int i = 0; i < transformed_cloud->size(); i++)
			transformed_cloud->at(i).r = 255;
		draw_pointcloud(map_img, transformed_cloud, map, 1);
		*/
	}

	if (draw_particles)
	{
		for (int i = 0; i < pf._n; i++)
			draw_particle(map_img, pf._p[i], map, Scalar(255, 255, 255));
	}

	draw_pose(map, map_img, pf_pose, Scalar(0, 0, 255));
	draw_pose(map, map_img, current_pose, Scalar(0, 255, 0));

	return map_img;
}


void
colorize_cloud_according_to_segmentation(PointCloud<PointXYZRGB>::Ptr cloud)
{
	CityScapesColorMap colormap;
	for (int i = 0; i < cloud->size(); i++)
	{
		Scalar color = colormap.color(cloud->at(i).r);
		cloud->at(i).r = color[0];
		cloud->at(i).g = color[1];
		cloud->at(i).b = color[2];
	}
}
