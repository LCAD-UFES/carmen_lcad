
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <opencv/cv.hpp>
#include <cfloat>

#include "segmap_pose2d.h"
#include "segmap_util.h"
#include "segmap_car_config.h"
#include <boost/algorithm/string.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;


Mat
segmented_image_view(Mat &m)
{
	CityScapesColorMap color_map;
	Mat view(m.rows, m.cols, CV_8UC3);

	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			int cl = m.data[3 * (i * m.cols + j)];
			Scalar color;

			color = color_map.color(cl);

			view.data[3 * (i * view.cols + j)] = color[2];
			view.data[3 * (i * view.cols + j) + 1] = color[1];
			view.data[3 * (i * view.cols + j) + 2] = color[0];
		}
	}

	return view;
}


double
normalize_theta(double theta)
{
	double multiplier;

	if (theta >= -M_PI && theta < M_PI)
		return theta;

	multiplier = floor(theta / (2*M_PI));
	theta = theta - multiplier*2*M_PI;
	if (theta >= M_PI)
		theta -= 2*M_PI;
	if (theta < -M_PI)
		theta += 2*M_PI;

	return theta;
}


double
radians_to_degrees(double theta)
{
	return (theta * 180.0 / M_PI);
}


double
degrees_to_radians(double theta)
{
	return (theta * M_PI / 180.0);
}


Matrix<double, 4, 4>
pose6d_to_matrix(double x, double y, double z, double roll, double pitch, double yaw)
{
	Matrix<double, 3, 3> Rx, Ry, Rz, R;
	Matrix<double, 4, 4> T;

	double rx, ry, rz;

	rx = roll;
	ry = pitch;
	rz = yaw;

    Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
    Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
    R  = Rz * Ry * Rx;

    T << R(0, 0), R(0, 1), R(0, 2), x,
    	R(1, 0), R(1, 1), R(1, 2), y,
		R(2, 0), R(2, 1), R(2, 2), z,
		0, 0, 0, 1;

    return T;
}


Matrix<double, 4, 4> 
pose3d_to_matrix(double x, double y, double theta)
{
	return pose6d_to_matrix(x, y, 0., 0., 0., theta);
}


void
oxts2Mercartor(vector<vector<double>> &data, vector<Matrix<double, 4, 4>> &poses)
{
	double scale = cos(data[0][0] * M_PI / 180.);
	double er = 6378137;

	Matrix<double, 4, 4> p0;

	for (int i = 0; i < data.size(); i++)
	{
		double x, y, z;
		x = scale * data[i][1] * M_PI * er / 180;
		y = scale * er * log(tan((90 + data[i][0]) * M_PI / 360));
		z = data[i][2];

		double rx = data[i][3]; // roll
		double ry = data[i][4]; // pitch
		double rz = data[i][5]; // heading

		Matrix<double, 3, 3> Rx, Ry, Rz, R;
		Matrix<double, 4, 4> p;

		Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
		Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
		Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
		R  = Rz * Ry * Rx;

		p << R(0, 0),  R(0, 1),  R(0, 2), x,
				R(1, 0),  R(1, 1),  R(1, 2), y,
				R(2, 0),  R(2, 1),  R(2, 2), z,
				0, 0, 0, 1;

		if (i == 0)
			p0 = p;

		p = p0.inverse() * p;
		poses.push_back(p);
	}
}


vector<double>
read_vector(char *name)
{
	double d;
	vector<double> data;

	FILE *f = fopen(name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", name));

	while (!feof(f))
	{
		fscanf(f, "%lf\n", &d);
		data.push_back(d);
	}

	fclose(f);
	return data;
}


// debug
void
print_vector(vector<double> &v)
{
	for (int i = 0; i < v.size(); i++)
		printf("%.2lf ", v[i]);

	printf("\n");
}


int
argmax(double *v, int size)
{
	int p = 0;

	for (int i = 1; i < size; i++)
		if (v[i] > v[p])
			p = i;

	return p;
}


int
argmin(double *v, int size)
{
	int p = 0;

	for (int i = 1; i < size; i++)
		if (v[i] < v[p])
			p = i;

	return p;
}


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


double
dist2d(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}


void
ackerman_motion_model(double &x, double &y, double &th, double v, double phi, double dt)
{
	if (fabs(phi) > degrees_to_radians(80))
		exit(printf("Error phi = %lf\n", radians_to_degrees(phi)));

	double ds = dt * v;

	th += (ds / distance_between_front_and_rear_axles) * tan(phi);
	th = normalize_theta(th);
	x += ds * cos(th);
	y += ds * sin(th);
}


void
ackerman_motion_model(Pose2d &pose, double v, double phi, double dt)
{
	ackerman_motion_model(pose.x, pose.y, pose.th, v, phi, dt);
}


void
correct_point(Pose2d &correction,
		Matrix<double, 4, 4> vel2car,
		PointXYZRGB &point)
{
	Matrix<double, 4, 1> p, corrected;
	Matrix<double, 4, 4> correction_mat;

	p << point.x, point.y, point.z, 1.;
	correction_mat = Pose2d::to_matrix(correction);

	// move the point to the car reference frame, apply the correction due to car motion, 
	// and move back to the velodyne frame.
	corrected = vel2car.inverse() * correction_mat * vel2car * p;

	double m = corrected(3, 0);

	// how to handle points at infinity?
	if (m == 0)
	{
		printf("Warning: point projected to infinity: %lf %lf %lf -> %lf %lf %lf %lf correction: %lf %lf %lf\n",
				point.x, point.y, point.z,
				corrected(0, 0), corrected(1, 0), corrected(2, 0), corrected(3, 0),
				correction.x, correction.y, correction.th);

		m = 1.0;
	}

	point.x = corrected(0, 0) / m;
	point.y = corrected(1, 0) / m;
	point.z = corrected(2, 0) / m;
}

/*
void
transform_pointcloud(PointCloud<PointXYZRGB>::Ptr cloud,
		PointCloud<PointXYZRGB>::Ptr transformed_cloud,
		Pose2d &pose,
		Matrix<double, 4, 4> &vel2car,
		double v, double phi)
{
	Pose2d correction(0., 0., 0.);
	transformed_cloud->clear();

	Matrix<double, 4, 4> pose_t = Pose2d::to_matrix(pose);

	for (int j = 0; j < cloud->size(); j++)
	{
		PointXYZRGB point = PointXYZRGB(cloud->at(j));

		if (point.x < MAX_RANGE && point.y < MAX_RANGE&& point.z < MAX_RANGE)
		{
			correct_point(correction, vel2car, pose_t, point);
			transformed_cloud->push_back(point);
		}

		ackerman_motion_model(correction, v, phi, (TIME_SPENT_IN_EACH_SCAN / 32.));
	}
}
*/

FILE*
safe_fopen(const char *path, const char *mode)
{
	FILE *f = fopen(path, mode);

	if (f == NULL)
		exit(printf("fopen failed with path: '%s', and mode '%s'\n", path, mode));

	return f;
}


void
spherical2cartersian(double v_angle, double h_angle, double radius, 
										 double *x, double *y, double *z)
{
	double cos_rot_angle = cos(h_angle);
	double sin_rot_angle = sin(h_angle);

	double cos_vert_angle = cos(v_angle);
	double sin_vert_angle = sin(v_angle);

	double xy_distance = radius * cos_vert_angle;

	*x = (xy_distance * cos_rot_angle);
	*y = (xy_distance * sin_rot_angle);
	*z = (radius * sin_vert_angle);
}


std::vector<std::string>
string_split(std::string s, std::string pattern)
{
	vector<std::string> splitted, splitted_without_empties;

	boost::split(splitted, s, boost::is_any_of(pattern));

	for (int i = 0; i < splitted.size(); i++)
	{
		if (splitted[i].size() > 0)
			splitted_without_empties.push_back(splitted[i]);
	}

	return splitted_without_empties;
}


std::string
file_name_from_path(const char *path)
{
	vector<string> splitted = string_split(path, "/");
	return std::string(splitted[splitted.size() - 1]);
}


std::string
default_odom_calib_path(const char *log_path)
{
	std::string log_name = file_name_from_path(log_path);
	return (string("/dados/data2/data_") + log_name + string("/odom_calib.txt"));
}


std::string
default_fused_odom_path(const char *log_path)
{
	std::string log_name = file_name_from_path(log_path);
	return (string("/dados/data2/data_") + log_name + string("/fused_odom.txt"));
}


