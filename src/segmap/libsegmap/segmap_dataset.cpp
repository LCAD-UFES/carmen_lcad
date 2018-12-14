
#include <opencv/cv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include "segmap_pose2d.h"
#include "segmap_util.h"
#include "segmap_dataset.h"
#include "segmap_viewer.h"
#include <pcl/visualization/pcl_visualizer.h>


using namespace cv;
using namespace pcl;

//pcl::visualization::PCLVisualizer *myviewer = new pcl::visualization::PCLVisualizer("CloudViewer2");

void
DatasetInterface::load_fused_pointcloud_and_camera(int i, PointCloud<PointXYZRGB>::Ptr cloud, int view)
{
	int p, x, y;
	double range;
	Matrix<double, 3, 1> pixel;
	PointXYZRGB point;
	PointCloud<PointXYZRGB>::Ptr raw_cloud(new PointCloud<PointXYZRGB>);

	cloud->clear();

	load_pointcloud(i, raw_cloud);
	Mat img = load_image(i);

	Mat viewer_img;

	if (view)
	{
		if (_use_segmented)
			viewer_img = segmented_image_view(img);
		else
			viewer_img = img.clone();
	}

	int top_limit = (50. / 480.) * img.rows;
	int bottom_limit = img.rows - (110. / 480.) * img.rows;

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		point = raw_cloud->at(i);
		range = sqrt(pow(point.x, 2) + pow(point.y, 2));
    	pixel = transform_vel2cam(point);

		x = pixel(0, 0) / pixel(2, 0);
		y = pixel(1, 0) / pixel(2, 0);

		if (range < 4.0 || range > 70. || (fabs(point.x) < 6.0 && fabs(point.y) < 4.))
			point.x = point.y = point.z = DBL_MAX;

		// to use fused camera and velodyne
		// if (0)
		if (point.x > 0 && x >= 0 && x < img.cols && y >= 0 && y < img.rows && (!_use_segmented || (y > top_limit && y < bottom_limit)))
		{
			pcl::PointXYZRGB point2;

			point2.x = point.x;
			point2.y = point.y;
			point2.z = point.z;

			// colors
			p = 3 * (y * img.cols + x);
			point2.r = img.data[p + 2];
			point2.g = img.data[p + 1];
			point2.b = img.data[p + 0];

			if (view)
				circle(viewer_img, Point(x, y), 2, Scalar(0,0,255), -1);

			cloud->push_back(point2);
		}

		// to use remission
		else if (0)
		// else if (1) //point.z < 0.)
		{
			point.r *= 3;
			point.g *= 3;
			point.b *= 3;
			cloud->push_back(point);
		}
	}

    /*
	myviewer->setBackgroundColor(1, 1, 1);
	myviewer->removeAllPointClouds();
	myviewer->addCoordinateSystem(2);
	myviewer->addPointCloud(raw_cloud, "raw_cloud");
	myviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_cloud");
	myviewer->spinOnce();
    */

	if (view)
	{
		if (_use_segmented)
		{
			sprintf(_name, "%s/bb3/%lf-r.png", _path.c_str(), data[i].image_time);
			Mat aux_img = imread(_name);
			Mat view_copy = viewer_img.clone();
			cv::hconcat(aux_img, view_copy, viewer_img);
			flip(viewer_img, viewer_img, +1);
		}

		double resize_rate = 1.0; //640. / (double) viewer_img.cols;
		int height = resize_rate * viewer_img.rows;
		int width = resize_rate * viewer_img.cols;

		Mat resized(480, 640, CV_8UC3);
		resize(viewer_img, resized, Size(width, height));
		imshow("cam_vel_fused", resized);
	}
}


void
DatasetCarmen::_init_vel2cam_transform(int image_height, int image_width)
{
	Matrix<double, 3, 4> projection;
	Matrix<double, 4, 4> velodyne2board;
	Matrix<double, 4, 4> cam2board;

	velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	cam2board = pose6d_to_matrix(0.245, -0.04, 0.210, -0.017453, 0.026037, -0.023562);

	// This is a rotation to change the ref. frame from x: forward, y: left, z: up
	// to x: right, y: down, z: forward.
	Matrix<double, 4, 4> R;
	R = pose6d_to_matrix(0., 0., 0., 0., M_PI/2., M_PI/2);

	double fx_factor = 0.764749;
	double fy_factor = 1.01966;
	double cu_factor = 0.505423;
	double cv_factor = 0.493814;
	double pixel_size = 0.00000375;

    double fx_meters = fx_factor * image_width * pixel_size;
    double fy_meters = fy_factor * image_height * pixel_size;

    double cu = cu_factor * image_width;
    double cv = cv_factor * image_height;

    // see http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
    // Note: Storing cu and cv in the 3rd column instead of the 4th is a trick.
    // To compute the pixel coordinates we divide the first two
    // dimensions of the point in homogeneous coordinates by the third one (which is Z).
	projection << fx_meters / pixel_size, 0, cu, 0,
				  0, fy_meters / pixel_size, cv, 0,
				  0, 0, 1, 0.;

	_vel2cam = projection * R * cam2board.inverse() * velodyne2board;
}


void
DatasetCarmen::_init_vel2car_transform()
{
	Matrix<double, 4, 4> velodyne2board;
	Matrix<double, 4, 4> board2car;

	velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	board2car = pose6d_to_matrix(0.572, 0, 1.394, 0.0, 0.0122173048, 0.0);

	_vel2car = board2car * velodyne2board;
}


DatasetCarmen::DatasetCarmen(string path, int use_segmented) :
	DatasetInterface(path, use_segmented)
{
	load_data();
	_unknown_class = CityScapesColorMap().n_classes + 1;
	_init_vel2car_transform();
	_init_vel2cam_transform(image_height, image_width);
}


Mat
DatasetCarmen::load_image(int i)
{
	if (_use_segmented)
		sprintf(_name, "%s/semantic/%lf-r.png", _path.c_str(), data[i].image_time);
	else
		//sprintf(_name, "%s/bb3/%010d.png", _path.c_str(), i);
		sprintf(_name, "%s/bb3/%lf-r.png", _path.c_str(), data[i].image_time);

	Mat raw_img = imread(_name);

    if (raw_img.data == 0 || raw_img.rows == 0 || raw_img.cols == 0)
    	exit(printf("Error: Image '%s' not found.\n", _name));

	Mat resized;

	if (_use_segmented)
	{
		int height = (int) (0.75 * raw_img.cols);
		resized = Mat::ones(height, raw_img.cols, CV_8UC3) * 20;
		int top_limit = (50. / 480.) * height;
		int bottom_limit = height - (110. / 480.) * height;
		raw_img.copyTo(resized(Rect(0, top_limit, raw_img.cols, bottom_limit - top_limit)));
	}
	else
		resized = raw_img;

	//Mat img = Mat::ones(_image_height, _image_width, CV_8UC3) * _unknown_class;
	//Mat roi = img(Rect(0, (int) (0.1 * _image_width), _image_width, (int) (_image_width / 2.)));
	//resize(resized, roi, roi.size());
	//return img;
	return resized;
}


void
DatasetCarmen::load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud)
{
	int success;

	sprintf(_name, "%s/velodyne/%lf.ply", _path.c_str(), data[i].velodyne_time);
	//sprintf(_name, "%s/velodyne/%010d.ply", _path.c_str(), i);

	success = pcl::io::loadPLYFile(_name, *cloud);

	if (success < 0 || cloud->size() == 0)
		exit(printf("Cloud %s not found.\n", _name));
}


Matrix<double, 3, 1>
DatasetCarmen::transform_vel2cam(PointXYZRGB &p)
{
    Matrix<double, 4, 1> p_velodyne;
    Matrix<double, 3, 1> p_img;

    p_velodyne << p.x, p.y, p.z, 1.;
    p_img = _vel2cam * p_velodyne;

	return p_img;
}


Matrix<double, 4, 4>
DatasetCarmen::transform_vel2car()
{
	return _vel2car;
}


void
DatasetCarmen::load_data()
{
	string data_file;

	data_file = _path + "/sync.txt";
	FILE *f = fopen(data_file.c_str(), "r");

	if (f == NULL)
		exit(printf("Error: file '%s' not found.\n", data_file.c_str()));

	int n_read;
	char dummy[256];
	double offset_x = 7757677.517731;
	double offset_y = -363602.117405;

	while (!feof(f))
	{
		DataSample sample;

		n_read = fscanf(f, "\n%s %s %s %lf ", dummy, dummy, dummy, &sample.velodyne_time);
		if (n_read != 4) continue;

		n_read = fscanf(f, " %s %s %d %d %s %s %lf ", dummy, dummy, &image_width, &image_height, dummy, dummy, &sample.image_time);
		if (n_read != 7) continue;

		n_read = fscanf(f, " %s %s %lf %lf %d %lf ", dummy, dummy, &sample.gps.x, &sample.gps.y, &sample.gps_quality, &sample.gps_time);
		if (n_read != 6) continue;

		n_read = fscanf(f, " %s %s %lf %s %s ", dummy, dummy, &sample.gps.th, dummy, dummy);
		if (n_read != 5) continue;

		n_read = fscanf(f, " %s %lf %lf %lf\n", dummy, &sample.v, &sample.phi, &sample.odom_time);
		if (n_read != 4) continue;

		sample.gps.x -= offset_x;
		sample.gps.y -= offset_y;
		sample.gps.y = -sample.gps.y;
		sample.gps.th = normalize_theta(-sample.gps.th);
		sample.phi = normalize_theta(-sample.phi);

		data.push_back(sample);
	}

	fclose(f);

	data_file = _path + "/optimized.txt";
	f = fopen(data_file.c_str(), "r");

	if (f == NULL)
		exit(printf("File '%s' not found.\n", data_file.c_str()));

	for (int i = 0; i < data.size(); i++)
	{
		fscanf(f, "\n%s %lf %lf %lf %s %s %s %s %s\n",
				dummy, &data[i].pose.x, &data[i].pose.y, &data[i].pose.th,
				dummy, dummy, dummy, dummy, dummy);

		data[i].pose.x -= offset_x;
		data[i].pose.y -= offset_y;
		data[i].pose.y = -data[i].pose.y;
		data[i].pose.th = normalize_theta(-data[i].pose.th);

//		printf("%lf %lf %lf %lf %lf %lf\n",
//				data[i].gps.x, data[i].gps.y, data[i].gps.th,
//				data[i].pose.x, data[i].pose.y, data[i].pose.th);
	}

	fclose(f);
}


void
DatasetKitti::_init_vel2cam_transform()
{
	Matrix<double, 4, 4> R00;
	Matrix<double, 3, 4> P02;
	Matrix<double, 4, 4> Rt;

	R00 << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0.,
			-9.869795e-03, 9.999421e-01, -4.278459e-03, 0.,
			7.402527e-03, 4.351614e-03, 9.999631e-01, 0.,
			0., 0., 0., 1.;

	Rt << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
			1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
			9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
			0., 0., 0., 1.;

	P02 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
			0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
			0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03;

	_vel2cam = P02 * R00 * Rt;
}


Mat
DatasetKitti::load_image(int i)
{
	if (_use_segmented)
		sprintf(_name, "/dados/experiments/2018_segmap/results_semantic_segmentation/imgs_kitti/%010d_trainval.png", i);
	else
		sprintf(_name, "%s/image_02/data/%010d.png", _path.c_str(), i);

    Mat img = imread(_name);
    Mat resized;

    if (img.data == 0 || img.rows == 0 || img.cols == 0)
    	exit(printf("Error: Image '%s' not found.\n", _name));

    if (img.rows != 375)
    {
        resized = Mat(375, 1242, CV_8UC3);
        resize(img, resized, resized.size());
    }
    else
        resized = img;

	return resized;
}


void
DatasetKitti::load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud)
{
	// pointers
	static int num;
	static float *data;
	static int first = 1;

	sprintf(_name, "%s/velodyne_points/data/%010d.bin", _path.c_str(), i);

	num = 1000000;

	if (first)
	{
		data = (float*) malloc(num * sizeof(float));
		first = 0;
	}

	// load point cloud
	FILE *stream;
	stream = fopen(_name, "rb");
	num = fread(data, sizeof(float), num, stream) / 4;
	fclose(stream);

	float *px = data+0;
	float *py = data+1;
	float *pz = data+2;
	float *pr = data+3;

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


Matrix<double, 3, 1>
DatasetKitti::transform_vel2cam(PointXYZRGB &p)
{
    Matrix<double, 4, 1> p_velodyne;
    p_velodyne << p.x, p.y, p.z, 1.;
	return _vel2cam * p_velodyne;
}


Matrix<double, 4, 4>
DatasetKitti::transform_vel2car()
{
	// TODO.
	return pose6d_to_matrix(0, 0, 0, 0, 0, 0);
}


void
DatasetKitti::_load_timestamps(vector<double> &times)
{
	sprintf(_name, "%s/oxts/timestamps.txt", _path.c_str());

	int dummy;
	double t;
	FILE *f = fopen(_name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", _name));

	while(fscanf(f, "%d-%d-%d %d:%d:%lf", &dummy, &dummy, &dummy, &dummy, &dummy, &t) == 6)
		times.push_back(t);

	fclose(f);
}


void
DatasetKitti::_load_oxts(vector<double> &times, vector<vector<double>> &data)
{
	for (int i = 0; i < times.size(); i++)
	{
		sprintf(_name, "%s/oxts/data/%010d.txt", _path.c_str(), i);
		data.push_back(read_vector(_name));
	}
}


void
DatasetKitti::_estimate_v(vector<Matrix<double, 4, 4>> &poses, vector<double> &ts, vector<pair<double, double>> &odom)
{
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
			odom.push_back(pair<double, double>(ds / dt, 0));

		// add the same odom to account for the fact that we don't know the
		// values for the first step.
		if (i == 1)
			odom.push_back(pair<double, double>(ds / dt, 0));
	}
}


void
DatasetKitti::load_data()
{
	// TODO.
//    vector<vector<double>> data;
//
//	_load_timestamps(times);
//	_load_oxts(times, data);
//	oxts2Mercartor(data, poses);
//
//	for (int i = 0; i < poses.size(); i++)
//	{
//		_gps.push_back(Pose2d::from_matrix(poses[i]));
//		_gps_quality.push_back(1);
//	}
//
//	_estimate_v(poses, times, odom);
}

