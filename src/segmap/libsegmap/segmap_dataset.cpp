
#include <carmen/carmen.h>
#include <carmen/Gdc_Coord_3d.h>
#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_To_Utm_Converter.h>

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


double velodyne_vertical_angles[32] =
{
		-30.6700000, -29.3300000, -28.0000000, -26.6700000, -25.3300000, -24.0000000, -22.6700000, -21.3300000,
		-20.0000000, -18.6700000, -17.3300000, -16.0000000, -14.6700000, -13.3300000, -12.0000000, -10.6700000,
		-9.3299999, -8.0000000, -6.6700001, -5.3299999, -4.0000000, -2.6700001, -1.3300000, 0.0000000, 1.3300000,
		2.6700001, 4.0000000, 5.3299999, 6.6700001, 8.0000000, 9.3299999, 10.6700000
};

int velodyne_ray_order[32] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};


void
DatasetInterface::load_fused_pointcloud_and_camera(int i, PointCloud<PointXYZRGB>::Ptr cloud, double v, double phi, int view, Mat *output_img_view)
{
	int p, x, y;
	double range;
	Matrix<double, 3, 1> pixel;
	PointXYZRGB point;
	PointCloud<PointXYZRGB>::Ptr raw_cloud(new PointCloud<PointXYZRGB>);

	cloud->clear();
	load_pointcloud(i, raw_cloud, v, phi);
    
	Mat img = load_image(i);
	Mat viewer_img;

	if (view)
	{
		if (_use_segmented)
		{
			viewer_img = segmented_image_view(img);
		}
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

        //printf("pixel %lf %lf %lf\n", pixel(0, 0), pixel(1, 0), pixel(2, 0));

		x = pixel(0, 0) / pixel(2, 0);
		y = pixel(1, 0) / pixel(2, 0);

		if (range < 4.0 || range > 70. || (fabs(point.x) < 6.0 && fabs(point.y) < 4.))
			point.x = point.y = point.z = MAX_RANGE; 

		pcl::PointXYZRGB point2(point);

		//point2.x = point.x;
		//point2.y = point.y;
		//point2.z = point.z;
		//point2.r = point.r;
		//point2.g = point.g;
		//point2.b = point.b;

		// to use fused camera and velodyne
		if (0)
		//if ((point.x > 0 && x >= 0 && x < img.cols) && (y >= 0 && y < img.rows) 
		 	//&& (!_use_segmented || (y > top_limit && y < bottom_limit))) // && (point.z < 0))
		{
			// colors
			p = 3 * (y * img.cols + x);
			point2.r = img.data[p + 2];
			point2.g = img.data[p + 1];
			point2.b = img.data[p + 0];

			if (view)
				circle(viewer_img, Point(x, y), 2, Scalar(0,0,255), -1);

            /*
            printf("** WARNING: REMOVING MAX RANGE AT SEGMAP_DATASET.CPP **\n");
            if (point.x < 70.)
            */

    		cloud->push_back(point2);
		}
		// to use remission
		else if (1)
		// else if (point.z < 0.)
		{
			cloud->push_back(point2);
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
		    char name[512];
			sprintf(name, "%s/bb3/%lf-r.png", _path.c_str(), data[i].image_time);
			Mat aux_img = imread(name);
			Mat view_copy = viewer_img.clone();
			cv::vconcat(aux_img, view_copy, viewer_img);
			//flip(viewer_img, viewer_img, +1);
		}

		double resize_rate = 750. / (double) viewer_img.rows;
		int height = resize_rate * viewer_img.rows;
		int width = resize_rate * viewer_img.cols;

		Mat resized(height, width, CV_8UC3);
		resize(viewer_img, resized, Size(width, height));
		//imshow("cam_vel_fused", resized);
		if (output_img_view != NULL)
			resized.copyTo(*output_img_view);

	}

	transformPointCloud(*cloud, *cloud, transform_vel2car());
}


void
DatasetCarmen::_init_vel2cam_transform(int image_height, int image_width)
{
	Matrix<double, 3, 4> projection;
	Matrix<double, 4, 4> velodyne2board;
	Matrix<double, 4, 4> cam2board;
	Matrix<double, 4, 4> velodyne2cam;

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

	printf("Camera parameters: focal length: %lf cu: %lf cv: %lf\n",
		sqrt(pow(fx_factor * image_width, 2) + pow(fy_factor * image_height, 2)),
		cu, cv
	);

    // see http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
    // Note: Storing cu and cv in the 3rd column instead of the 4th is a trick.
    // To compute the pixel coordinates we divide the first two
    // dimensions of the point in homogeneous coordinates by the third one (which is Z).
	projection << fx_meters / pixel_size, 0, cu, 0,
				  0, fy_meters / pixel_size, cv, 0,
				  0, 0, 1, 0.;
				  
	/*				  
    //cout << projection << endl;
	Matrix<double, 4, 4> mat = cam2board.inverse() * velodyne2board;
    Matrix<double, 3, 3> Rmat;
    Rmat << mat(0, 0), mat(0, 1), mat(0, 2),
        mat(1, 0), mat(1, 1), mat(1, 2),
        mat(2, 0), mat(2, 1), mat(2, 2);

    Matrix<double, 3, 1> ypr = Rmat.eulerAngles(2, 1, 0);

	cout << "vel2cam:" << 
        mat(0, 3) << " " << mat(1, 3) << " " << mat(2, 3) << " " <<
        ypr(2, 0) << " " << ypr(1, 0) << " " << ypr(0, 0) << 
        endl;
	//cout << R << endl;
	*/

	_vel2cam = projection * R * cam2board.inverse() * velodyne2board;
    //_vel2cam = projection * pose6d_to_matrix(0.04, 0.115, -0.27, -M_PI/2-0.052360, -0.034907, -M_PI/2-0.008727).inverse();
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


void
DatasetCarmen::_load_velodyne_intensity_calibration()
{
	const char *path = "data/calibration_table.txt";
	
	FILE *calibration_file_bin = fopen(path, "r");
	if (!calibration_file_bin)
		exit(printf("Error: file '%s' does not exist.\n", path));

	_velodyne_intensity_calibration = (uchar ***) calloc(32, sizeof(uchar **));

	for (int i = 0; i < 32; i++)
	{
		_velodyne_intensity_calibration[i] = (uchar **) calloc(10, sizeof(uchar *));
		
		for (int j = 0; j < 10; j++)
			_velodyne_intensity_calibration[i][j] = (uchar *) calloc(256, sizeof(uchar));
	}

	int laser, ray_size, intensity;
	long accumulated_intennsity, count;
	float val, max_val = 0.0, min_val = 255.0;
	
	while (fscanf(calibration_file_bin, "%d %d %d %f %ld %ld", &laser, &ray_size, &intensity, &val, &accumulated_intennsity, &count) == 6)
	{
		_velodyne_intensity_calibration[laser][ray_size][intensity] = (uchar) val;

		if (val > max_val)
			max_val = val;
		
		if (val < min_val)
			min_val = val;
	}

	for (int i = 0; i < 32; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			for (int k = 0; k < 256; k++)
			{
				val = _velodyne_intensity_calibration[i][j][k];
				val = (val - min_val) / (max_val - min_val);

				if (val > 1.0)
					val = 1.0;

				if (val < 0.0)
					val = 0.0;

				_velodyne_intensity_calibration[i][j][k] = (uchar) (val * 255.);
			}
		}
	}

	fclose(calibration_file_bin);
}


DatasetCarmen::DatasetCarmen(string path, int use_segmented) :
	DatasetInterface(path, use_segmented)
{
	load_data();
	_load_velodyne_intensity_calibration();
	_unknown_class = CityScapesColorMap().n_classes + 1;
	_init_vel2car_transform();
	_init_vel2cam_transform(image_height, image_width);
}


void 
DatasetCarmen::_segment_lane_marks(Mat &m, int i)
{
    char name[512];
	double r, g, b, gray;
	sprintf(name, "%s/bb3/%lf-r.png", _path.c_str(), data[i].image_time);
	Mat bgr_img = imread(name);

	for (int i = 450; i < 750; i++)
	{
		for (int j = 0; j < bgr_img.cols; j++)
		{
			b = bgr_img.data[3 * (i * bgr_img.cols + j)];
			g = bgr_img.data[3 * (i * bgr_img.cols + j) + 1];
			r = bgr_img.data[3 * (i * bgr_img.cols + j) + 2];
			gray = (b + g + r) / 3;

			if (gray > 40)
			{
				m.data[3 * (i * m.cols + j)] = 20;
				m.data[3 * (i * m.cols + j) + 1] = 20;
				m.data[3 * (i * m.cols + j) + 2] = 20;
			}
		}
	}
}


Mat
DatasetCarmen::load_image(int i)
{
    char name[512];

	if (_use_segmented)
	{
		sprintf(name, "%s/semantic/%lf-r.png", _path.c_str(), data[i].image_time);
	}
	else
		sprintf(name, "%s/bb3/%lf-r.png", _path.c_str(), data[i].image_time);

	Mat raw_img = imread(name);

    if (raw_img.data == 0 || raw_img.rows == 0 || raw_img.cols == 0)
    	exit(printf("Error: Image '%s' not found.\n", name));

	Mat resized;

	if (_use_segmented)
	{
		int height = (int) (0.75 * raw_img.cols);
		resized = Mat::ones(height, raw_img.cols, CV_8UC3) * 19;
		int top_limit = (50. / 480.) * height;
		int bottom_limit = height - (110. / 480.) * height;
		raw_img.copyTo(resized(Rect(0, top_limit, raw_img.cols, bottom_limit - top_limit)));

		if (_path.find("aeroporto") != std::string::npos)
			_segment_lane_marks(resized, i);
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
DatasetCarmen::load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud, double v, double phi)
{
	int success;

    char name[512];
	sprintf(name, "%s/velodyne/%lf.ply", _path.c_str(), data[i].velodyne_time);

	success = pcl::io::loadPLYFile(name, *cloud);

	if (success < 0 || cloud->size() == 0)
		exit(printf("Cloud %s not found.\n", name));

	// correct the points positions considering of the car motion.	
	Pose2d correction(0., 0., 0.);

	for (int i = 0; i < cloud->size(); i++)
	{
		correct_point(correction, _vel2car, cloud->at(i));
		ackerman_motion_model(correction, v, phi, (TIME_SPENT_IN_EACH_SCAN / 32.));
	}
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
	double q0, q1, q2, q3;

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

		n_read = fscanf(f, " %s %s %s %s %lf %lf %lf %lf %s %s %s %s %s %s %s %s %lf\n", 
			dummy, dummy, dummy, dummy, 
			&q0, &q1, &q2, &q3,
			dummy, dummy, dummy, 
			dummy, dummy, dummy, 
			dummy, dummy, &sample.xsens_time);
		if (n_read != 17) exit(printf("Error: xsens data not found at '%s'!\n", data_file.c_str()));

		sample.xsens = Quaterniond(q0, q1, q2, q3);
		sample.gps.x -= offset_x;
		sample.gps.y -= offset_y;
		sample.gps.y = -sample.gps.y;
		sample.gps.th = normalize_theta(-sample.gps.th);
		sample.phi = normalize_theta(-sample.phi);
		sample.pose = Pose2d(0., 0., 0.);
		sample.pose_registered_to_map = Pose2d(0., 0., 0.);
		sample.pose_with_loop_closure = Pose2d(0., 0., 0.);

		data.push_back(sample);
	}

	fclose(f);

	string name = _path + "/odom_calib_stderr.txt";
	f = fopen(name.c_str(), "r");

	if (f == NULL)
		exit(printf("Error: file '%s' not found.\n", name.c_str()));

	fscanf(f, "%s %s %lf %s %s %s %lf %lf %s %s %lf",
		dummy, dummy, &odom_calib.mult_v, dummy, dummy, dummy, 
		&odom_calib.mult_phi, &odom_calib.add_phi, dummy, dummy, 
		&odom_calib.init_angle);

	data_file = _path + "/optimized_to_map.txt";
	f = fopen(data_file.c_str(), "r");

	if (f == NULL)
	{
		printf("Warning: File '%s' not found. Filling poses with zero.\n", data_file.c_str());
	}
	else
	{
		printf("Reading poses from '%s'\n", data_file.c_str());

		for (int i = 0; i < data.size(); i++)
		{
			fscanf(f, "\n%s %lf %lf %lf %s %s %s %s %s\n",
					dummy, 
					&data[i].pose_registered_to_map.x, 
					&data[i].pose_registered_to_map.y, 
					&data[i].pose_registered_to_map.th,
					dummy, dummy, dummy, dummy, dummy);

			data[i].pose_registered_to_map.x -= offset_x;
			data[i].pose_registered_to_map.y -= offset_y;
			data[i].pose_registered_to_map.y = -data[i].pose_registered_to_map.y;
			data[i].pose_registered_to_map.th = normalize_theta(-data[i].pose_registered_to_map.th);

	//		printf("%lf %lf %lf %lf %lf %lf\n",
	//				data[i].gps.x, data[i].gps.y, data[i].gps.th,
	//				data[i].pose.x, data[i].pose.y, data[i].pose.th);
		}

		fclose(f);
	}

	data_file = _path + "/optimized_with_gicp.txt";
	f = fopen(data_file.c_str(), "r");

	if (f == NULL)
	{
		printf("Warning: File '%s' not found. Filling poses with zero.\n", data_file.c_str());
	}
	else
	{
		printf("Reading poses from '%s'\n", data_file.c_str());

		for (int i = 0; i < data.size(); i++)
		{
			fscanf(f, "\n%s %lf %lf %lf %s %s %s %s %s\n",
					dummy, 
					&data[i].pose_with_loop_closure.x, 
					&data[i].pose_with_loop_closure.y, 
					&data[i].pose_with_loop_closure.th,
					dummy, dummy, dummy, dummy, dummy);

			data[i].pose_with_loop_closure.x -= offset_x;
			data[i].pose_with_loop_closure.y -= offset_y;
			data[i].pose_with_loop_closure.y = -data[i].pose_with_loop_closure.y;
			data[i].pose_with_loop_closure.th = normalize_theta(-data[i].pose_with_loop_closure.th);

	//		printf("%lf %lf %lf %lf %lf %lf\n",
	//				data[i].gps.x, data[i].gps.y, data[i].gps.th,
	//				data[i].pose.x, data[i].pose.y, data[i].pose.th);
		}

		fclose(f);
	}

	data_file = _path + "/optimized.txt";
	f = fopen(data_file.c_str(), "r");

	if (f == NULL)
	{
		printf("Warning: File '%s' not found. Filling poses with zeros.\n", data_file.c_str());	
	}
	else
	{
		printf("Reading poses from '%s'\n", data_file.c_str());

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
DatasetKitti::load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud, double v, double phi)
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


NewCarmenDataset::NewCarmenDataset(char *path, int sync_type)
{
	_fptr = safe_fopen(path, "r");
	_sync_type = sync_type;
	_sample = new DataSample();

	_velodyne_path = string(path) + "_velodyne";
	_images_path = string(path) + "_bumblebee";

	_load_odometry_calibration(path);
}


NewCarmenDataset::~NewCarmenDataset()
{
	fclose(_fptr);
	delete(_sample);
	_clear_synchronization_queues();
}


void
NewCarmenDataset::reset()
{
	rewind(_fptr);
}


void 
NewCarmenDataset::_load_odometry_calibration(char *path)
{
	vector<char*> splitted = string_split(path, "/");
	string odom_calib = "/dados/data2/data_" + string(splitted[splitted.size() - 1]) + "/odom_calib.txt";
	
	FILE *f = fopen(odom_calib.c_str(), "r");
	
	if (f != NULL)
	{
		fscanf(f, "bias v: %lf 0.000000 bias phi: %lf %lf Initial Angle: %lf",
			&_calib.mult_v, 
			&_calib.mult_phi,
			&_calib.add_phi,
			&_calib.init_angle);

		fclose(f);
	}
	else
	{
		printf("Warning: odometry calibration file not found. Assuming default values.\n");
		
		_calib.mult_phi = _calib.mult_v = 1.0;
		_calib.init_angle = 0.;
		_calib.add_phi = 0.;
	}

	printf("Odom calibration: bias v: %lf 0.000000 bias phi: %lf %lf\n", 
		_calib.mult_v, _calib.mult_phi, _calib.add_phi);
}


void
NewCarmenDataset::_free_queue(vector<char*> queue)
{
	for (int i = 0; i < queue.size(); i++)
		free(queue[i]);
}


void 
NewCarmenDataset::_clear_synchronization_queues()
{
	_free_queue(_imu_queue);
	_free_queue(_gps_position_queue);
	_free_queue(_gps_orientation_queue);
	_free_queue(_odom_queue);
	_free_queue(_camera_queue);
	_free_queue(_velodyne_queue);

	_imu_queue.clear();
	_gps_position_queue.clear();
	_gps_orientation_queue.clear();
	_odom_queue.clear();
	_camera_queue.clear();
	_velodyne_queue.clear();
}


void 
NewCarmenDataset::_add_message_to_queue(char *line)
{
	// ignore small lines
	if (strlen(line) > 10)
	{
		if (!strncmp("NMEAGGA", line, strlen("NMEAGGA")) && line[strlen("NMEAGGA") + 1] == '1')
			_gps_position_queue.push_back(string_copy(line));
		else if (!strncmp("NMEAHDT", line, strlen("NMEAHDT")))
			_gps_orientation_queue.push_back(string_copy(line));
		else if (!strncmp("ROBOTVELOCITY_ACK", line, strlen("ROBOTVELOCITY_ACK")))
			_odom_queue.push_back(string_copy(line));
		else if (!strncmp("XSENS_QUAT", line, strlen("XSENS_QUAT")))
			_imu_queue.push_back(string_copy(line));
		else if (!strncmp("BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3", line, strlen("BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3")))
			_camera_queue.push_back(string_copy(line));
		else if (!strncmp("VELODYNE_PARTIAL_SCAN_IN_FILE", line, strlen("VELODYNE_PARTIAL_SCAN_IN_FILE")))
			_velodyne_queue.push_back(string_copy(line));
	}
}


vector<char*> 
NewCarmenDataset::_find_nearest(vector<char*> &queue, double ref_time)
{
	double msg_time, nearest_time;
	vector<char*> splitted;
	vector<char*> most_sync;

	nearest_time = 0;

	for (int i = 0; i < queue.size(); i++)
	{
		splitted = string_split(queue[i], " ");
		msg_time = atof(splitted[splitted.size() - 3]);

		if (fabs(msg_time - ref_time) < fabs(nearest_time - ref_time))
		{
			nearest_time = msg_time;
			most_sync = vector<char*>(splitted);
		}
	}

	return most_sync;
}


void 
NewCarmenDataset::_parse_odom(vector<char*> data, DataSample *sample)
{
	sample->v = atof(data[1]);
	sample->phi = atof(data[2]);
	sample->odom_time = atof(data[3]);
}


void 
NewCarmenDataset::_parse_imu(vector<char*> data, DataSample *sample)
{
	sample->xsens = Quaterniond(
		atof(data[4]),
		atof(data[5]),
		atof(data[6]),
		atof(data[7])
	);

	sample->xsens_time = atof(data[data.size() - 3]);
}


void
NewCarmenDataset::_parse_velodyne(vector<char*> data, DataSample *sample, string velodyne_path)
{
	vector<char*> splitted = string_split(data[1], "/");
	
	int n = splitted.size();
	string path = velodyne_path + "/" + 
		splitted[n - 3] + "/" + 
		splitted[n - 2] + "/" + 
		splitted[n - 1];

	sample->n_laser_shots = atoi(data[2]);
	sample->velodyne_path = path;
	sample->velodyne_time =  atof(data[data.size() - 3]);
}


void 
NewCarmenDataset::_parse_camera(vector<char*> data, DataSample *sample, string image_path)
{
	vector<char*> splitted = string_split(data[1], "/");
	
	int n = splitted.size();
	string path = image_path + "/" + 
		splitted[n - 3] + "/" + 
		splitted[n - 2] + "/" + 
		splitted[n - 1];

	sample->image_path = path;
	sample->image_height = atoi(data[2]);
	sample->image_width = atoi(data[3]);
	sample->image_time = atof(data[data.size() - 3]);
}


void 
NewCarmenDataset::_parse_gps_position(vector<char*> data, DataSample *sample)
{
	double lt = carmen_global_convert_degmin_to_double(atof(data[3]));
	double lg = carmen_global_convert_degmin_to_double(atof(data[5]));

	// verify the latitude and longitude orientations
	if ('S' == data[4][0]) lt = -lt;
	if ('W' == data[6][0]) lg = -lg;

	// convert to x and y coordinates
	Gdc_Coord_3d gdc = Gdc_Coord_3d(lt, lg, atof(data[10]));

	// Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc , utm);

	sample->gps.x = utm.y;
	sample->gps.y = -utm.x;

	sample->gps_quality = atoi(data[7]);
	sample->gps_time = atof(data[data.size() - 3]);
}


void 
NewCarmenDataset::_parse_gps_orientation(vector<char*> data, DataSample *sample)
{
	sample->gps.th = atof(data[2]);
	sample->gps_orientation_quality = atoi(data[3]);
}


void
NewCarmenDataset::_assemble_data_package_from_queues()
{
	double ref_time;
	
	if (_sync_type == SYNC_BY_CAMERA) 
	{
		// most recent camera message
		vector<char*> splitted = string_split(_camera_queue[_camera_queue.size() - 1], " ");
		ref_time = atof(splitted[splitted.size() - 3]);

		_parse_camera(splitted, _sample, _images_path);
		_parse_velodyne(_find_nearest(_velodyne_queue, ref_time), _sample, _velodyne_path);
	}
	else if (_sync_type == SYNC_BY_LIDAR)
	{
		// most recent velodyne message
		vector<char*> splitted = string_split(_velodyne_queue[_velodyne_queue.size() - 1], " ");
		ref_time = atof(splitted[splitted.size() - 3]);

		_parse_velodyne(splitted, _sample, _velodyne_path);
		_parse_camera(_find_nearest(_camera_queue, ref_time), _sample, _images_path);
	}
	else
		exit(printf("Error: invalid sync type: '%d'\n", _sync_type));
	
	_parse_odom(_find_nearest(_odom_queue, ref_time), _sample);
	_parse_imu(_find_nearest(_imu_queue, ref_time), _sample);
	_parse_gps_position(_find_nearest(_gps_position_queue, ref_time), _sample);
	_parse_gps_orientation(_find_nearest(_gps_orientation_queue, ref_time), _sample);

	_sample->v *= _calib.mult_v;
	_sample->phi = normalize_theta(_sample->phi * _calib.mult_phi + _calib.add_phi);
}


DataSample* 
NewCarmenDataset::next_data_package()
{
	int all_sensors_were_received;
	static char _line[_MAX_LINE_LENGTH];

	_clear_synchronization_queues();
	all_sensors_were_received = 0;

	while (!all_sensors_were_received)
	{
		if (feof(_fptr))
			break;

		fscanf(_fptr, "\n%[^\n]\n", _line);
		_add_message_to_queue(_line);

		if (_velodyne_queue.size() > 0 && _camera_queue.size() > 0 && 
			_imu_queue.size() > 0 && _odom_queue.size() > 0 && 
			_gps_position_queue.size() > 0 && _gps_orientation_queue.size() > 0)
			all_sensors_were_received = 1;
	}

	// a data package could not be created because the log is over.
	if (!all_sensors_were_received) 
		return NULL;
	
	_assemble_data_package_from_queues();
	return _sample;
}


Mat 
NewCarmenDataset::read_image(DataSample *sample)
{
	static int image_size = sample->image_height * sample->image_width * 3;
	static unsigned char *raw_right = (unsigned char*) calloc (image_size, sizeof(unsigned char));
	static Mat img_r = Mat(sample->image_width, sample->image_height, CV_8UC3, raw_right, 0);
	
	FILE *image_file = safe_fopen(sample->image_path.c_str(), "rb");
	// jump the left image
	fseek(image_file, image_size * sizeof(unsigned char), SEEK_SET);
	fread(raw_right, image_size, sizeof(unsigned char), image_file);
	fclose(image_file);
	// carmen images are stored as rgb
	cvtColor(img_r, img_r, COLOR_RGB2BGR);

	return img_r;
}


PointXYZRGB
NewCarmenDataset::_compute_point_from_velodyne(double v_angle, double h_angle, double radius, unsigned char intensity)
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
NewCarmenDataset::read_pointcloud(DataSample *sample)
{
	static PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	FILE *f = safe_fopen(sample->velodyne_path.c_str(), "rb");

	double h_angle, v_angle;
	unsigned short distances[32];
	unsigned char intensities[32];
	double range;

	cloud->clear();

	for(int i = 0; i < sample->n_laser_shots; i++)
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

	    	PointXYZRGB point = _compute_point_from_velodyne(v_angle, -h_angle, range, intensities[velodyne_ray_order[j]]);
	    	cloud->push_back(point);
	    }
	}

    fclose(f);
	return cloud;
}
