
#include <Eigen/Core>
#include <opencv/cv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_sensors.h>
#include <carmen/segmap_viewer.h>
#include <carmen/segmap_util.h>
#include <string>


using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;


void
draw_laser_in_cam(PointCloud<PointXYZRGB>::Ptr cloud, 
                  Matrix<double, 4, 4> &lidar2cam,
                  Matrix<double, 3, 4> &projection,
                  Mat &img)
{
	Mat orig = img.clone();
	Matrix<double, 4, 1> plidar, pcam;
	Matrix<double, 3, 1> ppixelh;
	Point ppixel;

	for (int i = 0; i < cloud->size(); i++)
	{
		plidar << cloud->at(i).x, cloud->at(i).y, cloud->at(i).z, 1;
		pcam = lidar2cam * plidar;

		if (pcam(0, 0) / pcam(3, 0) > 0)
		{
			ppixelh = projection * pcam;
			ppixel.x = (ppixelh(0, 0) / ppixelh(2, 0)) * img.cols;
			ppixel.y = (ppixelh(1, 0) / ppixelh(2, 0)) * img.rows;

			if (ppixel.x >= 0 && ppixel.x < img.cols && ppixel.y >= 0 && ppixel.y < img.rows)
				circle(img, ppixel, 2, Scalar(0,0,255), -1);
		}
	}
}


int 
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <log path>\n", argv[0]));

	Matrix<double, 4, 1> point, pcam;
	Matrix<double, 3, 1> hpixel;
	Matrix<double, 4, 4> cam_wrt_velodyne, vel2cam;

	double droll = 0, dpitch = 0, dyaw = 0, dx = 0, dy = 0, dz = 0;

	PointCloudViewer viewer;
	DataSample *sample;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	string odom_calib_path = default_odom_calib_path(argv[1]);
	NewCarmenDataset dataset(argv[1], odom_calib_path);

	Matrix<double, 3, 4> projection = dataset.projection_matrix();

	double default_x = -0.020000;
	double default_y = 0.125000;
	double default_z = -0.27;

	double default_roll = -0.015708;
	double default_pitch = 0.048869;
	double default_yaw = 0.005236;

	int pos = 0;
	sample = dataset[0];

	while (1)
	{
		double x, y, z;
		x = default_x + dx;
		y = default_y + dy;
		z = default_z + dz;

		double roll = default_roll + droll;
		double pitch = default_pitch + dpitch;
		double yaw = default_yaw + dyaw;

		printf("x: %lf y: %lf z: %lf roll: %lf pitch: %lf yaw: %lf\n", x, y, z, roll, pitch, yaw);

		cam_wrt_velodyne = pose6d_to_matrix(x, y, z, roll, pitch, yaw);
		vel2cam = cam_wrt_velodyne.inverse();

		CarmenLidarLoader loader(sample->velodyne_path.c_str(), sample->n_laser_shots,
		                         dataset.intensity_calibration);

		load_as_pointcloud(&loader, cloud);
		printf("n points: %ld\n", cloud->size());
		Mat img = load_image(sample);

		draw_laser_in_cam(cloud, vel2cam, projection, img);

		char c = ' ';

		while (1)
		{
			imshow("img", img);
			c = waitKey(50);

			if (c == 'q') { droll += DEG2RAD(0.1); break; }
			if (c == 'a') { droll -= DEG2RAD(0.1); break; }
			if (c == 'w') { dpitch += DEG2RAD(0.1); break; }
			if (c == 's') { dpitch -= DEG2RAD(0.1); break; }
			if (c == 'e') { dyaw += DEG2RAD(0.1); break; }
			if (c == 'd') { dyaw -= DEG2RAD(0.1); break; }
			if (c == 't') { dx += 0.01; break; }
			if (c == 'g') { dx -= 0.01; break; }
			if (c == 'y') { dy += 0.01; break; }
			if (c == 'h') { dy -= 0.01; break; }
			if (c == 'u') { dz += 0.01; break; }
			if (c == 'j') { dz -= 0.01; break; }
			if (c == 'p') { dx = dy = dz = droll = dpitch = dyaw = 0.; break; }
			if (c == 'n') { sample = dataset[++pos % dataset.size()]; break; }
		}
	}

	return 0;
}
