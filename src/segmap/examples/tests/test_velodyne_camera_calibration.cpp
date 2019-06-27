
#include <string>
#include <opencv/cv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>
#include <carmen/segmap_preproc.h>
#include <carmen/command_line.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_constructors.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_conversions.h>

using namespace Eigen;
using namespace cv;
using namespace pcl;
using namespace std;


int 
main(int argc, char **argv)
{
	CommandLineArguments args;

	add_default_slam_args(args);
	add_default_sensor_preproc_args(args);
	args.parse(argc, argv);

	string log_path = args.get<string>("log_path");

	NewCarmenDataset *dataset = create_dataset(log_path, args, "graphslam");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);

	/*
    From carmen-ford-escape.ini

    velodyne_x	0.145
    velodyne_y	0.0
    velodyne_z	0.48
    velodyne_roll	0.0
    velodyne_pitch	-0.0227
    velodyne_yaw	-0.01 # 0.09

    camera3_x		0.245   	#0.130 		#teste do tracker 0.25			# 0.325 			# 1.23
    camera3_y		0.115 # -0.04		#0.149		# -0.28
    camera3_z		0.210 		#0.214 			#0.146			# 1.13
    camera3_roll	-0.017453 		# -0.017453 	#0.0		# 0.0
    camera3_pitch	0.026037		# -0.034907 	#0.0		# 0.053
    camera3_yaw		-0.023562 		# -0.017453	#0.0 		#0.5585			# 0.09
	 */
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	Matrix<double, 4, 1> point, pcam;
	Matrix<double, 3, 1> hpixel;
	Matrix<double, 3, 4> projection;
	Matrix<double, 4, 4> cam_wrt_velodyne, vel2cam;

	double droll = 0, dpitch = 0, dyaw = 0, dx = 0, dy = 0, dz = 0;

	projection <<
			489.439, 0, 323.471, 0,
			0, 489.437, 237.031, 0,
			0, 0, 1, 0
			;

	double default_x = 0.04;
	double default_y = 0.115;
	double default_z = -0.27;

	double default_roll = -0.052360;
	double default_pitch = -0.034907;
	double default_yaw = 0.008727;

	int pause = 1;

	for (int i = 0; i < dataset->size(); i++)
	{
		if (fabs(dataset->at(i)->v) < 1.0)
			continue;

		double x, y, z;
		x = default_x + dx;
		y = default_y + dy;
		z = default_z + dz;

		double roll = default_roll + droll;
		double pitch = default_pitch + dpitch;
		double yaw = default_yaw + dyaw;

		printf("x: %lf y: %lf z: %lf roll: %lf pitch: %lf yaw: %lf\n", x, y, z, roll, pitch, yaw);

		// cam wrt vel from segmap_dataset: 0.0942407 -0.038998 -0.272209 3.12383 3.09286 3.12802
		cam_wrt_velodyne = pose6d_to_matrix(x, y, z,
		                                    -M_PI/2 + roll,
		                                    0 + pitch,
		                                    -M_PI/2 + yaw);

		vel2cam = cam_wrt_velodyne.inverse();

		/*
        // BASIC EXAMPLES OF TRANSFORMATION FROM VELODYNE TO CAMERA

        // Saida esperada: [0.115, -0.27, 0.9]
        point << 1., 0., 0., 1.;
        cout << "vel2cam * (1, 0, 0):" << endl;
        cout << vel2cam * point << endl << endl;

        // Saida esperada: [0.885, -0.27, -0.1]
        point << 0., 1., 0., 1.;
        cout << "vel2cam * (0, 1, 0):" << endl;
        cout << vel2cam * point << endl << endl;

        // Saida esperada: [0.115, -1.27, -0.1]
        point << 0., 0., 1., 1.;
        cout << "vel2cam * (0, 0, 1):" << endl;
        cout << vel2cam * point << endl << endl;

        return 0;     
		 */

		preproc.reinitialize(dataset->at(i));
		Mat img_with_velodyne = preproc.read_img(dataset->at(i));
		load_as_pointcloud(preproc, cloud, SensorPreproc::SENSOR_REFERENCE);

		for (int i = 0; i < cloud->size(); i++)
		{
			point << cloud->at(i).x, cloud->at(i).y, cloud->at(i).z, 1;

			pcam = vel2cam * point;
			hpixel = projection * pcam;
			Point pixel((int) hpixel(0, 0) / hpixel(2, 0), (int) hpixel(1, 0) / hpixel(2, 0));
			double r = sqrt(pow(point(0, 0), 2) + pow(point(1, 0), 2) + pow(point(2, 0), 2));

			if (pcam(2, 0) > 0. && r < 70.)
				circle(img_with_velodyne, pixel, 2, Scalar(0, 0, 255), -1);
		}

		char c = ' ';

		while (1)
		{
			imshow("img", img_with_velodyne);
			c = waitKey(50);

			if (c == 'q') { droll += DEG2RAD(0.5); break; }
			if (c == 'a') { droll -= DEG2RAD(0.5); break; }
			if (c == 'w') { dpitch += DEG2RAD(0.5); break; }
			if (c == 's') { dpitch -= DEG2RAD(0.5); break; }
			if (c == 'e') { dyaw += DEG2RAD(0.5); break; }
			if (c == 'd') { dyaw -= DEG2RAD(0.5); break; }
			if (c == 't') { dx += 0.01; break; }
			if (c == 'g') { dx -= 0.01; break; }
			if (c == 'y') { dy += 0.01; break; }
			if (c == 'h') { dy -= 0.01; break; }
			if (c == 'u') { dz += 0.01; break; }
			if (c == 'j') { dz -= 0.01; break; }
			if (c == 'p') { dx = dy = dz = droll = dpitch = dyaw = 0.; break; }
			if (c == 'l') { pause = !pause; }
			if (c == 'n') { break; }

			if (!pause)
				break;
		}
	}

	return 0;
}
