
#include <opencv/cv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>

using namespace Eigen;
using namespace cv;
using namespace pcl;


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


int 
main()
{
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
 
    pcl::visualization::PCLVisualizer viewer("cloud");
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    
    pcl::io::loadPLYFile("1544204839.806388-minus_h.ply", *cloud);
    Mat img = imread("1544204840.171747-r.png");
    
    Matrix<double, 4, 1> point, pcam;
    Matrix<double, 3, 1> hpixel;
    Matrix<double, 3, 4> projection;
    Matrix<double, 4, 4> cam_wrt_velodyne, vel2cam;
 
    // cam wrt vel from segmap_dataset: 0.0942407 -0.038998 -0.272209 3.12383 3.09286 3.12802
    cam_wrt_velodyne = pose6d_to_matrix(0.1, 0.115, -0.27, 
        -M_PI/2, // + DEG2RAD(0), 
        0, //  + DEG2RAD(0), 
        -M_PI/2); //  + DEG2RAD(0));
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
    
    projection << 
        489.439, 0, 323.471, 0,
        0, 489.437, 237.031, 0,
        0, 0, 1, 0
    ;
 
    for (int i = 0; i < cloud->size(); i++)
    {
        point << cloud->at(i).x, cloud->at(i).y, cloud->at(i).z, 1;
        
        pcam = vel2cam * point;
        hpixel = projection * pcam;
        Point pixel((int) hpixel(0, 0) / hpixel(2, 0), (int) hpixel(1, 0) / hpixel(2, 0));
        double r = sqrt(pow(point(0, 0), 2) + pow(point(1, 0), 2) + pow(point(2, 0), 2));
        
        if (pcam(2, 0) > 0. && r < 70.)
            circle(img, pixel, 2, Scalar(0, 0, 255), -1);
    }
    
    imshow("img", img);
    waitKey(100);
    
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(2);
	viewer.addPointCloud(cloud, "cloud");
	viewer.spin();

    return 0;
}
