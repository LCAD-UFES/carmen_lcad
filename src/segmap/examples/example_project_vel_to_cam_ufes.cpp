
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
    pcl::visualization::PCLVisualizer viewer("cloud");
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    
    pcl::io::loadPLYFile("/dados/data/data_log-jardim_da_penha-20181207-2.txt/velodyne/1544204839.806388.ply", *cloud);
    Mat img = imread("/dados/data/data_log-jardim_da_penha-20181207-2.txt/bb3/1544204840.171747-r.png");
    
    Matrix<double, 4, 4> velodyne2cam;
    velodyne2cam <<
        0.998721,  -0.013557, -0.0487156,   -0.10791,
        0.0127058,   0.999762, -0.0177392,  0.0329626,
        0.0489445,  0.0170975,   0.998655,   0.267897,
        0,          0,          0,          1
	;
 
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
 
    //velodyne2cam = pose6d_to_matrix(-0.025, -0.115, 0.27, 0, 0, 0);
    Matrix<double, 4, 4> vel_wrt_cam_t = pose6d_to_matrix(-0.025, -0.115, 0.27, 0, 0, 0);

    Matrix<double, 4, 4> R;
     /*
     R <<
        3.7494e-33,          -1, 6.12323e-17,           0,
        6.12323e-17, 6.12323e-17,           1,           0,
                 -1,           0, 6.12323e-17,           0,
                  0,          0,           0,           1
    ;
    */
    R = pose6d_to_matrix(0, 0, 0, -M_PI/2., M_PI/2., 0);

    ///*
    Matrix<double, 4, 1> bla;
    bla << 1., 0., 0., 0.;
    cout << "X:" << endl;
    cout << R * bla << endl << endl;

    bla << 0., 1., 0., 0.;
    cout << "Y:" << endl;
    cout << R * bla << endl << endl;

    bla << 0., 0., 1., 0.;
    cout << "Z:" << endl;
    cout << R * bla << endl << endl;
     
    return 0;     
    //*/
     
    Matrix<double, 3, 4> projection;
    projection << 
        489.439,       0, 323.471,       0,
        0, 489.437, 237.031,       0,
        0,       0,       1,       0
    
    ;
 
    Matrix<double, 4, 1> point, pcam, prot;
    Matrix<double, 3, 1> hpixel;
 
    for (int i = 0; i < 1; i++) // cloud->size(); i++)
    {
        Matrix<double, 4, 1> point;
        point << 
            2, // cloud->at(i).x,
            0, // cloud->at(i).y,
            0, // cloud->at(i).z,            
            1;
        
        pcam = velodyne2cam * point;
        prot = R * pcam;
        hpixel = projection * prot;
        Point pixel((int) hpixel(0, 0) / hpixel(2, 0), (int) hpixel(1, 0) / hpixel(2, 0));
        double r = sqrt(pow(point(0,0), 2) + pow(point(1,0), 2) + pow(point(2,0), 2));
        
        cout << "point" << endl;
        cout << point << endl << endl;
        cout << "pcam" << endl;
        cout << pcam << endl << endl;
        cout << "prot" << endl;
        cout << prot << endl << endl;
        cout << "hpixel" << endl;
        cout << hpixel << endl << endl;
        cout << "pixel" << endl;
        cout << pixel.x << " " << pixel.y << endl << endl;
        
        if (pcam(0, 0) > 0 && r < 70.)
            circle(img, pixel, 2, Scalar(0,0,255), -1);
    }
    
    imshow("img", img);
    waitKey(100);
    
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(2);
	viewer.addPointCloud(cloud, "cloud");
	viewer.spin();
    
    return 0;
}
