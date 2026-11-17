
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace cv;
using namespace Eigen;


int 
main()
{
    int32_t num = 1000000;
    float *data = (float*) malloc(num * sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen ("velodyne_0000000000.bin", "rb");
    num = fread(data, sizeof(float), num, stream) / 4;

    Mat orig = imread("cam2_0000000000.png");
    Mat img = orig.clone();
    //Mat img = Mat::zeros(375, 1242, CV_8UC3);

    Matrix<double, 4, 4> R00, R02, R03;
    Matrix<double, 3, 4> P00, P02, P03;
    Matrix<double, 4, 4> Rt, Rt_02, Rt_03;
    Matrix<double, 4, 1> P;
    Matrix<double, 3, 1> pixel;

    R00 << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0., 
         -9.869795e-03, 9.999421e-01, -4.278459e-03, 0., 
          7.402527e-03, 4.351614e-03, 9.999631e-01, 0.,
          0., 0., 0., 1.;

    P00 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00, 
           0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00, 
           0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;

    Rt << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
              1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
              9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
              0., 0., 0., 1.;

    R02 << 9.998817e-01, 1.511453e-02, -2.841595e-03, 0.,
    		-1.511724e-02, 9.998853e-01, -9.338510e-04, 0.,
			2.827154e-03, 9.766976e-04, 9.999955e-01, 0.,
			0, 0, 0, 1;

	P02 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
			0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
			0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03;

	Rt_02 << 9.999758e-01, -5.267463e-03, -4.552439e-03, 5.956621e-02,
			5.251945e-03, 9.999804e-01, -3.413835e-03, 2.900141e-04,
			4.570332e-03, 3.389843e-03, 9.999838e-01, 2.577209e-03,
			0, 0, 0, 1;

    R03 << 9.998321e-01, -7.193136e-03, 1.685599e-02, 0,
    		7.232804e-03, 9.999712e-01, -2.293585e-03, 0,
			-1.683901e-02, 2.415116e-03, 9.998553e-01, 0,
			0, 0, 0, 1;

	P03 << 7.215377e+02, 0.000000e+00, 6.095593e+02, -3.395242e+02,
			0.000000e+00, 7.215377e+02, 1.728540e+02, 2.199936e+00,
			0.000000e+00, 0.000000e+00, 1.000000e+00, 2.729905e-03;

	Rt_03 << 9.995599e-01, 1.699522e-02, -2.431313e-02, -4.731050e-01,
			-1.704422e-02, 9.998531e-01, -1.809756e-03, 5.551470e-03,
			2.427880e-02, 2.223358e-03, 9.997028e-01, -5.250882e-03,
			0, 0, 0, 1;

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(1., 1., 1.);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int32_t i=0; i < num; i++)
    {
        px += 4; py += 4; pz += 4; pr += 4;

        if (*px > 0)
        {
            int x, y;

            P << *px, *py, *pz, 1.;

            //pixel = P00 * R00 * Rt * P;
            pixel = P02 * R00 * Rt * P;

            x = pixel(0, 0) / pixel(2, 0);
            y = pixel(1, 0) / pixel(2, 0);

			pcl::PointXYZRGB point;
			point.x = *px;
			point.y = *py;
			point.z = *pz;
			point.r = 255;
			point.g = 0;
			point.b = 0;

			if (x >= 0 && x < img.cols && y >= 0 && y < img.rows)
            {
            	circle(img, Point(x, y), 1, Scalar(0, 0, 255), -1);

    			point.r = orig.data[3 * (y * orig.cols + x) + 2];
    			point.g = orig.data[3 * (y * orig.cols + x) + 1];
    			point.b = orig.data[3 * (y * orig.cols + x) + 0];
            }

			cloud->push_back(point);
        }
    }

    fclose(stream);

    viewer.addPointCloud(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

    imshow("img", img);

    while (1)
    {
		viewer.spinOnce();
		waitKey(1);
    }

    return 0;
}






