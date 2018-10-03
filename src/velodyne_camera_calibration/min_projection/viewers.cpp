
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <pcl/common/transforms.h>


using namespace cv;
using namespace std;


int img_x = -1, img_y = -1;
float pcl_x = -1, pcl_y = -1, pcl_z = -1;


void 
pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    float x, y, z;
    std::cout << "[INOF] Point picking event occurred." << std::endl;

    if (event.getPointIndex () == -1)
        return;

    event.getPoint(x, y, z);
    std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;

    pcl_x = x;
    pcl_y = y;
    pcl_z = z;
}


void 
CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  (event == EVENT_LBUTTONUP)
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        img_x = x;
        img_y = y;
    }
}


int 
main (int argc, char** argv)
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

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int32_t i=0; i < num; i++) 
    {
        px += 4; py += 4; pz += 4; pr += 4;

        if (*px > 0)
        {
            double r = (*pr);
            double b = 1. - (*pr);

            pcl::PointXYZRGB point;
            point.x = *px;
            point.y = *py;
            point.z = *pz;
            point.r = 255 * r;
            point.g = 0;
            point.b = 255 * b;
            cloud->push_back(point);
        }
    }

    fclose(stream);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // theta radians around X axis
    transform.rotate(Eigen::AngleAxisf(-M_PI / 2., Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(M_PI / 2., Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    viewer.setBackgroundColor(1., 1., 1.);
    viewer.addPointCloud(transformed_cloud, "cloud");
    viewer.registerPointPickingCallback(pointPickingEventOccurred, (void*) &viewer);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

    Mat img = imread("cam0_0000000000.png");
    namedWindow("img", 1);
    setMouseCallback("img", CallBackFunc, NULL);

    while (1)
    {
        Mat img_view = img.clone();
        
        if (img_x != -1 && img_y != -1)
            cv::circle(img_view, cv::Point(img_x, img_y), 5, Scalar(0, 255, 0), -1);

        imshow("img", img_view);

        viewer.removeAllShapes();

        if (pcl_x != -1 && pcl_y != -1 && pcl_z != -1)
        {
            pcl::PointXYZ center;
            center.x = pcl_x;
            center.y = pcl_y;
            center.z = pcl_z;
            
            viewer.addSphere(center, .1, 0, 255, 0);
        }

        waitKey(1);
        viewer.spinOnce();
    }

    return 0;
}


