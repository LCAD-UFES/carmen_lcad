
#include <carmen/segmap_util.h>
#include <carmen/segmap_dataset.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>

using namespace pcl;
using namespace cv;


const double starting_angle = degrees_to_radians(-50);
const double fov = degrees_to_radians(100);
const double resolution = degrees_to_radians(0.5);
const double nrows = 32;
const double ncols = (int) (fov / resolution + 1.0);


void
convert_pointcloud_to_images(PointCloud<PointXYZRGB>::Ptr cloud, Mat *input, Mat *output)
{
    double hangle;
    double vangle;
    double range;

    (*input) = Mat::zeros(nrows, ncols, CV_8UC1);
    (*output) = Mat::zeros(nrows, ncols, CV_8UC1);

    for (int i = 0; i < cloud->size(); i++)
    {
        hangle = 0;
        vangle = 0;
        range = 0;
    }
}


int 
main()
{
    DatasetCarmen dataset("/dados/data/data_log_estacionamentos-20181130.txt", 0);

	PointCloud<PointXYZRGB>::Ptr cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);
    pcl::visualization::PCLVisualizer *viewer = NULL;
    viewer = new pcl::visualization::PCLVisualizer();
    viewer->setBackgroundColor(1, 1, 1);

    for (int i = 0; i < dataset.data.size(); i++)
    {
        dataset.load_fused_pointcloud_and_camera(i, cloud, 0, 0, 1);
		viewer->addPointCloud(cloud, "pointcloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pointcloud");
		viewer->spin();
    }

    return 0;
}