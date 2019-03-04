
#include <cstdio>
#include "segmap_dataset.h"
#include "segmap_util.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;


int 
main()
{
    DataSample* data_package;

    NewCarmenDataset dataset = 
        NewCarmenDataset("/dados/log_estacionamentos-20181130-test.txt",
                         NewCarmenDataset::SYNC_BY_CAMERA);

    pcl::visualization::PCLVisualizer *cloud_viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
    cloud_viewer->setBackgroundColor(1, 1, 1);

    Pose2d dead_reckoning;
    double previous_time = 0;

    Pose2d gps0;

    while (data_package = dataset.next_data_package())
    {
        if (previous_time > 0.)
        {
            ackerman_motion_model(dead_reckoning, 
                data_package->v, data_package->phi, 
                data_package->odom_time - previous_time);
        }
        else
        {
            gps0 = data_package->gps;
        }

        printf("gps: %lf %lf ", data_package->gps.x, data_package->gps.y);
        printf("odom: %lf %lf ", dead_reckoning.x + gps0.x, dead_reckoning.y + gps0.y);
        printf("gps time: %lf ", data_package->gps_time);
        printf("image time: %lf ", data_package->image_time);
        printf("velodyne time: %lf ", data_package->velodyne_time);
        printf("odom time: %lf ", data_package->odom_time);
        printf("xsens time: %lf\n", data_package->xsens_time);
        
        previous_time = data_package->odom_time;

        printf("Image path: %s velodyne path: %s\n",
            data_package->image_path.c_str(),
            data_package->velodyne_path.c_str()
        );

        Mat img = dataset.read_image((char*) data_package->image_path.c_str());
        PointCloud<PointXYZRGB>::Ptr cloud = dataset.read_pointcloud((char*) data_package->velodyne_path.c_str());

        cloud_viewer->removeAllPointClouds();
        cloud_viewer->addPointCloud(cloud);
        cloud_viewer->spinOnce();

        imshow("img", img);
        waitKey(1);
    }
    
    printf("Ok\n");
    return 0;
}