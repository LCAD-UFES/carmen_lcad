
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_dataset.h>


int 
main(int argc, char **argv)
{
    if (argc < 2)
        exit(printf("Error: Use %s <log_dir>\n", argv[0]));

    DataSample *sample;
    NewCarmenDataset dataset(argv[1]);

    Matrix<double, 4, 4> pose;
    PointCloud<PointXYZRGB>::Ptr cloud;
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
	viewer->setBackgroundColor(.5, .5, .5);

    while (sample = dataset.next_data_package())
    {
        Eigen::Affine3f rotation(sample->xsens);
        cloud = NewCarmenDataset::read_pointcloud(sample);
        viewer->addCoordinateSystem(2., rotation);
        viewer->addPointCloud(cloud);
        viewer->removeAllCoordinateSystems();
        viewer->removeAllPointClouds();
    }

    return 0;
}
