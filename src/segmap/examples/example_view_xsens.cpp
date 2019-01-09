
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_dataset.h>

int 
main(int argc, char **argv)
{
    if (argc < 3)
        exit(printf("Error: Use %s <dataset_dir> <transforms_file_graphslam_format>\n", argv[0]));

    DatasetCarmen dataset(argv[1], 0);

    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
	viewer->setBackgroundColor(.5, .5, .5);

    for (int i = 0; i < dataset.data.size(); i++)
    {
        
    }

    return 0;
}
