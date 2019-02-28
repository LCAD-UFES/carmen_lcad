
#include <cstdio>
#include <cstring>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <carmen/segmap_dataset.h>


using namespace cv;
using namespace pcl;


void
save_data(PointCloud<PointXYZRGB>::Ptr cloud, Mat &img, int sample_id)
{
    char path[256];
    
    sprintf(path, "/dados/calibration_data/bb3/img%04d.png", sample_id);
    imwrite(path, img);

	sprintf(path, "/dados/calibration_data/velodyne/cloud%04d.txt", sample_id);
	FILE *f = fopen(path, "w");

	fprintf(f, "%ld\n", cloud->size());
	for (int i = 0; i < cloud->size(); i++)
	{
		fprintf(f, "%lf %lf %lf %d\n", 
			cloud->at(i).x, 
			cloud->at(i).y, 
			cloud->at(i).z, 
			cloud->at(i).r
		);
	}

	fclose(f);
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Error: Use %s <log data directory>\n", argv[0]));

	DatasetInterface *dataset;
    dataset = new DatasetCarmen(argv[1], 0);

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	int sample_id = 1;
    int step = dataset->data.size() / 50;
	for (int i = 0; i < dataset->data.size(); i += step)
	{
		cloud->clear();
		dataset->load_pointcloud(i, cloud, dataset->data[i].v, dataset->data[i].phi);
        Mat img = dataset->load_image(i);
        save_data(cloud, img, sample_id);
		sample_id++;
	}

	printf("Done\n");
	return 0;
}



