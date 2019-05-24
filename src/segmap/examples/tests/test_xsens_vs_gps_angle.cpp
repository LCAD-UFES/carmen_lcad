
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_conversions.h>

using namespace pcl;
using namespace Eigen;


int 
main(int argc, char **argv)
{
	DataSample *sample;
	NewCarmenDataset dataset("/dados/log-estacionamento-ambiental-20181208.txt",
													 "../../../carmen-ford-escape.ini");

	//Matrix<double, 4, 4> pose;
	//PointCloud<PointXYZRGB>::Ptr cloud;
	//pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("CloudViewer");
	//viewer->setBackgroundColor(1, 1, 1);

	double yaw, pitch, roll;

	for (int i = 0; i < dataset.size(); i++)
	{
		sample = dataset[i];

		if (i>0) //fabs(sample->v) > 1.0)
		{
			double gps_angle = atan2(sample->gps.y - dataset[i-1]->gps.y,
															 sample->gps.x - dataset[i-1]->gps.x);

			Matrix<double, 3, 3> mat = sample->xsens.toRotationMatrix();
			getEulerYPR(mat, yaw, pitch, roll);

			printf("gps_angle: %lf xsens_yaw: %lf\n",
						 radians_to_degrees(gps_angle),
						 radians_to_degrees(yaw) + 25);
		}

		//Eigen::Affine3f rotation(sample->xsens);
		//viewer->addCoordinateSystem(2., rotation);
		//viewer->addPointCloud(cloud);
		//viewer->removeAllCoordinateSystems();
		//viewer->removeAllPointClouds();
	}

	return 0;
}
