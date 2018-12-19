
#include <carmen/carmen.h>
#include <iostream>
#include <deque>
#include <vector>
#include <cmath>
#include "g2o/types/slam2d/se2.h"
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <pcl/io/ply_io.h>
#include <carmen/segmap_dataset.h>

using namespace std;
using namespace g2o;
using namespace pcl;
using namespace Eigen;
using namespace cv;


const double distance_between_front_and_rear_axles = 2.625;

class SyncDataSample
{
public:
	char cloud_path[256];
	int n_rays;
	double cloud_time;

	char image_path[256];
	int w, h, size, n;
	double image_time;

	double x, y, quality, gps_time;
	double angle, angle_quality, gps2_time;

	double v, phi, ack_time;
};


class OdomData
{
public:
	double mult_v, mult_phi, add_phi, init_angle;
};


class Data
{
public:
	string dirname;
	vector<SyncDataSample> sync;
	vector<SE2> dead_reckoning;
	vector<int> vertices_ids;
	OdomData odom;

	void create_dead_reckoning()
	{
		double x, y, ds, th, dt, v, phi;

		x = y = th = 0.;
		th = odom.init_angle;
		dead_reckoning.push_back(SE2(x, y, th));

		for (size_t i = 1; i < sync.size(); i++)
		{
			dt = sync[i].ack_time - sync[i - 1].ack_time;
			v = sync[i - 1].v * odom.mult_v;
			phi = sync[i - 1].phi * odom.mult_phi + odom.add_phi;

			ds = v * dt;
			x += ds * cos(th);
			y += ds * sin(th);
			th += ds * tan(phi) / distance_between_front_and_rear_axles;
			th = carmen_normalize_theta(th);

			dead_reckoning.push_back(SE2(x, y, th));
		}
	}

	void load_odom_calib()
	{
		string name = dirname + "/odom_calib_stderr.txt";
		FILE *f = fopen(name.c_str(), "r");

		if (f == NULL)
			exit(printf("Error: file '%s' not found.\n", name.c_str()));

		char dummy[64];

		fscanf(f, "%s %s %lf %s %s %s %lf %lf %s %s %lf",
			dummy, dummy, &odom.mult_v, dummy, dummy, dummy, 
			&odom.mult_phi, &odom.add_phi, dummy, dummy, 
			&odom.init_angle);

        //odom.mult_v = 1.0;
        //odom.add_phi = 0.0;

		printf("Odom data: %lf %lf %lf %lf\n", 
			odom.mult_v, odom.mult_phi, odom.add_phi, odom.init_angle);
	}

	void load_sync()
	{
		string name = dirname + "/sync.txt";
		FILE *f = fopen(name.c_str(), "r");

		if (f == NULL)
			exit(printf("Error: file '%s' not found.\n", name.c_str()));

		char dummy[256];
		int idummy;

		while (!feof(f))
		{
			SyncDataSample d;

			char c = fgetc(f);
			if (c != 'V')
			{
				printf("skipping char %c at line %ld\n", c, sync.size()+1);
				continue;
			}

			fscanf(f, "\n%s %s %d %lf ",
					dummy, d.cloud_path, &d.n_rays, &d.cloud_time);

			fscanf(f, " %s %s %d %d %d %d %lf ",
					dummy, d.image_path, &d.w, &d.h, &d.size, &d.n, &d.image_time);

			fscanf(f, " %s %d %lf %lf %lf %lf ",
				dummy, &idummy, &d.x, &d.y, &d.quality, &d.gps_time);

			fscanf(f, " %s %d %lf %lf %lf ",
				dummy,  &idummy, &d.angle, &d.angle_quality, &d.gps2_time);

			fscanf(f, " %s %lf %lf %lf\n",
				dummy, &d.v, &d.phi, &d.ack_time);

			sync.push_back(d);
		}

		printf("N lines in sync file: %ld\n", sync.size());
		fclose(f);
	}

	Data(char *param_dirname)
	{
		dirname = string(param_dirname);

		printf("Loading data from '%s'\n", param_dirname);

		load_sync();
		load_odom_calib();
		create_dead_reckoning();

		printf("Load done.\n");
	}
};


void
load_pointcloud(char *dir_name, double time, PointCloud<PointXYZRGB>::Ptr cloud)
{
	PointCloud<PointXYZRGB>::Ptr raw_cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);

	raw_cloud->clear();
	cloud->clear();

	char name[256];
	sprintf(name, "%s/velodyne/%lf.ply", dir_name, time);
	int success = pcl::io::loadPLYFile(name, *raw_cloud);

	// /*
	for (int i = 0; i < raw_cloud->size(); i++)
	{
		if (fabs(raw_cloud->at(i).x) > 5.0 || fabs(raw_cloud->at(i).y) > 2.0) // || raw_cloud->at(i).z < 0.))
			cloud->push_back(raw_cloud->at(i));
	}
	// */
	// copyPointCloud(*raw_cloud, *cloud);

	if (success < 0 || cloud->size() <= 0)
		exit(printf("Could not load pointcloud!\n"));
}


Matrix<double, 4, 4>
create_transformation_matrix(double x, double y, double th)
{
	Matrix<double, 4, 4> m;

	m << cos(th), -sin(th), 0, x,
			sin(th), cos(th), 0, y,
			0, 0, 1, 0,
			0, 0, 0, 1;

	return m;
}


void
run_gicp(PointCloud<PointXYZRGB>::Ptr source, PointCloud<PointXYZRGB>::Ptr target, Matrix<double, 4, 4> *correction, int *converged, PointCloud<PointXYZRGB>::Ptr output)
{
	static GeneralizedIterativeClosestPoint<PointXYZRGB, PointXYZRGB> *gicp = NULL;
	static pcl::VoxelGrid<pcl::PointXYZRGB> *grid = NULL;
	static PointCloud<PointXYZRGB>::Ptr source_leafed(new PointCloud<PointXYZRGB>);
	static PointCloud<PointXYZRGB>::Ptr target_leafed(new PointCloud<PointXYZRGB>);

	if (gicp == NULL)
	{
		gicp = new GeneralizedIterativeClosestPoint<PointXYZRGB, PointXYZRGB>();
		gicp->setMaximumIterations(200);
		gicp->setTransformationEpsilon(1e-5);
		gicp->setMaxCorrespondenceDistance(5.0);

		const double leaf_size = 0.3;
		grid = new pcl::VoxelGrid<pcl::PointXYZRGB>();
		grid->setLeafSize(leaf_size, leaf_size, leaf_size);
	}

	grid->setInputCloud(source);
	grid->filter(*source_leafed);

	grid->setInputCloud(target);
	grid->filter(*target_leafed);

	gicp->setInputCloud(source_leafed);
	gicp->setInputTarget(target_leafed);
	gicp->align(*output);

	*correction = gicp->getFinalTransformation().cast<double>();
	*converged = gicp->hasConverged();
}


double theta_from_matrix(Matrix<double, 4, 4> &m)
{
	// extract rotation matrix
	static Matrix<double, 3, 3> R;
	R << m(0, 0), m(0, 1), m(0, 2),
			m(1, 0), m(1, 1), m(1, 2),
			m(2, 0), m(2, 1), m(2, 2);

	// Important:
	// Extracting the yaw component from the rotation matrix is not
	// the right wayt of computing theta. Note that a rotation of yaw=pitch=roll=0
	// is equivalent to a rotation of yaw=pitch=roll=3.14 (in this order), but
	// yaw=0. is the opposite of yaw=3.14.
	// Matrix<double, 3, 1> ypr = R.eulerAngles(2, 1, 0);

	// Here I'm using the ad-hoc approach of rotating an unitary vector
	// and computing theta using the x and y coordinates. TODO: find a principled approach.
	static Matrix<double, 3, 1> unit;
	unit << 1, 0, 0;
	unit = R * unit;

	return atan2(unit(1, 0), unit(0, 0));
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 2)
		exit(printf("Use %s <data-directories>\n", argv[0]));

	DatasetCarmen dataset(argv[1], 0);

	PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr target(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr aligned(new PointCloud<PointXYZRGB>);

    deque<string> clouds_on_viewer;
	pcl::visualization::PCLVisualizer viewer("CloudViewer");
	//pcl::visualization::PCLVisualizer viewer2("CloudViewer2");
	//viewer2.setBackgroundColor(1, 1, 1);

	viewer.setBackgroundColor(0, 0, 1);
	viewer.removeAllPointClouds();
	viewer.addCoordinateSystem(1.0);

	char cloud_name[64];
	int pause_viewer = 1;

	Matrix<double, 4, 4> target_pose;
	Matrix<double, 4, 4> source_pose;
	Matrix<double, 4, 4> correction;
	int converged;

	//load_pointcloud(argv[1], data[0].sync[0].cloud_time, target);
    dataset.load_fused_pointcloud_and_camera(0, target, 1);
	target_pose = create_transformation_matrix(0, 0, 0);

	//viewer.addPointCloud(target, "cloud0");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud0");

    double time_last_icp = 0;

	//for (int i = 1; i < data[0].sync.size(); i++)
    for (int i = 1; i < dataset.data.size(); i++)
	{
		if (fabs(dataset.data[i].v) < 0.2 || dataset.data[i].velodyne_time - time_last_icp < 0.05) // || fabs(data[0].sync[i].phi) < carmen_degrees_to_radians(20.))
			continue;

        time_last_icp = dataset.data[i].velodyne_time;

		source->clear();
		aligned->clear();
		//load_pointcloud(argv[1], data[0].sync[i].cloud_time, source);
        dataset.load_fused_pointcloud_and_camera(i, source, 1);
		run_gicp(source, target, &correction, &converged, aligned);

		printf("Step: %d Converged: %d Correction: %lf %lf %lf %lf\n", 
            i, converged,
            correction(0, 3) / correction(3, 3),
            correction(1, 3) / correction(3, 3),
            correction(2, 3) / correction(3, 3),
            theta_from_matrix(correction)
        );

/*
		viewer2.removeAllPointClouds();

		viewer2.addPointCloud(source, "source");
		viewer2.addPointCloud(target, "target");
		viewer2.addPointCloud(aligned, "transformed");
		
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");

		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source");
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target");
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "transformed");
*/

		target->clear();
        copyPointCloud(*source, *target);

		if (converged)
		{
			/*
            correction = create_transformation_matrix(
                correction(0, 3) / correction(3, 3),
                correction(1, 3) / correction(3, 3),
                theta_from_matrix(correction)
            );
			*/

			target_pose = target_pose * correction;
			sprintf(cloud_name, "cloud%d", i);

	        pcl::transformPointCloud(
		        *source, 
		        *aligned, 
		        target_pose);

			// /*
            for (int j = 0; j < aligned->size(); j++)
            {
                // int b = ((aligned->at(j).z + 5.0) / 10.) * 255;
                // if (b < 0) b = 0;
                // if (b > 255) b = 255;
                int mult = 2;

				int color = mult * (int) aligned->at(j).r;
				if (color > 255) color = 255;
				else if (color < 0) color = 0;
			
                aligned->at(j).r = (unsigned char) color;

				color = mult * (int) aligned->at(j).g;
				if (color > 255) color = 255;
				else if (color < 0) color = 0;

                aligned->at(j).g = (unsigned char) color;

				color = mult * (int) aligned->at(j).b;
				if (color > 255) color = 255;
				else if (color < 0) color = 0;

                aligned->at(j).b = (unsigned char) color;
            }
			// */

            clouds_on_viewer.push_back(string(cloud_name));

            if (clouds_on_viewer.size() > 100)
            {
                viewer.removePointCloud(clouds_on_viewer[0]);
                clouds_on_viewer.pop_front();
            }
    
			viewer.addPointCloud(aligned, cloud_name);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
			printf("target pose: %lf %lf %lf\n", 
				target_pose(0, 3) / target_pose(3, 3), 
				target_pose(1, 3) / target_pose(3, 3), 
				theta_from_matrix(target_pose));

			//Matrix4f target_pose_float = target_pose.cast<float>();
			Eigen::Affine3f affine;
			affine = target_pose.cast<float>();
            viewer.addCoordinateSystem(1., affine);
		}

		imshow("viewer", Mat::zeros(300, 300, CV_8UC3));

		char c = ' ';
		while (1)
		{
			//viewer2.spinOnce();
			viewer.spinOnce();
			c = waitKey(5);

			if (c == 's')
				pause_viewer = !pause_viewer;

			if (!pause_viewer || (pause_viewer && c == 'n'))
				break;
		} 
	}
	
	return 0;
}
