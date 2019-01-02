

#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cfloat>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <pcl/io/ply_io.h>


using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;


void
find_loop_closure_poses(vector<Data> &lines, vector<pair<int, int>> &loop_closure_indices)
{
	for (uint i = 0; i < lines.size(); i++)
	{
        double last_t = 0;

		for (uint j = i + 1; j < lines.size(); j++)
		{
			double delta_x = lines[i].x - lines[j].x;
			double delta_y = lines[i].y - lines[j].y;
			double delta_t = lines[i].gps_time - lines[j].gps_time;

			double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

			if ((dist > 3.0 && dist < 10.0 && fabs(lines[j].gps_time - last_t) > 10.)) // || (dist > 10.0 && fabs(delta_t) > 20.))
            {
				loop_closure_indices.push_back(pair<int, int>(i, j));
                last_t = lines[j].gps_time;
            }
		}
	}

	printf("Num loop closures: %ld\n", loop_closure_indices.size());
}


void
load_pointcloud(Data &d, PointCloud<PointXYZRGB>::Ptr cloud, char *dir_name)
{
	char name[256];
	sprintf(name, "%s/velodyne/%lf.ply", dir_name, d.cloud_time);
	int success = pcl::io::loadPLYFile(name, *cloud);

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
run_gicp(vector<Data> &lines, vector<pair<int, int>> &loop_closure_indices, vector<Matrix<double, 4, 4>> &transforms, vector<bool> &converged, char *dir_name)
{
	int i;

	GeneralizedIterativeClosestPoint<PointXYZRGB, PointXYZRGB> gicp;
	gicp.setMaximumIterations(200);
	gicp.setTransformationEpsilon(1e-3);
	gicp.setMaxCorrespondenceDistance(20.0);

    /*
	visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(.5, .5, .5);
	viewer.removeAllPointClouds();
    */

    int n_processed_clouds = 0;

	#pragma omp parallel for default(none) shared(lines, dir_name, n_processed_clouds, transforms, converged, loop_closure_indices) private(i, gicp)
	for (i = 0; i < loop_closure_indices.size(); i++)
	{
		PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>), target(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr source_world(new PointCloud<PointXYZRGB>), target_world(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr source_leafed(new PointCloud<PointXYZRGB>), target_leafed(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr output(new PointCloud<PointXYZRGB>), output_transformed(new PointCloud<PointXYZRGB>);

	    const double leaf_size = 0.25;
        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        grid.setLeafSize(leaf_size, leaf_size, leaf_size);

		Data source_data = lines[loop_closure_indices[i].first];
		Data target_data = lines[loop_closure_indices[i].second];

		load_pointcloud(source_data, source, dir_name);
		load_pointcloud(target_data, target, dir_name);

	    grid.setInputCloud(source);
	    grid.filter(*source_leafed);

	    grid.setInputCloud(target);
	    grid.filter(*target_leafed);

	    Matrix<double, 4, 4> source_t = create_transformation_matrix(source_data.x - target_data.x,
	    		source_data.y - target_data.y, source_data.angle);
	    Matrix<double, 4, 4> target_t = create_transformation_matrix(0., 0., target_data.angle);
	    Matrix<double, 4, 4> gicp_correction;

	    pcl::transformPointCloud(*source_leafed, *source_world, (target_t.inverse() * source_t).cast<float>());

	    gicp.setInputCloud(source_world);
	    gicp.setInputTarget(target_leafed);
	    gicp.align(*output);
	    gicp_correction = gicp.getFinalTransformation().cast<double>();

        /*
	    //output_transformed = output;
	    //output_transformed = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>(*target_leafed));
	    pcl::transformPointCloud(*source_leafed, *output_transformed, (gicp_correction * target_t.inverse() * source_t).cast<float>());
	    //	// save_clouds_for_debug(*source_pointcloud, *target_pointcloud, out_pcl_pointcloud_transformed);
	    //for (int k = 0; k < output_transformed->size(); k++)
	    //{
	    	//output_transformed->at(k).g = 255;
	    //}

		viewer.removeAllPointClouds();
		viewer.addPointCloud(source_world, "source");
		viewer.addPointCloud(target_leafed, "target");
		viewer.addPointCloud(output_transformed, "transformed");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");

		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "transformed");

		char c = ' ';
		while (c != 'n')
		{
			imshow("map", Mat::zeros(300, 300, CV_8UC3));
			viewer.spinOnce();
			c = waitKey(5);
		}
        */
		transforms[i] = (gicp_correction * target_t.inverse() * source_t).inverse();
		converged[i] = gicp.hasConverged();

        #pragma omp critical
        {
            n_processed_clouds++;

            if (n_processed_clouds % 100 == 0)
    	    	printf("%d processed clouds of %ld\n", n_processed_clouds, loop_closure_indices.size());
        }
	}
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


void
write_output(char *filename, vector<pair<int, int>> &loop_closure_indices, vector<Matrix<double, 4, 4>> &transforms, vector<bool> &converged)
{
	FILE *f = fopen(filename, "w");

	for (unsigned int i = 0; i < loop_closure_indices.size(); i++)
	{
		double x, y, th;

		x = transforms[i](0, 3) / transforms[i](3, 3);
		y = transforms[i](1, 3) / transforms[i](3, 3);
		th = theta_from_matrix(transforms[i]);

		fprintf(f, "%d %d %d %lf %lf %lf\n",
				loop_closure_indices[i].first, loop_closure_indices[i].second,
				(int) converged[i], x, y, th);
	}

	fclose(f);
}


int
main(int argc, char **argv)
{
	if (argc < 3)
	{
		printf("\nError: Use %s <data_directory> <output_file>\n\n", argv[0]);
		exit(0);
	}

	vector<Data> lines;
	vector<pair<int, int>> loop_closure_indices;

	load_data(argv[1], lines);
	find_loop_closure_poses(lines, loop_closure_indices);

	if (loop_closure_indices.size() > 0)
	{
		vector<Matrix<double, 4, 4>> transforms(loop_closure_indices.size());
		vector<bool> converged(loop_closure_indices.size());
		run_gicp(lines, loop_closure_indices, transforms, converged, argv[1]);
		write_output(argv[2], loop_closure_indices, transforms, converged);
	}

	return 0;
}

