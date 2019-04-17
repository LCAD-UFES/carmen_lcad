
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <deque>
#include <string>
#include <random>
#include <iostream>
#include <Eigen/Geometry>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "libsegmap/segmap_car_config.h"
#include "libsegmap/segmap_grid_map.h"
#include "libsegmap/segmap_particle_filter.h"
#include "libsegmap/segmap_pose2d.h"
#include "libsegmap/segmap_util.h"
#include "libsegmap/segmap_dataset.h"
#include "libsegmap/segmap_viewer.h"

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;


void
increase_bightness(PointCloud<PointXYZRGB>::Ptr aligned)
{
	// /*
	for (int j = 0; j < aligned->size(); j++)
	{
		// int b = ((aligned->at(j).z + 5.0) / 10.) * 255;
		// if (b < 0) b = 0;
		// if (b > 255) b = 255;
		int mult = 3;

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
}


int
interpolate_cell_value_using_kernel(PointXYZRGB &p, GridMap &map, double i, double j, int n_kernel_cells)
{
    int ki, kj;

    double color = 0;
    int n = 0;

    for (ki = 0; ki < n_kernel_cells; ki++)
    {
        for (kj = 0; kj <= n_kernel_cells; kj++)
        {
            PointXYZRGB kp;
            kp.y = map.yo + i + (ki - n_kernel_cells / 2.) * map.m_by_pixels;
            kp.x = map.xo + j + (kj - n_kernel_cells / 2.) * map.m_by_pixels;

            vector<double> data = map.read_cell(kp);

            if (data[3] > 1.)
            {
                color += data[0];
                n += 1;
            }
        }
    }

    if (n > 2)
    {
        color /= (double) n;
        p.r = p.g = p.b = (unsigned char) color;
        return 1;
    }
    else
        return 0;
}


void
fill_map_using_polygons(GridMap &map)
{
    double i = 0, j = 0;
    int k;

    vector<PointXYZRGB> points_to_add;

    for (i = map.m_by_pixels / 2.; i < map.height_meters; i += map.m_by_pixels)
    {
        for (j = map.m_by_pixels / 2.; j < map.width_meters; j += map.m_by_pixels)
        {
            PointXYZRGB p;
            p.y = map.yo + i;
            p.x = map.xo + j;

            vector<double> data = map.read_cell(p);

            if (data[3] == 1.) // cell was not observed
            {
                if (interpolate_cell_value_using_kernel(p, map, i, j, 3))
                    points_to_add.push_back(p);
            }
        }
    }

    for (k = 0; k < points_to_add.size(); k++)
        map.add_point(points_to_add[k]);
}


#define VIEW 1

void
create_map(GridMap &map, DatasetInterface &dataset, char path_save_maps[])
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud2(new PointCloud<PointXYZRGB>);

#if VIEW
	int pause_viewer = 1;

	pcl::visualization::PCLVisualizer viewer("CloudViewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.removeAllPointClouds();
	viewer.addCoordinateSystem(2);
#endif

	Matrix<double, 4, 4> vel2car = dataset.transform_vel2car();
	Mat img_view;
	char map_name[512];

	deque<string> cloud_names;
	int step = 1;

	Matrix<double, 3, 3> rot3d;
	Matrix<double, 4, 4> transf;

	for (int i = 0; i < dataset.data.size(); i += step)
	{
		if (fabs(dataset.data[i].v) < 0.1)
			continue;

		Pose2d pose = dataset.data[i].pose;

		cloud->clear();
		transformed_cloud->clear();
		dataset.load_fused_pointcloud_and_camera(i, cloud, dataset.data[i].v, dataset.data[i].phi, 1, &img_view);
		//pose.x = pose.y = 0.;
		//pose.th = normalize_theta(-pose.th - degrees_to_radians(18));
		
		/*
		double roll, pitch, yaw;
		Vector3d euler = dataset.data[i].xsens.toRotationMatrix().eulerAngles(2, 1, 0);
		yaw = euler[0]; pitch = euler[1]; roll = euler[2];

		yaw = normalize_theta(-yaw - degrees_to_radians(18));

		Matrix<double, 4, 4> mat = pose6d_to_matrix(dataset.data[i].gps.x, dataset.data[i].gps.y, 0., roll, pitch, yaw);
		Pose2d gps_imu_pose(dataset.data[i].gps.x, dataset.data[i].gps.y, yaw);
		//Matrix<double, 4, 4> mat = pose6d_to_matrix(pose.x, pose.y, 0, 0, 0, pose.th);

		Matrix<double, 4, 4> mat, correction;
		Matrix<double, 3, 3> rot = dataset.data[i].xsens.toRotationMatrix();
		mat << rot(0, 0), rot(0, 1), rot(0, 2), dataset.data[i].gps.x, 
			rot(1, 0), rot(1, 1), rot(1, 2), dataset.data[i].gps.y, 
			rot(2, 0), rot(2, 1), rot(2, 2), 0, 
			0, 0, 0, 1;

		correction = pose6d_to_matrix(0, 0, 0, 0, 0, degrees_to_radians(18.));
		mat = mat * correction;
		*/
        Matrix<double, 4, 4> mat = pose6d_to_matrix(pose.x, pose.y, 0, 0, 0, pose.th);
		pcl::transformPointCloud(*cloud, *transformed_cloud, mat);

		map.reload(pose.x, pose.y);

		for (int j = 0; j < transformed_cloud->size(); j++)
            map.add_point(transformed_cloud->at(j));

        fill_map_using_polygons(map);

		Mat map_img = map.to_image().clone();
		draw_pose(map, map_img, pose, Scalar(0, 255, 0));

		Mat concat;
		hconcat(map_img, img_view, concat);
		//sprintf(map_name, "%s/step_%010d.png", path_save_maps, i);
		//imwrite(map_name, concat);

#if VIEW

		/*
		rot3d = dataset.data[i].xsens;
		transf << rot3d(0, 0), rot3d(0, 1), rot3d(0, 2), 0,
			rot3d(1, 0), rot3d(1, 1), rot3d(1, 2), 0,
			rot3d(2, 0), rot3d(2, 1), rot3d(2, 2), 0,
			0, 0, 0, 1;
		pcl::transformPointCloud(*cloud, *transformed_cloud2, transf);
		*/

		if (map._map_type == GridMapTile::TYPE_SEMANTIC)
			colorize_cloud_according_to_segmentation(transformed_cloud);
		increase_bightness(transformed_cloud);

        ///*
		char *cloud_name = (char *) calloc (32, sizeof(char));
		sprintf(cloud_name, "cloud%d", i);

		//viewer.removeAllPointClouds();
		//for (int j = 0; j < transformed_cloud->size(); j++)
		    //transformed_cloud->at(j).z = 0.;
		
		viewer.addPointCloud(transformed_cloud, cloud_name);
		//viewer.addPointCloud(transformed_cloud2, "xsens");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "xsens");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "xsens");
		//cloud_names.push_back(cloud_name);

		//if (cloud_names.size() >= 300)
		//{
		//	viewer.removePointCloud(cloud_names[0]);
		//	cloud_names.pop_front();
		//}
		//*/

		//view(pf, map, dataset.data[i].pose, cloud, transformed_cloud, &vel2car, dataset.data[i].v, dataset.data[i].phi);
		
		imshow("concat", concat);

		char c = ' ';
		while (1)
		{
			viewer.spinOnce();
			c = waitKey(5);

			if (c == 's')
				pause_viewer = !pause_viewer;
			if (!pause_viewer || (pause_viewer && c == 'n'))

				break;
			if (c == 'r')
			{
				printf("Reinitializing\n");
				i = 0;
			}
			if (c == 'f')
				step *= 2;
			if (c == 'g')
			{
				step /= 2;
				if (step < 1) step = 1;
			}
		} 
		//*/
#endif 

//		if (i > 500 && i < dataset.data.size() - 1000)
//			i = dataset.data.size() - 1000;
	}
	
	/*
	for (int i = 0; i < 500; i++)
	{
		if (fabs(dataset.data[i].v) < 0.1)
			continue;

		Pose2d pose = dataset.data[i].pose;

		cloud->clear();
		transformed_cloud->clear();
		dataset.load_fused_pointcloud_and_camera(i, cloud, dataset.data[i].v, dataset.data[i].phi, 1);
		pcl::transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(pose));
		
		for (int j = 0; j < transformed_cloud->size(); j++)
		    transformed_cloud->at(j).z += .7;
		
        viewer.removePointCloud("bola");
		viewer.addPointCloud(transformed_cloud, "bola");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "bola");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.5, 0, "bola");
        
        char c = ' ';
		while (1)
		{
			viewer.spinOnce();
			c = waitKey(5);

			if (c == 's')
				pause_viewer = !pause_viewer;
			if (!pause_viewer || (pause_viewer && c == 'n'))

				break;
			if (c == 'r')
			{
				printf("Reinitializing\n");
				i = 0;
			}
			if (c == 'f')
				step *= 2;
			if (c == 'g')
			{
				step /= 2;
				if (step < 1) step = 1;
			}
		} 
	}
	*/

	//waitKey(-1);
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Error: Use %s <log data directory>\n", argv[0]));

	char path_save_maps[256];
	char dataset_name[256];
	char map_name[256];

	sprintf(dataset_name, "/dados/data/%s", argv[1]);
	sprintf(map_name, "/dados/maps/map_%s", argv[1]);
	sprintf(path_save_maps, "/dados/map_imgs/%s", argv[1]);

	char cmd[256];
	sprintf(cmd, "rm -rf %s && mkdir %s", path_save_maps, path_save_maps);
	system(cmd);

	sprintf(cmd, "rm -rf %s && mkdir %s", map_name, map_name);
	system(cmd);

	printf("dataset_name: %s\n", dataset_name);
	printf("map_name: %s\n", map_name);
	printf("path to save maps: %s\n", path_save_maps);

	DatasetInterface *dataset;
    dataset = new DatasetCarmen(dataset_name, 0);
	GridMap map(map_name, 50., 50., 0.2, GridMapTile::TYPE_VISUAL, 0);
	create_map(map, *dataset, path_save_maps);

	printf("Done\n");
	return 0;
}


