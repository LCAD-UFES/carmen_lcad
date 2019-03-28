
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

#include <carmen/segmap_dataset.h>
#include <carmen/segmap_dataset_old.h>

#include <carmen/segmap_grid_map.h>

#include <carmen/carmen_lidar_reader.h>
#include <carmen/carmen_semantic_segmentation_reader.h>
#include <carmen/carmen_image_reader.h>

#include <carmen/lidar_shot.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_particle_filter_viewer.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>

#include <carmen/command_line.h>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;

#define VIEW 1
#define USE_NEW 1

enum GridMapType
{
	MAP_REMISSION,
	MAP_VISUAL,
	MAP_SEMANTIC,
};


void
get_pixel_position(double x, double y, double z, Matrix<double, 4, 4> &lidar2cam,
                   Matrix<double, 3, 4> &projection, cv::Mat &img, cv::Point *ppixel, int *is_valid)
{
	static Matrix<double, 4, 1> plidar, pcam;
	static Matrix<double, 3, 1> ppixelh;

	*is_valid = 0;
	//plidar << x, y, z, 1.;
	plidar(0, 0) = x;
	plidar(1, 0) = y;
	plidar(2, 0) = z;
	plidar(3, 0) = 1.;

	pcam = lidar2cam * plidar;

	// test to check if the point is in front of the camera.
	// points behind the camera can also be projected into the image plan.
	if (pcam(0, 0) / pcam(3, 0) > 0)
	{
		ppixelh = projection * pcam;

		ppixel->y = (ppixelh(1, 0) / ppixelh(2, 0)) * img.rows;
		ppixel->x = (ppixelh(0, 0) / ppixelh(2, 0)) * img.cols;

		// check if the point is visible by the camera.
		if (ppixel->x >= 0 && ppixel->x < img.cols && ppixel->y >= 0 && ppixel->y < img.rows)
			*is_valid = 1;
	}
}


PointCloud<PointXYZRGB>::Ptr
filter_pointcloud(PointCloud<PointXYZRGB>::Ptr raw_cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);
	cloud->clear();

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		double range = sqrt(pow(raw_cloud->at(i).x, 2) + pow(raw_cloud->at(i).y, 2));

		if (((fabs(raw_cloud->at(i).x) > 6.0) || (fabs(raw_cloud->at(i).y) > 4.0)) // remove rays that hit car
		    && (range < 70.0)  // remove max range
		    && (range > 4.0)  // remove max range
		    //&& raw_cloud->at(i).z > -2.5  // remove points bellow ground
		    //&& raw_cloud->at(i).z < -0.  // remove tree tops
		    //&& (raw_cloud->at(i).x != 0 && raw_cloud->at(i).y != 0)
		    //&& (!isnan(raw_cloud->at(i).x) && !isnan(raw_cloud->at(i).y) && !isnan(raw_cloud->at(i).z) && !isnan(raw_cloud->at(i).r))
		    //&& (!isinf(raw_cloud->at(i).x) && !isinf(raw_cloud->at(i).y) && !isinf(raw_cloud->at(i).z) && !isinf(raw_cloud->at(i).r))
		     )
		{
			//printf("%lf %lf %lf\n", raw_cloud->at(i).x, raw_cloud->at(i).y, raw_cloud->at(i).z);
			cloud->push_back(PointXYZRGB(raw_cloud->at(i)));
		}
	}

	return cloud;
}


unsigned char
brighten(unsigned char val, unsigned int multiplier = 5)
{
	unsigned int brightened = val * multiplier;
	if (brightened > 255) return 255;
	else return brightened;
}


void
increase_brightness(PointCloud<PointXYZRGB>::Ptr cloud, int mult = 3)
{
	// /*
	for (int j = 0; j < cloud->size(); j++)
	{
		// int b = ((aligned->at(j).z + 5.0) / 10.) * 255;
		// if (b < 0) b = 0;
		// if (b > 255) b = 255;
		int color = mult * (int) cloud->at(j).r;
		if (color > 255)
			color = 255;
		else if (color < 0)
			color = 0;

		cloud->at(j).r = (unsigned char) color;

		color = mult * (int) cloud->at(j).g;
		if (color > 255)
			color = 255;
		else if (color < 0)
			color = 0;

		cloud->at(j).g = (unsigned char) color;

		color = mult * (int) cloud->at(j).b;
		if (color > 255)
			color = 255;
		else if (color < 0)
			color = 0;

		cloud->at(j).b = (unsigned char) color;
	}
	// */
}


void
fuse_cam_and_lidar(PointCloud<PointXYZRGB>::Ptr cloud, Matrix<double, 4, 4> &lidar2cam,
					Matrix<double, 3, 4> &projection, Mat &img,
					PointCloud<PointXYZRGB>::Ptr colored)
{
	Mat orig = img.clone();
	Point ppixel;
	int is_valid;

	colored->clear();

	for (int i = 0; i < cloud->size(); i++)
	{
		get_pixel_position(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z,
												lidar2cam, projection, img, &ppixel, &is_valid);

		if (is_valid)
		{
			circle(img, ppixel, 2, Scalar(0, 0, 255), -1);

			PointXYZRGB point = cloud->at(i);
			point.r = orig.data[3 * (ppixel.y * orig.cols + ppixel.x) + 2];
			point.g = orig.data[3 * (ppixel.y * orig.cols + ppixel.x) + 1];
			point.b = orig.data[3 * (ppixel.y * orig.cols + ppixel.x) + 0];
			colored->push_back(point);
		}
	}

	imshow("fused_img", img);
}


Matrix<double, 3, 3>
move_xsens_to_car(Matrix<double, 3, 3> xsens, Matrix<double, 4, 4> xsens2car)
{
	// static to prevent reallocation.
	static Matrix<double, 4, 4> xsens4x4;
	static Matrix<double, 3, 3> xsens_car;

	xsens4x4(0, 0) = xsens(0, 0);
	xsens4x4(0, 1) = xsens(0, 1);
	xsens4x4(0, 2) = xsens(0, 2);
	xsens4x4(0, 3) = 0;
	xsens4x4(1, 0) = xsens(1, 0);
	xsens4x4(1, 1) = xsens(1, 1);
	xsens4x4(1, 2) = xsens(1, 2);
	xsens4x4(1, 3) = 0;
	xsens4x4(2, 0) = xsens(2, 0);
	xsens4x4(2, 1) = xsens(2, 1);
	xsens4x4(2, 2) = xsens(2, 2);
	xsens4x4(2, 3) = 0;
	xsens4x4(3, 0) = 0;
	xsens4x4(3, 1) = 0;
	xsens4x4(3, 2) = 0;
	xsens4x4(3, 3) = 1;

	xsens4x4 = xsens2car * xsens4x4;

	xsens_car(0, 0) = xsens4x4(0, 0);
	xsens_car(0, 1) = xsens4x4(0, 1);
	xsens_car(0, 2) = xsens4x4(0, 2);
	xsens_car(1, 0) = xsens4x4(1, 0);
	xsens_car(1, 1) = xsens4x4(1, 1);
	xsens_car(1, 2) = xsens4x4(1, 2);
	xsens_car(2, 0) = xsens4x4(2, 0);
	xsens_car(2, 1) = xsens4x4(2, 1);
	xsens_car(2, 2) = xsens4x4(2, 2);

	return xsens_car;
}


void
transform_cloud(DataSample *sample, Pose2d &pose,
                PointCloud<PointXYZRGB>::Ptr cloud,
                Matrix<double, 4, 4> &xsens2car,
                Matrix<double, 4, 4> &vel2car,
                int use_xsens)
{
	Matrix<double, 3, 3> mat;
	double roll, pitch, yaw;

	yaw = pitch = roll = 0.;

	if (use_xsens)
	{
		// convert xsens data to roll, pitch, yaw
		mat = sample->xsens.toRotationMatrix();
		mat = move_xsens_to_car(mat, xsens2car);
		getEulerYPR(mat, yaw, pitch, roll);
	}

	Matrix<double, 4, 4> car2world = pose6d_to_matrix(pose.x, pose.y, 0., roll, pitch, pose.th);
	Matrix<double, 4, 4> t = car2world * vel2car;

	transformPointCloud(*cloud, *cloud, t);
}


int
spherical_point_is_valid(double range)
{
	// range max
  if ((range >= 70.0) || (range < 4.0))
  		return 0;

  return 1;
}


void
compute_point_in_different_references(double v_angle, double h_angle, double range,
                                    	Matrix<double, 4, 1> *p_sensor,
                                    	Matrix<double, 4, 1> *p_car,
                                    	Matrix<double, 4, 1> *p_world,
                                      Matrix<double, 4, 4> &vel2car,
                                      Matrix<double, 4, 4> &car2world,
                                      Matrix<double, 4, 4> &vel2world)
{
	double x, y, z;

	spherical2cartersian(v_angle, h_angle, range, &x, &y, &z);

	(*p_sensor)(0, 0) = x;
	(*p_sensor)(1, 0) = y;
	(*p_sensor)(2, 0) = z;
	(*p_sensor)(3, 0) = 1;

	//(*p_car) = vel2car * (*p_sensor);
	//(*p_world) = car2world * (*p_car);
	(*p_world) = vel2world * (*p_sensor);
}


int
point3d_is_valid(Matrix<double, 4, 1> &p_sensor,
                 Matrix<double, 4, 1> &p_car,
                 Matrix<double, 4, 1> &p_world)
{
	// just for avoiding warnings so far.
	(void) p_car;
	(void) p_world;

	if ((fabs(p_sensor(0, 0)) > 6.0) || (fabs(p_sensor(1, 0)) > 4.0)) // remove rays that hit car
				//&& raw_cloud->at(i).z > -2.5  // remove points bellow ground
				//&& raw_cloud->at(i).z < -0.  // remove tree tops
				//&& (raw_cloud->at(i).x != 0 && raw_cloud->at(i).y != 0)
				//&& (!isnan(raw_cloud->at(i).x) && !isnan(raw_cloud->at(i).y) && !isnan(raw_cloud->at(i).z) && !isnan(raw_cloud->at(i).r))
				//&& (!isinf(raw_cloud->at(i).x) && !isinf(raw_cloud->at(i).y) && !isinf(raw_cloud->at(i).z) && !isinf(raw_cloud->at(i).r))
		return 1;

	return 0;
}


PointXYZRGB
create_point_and_update_intensity(Matrix<double, 4, 1> &p_sensor,
                                  Matrix<double, 4, 1> &p_world,
                                  Matrix<double, 4, 4> &vel2cam,
                                  Matrix<double, 3, 4> &projection,
                                  unsigned char intensity,
                                  Mat &img,
                                  GridMapType map_type,
                                  int *valid)
{
	PointXYZRGB point;

	point.x = p_world(0, 0) / p_world(3, 0);
	point.y = p_world(1, 0) / p_world(3, 0);
	point.z = p_world(2, 0) / p_world(3, 0);

	if (map_type == MAP_REMISSION)
	{
		unsigned char brightened = brighten(intensity);

		point.r = brightened;
		point.g = brightened;
		point.b = brightened;

		*valid = 1;
	}
	else
	{
		// point color from image.
		cv::Point pos_pixel;

		get_pixel_position(p_sensor(0, 0),
		                   p_sensor(1, 0),
		                   p_sensor(2, 0),
		                   vel2cam, projection,
		                   img, &pos_pixel, valid);

		if (*valid)
		{
			point.r = img.data[3 * (pos_pixel.y * img.cols + pos_pixel.x) + 2];
			point.g = img.data[3 * (pos_pixel.y * img.cols + pos_pixel.x) + 1];
			point.b = img.data[3 * (pos_pixel.y * img.cols + pos_pixel.x) + 0];
		}
	}

	return point;
}


void
update_map_with_shot(LidarShot *shot,
                     Matrix<double, 4, 4> &vel2car,
                     Matrix<double, 4, 4> &vel2cam,
                     Matrix<double, 4, 4> &car2world,
                     Matrix<double, 4, 4> &vel2world,
                     Matrix<double, 3, 4> &projection,
                     GridMap *map,
                     Mat &img,
                     GridMapType map_type)
{
	int valid;
	PointXYZRGB p_for_mapping;

	static Matrix<double, 4, 1> p_sensor;
	static Matrix<double, 4, 1> p_car;
	static Matrix<double, 4, 1> p_world;

	// the remaining coordinates will be initialized below.
	p_sensor(3, 0) = p_car(3, 0) = p_world(3, 0) = 1.;

	for (int i = 0; i < shot->n; i++)
	{
		// this test is performed first to prevent
		// additional calculations as soon as possible.
		if (!spherical_point_is_valid(shot->ranges[i]))
			continue;

		compute_point_in_different_references(shot->v_angles[i],
		                                      shot->h_angle,
		                                      shot->ranges[i],
		                                    	&p_sensor, &p_car, &p_world,
		                                      vel2car, car2world, vel2world);

		if (!point3d_is_valid(p_sensor, p_car, p_world))
			continue;

		p_for_mapping = create_point_and_update_intensity(p_sensor,
		                                                  p_world,
		                                                  vel2cam,
		                                                  projection,
		                                                  shot->intensities[i],
		                                                  img,
		                                                  map_type,
		                                                  &valid);

		if (valid)
			map->add_point(p_for_mapping);
	}
}


Matrix<double, 4, 4>
compute_transform_car2world(DataSample *sample, Pose2d &pose, int use_xsens,
                            Matrix<double, 4, 4> &xsens2car)
{
	// static to prevent reallocation
	static Matrix<double, 4, 4> car2world;

	double roll, pitch, yaw;
	yaw = pitch = roll = 0.;

	if (use_xsens)
	{
		Matrix<double, 3, 3> mat;
		// convert xsens data to roll, pitch, yaw
		mat = sample->xsens.toRotationMatrix();
		mat = move_xsens_to_car(mat, xsens2car);
		getEulerYPR(mat, yaw, pitch, roll);
	}

	car2world = pose6d_to_matrix(pose.x, pose.y, 0., roll, pitch, pose.th);
	return car2world;
}


void
update_map(DataSample *sample, GridMap *map,
           CarmenLidarLoader &vloader,
           CarmenImageLoader &iloader,
           SemanticSegmentationLoader &sloader,
           Matrix<double, 4, 4> &vel2car,
           Matrix<double, 4, 4> &vel2cam,
           Matrix<double, 3, 4> &projection,
           Matrix<double, 4, 4> &xsens2car,
           Pose2d &offset,
           int use_xsens,
           GridMapType map_type)
{
	Mat img;
	LidarShot *shot;

	Pose2d pose;
	pose = sample->pose;
	pose.x -= offset.x;
	pose.y -= offset.y;

	int use_new = 0;

	map->reload(pose.x, pose.y);

	vloader.reinitialize(sample->velodyne_path,
											 sample->n_laser_shots);

	if (map_type == MAP_VISUAL)
		img = iloader.load(sample);
	else if (map_type == MAP_SEMANTIC)
		img = sloader.load(sample);

	if (use_new)
	{
		static Matrix<double, 4, 4> car2world, vel2world;
		car2world = compute_transform_car2world(sample, pose, use_xsens, xsens2car);
		vel2world = car2world * vel2car;

		while (!vloader.done())
		{
			shot = vloader.next();
			update_map_with_shot(shot,
													 vel2car,
													 vel2cam,
													 car2world,
													 vel2world,
													 projection,
													 map,
													 img,
													 map_type);
		}
	}
	else
	{
		static PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
		static PointCloud<PointXYZRGB>::Ptr colored(new PointCloud<PointXYZRGB>);

		load_as_pointcloud(&vloader, cloud);
		cloud = filter_pointcloud(cloud);

		if (map_type == MAP_REMISSION)
		{
			increase_brightness(cloud, 5);
			colored = cloud;
		}
		else if (map_type == MAP_VISUAL || map_type == MAP_SEMANTIC)
			fuse_cam_and_lidar(cloud, vel2cam,
												 projection, img,
												 colored);

		transform_cloud(sample, pose, colored,
		                xsens2car, vel2car,
		                use_xsens);

		for (int j = 0; j < colored->size(); j++)
			map->add_point(colored->at(j));
	}
}


void
view(GridMap &map, DataSample *sample, Pose2d &offset, PointCloudViewer &viewer)
{
	Pose2d pose;
	pose = sample->pose;

	pose.x -= offset.x;
	pose.y -= offset.y;

	Mat map_img = map.to_image().clone();
	draw_pose(map, map_img, pose, Scalar(0, 255, 0));

	// flip vertically.
	Mat map_view;
	flip(map_img, map_view, 0);

	//viewer.clear();
	//viewer.show(colored);
	//viewer.show(img, "img", 640);
	//viewer.show(simg, "simg", 640);
	//viewer.show(simg_view, "simg_view", 640);
	viewer.show(map_view, "map", 640);
	viewer.loop();
}


#if USE_NEW
void
create_map(GridMap &map, const char *log_path, NewCarmenDataset *dataset,
						char path_save_maps[], int use_xsens, int step, GridMapType map_type)
{
	DataSample *sample;
	PointCloudViewer viewer(1);

	Mat img, simg, simg_view;

	Matrix<double, 4, 4> vel2car = dataset->vel2car();
	Matrix<double, 4, 4> vel2cam = dataset->vel2cam();
	Matrix<double, 3, 4> projection = dataset->projection_matrix();
	Matrix<double, 4, 4> xsens2car = dataset->xsens2car();

	CarmenLidarLoader vloader;
	CarmenImageLoader iloader;
	SemanticSegmentationLoader sloader(log_path);
	Pose2d pose, offset;

	offset = dataset->at(0)->pose;

	for (int i = 0; i < dataset->size(); i += step)
	{
		sample = dataset->at(i);

		if (fabs(sample->v) < 1.0)
			continue;

		update_map(sample, &map,
		           vloader,
		           iloader,
		           sloader,
		           vel2car,
		           vel2cam,
		           projection,
		           xsens2car,
		           offset,
		           use_xsens,
		           map_type);

		//view(map, sample, offset, viewer);
	}
}


#else
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
	int step = 4;

	Matrix<double, 3, 3> rot3d;
	Matrix<double, 4, 4> transf;

	for (int i = 0; i < dataset.data.size(); i += step)
	{
		if (fabs(dataset.data[i].v) < 0.1)
		continue;

		Pose2d pose = dataset.data[i].pose;

		cloud->clear();
		transformed_cloud->clear();
		dataset.load_fused_pointcloud_and_camera(i, cloud, dataset.data[i].v, dataset.data[i].phi, 0, 1, &img_view);
		increase_brightness(cloud, 5);
		//pose.x = pose.y = 0.;
		//pose.th = normalize_theta(-pose.th - degrees_to_radians(18));

		//double roll, pitch, yaw;
		//Vector3d euler = dataset.data[i].xsens.toRotationMatrix().eulerAngles(2, 1, 0);
		//yaw = euler[0]; pitch = euler[1]; roll = euler[2];

		//printf("roll: %lf pitch: %lf yaw: %lf\n",
		//radians_to_degrees(roll), radians_to_degrees(pitch), radians_to_degrees(yaw));

		//Matrix<double, 4, 4> mat = pose6d_to_matrix(pose.x, pose.y, 0., roll, pitch, pose.th);
		Matrix<double, 4, 4> mat = pose6d_to_matrix(pose.x, pose.y, 0, 0, 0, pose.th);
		pcl::transformPointCloud(*cloud, *transformed_cloud, mat);

		map.reload(pose.x, pose.y);

		for (int j = 0; j < transformed_cloud->size(); j++)
		map.add_point(transformed_cloud->at(j));

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
		//increase_bightness(transformed_cloud);

		///*
		char *cloud_name = (char *) calloc (32, sizeof(char));
		sprintf(cloud_name, "cloud%d", i);

		//viewer.removeAllPointClouds();
		//for (int j = 0; j < transformed_cloud->size(); j++)
		//transformed_cloud->at(j).z = 0.;

		viewer.removeAllPointClouds();
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
	}
}
#endif


GridMapType
parse_map_type(string map_type)
{
	if (map_type.compare("remission") == 0)
		return MAP_REMISSION;
	else if (map_type.compare("visual") == 0)
		return MAP_VISUAL;
	else if (map_type.compare("semantic") == 0)
		return MAP_SEMANTIC;
	else
		exit(printf("Error: invalid map type '%s'.\n", map_type.c_str()));
}


int
main(int argc, char **argv)
{
	int grid_map_type;
	GridMapType map_type;
	string log_path, map_path;
	double resolution, tile_size;
	po::variables_map args;

	CommandLineArguments args_parser;

	args_parser.add_positional<string>("log_path", "Path of a log", 1);
	args_parser.add<double>("resolution,r", "Map resolution", 0.2);
	args_parser.add<double>("tile_size,s", "Map tiles size", 50);
	args_parser.add<string>("map_path,m", "Path to save the maps", "/tmp");
	args_parser.add<int>("use_xsens,x", "Whether or not to use pitch, and roll angles from xsens", 1);
	args_parser.add<int>("step", "Number of data packages to skip", 1);
	args_parser.add<string>("map_type,t", "Type of grid map [remission | visual | semantic]", "remission");
	args_parser.save_config_file("data/mapper_config.txt");
	args_parser.parse(argc, argv);

	resolution = args_parser.get<double>("resolution");
	tile_size = args_parser.get<double>("tile_size");
	log_path = args_parser.get<string>("log_path");
	map_path = args_parser.get<string>("map_path");
	map_type = parse_map_type(args_parser.get<string>("map_type"));

	printf("map path: %s\n", map_path.c_str());
	printf("tile size: %lf\n", tile_size);
	printf("resolution: %lf\n", resolution);

	/*
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
	 */

	grid_map_type = (map_type == MAP_SEMANTIC) ? (GridMapTile::TYPE_SEMANTIC) : (GridMapTile::TYPE_VISUAL);
	GridMap map(map_path, tile_size, tile_size, resolution, grid_map_type, 1);

#if USE_NEW	

	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());
	string graphslam_path = default_graphslam_path(log_path.c_str());

	NewCarmenDataset *dataset;
	dataset = new NewCarmenDataset(log_path, odom_calib_path, graphslam_path);
	create_map(map, log_path.c_str(), dataset, "/tmp",
						 args_parser.get<int>("use_xsens"),
						 args_parser.get<int>("step"),
						 map_type);

#else
	DatasetInterface *dataset = new DatasetCarmen("/dados/data/data_log_volta_da_ufes-20180907-2.txt", 0);
	create_map(map, *dataset, "/tmp/");
#endif

	printf("Done\n");
	return 0;
}

