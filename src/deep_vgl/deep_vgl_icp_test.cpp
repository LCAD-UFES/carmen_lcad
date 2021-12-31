
/**
 * @description
 * DNN Global Localizer
 *
 * @author Alberto F. De Souza
 */

#include <stdio.h>
#include <string.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <tf.h>
#include <vector>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/web_cam_interface.h>
#include <carmen/stereo_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/xsens_interface.h>
#include <carmen/gps_xyz_messages.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/camera_drivers_interface.h>
#include <gflags/gflags.h>

#include "network.h"
#include "parser.h"

using namespace std;
using namespace tf;
using namespace cv;

DEFINE_string(poses_and_labels, "", "poses and labels file");

DEFINE_string(config_file, "", "darknet config file");

DEFINE_string(weights_file, "", "network weights file in darknet format");

DEFINE_string(images_list, "", "list of images used for training, test or validation");

DEFINE_string(pcl_path, "", "list of images used for training, test or validation");

// trecho incluido para GICP

typedef struct
{
	carmen_pose_3D_t odometry_pose;
	carmen_pose_3D_t gps_pose;
	double timestamp;
} Line;

//pcl::visualization::PCLVisualizer *viewer;

int newCloudId, predCloudId;

char temp_path[2048];

double velodyne_vertical_angles[32] = {-30.6700000, -29.3300000, -28.0000000, -26.6700000, -25.3300000, -24.0000000, -22.6700000, -21.3300000,
									   -20.0000000, -18.6700000, -17.3300000, -16.0000000, -14.6700000, -13.3300000, -12.0000000, -10.6700000,
									   -9.3299999, -8.0000000, -6.6700001, -5.3299999, -4.0000000, -2.6700001, -1.3300000, 0.0000000, 1.3300000,
									   2.6700001, 4.0000000, 5.3299999, 6.6700001, 8.0000000, 9.3299999, 10.6700000};

int my_velodyne_ray_order[32] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};

pcl::PointXYZ
compute_point_from_velodyne(double v_angle, double h_angle, double radius, unsigned char intensity)
{
	// build a new point
	pcl::PointXYZ point;

	double cos_rot_angle = cos(h_angle);
	double sin_rot_angle = sin(h_angle);

	double cos_vert_angle = cos(v_angle);
	double sin_vert_angle = sin(v_angle);

	double xy_distance = radius * cos_vert_angle;

	point.x = (xy_distance * cos_rot_angle);
	point.y = (xy_distance * sin_rot_angle);
	point.z = (radius * sin_vert_angle);

	// point.r = intensity;
	// point.g = intensity;
	// point.b = intensity;
	point.data[3] = 1.0f;
	return point;
}

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
// pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

vector<Line> input_data;

pcl::PointCloud<pcl::PointXYZ>::Ptr
GridFiltering(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud, double size)
{
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

	grid.setLeafSize(size, size, size);
	grid.setInputCloud(inputPointCloud);
	grid.filter(*outputPointCloud);

	return outputPointCloud;
}

Eigen::Matrix<double, 4, 4>
transform_carmen_pose_to_pcl_pose(carmen_pose_3D_t *carmen_pose)
{
	Eigen::Matrix<double, 4, 4> pcl_pose;

	Quaternion quat(carmen_pose->orientation.yaw,
					carmen_pose->orientation.pitch,
					carmen_pose->orientation.roll);

	tf::Matrix3x3 rotation(quat);

	pcl_pose(0, 0) = rotation[0][0];
	pcl_pose(0, 1) = rotation[0][1];
	pcl_pose(0, 2) = rotation[0][2];
	pcl_pose(1, 0) = rotation[1][0];
	pcl_pose(1, 1) = rotation[1][1];
	pcl_pose(1, 2) = rotation[1][2];
	pcl_pose(2, 0) = rotation[2][0];
	pcl_pose(2, 1) = rotation[2][1];
	pcl_pose(2, 2) = rotation[2][2];

	pcl_pose(0, 3) = carmen_pose->position.x;
	pcl_pose(1, 3) = carmen_pose->position.y;
	pcl_pose(2, 3) = carmen_pose->position.z;

	pcl_pose(3, 0) = 0;
	pcl_pose(3, 1) = 0;
	pcl_pose(3, 2) = 0;
	pcl_pose(3, 3) = 1;

	return pcl_pose;
}

void transform_pcl_pose_to_carmen_pose(Eigen::Matrix<double, 4, 4> pcl_pose, carmen_pose_3D_t *carmen_pose)
{
	tf::Matrix3x3 rotation;
	double roll, pitch, yaw;

	rotation[0][0] = pcl_pose(0, 0);
	rotation[0][1] = pcl_pose(0, 1);
	rotation[0][2] = pcl_pose(0, 2);
	rotation[1][0] = pcl_pose(1, 0);
	rotation[1][1] = pcl_pose(1, 1);
	rotation[1][2] = pcl_pose(1, 2);
	rotation[2][0] = pcl_pose(2, 0);
	rotation[2][1] = pcl_pose(2, 1);
	rotation[2][2] = pcl_pose(2, 2);

	rotation.getRPY(roll, pitch, yaw);

	carmen_pose->orientation.roll = roll;
	carmen_pose->orientation.pitch = pitch;
	carmen_pose->orientation.yaw = yaw;
	carmen_pose->position.x = pcl_pose(0, 3);
	carmen_pose->position.y = pcl_pose(1, 3);
	carmen_pose->position.z = pcl_pose(2, 3);
}

void load_pointcloud_from_file(const char *path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	int c;
	double h_angle, v_angle;
	unsigned short distances[32];
	unsigned char intensities[32];
	double range;
	double max, may, maz, mix, miy, miz = 0;

	FILE *f = fopen(path, "rb");

	if (f == NULL)
		exit(printf("File '%s' not found.\n", path));

	while (!feof(f))
	{
		c = fread(&h_angle, sizeof(double), 1, f);
		if (c != 1)
			break; // reading error

		c = fread(distances, sizeof(unsigned short), 32, f);
		if (c != 32)
			break; // reading error

		fread(intensities, sizeof(unsigned char), 32, f);
		if (c != 32)
			break; // reading error

		h_angle = M_PI * (-h_angle) / 180.;

		for (int j = 0; j < 32; j++)
		{
			range = (double)distances[my_velodyne_ray_order[j]] / 500.;
			v_angle = velodyne_vertical_angles[j];
			v_angle = M_PI * v_angle / 180.;

			pcl::PointXYZ point = compute_point_from_velodyne(v_angle, h_angle, range, intensities[my_velodyne_ray_order[j]]);
			// max = max>point.x?max:point.x;
			// may = may>point.y?may:point.y;
			// maz = maz>point.z?maz:point.z;

			// mix = mix<point.x?mix:point.x;
			// miy = miy<point.y?miy:point.y;
			// miz = miz<point.z?miz:point.z;

			// printf("max: %lf may: %lf maz: %lf \n",max,may,maz);
			// printf("mix: %lf miy: %lf miz: %lf \n",mix,miy,miz);
			// if
			// (
			// 	(
			// 		(point.x < -10 ) ||
			// 		(point.x > 3 ) ||
			// 		(point.y > 2 ) ||
			// 		(point.y < -2 )
			// 	) &&
			// 	(point.z > -2.8)
			// )
			// if (((point.x < -10 ) || (point.x > 3 ) || (point.y > 2) || (point.y < -2 )) && (point.z > -2.8))
			cloud->push_back(point);
		}
	}

	fclose(f);
}

void load_pointcloud_from_velodyne(carmen_velodyne_partial_scan_message *lidar, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	int c;
	double h_angle, v_angle;
	short unsigned int *distances;
	unsigned char *intensities;
	double range;
	int shots = lidar->number_of_32_laser_shots;

	double by500 = (double)(1 / 500.0); // para poupar tempo no loop
	double by180 = (double)(1 / 180.0); // para poupar tempo no loop

	for (int i = 0; i < shots; i++)
	{
		h_angle = lidar->partial_scan[i].angle;

		distances = lidar->partial_scan[i].distance;

		intensities = lidar->partial_scan[i].intensity;

		h_angle = M_PI * (-h_angle) / 180.;

		for (int j = 0; j < 32; j++)
		{
			range = (double)distances[my_velodyne_ray_order[j]] * by500;
			v_angle = velodyne_vertical_angles[j];
			v_angle = M_PI * v_angle * by180;

			pcl::PointXYZ point = compute_point_from_velodyne(v_angle, h_angle, range, intensities[my_velodyne_ray_order[j]]);
			cloud->push_back(point);
		}
	}
}

void initialize_icp()
{
	// gicp.setEuclideanFitnessEpsilon(1e-06);
	gicp.setMaximumIterations(10);
	gicp.setTransformationEpsilon(1e-5);
	gicp.setRotationEpsilon(1e-5);
	gicp.setMaxCorrespondenceDistance(20.0);
}

// int perform_icp2(pcl::PointCloud<pcl::PointXYZ>::Ptr source_pointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_pointcloud,
// 				 Eigen::Matrix<double, 4, 4> *correction, Eigen::Matrix<double, 4, 4> *pose)
// {

// 	pcl::PointCloud<pcl::PointXYZ> out_pcl_pointcloud;
// 	pcl::PointCloud<pcl::PointXYZ> out_pcl_pointcloud_transformed;

// 	pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered = GridFiltering(source_pointcloud, 0.1);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered = GridFiltering(target_pointcloud, 0.1);
// 	gicp.setInputSource(source_filtered);
// 	gicp.setInputTarget(target_filtered);

// 	// gicp.setInputCloud(source_pointcloud);
// 	// gicp.setInputTarget(target_pointcloud);

// 	gicp.align(out_pcl_pointcloud);

// 	if (gicp.hasConverged())
// 	{
// 		(*correction) = gicp.getFinalTransformation().cast<double>();
// 		Eigen::Matrix<double, 4, 4> odom = ((*correction) * (*pose).inverse()).cast<double>();
// 		pcl::transformPointCloud(out_pcl_pointcloud, out_pcl_pointcloud_transformed, odom);
// 		// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
// 		// viewer->addCoordinateSystem(1.0);
// 		// viewer->setBackgroundColor(0, 0, 0);
// 		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(out_pcl_pointcloud_transformed.makeShared(), 0, 0, 200);
// 		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(target_filtered, 0, 200, 0);
// 		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(source_pointcloud, 0, 200, 0);
// 		char newcloud[256];
// 		char predcloud[256];

// 		sprintf(newcloud, "newCloud%d", newCloudId);
// 		sprintf(predcloud, "predCloud%d", newCloudId - 1);

// 		if (!viewer->updatePointCloud<pcl::PointXYZ>(target_filtered, single_color3, predcloud))
// 		{
// 			viewer->addPointCloud<pcl::PointXYZ>(target_filtered, single_color3, predcloud);
// 		}
// 		viewer->addPointCloud<pcl::PointXYZ>(out_pcl_pointcloud_transformed.makeShared(), single_color1, newcloud);

// 		// viewer->addPointCloud<pcl::PointXYZ>(source_pointcloud, single_color2, "sample_cloud_2");

// 		// while (!viewer->wasStopped())
// 		// {
// 		viewer->spinOnce(100);
// 		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
// 		// }
// 		return 1;
// 	}

// 	return 0;
// }

int perform_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr source_pointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_pointcloud,
				Eigen::Matrix<double, 4, 4> *correction, Eigen::Matrix<double, 4, 4> *pose)
{

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(200, 1));
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	// // Fill in the CloudIn data
	// for (auto &point : *cloud_in)
	// {
	// 	point.x = 1024 * rand() / (RAND_MAX + 1.0f);
	// 	point.y = 1024 * rand() / (RAND_MAX + 1.0f);
	// 	point.z = 1024 * rand() / (RAND_MAX + 1.0f);
	// }

	// *cloud_out = *cloud_in;

	// for (auto &point : *cloud_out)
	// 	point.x += 1.0f;

	pcl::PointCloud<pcl::PointXYZ> out_pcl_pointcloud;
	pcl::PointCloud<pcl::PointXYZ> out_pcl_pointcloud_transformed;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered = GridFiltering(cloud_in, 0.1);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered = GridFiltering(cloud_out, 0.1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered = GridFiltering(source_pointcloud, 0.1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered = GridFiltering(target_pointcloud, 0.1);
	
	// for (auto &point : *source_pointcloud)
	// 	point.y += 5.0f;

	gicp.setInputSource(source_filtered);

	// for (auto &point : *target_filtered)
	// 	point.x += 20.0f;

	gicp.setInputTarget(target_filtered);

	// gicp.setInputCloud(source_pointcloud);
	// gicp.setInputTarget(target_pointcloud);

	gicp.align(out_pcl_pointcloud);

	if (gicp.hasConverged())
	{
		(*correction) = gicp.getFinalTransformation().cast<double>();
		// Eigen::Matrix<double, 4, 4> odom = ((*correction).inverse()).cast<double>();
		// pcl::transformPointCloud(out_pcl_pointcloud, out_pcl_pointcloud_transformed, (*correction));
		// // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		// // viewer->addCoordinateSystem(1.0);
		// // viewer->setBackgroundColor(0, 0, 0);
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(out_pcl_pointcloud.makeShared(), 0, 0, 200);
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(target_filtered, 0, 200, 0);
		// //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(source_pointcloud, 200, 0, 0);
		// char newcloud[256];
		// char predcloud[256];

		// sprintf(newcloud, "newCloud%d", newCloudId);
		// sprintf(predcloud, "predCloud%d", predCloudId);

		// if (!viewer->updatePointCloud<pcl::PointXYZ>(target_filtered, single_color3, predcloud))
		// {
		// 	viewer->addPointCloud<pcl::PointXYZ>(target_filtered, single_color3, predcloud);
		// }
		// viewer->addPointCloud<pcl::PointXYZ>(out_pcl_pointcloud_transformed.makeShared(), single_color1, newcloud);

		//viewer->addPointCloud<pcl::PointXYZ>(source_pointcloud, single_color2, "sample_cloud_2");

		// while (!viewer->wasStopped())
		// {
		//viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		// }
		return 1;
	}

	return 0;
}

int process_data(char *read_scan_path, char *predicted_scan_path, carmen_point_t *pose)
{
	carmen_pose_3D_t measured_pose_out;
	Eigen::Matrix<double, 4, 4> correction, guess;

	measured_pose_out.position.x = pose->x;
	measured_pose_out.position.y = pose->y;
	measured_pose_out.position.z = 0;
	measured_pose_out.orientation.roll = 0;
	measured_pose_out.orientation.pitch = 0;
	measured_pose_out.orientation.yaw = pose->theta;

	guess = transform_carmen_pose_to_pcl_pose(&measured_pose_out);

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

	load_pointcloud_from_file(predicted_scan_path, target_pointcloud);

	load_pointcloud_from_file(read_scan_path, source_pointcloud);

	if (source_pointcloud->size() <= 0)
		exit(printf("Error: Source empty: %s\n", read_scan_path));

	if (target_pointcloud->size() <= 0)
		exit(printf("Error: Target empty: %s\n", predicted_scan_path));

	if (perform_icp(source_pointcloud, target_pointcloud, &correction, &guess))
	{

		// cout << "correction:\n" << correction << endl;
		// cout << "pose:\n" << guess << endl;
		// cout << "resultado:\n" << guess*correction << endl;
		transform_pcl_pose_to_carmen_pose((guess*correction), &measured_pose_out);
		pose->x = measured_pose_out.position.x;
		pose->y = measured_pose_out.position.y;
		pose->theta = measured_pose_out.orientation.yaw;
		return 1;
	}
	else
		fprintf(stderr, "Not converged!\n");

	return 0;
}

char *getFileNameFromPath(char *path)
{
	for (size_t i = strlen(path) - 1; i; i--)
	{
		if (path[i] == '/')
		{
			return &path[i + 1];
		}
	}
	return path;
}

void remove_ext(char *fname)
{
	char *end = fname + strlen(fname);

	while (end > fname && *end != '.')
	{
		--end;
	}

	if (end > fname)
	{
		*end = '\0';
	}
}
// fim do trecho GICP

//#pragma pack(push, 2)
struct RGB
{
	uchar B;
	uchar G;
	uchar R;
};

static char velodyne_data_path[2048];
static int camera;
static int bumblebee_basic_width;
static int bumblebee_basic_height;

network net;
char **learned_poses;
int last_correct_prediction = -1;
int contador = 0;
/****
 * usados para crop_image()
 *  esquerda  topo	   largura     altura		****/
int delta_x, delta_y, crop_width, crop_height;

/***
 * usados para lidar
 *  usar lidar   angulo a esquerda   angulo a direite
 */

int use_lidar, angle_left, angle_right;

cv::Mat
convert_darknet_image_to_cv_mat(image img)
{
	int channels = img.c;
	int width = img.w;
	int height = img.h;
	cv::Mat mat = cv::Mat(height, width, CV_8UC(channels));
	int step = mat.step;

	for (int y = 0; y < img.h; ++y)
	{
		for (int x = 0; x < img.w; ++x)
		{
			for (int c = 0; c < img.c; ++c)
			{
				float val = img.data[c * img.h * img.w + y * img.w + x];
				mat.data[y * step + x * img.c + c] = (unsigned char)(val * 255);
			}
		}
	}

	if (mat.channels() == 3)
		cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
	else if (mat.channels() == 4)
		cv::cvtColor(mat, mat, cv::COLOR_RGBA2BGR);

	return mat;
}

image convert_image_msg_to_darknet_image(unsigned int w, unsigned int h, unsigned char *data)
{
	unsigned int c = 3; // Number of channels
	image image = make_image(w, h, c);

	if (data == NULL)
		return image;

	for (unsigned int k = 0; k < c; ++k)
	{
		for (unsigned int j = 0; j < h; ++j)
		{
			for (unsigned int i = 0; i < w; ++i)
			{
				int dst_index = i + (w * j) + (w * h * k);
				int src_index = k + (c * i) + (c * w * j);
				image.data[dst_index] = (float)(data[src_index] / 255.0); // 255 because of conversion Uchar (byte) ti float
			}
		}
	}

	return (image);
}

double
infer_pose(carmen_point_t *pose, double width, double height, int dx, int dy, int w, int h, unsigned char *image_raw, char *path)
{
	image img = convert_image_msg_to_darknet_image(width, height, image_raw);
	image img_without_car_hood = crop_image(img, dx, dy, w, h); // crop_image() nao faz free()
	int min = net.h < net.w ? net.h : net.w;
	image resized_img_without_car_hood = resize_min(img_without_car_hood, min);
	image cropped_resized_img_without_car_hood = crop_image(resized_img_without_car_hood, (resized_img_without_car_hood.w - net.w) / 2, (resized_img_without_car_hood.h - net.h) / 2, net.w, net.h);
	// cv::Mat mat = convert_darknet_image_to_cv_mat(cropped_resized_img_without_car_hood);
	// cv::namedWindow("Cropped Image", cv::WINDOW_NORMAL);
	// cv::imshow("Cropped Image", mat);

	float *predictions = network_predict(net, cropped_resized_img_without_car_hood.data);

	int selected_pose_label;
	top_k(predictions, net.outputs, 1, &selected_pose_label);
	predCloudId = selected_pose_label;
	char predicted_image_file_name[2048];
	sscanf(learned_poses[selected_pose_label], "%lf %lf %lf %s", &(pose->x), &(pose->y), &(pose->theta), predicted_image_file_name);
	// printf("confidence %lf, %lf %lf %lf %s\n", predictions[selected_pose_label], pose->x, pose->y, pose->theta, predicted_image_file_name);

	// Mat pose_image = imread(predicted_image_file_name, IMREAD_COLOR);
	// if (predictions[selected_pose_label] < 0.00)
	// 	pose_image = Mat::zeros(Size(pose_image.cols, pose_image.rows), pose_image.type());
	// imshow("dnn_visual_gl", pose_image);
	// waitKey(1);

	free_image(img);
	free_image(img_without_car_hood);
	free_image(resized_img_without_car_hood);
	free_image(cropped_resized_img_without_car_hood);
	if (path != 0)
		strcpy(path, predicted_image_file_name);
	return (predictions[selected_pose_label]);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void velodyne_partial_scan_handler(char *path)
{

	carmen_point_t pose, delta;
	Mat bgrimg = imread(path, IMREAD_COLOR);
	Mat resized_image;
	cvtColor(bgrimg, resized_image, CV_BGR2RGB);

	char file_path[2048];
	char read_cloud[2048];
	strcpy(read_cloud, path);
	// inferir pose
	double confidence = infer_pose(&pose, crop_width, crop_height, 0, 0, 640, 480, resized_image.data, file_path);

	remove_ext(file_path);
	// remove_ext(temp_path);
	char *filename = getFileNameFromPath(file_path);
	// char *filename = getFileNameFromPath(temp_path);
	char *pred_pointcloud_file;

	remove_ext(read_cloud);
	char *rc_filename = getFileNameFromPath(read_cloud);
	char *read_pointcloud_file;

	strcpy(temp_path, read_cloud);

	//asprintf(&pred_pointcloud_file, "%s/live/%s.%s", velodyne_data_path, filename, "pointcloud");
	asprintf(&pred_pointcloud_file, "%s/base/%s.%s", velodyne_data_path, filename, "pointcloud");
	asprintf(&read_pointcloud_file, "%s/live/%s.%s", velodyne_data_path, rc_filename, "pointcloud");
	/**
	 * AQUI vai calcular o GICP da pose que chegou com a pose que estimou e publicar a diferença
	 **/
	delta.theta = pose.theta;
	delta.x = pose.x;
	delta.y = pose.y;

	int ret = process_data(read_pointcloud_file, pred_pointcloud_file, &delta);
	// printf("Deu certo? %d\n", ret);
	printf("prediction: %lf %lf -> ICP: %lf %lf \n", pose.x, pose.y, delta.x, delta.y);

	/**
	 * FIM do GICP
	 **/
}

//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
//          Initializations                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

void initialize_structures(char *cfgfile, char *weightfile, char *learned_poses_filename, int dx, int dy, int w, int h, int use_ldr, int angle_lft, int angle_rgt)
{
	delta_x = dx;
	delta_y = dy;
	crop_width = w;
	crop_height = h;
	use_lidar = use_ldr ? use_ldr : 0;
	angle_left = angle_lft ? std::abs(angle_lft) : 45;
	angle_right = angle_rgt ? std::abs(angle_rgt) : 45;
	net = parse_network_cfg_custom(cfgfile, 1, 0);
	if (weightfile)
		load_weights(&net, weightfile);

	set_batch_network(&net, 1);
	srand(2222222);

	fuse_conv_batchnorm(net);
	calculate_binary_weights(net);

	learned_poses = get_labels(learned_poses_filename);
	printf("all done on darknet\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	string config_file, weight_file, poses_list, images_list, pcl_path;
	::google::ParseCommandLineFlags(&argc, &argv, true);

	if (FLAGS_poses_and_labels == "" || FLAGS_weights_file == "" || FLAGS_config_file == "" || FLAGS_images_list == "" || FLAGS_pcl_path == "")
	{
		printf(" Usage: %s --poses_and_labels config/poses_and_labels.txt  --weights_file config/classifier.weights --config_file config/config.cfg --images_list config/test.txt --pcl_path /dados/log_ufes.txt_lidar \n", argv[0]);
		exit(1);
	}
	else
	{
		poses_list = FLAGS_poses_and_labels;
		weight_file = FLAGS_weights_file;
		config_file = FLAGS_config_file;
		images_list = FLAGS_images_list;
		pcl_path = FLAGS_pcl_path;
	}
	int classes_qtd = 0;
	std::ifstream labels_file;
	std::string line;

	strcpy(velodyne_data_path, pcl_path.c_str());

	labels_file.open(poses_list.c_str());
	if (!labels_file)
	{
		printf("Could not open labels file %s\n", poses_list.c_str());
		exit(1);
	}

	initialize_structures((char *)config_file.c_str(), (char *)weight_file.c_str(), (char *)poses_list.c_str(), 0, 0, 640, 480, 1, 135, 135);
	initialize_icp();
	// viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
	// viewer->addCoordinateSystem(1.0);
	// viewer->setBackgroundColor(0, 0, 0);
	std::ifstream images_file;
	images_file.open((char *)images_list.c_str());
	newCloudId = 0;
	predCloudId = 0;
	if (images_file.is_open())
	{
		getline(images_file, line);
		strcpy(temp_path, (char *)line.c_str());

		while (getline(images_file, line))
		{
			newCloudId += 1;
			// printf("TESTE: %s\n", line.c_str());
			velodyne_partial_scan_handler((char *)line.c_str());
		}
	}

	return (0);
}

// int
//  main ()
// {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

//   // Fill in the CloudIn data
//   for (auto& point : *cloud_in)
//   {
//     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
//   }

//   std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;

//   for (auto& point : *cloud_in)
//     std::cout << point << std::endl;

//   *cloud_out = *cloud_in;

//   std::cout << "size:" << cloud_out->size() << std::endl;
//   for (auto& point : *cloud_out)
//     point.x += 1.0f;

//   std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;

//   for (auto& point : *cloud_out)
//     std::cout << point << std::endl;

//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource(cloud_in);
//   icp.setInputTarget(cloud_out);

//   pcl::PointCloud<pcl::PointXYZ> Final;
//   icp.align(Final);

//   std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//   icp.getFitnessScore() << std::endl;
//   std::cout << icp.getFinalTransformation() << std::endl;

//  return (0);
// }
