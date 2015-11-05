#include "voslam_util.h"

IplImage* depth_src = NULL;
IplImage* depth_dest = NULL;
float* depth_src_data = NULL;

void convert_16u_to_32f_raw_depth(unsigned short* depth_src, float* depth_dest, int width, int height, float max_depth)
{
	for(int i = 0; i < (width * height); i++)
		depth_dest[i] = (float) (depth_src[i] / max_depth);
}

void convert_32f_to_16u_raw_depth(float* depth_src, unsigned short* depth_dest, int width, int height, float max_depth)
{
	for(int i = 0; i < (width * height); i++)
		depth_dest[i] = (unsigned short) (depth_src[i] * max_depth);
}

void convert_rgb_image_to_gray_image(unsigned char *rgb_image, unsigned char* gray_image, int width, int height)
{
	for(int i = 0; i < (width * height); i++)
		gray_image[i] = (0.30 * rgb_image[3 * i]) + (0.59 * rgb_image[3 * i + 1]) + (0.11 * rgb_image[3 * i + 2]);
}

void apply_bilateral_filter(unsigned short* src, unsigned short* dest, stereo_util stereo_reprojection_params)
{
	if(depth_src == NULL && depth_dest == NULL && depth_src_data == NULL)
	{
		depth_src = cvCreateImage(cvSize(stereo_reprojection_params.width, stereo_reprojection_params.height), IPL_DEPTH_32F, 1);
		depth_dest = cvCreateImage(cvSize(stereo_reprojection_params.width, stereo_reprojection_params.height), IPL_DEPTH_32F, 1);
		depth_src_data = (float*) malloc (stereo_reprojection_params.width * stereo_reprojection_params.height * sizeof(float));
	}

	convert_16u_to_32f_raw_depth(src, depth_src_data, stereo_reprojection_params.width, stereo_reprojection_params.height, STEREO_RANGE_MAX * 1000.0);
	depth_src->imageData = (char*) depth_src_data;

	cvSmooth(depth_src, depth_dest, CV_BILATERAL, 3, 3, 1, 1);

	convert_32f_to_16u_raw_depth((float*) depth_dest->imageData, dest, stereo_reprojection_params.width, stereo_reprojection_params.height, STEREO_RANGE_MAX * 1000.0);
}

Eigen::Matrix<float, 4, 4> voslam_pointcloud_pose_to_eigen_transform(carmen_voslam_pose_t pose)
{
	Eigen::Matrix<float, 4, 4> result;

	//rotation
	result(0, 0) = pose.rotation[0][0];
	result(0, 1) = pose.rotation[0][1];
	result(0, 2) = pose.rotation[0][2];
	result(1, 0) = pose.rotation[1][0];
	result(1, 1) = pose.rotation[1][1];
	result(1, 2) = pose.rotation[1][2];
	result(2, 0) = pose.rotation[2][0];
	result(2, 1) = pose.rotation[2][1];
	result(2, 2) = pose.rotation[2][2];

	//translation
	result(0, 3) = pose.position[0];
	result(1, 3) = pose.position[1];
	result(2, 3) = pose.position[2];

	result(3, 0) = 0.0;
	result(3, 1) = 0.0;
	result(3, 2) = 0.0;
	result(3, 3) = 1.0;

	return result;
}

carmen_voslam_pose_t eigen_transform_to_voslam_pointcloud_pose(Eigen::Matrix<float, 4, 4> transform)
{
	carmen_voslam_pose_t pose;
	double roll, pitch, yaw;

	//rotation
	pose.rotation[0][0] = transform(0, 0);
	pose.rotation[0][1] = transform(0, 1);
	pose.rotation[0][2] = transform(0, 2);
	pose.rotation[1][0] = transform(1, 0);
	pose.rotation[1][1] = transform(1, 1);
	pose.rotation[1][2] = transform(1, 2);
	pose.rotation[2][0] = transform(2, 0);
	pose.rotation[2][1] = transform(2, 1);
	pose.rotation[2][2] = transform(2, 2);

	pose.rotation.getRPY(roll, pitch, yaw);
	pose.orientation = tf::Vector3(roll, pitch, yaw);

	//translation
	pose.position[0] = transform(0, 3);
	pose.position[1] = transform(1, 3);
	pose.position[2] = transform(2, 3);

	return pose;
}

Eigen::Matrix<float, 4, 4> compute_pcl_transform_between_voslam_pointclouds(carmen_voslam_pointcloud_t *source_voslam_pointcloud, carmen_voslam_pointcloud_t *target_voslam_pointcloud)
{
	Eigen::Matrix<float, 4, 4> src_transform;
	Eigen::Matrix<float, 4, 4> tgt_transform;
	Eigen::Matrix<float, 4, 4> diff_transform;

	src_transform = voslam_pointcloud_pose_to_eigen_transform(source_voslam_pointcloud->pose);
	tgt_transform = voslam_pointcloud_pose_to_eigen_transform(target_voslam_pointcloud->pose);

	diff_transform = tgt_transform * src_transform.inverse();

	return diff_transform;
}

void print_eigen_matrix(Eigen::Matrix<float, 4, 4>& matrix, char* name)
{
	printf("%s: \n", name);
	printf(" %6.2f  %6.2f  %6.2f  %6.2f \n %6.2f  %6.2f  %6.2f  %6.2f \n %6.2f  %6.2f  %6.2f  %6.2f \n %6.2f  %6.2f  %6.2f  %6.2f \n\n",
			matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0,3),
			matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1,3),
			matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2,3),
			matrix(3, 0), matrix(3, 1), matrix(3, 2), matrix(3,3));
}

void print_voslam_graph(std::vector<carmen_voslam_pointcloud_t> list)
{
	carmen_voslam_pose_t transform;

	for(int i = 0; i < list.size(); i++)
		printf("VERTEX_SE2 %d %f %f %f\n", i, list[i].pose.position[0],  list[i].pose.position[2], list[i].pose.orientation[2] - list[0].pose.orientation[2]);

	for(int i = 0; i < list.size(); i++)
	{
		if(i > 0)
		{
			transform = eigen_transform_to_voslam_pointcloud_pose(compute_pcl_transform_between_voslam_pointclouds(&list[i-1], &list[i]));
			printf("EDGE_SE2 %d %d %f %f %f 500 0 0 500 0 5000\n", i-1, i, transform.position[0], transform.position[2], transform.orientation[2]);
		}

		for(int j = 0; j < list[i].loop_partners_indexes.size(); j++)
		{
			transform = eigen_transform_to_voslam_pointcloud_pose(compute_pcl_transform_between_voslam_pointclouds(&list[i], &list[list[i].loop_partners_indexes[j]]));
			printf("EDGE_SE2 %d %d %f %f %f 500 0 0 500 0 5000\n", i, list[i].loop_partners_indexes[j], transform.position[0], transform.position[2], transform.orientation[2]);
		}
	}
}

void save_keyframe_pointclouds(std::vector<carmen_voslam_pointcloud_t> list)
{
	char filename[64];

	for (int i = 0; i < list.size(); i++)
	{
		sprintf(filename, "keyframe%3d.ply", i);
		pcl::io::savePLYFile(filename, *(list[i].pointcloud));
	}
}
