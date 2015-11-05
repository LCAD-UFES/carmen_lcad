#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <carmen/localize_ackerman_core.h>
#include <carmen/rotation_geometry.h>

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

#include <tf.h>
#include <pcl/io/ply_io.h>



int first_iteraction = 1;
int current_point_cloud_partial_scan_index = 0;
int received_enough_pointclouds_partial_scans = 0;
carmen_pose_3D_t *corrected_pose;
rotation_matrix *corrected_pose_rotation = NULL;

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;


static Eigen::Matrix<double, 4, 4>
generate_eigen_pose_from_carmen_pose(carmen_pose_3D_t *robot_pose)
{
	tf::Transform pose;
	Eigen::Matrix<double, 4, 4> result;
	tf::Matrix3x3 rotation;

	pose.setOrigin(
			tf::Vector3(robot_pose->position.x, robot_pose->position.y,
					robot_pose->position.z));
	pose.setRotation(
			tf::Quaternion(robot_pose->orientation.yaw,
					robot_pose->orientation.pitch,
					robot_pose->orientation.roll));

	rotation = tf::Matrix3x3(pose.getRotation());

	//rotation
	result(0, 0) = rotation[0][0];
	result(0, 1) = rotation[0][1];
	result(0, 2) = rotation[0][2];
	result(1, 0) = rotation[1][0];
	result(1, 1) = rotation[1][1];
	result(1, 2) = rotation[1][2];
	result(2, 0) = rotation[2][0];
	result(2, 1) = rotation[2][1];
	result(2, 2) = rotation[2][2];

	//translation
	result(0, 3) = pose.getOrigin().x();
	result(1, 3) = pose.getOrigin().y();
	result(2, 3) = pose.getOrigin().z();

	result(3, 0) = 0.0;
	result(3, 1) = 0.0;
	result(3, 2) = 0.0;
	result(3, 3) = 1.0;

	return result;
}


static void
transform_pcl_pose_to_carmen_pose(Eigen::Matrix<double, 4, 4> pcl_corrected_pose,
		carmen_pose_3D_t *corrected_pose)
{
	tf::Matrix3x3 rotation;
	double roll, pitch, yaw;

	rotation[0][0] = pcl_corrected_pose(0, 0);
	rotation[0][1] = pcl_corrected_pose(0, 1);
	rotation[0][2] = pcl_corrected_pose(0, 2);
	rotation[1][0] = pcl_corrected_pose(1, 0);
	rotation[1][1] = pcl_corrected_pose(1, 1);
	rotation[1][2] = pcl_corrected_pose(1, 2);
	rotation[2][0] = pcl_corrected_pose(2, 0);
	rotation[2][1] = pcl_corrected_pose(2, 1);
	rotation[2][2] = pcl_corrected_pose(2, 2);

	rotation.getRPY(roll, pitch, yaw);
	corrected_pose->orientation.roll = roll;
	corrected_pose->orientation.pitch = pitch;
	corrected_pose->orientation.yaw = yaw;

	corrected_pose->position.x = pcl_corrected_pose(0, 3);
	corrected_pose->position.y = pcl_corrected_pose(1, 3);
	corrected_pose->position.z = pcl_corrected_pose(2, 3);

	compute_rotation_matrix(corrected_pose_rotation,
			corrected_pose->orientation);
}



int
main(int argv, char **agrc)
{
	int i;
	FILE *fin = fopen(agrc[1], "r");
	FILE *fin2 = fopen(agrc[2], "r");
	FILE *fin3 = fopen(agrc[3], "r");
	FILE *fout = fopen("localize.txt", "w");
	FILE *fout2 = fopen("xsens.txt", "w");
	FILE *fout3 = fopen("kalman.txt", "w");
	carmen_pose_3D_t pose;
	carmen_pose_3D_t centro_pose;

	memset(&centro_pose, 0, sizeof(carmen_pose_3D_t));
	memset(&pose, 0, sizeof(carmen_pose_3D_t));

	double time;
	Eigen::Matrix<double, 4, 4> T;
	Eigen::Matrix<double, 4, 4> T1;
	Eigen::Matrix<double, 4, 4> F;
	centro_pose.position.x = 1.3775;
	F = generate_eigen_pose_from_carmen_pose(&centro_pose);
	fscanf(fin, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);
	T1 = generate_eigen_pose_from_carmen_pose(&pose);

	for (i = 0; i < 1500; i++)
	{
		fscanf(fin, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);
		if (feof(fin))
			break;
		T = T1.inverse() * generate_eigen_pose_from_carmen_pose(&pose);
		transform_pcl_pose_to_carmen_pose(T * F, &pose);
	//	printf("%lf %lf %lf\n", pose.position.x, pose.position.y, pose.orientation.yaw);

	}
	fscanf(fin, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);
	T1 = generate_eigen_pose_from_carmen_pose(&pose);

	for (i = 0; i < 4000; i++)
	{
		fscanf(fin, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);
		if (feof(fin))
			break;
		T = T1.inverse() * generate_eigen_pose_from_carmen_pose(&pose);
		transform_pcl_pose_to_carmen_pose(T * F, &pose);
		fprintf(fout, "%lf %lf %lf\n", pose.position.x, pose.position.y, pose.orientation.yaw);

	}

	for (i = 0; i < 5280; i++)
	{
		fscanf(fin2, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);
		if (feof(fin2))
			break;
		T = T1.inverse() * generate_eigen_pose_from_carmen_pose(&pose);
		transform_pcl_pose_to_carmen_pose(T * F, &pose);
	//	printf("%lf %lf %lf\n", pose.position.x, pose.position.y, pose.orientation.yaw);

	}
	fscanf(fin2, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);

	for (i = 0; i < 16580; i++)
	{
		fscanf(fin2, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);
		if (feof(fin2))
			break;
		T = T1.inverse() * generate_eigen_pose_from_carmen_pose(&pose);
		transform_pcl_pose_to_carmen_pose(T * F, &pose);
		fprintf(fout2, "%lf %lf %lf\n", pose.position.x, pose.position.y, pose.orientation.yaw);

	}

	for (i = 0; i < 5280; i++)
	{
		fscanf(fin3, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);
		if (feof(fin3))
			break;
		T = T1.inverse() * generate_eigen_pose_from_carmen_pose(&pose);
		transform_pcl_pose_to_carmen_pose(T * F, &pose);
	//	printf("%lf %lf %lf\n", pose.position.x, pose.position.y, pose.orientation.yaw);

	}
	fscanf(fin3, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);

	for (i = 0; i < 16580; i++)
	{
		fscanf(fin3, "%lf %lf %lf %lf", &pose.position.x, &pose.position.y, &pose.orientation.yaw, &time);
		if (feof(fin3))
			break;
		T = T1.inverse() * generate_eigen_pose_from_carmen_pose(&pose);
		transform_pcl_pose_to_carmen_pose(T * F, &pose);
		fprintf(fout3, "%lf %lf %lf\n", pose.position.x, pose.position.y, pose.orientation.yaw);

	}
}
