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

#include "slam_icp_velodyne.h"

extern carmen_pose_3D_t sensor_board_1_pose;
extern rotation_matrix *sensor_board_1_to_car_matrix;
extern double robot_wheel_radius;
extern double robot_length;
extern double robot_width;
extern double distance_between_rear_car_and_rear_wheels;
extern double highest_sensor;
extern double distance_between_front_and_rear_axles;
extern double highest_point;

int first_iteraction = 1;
int current_point_cloud_partial_scan_index = 0;
int received_enough_pointclouds_partial_scans = 0;
carmen_pose_3D_t *corrected_pose;
rotation_matrix *corrected_pose_rotation = NULL;

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
Eigen::Matrix<double, 4, 4> accumulated_correction_transform[NUM_VELODYNE_POINT_CLOUDS];

static void build_sensor_point_cloud(spherical_point_cloud **points,
		unsigned char **intensity, int *point_cloud_index, int num_points,
		int max_point_buffer) {
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points)
		intensity[*point_cloud_index] = (unsigned char *) realloc(
				(void *) intensity[*point_cloud_index],
				num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}

int point_is_valid(carmen_sphere_coord_t sphere_point,
		sensor_parameters_t *velodyne_params) {
	carmen_vector_3D_t point_position_in_the_robot =
			carmen_get_sensor_sphere_point_in_robot_cartesian_reference(
					sphere_point, velodyne_params->pose, sensor_board_1_pose,
					velodyne_params->sensor_to_support_matrix,
					sensor_board_1_to_car_matrix);

	double range = sphere_point.length;
	double obstacle_z;

	//	if ((sphere_point.horizontal_angle > M_PI / 2.0) || (sphere_point.horizontal_angle < -M_PI / 2.0))
	//		return 0;

	int ray_hit_the_car = carmen_prob_models_ray_hit_the_robot(
			distance_between_rear_car_and_rear_wheels, robot_length,
			robot_width, point_position_in_the_robot.x,
			point_position_in_the_robot.y);

	if (ray_hit_the_car || (range >= velodyne_params->range_max)
			|| point_position_in_the_robot.z >= highest_point)
		return 0;

	return 1;
}

carmen_vector_3D_t get_global_point_from_velodyne_point(
		carmen_sphere_coord_t sphere_point,
		sensor_parameters_t *velodyne_params,
		rotation_matrix *r_matrix_car_to_global,
		carmen_vector_3D_t robot_position) {
	carmen_vector_3D_t point_position_in_the_robot =
			carmen_get_sensor_sphere_point_in_robot_cartesian_reference(
					sphere_point, velodyne_params->pose, sensor_board_1_pose,
					velodyne_params->sensor_to_support_matrix,
					sensor_board_1_to_car_matrix);

	carmen_vector_3D_t global_point_position_in_the_world =
			carmen_change_sensor_reference(robot_position,
					point_position_in_the_robot, r_matrix_car_to_global);

	return global_point_position_in_the_world;
}

void add_velodyne_point_to_pointcloud(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud,
		unsigned char intensity, carmen_vector_3D_t point) {
	pcl::PointXYZRGB p3D;

	p3D.x = point.x;
	p3D.y = point.y;
	p3D.z = point.z;
	p3D.r = intensity;
	p3D.g = intensity;
	p3D.b = intensity;

	pointcloud->push_back(p3D);
}

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
LeafSize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud, double size)
{
       pcl::VoxelGrid<pcl::PointXYZRGB> grid;
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointCloud  (new pcl::PointCloud<pcl::PointXYZRGB>);
       grid.setLeafSize(size, size, size);
       grid.setInputCloud(inputPointCloud);
       grid.filter(*outputPointCloud);
       return outputPointCloud;
}

static void add_velodyne_spherical_points_to_pointcloud(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud,
		spherical_point_cloud *v_zt, unsigned char *intensity,
		rotation_matrix *r_matrix_car_to_global, carmen_pose_3D_t *robot_pose,
		sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data) {
	int i, j, k;
	double dt = 0;
	carmen_pose_3D_t robot_interpolated_position;
	double v = velodyne_data->robot_velocity[velodyne_data->point_cloud_index].x;
	double phi = velodyne_data->robot_phi[velodyne_data->point_cloud_index];

	carmen_robot_ackerman_config_t car_config;
	car_config.distance_between_front_and_rear_axles = distance_between_front_and_rear_axles;
	car_config.distance_between_rear_wheels = robot_width;

	for (i = 0, j = 0; i < v_zt->num_points;
			i = i + 1 * velodyne_params->vertical_resolution, j += 1) {
		dt = j * velodyne_params->time_spent_by_each_scan;
		robot_interpolated_position =
				carmen_ackerman_interpolated_robot_position_at_time(*robot_pose,
						dt, v, phi, distance_between_front_and_rear_axles);

		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global,
				robot_interpolated_position.orientation);

		carmen_prob_models_compute_relevant_map_coordinates(velodyne_data, velodyne_params, i, robot_interpolated_position.position, sensor_board_1_pose,
				r_matrix_car_to_global, sensor_board_1_to_car_matrix, robot_wheel_radius, 0.0, 0.0, &car_config, 0);


		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(velodyne_data, velodyne_params, i,
				2.0, 10.0, velodyne_params->delta_difference_mean, velodyne_params->delta_difference_stddev);


		for (k = 0; k < velodyne_params->vertical_resolution; k++)
		{
			carmen_vector_3D_t point = get_global_point_from_velodyne_point(
					v_zt->sphere_points[i + k], velodyne_params,
					r_matrix_car_to_global,
					robot_interpolated_position.position);

			if (velodyne_data->occupancy_log_odds_of_each_ray_target[k] > velodyne_params->log_odds.log_odds_l0)
			{
//				printf("aqui\n");
							if (point_is_valid(v_zt->sphere_points[i + k], velodyne_params))
				add_velodyne_point_to_pointcloud(pointcloud, intensity[i + k],
						point);
			}
		}
	}
}



Eigen::Matrix<double, 4, 4> generate_eigen_pose_from_carmen_pose(
		carmen_pose_3D_t *robot_pose) {
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



void accumulate_partial_scan(
		carmen_velodyne_partial_scan_message *velodyne_message,
		sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data,
		carmen_pose_3D_t *robot_pose, carmen_vector_3D_t *robot_velocity,
		double phi, double robot_timestamp) {
	static int velodyne_message_id;
	int num_points = velodyne_message->number_of_32_laser_shots
			* velodyne_params->vertical_resolution;

	if (velodyne_data->last_timestamp == 0.0) {
		velodyne_data->last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2;// correntemente sao necessarias pelo menos 2 mensagens para se ter uma volta completa de velodyne

		return;
	}

	velodyne_data->current_timestamp = velodyne_message->timestamp;
	build_sensor_point_cloud(&(velodyne_data->points),
			(velodyne_data->intensity), &(velodyne_data->point_cloud_index),
			num_points, NUM_VELODYNE_POINT_CLOUDS);
	carmen_velodyne_partial_scan_update_points(velodyne_message,
			velodyne_params->vertical_resolution,
			&(velodyne_data->points[velodyne_data->point_cloud_index]),
			(velodyne_data->intensity[velodyne_data->point_cloud_index]),
			velodyne_params->ray_order, velodyne_params->vertical_correction,
			velodyne_params->range_max);

	velodyne_data->robot_pose[velodyne_data->point_cloud_index] = *robot_pose;
	velodyne_data->robot_phi[velodyne_data->point_cloud_index] = phi;
	velodyne_data->robot_velocity[velodyne_data->point_cloud_index] =
			*robot_velocity;
	velodyne_data->robot_timestamp[velodyne_data->point_cloud_index] =
			robot_timestamp;

	velodyne_message_id++;
	velodyne_data->last_timestamp = velodyne_message->timestamp;

	// fill the partial scans array
	if (current_point_cloud_partial_scan_index >= 1)
	{
		received_enough_pointclouds_partial_scans = 1;
	} else
	{
		accumulated_correction_transform[velodyne_data->point_cloud_index] = generate_eigen_pose_from_carmen_pose(robot_pose);
		current_point_cloud_partial_scan_index++;
	}

}

int two_complete_clouds_have_been_acquired() {
	if (received_enough_pointclouds_partial_scans)
		return 1;

	return 0;
}

void transform_pcl_pose_to_carmen_pose(
		Eigen::Matrix<double, 4, 4> pcl_corrected_pose,
		carmen_pose_3D_t *corrected_pose) {
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

void initializeICP() {
	gicp.setMaximumIterations(50);
	gicp.setTransformationEpsilon(1e-5);
	gicp.setRotationEpsilon(1e-5);
	gicp.setMaxCorrespondenceDistance(0.1 /* 0.1 */);
	gicp.setEuclideanFitnessEpsilon(0.1);
}

int runGeneralizedICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pointcloud,
		Eigen::Matrix<double, 4, 4> source_pointcloud_pose,
		Eigen::Matrix<double, 4, 4> target_pointcloud_pose,
		Eigen::Matrix<double, 4, 4> *out_pose,
		int current_index,
		int before_index)
{

	static int is_first_icp_run = 1;

	Eigen::Matrix<double, 4, 4> out_transform;
	Eigen::Matrix<double, 4, 4> src_transform;
	Eigen::Matrix<double, 4, 4> final_transform;

	/**************************************/
	/********** Initializations ***********/
	/**************************************/

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_pcl_pointcloud_corrected =
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(
					new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_pcl_pointcloud_corrected =
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(
					new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud < pcl::PointXYZRGB > out_pcl_pointcloud;
	pcl::PointCloud < pcl::PointXYZRGB > out_pcl_pointcloud_transformed;

	/**************************************/
	/**************** GICP ****************/
	/**************************************/

//	if (is_first_icp_run) {
//		accumulated_correction_transform =
//				Eigen::Matrix<double, 4, 4>().Identity();
//		is_first_icp_run = 0;
//	}

//	source_pointcloud_pose = accumulated_correction_transform
//			* source_pointcloud_pose;
//	target_pointcloud_pose = accumulated_correction_transform
//			* target_pointcloud_pose;

	pcl::transformPointCloud(*source_pointcloud, *src_pcl_pointcloud_corrected,
			(target_pointcloud_pose.inverse() * source_pointcloud_pose).cast<float>());

	pcl::transformPointCloud(*target_pointcloud, *tgt_pcl_pointcloud_corrected,
			Eigen::Matrix<float, 4, 4>().Identity());

	gicp.setInputCloud(src_pcl_pointcloud_corrected);
	gicp.setInputTarget(tgt_pcl_pointcloud_corrected);
	gicp.align(out_pcl_pointcloud);

	out_transform = //target_pointcloud_pose *
			gicp.getFinalTransformation().cast<double>();
//			* target_pointcloud_pose.inverse();
	//*corrected_pose = final_transform = source_pointcloud_pose;

	src_pcl_pointcloud_corrected->clear();
	tgt_pcl_pointcloud_corrected->clear();

	if (gicp.hasConverged())
	{
		*out_pose = out_transform;
		return 1;
	}

	return 0;
}

void show_pointclouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt,
		carmen_vector_3D_t position) {
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	viewer.setBackgroundColor(0, 0, 0);

	viewer.addPointCloud < pcl::PointXYZRGB > (src, "Source Cloud");
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0,
			"Source Cloud");

	viewer.addPointCloud < pcl::PointXYZRGB > (tgt, "Target Cloud");
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0, 1.0,
			"Target Cloud");

	viewer.addCoordinateSystem(1.0, position.x, position.y, position.z, 0);
	viewer.initCameraParameters();

	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	};
}

int slam_icp_velodyne_partial_scan(
		carmen_velodyne_partial_scan_message *velodyne_message,
		sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data,
		carmen_pose_3D_t *robot_pose, carmen_vector_3D_t *robot_velocity,
		double phi, double last_globalpos_timestamp, carmen_pose_3D_t *corrected_pose_out) {
	// initializations
	static carmen_pose_3D_t zero_pose;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud, target_pointcloud, source_pointcloud_leafed, target_pointcloud_leafed;
	static rotation_matrix *r_matrix_car_to_global;
	int converge = 0, current_point_cloud_index, before_point_cloud_index;
	Eigen::Matrix<double, 4, 4> source_pointcloud_pose, target_pointcloud_pose;
	Eigen::Matrix<double, 4, 4> pcl_corrected_pose, correction_transform;
	static int i = 1;
	static int icp_lag = 1;

	if (first_iteraction) {
		memset(&zero_pose, 0, sizeof(carmen_pose_3D_t));
		robot_pose->orientation.pitch = robot_pose->orientation.roll =
				robot_pose->position.z = 0.0;
		corrected_pose = (carmen_pose_3D_t *) calloc(1,
				sizeof(carmen_pose_3D_t));
		*corrected_pose = *robot_pose;
		first_iteraction = 0;
		source_pointcloud =
				boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(
						new pcl::PointCloud<pcl::PointXYZRGB>);
		target_pointcloud =
				boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(
						new pcl::PointCloud<pcl::PointXYZRGB>);
		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global,
				zero_pose.orientation);
	}
	*corrected_pose = *robot_pose;
	accumulate_partial_scan(velodyne_message, velodyne_params, velodyne_data,
			robot_pose, robot_velocity, phi, last_globalpos_timestamp);

	if (two_complete_clouds_have_been_acquired()) {
		current_point_cloud_index = velodyne_data->point_cloud_index;
		before_point_cloud_index = ((velodyne_data->point_cloud_index - icp_lag)
									+ NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;
//		if (i < 3)
//		{
//			before_point_cloud_index = ((velodyne_data->point_cloud_index - i)
//							+ NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;
//			i++;
//		}

		add_velodyne_spherical_points_to_pointcloud(target_pointcloud,
				&(velodyne_data->points[before_point_cloud_index]),
				velodyne_data->intensity[before_point_cloud_index],
				r_matrix_car_to_global, &zero_pose, velodyne_params,
				velodyne_data);

		target_pointcloud_leafed = LeafSize(target_pointcloud, 0.2);

		add_velodyne_spherical_points_to_pointcloud(source_pointcloud,
				&(velodyne_data->points[current_point_cloud_index]),
				velodyne_data->intensity[current_point_cloud_index],
				r_matrix_car_to_global, &zero_pose, velodyne_params,
				velodyne_data);

		source_pointcloud_leafed = LeafSize(source_pointcloud, 0.2);

		source_pointcloud_pose = generate_eigen_pose_from_carmen_pose(
				&velodyne_data->robot_pose[current_point_cloud_index]);
		target_pointcloud_pose = generate_eigen_pose_from_carmen_pose(
				&velodyne_data->robot_pose[before_point_cloud_index]);

		source_pointcloud_pose = //accumulated_correction_transform[before_point_cloud_index] *
				//generate_eigen_pose_from_carmen_pose(&velodyne_data->robot_pose[before_point_cloud_index]).inverse() *
				generate_eigen_pose_from_carmen_pose(&velodyne_data->robot_pose[current_point_cloud_index]);

//		before_point_cloud_index = ((velodyne_data->point_cloud_index - 1)
//						+ NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;

		target_pointcloud_pose =
				generate_eigen_pose_from_carmen_pose(&velodyne_data->robot_pose[before_point_cloud_index]);//.inverse() *
				//generate_eigen_pose_from_carmen_pose(&velodyne_data->robot_pose[current_point_cloud_index]);

		converge = runGeneralizedICP(source_pointcloud_leafed, target_pointcloud_leafed,
				source_pointcloud_pose, target_pointcloud_pose,
				&correction_transform,
				current_point_cloud_index,
				before_point_cloud_index);
//
//		before_point_cloud_index = ((velodyne_data->point_cloud_index - 1)
//								+ NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;
//
//				target_pointcloud_pose =
//						generate_eigen_pose_from_carmen_pose(&velodyne_data->robot_pose[before_point_cloud_index]);//.inverse() *



//				* generate_eigen_pose_from_carmen_pose(
//						&velodyne_data->robot_pose[current_point_cloud_index]);

		source_pointcloud->clear();
		target_pointcloud->clear();
		source_pointcloud_leafed->clear();
		target_pointcloud_leafed->clear();

		if (converge)
		{
			accumulated_correction_transform[current_point_cloud_index] =
							accumulated_correction_transform[before_point_cloud_index] *
							correction_transform *	target_pointcloud_pose.inverse() * source_pointcloud_pose;

			pcl_corrected_pose = accumulated_correction_transform[current_point_cloud_index];
			transform_pcl_pose_to_carmen_pose(pcl_corrected_pose,
						corrected_pose);
		}else
		{
			velodyne_data->point_cloud_index = ((velodyne_data->point_cloud_index - 1)
										+ NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;
//			accumulated_correction_transform[current_point_cloud_index] =
//							accumulated_correction_transform[before_point_cloud_index] *
//							target_pointcloud_pose.inverse() * source_pointcloud_pose;
		}
	}

	corrected_pose->orientation.pitch = corrected_pose->orientation.roll =
			corrected_pose->position.z = 0.0;

	*corrected_pose_out = *corrected_pose;

	return converge;
}

carmen_pose_3D_t apply_transform(carmen_pose_3D_t *robot_pose) {

	Eigen::Matrix<double, 4, 4> source_pointcloud_pose;

	source_pointcloud_pose = accumulated_correction_transform[0]
			* generate_eigen_pose_from_carmen_pose(robot_pose);
	transform_pcl_pose_to_carmen_pose(source_pointcloud_pose, corrected_pose);

	corrected_pose->orientation.pitch = corrected_pose->orientation.roll =
			corrected_pose->position.z = 0.0;

	return (*corrected_pose);
}

//carmen_pose_3D_t pose;
//carmen_pose_3D_t pose_sum;
//memset(&pose_sum, 0, sizeof(carmen_pose_3D_t));
//
//Eigen::Matrix<double, 4, 4> fused_odometry = Eigen::Matrix<double, 4, 4>().Identity();
//for (i = 0; i < icp_lag; i++)
//{
//	current_point_cloud_index = ((velodyne_data->point_cloud_index - i)
//								+ NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;
//	before_point_cloud_index = ((velodyne_data->point_cloud_index - (i + 1))
//								+ NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;
//
//	pcl_corrected_pose = accumulated_correction_transform[current_point_cloud_index] * fused_odometry;
//
//	source_pointcloud_pose = generate_eigen_pose_from_carmen_pose(&velodyne_data->robot_pose[current_point_cloud_index]);
//
//	target_pointcloud_pose = generate_eigen_pose_from_carmen_pose(&velodyne_data->robot_pose[before_point_cloud_index]);
//
//	fused_odometry = target_pointcloud_pose.inverse() * source_pointcloud_pose * fused_odometry;
//
//	transform_pcl_pose_to_carmen_pose(pcl_corrected_pose,
//			&pose);
//
//	pose_sum.position.x += pose.position.x;
//	pose_sum.position.y += pose.position.y;
//	pose_sum.orientation.yaw += pose.orientation.yaw;
//
//}
//
//pose_sum.position.x /= icp_lag;
//pose_sum.position.y /= icp_lag;
//pose_sum.orientation.yaw = carmen_normalize_theta(pose_sum.orientation.yaw / icp_lag);
