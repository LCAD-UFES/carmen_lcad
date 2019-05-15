//#include <pcl/common/eigen.h>
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

#include <tf.h>
#include "graphslam_params.h"
#include "velodyne_util.h"

using namespace tf;

carmen_localize_ackerman_param_t localize_param;
double timestamp_last_velodyne_received = 0;
char remission_file[256];

extern sensor_parameters_t velodyne_params;
extern sensor_data_t velodyne_data;

extern double robot_length;
extern double robot_width;
//extern double highest_sensor;
extern int use_raw_laser;
extern double max_range;
extern int correction_type;
extern int number_of_sensors;
extern double robot_wheel_radius;
extern carmen_pose_3D_t car_pose;
extern carmen_pose_3D_t velodyne_pose;
extern carmen_pose_3D_t gps_pose;
extern carmen_pose_3D_t xsens_pose;
extern double safe_range_above_sensors;
// extern ProbabilisticMapParams map_params;
extern sensor_data_t *spherical_sensor_data;
extern carmen_pose_3D_t sensor_board_1_pose;
extern sensor_parameters_t *spherical_sensor_params;
extern rotation_matrix *sensor_board_1_to_car_matrix;
extern double distance_between_rear_car_and_rear_wheels;
extern double distance_between_front_and_rear_axles;
extern double highest_point;
extern carmen_localize_ackerman_particle_filter_p filter;


carmen_map_t local_map;
carmen_robot_ackerman_config_t car_config;
carmen_compact_map_t local_compacted_map;
carmen_map_t local_sum_remission_map;
carmen_map_t local_mean_remission_map;
carmen_map_t local_variance_remission_map;
carmen_map_t local_sum_sqr_remission_map;
carmen_map_t local_count_remission_map;
carmen_compact_map_t local_compacted_mean_remission_map;
carmen_compact_map_t local_compacted_variance_remission_map;
carmen_localize_ackerman_binary_map_t binary_map;
cell_coords_t **map_cells_hit_by_each_rays = NULL;

int velodyne_viewer = 0;

tf::Transform board_to_velodyne_pose;
tf::Transform board_to_xsens_pose;
tf::Transform board_to_gps_pose;
tf::Transform car_to_board_pose;
tf::Transform world_to_car_pose;

rotation_matrix* r_matrix_car_to_global = NULL;
tf::Transformer transformer(false);

int current_point_cloud_partial_scan_index = 0;
int received_enough_pointclouds_partial_scans = 0;
carmen_pose_3D_t *corrected_pose;
rotation_matrix *corrected_pose_rotation = NULL;

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;


void
initialize_transforms()
{//
	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose.position.x, car_pose.position.y, car_pose.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose.orientation.yaw, car_pose.orientation.pitch, car_pose.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_1_pose.position.x, sensor_board_1_pose.position.y, sensor_board_1_pose.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_1_pose.orientation.yaw, sensor_board_1_pose.orientation.pitch, sensor_board_1_pose.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// velodyne pose with respect to the board
	board_to_velodyne_pose.setOrigin(tf::Vector3(velodyne_pose.position.x, velodyne_pose.position.y, velodyne_pose.position.z));
	board_to_velodyne_pose.setRotation(tf::Quaternion(velodyne_pose.orientation.yaw, velodyne_pose.orientation.pitch, velodyne_pose.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_velodyne_transform(board_to_velodyne_pose, tf::Time(0), "/board", "/velodyne");
	transformer.setTransform(board_to_velodyne_transform, "board_to_velodyne_transform");
}


static void
init_velodyne_points()
{
	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *) calloc (NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *) calloc (NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *) calloc (NUM_VELODYNE_POINT_CLOUDS, sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *) calloc (NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	unsigned char **intensity = (unsigned char **) calloc (NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char*));
	double *phi = (double *) calloc (NUM_VELODYNE_POINT_CLOUDS, sizeof(double));

	velodyne_data.points = velodyne_points;
	velodyne_data.intensity = intensity;
	velodyne_data.robot_pose = robot_pose;
	velodyne_data.robot_timestamp = robot_timestamp;
	velodyne_data.robot_velocity = robot_velocity;
	velodyne_data.robot_phi = phi;
}


void
initializeICP()
{
	gicp.setMaximumIterations(2000);
	gicp.setTransformationEpsilon(1e-5);
	gicp.setRotationEpsilon(1e-5);
	gicp.setMaxCorrespondenceDistance(10.0);
}


void
load_parameters(int argc, char **argv)
{
	ProbabilisticMapParams map_params;

	char *carmen_path = getenv("CARMEN_HOME");
	sprintf(remission_file, "%s/data/remission_calibration.txt", carmen_path);
	printf("remission_file: %s\n", remission_file);

	read_parameters_without_mapper(argc, argv, &localize_param, &map_params, remission_file);

	initialize_transforms();
	init_velodyne_points();
	initializeICP();
}


static void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points, int max_point_buffer)
{
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points)
			intensity[*point_cloud_index] = (unsigned char *)realloc((void *)intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


int
two_complete_clouds_have_been_acquired()
{
	if (received_enough_pointclouds_partial_scans)
		return 1;

	return 0;
}


carmen_vector_3D_t
get_global_point_from_velodyne_point(carmen_sphere_coord_t sphere_point, sensor_parameters_t *velodyne_params, rotation_matrix *r_matrix_car_to_global, carmen_vector_3D_t robot_position)
{
	carmen_vector_3D_t point_position_in_the_robot = carmen_get_sensor_sphere_point_in_robot_cartesian_reference(sphere_point, velodyne_params->pose, sensor_board_1_pose,
			velodyne_params->sensor_to_support_matrix, sensor_board_1_to_car_matrix);

	carmen_vector_3D_t global_point_position_in_the_world = carmen_change_sensor_reference(robot_position, point_position_in_the_robot, r_matrix_car_to_global);

	return global_point_position_in_the_world;
}


int
point_is_valid(carmen_sphere_coord_t sphere_point, sensor_parameters_t *velodyne_params)
{
	carmen_vector_3D_t point_position_in_the_robot = carmen_get_sensor_sphere_point_in_robot_cartesian_reference(sphere_point, velodyne_params->pose, sensor_board_1_pose,
			velodyne_params->sensor_to_support_matrix, sensor_board_1_to_car_matrix);

	double range = sphere_point.length;

//	if ((sphere_point.horizontal_angle > M_PI / 2.0) || (sphere_point.horizontal_angle < -M_PI / 2.0))
//		return 0;

	int ray_hit_the_car = carmen_prob_models_ray_hit_the_robot(
			distance_between_rear_car_and_rear_wheels,
			robot_length, robot_width, point_position_in_the_robot.x,
			point_position_in_the_robot.y);

	if (ray_hit_the_car || (range >= velodyne_params->range_max) || (point_position_in_the_robot.z >= highest_point))
		return 0;

	return 1;
}


void
add_velodyne_point_to_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, unsigned char intensity, carmen_vector_3D_t point)
{
	pcl::PointXYZRGB p3D;

	p3D.x = point.x;
	p3D.y = point.y;
	p3D.z = point.z;
	p3D.r = intensity;
	p3D.g = intensity;
	p3D.b = intensity;

	pointcloud->push_back(p3D);
}


static void
add_velodyne_spherical_points_to_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, spherical_point_cloud *v_zt, unsigned char *intensity, rotation_matrix *r_matrix_car_to_global,
				 carmen_pose_3D_t *robot_pose, sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data)
{
	int i, j, k;
	double dt = 0;
	carmen_pose_3D_t robot_interpolated_position;
	double v = velodyne_data->robot_velocity[velodyne_data->point_cloud_index].x;
	double phi = velodyne_data->robot_phi[velodyne_data->point_cloud_index];

	for (i = 0, j = 0; i < v_zt->num_points; i = i +  velodyne_params->vertical_resolution, j++, dt += velodyne_params->time_spent_by_each_scan)
	{
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(*robot_pose, dt, v, phi, distance_between_front_and_rear_axles);
		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		for(k = 0; k < velodyne_params->vertical_resolution; k++)
		{
			//carmen_vector_3D_t point = get_global_point_from_velodyne_point(v_zt->sphere_points[i + k], velodyne_params, r_matrix_car_to_global, robot_pose->position);
			carmen_vector_3D_t point = get_global_point_from_velodyne_point(v_zt->sphere_points[i + k], velodyne_params, r_matrix_car_to_global, robot_interpolated_position.position);

			if (point_is_valid(v_zt->sphere_points[i + k], velodyne_params))
				add_velodyne_point_to_pointcloud(pointcloud, intensity[i + k], point);
		}
	}
}


void
accumulate_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data)
{
	static int velodyne_message_id;
	int num_points = velodyne_message->number_of_32_laser_shots * velodyne_params->vertical_resolution;

	if (velodyne_data->last_timestamp == 0.0)
	{
		velodyne_data->last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2;		// correntemente sao necessarias pelo menos 2 mensagens para se ter uma volta completa de velodyne

		return;
	}

	velodyne_data->current_timestamp = velodyne_message->timestamp;
	build_sensor_point_cloud(&(velodyne_data->points), (velodyne_data->intensity), &(velodyne_data->point_cloud_index), num_points, NUM_VELODYNE_POINT_CLOUDS);
	carmen_velodyne_partial_scan_update_points(velodyne_message,
			velodyne_params->vertical_resolution,
			&(velodyne_data->points[velodyne_data->point_cloud_index]),
			(velodyne_data->intensity[velodyne_data->point_cloud_index]),
			velodyne_params->ray_order,
			velodyne_params->vertical_correction,
			velodyne_params->range_max,
			velodyne_message->timestamp);

	velodyne_message_id++;
	velodyne_data->last_timestamp = velodyne_message->timestamp;

	// fill the partial scans array
	if (current_point_cloud_partial_scan_index >= 1)
	{
		received_enough_pointclouds_partial_scans = 1;
	}
	else
		current_point_cloud_partial_scan_index++;
}


void
save_pointcloud_to_file(char *filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
	FILE *f = fopen(filename, "w");

	fprintf(f, "%ld\n", pointcloud->size());

	for (long i = 0; i < (long) pointcloud->size(); i++)
	{
		fprintf(f, "%f %f %f %d %d %d\n",
			pointcloud->at(i).x, pointcloud->at(i).y, pointcloud->at(i).z,
			(int) pointcloud->at(i).r, (int) pointcloud->at(i).g, (int) pointcloud->at(i).b
		);
	}

	fclose(f);
}


int
accumulate_clouds(carmen_velodyne_partial_scan_message *velodyne_message, char *velodyne_storage_dir, double v, double phi)
{
	static char cloud_name[1024];
	static int first_iteraction = 1;
	static carmen_pose_3D_t zero_pose;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud;
	static rotation_matrix *r_matrix_car_to_global;
	int current_point_cloud_index;

	carmen_vector_3D_t robot_velocity = {v, 0.0, 0.0};
	double pose_timestamp = 0.0;

	if (first_iteraction)
	{
		memset(&zero_pose, 0, sizeof(carmen_pose_3D_t));
		source_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, zero_pose.orientation);
		first_iteraction = 0;
	}

	accumulate_partial_scan(velodyne_message, &velodyne_params, &velodyne_data);

	if (two_complete_clouds_have_been_acquired())
	{
		current_point_cloud_index = velodyne_data.point_cloud_index;

		velodyne_data.robot_pose[velodyne_data.point_cloud_index] = zero_pose;
		velodyne_data.robot_phi[velodyne_data.point_cloud_index] = phi;
		velodyne_data.robot_velocity[velodyne_data.point_cloud_index] = robot_velocity;
		velodyne_data.robot_timestamp[velodyne_data.point_cloud_index] = pose_timestamp;

		add_velodyne_spherical_points_to_pointcloud(source_pointcloud, &(velodyne_data.points[current_point_cloud_index]), velodyne_data.intensity[current_point_cloud_index], r_matrix_car_to_global,
				&zero_pose, &velodyne_params, &velodyne_data);

		sprintf(cloud_name, "%s/%lf.ply", velodyne_storage_dir, velodyne_message->timestamp);
		save_pointcloud_to_file(cloud_name, source_pointcloud);

		// DEBUG:
//		char cloud_name[1024];
//		sprintf(cloud_name, "%s/CLOUDS-%lf.ply", velodyne_storage_dir, velodyne_message->timestamp);
//		pcl::io::savePLYFile(cloud_name, *source_pointcloud);

		source_pointcloud->clear();
		return 1;
	}

	return 0;
}
