#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/laser_ldmrs_interface.h>
#include <carmen/laser_ldmrs_utils.h>
#include <carmen/rotation_geometry.h>
#include <carmen/mapper_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/stereo_mapping_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_util.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <carmen/parking_assistant_interface.h>
#include <carmen/libdeeplab.h>
#include <omp.h>
#include "mapper.h"
#include <sys/stat.h>

#include "message_interpolation.cpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION


carmen_map_t offline_map;
// TODO: @@@ Alberto: essa variavel eh definida como externa dentro da lib do mapper. Corrigir!
carmen_localize_ackerman_globalpos_message *globalpos_history;
int last_globalpos;

static int visual_odometry_is_global_pos = 0;
static int parking_assistant_found_safe_space = 0;

tf::Transformer tf_transformer(false);
MessageInterpolation<carmen_localize_ackerman_globalpos_message, carmen_ultrasonic_sonar_sensor_message> interpolator(1);

static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t sensor_board_pose_g;
static carmen_pose_3D_t ultrasonic_sensor_r1_g;
static carmen_pose_3D_t ultrasonic_sensor_r2_g;
static carmen_pose_3D_t ultrasonic_sensor_l1_g;
static carmen_pose_3D_t ultrasonic_sensor_l2_g;


double safe_range_above_sensors;
double robot_wheel_radius;

int use_simulator_pose = 0;

double highest_sensor = 0.0;

int merge_with_offline_map;
int build_snapshot_map;
int update_cells_below_car;
int update_and_merge_with_mapper_saved_maps;
int update_and_merge_with_snapshot_map;
int decay_to_offline_map;
int create_map_sum_and_count;
int use_remission;

int process_semantic = 0;
int file_warnings = 1;
int verbose = 0;

carmen_pose_3D_t sensor_board_1_pose;
carmen_pose_3D_t front_bullbar_pose;

sensor_parameters_t *sensors_params;
sensor_parameters_t ultrasonic_sensor_params;
sensor_data_t *sensors_data;
int number_of_sensors;

carmen_camera_parameters camera_params[MAX_CAMERA_INDEX + 1];
carmen_pose_3D_t camera_pose[MAX_CAMERA_INDEX + 1];
camera_data_t camera_data[MAX_CAMERA_INDEX + 1];
cv::Mat camera_image_semantic[MAX_CAMERA_INDEX + 1];
int camera_alive[MAX_CAMERA_INDEX + 1];
int active_cameras;

static long globalpos_msg_count = 0;
static long truepos_msg_count = 0;
static long *sensor_msg_count;
static long *map_update_count;
static long camera_msg_count[MAX_CAMERA_INDEX + 1];
static long camera_filter_count[MAX_CAMERA_INDEX + 1];
static long camera_datmo_count[MAX_CAMERA_INDEX + 1];

carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t laser_ldmrs_pose;

char *map_path;

int publish_moving_objects_raw_map;

carmen_rddf_annotation_message last_rddf_annotation_message;
int robot_near_strong_slow_down_annotation = 0;

bool offline_map_available = false;
int ok_to_publish = 0;
int number_of_threads = 1;

int camera3_ready = 0;

/******variables for neural_mapper dataset*****/
int generate_neural_mapper_dataset = 0;
int neural_mapper_max_distance_meters = 0;
int neural_mapper_data_pace = 0;
/**********************/

rotation_matrix *r_matrix_car_to_global = NULL;

int use_truepos = 0;

extern carmen_mapper_virtual_laser_message virtual_laser_message;

extern carmen_moving_objects_point_clouds_message moving_objects_message;

extern carmen_mapper_virtual_scan_message virtual_scan_message;

carmen_robot_ackerman_config_t car_config;
carmen_map_config_t map_config;

char *calibration_file = NULL;
char *save_calibration_file = NULL;

static char *segmap_dirname = (char *) "/dados/log_dante_michelini-20181116.txt_segmap";


inline double
distance2(image_cartesian a, image_cartesian b)
{
	double dx = a.cartesian_x - b.cartesian_x;
	double dy = a.cartesian_y - b.cartesian_y;

	return (dx * dx + dy * dy);
}


vector<int>
query(double d2, int i, const vector<image_cartesian> &points, std::vector<bool> clustered)
{
	vector<int> neighbors;
	const image_cartesian &point = points[i];

	for (size_t j = 0; j < points.size(); j++)
	{
		if ((distance2(point, points[j]) < d2) && !clustered[j])
			neighbors.push_back(j);
	}

	return (neighbors);
}


vector<vector<image_cartesian>>
dbscan_compute_clusters(double d2, size_t density, const vector<image_cartesian> &points)
{
	vector<vector<image_cartesian>> clusters;
	vector<bool> clustered(points.size(), false);

	for (size_t i = 0; i < points.size(); ++i)
	{
		// Ignore already clustered points.
		if (clustered[i])
			continue;

		// Ignore points without enough neighbors.
		vector<int> neighbors = query(d2, i, points, clustered);
		if (neighbors.size() < density)
			continue;

		// Create a new cluster with the i-th point as its first element.
		vector<image_cartesian> c;
		clusters.push_back(c);
		vector<image_cartesian> &cluster = clusters.back();
		cluster.push_back(points[i]);
		clustered[i] = true;

		// Add the point's neighbors (and possibly their neighbors) to the cluster.
		for (size_t j = 0; j < neighbors.size(); ++j)
		{
			int k = neighbors[j];
			if (clustered[k])
				continue;

			cluster.push_back(points[k]);
			clustered[k] = true;

			vector<int> farther = query(d2, k, points, clustered);
			if (farther.size() >= density)
				neighbors.insert(neighbors.end(), farther.begin(), farther.end());
		}
	}
	return (clusters);
}


cv::Scalar
cluster_color[] =
{
	cv::Scalar(255, 0, 0),
    cv::Scalar(255, 255, 0),
    cv::Scalar(255, 0, 255),
    cv::Scalar(0, 255, 255),
    cv::Scalar(150, 150, 150),
    cv::Scalar(150, 150, 0),
    cv::Scalar(150, 0, 150),
    cv::Scalar(0, 150, 150),
    cv::Scalar(107, 142, 35),
    cv::Scalar(152, 251, 152),
    cv::Scalar(70, 130, 180),
    cv::Scalar(220, 20, 60),
    cv::Scalar(255, 0, 0),
    cv::Scalar(0, 0, 142),
    cv::Scalar(0, 0, 70),
    cv::Scalar(0, 60, 100),
    cv::Scalar(0, 80, 100),
    cv::Scalar(0, 0, 230),
    cv::Scalar(119, 11, 32),
};


static const cv::Vec3b
colormap[] =
{
    cv::Vec3b(128, 64, 128),
    cv::Vec3b(244, 35, 232),
    cv::Vec3b(70, 70, 70),
    cv::Vec3b(102, 102, 156),
    cv::Vec3b(190, 153, 153),
    cv::Vec3b(153, 153, 153),
    cv::Vec3b(250, 170, 30),
    cv::Vec3b(220, 220, 0),
    cv::Vec3b(107, 142, 35),
    cv::Vec3b(152, 251, 152),
    cv::Vec3b(70, 130, 180),
    cv::Vec3b(220, 20, 60),
    cv::Vec3b(255, 0, 0),
    cv::Vec3b(0, 0, 142),
    cv::Vec3b(0, 0, 70),
    cv::Vec3b(0, 60, 100),
    cv::Vec3b(0, 80, 100),
    cv::Vec3b(0, 0, 230),
    cv::Vec3b(119, 11, 32),
};


void
get_occupancy_log_odds_of_each_ray_target(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int scan_index)
{
	int thread_id = omp_get_thread_num();
	int point_cloud_index = sensor_data->point_cloud_index;
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	int N = v_zt.num_points / sensor_params->vertical_resolution;
	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];
	double dt = sensor_params->time_spent_by_each_scan;
	double dt1 = sensor_data->points_timestamp[point_cloud_index] - sensor_data->robot_timestamp[point_cloud_index] - (double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
	carmen_pose_3D_t robot_pose = sensor_data->robot_pose[point_cloud_index];
	double dt2 = dt * scan_index / sensor_params->vertical_resolution;
	robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_pose,
			dt1 + dt2, v, phi, car_config.distance_between_front_and_rear_axles);

	r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

	change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[scan_index].horizontal_angle);

	carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data, sensor_params, scan_index, robot_interpolated_position.position,
			sensor_params->sensor_support_pose, r_matrix_car_to_global, sensor_params->support_to_car_matrix,
			robot_wheel_radius, offline_map.config.x_origin, offline_map.config.y_origin, &car_config, robot_near_strong_slow_down_annotation, thread_id, use_remission);

	carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, scan_index, highest_sensor, safe_range_above_sensors,
			robot_near_strong_slow_down_annotation, thread_id);
			// Updates: sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][ray_index] : 0 <= ray_index < 32
}


void
filter_sensor_data_using_one_image(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int camera_index, int image_index)
{
	camera_filter_count[camera_index]++;
	int filter_datmo_count = 0;
	cv::Scalar laser_ray_color;
	int image_width  = camera_data[camera_index].width[image_index];
	int image_height = camera_data[camera_index].height[image_index];
	double fx_meters = camera_params[camera_index].fx_factor * camera_params[camera_index].pixel_size * image_width;
	double fy_meters = camera_params[camera_index].fy_factor * camera_params[camera_index].pixel_size * image_height;
	double cu = camera_params[camera_index].cu_factor * image_width;
	double cv = camera_params[camera_index].cv_factor * image_height;
//	double fov = camera_params[camera_index].fov;
	double map_resolution = map_config.resolution;
	int img_planar_depth = (double) 0.5 * sensor_params->range_max / map_resolution;
	cv::Mat img_planar = cv::Mat(cv::Size(img_planar_depth * 2, img_planar_depth), CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat img = camera_image_semantic[camera_index];
	int cloud_index = sensor_data->point_cloud_index;
	int number_of_laser_shots = sensor_data->points[cloud_index].num_points / sensor_params->vertical_resolution;
	int thread_id = omp_get_thread_num();

	vector<image_cartesian> points;

	for (int j = 0; j < number_of_laser_shots; j++)
	{
		int scan_index = j * sensor_params->vertical_resolution;
		double horizontal_angle = - sensor_data->points[cloud_index].sphere_points[scan_index].horizontal_angle;

		if (fabs(carmen_normalize_theta(horizontal_angle - camera_pose[camera_index].orientation.yaw)) > M_PI_2) // Disregard laser shots out of the camera's field of view
			continue;

//		if (horizontal_angle > M_PI_2 || horizontal_angle < M_PI_2 ) // Disregard laser shots out of the camera's field of view
//			continue;

		get_occupancy_log_odds_of_each_ray_target(sensor_params, sensor_data, scan_index);

//		double previous_range = sensor_data->points[cloud_index].sphere_points[p].length;
//		double previous_vertical_angle = sensor_data->points[cloud_index].sphere_points[p].vertical_angle;
//		tf::Point previous_velodyne_p3d = spherical_to_cartesian(horizontal_angle, previous_vertical_angle, previous_range);

		for (int i = 1; i < sensor_params->vertical_resolution; i++)
		{
			double vertical_angle = sensor_data->points[cloud_index].sphere_points[scan_index + i].vertical_angle;
			double range = sensor_data->points[cloud_index].sphere_points[scan_index + i].length;

			tf::Point velodyne_p3d = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
			tf::Point camera_p3d = move_to_camera_reference(velodyne_p3d, velodyne_pose, camera_pose[camera_index]);

//			if (velodyne_p3d.x() > 0.0)
//				continue;

			int image_x = fx_meters * ( camera_p3d.y() / camera_p3d.x()) / camera_params[camera_index].pixel_size + cu;
			int image_y = fy_meters * (-camera_p3d.z() / camera_p3d.x()) / camera_params[camera_index].pixel_size + cv;

//			if (image_x < 0 || image_x >= image_width || image_y < 0 || image_y >= image_height) // Disregard laser rays out of the image window
//				continue;

//			double log_odds = get_log_odds_via_unexpeted_delta_range(sensor_params, sensor_data, i, scan_index, robot_near_strong_slow_down_annotation, thread_id);
			double log_odds = sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i];
			double prob = carmen_prob_models_log_odds_to_probabilistic(log_odds);

			// Jose's method for checking if a point is an obstacle
//			double delta_x = abs(velodyne_p3d.x() - previous_velodyne_p3d.x());
//			double delta_z = abs(velodyne_p3d.z() - previous_velodyne_p3d.z());
//			double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));
//			previous_velodyne_p3d = velodyne_p3d;

//			if (range <= MIN_RANGE || range >= sensor_params->range_max) // Disregard laser rays out of distance range
//				laser_ray_color = cv::Scalar(0, 255, 255);
//			else if (line_angle <= MIN_ANGLE_OBSTACLE || line_angle >= MAX_ANGLE_OBSTACLE) // Disregard laser rays that didn't hit an obstacle
//				laser_ray_color = cv::Scalar(0, 255, 0);
//			else if (!is_moving_object(camera_data[camera_index].semantic[image_index][image_x + image_y * image_width])) // Disregard if it is not a moving object
//			/*else*/ if (camera_data[camera_index].semantic[image_index][image_x + image_y * image_width] < 11)
//				laser_ray_color = cv::Scalar(255, 0, 0);
//			else
//			{
//				laser_ray_color = cv::Scalar(0, 0, 255);
//				sensor_data->points[cloud_index].sphere_points[p].length = 0.01; //sensor_params->range_max; // Make this laser ray out of range
//				filter_datmo_count++;
//			}
//			if (line_angle > MIN_ANGLE_OBSTACLE && line_angle < MAX_ANGLE_OBSTACLE && range > MIN_RANGE && range < sensor_params->range_max) // Laser ray probably hit an obstacle
//			if (log_odds > sensors_params->log_odds.log_odds_l0 && range > MIN_RANGE && range < sensor_params->range_max) // Laser ray probably hit an obstacle
			if (prob > 0.5 && range > MIN_RANGE && range < sensor_params->range_max) // Laser ray probably hit an obstacle
			{
				image_cartesian point;
				point.shot_number = j;
				point.ray_number  = i;
				point.image_x     = image_x;
				point.image_y     = image_y;
				point.cartesian_x = camera_p3d.x();
				point.cartesian_y = -camera_p3d.y();  // Must be inverted because Velodyne angle is reversed with CARMEN angles
				point.cartesian_z = camera_p3d.z();
				points.push_back(point);

				if (verbose >= 2)
				{
					int ix = (double) image_x / image_width  * img.cols / 2;
					int iy = (double) image_y / image_height * img.rows;
					if (ix >= 0 && ix < (img.cols / 2) && iy >= 0 && iy < img.rows)
					{
						circle(img, cv::Point(ix, iy), 1, cv::Scalar(0, 0, 255), 1, 8, 0);
						circle(img, cv::Point(ix + img.cols / 2, iy), 1, cv::Scalar(0, 0, 255), 1, 8, 0);
					}
					int px = (double) camera_p3d.y() / map_resolution + img_planar_depth;
					int py = (double) img_planar.rows - 1 - camera_p3d.x() / map_resolution;
					if (px >= 0 && px < img_planar.cols && py >= 0 && py < img_planar.rows)
						img_planar.at<cv::Vec3b>(cv::Point(px, py)) = cv::Vec3b(0, 0, 255);
				}
			}
		}
	}

	vector<vector<image_cartesian>> filtered_points = dbscan_compute_clusters(0.5, 5, points);;

	for (unsigned int i = 0; i < filtered_points.size(); i++)
	{
		bool is_moving_obstacle = false;
		unsigned int cont = 0;
		for (unsigned int j = 0; j < filtered_points[i].size(); j++)
		{
			if (filtered_points[i][j].image_x < 0 || filtered_points[i][j].image_x >= image_width || filtered_points[i][j].image_y < 0 || filtered_points[i][j].image_y >= image_height) // Disregard laser rays out of the image window
				continue;

			if ((camera_data[camera_index].semantic[image_index] != NULL) &&
			    (camera_data[camera_index].semantic[image_index][filtered_points[i][j].image_x + (filtered_points[i][j].image_y * image_width)] >= 11)) // 0 to 10 : Static objects
				cont++;
			if (cont > 10 || cont > (filtered_points[i].size() / 5))
			{
				is_moving_obstacle = true;
				break;
			}
		}

		if (is_moving_obstacle)
		{
			for (unsigned int j = 0; j < filtered_points[i].size(); j++)
			{
				int p = filtered_points[i][j].shot_number * sensor_params->vertical_resolution + filtered_points[i][j].ray_number;
				sensor_data->points[cloud_index].sphere_points[p].length = 0.01; //sensor_params->range_max; // Make this laser ray out of range
				filter_datmo_count++;

				if (verbose >= 2)
				{
					int ix = (double) filtered_points[i][j].image_x / image_width  * img.cols / 2;
					int iy = (double) filtered_points[i][j].image_y / image_height * img.rows;
					if (ix >= 0 && ix < (img.cols / 2) && iy >= 0 && iy < img.rows)
						circle(img, cv::Point(ix, iy), 1, cluster_color[i], 1, 8, 0);

					int px = (double) -filtered_points[i][j].cartesian_y / map_resolution + img_planar_depth;
					int py = (double) img_planar.rows - 1 - filtered_points[i][j].cartesian_x / map_resolution;
					if (px >= 0 && px < img_planar.cols && py >= 0 && py < img_planar.rows)
						img_planar.at<cv::Vec3b>(cv::Point(px, py)) = cv::Vec3b(cluster_color[i][0], cluster_color[i][1], cluster_color[i][2]);
				}
			}
		}
	}
	if (verbose >= 2)
	{
		imshow("Image Semantic Segmentation", img);
    	resize(img_planar, img_planar, cv::Size(0, 0), 3.5, 3.5, cv::INTER_NEAREST);
		imshow("Velodyne Semantic Map", img_planar);
		cv::waitKey(1);
	}
	if (filter_datmo_count)
		camera_datmo_count[camera_index]++;
}


vector<carmen_vector_2D_t> moving_objecst_cells_vector;


void
erase_moving_obstacles_cells(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int camera_index, int image_index)
{
	camera_filter_count[camera_index]++;
	int filter_datmo_count = 0;
	int image_width  = camera_data[camera_index].width[image_index];
	int image_height = camera_data[camera_index].height[image_index];
	double fx_meters = camera_params[camera_index].fx_factor * camera_params[camera_index].pixel_size * image_width;
	double fy_meters = camera_params[camera_index].fy_factor * camera_params[camera_index].pixel_size * image_height;
	double cu = camera_params[camera_index].cu_factor * image_width;
	double cv = camera_params[camera_index].cv_factor * image_height;
	double map_resolution = map_config.resolution;
	int img_planar_depth = (double) 0.5 * sensor_params->range_max / map_resolution;
	int cloud_index = sensor_data->point_cloud_index;
	int number_of_laser_shots = sensor_data->points[cloud_index].num_points / sensor_params->vertical_resolution;
	int thread_id = omp_get_thread_num();

	carmen_map_p map = get_the_map();
	cv::Mat img_planar = cv::Mat(cv::Size(img_planar_depth * 2, img_planar_depth), CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat cluster_img = cv::Mat(map->config.x_size, map->config.y_size, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat img = camera_image_semantic[camera_index];

	vector<image_cartesian> rays;

	for (int j = 0; j < number_of_laser_shots; j++)
	{
		int scan_index = j * sensor_params->vertical_resolution;
		double horizontal_angle = - sensor_data->points[cloud_index].sphere_points[scan_index].horizontal_angle;

//		if (fabs(carmen_normalize_theta(horizontal_angle - camera_pose[camera_index].orientation.yaw)) > M_PI_2) // Disregard laser shots out of the camera's field of view
//			continue;

		// Compute the log odds each ray the log odds is the logarithm of the probability
		get_occupancy_log_odds_of_each_ray_target(sensor_params, sensor_data, scan_index);

		for (int i = 1; i < sensor_params->vertical_resolution; i++)
		{
			double range = sensor_data->points[cloud_index].sphere_points[scan_index + i].length;

			tf::Point velodyne_p3d = spherical_to_cartesian(horizontal_angle, sensor_data->points[cloud_index].sphere_points[scan_index + i].vertical_angle, range);
			tf::Point camera_p3d = move_to_camera_reference(velodyne_p3d, velodyne_pose, camera_pose[camera_index]);

			int image_x = fx_meters * ( camera_p3d.y() / camera_p3d.x()) / camera_params[camera_index].pixel_size + cu;
			int image_y = fy_meters * (-camera_p3d.z() / camera_p3d.x()) / camera_params[camera_index].pixel_size + cv;

			double prob = carmen_prob_models_log_odds_to_probabilistic(sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i]);

			if (prob > 0.5 && range > MIN_RANGE && range < sensor_params->range_max) // Laser ray probably hit an obstacle
			{
				image_cartesian ray;
				ray.shot_number = 0;
				if (image_x >= 0 && image_x <= image_width && image_y >= 0 && image_y <= image_height) // Disregard laser rays out of the image window
					ray.ray_number  = camera_data[camera_index].semantic[image_index][image_x + (image_y * image_width)];
				else
					ray.ray_number  = -1;
				ray.image_x = image_x;
				ray.image_y = image_y;
				ray.cartesian_x = sensor_data->ray_position_in_the_floor[thread_id][i].x;
				ray.cartesian_y = sensor_data->ray_position_in_the_floor[thread_id][i].y;
				ray.cartesian_z = 0.0;
				rays.push_back(ray);

				//printf("%lf O %lf\n", sensor_data->ray_position_in_the_floor[thread_id][i].x, sensor_data->ray_origin_in_the_floor[thread_id][i].x);

				if (verbose >= 2)
				{
					int ix = (double) image_x / image_width  * img.cols / 2;
					int iy = (double) image_y / image_height * img.rows;
					if (ix >= 0 && ix < (img.cols / 2) && iy >= 0 && iy < img.rows)
					{
						circle(img, cv::Point(ix, iy), 1, cv::Scalar(0, 0, 255), 1, 8, 0);
						circle(img, cv::Point(ix + img.cols / 2, iy), 1, cv::Scalar(0, 0, 255), 1, 8, 0);
					}
//					int px = (double) camera_p3d.y() / map_resolution + img_planar_depth;
//					int py = (double) img_planar.rows - 1 - camera_p3d.x() / map_resolution;
//					if (px >= 0 && px < img_planar.cols && py >= 0 && py < img_planar.rows)
//						img_planar.at<cv::Vec3b>(cv::Point(px, py)) = cv::Vec3b(0, 0, 255);
					int px = (double) (ray.cartesian_x - map_config.x_origin) / map_config.resolution;
					int py = (double) (ray.cartesian_y - map_config.y_origin) / map_config.resolution;
					//cluster_img.at<cv::Vec3b>(cv::Point(px, py)) = cv::Vec3b(cluster_color[i][0], cluster_color[i][1], cluster_color[i][2]);
					//printf ("%d %d\n", px, (map_config.y_size - 1 - py));
					rectangle(cluster_img, cv::Rect(px, (map_config.y_size - 1 - py), 1, 1), cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
				}
			}
		}
	}

	vector<vector<image_cartesian>> filtered_points = dbscan_compute_clusters(0.2, 5, rays);

	for (unsigned int i = 0; i < filtered_points.size(); i++)
	{
		bool is_moving_obstacle = false;
		unsigned int cont = 0;
		for (unsigned int j = 0; j < filtered_points[i].size(); j++)
		{
			if (filtered_points[i][j].ray_number < 0) // Disregard laser rays out of the image window
				continue;

			if ((camera_data[camera_index].semantic[image_index] != NULL) &&  // TODO is it necessary?
			    (camera_data[camera_index].semantic[image_index][filtered_points[i][j].image_x + (filtered_points[i][j].image_y * image_width)] >= 11)) // 0 to 10 : Static objects
				cont++;
			if (cont > 10 || cont > (filtered_points[i].size() / 5))
			{
				is_moving_obstacle = true;
				break;
			}
		}

		if (is_moving_obstacle)
		{
			for (unsigned int j = 0; j < filtered_points[i].size(); j++)
			{
				carmen_vector_2D_t point;
				point.x = filtered_points[i][j].cartesian_x;
				point.y = filtered_points[i][j].cartesian_y;

				moving_objecst_cells_vector.push_back(point);
				filter_datmo_count++;

				if (verbose >= 2)   // TODO color the clusters on the camera image
				{
					int px = (double) (point.x - map_config.x_origin) / map_config.resolution;
					int py = (double) (point.y - map_config.y_origin) / map_config.resolution;
					//cluster_img.at<cv::Vec3b>(cv::Point(px, py)) = cv::Vec3b(cluster_color[i][0], cluster_color[i][1], cluster_color[i][2]);
					//printf ("%d %d\n", px, (map_config.y_size - 1 - py));
					rectangle(cluster_img, cv::Rect(px, (map_config.y_size - 1 - py), 1, 1), cluster_color[i], CV_FILLED, 8, 0);
				}
			}
		}
	}
	if (verbose >= 2)
	{
		int x = ((sensor_data->ray_origin_in_the_floor[thread_id][1].x  - map_config.x_origin)  / map_config.resolution) - 100;
		int y = (map_config.y_size - 1 - ((sensor_data->ray_origin_in_the_floor[thread_id][1].y  - map_config.x_origin) / map_config.resolution)) - 100;
		//printf("%d %d %d %d\n", cluster_img.cols, cluster_img.rows, x, y);

		cv::Mat cimg = cluster_img;
		if (x >= 0 && x <= cluster_img.cols && y >= 0 && y <= cluster_img.rows)
			cimg = cluster_img(cv::Rect(x, y, 200, 200));
		resize(cimg, cimg, cv::Size(0, 0), 3.5, 3.5, cv::INTER_NEAREST);

		imshow("Image Semantic Segmentation", img);
		imshow("Clusters", cimg);
		//cv::waitKey(1);
		cv_draw_map(moving_objecst_cells_vector);
	}
	if (filter_datmo_count)
		camera_datmo_count[camera_index]++;
}


void
erase_moving_obstacles_cells_old(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int camera_index, int image_index)
{
	camera_filter_count[camera_index]++;
	int image_width  = camera_data[camera_index].width[image_index];
	int image_height = camera_data[camera_index].height[image_index];
	double fx_meters = camera_params[camera_index].fx_factor * camera_params[camera_index].pixel_size * image_width;
	double fy_meters = camera_params[camera_index].fy_factor * camera_params[camera_index].pixel_size * image_height;
	double cu = camera_params[camera_index].cu_factor * image_width;
	double cv = camera_params[camera_index].cv_factor * image_height;
	int cloud_index = sensor_data->point_cloud_index;
	int number_of_laser_shots = sensor_data->points[cloud_index].num_points / sensor_params->vertical_resolution;
	int thread_id = omp_get_thread_num();

	//vector<carmen_vector_2D_t> moving_objecst_cells_vector;

	for (int j = 0; j < number_of_laser_shots; j++)
	{
		int scan_index = j * sensor_params->vertical_resolution;
		double horizontal_angle = - sensor_data->points[cloud_index].sphere_points[scan_index].horizontal_angle;

		if (fabs(carmen_normalize_theta(horizontal_angle - camera_pose[camera_index].orientation.yaw)) > M_PI_2) // Disregard laser shots out of the camera's field of view
			continue;

		//if (horizontal_angle > M_PI_2 || horizontal_angle < M_PI_2 ) // Disregard laser shots out of the camera's field of view
		//	continue;

		get_occupancy_log_odds_of_each_ray_target(sensor_params, sensor_data, scan_index);

		for (int i = 1; i < sensor_params->vertical_resolution; i++)
		{
			double vertical_angle = sensor_data->points[cloud_index].sphere_points[scan_index + i].vertical_angle;
			double range = sensor_data->points[cloud_index].sphere_points[scan_index + i].length;

			tf::Point velodyne_p3d = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
			tf::Point camera_p3d = move_to_camera_reference(velodyne_p3d, velodyne_pose, camera_pose[camera_index]);

			int image_x = fx_meters * ( camera_p3d.y() / camera_p3d.x()) / camera_params[camera_index].pixel_size + cu;
			int image_y = fy_meters * (-camera_p3d.z() / camera_p3d.x()) / camera_params[camera_index].pixel_size + cv;

			double log_odds = sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i];
			double prob = carmen_prob_models_log_odds_to_probabilistic(log_odds);

			if ((prob > 0.5 && range > MIN_RANGE && range < sensor_params->range_max) &&                                                                            // Laser ray probably hit an obstacle
				(camera_data[camera_index].semantic[image_index] != NULL) && (image_x >= 0 && image_x <= image_width && image_y >= 0 && image_y <= image_height) && // Disregard laser rays out of the image window
				(camera_data[camera_index].semantic[image_index][image_x + (image_y * image_width)] >= 11))                                                         // 0 to 10 : Static objects >= 11 Moving Objects
			{
				printf ("PIF %lf %lf\n", sensor_data->ray_position_in_the_floor[thread_id][i].x, sensor_data->ray_position_in_the_floor[thread_id][i].y);
				moving_objecst_cells_vector.push_back(sensor_data->ray_position_in_the_floor[thread_id][i]);
			}
		}
	}
	if (verbose >= 2)
	{
//		printf ("S %d\n", (int) moving_objecst_cells_vector.size());
		//imshow("Image Semantic Segmentation", cv::Mat(camera_data[camera_index].height[image_index], camera_data[camera_index].width[image_index], CV_8UC1, camera_data[camera_index].semantic[image_index], 0));
		//imshow("Image Semantic Segmentation", camera_image_semantic[camera_index]);
		//cv::waitKey(1);
		cv_draw_map(moving_objecst_cells_vector);
	}
}


void
filter_sensor_data_using_one_image_old(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int camera_index, int image_index)
{
	camera_filter_count[camera_index]++;
	int filter_datmo_count = 0;
	cv::Scalar laser_ray_color;
	int image_width  = camera_data[camera_index].width[image_index];
	int image_height = camera_data[camera_index].height[image_index];
	double fx_meters = camera_params[camera_index].fx_factor * camera_params[camera_index].pixel_size * image_width;
	double fy_meters = camera_params[camera_index].fy_factor * camera_params[camera_index].pixel_size * image_height;
	double cu = camera_params[camera_index].cu_factor * image_width;
	double cv = camera_params[camera_index].cv_factor * image_height;
	double fov = camera_params[camera_index].fov;
	cv::Mat img = camera_image_semantic[camera_index];
	int cloud_index = sensor_data->point_cloud_index;
	int number_of_laser_shots = sensor_data->points[cloud_index].num_points / sensor_params->vertical_resolution;
//	int thread_id = omp_get_thread_num();

	for (int j = 0; j < number_of_laser_shots; j++)
	{
		int p = j * sensor_params->vertical_resolution;
		double horizontal_angle = - sensor_data->points[cloud_index].sphere_points[p].horizontal_angle;
		if (fabs(carmen_normalize_theta(horizontal_angle - camera_pose[camera_index].orientation.yaw)) > fov) // Disregard laser shots out of the camera's field of view
			continue;

		double previous_range = sensor_data->points[cloud_index].sphere_points[p].length;
		double previous_vertical_angle = sensor_data->points[cloud_index].sphere_points[p].vertical_angle;
		tf::Point previous_velodyne_p3d = spherical_to_cartesian(horizontal_angle, previous_vertical_angle, previous_range);

		for (int i = 1; i < sensor_params->vertical_resolution; i++)
		{
			p++;
			double vertical_angle = sensor_data->points[cloud_index].sphere_points[p].vertical_angle;
			double range = sensor_data->points[cloud_index].sphere_points[p].length;
			tf::Point velodyne_p3d = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
			tf::Point camera_p3d = move_to_camera_reference(velodyne_p3d, velodyne_pose, camera_pose[camera_index]);
			int image_x = fx_meters * ( camera_p3d.y() / camera_p3d.x()) / camera_params[camera_index].pixel_size + cu;
			int image_y = fy_meters * (-camera_p3d.z() / camera_p3d.x()) / camera_params[camera_index].pixel_size + cv;
			if (image_x < 0 || image_x >= image_width || image_y < 0 || image_y >= image_height) // Disregard laser rays out of the image window
				continue;

//			double log_odds = get_log_odds_via_unexpeted_delta_range(sensor_params, sensor_data, i, j, robot_near_strong_slow_down_annotation, thread_id);

			// Jose's method for checking if a point is an obstacle
			double delta_x = fabs(velodyne_p3d.x() - previous_velodyne_p3d.x());
			double delta_z = fabs(velodyne_p3d.z() - previous_velodyne_p3d.z());
			double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));
			previous_velodyne_p3d = velodyne_p3d;

			(void) line_angle;
//			if (range <= MIN_RANGE || range >= sensor_params->range_max) // Disregard laser rays out of distance range
//				laser_ray_color = cv::Scalar(0, 255, 255);
//			else
//			if (line_angle <= MIN_ANGLE_OBSTACLE || line_angle >= MAX_ANGLE_OBSTACLE) // Disregard laser rays that didn't hit an obstacle
//			if (log_odds < MIN_LOG_ODDS) // Disregard laser rays that didn't hit an obstacle
//				laser_ray_color = cv::Scalar(0, 255, 0);
//			else
			if ((camera_data[camera_index].semantic[image_index] != NULL) &&
			    (camera_data[camera_index].semantic[image_index][image_x + image_y * image_width] < 11)) // Disregard if it is not a moving object (0 to 10)
//				(!is_moving_object(camera_data[camera_index].semantic[image_index][image_x + image_y * image_width])) // Disregard if it is not a moving object
				laser_ray_color = cv::Scalar(255, 0, 0);
			else
			{
				laser_ray_color = cv::Scalar(0, 0, 255);
				sensor_data->points[cloud_index].sphere_points[p].length = 0.01; //sensor_params->range_max; // Make this laser ray out of range
				filter_datmo_count++;
			}
			if (verbose >= 2)
			{
				int ix = (double) image_x / image_width  * img.cols / 2;
				int iy = (double) image_y / image_height * img.rows;
				circle(img, cv::Point(ix, iy), 1, laser_ray_color, 1, 8, 0);
				circle(img, cv::Point(ix + img.cols / 2, iy), 1, laser_ray_color, 1, 8, 0);
			}
		}
	}
	if (verbose >= 2)
		imshow("Image Semantic Segmentation", img), cv::waitKey(1);
	if (filter_datmo_count)
		camera_datmo_count[camera_index]++;
}


int
filter_sensor_data_using_image_semantic_segmentation(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data)
{
	int filter_cameras = 0;

	if (active_cameras == 0)
		return 0;

	for (int camera = 1; camera <= MAX_CAMERA_INDEX; camera++)
	{
		if (camera_alive[camera] < 0)
			continue;

		int nearest_index = -1;
		double nearest_time_diff = DBL_MAX;

		for (int i = 0; i < NUM_CAMERA_IMAGES; i++)
		{
			if (camera_data[camera].image[i] == NULL)
				continue;

			int j = sensor_data->point_cloud_index;
			double time_difference = fabs(camera_data[camera].timestamp[i] - sensor_data->points_timestamp[j] - CAMERA_DELAY);
			if (time_difference < nearest_time_diff)
			{
				nearest_index = i;
				nearest_time_diff = time_difference;
			}
		}

		if (nearest_time_diff > MAX_TIMESTAMP_DIFFERENCE)
			continue;

		//filter_sensor_data_using_one_image(sensor_params, sensor_data, camera, nearest_index);
		erase_moving_obstacles_cells(sensor_params, sensor_data, camera, nearest_index);

		filter_cameras++;
	}

	return filter_cameras;
}


bool
check_lidar_camera_max_timestamp_difference(sensor_data_t *sensor_data)
{
	if (active_cameras == 0)
		return false;

	for (int camera = 1; camera <= MAX_CAMERA_INDEX; camera++)
	{
		if (camera_alive[camera] < 0)
			continue;

		for (int i = 0; i < NUM_CAMERA_IMAGES; i++)
		{
			if (camera_data[camera].image[i] == NULL)
				continue;

			if (fabs(camera_data[camera].timestamp[i] - sensor_data->points_timestamp[sensor_data->point_cloud_index] - CAMERA_DELAY) < MAX_TIMESTAMP_DIFFERENCE)
				return true;
		}
	}
	return false;
}


void
include_sensor_data_into_map(int sensor_number, carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	map_update_count[sensor_number]++;
	int i, old_point_cloud_index = -1;
	int nearest_global_pos = 0;
	double nearest_time = globalpos_message->timestamp;
	double old_globalpos_timestamp;
	carmen_pose_3D_t old_robot_position;

	for (i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		double time_difference = fabs(sensors_data[sensor_number].points_timestamp[i] - globalpos_message->timestamp);
		if (time_difference < nearest_time)
		{
			nearest_global_pos = i;
			nearest_time = time_difference;
		}
		if (time_difference == 0.0)
			break;
	}

	i = nearest_global_pos;
	old_point_cloud_index = sensors_data[sensor_number].point_cloud_index;
	old_robot_position = sensors_data[sensor_number].robot_pose[i];
	old_globalpos_timestamp = sensors_data[sensor_number].robot_timestamp[i];
	sensors_data[sensor_number].point_cloud_index = i;
	sensors_data[sensor_number].robot_pose[i] = globalpos_message->pose;
	sensors_data[sensor_number].robot_timestamp[i] = globalpos_message->timestamp;

////Used in Article
//	if (filter_sensor_data_using_image_semantic_segmentation(&sensors_params[sensor_number], &sensors_data[sensor_number]) > 0)
//		run_mapper(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);

////New - Erase cells occupied by moving obstacles
	if (check_lidar_camera_max_timestamp_difference(&sensors_data[sensor_number]))
	{
		run_mapper(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);
		filter_sensor_data_using_image_semantic_segmentation(&sensors_params[sensor_number], &sensors_data[sensor_number]);
	}
	//cv_draw_map();

	sensors_data[sensor_number].point_cloud_index = old_point_cloud_index;
	sensors_data[sensor_number].robot_pose[i] = old_robot_position;
	sensors_data[sensor_number].robot_timestamp[i] = old_globalpos_timestamp;
}


void
free_virtual_scan_message()
{
	if (virtual_scan_message.num_sensors != 0)
	{
		for (int i = 0; i < virtual_scan_message.num_sensors; i++)
		{
			free(virtual_scan_message.virtual_scan_sensor[i].points);
			free(virtual_scan_message.virtual_scan_sensor);
			virtual_scan_message.virtual_scan_sensor = NULL;
			virtual_scan_message.num_sensors = 0;
		}
	}
}


void
print_stats_header()
{
	fprintf(stderr, "\n%9s %9s ", "globalpos", "truepos");

	for (int i = 0; i < number_of_sensors; i++)
		if (sensors_params[i].alive)
			fprintf(stderr, "%8s%d %8s%d ", "sensor", i, "update", i);

	for (int k = 1; k <= MAX_CAMERA_INDEX; k++)
		if (camera_alive[k] >= 0)
			fprintf(stderr, "%8s%d %8s%d %8s%d ", "camera", k, "filter", k, "datmo", k);

	fprintf(stderr, "\n");
}


void
print_stats()
{
	const double time_interval = 5.0;
	static double last_print_time = globalpos_history[0].timestamp;
	if (globalpos_history[last_globalpos].timestamp - last_print_time < time_interval)
		return;

	const int lines_page = 50;
	static int line_count = 0;
	if (line_count % lines_page == 0)
		print_stats_header();

	fprintf(stderr, "%9ld %9ld ", globalpos_msg_count, truepos_msg_count);

	for (int i = 0; i < number_of_sensors; i++)
		if (sensors_params[i].alive)
			fprintf(stderr, "%9ld %9ld ", sensor_msg_count[i], map_update_count[i]);

	for (int k = 1; k <= MAX_CAMERA_INDEX; k++)
		if (camera_alive[k] >= 0)
			fprintf(stderr, "%9ld %9ld %9ld ", camera_msg_count[k], camera_filter_count[k], camera_datmo_count[k]);

	fprintf(stderr, "\n");

	last_print_time = globalpos_history[last_globalpos].timestamp;
	line_count++;
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_map(double timestamp)
{
	mapper_publish_map(timestamp);
}


void
publish_virtual_scan(double timestamp)
{
	carmen_mapper_publish_virtual_scan_message(&virtual_scan_message, timestamp);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	globalpos_msg_count++;
	if (visual_odometry_is_global_pos)
		interpolator.AddMessageToInterpolationList(globalpos_message);
	else
		mapper_set_robot_pose_into_the_map(globalpos_message, update_cells_below_car);

	// Map annotations handling
	double distance_to_nearest_annotation = 1000.0;
	int index_of_nearest_annotation = 0;
	for (int i = 0; i < last_rddf_annotation_message.num_annotations; i++)
	{
		double distance_to_annotation = DIST2D(last_rddf_annotation_message.annotations[i].annotation_point,
				globalpos_history[last_globalpos].pose.position);
		if ((distance_to_annotation < distance_to_nearest_annotation) &&
			((last_rddf_annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BUMP) ||
			 (last_rddf_annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BARRIER) ||
			 ((last_rddf_annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			  (last_rddf_annotation_message.annotations[i].annotation_code <= RDDF_ANNOTATION_CODE_SPEED_LIMIT_20))))
		{
			distance_to_nearest_annotation = distance_to_annotation;
			index_of_nearest_annotation = i;
		}
	}
	if (((distance_to_nearest_annotation < 35.0) &&
		 carmen_rddf_play_annotation_is_forward(globalpos_message->globalpos,
				 last_rddf_annotation_message.annotations[index_of_nearest_annotation].annotation_point)) ||
		(distance_to_nearest_annotation < 8.0))
		robot_near_strong_slow_down_annotation = 1;
	else
		robot_near_strong_slow_down_annotation = 0;

	if (ok_to_publish)
	{
		free_virtual_scan_message();

		// A ordem eh importante
		if (sensors_params[VELODYNE].alive)
			include_sensor_data_into_map(VELODYNE, globalpos_message);
		if (sensors_params[LASER_LDMRS].alive && !robot_near_strong_slow_down_annotation)
			include_sensor_data_into_map(LASER_LDMRS, globalpos_message);
		if (sensors_params[3].alive && camera3_ready)	// camera 3
			include_sensor_data_into_map(3, globalpos_message);

//		cv_draw_map();

		camera3_ready = 0;

		publish_map(globalpos_message->timestamp);
		publish_virtual_scan(globalpos_message->timestamp);
	}

	if (verbose >= 1)
		print_stats();
}


static void
true_pos_message_handler(carmen_simulator_ackerman_truepos_message *pose)
{
	truepos_msg_count++;
	if (offline_map_available)
	{
		map_decay_to_offline_map(get_the_map());
		publish_map(pose->timestamp);
	}
}


static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	sensor_msg_count[VELODYNE]++;
	mapper_velodyne_partial_scan(VELODYNE, velodyne_message);
}


static void
laser_ldrms_message_handler(carmen_laser_ldmrs_message *laser) // old handler not used anymore
{
	sensor_msg_count[LASER_LDMRS]++;
	carmen_velodyne_partial_scan_message partial_scan_message = carmen_laser_ldmrs_convert_laser_scan_to_partial_velodyne_message(laser, laser->timestamp);

	if (partial_scan_message.number_of_32_laser_shots > 0)
	{
		mapper_velodyne_partial_scan(LASER_LDMRS, &partial_scan_message);
		free(partial_scan_message.partial_scan);
	}
}


static void
laser_ldrms_new_message_handler(carmen_laser_ldmrs_new_message *laser)
{
	sensor_msg_count[LASER_LDMRS]++;

//	FILE *f = fopen("scan.txt", "a");
//	fprintf(f, "\n\n%d %lf %lf %d %f %f %d \n\n",
//			laser->scan_number,
//			laser->scan_start_time,
//			laser->scan_end_time,
//			laser->angle_ticks_per_rotation,
//			laser->start_angle,
//			laser->end_angle,
//			laser->scan_points);
//
//	for (int i = 0; i < laser->scan_points; i++)
//	{
//		fprintf(f, "index %d, layer %d, ha %f, va %f, d %f, flags %d \n", i, laser->arraypoints[i].layer,
//					carmen_radians_to_degrees(laser->arraypoints[i].horizontal_angle),
//					carmen_radians_to_degrees(laser->arraypoints[i].vertical_angle),
//					laser->arraypoints[i].radial_distance,
//					laser->arraypoints[i].flags);
//	}
//	fflush(f);
//	fclose(f);

	carmen_velodyne_partial_scan_message partial_scan_message = carmen_laser_ldmrs_new_convert_laser_scan_to_partial_velodyne_message(laser, laser->timestamp);

//	f = fopen("scan.txt", "a");
//	fprintf(f, "\n\n%d %lf %lf %d %f %f %d \n\n",
//			laser->scan_number,
//			laser->scan_start_time,
//			laser->scan_end_time,
//			laser->angle_ticks_per_rotation,
//			laser->start_angle,
//			laser->end_angle,
//			laser->scan_points);
//
//	for (int i = 0; i < partial_scan_message.number_of_32_laser_shots; i++)
//	{
//		for (int j = 0; j < 4; j++)
//			fprintf(f, "index %d, ha %lf, d %lf\n", i,
//						partial_scan_message.partial_scan[i].angle,
//						(double) partial_scan_message.partial_scan[i].distance[j] / 500.0);
//	}
//	fflush(f);
//	fclose(f);

	if (partial_scan_message.number_of_32_laser_shots > 0)
	{
		mapper_velodyne_partial_scan(1, &partial_scan_message);
		free(partial_scan_message.partial_scan);
	}
}


static void
velodyne_variable_scan_message_handler2(carmen_velodyne_variable_scan_message *message)
{
	sensor_msg_count[2]++;
	mapper_velodyne_variable_scan(2, message);
}


static void
velodyne_variable_scan_message_handler3(carmen_velodyne_variable_scan_message *message)
{
	sensor_msg_count[3]++;
	camera3_ready = mapper_velodyne_variable_scan(3, message);
}


static void
velodyne_variable_scan_message_handler4(carmen_velodyne_variable_scan_message *message)
{
	sensor_msg_count[4]++;
	mapper_velodyne_variable_scan(4, message);
}


static void
velodyne_variable_scan_message_handler5(carmen_velodyne_variable_scan_message *message)
{
	sensor_msg_count[5]++;
	mapper_velodyne_variable_scan(5, message);
}


static void
velodyne_variable_scan_message_handler6(carmen_velodyne_variable_scan_message *message)
{
	sensor_msg_count[6]++;
	mapper_velodyne_variable_scan(6, message);
}


static void
velodyne_variable_scan_message_handler7(carmen_velodyne_variable_scan_message *message)
{
	sensor_msg_count[7]++;
	mapper_velodyne_variable_scan(7, message);
}


static void
velodyne_variable_scan_message_handler8(carmen_velodyne_variable_scan_message *message)
{
	sensor_msg_count[8]++;
	mapper_velodyne_variable_scan(8, message);
}


static void
velodyne_variable_scan_message_handler9(carmen_velodyne_variable_scan_message *message)
{
	sensor_msg_count[9]++;
	mapper_velodyne_variable_scan(9, message);
}


static void
offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	static bool first_time = true;
	carmen_position_t map_origin;

	if (first_time)
	{
		offline_map_available = true;
		first_time = false;
	}

	map_origin.x = msg->config.x_origin;
	map_origin.y = msg->config.y_origin;

	memcpy(offline_map.complete_map, msg->complete_map, msg->config.x_size * msg->config.y_size * sizeof(double));
	offline_map.config = msg->config;

	mapper_change_map_origin_to_another_map_block(&map_origin);

	if (merge_with_offline_map)
		mapper_merge_online_map_with_offline_map(&offline_map);
}


static void
parking_sensor_goal_message_handler(carmen_parking_assistant_goal_message *message __attribute__ ((unused)))
{
	parking_assistant_found_safe_space = 1;
}


static void
ultrasonic_sensor_message_handler(carmen_ultrasonic_sonar_sensor_message *message)
{
	carmen_point_t Xt_r1, Xt_r2, Xt_l1, Xt_l2;

	tf::Transform world_to_car_pose;
	double yaw, pitch, roll;

	int i;

	carmen_localize_ackerman_globalpos_message globalpos_message;
	globalpos_message = interpolator.InterpolateMessages(message);

	mapper_set_robot_pose_into_the_map(&globalpos_message, update_cells_below_car);

	if (parking_assistant_found_safe_space)
	{
		publish_map(message->timestamp);
		return;
	}

	world_to_car_pose.setOrigin(tf::Vector3(globalpos_message.pose.position.x, globalpos_message.pose.position.y, globalpos_message.pose.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(globalpos_message.pose.orientation.yaw, globalpos_message.pose.orientation.pitch, globalpos_message.pose.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	tf_transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	//SENSOR R1 - FRONTAL
	tf::StampedTransform world_to_ultrasonic_sensor_r1;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_r1", tf::Time(0), world_to_ultrasonic_sensor_r1);
	Xt_r1.x = world_to_ultrasonic_sensor_r1.getOrigin().x();
	Xt_r1.y = world_to_ultrasonic_sensor_r1.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_r1.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_r1.theta = yaw;

	double range[180];

	for (i=0 ; i<180 ; i++)
		range[i] = (double) message->sensor[3];

	mapper_update_grid_map(Xt_r1, range, &ultrasonic_sensor_params);

	//SENSOR R2 - LATERAL FRONTAL
	tf::StampedTransform world_to_ultrasonic_sensor_r2;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_r2", tf::Time(0), world_to_ultrasonic_sensor_r2);
	Xt_r2.x = world_to_ultrasonic_sensor_r2.getOrigin().x();
	Xt_r2.y = world_to_ultrasonic_sensor_r2.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_r2.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_r2.theta = yaw;

	for (i = 0; i < 180 ;i++)
		range[i] = (double) message->sensor[2];

	mapper_update_grid_map(Xt_r2, range, &ultrasonic_sensor_params);

	//SENSOR L2 - LATERAL TRASEIRO
	tf::StampedTransform world_to_ultrasonic_sensor_l2;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_l2", tf::Time(0), world_to_ultrasonic_sensor_l2);
	Xt_l2.x = world_to_ultrasonic_sensor_l2.getOrigin().x();
	Xt_l2.y = world_to_ultrasonic_sensor_l2.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_l2.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_l2.theta = yaw;

	for (i = 0; i < 180 ;i++)
		range[i] = (double) message->sensor[1];

	mapper_update_grid_map(Xt_l2, range, &ultrasonic_sensor_params);

	//SENSOR L1 - TRASEIRO
	tf::StampedTransform world_to_ultrasonic_sensor_l1;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_l1", tf::Time(0), world_to_ultrasonic_sensor_l1);
	Xt_l1.x = world_to_ultrasonic_sensor_l1.getOrigin().x();
	Xt_l1.y = world_to_ultrasonic_sensor_l1.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_l1.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_l1.theta = yaw;

	for (i = 0; i < 180; i++)
		range[i] = (double) message->sensor[0];

	mapper_update_grid_map(Xt_l1, range, &ultrasonic_sensor_params);

	publish_map(message->timestamp);
}


static void
rddf_annotation_message_handler(carmen_rddf_annotation_message *message)
{
	last_rddf_annotation_message = *message;
}


static void
carmen_mapper_virtual_laser_message_handler(carmen_mapper_virtual_laser_message *message)
{
	virtual_laser_message = *message;
}


void
carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message *moving_objects_point_clouds_message)
{
	moving_objects_message = *moving_objects_point_clouds_message;
}


void
save_image_semantic_map_to_disk(unsigned char *segmap, int width, int height, int camera, char camera_side, double timestamp, char *dirname)
{
	char path[2000];
	strcpy(path, dirname);
	if (path[strlen(path) - 1] != '/')
		strcat(path, "/");

	struct stat st;
	if (stat(path, &st) == -1)
		mkdir(path, 0777);

	int subdir1 = (int) timestamp / 10000 * 10000;
	sprintf(path, "%s%d/", path, subdir1);
	if (stat(path, &st) == -1)
		mkdir(path, 0777);

	int subdir2 = (int) timestamp / 100 * 100;
	sprintf(path, "%s%d/", path, subdir2);
	if (stat(path, &st) == -1)
		mkdir(path, 0777);

	sprintf(path, "%s%.6lf.bb%d-%c.segmap", path, timestamp, camera, camera_side);
	int segmap_size = width * height;
	carmen_FILE *segmap_file = carmen_fopen(path, "wb");
	carmen_fwrite(segmap, sizeof(unsigned char), segmap_size, segmap_file);
	carmen_fclose(segmap_file);
}


unsigned char *
open_semantic_image(int width, int height, int camera, char camera_side, double timestamp, char *dirname)
{
	(void) camera;
	char complete_path[1024];
	sprintf(complete_path, "%s/%.6lf-%c.segmap", dirname, timestamp, camera_side);

	carmen_FILE *segmap_file = carmen_fopen(complete_path, "rb");

	if (!segmap_file)
	{
		fprintf (stderr, "Error: Segmap file not found! %s\n", complete_path);
		return NULL;
	}

	int segmap_size = width * height;
	unsigned char *segmap = (unsigned char *) malloc (sizeof(unsigned char) * segmap_size);

	if (!segmap)
	{
		fprintf (stderr, "Error: Could not allocate memory for segmap file! %s\n", complete_path);
		return NULL;
	}

	int bytes_read = carmen_fread(segmap, sizeof(unsigned char), segmap_size, segmap_file);

	if (bytes_read != segmap_size)
		fprintf(stderr, "Error: Segmap file size must be %d bytes, but %d bytes were read from disk! %s\n", segmap_size, bytes_read, complete_path);

	carmen_fclose(segmap_file);

	return segmap;
}


void
bumblebee_basic_image_handler(int camera, carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	camera_msg_count[camera]++;
	int i = (camera_data[camera].current_index + 1) % NUM_CAMERA_IMAGES;
	char camera_side = (camera_alive[camera] == 0) ? 'l' : 'r';  // left or right side

	camera_data[camera].current_index = i;
	camera_data[camera].width[i] = image_msg->width;
	camera_data[camera].height[i] = image_msg->height;
	camera_data[camera].image_size[i] = image_msg->image_size;
	camera_data[camera].isRectified[i] = image_msg->isRectified;
	camera_data[camera].timestamp[i] = image_msg->timestamp;

	if (camera_data[camera].image[i] != NULL)
		free(camera_data[camera].image[i]);

	camera_data[camera].image[i] = (unsigned char *) malloc(sizeof(unsigned char) * image_msg->image_size);
	memcpy(camera_data[camera].image[i], ((camera_alive[camera] == 0) ? image_msg->raw_left : image_msg->raw_right), image_msg->image_size);

	if (camera_data[camera].semantic[i] != NULL)
		free(camera_data[camera].semantic[i]);

	if (process_semantic)
		camera_data[camera].semantic[i] = process_image(image_msg->width, image_msg->height, camera_data[camera].image[i]);
	else
		camera_data[camera].semantic[i] = open_semantic_image(image_msg->width, image_msg->height, camera, camera_side, image_msg->timestamp, segmap_dirname);

	if (verbose >= 2)
	{
	    cv::Mat image_cv = cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3, camera_data[camera].image[i]);
		cv::Mat semantic_cv = cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3, cv::Scalar(0));
		if (camera_data[camera].semantic[i] != NULL)
			for (int y = 0; y < semantic_cv.rows; y++)
				for (int x = 0; x < semantic_cv.cols; x++)
					semantic_cv.at<cv::Vec3b>(cv::Point(x, y)) = colormap[camera_data[camera].semantic[i][x + y * semantic_cv.cols]];

//		cv::Mat semantic_cv = cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC1, camera_data[camera].semantic[i]);
//		cv::Mat semantic_cv = cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3, color_image(image_msg->width, image_msg->height, camera_data[camera].semantic[i]));

	    int window_widht = BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH;
	    int window_height = (double) image_msg->height * window_widht / image_msg->width;
	    if (window_widht != image_msg->width)
	    {
	    	resize(image_cv, image_cv, cv::Size(window_widht, window_height));
	    	resize(semantic_cv, semantic_cv, cv::Size(window_widht, window_height));
	    }
	    hconcat(image_cv, semantic_cv, camera_image_semantic[camera]);
		cvtColor(camera_image_semantic[camera], camera_image_semantic[camera], CV_RGB2BGR);
//		imshow("Image Semantic Segmentation", camera_image_semantic[camera]);
//		cv::waitKey(1);
	}
}


void
bumblebee_basic1_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(1, image_msg);
}


void
bumblebee_basic2_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(2, image_msg);
}


void
bumblebee_basic3_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(3, image_msg);
}


void
bumblebee_basic4_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(4, image_msg);
}


void
bumblebee_basic5_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(5, image_msg);
}


void
bumblebee_basic6_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(6, image_msg);
}


void
bumblebee_basic7_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(7, image_msg);
}


void
bumblebee_basic8_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(8, image_msg);
}


void
bumblebee_basic9_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(9, image_msg);
}


void (*image_handler[]) (carmen_bumblebee_basic_stereoimage_message *) =
{
		NULL,
		bumblebee_basic1_image_handler,
		bumblebee_basic2_image_handler,
		bumblebee_basic3_image_handler,
		bumblebee_basic4_image_handler,
		bumblebee_basic5_image_handler,
		bumblebee_basic6_image_handler,
		bumblebee_basic7_image_handler,
		bumblebee_basic8_image_handler,
		bumblebee_basic9_image_handler,
};


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (update_and_merge_with_mapper_saved_maps)
			mapper_save_current_map();

		if (sensors_params[0].save_calibration_file)
			fclose(sensors_params[0].save_calibration_file);

		carmen_ipc_disconnect();
		fprintf(stderr, "Shutdown mapper_main\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intensity, carmen_pose_3D_t **robot_pose_out,
		carmen_vector_3D_t **robot_velocity_out, double **robot_timestamp_out, double **robot_phi_out, double **points_timestamp_out)
{
	int i;

	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *)malloc(NUM_VELODYNE_POINT_CLOUDS * sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*intensity = (unsigned char **)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char *));
	*robot_phi_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*points_timestamp_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));

	carmen_test_alloc(velodyne_points);

	for (i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		velodyne_points[i].num_points = 0;
		velodyne_points[i].sphere_points = NULL;
	}

	*velodyne_points_out = velodyne_points;
	*robot_pose_out = robot_pose;
	*robot_velocity_out = robot_velocity;
	*robot_timestamp_out = robot_timestamp;
}


void
sensors_params_handler(char *module, char *variable, __attribute__((unused)) char *value)
{
	if (strcmp(module, "mapper") == 0)
	{
		int i = number_of_sensors;

		if (strcmp(variable, "unsafe_height_above_ground") == 0)
		{
			for (i = 1; i < number_of_sensors; i++)
			{
				sensors_params[i].unsafe_height_above_ground = sensors_params[0].unsafe_height_above_ground;
			}
			return;
		}

		if (strcmp(variable, "velodyne") == 0)
			i = 0;
		else if (strcmp(variable, "laser_ldmrs") == 0)
			i = 1;
		else if (strncmp(variable, "stereo_velodyne", 15) == 0 && strlen(variable) == 16)
			i = variable[15] - '0';

		if (i < number_of_sensors && sensors_params[i].alive && sensors_params[i].name == NULL)
		{
			sensors_params[i].name = (char *) calloc(strlen(variable) + 1, sizeof(char));
			strcpy(sensors_params[i].name, variable);
			return;
		}
	}
}


static void
get_alive_sensors(int argc, char **argv)
{
	int i;

	sensors_params = (sensor_parameters_t *) calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(sensors_params);

	sensors_data = (sensor_data_t *) calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(sensors_data);

	carmen_param_t param_list[] =
	{
		{(char *) "mapper", (char *) "velodyne", CARMEN_PARAM_ONOFF, &sensors_params[0].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "laser_ldmrs", CARMEN_PARAM_ONOFF, &sensors_params[1].alive, 1, sensors_params_handler},
//			{(char *) "mapper", (char *) "stereo_velodyne1", CARMEN_PARAM_ONOFF, &sensors_params[1].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne2", CARMEN_PARAM_ONOFF, &sensors_params[2].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne3", CARMEN_PARAM_ONOFF, &sensors_params[3].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne4", CARMEN_PARAM_ONOFF, &sensors_params[4].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne5", CARMEN_PARAM_ONOFF, &sensors_params[5].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne6", CARMEN_PARAM_ONOFF, &sensors_params[6].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne7", CARMEN_PARAM_ONOFF, &sensors_params[7].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne8", CARMEN_PARAM_ONOFF, &sensors_params[8].alive, 1, sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne9", CARMEN_PARAM_ONOFF, &sensors_params[9].alive, 1, sensors_params_handler},
//			{(char *) "mapper", (char *) "stereo_mapping", CARMEN_PARAM_ONOFF, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].alive, 1, sensors_params_handler},

		{(char *) "mapper", (char *) "velodyne_locc", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "laser_ldmrs_locc", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_occ, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_velodyne1_locc", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne2_locc", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne3_locc", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne4_locc", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne5_locc", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne6_locc", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne7_locc", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne8_locc", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne9_locc", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_occ, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_mapping_locc", CARMEN_PARAM_DOUBLE, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].log_odds.log_odds_occ, 1, NULL},

		{(char *) "mapper", (char *) "velodyne_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "laser_ldmrs_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_free, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_velodyne1_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne2_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne3_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne4_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne5_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne6_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne7_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne8_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne9_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_free, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_mapping_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].log_odds.log_odds_free, 1, NULL},

		{(char *) "mapper", (char *) "velodyne_l0", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "laser_ldmrs_l0", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_l0, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_velodyne1_l0", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne2_l0", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne3_l0", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne4_l0", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne5_l0", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne6_l0", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne7_l0", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne8_l0", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne9_l0", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_l0, 1, NULL},

		{(char *) "mapper", (char *) "velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[0].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "laser_ldmrs_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[1].unexpeted_delta_range_sigma, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_velodyne1_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[1].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne2_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[2].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne3_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[3].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne4_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[4].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne5_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[5].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne6_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[6].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne7_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[7].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne8_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[8].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne9_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[9].unexpeted_delta_range_sigma, 1, NULL},

		{(char *) "mapper", (char *) "unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &sensors_params[0].unsafe_height_above_ground, 1, sensors_params_handler},

		{(char *) "mapper",  (char *) "velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &sensors_params[0].range_max_factor, 1, NULL}
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	sensor_msg_count = (long *) calloc(number_of_sensors, sizeof(long));
	map_update_count = (long *) calloc(number_of_sensors, sizeof(long));

	for (i = 0; i < number_of_sensors; i++)
	{
		sensors_params[i].unsafe_height_above_ground = sensors_params[0].unsafe_height_above_ground;

		sensors_data[i].ray_position_in_the_floor = (carmen_vector_2D_t **)  calloc(number_of_threads, sizeof(carmen_vector_2D_t*));
		sensors_data[i].maxed = (int **) calloc(number_of_threads, sizeof(int*));
		sensors_data[i].obstacle_height = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].occupancy_log_odds_of_each_ray_target = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].point_cloud_index = 0;
		sensors_data[i].points = NULL;
		sensors_data[i].ray_origin_in_the_floor = (carmen_vector_2D_t **) calloc(number_of_threads, sizeof(carmen_vector_2D_t*));
		sensors_data[i].ray_size_in_the_floor = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].processed_intensity = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].ray_hit_the_robot = (int **) calloc(number_of_threads, sizeof(int*));
		sensors_data[i].ray_that_hit_the_nearest_target = (int *) calloc(number_of_threads, sizeof(int));

		sensors_params[i].name = NULL;
		sensors_params[i].ray_order = NULL;
		sensors_params[i].sensor_to_support_matrix = NULL;
		sensors_params[i].vertical_correction = NULL;
		sensors_params[i].vertical_resolution = 0;

		for (int j = 0; j < number_of_threads; j++)
		{
			sensors_data[i].ray_position_in_the_floor[j] = NULL;
			sensors_data[i].maxed[j] = NULL;
			sensors_data[i].obstacle_height[j] = NULL;
			sensors_data[i].occupancy_log_odds_of_each_ray_target[j] = NULL;
			sensors_data[i].ray_origin_in_the_floor[j] = NULL;
			sensors_data[i].ray_size_in_the_floor[j] = NULL;
			sensors_data[i].processed_intensity[j] = NULL;
			sensors_data[i].ray_hit_the_robot[j] = NULL;
		}

		if (sensors_params[i].alive)
		{
			sensors_params[i].name = (char *) calloc(strlen(param_list[i].variable) + 1, sizeof(char));
			strcpy(sensors_params[i].name, param_list[i].variable);
		}

		sensor_msg_count[i] = 0;
		map_update_count[i] = 0;
	}
}


static int *
generates_ray_order(int size)
{
	int i;

	int *ray_order = (int *) malloc(size * sizeof(int));
	carmen_test_alloc(ray_order);

	for (i = 0; i < size; i++)
		ray_order[i] = i;

	return ray_order;
}


static void
get_sensors_param(int argc, char **argv)
{
	int i, j;
	int flipped;
	int horizontal_resolution;
	char stereo_velodyne_string[256];

	int stereo_velodyne_vertical_roi_ini;
	int stereo_velodyne_vertical_roi_end;

	int stereo_velodyne_horizontal_roi_ini;
	int stereo_velodyne_horizontal_roi_end;

	int roi_ini, roi_end;

	// Velodyne

	if (calibration_file)
		sensors_params[0].calibration_table = load_calibration_table(calibration_file);
	else
		sensors_params[0].calibration_table = load_calibration_table((char *) "calibration_table.txt");

	if (save_calibration_file)
		sensors_params[0].save_calibration_file = fopen(save_calibration_file, "w"); // Eh fechado em shutdown_module()
	else
		sensors_params[0].save_calibration_file = NULL;

	sensors_params[0].pose = velodyne_pose;
	sensors_params[0].sensor_support_pose = sensor_board_1_pose;
	sensors_params[0].support_to_car_matrix = create_rotation_matrix(sensors_params[0].sensor_support_pose.orientation);
	sensors_params[0].sensor_robot_reference = carmen_change_sensor_reference(
			sensors_params[0].sensor_support_pose.position, sensors_params[0].pose.position, sensors_params[0].support_to_car_matrix);
	sensors_params[0].height = sensors_params[0].sensor_robot_reference.z + robot_wheel_radius;

	if (sensors_params[0].height > highest_sensor)
		highest_sensor = sensors_params[0].height;

	if (sensors_params[0].alive && !strcmp(sensors_params[0].name, "velodyne"))
	{
		sensors_params[0].sensor_type = VELODYNE;
		sensors_params[0].ray_order = carmen_velodyne_get_ray_order();
		sensors_params[0].vertical_correction = carmen_velodyne_get_vertical_correction();
		sensors_params[0].delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
		sensors_params[0].delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

		carmen_param_t param_list[] =
		{
			{sensors_params[0].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[0].vertical_resolution, 0, NULL},
			{(char *) "mapper", (char *) "velodyne_range_max", CARMEN_PARAM_DOUBLE, &sensors_params[0].range_max, 0, NULL},
			{sensors_params[0].name, (char *) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &sensors_params[0].time_spent_by_each_scan, 0, NULL},
		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&sensors_data[0].points, &sensors_data[0].intensity, &sensors_data[0].robot_pose, &sensors_data[0].robot_velocity, &sensors_data[0].robot_timestamp, &sensors_data[0].robot_phi, &sensors_data[0].points_timestamp);
		sensors_params[0].sensor_to_support_matrix = create_rotation_matrix(sensors_params[0].pose.orientation);
		sensors_params[0].current_range_max = sensors_params[0].range_max;
		sensors_data[0].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&sensors_data[0], sensors_params[0].vertical_resolution, number_of_threads);

		sensors_params[0].remission_calibration = NULL;//(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));
//		FILE *f = fopen("../data/remission_calibration.txt", "r");
//		for (i = 0; i < 256 * sensors_params[0].vertical_resolution; i++)
//		{
//			fscanf(f, "%lf", &sensors_params[0].remission_calibration[i]);
//		}
//		fclose(f);
	}

	// LDMRS

	sensors_params[1].calibration_table = NULL;
	sensors_params[1].save_calibration_file = NULL;

	if (sensors_params[1].alive && !strcmp(sensors_params[1].name, "laser_ldmrs"))
	{
		sensors_params[1].sensor_type = LASER_LDMRS;
		sensors_params[1].pose = laser_ldmrs_pose;
		sensors_params[1].sensor_support_pose = front_bullbar_pose;
		sensors_params[1].support_to_car_matrix = create_rotation_matrix(sensors_params[1].sensor_support_pose.orientation);
		sensors_params[1].sensor_robot_reference = carmen_change_sensor_reference(
				sensors_params[1].sensor_support_pose.position, sensors_params[1].pose.position, sensors_params[1].support_to_car_matrix);
		sensors_params[1].height = sensors_params[1].sensor_robot_reference.z + robot_wheel_radius;

		if (sensors_params[1].height > highest_sensor)
			highest_sensor = sensors_params[1].height;

		static int ray_order[4] = {0, 1, 2, 3};
		sensors_params[1].ray_order = ray_order;

		static double vertical_correction[4] = {-1.2, -0.4, 0.4, 1.2};
		sensors_params[1].vertical_correction = vertical_correction;

		static double delta_difference_mean[4] = {0, 0, 0, 0};
		sensors_params[1].delta_difference_mean = delta_difference_mean;

		static double delta_difference_stddev[4] = {0, 0, 0, 0};
		sensors_params[1].delta_difference_stddev = delta_difference_stddev;

		carmen_param_t param_list[] =
		{
			{(char *) "laser_ldmrs", (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[1].vertical_resolution, 0, NULL},
			{(char *) "laser_ldmrs", (char *) "range_max", CARMEN_PARAM_DOUBLE, &sensors_params[1].range_max, 0, NULL},
			{(char *) "laser_ldmrs", (char *) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &sensors_params[1].time_spent_by_each_scan, 0, NULL},
			{(char *) "laser_ldmrs", (char *) "cutoff_negative_acceleration", CARMEN_PARAM_DOUBLE, &sensors_params[1].cutoff_negative_acceleration, 0, NULL},
		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&sensors_data[1].points, &sensors_data[1].intensity, &sensors_data[1].robot_pose, &sensors_data[1].robot_velocity, &sensors_data[1].robot_timestamp, &sensors_data[1].robot_phi, &sensors_data[1].points_timestamp);
		sensors_params[1].sensor_to_support_matrix = create_rotation_matrix(sensors_params[1].pose.orientation);
		sensors_params[1].current_range_max = sensors_params[1].range_max;
		sensors_data[1].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&sensors_data[1], sensors_params[1].vertical_resolution, number_of_threads);

		sensors_params[1].remission_calibration = NULL;//(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));
	}

	for (i = 2; i < number_of_sensors; i++)
	{
		sensors_params[i].calibration_table = NULL;
		sensors_params[i].save_calibration_file = NULL;

		if (sensors_params[i].alive)
		{
			sensors_params[i].sensor_type = CAMERA;
			sensors_params[i].pose = get_stereo_velodyne_pose_3D(argc, argv, i);
			sensors_params[i].sensor_support_pose = sensor_board_1_pose;
			sensors_params[i].support_to_car_matrix = create_rotation_matrix(sensors_params[i].sensor_support_pose.orientation);
			sensors_params[i].sensor_robot_reference = carmen_change_sensor_reference(
					sensors_params[i].sensor_support_pose.position, sensors_params[i].pose.position, sensors_params[i].support_to_car_matrix);
			sensors_params[i].height = sensors_params[i].sensor_robot_reference.z + robot_wheel_radius;

			if (sensors_params[i].height > highest_sensor)
				highest_sensor = sensors_params[i].height;

			sensors_params[i].time_spent_by_each_scan = 0.0;

			sprintf(stereo_velodyne_string, "%s%d", "stereo", i);


			carmen_param_t param_list[] =
			{
				{sensors_params[i].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[i].vertical_resolution, 0, NULL},
				{sensors_params[i].name, (char *) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 0, NULL},
				{sensors_params[i].name, (char *) "flipped", CARMEN_PARAM_ONOFF, &flipped, 0, NULL},
				{sensors_params[i].name, (char *) "range_max", CARMEN_PARAM_DOUBLE, &sensors_params[i].range_max, 0, NULL},
				{sensors_params[i].name, (char *) "vertical_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_ini, 0, NULL },
				{sensors_params[i].name, (char *) "vertical_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_end, 0, NULL },
				{sensors_params[i].name, (char *) "horizontal_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_ini, 0, NULL },
				{sensors_params[i].name, (char *) "horizontal_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_end, 0, NULL }
			};

			carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

			if (flipped)
			{
				sensors_params[i].vertical_resolution = horizontal_resolution;
				roi_ini = stereo_velodyne_horizontal_roi_ini;
				roi_end = stereo_velodyne_horizontal_roi_end;
			}
			else
			{
				roi_ini = stereo_velodyne_vertical_roi_ini;
				roi_end = stereo_velodyne_vertical_roi_end;
			}

			if (sensors_params[i].vertical_resolution > (roi_end - roi_ini))
			{
				carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");
			}
			sensors_params[i].range_max_factor = 1.0;
			sensors_params[i].ray_order = generates_ray_order(sensors_params[i].vertical_resolution);
			sensors_params[i].vertical_correction = get_stereo_velodyne_correction(flipped, i, sensors_params[i].vertical_resolution, roi_ini, roi_end, 0, 0);
			init_velodyne_points(&sensors_data[i].points, &sensors_data[i].intensity, &sensors_data[i].robot_pose, &sensors_data[i].robot_velocity,  &sensors_data[i].robot_timestamp, &sensors_data[i].robot_phi, &sensors_data[i].points_timestamp);
			sensors_params[i].sensor_to_support_matrix = create_rotation_matrix(sensors_params[i].pose.orientation);
			sensors_params[i].current_range_max = sensors_params[i].range_max;
			sensors_data[i].point_cloud_index = 0;
			carmen_prob_models_alloc_sensor_data(&sensors_data[i], sensors_params[i].vertical_resolution, number_of_threads);

			//TODO : tem que fazer esta medida para as cameras igual foi feito para o velodyne
			sensors_params[i].delta_difference_mean = (double *)calloc(50, sizeof(double));
			sensors_params[i].delta_difference_stddev = (double *)calloc(50, sizeof(double));
			for (j = 0; j < 50; j++)
				sensors_params[i].delta_difference_stddev[j] = 1.0;
		}
	}
}


static const char *
usage()
{
	const char *msg = (const char *)
		"\nUsage: %s -map_path <path> [args]\n"
		" args:\n"
		"    -camera<n> left|right                  : active cameras for datmo\n"
		"    -calibration_file <file>               : calibration file for loading\n"
		"    -save_calibration_file <file>          : calibration file for saving\n"
		"    -generate_neural_mapper_dataset on|off : neural mapper dataset option\n"
		"    -neural_mapper_max_distance_meters <n> : neural mapper maximum distance in meters\n"
		"    -neural_mapper_data_pace <n>           : neural mapper data pace\n"
		"    -process_semantic on|off               : process semantic segmentation using Deeplab\n"
		"    -file_warnings on|off                  : make or suppress file warnings\n"
		"    -verbose <n>                           : verbose option\n"
		"\n";

	return msg;
}


static void
init_camera_data(int **width, int **height, int **image_size, int **isRectified, unsigned char ***image, unsigned char ***semantic, double **timestamp)
{
	*width = (int *) calloc(NUM_CAMERA_IMAGES, sizeof(int));
	*height = (int *) calloc(NUM_CAMERA_IMAGES, sizeof(int));
	*image_size = (int *) calloc(NUM_CAMERA_IMAGES, sizeof(int));
	*isRectified = (int *) calloc(NUM_CAMERA_IMAGES, sizeof(int));
	*image = (unsigned char **) calloc(NUM_CAMERA_IMAGES, sizeof(unsigned char *));
	*semantic = (unsigned char **) calloc(NUM_CAMERA_IMAGES, sizeof(unsigned char *));
	*timestamp = (double *) calloc(NUM_CAMERA_IMAGES, sizeof(double));

	for (int i = 0; i < NUM_CAMERA_IMAGES; i++)
	{
		(*image)[i] = NULL;
		(*semantic)[i] = NULL;
	}
}


static void
get_camera_param(int argc, char **argv, int camera)
{
	char bumblebee_name[256];
	char camera_name[256];

	if (camera_alive[camera] >= 0)
	{
		sprintf(bumblebee_name, "bumblebee_basic%d", camera);
		sprintf(camera_name, "camera%d", camera);

		carmen_param_t param_list[] =
		{
			{bumblebee_name, (char*) "fx",         CARMEN_PARAM_DOUBLE, &camera_params[camera].fx_factor,       0, NULL },
			{bumblebee_name, (char*) "fy",         CARMEN_PARAM_DOUBLE, &camera_params[camera].fy_factor,       0, NULL },
			{bumblebee_name, (char*) "cu",         CARMEN_PARAM_DOUBLE, &camera_params[camera].cu_factor,       0, NULL },
			{bumblebee_name, (char*) "cv",         CARMEN_PARAM_DOUBLE, &camera_params[camera].cv_factor,       0, NULL },
			{bumblebee_name, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_params[camera].pixel_size,      0, NULL },
			{bumblebee_name, (char*) "fov",        CARMEN_PARAM_DOUBLE, &camera_params[camera].fov,             0, NULL },

			{camera_name,    (char*) "x",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.x,        0, NULL },
			{camera_name,    (char*) "y",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.y,        0, NULL },
			{camera_name,    (char*) "z",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.z,        0, NULL },
			{camera_name,    (char*) "roll",       CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.roll,  0, NULL },
			{camera_name,    (char*) "pitch",      CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.pitch, 0, NULL },
			{camera_name,    (char*) "yaw",        CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.yaw,   0, NULL },
		};

		carmen_param_allow_unfound_variables(0);
		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

		camera_data[camera].current_index = -1;
		init_camera_data(&camera_data[camera].width, &camera_data[camera].height, &camera_data[camera].image_size, &camera_data[camera].isRectified,
				&camera_data[camera].image, &camera_data[camera].semantic, &camera_data[camera].timestamp);

		camera_msg_count[camera] = 0;
		camera_filter_count[camera] = 0;
		camera_datmo_count[camera] = 0;
	}
}


static void
read_camera_parameters(int argc, char **argv)
{
	char *camera_side[MAX_CAMERA_INDEX + 1] = {NULL};

	carmen_param_t camera_param_list[] =
	{
		{(char *) "commandline", (char *) "camera1", CARMEN_PARAM_STRING, &camera_side[1], 0, NULL},
		{(char *) "commandline", (char *) "camera2", CARMEN_PARAM_STRING, &camera_side[2], 0, NULL},
		{(char *) "commandline", (char *) "camera3", CARMEN_PARAM_STRING, &camera_side[3], 0, NULL},
		{(char *) "commandline", (char *) "camera4", CARMEN_PARAM_STRING, &camera_side[4], 0, NULL},
		{(char *) "commandline", (char *) "camera5", CARMEN_PARAM_STRING, &camera_side[5], 0, NULL},
		{(char *) "commandline", (char *) "camera6", CARMEN_PARAM_STRING, &camera_side[6], 0, NULL},
		{(char *) "commandline", (char *) "camera7", CARMEN_PARAM_STRING, &camera_side[7], 0, NULL},
		{(char *) "commandline", (char *) "camera8", CARMEN_PARAM_STRING, &camera_side[8], 0, NULL},
		{(char *) "commandline", (char *) "camera9", CARMEN_PARAM_STRING, &camera_side[9], 0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, camera_param_list, sizeof(camera_param_list) / sizeof(camera_param_list[0]));

	active_cameras = 0;
	for (int i = 1; i <= MAX_CAMERA_INDEX; i++)
	{
		camera_alive[i] = -1;
		if (camera_side[i] == NULL)
			continue;

		if (strcmp(camera_side[i], "left") == 0 || strcmp(camera_side[i], "0") == 0)
			camera_alive[i] = 0;
		else if (strcmp(camera_side[i], "right") == 0 || strcmp(camera_side[i], "1") == 0)
			camera_alive[i] = 1;
		else
			carmen_die("-camera%d %s: Wrong camera side option. Must be either left or right\n", i, camera_side[i]);

		active_cameras++;
		get_camera_param(argc, argv, i);
	}
	if (active_cameras == 0)
		fprintf(stderr, "No cameras active for datmo\n\n");
	else
	if (process_semantic)
		initialize_inference_context();
}


/* read all parameters from .ini file and command line. */
static void
read_parameters(int argc, char **argv,
		carmen_map_config_t *map_config,
		carmen_robot_ackerman_config_t *p_car_config)
{
	if (argc > 1 && strcmp(argv[1], "-h") == 0)
		carmen_die(usage(), argv[0]);

	read_camera_parameters(argc, argv);

	double robot_vertical_displacement_from_center;
	double map_resolution, map_width, map_height;

	carmen_param_t param_list[] =
	{
		{(char *) "robot",  (char *) "distance_between_front_car_and_front_wheels", 	CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_car_and_front_wheels), 1, NULL},
		{(char *) "robot",  (char *) "distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_and_rear_axles), 1, NULL},
		{(char *) "robot",  (char *) "distance_between_rear_car_and_rear_wheels",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_car_and_rear_wheels), 1, NULL},
		{(char *) "robot",  (char *) "distance_between_rear_wheels",			CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_wheels), 1, NULL},
		{(char *) "robot",  (char *) "length", CARMEN_PARAM_DOUBLE, &p_car_config->length, 0, NULL},
		{(char *) "robot",  (char *) "width", CARMEN_PARAM_DOUBLE, &p_car_config->width, 0, NULL},
		{(char *) "robot",  (char *) "vertical_displacement_from_center", CARMEN_PARAM_DOUBLE, &robot_vertical_displacement_from_center, 0, NULL},
		{(char *) "robot",  (char *) "wheel_radius", CARMEN_PARAM_DOUBLE, &(robot_wheel_radius), 0, NULL},

		{(char *) "sensor_board_1",  (char *) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
		{(char *) "sensor_board_1",  (char *) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
		{(char *) "sensor_board_1",  (char *) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
		{(char *) "sensor_board_1",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
		{(char *) "sensor_board_1",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
		{(char *) "sensor_board_1",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},

		{(char *) "front_bullbar",  (char *) "x", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.x),	0, NULL},
		{(char *) "front_bullbar",  (char *) "y", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.y),	0, NULL},
		{(char *) "front_bullbar",  (char *) "z", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.z),	0, NULL},
		{(char *) "front_bullbar",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.roll),0, NULL},
		{(char *) "front_bullbar",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.pitch),0, NULL},
		{(char *) "front_bullbar",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.yaw),	0, NULL},

		{(char *) "velodyne",  (char *) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{(char *) "velodyne",  (char *) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{(char *) "velodyne",  (char *) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{(char *) "velodyne",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{(char *) "velodyne",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{(char *) "velodyne",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

		{(char *) "laser_ldmrs",  (char *) "x", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.x), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "y", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.y), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "z", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.z), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.roll), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.pitch), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.yaw), 0, NULL},

		{(char *) "mapper",  (char *) "number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL},
		{(char *) "mapper",  (char *) "safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},

		{(char *) "mapper",  (char *) "map_grid_res", CARMEN_PARAM_DOUBLE, &map_resolution, 0, NULL},
		{(char *) "mapper",  (char *) "map_width", CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
		{(char *) "mapper",  (char *) "map_height", CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},

		{(char *) "mapper",  (char *) "moving_objects_raw_map", CARMEN_PARAM_ONOFF, &publish_moving_objects_raw_map, 0, NULL},

		{(char *) "mapper",  (char *) "build_snapshot_map", CARMEN_PARAM_ONOFF, &build_snapshot_map, 0, NULL},
		{(char *) "mapper",  (char *) "merge_with_offline_map", CARMEN_PARAM_ONOFF, &merge_with_offline_map, 0, NULL},
		{(char *) "mapper",  (char *) "update_and_merge_with_mapper_saved_maps", CARMEN_PARAM_ONOFF, &update_and_merge_with_mapper_saved_maps, 0, NULL},
		{(char *) "mapper",  (char *) "update_cells_below_car", CARMEN_PARAM_ONOFF, &update_cells_below_car, 0, NULL},
		{(char *) "mapper",  (char *) "decay_to_offline_map", CARMEN_PARAM_ONOFF, &decay_to_offline_map, 0, NULL},
		{(char *) "mapper",  (char *) "create_map_sum_and_count", CARMEN_PARAM_ONOFF, &create_map_sum_and_count, 0, NULL},
		{(char *) "mapper",  (char *) "use_remission", CARMEN_PARAM_ONOFF, &use_remission, 0, NULL},

		{(char *) "mapper",  (char *) "update_and_merge_with_snapshot_map", CARMEN_PARAM_ONOFF, &update_and_merge_with_snapshot_map, 0, NULL},
		{(char *) "mapper",  (char *) "number_of_threads", CARMEN_PARAM_INT, &number_of_threads, 0, NULL},

		{(char *) "commandline",  (char *) "map_path", CARMEN_PARAM_STRING, &map_path, 0, NULL},

		{(char *) "visual_odometry", (char *) "is_global_pos", CARMEN_PARAM_ONOFF, &visual_odometry_is_global_pos, 0, NULL},

		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_sampling_step", CARMEN_PARAM_INT, &ultrasonic_sensor_params.sampling_step, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_num_beams", CARMEN_PARAM_INT, &ultrasonic_sensor_params.laser_beams, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_fov_range", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.fov_range, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_max_range", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.range_max, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_lambda_short", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.lambda_short, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_sigma_zhit", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.sigma_zhit, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_zhit", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zhit, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_zmax", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zmax, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_zrand", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zrand, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_zshort", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zshort, 0, NULL},

		{(char *) "grid_mapping", (char *) "map_locc", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_occ, 0, NULL},
		{(char *) "grid_mapping", (char *) "map_lfree", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_free, 0, NULL},
		{(char *) "grid_mapping", (char *) "map_l0", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_l0, 0, NULL},

		{(char *) "ultrasonic_sensor_r1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.x), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.y), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.z), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.yaw), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.pitch), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.roll), 0, NULL},

		{(char *) "ultrasonic_sensor_r2", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.x), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.y), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.z), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.yaw), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.pitch), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.roll), 0, NULL},

		{(char *) "ultrasonic_sensor_l2", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.x), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.y), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.z), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.yaw), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.pitch), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.roll), 0, NULL},

		{(char *) "ultrasonic_sensor_l1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.x), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.y), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.z), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.yaw), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.pitch), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.roll), 0, NULL},

		{(char *) "behavior_selector", (char *) "use_truepos", CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL}
	};

	carmen_param_allow_unfound_variables(0);
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	ultrasonic_sensor_params.current_range_max = ultrasonic_sensor_params.range_max;

	if (map_width != map_height)
		carmen_die("Wrong map size: width (%f) must be equal to height (%f).", map_width, map_height);

	if ((((int) map_width) % 3) != 0)
		carmen_die("Wrong map size: width (%f) and height (%f) must be multiple of 3.", map_width, map_height);

	map_config->x_size = round(map_width / map_resolution);
	map_config->y_size = round(map_height / map_resolution);
	map_config->resolution = map_resolution;

	carmen_grid_mapping_init_parameters(map_resolution, map_width);

	get_alive_sensors(argc, argv);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline", (char *) "calibration_file", CARMEN_PARAM_STRING, &calibration_file, 0, NULL},
		{(char *) "commandline", (char *) "save_calibration_file", CARMEN_PARAM_STRING, &save_calibration_file, 0, NULL},
		{(char *) "commandline", (char *) "generate_neural_mapper_dataset", CARMEN_PARAM_ONOFF, &generate_neural_mapper_dataset, 0, NULL},
		{(char *) "commandline", (char *) "neural_mapper_max_distance_meters", CARMEN_PARAM_INT, &neural_mapper_max_distance_meters, 0, NULL},
		{(char *) "commandline", (char *) "neural_mapper_data_pace", CARMEN_PARAM_INT, &neural_mapper_data_pace, 0, NULL},
		{(char *) "commandline", (char *) "process_semantic", CARMEN_PARAM_ONOFF, &process_semantic, 1, NULL},
		{(char *) "commandline", (char *) "file_warnings", CARMEN_PARAM_ONOFF, &file_warnings, 1, NULL},
		{(char *) "commandline", (char *) "verbose", CARMEN_PARAM_INT, &verbose, 1, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	get_sensors_param(argc, argv);
}


static void
carmen_subscribe_to_ultrasonic_relevant_messages()
{
	carmen_parking_assistant_subscribe_goal(NULL, (carmen_handler_t) parking_sensor_goal_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ultrasonic_sonar_sensor_subscribe(NULL, (carmen_handler_t) ultrasonic_sensor_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
subscribe_to_ipc_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[0].alive)
		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[1].alive)
	{
		carmen_laser_subscribe_ldmrs_message(NULL, (carmen_handler_t) laser_ldrms_message_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_laser_subscribe_ldmrs_new_message(NULL, (carmen_handler_t) laser_ldrms_new_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[2].alive)
		carmen_stereo_velodyne_subscribe_scan_message(2, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler2, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[3].alive)
		carmen_stereo_velodyne_subscribe_scan_message(3, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler3, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[4].alive)
		carmen_stereo_velodyne_subscribe_scan_message(4, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler4, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[5].alive)
		carmen_stereo_velodyne_subscribe_scan_message(5, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler5, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[6].alive)
		carmen_stereo_velodyne_subscribe_scan_message(6, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler6, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[7].alive)
		carmen_stereo_velodyne_subscribe_scan_message(7, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler7, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[8].alive)
		carmen_stereo_velodyne_subscribe_scan_message(8, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler8, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[9].alive)
		carmen_stereo_velodyne_subscribe_scan_message(9, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler9, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);

	if (!use_truepos) // This flag is for a special kind of operation where the sensor pipeline listen to the globalpos and the planning pipeline to the truepos
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) true_pos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (visual_odometry_is_global_pos)
		carmen_subscribe_to_ultrasonic_relevant_messages();

	carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_mapper_subscribe_virtual_laser_message(NULL, (carmen_handler_t) carmen_mapper_virtual_laser_message_handler, CARMEN_SUBSCRIBE_LATEST);

	// draw moving objects
//	carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler, CARMEN_SUBSCRIBE_LATEST);

	for (int camera = 1; camera <= MAX_CAMERA_INDEX; camera++)
	{
		if (camera_alive[camera] >= 0)
			carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler[camera], CARMEN_SUBSCRIBE_LATEST);
	}
}


static void
define_mapper_messages()
{
	/* register initialize message */
	carmen_map_server_define_compact_cost_map_message();
	carmen_mapper_define_messages();
	carmen_mapper_define_virtual_scan_message();
}


void
initialize_transforms()
{
	tf::Transform board_to_camera_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;
	tf::Transform ultrasonic_sensor_r1_to_car_pose;
	tf::Transform ultrasonic_sensor_r2_to_car_pose;
	tf::Transform ultrasonic_sensor_l1_to_car_pose;
	tf::Transform ultrasonic_sensor_l2_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	tf_transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_pose_g.position.x, sensor_board_pose_g.position.y, sensor_board_pose_g.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_pose_g.orientation.yaw, sensor_board_pose_g.orientation.pitch, sensor_board_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	tf_transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	tf_transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

	// initial ultrasonic sensor r1 pose with respect to the car
	ultrasonic_sensor_r1_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_r1_g.position.x, ultrasonic_sensor_r1_g.position.y, ultrasonic_sensor_r1_g.position.z));
	ultrasonic_sensor_r1_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_r1_g.orientation.yaw, ultrasonic_sensor_r1_g.orientation.pitch, ultrasonic_sensor_r1_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_r1_to_car_transform(ultrasonic_sensor_r1_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_r1");
	tf_transformer.setTransform(ultrasonic_sensor_r1_to_car_transform, "ultrasonic_sensor_r1_to_car_transform");

	// initial ultrasonic sensor r2 pose with respect to the car
	ultrasonic_sensor_r2_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_r2_g.position.x, ultrasonic_sensor_r2_g.position.y, ultrasonic_sensor_r2_g.position.z));
	ultrasonic_sensor_r2_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_r2_g.orientation.yaw, ultrasonic_sensor_r2_g.orientation.pitch, ultrasonic_sensor_r2_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_r2_to_car_transform(ultrasonic_sensor_r2_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_r2");
	tf_transformer.setTransform(ultrasonic_sensor_r2_to_car_transform, "ultrasonic_sensor_r2_to_car_transform");

	// initial ultrasonic sensor l2 pose with respect to the car
	ultrasonic_sensor_l2_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_l2_g.position.x, ultrasonic_sensor_l2_g.position.y, ultrasonic_sensor_l2_g.position.z));
	ultrasonic_sensor_l2_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_l2_g.orientation.yaw, ultrasonic_sensor_l2_g.orientation.pitch, ultrasonic_sensor_l2_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_l2_to_car_transform(ultrasonic_sensor_l2_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_l2");
	tf_transformer.setTransform(ultrasonic_sensor_l2_to_car_transform, "ultrasonic_sensor_l2_to_car_transform");

	// initial ultrasonic sensor l1 pose with respect to the car
	ultrasonic_sensor_l1_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_l1_g.position.x, ultrasonic_sensor_l1_g.position.y, ultrasonic_sensor_l1_g.position.z));
	ultrasonic_sensor_l1_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_l1_g.orientation.yaw, ultrasonic_sensor_l1_g.orientation.pitch, ultrasonic_sensor_l1_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_l1_to_car_transform(ultrasonic_sensor_l1_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_l1");
	tf_transformer.setTransform(ultrasonic_sensor_l1_to_car_transform, "ultrasonic_sensor_l1_to_car_transform");
}
//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	carmen_map_config_t map_config;
	carmen_robot_ackerman_config_t car_config;

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv, &map_config, &car_config);

	initialize_transforms();

	mapper_initialize(&map_config, car_config);

	/* Register messages */
	define_mapper_messages();

	/* Subscribe to relevant messages */
	subscribe_to_ipc_messages();

	carmen_ipc_dispatch();

	return (0);
}
