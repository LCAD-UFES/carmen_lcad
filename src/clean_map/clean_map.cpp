#include "clean_map.h"

#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>
#include <carmen/mapper_interface.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include <vector>


#define	UPDATE_CELLS_CROSSED_BY_RAYS		1
#define	DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS	0

extern double safe_range_above_sensors;
extern double robot_wheel_radius;

extern double highest_sensor;

extern int merge_with_offline_map;
extern int update_and_merge_with_clean_map_saved_maps;
extern int build_snapshot_map;
extern int update_and_merge_with_snapshot_map;
extern int decay_to_offline_map;
extern int create_map_sum_and_count;

extern carmen_pose_3D_t sensor_board_1_pose;
extern rotation_matrix *board_to_car_matrix;

extern sensor_parameters_t *sensors_params;
extern sensor_data_t *sensors_data;
extern int number_of_sensors;

extern char *map_path;

extern int publish_moving_objects_raw_map;

extern int robot_near_strong_slow_down_annotation;
extern int ok_to_publish;
extern int number_of_threads;

extern carmen_localize_ackerman_map_t localize_map;

#define      HUGE_DISTANCE     32000

/**
 * The map
 */

carmen_map_t occupancy_map, sum_remission_map, sum_sqr_remission_map, count_remission_map, moving_objects_raw_map,
				sum_occupancy_map, mean_occupancy_map, count_occupancy_map; //, variance_occupancy_map;

carmen_map_t cost_map;

extern carmen_map_t offline_map;

extern rotation_matrix *r_matrix_car_to_global;

int globalpos_initialized = 0;
extern carmen_localize_ackerman_globalpos_message *globalpos_history;
extern int last_globalpos;

carmen_robot_ackerman_config_t car_config;
carmen_map_config_t map_config;

double x_origin, y_origin; // map origin in meters

static carmen_laser_laser_message flaser; // moving_objects


// Inicio do teste: moving_objects - Eduardo
void
publish_frontlaser(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;

	//carmen_simulator_ackerman_calc_laser_msg(&flaser, simulator_config, 0);

	flaser.timestamp = timestamp;
	err = IPC_publishData(CARMEN_LASER_FRONTLASER_NAME, &flaser);
	carmen_test_ipc(err, "Could not publish laser_frontlaser_message",
			CARMEN_LASER_FRONTLASER_NAME);
}

void
build_front_laser_message_from_velodyne_point_cloud(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, spherical_point_cloud v_zt, int i)
{
	static int first = 1;

	if (first)
	{
		flaser.host = carmen_get_host();
		flaser.num_readings = 720;
		flaser.range = (double *)calloc
				(720, sizeof(double));
		carmen_test_alloc(flaser.range);

		flaser.num_remissions = 0;
		flaser.remission = 0;

		flaser.config.angular_resolution = 0.5;
		flaser.config.fov = sensor_params->fov_range;
		flaser.config.maximum_range = sensor_params->range_max;
		flaser.config.remission_mode = REMISSION_NONE;
		flaser.config.start_angle = sensor_params->start_angle;
		first = 0;
	}

	int laser_ray_angle_index = 0;
	laser_ray_angle_index = (int)(v_zt.sphere_points[i].horizontal_angle / 0.5) % (720);

	//if (carmen_prob_models_log_odds_to_probabilistic(sensor_data->occupancy_log_odds_of_each_ray_target[sensor_data->ray_that_hit_the_nearest_target]) > 0.95)
	flaser.range[laser_ray_angle_index] = sensor_data->ray_size_in_the_floor[0][sensor_data->ray_that_hit_the_nearest_target[0]];

	if (sensor_data->maxed[0][sensor_data->ray_that_hit_the_nearest_target[0]])
			flaser.range[laser_ray_angle_index] = sensor_params->current_range_max;

//	printf("%lf ", flaser.range[laser_ray_angle_index]);

}

//teste: fim

int
get_nearest_timestamp_index(double *robot_timestamp, spherical_point_cloud *points, int cindex)
{
	int index_nearest_timestamp = 0;
	double timestamp_diff = 10.0;
	int j = cindex;

	for (int i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		double diff = fabs(robot_timestamp[j] - points[i].timestamp);
		//printf("diff = %lf, rt = %lf, vt = %lf\n", robot_timestamp[j] - points[i].timestamp, robot_timestamp[j], points[i].timestamp);
		if (diff < timestamp_diff)
		{
			timestamp_diff = diff;
			index_nearest_timestamp = i;
		}
	}
//
//	if (timestamp_diff != 0.0)
//	{
//		j = ((cindex - 1) < 0)? NUM_VELODYNE_POINT_CLOUDS - 1: cindex - 1;
//		for (int i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
//		{
//			double diff = fabs(robot_timestamp[j] - points[i].timestamp);
//			//printf("diff = %lf, rt = %lf, vt = %lf\n", robot_timestamp[j] - points[i].timestamp, robot_timestamp[j], points[i].timestamp);
//			if (diff < timestamp_diff)
//			{
//				timestamp_diff = diff;
//				index_nearest_timestamp = i;
//			}
//		}
//	}

	//printf("time diff = %lf, index = %d, cindex = %d\n", timestamp_diff, index_nearest_timestamp, cindex);

	return (index_nearest_timestamp);
}

cv::Mat rotate(cv::Mat src, double angle)
{
    cv::Mat dst;
    cv::Point2f pt(src.cols/2., src.rows/2.);
    cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
    return dst;
}

void
segment_remission_map(carmen_map_t *remission_map, carmen_map_t *map)
{
	cv::Mat map_img = cv::Mat::zeros(remission_map->config.x_size, remission_map->config.y_size, CV_8UC1);
	cv::Mat occupancy_map_img = cv::Mat::zeros(remission_map->config.x_size, remission_map->config.y_size, CV_8UC1);
	cv::Mat offline_map_img = cv::Mat::zeros(remission_map->config.x_size, remission_map->config.y_size, CV_8UC3);

	int erosion_size = 1;
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE,
			cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
			cv::Point( erosion_size, erosion_size ));


	for (int i = 0; i < remission_map->config.x_size; i++)
	{
		for (int j = 0; j < remission_map->config.x_size; j++)
		{
			//if (remission_map->map[i][j] < 0.0)
				//continue;

//			uchar aux = (uchar)((255.0 * (1.0 - (remission_map->map[i][j] < 0 ? 1 : remission_map->map[i][j]))) + 0.5);
			uchar aux = (uchar) 3.5 * (255.0 * (1.0 - (remission_map->map[i][j] < 0 ? 1 : remission_map->map[i][j])) + 0.5);
			map_img.at<uchar>(i, j) = aux;

			aux = 255 * (map->map[i][j] > 0.5 ? 1.0 : 0.0);
			occupancy_map_img.at<uchar>(i, j) = aux;
		}
	}

//	cv::Mat map_img1 = cv::Mat::zeros(600, 800, CV_8UC1);
//	map_img1 = map_img;
//	cv::imshow("map_img1", map_img1);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contours2;

    //cv::medianBlur(map_img, map_img, 3);

	cv::threshold(map_img, map_img, 235/*isto pode ser calibrado*/, 255, cv::THRESH_BINARY);

//	cv::Mat map_img2 = cv::Mat::zeros(600, 800, CV_8UC1);
//	map_img2 = map_img;
//	cv::imshow("map_img2", map_img2);

	cv::erode(map_img, map_img, element);

//	cv::Mat map_img3 = cv::Mat::zeros(600, 800, CV_8UC1);
//	map_img3 = map_img;
//	cv::imshow("map_img3", map_img3);
//	cv::waitKey(0);

	findContours(map_img.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

//	double max_area = -DBL_MAX;
//	int index = 0;

	for (uint i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i])* map->config.resolution;

		if (area < 1000)
			drawContours(map_img, contours, i, CV_RGB(0, 0, 0), -1);
	}

	cv::dilate(map_img, map_img, element);
	cv::dilate(map_img, map_img, element);
	cv::dilate(map_img, map_img, element);
	cv::dilate(map_img, map_img, element);

	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);

	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);

	for (int i = 0; i < remission_map->config.x_size; i++)
	{
		for (int j = 0; j < remission_map->config.x_size; j++)
		{
			if (map->map[i][j] < 0.0)
			{
				offline_map_img.at<cv::Vec3b>(i, j) = cv::Vec3b(180, 0, 0);
				continue;
			}

			if (map_img.at<uchar>(i, j) > 128)
			{
				offline_map_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
				continue;
			}

			uchar aux = 255 * (1.0 - map->map[i][j]);
			offline_map_img.at<cv::Vec3b>(i, j) = cv::Vec3b(aux, aux, aux);
		}
	}

	cv::imshow("offline_map_img", rotate(offline_map_img, 90));
	cv::imshow("map_img", rotate(map_img, 90));
	cv::waitKey(33);
}


void
show_map(carmen_map_t *map)
{
	cv::Mat map_img = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC1);
	cv::Mat map_img_bkp = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC1);
	cv::Mat offline_map_img = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC1);
	static cv::Mat last_map_img;
	int erosion_size = 2;
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE,
	                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
	                                       cv::Point( erosion_size, erosion_size ));


	cv::Mat map_img2 = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC1);
	cv::Mat map_img_out = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC3);
	cv::Mat last_map_img_out = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC3);

	//#######################copia de mapa do carmen para imagem opencv
	for (int i = 0; i < map->config.x_size; i++)
	{
		for (int j = 0; j < map->config.x_size; j++)
		{
			uchar aux = 255 * (map->map[i][j] > 0.5 ? 1.0 : 0.0);
			map_img_bkp.at<uchar>(i, j)  = map_img.at<uchar>(i, j) = aux;

			aux = 255 * (offline_map.map[i][j] > 0.5 ? 1.0 : 0.0);
			offline_map_img.at<uchar>(i, j) = aux;
		}
	}

	cv::dilate( map_img, map_img, element);
	cv::dilate( offline_map_img, offline_map_img, element);

	//cv::dilate( map_img, map_img, element);
	map_img.copyTo(map_img2);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contours2;

	//#################remove objetos estaticos e grandes
	findContours(map_img.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
//	findContours(last_map_img.clone(), contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	for (uint i = 0; i < contours.size(); i++)
	{
		int count = 0;
		cv::Point2i minP = cv::Point(INT_MAX, INT_MAX);
		cv::Point2i maxP = cv::Point(-INT_MAX, -INT_MAX);
		for (uint j = 0; j < contours[i].size(); j++)
		{
			if (maxP.x < contours[i][j].x)
				maxP.x = contours[i][j].x;

			if (maxP.y < contours[i][j].y)
				maxP.y = contours[i][j].y;

			if (minP.x > contours[i][j].x)
				minP.x = contours[i][j].x;

			if (minP.y > contours[i][j].y)
				minP.y = contours[i][j].y;

		}
		cv::Mat roi = map_img2(cv::Rect(minP, maxP));

		for (int y = 0; y < roi.rows; y++)
			for (int x = 0; x < roi.cols; x++)
				if (offline_map_img.at<uchar>(y + minP.y, x + minP.x) > 128)
					count++;

		double area = contourArea(contours[i]) * map->config.resolution;

		if (((double) count / contours[i].size()) > 0.9 || area < 1.0 || area > 200.0)
			drawContours(map_img2, contours, i, CV_RGB(0, 0, 0), -1);
		else
		{
			std::vector<cv::Point> contours_poly;
			cv::approxPolyDP(cv::Mat(contours[i]), contours_poly, 3, true);
			cv::Rect r = cv::boundingRect(cv::Mat(contours_poly));
			cv::rectangle( offline_map_img, r.tl(), r.br(), CV_RGB(180, 180, 180), 2, 8, 0 );
			drawContours(offline_map_img, contours, i, CV_RGB(128, 128, 128), -1);
		}
	}

//	cv::dilate( map_img2, map_img2, element);

	if (last_map_img.empty())
	{
		last_map_img = map_img2;
		return;
	}
	contours.erase(contours.begin(),contours.end());

	//###################associação dos objetos
	findContours(map_img2.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
	findContours(last_map_img.clone(), contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	std::vector<cv::Moments> map_mu(contours.size());
    std::vector<cv::Moments> last_map_mu(contours2.size());

    for (uint i = 0; i < contours.size(); i++)
    	map_mu[i] = moments(contours[i], false);

    for (uint i = 0; i < contours2.size(); i++)
    	last_map_mu[i] = moments(contours2[i], false);

    std::vector<cv::Point3d> objs(map_mu.size());


    for (uint i = 0; i < map_mu.size(); i++)
    {
    	int x_center = map_mu[i].m10 / map_mu[i].m00;
    	int y_center = map_mu[i].m01 / map_mu[i].m00;
    	double min_dist = DBL_MAX;
    	double index = -1;

    	for (uint j = 0; j < last_map_mu.size(); j++)
    	{
    		int x_center2 = last_map_mu[j].m10 / last_map_mu[j].m00;
    		int y_center2 = last_map_mu[j].m01 / last_map_mu[j].m00;
    		double dist = sqrt(pow(x_center - x_center2, 2) + pow(y_center - y_center2, 2));
    		if (min_dist > dist)
    		{
    			min_dist = dist;
    			index = j;
    		}
    	}
//    	if ((min_dist * map->config.resolution) > 1.0)
    		objs[i] = cv::Point3d(min_dist, i, index);
//    	else
//    		objs[i] = cv::Point3d(min_dist, -1.0, -1.0);
    }

    for (uint i = 0; i < objs.size(); i++)
    {
    	if ((int)objs[i].z < 0.0)
    		continue;

    	for (uint j = i + 1; j < objs.size(); j++)
    		if ((int)objs[i].z == (int)objs[j].z && (objs[j].x * map->config.resolution) > 1.5)
    		{
    			if (objs[i].x < objs[j].x)
    				objs[j].z = -1.0;
    			else
    				objs[i].z = -1.0;

    		}
    }

    //##################pinta as associações
    cv::RNG rng(12345);
    for (uint i = 0; i < objs.size(); i++)
    {
    	uchar R, G, B;
    	cv::Scalar color;
    	if ((int)objs[i].z < 0.0)
    	{
    		R = 255; G = 255; B = 255;
    	}
    	else
    	{
    		R = rng.uniform(0, 255); G = rng.uniform(0, 255); B = rng.uniform(0, 255);
    	}
    	color = cv::Scalar(R, G, B);


		cv::Point2i minP = cv::Point(INT_MAX, INT_MAX);
		cv::Point2i maxP = cv::Point(-INT_MAX, -INT_MAX);
		for (uint j = 0; j < contours[i].size(); j++)
		{
			if (maxP.x < contours[i][j].x)
				maxP.x = contours[i][j].x;

			if (maxP.y < contours[i][j].y)
				maxP.y = contours[i][j].y;

			if (minP.x > contours[i][j].x)
				minP.x = contours[i][j].x;

			if (minP.y > contours[i][j].y)
				minP.y = contours[i][j].y;

		}
		cv::Mat roi = map_img_bkp(cv::Rect(minP, maxP));

		for (int y = 0; y < roi.rows; y++)
			for (int x = 0; x < roi.cols; x++)
				if (roi.at<uchar>(y, x) > 128)
					map_img_out.at<cv::Vec3b>(y + minP.y, x + minP.x) = cv::Vec3b(R, G, B);


    	//if ((int)objs[i].y > 0.0)
    		//drawContours(map_img_out, contours, (int)objs[i].y, color, -1);

    	if ((int)objs[i].z > 0.0)
    		drawContours(last_map_img_out, contours2, (int)objs[i].z, color, -1);

    }

	//cv::medianBlur(map_img2, map_img2, 3);

    cv::imshow("map_img_out", map_img_out);
    cv::imshow("last_map_img_out", last_map_img_out);


    cv::imshow("offline_map_img", offline_map_img);
	//cv::imshow("last_map_img", last_map_img);
	cv::imshow("map_img", map_img);
	//cv::imshow("map", map_img2);
	cv::waitKey(33);
	last_map_img = map_img2;
}


//static void
//update_cells_in_the_velodyne_perceptual_field(carmen_map_t *snapshot_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global,
//					      int point_cloud_index, int update_cells_crossed_by_rays, int build_snapshot_map __attribute__ ((unused)))
//{
//	int tid = omp_get_thread_num();
//	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
//	int N = v_zt.num_points / sensor_params->vertical_resolution;
//
//	double v = sensor_data->robot_velocity[point_cloud_index].x;
//	double phi = sensor_data->robot_phi[point_cloud_index];
//
//	double dt = 1.0 / (1808.0 * 12.0);
//	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
//	int i = 0;
//
//	// Ray-trace the grid
//	#pragma omp for
//	for (int j = 0; j < N; j += 1)
//	{
//		i = j * sensor_params->vertical_resolution;
//		double dt2 = j * dt;
//		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(sensor_data->robot_pose[point_cloud_index], dt2, v, phi, car_config.distance_between_front_and_rear_axles);
//		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);
//
//		change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[i].horizontal_angle);
//
//		carmen_prob_models_compute_relevant_map_coordinates(sensor_data, sensor_params, i, robot_interpolated_position.position, sensor_board_1_pose,
//				r_matrix_robot_to_global, board_to_car_matrix, robot_wheel_radius, x_origin, y_origin, &car_config, robot_near_bump_or_barrier, tid);
//
//		carmen_prob_models_get_occuppancy_log_odds_by_height(sensor_data, sensor_params, i, highest_sensor, safe_range_above_sensors,
//				robot_near_bump_or_barrier, tid);
//
//		sensor_params->log_odds.log_odds_free = -37;
//
//		carmen_prob_models_update_cells_crossed_by_ray(snapshot_map, sensor_params, sensor_data, tid);
//
//		//sensor_params->log_odds.log_odds_occ = 37;
//
//		//carmen_prob_models_update_log_odds_of_nearest_target(snapshot_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid);
//		carmen_prob_models_update_log_odds_of_cells_hit_by_rays(snapshot_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid);
//
//	}

//	segment_remission_map(&localize_map.carmen_mean_remission_map, &localize_map.carmen_map);
	//show_map(snapshot_map);
	//show_map(&offline_map);
	//printf("\n###############################################################\n");
//}


void
set_map_equal_offline_map(carmen_map_t *current_map)
{
	int xi, yi;

	for (xi = 0; xi < current_map->config.x_size; xi++)
		for (yi = 0; yi < current_map->config.y_size; yi++)
			current_map->map[xi][yi] = offline_map.map[xi][yi];
}


void
add_offline_map_over_unknown(carmen_map_t *current_map)
{
	int xi, yi;

	for (xi = 0; xi < current_map->config.x_size; xi++)
		for (yi = 0; yi < current_map->config.y_size; yi++)
			if (current_map->map[xi][yi] < 0.0)
				current_map->map[xi][yi] = offline_map.map[xi][yi];
}


void
map_decay_to_offline_map(carmen_map_t *current_map)
{
	//int xi, yi;

	#pragma omp for
	for (int xi = 0; xi < current_map->config.x_size; xi++)
	{
		for (int yi = 0; yi < current_map->config.y_size; yi++)
		{
			if (current_map->map[xi][yi] >= 0.0)
			{
				//current_map->map[xi][yi] = (50.0 * current_map->map[xi][yi] + offline_map.map[xi][yi]) / 51.0;
				current_map->map[xi][yi] = (10.0 * current_map->map[xi][yi] + offline_map.map[xi][yi]) / 11.0;
				//current_map->map[xi][yi] = carmen_prob_models_log_odds_to_probabilistic((get_log_odds(current_map->map[xi][yi]) + get_log_odds(offline_map.map[xi][yi])) / 2.0);
				//if (fabs(current_map->map[xi][yi] - 0.5) < 0.1)
				//	current_map->map[xi][yi] = -1.0;
			}
			else
				current_map->map[xi][yi] = offline_map.map[xi][yi];
		}
	}
}


void
initialize_first_map_block_origin(carmen_map_t *current_carmen_map, carmen_position_t *map_origin, char map_type)
{
	current_carmen_map->complete_map = NULL;
	x_origin = map_origin->x;
	y_origin = map_origin->y;

	if (update_and_merge_with_clean_map_saved_maps)
	{
		carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, map_type, x_origin, y_origin, current_carmen_map);

		if (current_carmen_map->complete_map == NULL)
			carmen_grid_mapping_initialize_map(current_carmen_map, occupancy_map.config.x_size, occupancy_map.config.resolution);
	}
	else		
		carmen_grid_mapping_get_buffered_map(x_origin, y_origin, current_carmen_map);
}



void
clean_map_change_map_origin_to_another_map_block(carmen_position_t *map_origin)
{
	static int first_time = 1;

	static carmen_map_t new_carmen_map, new_sum_remission_map, new_sum_sqr_remission_map, new_count_remission_map,
								new_sum_occupancy_map, new_mean_occupancy_map, new_count_occupancy_map; //new_variance_occupancy_map;

	if (first_time)
	{
		initialize_first_map_block_origin(&occupancy_map, map_origin, 'm');
		initialize_first_map_block_origin(&moving_objects_raw_map, map_origin, 'm');
		initialize_first_map_block_origin(&sum_remission_map, map_origin, 's');
		initialize_first_map_block_origin(&sum_sqr_remission_map, map_origin, '2');
		initialize_first_map_block_origin(&count_remission_map, map_origin, 'c');

		if (create_map_sum_and_count)
		{
			initialize_first_map_block_origin(&sum_occupancy_map, map_origin, 'u');
			initialize_first_map_block_origin(&mean_occupancy_map, map_origin, 'e');
			initialize_first_map_block_origin(&count_occupancy_map, map_origin, 'o');
			//		initialize_first_map_block_origin(&variance_occupancy_map, map_origin, 'v');
		}

		carmen_grid_mapping_create_new_map(&new_carmen_map, occupancy_map.config.x_size, occupancy_map.config.y_size, occupancy_map.config.resolution, 'm');
		carmen_grid_mapping_create_new_map(&new_sum_remission_map, sum_remission_map.config.x_size, sum_remission_map.config.y_size, sum_remission_map.config.resolution, 's');
		carmen_grid_mapping_create_new_map(&new_sum_sqr_remission_map, sum_sqr_remission_map.config.x_size, sum_sqr_remission_map.config.y_size, sum_sqr_remission_map.config.resolution, '2');
		carmen_grid_mapping_create_new_map(&new_count_remission_map, count_remission_map.config.x_size, count_remission_map.config.y_size, count_remission_map.config.resolution, 'c');

		if (create_map_sum_and_count)
		{
			carmen_grid_mapping_create_new_map(&new_sum_occupancy_map, sum_occupancy_map.config.x_size, sum_occupancy_map.config.y_size, sum_occupancy_map.config.resolution, 'u');
			carmen_grid_mapping_create_new_map(&new_mean_occupancy_map, mean_occupancy_map.config.x_size, mean_occupancy_map.config.y_size, mean_occupancy_map.config.resolution, 'e');
			carmen_grid_mapping_create_new_map(&new_count_occupancy_map, count_occupancy_map.config.x_size, count_occupancy_map.config.y_size, count_occupancy_map.config.resolution, 'o');
			//		carmen_grid_mapping_create_new_map(&new_variance_occupancy_map, variance_occupancy_map.config.x_size, variance_occupancy_map.config.y_size, variance_occupancy_map.config.resolution);
		}

		first_time = 0;
	}

	//verify if its necessery to change the map
	if (carmen_grid_mapping_is_map_changed(map_origin, x_origin, y_origin))
	{
		x_origin = map_origin->x;
		y_origin = map_origin->y;

		if (update_and_merge_with_clean_map_saved_maps)
		{
			//save the current map in the given map_path
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', &occupancy_map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, 's', &sum_remission_map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, '2', &sum_sqr_remission_map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', &count_remission_map);

			if (create_map_sum_and_count)
			{
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'u', &sum_occupancy_map);
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'e', &mean_occupancy_map);
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'o', &count_occupancy_map);
				//			carmen_grid_mapping_save_block_map_by_origin(map_path, 'v', &variance_occupancy_map);
			}

			//get new map with integrated information of the old map
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'm', x_origin, y_origin, &new_carmen_map);
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 's', x_origin, y_origin, &new_sum_remission_map);
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, '2', x_origin, y_origin, &new_sum_sqr_remission_map);
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'c', x_origin, y_origin, &new_count_remission_map);

			if (create_map_sum_and_count)
			{
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'u', x_origin, y_origin, &new_sum_occupancy_map);
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'e', x_origin, y_origin, &new_mean_occupancy_map);
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'o', x_origin, y_origin, &new_count_occupancy_map);
				//			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'v', x_origin, y_origin, &new_variance_occupancy_map);
			}

		}
		else
		{
			carmen_grid_mapping_update_map_buffer(&occupancy_map, 'm');
			carmen_grid_mapping_get_buffered_map(x_origin, y_origin, &new_carmen_map, 'm');
		}

		//destroy current map and assign new map to current map
		carmen_grid_mapping_swap_maps_and_clear_old_map(&occupancy_map, &new_carmen_map);

		if (update_and_merge_with_clean_map_saved_maps)
		{
			carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_remission_map, &new_sum_remission_map);
			carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_sqr_remission_map, &new_sum_sqr_remission_map);
			carmen_grid_mapping_swap_maps_and_clear_old_map(&count_remission_map, &new_count_remission_map);

			if (create_map_sum_and_count)
			{
				carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_occupancy_map, &new_sum_occupancy_map);
				carmen_grid_mapping_swap_maps_and_clear_old_map(&mean_occupancy_map, &new_mean_occupancy_map);
				carmen_grid_mapping_swap_maps_and_clear_old_map(&count_occupancy_map, &new_count_occupancy_map);
				//			carmen_grid_mapping_swap_maps_and_clear_old_map(&variance_occupancy_map, &new_variance_occupancy_map);
			}

		}
	}

	moving_objects_raw_map.config.x_origin = x_origin;
	moving_objects_raw_map.config.y_origin = y_origin;

	occupancy_map.config.x_origin = x_origin;
	occupancy_map.config.y_origin = y_origin;

	sum_remission_map.config.x_origin = x_origin;
	sum_remission_map.config.y_origin = y_origin;

	sum_sqr_remission_map.config.x_origin = x_origin;
	sum_sqr_remission_map.config.y_origin = y_origin;

	count_remission_map.config.x_origin = x_origin;
	count_remission_map.config.y_origin = y_origin;

	if (create_map_sum_and_count)
	{
		sum_occupancy_map.config.x_origin = x_origin;
		sum_occupancy_map.config.y_origin = y_origin;

		mean_occupancy_map.config.x_origin = x_origin;
		mean_occupancy_map.config.y_origin = y_origin;

		count_occupancy_map.config.x_origin = x_origin;
		count_occupancy_map.config.y_origin = y_origin;

		//	variance_occupancy_map.config.x_origin = x_origin;
		//	variance_occupancy_map.config.y_origin = y_origin;
	}

}


int
run_clean_map(/*sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global*/)
{
	//carmen_point_t world_pose;
	//carmen_position_t map_origin;

	if (!globalpos_initialized)
		return (0);

	//world_pose = globalpos_history[last_globalpos].globalpos;
	//carmen_grid_mapping_get_map_origin(&world_pose, &map_origin.x, &map_origin.y);

//	build_map_using_velodyne(sensor_params, sensor_data, r_matrix_robot_to_global);
	segment_remission_map(&localize_map.carmen_mean_remission_map, &localize_map.carmen_map);
	
	return (1);
}


int
run_snapshot_clean_map()
{
	int i;
	int current_point_cloud_index;//, before_point_cloud_index;
	static rotation_matrix *r_matrix_robot_to_global = NULL;
	static carmen_map_t *snapshot_map = NULL;
	
	snapshot_map = carmen_prob_models_check_if_new_snapshot_map_allocation_is_needed(snapshot_map, &occupancy_map);
	
	if (!globalpos_initialized)
		return (0);

	if (sensors_params[0].alive)
	{
		current_point_cloud_index =  sensors_data[0].point_cloud_index;
//		before_point_cloud_index =  ((sensors_data[0].point_cloud_index - 1) + NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;

//		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[0].robot_pose[before_point_cloud_index].orientation);
//		update_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[0], &sensors_data[0], r_matrix_robot_to_global, before_point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS);
//		carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);

		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[0].robot_pose[current_point_cloud_index].orientation);
//		update_cells_in_the_velodyne_perceptual_field( snapshot_map, &sensors_params[0], &sensors_data[0], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
//		carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);
	}

	for (i = 1; i < number_of_sensors; i++)
	{
		if (sensors_params[i].alive)
		{
			current_point_cloud_index =  sensors_data[i].point_cloud_index;
			r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[i].robot_pose[current_point_cloud_index].orientation);
//			update_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[i], &sensors_data[i], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
			carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&occupancy_map, snapshot_map);
		}
	}

	return (1);
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
clean_map_velodyne_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message)
{
	static int velodyne_message_id;
	//int ok_to_publish;

	int num_points = velodyne_message->number_of_32_laser_shots * sensors_params[0].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (sensors_data[0].last_timestamp == 0.0)
	{
		sensors_data[0].last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2;		// correntemente sao necessarias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return (ok_to_publish);
	}
	
	sensors_data[0].current_timestamp = velodyne_message->timestamp;

	build_sensor_point_cloud(&sensors_data[0].points, sensors_data[0].intensity, &sensors_data[0].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_partial_scan_update_points(velodyne_message, sensors_params[0].vertical_resolution,
			&sensors_data[0].points[sensors_data[0].point_cloud_index], sensors_data[0].intensity[sensors_data[0].point_cloud_index],
			sensors_params[0].ray_order,
			sensors_params[0].vertical_correction, sensors_params[0].range_max, velodyne_message->timestamp);

	sensors_data[0].robot_pose[sensors_data[0].point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data[0].robot_velocity[sensors_data[0].point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data[0].robot_timestamp[sensors_data[0].point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data[0].robot_phi[sensors_data[0].point_cloud_index] = globalpos_history[last_globalpos].phi;
	sensors_data[0].points_timestamp[sensors_data[0].point_cloud_index] = velodyne_message->timestamp;

	if (velodyne_message_id >= 0)
	{
		//if (build_snapshot_map)
		ok_to_publish = 1;
		//else

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	sensors_data[0].last_timestamp = velodyne_message->timestamp;
	
	return (ok_to_publish);
}


int
clean_map_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message)
{
	static int message_id;


	int num_points = message->number_of_shots * sensors_params[sensor_number].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (sensors_data[sensor_number].last_timestamp == 0.0)
	{
		sensors_data[sensor_number].last_timestamp = message->timestamp;
		message_id = -2;		// correntemente sďż˝o necessďż˝rias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return (ok_to_publish);
	}
	
	sensors_data[sensor_number].last_timestamp = sensors_data[sensor_number].current_timestamp = message->timestamp;

	build_sensor_point_cloud(&sensors_data[sensor_number].points, sensors_data[sensor_number].intensity, &sensors_data[sensor_number].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_variable_scan_update_points(message, sensors_params[sensor_number].vertical_resolution,
			&sensors_data[sensor_number].points[sensors_data[sensor_number].point_cloud_index],
			sensors_data[sensor_number].intensity[sensors_data[sensor_number].point_cloud_index],
			sensors_params[sensor_number].ray_order, sensors_params[sensor_number].vertical_correction,
			sensors_params[sensor_number].range_max, message->timestamp);

	sensors_data[sensor_number].robot_pose[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data[sensor_number].robot_velocity[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data[sensor_number].robot_timestamp[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data[sensor_number].robot_phi[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].phi;

	if (message_id >= 0)
	{
		if (build_snapshot_map)
			ok_to_publish = 1;
		else
			ok_to_publish = run_clean_map(/*&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global*/);

		if (message_id > 1000000)
			message_id = 0;
	}
	message_id++;
	sensors_data[sensor_number].last_timestamp = message->timestamp;
	
	return (ok_to_publish);
}


void
clean_map_merge_online_map_with_offline_map(carmen_map_t *offline_map)
{
	for (int i = 0; i < (occupancy_map.config.x_size * occupancy_map.config.y_size); i++)
		if (occupancy_map.complete_map[i] < 0.0)
       			occupancy_map.complete_map[i] = offline_map->complete_map[i];
}


//void
//clean_map_publish_map(double timestamp)
//{
//	if (build_snapshot_map)
//	{
//		memcpy(map.complete_map, offline_map.complete_map, offline_map.config.x_size *  offline_map.config.y_size * sizeof(double));
//		run_snapshot_clean_map();
//	}
//
//}


void
clean_map_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR)
{
	static double initial_time = 0.0;

	if (initial_time == 0.0)
		initial_time = carmen_get_time();
	
	if ((carmen_get_time() - initial_time) > 2.0)
		globalpos_initialized = 1;
	else
		return;

	last_globalpos = (last_globalpos + 1) % GLOBAL_POS_QUEUE_SIZE;

	globalpos_history[last_globalpos] = *globalpos_message;

	r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, globalpos_history[last_globalpos].pose.orientation);

	occupancy_map.config.x_origin = x_origin;
	occupancy_map.config.y_origin = y_origin;

	if (UPDATE_CELLS_BELOW_CAR)
		carmen_prob_models_updade_cells_bellow_robot(globalpos_message->globalpos, &occupancy_map, 0.0, &car_config);
}


void
clean_map_update_grid_map(carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params)
{
	carmen_update_cells_in_the_sensor_perceptual_field(&occupancy_map, xt, zt, sensor_params);
}


void
clean_map_save_current_map()
{

	//save the current map in the given map_path
	carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', &occupancy_map);
	carmen_grid_mapping_save_block_map_by_origin(map_path, 's', &sum_remission_map);
	carmen_grid_mapping_save_block_map_by_origin(map_path, '2', &sum_sqr_remission_map);
	carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', &count_remission_map);

	if (create_map_sum_and_count)
	{
		carmen_grid_mapping_save_block_map_by_origin(map_path, 'u', &sum_occupancy_map);
		carmen_grid_mapping_save_block_map_by_origin(map_path, 'e', &mean_occupancy_map);
		carmen_grid_mapping_save_block_map_by_origin(map_path, 'o', &count_occupancy_map);
		//	carmen_grid_mapping_save_block_map_by_origin(map_path, 'v', &variance_occupancy_map);
	}

}


void
clean_map_initialize(carmen_map_config_t *main_map_config, carmen_robot_ackerman_config_t main_car_config)
{
	car_config = main_car_config;
	map_config = *main_map_config;
	
	carmen_grid_mapping_create_new_map(&occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution);
	carmen_grid_mapping_create_new_map(&offline_map, map_config.x_size, map_config.y_size, map_config.resolution);
	carmen_grid_mapping_create_new_map(&sum_remission_map, map_config.x_size, map_config.y_size, map_config.resolution);
	carmen_grid_mapping_create_new_map(&sum_sqr_remission_map, map_config.x_size, map_config.y_size, map_config.resolution);
	carmen_grid_mapping_create_new_map(&count_remission_map, map_config.x_size, map_config.y_size, map_config.resolution);

	if (create_map_sum_and_count)
	{
		carmen_grid_mapping_create_new_map(&sum_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution);
		carmen_grid_mapping_create_new_map(&mean_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution);
		carmen_grid_mapping_create_new_map(&count_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution);
		//	carmen_grid_mapping_create_new_map(&variance_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution);
	}

	globalpos_initialized = 0; // Only considered initialized when first message is received

	globalpos_history = (carmen_localize_ackerman_globalpos_message *) calloc(GLOBAL_POS_QUEUE_SIZE, sizeof(carmen_localize_ackerman_globalpos_message));

	last_globalpos = 0;
}
