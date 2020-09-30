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

#include "moving_objects2.h"

#define	UPDATE_CELLS_CROSSED_BY_RAYS		1
#define	DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS	0

extern double safe_range_above_sensors;
extern double robot_wheel_radius;

extern double highest_sensor;

extern int merge_with_offline_map;
extern int update_and_merge_with_moving_objects2_saved_maps;
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

extern cv::Mat road_map;

extern carmen_map_t offline_map;

extern rotation_matrix *r_matrix_car_to_global;

int globalpos_initialized = 0;
extern carmen_localize_ackerman_globalpos_message *globalpos_history;
extern int last_globalpos;

carmen_robot_ackerman_config_t car_config;
carmen_map_config_t map_config;

extern double x_origin, y_origin; // map origin in meters


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

cv::Mat
segment_remission_map(carmen_map_t *remission_map, carmen_map_t *map)
{
	cv::Mat map_img = cv::Mat::zeros(remission_map->config.x_size, remission_map->config.y_size, CV_8UC1);
	cv::Mat occupancy_map_img = cv::Mat::zeros(remission_map->config.x_size, remission_map->config.y_size, CV_8UC1);

	int erosion_size = 1;
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE,
			cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
			cv::Point( erosion_size, erosion_size ));


	for (int i = 0; i < remission_map->config.x_size; i++)
	{
		for (int j = 0; j < remission_map->config.x_size; j++)
		{
			uchar aux = (uchar)((255.0 * (1.0 - (remission_map->map[i][j] < 0 ? 1 : remission_map->map[i][j]))) + 0.5);
			map_img.at<uchar>(i, j) = aux;

			aux = 255 * (map->map[i][j] > 0.5 ? 1.0 : 0.0);
			occupancy_map_img.at<uchar>(i, j) = aux;
		}
	}
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contours2;

    cv::medianBlur(map_img, map_img, 5);
	cv::threshold(map_img, map_img, 235, 255, cv::THRESH_BINARY);

	findContours(occupancy_map_img.clone(), contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	cv::dilate(occupancy_map_img, occupancy_map_img, element);

	findContours(occupancy_map_img.clone(), contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
	for (uint i = 0; i < contours2.size(); i++)
	{
		drawContours(map_img, contours2, i, CV_RGB(0, 0, 0), -1);
	}

	findContours(map_img.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	for (uint i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i])* map->config.resolution;

		if (area < 500)
			drawContours(map_img, contours, i, CV_RGB(0, 0, 0), -1);
	}

	erosion_size = 3;
	element = getStructuringElement( cv::MORPH_ELLIPSE,
			cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
			cv::Point( erosion_size, erosion_size ));
	//cv::medianBlur(map_img, map_img, 5);
	cv::dilate(map_img, map_img, element);

	for (uint i = 0; i < contours2.size(); i++)
	{
		drawContours(map_img, contours2, i, CV_RGB(0, 0, 0), -1);
	}

	erosion_size = 1;
	element = getStructuringElement( cv::MORPH_ELLIPSE,
			cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
			cv::Point( erosion_size, erosion_size ));

	cv::dilate(map_img, map_img, element);
	cv::erode(map_img, map_img, element);

	cv::threshold(map_img, map_img, 235, 255, cv::THRESH_BINARY_INV);
	return map_img;
}


void
copy_carmen_map_to_opencv(carmen_map_t *map, carmen_map_t *offline_grid_map,
		cv::Mat *map_img, cv::Mat *map_img_bkp, cv::Mat *offline_map_img)
{
	for (int i = 0; i < map->config.x_size; i++)
	{
		for (int j = 0; j < map->config.x_size; j++)
		{
			uchar aux = 255 * (map->map[i][j] > 0.5 ? 1.0 : 0.0);
			map_img_bkp->at<uchar>(i, j) = map_img->at<uchar>(i, j) = aux;

			aux = 255 * (offline_grid_map->map[i][j] > 0.5 ? 1.0 : 0.0);
			offline_map_img->at<uchar>(i, j) = aux;
		}
	}
}

void
find_min_max_contour_points(cv::Point2i &minP, cv::Point2i &maxP, std::vector<cv::Point> &contour)
{
	minP = cv::Point(INT_MAX, INT_MAX);
	maxP = cv::Point(-INT_MAX, -INT_MAX);

	for (uint i = 0; i < contour.size(); i++)
	{
		if (maxP.x < contour[i].x)
			maxP.x = contour[i].x;

		if (maxP.y < contour[i].y)
			maxP.y = contour[i].y;

		if (minP.x > contour[i].x)
			minP.x = contour[i].x;

		if (minP.y > contour[i].y)
			minP.y = contour[i].y;
	}
}

inline double
euclidian_distanvce2D(cv::Point &a, cv::Point &b)
{
	double diffX = a.x - b.x;
	double diffY = a.y - b.y;
	return sqrt((diffX * diffX) + (diffY * diffY));
}


void
show_clusters(int rows, int cols, std::vector<std::vector<cv::Point> > &clusters, std::vector<cv::Point> &centroids,char *name)
{
	std::vector<cv::Vec3b> colors;
	int n_labels = clusters.size();
	for (int i = 0; i < n_labels; ++i)
	{
		colors.push_back(cv::Vec3b(rand() & 255, rand() & 255, rand() & 255));
	}

	// Draw the points
	cv::Mat3b res(rows, cols, cv::Vec3b(0, 0, 0));
	for (uint i = 0; i < clusters.size(); ++i)
	{
		for (uint j = 0; j < clusters[i].size(); ++j)
			res(clusters[i][j]) = colors[i];
	}

	// Draw centroids
	for (int i = 0; i < n_labels; ++i)
	{
		cv::circle(res, centroids[i], 3, cv::Scalar(colors[i][0], colors[i][1], colors[i][2]), CV_FILLED);
		cv::circle(res, centroids[i], 10, cv::Scalar(255 - colors[i][0], 255 - colors[i][1], 255 - colors[i][2]));
	}

	cv::imshow(name, res);
	cv::waitKey(33);

}

void
create_groups(cv::Mat &map, std::vector<std::vector<cv::Point> > &clusters, std::vector<cv::Point> &centroids)
{
	cv::Mat map_img = map.clone();
	std::vector<std::vector<cv::Point> > contours;
	findContours(map_img.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	std::vector<std::vector<int> > contours_group(contours.size());

	for(uint i = 0; i < contours.size(); i++)
	{
		for (uint j = i; j < contours.size(); j++)
		{
			for (uint k = 0; k < contours[i].size(); k++)
			{
				for (uint l = 0; l < contours[j].size(); l++)
				{
					if (euclidian_distanvce2D(contours[i][k], contours[j][l]) < 5)
					{
						contours_group[i].push_back(j);
						k = contours[i].size();
						break;
					}
				}
			}
		}
	}

	for(uint i = 0; i < contours_group.size(); i++)
	{
		for (uint j = 0; j < contours_group[i].size(); j++)
		{
			for (uint k = i + 1; k < contours_group.size(); k++)
			{
				for (uint l = 0; l < contours_group[k].size(); l++)
				{
					if (contours_group[i][j] == contours_group[k][l])
					{
						std::vector<int> aux;
						for (uint m = 0; m < contours_group[k].size(); m++)
						{
							bool include = 1;
							for (uint n = 0; n < contours_group[i].size(); n++)
							{
								if (contours_group[k][m] == contours_group[i][n])
								{
									include = 0;
									break;
								}
							}
							if (include)
								aux.push_back(contours_group[k][m]);
						}
						contours_group[i].insert(contours_group[i].end(), aux.begin(), aux.end());
						contours_group[k].erase(contours_group[k].begin(), contours_group[k].end());
					}
				}
			}
		}
	}

	for (uint i = 0; i < contours_group.size(); i++)
	{
		if (contours_group[i].empty())
			continue;
		std::vector<cv::Point> cluster;
		cv::Point2i center = cv::Point(0, 0);
		for (uint j = 0; j < contours_group[i].size(); j++)
		{
			cv::Point2i minP, maxP;
			find_min_max_contour_points(minP, maxP, contours[contours_group[i][j]]);

			cv::Mat roi = map_img(cv::Rect(minP, maxP));

			for (int y = 0; y < roi.rows; y++)
				for (int x = 0; x < roi.cols; x++)
					if (roi.at<uchar>(y, x) > 128)
					{
						cluster.push_back(cv::Point(x + minP.x, y + minP.y));
						center.x += x + minP.x;
						center.y += y + minP.y;
					}
		}
		if (cluster.empty())
			continue;

		center.x /= cluster.size();
		center.y /= cluster.size();
		clusters.push_back(cluster);
		centroids.push_back(center);
	}

//	show_clusters(map_img.rows, map_img.cols, clusters, centroids, (char*)"Clusters0");
}


void
remove_static_objects(cv::Mat offline_map_img, carmen_map_t *map, std::vector<std::vector<cv::Point> > &clusters, std::vector<cv::Point> &centroids)
{
	for (uint i = 0; i < clusters.size(); i++)
	{
		int count = 0;
		for (uint j = 0; j < clusters[i].size(); j++)
		{
			if (offline_map_img.at<uchar>(clusters[i][j].y, clusters[i][j].x) > 128)
				count++;
		}

		double area = clusters[i].size() * map->config.resolution; // resulução da area ao quadrado

		if (((double) count / clusters[i].size()) > 0.9 /*|| area < 1.0*/ || area > 200.0)
		{
			clusters.erase(clusters.begin() + i);
			centroids.erase(centroids.begin() + i);
		}
	}
}


void
associate_objects(std::vector<cv::Point3d> &objs, std::vector<cv::Point> &centroids, std::vector<cv::Point> &last_centroids, double map_resolution)
{
	if (centroids.empty())
		return;

	objs.resize(centroids.size());

	for (uint i = 0; i < centroids.size(); i++)
	{
		int x_center = centroids[i].x;
		int y_center = centroids[i].y;
		double min_dist = DBL_MAX;
		double index = -1;

		for (uint j = 0; j < last_centroids.size(); j++)
		{
			int x_center2 = last_centroids[j].x;
			int y_center2 = last_centroids[j].y;
			double dist = sqrt(pow(x_center - x_center2, 2) + pow(y_center - y_center2, 2));
			if (min_dist > dist)
			{
				min_dist = dist;
				index = j;
			}
		}

		objs[i] = cv::Point3d(min_dist, i, index);
	}

	for (uint i = 0; i < objs.size(); i++)
	{
		if ((int)objs[i].z < 0.0)
			continue;

		for (uint j = i + 1; j < objs.size(); j++)
			if ((int)objs[i].z == (int)objs[j].z && (objs[j].x * map_resolution) > 1.5)
			{
				if (objs[i].x < objs[j].x)
					objs[j].z = -1.0;
				else
					objs[i].z = -1.0;

			}
	}
}


void
draw_associations(std::vector<cv::Point3d> objs, std::vector<std::vector<cv::Point> > &clusters, cv::Mat *map_img_out)
{

	std::vector<cv::RotatedRect> minRect(clusters.size());

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

		minRect[i] = cv::minAreaRect(cv::Mat(clusters[i]));


		for (uint j = 0; j < clusters[i].size(); j++)
			map_img_out->at<cv::Vec3b>(clusters[i][j].y, clusters[i][j].x) = cv::Vec3b(R, G, B);

		if ((int)objs[i].z > 0.0)
		{
			//drawContours(*last_map_img_out, contours2, (int)objs[i].z, color, -1);

			//desenha os retangulos
			cv::Point2f rect_points[4];
			minRect[i].points(rect_points);
			for( int j = 0; j < 4; j++ )
				cv::line( *map_img_out, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0, 0, 255), 1, 8 );
		}
	}

}


void
filter_objects_and_associate(carmen_map_t *map, carmen_map_t *offline_grid_map, cv::Mat &road_map)
{
	cv::Mat map_img = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC1);
	cv::Mat map_img_bkp = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC1);
	cv::Mat offline_map_img = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC1);
	//static cv::Mat last_map_img;
	int erosion_size = 2;
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE,
	                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
	                                       cv::Point( erosion_size, erosion_size ));

	static std::vector<std::vector<cv::Point> > last_clusters;
	static std::vector<cv::Point> last_centroids;

	cv::Mat map_img2 = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC1);
	cv::Mat map_img_out = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC3);
	//cv::Mat last_map_img_out = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC3);

	std::vector<std::vector<cv::Point> > contours;

	// #### copia de mapa do carmen para imagem opencv
	copy_carmen_map_to_opencv(map, offline_grid_map, &map_img, &map_img_bkp, &offline_map_img);

	std::vector<std::vector<cv::Point> > clusters;
	std::vector<cv::Point> centroids;

	findContours(road_map.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	for (uint i = 0; i < contours.size(); i++)
	{
		drawContours(map_img, contours, i, CV_RGB(0, 0, 0), -1);
	}
	//cv::dilate( map_img, map_img, element);
	cv::dilate( offline_map_img, offline_map_img, element);

	create_groups(map_img, clusters, centroids);

	map_img.copyTo(map_img2);

	// #### remove objetos estáticos e grandes
	remove_static_objects(offline_map_img, map, clusters, centroids);

	if (last_clusters.empty())
	{
		last_clusters = clusters;
		last_centroids = centroids;
		return;
	}
	show_clusters(map_img.rows, map_img.cols, clusters, centroids, (char*)"Clusters1");
	show_clusters(map_img.rows, map_img.cols, last_clusters, last_centroids, (char*)"Clusters");

	std::vector<cv::Point3d> objs;
	// #### associação dos objetos
	associate_objects(objs, centroids, last_centroids, map->config.resolution);

	// #### pinta as associações
	draw_associations(objs, clusters, &map_img_out);


	// TODO publicar a mensagem para o viewer 3D e implementar o filtro de partículas.
	cv::imshow("map_img_out", map_img_out);
    //cv::imshow("last_map_img_out", last_map_img_out);
    //cv::imshow("offline_map_img", offline_map_img);
	//cv::imshow("last_map_img", last_map_img);
	//cv::imshow("map_img", map_img);
	//cv::imshow("map", map_img2);
	//cv::waitKey(33);
	last_clusters = clusters;
	last_centroids = centroids;
}


static void
update_cells_in_the_velodyne_perceptual_field(carmen_map_t *snapshot_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global,
					      int point_cloud_index, int update_cells_crossed_by_rays, int build_snapshot_map __attribute__ ((unused)))
{
	int tid = omp_get_thread_num();
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	int N = v_zt.num_points / sensor_params->vertical_resolution;

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];

	double dt = 1.0 / (1808.0 * 12.0);
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
	int i = 0;

	// Ray-trace the grid
	#pragma omp for
	for (int j = 0; j < N; j += 1)
	{
		i = j * sensor_params->vertical_resolution;
		double dt2 = j * dt;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(sensor_data->robot_pose[point_cloud_index], dt2, v, phi, car_config.distance_between_front_and_rear_axles);
		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		carmen_prob_models_compute_relevant_map_coordinates(sensor_data, sensor_params, i, robot_interpolated_position.position, sensor_board_1_pose,
				r_matrix_robot_to_global, board_to_car_matrix, robot_wheel_radius, x_origin, y_origin, &car_config, robot_near_strong_slow_down_annotation, tid);

		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, i, highest_sensor, safe_range_above_sensors,
				robot_near_strong_slow_down_annotation, tid);

		//sensor_params->log_odds.log_odds_free = -37;

		carmen_prob_models_update_cells_crossed_by_ray(snapshot_map, sensor_params, sensor_data, tid);

		//sensor_params->log_odds.log_odds_occ = 37;

		carmen_prob_models_update_log_odds_of_nearest_target(snapshot_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid);
//		carmen_prob_models_update_log_odds_of_cells_hit_by_rays(snapshot_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid);

	}

	cv::Mat road_map = segment_remission_map(&localize_map.carmen_mean_remission_map, &localize_map.carmen_map);
	filter_objects_and_associate(snapshot_map, &localize_map.carmen_map, road_map);
	//show_map(&offline_map);
	//printf("\n###############################################################\n");
}


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


static void
build_map_using_velodyne(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	//int N = 4;
	static carmen_map_t *snapshot_map;
	static int first = 1;
	if (first)
	{
		snapshot_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
		first = 0;
	}

#pragma omp parallel num_threads(number_of_threads)
	{
		int tid = omp_get_thread_num();
		snapshot_map = carmen_prob_models_check_if_new_snapshot_map_allocation_is_needed(snapshot_map, &occupancy_map);
		//set_map_equal_offline_map(&map);
		//add_offline_map_over_unknown(&map);

		if(decay_to_offline_map)
			map_decay_to_offline_map(&occupancy_map);

		// @@@ Alberto: Mapa padrao Lucas -> colocar DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS ao inves de UPDATE_CELLS_CROSSED_BY_RAYS
		//update_cells_in_the_velodyne_perceptual_field(&map, snapshot_map, sensor_params, sensor_data, r_matrix_robot_to_global, sensor_data->point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, update_and_merge_with_snapshot_map);
		update_cells_in_the_velodyne_perceptual_field(snapshot_map, sensor_params, sensor_data, r_matrix_robot_to_global, sensor_data->point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS, update_and_merge_with_snapshot_map);
		carmen_prob_models_update_current_map_with_snapshot_map_and_clear_snapshot_map(&occupancy_map, snapshot_map);
	}
}


void
initialize_first_map_block_origin(carmen_map_t *current_carmen_map, carmen_position_t *map_origin, char map_type)
{
	current_carmen_map->complete_map = NULL;
	x_origin = map_origin->x;
	y_origin = map_origin->y;

	if (update_and_merge_with_moving_objects2_saved_maps)
	{
		carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, map_type, x_origin, y_origin, current_carmen_map);

		if (current_carmen_map->complete_map == NULL)
			carmen_grid_mapping_initialize_map(current_carmen_map, occupancy_map.config.x_size, occupancy_map.config.resolution);
	}
	else		
		carmen_grid_mapping_get_buffered_map(x_origin, y_origin, current_carmen_map, map_type);
}



void
moving_objects2_change_map_origin_to_another_map_block(carmen_position_t *map_origin)
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

		if (update_and_merge_with_moving_objects2_saved_maps)
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
			carmen_grid_mapping_get_buffered_map(x_origin, y_origin, &new_carmen_map);
		}

		//destroy current map and assign new map to current map
		carmen_grid_mapping_swap_maps_and_clear_old_map(&occupancy_map, &new_carmen_map);

		if (update_and_merge_with_moving_objects2_saved_maps)
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
run_moving_objects2(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	//carmen_point_t world_pose;
	//carmen_position_t map_origin;

	if (!globalpos_initialized)
		return (0);

	//world_pose = globalpos_history[last_globalpos].globalpos;
	//carmen_grid_mapping_get_map_origin(&world_pose, &map_origin.x, &map_origin.y);

	build_map_using_velodyne(sensor_params, sensor_data, r_matrix_robot_to_global);
	
	return (1);
}


int
run_snapshot_moving_objects2()
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
		update_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[0], &sensors_data[0], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
		carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&occupancy_map, snapshot_map);
	}

	for (i = 1; i < number_of_sensors; i++)
	{
		if (sensors_params[i].alive)
		{
			current_point_cloud_index =  sensors_data[i].point_cloud_index;
			r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[i].robot_pose[current_point_cloud_index].orientation);
			update_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[i], &sensors_data[i], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
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
moving_objects2_velodyne_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message)
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
moving_objects2_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message)
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
			ok_to_publish = run_moving_objects2(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);

		if (message_id > 1000000)
			message_id = 0;
	}
	message_id++;
	sensors_data[sensor_number].last_timestamp = message->timestamp;
	
	return (ok_to_publish);
}


void
moving_objects2_merge_online_map_with_offline_map(carmen_map_t *offline_map)
{
	for (int i = 0; i < (occupancy_map.config.x_size * occupancy_map.config.y_size); i++)
		if (occupancy_map.complete_map[i] < 0.0)
       			occupancy_map.complete_map[i] = offline_map->complete_map[i];
}


void
moving_objects2_publish_map(double timestamp)
{
	if (build_snapshot_map)
	{
		memcpy(occupancy_map.complete_map, offline_map.complete_map, offline_map.config.x_size *  offline_map.config.y_size * sizeof(double));
		run_snapshot_moving_objects2();
	}
}


void
moving_objects2_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR)
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
moving_objects2_update_grid_map(carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params)
{
	carmen_update_cells_in_the_sensor_perceptual_field(&occupancy_map, xt, zt, sensor_params);
}


void
moving_objects2_save_current_map()
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
moving_objects2_initialize(carmen_map_config_t *main_map_config, carmen_robot_ackerman_config_t main_car_config)
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
