#include "neural_mapper_io.h"


Neural_map_queue *neural_mapper_acumulator;
Neural_map new_neural_map = Neural_map();

static int neural_mapper_max_distance_meters;
static int neural_map_num_clouds;
static double last_x_meters_global = 0.0;
static double last_y_meters_global = 0.0;
static int map_index;


void
neural_mapper_initialize(int max_distance_meters, int num_clouds, carmen_map_config_t map_config)
{
	neural_mapper_max_distance_meters = max_distance_meters;
	neural_map_num_clouds = num_clouds;
	int img_dimensions = int(2*neural_mapper_max_distance_meters/map_config.resolution);
	new_neural_map = Neural_map(img_dimensions, img_dimensions, map_config.resolution, 0.0, 0.0, 0.0, neural_mapper_max_distance_meters);
	neural_mapper_acumulator = new Neural_map_queue(neural_map_num_clouds, img_dimensions, img_dimensions, map_config.resolution, neural_mapper_max_distance_meters);
}


void
neural_mapper_export_dataset_as_png(bool get_next_map, char path[])
{
	if (get_next_map)
	{
		neural_mapper_acumulator->export_png(path, map_index);
		map_index++;
	}
}


void
neural_mapper_export_dataset_as_binary_file(bool get_next_map, char path[], double current_timestamp, carmen_pose_3D_t neural_mapper_robot_pose)
{
	if (get_next_map)
	{
		neural_mapper_acumulator->export_as_binary_file(path, map_index, current_timestamp, neural_mapper_robot_pose);
		map_index++;
	}
}


void
neural_mapper_update_queue_and_clear_maps()
{
	neural_mapper_acumulator->push(new_neural_map);
	new_neural_map.clear_maps();
}


bool
neural_mapper_compute_travelled_distance(carmen_position_t *neural_mapper_car_position_according_to_map, carmen_pose_3D_t neural_mapper_robot_pose,
										double x_origin, double y_origin, int neural_mapper_data_pace)
{
	neural_mapper_car_position_according_to_map->x = neural_mapper_robot_pose.position.x - x_origin;
	neural_mapper_car_position_according_to_map->y = neural_mapper_robot_pose.position.y - y_origin;
	new_neural_map.set_car_x(neural_mapper_car_position_according_to_map->x);
	new_neural_map.set_car_y(neural_mapper_car_position_according_to_map->y);
	new_neural_map.set_car_rotation(neural_mapper_robot_pose.orientation.yaw);

	double travelled_distance = sqrt(pow((neural_mapper_robot_pose.position.x - last_x_meters_global), 2.0) + pow((neural_mapper_robot_pose.position.y - last_y_meters_global), 2.0));
	bool get_next_map = (travelled_distance > neural_mapper_data_pace);
	if (get_next_map)
	{
		last_x_meters_global = neural_mapper_robot_pose.position.x;
		last_y_meters_global = neural_mapper_robot_pose.position.y;
	}

	return get_next_map;
}


void
neural_mapper_update_inputs_maps_with_new_point(int x_index, int y_index, double z_meters)
{

    // update number of points
    if (new_neural_map.raw_number_of_lasers_map.map[x_index][y_index]==-1.0)
    	new_neural_map.raw_number_of_lasers_map.map[x_index][y_index] = 1.0;
    else
	    new_neural_map.raw_number_of_lasers_map.map[x_index][y_index]++;
    int n = new_neural_map.raw_number_of_lasers_map.map[x_index][y_index];

    // update min and max values
	if (n == 1)
	{
		new_neural_map.raw_min_hight_map.map[x_index][y_index] = z_meters;
		new_neural_map.raw_max_hight_map.map[x_index][y_index] = z_meters;
	}
	else if (z_meters < new_neural_map.raw_min_hight_map.map[x_index][y_index])
	    new_neural_map.raw_min_hight_map.map[x_index][y_index] = z_meters;
	else if (z_meters > new_neural_map.raw_max_hight_map.map[x_index][y_index])
	    new_neural_map.raw_max_hight_map.map[x_index][y_index] = z_meters;

	// update mean
	if (n == 1)
		new_neural_map.raw_mean_hight_map.map[x_index][y_index] = z_meters;
	else
		new_neural_map.raw_mean_hight_map.map[x_index][y_index] = ((new_neural_map.raw_mean_hight_map.map[x_index][y_index]*(n-1)) + z_meters)/(n);


	new_neural_map.raw_square_sum_map.map[x_index][y_index] += (z_meters*z_meters);

}



int
neural_mapper_update_input_maps(sensor_data_t * sensor_data, sensor_parameters_t *sensor_params, int thread_id,
		carmen_map_t *log_odds_snapshot_map, carmen_map_config_t map_config, double x_origin, double y_origin,
		double highest_sensor, double safe_range_above_sensors)
{

	for (int i = 0; i < sensor_params->vertical_resolution; i++)
	{
		//printf("sensor_x: %lf, sensor_y: %lf, car_x: %lf, car_y: %lf, dif_x: %lf\n", sensor_data->ray_position_in_the_floor[thread_id][i].x, sensor_data->ray_position_in_the_floor[thread_id][i].y, sensor_data->ray_origin_in_the_floor[thread_id][i].x, sensor_data->ray_origin_in_the_floor[thread_id][i].y, round(sensor_data->ray_position_in_the_floor[thread_id][i].x - sensor_data->ray_origin_in_the_floor[thread_id][i].x));
		int center = round(neural_mapper_max_distance_meters/0.2);
		int x_ray_index = round((sensor_data->ray_position_in_the_floor[thread_id][i].x -
								(sensor_data->robot_pose[sensor_data->point_cloud_index].position.x - x_origin))/log_odds_snapshot_map->config.resolution) + center;
		int y_ray_index = round((sensor_data->ray_position_in_the_floor[thread_id][i].y - (sensor_data->robot_pose[sensor_data->point_cloud_index].position.y - y_origin))/log_odds_snapshot_map->config.resolution) + center;
		double z = sensor_data->obstacle_height[thread_id][i];

		//remission = sensor_data->processed_intensity[thread_id][i];
		double x_meters_distance = abs(sensor_data->ray_position_in_the_floor[thread_id][i].x - sensor_data->ray_origin_in_the_floor[thread_id][i].x);
		double y_meters_distance = abs(sensor_data->ray_position_in_the_floor[thread_id][i].y - sensor_data->ray_origin_in_the_floor[thread_id][i].y);
		double distance_meters = (sqrt(pow(x_meters_distance,2)+pow(y_meters_distance,2)));
		//printf("Distances = %d %d and max = %d\n", x_meters_distance, y_meters_distance, neural_mapper_max_distance_meters);
		bool unaceptable_height = carmen_prob_models_unaceptable_height(z, highest_sensor, safe_range_above_sensors);
		//printf("%lf \n", z);
		if(distance_meters < neural_mapper_max_distance_meters && x_ray_index >= 0 &&
				x_ray_index < int(2*neural_mapper_max_distance_meters/map_config.resolution) &&
				y_ray_index >= 0 && y_ray_index < int(2*neural_mapper_max_distance_meters/map_config.resolution) &&
				!(unaceptable_height))
		{
//			printf("z: %lf", z);
			//update_neural_mapper_inputs_maps_with_new_point(x_ray_index, y_ray_index, z, remission);
			neural_mapper_update_inputs_maps_with_new_point(x_ray_index, y_ray_index, z);
		}
	}

	return 0;
}


int
neural_mapper_update_output_map(carmen_map_t offline_map, carmen_position_t car_position)
{
	for (int i = 0; i < new_neural_map.neural_mapper_occupancy_map.config.y_size; i++)
	{
		for (int j = 0; j < new_neural_map.neural_mapper_occupancy_map.config.x_size; j++)
		{
			int i_on_mapper = (i + round(car_position.x/0.2) - round(neural_mapper_max_distance_meters/0.2));
			int j_on_mapper = (j + round(car_position.y/0.2) - round(neural_mapper_max_distance_meters/0.2));
			if(i_on_mapper < 1050 && i_on_mapper >= 0 && j_on_mapper < 1050 && j_on_mapper >= 0)
			{
				// labels: unknown = 0 (antes era 1); empty = 1 (antes era)2; occupied = 2 (antes era 3);
				// unknown
				if(offline_map.map[i_on_mapper][j_on_mapper] < 0.0)
				{
					new_neural_map.neural_mapper_occupancy_map.map[i][j] = 0;
				}
				// occupied
				else if(offline_map.map[i_on_mapper][j_on_mapper] >= 0.5)
					new_neural_map.neural_mapper_occupancy_map.map[i][j] = 2; // 3
				// empty
				else
					//printf("Valor de vazio? %lf\n", offline_map.map[i_on_mapper][j_on_mapper]);
					new_neural_map.neural_mapper_occupancy_map.map[i][j] = 1; // 2
			}
			else
				new_neural_map.neural_mapper_occupancy_map.map[i][j] = 0;
		}
	}

	return 0;
}

cv::Mat
neural_map_run_foward(int size)
{
	printf("Entrei aqui\n");
	cv::Mat a;
	neural_mapper_acumulator->foward_map(size);
	printf("Voltou hein\n");
	return a;

}
