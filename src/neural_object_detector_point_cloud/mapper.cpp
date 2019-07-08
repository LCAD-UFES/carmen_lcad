#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>
#include <carmen/mapper_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/global_graphics.h>
#include <omp.h>

#include <iostream>
#include <fstream>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include "mapper.h"

#define	UPDATE_CELLS_CROSSED_BY_RAYS		1
#define	DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS	0

extern double safe_range_above_sensors;
extern double robot_wheel_radius;

extern double highest_sensor;

extern int update_and_merge_with_mapper_saved_maps;
extern int build_snapshot_map;
extern int update_and_merge_with_snapshot_map;
extern int decay_to_offline_map;
extern int create_map_sum_and_count;
extern int use_remission;

extern sensor_parameters_t *sensors_params;
extern sensor_data_t *sensors_data;
extern int number_of_sensors;

extern char *map_path;


extern int robot_near_strong_slow_down_annotation;
extern int ok_to_publish;
extern int number_of_threads;

extern int file_warnings;

#define HUGE_DISTANCE     32000

#define MAX_VIRTUAL_LASER_SAMPLES 10000

/**
 * The map
 */

carmen_map_t map, sum_remission_map, sum_sqr_remission_map, count_remission_map, moving_objects_raw_map,
				sum_occupancy_map, mean_occupancy_map, count_occupancy_map; //, variance_occupancy_map;

carmen_map_t cost_map;

/*************************************
* Variables for Neural mapper dataset generation maps
*/
carmen_map_t 	normalized, raw_mean_hight_map, raw_max_hight_map, raw_min_hight_map, standard_deviation_map, raw_reflectivity_map, raw_number_of_lasers_map, neural_mapper_occupancy_map, raw_square_sum_map, std_map;
extern int generate_neural_mapper_dataset;
extern int neural_mapper_max_distance_meters;
static double last_x_meters_global = 0.;
static double last_y_meters_global = 0.;
static double x_meters_car_position_according_to_map = 0.;
static double y_meters_car_position_according_to_map = 0.;
static bool get_next_map = false;

//a cada neural_mapper_data_pace uma amostra do neural mapper sera gerada
extern int neural_mapper_data_pace;
//int number_of_data_passed = 0;
static int map_index = 0;
/***********************************/

extern carmen_map_t offline_map;

extern rotation_matrix *r_matrix_car_to_global;

int globalpos_initialized = 0;
extern carmen_localize_ackerman_globalpos_message *globalpos_history;
extern int last_globalpos;

extern carmen_robot_ackerman_config_t car_config;
extern carmen_map_config_t map_config;

double x_origin, y_origin; // map origin in meters

static carmen_laser_laser_message flaser; // moving_objects

extern carmen_rddf_annotation_message last_rddf_annotation_message;

carmen_mapper_virtual_laser_message virtual_laser_message;

carmen_mapper_virtual_scan_message virtual_scan_message;

carmen_moving_objects_point_clouds_message moving_objects_message;


//////////////////////////////////////  RANIK  ///////////////////////////////////////////////////////
double*
convert_map_to_array(carmen_map_p updated_map)
{
    unsigned int width = updated_map->config.x_size;
    unsigned int height = updated_map->config.y_size;
    unsigned int size = width * height;
    unsigned int row = 0, col = 0, index = 0;
    double *array_map = (double *) malloc (size * sizeof(double));

    for (unsigned int i = 0; i < size; ++i)
    {
        row = (height - 1) - i % height;

        col = i / height;

        index = row * width + col;

        array_map[index] =  1 - updated_map->complete_map[i];
    }
    return (array_map);
}


void
color_moving_objecst_cells(std::vector<carmen_vector_2D_t> moving_objecst_cells_vector, cv::Mat &map_img)
{
	int x = 0, y = 0;

	for (int i = 0; i < moving_objecst_cells_vector.size(); i++)
	{
		//printf("%d %d\n", moving_objecst_cells_vector[i].x, moving_objecst_cells_vector[i].y);

		x = (double) (moving_objecst_cells_vector[i].x - map_config.x_origin) / map_config.resolution;
		y = (double) (moving_objecst_cells_vector[i].y - map_config.y_origin) / map_config.resolution;
		if (x < 0 || x > map_img.cols || y < 0 || y > map_img.rows)
		{
			//printf("Cell out of Map %d %d\n", x, y);
			continue;
		}
		//printf("FOI\n");
		rectangle(map_img,cv::Rect(x, (map_config.y_size - 1 - y), 1, 1), 0.5, CV_FILLED, 8, 0);
	}
}


void
cv_draw_map(std::vector<carmen_vector_2D_t> moving_objecst_cells_vector)
{
	double *inverted_map = convert_map_to_array(&map);

	//unsigned int height = map.config.y_size;
	//double inverse_resolution = 1.0 / map.config.resolution;

	cv::Mat map_img = cv::Mat(map.config.x_size, map.config.y_size, CV_64FC1 , inverted_map, 0);

	//cv::Mat map_img_3c;
	//map_img.convertTo(map_img_3c, CV_64FC3);

	color_moving_objecst_cells(moving_objecst_cells_vector, map_img);

	cv::imshow("Map", map_img);
	cv::waitKey(1);
	free(inverted_map);
}
//////////////////////////////////////////////////////////////////////////////////////////////////



carmen_map_t *
get_the_map()
{
	return (&map);
}


void
change_sensor_rear_range_max(sensor_parameters_t *sensor_params, double angle)
{
	if ((angle > M_PI / 2.0) || (angle < -M_PI / 2.0))
		sensor_params->current_range_max = sensor_params->range_max / sensor_params->range_max_factor;
	else
		sensor_params->current_range_max = sensor_params->range_max;
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


void
compute_virtual_scan_point(int ray_id, int tid, sensor_parameters_t* sensor_params, sensor_data_t* sensor_data,
		carmen_map_t* log_odds_snapshot_map, bool is_ldmrs = false)
{
	if (!sensor_data->maxed[tid][ray_id])
	{
		int sensor = virtual_scan_message.num_sensors - 1;
		int ray_num = virtual_scan_message.virtual_scan_sensor[sensor].num_points;

		cell_coords_t cell_hit_by_ray;
		double x = sensor_data->ray_position_in_the_floor[tid][ray_id].x;
		double y = sensor_data->ray_position_in_the_floor[tid][ray_id].y;
		cell_hit_by_ray.x = round(x / log_odds_snapshot_map->config.resolution);
		cell_hit_by_ray.y = round(y / log_odds_snapshot_map->config.resolution);
		if (map_grid_is_valid(log_odds_snapshot_map, cell_hit_by_ray.x, cell_hit_by_ray.y) &&
			((sensor_data->occupancy_log_odds_of_each_ray_target[tid][ray_id] > sensor_params->log_odds.log_odds_occ / 10.0) || is_ldmrs) &&
			(offline_map.map[cell_hit_by_ray.x][cell_hit_by_ray.y] <= 0.5))
		{
			virtual_scan_message.virtual_scan_sensor[sensor].points[ray_num].x = x + x_origin;
			virtual_scan_message.virtual_scan_sensor[sensor].points[ray_num].y = y + y_origin;
			virtual_scan_message.virtual_scan_sensor[sensor].num_points = ray_num + 1;
		}
	}
}


void
build_virtual_scan_message_ldmrs(int tid, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, carmen_map_t *log_odds_snapshot_map)
{
	double min_ray_size_in_the_floor = 1000.0;
	int nearest_target = 0;
	
	for (int k = 0; k < sensor_params->vertical_resolution; k++)
	{
		double ray_size_in_the_floor = sensor_data->ray_size_in_the_floor[tid][k];

		if (ray_size_in_the_floor > min_ray_size_in_the_floor)
		{
			ray_size_in_the_floor = min_ray_size_in_the_floor;
			nearest_target = k;
		}
	}
	compute_virtual_scan_point(nearest_target, tid, sensor_params, sensor_data, log_odds_snapshot_map, true);
}


void
build_virtual_scan_message_velodyne(int tid, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, carmen_map_t *log_odds_snapshot_map)
{
	int k = sensor_data->ray_that_hit_the_nearest_target[tid];
	compute_virtual_scan_point(k, tid, sensor_params, sensor_data, log_odds_snapshot_map);
}


void
initialize_virtual_scan_message_update(int sensor_id, carmen_pose_3D_t robot_pose, carmen_pose_3D_t robot_interpolated_position,
		double time_spent_in_the_entire_sensor_sweep, double last_sensor_angle, double v, double phi,
		int num_points, int point_cloud_index, rotation_matrix *r_matrix_robot_to_global, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data)
{
	r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_pose.orientation);
	carmen_vector_3D_t sensor_position_in_the_world = carmen_change_sensor_reference(robot_interpolated_position.position,
			sensor_params->sensor_robot_reference, r_matrix_robot_to_global);

	int n = virtual_scan_message.num_sensors;
	virtual_scan_message.virtual_scan_sensor = (carmen_virtual_scan_sensor_t *) realloc(virtual_scan_message.virtual_scan_sensor, (n + 1) * sizeof(carmen_virtual_scan_sensor_t));

	carmen_point_t global_pos = {robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.yaw};
	carmen_point_t sensor_pos = {sensor_position_in_the_world.x, sensor_position_in_the_world.y, robot_interpolated_position.orientation.yaw};
	virtual_scan_message.virtual_scan_sensor[n].sensor_pos = sensor_pos;
	virtual_scan_message.virtual_scan_sensor[n].global_pos = global_pos;
	virtual_scan_message.virtual_scan_sensor[n].sensor_id = sensor_id;
	virtual_scan_message.virtual_scan_sensor[n].time_spent_in_the_entire_sensor_sweep = time_spent_in_the_entire_sensor_sweep;
	virtual_scan_message.virtual_scan_sensor[n].last_sensor_angle = last_sensor_angle;
	virtual_scan_message.virtual_scan_sensor[n].v = v;
	virtual_scan_message.virtual_scan_sensor[n].w = tan(phi) * (v / car_config.distance_between_front_and_rear_axles);
	virtual_scan_message.virtual_scan_sensor[n].num_points = 0; // O total de pontos alocados abaixo pode nao ser totalmente alocado, pois os pontos realmente usados sao preenchidos sob demanda;
	virtual_scan_message.virtual_scan_sensor[n].points = (carmen_point_t *) malloc(num_points * sizeof(carmen_point_t));
	virtual_scan_message.virtual_scan_sensor[n].timestamp = sensor_data->points_timestamp[point_cloud_index];

	virtual_scan_message.num_sensors = n + 1;
}


/**********************************************************
* Functions for generate neural mapper dataset
***********************************************************/

int 
map_to_bitmap(carmen_map_t complete_map, char* bitmap_name)
{
	// headers do bitmap
	FILE *f;

	unsigned char *img = NULL;

	int width = complete_map.config.x_size;
	int height = complete_map.config.y_size;
	int i;
	unsigned char r,g,b,prob;

	int filesize = 54 + 3*width*height;

	if (img)
	    free(img);

	img = (unsigned char *)calloc(3 * width * height, sizeof(unsigned char));


	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			if (complete_map.map[x][y] == -1)
			{
				r = 0;
				g = 0;
				b = 255;
			} 
			else 
			{
				prob = (unsigned char) ((1 - complete_map.map[x][y]) * 255.0); // converte a probabilidade para [0-255]
				r = prob;
				g = prob;
				b = prob;
			}
			img[(x+(height-1-y)*width)*3+2] = (unsigned char)(r);	// height -1 -y pois no mapa a origem é
			img[(x+(height-1-y)*width)*3+1] = (unsigned char)(g);	// no canto inferior esquerdo e no
			img[(x+(height-1-y)*width)*3+0] = (unsigned char)(b);	// bitmap é no canto superior esquerdo
		}
	}

	// preenche os headers do bitmap
	unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
	unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
	unsigned char bmppad[3] = {0,0,0};

	bmpfileheader[ 2] = (unsigned char)(filesize    );
	bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
	bmpfileheader[ 4] = (unsigned char)(filesize>>16);
	bmpfileheader[ 5] = (unsigned char)(filesize>>24);

	bmpinfoheader[ 4] = (unsigned char)(width    );
	bmpinfoheader[ 5] = (unsigned char)(width>> 8);
	bmpinfoheader[ 6] = (unsigned char)(width>>16);
	bmpinfoheader[ 7] = (unsigned char)(width>>24);
	bmpinfoheader[ 8] = (unsigned char)(height    );
	bmpinfoheader[ 9] = (unsigned char)(height>> 8);
	bmpinfoheader[10] = (unsigned char)(height>>16);
	bmpinfoheader[11] = (unsigned char)(height>>24);

	char bitmap_file_name[1024];

	sprintf(bitmap_file_name,"%s.bmp",bitmap_name);

	f = fopen(bitmap_file_name,"wb");
	fwrite(bmpfileheader,1,14,f);
	fwrite(bmpinfoheader,1,40,f);
	for(i=0; i<height; i++)
	{
	    fwrite(img+(width*(height-i-1)*3),3,width,f);
	    fwrite(bmppad,1,(4-(width*3)%4)%4,f);			// a largura deve ser multipla de 4 no bitmap
	}
	fclose(f);

//	printf("Completo!\n%d %d\n", complete_map.config.x_size, complete_map.config.y_size);

	return 0;
}
/*
carmen_map_t 
fixed_normalize_map(carmen_map_t *value_map)
{
	double max = 5.;
	double min = -0.5;

	//normalization
	for (int i = 0; i < value_map->config.y_size;i++)
	{
		for (int j = 0; j < value_map->config.x_size;j++)
		{
			if (value_map->map[j][i] != -1)
			{
				float temp = ((value_map->map[j][i]-min)/(max-min));
				if(temp > 10.0)
					temp = 0.0;
				if(temp < 0.0)
					temp = 0.0;
				normalized.map[j][i] = temp;
			}
		}
	}
	//printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
	return normalized;
}
*/
// normalize map to 0-255 values
carmen_map_t
fixed_normalize_map(carmen_map_t value_map, double new_max)
{
	double min = -1.0;
	double max = 5; // altura maxima considerada de 5 metros

	carmen_map_t normalized;
	carmen_grid_mapping_create_new_map(&normalized, value_map.config.x_size, value_map.config.y_size, value_map.config.resolution, 'u');

	//normalization
	for(int i = 0; i < value_map.config.y_size;i++){
		for(int j = 0; j < value_map.config.x_size;j++){
			double temp = (value_map.map[j][i]-min)*(new_max)/(max-min);
			normalized.map[j][i] = temp;
			//printf("%lf\n", temp);
		}
	}

	//printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
	return normalized;
}

// normalize map to 0-1 values
carmen_map_t 
normalize_map(carmen_map_t *value_map)
{
	double max = -2.0;
	double min = 10.0;


	//find mean, min and max values
	for (int i = 0; i < value_map->config.y_size;i++)
	{
		for (int j = 0; j < value_map->config.x_size;j++)
		{
			if (value_map->map[j][i] != -1)
			{
				if (max < value_map->map[j][i])
					max = value_map->map[j][i];
				if (min > value_map->map[j][i])
					min = value_map->map[j][i];
			}
		}
	}

	//normalization
	for (int i = 0; i < value_map->config.y_size;i++)
	{
		for (int j = 0; j < value_map->config.x_size;j++)
		{
			if (value_map->map[j][i] != -1)
			{
				float temp = ((value_map->map[j][i]-min)/(max-min));
				normalized.map[j][i] = temp;
			}
		}
	}
	//printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
	return normalized;
}

/*Verificar algum clean map e remover essa*/
void
set_map_to_void(carmen_map_t * map)
{
	for (int i = 0; i < map->config.y_size;i++)
	{
		for (int j = 0; j < map->config.x_size;j++)
		{
			map->map[j][i]= -1.;
		}
	}
}

// joga todos os valores para 0
void 
set_map_values_to_zero(carmen_map_t *value_map){
	for(int i = 0; i < value_map->config.y_size;i++){
		for(int j = 0; j < value_map->config.x_size;j++){
			value_map->map[j][i]= 0;
		}
	}
}
// display the value of interest of the map in a grayscale img
void
show_map(carmen_map_t *value_map, char *name)
{

	map_to_bitmap(fixed_normalize_map(*value_map, 255.), name);
	set_map_to_void(value_map);
	set_map_to_void(&normalized);
}

//save png map with correct name pattern for later processing of the images
void 
map_to_csv(carmen_map_t complete_map, char* csv_name)
{
	char csv_file_name[1024];

	sprintf(csv_file_name,"%s.csv",csv_name);

	std::ofstream myfile;
    myfile.open (csv_file_name);

    int width = complete_map.config.x_size;
	int height = complete_map.config.y_size;

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			myfile << complete_map.map[x][y];
			if (x < width-1)
				myfile << ",";
		}
		if (y < height-1)
			myfile << "\n";
    }
    myfile.close();
}


void
map_to_png(carmen_map_t complete_map, char* csv_name, bool is_label)
{

	carmen_map_t png_map = complete_map;
	if(!is_label)
		png_map = fixed_normalize_map(complete_map, 255.);

	char png_file_name[1024];
//	int width = complete_map.config.x_size;
//	int height = complete_map.config.y_size;

//	static cv::Mat neural_map_img = cv::Mat(cv::Size(width, height), CV_8UC1);
	sprintf(png_file_name,"%s.png",csv_name);

// jeito mais rapido de gravar com Opencv
	cv::Mat png_mat = cv::Mat(complete_map.config.x_size, complete_map.config.y_size, CV_64FC1, *png_map.map);

/*
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			neural_map_img.at<uchar>(height - 1 - j, i) = complete_map.map[i][j];
		}
	}
*/
	cv::imwrite(png_file_name, png_mat);
}


void
//save_map(carmen_map_t map, char* map_name, char* path, double timestamp, double x_meters_position_on_map,  double y_meters_position_on_map, double resolution, double angle, bool is_label)
save_map(carmen_map_t map, char* map_name, char* path, bool is_label)
{
	//printf("%lf - %lf - %lf - %lf - %lf - %lf\n", timestamp, x_meters_position_on_map, y_meters_position_on_map, (180*pose.orientation.pitch/3.1413), (180*pose.orientation.roll/3.1413), (180*pose.orientation.yaw/3.1413));
	char name[500];
	//sprintf(name, "%s/%lf_%s_%0.3lf_%0.3lf_%.2lf_%0.6lf", path, timestamp, map_name, x_meters_position_on_map, y_meters_position_on_map, resolution, angle);
	if(!is_label)
		sprintf(name, "%s/data/%d_%s", path, map_index, map_name);
	else
		sprintf(name, "%s/labels/%d_%s", path, map_index, map_name);
	//printf("%s\n", name);
	//map_to_csv(map, name);
	map_to_png(map, name, is_label);
	if(is_label)
	{
		sprintf(name, "%s/labels/%d_view", path, map_index);
		map_to_png(map, name, false);
	}
	set_map_to_void(&map);
}

void
update_neural_mapper_inputs_maps_with_new_point(int x_index, int y_index, double z_meters, double remission)
{
	//printf("METERS: (%lf)x(%lf)\n", x_meters, y_meters);
 
	//Find index in map coordinates
	// offline map nao foi gerado nesse ponto!

	// coloca imagem no meio
	//int middle = round(neural_mapper_max_distance_meters/2);

	//int x_index = round(x_meters/ 0.2) + middle;
    //int y_index = round(y_meters/ 0.2) + middle;
	
	//printf("INDEX: (%d)x(%d)\n", x_index, y_index);
	
    // update number of points
    if (raw_number_of_lasers_map.map[x_index][y_index]==-1)
    	raw_number_of_lasers_map.map[x_index][y_index] = 1;
    else
	    raw_number_of_lasers_map.map[x_index][y_index]++;
    int n = raw_number_of_lasers_map.map[x_index][y_index];

    // update min and max values
	if (n == 1)
	{
		raw_min_hight_map.map[x_index][y_index] = z_meters;
		raw_max_hight_map.map[x_index][y_index] = z_meters;
	}
	else if (z_meters < raw_min_hight_map.map[x_index][y_index])
	    raw_min_hight_map.map[x_index][y_index] = z_meters;
	else if (z_meters > raw_max_hight_map.map[x_index][y_index])
	    raw_max_hight_map.map[x_index][y_index] = z_meters;

	// update mean
	if (n == 1)
		raw_mean_hight_map.map[x_index][y_index] = z_meters;
	else
		raw_mean_hight_map.map[x_index][y_index] = ((raw_mean_hight_map.map[x_index][y_index]*(n-1)) + z_meters)/(n);

	//update variance
	if(n > 0)
	{
		double V1 = raw_square_sum_map.map[x_index][y_index];
		double T2 = raw_mean_hight_map.map[x_index][y_index];
		double V2 = T2*T2/n;

		//printf("%lf %lf\n", standard_deviation_map.map[x_index][y_index], raw_mean_hight_map.map[x_index][y_index]);
		std_map.map[x_index][y_index] = (V1 - V2)/n;

	}

	raw_square_sum_map.map[x_index][y_index] += (z_meters*z_meters);

	// update reflectivity
	if (n == 1)
		raw_reflectivity_map.map[x_index][y_index] = remission;
	else
	{
		double last_meanReflectivity = raw_reflectivity_map.map[x_index][y_index];
		raw_reflectivity_map.map[x_index][y_index] = ((last_meanReflectivity*(n-1))+remission)/n;
	}
}


int update_neural_mapper_input_maps(sensor_data_t * sensor_data, sensor_parameters_t *sensor_params, int thread_id, carmen_map_t *log_odds_snapshot_map)
{

	for (int i = 0; i < sensor_params->vertical_resolution; i++)
	{
		//printf("sensor_x: %lf, sensor_y: %lf, car_x: %lf, car_y: %lf, dif_x: %lf\n", sensor_data->ray_position_in_the_floor[thread_id][i].x, sensor_data->ray_position_in_the_floor[thread_id][i].y, sensor_data->ray_origin_in_the_floor[thread_id][i].x, sensor_data->ray_origin_in_the_floor[thread_id][i].y, round(sensor_data->ray_position_in_the_floor[thread_id][i].x - sensor_data->ray_origin_in_the_floor[thread_id][i].x));
		int center = round(neural_mapper_max_distance_meters/0.2);
		int x_ray_index = round((sensor_data->ray_position_in_the_floor[thread_id][i].x - (sensor_data->robot_pose[sensor_data->point_cloud_index].position.x - x_origin))/log_odds_snapshot_map->config.resolution) + center;
		int y_ray_index = round((sensor_data->ray_position_in_the_floor[thread_id][i].y - (sensor_data->robot_pose[sensor_data->point_cloud_index].position.y - y_origin))/log_odds_snapshot_map->config.resolution) + center;
/*
		int point_x = round((sensor_data->ray_position_in_the_floor[thread_id][i].x/0.2) + center);
		int point_y = round((sensor_data->ray_position_in_the_floor[thread_id][i].y/0.2) + center);
		int x_ray_index = point_x;
		int y_ray_index = point_y;
*/
		//printf("ID: %d | ray_x = %lf | robot_x = %lf| map_x = %lf | dif = %lf | index_x = %d\n", map_index, sensor_data->ray_position_in_the_floor[thread_id][i].x, sensor_data->robot_pose[sensor_data->point_cloud_index].position.x, log_odds_snapshot_map->config.x_origin, (sensor_data->robot_pose[sensor_data->point_cloud_index].position.x - log_odds_snapshot_map->config.x_origin), x_ray_index);
		//printf("ID: %d | ray_y = %lf | robot_y = %lf| map_y = %lf | dif = %lf | index_y = %d\n", map_index, sensor_data->ray_position_in_the_floor[thread_id][i].y, sensor_data->robot_pose[sensor_data->point_cloud_index].position.y, log_odds_snapshot_map->config.y_origin, (sensor_data->robot_pose[sensor_data->point_cloud_index].position.y - log_odds_snapshot_map->config.y_origin), y_ray_index);

		//printf("%d, %d \n", x_ray_index, y_ray_index);

		double remission = 0.0;


		//		printf("X: %.3lf Y: %.3lf \n", x, y);
		//Necessario rotacionar para manter na refencia fixa.
//		sensor_params->pose.orientation
//
		double z = sensor_data->obstacle_height[thread_id][i];

		remission = sensor_data->processed_intensity[thread_id][i];
		double x_meters_distance = abs(sensor_data->ray_position_in_the_floor[thread_id][i].x - sensor_data->ray_origin_in_the_floor[thread_id][i].x);
		double y_meters_distance = abs(sensor_data->ray_position_in_the_floor[thread_id][i].y - sensor_data->ray_origin_in_the_floor[thread_id][i].y);
		double distance_meters = (sqrt(pow(x_meters_distance,2)+pow(y_meters_distance,2)));
		//printf("Distances = %d %d and max = %d\n", x_meters_distance, y_meters_distance, neural_mapper_max_distance_meters);

		if(distance_meters < neural_mapper_max_distance_meters && x_ray_index >= 0 && x_ray_index < int(2*neural_mapper_max_distance_meters/map_config.resolution) && y_ray_index >= 0 && y_ray_index < int(2*neural_mapper_max_distance_meters/map_config.resolution) && z < 5)
		{
			//printf("I AM HERE\n");
			update_neural_mapper_inputs_maps_with_new_point(x_ray_index, y_ray_index, z, remission);
		}
	}

	return 0;
}

int update_neural_mapper_output_map(carmen_map_t offline_map, carmen_map_t label_map, double car_x, double car_y){
	//indices iniciais do offline map (pois)
	//printf("%lf, %lf \n", x_origin, y_origin);

	for (int i = 0; i < label_map.config.y_size; i++)
	{
		for (int j = 0; j < label_map.config.x_size; j++)
		{

			int i_on_mapper = (i + round(car_x/0.2) - round(neural_mapper_max_distance_meters/0.2));
			int j_on_mapper = (j + round(car_y/0.2) - round(neural_mapper_max_distance_meters/0.2));
			if(i_on_mapper < 1050 && i_on_mapper >= 0 && j_on_mapper < 1050 && j_on_mapper >= 0)
			{

				double y_point_meters = (double)j*0.2;
				double x_point_meters = (double)i*0.2;

				double distance_meters = (sqrt(pow((x_point_meters - neural_mapper_max_distance_meters),2)+pow((y_point_meters - neural_mapper_max_distance_meters),2)));
				// labels: unknown = 1; empty = 2; occupied = 3;
				// unknown
				if(offline_map.map[i_on_mapper][j_on_mapper] < 0.0 || distance_meters > neural_mapper_max_distance_meters)
				{
					label_map.map[i][j] = 1;
				}
				// occupied
				else if(offline_map.map[i_on_mapper][j_on_mapper] >= 0.5)
					label_map.map[i][j] = 3; // 3
				// empty
				else
					//printf("Valor de vazio? %lf\n", offline_map.map[i_on_mapper][j_on_mapper]);
					label_map.map[i][j] = 2; // 2
			}
			else
				label_map.map[i][j] = 1;
		}
	}

	return 0;
}

/******************End of generate neural mapper dataset****************************************/


//FILE *plot_data;

static void
update_log_odds_of_cells_in_the_velodyne_perceptual_field(carmen_map_t *log_odds_snapshot_map, sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global, int point_cloud_index, int update_cells_crossed_by_rays,
		int build_snapshot_map __attribute__ ((unused)))
{
	int tid = omp_get_thread_num();
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	int N = v_zt.num_points / sensor_params->vertical_resolution;

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];

	double dt = sensor_params->time_spent_by_each_scan;
	double dt1 = sensor_data->points_timestamp[point_cloud_index] - sensor_data->robot_timestamp[point_cloud_index] - (double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
	int i = 0;

//	plot_data = fopen("plot_data.dat", "w");
	// Ray-trace the grid
	carmen_pose_3D_t robot_pose = sensor_data->robot_pose[point_cloud_index];
//	robot_pose.position.z = 0.0;
//	robot_pose.orientation.pitch = 0.0;
//	robot_pose.orientation.roll = 0.0;

	initialize_virtual_scan_message_update(VELODYNE, robot_pose, robot_interpolated_position, dt * (double) N, v_zt.sphere_points[sensor_params->vertical_resolution * (N - 1)].horizontal_angle,
			v, phi, N, point_cloud_index, r_matrix_robot_to_global, sensor_params, sensor_data);
	for (int j = 0; j < N; j += 1)
	{
		i = j * sensor_params->vertical_resolution;
		double dt2 = j * dt;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_pose,
				dt1 + dt2, v, phi, car_config.distance_between_front_and_rear_axles);
		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[i].horizontal_angle);

		carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data, sensor_params, i, robot_interpolated_position.position,
				sensor_params->sensor_support_pose, r_matrix_robot_to_global, sensor_params->support_to_car_matrix,
				robot_wheel_radius, x_origin, y_origin, &car_config, robot_near_strong_slow_down_annotation, tid, use_remission);

/*******function for update the neural mapper maps-@@@Vinicius *****************/

		if (generate_neural_mapper_dataset && get_next_map) // number_of_data_passed >= neural_mapper_data_pace)
		{
			update_neural_mapper_input_maps(sensor_data, sensor_params, tid, log_odds_snapshot_map);
			//printf("Robot Pose: %lf, %lf , %lf\n", robot_pose_for_neural_mapper.position.x, robot_pose_for_neural_mapper.position.y, robot_pose_for_neural_mapper.position.z);
		}

//		fprintf(plot_data, "%lf %lf %lf",
//				sensor_data->ray_origin_in_the_floor[tid][1].x,
//				sensor_data->ray_origin_in_the_floor[tid][1].y,
//				sensor_data->ray_size_in_the_floor[tid][1]);
//		fprintf(plot_data, "%lf %lf %d",
//				sensor_data->ray_position_in_the_floor[tid][0].x,
//				sensor_data->ray_position_in_the_floor[tid][0].y, 1);
		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, i, highest_sensor, safe_range_above_sensors,
				robot_near_strong_slow_down_annotation, tid);

		build_virtual_scan_message_velodyne(tid, sensor_params, sensor_data, log_odds_snapshot_map);

		if (update_cells_crossed_by_rays == UPDATE_CELLS_CROSSED_BY_RAYS)
		{
			if (create_map_sum_and_count)
				carmen_prob_models_update_sum_and_count_cells_crossed_by_ray(&map, &sum_occupancy_map, &count_occupancy_map, sensor_params, sensor_data, tid);
			else
				carmen_prob_models_update_cells_crossed_by_ray(&map, sensor_params, sensor_data, tid);
		}
		// carmen_prob_models_upgrade_log_odds_of_cells_hit_by_rays(map, sensor_params, sensor_data);
		if (create_map_sum_and_count)
			carmen_prob_models_update_sum_and_count_of_cells_hit_by_rays(&map, &sum_occupancy_map, &count_occupancy_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid);
		else
			carmen_prob_models_update_log_odds_of_cells_hit_by_rays(log_odds_snapshot_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid);

		if (update_and_merge_with_mapper_saved_maps && use_remission)
			carmen_prob_models_update_intensity_of_cells_hit_by_rays(&sum_remission_map, &sum_sqr_remission_map, &count_remission_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, NULL, tid);

		//Lucas: Mapa para deteccao de objetos moveis
		//carmen_prob_models_update_log_odds_of_cells_hit_by_rays(&moving_objects_raw_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors);
		//build_front_laser_message_from_velodyne_point_cloud (sensor_params, sensor_data, v_zt, i);
		//i = i +  sensor_params->vertical_resolution;
//		fprintf(plot_data, "\n");
	}
//	fprintf(plot_data, "\n");
//	fclose(plot_data);
//	system("cp plot_data.dat plot_data2.dat");
}


double
get_acceleration(double v, double timestamp)
{
	static double a = 0.0;
	static double previous_v = 0.0;
	static double previous_timestamp = 0.0;

	if (previous_timestamp == 0.0)
	{
		previous_timestamp = timestamp;
		return (0.0);
	}

	double dt = timestamp - previous_timestamp;
	if (dt < 0.01)
		return (a);

	double current_a = (v - previous_v) / dt;

	a = a + 0.5 * (current_a - a); // para suaviar um pouco

	previous_v = v;
	previous_timestamp = timestamp;

	return (a);
}

//Tem outra variavel com mesmo nome na funcao acima
//FILE *plot_data;

static void
update_log_odds_of_cells_in_the_laser_ldmrs_perceptual_field(carmen_map_t *log_odds_snapshot_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data,
		rotation_matrix *r_matrix_robot_to_global, int point_cloud_index, int update_cells_crossed_by_rays __attribute__ ((unused)),
		int build_snapshot_map __attribute__ ((unused)))
{
	int tid = omp_get_thread_num();
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	int N = v_zt.num_points / sensor_params->vertical_resolution;

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];

	// O codigo abaixo tenta nao registrar o LDMRS no mapa quando de freiadas
	static double blind_timestamp = 0.0;
	double a = get_acceleration(v, sensor_data->robot_timestamp[point_cloud_index]);
	if (a < -sensor_params->cutoff_negative_acceleration)
	{
		blind_timestamp = sensor_data->points_timestamp[point_cloud_index];
		return;
	}
	if ((sensor_data->points_timestamp[point_cloud_index] - blind_timestamp) < 0.5)
		return;

	double dt = sensor_params->time_spent_by_each_scan;
	double dt1 = sensor_data->points_timestamp[point_cloud_index] - sensor_data->robot_timestamp[point_cloud_index] - (double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
	int i = 0;

//	plot_data = fopen("plot_data.dat", "a");
	carmen_pose_3D_t robot_pose = sensor_data->robot_pose[point_cloud_index];
	robot_pose.position.z = 0.0;
	robot_pose.orientation.pitch = 0.0;
	robot_pose.orientation.roll = 0.0;

	initialize_virtual_scan_message_update(LASER_LDMRS, robot_pose, robot_interpolated_position, dt * (double) N, v_zt.sphere_points[sensor_params->vertical_resolution * (N - 1)].horizontal_angle,
			v, phi, N, point_cloud_index, r_matrix_robot_to_global, sensor_params, sensor_data);
	for (int j = 0; j < N; j += 1)
	{
		i = j * sensor_params->vertical_resolution;
		double dt2 = j * dt;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_pose,
				dt1 + dt2, v, phi, car_config.distance_between_front_and_rear_axles);
		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data, sensor_params, i, robot_interpolated_position.position,
				sensor_params->sensor_support_pose, r_matrix_robot_to_global, sensor_params->support_to_car_matrix,
				robot_wheel_radius, x_origin, y_origin, &car_config, robot_near_strong_slow_down_annotation, tid, 0);

//		fprintf(plot_data, "%lf %lf %lf",
//				sensor_data->ray_position_in_the_floor[tid][1].x,
//				sensor_data->ray_position_in_the_floor[tid][1].y,
//				sensor_data->ray_size_in_the_floor[tid][1]);
//		fprintf(plot_data, "%lf %lf %d",
//				sensor_data->ray_position_in_the_floor[tid][0].x,
//				sensor_data->ray_position_in_the_floor[tid][0].y, 1);
//		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, i, highest_sensor, safe_range_above_sensors,
//				robot_near_bump_or_barrier, tid);
//
//		if (update_cells_crossed_by_rays == UPDATE_CELLS_CROSSED_BY_RAYS)
//			carmen_prob_models_update_cells_crossed_by_ray(snapshot_map, sensor_params, sensor_data, tid);

		build_virtual_scan_message_ldmrs(tid, sensor_params, sensor_data, log_odds_snapshot_map);

		carmen_prob_models_update_log_odds_of_cells_hit_by_ldmrs_rays(log_odds_snapshot_map, sensor_params, sensor_data, tid);

//		fprintf(plot_data, "\n");
	}
//	fprintf(plot_data, "dt %lf, dt1 %lf\n", dt, dt1);
//	fclose(plot_data);
//	system("cp plot_data.dat plot_data2.dat");
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
	for (int i = 0; i < current_map->config.x_size * current_map->config.y_size; i++)
	{
		if (current_map->complete_map[i] >= 0.0)
		{
			//current_map->complete_map[i] = (50.0 * current_map->complete_map[i] + offline_map.complete_map[i]) / 51.0;
			current_map->complete_map[i] = (3.0 * current_map->complete_map[i] + offline_map.complete_map[i]) / 4.0;
		}
		else
			current_map->complete_map[i] = offline_map.complete_map[i];
	}
}


int
run_mapper(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	//int N = 4;
	static carmen_map_t *log_odds_snapshot_map;
	static int first = 1;

	if (!globalpos_initialized)
		return (0);

	if (first)
	{
		log_odds_snapshot_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
		first = 0;
	}

#pragma omp parallel num_threads(number_of_threads)
	{
		log_odds_snapshot_map = carmen_prob_models_check_if_new_log_odds_snapshot_map_allocation_is_needed(log_odds_snapshot_map, &map);
		//set_map_equal_offline_map(&map);
		//add_offline_map_over_unknown(&map);

		if (sensor_params->sensor_type == LASER_LDMRS)
		{
			update_log_odds_of_cells_in_the_laser_ldmrs_perceptual_field(log_odds_snapshot_map, sensor_params, sensor_data, r_matrix_robot_to_global, sensor_data->point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS, update_and_merge_with_snapshot_map);
			carmen_prob_models_clear_cells_hit_by_single_ray(log_odds_snapshot_map, sensor_params->log_odds.log_odds_occ,
					sensor_params->log_odds.log_odds_l0);
			carmen_prob_models_overwrite_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(&map, log_odds_snapshot_map,
					sensor_params->log_odds.log_odds_l0);
		}
		else // Velodyne and others
		{
			if (decay_to_offline_map)
				map_decay_to_offline_map(&map);
/****************posicao do carro em relacao ao mapa para neural mapper */
			carmen_pose_3D_t robot_pose_for_neural_mapper = sensor_data->robot_pose[sensor_data->point_cloud_index];

			if (generate_neural_mapper_dataset)
			{
				double x_meters_global = robot_pose_for_neural_mapper.position.x;
				double y_meters_global = robot_pose_for_neural_mapper.position.y;

				x_meters_car_position_according_to_map = x_meters_global- x_origin;
				y_meters_car_position_according_to_map = y_meters_global - y_origin;

				double travelled_distance = sqrt(pow((x_meters_global - last_x_meters_global),2)+pow((y_meters_global - last_y_meters_global),2));
				printf("%lf\n", travelled_distance);
				get_next_map = (travelled_distance > neural_mapper_data_pace);
				if(get_next_map)
				{
					last_x_meters_global = x_meters_global;
					last_y_meters_global = y_meters_global;		
				}
			}			
/**********************************/
			// @@@ Alberto: Mapa padrao Lucas -> colocar DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS ao inves de UPDATE_CELLS_CROSSED_BY_RAYS
			update_log_odds_of_cells_in_the_velodyne_perceptual_field(log_odds_snapshot_map, sensor_params, sensor_data, r_matrix_robot_to_global,
					sensor_data->point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS, update_and_merge_with_snapshot_map);
			carmen_prob_models_update_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(&map, log_odds_snapshot_map,
					sensor_params->log_odds.log_odds_l0);
//			carmen_grid_mapping_save_map((char *) "test.map", &map);

		/****************Neural Mapper********/	
// aqui sensor data ja foi apagado 
 // when points are computed, save_the_maps NEURAL MAPPER:
			if (generate_neural_mapper_dataset && get_next_map)//number_of_data_passed >= neural_mapper_data_pace)
			{
					update_neural_mapper_output_map(offline_map, neural_mapper_occupancy_map, x_meters_car_position_according_to_map, y_meters_car_position_according_to_map);
				    char path[100] = "/dados/neural_mapper_png_dataset/partial_3channels";
				    /*
				    save_map(raw_number_of_lasers_map, (char *) "numb", path, sensor_data->current_timestamp, x_meters_car_position_according_to_map, y_meters_car_position_according_to_map, map_config.resolution, robot_pose_for_neural_mapper.orientation.yaw, false);
				    save_map(raw_mean_hight_map, (char *) "mean", path, sensor_data->current_timestamp, x_meters_car_position_according_to_map, y_meters_car_position_according_to_map, map_config.resolution, robot_pose_for_neural_mapper.orientation.yaw, false);
				    save_map(raw_max_hight_map, (char *) "max", path, sensor_data->current_timestamp, x_meters_car_position_according_to_map, y_meters_car_position_according_to_map, map_config.resolution, robot_pose_for_neural_mapper.orientation.yaw, false);
				    save_map(raw_min_hight_map, (char *) "min", path, sensor_data->current_timestamp, x_meters_car_position_according_to_map, y_meters_car_position_according_to_map, map_config.resolution, robot_pose_for_neural_mapper.orientation.yaw, false);
					save_map(std_map, (char *) "std", path, sensor_data->current_timestamp, x_meters_car_position_according_to_map, y_meters_car_position_according_to_map, map_config.resolution, robot_pose_for_neural_mapper.orientation.yaw, false);
					save_map(neural_mapper_occupancy_map, (char *) "label", path, sensor_data->current_timestamp, x_meters_car_position_according_to_map, y_meters_car_position_according_to_map, map_config.resolution, robot_pose_for_neural_mapper.orientation.yaw, true);
					number_of_data_passed = 0;
					*/
				    save_map(raw_number_of_lasers_map, (char *) "partial_numb", path, false);
				    save_map(raw_mean_hight_map, (char *) "partial_mean", path, false);
				    save_map(raw_max_hight_map, (char *) "partial_max", path, false);
				    save_map(raw_min_hight_map, (char *) "partial_min", path, false);
					save_map(std_map, (char *) "partial_std", path, false);
					save_map(neural_mapper_occupancy_map, (char *) "partial_label", path, true);
					map_index++;		    		
			}
			//number_of_data_passed++;

/*************************************************/
		}
	}

	return (1);
}


void
initialize_first_map_block_origin(carmen_map_t *current_carmen_map, carmen_position_t *map_origin, char map_type)
{
	current_carmen_map->complete_map = NULL;
	x_origin = map_origin->x;
	y_origin = map_origin->y;

	if (update_and_merge_with_mapper_saved_maps)
	{
		carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(map_path, map_type, x_origin, y_origin, current_carmen_map, file_warnings);

		if (current_carmen_map->complete_map == NULL)
			carmen_grid_mapping_initialize_map(current_carmen_map, map.config.x_size, map.config.resolution, map_type);
	}
	else		
		carmen_grid_mapping_get_buffered_map(x_origin, y_origin, current_carmen_map, map_type);
}


void
mapper_change_map_origin_to_another_map_block(carmen_position_t *map_origin)
{
	static int first_time = 1;

	static carmen_map_t new_carmen_map, new_sum_remission_map, new_sum_sqr_remission_map, new_count_remission_map,
								new_sum_occupancy_map, new_mean_occupancy_map, new_count_occupancy_map;

	if (first_time)
	{
		initialize_first_map_block_origin(&map, map_origin, 'm');
		initialize_first_map_block_origin(&moving_objects_raw_map, map_origin, 'm');
		if (use_remission)
		{
			initialize_first_map_block_origin(&sum_remission_map, map_origin, 's');
			initialize_first_map_block_origin(&sum_sqr_remission_map, map_origin, '2');
			initialize_first_map_block_origin(&count_remission_map, map_origin, 'c');
		}

		if (create_map_sum_and_count)
		{
			initialize_first_map_block_origin(&sum_occupancy_map, map_origin, 'u');
			initialize_first_map_block_origin(&mean_occupancy_map, map_origin, 'e');
			initialize_first_map_block_origin(&count_occupancy_map, map_origin, 'o');
		}

		carmen_grid_mapping_create_new_map(&new_carmen_map, map.config.x_size, map.config.y_size, map.config.resolution, 'm');
		if (use_remission)
		{
			carmen_grid_mapping_create_new_map(&new_sum_remission_map, sum_remission_map.config.x_size, sum_remission_map.config.y_size, sum_remission_map.config.resolution, 's');
			carmen_grid_mapping_create_new_map(&new_sum_sqr_remission_map, sum_sqr_remission_map.config.x_size, sum_sqr_remission_map.config.y_size, sum_sqr_remission_map.config.resolution, '2');
			carmen_grid_mapping_create_new_map(&new_count_remission_map, count_remission_map.config.x_size, count_remission_map.config.y_size, count_remission_map.config.resolution, 'c');
		}

		if (create_map_sum_and_count)
		{
			carmen_grid_mapping_create_new_map(&new_sum_occupancy_map, sum_occupancy_map.config.x_size, sum_occupancy_map.config.y_size, sum_occupancy_map.config.resolution, 'u');
			carmen_grid_mapping_create_new_map(&new_mean_occupancy_map, mean_occupancy_map.config.x_size, mean_occupancy_map.config.y_size, mean_occupancy_map.config.resolution, 'e');
			carmen_grid_mapping_create_new_map(&new_count_occupancy_map, count_occupancy_map.config.x_size, count_occupancy_map.config.y_size, count_occupancy_map.config.resolution, 'o');
		}

		first_time = 0;
	}

	if (carmen_grid_mapping_is_map_changed(map_origin, x_origin, y_origin))
	{
		x_origin = map_origin->x;
		y_origin = map_origin->y;

		if (update_and_merge_with_mapper_saved_maps)
		{
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', &map);
			if (use_remission)
			{
				carmen_grid_mapping_save_block_map_by_origin(map_path, 's', &sum_remission_map);
				carmen_grid_mapping_save_block_map_by_origin(map_path, '2', &sum_sqr_remission_map);
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', &count_remission_map);
			}

			if (create_map_sum_and_count)
			{
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'u', &sum_occupancy_map);
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'e', &mean_occupancy_map);
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'o', &count_occupancy_map);
			}

			// get new map with integrated information of the old map
			carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(map_path, 'm', x_origin, y_origin, &new_carmen_map, file_warnings);
			if (use_remission)
			{
				carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(map_path, 's', x_origin, y_origin, &new_sum_remission_map, file_warnings);
				carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(map_path, '2', x_origin, y_origin, &new_sum_sqr_remission_map, file_warnings);
				carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(map_path, 'c', x_origin, y_origin, &new_count_remission_map, file_warnings);
			}

			if (create_map_sum_and_count)
			{
				carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(map_path, 'u', x_origin, y_origin, &new_sum_occupancy_map, file_warnings);
				carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(map_path, 'e', x_origin, y_origin, &new_mean_occupancy_map, file_warnings);
				carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(map_path, 'o', x_origin, y_origin, &new_count_occupancy_map, file_warnings);
			}
		}
		else
		{
			carmen_grid_mapping_update_map_buffer(&map, 'm');
			carmen_grid_mapping_get_buffered_map(x_origin, y_origin, &new_carmen_map, 'm');
		}

		//destroy current map and assign new map to current map
		carmen_grid_mapping_swap_maps_and_clear_old_map(&map, &new_carmen_map);

		if (update_and_merge_with_mapper_saved_maps)
		{
			if (use_remission)
			{
				carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_remission_map, &new_sum_remission_map);
				carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_sqr_remission_map, &new_sum_sqr_remission_map);
				carmen_grid_mapping_swap_maps_and_clear_old_map(&count_remission_map, &new_count_remission_map);
			}

			if (create_map_sum_and_count)
			{
				carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_occupancy_map, &new_sum_occupancy_map);
				carmen_grid_mapping_swap_maps_and_clear_old_map(&mean_occupancy_map, &new_mean_occupancy_map);
				carmen_grid_mapping_swap_maps_and_clear_old_map(&count_occupancy_map, &new_count_occupancy_map);
			}
		}
	}

	moving_objects_raw_map.config.x_origin = x_origin;
	moving_objects_raw_map.config.y_origin = y_origin;

	map.config.x_origin = x_origin;
	map.config.y_origin = y_origin;

	if (use_remission)
	{
		sum_remission_map.config.x_origin = x_origin;
		sum_remission_map.config.y_origin = y_origin;

		sum_sqr_remission_map.config.x_origin = x_origin;
		sum_sqr_remission_map.config.y_origin = y_origin;

		count_remission_map.config.x_origin = x_origin;
		count_remission_map.config.y_origin = y_origin;
	}

	if (create_map_sum_and_count)
	{
		sum_occupancy_map.config.x_origin = x_origin;
		sum_occupancy_map.config.y_origin = y_origin;

		mean_occupancy_map.config.x_origin = x_origin;
		mean_occupancy_map.config.y_origin = y_origin;

		count_occupancy_map.config.x_origin = x_origin;
		count_occupancy_map.config.y_origin = y_origin;
	}
}


int
run_snapshot_mapper()
{	// Esta funcao nao esta funcionando direito pois mistura mapas log odds com mapas probabilisticos...
	int i;
	int current_point_cloud_index;//, before_point_cloud_index;
	static rotation_matrix *r_matrix_robot_to_global = NULL;
	static carmen_map_t *snapshot_map = NULL;
	
	snapshot_map = carmen_prob_models_check_if_new_snapshot_map_allocation_is_needed(snapshot_map, &map);
	
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
		update_log_odds_of_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[0], &sensors_data[0], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
		carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);
	}

	for (i = 1; i < number_of_sensors; i++)
	{
		if (sensors_params[i].alive)
		{
			current_point_cloud_index =  sensors_data[i].point_cloud_index;
			r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[i].robot_pose[current_point_cloud_index].orientation);
			update_log_odds_of_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[i], &sensors_data[i], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
			carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);
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
		intensity[*point_cloud_index] = (unsigned char *) realloc((void *) intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


static void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points, int max_point_buffer,
		int use_remission)
{
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points && use_remission)
		intensity[*point_cloud_index] = (unsigned char *) realloc((void *) intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


int
mapper_velodyne_partial_scan(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message)
{
	static int velodyne_message_id;
	//int ok_to_publish;

	int num_points = velodyne_message->number_of_32_laser_shots * sensors_params[sensor_number].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (sensors_data[sensor_number].last_timestamp == 0.0)
	{
		sensors_data[sensor_number].last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2;		// antigamente eram necessarias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return (ok_to_publish);
	}
	
	sensors_data[sensor_number].current_timestamp = velodyne_message->timestamp;

	build_sensor_point_cloud(&sensors_data[sensor_number].points, sensors_data[sensor_number].intensity,
			&sensors_data[sensor_number].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS, use_remission);

	carmen_velodyne_partial_scan_update_points_with_remission_check(velodyne_message, sensors_params[sensor_number].vertical_resolution,
			&sensors_data[sensor_number].points[sensors_data[sensor_number].point_cloud_index],
			sensors_data[sensor_number].intensity[sensors_data[sensor_number].point_cloud_index],
			sensors_params[sensor_number].ray_order, sensors_params[sensor_number].vertical_correction, sensors_params[sensor_number].range_max,
			velodyne_message->timestamp, use_remission);

	sensors_data[sensor_number].robot_pose[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data[sensor_number].robot_velocity[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data[sensor_number].robot_timestamp[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data[sensor_number].robot_phi[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].phi;
	sensors_data[sensor_number].points_timestamp[sensors_data[sensor_number].point_cloud_index] = velodyne_message->timestamp;

	if (velodyne_message_id >= 0)
	{
		//if (build_snapshot_map)
		ok_to_publish = 1;
		//else

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	sensors_data[sensor_number].last_timestamp = velodyne_message->timestamp;
	
	return (ok_to_publish);
}


int
mapper_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message)
{
	static int message_id;

	int num_points = message->number_of_shots * sensors_params[sensor_number].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (sensors_data[sensor_number].last_timestamp == 0.0)
	{
		sensors_data[sensor_number].last_timestamp = message->timestamp;
		message_id = -2;		// antigamente eram necessarias pelo menos 2 mensagens para ter uma volta completa de velodyne

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
	sensors_data[sensor_number].points_timestamp[sensors_data[sensor_number].point_cloud_index] = message->timestamp;

	if (message_id >= 0)
	{
		ok_to_publish = 1;

		if (message_id > 1000000)
			message_id = 0;
	}
	message_id++;
	sensors_data[sensor_number].last_timestamp = message->timestamp;
	
	return (ok_to_publish);
}


void
mapper_merge_online_map_with_offline_map(carmen_map_t *offline_map)
{
	for (int i = 0; i < (map.config.x_size * map.config.y_size); i++)
		if (map.complete_map[i] < 0.0)
       			map.complete_map[i] = offline_map->complete_map[i];
}


void
add_virtual_laser_points(carmen_map_t *map, carmen_mapper_virtual_laser_message *virtual_laser_message)
{
	for (int i = 0; i < virtual_laser_message->num_positions; i++)
	{
		if (virtual_laser_message->colors[i] == CARMEN_PURPLE)
		{
			int x = round((virtual_laser_message->positions[i].x - map->config.x_origin) / map->config.resolution);
			int y = round((virtual_laser_message->positions[i].y - map->config.y_origin) / map->config.resolution);

			if (x >= 0 && x < map->config.x_size && y >= 0 && y < map->config.y_size)
				map->map[x][y] = 1.0;
		}
	}
//	virtual_laser_message->num_positions = 0;
}


void
draw_line(carmen_map_t *map, double x_1, double y_1, double x_2, double y_2)
{

	int x1 = round((x_1 - map->config.x_origin) / map->config.resolution);
	int y1 = round((y_1 - map->config.y_origin) / map->config.resolution);
	int x2 = round((x_2 - map->config.x_origin) / map->config.resolution);
	int y2 = round((y_2 - map->config.y_origin) / map->config.resolution);

	int w = x2 - x1;
	int h = y2 - y1;
	int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;

	if (w<0)
		dx1 = -1;
	else
		if (w>0)
			dx1 = 1;
	if (h<0)
		dy1 = -1;
	else
		if (h>0)
			dy1 = 1;
	if (w<0)
		dx2 = -1;
	else
		if (w>0)
			dx2 = 1;

	int longest = abs(w) ;
	int shortest = abs(h) ;
	if (!(longest>shortest))
	{
		longest = abs(h) ;
		shortest = abs(w) ;
		if (h<0)
			dy2 = -1;
		else
			if (h>0)
				dy2 = 1;
		dx2 = 0 ;
	}
	int numerator = longest >> 1 ;
	for (int i=0;i<=longest;i++)
	{
		if (x1 >= 0 && x1 < map->config.x_size && y1 >= 0 && y1 < map->config.y_size)
			map->map[x1][y1] = 1.0;

		numerator += shortest ;
		if (!(numerator<longest))
		{
			numerator -= longest ;
			x1 += dx1 ;
			y1 += dy1 ;
		}
		else
		{
			x1 += dx2 ;
			y1 += dy2 ;
		}
	}
}


double
x_coord(double length2, double width2, double theta, double x)
{
	return length2 * cos(theta) - width2 *sin(theta) + x;
}


double
y_coord(double length2, double width2, double theta, double y)
{
	return length2 * sin(theta) + width2 *cos(theta) + y;
}


void
draw_rectangle(carmen_map_t *map, double x, double y, double length, double width, double theta)
{
	carmen_vector_2D_t p1, p2, p3, p4;
	double length2 = length/2.0;
	double width2 = width/2.0;

	p1.x = x_coord(-length2, width2, theta, x);
	p1.y = y_coord(-length2, width2, theta, y);
	p2.x = x_coord(-length2, -width2, theta, x);
	p2.y = y_coord(-length2, -width2, theta, y);
	p3.x = x_coord(length2, -width2, theta, x);
	p3.y = y_coord(length2, -width2, theta, y);
	p4.x = x_coord(length2, width2, theta, x);
	p4.y = y_coord(length2, width2, theta, y);

	draw_line(map, p1.x, p1.y, p2.x, p2.y);
	draw_line(map, p2.x, p2.y, p3.x, p3.y);
	draw_line(map, p3.x, p3.y, p4.x, p4.y);
	draw_line(map, p4.x, p4.y, p1.x, p1.y);

}


void
add_moving_objects(carmen_map_t *map, carmen_moving_objects_point_clouds_message *moving_objects_message)
{
	for (int i = 0; i < moving_objects_message->num_point_clouds; i++)
	{
		if (moving_objects_message->point_clouds[i].point_size)
			draw_rectangle(map,
					moving_objects_message->point_clouds[i].object_pose.x,
					moving_objects_message->point_clouds[i].object_pose.y,
					moving_objects_message->point_clouds[i].length,
					moving_objects_message->point_clouds[i].width,
					carmen_normalize_theta(moving_objects_message->point_clouds[i].orientation));
	}
}


void
mapper_publish_map(double timestamp)
{
	if (build_snapshot_map)
	{
		memcpy(map.complete_map, offline_map.complete_map, offline_map.config.x_size *  offline_map.config.y_size * sizeof(double));
		// memset(map.complete_map, 0, offline_map.config.x_size *  offline_map.config.y_size * sizeof(double));         // Uncomment to see the snapshot_map on viewer 3D, on carmen-ford-scape.ini turn on mapper_build_snapshot_map an turn off mapper_decay_to_offline_map
		run_snapshot_mapper();
	}

	add_virtual_laser_points(&map, &virtual_laser_message);

//	add_moving_objects(&map, &moving_objects_message);

	carmen_mapper_publish_map_message(&map, timestamp);
//	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, timestamp);
//	printf("n = %d\n", virtual_laser_message.num_positions);
}


void
mapper_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR)
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

	map.config.x_origin = x_origin;
	map.config.y_origin = y_origin;

	if (UPDATE_CELLS_BELOW_CAR)
		carmen_prob_models_updade_cells_bellow_robot(globalpos_message->globalpos, &map, 0.0, &car_config);
}


void
mapper_update_grid_map(carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params)
{
	carmen_update_cells_in_the_sensor_perceptual_field(&map, xt, zt, sensor_params);
}


void
mapper_save_current_map()
{
	carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', &map);
	if (use_remission)
	{
		carmen_grid_mapping_save_block_map_by_origin(map_path, 's', &sum_remission_map);
		carmen_grid_mapping_save_block_map_by_origin(map_path, '2', &sum_sqr_remission_map);
		carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', &count_remission_map);
	}

	if (create_map_sum_and_count)
	{
		carmen_grid_mapping_save_block_map_by_origin(map_path, 'u', &sum_occupancy_map);
		carmen_grid_mapping_save_block_map_by_origin(map_path, 'e', &mean_occupancy_map);
		carmen_grid_mapping_save_block_map_by_origin(map_path, 'o', &count_occupancy_map);
	}
}


void
mapper_initialize(carmen_map_config_t *main_map_config, carmen_robot_ackerman_config_t main_car_config)
{
	car_config = main_car_config;
	map_config = *main_map_config;
	
	carmen_grid_mapping_create_new_map(&map, map_config.x_size, map_config.y_size, map_config.resolution, 'm');
	carmen_grid_mapping_create_new_map(&offline_map, map_config.x_size, map_config.y_size, map_config.resolution, 'm');
	if (use_remission)
	{
		carmen_grid_mapping_create_new_map(&sum_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, 's');
		carmen_grid_mapping_create_new_map(&sum_sqr_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, '2');
		carmen_grid_mapping_create_new_map(&count_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, 'c');
	}

	if (create_map_sum_and_count)
	{
		carmen_grid_mapping_create_new_map(&sum_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&mean_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution, 'e');
		carmen_grid_mapping_create_new_map(&count_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution, 'o');
		//	carmen_grid_mapping_create_new_map(&variance_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution);
	}

/*********Initialize maps for neural_mapper_dataset, *****************/
//TODO:check if is correctly

	if (generate_neural_mapper_dataset)
	{
		printf("%d\n", int(2*neural_mapper_max_distance_meters/map_config.resolution));
		carmen_grid_mapping_create_new_map(&raw_mean_hight_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&raw_max_hight_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&raw_min_hight_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&standard_deviation_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&raw_reflectivity_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&raw_number_of_lasers_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&normalized, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&raw_square_sum_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&neural_mapper_occupancy_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(&std_map, int(2*neural_mapper_max_distance_meters/map_config.resolution), int(2*neural_mapper_max_distance_meters/map_config.resolution), map_config.resolution, 'u');

//????
	    int center_pos_x = (int)(neural_mapper_max_distance_meters/2);
	    int center_pos_y = (int)(neural_mapper_max_distance_meters/2);

	    //printf("%lf\n", map_config.resolution);
	    raw_reflectivity_map.config.x_origin = center_pos_x;
	    raw_mean_hight_map.config.x_origin = center_pos_x;
	    raw_max_hight_map.config.x_origin = center_pos_x;
	    raw_min_hight_map.config.x_origin = center_pos_x;
	    raw_number_of_lasers_map.config.x_origin = center_pos_x;
	    standard_deviation_map.config.x_origin = center_pos_x;
	    normalized.config.x_origin = center_pos_x;
		neural_mapper_occupancy_map.config.x_origin = center_pos_x;
   		raw_square_sum_map.config.x_origin = center_pos_x;
   		std_map.config.x_origin = center_pos_x;

	    raw_reflectivity_map.config.y_origin = center_pos_y;
	    raw_mean_hight_map.config.y_origin = center_pos_y;
	    raw_max_hight_map.config.y_origin = center_pos_y;
	    raw_min_hight_map.config.y_origin = center_pos_y;
	    raw_number_of_lasers_map.config.y_origin = center_pos_y;
	    standard_deviation_map.config.y_origin = center_pos_y;
	    normalized.config.y_origin = center_pos_y;
		neural_mapper_occupancy_map.config.y_origin = center_pos_y;
    	raw_square_sum_map.config.y_origin = center_pos_y;
    	std_map.config.y_origin = center_pos_y;

	}

/**************************/

	globalpos_initialized = 0; // Only considered initialized when first message is received

	globalpos_history = (carmen_localize_ackerman_globalpos_message *) calloc(GLOBAL_POS_QUEUE_SIZE, sizeof(carmen_localize_ackerman_globalpos_message));

	memset(&last_rddf_annotation_message, 0, sizeof(last_rddf_annotation_message));

	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
//	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
//	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
//	virtual_laser_message.host = carmen_get_host();

	memset(&virtual_scan_message, 0, sizeof(carmen_mapper_virtual_scan_message));
	virtual_scan_message.host = carmen_get_host();

	last_globalpos = 0;
}
