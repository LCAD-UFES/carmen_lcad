	/*
 * neural_map.cpp
 *
 *  Created on: Nov 29, 2018
 *      Author: vinicius
 */

#include "neural_map.h"
#include <math.h>

// TODO Colocar caminhos fixos parametro global

Neural_map::Neural_map(){}

Neural_map::Neural_map(int size_x, int size_y, double resolution, double car_x, double car_y, double car_rotation, int neural_mapper_max_dist)
{
	this->car_x = car_x;
	this->car_y = car_y;
	this->size_x = size_x;
	this->size_y = size_y;
	this->resolution = resolution;
	this->rotation = car_rotation;
	this->neural_mapper_max_dist = neural_mapper_max_dist;
	printf("TA PEGANDO MEMORIA\n");
	carmen_grid_mapping_create_new_map(&raw_min_hight_map, size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_max_hight_map, size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_mean_hight_map, size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_number_of_lasers_map, size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_square_sum_map, size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&neural_mapper_occupancy_map, size_x, size_y, resolution, 'u');


	carmen_grid_mapping_create_new_map(&normalized_max, size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&normalized_mean, size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&normalized_min,  size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&normalized_numb, size_x, size_y, resolution, 'u');
	carmen_grid_mapping_create_new_map(&normalized_std,  size_x, size_y, resolution, 'u');


	int center_pos_x = (int)(this->neural_mapper_max_dist/2);
	int center_pos_y = (int)(this->neural_mapper_max_dist/2);

	//printf("%lf\n", map_config.resolution);
	this->raw_mean_hight_map.config.x_origin = center_pos_x;
	this->raw_max_hight_map.config.x_origin = center_pos_x;
	this->raw_min_hight_map.config.x_origin = center_pos_x;
	this->raw_number_of_lasers_map.config.x_origin = center_pos_x;
	this->neural_mapper_occupancy_map.config.x_origin = center_pos_x;
	this->raw_square_sum_map.config.x_origin = center_pos_x;

	this->raw_mean_hight_map.config.y_origin = center_pos_y;
	this->raw_max_hight_map.config.y_origin = center_pos_y;
	this->raw_min_hight_map.config.y_origin = center_pos_y;
	this->raw_number_of_lasers_map.config.y_origin = center_pos_y;
	this->neural_mapper_occupancy_map.config.y_origin = center_pos_y;
	this->raw_square_sum_map.config.y_origin = center_pos_y;

}


void
Neural_map::set_car_x(double car_x)
{
	this->car_x = car_x;
}


void
Neural_map::set_car_y(double car_y)
{
	this->car_y = car_y;
}


void
Neural_map::set_car_rotation(double rotation)
{
	this->rotation = rotation;
}


void
Neural_map::clear_maps()
{
	for (int y = 0; y < this->size_y; y++)
	{
		for (int x = 0; x < this->size_x; x++)
		{
			this->raw_min_hight_map.map[x][y]= -1.0;
			this->raw_max_hight_map.map[x][y]= -1.0;
			this->raw_mean_hight_map.map[x][y]= -1.0;
			this->raw_square_sum_map.map[x][y]= -1.0;
			this->raw_number_of_lasers_map.map[x][y]= -1.0;
			this->neural_mapper_occupancy_map.map[x][y]= -1.0;
			this->normalized_max.map[x][y]= -1.0;
			this->normalized_mean.map[x][y]= -1.0;
			this->normalized_min.map[x][y]= -1.0;
			this->normalized_numb.map[x][y]= -1.0;
			this->normalized_std.map[x][y]= -1.0;
		}
	}
}


void
Neural_map::update_maps(Neural_map new_map)
{

	for (int y = 0; y < this->size_y;y++)
	{
		for (int x = 0; x < this->size_x;x++)
		{
			this->raw_min_hight_map.map[x][y] = new_map.raw_min_hight_map.map[x][y];
			this->raw_max_hight_map.map[x][y] = new_map.raw_max_hight_map.map[x][y];
			this->raw_mean_hight_map.map[x][y] = new_map.raw_mean_hight_map.map[x][y];
			this->raw_square_sum_map.map[x][y] = new_map.raw_square_sum_map.map[x][y];
			this->raw_number_of_lasers_map.map[x][y] = new_map.raw_number_of_lasers_map.map[x][y];
			this->neural_mapper_occupancy_map.map[x][y]= new_map.neural_mapper_occupancy_map.map[x][y];
		}
	}
}


//---------------------------Neural_map_queue-----------------------------------------------

Neural_map_queue::Neural_map_queue(int n_maps, int size_x, int size_y, double resolution, int max_dist)
{
	printf("TA CRIANDO FILA DE NOVO\n");
	this->n_maps = n_maps;
	this->size_x = size_x;
	this->size_y = size_y;
	this->resolution = resolution;

	this->neural_maps = (Neural_map *) malloc(n_maps*sizeof(Neural_map));
	for(int i = 0; i < n_maps; i++)
	{
		this->neural_maps[i] = Neural_map(size_x, size_y, resolution, 0., 0., 0.,max_dist);
	}
	this->output_map = Neural_map(size_x, size_y, resolution, 0., 0., 0.,max_dist);
}


// normalize map to 0-255 values
carmen_map_t
Neural_map_queue::fixed_normalize_map(carmen_map_t value_map, double new_max, double last_max, double min)
{
	carmen_map_t normalized = value_map;

	//normalization
	for(int i = 0; i < value_map.config.y_size;i++){
		for(int j = 0; j < value_map.config.x_size;j++){
			double new_val = value_map.map[j][i];
			if(new_val < -1)
			{
				normalized.map[j][i] = -1;
			}
			normalized.map[j][i] = (new_val-min)*(new_max)/(last_max-min);
		}
	}

	//printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
	return normalized;
}


// normalize map to 0-1 values
carmen_map_t
Neural_map_queue::fixed_normalize_map_2(carmen_map_t value_map, double new_max, double last_max, double min)
{
	carmen_map_t normalized = value_map;

	//normalization
	for(int i = 0; i < value_map.config.y_size;i++){
		for(int j = 0; j < value_map.config.x_size;j++){
			double new_val = value_map.map[j][i];
			if(new_val < min)
			{
				normalized.map[j][i] = min;
			}
			normalized.map[j][i] = (new_val-min)*(new_max)/(last_max-min);
		}
	}
	//printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
	return normalized;
}

double
Neural_map_queue::fixed_normalize_cell(double value_map, double new_max, double last_max, double min)
{
	double new_val = value_map;
	if(new_val < min)
		new_val = min;
	double normalized = (new_val-min)*(new_max)/(last_max-min);

	return normalized;
}


void
Neural_map_queue::fixed_normalize_map_all_maps(carmen_map_t *value_map, carmen_map_t *value_map2, carmen_map_t *value_map3, carmen_map_t *value_map4, carmen_map_t *value_map5)
{
	double x_size = output_map.raw_max_hight_map.config.x_size;
	double y_size = output_map.raw_max_hight_map.config.y_size;

	double min = -1.0;
	double new_max = 1.0;

	double last_map_max  = 1.852193;
	double last_map_mean = 1.852193;
	double last_map_min  = 1.852193;
	double last_map_numb = 64;
	double last_map_std  = 15;

	//normalization
	for(int i = 0; i < x_size;i++){
		for(int j = 0; j < y_size;j++){
			value_map->map[i][j] = fixed_normalize_cell(output_map.raw_max_hight_map.map[i][j]  ,       new_max, last_map_max, min);
			value_map2->map[i][j] = fixed_normalize_cell(output_map.raw_mean_hight_map.map[i][j],       new_max, last_map_mean, min);
			value_map3->map[i][j] = fixed_normalize_cell(output_map.raw_min_hight_map.map[i][j]       , new_max, last_map_min , min);
			value_map4->map[i][j] = fixed_normalize_cell(output_map.raw_number_of_lasers_map.map[i][j], new_max, last_map_numb, min);
			value_map5->map[i][j] = fixed_normalize_cell(output_map.raw_square_sum_map.map[i][j],       new_max, last_map_std , min);
		}
	}
}


cv::Mat
Neural_map_queue::convert_to_rgb(carmen_map_t* complete_map, int x_size, int y_size)
{
	cv::Mat neural_map_img = cv::Mat(cv::Size(x_size, y_size), CV_8UC3);

	for (int y = 0; y < y_size; y++)
	{
		for (int x = 0; x < x_size; x++)
		{
			//printf("Meus valores aqui X: %d Y %d:  %lf\n",x,y,complete_map.map[x][y]);
			if (complete_map->map[x][y] == 0.0)//desconhecido-blue
				neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255,120,0);
			else if (complete_map->map[x][y] == 1.0)//Livre-white
				neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255,255,255);
			else if (complete_map->map[x][y] == 2.0)//ocupado
				neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(0,0,0);
		}
	}

	return neural_map_img;
}


cv::Mat
Neural_map_queue::convert_prob_to_rgb(cv::Mat *image_prob, int x_size, int y_size)
{
	cv::Mat neural_map_img = cv::Mat(cv::Size(x_size, y_size), CV_8UC3);
	cv::Vec3d pixel;

	for (int y = 0; y < y_size; y++)
	{
		for (int x = 0; x < x_size; x++)
		{
			pixel = image_prob->at<cv::Vec3d>(x,y);
			if(pixel[0] > pixel[1] && pixel[0] > pixel[2])
				neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255,120,0);
			else if(pixel[1] > pixel[0] && pixel[1] > pixel[2])
				neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255,255,255);
			else
				neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(0,0,0);
		}
	}

	return neural_map_img;
}


cv::Mat
Neural_map_queue::map_to_png2(carmen_map_t complete_map, bool is_label, double map_max, double map_min, bool rgb_map)
{
	carmen_map_t png_map = complete_map;
	if(!is_label)
		png_map = fixed_normalize_map(complete_map, 255., map_max, map_min);

	if (rgb_map)
	{
		cv::Mat neural_map_img = cv::Mat(cv::Size(complete_map.config.x_size, complete_map.config.y_size), CV_8UC3);
		for (int y = 0; y < complete_map.config.y_size; y++)
		{
			for (int x = 0; x < complete_map.config.x_size; x++)
			{
				//				printf("Meus valores aqui X: %d Y %d:  %lf\n",x,y,complete_map.map[x][y]);
				if (complete_map.map[x][y] <= 2.0)//desconhecido-blue
					neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255,120,0);
				else if (complete_map.map[x][y] <= 130.0)//Livre-white
					neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255,255,255);
				else if (complete_map.map[x][y] <= 260.0)//ocupado
					neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(0,0,0);
			}
		}
		return neural_map_img;
	}
	else
	{
		// jeito mais rapido de gravar com Opencv
		cv::Mat png_mat = cv::Mat(complete_map.config.x_size, complete_map.config.y_size, CV_64FC1, *png_map.map);
		return png_mat;
	}
}


//void
//Neural_map_queue::map_to_png3(carmen_map_t complete_map, char* csv_name, double map_max, double map_min)
//{
//
//	carmen_map_t png_map = complete_map;
//	png_map = fixed_normalize_map(complete_map, 255., map_max, map_min);
//
//	char png_file_name[1024];
//	sprintf(png_file_name,"%s.png",csv_name);
//	// jeito mais rapido de gravar com Opencv
//	cv::Mat png_mat = cv::Mat(complete_map.config.x_size, complete_map.config.y_size, CV_64FC1, *png_map.map);
//	cv::Mat png_mat2 = cv::Mat(complete_map.config.x_size, complete_map.config.y_size, CV_8UC1, *png_map.map);
////	cv::imwrite(png_file_name, png_mat);
//	cv::imshow("Window1", png_mat);
//	cv::imshow("Window2", png_mat2);
//	cv::waitKey(0);
//}

//


void
Neural_map_queue::convert_predicted_to_log_ods_snapshot_map(carmen_map_t* log_ods_snapshot, cv::Mat *image_prob, carmen_pose_3D_t *car_position, double x_origin, double y_origin)
{
//	printf("X_size: %d\n", log_ods_snapshot->config.x_size);
	cv::Vec3d pixel;
	int x_size = this->output_map.size_x;
	int y_size = this->output_map.size_y;

//	printf("size Dx Dy: %d %d \n", x_size, y_size);

	// WARNING unused variable
	// int center = log_ods_snapshot->config.x_size/2;
	int velodyne_max_range = this->output_map.neural_mapper_max_dist;
	double map_resolution = this->output_map.resolution;
	int radius = (velodyne_max_range/map_resolution);
	//int new_size = x_size;
	double prob = -1.0;
	int dx = round(car_position->position.x - x_origin) / map_resolution;
	int dy = round(car_position->position.y - y_origin) / map_resolution;
	if (car_position->position.x == 0.0 && car_position->position.y == 0.0)
		printf("Dx Dy: %d %d %lf \n", dx, dy, x_origin);
	else
	{
		//coordenadas do carro no mapa.
//		printf("car_position != 0 Dx Dy: %d %d %lf \n", dx, dy, x_origin);

		//diferenca entre a posicao do carro e o centro do mapa
//			dx -= center;
//			dy -= center;

		for (int i = 0; i < y_size; i++)
		{
			for (int j = 0; j < x_size; j++)
			{
				int y = i-radius;
				int x = j-radius;
				int k = sqrt((x*x) + (y*y));
				if(k > radius)
					continue;

				int l = abs(round(i + (dx - radius)));
				int m = abs(round(j + (dy - radius)));

				// WARNING UNUSED VARIABLE, remove or comment it
				// double angle_point = carmen_normalize_theta(atan2(m - dy, l - dx) - car_position->orientation.yaw);

				if(m < 1050 && m >= 0 && l < 1050 && l >= 0 )//&& fabs(angle_point) < M_PI_2)
				{
					//				printf("L M: %d %d \n", l, m);
					//				printf("I J: %d %d \n", i, j);
					pixel = image_prob->at<cv::Vec3d>(i,j);
					//				printf("logodds: %lf \n", carmen_prob_models_probabilistic_to_log_odds(-1.0));

					if(pixel[0] > pixel[1] && pixel[0] > pixel[2])
						log_ods_snapshot->map[l][m] = carmen_prob_models_probabilistic_to_log_odds(-1.0);
					else if(pixel[1] > pixel[0] && pixel[1] > pixel[2])
					{
						prob = 1.0 - (pixel[1]/(pixel[1]+pixel[2]));
						log_ods_snapshot->map[l][m] = carmen_prob_models_probabilistic_to_log_odds(prob);
					}
					else
					{
						prob =(pixel[2]/(pixel[2]+pixel[1]));
						log_ods_snapshot->map[l][m] = carmen_prob_models_probabilistic_to_log_odds(prob);
					}
				}
			}
		}
	}
}


void
Neural_map_queue::foward_map(carmen_map_t *log_ods_snapshot, int size, carmen_pose_3D_t *car_position, double x_origin, double y_origin)//, char* map_name, char* path, bool is_label, double rotation, double map_max, int map_index)
{
	acumulate_maps();

	int width = size;
	cv::Mat png_mat = cv::Mat(width, width, CV_8UC3);

	acumulate_maps();

	this->fixed_normalize_map_all_maps(&this->output_map.normalized_max,
			                     &this->output_map.normalized_mean,
			                     &this->output_map.normalized_min,
			                     &this->output_map.normalized_numb,
			                     &this->output_map.normalized_std);

	double *result = process_map_neural_mapper(size, &this->output_map.normalized_max ,
													 &this->output_map.normalized_mean,
													 &this->output_map.normalized_min ,
													 &this->output_map.normalized_numb,
													 &this->output_map.normalized_std);

	cv::Mat image_inference = cv::Mat(size, size, CV_64FC3, result);
	convert_predicted_to_log_ods_snapshot_map(log_ods_snapshot, &image_inference, car_position, x_origin, y_origin);

//	copy_complete_map_to_map(&predicted_map, predicted_map.config);
//	image_inference = convert_to_rgb(&predicted_map, size, size);
//	png_mat = convert_prob_to_rgb(&image_inference, size, size);
	//destroy_maps
	this->output_map.clear_maps();

//	cv::imshow("Predicted", png_mat);
//	cv::waitKey(1);

}



void
Neural_map_queue::map_to_png(carmen_map_t complete_map, char* csv_name, bool is_label, double map_max, double map_min, bool rgb_map)
{

	carmen_map_t png_map = complete_map;
	if(!is_label)
		png_map = fixed_normalize_map(complete_map, 255., map_max, map_min);

	char png_file_name[1024];
	sprintf(png_file_name,"%s.png",csv_name);

	if (rgb_map)
	{
		cv::Mat neural_map_img = cv::Mat(cv::Size(complete_map.config.x_size, complete_map.config.y_size), CV_8UC3);
		for (int y = 0; y < complete_map.config.y_size; y++)
		{
			for (int x = 0; x < complete_map.config.x_size; x++)
			{
//				printf("Meus valores aqui X: %d Y %d:  %lf\n",x,y,complete_map.map[x][y]);
				if (complete_map.map[x][y] <= 2.0)//desconhecido-blue
					neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255,120,0);
				else if (complete_map.map[x][y] <= 130.0)//Livre-white
					neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255,255,255);
				else if (complete_map.map[x][y] <= 260.0)//ocupado
					neural_map_img.at<cv::Vec3b>(x, y) = cv::Vec3b(0,0,0);
			}
		}
		cv::imwrite(png_file_name, neural_map_img);
		neural_map_img.release();
	}
	else
	{
	// jeito mais rapido de gravar com Opencv
		cv::Mat png_mat = cv::Mat(complete_map.config.x_size, complete_map.config.y_size, CV_64FC1, *png_map.map);
		cv::imwrite(png_file_name, png_mat);
		png_mat.release();
	}

}


void
Neural_map_queue::save_map_as_compact_map_binary_file(carmen_map_t map, char* map_name, char* path, bool is_label, double rotation, int map_index)
{
	char name[500];
	int map_size = 1;
//	char name_map[8];
	carmen_compact_map_t compact_metrics_map;

	if(!is_label)
		sprintf(name, "%s/data/%d_%d_%lf_%s", path, map_index, map_size, rotation, map_name);
	else
		sprintf(name, "%s/labels/%d_%d_%lf_%s", path, map_index, map_size, rotation, map_name);

	carmen_prob_models_create_compact_map(&compact_metrics_map, &map, -1.0);
	carmen_prob_models_save_compact_map_as_binary_file(&compact_metrics_map, name);
	carmen_prob_models_free_compact_map(&compact_metrics_map);

	//printf("%s\n", name);


//TODO Salvar o ground truth como png para comparar visualmente tambem - Lembrar que o mapa i j = opencv j i
	if(is_label)
	{
		sprintf(name, "%s/labels/%d_%lf_view", path, map_index, rotation);
		map_to_png(map, name, false, 2, 0, true);
	}

}


void
Neural_map_queue::save_map_as_binary_file(carmen_map_t map, char* map_name, char* path, bool is_label,
		double rotation, double map_max __attribute__ ((unused)), int map_index, double current_timestamp, carmen_pose_3D_t neural_mapper_robot_pose)
{
	FILE *map_file;
	char name[1024];
	int map_size = map.config.x_size*map.config.y_size;
//	carmen_map_t png_map = map;
//	double map_min = -1.0;

//	if(!is_label) // A etapa de normalizacao ficarah no posprocessamento
//		png_map = fixed_normalize_map_2(map, 1.0, map_max, map_min);
	//sprintf(name, "%s/%lf_%s_%0.3lf_%0.3lf_%.2lf_%0.6lf", path, timestamp, map_name, x_meters_position_on_map, y_meters_position_on_map, resolution, angle);
	if(!is_label)
		sprintf(name, "%s/data/%d_%d_%lf_%lf_%lf_%lf_%s", path, map_index, map_size, rotation,
				neural_mapper_robot_pose.position.x, neural_mapper_robot_pose.position.y, current_timestamp, map_name);
	else
		sprintf(name, "%s/labels/%d_%d_%lf_%lf_%lf_%lf_%s", path, map_index, map_size, rotation,
				neural_mapper_robot_pose.position.x, neural_mapper_robot_pose.position.y, current_timestamp, map_name);

	map_file= fopen(name, "wb");
	if(map.complete_map != NULL)
		fwrite(map.complete_map, map_size, sizeof(double), map_file);
	else
		printf("não deu \n");
	//printf("%s\n", name);
	//map_to_csv(map, name);
	fclose(map_file);
	//Just to debug-comment for generate the dataset
//	if(!is_label)
//		map_to_png(map, name, is_label, map_max, -1);

//TODO Salvar o ground truth como png para comparar visualmente tambem - Lembrar que o mapa i j = opencv j i
	if(is_label)
	{
		sprintf(name, "%s/labels/%d_%lf_view", path, map_index, rotation);
		map_to_png(map, name, false, 2, 0, true);
	}

//	sprintf(name, "%s/labels/%d_%lf_view", path, map_index, rotation);
//	map_file = fopen(gt, "wb");
//	map_to_png(map, name, false, 3, 1, true);

}


void
Neural_map_queue::save_map_as_png(carmen_map_t map, char* map_name, char* path, bool is_label, double rotation, double map_max, int map_index)
{
	char name[500];
	//sprintf(name, "%s/%lf_%s_%0.3lf_%0.3lf_%.2lf_%0.6lf", path, timestamp, map_name, x_meters_position_on_map, y_meters_position_on_map, resolution, angle);
	if(!is_label)
		sprintf(name, "%s/data/%d_%lf_%s", path, map_index, rotation, map_name);
	else
		sprintf(name, "%s/labels/%d_%lf_%s", path, map_index, rotation,map_name);
	//printf("%s\n", name);
	//map_to_csv(map, name);
	map_to_png(map, name, is_label, map_max, -1);
	if(is_label)
	{
		sprintf(name, "%s/labels/%d_%lf_view", path, map_index, rotation);
		map_to_png(map, name, false, 3, 1, true);
	}
}


void
Neural_map_queue::update(Neural_map new_map, int pos)
{
	this->neural_maps[pos].set_car_x(new_map.car_x);
	this->neural_maps[pos].set_car_y(new_map.car_y);
	this->neural_maps[pos].set_car_rotation(new_map.rotation);
	this->neural_maps[pos].update_maps(new_map);
}


void
Neural_map_queue::push(Neural_map new_map)
{
	for (int i = n_maps - 1; i > 0; i--)
	{
		update(this->neural_maps[i - 1], i);
	}
	update(new_map, 0);
}


void
Neural_map_queue::acumulate_maps()
{
	output_map.car_x = this->neural_maps[0].car_x;
	output_map.car_y = this->neural_maps[0].car_y;
	output_map.rotation =  this->neural_maps[0].rotation;
	double last_x = output_map.car_x;
	double last_y = output_map.car_y;
	double resolution = output_map.resolution;

	for (int y = 0; y < this->size_y; y++)
	{
		for (int x = 0; x < this->size_x; x++)
		{
			for(int i = 0; i < n_maps; i++)
			{
				double old_x = this->neural_maps[i].car_x;
				double old_y = this->neural_maps[i].car_y;
				int x_correction = int((old_x - last_x)/resolution);
				int y_correction = int((old_y - last_y)/resolution);
				int fixed_x = x + x_correction;
				int fixed_y = y + y_correction;


				if(fixed_x >= 0 && fixed_x < output_map.size_x && fixed_y >= 0 && fixed_y < output_map.size_y)
				{
					//printf("(%d %d) x (%d %d) %d %d %lf %lf\n", fixed_x, fixed_y, x, y, x_correction, y_correction, old_x, old_y);

					// numb
					if(this->neural_maps[i].raw_number_of_lasers_map.map[x][y] != -1)
					{
						output_map.raw_number_of_lasers_map.map[fixed_x][fixed_y] += this->neural_maps[i].raw_number_of_lasers_map.map[x][y];
					}
					// min
					if(output_map.raw_min_hight_map.map[fixed_x][fixed_y] == -1 || output_map.raw_min_hight_map.map[fixed_x][fixed_y] > this->neural_maps[i].raw_min_hight_map.map[x][y])
					{
						output_map.raw_min_hight_map.map[fixed_x][fixed_y] = this->neural_maps[i].raw_min_hight_map.map[x][y];
					}

					// max
					if(output_map.raw_max_hight_map.map[fixed_x][fixed_y] < this->neural_maps[i].raw_max_hight_map.map[x][y])
					{
						output_map.raw_max_hight_map.map[fixed_x][fixed_y] = this->neural_maps[i].raw_max_hight_map.map[x][y];
					}
					// mean
					if(this->neural_maps[i].raw_mean_hight_map.map[x][y] != -1.0)
					{
						if(output_map.raw_mean_hight_map.map[fixed_x][fixed_y] == -1.0)
						{
							output_map.raw_mean_hight_map.map[fixed_x][fixed_y] = 0;
						}
						double sum1 = (output_map.raw_mean_hight_map.map[fixed_x][fixed_y]*output_map.raw_number_of_lasers_map.map[fixed_x][fixed_y]);
						double sum2 = (this->neural_maps[i].raw_mean_hight_map.map[x][y]*this->neural_maps[i].raw_number_of_lasers_map.map[x][y]);
						output_map.raw_mean_hight_map.map[fixed_x][fixed_y] = (sum1+sum2)/(output_map.raw_number_of_lasers_map.map[fixed_x][fixed_y]+this->neural_maps[i].raw_number_of_lasers_map.map[x][y]);
					}
					// square sum
					if(output_map.raw_square_sum_map.map[fixed_x][fixed_y] == -1.0)
					{
						output_map.raw_square_sum_map.map[fixed_x][fixed_y] = 0;
					}
					output_map.raw_square_sum_map.map[fixed_x][fixed_y] += this->neural_maps[i].raw_square_sum_map.map[x][y];
					// std
					if(i == (this->n_maps-1))
					{
						if(output_map.raw_number_of_lasers_map.map[fixed_x][fixed_y] > 0)
						{
							double V1 = output_map.raw_square_sum_map.map[fixed_x][fixed_y];
							double T2 = output_map.raw_mean_hight_map.map[fixed_x][fixed_y];
							double V2 = T2*T2/output_map.raw_number_of_lasers_map.map[fixed_x][fixed_y];
							output_map.raw_square_sum_map.map[fixed_x][fixed_y] = (V1 - V2)/output_map.raw_number_of_lasers_map.map[fixed_x][fixed_y];
						}
					}
					/*
						// update reflectivity
						if (n == 1)
							raw_reflectivity_map.map[x_index][y_index] = remission;
						else
						{
							double last_meanReflectivity = raw_reflectivity_map.map[x_index][y_index];
							raw_reflectivity_map.map[x_index][y_index] = ((last_meanReflectivity*(n-1))+remission)/n;
						}
					 */
				}
			}
		}
	}
	//output_map.raw_number_of_lasers_map.map[x][y] = this->neural_maps[0].raw_number_of_lasers_map.map[x][y];
	output_map.neural_mapper_occupancy_map = this->neural_maps[0].neural_mapper_occupancy_map;
}


void
Neural_map_queue::export_png(char* path, int map_index)
{
	acumulate_maps();
	//save_map(*export_map.raw_min_hight_map, (char *) "min", path, false, this->neural_maps[n_maps-1].rotation);
	//save_map(*export_map.raw_max_hight_map, (char *) "max", path, false, this->neural_maps[n_maps-1].rotation);
	//save_map(*export_map.raw_mean_hight_map, (char *) "mean", path, false, this->neural_maps[n_maps-1].rotation);
	//save_map(*export_map.raw_square_sum_map, (char *) "std", path, false, this->neural_maps[n_maps-1].rotation);
	this->save_map_as_png(output_map.raw_number_of_lasers_map, (char *) "numb", path, false, this->neural_maps[0].rotation, 50, map_index);
	this->save_map_as_png(output_map.raw_min_hight_map, (char *) "min", path, false, this->neural_maps[0].rotation, 5, map_index);
	this->save_map_as_png(output_map.raw_max_hight_map, (char *) "max", path, false, this->neural_maps[0].rotation, 5, map_index);
	this->save_map_as_png(output_map.raw_mean_hight_map, (char *) "mean", path, false, this->neural_maps[0].rotation, 5, map_index);
	this->save_map_as_png(output_map.raw_square_sum_map, (char *) "std", path, false, this->neural_maps[0].rotation, 20, map_index);
	this->save_map_as_png(output_map.neural_mapper_occupancy_map, (char *) "label", path, true, this->neural_maps[0].rotation, 0, map_index);
	this->output_map.clear_maps();
}


void
Neural_map_queue::export_as_binary_file(char* path, int map_index, double current_timestamp, carmen_pose_3D_t neural_mapper_robot_pose)
{
	acumulate_maps();
	//save_map(*export_map.raw_min_hight_map, (char *) "min", path, false, this->neural_maps[n_maps-1].rotation);
	//save_map(*export_map.raw_max_hight_map, (char *) "max", path, false, this->neural_maps[n_maps-1].rotation);
	//save_map(*export_map.raw_mean_hight_map, (char *) "mean", path, false, this->neural_maps[n_maps-1].rotation);
	//save_map(*export_map.raw_square_sum_map, (char *) "std", path, false, this->neural_maps[n_maps-1].rotation);
	this->save_map_as_binary_file(output_map.raw_number_of_lasers_map, (char *) "numb", path, false, this->neural_maps[0].rotation, 64,   map_index, current_timestamp, neural_mapper_robot_pose);
	this->save_map_as_binary_file(output_map.raw_min_hight_map, (char *) "min", path, false, this->neural_maps[0].rotation, 1.852193,     map_index, current_timestamp, neural_mapper_robot_pose);
	this->save_map_as_binary_file(output_map.raw_max_hight_map, (char *) "max", path, false, this->neural_maps[0].rotation, 1.852193,     map_index, current_timestamp, neural_mapper_robot_pose);
	this->save_map_as_binary_file(output_map.raw_mean_hight_map, (char *) "mean", path, false, this->neural_maps[0].rotation, 1.852193,   map_index, current_timestamp, neural_mapper_robot_pose);
	this->save_map_as_binary_file(output_map.raw_square_sum_map, (char *) "std", path, false, this->neural_maps[0].rotation, 15,          map_index, current_timestamp, neural_mapper_robot_pose);
	this->save_map_as_binary_file(output_map.neural_mapper_occupancy_map, (char *) "label", path, true, this->neural_maps[0].rotation, 0, map_index, current_timestamp, neural_mapper_robot_pose);
	this->output_map.clear_maps();
}


//
//
//
//void
//Neural_map_queue::predict_map(char* path, int map_index)
//{
//	acumulate_maps();
//
//	this->send_maps(output_map.raw_number_of_lasers_map, output_map.raw_min_hight_map, output_map.raw_max_hight_map, output_map.raw_mean_hight_map,
//			output_map.raw_square_sum_map, output_map.neural_mapper_occupancy_map
//			(char *) "numb", path, false, this->neural_maps[0].rotation, 50, map_index);
//
//	this->output_map.clear_maps();
//
//}


std::vector<cv::Mat>
Neural_map_queue::get_maps()
{
	acumulate_maps();

	std::vector<cv::Mat> statistic_maps;
	statistic_maps.push_back(this->map_to_png2(output_map.raw_number_of_lasers_map,false, 50,-1,false));
	statistic_maps.push_back(this->map_to_png2(output_map.raw_min_hight_map, false, 5, -1, false));
	statistic_maps.push_back(this->map_to_png2(output_map.raw_max_hight_map, false, 5, -1, false));
	statistic_maps.push_back(this->map_to_png2(output_map.raw_mean_hight_map, false, 5, -1, false));
	statistic_maps.push_back(this->map_to_png2(output_map.raw_square_sum_map, false, 20, -1, false));

	return statistic_maps;

}
