#include <iostream>
#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>

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



// normalize map to 0-255 values
carmen_map_t
fixed_normalize_map(carmen_map_t value_map, double new_max, double last_max, double min)
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


//void
//save_map_as_png(carmen_map_t map, char* map_name, char* path, bool is_label, double rotation, double map_max, int map_index)
//{
//	char name[500];
//	//sprintf(name, "%s/%lf_%s_%0.3lf_%0.3lf_%.2lf_%0.6lf", path, timestamp, map_name, x_meters_position_on_map, y_meters_position_on_map, resolution, angle);
//	if(!is_label)
//		sprintf(name, "%s/data/%d_%lf_%s", path, map_index, rotation, map_name);
//	else
//		sprintf(name, "%s/labels/%d_%lf_%s", path, map_index, rotation,map_name);
//	//printf("%s\n", name);
//	//map_to_csv(map, name);
//	map_to_png(map, name, is_label, map_max, -1);
//	if(is_label)
//	{
//		sprintf(name, "%s/labels/%d_%lf_view", path, map_index, rotation);
//		map_to_png(map, name, false, 3, 1, true);
//	}
//}


void
map_to_png_check(carmen_map_t complete_map, bool is_label, double map_max, double map_min, bool rgb_map)
{

	carmen_map_t png_map = complete_map;
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
		cv::imshow("windowD", neural_map_img);
		neural_map_img.release();
	}
	else
	{
	// jeito mais rapido de gravar com Opencv
		cv::Mat png_mat = cv::Mat(complete_map.config.x_size, complete_map.config.y_size, CV_64FC1, *png_map.map);
		cv::imwrite("/dados/neural_mapper/data_13-08-19/data/1_0.680134_std.png", png_mat);
	}
}

int
main()
{
	FILE *binary_map;
	binary_map = fopen("/dados/neural_mapper/data_13-08-19/data/1_0.680134_std", "rb");
	int size_map = 600*600;
	carmen_map_t map;
	carmen_grid_mapping_create_new_map(&map, 600, 600, 0.2, 'm');

	fread(map.complete_map, size_map, sizeof(double), binary_map);
	map_to_png_check(map, 0, 20.0, -1.0, false);
	fclose(binary_map);

	return 0;

}
