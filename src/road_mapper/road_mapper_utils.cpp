#include "road_mapper_utils.h"

cv::Mat
rotate(cv::Mat src, cv::Point pt, double angle)
{
    cv::Mat dst;
    cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows), cv::INTER_NEAREST);
    return dst;
}

void
remission_map_to_image(carmen_map_p map, cv::Mat *remission_map_img, int channels)
{
	int i = 0, j = 0;
	for (i = 0; i < map->config.x_size; i++)
	{
		for (j = 0; j < map->config.y_size; j++)
		{
			uchar aux = 255 - (uchar) 3.5 * (255.0 * (1.0 - (map->map[i][j] < 0 ? 1 : map->map[i][j])) + 0.5);
//			uchar aux = (uchar) (255.0 * ((map->map[i][j] < 0 ? 0.4 : map->map[i][j])) + 0.5);
			if (channels == 1)
			{
//				remission_map_img->at<uchar>(i, j) = aux;
				remission_map_img->at<uchar>(map->config.y_size - 1 - j, i) = aux;
			}
			else
			{
				cv::Vec3b color = cv::Vec3b(aux, aux, aux);
//				remission_map_img->at<cv::Vec3b>(i, j) = color;
				remission_map_img->at<cv::Vec3b>(map->config.y_size - 1 - j, i) = color;
			}
		}
	}
//	cv::Point pt(remission_map_img->cols/2.0, remission_map_img->rows/2.0);
//	*remission_map_img = rotate(*remission_map_img, pt, 90);
}

void
offline_map_to_image(carmen_map_p map, cv::Mat *offline_map_img, int channels)
{
	unsigned char *image_data = carmen_graphics_convert_to_image(map, 0);

	int i = 0, j = 0;
	for (i = 0; i < map->config.x_size; i++)
	{
		for (j = 0; j < map->config.y_size; j++)
		{
			uchar red   = (uchar) image_data[((i * map->config.y_size + j) * 3)];
			uchar green = (uchar) image_data[((i * map->config.y_size + j) * 3) + 1];
			uchar blue  = (uchar) image_data[((i * map->config.y_size + j) * 3) + 2];
			if (channels == 1)
			{
				offline_map_img->at<uchar>(map->config.y_size - 1 - j, i) = (red + green + blue) / 3;
			}
			else
			{
				cv::Vec3b color = cv::Vec3b(blue, green, red);
				offline_map_img->at<cv::Vec3b>(map->config.y_size - 1 - j, i) = color;
			}
		}
	}
	free(image_data);
}

void
road_map_to_image(carmen_map_p map, cv::Mat *road_map_img)
{
	road_prob *cell_prob;
	cv::Vec3b color;
	uchar blue;
	uchar green;
	uchar red;
	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
			road_mapper_cell_color(cell_prob, &blue, &green, &red);
			color[0] = blue;
			color[1] = green;
			color[2] = red;
//			road_map_img->at<cv::Vec3b>(x, y) = color;
			road_map_img->at<cv::Vec3b>(map->config.y_size - 1 - y, x) = color;
		}
	}
//	cv::Point pt(road_map_img->cols/2.0, road_map_img->rows/2.0);
//	*road_map_img = rotate(*road_map_img, pt, 90);
}

void
road_map_to_image_black_and_white(carmen_map_p map, cv::Mat *road_map_img, const int class_bits)
{
	road_prob *cell_prob;
	uchar intensity;
	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
			road_mapper_cell_black_and_white(cell_prob, &intensity, class_bits);
//			road_map_img->at<uchar>(x, y) = intensity;
			road_map_img->at<uchar>(map->config.y_size - 1 - y, x) = intensity;
		}
	}
//	cv::Point pt(road_map_img->cols/2.0, road_map_img->rows/2.0);
//	*road_map_img = rotate(*road_map_img, pt, 90);
}

int
global_pos_on_map_q4(carmen_point_t global_pos, carmen_map_p *maps, int maps_size)
{
	carmen_map_p map = maps[0];
	double q4_width = (map->config.resolution * map->config.x_size / 3.0);
	double q4_height = (map->config.resolution * map->config.y_size / 3.0);
	double x_origin_q4 = 0.0;
	double y_origin_q4 = 0.0;
	int i;
	for (i = 0; i < maps_size; i++)
	{
		map = maps[i];
		x_origin_q4 = (map->config.x_origin + q4_width);
		y_origin_q4 = (map->config.y_origin + q4_height);

		if (global_pos.x >= x_origin_q4 && global_pos.x <  x_origin_q4 + q4_width &&
				global_pos.y >= y_origin_q4 && global_pos.y < y_origin_q4 + q4_height)
			return i;
	}
	return -1;
}

int
maps_has_same_origin(carmen_map_p map1, carmen_map_p map2)
{
	return (map1->config.x_origin == map2->config.x_origin &&
			map1->config.y_origin == map2->config.y_origin);
}

carmen_map_p
alloc_map_pointer(void)
{
	carmen_map_p map;
	map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	map->config.x_origin = map->config.y_origin = 0.0001;
	map->complete_map = NULL;
	map->map = NULL;
	return map;
}

void
free_map_pointer(carmen_map_p map)
{
	if (map->complete_map) free(map->complete_map);
	if (map->map) free(map->map);
	free(map);
}
