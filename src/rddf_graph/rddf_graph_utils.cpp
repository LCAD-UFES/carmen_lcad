#include "rddf_graph_utils.h"


using namespace std;


cv::Mat
rotate(cv::Mat src, cv::Point pt, double angle)
{
    cv::Mat dst;
    cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows), cv::INTER_NEAREST);
    return dst;
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
			//road_map_img->at<cv::Vec3b>(x, y) = color;
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


void
show_road_map(carmen_map_p road_map, int x, int y)
{
	//road_prob *cell_prob;
	cv::namedWindow("road_map", cv::WINDOW_AUTOSIZE);
	cv::Mat image1;
	int thickness = -1;
	int lineType = 8;
	cv::Point p;
	//p.x = road_map->config.x_size/2;
	//p.y = road_map->config.y_size/2;

	p.x = x;
	p.y = road_map->config.y_size - 1 -  y;

	image1 = cv::Mat(road_map->config.y_size, road_map->config.x_size, CV_8UC3, cv::Scalar::all(0));
	road_map_to_image(road_map, &image1);
	cv::circle(image1, p, 5,cv::Scalar( 255, 0, 0 ),thickness,lineType);
	//while((cv::waitKey() & 0xff) != 27);
	cv::imshow("road_map", image1);
	cv::waitKey();
	//cv::destroyWindow("road_map");
}


void
print_map_in_terminal (carmen_map_p map)
{
	road_prob *cell_prob;

	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
			if (point_is_lane_center(map, x, y) == true)
			//if (cell_prob->lane_center != 0)
			{
				printf("%hu ", cell_prob->lane_center);
				//printf("X");
			}
			else
				printf(".");
		}
		printf("\n");
	}

}


/*void
display_graph_in_image(carmen_map_p map, vector<rddf_graph_node*> &closed_set)
{
	cv::Point p;
	cv::Mat image_graph;
	cv::Vec3b color;
	uchar blue;
	uchar green;
	uchar red;
	int thickness = -1;
	int lineType = 8;

	srand (time(NULL));

	image_graph = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar(255,255,255));

	for(unsigned int i = 0; i < closed_set.size(); i++)
	{
		blue = rand()%256;
		green = rand()%256;
		red = rand()%256;
		color[0] = blue;
		color[1] = green;
		color[2] = red;
		//printf("\t%do graph nodes: %d\n", i+1, count_graph_nodes(closed_set[i]));
		for(rddf_graph_node *aux = closed_set[i]; aux != NULL; aux = aux->prox)
		{
			//image_graph->at<cv::Vec3b>(map->config.y_size - 1 - y, x) = color;
			p.x = map->config.y_size - 1 - aux->y;
			p.y = aux->x;

			//p.x = aux->x;
			//p.y = aux->y;
			cv::circle(image_graph, p, 0.2,cv::Scalar( blue, green, red ),thickness,lineType);
			//image_graph.at<cv::Vec3b>(aux->x, aux->y) = color;
		}

		cv::imshow("graph in image", image_graph);
		//cv::waitKey();
	}
	while((cv::waitKey() & 0xff) != 27);
	//cv::waitKey();

		for (int i=0;i<map->config.x_size;i++){
			for (int j=0;j<map->config.y_size;j++){
				printf("%d",check_matrix[i][j]);
			}
			printf("\n");
		}*/
//}


void
fade_image(cv::Mat *road_map_img)
{
	double alpha = 0.6; double beta;
	cv::Mat copy;
	copy = cv::Mat(road_map_img->rows, road_map_img->cols, CV_8UC3, cv::Scalar(255,255,255));
	beta = ( 1.0 - alpha );
	cv::addWeighted( copy, alpha, *road_map_img, beta, 0.0, *road_map_img);
}


void
display_graph_over_map(carmen_map_p map, rddf_graph_t *graph, int** already_visited, string file_name, int begin)
{


	cv::Point p;
	cv::Mat image;
	cv::Mat image_already_visited;
	cv::Vec3b color;
	uchar blue;
	uchar green;
	uchar red;
	string file_extension = "png";
	file_name = file_name + file_extension;
	printf("%s\n",file_name.c_str());
	//getchar();
	int thickness = -1;
	int lineType = 8;

	cv::namedWindow(file_name, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(file_name, 78 + map->config.x_size, 10);
	cv::namedWindow("already_visited", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("already_visited", 78 + 2*map->config.x_size, 10);

	srand (time(NULL));

	image = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar(255,255,255));
	image_already_visited = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar(255,255,255));
	road_map_to_image(map, &image);
	fade_image(&image);

	for(int x = 0; x < map->config.x_size; x++)
	{
		for(int y = 0; y < map->config.y_size; y++)
		{
			if(already_visited[x][y] == 1)
			{
				p.x = x;
				p.y = map->config.y_size - 1 - y;
				cv::circle(image_already_visited, p, 0.4,cv::Scalar( 0, 0, 0 ),thickness,lineType);
			}
		}
	}

	//for(int i = 0; i < rddf_graphs->size; i++)
	//{
		blue = 0;//rand()%256;
		green = 0;//rand()%256;
		red = 0;//rand()%256;
		color[0] = blue;
		color[1] = green;
		color[2] = red;
		//printf("\t%do graph nodes: %d\n", i+1, rddf_graphs[i].graph->size);
		for(int j=begin; j<graph->size;j++)
		{
			p.x = graph->point[j].x;
			p.y = map->config.y_size - 1 - graph->point[j].y;
			//p.x = aux->x;
			//p.y = aux->y;
			cv::circle(image, p, 0.4,cv::Scalar( blue, green, red ),thickness,lineType);
			//cv::imshow(file_name, image);
			//cv::imshow("already_visited", image_already_visited);
			//cv::waitKey(1);
		}

		cv::imshow(file_name, image);
		cv::imshow("already_visited", image_already_visited);
		//cv::imwrite("test.png", image);
		//cv::waitKey();
	//}
	//while((cv::waitKey() & 0xff) != 27);
	//cv::imwrite(file_name, image);
	cv::waitKey();
	cv::destroyWindow(file_name);
	/*

		for (int i=0;i<map->config.x_size;i++){
			for (int j=0;j<map->config.y_size;j++){
				printf("%d",check_matrix[i][j]);
			}
			printf("\n");
		}*/
}


int**
alloc_matrix(int r, int c)
{
	int **matrix;
	matrix = (int **) calloc (r,sizeof(int*));
	if (matrix == NULL)
	{
		printf ("** Error: Unsuficient Memory **alloc_matrix **");
	    return (NULL);
	}

	for (int i = 0; i < r; i++)
	{
		matrix[i] = (int *) calloc (c,sizeof(int));
		if (matrix[i] == NULL)
		{
			printf ("** Error: Unsuficient Memory **alloc_matrix **");
		    return (NULL);
		}
	}
	return matrix;
}


bool
point_is_bigger(carmen_map_p map, int x, int y)
{
	road_prob *cell_prob;
	cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
	int xBigger=x, yBigger=y;
	for (int l = x-1; l < x+1; l++)
	{
		for (int c = y-1; c < y+1; c++)
		{
			if(cell_prob->lane_center<road_mapper_double_to_prob(&map->map[l][c])->lane_center)
			{

				return false;
			}

		}
	}
	return true;
}


bool
already_visited_exists(string full_path)
{
	if( access( full_path.c_str(), F_OK ) != -1 )
		return true;
	else
		return false;
}


int**
open_and_fill_road_map_already_visited(carmen_map_p map, int **already_visited, string parsed_filename)
{
	string already_visited_filename = "already_visited/" + parsed_filename + "txt";
	FILE *f = fopen(already_visited_filename.c_str(), "r");
	int num;

	if(f==NULL)
		printf("Nao consegui abrir o arquivo");
	else
	{
		for(int x = 0; x < map->config.x_size; x++)
		{
			for(int y = 0; y < map->config.y_size; y++)
			{
				if (!fscanf(f, "%d ", &num))
					break;
				already_visited[x][y] = num;
			}
			//printf("\n");
		}
		fclose (f);


		return (already_visited);
	}

}


string
parse_filename(string str_road_map_filename)
{
	int l;
	int last_bar_position;
	string file_name;

	for(l=0; l<str_road_map_filename.length();l++)
	{
		if(str_road_map_filename[l] == '/')
			last_bar_position = l;

	}
	file_name = str_road_map_filename.substr(last_bar_position+1,str_road_map_filename.length()-1-l);
	file_name = file_name.substr(0,file_name.length()-3);

	return (file_name);
}


void parse_world_origin_to_road_map(string road_map_filename)
{
	string x_origin;
	string y_origin;
	string coordinates;
	int last_bar_position=0;
	int last_trace_position=0;
	int last_underline_position=0;
	int last_dot_position=0;
	unsigned int l;
	for(l=1; l<=road_map_filename.length();l++)
	{
		if(road_map_filename[l] == '/')
			last_bar_position = l;
		if(road_map_filename[l] == '.')
			last_dot_position = l;
		if(road_map_filename[l] == '_')
			last_underline_position = l;
		if(road_map_filename[l] == '-')
			last_trace_position = l;
	}
	x_origin = road_map_filename.substr(last_bar_position+2,last_underline_position-last_bar_position-2);
	y_origin = road_map_filename.substr(last_trace_position,last_dot_position-last_trace_position);


	g_x_origin = atof(x_origin.c_str());
	g_y_origin = atof(y_origin.c_str());

}


bool
point_is_lane_center (carmen_map_p map, int x, int y)
{
	road_prob *cell_prob;
	cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
	double center = cell_prob->lane_center;

	/*//0 X 0
	if(x == 0 && y == 0)
	{
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

		if( (center > center_y_plus_2 && center > center_y_plus_1) ||
			(center > center_x_plus_2 && center > center_x_plus_1) )
			return true;
		else
			return false;
	}

	// 0 X 1
		else if(x == 0 && y == 1)
		{
			double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
			double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
			double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
			double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
			double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

			if( (center > center_y_minus_1 && (center > center_y_plus_2 && center > center_y_plus_1)) ||
				(center > center_x_plus_2 && center > center_x_plus_1) )
				return true;
			else
				return false;
		}

	// 0 X 2 ~ 347
	else if(x == 0 && (y >= 2 && y <= 347))
	{
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

		if( ((center > center_y_minus_2 && center > center_y_minus_1) && (center > center_y_plus_2 && center > center_y_plus_1)) && //se ponto x,y for menor que os dois y a frente
			(center > center_x_plus_2 && center > center_x_plus_1) )
			return true;
		else
			return false;
	}

	// 0 X 348
	else if(x == 0 && y == map->config.y_size-2)
	{
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

		if( (center > center_y_plus_1 && (center > center_y_minus_2 && center > center_y_minus_1)) ||
			(center > center_x_plus_2 && center > center_x_plus_1) )
			return true;
		else
			return false;
	}

	//0 X 349
		else if (x == 0 && y == map->config.y_size-1)
		{
			double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
			double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
			double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
			double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

			if( (center > center_y_minus_2 && center > center_y_minus_1) ||
				(center > center_x_plus_2 && center > center_x_plus_1) )
				return true;
			else
				return false;
		}

	// 1 X 0
		else if(x == 1 && y == 0)
		{
			double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
			double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
			double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;
			double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
			double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

			if( ((center > center_y_plus_2 && center > center_y_plus_1)) ||
				(center > center_x_minus_1 && (center > center_x_plus_2 && center > center_x_plus_1)) )
				return true;
			else
				return false;
		}

	// 1 X 1
	else if(x == 1 && y == 1)
	{
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

		if( (center > center_y_minus_1 && (center > center_y_plus_2 && center > center_y_plus_1)) ||
			(center > center_x_minus_1 && (center > center_x_plus_2 && center > center_x_plus_1)) )
			return true;
		else
			return false;
	}

	// 1 X 2 ~ 347
		else if((y >= 2 && y<= map->config.x_size-3) && x == 1)
		{
			double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
			double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
			double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
			double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
			double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
			double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
			double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;
			if(x==1 && y==122){
					printf("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf",center_y_minus_2,center_y_minus_1,center,center_y_plus_1,center_y_plus_2,center_x_plus_1,center_x_plus_2,center_x_minus_1);getchar();
												}

			if( ((center > center_y_minus_2 && center > center_y_minus_1) && (center > center_y_plus_2 && center > center_y_plus_1)) &&
				(center > center_x_minus_1 && (center > center_x_plus_2 && center > center_x_plus_1)) )
			{
				return true;
			}
			else
				return false;
		}

	// 1 X 348
	else if(x == 1 && y == map->config.y_size-2)
	{
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

		if( (center > center_y_plus_1 && (center > center_y_minus_2 && center > center_y_minus_1)) ||
			(center > center_x_minus_1 && (center > center_x_plus_2 && center > center_x_plus_1)) )
			return true;
		else
			return false;
	}

	// 1 X 349
	else if(x == 1 && y == map->config.y_size-1)
	{
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;

		if( ((center > center_y_minus_2 && center > center_y_minus_1)) ||
			(center > center_x_minus_1 && (center > center_x_plus_2 && center > center_x_plus_1)) )
			return true;
		else
			return false;
	}

	// 2 ~ 347 X 0
	else if((x >= 2 && x<= map->config.x_size-3) && y == 0)
	{
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( (center > center_y_plus_2 && center > center_y_plus_1) ||
			((center > center_x_minus_2 && center > center_x_minus_1) && (center > center_x_plus_2 && center > center_x_plus_1)) )
			return true;
		else
			return false;
	}

	// 2 ~ 347 X 1
	else if((x >= 2 && x<= map->config.x_size-3) && y == 1)
	{
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( (center > center_y_minus_1 && (center > center_y_plus_2 && center > center_y_plus_1)) ||
			((center > center_x_minus_2 && center > center_x_minus_1) && (center > center_x_plus_2 && center > center_x_plus_1)) )
			return true;
		else
			return false;
	}

	// 2 ~ 347 X 348
	else if((x >= 2 && x<= map->config.x_size-3) && y == map->config.y_size-2)
	{
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( (center > center_y_plus_1 && (center > center_y_minus_2 && center > center_y_minus_1)) ||
			((center > center_x_minus_2 && center > center_x_minus_1) && (center > center_x_plus_2 && center > center_x_plus_1)) )
			return true;
		else
			return false;
	}

	// 2 ~ 347 X 349
	else if((x >= 2 && x<= map->config.x_size-3) && y == map->config.y_size-1)
	{
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( (center > center_y_minus_2 && center > center_y_minus_1) ||
			((center > center_x_minus_2 && center > center_x_minus_1) && (center > center_x_plus_2 && center > center_x_plus_1)) )
			return true;
		else
			return false;
	}

	// 348 X 0
	else if(x == map->config.x_size-1 && y == 0)
	{
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( ((center > center_y_plus_2 && center > center_y_plus_1)) ||
			(center > center_x_plus_1 && (center > center_x_minus_2 && center > center_x_minus_1)) )
			return true;
		else
			return false;
	}

	// 348 X 1
	else if(x == map->config.x_size-2 && y == 1)
	{
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( (center > center_y_minus_1 && (center > center_y_plus_2 && center > center_y_plus_1)) ||
			(center > center_x_plus_1 && (center > center_x_minus_2 && center > center_x_minus_1)) )
			return true;
		else
			return false;
	}

	// 348 X 2 ~ 347
		else if(x == map->config.x_size-2 && (y >= 2 && y <= 347))
		{
			double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
			double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
			double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
			double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
			double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
			double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
			double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

			if( ((center > center_y_minus_2 && center > center_y_minus_1) && (center > center_y_plus_2 && center > center_y_plus_1)) || //se ponto x,y for menor que os dois y a frente
				((center > center_x_minus_2 && center > center_x_minus_1) && center > center_x_plus_1) )
				return true;
			else
				return false;
		}

	// 348 X 348
		else if(x == map->config.x_size-2 && y == map->config.y_size-2)
		{
			double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
			double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
			double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
			double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
			double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
			double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

			if( (center > center_y_plus_1 && (center > center_y_minus_2 && center > center_y_minus_1)) ||
				(center > center_x_plus_1 && (center > center_x_minus_2 && center > center_x_minus_1)) )
				return true;
			else
				return false;
		}

	// 348 X 349
	else if(x == map->config.x_size-1 && y == 0)
	{
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( ((center > center_y_minus_2 && center > center_y_minus_1)) ||
			(center > center_x_plus_1 && (center > center_x_minus_2 && center > center_x_minus_1)) )
			return true;
		else
			return false;
	}

	//349 X 0
	else if(x == map->config.x_size-1 && y == 0)
	{
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( (center > center_y_plus_2 && center > center_y_plus_1) ||
			(center > center_x_minus_2 && center > center_x_minus_1) )
			return true;
		else
			return false;
	}

	// 349 X 1
		else if(x == map->config.x_size-1 && y == 1)
		{
			double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
			double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
			double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
			double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
			double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

			if( (center > center_y_minus_1 && (center > center_y_plus_2 && center > center_y_plus_1)) ||
				(center > center_x_minus_2 && center > center_x_minus_1) )
				return true;
			else
				return false;
		}

	// 349 X 2 ~ 347
		else if(x == map->config.x_size-1 && (y >= 2 && y <= 347))
		{
			double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
			double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
			double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
			double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
			double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
			double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

			if( ((center > center_y_minus_2 && center > center_y_minus_1) && (center > center_y_plus_2 && center > center_y_plus_1)) || //se ponto x,y for menor que os dois y a frente
				(center > center_x_minus_2 && center > center_x_minus_1) )
				return true;
			else
				return false;
		}

	// 349 X 348
		else if(x == map->config.x_size-1 && y == map->config.y_size-2)
		{
			double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
			double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
			double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
			double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
			double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

			if( (center > center_y_plus_1 && (center > center_y_minus_2 && center > center_y_minus_1)) ||
				(center > center_x_minus_2 && center > center_x_minus_1) )
				return true;
			else
				return false;
		}

	//349 X 349
	else if (x == map->config.x_size-1 && y == map->config.y_size-1)
	{
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if( (center > center_y_minus_2 && center > center_y_minus_1) ||
			(center > center_x_minus_2 && center > center_x_minus_1) )
			return true;
		else
			return false;
	}

	else if((x >= 2 && x <= map->config.x_size-3) && (y >= 2 && x <= map->config.y_size-3))//*/
	//if(point_is_in_map(map, x-2,y+2) && point_is_in_map(map, x+2,y-2) && point_is_in_map(map, x-1,y+1) && point_is_in_map(map, x+1,y-1))
	//{
		double center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y+2])->lane_center;
		double center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y+1])->lane_center;
		double center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y-2])->lane_center;
		double center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y-1])->lane_center;
		double center_x_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y])->lane_center;
		double center_x_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y])->lane_center;
		double center_x_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y])->lane_center;
		double center_x_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y])->lane_center;

		if(	((center > center_y_minus_2 && center > center_y_minus_1) && (center > center_y_plus_2 && center > center_y_plus_1)) || //se ponto x,y for menor que os dois y a frente
			((center > center_x_minus_2 && center > center_x_minus_1) && (center > center_x_plus_2 && center > center_x_plus_1)) )

		{

			return true;
		}
		else
			return false;


	//}
	//else
		//return false;

}


string
get_new_road_map_filename(string str_road_map_filename, int op)
{
	string folder;
	string x_origin;
	string y_origin;
	string extension = ".map";
	stringstream strs;
	double new_value;
	char *err;

	int last_bar_position=0;
	int last_trace_position=0;
	int last_underline_position=0;
	int last_dot_position=0;
	unsigned int l;
	for(l=1; l<=str_road_map_filename.length();l++)
	{
		if(str_road_map_filename[l] == '/')
			last_bar_position = l;
		if(str_road_map_filename[l] == '.')
			last_dot_position = l;
		if(str_road_map_filename[l] == '_')
			last_underline_position = l;
		if(str_road_map_filename[l] == '-')
			last_trace_position = l;
	}
	folder = str_road_map_filename.substr(0,last_bar_position+1);
	x_origin = str_road_map_filename.substr(last_bar_position+2,last_underline_position-last_bar_position-2);
	y_origin = str_road_map_filename.substr(last_trace_position,last_dot_position-last_trace_position);

	if (op == 1)
	{
		new_value = atof(x_origin.c_str()) + 70;
		strs.precision(7);
		strs << new_value;
		str_road_map_filename = folder + "r" + strs.str() + "_" + y_origin + extension;
		return (str_road_map_filename);
	}
	else if (op == 2)
	{
		new_value = atof(y_origin.c_str()) - 70;
		strs.precision(6);
		strs << new_value;
		str_road_map_filename = folder + "r" + x_origin + "_" + strs.str() + extension;
		return (str_road_map_filename);
	}


}


void
save_current_already_visited (int **already_visited, carmen_map_p map, string parsed_filename)
{
	FILE* f;
	string file_extension = "txt";
	string destination = "already_visited/";
	int i, j;

	parsed_filename = destination + parsed_filename + file_extension;
	f = fopen (parsed_filename.c_str(), "w");
	for(i = 0; i < map->config.x_size; i++)
	{
		for(j = 0; j < map->config.y_size; j++)
		{
			fprintf(f, "%d ", already_visited[i][j]);
		}
		fprintf(f,"\n");
	}
	fclose (f);

}


void set_world_origin_to_road_map(carmen_map_p road_map)
{
	road_map->config.x_origin = g_x_origin;
	road_map->config.y_origin = g_y_origin;
}


void
get_next_road_map (carmen_map_p *map, string &str_road_map_filename, int op)
{
	char *road_map_filename;

	if(op == 1 || op == 2)
		str_road_map_filename = get_new_road_map_filename(str_road_map_filename, op);

	road_map_filename = strdup(str_road_map_filename.c_str());
	bool valid_map_on_file = (carmen_map_read_gridmap_chunk(road_map_filename, *map) == 0);
	parse_world_origin_to_road_map(str_road_map_filename);
	set_world_origin_to_road_map(*map);
	if(!valid_map_on_file)
	{
		printf("mapa eh invalido!\n");getchar();
	}

}


void
clear_already_visited (int **already_visited, int c)
{
	for(int i = 0; i < c; i++)
	{
		free (already_visited[i]);
	}

	free (already_visited);
}


bool
point_is_in_map(carmen_map_p map, int x, int y)
{
	if (x >= 0 && x < map->config.x_size && y >= 0 && y < map->config.y_size)
		return (true);
	else
		return (false);
}


bool
point_is_already_visited(carmen_map_p already_visited, int x, int y)
{
	if(already_visited->map[x][y] == 1)
		return true;
	else
		return false;

}


void
get_new_already_visited_map(carmen_map_p already_visited, carmen_map_p map, carmen_point_t pose)
{
	carmen_map_t new_already_visited;
	new_already_visited.complete_map = NULL;
	double x_origin, y_origin;
	carmen_grid_mapping_get_map_origin(&pose, &x_origin, &y_origin);
	get_already_visited_by_origin_x_y("already_visited",'a',x_origin,y_origin,&new_already_visited,map);
	carmen_grid_mapping_switch_maps(already_visited,&new_already_visited);
}


void
get_new_map(carmen_map_p map, carmen_point_t pose)
{
	carmen_map_t new_map;
	new_map.complete_map = NULL;
	int count_maps = carmen_grid_mapping_get_block_map_by_origin(g_road_map_folder,'r',pose,&new_map);
	carmen_grid_mapping_switch_maps(map,&new_map);
}


bool
check_limits_of_central_road_map(int x, int y)
{
	/* Limits:
	 4 * * 3
	 * * * *
	 * * * *
	 1 * * 2
	 */
				    /*x1  y1  x2  y2  x3  y3  x4  y4*/
	int limits[8] = {350,350,700,350,700,700,350,700};
	for(int i=1; i<8; i=i+2)
	{
		if(x == limits[i-1]+1)
			return true;
		if(y == limits[i]+1)
			return true;
	}
	return false;
}


bool
get_neighbour(carmen_position_t *neighbour, carmen_position_t current, carmen_map_p already_visited, carmen_map_p map)
{
	for (int x = current.x - 1; x <= current.x + 1; x++)
	{
		for (int y = current.y - 1; y <= current.y + 1; y++)
		{
			if (point_is_in_map(map, x, y) && !point_is_already_visited(already_visited,x,y)) //só pode processar o ponto caso ele esteja entre 0 e tam_max
			{
				if(check_limits_of_central_road_map(x,y))
				{
					carmen_point_t pose;
					pose.x = (x*0.2)+map->config.x_origin;
					pose.y = (y*0.2)+map->config.y_origin;
					get_new_map(map,pose);
					carmen_grid_mapping_save_block_map_by_origin("already_visited",'a',already_visited);
					get_new_already_visited_map(already_visited,map,pose);
					cout<<x<<"\t"<<y-350<<endl;
					show_road_map(map,x,y-350);
					y -=349;
				}



				already_visited->map[x][y] = 1;
				if (point_is_lane_center(map, x, y))
				{
					//cout<<"x y if lane center "<<x<<"\t"<<y<<endl;getchar();
					neighbour->x = x;
					neighbour->y = y;
					return (true);
				}
			}
		}
	}
	return (false);

}


rddf_graph_t *
add_point_to_graph_branch(rddf_graph_t * graph, int x, int y, int branch_node)
{
	return (graph);
}


rddf_graph_t *
add_point_to_graph(carmen_map_p map, rddf_graph_t *graph, int x, int y)
{
	if (graph == NULL)
	{
		graph = (rddf_graph_t *) malloc(sizeof(rddf_graph_t));
		graph->point = (carmen_position_t *) malloc(sizeof(carmen_position_t));
		graph->point[0].x = x;
		graph->point[0].y = y;

		graph->world_coordinate = (carmen_position_t *) malloc(sizeof(carmen_position_t));
		graph->world_coordinate[0].x = (x * map->config.resolution) + map->config.x_origin;
		graph->world_coordinate[0].y = (y * map->config.resolution) + map->config.y_origin;

		graph->edge = (rddf_graph_edges_t *) malloc(sizeof(rddf_graph_edges_t));
		graph->edge[0].point = (int *) malloc(sizeof(int));
		graph->edge[0].point = NULL;
		graph->edge[0].size = 0;

		graph->size = 1;
	}
	else
	{
		graph->point = (carmen_position_t *) realloc(graph->point, (graph->size + 1) * sizeof(carmen_position_t));
		graph->point[graph->size].x = x;
		graph->point[graph->size].y = y;

		graph->world_coordinate = (carmen_position_t *) realloc(graph->world_coordinate, (graph->size + 1) * sizeof(carmen_position_t));
		graph->world_coordinate[graph->size].x = (x * map->config.resolution) + map->config.x_origin;
		graph->world_coordinate[graph->size].y = (y * map->config.resolution) + map->config.y_origin;

		graph->edge = (rddf_graph_edges_t *) realloc(graph->edge, (graph->size + 1) * sizeof(rddf_graph_edges_t));
		graph->edge[graph->size].point = (int *) malloc (sizeof(int));// realloc(graph->edge[graph->size].point, (graph->edge[graph->size].size + 1) * sizeof(int));
		graph->edge[graph->size].point[0] = graph->size + 1;//graph->edge[graph->size].point[graph->edge[graph->size].size] = graph->size + 1;
		graph->edge[graph->size].size  += 1;

		graph->size += 1;
	}

	return (graph);
}


rddf_graph_t *
A_star(rddf_graph_t *graph, int x, int y, carmen_map_p map, carmen_map_p already_visited)
{
	vector<carmen_position_t> open_set;
	carmen_position_t current;
	int count = 0;
	int num_iter = 0;

	//graph = NULL;
	graph = add_point_to_graph(map,graph, x, y);

	if(graph->size == 1)
		open_set.push_back(graph->point[0]);
	else
		open_set.push_back(graph->point[graph->size-1]);

	while (!open_set.empty())
	{
		current = open_set.back();
		open_set.pop_back();

		carmen_position_t neighbour;
		int number_of_neighbours = 0;
		int branch_node;
		while (get_neighbour(&neighbour, current, already_visited, map))
		{
			open_set.push_back(neighbour);

			//if (number_of_neighbours == 0)
			//{
				//current = open_set.back();
				/*if(neighbour.y == 0){
					cout<<graph->size<<endl;
					cout<<neighbour.x<<"\t"<<neighbour.y<<endl;getchar();
				}*/
				graph = add_point_to_graph(map, graph, neighbour.x, neighbour.y);
				show_road_map(map,neighbour.x,neighbour.y);
				cout<<"center!"<<neighbour.x<<"\t"<<neighbour.y<<endl;
				//display_graph_over_map(map, graph, already_visited, "oi", 0);
				branch_node = graph->size - 1;
			//}


			//else
				//graph = add_point_to_graph_branch(graph, neighbour.x, neighbour.y, branch_node);
		}
	}
	//cout<<graph->size<<endl;getchar();
	return (graph);
}


rddf_graphs_of_map_t *
add_graph_to_graph_list(rddf_graphs_of_map_t * rddf_graphs, rddf_graph_t *graph)
{
	//rddf_graphs.push_back(graph);
	if(rddf_graphs == NULL)
	{
		rddf_graphs = (rddf_graphs_of_map_t *) malloc (sizeof(rddf_graphs_of_map_t));
		rddf_graphs[0].graph = (rddf_graph_t *) malloc (sizeof(rddf_graph_t));
		rddf_graphs[0].graph = graph;
		rddf_graphs->size = 1;
	}
	else
	{
		rddf_graphs = (rddf_graphs_of_map_t *) realloc(rddf_graphs, (rddf_graphs->size + 1) * sizeof(rddf_graphs_of_map_t));
		rddf_graphs[rddf_graphs->size].graph = (rddf_graph_t *) malloc (sizeof(rddf_graph_t));
		rddf_graphs[rddf_graphs->size].graph = graph;
		rddf_graphs->size += 1;

	}

	return (rddf_graphs);
}


static double*
get_cell_pointer(double *map, int cell_number)
{
	switch (cell_number)
	{
	case 0:
		return map;
		break;
	case 1:
		return &map[local_gridmap_count * local_gridmap_count_3];
		break;
	case 2:
		return &map[local_gridmap_count * 2 * local_gridmap_count_3];
		break;
	case 3:
		return &map[local_gridmap_count_3];
		break;
	case 4:
		return &map[local_gridmap_count * local_gridmap_count_3 + local_gridmap_count_3];
		break;
	case 5:
		return &map[local_gridmap_count * 2 * local_gridmap_count_3 + local_gridmap_count_3];
		break;
	case 6:
		return &map[2 * local_gridmap_count_3];
		break;
	case 7:
		return &map[2 * local_gridmap_count_3 + local_gridmap_count * local_gridmap_count_3];
		break;
	case 8:
		return &map[2 * local_gridmap_count_3 + local_gridmap_count * local_gridmap_count_3 * 2];
		break;
	}
	return NULL;
}

//################### Matrix Organization ######################
/*
 Interface
 | 6 | 7 | 8 |
 | 3 | 4 | 5 |
 | 0 | 1 | 2 |

 Memory
 | 0 | 3 | 6 |
 | 1 | 4 | 7 |
 | 2 | 5 | 8 |
 */
//##############################################################


static void
copy_cell_to_map(carmen_map_t *map, double **cell, int cell_number)
{
	int i;

	double *map_cell;
	map_cell = get_cell_pointer(map->complete_map, cell_number);

	for (i = 0; i < local_gridmap_count_3; i++)
	{
		memcpy(map_cell + i * local_gridmap_count, cell[i], local_gridmap_count_3 * sizeof(double));
	}
}


int
get_already_visited_by_origin_x_y(char *map_path, char map_type, double x_origin, double y_origin, carmen_map_t *already_visited, carmen_map_p map)
{
	carmen_map_t unk_already_visited;

	char full_map_path[1024];
	double local_x_origin, local_y_origin;
	int block_already_visited_exists_on_file = 1;
	int count_maps_on_file = 0;
	int i, j, k;
	int block_map_exists_on_file = 1;

	if (already_visited->complete_map == NULL)
	{
		carmen_grid_mapping_initialize_map(already_visited, 210/0.2, 0.2, map_type);
	}
	else
		carmen_grid_mapping_set_unknown_value(already_visited, map_type);

	for (i = 0, k = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++, k++)
		{
			local_x_origin = x_origin + j * (210 / 3.0);
			local_y_origin = y_origin + i * (210 / 3.0);

			sprintf(full_map_path, "%s/%c%d_%d.map", map_path, map_type, (int)local_x_origin, (int)local_y_origin);

			block_map_exists_on_file = carmen_map_read_gridmap_chunk(full_map_path, &unk_already_visited) != -1;

			if (block_map_exists_on_file)
			{
				copy_cell_to_map(already_visited, unk_already_visited.map, k);
				free(unk_already_visited.map);
				free(unk_already_visited.complete_map);
				free(unk_already_visited.config.map_name);
				count_maps_on_file++;
			}
		}
	}

	strcpy(already_visited->config.origin, "from grid_mapping (built from blocks in files)");
	already_visited->config.x_origin = x_origin;
	already_visited->config.y_origin = y_origin;

	return count_maps_on_file > 0;
}


void
generate_road_map_graph(carmen_map_p map, std::string str_road_map_filename)
{
	carmen_map_t already_visited;
	vector <string> road_map_filenames;
	string parsed_filename;
	rddf_graphs_of_map_t *rddf_graphs = NULL;
	rddf_graph_t *graph = NULL;
	int last_x, last_y, last_graph_size;
	bool road_begin = true;

	parsed_filename = parse_filename(str_road_map_filename);
	road_map_filenames.push_back(str_road_map_filename);

	already_visited.complete_map = NULL;
	get_already_visited_by_origin_x_y("already_visited",'a',g_x_origin,g_y_origin,&already_visited,map);

	for (int x = (map->config.x_size/2) - 1; x < map->config.x_size; x++)
	{
		for (int y = (map->config.y_size/2) - 1; y <= map->config.y_size; y++)
		{
			//show_road_map(map,x,y);
			/*if(x==701)
			{
				carmen_map_t new_map;
				new_map.complete_map = NULL;
				carmen_point_t pose;
				pose.x = (x*0.2)+map->config.x_origin;
				pose.y = (y*0.2)+map->config.y_origin;
				int count_maps = carmen_grid_mapping_get_block_map_by_origin(g_road_map_folder,'r',pose,&new_map);
				carmen_grid_mapping_switch_maps(map,&new_map);
				//carmen_grid_mapping_create_new_map(new_map, 210,210,0.2,'r');
				//carmen_grid_mapping_change_blocks_map_by_origin(pose, map, new_map);
				show_road_map(map,x,y);
				x = (map->config.x_size/2) - 1;
				//cout<<new_map->config.x_origin<<"\t"<<new_map->config.y_origin;
				//getchar();
			}*/

			if (point_is_already_visited(&already_visited, x, y))
				continue;

			else
			{
				already_visited.map[x][y] = 1;

				if (point_is_lane_center(map, x, y))
				{
					cout<<"center!"<<x<<"\t"<<y<<endl;
					if (road_begin)
					{
						last_x = x;
						last_y = y;
						road_begin = false;
					}
					if (graph == NULL)
						last_graph_size = 0;
					else
						last_graph_size = graph->size;

					graph = A_star(graph, x, y, map, &already_visited);
					cout<<graph->size<<endl;
					cout<<"graph begin\t"<<graph->point[0].x<<"\t"<<graph->point[0].y<<endl;
					cout<<"graph end\t"<<graph->point[graph->size-1].x<<"\t"<<graph->point[graph->size-1].y<<endl;
					//display_graph_over_map(map, graph, already_visited, parsed_filename, last_graph_size);
					show_road_map(map,x,y);


					//rddf_graphs = add_graph_to_graph_list(rddf_graphs, graph);
				}
			}
		}
	}

	//printf("Graphs in map: %d\n", rddf_graphs.size());
	//display_graph_over_map(map, rddf_graphs, parsed_filename);
}


/*if (graph->point[graph->size-1].x == 2 || graph->point[graph->size-1].x == map->config.x_size-3)
					{
						cout<<"entrei em X"<<endl;
						save_current_already_visited (already_visited, map, parsed_filename);
						get_next_road_map (&map, str_road_map_filename, 1);
						cout << "File " << str_road_map_filename << " being displayed"<< endl;
						road_map_filenames.push_back(str_road_map_filename);
						parsed_filename = parse_filename(str_road_map_filename);
						clear_already_visited (already_visited, map->config.y_size);
						already_visited = alloc_matrix(map->config.x_size, map->config.y_size);
						if (already_visited_exists(parsed_filename))
						{
							already_visited = open_and_fill_road_map_already_visited(map, already_visited, parsed_filename);
						}
						x = 2;
						//y = graph->point[graph->size-1].y;
					}
					else if (graph->point[graph->size-1].y  == 2 || graph->point[graph->size-1].y  == map->config.y_size-3)
					//else if(map->config.y_size - 1 - graph->point[graph->size-1].y  == map->config.y_size-3)
					{
						cout<<"entrei em Y"<<endl;
						save_current_already_visited (already_visited, map, parsed_filename);
						get_next_road_map (&map, str_road_map_filename, 2);
						cout << "File " << str_road_map_filename << " being displayed"<< endl;
						road_map_filenames.push_back(str_road_map_filename);
						parsed_filename = parse_filename(str_road_map_filename);
						clear_already_visited (already_visited, map->config.y_size);
						already_visited = alloc_matrix(map->config.x_size, map->config.y_size);
						if(already_visited_exists(parsed_filename))
						{
							already_visited = open_and_fill_road_map_already_visited(map, already_visited, parsed_filename);
						}
						//x = graph->point[graph->size-1].x;
						y = 2;
						printf("point \t %d X %d\n",x,y);
					}
					else
					{
						save_current_already_visited (already_visited, map, parsed_filename);
						rddf_graphs = add_graph_to_graph_list(rddf_graphs, graph);
						graph = NULL;
						str_road_map_filename = road_map_filenames[0];
						get_next_road_map (&map, str_road_map_filename, 0);
						road_map_filenames.clear();
						parsed_filename = parse_filename(str_road_map_filename);
						clear_already_visited (already_visited, map->config.y_size);
						already_visited = alloc_matrix(map->config.x_size, map->config.y_size);
						already_visited = open_and_fill_road_map_already_visited(map, already_visited, parsed_filename);

						x = last_x;
						y = last_y;
						road_begin = true;

					}*/
