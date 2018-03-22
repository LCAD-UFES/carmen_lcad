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
/*void
print_list (rddf_graph_node *l)
{
	printf("Point in list:\n");
	for(rddf_graph_node *aux = l; aux!=NULL; aux = aux->prox)
	{
		printf("\t%d X %d\n", aux->x, aux->y);
	}
}*/

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


/*int
count_graph_nodes(rddf_graph_node *graph)
{
	int count=0;
	for (rddf_graph_node *aux = graph; aux != NULL; aux = aux->prox)
	{
		count++;
	}
	return count;

}*/


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
	/*

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


/*void
display_graph_over_map(carmen_map_p map, vector<rddf_graph_node*> &closed_set, string str_road_map_filename)
{
	cv::Point p;
	cv::Mat image;
	cv::Mat image_graph;
	cv::Vec3b color;
	uchar blue;
	uchar green;
	uchar red;
	string file_name;
	string file_extension = "png";
	int last_bar_position=0;
	unsigned int l;
	for(l=0; l<str_road_map_filename.length();l++)
	{
		if(str_road_map_filename[l] == '/')
			last_bar_position = l;

	}
	file_name = str_road_map_filename.substr(last_bar_position+1,str_road_map_filename.length()-1-l);
	file_name = file_name.substr(0,file_name.length()-3);
	file_name = file_name + file_extension;
	printf("%s\n",file_name.c_str());
	//getchar();
	int thickness = -1;
	int lineType = 8;

	srand (time(NULL));

	image = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar(255,255,255));
	image_graph = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar(255,255,255));
	road_map_to_image(map, &image);
	fade_image(&image);

	for(unsigned int i = 0; i < closed_set.size(); i++)
	{
		blue = rand()%256;
		green = rand()%256;
		red = rand()%256;
		color[0] = blue;
		color[1] = green;
		color[2] = red;
		printf("\t%do graph nodes: %d\n", i+1, count_graph_nodes(closed_set[i]));
		for(rddf_graph_node *aux = closed_set[i]; aux != NULL; aux = aux->prox)
		{
			//image_graph->at<cv::Vec3b>(map->config.y_size - 1 - y, x) = color;
			p.x = aux->x;
			p.y = map->config.y_size - 1 - aux->y;
			//p.x = aux->x;
			//p.y = aux->y;
			cv::circle(image, p, 0.2,cv::Scalar( blue, green, red ),thickness,lineType);
			//image_graph.at<cv::Vec3b>(aux->x, aux->y) = color;
		}

		cv::imshow("graph in image", image);
		//cv::imwrite("test.png", image);
		//cv::waitKey();
	}
	//while((cv::waitKey() & 0xff) != 27);
	//cv::imwrite(file_name, image);
	cv::waitKey();
	/*

		for (int i=0;i<map->config.x_size;i++){
			for (int j=0;j<map->config.y_size;j++){
				printf("%d",check_matrix[i][j]);
			}
			printf("\n");
		}*/
//}


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
point_is_lane_center (carmen_map_p map, int x, int y)
{
	//road_prob *cell_prob_ant2;
	//road_prob *cell_prob_ant1;
	road_prob *cell_prob;
	//road_prob *cell_prob_post1;
	//road_prob *cell_prob_post2;

	//para mapas na vertical, estamos checando o ponto com o y anterior e com o y posterior MELHORAR CHECAGEM!
	//cell_prob_ant2 = road_mapper_double_to_prob(&map->map[x][y-2]);
	//cell_prob_ant1 = road_mapper_double_to_prob(&map->map[x][y-1]);
	cell_prob = road_mapper_double_to_prob(&map->map[x][y]); //pq o mapa está invertido??? PERGUNTAR RAFAEL!
	//cell_prob_post1 = road_mapper_double_to_prob(&map->map[x][y+1]);
	//cell_prob_post2 = road_mapper_double_to_prob(&map->map[x][y+2]);



	if(point_is_in_map(map, x-2,y+2) && point_is_in_map(map, x+2,y-2) && point_is_in_map(map, x-1,y+1) && point_is_in_map(map, x+1,y-1))
	{
		//if(point_is_bigger(map,x,y))
		//{
			if(
					((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y-2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y-1])->lane_center) && //se ponto x,y for menor que os dois y atras
							(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y+2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y+1])->lane_center)//) || //se ponto x,y for menor que os dois y a frente
							//((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-2][y])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-1][y])->lane_center) && //se ponto x,y for menor que os dois x a acima
								//	(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+2][y])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+1][y])->lane_center)) || //se ponto x,y for menor que os dois x a abaixo

									//((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-2][y+2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-1][y+1])->lane_center) &&
										//	(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+2][y-2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+1][y-1])->lane_center)) ||
											//((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-2][y-2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-1][y-1])->lane_center) &&
												//	(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+2][y+2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+1][y+1])->lane_center)
											))
			{

				return true;
			}
			else
				return false;

		//}
		//else
			//return false;

	}
	else
		return false;




	/*if(cell_prob->lane_center > cell_prob_ant2->lane_center && cell_prob->lane_center > cell_prob_ant1->lane_center && cell_prob->lane_center > cell_prob_post1->lane_center && cell_prob->lane_center > cell_prob_post2->lane_center)
	{
		return true;
	}
	else
		return false;*/
}


/*void
print_open_set(std::vector<rddf_graph_node*> &open_set)
{
	printf("Points in open_set:\n");
	for(unsigned int i = 0; i < open_set.size(); i++)
	{
		printf("\t%d\n",i);
		for(rddf_graph_node *aux = open_set[i]; aux != NULL; aux = aux->prox)
			printf("\t\t%d X %d\n", aux->x,aux->y );

		printf("\n");
	}
	getchar();
}*/


bool
point_is_in_map(carmen_map_p map, int x, int y)
{
	if (x >= 0 && x < map->config.x_size && y >= 0 && y < map->config.y_size)
		return (true);
	else
		return (false);
}


bool
point_is_already_visited(int **already_visited, int x, int y)
{
	if(already_visited[x][y] == 1)
		return true;
	else
		return false;
}


/*void
expand_neighbours_of_point (carmen_map_p map, rddf_graph_node *current, vector<rddf_graph_node*> &open_set, int **already_visited, int *count)
{
	rddf_graph_node* next;
	for (int x = current->x - 1; x <= current->x + 1; x++)
	{
		for (int y = current->y - 1; y <= current->y + 1; y++)
		{
			if (point_is_in_map(map, x, y) && !point_is_already_visited(already_visited,x,y)) //só pode processar o ponto caso ele esteja entre 0 e tam_max
			{
				already_visited[x][y] = 1;

				if (point_is_lane_center(map, x, y))
				{
					next = (rddf_graph_node *) malloc(sizeof(rddf_graph_node));
					next->x = x;
					next->y = y;
					next->prox = current;
					open_set.push_back(next);
					(*count) = (*count)+1;
				}
			}
		}
	}
}*/


bool
get_neighbour(carmen_position_t *neighbour, carmen_position_t current, int ** already_visited, carmen_map_p map)
{
	for (int x = current.x - 1; x <= current.x + 1; x++)
	{
		for (int y = current.y - 1; y <= current.y + 1; y++)
		{
			if (point_is_in_map(map, x, y) && !point_is_already_visited(already_visited,x,y)) //só pode processar o ponto caso ele esteja entre 0 e tam_max
			{
				already_visited[x][y] = 1;

				if (point_is_lane_center(map, x, y))
				{
					neighbour->x = x;
					neighbour->y = y;
					return true;
				}
			}
		}
	}
	return false;
}


rddf_graph_t *
add_point_to_graph(rddf_graph_t *graph, int x, int y)
{
	carmen_position_t ponto;
	if (graph == NULL)
	{
		graph = (rddf_graph_t *) malloc(sizeof(rddf_graph_t));
		graph->point = (carmen_position_t *) malloc(sizeof(carmen_position_t));
		graph->point[0].x = x;
		graph->point[0].y = y;

		//graph->edge = (rddf_graph_edges_t *) malloc(sizeof(rddf_graph_edges_t));
		//graph->edge[0].point = NULL;
		//graph->edge[0].size = 0;

		graph->size = 1;
	}
	else
	{
		graph->point = (carmen_position_t *) realloc(graph->point, (graph->size + 1) * sizeof(carmen_position_t));
		graph->point[graph->size].x = x;
		graph->point[graph->size].y = y;
		ponto.x = graph->point[graph->size].x;
		ponto.y = graph->point[graph->size].y;

		//graph->edge = (rddf_graph_edges_t *) realloc(graph->edge, (graph->size + 1) * sizeof(rddf_graph_edges_t));
		//graph->edge[graph->size].point = (int *) realloc(graph->edge[graph->size].point, (graph->edge[graph->size].size + 1) * sizeof(int));
		//graph->edge[graph->size].point[graph->edge[graph->size].size] = graph->size + 1;
		//graph->edge[graph->size].size += 1;

		graph->size += 1;
	}

	return (graph);
}


rddf_graph_t *
A_star(int x, int y, carmen_map_p map, int **already_visited)
{
	vector<carmen_position_t> open_set;
	carmen_position_t current;
	int count = 0;
	int num_iter = 0;

	rddf_graph_t *graph = NULL;
	graph = add_point_to_graph(graph, x, y);

	//current = NULL;
	open_set.push_back(graph->point[0]);

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
				graph = add_point_to_graph(graph, neighbour.x, neighbour.y);
				branch_node = graph->size - 1;
			//}


			//else
				//graph = add_point_to_graph(graph, neighbour.x, neighbour.y, branch_node);
		}
		//printf("%d\n", graph->size);getchar();
		// ARUMAR!!!! expand_neighbours_of_point(map, current, open_set, already_visited, &count);
	}
	printf("Terminei!\n"); getchar();


	return (graph);
}


rddf_graph_t **
add_graph_to_graph_list(rddf_graph_t **rddf_graphs, rddf_graph_t *graph)
{
	if(rddf_graphs = NULL)
	{
		rddf_graphs = (rddf_graph_t **) malloc (sizeof(rddf_graph_t *));
		rddf_graphs[0] = (rddf_graph_t *) malloc ((graph->size) * sizeof(rddf_graph_t));
		rddf_graphs[0] = graph;
		rddf_graphs = 1;
		//preciso arrumar uma forma de controlar o tamanho do rddf_graphs!
	}
	else
	{

	}

	return (rddf_graphs);


}


void
generate_road_map_graph(carmen_map_p map, std::string str_road_map_filename)
{
	int **already_visited;

	rddf_graph_t **rddf_graphs = NULL;

	already_visited = alloc_matrix(map->config.x_size, map->config.y_size);
	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			if (point_is_already_visited(already_visited, x, y))
				continue;
			else
			{
				already_visited[x][y] = 1;
				if (point_is_lane_center(map, x, y))
				{
					rddf_graph_t *graph = A_star(x, y, map, already_visited);
					rddf_graphs = add_graph_to_graph_list(rddf_graphs, graph);
				}
			}
		}
	}

	//display_graph_over_map(map, closed_set, str_road_map_filename);
}
