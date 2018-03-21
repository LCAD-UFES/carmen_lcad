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
print_list (rddf_graph_node *l)
{
	printf("Point in list:\n");
	for(rddf_graph_node *aux = l; aux!=NULL; aux = aux->prox)
	{
		printf("\t%d X %d\n", aux->x, aux->y);
	}
}

void
print_map_in_terminal (carmen_map_p map)
{
	unsigned short next_lane_center;
	road_prob *cell_prob;

	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
			if (road_is_center(map, x, y, &next_lane_center) == true)
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


int
count_graph_nodes(rddf_graph_node *graph)
{
	int count=0;
	for (rddf_graph_node *aux = graph; aux != NULL; aux = aux->prox)
	{
		count++;
	}
	return count;

}


void
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
}


void
fade_image(cv::Mat *road_map_img)
{
	double alpha = 0.8; double beta;
	cv::Mat copy;
	copy = cv::Mat(road_map_img->rows, road_map_img->cols, CV_8UC3, cv::Scalar(255,255,255));
	beta = ( 1.0 - alpha );
	cv::addWeighted( copy, alpha, *road_map_img, beta, 0.0, *road_map_img);
}


void
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
	int l;
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
		//printf("\t%do graph nodes: %d\n", i+1, count_graph_nodes(closed_set[i]));
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
road_is_center (carmen_map_p map, int x, int y, unsigned short *next_lane_center)
{
	road_prob *cell_prob_ant2;
	road_prob *cell_prob_ant1;
	road_prob *cell_prob;
	road_prob *cell_prob_post1;
	road_prob *cell_prob_post2;

	//para mapas na vertical, estamos checando o ponto com o y anterior e com o y posterior MELHORAR CHECAGEM!
	cell_prob_ant2 = road_mapper_double_to_prob(&map->map[x][y-2]);
	cell_prob_ant1 = road_mapper_double_to_prob(&map->map[x][y-1]);
	cell_prob = road_mapper_double_to_prob(&map->map[x][y]); //pq o mapa está invertido??? PERGUNTAR RAFAEL!
	cell_prob_post1 = road_mapper_double_to_prob(&map->map[x][y+1]);
	cell_prob_post2 = road_mapper_double_to_prob(&map->map[x][y+2]);

	if(point_is_in_map(map, x-2,y+2) && point_is_in_map(map, x+2,y-2) && point_is_in_map(map, x-1,y+1) && point_is_in_map(map, x+1,y-1))
	{
		if(
			((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y-2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y-1])->lane_center) && //se ponto x,y for menor que os dois y atras
			(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y+2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y+1])->lane_center)) || //se ponto x,y for menor que os dois y a frente
				((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-2][y])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-1][y])->lane_center) && //se ponto x,y for menor que os dois x a acima
				(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+2][y])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+1][y])->lane_center)) || //se ponto x,y for menor que os dois x a abaixo

				((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-2][y+2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-1][y+1])->lane_center) &&
					(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+2][y-2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+1][y-1])->lane_center)) ||
						((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-2][y-2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-1][y-1])->lane_center) &&
						(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+2][y+2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+1][y+1])->lane_center)
		))
		{

			return true;
		}
		else
			return false;

	}
	else
		return false;




	/*if(cell_prob->lane_center > cell_prob_ant2->lane_center && cell_prob->lane_center > cell_prob_ant1->lane_center && cell_prob->lane_center > cell_prob_post1->lane_center && cell_prob->lane_center > cell_prob_post2->lane_center)
	{
		*next_lane_center = cell_prob->lane_center;
		return true;
	}
	else
		return false;*/
}


void
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
point_is_already_visited(int **already_visited, int x, int y)
{
	if(already_visited[x][y] == 1)
		return true;
	else
		return false;
}


void
expand_neighbours(carmen_map_p map, vector<rddf_graph_node*> &open_set, vector<rddf_graph_node*> &closed_set, int **already_visited, int *called_expand)
{
	(*called_expand) = (*called_expand) +1;
	//printf("\nexpand_neighbours:\n");
	unsigned short next_lane_center;
	rddf_graph_node* current;
	rddf_graph_node* p;
	int count = 0;
	int neighbours_checked = 0;
	int num_iter = 0;
	int zero_matrix[350][350];

	current = NULL;

	while (!open_set.empty())
	{
		current = open_set.back();
		open_set.pop_back();
		//open_set.clear();
		count = 0;
		//printf("\tCurrent: %dX%d", current->x, current->y);
		//printf("\topen_set size: %lu", open_set.size());
		//getchar();

		//explorando os pontos vizinhos de current
		for (int x = current->x - 1; x <= current->x + 1; x++)
		{
			for (int y = current->y - 1; y <= current->y + 1; y++)
			{
				if (point_is_in_map(map, x, y) && !point_is_already_visited(already_visited,x,y)) //só pode processar o ponto caso ele esteja entre 0 e tam_max
				{
					already_visited[x][y] = 1;
					neighbours_checked++;
					//printf("\tPoint in Map at %d X %d\n", x,y);
					//getchar();
					if (road_is_center(map, x, y, &next_lane_center))
					{
						//printf("\tNext road center at \"expand_neighbours\" %d X %d\tCurrent has %d points\n", x,y, count_graph_nodes(current));
						//getchar();
						p = (rddf_graph_node *) malloc(sizeof(rddf_graph_node));
						p->x = x;
						p->y = y;
						p->prox = current;
						open_set.push_back(p);
						count++;

					}
				}
			}
		}
		//printf("Neighbours of point %dX%d checked: %d", current->x, current->y, neighbours_checked);
		//getchar();
		neighbours_checked = 0;

		//printf("List size: %d\n", count_graph_nodes(current));
		//printf("OpenSet size: %d\n", open_set.size());
		/*if (open_set.size()==121)
		{
			for (int x = 0; x < map->config.x_size; x++)
							{
								for (int y = 0; y < map->config.y_size; y++)
								{
									zero_matrix[x][y] = 0;
									//printf("%d", already_visited[x][y]);

								}
								printf("\n");
							}
			for (rddf_graph_node *aux = current; aux!=NULL; aux=aux->prox){
				zero_matrix[aux->x][aux->y]=1;
			}
			for (int x = 0; x < map->config.x_size; x++)
										{
											for (int y = 0; y < map->config.y_size; y++)
											{
												//zero_matrix = 0;
												printf("%d", zero_matrix[x][y]);

											}
											printf("\n");
										}
			//printf("Count Tam: %d \n", count);
						//getchar();

		}*/

		if (count == 0)
		{
			//printf("Count Tam: %d \t List size: %d \t OpenSet size: %d\n", count, count_graph_nodes(current),open_set.size());
			closed_set.push_back(current);
		}
		num_iter++;


	}
	//printf("Num Iter: %d\n",num_iter);
	//getchar();
	/*for (int x = 0; x < map->config.x_size; x++)
				{
					for (int y = 0; y < map->config.y_size; y++)
					{
						printf("%d", already_visited[x][y]);
					}
					printf("\n");
				}
			getchar();*/
	//printf("Closed Set Size: %d\n", closed_set.size());
	//getchar();
}


void
road_map_find_center(carmen_map_p map, std::string str_road_map_filename)
{
	//printf("road_map_find_center:\n");
	int **already_visited;
	//int **check_matrix;
	rddf_graph_node *p;
	unsigned short next_lane_center=0;
	std::vector<rddf_graph_node*> open_set;
	std::vector<rddf_graph_node*> closed_set;
	int called_expand = 0;

	already_visited = alloc_matrix(map->config.x_size, map->config.y_size);
	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			already_visited[x][y] = 0;
		}
	}
	//check_matrix = alloc_matrix(map->config.x_size, map->config.y_size);
	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			if (point_is_already_visited(already_visited, x, y))
			{
				//printf("Ja visitei o %dX%d!\n",x,y);
				continue;					// Jump to next y
			}
			else
			{
				already_visited[x][y] = 1;
				//printf("\tPoint at: %d X %d: %hu\n", x,y,next_lane_center);
				if (road_is_center(map, x, y, &next_lane_center))
				{
					//printf("\tLane Center before expanding at %d X %d: %hu\n", x,y,next_lane_center);
					//getchar();
					p = (rddf_graph_node *) malloc(sizeof(rddf_graph_node));
					p->x = x;
					p->y = y;
					p->prox = NULL;

					open_set.push_back(p);

					expand_neighbours(map, open_set, closed_set, already_visited, &called_expand);
					//printf("Closed Set Size: %d\n", closed_set.size());
				}
			}
		}
	}
	printf("Called Expand: %d\n", called_expand);
	//printf("Graphs Found: %lu\n", closed_set.size());

	display_graph_over_map(map, closed_set, str_road_map_filename);
}
