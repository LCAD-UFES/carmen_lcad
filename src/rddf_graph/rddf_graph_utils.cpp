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
check_neighbours(int x, int y, carmen_map_p map, int **already_visited, t_list *list, bool has_point)
{
	road_prob *cell_prob;
	int bigger=0;
	t_point p;
	if(has_point == true){
		if(x!=0)
			{
				for (int i = x-1; i <= x+1; i++)
				{
					for (int j = y-1; j <= y+1; j++)
					{
						already_visited[x][y] = 1;
						cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
						//printf("Lane Center Neighbour at %d X %d: %hu\n", i,j,cell_prob->lane_center);
						if (bigger < cell_prob->lane_center)
						{
							bigger = cell_prob->lane_center;
							p.x = x;
							p.y = y;
							has_point = true;
						}
						else
							has_point = false;
					}
				}
				list = insert_in_list(list, p);
				//print_list(list);
				check_neighbours(x, y, map, already_visited, list, has_point);
			}

			else
			{
				for (int i = x; i <= x+1; i++)
						{
							for (int j = y-1; j <= y+1; j++)
							{
								already_visited[i][j] = 1;
								cell_prob = road_mapper_double_to_prob(&map->map[i][j]);
								//printf("Lane Center Neighbour at %d X %d: %hu\n", i,j,cell_prob->lane_center);
								if (bigger < cell_prob->lane_center)
								{
									bigger = cell_prob->lane_center;
									p.x = i;
									p.y = j;
									has_point = true;
								}
								else
									has_point = false;
							}
						}
				list = insert_in_list(list, p);
				//print_list(list);
				check_neighbours(x, y, map, already_visited, list, has_point);

			}
	}





}*/

void
print_list (rddf_graph_node *l)
{
	printf("Point in list:\n");
	for(rddf_graph_node *aux = l; aux!=NULL; aux = aux->prox)
	{
		printf("\t%d X %d\n", aux->x, aux->y);
	}
}

/*t_list
*insert_in_list (t_list *l, t_point p) //funcão para inserir um ponto na lista
{
	t_list *node = (t_list*) malloc (sizeof(t_list));
	if (node == NULL)
	{
		printf ("** Error: Unsuficient Memory *insert_in_list **");
	    return (NULL);
	}
	node->p = p;
	node->prox = l;
	return node;
}*/


/*t_list*
create_list() //função para criar lista de caminhos
{
	return NULL;
}*/

void
print_closed_set_in_image(carmen_map_p map, vector<rddf_graph_node*> &closed_set)
{

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
is_center (carmen_map_p map, int x, int y, unsigned short *next_lane_center)
{
	road_prob *cell_prob;
	road_prob *cell_prob_ant;
	road_prob *cell_prob_post;

	//para mapas na vertical, estamos checando o ponto com o y anterior e com o y posterior MELHORAR CHECAGEM!
	cell_prob_ant = road_mapper_double_to_prob(&map->map[x][y-1]);
	cell_prob = road_mapper_double_to_prob(&map->map[x][y]); //pq o mapa está invertido??? PERGUNTAR RAFAEL!
	cell_prob_post = road_mapper_double_to_prob(&map->map[x][y+1]);
	//printf("%dX%d: %hu %hu %hu\n", x,y,cell_prob_ant->lane_center,cell_prob->lane_center,cell_prob_post->lane_center);
	//getchar();

	if(cell_prob->lane_center > cell_prob_ant->lane_center && cell_prob->lane_center > cell_prob_post->lane_center)
	{
		*next_lane_center = cell_prob->lane_center;
		return true;
	}
	else
		return false;
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


/*void
road_map_find_center(carmen_map_p map)
{

	int **already_visited;
	t_point p;
	unsigned short next_lane_center=0;
	t_list *list;
	std::vector<t_point> open_set;
	std::vector<t_list> closed_set;
	int waypoints=0;

	list = create_list();

	already_visited = alloc_matrix(map->config.x_size, map->config.x_size);

	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			already_visited[x][y]=1;

			if(is_center(map,x,y,&next_lane_center) == true)
				//if(cell_prob->lane_center!=0)
			{
				waypoints++;
				if(waypoints == 1)//se é igual a 1 significa que ele achou o primeiro centro de pista, logo, já posso inseri-lo na lista
				{
					p.x = x;
					p.y = y;
					list = insert_in_list(list, p);
				}
				open_set.push_back(p);

				//print_list(list);
				while(!open_set.empty())
				{
					//o ponto a ser explorado estará sempre na posicao zero do open_set
					for (int i = open_set[0].x-1; i <= open_set[0].x+1; i++)
					{
						if(i<0) //em caso de dimensao negativa, pula loop
							continue;
						for (int j = open_set[0].y-1; j <= open_set[0].y+1; j++)
						{
							if(j<0) //em caso de dimensao negativa, pula loop
								continue;
							if(already_visited[i][j] == 1)
							{
								continue;
							}

							already_visited[i][j] = 1;
							//printf("Lane Center Neighbour at %d X %d: %hu\n", i,j,cell_prob->lane_center);
							if (is_center(map,i,j,&next_lane_center) == true)
							{
								//printf("Lane Center Neighbour at %d X %d: %hu\n", i,j,next_lane_center);
								p.x = i;
								p.y = j;
								open_set.push_back(p);
								print_open_set(open_set);
							}
						}

					}
					if(open_set.size() == 2)
					{
						list = insert_in_list(list, open_set[1]);
						open_set.erase(open_set.begin());
						print_list(list);
						getchar();
					}
					system("clear");



				}



				//else
				//printf(".");


			}
			//printf("\n");
			//getchar();

		}
		//print_list(list);
		//	cv::Point pt(road_map_img->cols/2.0, road_map_img->rows/2.0);
		//	*road_map_img = rotate(*road_map_img, pt, 90);
	}
}*/

bool
point_is_in_map(carmen_map_p map, int i, int j)
{

	if (i >= 0 && i <= map->config.x_size && j >= 0 && j <= map->config.y_size)
		return (true);
	else
		return (false);
}


void
expand_neighbours(carmen_map_p map, vector<rddf_graph_node*> &open_set, vector<rddf_graph_node*> &closed_set, int **already_visited)
{
	printf("\nexpand_neighbours:\n");
	unsigned short next_lane_center;
	rddf_graph_node* current;
	rddf_graph_node* p;
	int count = 0;

	current = NULL;

	while (!open_set.empty())
	{
		current = open_set.back();
		open_set.pop_back();
		count = 0;
		printf("\tCurrent: %dX%d", current->x, current->y);
		printf("\topen_set size: %d", open_set.size());
		getchar();

		//explorando os pontos vizinhos de current
		for (int x = current->x - 1; x <= current->x + 1; x++)
		{
			for (int y = current->y - 1; y <= current->y + 1; y++)
			{

				if (point_is_in_map(map, x, y) && already_visited[x][y] != 1) //só pode processar o ponto caso ele esteja entre 0 e tam_max
				{
					already_visited[x][y] = 1;
					//printf("\tPoint in Map at %d X %d\n", x,y);
					//getchar();
					if (is_center(map, x, y, &next_lane_center))
					{
						//printf("\tLane Center at %d X %d\n", x,y);
						//getchar();
						p = (rddf_graph_node*)malloc(sizeof(rddf_graph_node));
						p->x = x;
						p->y = y;
						p->prox = current;
						//current = insert_in_list(current, p);
						//printf("\tNext Lane Center at %d X %d: %hu\n", current->x,current->y, next_lane_center);
						//getchar();
						open_set.push_back(p);
						//print_open_set(open_set);

						count++;
						//printf("\tCount: %d\n\n",count);
						//getchar();
					}
				}
			}
		}
		if (count == 0)
			closed_set.push_back(current);
	}
	//getchar();

}

void
road_map_find_center(carmen_map_p map)
{
	printf("road_map_find_center:\n");
	int **already_visited;
	int **check_matrix;
	rddf_graph_node *p;
	unsigned short next_lane_center=0;
	std::vector<rddf_graph_node*> open_set;
	std::vector<rddf_graph_node*> closed_set;

	already_visited = alloc_matrix(map->config.x_size, map->config.y_size);
	check_matrix = alloc_matrix(map->config.x_size, map->config.y_size);

	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			if(already_visited[x][y] == 1)
				continue;					// Jump to next y

			already_visited[x][y] = 1;

			if (is_center(map, x, y, &next_lane_center) == true)
			{
				//printf("\tLane Center at %d X %d: %hu\n", x,y,next_lane_center);
				p = (rddf_graph_node*)malloc(sizeof(rddf_graph_node));
				p->x = x;
				p->y = y;
				p->prox = NULL;

				//l = insert_in_list(l,p);

				open_set.push_back(p);


				expand_neighbours(map, open_set, closed_set, already_visited);
				//printf("Closed Set Size: %d\n", closed_set.size());
				//print_open_set(closed_set);
				/*for (int i=0;i<map->config.x_size;i++){
					for (int j=0;j<map->config.y_size;j++){
						printf("%d",already_visited[i][j]);
					}
					printf("\n");
				}*/
				//printf("Saiu\n");
			}
		}
	}
	//printf("Closed Set Size: %d\n", closed_set.size());
	//print_open_set(closed_set);

	print_closed_set_in_image(map, closed_set);

	for(unsigned int i = 0; i < closed_set.size(); i++)
	{
		//printf("\t%d\n",i);
		for(rddf_graph_node *aux = closed_set[i]; aux != NULL; aux = aux->prox)
			check_matrix[aux->x][aux->y]=1;

	}

	for (int i=0;i<map->config.x_size;i++){
		for (int j=0;j<map->config.y_size;j++){
			printf("%d",check_matrix[i][j]);
		}
		printf("\n");
	}

}
