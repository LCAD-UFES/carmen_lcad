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

	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			if (road_is_center(map, y, x, &next_lane_center) == true)
			{
				printf("X");
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
		printf("\t%do graph nodes: %d\n", i+1, count_graph_nodes(closed_set[i]));
		for(rddf_graph_node *aux = closed_set[i]; aux != NULL; aux = aux->prox)
		{
			//image_graph->at<cv::Vec3b>(map->config.y_size - 1 - y, x) = color;
			p.x = aux->x;
			p.y = aux->y;
			cv::circle(image_graph, p, 0.2,cv::Scalar( blue, green, red ),thickness,lineType);
			//image_graph.at<cv::Vec3b>(aux->x, aux->y) = color;
		}

		cv::imshow("graph in image", image_graph);
		cv::waitKey();
	}

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
	//printf("%dX%d: %hu %hu %hu\n", x,y,cell_prob_ant->lane_center,cell_prob->lane_center,cell_prob_post->lane_center);
	//getchar();


	if(((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y-2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y-1])->lane_center) &&
			(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y+2])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x][y+1])->lane_center)) ||
			((cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-2][y])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-1][y])->lane_center) &&
					(cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x+2][y])->lane_center) && (cell_prob->lane_center > road_mapper_double_to_prob(&map->map[x-2][y])->lane_center)))
	{

		return true;
	}
	else
		return false;


	/*if(cell_prob->lane_center > cell_prob_ant->lane_center && cell_prob->lane_center > cell_prob_post->lane_center)
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
		printf("\topen_set size: %lu", open_set.size());
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
					if (road_is_center(map, x, y, &next_lane_center))
					{
						//printf("\tLane Center at %d X %d\n", x,y);
						//getchar();
						p = (rddf_graph_node*)malloc(sizeof(rddf_graph_node));
						p->x = x;
						p->y = y;
						p->prox = current;
						open_set.push_back(p);
						count++;

					}
				}
			}
		}

		if (count == 0)
			closed_set.push_back(current);
	}
}


void
road_map_find_center(carmen_map_p map)
{
	//printf("road_map_find_center:\n");
	int **already_visited;
	//int **check_matrix;
	rddf_graph_node *p;
	unsigned short next_lane_center=0;
	std::vector<rddf_graph_node*> open_set;
	std::vector<rddf_graph_node*> closed_set;

	already_visited = alloc_matrix(map->config.x_size, map->config.y_size);
	//check_matrix = alloc_matrix(map->config.x_size, map->config.y_size);

	for (int x = 5; x < map->config.x_size-5; x++)
	{
		for (int y = 5; y < map->config.y_size-5; y++)
		{
			if(already_visited[x][y] == 1)
				continue;					// Jump to next y

			already_visited[x][y] = 1;
			//printf("\tPoint at: %d X %d: %hu\n", x,y,next_lane_center);
			if (road_is_center(map, x, y, &next_lane_center))
			{
				printf("\tLane Center at %d X %d: %hu\n", x,y,next_lane_center);
				p = (rddf_graph_node*)malloc(sizeof(rddf_graph_node));
				p->x = x;
				p->y = y;
				p->prox = NULL;

				open_set.push_back(p);


				expand_neighbours(map, open_set, closed_set, already_visited);
				//printf("Closed Set Size: %d\n", closed_set.size());
			}
		}
	}
	printf("Graphs Found: %lu\n", closed_set.size());

	display_graph_in_image(map, closed_set);
}
