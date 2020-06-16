#include "rddf_graph_utils.h"

using namespace std;


void
paint_image (int x, int y, cv::Vec3b pixel_color, cv::Mat *image)
{
	image->at<cv::Vec3b>(cv::Point(x, y))[0] = pixel_color[0];
	image->at<cv::Vec3b>(cv::Point(x, y))[1] = pixel_color[1];
	image->at<cv::Vec3b>(cv::Point(x, y))[2] = pixel_color[2];
}


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
get_map_origin(carmen_point_t *global_pose, double *x_origin, double *y_origin)
{
	*x_origin = floor((floor(global_pose->x / (local_gridmap_size / 3.0) ) - 1.0) * (local_gridmap_size / 3.0));
	*y_origin = floor((floor(global_pose->y / (local_gridmap_size / 3.0) ) - 1.0) * (local_gridmap_size / 3.0));
}


graph_t**
add_to_list_undir(graph_t **adjacent_list, int u, int v, rddf_graph_t *graph){
    //printf("%d %d %d\n", u,v,w);
    graph_t *c, *p;
    c = new_node;
    c->vertex = v;
    c->world_coordinate = graph->world_coordinate[v];
    c->prox = NULL;

    if(adjacent_list[u] == NULL){
        adjacent_list[u] = c;
    }
    else{
        p = adjacent_list[u];
        while ( p -> prox != NULL ){
            p = p -> prox;
        }
        p -> prox = c;

    }

    return (adjacent_list);
}


void
print_graph_2 (graph_t **graph)
{
	for (int i = 0; i < 100; i++)
	{
		printf ("[%d]: ", i);
		graph_t *p;
		for(p = graph[i]; p!=NULL; p = p->prox)
		{
			printf ("%d", p->vertex);
			if (p->prox == NULL)
				continue;
			else
				printf(" -> ");

		}
		printf("\n");
	}
}


graph_t **
read_graph_from_file(graph_t **graph, rddf_graph_t *vertexes, FILE *f)
{
	int u, v;

	graph = (graph_t**)malloc((vertexes->size)*sizeof(graph_t*));

	for(int i=0; i<vertexes->size;i++){
		graph[i] = NULL;
	}

	while (fscanf(f, "%d %d", &u, &v) != EOF)
	{
		graph = add_to_list_undir(graph, u, v, vertexes);
		graph = add_to_list_undir(graph, v, u, vertexes);
	}
	//cout<<vertexes->size<<endl;

	return (graph);
}


rddf_graph_t *
read_vertexes_from_file (rddf_graph_t *vertexes, FILE *f)
{
	int number_of_edges;

	vertexes = (rddf_graph_t *) malloc (sizeof(rddf_graph_t));

	fscanf(f, "%d\n", &vertexes->size);
	fscanf(f, "%d\n", &number_of_edges);
	vertexes->world_coordinate = (carmen_position_t *) malloc (vertexes->size * sizeof(carmen_position_t));

	for (int i = 0; i < vertexes->size; i++)
	{
		fscanf(f, "%lf %lf\n", &vertexes->world_coordinate[i].x, &vertexes->world_coordinate[i].y);
	}


	return (vertexes);
}


void
already_visited_to_image(carmen_map_p map, cv::Mat *road_map_img)
{
	cv::Vec3b color;

	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			if (map->map[x][y] == 1)
			{
				color[0] = 0;
				color[1] = 0;
				color[2] = 0;
			}
			else
			{
				color[0] = 255;
				color[1] = 255;
				color[2] = 255;
			}
			//road_map_img->at<cv::Vec3b>(x, y) = color;
			road_map_img->at<cv::Vec3b>(map->config.y_size - 1 - y, x) = color;
		}
	}
//	cv::Point pt(road_map_img->cols/2.0, road_map_img->rows/2.0);
//	*road_map_img = rotate(*road_map_img, pt, 90);
}


void
draw_lines (carmen_map_p map, cv::Mat *image)
{

	cv::Point line_1_begin;
	line_1_begin.x = 0;
	line_1_begin.y = 350;
	cv::Point line_1_end;
	line_1_end.x = map->config.x_size-1;
	line_1_end.y = 350;

	cv::Point line_2_begin;
	line_2_begin.x = 0;
	line_2_begin.y = 700;
	cv::Point line_2_end;
	line_2_end.x = map->config.x_size-1;
	line_2_end.y = 700;

	cv::Point line_3_begin;
	line_3_begin.x = 350;
	line_3_begin.y = 0;
	cv::Point line_3_end;
	line_3_end.x = 350;
	line_3_end.y = map->config.y_size-1;

	cv::Point line_4_begin;
	line_4_begin.x = 700;
	line_4_begin.y = 0;
	cv::Point line_4_end;
	line_4_end.x = 700;
	line_4_end.y = map->config.y_size-1;

	cv::line(*image,line_1_begin,line_1_end,cv::Scalar(255, 0, 0));
	cv::line(*image,line_2_begin,line_2_end,cv::Scalar(255, 0, 0));
	cv::line(*image,line_3_begin,line_3_end,cv::Scalar(255, 0, 0));
	cv::line(*image,line_4_begin,line_4_end,cv::Scalar(255, 0, 0));

}


void
show_road_map(carmen_map_p map, int x, int y)
{
	//road_prob *cell_prob;
	cv::namedWindow("road_map", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("road_map", map->config.x_size / 2, 10);
	cv::Mat image1;
	int thickness = -1;
	int lineType = 8;
	cv::Point p;

	//p.x = road_map->config.x_size/2;
	//p.y = road_map->config.y_size/2;
	cv::Size size(map->config.x_size * 0.5, map->config.y_size * 0.5);

	p.x = x / 2;
	p.y = (map->config.y_size - 1 - y) / 2;

	image1 = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar::all(0));
	road_map_to_image(map, &image1);
	draw_lines(map, &image1);
	cv::resize(image1, image1, size);
	cv::circle(image1, p, 2.5, cv::Scalar(255, 0, 0), thickness, lineType);

	//while((cv::waitKey() & 0xff) != 27);
	cv::imshow("road_map", image1);
	cv::waitKey(1);
	//cv::destroyWindow("road_map");
}


void
show_already_visited(carmen_map_p map)
{
	//road_prob *cell_prob;
	cv::namedWindow("already_visited", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("already_visited", (map->config.x_size / 2) * 2.05, 10);
	cv::Mat image1;
	//int thickness = -1;
	//int lineType = 8;
	cv::Size size(map->config.x_size * 0.5, map->config.y_size * 0.5);

	image1 = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar::all(0));
	already_visited_to_image(map, &image1);
	draw_lines(map, &image1);
	cv::resize(image1, image1, size);
	//while((cv::waitKey() & 0xff) != 27);
	cv::imshow("already_visited", image1);
	cv::waitKey(1);
	//cv::destroyWindow("road_map");
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
	double alpha = 0.6;
	double beta;
	cv::Mat copy;
	copy = cv::Mat(road_map_img->rows, road_map_img->cols, CV_8UC3, cv::Scalar(255, 255, 255));
	beta = (1.0 - alpha);
	cv::addWeighted(copy, alpha, *road_map_img, beta, 0.0, *road_map_img);
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
	printf("%s\n", file_name.c_str());
	//getchar();
	int thickness = -1;
	int lineType = 8;

	cv::namedWindow(file_name, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(file_name, 78 + map->config.x_size, 10);
	cv::namedWindow("already_visited", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("already_visited", 78 + 2 * map->config.x_size, 10);

	srand(time(NULL));

	image = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar(255, 255, 255));
	image_already_visited = cv::Mat(map->config.y_size, map->config.x_size, CV_8UC3, cv::Scalar(255, 255, 255));
	road_map_to_image(map, &image);
	fade_image(&image);

	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			if (already_visited[x][y] == 1)
			{
				p.x = x;
				p.y = map->config.y_size - 1 - y;
				cv::circle(image_already_visited, p, 0.4, cv::Scalar(0, 0, 0), thickness, lineType);
			}
		}
	}

	//for(int i = 0; i < rddf_graphs->size; i++)
	//{
	blue = 0;	//rand()%256;
	green = 0;	//rand()%256;
	red = 0;	//rand()%256;
	color[0] = blue;
	color[1] = green;
	color[2] = red;
	//printf("\t%do graph nodes: %d\n", i+1, rddf_graphs[i].graph->size);
	for (int j = begin; j < graph->size; j++)
	{
		p.x = graph->point[j].x;
		p.y = map->config.y_size - 1 - graph->point[j].y;
		//p.x = aux->x;
		//p.y = aux->y;
		cv::circle(image, p, 0.4, cv::Scalar(blue, green, red), thickness, lineType);
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


bool
point_is_lane_center_1(carmen_map_p map, int x, int y)
{
	road_prob *cell_prob;
	cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
	double center = cell_prob->lane_center;

	double diag_x_y_minus_1 = 0.0;
	double diag_x_y_plus_1 = 0.0;
	double diag_x_plus_1_y_minus_1 = 0.0;
	double diag_x_minus_1_y_plus_1 = 0.0;
	double center_y_plus_1 = 0.0;
	double center_y_minus_1 = 0.0;
	double center_x_plus_1 = 0.0;
	double center_x_minus_1 = 0.0;


	if (point_is_in_map(map, x-1, y-1))
		diag_x_y_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y-1])->lane_center;
	if (point_is_in_map(map, x+1, y+1))
		diag_x_y_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y+1])->lane_center;
	if (point_is_in_map(map, x+1, y-1))
		diag_x_plus_1_y_minus_1 = road_mapper_double_to_prob(&map->map[x+1][y-1])->lane_center;
	if (point_is_in_map(map, x-1, y+1))
		diag_x_minus_1_y_plus_1 = road_mapper_double_to_prob(&map->map[x-1][y+1])->lane_center;
	if (point_is_in_map(map, x, y+1))
		center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y + 1])->lane_center;
	if (point_is_in_map(map, x, y-1))
		center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y - 1])->lane_center;
	if (point_is_in_map(map, x+1, y))
		center_x_plus_1 = road_mapper_double_to_prob(&map->map[x + 1][y])->lane_center;
	if (point_is_in_map(map, x-1, y))
		center_x_minus_1 = road_mapper_double_to_prob(&map->map[x - 1][y])->lane_center;

	if(center > 55555)
	{
		if (((center > center_y_minus_1) && (center > center_y_plus_1)) ||
			((center > center_x_minus_1) && (center > center_x_plus_1))// ||
				//((center > diag_x_y_minus_1) && (center > diag_x_y_plus_1)) ||
				//((center > diag_x_minus_1_y_plus_1) && (center > diag_x_plus_1_y_minus_1))
				)
			return true;
		else
			return false;
	}
	else
		return false;
}


bool
point_is_lane_center_2(carmen_map_p map, int x, int y)
{
	road_prob *cell_prob;
	cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
	double center = cell_prob->lane_center;

	double diag_x_y_minus_2 = 0.0;
	double diag_x_y_minus_1 = 0.0;
	double diag_x_y_plus_2 = 0.0;
	double diag_x_y_plus_1 = 0.0;
	double diag_x_plus_1_y_minus_1 = 0.0;
	double diag_x_plus_2_y_minus_2 = 0.0;
	double diag_x_minus_1_y_plus_1 = 0.0;
	double diag_x_minus_2_y_plus_2 = 0.0;
	double center_y_plus_2 = 0.0;
	double center_y_plus_1 = 0.0;
	double center_y_minus_2 = 0.0;
	double center_y_minus_1 = 0.0;
	double center_x_plus_2 = 0.0;
	double center_x_plus_1 = 0.0;
	double center_x_minus_2 = 0.0;
	double center_x_minus_1 = 0.0;


	if (point_is_in_map(map, x-2, y-2))
		diag_x_y_minus_2 = road_mapper_double_to_prob(&map->map[x-2][y-2])->lane_center;
	if (point_is_in_map(map, x-1, y-1))
		diag_x_y_minus_1 = road_mapper_double_to_prob(&map->map[x-1][y-1])->lane_center;
	if (point_is_in_map(map, x+2, y+2))
		diag_x_y_plus_2 = road_mapper_double_to_prob(&map->map[x+2][y+2])->lane_center;
	if (point_is_in_map(map, x+1, y+1))
		diag_x_y_plus_1 = road_mapper_double_to_prob(&map->map[x+1][y+1])->lane_center;
	if (point_is_in_map(map, x+1, y-1))
		diag_x_plus_1_y_minus_1 = road_mapper_double_to_prob(&map->map[x+1][y-1])->lane_center;
	if (point_is_in_map(map, x+2, y-2))
		diag_x_plus_2_y_minus_2 = road_mapper_double_to_prob(&map->map[x+2][y-2])->lane_center;
	if (point_is_in_map(map, x-1, y+1))
		diag_x_minus_1_y_plus_1 = road_mapper_double_to_prob(&map->map[x-1][y+1])->lane_center;
	if (point_is_in_map(map, x-2, y+2))
		diag_x_minus_2_y_plus_2 = road_mapper_double_to_prob(&map->map[x-2][y+2])->lane_center;
	if (point_is_in_map(map, x, y+2))
		center_y_plus_2 = road_mapper_double_to_prob(&map->map[x][y + 2])->lane_center;
	if (point_is_in_map(map, x, y+1))
		center_y_plus_1 = road_mapper_double_to_prob(&map->map[x][y + 1])->lane_center;
	if (point_is_in_map(map, x, y-2))
		center_y_minus_2 = road_mapper_double_to_prob(&map->map[x][y - 2])->lane_center;
	if (point_is_in_map(map, x, y-1))
		center_y_minus_1 = road_mapper_double_to_prob(&map->map[x][y - 1])->lane_center;
	if (point_is_in_map(map, x+2, y))
		center_x_plus_2 = road_mapper_double_to_prob(&map->map[x + 2][y])->lane_center;
	if (point_is_in_map(map, x+1, y))
		center_x_plus_1 = road_mapper_double_to_prob(&map->map[x + 1][y])->lane_center;
	if (point_is_in_map(map, x-2, y))
		center_x_minus_2 = road_mapper_double_to_prob(&map->map[x - 2][y])->lane_center;
	if (point_is_in_map(map, x-1, y))
		center_x_minus_1 = road_mapper_double_to_prob(&map->map[x - 1][y])->lane_center;

	if(center > 55555)
	{
		if (((center > center_y_minus_2 && center > center_y_minus_1) && (center > center_y_plus_2 && center > center_y_plus_1)) ||
			((center > center_x_minus_2 && center > center_x_minus_1) && (center > center_x_plus_2 && center > center_x_plus_1)) ||
				((center > diag_x_y_minus_2 && center > diag_x_y_minus_1) && (center > diag_x_y_plus_2 && center > diag_x_y_plus_1)) ||
				((center > diag_x_minus_2_y_plus_2 && center > diag_x_minus_1_y_plus_1) && (center > diag_x_plus_2_y_minus_2 && center > diag_x_plus_1_y_minus_1))
				)
			return true;
		else
			return false;
	}
	else
		return false;
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
	if (already_visited->map[x][y] == 1)
		return true;
	else
		return false;
}


void
get_new_currents_and_x_y(carmen_position_t *current, int *x, int *y)
{
	if (current->x < 350)
	{
		current->x += 350;
		*x = current->x - 1;
		*y = current->y - 1;
	}
	else if (current->x > 700)
	{
		current->x -= 350;
		*x = current->x - 1;
		*y = current->y - 1;
	}

	if (current->y < 350)
	{
		current->y += 350;
		*x = current->x - 1;
		*y = current->y - 1;
	}
	else if (current->y > 700)
	{
		current->y -= 350;
		*x = current->x - 1;
		*y = current->y - 1;
	}

	//cout<<current->x<<"  "<<current->y<<endl;
	//getchar();
}


void
get_new_map_block(char *folder, char map_type, carmen_map_p map, carmen_point_t pose, int op)
{
	//printf("Carreguei novo mapa!\n");getchar();
	carmen_map_t new_map;
	new_map.complete_map = NULL;
	if(op == 1)
		carmen_grid_mapping_get_block_map_by_origin(folder, map_type, pose, &new_map);
	else if (op == 2)
		carmen_grid_mapping_get_block_map_by_origin_x_y(folder, map_type, pose.x, pose.y, &new_map);
	carmen_grid_mapping_switch_maps(map, &new_map);
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
	//{350,350,700,350,700,700,350,700};
	if (x <= 350 || y < 350 || x > 700 || y > 700)
		return true;

	return false;
}


bool
map_is_not_in_queue(carmen_point_t map_origin, vector <carmen_point_t> &map_queue)
{
	for (unsigned int i = 0; i < map_queue.size(); i++)
	{
		if (map_queue[i].x == map_origin.x && map_queue[i].y == map_origin.y)
			return (false);
	}

	return (true);
}


bool
get_neighbour(carmen_position_t *neighbour, carmen_position_t *current, carmen_map_p already_visited, carmen_map_p map, vector<carmen_position_t> &open_set, vector <carmen_point_t> &map_queue, char *road_map_folder)
//get_neighbour(carmen_position_t *neighbour, carmen_position_t *current, carmen_map_p already_visited, carmen_map_p map, double *x_origin, double *y_origin)
{
	double x_origin, y_origin;
	carmen_point_t map_origin;

	char already_visited_folder[] = "already_visited";
	//printf("%lf X %lf", current->x, current->y);
	//getchar();
	for (int x = current->x - 1; x <= current->x + 1; x++)
	{
		for (int y = current->y - 1; y <= current->y + 1; y++)
		{
			if (point_is_in_map(map, x, y) && !point_is_already_visited(already_visited, x, y))
			{
				//printf("Esta no mapa e nao foi visitado\n");getchar();
				already_visited->map[x][y] = 1;

				if (check_limits_of_central_road_map(current->x, current->y))
				{
					open_set.erase(open_set.begin(), open_set.end());
					carmen_point_t pose;
					pose.x = (x * 0.2) + map->config.x_origin;
					pose.y = (y * 0.2) + map->config.y_origin;
					carmen_grid_mapping_get_map_origin(&pose, &x_origin, &y_origin);
					get_new_map_block(road_map_folder, 'r', map, pose, 1);
					carmen_grid_mapping_save_block_map_by_origin(already_visited_folder, 'a', already_visited);
					get_new_map_block(already_visited_folder, 'a', already_visited, pose, 1);
					get_new_currents_and_x_y(current, &x, &y);
					map_origin.x = map->config.x_origin;
					map_origin.y = map->config.y_origin;
					if(map_is_not_in_queue(map_origin, map_queue))
					{
						map_queue.push_back(map_origin);
					}
					//show_road_map(map, current->x, current->y);
					//show_already_visited(already_visited);
				}

				if (point_is_lane_center_1(map, x, y))
				{
					neighbour->x = x;
					neighbour->y = y;
					return (true);
				}
			}
		}
	}
	return (false);

}


double
convert_image_coordinate_to_world_coordinate(int p, double map_resolution, double map_origin)
{
	double world_coordinate;
	world_coordinate = (p*map_resolution) + map_origin;
	return (world_coordinate);
}


rddf_graph_t *
add_point_to_graph_branch(carmen_map_p map, rddf_graph_t * graph, int x, int y, int branch_node)
{
	graph->point = (carmen_position_t *) realloc(graph->point, (graph->size + 1) * sizeof(carmen_position_t));
	graph->point[graph->size].x = x;
	graph->point[graph->size].y = y;
	//cout<<"\tPonto "<<graph->point[graph->size].x<<" "<<graph->point[graph->size].y<<endl;

	graph->world_coordinate = (carmen_position_t *) realloc(graph->world_coordinate, (graph->size + 1) * sizeof(carmen_position_t));
	graph->world_coordinate[graph->size].x = convert_image_coordinate_to_world_coordinate(x, map->config.resolution, map->config.x_origin);
	graph->world_coordinate[graph->size].y = convert_image_coordinate_to_world_coordinate(y, map->config.resolution, map->config.y_origin);

	graph->edge = (rddf_graph_edges_t *) realloc(graph->edge, (graph->size + 1) * sizeof(rddf_graph_edges_t));
	graph->edge[graph->size].point = (int *) malloc(sizeof(int));
	graph->edge[graph->size].point[0] = graph->size -1;
	graph->edge[graph->size].size = 1;

	graph->edge[branch_node].point = (int *) realloc(graph->edge[branch_node].point, (graph->edge[branch_node].size + 1) * sizeof(int));
	graph->edge[branch_node].point[graph->edge[branch_node].size] = graph->size + 1;
	//cout<<"\tAresta "<<graph->edge[branch_node].point[graph->edge[branch_node].size]<<endl;
	graph->edge[branch_node].size += 1;
	//cout<<"\tArestas do ponto "<<branch_node<<" "<<graph->edge[branch_node].size<<endl;

	graph->size += 1;
	//getchar();

	return (graph);
}


rddf_graph_t *
add_point_to_graph(carmen_map_p map, rddf_graph_t *graph, int x, int y, bool new_first_point)
{
	if (graph == NULL)
	{
		graph = (rddf_graph_t *) malloc(sizeof(rddf_graph_t));
		graph->point = (carmen_position_t *) malloc(sizeof(carmen_position_t));
		graph->point[0].x = x;
		graph->point[0].y = y;

		//cout<<"\tPonto "<<graph->point[0].x<<" "<<graph->point[0].y<<endl;

		graph->world_coordinate = (carmen_position_t *) malloc(sizeof(carmen_position_t));
		graph->world_coordinate[0].x = convert_image_coordinate_to_world_coordinate(x, map->config.resolution, map->config.x_origin);
		graph->world_coordinate[0].y = convert_image_coordinate_to_world_coordinate(y, map->config.resolution, map->config.y_origin);

		graph->edge = (rddf_graph_edges_t *) malloc(sizeof(rddf_graph_edges_t));
		//graph->edge[0].point = (int *) malloc(sizeof(int));
		//graph->edge[0].point[0] = 0; //usando zero ao invês de NULL para evitar warning;
		//cout<<"\tAresta "<<graph->edge[0].point[0]<<endl;
		graph->edge[0].size = 0;

		graph->size = 1;
	}
	else
	{
		graph->point = (carmen_position_t *) realloc(graph->point, (graph->size + 1) * sizeof(carmen_position_t));
		graph->point[graph->size].x = x;
		graph->point[graph->size].y = y;

		//cout<<"\tPonto "<<graph->point[graph->size].x<<" "<<graph->point[graph->size].y<<endl;

		graph->world_coordinate = (carmen_position_t *) realloc(graph->world_coordinate, (graph->size + 1) * sizeof(carmen_position_t));
		graph->world_coordinate[graph->size].x = convert_image_coordinate_to_world_coordinate(x, map->config.resolution, map->config.x_origin);
		graph->world_coordinate[graph->size].y = convert_image_coordinate_to_world_coordinate(y, map->config.resolution, map->config.y_origin);

		//graph->edge[graph->size].point = (int *) realloc(graph->edge[graph->size].point, (graph->edge[graph->size].size + 1) * sizeof(int)); //alberto way
		//graph->edge[graph->size].point[graph->edge[graph->size].size] = graph->size + 1;//alberto way
		//graph->edge[graph->size].size  += 1;

		if(new_first_point == true)
		{
			graph->edge = (rddf_graph_edges_t *) realloc(graph->edge, (graph->size + 1) * sizeof(rddf_graph_edges_t));
			graph->edge[graph->size].size = 0;
		}
		else
		{
			graph->edge = (rddf_graph_edges_t *) realloc(graph->edge, (graph->size + 1) * sizeof(rddf_graph_edges_t));
			graph->edge[graph->size].point = (int *) malloc(sizeof(int));
			graph->edge[graph->size].point[0] = graph->size -1;
			graph->edge[graph->size].size = 1;
		}
		//cout<<"\tAresta "<<graph->edge[graph->size].point[0]<<endl;


		//cout<<"\tArestas do ponto "<<graph->size<<" "<<graph->edge[graph->size].size<<endl;

		graph->size += 1;
	}
	//getchar();

	return (graph);
}

/*rddf_graph_t *
A_star(rddf_graph_t *graph, int x, int y, carmen_map_p map, carmen_map_p already_visited)
{
	char already_visited_folder[] = "already_visited";
	double x_origin, y_origin;
	cout<<"Building graph..."<<endl;
	vector<open_set_t> open_set;
	open_set_t o;
	open_set_t back;
	carmen_position_t current;

	graph = add_point_to_graph(map, graph, x, y);
	o.point = graph->point[0];
	o.point_world_origin.x = map->config.x_origin;
	o.point_world_origin.y = map->config.y_origin;
	open_set.push_back(o);
	//open_set.push_back(graph->world_coordinate[0]);

	while (!open_set.empty())
	{
		//printf("Openset size: %d:\n", open_set.size());
		//carmen_grid_mapping_save_block_map_by_origin(already_visited_folder, 'a', already_visited);
		//cout<<"Openset:"<<endl;
		//for(int i=0;i<open_set.size();i++){
		//	printf("\t%lf X %lf\n", open_set[i].x, open_set[i].y);
		//}
		//getchar();
		back = open_set.back();
		current = back.point;
		printf("\tif %lf != %lf || %lf != %lf\n", back.point_world_origin.x, map->config.x_origin, back.point_world_origin.y, map->config.y_origin);
		if(back.point_world_origin.x != map->config.x_origin || back.point_world_origin.y != map->config.y_origin)
		{
			cout<<"\tentrei!"<<endl;
			carmen_point_t pose;
			pose.x = back.point_world_origin.x;
			pose.y = back.point_world_origin.y;
			printf("\t%lf X %lf\n", pose.x, pose.y);
			//get_new_map_block(g_road_map_folder, 'r', map, pose);
			carmen_grid_mapping_get_block_map_by_origin_x_y(g_road_map_folder, 'r', pose.x, pose.y, map);
			carmen_grid_mapping_save_block_map_by_origin(already_visited_folder, 'a', already_visited);
			//get_new_map_block(already_visited_folder, 'a', already_visited, pose);
			carmen_grid_mapping_get_block_map_by_origin_x_y(already_visited_folder, 'a', pose.x, pose.y, already_visited);
			show_road_map(map, pose.x + (current.x * 0.2), pose.y + (current.y * 0.2));
			show_already_visited(already_visited);
		}
		getchar();
		//current.x = (current.x - map->config.x_origin) / map->config.resolution;
		//current.y = (current.y - map->config.y_origin) / map->config.resolution;
		open_set.pop_back();

		//printf("\tCurrent: %lf X %lf\n", current.x, current.y);

		//carmen_point_t pose;
		//pose.x = current.x;
		//pose.y = current.y;
		//get_new_map_block(g_road_map_folder, 'r', map, pose);
		//printf("RoadMap Origin: %lf\t%lf\n", map->config.x_origin, map->config.y_origin);
		//carmen_grid_mapping_save_block_map_by_origin(already_visited_folder, 'a', already_visited);
		//get_new_map_block(already_visited_folder, 'a', already_visited, pose);
		//open_set.erase(open_set.begin(), open_set.end()); //não está correto pois apaga o vetor, porém, it works!
		//MOSTRAR AO ALBERTO O MAPA SOBRESCREVENDO NA HORA EM QUE O MAPA DESEMPILHA

		carmen_position_t neighbour;
		//carmen_position_t neighbour_global_pos;
		int number_of_neighbours = 0;
		int branch_node;
		while (get_neighbour(&neighbour, &current, already_visited, map, &x_origin, &y_origin))
		{
			//neighbour_global_pos.x = (neighbour.x * map->config.resolution) + map->config.x_origin;
			//neighbour_global_pos.y = (neighbour.y * map->config.resolution) + map->config.y_origin;
			o.point = neighbour;
			o.point_world_origin.x = x_origin;
			o.point_world_origin.y = y_origin;
			open_set.push_back(o);
			//open_set.push_back(neighbour_global_pos);
			//printf("\tNeighbour: %lf X %lf\tNeighbourGlobal: %lf X %lf\n", neighbour.x, neighbour.y, neighbour_global_pos.x, neighbour_global_pos.y);

			if (number_of_neighbours == 0)
			{
				graph = add_point_to_graph(map, graph, neighbour.x, neighbour.y);
				show_road_map(map, neighbour.x, neighbour.y);
				show_already_visited(already_visited);
				//cout<<"GRAPH!"<<neighbour.x<<"\t"<<neighbour.y<<endl;
				//display_graph_over_map(map, graph, already_visited, "oi", 0);
				branch_node = graph->size - 1;
			}
			else
			{
				graph = add_point_to_graph_branch(map, graph, neighbour.x, neighbour.y, branch_node);
				//cout << "branch node!" << endl;
			}
			number_of_neighbours++;
		}
		//getchar();
	}
	cout <<"Finished a graph with size: "<< graph->size << endl<<endl;
	getchar();
	return (graph);
}*/


rddf_graph_t *
A_star(rddf_graph_t *graph, int x, int y, carmen_map_p map, carmen_map_p already_visited, vector <carmen_point_t> &map_queue, char *road_map_folder, bool view_graph_construction)
{
	cout<<"\tBuilding graph..."<<endl;
	vector<carmen_position_t> open_set;
	carmen_position_t current;
	bool new_first_point = true;
	graph = add_point_to_graph(map, graph, x, y, new_first_point);
	new_first_point = false;
	if(graph->size == 1)
		open_set.push_back(graph->point[0]);
	else
		open_set.push_back(graph->point[graph->size-1]);
	//open_set.push_back(graph->world_coordinate[0]);

	while (!open_set.empty())
	{
		current = open_set.back();
		open_set.pop_back();

		carmen_position_t neighbour;
		//carmen_position_t neighbour_global_pos;
		int number_of_neighbours = 0;
		int branch_node;
		while (get_neighbour(&neighbour, &current, already_visited, map, open_set, map_queue, road_map_folder))
		{
			open_set.push_back(neighbour);

			if (number_of_neighbours == 0)
			{
				graph = add_point_to_graph(map, graph, neighbour.x, neighbour.y, new_first_point);
				if(view_graph_construction)
				{
					show_road_map(map, neighbour.x, neighbour.y);
					show_already_visited(already_visited);
				}
				branch_node = graph->size - 1;
			}
			else
			{
				graph = add_point_to_graph_branch(map, graph, neighbour.x, neighbour.y, branch_node);
			}
			number_of_neighbours++;
		}
		//getchar();
	}
	cout <<"\tFinished a road"<< endl<<endl;
//	getchar();

	return (graph);
}


void
write_graph_for_gnuplot (rddf_graph_t * graph)
{
	FILE *f;
	if ((f = fopen("graphs.txt", "w")) == NULL )
	{
		printf("Error opening file\n");
	}

	for (int i = 0; i < graph->size; i++)
	{
		fprintf (f, "%lf %lf\n", graph->world_coordinate[i].x, graph->world_coordinate[i].y);
		//printf ("%lf %lf\n", graph->world_coordinate[i].x, graph->world_coordinate[i].y);

	}
	fclose(f);
}


void
write_graph_on_file(rddf_graph_t *graph, string str_map_identification)
{
	/*
	 File template:
	 number_of_vertexes
	 number_of_edges
	 VERTEXES: //ESSA LINHA NÃO EXISTE!!!!!!!!!!!
	 coordX_vertex_1 coordY_vertex_1
	 coordX_vertex_2 coordY_vertex_2
	 .
	 .
	 .
	 coordX_vertex_n coordY_vertex_n
	 CONECTIONS: //ESSA LINHA NÃO EXISTE!!!!!!!!!!!
	 vertex_1 vertex_2
	 vertex_1 vertex_3
	 .
	 .
	 .
	 vertex_a vertex_b
	 */
	FILE *f;
	char *map_id = NULL;
	char *carmen_enviroment_folder = NULL;
	carmen_enviroment_folder = getenv ("CARMEN_HOME");
	str_map_identification = str_map_identification + ".gr";
	map_id = &str_map_identification[0u];
	char graph_name[100] = "/data/graphs/graph-";
	strcat(carmen_enviroment_folder, graph_name);
	strcat(carmen_enviroment_folder, map_id);

	int total_number_of_edges = 0;
	if ((f = fopen(carmen_enviroment_folder, "wb")) == NULL )
	{
		printf("Error opening file %s\n", carmen_enviroment_folder);
		exit(0);
	}
	for (int i = 0; i < graph->size; i++)
	{
		total_number_of_edges += graph->edge[i].size;
	}

	fprintf (f, "%d\n", graph->size);
	fprintf (f, "%d\n", total_number_of_edges);
	//fprintf (f, "%s\n", "VERTEXES:");

	for (int i = 0; i < graph->size; i++)
	{
		fprintf (f, "%lf %lf\n", graph->world_coordinate[i].x, graph->world_coordinate[i].y);

	}

	//fprintf (f, "%s\n", "CONECTIONS:");
	int cont = 1;
	for (int i = 0; i < graph->size; i++)
	{
		for (int j = 0; j < graph->edge[i].size; j++)
		{
			//cout<<cont<<endl;
			//cont++;
			fprintf (f, "%d %d\n", i, graph->edge[i].point[j]);
		}
	}
	fclose(f);
}


void
print_graph_in_screen (rddf_graph_t *graph)
{
	for (int i = 0; i < 200; i++)
	{
		printf ("[%d]: ", i);
		for (int j = 0; j < graph->edge[i].size; j++)
		{
			printf ("%d", graph->edge[i].point[j]);
			if (j+1 == graph->edge[i].size)
				continue;
			else
				printf(" -> ");
		}
		printf ("\n");

	}
}


void
generate_road_map_graph(carmen_map_p map, char *road_map_folder, string str_map_identification, bool view_graph_construction)
{
	char already_visited_folder[] = "already_visited";
	carmen_map_t already_visited;
	string parsed_filename;
	rddf_graph_t *graph = NULL;
	vector <carmen_point_t> map_queue;
	carmen_point_t map_origin;
	carmen_point_t pose;

	already_visited.complete_map = NULL;
	carmen_grid_mapping_get_block_map_by_origin_x_y(already_visited_folder, 'a', map->config.x_origin, map->config.y_origin, &already_visited);
	map_origin.x = map->config.x_origin;
	map_origin.y = map->config.y_origin;
	map_queue.push_back(map_origin);
	int x,y;
	while(!map_queue.empty())
	{
		cout<<"Finding Lane Center..."<<endl;
		for (x = 0; x < map->config.x_size; x++)
		{
			for (y = 0; y < map->config.y_size; y++)
			{
				if(x == map->config.x_size - 1 && y == map->config.y_size - 1)
				{
					map_queue.erase(map_queue.begin(), map_queue.begin()+1);
					pose = map_queue[0];
					get_new_map_block(road_map_folder, 'r', map, pose, 2);
					carmen_grid_mapping_save_block_map_by_origin(already_visited_folder, 'a', &already_visited);
					get_new_map_block(already_visited_folder, 'a', &already_visited, pose, 2);
				}
				if (point_is_already_visited(&already_visited, x, y))
				{
					continue;
				}
				else
				{
					already_visited.map[x][y] = 1;

					if (point_is_lane_center_1(map, x, y))
					{
						graph = A_star(graph, x, y, map, &already_visited, map_queue, road_map_folder, view_graph_construction);
						pose = map_origin;
						get_new_map_block(road_map_folder, 'r', map, pose, 2);
						carmen_grid_mapping_save_block_map_by_origin(already_visited_folder, 'a', &already_visited);
						get_new_map_block(already_visited_folder, 'a', &already_visited, pose, 2);
					}
				}
			}
		}
	}
	cout<<"All graphs are generated!"<<endl;
	//show_road_map(map, 524, 524);
	//show_already_visited(&already_visited);
	//getchar();

	//write_graph_for_gnuplot (graph);
	//print_graph_in_screen (graph);
	write_graph_on_file(graph, str_map_identification);
}
