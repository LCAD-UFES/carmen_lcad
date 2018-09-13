#include "rddf_graph_utils.h"

using namespace std;
bool g_ipc_required = false;
char *g_graph_filename;
int graph_size;
double scale_factor = 0.2;
float circle_size = 0.3;

cv::Point point1, point2; /* vertical points of the bounding box */
int drag = 0;
cv::Rect rect; /* bounding box */
cv::Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 0;
int click = 0;
int click2 = 0;

static void
define_messages()
{
}


static void
register_handlers()
{
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (g_ipc_required)
			carmen_ipc_disconnect();
		exit(printf("rddf_graph_operations_on_graph_main: disconnected.\n"));
	}
}


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		cout<<"click!"<<endl;
		/*"left button clicked. ROI selection begins"<<endl;*/
		point1 = cv::Point(x, y);
		click = 1;
	}

	if (event == CV_EVENT_RBUTTONDOWN)
	{
		cout<<"clickR"<<endl;
		/*"left button clicked. ROI selection begins"<<endl;*/
		point2 = cv::Point(x, y);
		click2 = 1;
	}





	if (event == CV_EVENT_MOUSEMOVE && drag)
	{
		/* mouse dragged. ROI being selected */
		/*cv::Mat img1 = img.clone();
		point2 = cv::Point(x, y);
		rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
		imshow("image", img1);*/
	}

	if (event == CV_EVENT_LBUTTONUP && drag)
	{
		/*point2 = cv::Point(x, y);
		rect = cv::Rect(point1.x,point1.y,x-point1.x,y-point1.y);
		drag = 0;
		roiImg = img(rect);*/
	}

	if (event == CV_EVENT_LBUTTONUP)
	{
		/* ROI selected */
		/*select_flag = 1;
		drag = 0;*/
	}
}


void
translate_back (rddf_graph_t * vertexes, carmen_point_t point)
{
	for (int i = 0; i < graph_size; i++)
	{
		vertexes->world_coordinate[i].x += point.x;
		vertexes->world_coordinate[i].y += point.y;
	}
}


void
scale_points(rddf_graph_t * vertexes, double scale_factor)
{
	for (int i = 0; i < graph_size; i++)
	{
		//printf("Point was: %lf X %lf\n", vertexes->world_coordinate[i].x, vertexes->world_coordinate[i].y);
		vertexes->world_coordinate[i].x += scale_factor;
		vertexes->world_coordinate[i].y += scale_factor;
		//printf("Point new: %lf X %lf\n", vertexes->world_coordinate[i].x, vertexes->world_coordinate[i].y);getchar();
	}
}


void
translate_to_point (rddf_graph_t * vertexes, carmen_point_t point)
{
	for (int i = 0; i < graph_size; i++)
	{
		vertexes->world_coordinate[i].x -= point.x;
		vertexes->world_coordinate[i].y -= point.y;
	}
}


void
get_local_pos(carmen_position_t world_coordinate, double x_origin, double y_origin, double *x_local, double *y_local)
{
	*x_local = (world_coordinate.x - x_origin) /scale_factor;
	*y_local = (world_coordinate.y - y_origin) /scale_factor;
}


void
get_map_origin(carmen_point_t *global_pose, double *x_origin, double *y_origin)
{
	*x_origin = floor((floor(global_pose->x / (local_gridmap_size / 3.0) ) - 1.0) * (local_gridmap_size / 3.0));
	*y_origin = floor((floor(global_pose->y / (local_gridmap_size / 3.0) ) - 1.0) * (local_gridmap_size / 3.0));
}


void
draw(t_graph **graph, rddf_graph_t *vertexes, double x_origin, double y_origin, cv::Mat *image)
{
	cv::Point pt;
	cv::Point pt1;
	double x_local = 0;
	double y_local = 0;
	double graph_index_x = 0;
	double graph_index_y = 0;
	int thickness = -1;
	int lineType = 8;
	carmen_point_t img_pose;

	for (int i = 0; i < graph_size; i++)
	{
		//printf("%lf X %lf\n", vertexes->world_coordinate[i].x, vertexes->world_coordinate[i].y);
		t_graph* p;
		get_local_pos (vertexes->world_coordinate[i], x_origin, y_origin, &graph_index_x, &graph_index_y);
		pt.x = graph_index_x;
		pt.y = (1050 - 1 - graph_index_y);//map->config.y_size - 1 - y;
		cv::circle(*image, pt, circle_size, cv::Scalar(255, 0, 0), thickness, lineType);

		/*for(p = graph[i]; p!=NULL; p = p->prox)
		{
			get_local_pos (p->world_coordinate, x_origin, y_origin, &x_local, &y_local);
			img_pose.x = x_local;
			img_pose.y = y_local;
			if (img_pose.x < image->cols && img_pose.y < image->rows)
			{
				pt.x = img_pose.x;
				pt.y = (1050 - 1 - img_pose.y);//map->config.y_size - 1 - y;

				pt1.x = graph_index_x;
				pt1.y = (1050 - 1 - graph_index_y);

				cv::circle(*image, pt, 0.7, cv::Scalar(255, 0, 0), thickness, lineType);
				//if(pt != pt1)
					//cv::line(*image,pt, pt1, cv::Scalar(0, 255, 0), 1, 8);

			}
		}*/
	}
}


void
edit_graph(t_graph **graph, rddf_graph_t *vertexes)
{
	cv::Mat image;
	cv::Point pt;
	cv::Point pt1;
	cv::Point pt2;
	int thickness = -1;
	int lineType = 8;
	carmen_point_t first_graph_point;
	rddf_graph_t * vertexes_transformed = vertexes;

	carmen_point_t center_point_in_world_coordinate;
	double x_origin = 0;
	double y_origin = 0;


	image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::namedWindow("Graph", cv::WINDOW_AUTOSIZE);
	cv::Size size(1050 * 0.72, 1050 * 0.72);

	first_graph_point.x = graph[0]->world_coordinate.x;
	first_graph_point.y = graph[0]->world_coordinate.y;
	get_map_origin(&first_graph_point, &x_origin, &y_origin);
	x_origin+=70;
	y_origin+=70;
	//printf("Point %lf X %lf has origin in %lf X %lf", first_graph_point.x, first_graph_point.y, x_origin, y_origin);getchar();
	//cv::resize(image, image, size);

	draw (graph, vertexes, x_origin, y_origin, &image);

	cv::resize(image, image, size);

	int k;
	while(1)
	{
		k = (char)cv::waitKey(10);
		cv::setMouseCallback("Graph", CallBackFunc, NULL);
		if (click == 1)
		{
			cv::circle(image, point1, 2.1, cv::Scalar(0, 0, 255), thickness, lineType);
			click = 0;
		}

		if (k == 111) //o
		{
			//cout<<"Original Image"<<endl;
			scale_factor = 0.2;
			circle_size = 0.3;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			draw (graph, vertexes_transformed, x_origin, y_origin, &image);
			cv::resize(image, image, size);
		}

		if (k == 99) //c
		{
			cv::Point center_point = point1;
			center_point_in_world_coordinate.x = convert_image_coordinate_to_world_coordinate(center_point.x, 0.2, x_origin);
			center_point_in_world_coordinate.y = convert_image_coordinate_to_world_coordinate(center_point.y, 0.2, y_origin);
			//cout<<"Point centered in "<<point1.x<<" "<<point1.y<<endl;
			//printf("Point in world coordinate %lf X %lf\n", center_point_in_world_coordinate.x,  center_point_in_world_coordinate.y);
		}

		if (k == 122) //z
		{
			//cout<<"Zoom in"<<endl;
			scale_factor -= 0.025;
			circle_size += 0.2;
			//translate_to_point(vertexes_transformed, center_point_in_world_coordinate);
			//scale_points(vertexes_transformed, scale_factor);
			//translate_back(vertexes_transformed, center_point_in_world_coordinate);
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			//draw (graph, vertexes_transformed, center_point_in_world_coordinate.x, center_point_in_world_coordinate.y, &image);
			draw (graph, vertexes_transformed, x_origin, y_origin, &image);
			cv::resize(image, image, size);
		}

		if (k == 90) //Z
		{
			//cout<<"Zoom out"<<endl;
			circle_size -= 0.2;
			scale_factor += 0.025;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			//draw (graph, vertexes, center_point_in_world_coordinate.x, center_point_in_world_coordinate.y, &image);
			draw (graph, vertexes_transformed, x_origin, y_origin, &image);
			cv::resize(image, image, size);
		}


		if(k == 32)
		{
			if (click2 == 1)
			{
				cv::circle(image, point2, 2.1, cv::Scalar(255, 0, 0), thickness, lineType);
				click2 = 0;
			}
		}
		cv::imshow("Graph", image);
		/*if (select_flag == 1)
		{
			//cout<<"oi"<<endl;
			imshow("ROI", roiImg);
		}
		rectangle(img, rect, CV_RGB(255, 0, 0), 3, 8, 0);
		imshow("image", img);*/

		if (k == 27)
		{
			exit(1);
		}

	}



	//bool fromCenter = false;
	//cv::Rect2d r = cv::selectROI(image);
	//cv::Mat imCrop = image(r);

	    // Display Cropped Image
	    //imshow("Image", imCrop);
//	cv::Rect2d r = cv::selectROI(image);

	//cv::Point pt2(image.cols/2.0, image.rows/2.0);
	//image = rotate(image, pt2, 90);
	//flip(image, image, 0);


	cv::waitKey(0);
}


t_graph**
add_to_list_undir(t_graph **adjacent_list, int u, int v, rddf_graph_t *graph){
    //printf("%d %d %d\n", u,v,w);
    t_graph *c, *p;
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


t_graph **
read_graph_from_file(t_graph **graph, rddf_graph_t *vertexes, FILE *f)
{
	int u, v;

	graph = (t_graph**)malloc((graph_size)*sizeof(t_graph*));

	for(int i=0; i<graph_size;i++){
		graph[i] = NULL;
	}

	while (fscanf(f, "%d %d", &u, &v) != EOF)
	{
		graph = add_to_list_undir(graph, u, v, vertexes);
	}

	return (graph);
}


rddf_graph_t *
read_vertexes_from_file (rddf_graph_t *vertexes, FILE *f)
{
	int number_of_edges;

	vertexes = (rddf_graph_t *) malloc (sizeof(rddf_graph_t));

	fscanf(f, "%d\n", &vertexes->size);
	fscanf(f, "%d\n", &number_of_edges);
	graph_size = vertexes->size;

	//cout<<vertexes->size<<" "<<number_of_edges<<endl;

	vertexes->world_coordinate = (carmen_position_t *) malloc (vertexes->size * sizeof(carmen_position_t));

	for (int i = 0; i < vertexes->size; i++)
	{
		fscanf(f, "%lf %lf\n", &vertexes->world_coordinate[i].x, &vertexes->world_coordinate[i].y);
	}

	return (vertexes);
}


static void
read_parameters(int argc, char **argv)
{
	const char usage[] = "<graph_dir>/<graph>.bin";
	if (argc < 2){
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	g_graph_filename = argv[1];
}


int
main(int argc, char **argv)
{

	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}
	signal(SIGINT, shutdown_module);

	read_parameters(argc,argv);

	t_graph **graph = NULL;
	rddf_graph_t * vertexes = NULL;

	FILE *f;
	printf("* * * * Graph Editor Shortcuts * * * \n");
	printf("* o: displays the graph             *\n");
	printf("* Mouse Left: select a point        *\n");
	printf("* c: center in selected point       *\n");
	printf("* z: zoom in                        *\n");
	printf("* Z: zoom out                       *\n");
	printf("* esc: exit program                 *\n");
	printf("* * * * * * * * * * * * * * * * * * *\n");

	f = fopen (g_graph_filename,"rb");
	if(f == NULL)
	{
		printf("Graph file could not be read!\n");
		exit(1);
	}

	vertexes = read_vertexes_from_file(vertexes, f);
	graph = read_graph_from_file(graph, vertexes, f);
	fclose (f);

	edit_graph (graph, vertexes);

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
}
