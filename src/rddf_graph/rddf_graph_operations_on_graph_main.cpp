#include "rddf_graph_utils.h"

using namespace std;
bool g_ipc_required = false;
char *g_graph_filename;
int graph_size;
rddf_graph_t * vertexes;

cv::Point point1, point2; /* vertical points of the bounding box */
int drag = 0;
cv::Rect rect; /* bounding box */
cv::Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 0;

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
	if (event == CV_EVENT_LBUTTONDOWN && !drag)
	{
		cout<< "left button clicked. ROI selection begins"<<endl;
		point1 = cv::Point(x, y);
		drag = 1;
	}

	if (event == CV_EVENT_MOUSEMOVE && drag)
	{
		/* mouse dragged. ROI being selected */
		cv::Mat img1 = img.clone();
		point2 = cv::Point(x, y);
		rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
		imshow("image", img1);
	}

	if (event == CV_EVENT_LBUTTONUP && drag)
	{
		point2 = cv::Point(x, y);
		rect = cv::Rect(point1.x,point1.y,x-point1.x,y-point1.y);
		drag = 0;
		roiImg = img(rect);
	}

	if (event == CV_EVENT_LBUTTONUP)
	{
		/* ROI selected */
		select_flag = 1;
		drag = 0;
	}
}


void
get_local_pos(carmen_position_t world_coordinate, double x_origin, double y_origin, double *x_local, double *y_local)
{
	*x_local = (world_coordinate.x - x_origin) /0.2;
	*y_local = (world_coordinate.y - y_origin) /0.2;
}


void
get_map_origin(carmen_point_t *global_pose, double *x_origin, double *y_origin)
{
	*x_origin = floor((floor(global_pose->x / (local_gridmap_size / 3.0) ) - 1.0) * (local_gridmap_size / 3.0));
	*y_origin = floor((floor(global_pose->y / (local_gridmap_size / 3.0) ) - 1.0) * (local_gridmap_size / 3.0));
}


void
edit_graph(t_graph **graph)
{
	cv::Mat image;
	cv::Point pt;
	cv::Point pt1;
	cv::Point pt2;
	int thickness = -1;
	int lineType = 8;
	carmen_point_t first_graph_point;
	carmen_point_t img_pose;
	double x_origin = 0;
	double y_origin = 0;
	double x_local = 0;
	double y_local = 0;
	double graph_index_x = 0;
	double graph_index_y = 0;

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

	for (int i = 0; i < graph_size; i++)
	{
		//printf("%lf X %lf\n", vertexes->world_coordinate[i].x, vertexes->world_coordinate[i].y);
		t_graph* p;
		get_local_pos (vertexes->world_coordinate[i], x_origin, y_origin, &graph_index_x, &graph_index_y);

		for(p = graph[i]; p!=NULL; p = p->prox)
		{

			get_local_pos (p->world_coordinate, x_origin, y_origin, &x_local, &y_local);
			img_pose.x = x_local;
			img_pose.y = y_local;
			if (img_pose.x < image.cols && img_pose.y < image.rows)
			{
				pt.x = img_pose.x;
				pt.y = (1050 - 1 - img_pose.y);//map->config.y_size - 1 - y;

				pt1.x = graph_index_x;
				pt1.y = (1050 - 1 - graph_index_y);

				cv::circle(image, pt, 0.7, cv::Scalar(255, 255, 255), thickness, lineType);
				if(pt != pt1)
					cv::line(image,pt, pt1, cv::Scalar(0, 255, 0), 1, 8);

			}
		}
	}

	cv::resize(image, image, size);
	img = image;
	cv::imshow("Graph", img);
	int k;
	while(1)
	{
		cv::setMouseCallback("Graph", CallBackFunc, NULL);
		if (select_flag == 1)
		{
			//cout<<"oi"<<endl;
			imshow("ROI", roiImg); /* show the image bounded by the box */
		}
		rectangle(img, rect, CV_RGB(255, 0, 0), 3, 8, 0);
		imshow("image", img);
		k = cv::waitKey(10);
		if (k == 27)
		{
			break;
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


	//cv::waitKey(0);
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
read_graph_from_file(t_graph **graph)
{
	FILE *f;
	int number_of_edges;
	int u, v;


	vertexes = (rddf_graph_t *) malloc (sizeof(rddf_graph_t));

	f = fopen (g_graph_filename,"rb");
	if(f == NULL)
	{
		printf("Graph file could not be read!\n");
		exit(1);
	}

	fscanf(f, "%d\n", &vertexes->size);
	graph_size = vertexes->size;
	graph = (t_graph**)malloc((vertexes->size)*sizeof(t_graph*));
	for(int i=0; i<vertexes->size;i++){
		graph[i] = NULL;
	}
	fscanf(f, "%d\n", &number_of_edges);

	cout<<vertexes->size<<" "<<number_of_edges<<endl;

	vertexes->world_coordinate = (carmen_position_t *) malloc (vertexes->size * sizeof(carmen_position_t));

	for (int i = 0; i < vertexes->size; i++)
	{
		fscanf(f, "%lf %lf\n", &vertexes->world_coordinate[i].x, &vertexes->world_coordinate[i].y);
	}

	cout<<"read coordinates!"<<endl;

	while (fscanf(f, "%d %d", &u, &v) != EOF)
	{
		graph = add_to_list_undir(graph, u, v, vertexes);
	}

	return (graph);
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

	graph = read_graph_from_file(graph);

	edit_graph (graph);

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
}
