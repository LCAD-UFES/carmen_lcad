#include "rddf_graph_utils.h"

using namespace std;
bool g_ipc_required = false;
char *g_graph_filename;
double scale_factor = 0.2;
float circle_size = 0.3;
int graph_size;
cv::Point point1, point2; /* vertical points of the bounding box */
int drag = 0;
cv::Rect rect; /* bounding box */
cv::Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 0;
int click = 0;
int click2 = 0;
bool zoom_in = false;
bool zoom_out = false;
bool show_edges = false;
int refered_point = -999;

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
		//cout<<"click!"<<endl;
		/*"left button clicked. ROI selection begins"<<endl;*/
		point1 = cv::Point(x, y);
		click = 1;
	}

	if (event == CV_EVENT_RBUTTONDOWN)
	{
		//cout<<"clickR"<<endl;
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
translate_back (rddf_graph_t *vertexes, carmen_point_t point)
{
	for (int i = 0; i < graph_size; i++)
	{
		//printf("Point was: %lf X %lf\n", vertexes->world_coordinate[i].x, vertexes->world_coordinate[i].y);
		vertexes->world_coordinate[i].x += point.x;
		vertexes->world_coordinate[i].y += point.y;
		//printf("Point new: %lf X %lf\n", vertexes->world_coordinate[i].x, vertexes->world_coordinate[i].y);getchar();
	}
}


void
scale_points(rddf_graph_t *vertexes, double scale_factor)
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
translate_to_point (rddf_graph_t *vertexes, carmen_point_t point)
{
	for (int i = 0; i < graph_size; i++)
	{
		//printf("Point was: %lf X %lf\n", vertexes->world_coordinate[i].x, vertexes->world_coordinate[i].y);
		vertexes->world_coordinate[i].x -= point.x;
		vertexes->world_coordinate[i].y -= point.y;
		//printf("Point new: %lf X %lf\n", vertexes_transformed->world_coordinate[i].x, vertexes_transformed->world_coordinate[i].y);getchar();
	}
}


void
get_local_pos(carmen_position_t world_coordinate, double x_origin, double y_origin, double *x_local, double *y_local)
{
	*x_local = (world_coordinate.x - x_origin) /scale_factor;
	*y_local = (world_coordinate.y - y_origin) /scale_factor;
}



string
number_to_string (int Number)
{
	stringstream ss;
	ss << Number;
	return ss.str();
}


void
draw(graph_t **graph, rddf_graph_t *vertexes, double x_origin, double y_origin, cv::Mat *image)
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
		if(zoom_in == false)
		{
			get_local_pos (vertexes->world_coordinate[i], x_origin, y_origin, &graph_index_x, &graph_index_y);
			pt.x = graph_index_x;
			pt.y = (1050 - 1 - graph_index_y);//map->config.y_size - 1 - y;
			cv::circle(*image, pt, circle_size, cv::Scalar(255, 0, 0), thickness, lineType);
		}

		else
		{
			pt.x = vertexes->world_coordinate[i].x;
			pt.y = (1050 - 1 - vertexes->world_coordinate[i].y);//map->config.y_size - 1 - y;
			if(i==refered_point)
				cv::circle(*image, pt, circle_size*3, cv::Scalar(255, 0, 255), thickness, lineType);
			else
				cv::circle(*image, pt, circle_size, cv::Scalar(255, 0, 0), thickness, lineType);
		}

		if(show_edges == true){
			graph_t* p;
			for(p = graph[i]; p!=NULL; p = p->prox)
			{
				if(zoom_in == false){
					get_local_pos (p->world_coordinate, x_origin, y_origin, &x_local, &y_local);
					img_pose.x = x_local;
					img_pose.y = y_local;
					pt1.x = img_pose.x;
					pt1.y = (1050 - 1 - img_pose.y);//map->config.y_size - 1 - y;
				}
				else
				{
					img_pose.x = vertexes->world_coordinate[p->vertex].x;
					img_pose.y = vertexes->world_coordinate[p->vertex].y;
					pt1.x = img_pose.x;
					pt1.y = (1050 - 1 - img_pose.y);//map->config.y_size - 1 - y;
				}
				cv::line(*image,pt, pt1, cv::Scalar(0, 0, 255), 1+(circle_size*0.5), 8);
			}

		}
		/*if(show_edges == true)
		{
			cv::imwrite("out.jpg", *image);
			getchar();
		}*/
	}
}


double
distance_calculator (cv::Point clicked_point, carmen_position_t point)
{
	double distance;
	distance = sqrt ( pow((point.x - clicked_point.x),2) + pow((point.y - clicked_point.y),2));
	return (distance);
}


void
edit_graph(graph_t **graph, rddf_graph_t *vertexes)
{
	cv::Mat image;
	cv::Point pt;
	cv::Point pt1;
	cv::Point pt2;
	int thickness = -1;
	int lineType = 8;
	carmen_point_t first_graph_point;
	rddf_graph_t* vertexes_transformed = NULL;
	carmen_position_t vertexes_transformed_local;

	carmen_point_t center_point_in_world_coordinate;
	carmen_point_t translation_factor;
	translation_factor.x = 0.0;
	translation_factor.y = 0.0;
	double x_origin = 0;
	double y_origin = 0;


	image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::namedWindow("Graph", cv::WINDOW_AUTOSIZE);
	double map_scale = 0.72;
	cv::Size size(1050 * map_scale, 1050 * map_scale);


	first_graph_point.x = vertexes->world_coordinate[0].x;// graph[0]->world_coordinate.x;
	first_graph_point.y = vertexes->world_coordinate[0].y;//graph[0]->world_coordinate.y;
	get_map_origin(&first_graph_point, &x_origin, &y_origin);
	x_origin+=70;
	y_origin+=70;
	//printf("Point %lf X %lf has origin in %lf X %lf", first_graph_point.x, first_graph_point.y, x_origin, y_origin);getchar();
	//cv::resize(image, image, size);

	draw (graph, vertexes, x_origin, y_origin, &image);


	int k;
	cv::Point center_point;
	double scale_difference_between_real_map_size_and_image_size = 1050/(1050*map_scale);
	while(1)
	{
		k = (char)cv::waitKey(10);
		cv::setMouseCallback("Graph", CallBackFunc, NULL);
		if (click == 1)
		{
			cv::circle(image, point1, 1.5, cv::Scalar(0, 0, 255), thickness, lineType);
			click = 0;
		}

		if (k == 111) //o
		{
			scale_factor = 0.2;
			circle_size = 0.3;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			/*for(int i = 0; i < graph_size; i++)
						{
							printf("%lf X %lf\n",vertexes->world_coordinate[i].x, vertexes->world_coordinate[i].y );getchar();
						}*/
			draw (graph, vertexes, x_origin, y_origin, &image);

		}

		if (k == 99) //c
		{
			center_point = point1;
			center_point.x *= scale_difference_between_real_map_size_and_image_size;
			center_point.y *= scale_difference_between_real_map_size_and_image_size;
			center_point.y = 1050 - 1 - center_point.y;
			center_point_in_world_coordinate.x = convert_image_coordinate_to_world_coordinate(center_point.x, scale_factor, x_origin);
			center_point_in_world_coordinate.y = convert_image_coordinate_to_world_coordinate(center_point.y, scale_factor, y_origin);
			translation_factor.x = (center_point_in_world_coordinate.x - x_origin);
			translation_factor.y = (center_point_in_world_coordinate.y - y_origin);
			cout<<"Point centered in "<<center_point.x<<" "<<center_point.y<<endl;
			printf("Point in world coordinate %lf X %lf\n", center_point_in_world_coordinate.x,  center_point_in_world_coordinate.y);
			printf("Difference to translate %lf X %lf\n", translation_factor.x,  translation_factor.y);
		}

		if (k == 122) //z
		{
			zoom_in = true;
			scale_factor -= 0.175;
			circle_size += 0.99;
			vertexes_transformed = (rddf_graph_t*)malloc(graph_size * sizeof(rddf_graph_t));
			vertexes_transformed->world_coordinate = (carmen_position_t*)malloc(graph_size * sizeof(carmen_position_t));
			for(int i = 0; i < graph_size; i++)
			{
				vertexes_transformed->world_coordinate[i] = vertexes->world_coordinate[i];
			}
			//cout<<"Zoom in"<<endl;

			translate_to_point(vertexes_transformed, translation_factor);
			for(int i = 0; i < graph_size; i++)
			{
				get_local_pos(vertexes_transformed->world_coordinate[i], x_origin, y_origin, &vertexes_transformed_local.x, &vertexes_transformed_local.y);
				vertexes_transformed->world_coordinate[i] = vertexes_transformed_local;
				//printf("Point new: %lf X %lf\n", vertexes_transformed->world_coordinate[i].x, vertexes_transformed->world_coordinate[i].y);getchar();
			}

			translation_factor.x = translation_factor.x/0.2;
			translation_factor.y = translation_factor.y/0.2;
			translate_back(vertexes_transformed, translation_factor);
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			draw (graph, vertexes_transformed, x_origin, y_origin, &image);
			//draw (graph, vertexes, x_origin, y_origin, &image);
			zoom_in = false;

		}

		if (k == 90) //Z
		{
			circle_size -= 0.2;
			scale_factor += 0.025;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			draw (graph, vertexes, x_origin, y_origin, &image);
		}

		if (k == 109) //m
		{
			double distance;
			center_point = point1;
			center_point.x *= scale_difference_between_real_map_size_and_image_size;
			center_point.y *= scale_difference_between_real_map_size_and_image_size;
			center_point.y = 1050 - 1 - center_point.y;
			center_point_in_world_coordinate.x = convert_image_coordinate_to_world_coordinate(center_point.x, scale_factor, x_origin);
			center_point_in_world_coordinate.y = convert_image_coordinate_to_world_coordinate(center_point.y, scale_factor, y_origin);
			cout<<"Point centered in image "<<center_point.x<<" "<<center_point.y<<endl;
			for(int i = 0; i < graph_size; i++)
			{
				//get_local_pos(vertexes_transformed->world_coordinate[i], x_origin, y_origin, &vertexes_transformed_local.x, &vertexes_transformed_local.y);
				//vertexes_transformed->world_coordinate[i] = vertexes_transformed_local;
				distance = distance_calculator(center_point, vertexes_transformed->world_coordinate[i]);
				if (distance < 5.0)
				{
					refered_point = i;
					break;
				}
				//printf("Point new: %lf X %lf\n", vertexes_transformed->world_coordinate[i].x, vertexes_transformed->world_coordinate[i].y);getchar();
			}
			printf("Refered point: %lf X %lf\n", vertexes_transformed->world_coordinate[refered_point].x, vertexes_transformed->world_coordinate[refered_point].y);
			pt.x = (int)vertexes_transformed->world_coordinate[refered_point].x;
			pt.y = (int)vertexes_transformed->world_coordinate[refered_point].y;
			cout<<"Point centered in "<<pt.x<<" "<<pt.y<<endl;
			pt.y = 1050 - 1 - (int)vertexes_transformed->world_coordinate[refered_point].y;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			//cv::circle(image, point1, circle_size, cv::Scalar(255, 255, 255), thickness, lineType);
			//cv::circle(image, pt, circle_size*2, cv::Scalar(255, 0, 255), thickness, lineType);
			zoom_in = true;
			draw (graph, vertexes_transformed, x_origin, y_origin, &image);
			zoom_in = false;
		}

		if (k == 101) //e
		{
			//print_graph_2(graph);
			if(show_edges == true)
				show_edges = false;
			else
				show_edges = true;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			if(scale_factor == 0.2)
				draw (graph, vertexes, x_origin, y_origin, &image);
			else
			{
				zoom_in = true;
				draw (graph, vertexes_transformed, x_origin, y_origin, &image);
				zoom_in = false;
			}


		}

		if (k == 113) //q
		{
			//print_graph_2(graph);
			string filename;
			cout<<"Graph name: "<<endl;
			cin>>filename;
			cv::imwrite(filename, image);
		}

		if (k == 100) //d
		{
			x_origin+=210;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			draw (graph, vertexes, x_origin, y_origin, &image);
		}

		if (k == 97) //a
		{
			x_origin-=210;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			draw (graph, vertexes, x_origin, y_origin, &image);
		}

		if (k == 119) //w
		{
			y_origin+=210;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			draw (graph, vertexes, x_origin, y_origin, &image);
		}

		if (k == 115) //s
		{
			y_origin-=210;
			image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
			draw (graph, vertexes, x_origin, y_origin, &image);
		}


		//if(k == 32)
		//{
			/*if (click2 == 1)
			{
				//cv::circle(image, point2, 2.1, cv::Scalar(255, 0, 0), thickness, lineType);
				//click2 = 0;
				bool fromCenter = false;
				bool showCrosshair = false;
				cv::Rect2d r = cv::selectROI("image",image, fromCenter, showCrosshair);
				cout<<r.x<<" "<<r.y<<" "<<r.width<<" "<<r.height<<endl;
			}*/
		//}

		if (k == 27) //esc
		{
			exit(1);
		}

		cv::resize(image, image, size);
		cv::imshow("Graph", image);



	}



	//bool fromCenter = false;
	//cv::Rect2d r = cv::selectROI(image);
	//cv::Mat imCrop = image(r);

	    // Display Cropped ImageZZ
	    //imshow("Image", imCrop);
//	cv::Rect2d r = cv::selectROI(image);

	//cv::Point pt2(image.cols/2.0, image.rows/2.0);
	//image = rotate(image, pt2, 90);
	//flip(image, image, 0);


	cv::waitKey(0);
}


static void
read_parameters(int argc, char **argv)
{
	const char usage[] = "<graph_dir>/<graph>.gr";
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

	graph_t **graph = NULL;
	rddf_graph_t *vertexes = NULL;

	FILE *f;
	printf("* * * * Graph Editor Shortcuts * * * \n");
	printf("* o: displays the graph             *\n");
	printf("* Mouse Left: select a point        *\n");
	printf("* c: center in selected point       *\n");
	printf("* z: zoom in                        *\n");
	printf("* Z: zoom out                       *\n");
	printf("* e: show edges / hide edges        *\n");
	printf("* q: save image                     *\n");
	printf("* w/a/s/d: navigate on graph        *\n");
	printf("* esc: exit program                 *\n");
	printf("* * * * * * * * * * * * * * * * * * *\n");

	f = fopen (g_graph_filename,"rb");
	if(f == NULL)
	{
		printf("Graph file could not be read!\n");
		exit(1);
	}

	vertexes = read_vertexes_from_file(vertexes, f);
	graph_size = vertexes->size;
	graph = read_graph_from_file(graph, vertexes, f);
	fclose (f);

	//print_graph_2 (graph);


	edit_graph (graph, vertexes);

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
}
