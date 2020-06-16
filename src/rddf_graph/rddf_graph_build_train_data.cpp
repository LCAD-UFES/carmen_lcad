#include "rddf_graph_utils.h"

using namespace std;
bool g_ipc_required = false;
char *g_road_map_dir;
char *g_graph_filename;
bool view_save;


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


string
get_image_id (cv::Mat image)
{
	string image_id = "";
	int middle_x = ceil(image.rows/2);
	int middle_y = ceil(image.cols/2);
	for (int x = middle_x - 1; x <= middle_x + 1; x++)
	{
		for (int y = middle_y - 1; y <= middle_y + 1; y++)
		{
			cv::Vec3b pixel_color = image.at<cv::Vec3b>(cv::Point(x, y));
			if (int(pixel_color[0]) == 0 && int(pixel_color[1]) == 0 && int(pixel_color[2]) == 0)
				image_id += "1";
			else
				image_id += "0";
		}
	}
	return (image_id);
}


carmen_map_t
read_road_map (char *road_map_folder, double x_origin, double y_origin)
{
	carmen_map_t road_map;
	road_map.complete_map = NULL;
	int count_maps = carmen_grid_mapping_get_block_map_by_origin_x_y(road_map_folder, 'r', x_origin, y_origin, &road_map);
	if (count_maps > 0)
	{
		return road_map;
	}
	else
	{
		printf("Impossible to read road maps! Exiting...");
		exit(1);
	}

}


void
get_local_pos_2(carmen_position_t world_coordinate, double x_origin, double y_origin, double *x_local, double *y_local)
{
	*x_local = (world_coordinate.x - x_origin) /0.2;
	*y_local = (world_coordinate.y - y_origin) /0.2;
}


void
save_15_15_image(rddf_graph_t *vertexes, graph_t **graph, string str_road_map_folder)
{
	cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("image", 15*20, 0);
	cv::Mat image;
	cv::Mat image_15_15;
	cv::Mat image_15_15_scaled;
	cv::Mat image_3_3;
	cv::Mat image_3_3_scaled;
	carmen_point_t first_graph_point;
	carmen_map_t road_map;
	double x_origin = 0;
	double y_origin = 0;
	char *road_map_folder = NULL;
	double x_local, y_local;
	x_local = 0;
	y_local = 0;

	image = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
	first_graph_point.x = vertexes->world_coordinate[0].x;// graph[0]->world_coordinate.x;
	first_graph_point.y = vertexes->world_coordinate[0].y;//graph[0]->world_coordinate.y;
	get_map_origin(&first_graph_point, &x_origin, &y_origin);
	//x_origin+=70;
	//y_origin+=70;
	road_map_folder = &str_road_map_folder[0u];

	cv::Size size(15 * 20, 15 * 20);

	road_map = read_road_map (road_map_folder, x_origin, y_origin);
	road_map_to_image(&road_map, &image);
	FILE *f = fopen("database/database.txt", "w");
	int cont = 0;
	cout<<"Generating train data..."<<endl;
	for (int i = 0; i<vertexes->size; i++)
	//for (int i = 0; i<100; i++)
	{
		carmen_point_t pose;
		pose.x = vertexes->world_coordinate[i].x;
		pose.y = vertexes->world_coordinate[i].y;
		carmen_grid_mapping_get_map_origin(&pose, &x_origin, &y_origin);
		get_new_map_block(road_map_folder, 'r', &road_map, pose, 1);
		road_map_to_image(&road_map, &image);
		get_local_pos_2(vertexes->world_coordinate[i], x_origin, y_origin, &x_local, &y_local);

		image_15_15 = cv::Mat(15, 15, CV_8UC3, cv::Scalar(255, 255, 255));
		image_3_3 = cv::Mat(15, 15, CV_8UC3, cv::Scalar(255, 255, 255));
		y_local = 1050 - 1 - y_local;
		int integer_x_local = round(x_local);
		int integer_y_local = round(y_local);

		for (int x = integer_x_local - 7, a = 0; x <= integer_x_local + 7; x++, a++)
		{
			for (int y = integer_y_local - 7, b = 0; y <= integer_y_local + 7; y++, b++)
			{
				cv::Vec3b pixel_color = image.at<cv::Vec3b>(cv::Point(x, y));
				paint_image(a, b, pixel_color, &image_15_15);


				if(x == integer_x_local && y == integer_y_local)
				{
					cv::Vec3b pixel_color (0, 0, 0);
					paint_image(a, b, pixel_color, &image_3_3);
				}

				if((x >= integer_x_local-1 && x <= integer_x_local+1) && (y >= integer_y_local-1 && y <= integer_y_local+1))
				{
					graph_t *p;
					for (p = graph[i]; p != NULL; p = p->prox)
					{
						double x_neigh, y_neigh;
						get_local_pos_2(vertexes->world_coordinate[p->vertex], x_origin, y_origin, &x_neigh, &y_neigh);
						y_neigh = 1050 - 1 - y_neigh;
						int integer_x_neigh = round(x_neigh);
						int integer_y_neigh = round(y_neigh);

						if (x == integer_x_neigh && y == integer_y_neigh)
						{
							cv::Vec3b pixel_color (0, 0, 0);
							paint_image(a, b, pixel_color, &image_3_3);
						}

					}
				}
			}
		}

		char filename[50];
		string image_id;
		image_id = get_image_id (image_3_3);
		string folder = "database";
		sprintf (filename, "%.6d_", i);
		string str_filename(filename);
		//str_filename = str_filename + image_id + "_-1.jpg";

		if (view_save == false)
		{
			str_filename = folder + "/" + str_filename + image_id + "_-1.jpg";
			fprintf(f, "%d %s %s\n", i, str_filename.c_str(), image_id.c_str());
			cv::imwrite(str_filename, image_15_15);
			cont = i;
		}
		else
		{
			cv::resize(image_15_15, image_15_15_scaled, size);
			cv::resize(image_3_3, image_3_3_scaled, size);
			cv::imshow("image1", image_15_15_scaled);
			cv::imshow("image", image_3_3_scaled);
			cv::waitKey();
		}


		//getchar();

	}
	fclose(f);
	cout<<cont<<" train data generated!"<<endl;

}


static void
read_parameters(int argc, char **argv)
{
	const char usage[] = "<road_map_dir> <graph_dir>/<graph>.gr -v to view or -s to save database";
	if (argc < 4){
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	else
	{
		g_road_map_dir = argv[1];
		g_graph_filename = argv[2];
		if (strcmp(argv[3], "-v") == 0)
			view_save = true;
		else
			view_save = false;
	}
}


int
main(int argc, char **argv)
{
	read_parameters(argc,argv);

	//LER DO CARMEN INI!!!!!!!!!!!!!
	carmen_grid_mapping_init_parameters(0.2, 210);

	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}

	if (access("database/", F_OK) != 0)
		system("mkdir database/");
	//signal(SIGINT, shutdown_module);

	string str_graph_filename (g_graph_filename);
	string str_road_map_folder (g_road_map_dir);
	//parse_graph_folder (str_graph_filename, str_road_map_folder);


	graph_t **graph = NULL;
	rddf_graph_t *vertexes = NULL;

	FILE *f;

	f = fopen (g_graph_filename,"rb");
	if(f == NULL)
	{
		printf("Graph file could not be read!\n");
		exit(1);
	}

	vertexes = read_vertexes_from_file(vertexes, f);
	graph = read_graph_from_file(graph, vertexes, f);
	fclose (f);
	//print_graph_2 (graph);

	save_15_15_image (vertexes, graph, str_road_map_folder);

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
}
