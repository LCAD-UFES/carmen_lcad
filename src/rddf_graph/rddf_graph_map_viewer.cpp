#include "rddf_graph_utils.h"

using namespace std;
bool g_ipc_required = false;
char *g_road_map_1;
char *g_road_map_2;


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


void
show_road_maps(carmen_map_p road_map_1, carmen_map_p road_map_2, char *road_map_1_folder, char *road_map_2_folder, carmen_point_t road_map_origin)
{
	cv::namedWindow("ground_truth", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("infered", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("infered", 350*2, 0);
	cv::Size size(road_map_1->config.x_size/2, road_map_1->config.y_size/2);

	cv::Mat image_1;
	cv::Mat image_2;

	int k;
	while (1)
	{
		k = (char)cv::waitKey(10);
		if (k == 100) //d
		{
			road_map_origin.x+=210;
			get_new_map_block(road_map_1_folder, 'r', road_map_1, road_map_origin, 2);
			get_new_map_block(road_map_2_folder, 'r', road_map_2, road_map_origin, 2);
		}

		if (k == 97) //a
		{
			road_map_origin.x-=210;
			get_new_map_block(road_map_1_folder, 'r', road_map_1, road_map_origin, 2);
			get_new_map_block(road_map_2_folder, 'r', road_map_2, road_map_origin, 2);
		}

		if (k == 119) //w
		{
			road_map_origin.y+=210;
			get_new_map_block(road_map_1_folder, 'r', road_map_1, road_map_origin, 2);
			get_new_map_block(road_map_2_folder, 'r', road_map_2, road_map_origin, 2);
		}

		if (k == 115) //s
		{
			road_map_origin.y-=210;
			get_new_map_block(road_map_1_folder, 'r', road_map_1, road_map_origin, 2);
			get_new_map_block(road_map_2_folder, 'r', road_map_2, road_map_origin, 2);
		}

		image_1 = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
		image_2 = cv::Mat(1050, 1050, CV_8UC3, cv::Scalar(255, 255, 255));
		road_map_to_image(road_map_1, &image_1);
		road_map_to_image(road_map_2, &image_2);

		cv::resize(image_1, image_1, size);
		cv::resize(image_2, image_2, size);
		cv::imshow("ground_truth", image_1);
		cv::imshow("infered", image_2);
		//cv::waitKey();
	}

}


void
parse_road_map_dir_type_and_origin(string str_road_map_filename, string &str_road_map_folder, string &str_map_identification, carmen_point_p road_map_origin)
{
	//cout<<str_road_map_filename<<endl;
	string x_origin;
	string y_origin;
	string map_type;
	unsigned int l;
	int last_bar_position = 0;
	int last_trace_position = 0;
	int last_underline_position = 0;
	int last_dot_position = 0;
	string file_name;

	for (l = 0; l < str_road_map_filename.length(); l++)
	{
		if (str_road_map_filename[l] == '/')
			last_bar_position = l;
		if (str_road_map_filename[l] == '.')
			last_dot_position = l;
		if (str_road_map_filename[l] == '_')
			last_underline_position = l;
		if (str_road_map_filename[l] == '-')
			last_trace_position = l;

	}
	str_road_map_folder = str_road_map_filename.substr(0, last_bar_position);
	//cout<<str_road_map_folder<<endl;
	map_type = str_road_map_filename.substr(last_bar_position + 1, 1);
	x_origin = str_road_map_filename.substr(last_bar_position + 2, last_underline_position - last_bar_position - 2);
	y_origin = str_road_map_filename.substr(last_trace_position, last_dot_position - last_trace_position);
	road_map_origin->x = atof(x_origin.c_str());
	road_map_origin->y = atof(y_origin.c_str());

	for (l = 0; l < str_road_map_folder.length(); l++)
	{
		if (str_road_map_filename[l] == '/')
			last_bar_position = l;
	}
	str_map_identification = str_road_map_folder.substr(last_bar_position+1, str_road_map_folder.length());

	//getchar();
}


static void
read_parameters(int argc, char **argv)
{
	const char usage[] = "<road_map_folder_1>/<road_map>.map <road_map_folder_2>/<road_map>.map";
	if (argc < 3){
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	else
	{
		g_road_map_1 = argv[1];
		g_road_map_2 = argv[2];
	}
}


int
main(int argc, char **argv)
{
	read_parameters(argc,argv);
	carmen_grid_mapping_init_parameters(0.2, 210);

	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}
	//signal(SIGINT, shutdown_module);

	carmen_map_t road_map_1;
	carmen_map_t road_map_2;

	carmen_point_t road_map_origin;
	char *road_map_1_folder = NULL;
	char *road_map_2_folder = NULL;
	string str_road_map_1_folder;
	string str_road_map_2_folder;
	road_map_1.complete_map = NULL;
	road_map_2.complete_map = NULL;

	string str_road_map_1_filename(g_road_map_1);
	string str_road_map_2_filename(g_road_map_2);

	string str_map_identification;

	parse_road_map_dir_type_and_origin(str_road_map_1_filename, str_road_map_1_folder, str_map_identification, &road_map_origin);
	parse_road_map_dir_type_and_origin(str_road_map_2_filename, str_road_map_2_folder, str_map_identification, &road_map_origin);
	road_map_1_folder = &str_road_map_1_folder[0u];
	road_map_2_folder = &str_road_map_2_folder[0u];

	int count_maps_1 = carmen_grid_mapping_get_block_map_by_origin_x_y(road_map_1_folder, 'r', road_map_origin.x, road_map_origin.y, &road_map_1);
	int count_maps_2 = carmen_grid_mapping_get_block_map_by_origin_x_y(road_map_2_folder, 'r', road_map_origin.x, road_map_origin.y, &road_map_2);

	if (count_maps_1 > 0 && count_maps_2 > 0)
		show_road_maps(&road_map_1, &road_map_2, road_map_1_folder, road_map_2_folder, road_map_origin);

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
}
