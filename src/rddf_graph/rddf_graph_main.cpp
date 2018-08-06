#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <string.h>
#include <libgen.h>
#include <unistd.h> //função sleep
#include "rddf_graph_utils.h"

using namespace std;

string g_window_name1 = "road map";
string g_window_name2 = "road center vertical";
string g_window_name3 = "road center horizontal";
string g_window_name4 = "blended images";

#define MAX_PROB (pow(2.0, 16) - 1.0)

bool g_ipc_required = false;
int g_img_channels = 3;
int g_class_bits = 0;
char *g_road_map_dir = NULL;
char *g_road_map_folder = NULL;
int g_road_map_index = 1;
string str_g_road_map_folder;
char g_map_type;
double g_x_origin;
double g_y_origin;

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
		exit(printf("road_mapper_display_map: disconnected.\n"));
	}
}


void
parse_road_map_dir_type_and_origin(string str_road_map_filename)
{
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
	str_g_road_map_folder = str_road_map_filename.substr(0, last_bar_position);
	map_type = str_road_map_filename.substr(last_bar_position + 1, 1);
	x_origin = str_road_map_filename.substr(last_bar_position + 2, last_underline_position - last_bar_position - 2);
	y_origin = str_road_map_filename.substr(last_trace_position, last_dot_position - last_trace_position);
	g_x_origin = atof(x_origin.c_str());
	g_y_origin = atof(y_origin.c_str());
}


static void
read_parameters(int argc, char **argv)
//read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	const char usage[] = "[<road_map_dir>/<road_map>.map]";
	if (argc != 2)
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
	else
		g_road_map_dir = argv[1];
}


int
main(int argc, char **argv)
{
	read_parameters(argc, argv);
	carmen_grid_mapping_init_parameters(0.2, 210);

	if (access("already_visited/", F_OK) == 0)
		system("rm -f already_visited/*.*");
	else
		system("mkdir already_visited/");


	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}

	signal(SIGINT, shutdown_module);

	carmen_map_t road_map;
	road_map.complete_map = NULL;

	string str_road_map_filename(g_road_map_dir);
	parse_road_map_dir_type_and_origin(str_road_map_filename);
	g_road_map_folder = &str_g_road_map_folder[0u];

	int count_maps = carmen_grid_mapping_get_block_map_by_origin_x_y(g_road_map_folder, 'r', g_x_origin, g_y_origin, &road_map);
	carmen_grid_mapping_update_map_buffer(&road_map, 'r');

	if (count_maps > 0)
	{
		//show_road_map(&road_map,0,0);

		//parse_world_origin_to_road_map(str_road_map_filename);
		generate_road_map_graph(&road_map);
		//print_map_in_terminal(&road_map);getchar();
	}
	else
		cout << "rddf_graph_main: could not read offline map from file named: " << g_road_map_dir << endl;

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
	return 0;
}
