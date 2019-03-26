#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <string.h>
#include <libgen.h>
#include <unistd.h> //função sleep
#include "rddf_graph_utils.h"

using namespace std;


#define MAX_PROB (pow(2.0, 16) - 1.0)

bool g_ipc_required = false;
bool g_view_graph_construction;
int g_img_channels = 3;
int g_class_bits = 0;

char *g_road_map_dir = NULL;

int g_road_map_index = 1;
char g_map_type;


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
		exit(printf("rddf_graph_main: disconnected.\n"));
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
//read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	const char usage[] = "<road_map_dir>/<road_map>.map -v(to view graph construction)";
	if (argc < 2){
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	else{
		g_road_map_dir = argv[1];
		if(argc == 2)
		{
			g_view_graph_construction = false;
		}
		else if(argc == 3)
		{
			if (strcmp (argv[2], "-v") == 0)
			{
				g_view_graph_construction = true;
			}
			else
			{
				g_view_graph_construction = false;
				printf("Ignoring graph construction view.\n");
				printf("To view graph construction:\n\t%s %s\n\n\n", argv[0], usage);
			}

		}
	}
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
	carmen_point_t road_map_origin;
	char *road_map_folder = NULL;
	string str_road_map_folder;
	road_map.complete_map = NULL;

	string str_road_map_filename(g_road_map_dir);
	string str_map_identification;
	parse_road_map_dir_type_and_origin(str_road_map_filename, str_road_map_folder, str_map_identification, &road_map_origin);
	road_map_folder = &str_road_map_folder[0u];

	int count_maps = carmen_grid_mapping_get_block_map_by_origin_x_y(road_map_folder, 'r', road_map_origin.x, road_map_origin.y, &road_map);
	cout<<road_map.config.x_size<<" "<<road_map.config.y_size<<endl;
	//carmen_grid_mapping_update_map_buffer(&road_map, 'r');
	
	if (count_maps > 0)
	{
		//show_road_map(&road_map,0,0);

		//parse_world_origin_to_road_map(str_road_map_filename);
		generate_road_map_graph(&road_map, road_map_folder, str_map_identification, g_view_graph_construction);
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
