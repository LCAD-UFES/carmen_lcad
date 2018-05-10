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

#define LOCAL_MAP_SIZE (210)
#define GLOBAL_MAP_SIZE (1800)

static double global_gridmap_resolution = 0.6;
static double local_gridmap_resolution = 0.2;
static double local_gridmap_size = LOCAL_MAP_SIZE;
static int local_gridmap_count = LOCAL_MAP_SIZE / 0.2;
static int local_gridmap_count_3 = (LOCAL_MAP_SIZE/3) / 0.2;
static int global_gridmap_count = GLOBAL_MAP_SIZE / 0.6;
static int map_quadrant;


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


static double*
get_cell_pointer(double *map, int cell_number)
{
	switch (cell_number)
	{
	case 0:
		return map;
		break;
	case 1:
		return &map[local_gridmap_count * local_gridmap_count_3];
		break;
	case 2:
		return &map[local_gridmap_count * 2 * local_gridmap_count_3];
		break;
	case 3:
		return &map[local_gridmap_count_3];
		break;
	case 4:
		return &map[local_gridmap_count * local_gridmap_count_3 + local_gridmap_count_3];
		break;
	case 5:
		return &map[local_gridmap_count * 2 * local_gridmap_count_3 + local_gridmap_count_3];
		break;
	case 6:
		return &map[2 * local_gridmap_count_3];
		break;
	case 7:
		return &map[2 * local_gridmap_count_3 + local_gridmap_count * local_gridmap_count_3];
		break;
	case 8:
		return &map[2 * local_gridmap_count_3 + local_gridmap_count * local_gridmap_count_3 * 2];
		break;
	}
	return NULL;
}


static void
copy_cell_to_map(carmen_map_t *map, double **cell, int cell_number)
{
	int i;

	double *map_cell;
	map_cell = get_cell_pointer(map->complete_map, cell_number);

	for (i = 0; i < local_gridmap_count_3; i++)
	{
		memcpy(map_cell + i * local_gridmap_count, cell[i], local_gridmap_count_3 * sizeof(double));
	}
}


int
grid_mapping_get_block_map_by_origin_x_y(char *map_path, char map_type, double x_origin, double y_origin, carmen_map_t *new_map)
{
	carmen_map_t unk_map;
	char full_map_path[1024];
	double local_x_origin, local_y_origin;
	int block_map_exists_on_file = 1;
	int count_maps_on_file = 0;
	int i, j, k;

	if (new_map->complete_map == NULL)
		carmen_grid_mapping_initialize_map(new_map, local_gridmap_count, local_gridmap_resolution, map_type);
	else
		carmen_grid_mapping_set_unknown_value(new_map, map_type);


	for (i = 0, k = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++, k++)
		{
			local_x_origin = x_origin + j * (local_gridmap_size / 3.0);
			local_y_origin = y_origin + i * (local_gridmap_size / 3.0);

			sprintf(full_map_path, "%s/%c%d_%d.map", map_path, map_type, (int)local_x_origin, (int)local_y_origin);

			block_map_exists_on_file = carmen_map_read_gridmap_chunk(full_map_path, &unk_map) != -1;

			if (block_map_exists_on_file)
			{
				copy_cell_to_map(new_map, unk_map.map, k);
				free(unk_map.map);
				free(unk_map.complete_map);
				free(unk_map.config.map_name);
				count_maps_on_file++;
			}
		}
	}

	strcpy(new_map->config.origin, "from grid_mapping (built from blocks in files)");
	new_map->config.x_origin = x_origin;
	new_map->config.y_origin = y_origin;

	return count_maps_on_file > 0;

}

void
parse_road_map_dir_type_and_origin(string str_road_map_filename)
{
	string x_origin;
	string y_origin;
	string map_type;
	int l;
	int last_bar_position;
	int last_trace_position=0;
	int last_underline_position=0;
	int last_dot_position=0;
	string file_name;

	for(l=0; l<str_road_map_filename.length();l++)
	{
		if(str_road_map_filename[l] == '/')
			last_bar_position = l;
		if(str_road_map_filename[l] == '.')
			last_dot_position = l;
		if(str_road_map_filename[l] == '_')
			last_underline_position = l;
		if(str_road_map_filename[l] == '-')
			last_trace_position = l;

	}
	str_g_road_map_folder = str_road_map_filename.substr(0, last_bar_position);
	map_type = str_road_map_filename.substr(last_bar_position+1, 1);
	x_origin = str_road_map_filename.substr(last_bar_position+2,last_underline_position-last_bar_position-2);
	y_origin = str_road_map_filename.substr(last_trace_position,last_dot_position-last_trace_position);
	g_x_origin = atof(x_origin.c_str());
	g_y_origin = atof(y_origin.c_str());


}


static void
read_parameters(int argc, char **argv)
//read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	const char usage[] = "[<road_map_dir>/<road_map>.map]";
	if(argc!=2)
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
	else
		g_road_map_dir = argv[1];
}



int
main(int argc, char **argv)
{
	read_parameters(argc, argv);
	system("rm -f already_visited/*.*");


	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}

	signal(SIGINT, shutdown_module);

	carmen_map_t road_map;
	road_map.complete_map = NULL;
	//printf("%d \t %d\n",g_road_map_index, argc);
		//getchar();
	//for (int i = g_road_map_index; i < argc; i++)
	//{
		//char *road_map_filename = argv[i];
		//string str_road_map_filename(road_map_filename);
		//int c = grid_mapping_get_block_map_by_origin_x_y("../../data/road_mapper_map_voltadaufes_20180122",'r',7756420,-363930,&road_map);
		//cout<<road_map.config.x_size<<"\t"<<road_map.config.y_size<<"\t"<<endl;getchar();
		//show_road_map(&road_map);
		//parse_world_origin_to_road_map(str_road_map_filename);
		//bool valid_map_on_file = (carmen_map_read_gridmap_chunk(road_map_filename, &road_map) == 0);

		//set_world_origin_to_road_map(&road_map);
		//if (valid_map_on_file)
		//{
		//	cout << "File " << str_road_map_filename << " being displayed... ("
		//			<< (i - g_road_map_index + 1) << " of " << (argc - g_road_map_index) << ")" << endl;
		//	//road_mapper_display_road_map(&road_map, g_img_channels, g_class_bits);
		//	generate_road_map_graph(&road_map, str_road_map_filename);
			//print_map_in_terminal(&road_map);getchar();
		//}
		//else
		//	cout << "road_mapper_display_map: could not read offline map from file named: " << road_map_filename << endl;
	//}

	string str_road_map_filename(g_road_map_dir);
	parse_road_map_dir_type_and_origin(str_road_map_filename);
	g_road_map_folder = &str_g_road_map_folder[0u];
	int count_maps = grid_mapping_get_block_map_by_origin_x_y(g_road_map_folder,'r',g_x_origin,g_y_origin,&road_map);
	//cout<<road_map.config.x_size<<"\t"<<road_map.config.y_size<<"\t"<<road_map.config.x_origin<<"\t"<<road_map.config.y_origin<<"\t"<<endl;getchar();
	if(count_maps>0)
	{
		show_road_map(&road_map);

		//parse_world_origin_to_road_map(str_road_map_filename);
		generate_road_map_graph(&road_map, str_road_map_filename);
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
