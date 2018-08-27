#include "rddf_graph_utils.h"

using namespace std;
bool g_ipc_required = false;
char *g_graph_filename;

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


rddf_graph_t *
read_graph_from_file(rddf_graph_t *graph)
{
	FILE *f;
	int number_of_edges;

	graph = (rddf_graph_t *) malloc (sizeof(rddf_graph_t));

	f = fopen (g_graph_filename,"rb");
	fscanf(f, "%d\n", &graph->size);
	fscanf(f, "%d\n", &number_of_edges);

	graph->world_coordinate = (carmen_position_t *) malloc (graph->size * sizeof(carmen_position_t));

	for(int i = 0; i < graph->size; i++)
	{
		fscanf(f, "%lf %lf\n", &graph->world_coordinate[i].x, &graph->world_coordinate[i].y);
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

	rddf_graph_t *graph = NULL;

	graph = read_graph_from_file(graph);





	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
}
