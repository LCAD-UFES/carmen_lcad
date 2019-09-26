#include "road_map_path_planning_utils.h"

bool g_ipc_required = false;
t_osmnx params;

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
		exit(printf("road_network: disconnected.\n"));
	}
}


static void
read_parameters(int argc, char **argv)
//read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	const char usage[] = "<locale>, <city>, <country>, <simplify graph: 0 or 1?>, <plot graph: 0 or 1>";
	if (argc < 6){
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	else{
		params.locale = argv[1];
		params.city = argv[2];
		params.country = argv[3];
		params.graph_type = argv[4];
		params.plot = argv[5];
		params.python_command = "python3 osmnx/osp_test.py '" +string(params.locale)+","+string(params.city)+","+string(params.country)+"' "
								+ string(params.graph_type) + " " + string(params.plot);

		cout<< params.python_command<<endl;

	}
}


int
main(int argc, char **argv)
{

	read_parameters(argc, argv);

	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}
	signal(SIGINT, shutdown_module);
	process_graph(params.python_command);


	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}




	return 0;
}
