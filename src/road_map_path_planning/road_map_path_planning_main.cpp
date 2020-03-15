#include "road_map_path_planning_utils.h"

bool g_ipc_required = true;
carmen_gps_xyz_message current_gps_info;
t_osmnx params;

static void
define_messages()
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


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
gps_xyz_message_handler(carmen_gps_xyz_message *message)
{
	current_gps_info = *message;
	printf("Lat: %lf X Lon: %lf\n", current_gps_info.latitude, current_gps_info.longitude);
}

///////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

void
subscribe_messages()
{
	carmen_gps_xyz_subscribe_message(NULL, (carmen_handler_t) gps_xyz_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
read_parameters(int argc, char **argv)
//read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	const char usage[] = "<locale>, <city>, <country>, <simplify graph: 0 or 1?>, <plot graph: 0 or 1>, <generate route: 0 or 1>";
	if (argc < 7){
		printf("Incorrect Input!.\nUsage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	else{
		params.locale = argv[1];
		params.city = argv[2];
		params.country = argv[3];
		params.graph_type = argv[4];
		params.plot = argv[5];
		params.route = argv[6];
		params.python_command = "python3 osmnx/osp_test.py '" +string(params.locale)+","+string(params.city)+","+string(params.country)+"' "
								+ string(params.graph_type) + " " + string(params.plot) + " " + string(params.route);

//		string teste = to_string(45.7);

		cout<< params.python_command<<endl;

	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////




int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
//	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	define_messages();
	subscribe_messages();

	read_parameters(argc, argv);

//	if (g_ipc_required)
//	{
//		carmen_ipc_initialize(argc, argv);
//		carmen_param_check_version(argv[0]);
//		define_messages();
//		subscribe_messages();
//		signal(SIGINT, shutdown_module);
//		carmen_ipc_dispatch();
//	}

	cout<<"oi"<<endl;

	process_graph(params.python_command);

	//carmen_ipc_dispatch();

	return 0;
}
