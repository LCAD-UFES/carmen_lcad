#include "rddf_graph_utils.h"

using namespace std;

bool g_ipc_required = false;
int g_already_visited_index = 1;

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
		exit(printf("rddf_graph_already_visited_display: disconnected.\n"));
	}
}


void
rddf_graph_display_already_visited(carmen_map_p already_visited)
{
	cv::Mat image1;
	cv::namedWindow("already_visited", cv::WINDOW_AUTOSIZE);

	image1 = cv::Mat(already_visited->config.y_size, already_visited->config.x_size, CV_8UC3, cv::Scalar::all(0));
	already_visited_to_image(already_visited, &image1);
	cv::imshow("already_visited", image1);

	cout << "Press \"Esc\" key to continue...\n\n";
	while((cv::waitKey() & 0xff) != 27);
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

	carmen_map_t already_visited;
	for (int i = g_already_visited_index; i < argc; i++)
	{
		char *already_visited_filename = argv[i];
		bool valid_map_on_file = (carmen_map_read_gridmap_chunk(already_visited_filename, &already_visited) == 0);
		if (valid_map_on_file)
		{
			cout << "File " << string(already_visited_filename) << " being displayed... ("
					<< (i - g_already_visited_index + 1) << " of " << (argc - g_already_visited_index) << ")" << endl;
			rddf_graph_display_already_visited(&already_visited);
		}

	}

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
	return 0;
}
