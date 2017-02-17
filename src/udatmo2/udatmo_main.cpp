#include <carmen/carmen.h>
#include "udatmo_node.h"

static void shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("\nuDATMO: disconnected.\n");
		exit(0);
	}
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_udatmo_define_messages();
	carmen_udatmo_install_params(argc, argv);
	carmen_udatmo_subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}
