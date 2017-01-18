#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>

static void shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();

		printf("\nVirtual Scan: disconnected.\n");
		exit(0);
	}
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_virtual_scan_define_messages();
	carmen_virtual_scan_install_params(argc, argv);
	carmen_virtual_scan_subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}
