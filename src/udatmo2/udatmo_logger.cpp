#include "udatmo_interface.h"

#include <carmen/carmen.h>

static void print_message(carmen_udatmo_moving_obstacles_message *message)
{
	carmen_udatmo_moving_obstacle *obstacle = message->obstacles;
	printf("From host \"%s\" at time %.2f:", message->host, message->timestamp);
	for (int i = 0, n = message->num_obstacles; i < n; i++, obstacle++)
		printf("  Obstacle #%d: RDDF index %d, position (%f, %f, %f), speed %f m/s",
			   i + 1,
			   obstacle->rddf_index,
			   obstacle->x,
			   obstacle->y,
			   obstacle->theta,
			   obstacle->v);
}

static void shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("\nuDATMO logger: disconnected.\n");
		exit(0);
	}
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_udatmo_subscribe_moving_obstacles_message(NULL, print_message, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}
