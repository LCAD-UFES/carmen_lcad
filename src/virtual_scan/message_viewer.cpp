#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>
#include <carmen/global_graphics.h>


///////////////////////////////////////////////////////////////////////////////////////////////
//																						   //
// Publishers																				//
//																						   //
///////////////////////////////////////////////////////////////////////////////////////////////


template<class T> T *resize(T *buffer, size_t size)
{
	if (buffer == NULL)
		return (T*) malloc(sizeof(T) * size);
	else
		return (T*) realloc(buffer, sizeof(T) * size);
}


void
virtual_scan_publish_segments(carmen_mapper_virtual_scan_message *message)
{
	static carmen_mapper_virtual_laser_message virtual_laser_message = {0, NULL, NULL, 0.0, carmen_get_host()};

	int n = message->num_points;

	virtual_laser_message.num_positions = n;
	virtual_laser_message.positions = resize(virtual_laser_message.positions, virtual_laser_message.num_positions);
	virtual_laser_message.colors = resize(virtual_laser_message.colors, virtual_laser_message.num_positions);
	virtual_laser_message.timestamp = carmen_get_time();

	carmen_position_t *points = message->points;
	for (int i = 0; i < n; i++)
	{
		const carmen_position_t &global = points[i];
		virtual_laser_message.positions[i].x = global.x;
		virtual_laser_message.positions[i].y = global.y;
		virtual_laser_message.colors[i] = CARMEN_ORANGE;
	}

	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
}


///////////////////////////////////////////////////////////////////////////////////////////////
//																						   //
// Handlers																				  //
//																						   //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();

		printf("\nVirtual Scan: disconnected.\n");
		exit(0);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//																						   //
// Initializations																		   //
//																						   //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_virtual_scan_subscribe_messages()
{
	carmen_mapper_subscribe_virtual_scan_message(NULL, (carmen_handler_t) virtual_scan_publish_segments, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);
	carmen_virtual_scan_subscribe_messages();

	carmen_ipc_dispatch();

	return (0);
}
