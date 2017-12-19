#include "neighborhood_graph.h"

#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/virtual_scan_interface.h>


static virtual_scan::Graph graph;


///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Publishers																				 //
//																							 //
///////////////////////////////////////////////////////////////////////////////////////////////


template<class T> T *resize(T *buffer, size_t size)
{
	if (buffer == NULL)
		return (T*) malloc(sizeof(T) * size);
	else
		return (T*) realloc(buffer, sizeof(T) * size);
}


void
virtual_scan_publish_hypotheses(carmen_mapper_virtual_scan_message *data)
{
	static carmen_moving_objects_point_clouds_message message = {0, NULL, 0, carmen_get_host()};

	static double start_time = carmen_get_time();

	virtual_scan::Reading reading(data);
	graph.update(reading);
	virtual_scan::Subgraph &subgraph = graph.back();

	message.num_point_clouds = subgraph.nodes.size();
	message.point_clouds = resize(message.point_clouds, message.num_point_clouds);
	message.timestamp = carmen_get_time();
	int k = 0;

	std::cout << "Time: " << message.timestamp - start_time << ", pose: " << reading.origin << ", hypotheses: " << message.num_point_clouds << std::endl;

	memset(message.point_clouds, '\0', sizeof(t_point_cloud_struct) * message.num_point_clouds);
	for (auto node = subgraph.nodes.begin(), n = subgraph.nodes.end(); node != n; ++node, ++k)
	{
		const virtual_scan::Model *model = node->model;
		const virtual_scan::Pose &pose = node->pose.global;

		message.point_clouds[k].r = 0.0;
		message.point_clouds[k].g = 0.0;
		message.point_clouds[k].b = 1.0;
		message.point_clouds[k].linear_velocity = 0;
		message.point_clouds[k].orientation = pose.o;
		message.point_clouds[k].object_pose.x = pose.x;
		message.point_clouds[k].object_pose.y = pose.y;
		message.point_clouds[k].object_pose.z = 0.0;
		message.point_clouds[k].height = 0;
		message.point_clouds[k].length = model->length;
		message.point_clouds[k].width = model->width;
		message.point_clouds[k].geometric_model = model->id;
		message.point_clouds[k].point_size = 0;

//		std::cout << "    " << *model << ", pose (x, y, o) = " << node->pose.local << std::endl;
	}

	std::cout << std::endl;

	carmen_moving_objects_point_clouds_publish_message(&message);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Handlers																					 //
//																							 //
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
	carmen_mapper_subscribe_virtual_scan_message(NULL, (carmen_handler_t) virtual_scan_publish_hypotheses, CARMEN_SUBSCRIBE_LATEST);
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
