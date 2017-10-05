#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>
#include <carmen/global_graphics.h>
#include "virtual_scan.h"
#include <carmen/map_server_interface.h>

#define NUM_COLORS 4

double d_max;
carmen_mapper_virtual_laser_message virtual_laser_message;
char colors[NUM_COLORS] = {CARMEN_RED, CARMEN_GREEN, CARMEN_LIGHT_BLUE, CARMEN_ORANGE};
carmen_localize_ackerman_map_t localize_map;
//double x_origin = 0.0;
//double y_origin = 0.0;

///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


//void
//publish_virtual_scan(virtual_scan_segments_t *virtual_scan_segments)
//{
//	virtual_laser_message.host = carmen_get_host();
//	virtual_laser_message.num_positions = 0;
//	for (int i = 0; i < virtual_scan_segments->num_segments; i++)
//		virtual_laser_message.num_positions += virtual_scan_segments->segment[i].num_points;
//	virtual_laser_message.positions = (carmen_position_t *) malloc(virtual_laser_message.num_positions * sizeof(carmen_position_t));
//	virtual_laser_message.colors = (char *) malloc(virtual_laser_message.num_positions * sizeof(char));
//	int k = 0;
//	for (int i = 0; i < virtual_scan_segments->num_segments; i++)
//	{
//		char color = colors[i % NUM_COLORS];
//		for (int j = 0; j < virtual_scan_segments->segment[i].num_points; j++)
//		{
//			virtual_laser_message.positions[k].x = virtual_scan_segments->segment[i].point[j].x;
//			virtual_laser_message.positions[k].y = virtual_scan_segments->segment[i].point[j].y;
//			virtual_laser_message.colors[k] = color;
//			k++;
//		}
//	}
//	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//
//	free(virtual_laser_message.positions);
//	free(virtual_laser_message.colors);
//	free(virtual_scan_segments->segment);
//	free(virtual_scan_segments);
//}


void
publish_virtual_scan(virtual_scan_segment_classes_t *virtual_scan_segment_classes)
{
	virtual_laser_message.host = carmen_get_host();
	virtual_laser_message.num_positions = 0;
	for (int i = 0; i < virtual_scan_segment_classes->num_segments; i++)
		virtual_laser_message.num_positions += virtual_scan_segment_classes->segment[i].num_points;
	virtual_laser_message.positions = (carmen_position_t *) malloc(virtual_laser_message.num_positions * sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) malloc(virtual_laser_message.num_positions * sizeof(char));
	int k = 0;
	for (int i = 0; i < virtual_scan_segment_classes->num_segments; i++)
	{
		char color = colors[virtual_scan_segment_classes->segment_features[i].segment_class];
		for (int j = 0; j < virtual_scan_segment_classes->segment[i].num_points; j++)
		{
			virtual_laser_message.positions[k].x = virtual_scan_segment_classes->segment[i].point[j].x;
			virtual_laser_message.positions[k].y = virtual_scan_segment_classes->segment[i].point[j].y;
			virtual_laser_message.colors[k] = color;
			k++;
		}
	}
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());

	free(virtual_laser_message.positions);
	free(virtual_laser_message.colors);
	free(virtual_scan_segment_classes->segment);
	free(virtual_scan_segment_classes->segment_features);
	free(virtual_scan_segment_classes);
}


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_mapper_virtual_scan_message_handler(carmen_mapper_virtual_scan_message *message)
{
	virtual_scan_segment_classes_t *virtual_scan_segment_classes = detect_and_track_moving_objects(message);
	publish_virtual_scan(virtual_scan_segment_classes);
}


static void
localize_map_update_handler(carmen_map_server_localize_map_message *message)
{
	carmen_map_server_localize_map_message_to_localize_map(message, &localize_map);

//	x_origin = message->config.x_origin;
//	y_origin = message->config.y_origin;

//	necessary_maps_available = 1;
}


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



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_virtual_scan_install_params(int argc, char *argv[])
{
	carmen_param_t laser_param_list[] =
	{
		{(char *) "polar_slam", (char *) "laser_max_range", CARMEN_PARAM_DOUBLE, &d_max, 0, NULL}
	};

	carmen_param_install_params(argc, argv, laser_param_list, sizeof(laser_param_list) / sizeof(laser_param_list[0]));
}


void
carmen_virtual_scan_define_messages()
{

}


void
carmen_virtual_scan_subscribe_messages()
{
	carmen_mapper_subscribe_virtual_scan_message(NULL, (carmen_handler_t) carmen_mapper_virtual_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_map_server_subscribe_localize_map_message(NULL, (carmen_handler_t) localize_map_update_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_virtual_scan_install_params(argc, argv);
	carmen_virtual_scan_define_messages();

	signal(SIGINT, shutdown_module);
	carmen_virtual_scan_subscribe_messages();

	carmen_ipc_dispatch();

	return (0);
}
