#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/map_server_interface.h>
#include <carmen/map.h>
#include <carmen/grid_mapping.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/velodyne_interface.h>

#include "virtual_scan.h"

#define NUM_COLORS 4
#define NMC	250
#define T 10

double d_max;
carmen_mapper_virtual_laser_message virtual_laser_message;
//							L shaped	I shaped		Mass point
char colors[NUM_COLORS] = {CARMEN_RED, CARMEN_GREEN, CARMEN_LIGHT_BLUE, CARMEN_ORANGE};
carmen_localize_ackerman_map_t localize_map;
double x_origin = 0.0;
double y_origin = 0.0;
double map_resolution = 0.0;

int necessary_maps_available = 0;

int g_zi = 0;

virtual_scan_extended_t *g_virtual_scan_extended[NUMBER_OF_FRAMES_T];
virtual_scan_segment_classes_t *g_virtual_scan_segment_classes[NUMBER_OF_FRAMES_T];

virtual_scan_neighborhood_graph_t *g_neighborhood_graph = NULL;


///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Publishers																			     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////


void
virtual_scan_publish_moving_objects(carmen_moving_objects_point_clouds_message *moving_objects)
{
	if (moving_objects != NULL)
		carmen_moving_objects_point_clouds_publish_message(moving_objects);
}


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
virtual_scan_publish_box_models(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses)
{
	static carmen_moving_objects_point_clouds_message message = {0, NULL, 0, NULL};

	if (message.point_clouds != NULL)
		free(message.point_clouds);

	message.host = carmen_get_host();
	message.timestamp = carmen_get_time();

	int total = virtual_scan_num_box_models(virtual_scan_box_model_hypotheses);
	message.point_clouds = (t_point_cloud_struct *) calloc(total, sizeof(t_point_cloud_struct));
	message.num_point_clouds = total;

	virtual_scan_box_models_t *hypotheses = virtual_scan_box_model_hypotheses->box_model_hypotheses;
	for (int i = 0, k = 0, m = virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i < m; i++)
	{
		virtual_scan_box_model_t *boxes = hypotheses[i].box;
		for (int j = 0, n = hypotheses[i].num_boxes; j < n; j++, k++)
		{
			virtual_scan_box_model_t *box = (boxes + j);

			message.point_clouds[k].r = 0.0;
			message.point_clouds[k].g = 0.0;
			message.point_clouds[k].b = 1.0;
			message.point_clouds[k].linear_velocity = 0;
			message.point_clouds[k].orientation = box->theta;
			message.point_clouds[k].object_pose.x = box->x;
			message.point_clouds[k].object_pose.y = box->y;
			message.point_clouds[k].object_pose.z = 0.0;
			message.point_clouds[k].height = 0;
			message.point_clouds[k].length = box->length;
			message.point_clouds[k].width = box->width;
			message.point_clouds[k].geometric_model = box->c;
			message.point_clouds[k].point_size = 0; // 1
//			message.point_clouds[k].num_associated = timestamp_moving_objects_list[current_vector_index].objects[i].id;

			object_model_features_t &model_features = message.point_clouds[k].model_features;
			model_features.model_id = box->c;
			model_features.model_name = (char *) "name?";
			model_features.geometry.length = box->length;
			model_features.geometry.width = box->width;

//			message.point_clouds[k].points = (carmen_vector_3D_t *) malloc(1 * sizeof(carmen_vector_3D_t));
//			message.point_clouds[k].points[0].x = box->x;
//			message.point_clouds[k].points[0].y = box->y;
//			message.point_clouds[k].points[0].z = 0.0;
		}
	}

	carmen_moving_objects_point_clouds_publish_message(&message);
}


void
virtual_scan_publish_segments(virtual_scan_segment_classes_t *virtual_scan_segment_classes)
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
			virtual_laser_message.positions[k].x = virtual_scan_segment_classes->segment[i].points[j].x;
			virtual_laser_message.positions[k].y = virtual_scan_segment_classes->segment[i].points[j].y;
			virtual_laser_message.colors[k] = color;
			k++;
		}
	}
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());

	free(virtual_laser_message.positions);
	free(virtual_laser_message.colors);
}


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Handlers																					 //
//																							 //
///////////////////////////////////////////////////////////////////////////////////////////////

void
carmen_mapper_virtual_scan_message_handler(carmen_mapper_virtual_scan_message *message)
{
	if (necessary_maps_available)
	{
		virtual_scan_free_scan_extended(g_virtual_scan_extended[g_zi]);
		g_virtual_scan_extended[g_zi] = sort_virtual_scan(message);

		virtual_scan_free_segment_classes(g_virtual_scan_segment_classes[g_zi]);
		g_virtual_scan_segment_classes[g_zi] = virtual_scan_extract_segments(g_virtual_scan_extended[g_zi]);
		virtual_scan_publish_segments(g_virtual_scan_segment_classes[g_zi]);

		virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses = virtual_scan_fit_box_models(g_virtual_scan_segment_classes[g_zi]); // acrescentar numa lista de tamanho T e retornar o ultimo
		virtual_scan_publish_box_models(virtual_scan_box_model_hypotheses);

		g_neighborhood_graph = virtual_scan_update_neighborhood_graph(g_neighborhood_graph, virtual_scan_box_model_hypotheses); // usar os pontos vindos das funcoes acima
//		carmen_moving_objects_point_clouds_message *moving_objects = virtual_scan_infer_moving_objects(g_neighborhood_graph);
//		virtual_scan_publish_moving_objects(moving_objects);

		virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses); // remover o que está no fim de T
//		virtual_scan_free_moving_objects(moving_objects);

		g_zi++;
		if (g_zi >= NUMBER_OF_FRAMES_T)
			g_zi = 0;
	}
}


static void
localize_map_update_handler(carmen_map_server_localize_map_message *message)
{
	carmen_map_server_localize_map_message_to_localize_map(message, &localize_map);

	x_origin = message->config.x_origin;
	y_origin = message->config.y_origin;
	map_resolution = message->config.resolution;

	necessary_maps_available = 1;
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
//																						     //
// Initializations																		     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////


void
read_parameters(int argc, char *argv[])
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

	read_parameters(argc, argv);
	carmen_virtual_scan_define_messages();

	get_world_pose_with_velodyne_offset_initialize(argc, argv);

	memset(g_virtual_scan_extended, 0, sizeof(virtual_scan_extended_t *) * NUMBER_OF_FRAMES_T);
	memset(g_virtual_scan_segment_classes, 0, sizeof(virtual_scan_segment_classes_t *) * NUMBER_OF_FRAMES_T);

	signal(SIGINT, shutdown_module);
	carmen_virtual_scan_subscribe_messages();

	carmen_ipc_dispatch();

	return (0);
}
