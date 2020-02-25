#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/map_server_interface.h>
#include <carmen/map.h>
#include <carmen/grid_mapping.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_transforms.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/matrix.h>

#include "virtual_scan.h"

//#define SIMULATE_LATERAL_MOVING_OBSTACLE
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
double x_size = 0.0;
double y_size = 0.0;
double map_resolution = 0.0;

int necessary_maps_available = 0;

int g_zi = 0;

carmen_mapper_virtual_scan_message *g_virtual_scan_extended[NUMBER_OF_FRAMES_T];
virtual_scan_segment_classes_t *g_virtual_scan_segment_classes[NUMBER_OF_FRAMES_T];

carmen_point_t g_initial_pos;
carmen_point_t g_current_pos;

void
fill_in_moving_objects_message_element(int k, carmen_moving_objects_point_clouds_message *message, virtual_scan_box_model_t *box)
{
	message->point_clouds[k].r = 0.0;
	message->point_clouds[k].g = 0.0;
	message->point_clouds[k].b = 1.0;
	message->point_clouds[k].linear_velocity = 0.0;
	message->point_clouds[k].orientation = box->theta;
	message->point_clouds[k].object_pose.x = box->x;
	message->point_clouds[k].object_pose.y = box->y;
	message->point_clouds[k].object_pose.z = 0.0;
	message->point_clouds[k].height = box->width;
	message->point_clouds[k].length = box->length;
	message->point_clouds[k].width = box->width;
	message->point_clouds[k].geometric_model = box->c;
	message->point_clouds[k].point_size = 0;
	message->point_clouds[k].num_associated = 0;
	object_model_features_t& model_features = message->point_clouds[k].model_features;
	model_features.model_id = box->c;

	switch (box->c)
	{
		case BUS:
			model_features.model_name = (char *) ("Bus");
			model_features.red = 1.0;
			model_features.green = 0.0;
			model_features.blue = 0.0;
			break;
		case CAR:
			model_features.model_name = (char *) ("Car");
			model_features.red = 0.5;
			model_features.green = 1.0;
			model_features.blue = 0.0;
			break;
		case BIKE:
			model_features.model_name = (char *) ("Bike");
			model_features.red = 0.5;
			model_features.green = 0.5;
			model_features.blue = 0.5;
			break;
		case PEDESTRIAN:
			model_features.model_name = (char *) ("Pedestrian");
			model_features.red = 0.0;
			model_features.green = 1.0;
			model_features.blue = 1.0;
			break;
	}
	model_features.geometry.length = box->length;
	model_features.geometry.width = box->width;
	model_features.geometry.height = box->width;
}


carmen_moving_objects_point_clouds_message *
fill_in_moving_objects_message(virtual_scan_track_set_t *best_track_set, virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses = NULL)
{
	carmen_moving_objects_point_clouds_message *message = (carmen_moving_objects_point_clouds_message *) malloc(sizeof(carmen_moving_objects_point_clouds_message));
	message->host = carmen_get_host();
	message->timestamp = carmen_get_time();

	int num_moving_objects = 0;
	if (best_track_set != NULL)
		for (int i = 0; i < best_track_set->size; i++)
// 			if (best_track_set->tracks[i]->size > 2)
				for (int j = 0; j < best_track_set->tracks[i]->size; j++)
	//				if (best_track_set->tracks[i]->box_model_hypothesis[j].hypothesis_points.zi == g_zi)
						num_moving_objects++;

	if (virtual_scan_box_model_hypotheses != NULL)
		for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
			for (int j = 0; j < virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes; j++)
				num_moving_objects++;

	message->point_clouds = (t_point_cloud_struct *) malloc(sizeof(t_point_cloud_struct) * num_moving_objects);
	message->num_point_clouds = num_moving_objects;

	int k = 0;
	if (best_track_set != NULL)
	{
		for (int i = 0; i < best_track_set->size; i++)
		{
// 			if (best_track_set->tracks[i]->size > 2)
			{
				for (int j = 0; j < best_track_set->tracks[i]->size; j++)
				{
	//				if (best_track_set->tracks[i]->box_model_hypothesis[j].hypothesis_points.zi == g_zi)
					{
						virtual_scan_box_model_t box = best_track_set->tracks[i]->box_model_hypothesis[j].hypothesis;

	//					box.length = (j == best_track_set->tracks[i]->size - 1) ? box.length : 0.3;
	//					box.width = (j == best_track_set->tracks[i]->size - 1) ? box.width : 0.3;
						fill_in_moving_objects_message_element(k, message, &box);

						k++;
					}
				}
			}
		}
	}

	if (virtual_scan_box_model_hypotheses != NULL)
	{
		for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
		{
			for (int j = 0; j < virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes; j++)
			{
				virtual_scan_box_model_t box = virtual_scan_box_model_hypotheses->box_model_hypotheses[i].box[j];
				box.c = 'P'; 		// coloquei que eh pedestre soh para aparecer com cor azul
				box.length *= 0.8;	// reduzi para nao desenhar em cima de um hipotese do track set
				box.width *= 0.8;	// reduzi para nao desenhar em cima de um hipotese do track set
				fill_in_moving_objects_message_element(k, message, &box);

				k++;
			}
		}
	}

	return (message);
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Publishers																			     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////


void
virtual_scan_publish_moving_objects(virtual_scan_track_set_t *track_set, virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses = NULL)
{
	carmen_moving_objects_point_clouds_message *moving_objects = fill_in_moving_objects_message(track_set, virtual_scan_box_model_hypotheses);
	carmen_moving_objects_point_clouds_publish_message(moving_objects);
	virtual_scan_free_moving_objects(moving_objects);
}


void
publish_virtual_scan_extended(carmen_mapper_virtual_scan_message *virtual_scan_extended)
{
	virtual_laser_message.host = carmen_get_host();
	virtual_laser_message.num_positions = 0;
	for (int s = 0; s < virtual_scan_extended->num_sensors; s++)
		virtual_laser_message.num_positions += virtual_scan_extended->virtual_scan_sensor[s].num_points;

	virtual_laser_message.positions = (carmen_position_t *) malloc(virtual_laser_message.num_positions * sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) malloc(virtual_laser_message.num_positions * sizeof(char));

	int j = 0;
	for (int s = 0; s < virtual_scan_extended->num_sensors; s++)
	{
		for (int i = 0; i < virtual_scan_extended->virtual_scan_sensor[s].num_points; i++)
		{
			char color = CARMEN_ORANGE;
			virtual_laser_message.positions[j].x = virtual_scan_extended->virtual_scan_sensor[s].points[i].x;
			virtual_laser_message.positions[j].y = virtual_scan_extended->virtual_scan_sensor[s].points[i].y;
			virtual_laser_message.colors[j] = color;
			j++;
		}
	}
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());

	free(virtual_laser_message.positions);
	free(virtual_laser_message.colors);
}


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
			fill_in_moving_objects_message_element(k, &message, box);
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
	static double t_ini = 0.0;
	if (t_ini == 0.0)
		t_ini = carmen_get_time();

	bool warming_up = (carmen_get_time() - t_ini) < 2.0;
	
	g_current_pos = message->virtual_scan_sensor->global_pos;
	
	if (warming_up)
		g_initial_pos = g_current_pos;

	if (necessary_maps_available && !warming_up)
	{
//		publish_virtual_scan_extended(message);

		virtual_scan_free_scan_extended(g_virtual_scan_extended[g_zi]);
		g_virtual_scan_extended[g_zi] = sort_virtual_scan(message);

		carmen_mapper_virtual_scan_message *virtual_scan_extended_filtered = filter_virtual_scan(g_virtual_scan_extended[g_zi]);
//		publish_virtual_scan_extended(virtual_scan_extended_filtered);

		virtual_scan_free_segment_classes(g_virtual_scan_segment_classes[g_zi]);
		g_virtual_scan_segment_classes[g_zi] = virtual_scan_extract_segments(virtual_scan_extended_filtered);
		virtual_scan_publish_segments(g_virtual_scan_segment_classes[g_zi]);

// 		virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses = virtual_scan_fit_box_models(g_virtual_scan_segment_classes[g_zi], message->timestamp);
//		virtual_scan_publish_box_models(virtual_scan_box_model_hypotheses);

		virtual_scan_track_set_t *track_set = virtual_scan_infer_moving_objects(g_virtual_scan_segment_classes[g_zi], message->timestamp);
		virtual_scan_publish_moving_objects(track_set, NULL);
//		virtual_scan_publish_moving_objects(NULL, virtual_scan_box_model_hypotheses);
//		virtual_scan_publish_moving_objects(track_set, virtual_scan_box_model_hypotheses);

// 		virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses);

		g_zi++;
		if (g_zi >= NUMBER_OF_FRAMES_T)
			g_zi = 0;
	}
//	printf("%lf\n", carmen_get_time() - t_ini);
}


static void
localize_map_update_handler(carmen_map_server_localize_map_message *message)
{
	carmen_map_server_localize_map_message_to_localize_map(message, &localize_map);

	x_origin = message->config.x_origin;
	y_origin = message->config.y_origin;
	x_size = message->config.x_size;
	y_size = message->config.y_size;
	map_resolution = message->config.resolution;

	necessary_maps_available = 1;
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		
		virtual_scan_tracker_finalize();

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

	memset(g_virtual_scan_extended, 0, sizeof(carmen_mapper_virtual_scan_message *) * NUMBER_OF_FRAMES_T);
	memset(g_virtual_scan_segment_classes, 0, sizeof(virtual_scan_segment_classes_t *) * NUMBER_OF_FRAMES_T);

	signal(SIGINT, shutdown_module);
	carmen_virtual_scan_subscribe_messages();
	
	virtual_scan_tracker_initialize();

	carmen_ipc_dispatch();

	return (0);
}
