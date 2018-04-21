#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/map_server_interface.h>
#include <carmen/map.h>
#include <carmen/grid_mapping.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/collision_detection.h>

#include "virtual_scan.h"

#define SIMULATE_LATERAL_MOVING_OBSTACLE
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

#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
#define MAX_VIRTUAL_LASER_SAMPLES 10000
static carmen_rddf_road_profile_message *last_rddf_message = NULL;
#endif


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
			model_features.red = 1.0;
			model_features.green = 1.0;
			model_features.blue = 1.0;
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
	if (best_track_set == NULL)
		return (NULL);

	carmen_moving_objects_point_clouds_message *message = (carmen_moving_objects_point_clouds_message *) malloc(sizeof(carmen_moving_objects_point_clouds_message));
	message->host = carmen_get_host();
	message->timestamp = carmen_get_time();

	int num_moving_objects = 0;
	for (int i = 0; i < best_track_set->size; i++)
		if (best_track_set->tracks[i]->size > 2)
			for (int j = 0; j < best_track_set->tracks[i]->size; j++)
//				if (best_track_set->tracks[i]->box_model_hypothesis[j].zi == g_zi)
					num_moving_objects++;

	if (virtual_scan_box_model_hypotheses != NULL)
		for (int i = 0; i < virtual_scan_box_model_hypotheses->num_box_model_hypotheses; i++)
			for (int j = 0; j< virtual_scan_box_model_hypotheses->box_model_hypotheses[i].num_boxes; j++)
				num_moving_objects++;

	message->point_clouds = (t_point_cloud_struct *) malloc(sizeof(t_point_cloud_struct) * num_moving_objects);
	message->num_point_clouds = num_moving_objects;

	int k = 0;
	for (int i = 0; i < best_track_set->size; i++)
	{
		if (best_track_set->tracks[i]->size > 2)
		{
			for (int j = 0; j < best_track_set->tracks[i]->size; j++)
			{
	//			if (best_track_set->tracks[i]->box_model_hypothesis[j].zi == g_zi)
				{
					virtual_scan_box_model_t box = best_track_set->tracks[i]->box_model_hypothesis[j].hypothesis;

					box.length = (j == best_track_set->tracks[i]->size - 1) ? box.length : 0.1;
					box.width = (j == best_track_set->tracks[i]->size - 1) ? box.width : 0.1;
					fill_in_moving_objects_message_element(k, message, &box);

					k++;
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


#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
carmen_ackerman_traj_point_t
displace_pose(carmen_ackerman_traj_point_t robot_pose, double displacement)
{
	carmen_point_t displaced_robot_position = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&robot_pose, displacement);

	carmen_ackerman_traj_point_t displaced_robot_pose = robot_pose;
	displaced_robot_pose.x = displaced_robot_position.x;
	displaced_robot_pose.y = displaced_robot_position.y;

	return (displaced_robot_pose);
}


carmen_ackerman_traj_point_t *
compute_simulated_lateral_objects(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	if (!necessary_maps_available)
		return (NULL);

	carmen_rddf_road_profile_message *rddf = last_rddf_message;
	if (rddf == NULL)
		return (NULL);

	static carmen_ackerman_traj_point_t previous_pose = {0, 0, 0, 0, 0};
	static carmen_ackerman_traj_point_t returned_pose = {0, 0, 0, 0, 0};
	static double previous_timestamp = 0.0;
	static double initial_time = 0.0; // Simulation start time.
	static double disp = 0.0;

	if (initial_time == 0.0)
	{
		returned_pose = previous_pose = rddf->poses[0];
		returned_pose.x = previous_pose.x + disp * cos(previous_pose.theta + M_PI / 2.0);
		returned_pose.y = previous_pose.y + disp * sin(previous_pose.theta + M_PI / 2.0);

		previous_timestamp = timestamp;
		initial_time = timestamp;

		return (&returned_pose);
	}

	static double stop_t0 = 0;
	static double stop_t1 = 30;
	static double stop_t2 = 10;

	static double v;
	double t = timestamp - initial_time;
	if ((t > stop_t0) && (t < stop_t1) && disp < 3.0)
		disp += 0.03;
	if ((t > stop_t1) && disp > 0.0)
		disp -= 0.03;
	if (t < stop_t2)
		v = current_robot_pose_v_and_phi.v + 0.6;

//	else if (t > stop_tn)
//		initial_time = timestamp;

	double dt = timestamp - previous_timestamp;
	double dx = v * dt * cos(previous_pose.theta);
	double dy = v * dt * sin(previous_pose.theta);

	carmen_ackerman_traj_point_t pose_ahead;
	pose_ahead.x = previous_pose.x + dx;
	pose_ahead.y = previous_pose.y + dy;

	static carmen_ackerman_traj_point_t next_pose = {0, 0, 0, 0, 0};
	for (int i = 0, n = rddf->number_of_poses - 1; i < n; i++)
	{
		int status;
		next_pose = carmen_get_point_nearest_to_trajectory(&status, rddf->poses[i], rddf->poses[i + 1], pose_ahead, 0.1);
		if ((status == POINT_WITHIN_SEGMENT) || (status == POINT_BEFORE_SEGMENT))
			break;
	}

	returned_pose = previous_pose = next_pose;
	returned_pose.x = previous_pose.x + disp * cos(previous_pose.theta + M_PI / 2.0);
	returned_pose.y = previous_pose.y + disp * sin(previous_pose.theta + M_PI / 2.0);
	returned_pose = displace_pose(returned_pose, -12.0);
	previous_timestamp = timestamp;

	return (&returned_pose);
}


void
draw_moving_object_in_scan(carmen_mapper_virtual_scan_message *simulated_scan, carmen_ackerman_traj_point_t *simulated_object_pose)
{
	carmen_pose_3D_t world_pose = {{simulated_scan->globalpos.x, simulated_scan->globalpos.y, 0.0}, {0.0, 0.0, simulated_scan->globalpos.theta}};
	world_pose = get_world_pose_with_velodyne_offset(world_pose);
	carmen_position_t velodyne_pos = {world_pose.position.x, world_pose.position.y};

	double initial_angle = world_pose.orientation.yaw;

	carmen_rectangle_t rectangle = {simulated_object_pose->x, simulated_object_pose->y, simulated_object_pose->theta, 4.0, 1.5};

	int num_points = 0;
	for (double angle = -M_PI; angle < M_PI; angle += M_PI / (360 * 4.0))
		num_points++;
	simulated_scan->num_points = num_points;
	simulated_scan->points = (carmen_position_t *) malloc(num_points * sizeof(carmen_position_t));

	int i = 0;
	for (double angle = -M_PI; angle < M_PI; angle += M_PI / (360 * 4.0))
	{
		double max_distance = 20.0;
		carmen_position_t target = {velodyne_pos.x + max_distance * cos(carmen_normalize_theta(initial_angle + angle)),
									velodyne_pos.y + max_distance * sin(carmen_normalize_theta(initial_angle + angle))};
		carmen_position_t intersection;
		if (carmen_line_to_point_crossed_rectangle(&intersection, velodyne_pos, target, rectangle))
			simulated_scan->points[i] = intersection;
		else
			simulated_scan->points[i] = target;
		i++;
	}
}
#endif
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

	if (moving_objects != NULL)
		carmen_moving_objects_point_clouds_publish_message(moving_objects);

	virtual_scan_free_moving_objects(moving_objects);
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


void
publish_simulated_objects(carmen_ackerman_traj_point_t *simulated_object_pose, carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	carmen_mapper_virtual_scan_message simulated_scan;

	simulated_scan.host = carmen_get_host();
	simulated_scan.globalpos = globalpos_message->globalpos;
	simulated_scan.v = globalpos_message->v;
	simulated_scan.phi = globalpos_message->phi;
	simulated_scan.timestamp = globalpos_message->timestamp;

	draw_moving_object_in_scan(&simulated_scan, simulated_object_pose);
	carmen_mapper_publish_virtual_scan_message(&simulated_scan, simulated_scan.timestamp);
	free(simulated_scan.points);
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

		virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses = virtual_scan_fit_box_models(g_virtual_scan_segment_classes[g_zi]);
		virtual_scan_publish_box_models(virtual_scan_box_model_hypotheses);

//		g_neighborhood_graph = virtual_scan_update_neighborhood_graph(g_neighborhood_graph, virtual_scan_box_model_hypotheses);
//
//		virtual_scan_track_set_t *track_set = virtual_scan_infer_moving_objects(g_neighborhood_graph);
////		virtual_scan_publish_moving_objects(track_set, NULL);
//		virtual_scan_publish_moving_objects(track_set, virtual_scan_box_model_hypotheses);

		virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses);

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


#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	if (!necessary_maps_available || !last_rddf_message)
		return;

	carmen_ackerman_traj_point_t current_robot_pose_v_and_phi;

	current_robot_pose_v_and_phi.x = msg->globalpos.x;
	current_robot_pose_v_and_phi.y = msg->globalpos.y;
	current_robot_pose_v_and_phi.theta = msg->globalpos.theta;
	current_robot_pose_v_and_phi.v = msg->v;
	current_robot_pose_v_and_phi.phi = msg->phi;

	carmen_ackerman_traj_point_t *simulated_object_pose2 = compute_simulated_lateral_objects(current_robot_pose_v_and_phi, msg->timestamp);
	if (simulated_object_pose2)
		publish_simulated_objects(simulated_object_pose2, msg);
}


static void
rddf_handler(carmen_rddf_road_profile_message *rddf_msg)
{
	if (!necessary_maps_available)
		return;

	last_rddf_message = rddf_msg;
}
#endif


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

#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_road_profile_message(NULL, (carmen_handler_t) rddf_handler, CARMEN_SUBSCRIBE_LATEST);
#endif
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
