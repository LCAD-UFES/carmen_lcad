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
#include <carmen/kalman.h>

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
double map_resolution = 0.0;

int necessary_maps_available = 0;

int g_zi = 0;

carmen_mapper_virtual_scan_message *g_virtual_scan_extended[NUMBER_OF_FRAMES_T];
virtual_scan_segment_classes_t *g_virtual_scan_segment_classes[NUMBER_OF_FRAMES_T];

virtual_scan_neighborhood_graph_t *g_neighborhood_graph = NULL;

carmen_point_t g_initial_pos;

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
			if (best_track_set->tracks[i]->size > 2)
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
			if (best_track_set->tracks[i]->size > 2)
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
compute_simulated_lateral_objects(simulated_moving_object_initial_state_t *moving_object_state, carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	if (!necessary_maps_available)
		return (NULL);

	carmen_rddf_road_profile_message *rddf = last_rddf_message;
	if (rddf == NULL)
		return (NULL);

	if (moving_object_state->initial_time == 0.0)
	{
		moving_object_state->returned_pose = moving_object_state->previous_pose = rddf->poses[0];
		moving_object_state->returned_pose.x = moving_object_state->previous_pose.x + moving_object_state->lateral_disp * cos(moving_object_state->previous_pose.theta + M_PI / 2.0);
		moving_object_state->returned_pose.y = moving_object_state->previous_pose.y + moving_object_state->lateral_disp * sin(moving_object_state->previous_pose.theta + M_PI / 2.0);

		moving_object_state->previous_timestamp = timestamp;
		moving_object_state->initial_time = timestamp;

		moving_object_state->returned_pose = displace_pose(moving_object_state->returned_pose, moving_object_state->logitutinal_disp);

		return (&(moving_object_state->returned_pose));
	}

	static double stop_t0 = 0;
	static double stop_t1 = 30;
	static double stop_t2 = 10;

	static double v;
	double t = timestamp - moving_object_state->initial_time;
	if ((t > stop_t0) && (t < stop_t1) && moving_object_state->lateral_disp < 3.0)
		moving_object_state->lateral_disp += 0.03;
	if ((t > stop_t1) && moving_object_state->lateral_disp > 0.0)
		moving_object_state->lateral_disp -= 0.03;
	if (t < stop_t2)
		v = current_robot_pose_v_and_phi.v + 0.6;

//	else if (t > stop_tn)
//		moving_object_state->initial_time = timestamp;

	double dt = timestamp - moving_object_state->previous_timestamp;
	double dx = v * dt * cos(moving_object_state->previous_pose.theta);
	double dy = v * dt * sin(moving_object_state->previous_pose.theta);

	carmen_ackerman_traj_point_t pose_ahead;
	pose_ahead.x = moving_object_state->previous_pose.x + dx;
	pose_ahead.y = moving_object_state->previous_pose.y + dy;

	static carmen_ackerman_traj_point_t next_pose = {0, 0, 0, 0, 0};
	for (int i = 0, n = rddf->number_of_poses - 1; i < n; i++)
	{
		int status;
		next_pose = carmen_get_point_nearest_to_trajectory(&status, rddf->poses[i], rddf->poses[i + 1], pose_ahead, 0.1);
		if ((status == POINT_WITHIN_SEGMENT) || (status == POINT_BEFORE_SEGMENT))
			break;
	}

	moving_object_state->returned_pose = moving_object_state->previous_pose = next_pose;
	moving_object_state->returned_pose.x = moving_object_state->previous_pose.x + moving_object_state->lateral_disp * cos(moving_object_state->previous_pose.theta + M_PI / 2.0);
	moving_object_state->returned_pose.y = moving_object_state->previous_pose.y + moving_object_state->lateral_disp * sin(moving_object_state->previous_pose.theta + M_PI / 2.0);
	moving_object_state->returned_pose = displace_pose(moving_object_state->returned_pose, moving_object_state->logitutinal_disp);
	moving_object_state->previous_timestamp = timestamp;

	return (&(moving_object_state->returned_pose));
}


void
draw_moving_objects_in_scan(carmen_virtual_scan_sensor_t *simulated_scan, carmen_rectangle_t *moving_objects, int num_moving_objects, carmen_pose_3D_t robot_pose)
{
	carmen_pose_3D_t world_pose = get_world_pose_with_velodyne_offset(robot_pose);
	simulated_scan->sensor_pos = {world_pose.position.x, world_pose.position.y, world_pose.orientation.yaw};
	carmen_position_t velodyne_pos = {world_pose.position.x, world_pose.position.y};

	double initial_angle = world_pose.orientation.yaw;

	int num_points = 0;
	for (double angle = -M_PI; angle < M_PI; angle += M_PI / (360 * 4.0))
		num_points++;
//	simulated_scan->num_points = num_points;
	simulated_scan->points = (carmen_point_t *) malloc(num_points * sizeof(carmen_point_t));

	int i = 0;
	for (double angle = -M_PI; angle < M_PI; angle += M_PI / (360 * 2.0))
	{
		double max_distance = 70.0;
		carmen_position_t target = {velodyne_pos.x + max_distance * cos(carmen_normalize_theta(initial_angle + angle)),
									velodyne_pos.y + max_distance * sin(carmen_normalize_theta(initial_angle + angle))};

		carmen_position_t nearest_intersection;
		bool hit_obstacle = false;
		for (int j = 0; j < num_moving_objects; j++)
		{
			carmen_position_t intersection;
			if (carmen_line_to_point_crossed_rectangle(&intersection, velodyne_pos, target, moving_objects[j]))
			{
				if (hit_obstacle)
				{
					if (DIST2D(intersection, velodyne_pos) < DIST2D(nearest_intersection, velodyne_pos))
						nearest_intersection = intersection;
				}
				else
				{
					nearest_intersection = intersection;
					hit_obstacle = true;
				}
			}
		}
		if (hit_obstacle)
		{
			simulated_scan->points[i] = {nearest_intersection.x, nearest_intersection.y, 0.0};
			i++;
		}
//		else
//			simulated_scan->points[i] = {target.x, target.y, 0.0};
//		i++;
	}
	simulated_scan->num_points = i;
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


#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
void
publish_simulated_objects(carmen_rectangle_t *simulated_moving_objects, int num_moving_objects, carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	carmen_mapper_virtual_scan_message simulated_scan;

	simulated_scan.host = carmen_get_host();
	simulated_scan.timestamp = globalpos_message->timestamp;
	simulated_scan.num_sensors = 1;

	carmen_virtual_scan_sensor_t virtual_scan_sensor;
	virtual_scan_sensor.sensor_id = 0;
	virtual_scan_sensor.v = globalpos_message->v;
	virtual_scan_sensor.phi = globalpos_message->phi;
	virtual_scan_sensor.timestamp = globalpos_message->timestamp;
	draw_moving_objects_in_scan(&virtual_scan_sensor, simulated_moving_objects, num_moving_objects, globalpos_message->pose);

	simulated_scan.virtual_scan_sensor = &virtual_scan_sensor;
	carmen_mapper_publish_virtual_scan_message(&simulated_scan, simulated_scan.timestamp);

	free(virtual_scan_sensor.points);
}
#endif
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
	if (warming_up)
		g_initial_pos = message->virtual_scan_sensor->global_pos;

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

		virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses = virtual_scan_fit_box_models(g_virtual_scan_segment_classes[g_zi], message->timestamp);
//		virtual_scan_publish_box_models(virtual_scan_box_model_hypotheses);

		g_neighborhood_graph = virtual_scan_update_neighborhood_graph(g_neighborhood_graph, virtual_scan_box_model_hypotheses);

		virtual_scan_track_set_t *track_set = virtual_scan_infer_moving_objects(g_neighborhood_graph);
//		virtual_scan_publish_moving_objects(track_set, NULL);
//		virtual_scan_publish_moving_objects(NULL, virtual_scan_box_model_hypotheses);
//		virtual_scan_publish_moving_objects(track_set, virtual_scan_box_model_hypotheses);

		virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses);

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
	map_resolution = message->config.resolution;

	necessary_maps_available = 1;
}


#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	if (!necessary_maps_available || !last_rddf_message)
		return;

//	static int val = 0;
//	if ((val++ % 8) != 0) // pula
//		return;

	carmen_ackerman_traj_point_t current_robot_pose_v_and_phi;

	current_robot_pose_v_and_phi.x = msg->globalpos.x;
	current_robot_pose_v_and_phi.y = msg->globalpos.y;
	current_robot_pose_v_and_phi.theta = msg->globalpos.theta;
	current_robot_pose_v_and_phi.v = msg->v;
	current_robot_pose_v_and_phi.phi = msg->phi;

	static simulated_moving_object_initial_state_t simulated_moving_objects_initial_state[2] =
		{{{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, 0.0, 0.0,   0.0, 12.0, 4.5, 1.5},
		 {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, 0.0, 0.0,   0.0, 12.0, 4.5, 1.5}};
	carmen_rectangle_t simulated_objects[2];
	carmen_ackerman_traj_point_t *simulated_object_pose;

	int num_moving_objects = 0;
	simulated_object_pose = compute_simulated_lateral_objects(&(simulated_moving_objects_initial_state[num_moving_objects]), current_robot_pose_v_and_phi, msg->timestamp);
	if (simulated_object_pose)
		simulated_objects[num_moving_objects] = {simulated_object_pose->x, simulated_object_pose->y, simulated_object_pose->theta,
								simulated_moving_objects_initial_state[num_moving_objects].length, simulated_moving_objects_initial_state[num_moving_objects].width};

//	num_moving_objects++;
//	simulated_object_pose = compute_simulated_lateral_objects(&(simulated_moving_objects_initial_state[num_moving_objects]), current_robot_pose_v_and_phi, msg->timestamp);
//	if (simulated_object_pose)
//		simulated_objects[num_moving_objects] = {simulated_object_pose->x, simulated_object_pose->y, simulated_object_pose->theta,
//								simulated_moving_objects_initial_state[num_moving_objects].length, simulated_moving_objects_initial_state[num_moving_objects].width};

	if (simulated_object_pose)
		publish_simulated_objects(simulated_objects, num_moving_objects + 1, msg);
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

	memset(g_virtual_scan_extended, 0, sizeof(carmen_mapper_virtual_scan_message *) * NUMBER_OF_FRAMES_T);
	memset(g_virtual_scan_segment_classes, 0, sizeof(virtual_scan_segment_classes_t *) * NUMBER_OF_FRAMES_T);

	signal(SIGINT, shutdown_module);
	carmen_virtual_scan_subscribe_messages();

	carmen_ipc_dispatch();

	return (0);
}
