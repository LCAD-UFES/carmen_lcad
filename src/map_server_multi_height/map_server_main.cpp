#include <carmen/carmen.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/grid_mapping.h>
#include "map_server_messages.h"
#include "map_server_interface.h"
#include <carmen/carmen_rrt_util.h>
#include <carmen/localize_ackerman_core.h>
#include <carmen/localize_ackerman_likelihood_map.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <carmen/rddf_interface.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <carmen/download_map_interface.h>
#include <carmen/navigator_astar_interface.h>
#include <carmen/navigator_spline_interface.h>
#include <vector>

//TODO: ler da lista de parâmetros se for ler imagens já baixadas
#define google_maps_data_location "../data/google_maps"

static double initial_waiting_time;
static carmen_map_t *current_map;
static carmen_map_t *current_sum_remission_map;
static carmen_map_t *current_mean_remission_map;
static carmen_map_t *current_variance_remission_map;
static carmen_map_t *current_sum_sqr_remission_map;
static carmen_map_t *current_count_remission_map;
static carmen_map_t *current_road_map;
static double distance_to_update_lane_map = 1.0;
//static carmen_map_t *current_google_map;
static carmen_map_t height_maps[5];
static int max_height_level = 0;


static char *map_path;
static char *map_file_name = NULL;
static double last_time_changed = 0.0;
//static double last_time_changed_google_map = 0.0;
static double time_interval_for_map_change = 0.3;

static carmen_localize_ackerman_map_t localize_map;
static carmen_localize_ackerman_param_t localize_param;

static int block_map = 1;
static double map_grid_res;
static double map_width;
static double map_height;
static int initial_map_x;
static int initial_map_y;

static int publish_grid_mapping_map_at_startup = 0;
static int publish_google_map = 0;
static int lanemap_incoming_message_type = 0;
static int goal_source_path_planner = 0;
static int rddf_source_tracker = 0;
static carmen_point_t pose_g;

int is_first_rddf_message = 1;
carmen_rddf_road_profile_message *rddf_message = NULL;
carmen_robot_ackerman_config_t car_config;

int offline_map_published = 0;
carmen_map_t lane_map_g = {{0, 0, 0, "", NULL, 0, 0}, NULL, NULL};
carmen_compact_map_t compacted_lane_map_g = {0, NULL, NULL, NULL, {0, 0, 0, "", NULL, 0, 0}};

int use_truepos = 0;

double lane_width = 0.0;

//static void
//download_map_handler(carmen_download_map_message *message){
//
//	if (message != NULL){
//
//			int i, j;
//			double x_origin, y_origin;
//
//			if (current_google_map->complete_map == NULL || current_google_map->map == NULL){
//
//				//TODO: current_google_map->config.resolution
//				carmen_grid_mapping_initialize_map(current_google_map, message->width, 0.3);
//				current_google_map->config.map_name = "google_maps";
//				strcpy(current_google_map->config.origin, "from_google");
//			}
//
//			carmen_grid_mapping_get_map_origin(&pose_g, &x_origin, &y_origin);
//
//			current_google_map->config.x_origin = x_origin;
//			current_google_map->config.y_origin = y_origin;
//
//			for (i = 0; i < message->height; i++){
//				for (j = 0; j < message->width; j++){
//					current_google_map->map[message->height - i - 1][message->width - j -1] = (unsigned char)message->image_data[3 * (i * message->width + j)];
//					current_google_map->map[message->height - i - 1][message->width - j -1] /= 255.0;
//				}
//			}
//		}
//		else{
//			printf("error\n");
//		}
//}
//
//static void
//read_google_maps_image(carmen_map_t* current_google_map,double x_origin, double y_origin)
//{
//
//	char full_image_path[100];
////	static int count = 0;
//
//	sprintf(full_image_path, "%s/%c%d_%d.bmp", google_maps_data_location, 'm', (int)x_origin, (int)y_origin);
//
//	IplImage* img = cvLoadImage(full_image_path, CV_LOAD_IMAGE_COLOR);
//	if (img == NULL)
//		return;
////	sprintf(full_image_path, "%s/%c%d.bmp", google_maps_data_location, 'a', count);
////	cvSaveImage(full_image_path,img,0);
////	count++;
//	IplImage* img_gray = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
////	IplImage* img_gray2 = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
//
//	cvCvtColor(img, img_gray, CV_RGB2GRAY);
////	cvSmooth(img_gray, img_gray, CV_GAUSSIAN, 7, 7, 7, 7);
////	cvEqualizeHist(img_gray, img_gray);
//	//
//	//cvShowImage("1", img_gray2);
////	cvCanny(img_gray2, img_gray, 0, 255, 3);
////	cvShowImage("1", img_gray);
////	cvWaitKey(33);
//
////	printf("x_size: %d y_size: %d\n", current_google_map->config.x_size, current_google_map->config.y_size);
//
//	if (img != NULL)
//	{
//	//	printf("success in %s\n", full_image_path);
//
//		int i, j;
//
//		if (current_google_map->complete_map == NULL || current_google_map->map == NULL){
//
//			carmen_grid_mapping_initialize_map(current_google_map, img->width, 0.3);
//			current_google_map->config.map_name = "google_maps";
//			strcpy(current_google_map->config.origin, "from_google");
//		}
//
//		current_google_map->config.x_origin = x_origin;
//		current_google_map->config.y_origin = y_origin;
//
//		printf("x_size: %d\n",current_google_map->config.y_size);
//
//		for (i = 0; i < current_google_map->config.y_size; i++)
//		{
//			for (j = 0; j < current_google_map->config.x_size; j++)
//			{
//				current_google_map->map[current_google_map->config.y_size - i - 1][current_google_map->config.x_size - j - 1] = (unsigned char)img_gray->imageData[i * img_gray->widthStep + j];
//				current_google_map->map[current_google_map->config.y_size - i - 1][current_google_map->config.x_size - j - 1] /= 255.0;
//				//current_google_map->map[current_google_map->config.y_size - i - 1][current_google_map->config.x_size - j - 1] = 1.0 - current_google_map->map[current_google_map->config.y_size - i - 1][current_google_map->config.x_size - j - 1];
//				//current_google_map->map[current_google_map->config.y_size - i - 1][current_google_map->config.x_size - j - 1] -= 1.0;
//
//			}
//		}
//	}
//	else
//	{
////		printf("error in %s\n", full_image_path);
//	}
//
//	cvReleaseImage(&img);
//	cvReleaseImage(&img_gray);
//}


static int
is_valid_position(int x, int y, carmen_map_config_t map_config)
{
	return (x >= 0 && x < map_config.x_size && y >= 0 && y < map_config.y_size);
}


static void
to_map_pose(double x, double y, carmen_map_config_t *map_config, double *xout, double *yout)
{
	*xout = round((x - map_config->x_origin) / map_config->resolution);
	*yout = round((y - map_config->y_origin) / map_config->resolution);
}


// TODO: se nao fizer falta, pode apagar!
//std::vector<carmen_vector_2D_t>
//get_lane_points_on_map(carmen_rddf_road_profile_message *message, carmen_map_t *map)
//{
//	int i, j, distance;
//	double distance_p1p2_map;
//	double theta, delta_x, delta_y;
//	carmen_vector_2D_t p1_map, p2_map;
//	carmen_ackerman_traj_point_t p1, p2;
//	std::vector<carmen_vector_2D_t> lane_points_on_map;
//
//	for (i = 0; i < (message->number_of_poses - 1); i++)
//	{
//		p1 = message->poses[i];
//		p2 = message->poses[i + 1];
//
//		to_map_pose(p1.x, p1.y, &(map->config), &(p1_map.x), &(p1_map.y));
//		to_map_pose(p2.x, p2.y, &(map->config), &(p2_map.x), &(p2_map.y));
//
//		// TODO: check if the map->config copy isn't slowing down the program
//		if (is_valid_position(p1_map.x, p1_map.y, map->config) && is_valid_position(p2_map.x, p2_map.y, map->config))
//			continue;
//
//		theta = carmen_normalize_theta(atan2(p2_map.y - p1_map.y, p2_map.x - p1_map.x));
//		distance_p1p2_map = sqrt(pow(p1_map.x - p2_map.x, 2) + pow(p1_map.y - p2_map.y, 2));
//
//		delta_x = cos(theta);
//		delta_y = sin(theta);
//		distance = ceil(distance_p1p2_map);
//
//		for(j = 0; j < distance * 5.0; j++)
//		{
//			if(!is_valid_position(p1_map.x, p1_map.y, map->config))
//				break;
//
//			p1_map.x += (delta_x / 5.0);
//			p1_map.y += (delta_y / 5.0);
//
//			lane_points_on_map.push_back(p1_map);
//		}
//	}
//
//	return lane_points_on_map;
//}

static void
add_points_interval_to_the_map(carmen_map_t *lane_map, double lane_probability, carmen_ackerman_traj_point_t p1, carmen_ackerman_traj_point_t p2)
{
//	std::vector<carmen_vector_2D_t> lane_points_on_map;
	carmen_vector_2D_t p1_map, p2_map;
	double theta, delta_x, delta_y;
	double distance_p1p2_map;
	int i, distance;

	// Desloca para o eixo dianteiro do carro: era necessario para o rrt planner, mas nao para o mpp
//	p1.x += (car_config.distance_between_front_and_rear_axles * cos(p1.theta));
//	p1.y += (car_config.distance_between_front_and_rear_axles * sin(p1.theta));
//
//	p2.x += (car_config.distance_between_front_and_rear_axles * cos(p2.theta));
//	p2.y += (car_config.distance_between_front_and_rear_axles * sin(p2.theta));

	to_map_pose(p1.x, p1.y, &(lane_map->config), &(p1_map.x), &(p1_map.y));
	to_map_pose(p2.x, p2.y, &(lane_map->config), &(p2_map.x), &(p2_map.y));

	if (!is_valid_position(p1_map.x, p1_map.y, lane_map->config) || !is_valid_position(p2_map.x, p2_map.y, lane_map->config))
		return;

	theta = carmen_normalize_theta(atan2(p2_map.y - p1_map.y, p2_map.x - p1_map.x));
	distance_p1p2_map = sqrt(pow(p1_map.x - p2_map.x, 2) + pow(p1_map.y - p2_map.y, 2));

	delta_x = cos(theta);
	delta_y = sin(theta);
	distance = ceil(distance_p1p2_map);

	for(i = 0; i < distance * 2.0; i++)
	{
		p1_map.x += (delta_x / 2.0);
		p1_map.y += (delta_y / 2.0);

		int x = (int) round(p1_map.x);
		int y = (int) round(p1_map.y);
		if (is_valid_position(x, y, lane_map->config))
			lane_map->complete_map[x * lane_map->config.y_size + y] = lane_probability;
	}

}


// TODO: refactor! eu copiei as linhas que fazem a lane para frente para fazer a criacao da lane para tras (com pequenas alteracoes).
static void
add_lane_to_the_map(carmen_map_t *lane_map, carmen_rddf_road_profile_message *message, double lane_probability)
{
	int i;

	// foward direction
	for (i = 0; i < (message->number_of_poses - 1); i++)
		add_points_interval_to_the_map(lane_map, lane_probability, message->poses[i], message->poses[i + 1]);

	// backward direction
	for (i = 0; i < (message->number_of_poses_back - 1); i++)
		add_points_interval_to_the_map(lane_map, lane_probability, message->poses_back[i], message->poses_back[i + 1]);

	add_points_interval_to_the_map(lane_map, lane_probability, message->poses[0], message->poses_back[0]); //todo verificar se é necessario mesmo (conectar a parte de tras com a da frente)
}


static void
build_lane_map(carmen_rddf_road_profile_message *message, carmen_map_t *lane_map)
{
	carmen_prob_models_initialize_cost_map(lane_map, current_map->config, current_map->config.resolution);
	memset(lane_map->complete_map, 0, lane_map->config.x_size * lane_map->config.y_size * sizeof(double));

	add_lane_to_the_map(lane_map, message, 1.0);

	carmen_prob_models_convert_obstacles_map_to_cost_map(lane_map, lane_map, 0.5, lane_width, 1);
}


static void
construct_compressed_lane_map()
{
	if (rddf_message != NULL)
	{
		build_lane_map(rddf_message, &lane_map_g);

		// Nota: o compacted map deve ser desalocado e alocado novamente sempre que o lane map eh construido porque
		// o numero de celulas do mapa que fazem parte da lane sempre vai mudar
		if ((compacted_lane_map_g.coord_x != NULL) && ((compacted_lane_map_g.coord_y != NULL)) && ((compacted_lane_map_g.value != NULL)))
			carmen_prob_models_free_compact_map(&compacted_lane_map_g);

		carmen_prob_models_create_compact_map(&compacted_lane_map_g, &lane_map_g, 1.0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_compressed_lane_map()
{
	if ((compacted_lane_map_g.coord_x != NULL) && ((compacted_lane_map_g.coord_y != NULL)) && ((compacted_lane_map_g.value != NULL)))
	{
		// Nota: o timestamp da mensagem eh o tempo do rddf usado para construir o mapa. Nao sei se eh a melhor opcao...
		carmen_map_server_publish_compact_lane_map_message(&compacted_lane_map_g, rddf_message->timestamp);
	}
}


static void
publish_a_new_offline_map_if_robot_moved_to_another_block(carmen_point_t *pose, double timestamp)
{
	double x_origin, y_origin, time_now;

	if (map_file_name)
		return;

	carmen_grid_mapping_get_map_origin(pose, &x_origin, &y_origin);

	time_now = carmen_get_time();

	if ((time_now - last_time_changed) > time_interval_for_map_change && (current_map->config.x_origin != x_origin || current_map->config.y_origin != y_origin))
	{
		last_time_changed = time_now;

		carmen_grid_mapping_get_block_map_by_origin(map_path, 'm', *pose, current_map);
		carmen_grid_mapping_get_block_map_by_origin(map_path, 's', *pose, current_sum_remission_map);
		carmen_grid_mapping_get_block_map_by_origin(map_path, '2', *pose, current_sum_sqr_remission_map);
		carmen_grid_mapping_get_block_map_by_origin(map_path, 'c', *pose, current_count_remission_map);
		carmen_grid_mapping_get_block_map_by_origin(map_path, 'r', *pose, current_road_map);
		for (int i=0; i < max_height_level; i++)
			carmen_grid_mapping_get_block_map_by_origin(map_path, 'l'-i, *pose, &height_maps[i]);

		if (current_map->complete_map != NULL)
			carmen_prob_models_calc_mean_and_variance_remission_map(current_mean_remission_map, current_variance_remission_map, current_sum_remission_map, current_sum_sqr_remission_map, current_count_remission_map);

		carmen_to_localize_ackerman_map(current_map, current_mean_remission_map, current_variance_remission_map, &localize_map, &localize_param);
//		carmen_map_t temp_map;
//		temp_map.config = localize_map.config;
//		temp_map.complete_map = localize_map.complete_prob;
//		temp_map.map = localize_map.prob;
//		carmen_grid_mapping_save_map((char *) "test.map", &temp_map);

		strcpy(current_map->config.origin, "from_param_daemon");
		carmen_map_server_publish_offline_map_message(current_map, timestamp);
		for (int i=0; i < max_height_level; i++)
			carmen_map_server_publish_offline_multi_height_map_message(&height_maps[i], timestamp, i+1);
		carmen_map_server_publish_road_map_message(current_road_map, timestamp);
		offline_map_published = 1;
		construct_compressed_lane_map();
		publish_compressed_lane_map();
		carmen_map_server_publish_localize_map_message(&localize_map);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


static void
alloc_rddf_global_data(carmen_behavior_selector_road_profile_message *message)
{
	rddf_message = (carmen_rddf_road_profile_message *) calloc (1, sizeof(carmen_rddf_road_profile_message));

	rddf_message->annotations = (int *) calloc (message->number_of_poses, sizeof(int));
	rddf_message->poses = (carmen_ackerman_traj_point_t *) calloc (message->number_of_poses, sizeof(carmen_ackerman_traj_point_t));
	rddf_message->number_of_poses = message->number_of_poses;

	rddf_message->poses_back = (carmen_ackerman_traj_point_t *) calloc (message->number_of_poses_back, sizeof(carmen_ackerman_traj_point_t));
	rddf_message->number_of_poses_back = message->number_of_poses_back;
}


static void
realloc_rddf_global_data(carmen_behavior_selector_road_profile_message *message)
{
	if (message->number_of_poses != rddf_message->number_of_poses)
	{
		rddf_message->annotations = (int *) realloc (rddf_message->annotations, message->number_of_poses * sizeof(int));
		rddf_message->poses = (carmen_ackerman_traj_point_t *) realloc (rddf_message->poses, message->number_of_poses * sizeof(carmen_ackerman_traj_point_t));
	}

	if (message->number_of_poses_back != rddf_message->number_of_poses_back)
	{
		rddf_message->poses_back = (carmen_ackerman_traj_point_t *) realloc (rddf_message->poses_back, message->number_of_poses_back * sizeof(carmen_ackerman_traj_point_t));
	}

	rddf_message->number_of_poses = message->number_of_poses;
	rddf_message->number_of_poses_back = message->number_of_poses_back;
}


static void
copy_local_rddf_to_global_rddf(carmen_behavior_selector_road_profile_message *message)
{
	memcpy(rddf_message->annotations, message->annotations, message->number_of_poses * sizeof(int));
	memcpy(rddf_message->poses, message->poses, message->number_of_poses * sizeof(carmen_ackerman_traj_point_t));
	memcpy(rddf_message->poses_back, message->poses_back, message->number_of_poses_back * sizeof(carmen_ackerman_traj_point_t));
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	static int first_time = 1;
	carmen_point_t pose;

	pose = msg->globalpos;

	//TODO:
	pose_g = pose;

	publish_a_new_offline_map_if_robot_moved_to_another_block(&pose, msg->timestamp);

	// Force offline map publication when the first pose is received
	if (first_time)
	{
		if (current_map->complete_map != NULL)
			carmen_map_server_publish_offline_map_message(current_map, msg->timestamp);

		first_time = 0;
	}
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	static int first_time = 1;
	carmen_point_t pose;

	pose = msg->truepose;

	//TODO:
	pose_g = pose;

	publish_a_new_offline_map_if_robot_moved_to_another_block(&pose, msg->timestamp);

	// Force offline map publication when the first pose is received
	if (first_time)
	{
		if (current_map->complete_map != NULL)
			carmen_map_server_publish_offline_map_message(current_map, msg->timestamp);

		first_time = 0;
	}
}


static void
localize_ackerman_initialize_message_handler(carmen_localize_ackerman_initialize_message *msg)
{
	publish_a_new_offline_map_if_robot_moved_to_another_block(msg->mean, msg->timestamp);
}


static void
rddf_message_handler(carmen_behavior_selector_road_profile_message *message)
{
	static carmen_point_t pose_in_last_publish = {0.0, 0.0, 0.0};

	if (goal_source_path_planner || rddf_source_tracker)
	{
		if (message->number_of_poses <= 0)
			return;
	}

	else if (message->number_of_poses <= 0 || message->number_of_poses_back <= 0)
			return;

	if (is_first_rddf_message)
	{
		alloc_rddf_global_data(message);
		is_first_rddf_message = 0;
	}
	else
	{
		if (goal_source_path_planner || rddf_source_tracker)
		{
			if ((message->number_of_poses != rddf_message->number_of_poses))
				realloc_rddf_global_data(message);
		}

		else if ((message->number_of_poses != rddf_message->number_of_poses) ||
				(message->number_of_poses_back != rddf_message->number_of_poses_back))
			realloc_rddf_global_data(message);

		copy_local_rddf_to_global_rddf(message);

		double distance_without_lane_map;

		distance_without_lane_map =
				sqrt(pow(pose_g.x - pose_in_last_publish.x, 2) +
						pow(pose_g.y - pose_in_last_publish.y, 2));

		if ((distance_without_lane_map > distance_to_update_lane_map) || (offline_map_published))
		{
			construct_compressed_lane_map();
			publish_compressed_lane_map();
			pose_in_last_publish = pose_g;
			offline_map_published = 0;
		}
	}
}


static void
map_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	carmen_mapper_map_message map_msg;
	IPC_RETURN_TYPE err;

	if (current_map->complete_map != NULL)
	{
		IPC_freeByteArray(callData);

		map_msg.config = current_map->config;
		map_msg.complete_map = current_map->complete_map;
		map_msg.size = current_map->config.x_size * current_map->config.y_size;
		map_msg.host = carmen_get_host();
		map_msg.timestamp = carmen_get_time();

		err = IPC_respondData(msgRef, CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_NAME, &map_msg);
		carmen_test_ipc(err, "Could not respond", CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_NAME);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initialization                                                                            //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
read_localize_parameters(int argc, char **argv)
{
	double integrate_angle_deg;

	integrate_angle_deg = 1.0;

	carmen_param_t param_list[] =
	{
			{"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE, &localize_param.front_laser_offset, 0, NULL},
			{"robot", "rearlaser_offset", CARMEN_PARAM_DOUBLE, &localize_param.rear_laser_offset, 0, NULL},
			{"localize", "use_rear_laser", CARMEN_PARAM_ONOFF, &localize_param.use_rear_laser, 0, NULL},
			{"localize", "num_particles", CARMEN_PARAM_INT, &localize_param.num_particles, 0, NULL},
			{"localize", "laser_max_range", CARMEN_PARAM_DOUBLE, &localize_param.max_range, 1, NULL},
			{"localize", "min_wall_prob", CARMEN_PARAM_DOUBLE, &localize_param.min_wall_prob, 0, NULL},
			{"localize", "outlier_fraction", CARMEN_PARAM_DOUBLE, &localize_param.outlier_fraction, 0, NULL},
			{"localize", "update_distance", CARMEN_PARAM_DOUBLE, &localize_param.update_distance, 0, NULL},
			{"localize", "integrate_angle_deg", CARMEN_PARAM_DOUBLE, &integrate_angle_deg, 0, NULL},
			{"localize", "do_scanmatching", CARMEN_PARAM_ONOFF, &localize_param.do_scanmatching, 1, NULL},
			{"localize", "constrain_to_map", CARMEN_PARAM_ONOFF, &localize_param.constrain_to_map, 1, NULL},
			{"localize", "occupied_prob", CARMEN_PARAM_DOUBLE, &localize_param.occupied_prob, 0, NULL},
			{"localize", "global_evidence_weight", CARMEN_PARAM_DOUBLE, &localize_param.global_evidence_weight, 0, NULL},
			{"localize", "global_distance_threshold", CARMEN_PARAM_DOUBLE, &localize_param.global_distance_threshold, 1, NULL},
			{"localize", "global_test_samples", CARMEN_PARAM_INT, &localize_param.global_test_samples, 1, NULL},
			{"localize", "use_sensor", CARMEN_PARAM_ONOFF, &localize_param.use_sensor, 0, NULL},
			{"localize", "use_log_odds", CARMEN_PARAM_ONOFF, &localize_param.use_log_odds, 0, NULL},
			{"localize", "lmap_std", CARMEN_PARAM_DOUBLE, &localize_param.lmap_std, 0, NULL},
			{"localize", "tracking_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &localize_param.tracking_beam_minlikelihood, 0, NULL},
			{"localize", "tracking_beam_maxlikelihood", CARMEN_PARAM_DOUBLE, &localize_param.tracking_beam_maxlikelihood, 0, NULL},
			{"localize", "global_lmap_std", CARMEN_PARAM_DOUBLE, &localize_param.global_lmap_std, 0, NULL},
			{"localize", "global_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &localize_param.global_beam_minlikelihood, 0, NULL},
			{"localize", "global_beam_maxlikelihood", CARMEN_PARAM_DOUBLE, &localize_param.global_beam_maxlikelihood, 0, NULL},
			{(char*)"robot",  (char*)"distance_between_front_car_and_front_wheels", 	CARMEN_PARAM_DOUBLE, &(car_config.distance_between_front_car_and_front_wheels), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &(car_config.distance_between_front_and_rear_axles), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_rear_car_and_rear_wheels",		CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rear_car_and_rear_wheels), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_rear_wheels",			CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rear_wheels), 1, NULL},
			{(char *) "behavior_selector",   (char *) "goal_source_path_planner", 		CARMEN_PARAM_ONOFF,  &goal_source_path_planner, 0, NULL},
			{(char *) "rddf",   (char *) "source_tracker", 		CARMEN_PARAM_ONOFF,  &rddf_source_tracker, 0, NULL}
	};


	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	localize_param.integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);
}


static void
read_map_server_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
		{"map_server", "initial_waiting_time", 					CARMEN_PARAM_DOUBLE, &initial_waiting_time, 0, NULL},
		{"map_server", "map_grid_res", 							CARMEN_PARAM_DOUBLE, &map_grid_res, 0, NULL},
		{"map_server", "map_width", 							CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
		{"map_server", "map_height", 							CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},
		{"map_server", "time_interval_for_map_change", 			CARMEN_PARAM_DOUBLE, &time_interval_for_map_change, 0, NULL},
		{"map_server", "publish_google_map", 					CARMEN_PARAM_ONOFF, &publish_google_map, 1, NULL},
		{"behavior_selector", "use_truepos", 					CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL},
		{"model_predictive_planner", "obstacles_safe_distance",	CARMEN_PARAM_DOUBLE, &lane_width, 1, NULL},
		{"mapper", "height_max_level",							CARMEN_PARAM_INT, &max_height_level, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


static void
read_optional_map_server_parameters(int argc, char **argv)
{
	//default parameters
	block_map = 0;

	carmen_param_allow_unfound_variables(1);

	carmen_param_t optional_param_list[] = {
			{"commandline", "block_map", CARMEN_PARAM_ONOFF, &block_map, 0, NULL},
			{"commandline", "map_x", CARMEN_PARAM_INT, &initial_map_x, 0, NULL},
			{"commandline", "map_y", CARMEN_PARAM_INT, &initial_map_y, 0, NULL},
			{"commandline", "map_path", CARMEN_PARAM_STRING, &map_path, 0, NULL},
			{"commandline", "map", CARMEN_PARAM_STRING, &map_file_name, 0, NULL},
			{"commandline", "publish_grid_mapping_map_at_startup", CARMEN_PARAM_ONOFF, &publish_grid_mapping_map_at_startup, 0, NULL},
			{"commandline", "lanemap_incoming_message_type", CARMEN_PARAM_INT, &lanemap_incoming_message_type, 0, NULL} // 0 - road_profile (RDDF), 1 - astar, 2 - spline
	};

	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
}


static void
read_parameters(int argc, char **argv)
{
	read_map_server_parameters(argc, argv);
	read_localize_parameters(argc, argv);
	read_optional_map_server_parameters(argc, argv);
}


static void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_NAME);

	err = IPC_defineMsg(CARMEN_NAVIGATOR_SPLINE_PATH_NAME, IPC_VARIABLE_LENGTH, CARMEN_NAVIGATOR_SPLINE_PATH_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_SPLINE_PATH_NAME);

	carmen_mapper_define_messages();
	carmen_map_server_define_offline_map_message();
	carmen_map_server_define_offline_map_level1_message();
	carmen_map_server_define_road_map_message();
	// carmen_map_server_define_cost_map_message();
	carmen_map_server_define_compact_lane_map_message();
}


static void
register_handlers()
{
	IPC_RETURN_TYPE err;

	if (!use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	err = IPC_subscribe(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME, map_request_handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME);

	IPC_setMsgQueueLength(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME, 100);

	carmen_localize_ackerman_subscribe_initialize_message(NULL,
			(carmen_handler_t) localize_ackerman_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_subscribe_message(CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
                             NULL, sizeof (carmen_behavior_selector_road_profile_message), (carmen_handler_t) rddf_message_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_download_map_subscribe_message(NULL, (carmen_handler_t) download_map_handler, CARMEN_SUBSCRIBE_LATEST);

}


static void
initialize_structures(void)
{
	current_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_map->config.x_origin = current_map->config.y_origin = 0.0001;
	current_map->complete_map = NULL;
	current_map->map = NULL;

	current_sum_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_sum_remission_map->config.x_origin = current_map->config.y_origin = 0.0001;
	current_sum_remission_map->complete_map = NULL;
	current_sum_remission_map->map = NULL;

	current_sum_sqr_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_sum_sqr_remission_map->config.x_origin = current_map->config.y_origin = 0.0001;
	current_sum_sqr_remission_map->complete_map = NULL;
	current_sum_sqr_remission_map->map = NULL;

	current_count_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_count_remission_map->config.x_origin = current_map->config.y_origin = 0.0001;
	current_count_remission_map->complete_map = NULL;
	current_count_remission_map->map = NULL;

	current_mean_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_mean_remission_map->config.x_origin = current_map->config.y_origin = 0.0001;
	current_mean_remission_map->complete_map = NULL;
	current_mean_remission_map->map = NULL;

	carmen_grid_mapping_initialize_map(current_mean_remission_map, ((double)map_width / map_grid_res), map_grid_res, 'm');

	current_variance_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_variance_remission_map->config.x_origin = current_map->config.y_origin = 0.0001;
	current_variance_remission_map->complete_map = NULL;
	current_variance_remission_map->map = NULL;

	carmen_grid_mapping_initialize_map(current_variance_remission_map, ((double)map_width / map_grid_res), map_grid_res, 'm');

	localize_map.complete_distance = NULL;
	localize_map.complete_gprob = NULL;
	localize_map.carmen_map.complete_map = NULL;
	localize_map.carmen_map.map = NULL;
	localize_map.complete_prob = NULL;
	localize_map.complete_x_offset = NULL;
	localize_map.complete_y_offset = NULL;
	localize_map.distance = NULL;
	localize_map.gprob = NULL;
	localize_map.prob = NULL;
	localize_map.x_offset = NULL;
	localize_map.y_offset = NULL;

//	current_google_map = (carmen_map_p) malloc (sizeof(carmen_map_t));
//	current_google_map->config.x_origin = current_google_map->config.y_origin = 0.0001;
//	current_google_map->complete_map = NULL;
//	current_google_map->map = NULL;

	current_road_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_road_map->config.x_origin = current_road_map->config.y_origin = 0.0001;
	current_road_map->complete_map = NULL;
	current_road_map->map = NULL;

	carmen_grid_mapping_initialize_map(current_road_map, ((double)map_width / map_grid_res), map_grid_res, 'r');
}
////////////////////////////////////////////////////////////////////////////////////////////////


int 
main(int argc, char **argv)
{
	carmen_point_t pose;
	int no_valid_map_on_file;
	double timestamp;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);
	define_messages();

	carmen_grid_mapping_init_parameters(map_grid_res, map_width);

	usleep(initial_waiting_time * 1e6);

	initialize_structures();
	if (map_file_name != NULL)
	{
		no_valid_map_on_file = carmen_map_read_gridmap_chunk(map_file_name, current_map) != 0;
		if (no_valid_map_on_file)
			printf("map_server: could not read offline map from file named: %s\n", map_file_name);
	}
	else
	{	
		pose.x = initial_map_x;
		pose.y = initial_map_y;

		if (block_map)
		{
			carmen_grid_mapping_get_block_map_by_origin(map_path, 'm', pose, current_map);
			carmen_grid_mapping_get_block_map_by_origin(map_path, 's', pose, current_sum_remission_map);
			carmen_grid_mapping_get_block_map_by_origin(map_path, '2', pose, current_sum_sqr_remission_map);
			carmen_grid_mapping_get_block_map_by_origin(map_path, 'c', pose, current_count_remission_map);
			carmen_grid_mapping_get_block_map_by_origin(map_path, 'r', pose, current_road_map);
			for (int i=0; i < max_height_level; i++)
				carmen_grid_mapping_get_block_map_by_origin(map_path, 'l'-i, pose, &height_maps[i]);
		}
	}

	if (current_map->complete_map != NULL)
	{
		timestamp = carmen_get_time();

		carmen_prob_models_calc_mean_and_variance_remission_map(current_mean_remission_map, current_variance_remission_map,
				current_sum_remission_map, current_sum_sqr_remission_map, current_count_remission_map);

		carmen_to_localize_ackerman_map(current_map, current_mean_remission_map, current_variance_remission_map, &localize_map, &localize_param);
		carmen_map_server_publish_offline_map_message(current_map, timestamp);
		carmen_map_server_publish_road_map_message(current_road_map, timestamp);
		carmen_map_server_publish_localize_map_message(&localize_map);
		for (int i=0; i < max_height_level; i++)
		{
			carmen_map_server_publish_offline_multi_height_map_message(&height_maps[i], timestamp, i+1);
		}
		if (publish_grid_mapping_map_at_startup)
			carmen_mapper_publish_map_message(current_map, timestamp);
	}
	else
	{
		printf("map_server: could not get an offline map at startup!\n");
	}

	register_handlers();
	carmen_ipc_dispatch();

	return 0;
}
