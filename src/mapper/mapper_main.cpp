#include <string.h>
#include <stdlib.h>
#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/laser_ldmrs_interface.h>
#include <carmen/laser_ldmrs_utils.h>
#include <carmen/rotation_geometry.h>
#include <carmen/mapper_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/stereo_mapping_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <carmen/parking_assistant_interface.h>
#include <omp.h>
#include "mapper.h"

#include "message_interpolation.cpp"

carmen_map_t offline_map;
// TODO: essa variavel eh definida como externa dentro da lib do mapper. Corrigir!
carmen_localize_ackerman_globalpos_message *globalpos_history;
int last_globalpos;

static int visual_odometry_is_global_pos = 0;
static int parking_assistant_found_safe_space = 0;

tf::Transformer tf_transformer(false);
MessageInterpolation<carmen_localize_ackerman_globalpos_message, carmen_ultrasonic_sonar_sensor_message> interpolator(1);

static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t sensor_board_pose_g;
static carmen_pose_3D_t ultrasonic_sensor_r1_g;
static carmen_pose_3D_t ultrasonic_sensor_r2_g;
static carmen_pose_3D_t ultrasonic_sensor_l1_g;
static carmen_pose_3D_t ultrasonic_sensor_l2_g;

/**
 * Model params
 */

double safe_range_above_sensors;
double robot_wheel_radius;

double obstacle_cost_distance = 1.0;
double obstacle_probability_threshold = 0.5;

int use_simulator_pose = 0;

rotation_matrix *board_to_car_matrix = NULL;

rotation_matrix *bullbar_to_car_matrix = NULL;

double highest_sensor = 0.0;

int merge_with_offline_map;
int build_snapshot_map;
int update_cells_below_car;
int update_and_merge_with_mapper_saved_maps;
int update_and_merge_with_snapshot_map;
int decay_to_offline_map;
int create_map_sum_and_count;


carmen_pose_3D_t sensor_board_1_pose;
carmen_pose_3D_t front_bullbar;

sensor_parameters_t *sensors_params;
sensor_parameters_t ultrasonic_sensor_params;
sensor_data_t *sensors_data;
int number_of_sensors;

carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t sick_pose;

char *map_path;

int publish_moving_objects_raw_map;

carmen_rddf_annotation_message last_rddf_annotation_message;
int robot_near_bump_or_barrier = 0;

bool offline_map_available = false;
int ok_to_publish = 0;
int number_of_threads = 1;

rotation_matrix *r_matrix_car_to_global = NULL;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_map(double timestamp)
{
	mapper_publish_map(timestamp);
}


void
include_sensor_data_into_map(int sensor_number, carmen_localize_ackerman_globalpos_message* globalpos_message)
{
	int aux = -1;
	for (int i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		if (sensors_data[sensor_number].points_timestamp[i] == globalpos_message->timestamp)
		{
			aux = sensors_data[sensor_number].point_cloud_index;
			sensors_data[sensor_number].point_cloud_index = i;
			run_mapper(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);
			publish_map(globalpos_message->timestamp);
			sensors_data[sensor_number].point_cloud_index = aux;
			break;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{

	double distance_to_annotation;

	if (visual_odometry_is_global_pos)
		interpolator.AddMessageToInterpolationList(globalpos_message);
	else
		mapper_set_robot_pose_into_the_map(globalpos_message, update_cells_below_car);

	// Map annotations handling
	distance_to_annotation = DIST2D(last_rddf_annotation_message.annotation_point, globalpos_history[last_globalpos].pose.position);
	if (((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_BUMP) ||
		 (last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_BARRIER)) &&
		(distance_to_annotation < 35.0))
		robot_near_bump_or_barrier = 1;
	else
		robot_near_bump_or_barrier = 0;

	if (ok_to_publish)
	{
		if (sensors_params[0].alive)
			include_sensor_data_into_map(0, globalpos_message);
	}
}


static void
true_pos_message_handler(carmen_simulator_ackerman_truepos_message *pose)
{
	if (offline_map_available)
	{
		mapper_merge_online_map_with_offline_map(&offline_map);
		publish_map(pose->timestamp);
	}
}


static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	mapper_velodyne_partial_scan(0, velodyne_message);
}


void
velodyne_variable_scan_message_handler1(carmen_velodyne_variable_scan_message *message)
{
	mapper_velodyne_variable_scan(1, message);
}


static void
laser_ldrms_message_handler(carmen_laser_ldmrs_message* msg)
{
	vpLaserScan laserscan[4];
	carmen_laser_ldmrs_copy_message_to_laser_scan(laserscan, msg);
	carmen_velodyne_partial_scan_message partial_scan_message = carmen_laser_ldmrs_convert_laser_scan_to_partial_velodyne_message(laserscan, msg->timestamp);

	mapper_velodyne_partial_scan(1, &partial_scan_message);
}


static void
velodyne_variable_scan_message_handler2(carmen_velodyne_variable_scan_message *message)
{
	mapper_velodyne_variable_scan(2, message);
}


static void
velodyne_variable_scan_message_handler3(carmen_velodyne_variable_scan_message *message)
{
	//sensors_params[3].log_odds.log_odds_free = sensors_params[3].log_odds.log_odds_occ = sensors_params[3].log_odds.log_odds_l0 = -10.0;
	if (mapper_velodyne_variable_scan(3, message))
		publish_map(message->timestamp);
}


static void
velodyne_variable_scan_message_handler4(carmen_velodyne_variable_scan_message *message)
{
	mapper_velodyne_variable_scan(4, message);
}


static void
velodyne_variable_scan_message_handler5(carmen_velodyne_variable_scan_message *message)
{
	mapper_velodyne_variable_scan(5, message);
}


static void
velodyne_variable_scan_message_handler6(carmen_velodyne_variable_scan_message *message)
{
	mapper_velodyne_variable_scan(6, message);
}


static void
velodyne_variable_scan_message_handler7(carmen_velodyne_variable_scan_message *message)
{
	mapper_velodyne_variable_scan(7, message);
}


static void
velodyne_variable_scan_message_handler8(carmen_velodyne_variable_scan_message *message)
{
	mapper_velodyne_variable_scan(8, message);
}


static void
velodyne_variable_scan_message_handler9(carmen_velodyne_variable_scan_message *message)
{
	mapper_velodyne_variable_scan(9, message);
}


static void
offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	static bool first_time = true;
	carmen_position_t map_origin;

	if (first_time)
	{
		offline_map_available = true;
		first_time = false;
	}

	map_origin.x = msg->config.x_origin;
	map_origin.y = msg->config.y_origin;

	memcpy(offline_map.complete_map, msg->complete_map, msg->config.x_size * msg->config.y_size * sizeof(double));
	offline_map.config = msg->config;

	mapper_change_map_origin_to_another_map_block(&map_origin);

	if (merge_with_offline_map)
		mapper_merge_online_map_with_offline_map(&offline_map);
}


static void
parking_sensor_goal_message_handler(carmen_parking_assistant_goal_message *message __attribute__ ((unused)))
{
	parking_assistant_found_safe_space = 1;
}


static void
ultrasonic_sensor_message_handler(carmen_ultrasonic_sonar_sensor_message *message)
{
	carmen_point_t Xt_r1, Xt_r2, Xt_l1, Xt_l2;

	tf::Transform world_to_car_pose;
	double yaw, pitch, roll;

	int i;

	carmen_localize_ackerman_globalpos_message globalpos_message;
	globalpos_message = interpolator.InterpolateMessages(message);

	mapper_set_robot_pose_into_the_map(&globalpos_message, update_cells_below_car);

	if (parking_assistant_found_safe_space)
	{
		publish_map(message->timestamp);
		return;
	}

	world_to_car_pose.setOrigin(tf::Vector3(globalpos_message.pose.position.x, globalpos_message.pose.position.y, globalpos_message.pose.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(globalpos_message.pose.orientation.yaw, globalpos_message.pose.orientation.pitch, globalpos_message.pose.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	tf_transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	//SENSOR R1 - FRONTAL
	tf::StampedTransform world_to_ultrasonic_sensor_r1;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_r1", tf::Time(0), world_to_ultrasonic_sensor_r1);
	Xt_r1.x = world_to_ultrasonic_sensor_r1.getOrigin().x();
	Xt_r1.y = world_to_ultrasonic_sensor_r1.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_r1.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_r1.theta = yaw;

	double range[180];

	for (i=0 ; i<180 ; i++)
		range[i] = (double) message->sensor[3];

	mapper_update_grid_map(Xt_r1, range, &ultrasonic_sensor_params);

	//SENSOR R2 - LATERAL FRONTAL
	tf::StampedTransform world_to_ultrasonic_sensor_r2;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_r2", tf::Time(0), world_to_ultrasonic_sensor_r2);
	Xt_r2.x = world_to_ultrasonic_sensor_r2.getOrigin().x();
	Xt_r2.y = world_to_ultrasonic_sensor_r2.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_r2.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_r2.theta = yaw;

	for (i=0 ; i<180 ; i++)
			range[i] = (double) message->sensor[2];

	mapper_update_grid_map(Xt_r2, range, &ultrasonic_sensor_params);

	//SENSOR L2 - LATERAL TRASEIRO
	tf::StampedTransform world_to_ultrasonic_sensor_l2;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_l2", tf::Time(0), world_to_ultrasonic_sensor_l2);
	Xt_l2.x = world_to_ultrasonic_sensor_l2.getOrigin().x();
	Xt_l2.y = world_to_ultrasonic_sensor_l2.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_l2.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_l2.theta = yaw;

	for (i=0 ; i<180 ; i++)
			range[i] = (double) message->sensor[1];

	mapper_update_grid_map(Xt_l2, range, &ultrasonic_sensor_params);

	//SENSOR L1 - TRASEIRO
	tf::StampedTransform world_to_ultrasonic_sensor_l1;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_l1", tf::Time(0), world_to_ultrasonic_sensor_l1);
	Xt_l1.x = world_to_ultrasonic_sensor_l1.getOrigin().x();
	Xt_l1.y = world_to_ultrasonic_sensor_l1.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_l1.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_l1.theta = yaw;

	for (i=0 ; i<180 ; i++)
			range[i] = (double) message->sensor[0];

	mapper_update_grid_map(Xt_l1, range, &ultrasonic_sensor_params);

	publish_map(message->timestamp);
}


static void
rddf_annotation_message_handler(carmen_rddf_annotation_message *message)
{
	double distance_to_last_annotation = DIST2D(last_rddf_annotation_message.annotation_point, globalpos_history[last_globalpos].pose.position);
	double distance_to_new_annotation = DIST2D(message->annotation_point, globalpos_history[last_globalpos].pose.position);

	if (distance_to_new_annotation < distance_to_last_annotation)
		last_rddf_annotation_message = *message; // TODO: tratar isso direito
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (update_and_merge_with_mapper_saved_maps)
			mapper_save_current_map();

		carmen_ipc_disconnect();
		fprintf(stderr, "Shutdown mapper_main\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intencity, carmen_pose_3D_t **robot_pose_out,
		carmen_vector_3D_t **robot_velocity_out, double **robot_timestamp_out, double **robot_phi_out, double **points_timestamp_out)
{
	int i;

	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *)malloc(NUM_VELODYNE_POINT_CLOUDS * sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*intencity = (unsigned char **)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char*));
	*robot_phi_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*points_timestamp_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));


	carmen_test_alloc(velodyne_points);

	for (i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		velodyne_points[i].num_points = 0;
		velodyne_points[i].sphere_points = NULL;
	}

	*velodyne_points_out = velodyne_points;
	*robot_pose_out = robot_pose;
	*robot_velocity_out = robot_velocity;
	*robot_timestamp_out = robot_timestamp;
}


static void
get_alive_sensors(int argc, char **argv)
{
	int i;

	sensors_params = (sensor_parameters_t *) calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(sensors_params);

	sensors_data = (sensor_data_t *) calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(sensors_data);

	carmen_param_t param_list[] =
	{
			{(char*)"mapper", (char*)"velodyne", CARMEN_PARAM_ONOFF, &sensors_params[0].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne1", CARMEN_PARAM_ONOFF, &sensors_params[1].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne2", CARMEN_PARAM_ONOFF, &sensors_params[2].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne3", CARMEN_PARAM_ONOFF, &sensors_params[3].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne4", CARMEN_PARAM_ONOFF, &sensors_params[4].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne5", CARMEN_PARAM_ONOFF, &sensors_params[5].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne6", CARMEN_PARAM_ONOFF, &sensors_params[6].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne7", CARMEN_PARAM_ONOFF, &sensors_params[7].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne8", CARMEN_PARAM_ONOFF, &sensors_params[8].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne9", CARMEN_PARAM_ONOFF, &sensors_params[9].alive, 0, NULL},
			{(char*)"mapper", (char*)"stereo_mapping", CARMEN_PARAM_ONOFF, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].alive, 0, NULL},

			{(char*)"mapper", (char*)"velodyne_locc", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne1_locc", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne2_locc", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne3_locc", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne4_locc", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne5_locc", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne6_locc", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne7_locc", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne8_locc", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne9_locc", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"stereo_mapping_locc", CARMEN_PARAM_DOUBLE, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].log_odds.log_odds_occ, 0, NULL},

			{(char*)"mapper", (char*)"velodyne_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne1_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne2_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne3_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne4_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne5_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne6_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne7_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne8_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne9_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"stereo_mapping_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].log_odds.log_odds_free, 0, NULL},

			{(char*)"mapper", (char*)"velodyne_l0", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne1_l0", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne2_l0", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne3_l0", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne4_l0", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne5_l0", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne6_l0", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne7_l0", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne8_l0", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne9_l0", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_l0, 0, NULL},

			{(char*)"mapper", (char*)"velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[0].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne1_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[1].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne2_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[2].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne3_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[3].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne4_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[4].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne5_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[5].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne6_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[6].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne7_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[7].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne8_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[8].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"stereo_velodyne9_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[9].unexpeted_delta_range_sigma, 0, NULL},

			{(char*)"mapper", (char*)"unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &sensors_params[0].unsafe_height_above_ground, 0, NULL},

			{(char*)"mapper",  (char*)"velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &sensors_params[0].range_max_factor, 0, NULL}


	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	for (i = 0; i < number_of_sensors; i++)
	{
		if (i == STEREO_MAPPING_SENSOR_INDEX)
			continue;

		sensors_params[i].unsafe_height_above_ground = sensors_params[0].unsafe_height_above_ground;

		sensors_data[i].ray_position_in_the_floor = (carmen_vector_2D_t **)  calloc(number_of_threads, sizeof(carmen_vector_2D_t*));
		sensors_data[i].maxed = (int **) calloc(number_of_threads, sizeof(int*));
		sensors_data[i].obstacle_height = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].occupancy_log_odds_of_each_ray_target = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].point_cloud_index = 0;
		sensors_data[i].points = NULL;
		sensors_data[i].ray_origin_in_the_floor = (carmen_vector_2D_t **) calloc(number_of_threads, sizeof(carmen_vector_2D_t*));;
		sensors_data[i].ray_size_in_the_floor = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].processed_intensity = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].ray_hit_the_robot = (int **) calloc(number_of_threads, sizeof(int*));
		sensors_data[i].ray_that_hit_the_nearest_target = (int *) calloc(number_of_threads, sizeof(int));

		sensors_params[i].name = NULL;
		sensors_params[i].ray_order = NULL;
		sensors_params[i].sensor_to_board_matrix = NULL;
		sensors_params[i].vertical_correction = NULL;
		sensors_params[i].vertical_resolution = 0;

		for (int j = 0; j < number_of_threads; j++)
		{
			sensors_data[i].ray_position_in_the_floor[j] = NULL;
			sensors_data[i].maxed[j] = NULL;
			sensors_data[i].obstacle_height[j] = NULL;
			sensors_data[i].occupancy_log_odds_of_each_ray_target[j] = NULL;
			sensors_data[i].ray_origin_in_the_floor[j] = NULL;
			sensors_data[i].ray_size_in_the_floor[j] = NULL;
			sensors_data[i].processed_intensity[i] = NULL;
			sensors_data[i].ray_hit_the_robot[j] = NULL;
		}

		if (sensors_params[i].alive)
		{
			sensors_params[i].name = (char *)calloc(strlen(param_list[i].variable) + 1, sizeof(char));
			strcpy(sensors_params[i].name, param_list[i].variable);
		}
	}
}


static int *
generates_ray_order(int size)
{
	int i;

	int *ray_order = (int *) malloc(size * sizeof(int));
	carmen_test_alloc(ray_order);

	for (i = 0; i < size; i++)
		ray_order[i] = i;

	return ray_order;
}


static void
get_sensors_param(int argc, char **argv)
{
	int i, j;
	int flipped;
	int horizontal_resolution;
	char stereo_velodyne_string[256];

	int stereo_velodyne_vertical_roi_ini;
	int stereo_velodyne_vertical_roi_end;

	int stereo_velodyne_horizontal_roi_ini;
	int stereo_velodyne_horizontal_roi_end;

	int roi_ini, roi_end;

	//velodyne

	sensors_params[0].pose = velodyne_pose;
	sensors_params[0].sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, sensors_params[0].pose.position, board_to_car_matrix);

	sensors_params[0].height = sensors_params[0].sensor_robot_reference.z + robot_wheel_radius;

	if (sensors_params[0].height > highest_sensor)
		highest_sensor = sensors_params[0].height;

	if (sensors_params[0].alive && !strcmp(sensors_params[0].name,"velodyne"))
	{
		sensors_params[0].ray_order = carmen_velodyne_get_ray_order();
		sensors_params[0].vertical_correction = carmen_velodyne_get_vertical_correction();
		sensors_params[0].delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
		sensors_params[0].delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

		carmen_param_t param_list[] =
		{
				{sensors_params[0].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[0].vertical_resolution, 0, NULL},
				{(char *) "mapper", (char *) "velodyne_range_max", CARMEN_PARAM_DOUBLE, &sensors_params[0].range_max, 0, NULL},
				{sensors_params[0].name, (char *) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &sensors_params[0].time_spent_by_each_scan, 0, NULL},

		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&sensors_data[0].points, &sensors_data[0].intensity, &sensors_data[0].robot_pose, &sensors_data[0].robot_velocity, &sensors_data[0].robot_timestamp, &sensors_data[0].robot_phi, &sensors_data[0].points_timestamp);
		sensors_params[0].sensor_to_board_matrix = create_rotation_matrix(sensors_params[0].pose.orientation);
		sensors_data[0].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&sensors_data[0], sensors_params[0].vertical_resolution, number_of_threads);

		sensors_params[0].remission_calibration = NULL;//(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));
//		FILE *f = fopen("../data/remission_calibration.txt", "r");
//		for (i = 0; i < 256 * sensors_params[0].vertical_resolution; i++)
//		{
//			fscanf(f, "%lf", &sensors_params[0].remission_calibration[i]);
//		}
//		fclose(f);
	}


	//sick
	sensors_params[1].pose = sick_pose;
	sensors_params[1].sensor_robot_reference = carmen_change_sensor_reference(front_bullbar.position, sensors_params[0].pose.position, bullbar_to_car_matrix);

	sensors_params[1].height = sensors_params[0].sensor_robot_reference.z + robot_wheel_radius;

	if (sensors_params[1].height > highest_sensor)
		highest_sensor = sensors_params[0].height;

	if (sensors_params[1].alive)
	{
		int ray_order[32] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
		sensors_params[1].ray_order = ray_order;

		double vertical_correction[32] = {-1.2, -0.4, 0.4, 1.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		sensors_params[1].vertical_correction = vertical_correction;

		double delta_difference_mean[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		sensors_params[1].delta_difference_mean = delta_difference_mean;

		double delta_difference_stddev[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		sensors_params[1].delta_difference_stddev = delta_difference_stddev;

		carmen_param_t param_list[] =
		{
				{(char *) "sick", (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[1].vertical_resolution, 0, NULL},
				{(char *) "mapper", (char *) "velodyne_range_max", CARMEN_PARAM_DOUBLE, &sensors_params[1].range_max, 0, NULL},
				{(char *) "sick", (char *) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &sensors_params[1].time_spent_by_each_scan, 0, NULL},
		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&sensors_data[1].points, &sensors_data[1].intensity, &sensors_data[1].robot_pose, &sensors_data[1].robot_velocity, &sensors_data[1].robot_timestamp, &sensors_data[1].robot_phi, &sensors_data[1].points_timestamp);
		sensors_params[1].sensor_to_board_matrix = create_rotation_matrix(sensors_params[1].pose.orientation);
		sensors_data[1].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&sensors_data[1], sensors_params[1].vertical_resolution, number_of_threads);

		sensors_params[1].remission_calibration = NULL;//(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));
//		FILE *f = fopen("../data/remission_calibration.txt", "r");
//		for (i = 0; i < 256 * sensors_params[0].vertical_resolution; i++)
//		{
//			fscanf(f, "%lf", &sensors_params[0].remission_calibration[i]);
//		}
//		fclose(f);
	}

	for (i = 2; i < number_of_sensors; i++)
	{
		if (i == STEREO_MAPPING_SENSOR_INDEX)
			continue;

		if (sensors_params[i].alive)
		{
			sensors_params[i].pose = get_stereo_velodyne_pose_3D(argc, argv, i);

			sensors_params[i].sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, sensors_params[i].pose.position, board_to_car_matrix);
			sensors_params[i].height = sensors_params[i].sensor_robot_reference.z + robot_wheel_radius;

			if (sensors_params[i].height > highest_sensor)
				highest_sensor = sensors_params[i].height;

			sensors_params[i].time_spent_by_each_scan = 0.0;

			sprintf(stereo_velodyne_string, "%s%d", "stereo", i);


			carmen_param_t param_list[] =
			{
					{sensors_params[i].name, (char*) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[i].vertical_resolution, 0, NULL},
					{sensors_params[i].name, (char*) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 0, NULL},
					{sensors_params[i].name, (char*) "flipped", CARMEN_PARAM_ONOFF, &flipped, 0, NULL},
					{sensors_params[i].name, (char*) "range_max", CARMEN_PARAM_DOUBLE, &sensors_params[i].range_max, 0, NULL},
					{sensors_params[i].name, (char*) "vertical_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_ini, 0, NULL },
					{sensors_params[i].name, (char*) "vertical_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_end, 0, NULL },
					{sensors_params[i].name, (char*) "horizontal_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_ini, 0, NULL },
					{sensors_params[i].name, (char*) "horizontal_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_end, 0, NULL }

			};

			carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

			if (flipped)
			{
				sensors_params[i].vertical_resolution = horizontal_resolution;
				roi_ini = stereo_velodyne_horizontal_roi_ini;
				roi_end = stereo_velodyne_horizontal_roi_end;
			}
			else
			{
				roi_ini = stereo_velodyne_vertical_roi_ini;
				roi_end = stereo_velodyne_vertical_roi_end;
			}

			if (sensors_params[i].vertical_resolution > (roi_end - roi_ini))
			{
				carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");
			}
			sensors_params[i].range_max_factor = 1.0;
			sensors_params[i].ray_order = generates_ray_order(sensors_params[i].vertical_resolution);
			sensors_params[i].vertical_correction = get_stereo_velodyne_correction(flipped, i, sensors_params[i].vertical_resolution, roi_ini, roi_end, 0, 0);
			init_velodyne_points(&sensors_data[i].points, &sensors_data[i].intensity, &sensors_data[i].robot_pose, &sensors_data[i].robot_velocity,  &sensors_data[i].robot_timestamp, &sensors_data[i].robot_phi, &sensors_data[i].points_timestamp);
			sensors_params[i].sensor_to_board_matrix = create_rotation_matrix(sensors_params[i].pose.orientation);
			sensors_data[i].point_cloud_index = 0;
			carmen_prob_models_alloc_sensor_data(&sensors_data[i], sensors_params[i].vertical_resolution, number_of_threads);

			//TODO : tem que fazer esta medida para as cameras igual foi feito para o velodyne
			sensors_params[i].delta_difference_mean = (double *)calloc(50, sizeof(double));
			sensors_params[i].delta_difference_stddev = (double *)calloc(50, sizeof(double));
			for (j = 0; j < 50; j++)
				sensors_params[i].delta_difference_stddev[j] = 1.0;
		}
	}
}


/* read all parameters from .ini file and command line. */
static void
read_parameters(int argc, char **argv,
		carmen_map_config_t *map_config,
		carmen_robot_ackerman_config_t *p_car_config)
{
	double robot_vertical_displacement_from_center;

	double map_resolution, map_width, map_height;
	carmen_param_t param_list[] =
	{
			{(char*)"robot",  (char*)"distance_between_front_car_and_front_wheels", 	CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_car_and_front_wheels), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_and_rear_axles), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_rear_car_and_rear_wheels",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_car_and_rear_wheels), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_rear_wheels",			CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_wheels), 1, NULL},

			{(char*)"sensor_board_1",  (char*)"x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
			{(char*)"sensor_board_1",  (char*)"y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
			{(char*)"sensor_board_1",  (char*)"z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
			{(char*)"sensor_board_1",  (char*)"roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
			{(char*)"sensor_board_1",  (char*)"pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
			{(char*)"sensor_board_1",  (char*)"yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},

			{(char*)"velodyne",  (char*)"x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char*)"velodyne",  (char*)"y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char*)"velodyne",  (char*)"z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char*)"velodyne",  (char*)"roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char*)"velodyne",  (char*)"pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char*)"velodyne",  (char*)"yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

			{(char*)"front_bullbar",  (char*)"x", CARMEN_PARAM_DOUBLE, &(front_bullbar.position.x),	0, NULL},
			{(char*)"front_bullbar",  (char*)"y", CARMEN_PARAM_DOUBLE, &(front_bullbar.position.y),	0, NULL},
			{(char*)"front_bullbar",  (char*)"z", CARMEN_PARAM_DOUBLE, &(front_bullbar.position.z),	0, NULL},
			{(char*)"front_bullbar",  (char*)"roll", CARMEN_PARAM_DOUBLE, &(front_bullbar.orientation.roll),0, NULL},
			{(char*)"front_bullbar",  (char*)"pitch", CARMEN_PARAM_DOUBLE, &(front_bullbar.orientation.pitch),0, NULL},
			{(char*)"front_bullbar",  (char*)"yaw", CARMEN_PARAM_DOUBLE, &(front_bullbar.orientation.yaw),	0, NULL},


			{(char*)"sick",  (char*)"x", CARMEN_PARAM_DOUBLE, &(sick_pose.position.x), 0, NULL},
			{(char*)"sick",  (char*)"y", CARMEN_PARAM_DOUBLE, &(sick_pose.position.y), 0, NULL},
			{(char*)"sick",  (char*)"z", CARMEN_PARAM_DOUBLE, &(sick_pose.position.z), 0, NULL},
			{(char*)"sick",  (char*)"roll", CARMEN_PARAM_DOUBLE, &(sick_pose.orientation.roll), 0, NULL},
			{(char*)"sick",  (char*)"pitch", CARMEN_PARAM_DOUBLE, &(sick_pose.orientation.pitch), 0, NULL},
			{(char*)"sick",  (char*)"yaw", CARMEN_PARAM_DOUBLE, &(sick_pose.orientation.yaw), 0, NULL},


			{(char*)"robot",  (char*)"wheel_radius", CARMEN_PARAM_DOUBLE, &(robot_wheel_radius), 0, NULL},
			{(char*)"mapper",  (char*)"number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL},
			{(char*)"mapper",  (char*)"safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},

			{(char*)"robot",  (char*)"length", CARMEN_PARAM_DOUBLE, &p_car_config->length, 0, NULL},
			{(char*)"robot",  (char*)"width", CARMEN_PARAM_DOUBLE, &p_car_config->width, 0, NULL},
			{(char*)"robot",  (char*)"vertical_displacement_from_center", CARMEN_PARAM_DOUBLE, &robot_vertical_displacement_from_center, 0, NULL},

			{(char*)"mapper",  (char*)"map_grid_res", CARMEN_PARAM_DOUBLE, &map_resolution, 0, NULL},
			{(char*)"mapper",  (char*)"map_width", CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
			{(char*)"mapper",  (char*)"map_height", CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},

			{(char*)"mapper", (char*)"moving_objects_raw_map", CARMEN_PARAM_ONOFF, &publish_moving_objects_raw_map, 0, NULL},

			{(char*)"mapper",  (char*)"build_snapshot_map", CARMEN_PARAM_ONOFF, &build_snapshot_map, 0, NULL},
			{(char*)"mapper",  (char*)"merge_with_offline_map", CARMEN_PARAM_ONOFF, &merge_with_offline_map, 0, NULL},
			{(char*)"mapper",  (char*)"update_and_merge_with_mapper_saved_maps", CARMEN_PARAM_ONOFF, &update_and_merge_with_mapper_saved_maps, 0, NULL},
			{(char*)"mapper",  (char*)"update_cells_below_car", CARMEN_PARAM_ONOFF, &update_cells_below_car, 0, NULL},
			{(char*)"mapper",  (char*)"decay_to_offline_map", CARMEN_PARAM_ONOFF, &decay_to_offline_map, 0, NULL},
			{(char*)"mapper",  (char*)"create_map_sum_and_count", CARMEN_PARAM_ONOFF, &create_map_sum_and_count, 0, NULL},

			{(char*)"mapper",  (char*)"update_and_merge_with_snapshot_map", CARMEN_PARAM_ONOFF, &update_and_merge_with_snapshot_map, 0, NULL},
			{(char*)"mapper",  (char*)"number_of_threads", CARMEN_PARAM_INT, &number_of_threads, 0, NULL},


			{(char*)"commandline",  (char*)"map_path", CARMEN_PARAM_STRING, &map_path, 0, NULL},

			{(char *) "visual_odometry", (char *) "is_global_pos", CARMEN_PARAM_ONOFF, &visual_odometry_is_global_pos, 0, NULL},

			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_sampling_step", CARMEN_PARAM_INT, &ultrasonic_sensor_params.sampling_step, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_num_beams", CARMEN_PARAM_INT, &ultrasonic_sensor_params.laser_beams, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_fov_range", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.fov_range, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_max_range", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.range_max, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_lambda_short", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.lambda_short, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_sigma_zhit", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.sigma_zhit, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_zhit", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zhit, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_zmax", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zmax, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_zrand", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zrand, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_zshort", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zshort, 0, NULL},

			{(char *)"grid_mapping", (char *)"map_locc", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_occ, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_lfree", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_free, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_l0", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_l0, 0, NULL},

			{(char *) "ultrasonic_sensor_r1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.x), 0, NULL},
			{(char *) "ultrasonic_sensor_r1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.y), 0, NULL},
			{(char *) "ultrasonic_sensor_r1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.z), 0, NULL},
			{(char *) "ultrasonic_sensor_r1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.yaw), 0, NULL},
			{(char *) "ultrasonic_sensor_r1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.pitch), 0, NULL},
			{(char *) "ultrasonic_sensor_r1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.roll), 0, NULL},

			{(char *) "ultrasonic_sensor_r2", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.x), 0, NULL},
			{(char *) "ultrasonic_sensor_r2", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.y), 0, NULL},
			{(char *) "ultrasonic_sensor_r2", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.z), 0, NULL},
			{(char *) "ultrasonic_sensor_r2", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.yaw), 0, NULL},
			{(char *) "ultrasonic_sensor_r2", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.pitch), 0, NULL},
			{(char *) "ultrasonic_sensor_r2", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.roll), 0, NULL},

			{(char *) "ultrasonic_sensor_l2", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.x), 0, NULL},
			{(char *) "ultrasonic_sensor_l2", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.y), 0, NULL},
			{(char *) "ultrasonic_sensor_l2", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.z), 0, NULL},
			{(char *) "ultrasonic_sensor_l2", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.yaw), 0, NULL},
			{(char *) "ultrasonic_sensor_l2", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.pitch), 0, NULL},
			{(char *) "ultrasonic_sensor_l2", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.roll), 0, NULL},

			{(char *) "ultrasonic_sensor_l1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.x), 0, NULL},
			{(char *) "ultrasonic_sensor_l1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.y), 0, NULL},
			{(char *) "ultrasonic_sensor_l1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.z), 0, NULL},
			{(char *) "ultrasonic_sensor_l1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.yaw), 0, NULL},
			{(char *) "ultrasonic_sensor_l1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.pitch), 0, NULL},
			{(char *) "ultrasonic_sensor_l1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.roll), 0, NULL},

			{(char *)"rrt",	(char *)"obstacle_cost_distance",	CARMEN_PARAM_DOUBLE,	&obstacle_cost_distance,	1, NULL},
			{(char *)"rrt",	(char *)"obstacle_probability_threshold",	CARMEN_PARAM_DOUBLE,	&obstacle_probability_threshold,	1, NULL}

	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));


	if (map_width != map_height)
		carmen_die("Wrong map size: width (%f) must be equal to height (%f).", map_width, map_height);

	if ( (((int)map_width) % 3) != 0)
		carmen_die("Wrong map size: width (%f) and height (%f) must be multiple of 3.", map_width, map_height);

	map_config->x_size = round(map_width / map_resolution);
	map_config->y_size = round(map_height / map_resolution);
	map_config->resolution = map_resolution;

	carmen_grid_mapping_init_parameters(map_resolution, map_width);

	board_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);
	bullbar_to_car_matrix = create_rotation_matrix(front_bullbar.orientation);

	get_alive_sensors(argc, argv);

	get_sensors_param(argc, argv);
}


static void
carmen_subscribe_to_ultrasonic_relevant_messages()
{
	carmen_parking_assistant_subscribe_goal(NULL, (carmen_handler_t) parking_sensor_goal_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ultrasonic_sonar_sensor_subscribe(NULL, (carmen_handler_t) ultrasonic_sensor_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
subscribe_to_ipc_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
			(carmen_handler_t)carmen_localize_ackerman_globalpos_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[0].alive)
	{
		carmen_velodyne_subscribe_partial_scan_message(NULL,
				(carmen_handler_t)velodyne_partial_scan_message_handler,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[1].alive)
	{
//		carmen_stereo_velodyne_subscribe_scan_message(1, NULL,
//				(carmen_handler_t)velodyne_variable_scan_message_handler1,
//				CARMEN_SUBSCRIBE_LATEST);

		carmen_laser_subscribe_ldmrs_message(NULL, (carmen_handler_t) laser_ldrms_message_handler,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[2].alive)
	{
		carmen_stereo_velodyne_subscribe_scan_message(2, NULL,
				(carmen_handler_t)velodyne_variable_scan_message_handler2,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[3].alive)
	{
		carmen_stereo_velodyne_subscribe_scan_message(3, NULL,
				(carmen_handler_t)velodyne_variable_scan_message_handler3,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[4].alive)
	{
		carmen_stereo_velodyne_subscribe_scan_message(4, NULL,
				(carmen_handler_t)velodyne_variable_scan_message_handler4,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[5].alive)
	{
		carmen_stereo_velodyne_subscribe_scan_message(5, NULL,
				(carmen_handler_t)velodyne_variable_scan_message_handler5,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[6].alive)
	{
		carmen_stereo_velodyne_subscribe_scan_message(6, NULL,
				(carmen_handler_t)velodyne_variable_scan_message_handler6,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[7].alive)
	{
		carmen_stereo_velodyne_subscribe_scan_message(7, NULL,
				(carmen_handler_t)velodyne_variable_scan_message_handler7,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[8].alive)
	{
		carmen_stereo_velodyne_subscribe_scan_message(8, NULL,
				(carmen_handler_t)velodyne_variable_scan_message_handler8,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[9].alive)
	{
		carmen_stereo_velodyne_subscribe_scan_message(9, NULL,
				(carmen_handler_t)velodyne_variable_scan_message_handler9,
				CARMEN_SUBSCRIBE_LATEST);
	}

	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) true_pos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (visual_odometry_is_global_pos)
		carmen_subscribe_to_ultrasonic_relevant_messages();

	// esse handler eh subscribe_all porque todas as anotacoes precisam ser recebidas!
	carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) rddf_annotation_message_handler, CARMEN_SUBSCRIBE_ALL);
}


static void
define_mapper_messages()
{
	/* register initialize message */
	carmen_map_server_define_compact_cost_map_message();
	carmen_mapper_define_messages();
}

void initialize_transforms()
{
	tf::Transform board_to_camera_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;
	tf::Transform ultrasonic_sensor_r1_to_car_pose;
	tf::Transform ultrasonic_sensor_r2_to_car_pose;
	tf::Transform ultrasonic_sensor_l1_to_car_pose;
	tf::Transform ultrasonic_sensor_l2_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	tf_transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_pose_g.position.x, sensor_board_pose_g.position.y, sensor_board_pose_g.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_pose_g.orientation.yaw, sensor_board_pose_g.orientation.pitch, sensor_board_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	tf_transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	tf_transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

	// initial ultrasonic sensor r1 pose with respect to the car
	ultrasonic_sensor_r1_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_r1_g.position.x, ultrasonic_sensor_r1_g.position.y, ultrasonic_sensor_r1_g.position.z));
	ultrasonic_sensor_r1_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_r1_g.orientation.yaw, ultrasonic_sensor_r1_g.orientation.pitch, ultrasonic_sensor_r1_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_r1_to_car_transform(ultrasonic_sensor_r1_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_r1");
	tf_transformer.setTransform(ultrasonic_sensor_r1_to_car_transform, "ultrasonic_sensor_r1_to_car_transform");

	// initial ultrasonic sensor r2 pose with respect to the car
	ultrasonic_sensor_r2_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_r2_g.position.x, ultrasonic_sensor_r2_g.position.y, ultrasonic_sensor_r2_g.position.z));
	ultrasonic_sensor_r2_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_r2_g.orientation.yaw, ultrasonic_sensor_r2_g.orientation.pitch, ultrasonic_sensor_r2_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_r2_to_car_transform(ultrasonic_sensor_r2_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_r2");
	tf_transformer.setTransform(ultrasonic_sensor_r2_to_car_transform, "ultrasonic_sensor_r2_to_car_transform");

	// initial ultrasonic sensor l2 pose with respect to the car
	ultrasonic_sensor_l2_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_l2_g.position.x, ultrasonic_sensor_l2_g.position.y, ultrasonic_sensor_l2_g.position.z));
	ultrasonic_sensor_l2_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_l2_g.orientation.yaw, ultrasonic_sensor_l2_g.orientation.pitch, ultrasonic_sensor_l2_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_l2_to_car_transform(ultrasonic_sensor_l2_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_l2");
	tf_transformer.setTransform(ultrasonic_sensor_l2_to_car_transform, "ultrasonic_sensor_l2_to_car_transform");

	// initial ultrasonic sensor l1 pose with respect to the car
	ultrasonic_sensor_l1_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_l1_g.position.x, ultrasonic_sensor_l1_g.position.y, ultrasonic_sensor_l1_g.position.z));
	ultrasonic_sensor_l1_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_l1_g.orientation.yaw, ultrasonic_sensor_l1_g.orientation.pitch, ultrasonic_sensor_l1_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_l1_to_car_transform(ultrasonic_sensor_l1_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_l1");
	tf_transformer.setTransform(ultrasonic_sensor_l1_to_car_transform, "ultrasonic_sensor_l1_to_car_transform");


}
//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	carmen_map_config_t map_config;
	carmen_robot_ackerman_config_t car_config;

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv, &map_config, &car_config);

	initialize_transforms();

	mapper_initialize(&map_config, car_config);

	/* Register messages */
	define_mapper_messages();

	/* Subscribe to relevant messages */
	subscribe_to_ipc_messages();

	carmen_ipc_dispatch();

	return (0);
}
