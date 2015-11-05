
#include <carmen/carmen.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/grid_mapping_interface.h>
#include <carmen/grid_mapping_messages.h>
#include <carmen/map_server_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <prob_measurement_model.h>
#include <prob_map.h>

#include <carmen/localize_ackerman_core.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include "slam_icp_velodyne.h"

char remission_file[256];

#define FUSED_ODOMETRY_VECTOR_SIZE 50
static carmen_fused_odometry_message fused_odometry_vector[FUSED_ODOMETRY_VECTOR_SIZE];
static int fused_odometry_index = -1;

/* global variables */

carmen_pose_3D_t sensor_board_1_pose;
rotation_matrix *sensor_board_1_to_car_matrix;
double robot_wheel_radius;
double robot_length;
double robot_width;
double distance_between_front_and_rear_axles;
double distance_between_rear_car_and_rear_wheels;

double x_origin = 0.0;
double y_origin = 0.0;

sensor_parameters_t velodyne_params;
sensor_data_t velodyne_data;

carmen_localize_ackerman_globalpos_message globalpos;

extern double laser_ranges[10000];

double slam_icp_initialization_time = 0.0;
double slam_icp_update_distance = 0.0;
double slam_icp_update_angle = 0.0;

carmen_pose_3D_t pose_first_fused_odometry;
carmen_pose_3D_t pose_of_last_icp_running;

static carmen_pose_3D_t corrected_pose;
static carmen_pose_3D_t last_corrected_pose;
static double last_velodyne_timestamp;
double highest_point;


static int
get_fused_odometry_index_by_timestamp(double timestamp)
{
	double min_diff, diff;
	int min_index;

	min_diff = DBL_MAX;
	min_index = -1;

	for (int i = 0; i < FUSED_ODOMETRY_VECTOR_SIZE; i++)
	{
		diff = fabs(fused_odometry_vector[i].timestamp - timestamp);

		if (diff < min_diff)
		{
			min_diff = diff;
			min_index = i;
		}
	}

	return min_index;
}


void 
publish_globalpos(double v, double phi, double timestamp, carmen_pose_3D_t pose)
{
	IPC_RETURN_TYPE err;

	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();
//	globalpos.globalpos = summary->mean;
//	globalpos.globalpos_std = summary->std;
//	globalpos.odometrypos = summary->odometry_pos;
//	globalpos.globalpos_xy_cov = summary->xy_cov;
	globalpos.v = v;
	globalpos.phi = phi;
//	globalpos.converged = summary->converged;

	if (fused_odometry_index == -1)
	{
		globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = globalpos.pose.position.z = 0.0;
		globalpos.velocity.x = globalpos.velocity.y = globalpos.velocity.z = 0.0;
	}
	else
	{	// Aproveita alguns dados da fused_odometry. 
		// Os valores referentes aa globalpos corrente sao escritos abaixo.
		globalpos.pose = fused_odometry_vector[fused_odometry_index].pose;
		globalpos.velocity.x = v;
	}

	globalpos.globalpos.x = pose.position.x;
	globalpos.globalpos.y = pose.position.y;
	globalpos.globalpos.theta = pose.orientation.yaw;
	globalpos.pose = pose;
	globalpos.v = globalpos.velocity.x;

//	globalpos.pose.orientation.yaw = globalpos.globalpos.theta;
//	globalpos.pose.position.x = globalpos.globalpos.x;
//	globalpos.pose.position.y = globalpos.globalpos.y;
//	globalpos.pose.position.z = 0;
//
//	globalpos.velocity.x = v;
	
	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc_exit(err, "Could not publish",
			CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


void 
publish_particles(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, double timestamp)
{
	static carmen_localize_ackerman_particle_message pmsg;
	IPC_RETURN_TYPE err;

	pmsg.timestamp = timestamp;
	pmsg.host = carmen_get_host();
	pmsg.globalpos = summary->mean;
	pmsg.globalpos_std = summary->mean;
	pmsg.num_particles = filter->param->num_particles;
	pmsg.particles = (carmen_localize_ackerman_particle_ipc_p)filter->particles;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, &pmsg);
	carmen_test_ipc_exit(err, "Could not publish",
			CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);
	//fprintf(stderr, "P");
}


//void
//publish_sensor(carmen_localize_ackerman_particle_filter_p filter,
//		carmen_localize_ackerman_summary_p summary,
//		int num_readings,
//		float *range,
//		carmen_laser_laser_config_t laser_config,
//		int front,
//		double timestamp)
//{
//	static carmen_localize_ackerman_sensor_message sensor;
//	IPC_RETURN_TYPE err;
//
//	sensor.timestamp = timestamp;
//	sensor.host = carmen_get_host();
//	if(front) {
//		sensor.pose.x = summary->mean.x + filter->param->front_laser_offset *
//				cos(summary->mean.theta);
//		sensor.pose.y = summary->mean.y + filter->param->front_laser_offset *
//				sin(summary->mean.theta);
//		sensor.pose.theta = summary->mean.theta;
//		sensor.num_laser = 1;
//	}
//	else {
//		sensor.pose.x = summary->mean.x + filter->param->rear_laser_offset *
//				cos(summary->mean.theta + M_PI);
//		sensor.pose.y = summary->mean.y + filter->param->rear_laser_offset *
//				sin(summary->mean.theta + M_PI);
//		sensor.pose.theta = summary->mean.theta + M_PI;
//		sensor.num_laser = 2;
//	}
//	sensor.num_readings = num_readings;
//	sensor.laser_skip = filter->param->laser_skip;
//	sensor.config = laser_config;
//	sensor.range = range;
//	sensor.mask = filter->laser_mask;
//	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, &sensor);
//	carmen_test_ipc_exit(err, "Could not publish",
//			CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);
//}


static void
fused_odometry_handler(carmen_fused_odometry_message *msg)
{
	static int is_first_fused_odometry_message = 1;
	static double timestamp_first_fused_odometry_message = 0.0;
	static double last_timestamp = msg->timestamp;
	double v, phi, dt;
	carmen_pose_3D_t pose;

	// initialize time counting
	if (is_first_fused_odometry_message)
	{
		last_velodyne_timestamp = timestamp_first_fused_odometry_message = msg->timestamp;
		is_first_fused_odometry_message = 0;
		pose_first_fused_odometry = msg->pose;
		pose_of_last_icp_running = msg->pose;
		corrected_pose = msg->pose;
		return;
	}

	// check if enough time has passed
	if (fabs(msg->timestamp - timestamp_first_fused_odometry_message) > slam_icp_initialization_time)
	{
		fused_odometry_index = (fused_odometry_index + 1) % FUSED_ODOMETRY_VECTOR_SIZE;
		fused_odometry_vector[fused_odometry_index] = *msg;
		v = msg->velocity.x;
		phi = msg->phi;
		dt = msg->timestamp - last_velodyne_timestamp;

//		pose = carmen_ackerman_interpolated_robot_position_at_time(corrected_pose, dt, msg->velocity.x,  msg->phi,  distance_between_front_and_rear_axles);
//		publish_globalpos(v, phi, msg->timestamp, pose);
	}
	last_timestamp = msg->timestamp;
}

static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{

	if (fused_odometry_index < 2)
		return;
	double v, phi, ds, dt, dtheta, theta;
	carmen_pose_3D_t fused_odometry_pose = fused_odometry_vector[fused_odometry_index].pose;
	static int is_fisrt = 1;

	fused_odometry_pose.position.x -= pose_first_fused_odometry.position.x;
	fused_odometry_pose.position.y -= pose_first_fused_odometry.position.y;
	fused_odometry_pose.orientation.pitch = fused_odometry_pose.orientation.roll = 0.0;
	fused_odometry_pose.position.z = 0.0;

	dt = velodyne_message->timestamp - fused_odometry_vector[fused_odometry_index].timestamp;
	v = fused_odometry_vector[fused_odometry_index].velocity.x;
	phi = fused_odometry_vector[fused_odometry_index].phi;
	theta = fused_odometry_pose.orientation.yaw;
	ds = v * dt;

	fused_odometry_pose = carmen_ackerman_interpolated_robot_position_at_time(fused_odometry_pose, dt, v,  phi,  distance_between_front_and_rear_axles);

	dtheta = carmen_normalize_theta(pose_of_last_icp_running.orientation.yaw - fused_odometry_pose.orientation.yaw);
	ds = sqrt(pow(pose_of_last_icp_running.position.x - fused_odometry_pose.position.x, 2) +
			pow (pose_of_last_icp_running.position.y - fused_odometry_pose.position.y, 2));

	if ((ds >= slam_icp_update_distance) || (fabs(dtheta) >= slam_icp_update_angle))
	{
		int converge = slam_icp_velodyne_partial_scan(velodyne_message, &velodyne_params, &velodyne_data, &fused_odometry_pose, &fused_odometry_vector[fused_odometry_index].velocity, fused_odometry_vector[fused_odometry_index].phi, fused_odometry_vector[fused_odometry_index].timestamp, &corrected_pose);

		if (is_fisrt == 1)
		{
			is_fisrt = 0;
			pose_of_last_icp_running = fused_odometry_pose;
			return;
		}
		if (converge)
			pose_of_last_icp_running = fused_odometry_pose;
		else
			return;

		corrected_pose.position.x += pose_first_fused_odometry.position.x;
		corrected_pose.position.y += pose_first_fused_odometry.position.y;

		last_corrected_pose = corrected_pose;
		last_velodyne_timestamp = velodyne_message->timestamp;
		publish_globalpos(v, phi, velodyne_message->timestamp, corrected_pose);
		is_fisrt = -2;
	}
	else
	{
		if (is_fisrt == 2)
		{
			corrected_pose = apply_transform(&fused_odometry_pose);
			corrected_pose.position.x += pose_first_fused_odometry.position.x;
			corrected_pose.position.y += pose_first_fused_odometry.position.y;
			publish_globalpos(v, phi, velodyne_message->timestamp, corrected_pose);
		}
	}

}


static void 
read_parameters(int argc, char **argv, carmen_localize_ackerman_param_p param, ProbabilisticMapParams *p_map_params)
{
	double integrate_angle_deg;
	int i;
	carmen_pose_3D_t velodyne_pose;

	integrate_angle_deg = 1.0;

	carmen_param_t param_list[] = 
	{

			{(char*)"robot", (char*)"frontlaser_offset", CARMEN_PARAM_DOUBLE, &param->front_laser_offset, 0, NULL},
			{(char*)"robot", (char*)"rearlaser_offset", CARMEN_PARAM_DOUBLE, &param->rear_laser_offset, 0, NULL},
			{(char*)"robot", (char*)"length", CARMEN_PARAM_DOUBLE, &robot_length, 0, NULL},
			{(char*)"robot", (char*)"width", CARMEN_PARAM_DOUBLE, &robot_width, 0, NULL},
			{(char*)"robot", (char*)"distance_between_front_and_rear_axles",	CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 1, NULL},
			{(char*)"robot", (char*)"distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &distance_between_rear_car_and_rear_wheels, 1, NULL},

			{(char*)"robot", (char*)"wheel_radius", CARMEN_PARAM_DOUBLE, &robot_wheel_radius, 0, NULL},

			{(char*)"localize", (char*)"use_rear_laser", CARMEN_PARAM_ONOFF, &param->use_rear_laser, 0, NULL},
			{(char*)"localize", (char*)"num_particles", CARMEN_PARAM_INT, &param->num_particles, 0, NULL},
			{(char*)"localize", (char*)"laser_max_range", CARMEN_PARAM_DOUBLE, &param->max_range, 1, NULL},
			{(char*)"localize", (char*)"min_wall_prob", CARMEN_PARAM_DOUBLE, &param->min_wall_prob, 0, NULL},
			{(char*)"localize", (char*)"outlier_fraction", CARMEN_PARAM_DOUBLE, &param->outlier_fraction, 0, NULL},
			{(char*)"localize", (char*)"update_distance", CARMEN_PARAM_DOUBLE, &param->update_distance, 0, NULL},
			{(char*)"localize", (char*)"integrate_angle_deg", CARMEN_PARAM_DOUBLE, &integrate_angle_deg, 0, NULL},
			{(char*)"localize", (char*)"do_scanmatching", CARMEN_PARAM_ONOFF, &param->do_scanmatching, 1, NULL},
			{(char*)"localize", (char*)"constrain_to_map", CARMEN_PARAM_ONOFF, &param->constrain_to_map, 1, NULL},

			{(char*)"localize", (char*)"occupied_prob", CARMEN_PARAM_DOUBLE, &param->occupied_prob, 0, NULL},
			{(char*)"localize", (char*)"lmap_std", CARMEN_PARAM_DOUBLE, &param->lmap_std, 0, NULL},
			{(char*)"localize", (char*)"global_lmap_std", CARMEN_PARAM_DOUBLE, &param->global_lmap_std, 0, NULL},
			{(char*)"localize", (char*)"global_evidence_weight", CARMEN_PARAM_DOUBLE, &param->global_evidence_weight, 0, NULL},
			{(char*)"localize", (char*)"global_distance_threshold", CARMEN_PARAM_DOUBLE, &param->global_distance_threshold, 1, NULL},
			{(char*)"localize", (char*)"global_test_samples", CARMEN_PARAM_INT, &param->global_test_samples, 1, NULL},
			{(char*)"localize", (char*)"use_sensor", CARMEN_PARAM_ONOFF, &param->use_sensor, 0, NULL},
			{(char*)"localize", (char*)"tracking_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &param->tracking_beam_minlikelihood, 0, NULL},
			{(char*)"localize", (char*)"global_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &param->global_beam_minlikelihood, 0, NULL},

			{(char*)"sensor_board_1", (char*)"x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
			{(char*)"sensor_board_1", (char*)"y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
			{(char*)"sensor_board_1", (char*)"z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
			{(char*)"sensor_board_1", (char*)"roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
			{(char*)"sensor_board_1", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
			{(char*)"sensor_board_1", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},

			{(char*)"velodyne", (char*)"x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char*)"velodyne", (char*)"y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char*)"velodyne", (char*)"z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char*)"velodyne", (char*)"roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char*)"velodyne", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char*)"velodyne", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},
			{(char*)"velodyne", (char*)"vertical_resolution", CARMEN_PARAM_INT, &velodyne_params.vertical_resolution, 0, NULL},

			{(char*)"localize_ackerman", (char*) "velodyne_laser_max_range", CARMEN_PARAM_DOUBLE, &velodyne_params.range_max, 0, NULL},
			{(char*)"slam_icp", (char*) "initialization_time", CARMEN_PARAM_DOUBLE, &slam_icp_initialization_time, 0, NULL},
			{(char*)"slam_icp", (char*) "update_distance", CARMEN_PARAM_DOUBLE, &slam_icp_update_distance, 0, NULL},
			{(char*)"slam_icp", (char*) "update_angle", CARMEN_PARAM_DOUBLE, &slam_icp_update_angle, 0, NULL},
			{(char*)"slam_icp", (char*) "highest_point", CARMEN_PARAM_DOUBLE, &highest_point, 0, NULL},
			{(char*)"velodyne", (char*)"time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &velodyne_params.time_spent_by_each_scan, 0, NULL},

			{(char*)"mapper", (char*)"velodyne_locc", CARMEN_PARAM_DOUBLE, &velodyne_params.log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_lfree", CARMEN_PARAM_DOUBLE, &velodyne_params.log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_l0", CARMEN_PARAM_DOUBLE, &velodyne_params.log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &velodyne_params.unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_lambda_short_min", CARMEN_PARAM_DOUBLE, &velodyne_params.lambda_short_min, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_lambda_short_max", CARMEN_PARAM_DOUBLE, &velodyne_params.lambda_short_max, 0, NULL},
			{(char*)"mapper",  (char*)"velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &velodyne_params.range_max_factor, 0, NULL}


	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
	param->integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);
	sensor_board_1_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);
	slam_icp_update_angle = carmen_degrees_to_radians(slam_icp_update_angle);

	velodyne_params.alive = 1;

	velodyne_params.pose = velodyne_pose;
	velodyne_params.sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, velodyne_params.pose.position, sensor_board_1_to_car_matrix);

	velodyne_params.height = velodyne_params.sensor_robot_reference.z + robot_wheel_radius;
	
	velodyne_params.ray_order = carmen_velodyne_get_ray_order();
	velodyne_params.vertical_correction = carmen_velodyne_get_vertical_correction();

	velodyne_params.delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
	velodyne_params.delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

	velodyne_params.sensor_to_board_matrix = create_rotation_matrix(velodyne_params.pose.orientation);
	velodyne_params.current_range_max = velodyne_params.range_max;

	velodyne_params.sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, velodyne_params.pose.position, sensor_board_1_to_car_matrix);
	velodyne_params.height = velodyne_params.sensor_robot_reference.z + robot_wheel_radius;
	
	velodyne_params.remission_calibration = (double *) calloc(256 * velodyne_params.vertical_resolution, sizeof(double));

	FILE *f = fopen(remission_file, "r");
	for (i = 0; i < 256 * velodyne_params.vertical_resolution; i++)
	{
		fscanf(f, "%lf", &velodyne_params.remission_calibration[i]);
	}
	fclose(f);

	velodyne_data.point_cloud_index = 0;

	p_map_params->width = 2 * velodyne_params.range_max;
	p_map_params->height = 2 * velodyne_params.range_max;
	p_map_params->grid_sx = p_map_params->width /  p_map_params->grid_res;
	p_map_params->grid_sy = p_map_params->height /  p_map_params->grid_res;
	p_map_params->grid_size = p_map_params->grid_sx * p_map_params->grid_sy;

	carmen_prob_models_alloc_sensor_data(&velodyne_data, velodyne_params.vertical_resolution);
	carmen_param_allow_unfound_variables(1);
}


/**
 *  Messages registrations and subscriptions
 */

static void 
shutdown_localize(int x)
{
	if (x == SIGINT) 
	{
		carmen_verbose("Disconnecting from IPC network.\n");
		exit(1);
	}
}


int 
register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	/* register globalpos message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	carmen_velodyne_define_messages();

	return 0;
}


static void
subscribe_to_ipc_message()
{
	/* subscribe to initialization messages */
	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler,	CARMEN_SUBSCRIBE_LATEST);
	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
init_velodyne_points()
{
	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	unsigned char **intensity = (unsigned char **)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char*));
	double *phi = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));

	velodyne_data.points = velodyne_points;
	velodyne_data.intensity = intensity;
	velodyne_data.robot_pose = robot_pose;
	velodyne_data.robot_timestamp = robot_timestamp;
	velodyne_data.robot_velocity = robot_velocity;
	velodyne_data.robot_phi = phi;

	initializeICP();
}


int 
main(int argc, char **argv) 
{ 
	carmen_localize_ackerman_param_t param;
	ProbabilisticMapParams map_params;

	/* initialize carmen */
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	/* Setup exit handler */
	signal(SIGINT, shutdown_localize);

	if (argc < 2)
		exit(printf("Use %s <remission-calibration-data.txt>\n", argv[0]));

	strcpy(remission_file, argv[1]);

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv, &param, &map_params);

	/* Allocate memory for the particle filter */
	init_velodyne_points();

	/* register localize related messages */
	register_ipc_messages();

	/* subscribe to localize related messages */
	subscribe_to_ipc_message();

	/* Loop forever */
	carmen_ipc_dispatch();

	return 0;
}

