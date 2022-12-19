#include <carmen/carmen.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/xsens_interface.h>
#include <carmen/car_model.h>
#include <carmen/task_manager_interface.h>
#include <carmen/task_manager_messages.h>

#include <prob_measurement_model.h>
#include <prob_map.h>

#include <tf.h>
#include <carmen/mapper.h>

#include "localize_ackerman_core.h"
#include "localize_ackerman_messages.h"
#include "localize_ackerman_interface.h"
#include "localize_ackerman_velodyne.h"
#include "localize_ackerman_beta_particle_filter.h"
#include "localize_ackerman_trailers_theta.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;


static int necessary_maps_available = 0;

carmen_base_ackerman_odometry_message base_ackerman_odometry_vector[BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE];
int base_ackerman_odometry_index = -1;

carmen_fused_odometry_message fused_odometry_vector[FUSED_ODOMETRY_VECTOR_SIZE];
int g_fused_odometry_index = -1;

/* global variables */
carmen_map_t *new_map = NULL;
carmen_localize_ackerman_map_t localize_map;
carmen_localize_ackerman_particle_filter_p filter;
carmen_localize_ackerman_particle_filter_p beta_filter;
carmen_localize_ackerman_summary_t summary;

carmen_map_t local_map;
carmen_map_t local_sum_remission_map;
carmen_map_t local_mean_remission_map;
carmen_map_t local_variance_remission_map;
carmen_map_t local_sum_sqr_remission_map;
carmen_map_t local_count_remission_map;

carmen_compact_map_t local_compacted_map;
carmen_compact_map_t local_compacted_mean_remission_map;
carmen_compact_map_t local_compacted_variance_remission_map;
carmen_localize_ackerman_binary_map_t binary_map;

carmen_robot_ackerman_laser_message front_laser;

carmen_xsens_global_quat_message *xsens_global_quat_message = NULL;

// variables read via read_parameters()
carmen_robot_ackerman_config_t car_config;
carmen_semi_trailers_config_t semi_trailer_config;

extern int robot_publish_odometry;

int number_of_sensors;
extern sensor_parameters_t *spherical_sensor_params;
extern sensor_data_t *spherical_sensor_data;

extern int use_raw_laser;
extern int mapping_mode;

extern char *save_globalpos_file;
extern double save_globalpos_timestamp;
FILE *globalpos_file = NULL;

carmen_localize_ackerman_globalpos_message globalpos = {};

//extern double laser_ranges[10000];

cell_coords_t **map_cells_hit_by_each_rays = NULL;

carmen_point_t g_std;
int g_reinitiaze_particles = 10;
bool global_localization_requested = false;

carmen_behavior_selector_path_goals_and_annotations_message *behavior_selector_path_goals_and_annotations_message = NULL;

carmen_lidar_config lidar_config[MAX_NUMBER_OF_LIDARS + 10];
double robot_wheel_radius;
double highest_sensor;
char *calibration_file = NULL;
int number_of_threads = 1;
int mapping_mode = 0;
carmen_pose_3D_t velodyne_pose;
double safe_range_above_sensors;

#define GPS_XYZ_VECTOR_SIZE	10
carmen_gps_xyz_message gps_xyz_vector[GPS_XYZ_VECTOR_SIZE];
int g_gps_xyz_index = -1;

tf::Transformer tf_transformer;
extern double gps_correction_factor;
extern carmen_pose_3D_t sensor_board_1_pose;
extern carmen_pose_3D_t gps_pose_in_the_car;

// theta parameters
carmen_velodyne_partial_scan_message *last_velodyne_message;
carmen_velodyne_variable_scan_message *last_variable_message;
extern int lidar_to_compute_theta;


static void
publish_particles_name(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary,
			char *message_name, double timestamp)
{
	static carmen_localize_ackerman_particle_message pmsg;
	IPC_RETURN_TYPE err;

	pmsg.timestamp = timestamp;
	pmsg.host = carmen_get_host();
	pmsg.globalpos = summary->mean;
	pmsg.globalpos_std = summary->std;
	pmsg.num_particles = filter->param->num_particles;
	pmsg.particles = filter->particles;

	err = IPC_publishData(message_name, &pmsg);
	carmen_test_ipc_exit(err, "Could not publish", message_name);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_globalpos(carmen_localize_ackerman_summary_p summary, double v, double phi, double timestamp)
{
	if (!global_localization_requested)
		return;

	if (g_reinitiaze_particles)
	{
		g_reinitiaze_particles--;
//		globalpos.beta = 0.0;
//		return;
	}

	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();
	globalpos.globalpos = summary->mean;
	globalpos.globalpos_std = summary->std;
	globalpos.odometrypos = summary->odometry_pos;
	globalpos.globalpos_xy_cov = summary->xy_cov;
	globalpos.v = v;
	globalpos.phi = phi;
	globalpos.converged = summary->converged;
	globalpos.num_trailers = 0;

	static double last_timestamp = 0.0;
	if (last_timestamp == 0.0)
		last_timestamp = timestamp;
	if (semi_trailer_config.num_semi_trailers > 0)
	{
		globalpos.num_trailers = 1;
		globalpos.semi_trailer_engaged = 1;
		carmen_robot_and_trailers_traj_point_t robot_and_trailer_traj_point =
		{
				globalpos.globalpos.x,
				globalpos.globalpos.y,
				globalpos.globalpos.theta,
				globalpos.num_trailers,
				{0.0},
				globalpos.v,
				globalpos.phi
		};

		double delta_t = globalpos.timestamp - last_timestamp;

		if (last_velodyne_message)
			globalpos.trailer_theta[0] = compute_semi_trailer_theta1(robot_and_trailer_traj_point, delta_t,
				car_config, semi_trailer_config, spherical_sensor_params, (void*)last_velodyne_message, -1);
		else if (last_variable_message)
			globalpos.trailer_theta[0] = compute_semi_trailer_theta1(robot_and_trailer_traj_point, delta_t,
				car_config, semi_trailer_config, spherical_sensor_params, (void*)last_variable_message, lidar_to_compute_theta);
	
	}
	else
	{
		globalpos.semi_trailer_engaged = 0;
		globalpos.trailer_theta[0] = globalpos.globalpos.theta;

	}
	globalpos.semi_trailer_type = semi_trailer_config.num_semi_trailers;
	last_timestamp = timestamp;

	if (g_fused_odometry_index == -1)
	{
		globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = globalpos.pose.position.z = 0.0;
		globalpos.velocity.x = globalpos.velocity.y = globalpos.velocity.z = 0.0;

	}
	else
	{	// Aproveita alguns dados da fused_odometry. 
		// Os valores referentes aa globalpos corrente sao escritos abaixo.
		globalpos.pose = fused_odometry_vector[get_fused_odometry_index_by_timestamp(timestamp)].pose; 	
		globalpos.velocity = fused_odometry_vector[get_fused_odometry_index_by_timestamp(timestamp)].velocity;

	}

	globalpos.pose.orientation.yaw = globalpos.globalpos.theta;

	globalpos.pose.position.x = globalpos.globalpos.x;
	globalpos.pose.position.y = globalpos.globalpos.y;
	globalpos.pose.position.z = 0;
	globalpos.velocity.x = v;
	
	//globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = 0.0;

	if (save_globalpos_file)
	{
		if (globalpos_file == NULL)
			globalpos_file = fopen(save_globalpos_file, "w");
		if (globalpos_file && (timestamp >= save_globalpos_timestamp))
			fprintf(globalpos_file, "%lf %lf %lf %lf %lf %lf\n",
					globalpos.pose.position.x, globalpos.pose.position.y,
					globalpos.pose.orientation.yaw, v, phi, timestamp);
	}

	carmen_localize_ackerman_publish_globalpos_message(&globalpos);
}


void
publish_particles_prediction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, double timestamp)
{
	publish_particles_name(filter, summary, (char *) CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME, timestamp);
}


void
publish_particles_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, double timestamp)
{
	publish_particles_name(filter, summary, (char *) CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME, timestamp);
}


static void
publish_sensor(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, int num_readings,
		double *range, carmen_laser_laser_config_t laser_config, int front, double timestamp)
{
	static carmen_localize_ackerman_sensor_message sensor;
	IPC_RETURN_TYPE err;

	sensor.timestamp = timestamp;
	sensor.host = carmen_get_host();
	if (front)
	{
		sensor.pose.x = summary->mean.x + filter->param->front_laser_offset * cos(summary->mean.theta);
		sensor.pose.y = summary->mean.y + filter->param->front_laser_offset * sin(summary->mean.theta);
		sensor.pose.theta = summary->mean.theta;
		sensor.num_laser = 1;
	}
	else
	{
		sensor.pose.x = summary->mean.x + filter->param->rear_laser_offset * cos(summary->mean.theta + M_PI);
		sensor.pose.y = summary->mean.y + filter->param->rear_laser_offset * sin(summary->mean.theta + M_PI);
		sensor.pose.theta = summary->mean.theta + M_PI;
		sensor.num_laser = 2;
	}
	sensor.num_readings = num_readings;
	sensor.laser_skip = filter->param->laser_skip;
	sensor.config = laser_config;
	sensor.range = range;
	sensor.mask = filter->laser_mask;
	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, &sensor);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);
}


static void
publish_first_globalpos(carmen_localize_ackerman_initialize_message *initialize_msg)
{
	carmen_localize_ackerman_globalpos_message globalpos_ackerman_message = {};
	globalpos_ackerman_message.globalpos = *initialize_msg->mean;
	globalpos_ackerman_message.globalpos_std = *initialize_msg->std;
	globalpos_ackerman_message.odometrypos = *initialize_msg->std;

	globalpos_ackerman_message.timestamp = initialize_msg->timestamp;
	globalpos_ackerman_message.host = carmen_get_host();

	globalpos_ackerman_message.converged = 0;
	globalpos_ackerman_message.globalpos_xy_cov = 0.0;
	globalpos_ackerman_message.phi = 0.0;
	globalpos_ackerman_message.v = 0.0;

	globalpos_ackerman_message.num_trailers = initialize_msg->num_trailers;
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		globalpos_ackerman_message.trailer_theta[z] = initialize_msg->trailer_theta[z];
	globalpos.num_trailers = initialize_msg->num_trailers;
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		globalpos.trailer_theta[z] = initialize_msg->trailer_theta[z];
	
	globalpos_ackerman_message.semi_trailer_engaged = globalpos.semi_trailer_engaged;
	globalpos_ackerman_message.semi_trailer_type = globalpos.semi_trailer_type;

	carmen_localize_ackerman_publish_globalpos_message(&globalpos_ackerman_message);
}


static void
publish_globalpos_on_mapping_mode(carmen_fused_odometry_message *msg, double timestamp)
{
	if (g_fused_odometry_index != -1)
	{
		IPC_RETURN_TYPE err;
		carmen_pose_3D robot_pose = msg->pose;
		double dt = timestamp - msg->timestamp;
		// TODO @@@ Alberto: se dt for grande nao era o caso de nao publicar?
		robot_pose = carmen_ackerman_interpolated_robot_position_at_time(robot_pose, dt, msg->velocity.x, msg->phi,
				car_config.distance_between_front_and_rear_axles);

		globalpos.timestamp = timestamp;
		globalpos.host = carmen_get_host();
		globalpos.v = msg->velocity.x;
		globalpos.phi = msg->phi;
		globalpos.pose = robot_pose;
		globalpos.velocity = msg->velocity;
		globalpos.globalpos.x = globalpos.pose.position.x;
		globalpos.globalpos.y = globalpos.pose.position.y;
		globalpos.globalpos.theta = globalpos.pose.orientation.yaw;

		if (save_globalpos_file)
		{
			if (globalpos_file == NULL)
				globalpos_file = fopen(save_globalpos_file, "w");
			if (globalpos_file && (timestamp >= save_globalpos_timestamp))
				fprintf(globalpos_file, "%lf %lf %lf %lf %lf %lf %lf\n",
						globalpos.globalpos.x, globalpos.globalpos.y, 0.0,
						globalpos.globalpos.theta, globalpos.v, globalpos.phi, timestamp);
		}

		err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
		carmen_test_ipc_exit(err, "Could not publish",	CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
	}
}


static void
publish_carmen_base_ackerman_odometry()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_base_ackerman_odometry_message base_ackerman_odometry;

	base_ackerman_odometry.x = filter->particles[0].x;
	base_ackerman_odometry.y = filter->particles[0].y;
	base_ackerman_odometry.theta = filter->particles[0].theta;
	base_ackerman_odometry.v = filter->particles[0].v;
	base_ackerman_odometry.phi = filter->particles[0].phi;
	base_ackerman_odometry.timestamp = carmen_get_time();
	base_ackerman_odometry.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &base_ackerman_odometry);
	carmen_test_ipc(err, "Could not publish carmen_base_ackerman_odometry_message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
get_gps_xyz_index_by_timestamp(double timestamp)
{
	double min_diff, diff;
	int min_index;

	min_diff = DBL_MAX;
	min_index = -1;

	for (int i = 0; i < GPS_XYZ_VECTOR_SIZE; i++)
	{
		diff = fabs(gps_xyz_vector[i].timestamp - timestamp);

		if (diff < min_diff)
		{
			min_diff = diff;
			min_index = i;
		}
	}

	return (min_index);
}


static tf::Vector3
carmen_vector3_to_tf_vector3(carmen_vector_3D_t carmen_vector)
{
	tf::Vector3 tf_vector(carmen_vector.x, carmen_vector.y, carmen_vector.z);

	return (tf_vector);
}


static carmen_vector_3D_t
tf_vector3_to_carmen_vector3(tf::Vector3 tf_vector)
{
	carmen_vector_3D_t carmen_vector;
	carmen_vector.x = tf_vector.x();
	carmen_vector.y = tf_vector.y();
	carmen_vector.z = tf_vector.z();

	return (carmen_vector);
}


static tf::Quaternion
carmen_rotation_to_tf_quaternion(carmen_orientation_3D_t carmen_orientation)
{
	tf::Quaternion tf_quat(carmen_orientation.yaw, carmen_orientation.pitch, carmen_orientation.roll);

	return tf_quat;
}


static carmen_vector_3D_t
carmen_ackerman_interpolated_robot_position_at_time(carmen_vector_3D_t robot_pose, double dt, double v, double theta)
{
	carmen_vector_3D_t pose = robot_pose;
	int i;
	int steps = 1;
	double ds;

	ds = v * (dt / (double) steps);

	for (i = 0; i < steps; i++)
	{
		pose.x = pose.x + ds * cos(theta);
		pose.y = pose.y + ds * sin(theta);
	}

	return (pose);
}


void
initialize_tf_transforms()
{
	tf::Transform board_to_gps_pose;
	tf::Transform car_to_board_pose;

	tf::Time::init();

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_1_pose.position.x, sensor_board_1_pose.position.y, sensor_board_1_pose.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_1_pose.orientation.yaw, sensor_board_1_pose.orientation.pitch, sensor_board_1_pose.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	tf_transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// gps pose with respect to the board
	board_to_gps_pose.setOrigin(tf::Vector3(gps_pose_in_the_car.position.x, gps_pose_in_the_car.position.y, gps_pose_in_the_car.position.z));
	board_to_gps_pose.setRotation(tf::Quaternion(gps_pose_in_the_car.orientation.yaw, gps_pose_in_the_car.orientation.pitch, gps_pose_in_the_car.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_gps_transform(board_to_gps_pose, tf::Time(0), "/board", "/gps");
	tf_transformer.setTransform(board_to_gps_transform, "board_to_gps_transform");
}


carmen_vector_3D_t
get_car_pose_from_gps_pose(carmen_gps_xyz_message *gps_xyz_message, double theta, double v, double timestamp)
{
	tf::StampedTransform gps_to_car;
	tf_transformer.lookupTransform((char *) "/gps", (char *) "/car", tf::Time(0), gps_to_car);

	carmen_vector_3D_t gps_position;
	gps_position.x = gps_xyz_message->x;
	gps_position.y = gps_xyz_message->y;
	gps_position.z = gps_xyz_message->z;

	tf::Transform global_to_gps;
	global_to_gps.setOrigin(carmen_vector3_to_tf_vector3(gps_position));
	global_to_gps.setRotation(carmen_rotation_to_tf_quaternion({0.0, 0.0, theta}));

	tf::Transform global_to_car = global_to_gps * gps_to_car;

	carmen_vector_3D_t car_position = tf_vector3_to_carmen_vector3(global_to_car.getOrigin());

	car_position = carmen_ackerman_interpolated_robot_position_at_time(car_position, timestamp - gps_xyz_message->timestamp, v, theta);

	return (car_position);
}


void
gps_xyz_correction(carmen_localize_ackerman_particle_filter_t *xt_1, double timestamp)
{
	if (g_gps_xyz_index == -1)
		return;

	carmen_gps_xyz_message *gps_xyz_message = &(gps_xyz_vector[get_gps_xyz_index_by_timestamp(timestamp)]);

	if (fabs(gps_xyz_message->timestamp - timestamp) > 0.3)
		return;

	double gps_sigma_squared = 100.0 * 100.0;
	switch (gps_xyz_message->gps_quality)
	{
	case 1:
		gps_sigma_squared = 8.0 * 8.0;
		break;
	case 2:
		gps_sigma_squared = 4.0 * 4.0;
		break;
	case 4:
		gps_sigma_squared = 1.0 * 1.0;
		break;
	case 5:
	case 6:
		gps_sigma_squared = 2.0 * 2.0;
		break;
	}

	double normalization_factor = sqrt((2.0 * 2.0) * 2.0 * M_PI); // gps_weight = 1.0 no máximo com gps_xyz_message->gps_quality == 5
	for (int i = 0; i < xt_1->param->num_particles; i++)
	{
		carmen_vector_3D_t estimated_car_pose = get_car_pose_from_gps_pose(gps_xyz_message, xt_1->particles[i].theta,
				xt_1->particles[i].v, timestamp);
		double distance = DIST2D(xt_1->particles[i], estimated_car_pose);
		double distance_squared = distance * distance;
		double gps_weight = exp(-distance_squared / gps_sigma_squared) / sqrt(gps_sigma_squared * 2.0 * M_PI);

		// if ((xt_1->particles[i].weight + gps_weight * gps_correction_factor * normalization_factor) > 0)
		// 	printf("gps_correction: %lf %lf\n", xt_1->particles[i].weight, xt_1->particles[i].weight + gps_weight * gps_correction_factor * normalization_factor);
		xt_1->particles[i].weight = xt_1->particles[i].weight + gps_weight * gps_correction_factor * normalization_factor;
	}
//	printf("\n");
}


static void
velodyne_variable_scan_localize(carmen_velodyne_variable_scan_message *message, int sensor)
{	// For cameras and other sensors (not Velodyne)
	int odometry_index, fused_odometry_index;
	int velodyne_initilized;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(message->timestamp);
	fused_odometry_index = get_fused_odometry_index_by_timestamp(message->timestamp);

	if (mapping_mode)
	{
		publish_globalpos_on_mapping_mode(&fused_odometry_vector[fused_odometry_index], message->timestamp);
		return;
	}

	if (!necessary_maps_available || !global_localization_requested ||
		((base_ackerman_odometry_index < 0) && (filter->param->prediction_type != 2)))
		return;

	velodyne_initilized = localize_ackerman_velodyne_variable_scan_build_instanteneous_maps(message, &spherical_sensor_params[sensor], 
			&spherical_sensor_data[sensor], base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi);
	if (!velodyne_initilized)
		return;

	carmen_localize_ackerman_velodyne_prediction(filter, &base_ackerman_odometry_vector[odometry_index],
			xsens_global_quat_message,
			message->timestamp, car_config.distance_between_front_and_rear_axles);

	publish_particles_prediction(filter, &summary, message->timestamp);

	carmen_localize_ackerman_velodyne_correction(filter, &localize_map, &local_compacted_map, &local_compacted_mean_remission_map,
			&local_compacted_variance_remission_map, &binary_map);
	gps_xyz_correction(filter, message->timestamp);

	publish_particles_correction(filter, &summary, message->timestamp);

//	if (fabs(base_ackerman_odometry_vector[odometry_index].v) > 0.2)
	{
		carmen_localize_ackerman_velodyne_resample(filter);
	}

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);

		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v,
				base_ackerman_odometry_vector[odometry_index].phi, message->timestamp);

		if ((filter->param->prediction_type == 2) && !robot_publish_odometry)
			publish_carmen_base_ackerman_odometry();
	}

	if (g_reinitiaze_particles)
		carmen_localize_ackerman_initialize_particles_gaussians(filter, 1, &(summary.mean), &g_std);
}


////////////////////////////////////// RANIK ///////////////////////////////////////////////////////

double*
convert_map_to_array(carmen_map_p map)
{
    unsigned int width = map->config.x_size;
    unsigned int height = map->config.y_size;
    unsigned int size = width * height;
    unsigned int row = 0, col = 0, index = 0;
    double *array_map = (double *) malloc (size * sizeof(double));

    for (unsigned int i = 0; i < size; ++i)
    {
        row = (height - 1) - i % height;

        col = i / height;

        index = row * width + col;

        array_map[index] =  1 - map->complete_map[i];

//    	if (map->complete_map[i] != 0.0)
//    		printf("%lf\n", map->complete_map[i]);
    }
    return (array_map);

    exit(0);
}


void
cv_draw_map(carmen_map_p map)
{
	double *inverted_map = convert_map_to_array(map);

	Mat image_localize_map = Mat(map->config.x_size, map->config.y_size, CV_64FC1 , inverted_map, 0);

	imshow("Localize Map", image_localize_map);
	waitKey(1);
	free(inverted_map);
}


void
cv_draw_map_rotated(carmen_map_p map, double x, double y, double rotation_angle)
{
	double *inverted_map = convert_map_to_array(map);

	unsigned int height = map->config.y_size;
	double inverse_resolution = 1.0 / map->config.resolution;

	unsigned int row = height - floor((y - map->config.y_origin) * inverse_resolution + 0.5);
	unsigned int col = floor((x - map->config.x_origin) * inverse_resolution + 0.5);

	Mat image_localize_map = Mat(map->config.x_size, map->config.y_size, CV_64FC1 , inverted_map, 0);

//	//printf("%lf %lf %d %d\n", x, y, (int)x - 125, (int)y - 125);
//	if (row  > (unsigned int)map->config.x_size || col > (unsigned int) map->config.x_size)
//	{
//		printf("Out of map\n");
//		return;
//	}

	Mat roteted_map;
	//Mat rotation_matrix = getRotationMatrix2D(Point(map->config.x_size / 2, map->config.y_size / 2), 90 + rotation_angle, 1.0);
	Mat rotation_matrix = getRotationMatrix2D(Point(col, row), -rotation_angle, 1.0);
	warpAffine(image_localize_map, roteted_map, rotation_matrix, Size(image_localize_map.cols, image_localize_map.rows), INTER_NEAREST);


	//Rect myROI(col - 125, row - 125, 250, 250);
	//image_localize_map = image_localize_map(myROI);
	Rect myROI(col - 150, row - 150, 300, 300);
	roteted_map = roteted_map(myROI);

	//circle(image_localize_map, Point(x, y), 1, cvScalar(0, 0, 255), 25, 8, 0);

//	imshow("Localize Map", roteted_map);
	imshow("Localize Map", image_localize_map);
	waitKey(1);
	free(inverted_map);
}


void
cv_draw_compact_map(carmen_compact_map_t *compact_local_map)
{
	carmen_map_t local_map;
	carmen_grid_mapping_create_new_map(&local_map, compact_local_map->config.x_size, compact_local_map->config.y_size, compact_local_map->config.resolution, 'c');
	memset(local_map.complete_map, 0, local_map.config.x_size * local_map.config.y_size * sizeof(double));
	carmen_prob_models_uncompress_compact_map(&local_map, compact_local_map);

	double *inverted_map = convert_map_to_array(&local_map);

	Mat image_local_map = Mat(local_map.config.x_size, local_map.config.y_size, CV_64F , inverted_map, 0);

//	Rect myROI((local_map.config.x_size / 2) - 150, (local_map.config.y_size / 2) - 150, 300, 300);
//	image_local_map = image_local_map(myROI);

	imshow("Local Map", image_local_map);
	waitKey(1);

	free (local_map.complete_map);
	free (local_map.map);
	free(inverted_map);
}

double*
get_unknown_region(carmen_map_p map)
{
    unsigned int width = map->config.x_size;
    unsigned int height = map->config.y_size;
    unsigned int size = width * height;
    unsigned int row = 0, col = 0, index = 0;
    double *array_map = (double *) malloc (size * sizeof(double));

    for (unsigned int i = 0; i < size; ++i)
    {
        row = (height - 1) - i % height;

        col = i / height;

        index = row * width + col;

        if (map->complete_map[i] < 0.0)
        	array_map[index] =  0.0;
        else
        	array_map[index] =  1.0;

//    	if (map->complete_map[i] != 0.0)
//    		printf("%lf\n", map->complete_map[i]);
    }
    return (array_map);

    exit(0);
}


void
cv_draw_map_unknown_region(carmen_map_p map, double x, double y, double rotation_angle)
{
	double *inverted_map = get_unknown_region(map);

	unsigned int height = map->config.y_size;
	double inverse_resolution = 1.0 / map->config.resolution;

	unsigned int row = height - floor((y - map->config.y_origin) * inverse_resolution + 0.5);
	unsigned int col = floor((x - map->config.x_origin) * inverse_resolution + 0.5);

	Mat image_localize_map = Mat(map->config.x_size, map->config.y_size, CV_64FC1 , inverted_map, 0);

//	printf("%lf %lf %d %d\n", x, y, (int)x - 125, (int)y - 125);
	if (row  > (unsigned int)map->config.x_size || col > (unsigned int) map->config.x_size)
	{
		printf("Out of map\n");
		return;
	}

	Mat roteted_map;
	//Mat rotation_matrix = getRotationMatrix2D(Point(map->config.x_size / 2, map->config.y_size / 2), 90 + rotation_angle, 1.0);
	Mat rotation_matrix = getRotationMatrix2D(Point(col, row), -rotation_angle, 1.0);
	warpAffine(image_localize_map, roteted_map, rotation_matrix, Size(image_localize_map.cols, image_localize_map.rows), INTER_NEAREST);


	//Rect myROI(col - 125, row - 125, 250, 250);
	//image_localize_map = image_localize_map(myROI);
	Rect myROI(col - 150, row - 150, 300, 300);
	roteted_map = roteted_map(myROI);

	//circle(image_localize_map, Point(x, y), 1, cvScalar(0, 0, 255), 25, 8, 0);

	imshow("Unknown Map", roteted_map);
	waitKey(1);
	free(inverted_map);
}


void
display_maps()
{
	cv_draw_map(&localize_map.carmen_map);
	cv_draw_compact_map(&local_compacted_map);
	//cv_draw_map_unknown_region(&localize_map.carmen_map, summary.mean.x, summary.mean.y, carmen_radians_to_degrees(summary.mean.theta));
}
///////////////////////////// END RANIK ///////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{	// Used by Velodyne.
	int odometry_index, fused_odometry_index;
	int velodyne_initilized;

	last_velodyne_message = velodyne_message;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(velodyne_message->timestamp);
	fused_odometry_index = get_fused_odometry_index_by_timestamp(velodyne_message->timestamp);

	if (mapping_mode)
	{
		publish_globalpos_on_mapping_mode(&fused_odometry_vector[fused_odometry_index], velodyne_message->timestamp);
		return;
	}

	if (!necessary_maps_available || !global_localization_requested ||
		((base_ackerman_odometry_index < 0) && (filter->param->prediction_type != 2)))
		return;

	carmen_current_semi_trailer_data_t semi_trailer_data =
	{
			globalpos.semi_trailer_engaged,
			globalpos.semi_trailer_type,
			semi_trailer_config.semi_trailers[0].d,
			semi_trailer_config.semi_trailers[0].M,
			convert_theta1_to_beta(globalpos.globalpos.theta, globalpos.trailer_theta[0])
	};

	velodyne_initilized = localize_ackerman_velodyne_partial_scan_build_instanteneous_maps(&local_compacted_map, &local_compacted_mean_remission_map, &local_map,
			velodyne_message, &spherical_sensor_params[0], &spherical_sensor_data[0], base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi, semi_trailer_data);
	if (!velodyne_initilized)
		return;

	carmen_localize_ackerman_velodyne_prediction(filter,
			&base_ackerman_odometry_vector[odometry_index], xsens_global_quat_message,
			velodyne_message->timestamp, car_config.distance_between_front_and_rear_axles);

//	carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point =
//	{
//			globalpos.globalpos.x,
//			globalpos.globalpos.y,
//			globalpos.globalpos.theta,
//			globalpos.beta,
//			globalpos.v,
//			globalpos.phi
//	};
//
//	carmen_localize_ackerman_beta_prediction(beta_filter, robot_and_trailer_traj_point, car_config, semi_trailer_config, velodyne_message->timestamp - beta_filter->last_timestamp);

	publish_particles_prediction(filter, &summary, velodyne_message->timestamp);

	carmen_localize_ackerman_velodyne_correction(filter,
			&localize_map, &local_compacted_map, &local_compacted_mean_remission_map, &local_compacted_variance_remission_map, &binary_map);
	gps_xyz_correction(filter, velodyne_message->timestamp);

//	carmen_localize_ackerman_beta_correction(beta_filter,
//			&localize_map, &local_compacted_map, &local_compacted_mean_remission_map, &local_compacted_variance_remission_map, &binary_map);

	publish_particles_correction(filter, &summary, velodyne_message->timestamp);

	if (filter->initialized)
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);

//	if (beta_filter->initialized)
//		carmen_localize_ackerman_summarize_beta(filter, &summary);

	// if (fabs(base_ackerman_odometry_vector[odometry_index].v) > 0.2)
	{
		carmen_localize_ackerman_velodyne_resample(filter);

//		carmen_localize_ackerman_beta_resample(beta_filter);
	}

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);
		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi,
				velodyne_message->timestamp);

		if ((filter->param->prediction_type == 2) && !robot_publish_odometry)
			publish_carmen_base_ackerman_odometry();

//		static bool ft = true;
//		static double init_t = 0.0;
//		if (ft)
//		{
//			init_t = globalpos.timestamp;
//			ft = false;
//		}
//
//		FILE *caco = fopen("caco_gpos.txt", "a");
//		fprintf(caco, "%lf %lf %lf %lf %lf\n", globalpos.timestamp - init_t, velodyne_message->timestamp - init_t,
//				base_ackerman_odometry_vector[odometry_index].timestamp - init_t,
//				base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi);
//		fflush(caco);
//		fclose(caco);
	}

	if (g_reinitiaze_particles)
		carmen_localize_ackerman_initialize_particles_gaussians(filter, 1, &(summary.mean), &g_std);

	//display_maps();             // Ranik Display a crop of local and offline maps
}


static void
velodyne_variable_scan_message_handler1(carmen_velodyne_variable_scan_message *message)
{	// Variable scan because you can have more or less than 32 vertical rays. Used by stereo cameras or kinect.
	velodyne_variable_scan_localize(message, 1);
}


static void
velodyne_variable_scan_message_handler2(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 2);
}


static void
velodyne_variable_scan_message_handler3(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 3);
}


static void
velodyne_variable_scan_message_handler4(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 4);
}


static void
velodyne_variable_scan_message_handler5(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 5);
}


static void
velodyne_variable_scan_message_handler6(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 6);
}


static void
velodyne_variable_scan_message_handler7(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 7);
}


static void
velodyne_variable_scan_message_handler8(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 8);
}


static void
velodyne_variable_scan_message_handler9(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 9);
}


static void
localize_using_lidar(int sensor_number, carmen_velodyne_variable_scan_message *msg)
{
	int odometry_index, fused_odometry_index, instanteneous_maps_ok = 0;

	if (sensor_number == lidar_to_compute_theta)
		last_variable_message = msg;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(msg->timestamp);
	fused_odometry_index = get_fused_odometry_index_by_timestamp(msg->timestamp);

	if (mapping_mode)
	{
		publish_globalpos_on_mapping_mode(&fused_odometry_vector[fused_odometry_index], msg->timestamp);
		return;
	}

	if (!necessary_maps_available || !global_localization_requested || ((base_ackerman_odometry_index < 0) && (filter->param->prediction_type != 2)))
		return;

	carmen_current_semi_trailer_data_t semi_trailer_data =
	{
			globalpos.semi_trailer_engaged,
			globalpos.semi_trailer_type,
			semi_trailer_config.semi_trailers[0].d,
			semi_trailer_config.semi_trailers[0].M,
			convert_theta1_to_beta(globalpos.globalpos.theta, globalpos.trailer_theta[0])
	};

	instanteneous_maps_ok = localize_ackerman_variable_scan_build_instanteneous_maps(msg, &spherical_sensor_params[sensor_number], 
			&spherical_sensor_data[sensor_number], base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi, semi_trailer_data);
	
	if (!instanteneous_maps_ok)
		return;

	// TUDO the filter should be one for each lidar?
	carmen_localize_ackerman_velodyne_prediction(filter, &base_ackerman_odometry_vector[odometry_index], xsens_global_quat_message,
			msg->timestamp, car_config.distance_between_front_and_rear_axles);

	publish_particles_prediction(filter, &summary, msg->timestamp);

	carmen_localize_ackerman_velodyne_correction(filter, &localize_map, &local_compacted_map, &local_compacted_mean_remission_map,
			&local_compacted_variance_remission_map, &binary_map);

	publish_particles_correction(filter, &summary, msg->timestamp);

	if (filter->initialized)
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);

	// if (fabs(base_ackerman_odometry_vector[odometry_index].v) > 0.2)
	{
		carmen_localize_ackerman_velodyne_resample(filter);
	}

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);
		//TODO Transformar em parametro @@@Vinicius
		if (sensor_number == 10)
			publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi,	msg->timestamp);

		if ((filter->param->prediction_type == 2) && !robot_publish_odometry)
			publish_carmen_base_ackerman_odometry();
	}

	if (g_reinitiaze_particles)
		carmen_localize_ackerman_initialize_particles_gaussians(filter, 1, &(summary.mean), &g_std);
}

void
variable_scan_message_handler_0(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(10, message);
}

void
variable_scan_message_handler_1(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(11, message);
}

void
variable_scan_message_handler_2(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(12, message);
}

void
variable_scan_message_handler_3(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(13, message);
}

void
variable_scan_message_handler_4(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(14, message);
}

void
variable_scan_message_handler_5(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(15, message);
}

void
variable_scan_message_handler_6(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(16, message);
}

void
variable_scan_message_handler_7(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(17, message);
}

void
variable_scan_message_handler_8(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(18, message);
}

void
variable_scan_message_handler_9(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(19, message);
}

void
variable_scan_message_handler_10(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(20, message);
}

void
variable_scan_message_handler_11(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(21, message);
}

void
variable_scan_message_handler_12(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(22, message);
}

void
variable_scan_message_handler_13(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(23, message);
}

void
variable_scan_message_handler_14(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(24, message);
}

void
variable_scan_message_handler_15(carmen_velodyne_variable_scan_message *message)
{
	localize_using_lidar(25, message);
}


static void
robot_ackerman_frontlaser_handler(carmen_robot_ackerman_laser_message *flaser)
{
	if (!necessary_maps_available)
		return;

	carmen_localize_ackerman_run(filter, &localize_map, flaser, filter->param->front_laser_offset, 0,
			&base_ackerman_odometry_vector[base_ackerman_odometry_index], car_config.distance_between_front_and_rear_axles);

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize(filter, &summary, &localize_map, flaser->num_readings,
				flaser->range, filter->param->front_laser_offset,
				flaser->config.angular_resolution,
				flaser->config.start_angle, 0);
		publish_globalpos(&summary, flaser->v, flaser->phi, flaser->timestamp);
		publish_particles_correction(filter, &summary, flaser->timestamp);
		publish_sensor(filter, &summary, flaser->num_readings, flaser->range, flaser->config, 1, flaser->timestamp);
	}
}


static void
raw_laser_handler(carmen_laser_laser_message *laser)
{
	int odometry_index;

	if (!necessary_maps_available || base_ackerman_odometry_index < 0)
		return;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(laser->timestamp);

	carmen_localize_ackerman_run_with_raw_laser(filter, &localize_map,
			laser, &base_ackerman_odometry_vector[odometry_index],
			filter->param->front_laser_offset, car_config.distance_between_front_and_rear_axles);

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize(filter,
				&summary, &localize_map, laser->num_readings,
				laser->range, filter->param->front_laser_offset,
				laser->config.angular_resolution,
				laser->config.start_angle, 0);
		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v, base_ackerman_odometry_vector[odometry_index].phi,
				laser->timestamp);
		publish_particles_correction(filter, &summary, laser->timestamp);
		publish_sensor(filter, &summary, laser->num_readings, laser->range, laser->config, 1, laser->timestamp);
	}
}


static void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	base_ackerman_odometry_index = (base_ackerman_odometry_index + 1) % BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE;
	base_ackerman_odometry_vector[base_ackerman_odometry_index] = *msg;

//	localize_using_map_set_robot_pose_into_the_map(msg->v, msg->phi, msg->timestamp);
}


static void
fused_odometry_handler(carmen_fused_odometry_message *msg)
{
	static int is_first_fused_odometry_message = 1;

	if (is_first_fused_odometry_message)
	{
		is_first_fused_odometry_message = 0;
		return;
	}

	g_fused_odometry_index = (g_fused_odometry_index + 1) % FUSED_ODOMETRY_VECTOR_SIZE;
	fused_odometry_vector[g_fused_odometry_index] = *msg;
}


static void
carmen_xsens_subscribe_xsens_global_quat_message_handler(carmen_xsens_global_quat_message *message)
{
	xsens_global_quat_message = message;
}


static void
carmen_localize_ackerman_initialize_message_handler(carmen_localize_ackerman_initialize_message *initialize_msg)
{
	if (initialize_msg->distribution == CARMEN_INITIALIZE_GAUSSIAN)
	{
		carmen_localize_ackerman_initialize_particles_gaussians(filter,
			initialize_msg->num_modes, initialize_msg->mean, initialize_msg->std);

		g_std = initialize_msg->std[0];
//		g_std = {0.0, 0.0, 0.0};
		g_reinitiaze_particles = 10;

		filter->last_timestamp = initialize_msg->timestamp;

		publish_first_globalpos(initialize_msg); // Alberto: se publicar pode sujar o mapa devido a inicializacao.
	}
	else if (initialize_msg->distribution == CARMEN_INITIALIZE_UNIFORM)
	{
		//todo pode dar problema aqui se o mapa nao estiver inicializado
		carmen_localize_ackerman_initialize_particles_uniform(filter, &front_laser, &localize_map);
		publish_particles_correction(filter, &summary, initialize_msg->timestamp);
	}

	global_localization_requested = true;
}


static void
localize_map_update_handler(carmen_map_server_localize_map_message *message)
{
	carmen_map_server_localize_map_message_to_localize_map(message, &localize_map);

//	x_origin = message->config.x_origin;
//	y_origin = message->config.y_origin;

	necessary_maps_available = 1;
}


static void
globalpos_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_globalpos_message globalpos;
	//  FORMATTER_PTR formatter;

	if (!necessary_maps_available)
		return;

	/* formatter = */IPC_msgInstanceFormatter(msgRef);
	IPC_freeByteArray(callData);

	globalpos.timestamp = carmen_get_time();
	globalpos.host = carmen_get_host();
	globalpos.globalpos = summary.mean;
	globalpos.globalpos_std = summary.std;
	globalpos.globalpos_xy_cov = summary.xy_cov;
	globalpos.odometrypos = summary.odometry_pos;
	globalpos.converged = summary.converged;

	err = IPC_respondData(msgRef, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


static void 
map_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_map_query_message msg;
	carmen_localize_ackerman_map_message response;

#ifndef NO_ZLIB
	unsigned long compress_buf_size;
	int compress_return;
	unsigned char *compressed_map;
#endif

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &msg,
			sizeof(carmen_localize_ackerman_map_query_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall",
			IPC_msgInstanceName(msgRef));

	response.config = localize_map.config;

	if (msg.map_is_global_likelihood)
	{
		response.map_is_global_likelihood = 1;
		response.data = (unsigned char *) localize_map.complete_gprob;
		response.size = localize_map.config.x_size * localize_map.config.y_size * sizeof(double);
	}
	else
	{
		response.map_is_global_likelihood = 0;
		response.data = (unsigned char *) localize_map.complete_prob;
		response.size = localize_map.config.x_size * localize_map.config.y_size * sizeof(double);
	}

#ifndef NO_ZLIB
	compress_buf_size = response.size * 1.01 + 12;
	compressed_map = (unsigned char *) calloc(compress_buf_size, sizeof(unsigned char));
	carmen_test_alloc(compressed_map);
	compress_return = carmen_compress(
			(unsigned char *)compressed_map,
			(unsigned long *)&compress_buf_size,
			(unsigned char *)response.data,
			(unsigned long)response.size,
			Z_DEFAULT_COMPRESSION);
	if (compress_return != Z_OK) {
		free(compressed_map);
		response.compressed = 0;
	} else {
		response.size = compress_buf_size;
		response.data = compressed_map;
		response.compressed = 1;
	}
#else
	response.compressed = 0;
#endif

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_LOCALIZE_ACKERMAN_MAP_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_LOCALIZE_ACKERMAN_MAP_NAME);
}


static void
carmen_task_manager_set_semi_trailer_type_and_beta_message_handler(carmen_task_manager_set_semi_trailer_type_and_beta_message *message)
{
	if (semi_trailer_config.num_semi_trailers != message->semi_trailer_type)
	{
		char *fake_module_name = (char *) "carmen_task_manager_set_semi_trailer_type_and_beta_message_handler()";
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, 1, &fake_module_name, message->semi_trailer_type);
		globalpos.num_trailers = message->num_trailers;
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			globalpos.trailer_theta[z] = message->trailer_theta[z];
	}
}


static void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *msg)
{
	behavior_selector_path_goals_and_annotations_message = msg;
}


static void
gps_xyz_message_handler(carmen_gps_xyz_message *msg)
{
	static int is_first_gps_xyz_message = 1;

	if (is_first_gps_xyz_message)
	{
		is_first_gps_xyz_message = 0;
		return;
	}

	g_gps_xyz_index = (g_gps_xyz_index + 1) % GPS_XYZ_VECTOR_SIZE;
	gps_xyz_vector[g_gps_xyz_index] = *msg;
}


static void
shutdown_localize(int x)
{
	if (x == SIGINT)
	{
		if (globalpos_file)
			fclose(globalpos_file);

		carmen_verbose("Disconnecting from IPC network.\n");
		exit(1);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


int 
define_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);

	/* register map request message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_MAP_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_NAME);

	/* register globalpos request message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, IPC_VARIABLE_LENGTH, CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);

	carmen_velodyne_define_messages();

	return 0;
}


static void
subscribe_to_ipc_message()
{
	IPC_RETURN_TYPE err;

	carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) carmen_localize_ackerman_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler,	CARMEN_SUBSCRIBE_LATEST);

	if (!mapping_mode)
	{
		if (use_raw_laser)
		{
			carmen_laser_subscribe_laser1_message(NULL, (carmen_handler_t) raw_laser_handler, CARMEN_SUBSCRIBE_LATEST);
			carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		else
		{
			carmen_robot_ackerman_subscribe_frontlaser_message(&front_laser, (carmen_handler_t) robot_ackerman_frontlaser_handler, CARMEN_SUBSCRIBE_LATEST);
		}

		carmen_map_server_subscribe_localize_map_message(NULL, (carmen_handler_t) localize_map_update_handler, CARMEN_SUBSCRIBE_LATEST);

		/* subscribe to map request message */
		// TODO: create proper subscribe messages. Check with Alberto if there is a reason for doing these subscribes with the low level API.
		err = IPC_subscribe(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, map_query_handler, NULL);
		carmen_test_ipc(err, "Could not subscribe", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);
		IPC_setMsgQueueLength(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, 1);

		err = IPC_subscribe(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, globalpos_query_handler, NULL);
		carmen_test_ipc(err, "Could not subscribe", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME);
		IPC_setMsgQueueLength(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, 1);

		// stereo velodyne (cameras stereo)
		if ((number_of_sensors > 0) && spherical_sensor_params[0].alive)
			carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 1) && spherical_sensor_params[1].alive)
			carmen_stereo_velodyne_subscribe_scan_message(1, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler1, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 2) && spherical_sensor_params[2].alive)
			carmen_stereo_velodyne_subscribe_scan_message(2, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler2, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 3) && spherical_sensor_params[3].alive)
			carmen_stereo_velodyne_subscribe_scan_message(3, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler3, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 4) && spherical_sensor_params[4].alive)
			carmen_stereo_velodyne_subscribe_scan_message(4, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler4, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 5) && spherical_sensor_params[5].alive)
			carmen_stereo_velodyne_subscribe_scan_message(5, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler5, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 6) && spherical_sensor_params[6].alive)
			carmen_stereo_velodyne_subscribe_scan_message(6, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler6, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 7) && spherical_sensor_params[7].alive)
			carmen_stereo_velodyne_subscribe_scan_message(7, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler7, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 8) && spherical_sensor_params[8].alive)
			carmen_stereo_velodyne_subscribe_scan_message(8, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler8, CARMEN_SUBSCRIBE_LATEST);

		if ((number_of_sensors > 9) && spherical_sensor_params[9].alive)
			carmen_stereo_velodyne_subscribe_scan_message(9, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler9, CARMEN_SUBSCRIBE_LATEST);

		// IMU
		if (filter->param->prediction_type == 2) // use IMU based prediction
			carmen_xsens_subscribe_xsens_global_quat_message(NULL, (carmen_handler_t) carmen_xsens_subscribe_xsens_global_quat_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		if ((number_of_sensors > 0) && spherical_sensor_params[0].alive)
			carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

		// stereo velodyne camera 3
		if ((number_of_sensors > 3) && spherical_sensor_params[3].alive)
			carmen_stereo_velodyne_subscribe_scan_message(3, NULL, (carmen_handler_t) velodyne_variable_scan_message_handler3, CARMEN_SUBSCRIBE_LATEST);
	}

	// lidars
	if ((number_of_sensors > 10) && spherical_sensor_params[10].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_0, CARMEN_SUBSCRIBE_LATEST, 0);

	if ((number_of_sensors > 11) && spherical_sensor_params[11].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_1, CARMEN_SUBSCRIBE_LATEST, 1);

	if ((number_of_sensors > 12) && spherical_sensor_params[12].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_2, CARMEN_SUBSCRIBE_LATEST, 2);

	if ((number_of_sensors > 13) && spherical_sensor_params[13].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_3, CARMEN_SUBSCRIBE_LATEST, 3);

	if ((number_of_sensors > 14) && spherical_sensor_params[14].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_4, CARMEN_SUBSCRIBE_LATEST, 4);

	if ((number_of_sensors > 15) && spherical_sensor_params[15].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_5, CARMEN_SUBSCRIBE_LATEST, 5);

	if ((number_of_sensors > 16) && spherical_sensor_params[16].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_6, CARMEN_SUBSCRIBE_LATEST, 6);

	if ((number_of_sensors > 17) && spherical_sensor_params[17].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_7, CARMEN_SUBSCRIBE_LATEST, 7);

	if ((number_of_sensors > 18) && spherical_sensor_params[18].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_8, CARMEN_SUBSCRIBE_LATEST, 8);

	if ((number_of_sensors > 19) && spherical_sensor_params[19].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_9, CARMEN_SUBSCRIBE_LATEST, 9);

	if ((number_of_sensors > 20) && spherical_sensor_params[20].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_10, CARMEN_SUBSCRIBE_LATEST, 10);

	if ((number_of_sensors > 21) && spherical_sensor_params[21].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_11, CARMEN_SUBSCRIBE_LATEST, 11);

	if ((number_of_sensors > 22) && spherical_sensor_params[22].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_12, CARMEN_SUBSCRIBE_LATEST, 12);

	if ((number_of_sensors > 23) && spherical_sensor_params[23].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_13, CARMEN_SUBSCRIBE_LATEST, 13);

	if ((number_of_sensors > 24) && spherical_sensor_params[24].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_14, CARMEN_SUBSCRIBE_LATEST, 14);

	if ((number_of_sensors > 25) && spherical_sensor_params[25].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler_15, CARMEN_SUBSCRIBE_LATEST, 15);

	carmen_task_manager_subscribe_set_semi_trailer_type_and_beta_message(NULL, (carmen_handler_t) carmen_task_manager_set_semi_trailer_type_and_beta_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(NULL, (carmen_handler_t) path_goals_and_annotations_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_gps_xyz_subscribe_message(NULL, (carmen_handler_t) gps_xyz_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
init_local_maps(ProbabilisticMapParams map_params)
{
	init_probabilistic_grid_map_model(&map_params, NULL);

	init_carmen_map(&map_params, &local_map);
	init_carmen_map(&map_params, &local_mean_remission_map);
	init_carmen_map(&map_params, &local_sum_remission_map);
	init_carmen_map(&map_params, &local_sum_sqr_remission_map);
	init_carmen_map(&map_params, &local_count_remission_map);
	init_carmen_map(&map_params, &local_variance_remission_map);

	local_compacted_map.coord_x = NULL;
	local_compacted_map.coord_y = NULL;
	local_compacted_map.value = NULL;
	local_compacted_map.config.map_name = NULL;
	local_compacted_map.number_of_known_points_on_the_map = 0;

	local_compacted_mean_remission_map.coord_x = NULL;
	local_compacted_mean_remission_map.coord_y = NULL;
	local_compacted_mean_remission_map.value = NULL;
	local_compacted_mean_remission_map.config.map_name = NULL;
	local_compacted_mean_remission_map.number_of_known_points_on_the_map = 0;

	local_compacted_variance_remission_map.coord_x = NULL;
	local_compacted_variance_remission_map.coord_y = NULL;
	local_compacted_variance_remission_map.value = NULL;
	local_compacted_variance_remission_map.config.map_name = NULL;
	local_compacted_variance_remission_map.number_of_known_points_on_the_map = 0;

	memset(&binary_map, 0, sizeof(carmen_localize_ackerman_binary_map_t));
}


static void
init_localize_map()
{
	localize_map.carmen_map.complete_map = NULL;
	localize_map.complete_distance = NULL;
	localize_map.complete_gprob = NULL;
	localize_map.complete_prob = NULL;
	localize_map.complete_x_offset = NULL;
	localize_map.complete_y_offset = NULL;

	localize_map.carmen_map.map = NULL;
	localize_map.distance = NULL;
	localize_map.gprob = NULL;
	localize_map.prob = NULL;
	localize_map.x_offset = NULL;
	localize_map.y_offset = NULL;
}


void
nonblock(int state)
{
#define NB_ENABLE 		1
#define NB_DISABLE 		0

	struct termios ttystate;

	//get the terminal state
	tcgetattr(STDIN_FILENO, &ttystate);

	if (state == NB_ENABLE)
	{
		//turn off canonical mode
		ttystate.c_lflag &= ~ICANON;
		//minimum of number input read.
		ttystate.c_cc[VMIN] = 1;
	}
	else if (state == NB_DISABLE)
	{
		//turn on canonical mode
		ttystate.c_lflag |= ICANON;
	}
	//set the terminal attributes.
	tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

}


int
kbhit()
{
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
	return FD_ISSET(STDIN_FILENO, &fds);
}


void
timer_handler()
{
	if (kbhit() != 0)
	{
        char c = fgetc(stdin);

        switch (c)
        {
        case '1':
           	semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray = 0;
            break;

        case '2':
            // increase Velodyne single ray
        	semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray++;
            if (semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray > 31)
            	semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray = 0;
            if (semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray < 0)
            	semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray = 31;
            break;
        case '3':
            // decrease Velodyne single ray
        	semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray--;
            if (semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray > 31)
            	semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray = 0;
            if (semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray < 0)
            	semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray = 31;
            break;
        }
        printf("semi_trailer_config.beta_correct_velodyne_ray %d\n", semi_trailer_config.semi_trailers[0].beta_correct_velodyne_ray);
	}
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

	/* Initialize all the relevant parameters */
	carmen_localize_ackerman_read_parameters(argc, argv, &param, &map_params);
	initialize_tf_transforms();

#ifndef OLD_MOTION_MODEL
	param.motion_model = carmen_localize_ackerman_motion_initialize(argc, argv);
#endif

	/* Allocate memory for the particle filter */
	filter = carmen_localize_ackerman_particle_filter_initialize(&param);
//	beta_filter = carmen_localize_ackerman_particle_filter_initialize(&param);

	init_localize_map();
	init_local_maps(map_params);

	define_ipc_messages();
	subscribe_to_ipc_message();

	nonblock(NB_ENABLE);
//	double timer_period = 0.1;
//	carmen_ipc_addPeriodicTimer(timer_period, (TIMER_HANDLER_TYPE) timer_handler, NULL);
	carmen_ipc_dispatch();

	nonblock(NB_DISABLE);

	carmen_ipc_dispatch();

	return (0);
}
