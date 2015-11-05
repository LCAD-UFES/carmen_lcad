#include <string.h>
#include <carmen/carmen.h>
#include <carmen/visual_odometry_interface.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_transforms.h>
#include <tf.h>

#include "grid_mapping_interface.h"
#include "grid_mapping.h"
#include <carmen/global_graphics.h>

#include "message_interpolation.cpp"


/**
 * The map
 */
static carmen_map_t carmen_map;
static double x_origin, y_origin; // map origin in meters

/**
 * The robot pose
 */
static carmen_point_t Xt;

/**
 * The ultrasonic sensors poses
 */
static carmen_point_t Xt_r1;
static carmen_point_t Xt_r2;
static carmen_point_t Xt_l1;
static carmen_point_t Xt_l2;

/**
 * Model params
 */
static BeanRangeFinderMeasurementModelParams laser_params;
static ProbabilisticMapParams map_params;

/**
 * Parameters
 */
static double robot_length;
static double robot_width;
static double robot_vertical_displacement_from_center;

/**
 * When TRUE indicate that initial pose was defined.
 */
static int grid_mapping_initialized = 0;

static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t sensor_board_pose_g;
static carmen_pose_3D_t ultrasonic_sensor_r1_g;
static carmen_pose_3D_t ultrasonic_sensor_r2_g;
static carmen_pose_3D_t ultrasonic_sensor_l1_g;
static carmen_pose_3D_t ultrasonic_sensor_l2_g;

tf::Transformer tf_transformer(false);
MessageInterpolation<carmen_visual_odometry_pose6d_message, carmen_ultrasonic_sonar_sensor_message> interpolator(10);


void
grid_mapping_change_map_origin_to_another_map_block(carmen_position_t *map_origin)
{
	static int first_time = 1;
	carmen_map_t new_carmen_map;

	new_carmen_map.complete_map = NULL;

	if (first_time)
	{
		x_origin = map_origin->x;
		y_origin = map_origin->y;

		first_time = 0;
	}

	//verify if its necessery to change the map
	if (carmen_grid_mapping_is_map_changed(map_origin, x_origin, y_origin))
	{
		x_origin = map_origin->x;
		y_origin = map_origin->y;

		carmen_grid_mapping_update_map_buffer(&carmen_map);
		carmen_grid_mapping_get_buffered_map(x_origin, y_origin, &new_carmen_map);

		//destroy current map and assign new map to current map
		carmen_grid_mapping_switch_maps(&carmen_map, &new_carmen_map);
	}

	carmen_map.config.x_origin = x_origin;
	carmen_map.config.y_origin = y_origin;
}


static void
carmen_grid_mapping_update_grid_map(carmen_map_t *map, carmen_point_t xt, double *zt, BeanRangeFinderMeasurementModelParams *laser_params)
{
	carmen_point_t zt_pose;

	transform_robot_pose_to_laser_pose(&zt_pose, &xt);
	carmen_update_cells_in_the_laser_perceptual_field(map, zt_pose, zt, laser_params);
}



/*********************************************************
		   --- Handlers ---
 **********************************************************/

static void
offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	carmen_position_t map_origin;

	map_origin.x = msg->config.x_origin;
	map_origin.y = msg->config.y_origin;

	grid_mapping_change_map_origin_to_another_map_block(&map_origin);
}


static void
sensor_ackerman_message_handler(carmen_localize_ackerman_sensor_message *flaser)
{
	if (!grid_mapping_initialized)
		return;

	Xt = flaser->pose;

	carmen_grid_mapping_update_grid_map(&carmen_map, Xt, flaser->range, &laser_params);
	carmen_grid_mapping_publish_message(&carmen_map, flaser->timestamp);
}


static void
pose_initialize_ackerman_message_handler(carmen_localize_ackerman_initialize_message *localize_ackerman_initialize_message)
{
	Xt = *(localize_ackerman_initialize_message->mean);
	grid_mapping_initialized = TRUE;
}


static void
ultrasonic_sensor_message_handler(carmen_ultrasonic_sonar_sensor_message *message)
{

	tf::Transform world_to_car_pose;
	double yaw, pitch, roll;

	if (!grid_mapping_initialized)
		return;
	
	int i;

	carmen_visual_odometry_pose6d_message odometry_message;
	odometry_message = interpolator.InterpolateMessages(message);

	Xt.x = odometry_message.pose_6d.x;
	Xt.y = odometry_message.pose_6d.y;
	Xt.theta = odometry_message.pose_6d.yaw;

	carmen_update_cells_below_robot(&carmen_map, Xt);

	world_to_car_pose.setOrigin(tf::Vector3(odometry_message.pose_6d.x, odometry_message.pose_6d.y, odometry_message.pose_6d.z));
	world_to_car_pose.setRotation(tf::Quaternion(odometry_message.pose_6d.yaw, odometry_message.pose_6d.pitch, odometry_message.pose_6d.roll));
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

	carmen_grid_mapping_update_grid_map(&carmen_map, Xt_r1, range, &laser_params);
	carmen_grid_mapping_publish_message(&carmen_map, message->timestamp);

	//SENSOR R2 - LATERAL FRONTAL
	tf::StampedTransform world_to_ultrasonic_sensor_r2;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_r2", tf::Time(0), world_to_ultrasonic_sensor_r2);
	Xt_r2.x = world_to_ultrasonic_sensor_r2.getOrigin().x();
	Xt_r2.y = world_to_ultrasonic_sensor_r2.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_r2.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_r2.theta = yaw;

	for (i=0 ; i<180 ; i++)
			range[i] = (double) message->sensor[2];

	carmen_grid_mapping_update_grid_map(&carmen_map, Xt_r2, range, &laser_params);
	carmen_grid_mapping_publish_message(&carmen_map, message->timestamp);

	//SENSOR L2 - LATERAL TRASEIRO
	tf::StampedTransform world_to_ultrasonic_sensor_l2;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_l2", tf::Time(0), world_to_ultrasonic_sensor_l2);
	Xt_l2.x = world_to_ultrasonic_sensor_l2.getOrigin().x();
	Xt_l2.y = world_to_ultrasonic_sensor_l2.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_l2.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_l2.theta = yaw;

	for (i=0 ; i<180 ; i++)
			range[i] = (double) message->sensor[1];

	carmen_grid_mapping_update_grid_map(&carmen_map, Xt_l2, range, &laser_params);
	carmen_grid_mapping_publish_message(&carmen_map, message->timestamp);

	//SENSOR L1 - TRASEIRO
	tf::StampedTransform world_to_ultrasonic_sensor_l1;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_l1", tf::Time(0), world_to_ultrasonic_sensor_l1);
	Xt_l1.x = world_to_ultrasonic_sensor_l1.getOrigin().x();
	Xt_l1.y = world_to_ultrasonic_sensor_l1.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_l1.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_l1.theta = yaw;

	for (i=0 ; i<180 ; i++)
			range[i] = (double) message->sensor[0];

	carmen_grid_mapping_update_grid_map(&carmen_map, Xt_l1, range, &laser_params);
	carmen_grid_mapping_publish_message(&carmen_map, message->timestamp);


	/*static int count = 1;

	char mapname[256];
	sprintf(mapname, "map%d.jpg", count);

	printf("saving map...\n");
	carmen_graphics_write_map_as_jpeg(mapname, &carmen_map, 0);
	printf("saved map...\n");

	count++;*/

}


static void
visual_odometry_message_handler(carmen_visual_odometry_pose6d_message *message)
{
	if (!grid_mapping_initialized)
	{
		grid_mapping_initialized = TRUE;
	}

	message->pose_6d.x += 6.5;
	message->pose_6d.y += 6.5;
	interpolator.AddMessageToInterpolationList(message);
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		fprintf(stderr, "Shutdown grid_mapping\n");

		exit(0);
	}
}


/*********************************************************
	   --- Initialization functions ---
 **********************************************************/

static void
carmen_subscribe_to_relevant_messages()
{
	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t)pose_initialize_ackerman_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_sensor_message(NULL, (carmen_handler_t)sensor_ackerman_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ultrasonic_sonar_sensor_subscribe(NULL, (carmen_handler_t) ultrasonic_sensor_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_visual_odometry_subscribe_pose6d_message(NULL, (carmen_handler_t) visual_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void 
initialize_transforms()
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


/* read all parameters from .ini file and command line. */
static void
read_laser_parameters(int argc, char **argv,
	BeanRangeFinderMeasurementModelParams *p_laser_params,
	ProbabilisticMapParams *p_map_params)
{
	char camera_string[256];

	sprintf(camera_string, "%s%d", "camera", atoi(argv[1]));

	carmen_param_t param_list[] =
	{
			{(char *)"grid_mapping", (char *)"laser_sampling_step", CARMEN_PARAM_INT, &p_laser_params->sampling_step, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_num_beams", CARMEN_PARAM_INT, &p_laser_params->laser_beams, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_fov_range", CARMEN_PARAM_DOUBLE, &p_laser_params->fov_range, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_max_range", CARMEN_PARAM_DOUBLE, &p_laser_params->max_range, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_lambda_short", CARMEN_PARAM_DOUBLE, &p_laser_params->lambda_short, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_sigma_zhit", CARMEN_PARAM_DOUBLE, &p_laser_params->sigma_zhit, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_zhit", CARMEN_PARAM_DOUBLE, &p_laser_params->zhit, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_zmax", CARMEN_PARAM_DOUBLE, &p_laser_params->zmax, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_zrand", CARMEN_PARAM_DOUBLE, &p_laser_params->zrand, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_zshort", CARMEN_PARAM_DOUBLE, &p_laser_params->zshort, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_locc", CARMEN_PARAM_DOUBLE, &p_laser_params->locc, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_lfree", CARMEN_PARAM_DOUBLE, &p_laser_params->lfree, 0, NULL},
			{(char *)"grid_mapping", (char *)"laser_l0", CARMEN_PARAM_DOUBLE, &p_laser_params->l0, 0, NULL},

			{(char *)"robot", (char *)"frontlaser_offset", CARMEN_PARAM_DOUBLE, &p_laser_params->front_offset, 0, NULL},
			{(char *)"robot", (char *)"length", CARMEN_PARAM_DOUBLE, 				&robot_length, 0, NULL},
			{(char *)"robot", (char *)"width", CARMEN_PARAM_DOUBLE, 				&robot_width, 0, NULL},
			{(char *)"robot", (char *)"vertical_displacement_from_center", CARMEN_PARAM_DOUBLE, 	&robot_vertical_displacement_from_center, 0, NULL},

			{(char *)"grid_mapping", (char *)"map_grid_res", CARMEN_PARAM_DOUBLE, &p_map_params->grid_res, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_range_factor", CARMEN_PARAM_DOUBLE, &p_map_params->range_factor, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_width", CARMEN_PARAM_DOUBLE, &p_map_params->width, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_height", CARMEN_PARAM_DOUBLE, &p_map_params->height, 0, NULL},

			{(char *) "car", 			  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.x), 0, NULL},
			{(char *) "car", 			  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.y), 0, NULL},
			{(char *) "car", 			  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.z), 0, NULL},
			{(char *) "car", 			  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.yaw), 0, NULL},
			{(char *) "car", 			  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.pitch), 0, NULL},
			{(char *) "car", 			  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.roll), 0, NULL},

			{(char *) "sensor_board_1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.x), 0, NULL},
			{(char *) "sensor_board_1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.y), 0, NULL},
			{(char *) "sensor_board_1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.z), 0, NULL},
			{(char *) "sensor_board_1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.yaw), 0, NULL},
			{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.pitch), 0, NULL},
			{(char *) "sensor_board_1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.roll), 0, NULL},

			{(char *) camera_string, 	  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.x), 0, NULL},
			{(char *) camera_string,    (char *) "y", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.y), 0, NULL},
			{(char *) camera_string,    (char *) "z", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.z), 0, NULL},
			{(char *) camera_string,    (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.yaw), 0, NULL},
			{(char *) camera_string,    (char *) "pitch", CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.pitch), 0, NULL},
			{(char *) camera_string,    (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.roll), 0, NULL},

	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	p_laser_params->start_angle = -0.5 * p_laser_params->fov_range;

	p_map_params->range_max = p_laser_params->max_range;
	p_map_params->range_step = p_laser_params->fov_range / (double)(p_laser_params->laser_beams-1);
	p_map_params->range_start = p_laser_params->start_angle;
	p_map_params->num_ranges = p_laser_params->laser_beams;

	p_map_params->grid_sx = round(p_map_params->width / p_map_params->grid_res);
	p_map_params->grid_sy = round(p_map_params->height / p_map_params->grid_res);
	p_map_params->grid_size = p_map_params->grid_sx * p_map_params->grid_sy;

	p_map_params->robot_width = robot_width;
	p_map_params->robot_length = robot_length;
	p_map_params->robot_vertical_displacement_from_center = robot_vertical_displacement_from_center;

	carmen_grid_mapping_init_parameters(p_map_params->grid_res, p_map_params->width);
}


/* read all parameters from .ini file and command line. */
static void
read_ultrasonic_sensor_parameters(int argc, char **argv,
		BeanRangeFinderMeasurementModelParams *p_laser_params,
		ProbabilisticMapParams *p_map_params)
{
	char camera_string[256];

	sprintf(camera_string, "%s%d", "camera", atoi(argv[1]));

	carmen_param_t param_list[] = 
	{
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_sampling_step", CARMEN_PARAM_INT, &p_laser_params->sampling_step, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_num_beams", CARMEN_PARAM_INT, &p_laser_params->laser_beams, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_fov_range", CARMEN_PARAM_DOUBLE, &p_laser_params->fov_range, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_max_range", CARMEN_PARAM_DOUBLE, &p_laser_params->max_range, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_lambda_short", CARMEN_PARAM_DOUBLE, &p_laser_params->lambda_short, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_sigma_zhit", CARMEN_PARAM_DOUBLE, &p_laser_params->sigma_zhit, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_zhit", CARMEN_PARAM_DOUBLE, &p_laser_params->zhit, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_zmax", CARMEN_PARAM_DOUBLE, &p_laser_params->zmax, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_zrand", CARMEN_PARAM_DOUBLE, &p_laser_params->zrand, 0, NULL},
			{(char *)"grid_mapping", (char *)"ultrasonic_sensor_zshort", CARMEN_PARAM_DOUBLE, &p_laser_params->zshort, 0, NULL},

			{(char *)"robot", (char *)"frontlaser_offset", CARMEN_PARAM_DOUBLE, &p_laser_params->front_offset, 0, NULL},
			{(char *)"robot", (char *)"length", CARMEN_PARAM_DOUBLE, 				&robot_length, 0, NULL},
			{(char *)"robot", (char *)"width", CARMEN_PARAM_DOUBLE, 				&robot_width, 0, NULL},
			{(char *)"robot", (char *)"vertical_displacement_from_center", CARMEN_PARAM_DOUBLE, 	&robot_vertical_displacement_from_center, 0, NULL},


			{(char *)"grid_mapping", (char *)"map_locc", CARMEN_PARAM_INT, &p_map_params->locc, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_lfree", CARMEN_PARAM_INT, &p_map_params->lfree, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_l0", CARMEN_PARAM_INT, &p_map_params->l0, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_log_odds_max", CARMEN_PARAM_INT, &p_map_params->log_odds_max, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_log_odds_min", CARMEN_PARAM_INT, &p_map_params->log_odds_min, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_log_odds_bias", CARMEN_PARAM_INT, &p_map_params->log_odds_bias, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_grid_res", CARMEN_PARAM_DOUBLE, &p_map_params->grid_res, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_range_factor", CARMEN_PARAM_DOUBLE, &p_map_params->range_factor, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_width", CARMEN_PARAM_DOUBLE, &p_map_params->width, 0, NULL},
			{(char *)"grid_mapping", (char *)"map_height", CARMEN_PARAM_DOUBLE, &p_map_params->height, 0, NULL},

			{(char *) "car", 			  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.x), 0, NULL},
			{(char *) "car", 			  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.y), 0, NULL},
			{(char *) "car", 			  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.z), 0, NULL},
			{(char *) "car", 			  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.yaw), 0, NULL},
			{(char *) "car", 			  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.pitch), 0, NULL},
			{(char *) "car", 			  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.roll), 0, NULL},

			{(char *) "sensor_board_1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.x), 0, NULL},
			{(char *) "sensor_board_1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.y), 0, NULL},
			{(char *) "sensor_board_1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.z), 0, NULL},
			{(char *) "sensor_board_1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.yaw), 0, NULL},
			{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.pitch), 0, NULL},
			{(char *) "sensor_board_1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.roll), 0, NULL},

			{(char *) camera_string, 	  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.x), 0, NULL},
			{(char *) camera_string,    (char *) "y", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.y), 0, NULL},
			{(char *) camera_string,    (char *) "z", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.z), 0, NULL},
			{(char *) camera_string,    (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.yaw), 0, NULL},
			{(char *) camera_string,    (char *) "pitch", CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.pitch), 0, NULL},
			{(char *) camera_string,    (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.roll), 0, NULL},

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
			{(char *) "ultrasonic_sensor_l1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.roll), 0, NULL}


	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	p_laser_params->start_angle = -0.5 * p_laser_params->fov_range;

	p_map_params->range_max = p_laser_params->max_range;
	p_map_params->range_step = p_laser_params->fov_range / (double)(p_laser_params->laser_beams-1);
	p_map_params->range_start = p_laser_params->start_angle;
	p_map_params->num_ranges = p_laser_params->laser_beams;

	p_map_params->grid_sx = round(p_map_params->width / p_map_params->grid_res);
	p_map_params->grid_sy = round(p_map_params->height / p_map_params->grid_res);
	p_map_params->grid_size = p_map_params->grid_sx * p_map_params->grid_sy;

	p_map_params->robot_width = robot_width;
	p_map_params->robot_length = robot_length;
	p_map_params->robot_vertical_displacement_from_center = robot_vertical_displacement_from_center;

	carmen_grid_mapping_init_parameters(p_map_params->grid_res, p_map_params->width);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	
	if (argc == 3 && !strcmp("ultrasonic",argv[2]))
		read_ultrasonic_sensor_parameters(argc, argv, &laser_params, &map_params);
	else
		read_laser_parameters(argc, argv, &laser_params, &map_params);

	initialize_transforms();

	/* Create and initialize a probabilistic map */
	init_carmen_map(&map_params, &carmen_map);

	init_bean_range_finder_measurement_model(laser_params);

	/* Register my own messages */
	carmen_grid_mapping_define_messages();

	carmen_subscribe_to_relevant_messages();

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return (0);
}
