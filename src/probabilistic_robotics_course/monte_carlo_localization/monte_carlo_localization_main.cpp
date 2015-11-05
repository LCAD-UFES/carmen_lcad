#include <math.h>
#include "monte_carlo_localization_main.h"
#include <tf.h>
#include "../../mapper/message_interpolation.cpp"

#define BUFFER_SIZE 20

particle *particle_set_g;
int num_of_particles_g = 0;
double distance_between_front_rear_wheels_axles_g = 0;
double timestamp_previous_g = 0.0, timestamp_current_g = 0.0, delta_t_g = 0.0;
MessageInterpolation<carmen_velodyne_partial_scan_message, carmen_base_ackerman_odometry_message> interpolator_velodyne_g(BUFFER_SIZE);
MessageInterpolation<carmen_map_server_cost_map, carmen_base_ackerman_odometry_message> interpolator_map_g(BUFFER_SIZE);

carmen_pose_3D_t car_pose_g, sensor_board_pose_g, velodyne_pose_g;

likelihood_field_range_finder_measurement_model_params laser_model_params;
tf::Transformer transformer_g(false);

void initialize_transforms()
{
	tf::Transform board_to_velodyne_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer_g.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_pose_g.position.x, sensor_board_pose_g.position.y, sensor_board_pose_g.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_pose_g.orientation.yaw, sensor_board_pose_g.orientation.pitch, sensor_board_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	transformer_g.setTransform(car_to_board_transform, "car_to_board_transform");

	// velodyne pose with respect to the board
	board_to_velodyne_pose.setOrigin(tf::Vector3(velodyne_pose_g.position.x, velodyne_pose_g.position.y, velodyne_pose_g.position.z));
	board_to_velodyne_pose.setRotation(tf::Quaternion(velodyne_pose_g.orientation.yaw, velodyne_pose_g.orientation.pitch, velodyne_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(board_to_velodyne_pose, tf::Time(0), "/board", "/velodyne");
	transformer_g.setTransform(board_to_camera_transform, "board_to_velodyne_transform");
}

double
random(double max)
{
  return max * (double)rand() / (double)RAND_MAX;
}

double
random_from_negative_value_to_positive_value(double value)
{
  return value * (2.0 * random(1.0) - 1);
}

double
sample_normal_distribution(double variance){

	double standard_deviation = sqrt(variance);
	double sample = 0;
	int i;
	for (i = 0; i < 12; i++)
		sample += standard_deviation * (2.0 * random(1.0) - 1);

	return sample / 2.0;
}

carmen_point_t
sample_motion_model_velocity(double v, double phi, carmen_point_t pose_t_1){

	carmen_point_t pose_t;
	double delta_t = delta_t_g;
	double alpha_1 = 0.3, alpha_2 = 0.3;
	//TODO:: Revisar
	v = v * sample_normal_distribution(alpha_1 * v * v);
	phi = phi * sample_normal_distribution(alpha_2 * phi * phi);

	pose_t.x = pose_t_1.x + delta_t * v * cos(pose_t_1.theta);
	pose_t.y = pose_t_1.y + delta_t * v * sin(pose_t_1.theta);
	pose_t.theta = pose_t_1.theta + delta_t * v * tan(phi) / distance_between_front_rear_wheels_axles_g;

	return pose_t;
}

carmen_point_t
sample_motion_model(double v, double phi, carmen_point_t pose_t_1){

	carmen_point_t pose_t;

	pose_t = sample_motion_model_velocity(v, phi, pose_t_1);

	return pose_t;
}

void
compute_true_range(carmen_point_t pose_t, int map){

}

double
p_hit(){

}

double
p_short(){

}

double
p_max(){

}

double
p_rand(){

}

double
distance_to_the_nearest_neighbor(double x_z_t, double y_z_t){

	double dist;

	return dist;
}

double
prob_normal_distribution(double a, double variance){

	return 1 / sqrt(2 * M_PI * variance) * exp(- (1 / 2) * (a * a) / (variance * variance));
}

void
arrange_velodyne_vertical_angles_to_true_position(carmen_velodyne_partial_scan_message *velodyne_message)
{
	const int column_correspondence[32] =
	{
		0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
		24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
	};

	int i, j;
	unsigned short original_distances[32];

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		memcpy(original_distances, velodyne_message->partial_scan[i].distance, 32 * sizeof(unsigned short));

		for (j = 0; j < 32; j++)
		{
			velodyne_message->partial_scan[i].distance[column_correspondence[j]] = original_distances[j];
		}
	}
}

double
likelihood_field_range_finder_model(carmen_velodyne_partial_scan_message *velodyne_message, carmen_point_t pose_t, carmen_map_server_cost_map *map_message){

	int q = 1;
	double x_z_t, y_z_t;
	double z_hit, z_max, z_rand;
	carmen_point_t pose_velodyne;
	double sigma_hit;
	double dist;

	//paramos aqui! 20/11/2014 12h
//	tf::StampedTransform world_to_velodyne;
//	tf_transformer.lookupTransform("/world", "/velodyne", tf::Time(0), world_to_velodyne);
//	Xt_l1.x = world_to_ultrasonic_sensor_l1.getOrigin().x();
//	Xt_l1.y = world_to_ultrasonic_sensor_l1.getOrigin().y();
//	tf::Matrix3x3(world_to_ultrasonic_sensor_l1.getRotation()).getEulerYPR(yaw, pitch, roll);
//	Xt_l1.theta = yaw;

	const static double sorted_vertical_angles[32] =
	{
			-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
			-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
			-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
			5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
	};

	arrange_velodyne_vertical_angles_to_true_position(velodyne_message);

	int i, j;

	// *********************
	// ** TODO: ler do ini
	// *********************
	double CAR_HEIGHT = 1.725;
	double MAX_RANGE = laser_model_params.max_range;
	double MIN_RANGE = 3.0;

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		double hor_angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle));

		// for (j = 0; j < (32 - 1); j++)
		for (j = 0; j < 23; j++) // the 22nd angle is the 0.0. The next angles are higher (trees, etc)
		{
			double range_0 = (((double) velodyne_message->partial_scan[i].distance[j]) / 500.0);
			double range_1 = (((double) velodyne_message->partial_scan[i].distance[j + 1]) / 500.0);

			// **********************************************************
			// TODO: checar se essa eh a melhor forma de tratar max_range
			// **********************************************************
			if ((range_0 >= MAX_RANGE) || (range_0 <= MIN_RANGE))
				continue;

			if ((range_1 >= MAX_RANGE) || (range_1 <= MIN_RANGE))
				continue;

			double angle_0 = carmen_degrees_to_radians(sorted_vertical_angles[j]);
			double angle_1 = carmen_degrees_to_radians(sorted_vertical_angles[j + 1]);

			double cos_vert_angle0 = cos(angle_0);
			double cos_vert_angle1 = cos(angle_1);

			double xy_distance0 = range_0 * cos_vert_angle0;
			double xy_distance1 = range_1 * cos_vert_angle1;

			double delta_ray = xy_distance1 - xy_distance0;
			double next_ray_angle = -carmen_normalize_theta(angle_1 - angle_0) + atan(CAR_HEIGHT / xy_distance0);
			double expected_delta_ray = (CAR_HEIGHT - xy_distance0 * tan(next_ray_angle)) / tan(next_ray_angle);

			if (delta_ray < expected_delta_ray)
			{
				if ((delta_ray / expected_delta_ray) < 0.5)
				{

					//TODO: REVISAR
					double theta = carmen_normalize_theta(pose_t.theta + hor_angle);
					x_z_t = delta_ray * cos(theta);
					y_z_t = delta_ray * sin(theta);

					dist = distance_to_the_nearest_neighbor(x_z_t, y_z_t);

					double log_q = log(q) + log(z_hit) + log(prob_normal_distribution(dist, sigma_hit));
					q = exp(log_q);
					return q;
				}
			}
		}

	}

//	if(z_t != z_max){
//		x_z_t = pose_t.x + pose_velodyne.x * cos(pose_t.theta) - pose_velodyne.y * sin(pose_t.theta) + z_t * cos(pose_t.theta + pose_velodyne.theta);
//		y_z_t = pose_t.y + pose_velodyne.y * cos(pose_t.theta) + pose_velodyne.x * sin(pose_t.theta) + z_t * sin(pose_t.theta + pose_velodyne.theta);
//		dist = distance_to_the_nearest_neighbor(x_z_t, y_z_t);
//		q = z_hit * prob_normal_distribution(dist, sigma_hit);
//	}
//	return q;
}

double
measurement_model(carmen_velodyne_partial_scan_message *velodyne_message, carmen_point_t pose_t, carmen_map_server_cost_map *map_message){

	double weight_t;

	weight_t = likelihood_field_range_finder_model(velodyne_message, pose_t, map_message);

	return weight_t;
}

particle
resampling(particle *temporary_particle_set_t){

	particle particle_t;

	return particle_t;
}

particle*
algorithm_MCL(particle *particle_set_t_1, double v, double phi, carmen_velodyne_partial_scan_message *velodyne_message, carmen_map_server_cost_map *map_message)
{
	particle *temporary_particle_set_t = (particle*)malloc(sizeof(particle) * num_of_particles_g);
	particle *particle_set_t = (particle*)malloc(sizeof(particle) * num_of_particles_g);

	for(int m = 0; m < num_of_particles_g; m++){

		temporary_particle_set_t[m].pose = sample_motion_model(v, phi, particle_set_t_1[m].pose);
		temporary_particle_set_t[m].weigth = measurement_model(velodyne_message, temporary_particle_set_t[m].pose, map_message);
	}
	for(int m = 0; m < num_of_particles_g; m++){

		particle_set_t[m] = resampling(temporary_particle_set_t);
	}

	return particle_set_t;
}

//---------------------------------------------handlers

void
map_handler (carmen_map_server_cost_map *msg)
{
	static int first_time = 1;

	if(first_time){
		carmen_point_t original_pose;
		original_pose.x = msg->config.x_origin;
		original_pose.y = msg->config.y_origin;

		for(int i = 0; i < num_of_particles_g; i++){
			particle_set_g[i].pose.x = original_pose.x + random_from_negative_value_to_positive_value(msg->config.x_size / 2.0);
			particle_set_g[i].pose.y = original_pose.y + random_from_negative_value_to_positive_value(msg->config.y_size / 2.0);
			particle_set_g[i].pose.theta = random( 2 * M_PI);
		}

		first_time = 0;
	}

	interpolator_map_g.AddMessageToInterpolationList(msg);
}

void
velodyne_handler (carmen_velodyne_partial_scan_message *msg)
{
	static int i = 0;

	printf("n: %d \n", msg->number_of_32_laser_shots);
	interpolator_velodyne_g.AddMessageToInterpolationList(msg);
}

void
odometry_handler (carmen_base_ackerman_odometry_message *msg)
{

	printf("v: %lf phi: %lf\n", msg->v, msg->phi);

	//synchronize velodyne's message and map's message with odometry message
	carmen_map_server_cost_map map_message;
	carmen_velodyne_partial_scan_message velodyne_message;
	map_message = interpolator_map_g.InterpolateMessages(msg);
	velodyne_message = interpolator_velodyne_g.InterpolateMessages(msg);

	timestamp_current_g = msg->timestamp;
	delta_t_g = timestamp_current_g - timestamp_previous_g;
	timestamp_previous_g = timestamp_current_g;

	tf::Transform world_to_car_pose;

	// update car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
	world_to_car_pose.setRotation(tf::Quaternion(msg->theta, 0.0, 0.0));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer_g.setTransform(world_to_car_transform, "world_to_car_transform");

	particle_set_g = algorithm_MCL(particle_set_g, msg->v, msg->phi, &velodyne_message, &map_message);
}

//---------------------------------------------

void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("monte_carlo_localization_module: disconnected.\n");

    free(particle_set_g);

    exit(0);
  }
}

static void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
			{(char *)"robot",  (char*)"distance_between_front_rear_wheels_axles",		CARMEN_PARAM_DOUBLE, &distance_between_front_rear_wheels_axles_g, 1, NULL},

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

			{(char *) "velodyne",    (char *) "x", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.x), 0, NULL},
			{(char *) "velodyne",    (char *) "y", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.y), 0, NULL},
			{(char *) "velodyne",    (char *) "z", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.z), 0, NULL},
			{(char *) "velodyne",    (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.yaw), 0, NULL},
			{(char *) "velodyne",    (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.pitch), 0, NULL},
			{(char *) "velodyne",    (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.roll), 0, NULL},

			{(char *) "polar_slam", (char *) "laser_sampling_step", CARMEN_PARAM_INT, 	&(laser_model_params.sampling_step), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_num_beams", CARMEN_PARAM_INT, 		&(laser_model_params.laser_beams), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_fov_range", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.fov_range), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_max_range", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.max_range), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_lambda_short", CARMEN_PARAM_DOUBLE, &(laser_model_params.lambda_short), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_sigma_zhit", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.sigma_zhit), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_zhit", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zhit), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_zmax", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zmax), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_zrand", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zrand), 0, NULL},
			{(char *) "polar_slam", (char *) "laser_zshort", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zshort), 0, NULL},
			{(char *) "robot", (char *) "frontlaser_offset", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.front_offset), 0, NULL},
			{(char *) "robot", (char *) "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.side_offset), 0, NULL},
			{(char *) "robot", (char *) "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(laser_model_params.angular_offset), 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

int
main(int argc, char **argv)
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	if(argc < 1)
		exit(0);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Define messages that your module publishes */
	carmen_monte_carlo_localization_define_messages();

	num_of_particles_g = atoi(argv[1]);

	particle_set_g = (particle*)malloc(sizeof(particle) * num_of_particles_g);

	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_cost_map(NULL, (carmen_handler_t) map_handler,CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) odometry_handler, CARMEN_SUBSCRIBE_LATEST);

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return 0;
}
