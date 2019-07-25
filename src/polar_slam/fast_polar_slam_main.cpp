#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <carmen/carmen.h>
#include <carmen/laser_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <prob_motion_model.h>
#include <prob_measurement_model.h>

#include "fast_polar_particle_filter.h"
#include "fast_polar_slam_interface.h"

using namespace std;

typedef struct
{
	int num_spheres;
	int num_particles;
	int num_points_to_store;
	int num_angular_sections;

}PolarSlamParams;

carmen_polar_particle_filter *fast_particle_filter;

// parameters of the module
PolarSlamParams polar_slam_params;

// slam
int odometry_is_initialized = 0;
OdometryMotionCommand *ut = NULL;
OdometryMotionModelParams odometry_model_params;
BeanRangeFinderMeasurementModelParams laser_model_params;

// debug
struct timeval start, end_bla;

carmen_fast_polar_slam_best_particle_message best_particle_message;
carmen_fast_polar_slam_particles_message particles_message;

double velocity_from_fused_odometry = 0.0;
double phi_from_fused_odometry = 0.0;


void
start_count_time()
{
	gettimeofday(&start, NULL);
}


void
show_time_ellapsed()
{
	gettimeofday(&end_bla, NULL);
	double ellapsed = (end_bla.tv_sec - start.tv_sec) * 1000000 + (end_bla.tv_usec - start.tv_usec);
	printf("\ttime ellapsed: %.6lf s\n", ellapsed / 1000000.0);
}


void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		cvDestroyAllWindows();

		printf("polar slam module: disconnected.\n");
		exit(0);
	}
}


void
publish_particles_and_best_particle_message(double timestamp)
{
	// best particle message
	if (fast_particle_filter->best_particle != NULL) // at least one correction has been made
	{
		best_particle_message.map_size = fast_particle_filter->config.map_size;

		best_particle_message.particle_map = fast_particle_filter->best_particle->particle_map;
		best_particle_message.particle_map_occupation = fast_particle_filter->best_particle->particle_map_occupation;
		best_particle_message.particle_pose = fast_particle_filter->best_particle->particle_pose;
		best_particle_message.pose_before_movement = fast_particle_filter->best_particle->pose_before_movement;
		best_particle_message.weight = fast_particle_filter->best_particle->weight;

		best_particle_message.host = carmen_get_host();
		best_particle_message.timestamp = timestamp;

		carmen_fast_polar_slam_publish_best_particle_message(&best_particle_message);
	}

	// particles message
	particles_message.num_particles = fast_particle_filter->config.num_particles;

	for (int i = 0; i < fast_particle_filter->config.num_particles; i++)
	{
		particles_message.particle_pose[i] = fast_particle_filter->particles[i].particle_pose;
		particles_message.weight[i] = fast_particle_filter->particles[i].weight;
		particles_message.host = carmen_get_host();
		particles_message.timestamp = timestamp;
	}

	carmen_fast_polar_slam_publish_particles_message(&particles_message);
}


void
publish_global_pos(double v, double phi, double timestamp)
{
//	int i;

//	double mean_particle_x = 0.0;
//	double mean_particle_y = 0.0;
//	double mean_particle_theta = 0.0;
//	for(i = 0; i < fast_particle_filter->config.num_particles; i++)
//	{
//		mean_particle_x += fast_particle_filter->particles[i].particle_pose.position.x;
//		mean_particle_y += fast_particle_filter->particles[i].particle_pose.position.y;
//		mean_particle_theta += fast_particle_filter->particles[i].particle_pose.orientation.yaw;
//	}
//
//	mean_particle_x /= fast_particle_filter->config.num_particles;
//	mean_particle_y /= fast_particle_filter->config.num_particles;
//	mean_particle_theta /= fast_particle_filter->config.num_particles;
//	mean_particle_theta = carmen_normalize_theta(mean_particle_theta);

	if (fast_particle_filter->best_particle == NULL)
		return;

	// get the pose of the heavier particle
	double mean_particle_x = fast_particle_filter->best_particle->particle_pose.position.x;
	double mean_particle_y = fast_particle_filter->best_particle->particle_pose.position.y;
	double mean_particle_theta = fast_particle_filter->best_particle->particle_pose.orientation.yaw;

	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_globalpos_message globalpos;

	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();
	globalpos.globalpos.x = mean_particle_x;
	globalpos.globalpos.y = mean_particle_y;
	globalpos.globalpos.theta = mean_particle_theta;
	globalpos.globalpos_std.x = 0.0;
	globalpos.globalpos_std.y = 0.0;
	globalpos.globalpos_std.theta = 0.0;
	globalpos.odometrypos.x = 0.0;
	globalpos.odometrypos.y = 0.0;
	globalpos.odometrypos.theta = 0.0;
	globalpos.globalpos_xy_cov = 0.0;
	globalpos.v = v;
	globalpos.phi = phi;
	globalpos.converged = 1;

	globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = globalpos.pose.position.z = 0.0;
	globalpos.velocity.x = globalpos.velocity.y = globalpos.velocity.z = 0.0;

	globalpos.pose.orientation.yaw = globalpos.globalpos.theta;
	globalpos.pose.position.x = globalpos.globalpos.x;
	globalpos.pose.position.y = globalpos.globalpos.y;
	globalpos.pose.position.z = 0;
	globalpos.velocity.x = v;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


void
carmen_polar_slam_initialize_particle_filter(carmen_fused_odometry_message *fused_odometry_message)
{
	/**
	 * Initialize the odometry
	 */
	ut->initial.x = ut->final.x = fused_odometry_message->pose.position.x;
	ut->initial.y = ut->final.y = fused_odometry_message->pose.position.y;
	ut->initial.theta = ut->final.theta = fused_odometry_message->pose.orientation.yaw;

	/**
	 * Create the first particles
	 */
	carmen_pose_3D_t starting_pose;
	memset(&starting_pose, 0, sizeof(starting_pose));

	// TODO: a fused odometry possui todos os dados, preenche-los aqui!
	starting_pose.position.x = fused_odometry_message->pose.position.x;
	starting_pose.position.y = fused_odometry_message->pose.position.y;
	starting_pose.orientation.yaw = fused_odometry_message->pose.orientation.yaw;

	fast_particle_filter = carmen_polar_slam_create_particle_filter(
			&starting_pose,
			polar_slam_params.num_particles,
			polar_slam_params.num_spheres,
			polar_slam_params.num_angular_sections,
			polar_slam_params.num_points_to_store,
			laser_model_params.max_range
	);
}


void
carmen_polar_slam_initialize_motion_model()
{
	//
	// inicializa pose corrente e a anterior vindas da odometria com zero
	//
	ut = (OdometryMotionCommand *) calloc (1, sizeof(OdometryMotionCommand));
	init_odometry_motion_model(odometry_model_params);
}


void
carmen_polar_slam_initialize_beam_range_finder_model()
{
	// TODO: as seguintes entradas estao sendo inicializadas com zero: angle_step, starting_angle, front_offset,
	// side_offset e angular_offset. Checar se isso nao eh causa de bug
	init_bean_range_finder_measurement_model(laser_model_params);
}


void
carmen_polar_slam_initialize_global_data()
{
	carmen_polar_slam_initialize_motion_model();
	carmen_polar_slam_initialize_beam_range_finder_model();
}


void
predict_pose(carmen_fused_odometry_message *odometry)
{
	carmen_polar_slam_predict_pose_using_motion_model(fast_particle_filter, ut, odometry);
}


void
correct_pose(double *range, double *angles, int num_rays)
{
	carmen_polar_slam_correct_pose_using_beam_range_finder_model(fast_particle_filter, range, angles, num_rays);
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

// ********************************************************************************
// TODO: refactor: separar a funcao de projecao no chao e de publicacao da projecao
// ********************************************************************************
void
generate_2D_map_from_velodyne_pointcloud(carmen_velodyne_partial_scan_message *velodyne_message, double *range, double *angles)
{
	const static double sorted_vertical_angles[32] =
	{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
	};

	arrange_velodyne_vertical_angles_to_true_position(velodyne_message);

	int i, j;
	double min_ground_range;

	// *********************
	// ** TODO: ler do ini
	// *********************
	double CAR_HEIGHT = 1.725;
	double MAX_RANGE = laser_model_params.max_range;
	double MIN_RANGE = 3.0;

	carmen_fast_polar_slam_velodyne_projected_on_ground_message message;
	message.angles = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));
	message.ranges = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));
	message.intensity = (double *) malloc (velodyne_message->number_of_32_laser_shots * sizeof(double));

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		double hor_angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle));
		min_ground_range = MAX_RANGE;

		message.angles[i] = hor_angle;
		message.ranges[i] = MAX_RANGE;
		message.intensity[i] = 0.0;

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
					message.intensity[i] = 1.0;

					if ((xy_distance1 > MIN_RANGE) && (xy_distance1 < MAX_RANGE) && (xy_distance1 < min_ground_range))
						min_ground_range = xy_distance1;
				}
			}
		}

		message.ranges[i] = min_ground_range;
		range[i] = min_ground_range;
		angles[i] = hor_angle;
	}

	message.num_rays = velodyne_message->number_of_32_laser_shots;
	message.host = carmen_get_host();
	message.timestamp = velodyne_message->timestamp;

	carmen_fast_polar_slam_publish_velodyne_projected_message(&message);

	free(message.angles);
	free(message.ranges);
	free(message.intensity);
}


void
carmen_velodyne_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	if (!odometry_is_initialized)
		return;

	int num_rays = velodyne_message->number_of_32_laser_shots;

	double *angles = (double*) calloc (sizeof(double), num_rays);
	double *range = (double*) calloc (sizeof(double), num_rays);

	static int count = 0;

	generate_2D_map_from_velodyne_pointcloud(velodyne_message, range, angles);
//	printf("CORRECT POSE: ");
//	start_count_time();
	correct_pose(range, angles, num_rays);
//	show_time_ellapsed();
//	printf("ADD OBSTACLES: ");
//	start_count_time();
	carmen_polar_slam_add_obstacles_to_map(fast_particle_filter, range, angles, num_rays);
	publish_particles_and_best_particle_message(velodyne_message->timestamp);
	publish_global_pos(velocity_from_fused_odometry, phi_from_fused_odometry, velodyne_message->timestamp);

	free(angles);
	free(range);

//	show_time_ellapsed();
}


//void
//carmen_fused_odometry_handler(carmen_fused_odometry_message *fused_odometry_message)
//{
//	if (!odometry_is_initialized)
//	{
//		carmen_polar_slam_initialize_particle_filter(fused_odometry_message);
//
//		particles_message.particle_pose = (carmen_pose_3D_t *) calloc (fast_particle_filter->config.num_particles, sizeof(carmen_pose_3D_t));
//		particles_message.weight = (double *) calloc (fast_particle_filter->config.num_particles, sizeof(double));
//
//		odometry_is_initialized = 1;
//	}
//
//	predict_pose(fused_odometry_message);
//
//	if (fast_particle_filter->config.first_map_has_been_received)
//		carmen_polar_slam_move_particles_to_new_position(fast_particle_filter);
//
//	// TODO: colocar isso la na main pra ele publicar sempre o localizer. Pensar como pegar as informacoes de v e phi la.
//	publish_global_pos(fused_odometry_message->velocity.x, fused_odometry_message->phi, fused_odometry_message->timestamp);
//}


carmen_fused_odometry_message *fused_odometry_message = NULL;

void
carmen_base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *base_ackerman_odometry_message)
{
	if (fused_odometry_message == NULL)
		fused_odometry_message = (carmen_fused_odometry_message *) calloc (1, sizeof(carmen_fused_odometry_message));

	fused_odometry_message->pose.position.x = base_ackerman_odometry_message->x;
	fused_odometry_message->pose.position.y = base_ackerman_odometry_message->y;
	fused_odometry_message->pose.orientation.yaw = base_ackerman_odometry_message->theta;
	fused_odometry_message->velocity.x = base_ackerman_odometry_message->v;
	fused_odometry_message->phi = base_ackerman_odometry_message->phi;
	fused_odometry_message->timestamp = base_ackerman_odometry_message->timestamp;

	if (!odometry_is_initialized)
	{
		carmen_polar_slam_initialize_particle_filter(fused_odometry_message);

		particles_message.particle_pose = (carmen_pose_3D_t *) calloc (fast_particle_filter->config.num_particles, sizeof(carmen_pose_3D_t));
		particles_message.weight = (double *) calloc (fast_particle_filter->config.num_particles, sizeof(double));

		odometry_is_initialized = 1;
	}

	predict_pose(fused_odometry_message);

//	printf("BASE ACKERMAN MOVE: ");
//	start_count_time();

	if (fast_particle_filter->config.first_map_has_been_received)
		carmen_polar_slam_move_particles_to_new_position(fast_particle_filter);

	velocity_from_fused_odometry = fused_odometry_message->velocity.x;
	phi_from_fused_odometry = fused_odometry_message->phi;

//	show_time_ellapsed();
}


void
carmen_polar_slam_subscribe_messages()
{

//	carmen_fused_odometry_subscribe_fused_odometry_message(NULL,
//			(carmen_handler_t) carmen_fused_odometry_handler,
//			CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(NULL,
				(carmen_handler_t) carmen_base_ackerman_odometry_handler,
				CARMEN_SUBSCRIBE_LATEST);

	carmen_velodyne_subscribe_partial_scan_message(NULL,
			(carmen_handler_t) carmen_velodyne_handler,
			CARMEN_SUBSCRIBE_LATEST);

}


void
carmen_polar_slam_read_parameters(int argc, char *argv[])
{
	carmen_param_t basic_param_list[] =
	{
			{(char *) "polar_slam", (char *) "odom_a1", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha1), 1, NULL},
			{(char *) "polar_slam", (char *) "odom_a2", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha2), 1, NULL},
			{(char *) "polar_slam", (char *) "odom_a3", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha3), 1, NULL},
			{(char *) "polar_slam", (char *) "odom_a4", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha4), 1, NULL},
	};

	carmen_param_t laser_param_list[] =
	{
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
			{(char *) "robot", (char *) "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(laser_model_params.angular_offset), 0, NULL},
	};

	carmen_param_t polar_slam_param_list[] =
	{
			{(char *) "polar_slam", (char *) "num_spheres", 		CARMEN_PARAM_INT, &(polar_slam_params.num_spheres), 0, NULL},
			{(char *) "polar_slam", (char *) "num_particles", 			CARMEN_PARAM_INT, &(polar_slam_params.num_particles), 0, NULL},
			{(char *) "polar_slam", (char *) "num_points_to_store", 	CARMEN_PARAM_INT, &(polar_slam_params.num_points_to_store), 0, NULL},
			{(char *) "polar_slam", (char *) "num_angular_sections", 	CARMEN_PARAM_INT, &(polar_slam_params.num_angular_sections), 0, NULL}
	};

	carmen_param_install_params(argc, argv, basic_param_list, sizeof(basic_param_list) / sizeof(basic_param_list[0]));
	carmen_param_install_params(argc, argv, laser_param_list, sizeof(laser_param_list) / sizeof(laser_param_list[0]));
	carmen_param_install_params(argc, argv, polar_slam_param_list, sizeof(polar_slam_param_list) / sizeof(polar_slam_param_list[0]));
}


void
carmen_polar_slam_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	carmen_fast_polar_slam_define_messages();
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_polar_slam_define_messages();
	carmen_polar_slam_read_parameters(argc, argv);
	carmen_polar_slam_initialize_global_data();
	carmen_polar_slam_subscribe_messages();
	carmen_ipc_dispatch();

	return 0;
}

