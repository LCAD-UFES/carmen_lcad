#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/base_ackerman_messages.h>
#include <carmen/base_ackerman_interface.h>
#include <prob_measurement_model.h>
#include <prob_motion_model.h>
#include <prob_monte_carlo.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/velodyne_interface.h>
#include "slam_montecarlo.h"
#include "slam_messages.h"

static int num_particles; 		// number of particles for Pose and Map
static int num_particles_improved;  	// number of particles for improved proposal distribution of each Pose Particle
static carmen_point_t *_Xt = NULL;   	// particles of the prediction step
static carmen_point_t *Xt = NULL;    	// particles of the correction step
static double *Wt = NULL;  		// particles' weights
static double per_particles_resample; 	// minimum particles dispersion (in percents) for re-sampling

static double robot_length;
static double robot_width;
static double robot_vertical_displacement_from_center;
static double distance_between_front_and_rear_axles;
static double distance_between_rear_wheels; 
static double distance_between_rear_car_and_rear_wheels;
static double distance_between_front_car_and_front_wheels;

static double slam_frequency;
static double slam_last_run = 0.0;

static carmen_map_t **map = NULL; 	// particles maps at time t
static carmen_map_t **map_1 = NULL;	// particles maps at time (t-1)

static BeanRangeFinderMeasurementModelParams laser_params;
static OdometryMotionModelParams odometry_motion_params;
static VelocityMotionModelParams velocity_motion_params;
static AckermanMotionModelParams ackerman_motion_params;
static ProbabilisticMapParams map_params;

//static carmen_localize_particle_message particles_message;
//static carmen_localize_globalpos_message globalpos_message;
//static carmen_localize_sensor_message sensor_message;

static carmen_localize_ackerman_particle_message particles_ackerman_message;
static carmen_localize_ackerman_globalpos_message globalpos_ackerman_message;
static carmen_localize_ackerman_sensor_message sensor_ackerman_message;

static MotionModelTypes motion_model = 0;
static RangeSensorTypes range_sensor = 0;

static carmen_fused_odometry_message *odometry_history;
static int last_odometry;
static int odometry_initialized;
static int odometry_size;

static int pose_initialize = 0;

struct _velodyne_point_cloud
{
	int num_points;
	float *angle;
	float *range;
	carmen_vector_3D_t *car_position;
};

typedef struct _velodyne_point_cloud velodyne_point_cloud;

static velodyne_point_cloud *velodyne_points;
static int velodyne_point_cloud_index;
static int velodyne_initialized;
static int velodyne_size;
static carmen_pose_3D_t velodyne_pose;



int
check_slam_frequency()
{
	if (slam_frequency > 0.0)
	{
		if ((carmen_get_time() - slam_last_run) < (1.0/slam_frequency))
			return 0;
	}
	return 1;
}


/**
 * Low Variance Sampler, Table 4.4 of the book Probabilistic Robotics.
 */
void 
low_variance_sampler(carmen_point_t *Xt, carmen_point_t *_Xt,
		carmen_map_t **map, carmen_map_t **map_1,
		double *wt, int number_of_particles)
{
	int m, i;
	double r, c, U, invM, sum_weights;

	sum_weights = 0;
	for (m = 0; m < number_of_particles; m++)
	{
		//wt[m] = 1.0;
		sum_weights += wt[m];
	}

	invM = sum_weights / (double)number_of_particles; // To avoid problems with a Measurement Model that does not returns probability 1, we sum the weights up
	r = invM * carmen_double_random(1.0);
	c = wt[0];
	i = 0; // The book appears to have a bug: i should start with zero.
	for (m = 1; m <= number_of_particles; m++)
	{
		U = r + (double)(m - 1) * invM;
		while (U > c)
		{
			i = i + 1;
			c = c + wt[i];
		}
		Xt[m - 1] = _Xt[i];
		carmen_map_copy(map[m - 1], map_1[i]);
	}

	for (m = 0; m < number_of_particles; m++)
		Wt[m] = 1.0 / (double) number_of_particles; // after sampling, particles have the same weight
}


void 
resample(carmen_point_t *Xt, carmen_point_t *_Xt,
		carmen_map_t **map, carmen_map_t **map_1,
		double *Wt, int number_of_particles)
{
	int m;
	double mean_wt = 0.0;

	for (m = 0; m < number_of_particles; m++)
		mean_wt += Wt[m];

	mean_wt = mean_wt / number_of_particles;

	//normalize the log m_weights
	double gain = 1.0 / (1.0 * number_of_particles);

	double w_cum = 0.0;
	for (m = 0; m < number_of_particles; m++)
		w_cum += exp(gain * (Wt[m] - mean_wt));

	double m_neff = 0.0;
	for (m = 0; m < number_of_particles; m++)
	{
		double normalized_weight = Wt[m] / w_cum;
		m_neff += normalized_weight * normalized_weight;
	}
	m_neff = 1.0 / m_neff;

	if (0.0 < per_particles_resample * number_of_particles) // @@@ Alberto: este algoritmo nao esta funcionando... Assim, sempre faz o ressample.
	{
		low_variance_sampler(Xt, _Xt, map, map_1, Wt, number_of_particles);
	}
	else
	{
		for (m = 0; m < number_of_particles; m++)
		{
			Xt[m] = _Xt[m];
			carmen_map_copy(map[m], map_1[m]);
		}
	}

}


int
FastSLAM_occupancy_grids_improved_proposal_odometry(carmen_point_t *Xt_1, const OdometryMotionCommand *ut,
		carmen_map_t **map, carmen_map_t **map_1, double *zt,
		int number_of_particles, int number_of_particles_improved)
{
	int m;
	double max_wt = -1.0;
	int max_particle = 0;
	carmen_point_t zt_pose;

	for (m = 0; m < number_of_particles; m++)
	{
		_Xt[m] = improved_proposal_distribution_odometry(ut, Xt_1[m], map_1[m], zt, number_of_particles_improved);
		Wt[m] = carmen_beam_range_finder_measurement_model(zt, &(_Xt[m]), map_1[m]);

		carmen_update_cells_below_robot(map_1[m], _Xt[m]);
		transform_robot_pose_to_laser_pose(&zt_pose, &(_Xt[m]));
		carmen_update_cells_in_the_laser_perceptual_field(map_1[m], zt_pose, zt, &laser_params);
	}

	resample(Xt, _Xt, map, map_1, Wt, number_of_particles);

	for (m = 0; m < number_of_particles; m++)
	{
		if (Wt[m] > max_wt)
		{
			max_wt = Wt[m];
			max_particle = m;
		}
	}

	return max_particle;
}


int
FastSLAM_occupancy_grids_improved_proposal_ackerman(carmen_point_t *Xt_1, const AckermanMotionCommand *ut,
		carmen_map_t **map, carmen_map_t **map_1, float *zt,
		int number_of_particles, int number_of_particles_improved)
{
	int m;
	double max_wt = -1.0;
	int max_particle = 0;
	carmen_point_t zt_pose;

	for (m = 0; m < number_of_particles; m++)
	{
		_Xt[m] = improved_proposal_distribution_ackerman(ut, Xt_1[m], map_1[m], zt, number_of_particles_improved);
		Wt[m] = carmen_beam_range_finder_measurement_model(zt, &(_Xt[m]), map_1[m]);

		carmen_update_cells_below_robot(map_1[m], _Xt[m]);
		transform_robot_pose_to_laser_pose(&zt_pose, &(_Xt[m]));
		carmen_update_cells_in_the_laser_perceptual_field(map_1[m], zt_pose, zt, &laser_params);
	}

	resample(Xt, _Xt, map, map_1, Wt, number_of_particles);

	for (m = 0; m < number_of_particles; m++)
	{
		if (Wt[m] > max_wt)
		{
			max_wt = Wt[m];
			max_particle = m;
		}
	}

	return max_particle;
}


int
FastSLAM_occupancy_grids_improved_proposal_velocity(carmen_point_t *Xt_1, const VelocityMotionCommand *ut,
		carmen_map_t **map, carmen_map_t **map_1, float *zt,
		int number_of_particles, int number_of_particles_improved)
{
	int m;
	double max_wt = -1.0;
	int max_particle = 0;
	carmen_point_t zt_pose;

	for (m = 0; m < number_of_particles; m++)
	{
		_Xt[m] = improved_proposal_distribution_velocity(ut, Xt_1[m], map_1[m], zt, number_of_particles_improved);
		Wt[m] = carmen_beam_range_finder_measurement_model(zt, &(_Xt[m]), map_1[m]);

		carmen_update_cells_below_robot(map_1[m], _Xt[m]);
		transform_robot_pose_to_laser_pose(&zt_pose, &(_Xt[m]));
		carmen_update_cells_in_the_laser_perceptual_field(map_1[m], zt_pose, zt, &laser_params);
	}

	resample(Xt, _Xt, map, map_1, Wt, number_of_particles);

	for (m = 0; m < number_of_particles; m++)
	{
		if (Wt[m] > max_wt)
		{
			max_wt = Wt[m];
			max_particle = m;
		}
	}

	return max_particle;
}


void 
carmen_slam_montecarlo_destroy()
{	// @@@ Alberto: rever esta funcao pois ela nao libera a memoria direito (dos mapas com certeza)
	if (_Xt) 
		free(_Xt);
	if (Xt) 
		free(Xt);
	if (Wt) 
		free(Wt);
	if (map) 
		free(map);
	if (map_1) 
		free(map_1);
//	if (sensor_message.range)
//		free(sensor_message.range);
	if (sensor_ackerman_message.range)
		free(sensor_ackerman_message.range);
//	if (particles_message.particles)
//		free(particles_message.particles);
	if (particles_ackerman_message.particles)
		free(particles_ackerman_message.particles);
}


void
swap_map_1_with_map()
{
	carmen_map_t **temp;

	temp = map_1;
	map_1 = map;
	map = temp;
}


void
carmen_slam_publish_messages(const int best_particle_index, carmen_robot_laser_message *frontlaser)
{
	build_globalpos_message(&globalpos_message, Xt, Wt, num_particles, frontlaser->robot_pose, frontlaser->timestamp);
	build_sensor_message(&sensor_message, &globalpos_message.globalpos, frontlaser, frontlaser->timestamp);
	build_particles_message(&particles_message, &globalpos_message, num_particles, Xt, Wt, frontlaser->timestamp);

	carmen_localize_publish_globalpos_message(&globalpos_message);
	carmen_localize_publish_sensor_message(&sensor_message);
	carmen_localize_publish_particles_message(&particles_message);

	carmen_grid_mapping_publish_message(map[best_particle_index], frontlaser->timestamp);
}


void
carmen_slam_publish_messages_ackerman(const int best_particle_index, carmen_robot_ackerman_laser_message *frontlaser)
{
	build_globalpos_ackerman_message(&globalpos_ackerman_message, Xt, Wt, num_particles, frontlaser->robot_pose, frontlaser->v, frontlaser->phi, 1, frontlaser->timestamp);
	build_sensor_ackerman_message(&sensor_ackerman_message, &globalpos_ackerman_message.globalpos, frontlaser, frontlaser->timestamp);
	build_particles_ackerman_message(&particles_ackerman_message, &globalpos_ackerman_message, num_particles, Xt, Wt, frontlaser->timestamp);

	carmen_localize_ackerman_publish_globalpos_message(&globalpos_ackerman_message);
	carmen_localize_ackerman_publish_particles_message(&particles_ackerman_message);
	carmen_localize_ackerman_publish_sensor_message(&sensor_ackerman_message);

	carmen_grid_mapping_publish_message(map[best_particle_index], frontlaser->timestamp);
}


int
ok_to_perform_slam()
{
	if (!check_slam_frequency() || !pose_initialize)
		return (0);

	swap_map_1_with_map();

	return (1);
}


int
ok_to_perform_slam_ackerman()
{
	if (!check_slam_frequency() || !pose_initialize)
		return (0);

	swap_map_1_with_map();

	return (1);
}


void 
slam_odometry(carmen_robot_laser_message *frontlaser)
{
	static OdometryMotionCommand ut = INITIAL_ODOMETRY;

	update_odometry_motion_command(&ut, (carmen_point_t) frontlaser->robot_pose);
	int best_particle_index = FastSLAM_occupancy_grids_improved_proposal_odometry(Xt, &ut, map, map_1, frontlaser->range, num_particles, num_particles_improved);

	carmen_slam_publish_messages(best_particle_index, frontlaser);

	slam_last_run = carmen_get_time();
}


void 
slam_velocity(carmen_robot_laser_message *frontlaser)
{
	static VelocityMotionCommand ut = INITIAL_VELOCITY;

	int update = update_velocity_motion_command(&ut, frontlaser->tv, frontlaser->rv, frontlaser->timestamp);
	if (!update) 
		return;

	int best_particle_index = FastSLAM_occupancy_grids_improved_proposal_velocity(Xt, &ut, map, map_1, frontlaser->range, num_particles, num_particles_improved);

	carmen_slam_publish_messages(best_particle_index, frontlaser);

	slam_last_run = carmen_get_time();
}


void 
slam_ackerman(carmen_robot_ackerman_laser_message *frontlaser)
{
	static AckermanMotionCommand ut = INITIAL_VELOCITY;

	int update = update_ackerman_motion_command(&ut, frontlaser->v, frontlaser->phi, frontlaser->timestamp);
	if (!update) 
		return;

	int best_particle_index = FastSLAM_occupancy_grids_improved_proposal_ackerman(Xt, &ut, map, map_1, frontlaser->range, num_particles, num_particles_improved);

	carmen_slam_publish_messages_ackerman(best_particle_index, frontlaser);

	slam_last_run = carmen_get_time();
}


void 
robot_frontlaser_handler(carmen_robot_laser_message *frontlaser)
{
	if (ok_to_perform_slam())
		slam_odometry(frontlaser);
}


void 
robot_frontlaser_velocity_handler(carmen_robot_laser_message *frontlaser)
{
	if (ok_to_perform_slam())
		slam_velocity(frontlaser);
}


void 
robot_ackerman_frontlaser_handler(carmen_robot_ackerman_laser_message *frontlaser)
{
	if (ok_to_perform_slam_ackerman())
		slam_ackerman(frontlaser);
}


static void
init_fused_odometry(void)
{
	odometry_initialized = 0; // Only considered initialized when first message is received

	odometry_history = (carmen_fused_odometry_message *) malloc(odometry_size * sizeof(carmen_fused_odometry_message));

	carmen_fused_odometry_message initial_odometry;

	initial_odometry.pose.position.x = 0.0;
	initial_odometry.pose.position.y = 0.0;
	initial_odometry.pose.position.z = 0.0;

	initial_odometry.pose.orientation.roll  = 0.0;
	initial_odometry.pose.orientation.pitch = 0.0;
	initial_odometry.pose.orientation.yaw   = 0.0;

	initial_odometry.velocity.x = 0.0;
	initial_odometry.velocity.y = 0.0;
	initial_odometry.velocity.z = 0.0;

	initial_odometry.angular_velocity.roll  = 0.0;
	initial_odometry.angular_velocity.pitch = 0.0;
	initial_odometry.angular_velocity.yaw   = 0.0;

	initial_odometry.phi = 0.0;

	initial_odometry.timestamp = 0.0;

	for (int i = 0; i < odometry_size; i++)
		odometry_history[i] = initial_odometry;

	last_odometry = 0;
}


static void
carmen_fused_odometry_message_handler(carmen_fused_odometry_message *odometry_message)
{
	odometry_initialized = 1;

	last_odometry++;
	if (last_odometry >= odometry_size)
		last_odometry = 0;

	odometry_history[last_odometry] = *odometry_message;
}


static void
init_velodyne(void)
{
	velodyne_initialized = 0; // Only considered initialized when first message is received

	velodyne_points = (velodyne_point_cloud *) malloc(velodyne_size * sizeof(velodyne_point_cloud));

	for (int i = 0; i < velodyne_size; i++)
	{
		velodyne_points[i].num_points = 0;
		velodyne_points[i].angle = NULL;
		velodyne_points[i].range = NULL;
		velodyne_points[i].car_position = NULL;
	}

	velodyne_point_cloud_index = 0;
}


carmen_vector_3D_t
get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range, rotation_matrix *velodyne_to_car_matrix)
{
	double cos_rot_angle = cos(rot_angle);
	double sin_rot_angle = sin(rot_angle);

	double cos_vert_angle = cos(vert_angle);
	double sin_vert_angle = sin(vert_angle);

	double xy_distance = range * cos_vert_angle;

	carmen_vector_3D_t velodyne_reference;

	velodyne_reference.x = (xy_distance * sin_rot_angle);
	velodyne_reference.y = (xy_distance * cos_rot_angle);
	velodyne_reference.z = (range * sin_vert_angle);

	carmen_vector_3D_t car_reference = multiply_matrix_vector(velodyne_to_car_matrix, velodyne_reference);
	carmen_vector_3D_t reading = add_vectors(car_reference, velodyne_pose.position);

	return reading;
}


carmen_vector_3D_t
get_point_position_global_reference(carmen_vector_3D_t car_position, carmen_vector_3D_t car_reference, rotation_matrix* car_to_global_matrix)
{
	carmen_vector_3D_t global_reference = multiply_matrix_vector(car_to_global_matrix, car_reference);
	carmen_vector_3D_t point = add_vectors(global_reference, car_position);

	return point;
}


int
get_front_laser_beam_index(float velodyne_angle)
{
	int front_laser_beam_index;
	float angle_step, angle;

	// O angulo do Velodyne ee enviado no sentido horario, ao inves de anti-horario como pede Carmen
	angle = carmen_normalize_theta(2.0 * M_PI - velodyne_angle) - carmen_degrees_to_radians(laser_params.start_angle);
	angle_step = carmen_degrees_to_radians(laser_params.fov_range) / (float) laser_params.laser_beams;

	front_laser_beam_index = (int) (0.5 + angle / angle_step);
	if ((front_laser_beam_index >= laser_params.laser_beams) || (front_laser_beam_index < 0))
		return (-1);
	else
		return (front_laser_beam_index);
}


void 
compute_front_laser_message_from_velodyne(carmen_robot_laser_message *frontlaser, 
		velodyne_point_cloud *velodyne_points, int velodyne_point_cloud_index,
		carmen_pose_3D_t pose, double timestamp)
{
	//	static double vertical_correction[32] = { -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0,
	//	-24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001,
	//	-13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };

	int beams_colected;
	int front_laser_beam_index, last_front_laser_beam_index;
	float *velodyne_range;
	float *velodyne_angle;
	int velodyne_beam;

	velodyne_range = velodyne_points[velodyne_point_cloud_index].range;
	velodyne_angle = velodyne_points[velodyne_point_cloud_index].angle;
	velodyne_beam =  velodyne_points[velodyne_point_cloud_index].num_points - 1;

	beams_colected = 0;
	last_front_laser_beam_index = front_laser_beam_index = get_front_laser_beam_index(velodyne_angle[velodyne_beam]);
	while (beams_colected < laser_params.laser_beams)
	{
		while ((front_laser_beam_index == -1) || (last_front_laser_beam_index == front_laser_beam_index))
		{
			velodyne_beam -= 32;
			if (velodyne_beam <= 0)
			{
				velodyne_point_cloud_index--;
				if (velodyne_point_cloud_index < 0)
					velodyne_point_cloud_index = velodyne_size - 1;
				velodyne_range = velodyne_points[velodyne_point_cloud_index].range;
				velodyne_angle = velodyne_points[velodyne_point_cloud_index].angle;
				velodyne_beam =  velodyne_points[velodyne_point_cloud_index].num_points - 1;
			}
			front_laser_beam_index = get_front_laser_beam_index(velodyne_angle[velodyne_beam]);
		}
		frontlaser->range[front_laser_beam_index] = velodyne_range[velodyne_beam];
		frontlaser->tooclose[front_laser_beam_index] = (velodyne_range[velodyne_beam] < 2.0) ? 1: 0; // @@@ obter o valor 2.0 (distacia minima) do carmen.ini

		last_front_laser_beam_index = front_laser_beam_index;
		beams_colected++;
	}
	frontlaser->num_remissions = 0;
	frontlaser->remission = NULL;
	frontlaser->robot_pose.x = pose.position.x;
	frontlaser->robot_pose.y = pose.position.y;
	frontlaser->robot_pose.theta = pose.orientation.yaw;
	frontlaser->laser_pose = frontlaser->robot_pose; // @@@ A pose do velodyne deveria ser computada levando em consideracao sua posicao no carro e o deslocamento do carro no tempo para cada 32 raios
	frontlaser->tv = 1.0; // aparentemente nao usado no slam de odometria
	frontlaser->rv = 0.1; // aparentemente nao usado no slam de odometria
	frontlaser->forward_safety_dist = 0; // @@@ pode ser importante setar
	frontlaser->side_safety_dist = 0; // @@@ pode ser importante setar
	frontlaser->turn_axis = 0;
	frontlaser->timestamp = timestamp;
	frontlaser->host = carmen_get_host();;
}



static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	static double last_timestamp = 0.0;
	static carmen_robot_laser_message frontlaser;

	if (!odometry_initialized)
		return;

	if (last_timestamp == 0.0)
	{
		last_timestamp = velodyne_message->timestamp;
		frontlaser.id = -2;		// correntemente s�o necess�rias pelo menos 2 mensagens para ter uma volta completa de velodyne

		frontlaser.config.laser_type = SICK_LMS;
		frontlaser.config.start_angle = carmen_degrees_to_radians(laser_params.start_angle);
		frontlaser.config.fov = carmen_degrees_to_radians(laser_params.fov_range);
		frontlaser.config.angular_resolution = carmen_degrees_to_radians(laser_params.fov_range) / (float) laser_params.laser_beams;
		frontlaser.config.maximum_range = laser_params.max_range;
		frontlaser.config.accuracy = 0.02;
		frontlaser.config.remission_mode = REMISSION_NONE;

		frontlaser.num_readings = laser_params.laser_beams;
		frontlaser.range = (float *) malloc(frontlaser.num_readings * sizeof(float));
		frontlaser.tooclose = (char *) malloc(frontlaser.num_readings * sizeof(char));
		return;
	}

	velodyne_point_cloud_index++;
	if (velodyne_point_cloud_index >= velodyne_size)
		velodyne_point_cloud_index = 0;

	int num_points = velodyne_message->number_of_32_laser_shots * 32;
	velodyne_points[velodyne_point_cloud_index].num_points = num_points;
	velodyne_points[velodyne_point_cloud_index].angle = (float *) realloc((void *) (velodyne_points[velodyne_point_cloud_index].angle), num_points * sizeof(float));
	velodyne_points[velodyne_point_cloud_index].range = (float *) realloc((void *) (velodyne_points[velodyne_point_cloud_index].range), num_points * sizeof(float));
	velodyne_points[velodyne_point_cloud_index].car_position = (carmen_vector_3D_t *) realloc((void *) (velodyne_points[velodyne_point_cloud_index].car_position), num_points * sizeof(carmen_vector_3D_t));

	rotation_matrix *velodyne_to_car_matrix = create_rotation_matrix(velodyne_pose.orientation);
	rotation_matrix *r_matrix_car_to_global = create_rotation_matrix(odometry_history[last_odometry].pose.orientation);

	for (int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		double shot_time = last_timestamp + (velodyne_message->timestamp - last_timestamp) * ((double) i / (double) velodyne_message->number_of_32_laser_shots);
		carmen_vector_3D_t car_interpolated_position = carmen_get_interpolated_robot_position_at_time(odometry_history[last_odometry].pose, odometry_history[last_odometry].velocity, odometry_history[last_odometry].timestamp, shot_time, r_matrix_car_to_global);

		for (int j = 0; j < 32; j++)
		{
			float angle = carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle);
			float range = velodyne_message->partial_scan[i].distance[j] / 500.0;
			if (range == 0.0)  // @@@ Aparentemente o Velodyne diz range zero quando de range_max
				range = laser_params.max_range;

			velodyne_points[velodyne_point_cloud_index].angle[i*32 + j] = angle;
			velodyne_points[velodyne_point_cloud_index].range[i*32 + j] = range;
			velodyne_points[velodyne_point_cloud_index].car_position[i*32 + j] = car_interpolated_position;
		}
	}

	destroy_rotation_matrix(velodyne_to_car_matrix);
	destroy_rotation_matrix(r_matrix_car_to_global);

	if (frontlaser.id >= 0)
	{
		// @@@ A odometry_history[last_odometry].pose n�o esta sincronizada no tempo com a nuvem de pontos do velodyne
		compute_front_laser_message_from_velodyne(&frontlaser, velodyne_points, velodyne_point_cloud_index, 
				odometry_history[last_odometry].pose, velodyne_message->timestamp);

		if (ok_to_perform_slam())
			slam_odometry(&frontlaser);

		frontlaser.id++;
		if (frontlaser.id > 1000000)
			frontlaser.id = 0;

		velodyne_initialized = 1;
	}
	frontlaser.id++;
	last_timestamp = velodyne_message->timestamp;
}


/**
 * particles, number of particles
 */
void 
carmen_slam_montecarlo_initilize(int number_of_particles)
{
	if (motion_model == AckermanMotionModel)
	{
		globalpos_ackerman_message.host = carmen_get_host();

		particles_ackerman_message.num_particles = number_of_particles;
		particles_ackerman_message.particles = (carmen_localize_ackerman_particle_ipc_p) malloc(number_of_particles * sizeof(carmen_localize_ackerman_particle_ipc_t));
		particles_ackerman_message.host = carmen_get_host();

		sensor_ackerman_message.num_readings = laser_params.laser_beams;
		sensor_ackerman_message.mask = (char *) calloc(sensor_ackerman_message.num_readings, sizeof(char));
		sensor_ackerman_message.range = (float *) malloc(sensor_ackerman_message.num_readings * sizeof(float));
		sensor_ackerman_message.host = carmen_get_host();
		sensor_ackerman_message.laser_skip = laser_params.sampling_step;
		sensor_ackerman_message.num_laser = 1;

		for (int k = 0; k < laser_params.laser_beams; k += laser_params.sampling_step)
			sensor_ackerman_message.mask[k] = 1;
	}
	else
	{
		globalpos_message.host = carmen_get_host();

		particles_message.num_particles = number_of_particles;
		particles_message.particles = (carmen_localize_particle_ipc_p) malloc(number_of_particles * sizeof(carmen_localize_particle_ipc_t));
		particles_message.host = carmen_get_host();

		sensor_message.num_readings = laser_params.laser_beams;
		sensor_message.mask = (char *) calloc(sensor_message.num_readings, sizeof(char));
		sensor_message.range = (float *) malloc(sensor_message.num_readings * sizeof(float));
		sensor_message.host = carmen_get_host();
		sensor_message.laser_skip = laser_params.sampling_step;
		sensor_message.num_laser = 1;

		for (int k = 0; k < laser_params.laser_beams; k += laser_params.sampling_step)
			sensor_message.mask[k] = 1;
	}

	if (motion_model == VelocityMotionModel)
		init_velocity_motion_model(velocity_motion_params);
	else if (motion_model == AckermanMotionModel)
		init_ackerman_motion_model(ackerman_motion_params);
	else
		init_odometry_motion_model(odometry_motion_params);

	_Xt = (carmen_point_t *) calloc(number_of_particles, sizeof(carmen_point_t));
	Xt = (carmen_point_t *) calloc(number_of_particles, sizeof(carmen_point_t));
	Wt = (double *) calloc(number_of_particles, sizeof(double));

	init_bean_range_finder_measurement_model(laser_params);

	// Create map particles
	map = (carmen_map_t **) calloc(number_of_particles, sizeof(carmen_map_t *));
	map_1 = (carmen_map_t **) calloc(number_of_particles, sizeof(carmen_map_t *));
	map[0] = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
	init_carmen_map(&map_params, map[0]);
	map_1[0] = carmen_map_clone(map[0]);
	for (int i = 1; i < number_of_particles; i++)
	{
		map[i] = carmen_map_clone(map[0]);
		map_1[i] = carmen_map_clone(map[0]);
	}
	init_probabilistic_grid_map_model(&map_params, map[0]);

	init_fused_odometry();
	init_velodyne();
}


void
pose_initialize_handler(carmen_localize_initialize_message *msg)
{
	carmen_point_t std[1]; 

	std[0].x = 0.0; std[0].y = 0.0; std[0].theta = 0.0;

	carmen_slam_montecarlo_destroy();
	carmen_slam_montecarlo_initilize(num_particles);

	init_particles_from_gaussians(Xt, _Xt, Wt, msg->mean, std, 1, num_particles);
	build_globalpos_message(&globalpos_message, Xt, Wt, num_particles, msg->mean[0], msg->timestamp);
	carmen_localize_publish_globalpos_message(&globalpos_message);

	slam_last_run = carmen_get_time() + 0.5; //  wait 0.5 seconds in ok_to_perform_slam()
	pose_initialize = 1;
}


void
ackerman_pose_initialize_handler(carmen_localize_ackerman_initialize_message *msg)
{
	carmen_point_t std[1]; 

	std[0].x = 0.0; std[0].y = 0.0; std[0].theta = 0.0;


	carmen_slam_montecarlo_initilize(num_particles);

	init_particles_from_gaussians(Xt, _Xt, Wt, msg->mean, std, 1, num_particles);
	build_globalpos_ackerman_message(&globalpos_ackerman_message, Xt, Wt, num_particles, msg->mean[0], 0.0, 0.0, 1, msg->timestamp);
	carmen_localize_ackerman_publish_globalpos_message(&globalpos_ackerman_message);

	slam_last_run = carmen_get_time() + 0.5; //  wait 0.5 seconds in ok_to_perform_slam()
	pose_initialize = 1;
}


void 
carmen_slam_montecarlo_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_SENSOR_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_SENSOR_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);

	carmen_grid_mapping_define_messages();

	carmen_localize_define_globalpos_messages();

	carmen_localize_ackerman_define_globalpos_messages();
}


void 
shutdown_module(int sig)
{
	if (sig == SIGINT)
	{
		carmen_ipc_disconnect();

		fprintf(stderr, "\nDisconnecting (shutdown_module(%d) called).\n", sig);
		exit(0);
	}
}


void 
carmen_slam_read_parameters(int argc, char **argv,
		BeanRangeFinderMeasurementModelParams *p_laser_params,
		OdometryMotionModelParams *p_motion_model_params,
		VelocityMotionModelParams *p_velocity_model_params,
		AckermanMotionModelParams *p_ackerman_model_params,
		ProbabilisticMapParams *p_map_params)
{
	carmen_param_t basic_param_list[] = 
	{
			{"slam", "num_particles", CARMEN_PARAM_INT, 			&num_particles, 0, NULL},
			{"slam", "num_particles_improved", CARMEN_PARAM_INT, 		&num_particles_improved, 0, NULL},
			{"slam", "per_particles_resample", CARMEN_PARAM_DOUBLE, 	&per_particles_resample, 0, NULL},
			{"slam", "frequency", CARMEN_PARAM_DOUBLE, 			&slam_frequency, 1, NULL},
			{"slam", "odometry_size", CARMEN_PARAM_INT, 			&odometry_size, 0, NULL},
			{"slam", "velodyne_size", CARMEN_PARAM_INT, 			&velodyne_size, 0, NULL},

			{"slam", "motion_model", CARMEN_PARAM_INT, 		&motion_model, 0, NULL},

			{"slam", "ack_a1", CARMEN_PARAM_DOUBLE, 			&p_ackerman_model_params->alpha1, 0, NULL},
			{"slam", "ack_a2", CARMEN_PARAM_DOUBLE, 			&p_ackerman_model_params->alpha2, 0, NULL},
			{"slam", "ack_a3", CARMEN_PARAM_DOUBLE, 			&p_ackerman_model_params->alpha3, 0, NULL},
			{"slam", "ack_a4", CARMEN_PARAM_DOUBLE, 			&p_ackerman_model_params->alpha4, 0, NULL},

			{"slam", "vel_a1", CARMEN_PARAM_DOUBLE, 			&p_velocity_model_params->alpha1, 0, NULL},
			{"slam", "vel_a2", CARMEN_PARAM_DOUBLE, 			&p_velocity_model_params->alpha2, 0, NULL},
			{"slam", "vel_a3", CARMEN_PARAM_DOUBLE, 			&p_velocity_model_params->alpha3, 0, NULL},
			{"slam", "vel_a4", CARMEN_PARAM_DOUBLE, 			&p_velocity_model_params->alpha4, 0, NULL},
			{"slam", "vel_a5", CARMEN_PARAM_DOUBLE, 			&p_velocity_model_params->alpha5, 0, NULL},
			{"slam", "vel_a6", CARMEN_PARAM_DOUBLE, 			&p_velocity_model_params->alpha6, 0, NULL},

			{"slam", "map_locc", CARMEN_PARAM_INT, 			&p_map_params->locc, 0, NULL},
			{"slam", "map_lfree", CARMEN_PARAM_INT, 			&p_map_params->lfree, 0, NULL},
			{"slam", "map_l0", CARMEN_PARAM_INT, 			&p_map_params->l0, 0, NULL},
			{"slam", "map_log_odds_max", CARMEN_PARAM_INT, 		&p_map_params->log_odds_max, 0, NULL},
			{"slam", "map_log_odds_min", CARMEN_PARAM_INT, 		&p_map_params->log_odds_min, 0, NULL},
			{"slam", "map_log_odds_bias", CARMEN_PARAM_INT, 		&p_map_params->log_odds_bias, 0, NULL},
			{"slam", "map_grid_res", CARMEN_PARAM_DOUBLE, 		&p_map_params->grid_res, 0, NULL},
			{"slam", "map_range_factor", CARMEN_PARAM_DOUBLE, 	&p_map_params->range_factor, 0, NULL},
			{"slam", "map_width", CARMEN_PARAM_DOUBLE, 		&p_map_params->width, 0, NULL},
			{"slam", "map_height", CARMEN_PARAM_DOUBLE, 		&p_map_params->height, 0, NULL},

			{"slam", "odom_a1", CARMEN_PARAM_DOUBLE, 			&p_motion_model_params->alpha1, 1, NULL},
			{"slam", "odom_a2", CARMEN_PARAM_DOUBLE, 			&p_motion_model_params->alpha2, 1, NULL},
			{"slam", "odom_a3", CARMEN_PARAM_DOUBLE, 			&p_motion_model_params->alpha3, 1, NULL},
			{"slam", "odom_a4", CARMEN_PARAM_DOUBLE, 			&p_motion_model_params->alpha4, 1, NULL},

			{"robot", "length", CARMEN_PARAM_DOUBLE, 				&robot_length, 0, NULL},
			{"robot", "width", CARMEN_PARAM_DOUBLE, 				&robot_width, 0, NULL},
			{"robot", "vertical_displacement_from_center", CARMEN_PARAM_DOUBLE, 	&robot_vertical_displacement_from_center, 0, NULL},

			{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, 		&p_ackerman_model_params->L, 0, NULL},
			{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, 	&distance_between_front_and_rear_axles, 0, NULL},
			{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, 		&distance_between_rear_wheels, 0, NULL},
			{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, 	&distance_between_rear_car_and_rear_wheels, 0, NULL},
			{"robot", "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &distance_between_front_car_and_front_wheels, 0, NULL},

			{"slam", "range_sensor_type", CARMEN_PARAM_INT, 		&range_sensor, 0, NULL},
	};

	carmen_param_t laser_param_list[] = 
	{
			{"slam", "laser_sampling_step", CARMEN_PARAM_INT, 	&p_laser_params->sampling_step, 0, NULL},
			{"slam", "laser_num_beams", CARMEN_PARAM_INT, 		&p_laser_params->laser_beams, 0, NULL},
			{"slam", "laser_fov_range", CARMEN_PARAM_DOUBLE, 		&p_laser_params->fov_range, 0, NULL},
			{"slam", "laser_max_range", CARMEN_PARAM_DOUBLE, 		&p_laser_params->max_range, 0, NULL},
			{"slam", "laser_lambda_short", CARMEN_PARAM_DOUBLE, 	&p_laser_params->lambda_short, 0, NULL},
			{"slam", "laser_sigma_zhit", CARMEN_PARAM_DOUBLE, 	&p_laser_params->sigma_zhit, 0, NULL},
			{"slam", "laser_zhit", CARMEN_PARAM_DOUBLE, 		&p_laser_params->zhit, 0, NULL},
			{"slam", "laser_zmax", CARMEN_PARAM_DOUBLE, 		&p_laser_params->zmax, 0, NULL},
			{"slam", "laser_zrand", CARMEN_PARAM_DOUBLE, 		&p_laser_params->zrand, 0, NULL},
			{"slam", "laser_zshort", CARMEN_PARAM_DOUBLE, 		&p_laser_params->zshort, 0, NULL},

			{"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE, 		&p_laser_params->front_offset, 0, NULL},
			{"robot", "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, 	&p_laser_params->side_offset, 0, NULL},
			{"robot", "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, 	&p_laser_params->angular_offset, 0, NULL},
	};

	carmen_param_t velodyne_param_list[] = 
	{
			{"slam", "velodyne_sampling_step", CARMEN_PARAM_INT,	   &p_laser_params->sampling_step, 0, NULL},
			{"slam", "velodyne_num_beams", CARMEN_PARAM_INT,  	   &p_laser_params->laser_beams, 0, NULL},
			{"slam", "velodyne_fov_range", CARMEN_PARAM_DOUBLE,	   &p_laser_params->fov_range, 0, NULL},
			{"slam", "velodyne_start_angle", CARMEN_PARAM_DOUBLE,	   &p_laser_params->start_angle, 0, NULL},
			{"slam", "velodyne_max_range", CARMEN_PARAM_DOUBLE,	   &p_laser_params->max_range, 0, NULL},
			{"slam", "velodyne_lambda_short", CARMEN_PARAM_DOUBLE,	   &p_laser_params->lambda_short, 0, NULL},
			{"slam", "velodyne_sigma_zhit", CARMEN_PARAM_DOUBLE,	   &p_laser_params->sigma_zhit, 0, NULL},
			{"slam", "velodyne_zhit", CARMEN_PARAM_DOUBLE,		   &p_laser_params->zhit, 0, NULL},
			{"slam", "velodyne_zmax", CARMEN_PARAM_DOUBLE,		   &p_laser_params->zmax, 0, NULL},
			{"slam", "velodyne_zrand", CARMEN_PARAM_DOUBLE,		   &p_laser_params->zrand, 0, NULL},
			{"slam", "velodyne_zshort", CARMEN_PARAM_DOUBLE,  	   &p_laser_params->zshort, 0, NULL},

			{"velodyne", "x", CARMEN_PARAM_DOUBLE, 				&(velodyne_pose.position.x), 0, NULL},
			{"velodyne", "y", CARMEN_PARAM_DOUBLE, 				&(velodyne_pose.position.y), 0, NULL},
			{"velodyne", "z", CARMEN_PARAM_DOUBLE, 				&(velodyne_pose.position.z), 0, NULL},

			{"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE, 		&p_laser_params->front_offset, 0, NULL},
			{"robot", "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, 	&p_laser_params->side_offset, 0, NULL},
			{"robot", "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, 	&p_laser_params->angular_offset, 0, NULL},
	};

	carmen_param_install_params(argc, argv, basic_param_list, sizeof(basic_param_list) / sizeof(basic_param_list[0]));
	if (range_sensor == VelodyneHDL32E)
		carmen_param_install_params(argc, argv, velodyne_param_list, sizeof(velodyne_param_list) / sizeof(velodyne_param_list[0]));
	else
	{
		carmen_param_install_params(argc, argv, laser_param_list, sizeof(laser_param_list) / sizeof(laser_param_list[0]));
		p_laser_params->start_angle = -0.5 * p_laser_params->fov_range;
	}

	p_map_params->range_max = p_laser_params->max_range;
	p_map_params->range_step = p_laser_params->fov_range / (double)(p_laser_params->laser_beams-1);
	p_map_params->range_start = p_laser_params->start_angle;
	p_map_params->num_ranges = p_laser_params->laser_beams;

	p_map_params->grid_sx = round(p_map_params->width / p_map_params->grid_res);
	p_map_params->grid_sy = round(p_map_params->height / p_map_params->grid_res);
	p_map_params->grid_size = p_map_params->grid_sx * p_map_params->grid_sy;

	if (motion_model == AckermanMotionModel)
	{
		p_map_params->robot_width = distance_between_rear_wheels;
		p_map_params->robot_length = distance_between_front_and_rear_axles + distance_between_rear_car_and_rear_wheels + distance_between_front_car_and_front_wheels;
		p_map_params->robot_vertical_displacement_from_center = -((p_map_params->robot_length / 2.0) - distance_between_rear_car_and_rear_wheels);
	}
	else
	{
		p_map_params->robot_width = robot_width;
		p_map_params->robot_length = robot_length;
		p_map_params->robot_vertical_displacement_from_center = robot_vertical_displacement_from_center;
	}
}


static void
carmen_slam_subscribe_ipc_messages(void)
{
	if (range_sensor == SickLMS200)
	{
		if (motion_model == AckermanMotionModel)
		{
			carmen_robot_ackerman_subscribe_frontlaser_message(NULL, (carmen_handler_t) robot_ackerman_frontlaser_handler, CARMEN_SUBSCRIBE_LATEST);
			carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) ackerman_pose_initialize_handler, CARMEN_SUBSCRIBE_ALL);
		}
		else
		{
			if (motion_model == VelocityMotionModel)
				carmen_robot_subscribe_frontlaser_message(NULL, (carmen_handler_t) robot_frontlaser_velocity_handler, CARMEN_SUBSCRIBE_LATEST);
			else
				carmen_robot_subscribe_frontlaser_message(NULL, (carmen_handler_t) robot_frontlaser_handler, CARMEN_SUBSCRIBE_LATEST);

			carmen_localize_subscribe_initialize_message(NULL, (carmen_handler_t) pose_initialize_handler, CARMEN_SUBSCRIBE_ALL);
		}
	}
	else // range sensor VelodyneHDL32E
	{
		carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t)carmen_fused_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t)velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

		if (motion_model == AckermanMotionModel)
			carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) ackerman_pose_initialize_handler, CARMEN_SUBSCRIBE_ALL);
		else
			carmen_localize_subscribe_initialize_message(NULL, (carmen_handler_t) pose_initialize_handler, CARMEN_SUBSCRIBE_ALL);
	}
}


int 
main(int argc, char ** argv)
{
	signal(SIGINT, shutdown_module);
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_slam_read_parameters(argc, argv,
			&laser_params,
			&odometry_motion_params,
			&velocity_motion_params,
			&ackerman_motion_params,
			&map_params);

	carmen_slam_montecarlo_initilize(num_particles);
	carmen_slam_montecarlo_define_messages();
	carmen_slam_subscribe_ipc_messages();

	carmen_warn("SLAM Monte Carlo initialized.\n");

	IPC_dispatch();

	return 0;
}
