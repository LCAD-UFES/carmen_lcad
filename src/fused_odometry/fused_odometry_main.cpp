
#include <carmen/carmen.h>
#include <carmen/xsens_interface.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/visual_odometry_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/localize_ackerman_core.h>

#include "fused_odometry.h"
#include "xsens_xyz_handler.h"
#include "visual_odometry_handler.h"
#include "car_odometry_handler.h"
#include "fused_odometry_interface.h"

#include <tf.h>


static int argc_g;
static char **argv_g;

// Maessage handlers state
static xsens_xyz_handler *xsens_xyz_h = NULL;
static visual_odometry_handler *viso_h = NULL;
static car_odometry_handler *car_odom_h = NULL;

static int use_viso_odometry;
static int use_car_odometry;

// Fused odometry state
extern int num_particles;
extern carmen_fused_odometry_particle *xt;
extern carmen_fused_odometry_particle *_xt;
extern carmen_fused_odometry_message fused_odometry_message;

static carmen_vector_3D_t *particle_pos = NULL;
static double *particle_weight = NULL;

static carmen_vector_3D_t gps_initial_pos;

int kalman_filter;

extern carmen_fused_odometry_control ut;

static double fused_odometry_frequency;

static int publish_particles = 0;


static carmen_fused_odometry_particle_message 
compute_new_average_state(carmen_fused_odometry_particle *xt)
{
	carmen_fused_odometry_particle_message average_message;
	// this is necessary to take true average of angles
	double xsens_yaw_bias_x = 0.0;
	double xsens_yaw_bias_y = 0.0;
	double roll_x = 0.0;
	double roll_y = 0.0;
	double pitch_x = 0.0;
	double pitch_y = 0.0;
	double yaw_x = 0.0;
	double yaw_y = 0.0;
	int m;
	double invM;

	average_message.pose.position.x = 0.0;
	average_message.pose.position.y = 0.0;
	average_message.pose.position.z = 0.0;
	average_message.xsens_yaw_bias = 0.0;
	average_message.velocity.x = 0.0;
	average_message.velocity.y = 0.0;
	average_message.velocity.z = 0.0;
	average_message.phi = 0.0;
	average_message.angular_velocity.roll = 0.0;
	average_message.angular_velocity.pitch = 0.0;
	average_message.angular_velocity.yaw = 0.0;
	 
	invM = 1.0 / (double) num_particles;

	for (m = 0; m < num_particles; m++)
	{
		average_message.pose.position.x += 		xt[m].state.pose.position.x * invM;
		average_message.pose.position.y += 		xt[m].state.pose.position.y * invM;
		average_message.pose.position.z += 		xt[m].state.pose.position.z * invM;

		xsens_yaw_bias_x += cos(xt[m].state.xsens_yaw_bias) * invM;
		xsens_yaw_bias_y += sin(xt[m].state.xsens_yaw_bias) * invM;

		average_message.velocity.x += 			xt[m].state.velocity.x * invM;
		average_message.velocity.y += 			xt[m].state.velocity.y * invM;
		average_message.velocity.z += 			xt[m].state.velocity.z * invM;

		average_message.phi +=	xt[m].state.phi * invM;		

		average_message.angular_velocity.roll += 	xt[m].state.ang_velocity.roll * invM;
		average_message.angular_velocity.pitch += 	xt[m].state.ang_velocity.pitch * invM;
		average_message.angular_velocity.yaw += 	xt[m].state.ang_velocity.yaw * invM;

		roll_x += 	cos(xt[m].state.pose.orientation.roll) * invM;	
		roll_y += 	sin(xt[m].state.pose.orientation.roll) * invM;		
		pitch_x +=	cos(xt[m].state.pose.orientation.pitch) * invM;		
		pitch_y +=	sin(xt[m].state.pose.orientation.pitch) * invM;		
		yaw_x += 	cos(xt[m].state.pose.orientation.yaw) * invM;
		yaw_y += 	sin(xt[m].state.pose.orientation.yaw) * invM;	
	}

	average_message.xsens_yaw_bias = atan2(xsens_yaw_bias_y, xsens_yaw_bias_x);

	average_message.pose.orientation.roll = atan2(roll_y, roll_x);
	average_message.pose.orientation.pitch = atan2(pitch_y, pitch_x);
	average_message.pose.orientation.yaw = atan2(yaw_y, yaw_x);

	average_message.gps_position_at_turn_on = gps_initial_pos;

	average_message.weight_type = xt[0].weight_type;
	average_message.timestamp = xt[0].state.timestamp;
	average_message.host = carmen_get_host();

	// This is for displaying the particles position with the 3D_viewer
	for (m = 0; m < num_particles; m++)
	{
		particle_pos[m] = xt[m].state.pose.position;
		particle_weight[m] = xt[m].weight;
	}

	average_message.num_particles = num_particles;
	average_message.particle_pos = particle_pos;
	average_message.weights = particle_weight;

	return (average_message);
}


carmen_fused_odometry_message
assemble_fused_odometry_message(carmen_fused_odometry_particle_message particle_message)
{
	carmen_fused_odometry_message message;

	message.pose = particle_message.pose;
	message.xsens_yaw_bias = particle_message.xsens_yaw_bias;
	message.velocity = particle_message.velocity;
	message.angular_velocity = particle_message.angular_velocity;
	message.phi = particle_message.phi;
	message.gps_position_at_turn_on = particle_message.gps_position_at_turn_on;
	message.timestamp = particle_message.timestamp;
	message.host = carmen_get_host();

	return (message);
}


void 
publish_fused_odometry(void)
{
	IPC_RETURN_TYPE err;
	
	if (xt == NULL)
		return;

	carmen_fused_odometry_particle_message fused_odometry_particle_message = compute_new_average_state(xt);
	fused_odometry_message = assemble_fused_odometry_message(fused_odometry_particle_message);

	err = IPC_publishData(CARMEN_FUSED_ODOMETRY_NAME, &fused_odometry_message); 	
	carmen_test_ipc_exit(err, "Could not publish fused odometry", CARMEN_FUSED_ODOMETRY_NAME);

	// printf("yaw = %+2.3lf, bias = %+2.3lf\n", fused_odometry_particle_message.pose.orientation.yaw, fused_odometry_particle_message.xsens_yaw_bias);
	
	if (!kalman_filter && publish_particles)
	{
		if (fused_odometry_particle_message.num_particles > 200) // @@@ Alberto: Por alguma razao, o ipc trava depois de um tempo se o numero de particulas for maior que 500... Parece ser problema de thread no ipc...
			fused_odometry_particle_message.num_particles = 200;
		err = IPC_publishData(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, &fused_odometry_particle_message);
		carmen_test_ipc_exit(err, "Could not publish fused odometry particle message", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
	}
}


void
correct_fused_odometry_timming_and_publish()
{
	if ((xt != NULL) && is_global_pos_initialized())
	{	
/*		double dt = carmen_get_time() - last_prediction_time;
		double current_time = xt[0].state.timestamp + dt;
		prediction(current_time);

*/		publish_fused_odometry();
	}
}


static void 
define_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME,	IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_FRONTLASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);
}


static carmen_fused_odometry_state_vector
randomize_state_vector(carmen_fused_odometry_state_vector initial_state, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	carmen_fused_odometry_state_vector new_sv;
	carmen_point_t std = get_std_error(xsens_xyz_h, fused_odometry_parameters);

	new_sv.pose.position.x = initial_state.pose.position.x + carmen_gaussian_random(0.0, std.x);
	new_sv.pose.position.y = initial_state.pose.position.y + carmen_gaussian_random(0.0, std.y);
	new_sv.pose.position.z = 0.0; // initial_state.pose.position.z + carmen_gaussian_random(0.0, fused_odometry_parameters->xsens_gps_z_std_error);

	new_sv.pose.orientation.roll = 0.0; // initial_state.pose.orientation.roll + carmen_gaussian_random(0.0, fused_odometry_parameters->xsens_roll_std_error);
	new_sv.pose.orientation.pitch =	0.0; // initial_state.pose.orientation.pitch + carmen_gaussian_random(0.0, fused_odometry_parameters->xsens_pitch_std_error);
	new_sv.pose.orientation.yaw = initial_state.pose.orientation.yaw + carmen_gaussian_random(0.0, std.theta);

	new_sv.xsens_yaw_bias = 0.0; // initial_state.xsens_yaw_bias + carmen_gaussian_random(0.0, fused_odometry_parameters->xsens_yaw_bias_noise); 

	new_sv.velocity.x = 0.0; // initial_state.velocity.x + carmen_gaussian_random(0.0, 1.0);
	new_sv.velocity.y = 0.0; // initial_state.velocity.y + carmen_gaussian_random(0.0, 1.0);
	new_sv.velocity.z = 0.0; // initial_state.velocity.z + carmen_gaussian_random(0.0, 1.0);

	new_sv.ang_velocity.roll = 0.0; // initial_state.ang_velocity.roll	+ carmen_gaussian_random(0.0, 0.01);
	new_sv.ang_velocity.pitch = 0.0; // initial_state.ang_velocity.pitch	+ carmen_gaussian_random(0.0, 0.3);
	new_sv.ang_velocity.yaw = 0.0; // initial_state.ang_velocity.yaw	+ carmen_gaussian_random(0.0, 0.3);

	new_sv.phi = initial_state.phi; // + carmen_gaussian_random(0.0, fused_odometry_parameters->phi_noise_phi);

	new_sv.timestamp = initial_state.timestamp;

	return new_sv;	
}


void 
init_particles(carmen_fused_odometry_state_vector initial_state, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	int m;

	if (xt == NULL)
		xt = (carmen_fused_odometry_particle *) malloc(num_particles * sizeof(carmen_fused_odometry_particle));
	if (_xt == NULL)
		_xt = (carmen_fused_odometry_particle *) malloc(num_particles * sizeof(carmen_fused_odometry_particle));
	if (particle_pos == NULL)
		particle_pos = (carmen_vector_3D_t *) malloc(num_particles * sizeof(carmen_vector_3D_t));
	if (particle_weight == NULL)
		particle_weight = (double *) malloc(num_particles * sizeof(double));

	for (m = 0; m < num_particles; m++)
	{				
		xt[m].state = randomize_state_vector(initial_state, fused_odometry_parameters);
		xt[m].weight = 1.0 / (double) num_particles;
		xt[m].weight_type = -1;
	}

	gps_initial_pos = initial_state.pose.position;

}


void
carmen_fused_odometry_initialize(carmen_fused_odometry_parameters *fused_odometry_parameters)
{	
	if (xsens_xyz_h == NULL)
		xsens_xyz_h = create_xsens_xyz_handler(argc_g, argv_g, fused_odometry_parameters);
	else
		reset_xsens_xyz_handler(xsens_xyz_h);

	if (use_viso_odometry)
	{
		if (viso_h == NULL)
			viso_h = create_visual_odometry_handler(argc_g, argv_g);
		else
			reset_visual_odometry_handler(viso_h);
	}

	if (use_car_odometry)
	{
		if (car_odom_h == NULL)
			car_odom_h = create_car_odometry_handler(xsens_xyz_h, fused_odometry_parameters);
		else
			reset_car_odometry_handler(car_odom_h);
	}

	gps_initial_pos.x = 0.0;
	gps_initial_pos.y = 0.0;
	gps_initial_pos.z = 0.0;

	ut.v = 0.0;
	ut.phi = 0.0;
	ut.z = 0.0;
	ut.v_z = 0.0;
	ut.v_pitch = 0.0;
	ut.pitch = 0.0;
	ut.roll = 0.0;

	// last_prediction_time = carmen_get_time();
}


void
finalize_fused_odometry()
{
	destroy_xsens_xyz_handler(xsens_xyz_h);
	
	if (use_viso_odometry)
		destroy_visual_odometry_handler(viso_h);

	if (use_car_odometry)
		destroy_car_odometry_handler(car_odom_h);
}


//void
//fused_odometry_main_loop(void)
//{
//	while (1)
//	{
//		correct_fused_odometry_timming_and_publish();
//		carmen_ipc_sleep(1.0 / fused_odometry_frequency);
//	}
//}


static void 
initialize_carmen_parameters(int argc, char** argv, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	int num_items;

	carmen_param_t param_list[] = 
	{
		{(char *) "fused_odometry", 	(char *) "num_particles", CARMEN_PARAM_INT, &num_particles, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "frequency", CARMEN_PARAM_DOUBLE, &fused_odometry_frequency, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "use_viso_odometry", CARMEN_PARAM_INT, &use_viso_odometry, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "use_car_odometry", CARMEN_PARAM_INT, &use_car_odometry, 0, NULL},	

		{(char *) "fused_odometry", 	(char *) "velocity_noise_velocity", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->velocity_noise_velocity, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "velocity_noise_phi", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->velocity_noise_phi, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "phi_noise_phi", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->phi_noise_phi, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "phi_noise_velocity", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->phi_noise_velocity, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "pitch_v_noise_pitch_v", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->pitch_v_noise_pitch_v, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "pitch_v_noise_velocity", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->pitch_v_noise_velocity, 0, NULL},

		{(char *) "fused_odometry", 	(char *) "minimum_speed_for_correction", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->minimum_speed_for_correction, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "xsens_yaw_bias_noise", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->xsens_yaw_bias_noise, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "xsens_maximum_yaw_bias", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->xsens_maximum_yaw_bias, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "maximum_phi", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->maximum_phi, 0, NULL},
		
		{(char *) "fused_odometry", 	(char *) "xsens_gps_x_std_error", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->xsens_gps_x_std_error, 0, NULL},	
		{(char *) "fused_odometry", 	(char *) "xsens_gps_y_std_error", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->xsens_gps_y_std_error, 0, NULL},	
		{(char *) "fused_odometry", 	(char *) "xsens_gps_z_std_error", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->xsens_gps_z_std_error, 0, NULL},	

		{(char *) "fused_odometry", 	(char *) "xsens_roll_std_error", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->xsens_roll_std_error, 0, NULL},	
		{(char *) "fused_odometry", 	(char *) "xsens_pitch_std_error", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->xsens_pitch_std_error, 0, NULL},	
		{(char *) "fused_odometry", 	(char *) "xsens_yaw_std_error", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->xsens_yaw_std_error, 0, NULL},	

		{(char *) "fused_odometry", 	(char *) "kalman_filter", CARMEN_PARAM_ONOFF, &kalman_filter, 0, NULL},

		{(char *) "fused_odometry", 	(char *) "publish_particles", CARMEN_PARAM_ONOFF, &publish_particles, 0, NULL},

		{(char *) "robot", 	    	(char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &fused_odometry_parameters->axis_distance, 0, NULL},
	};
	
	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	fused_odometry_parameters->velocity_noise_phi = carmen_degrees_to_radians(fused_odometry_parameters->velocity_noise_phi);
	fused_odometry_parameters->phi_noise_phi = carmen_degrees_to_radians(fused_odometry_parameters->phi_noise_phi);
	fused_odometry_parameters->phi_noise_velocity = carmen_degrees_to_radians(fused_odometry_parameters->phi_noise_velocity);

	fused_odometry_parameters->pitch_v_noise_pitch_v = carmen_degrees_to_radians(fused_odometry_parameters->pitch_v_noise_pitch_v);
	fused_odometry_parameters->pitch_v_noise_velocity = carmen_degrees_to_radians(fused_odometry_parameters->pitch_v_noise_velocity);

	fused_odometry_parameters->xsens_yaw_bias_noise = carmen_degrees_to_radians(fused_odometry_parameters->xsens_yaw_bias_noise);
	fused_odometry_parameters->xsens_maximum_yaw_bias = carmen_degrees_to_radians(fused_odometry_parameters->xsens_maximum_yaw_bias);
	fused_odometry_parameters->maximum_phi = carmen_degrees_to_radians(fused_odometry_parameters->maximum_phi);

	fused_odometry_parameters->xsens_roll_std_error = carmen_degrees_to_radians(fused_odometry_parameters->xsens_roll_std_error);
	fused_odometry_parameters->xsens_pitch_std_error = carmen_degrees_to_radians(fused_odometry_parameters->xsens_pitch_std_error);
	fused_odometry_parameters->xsens_yaw_std_error = carmen_degrees_to_radians(fused_odometry_parameters->xsens_yaw_std_error);
}


int 
main(int argc, char** argv)
{ 	
	argc_g = argc;
	argv_g = argv;
	carmen_fused_odometry_parameters fused_odometry_parameters;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	initialize_carmen_parameters(argc, argv, &fused_odometry_parameters);

	define_ipc_messages();

	carmen_fused_odometry_initialize(&fused_odometry_parameters);

	// carmen_ipc_addPeriodicTimer(1.0 / fused_odometry_frequency, (TIMER_HANDLER_TYPE) correct_fused_odometry_timming_and_publish, NULL);

	carmen_ipc_dispatch();
	// fused_odometry_main_loop();

	return 0;
}
