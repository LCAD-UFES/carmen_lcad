
#include <carmen/carmen.h>
#include <carmen/xsens_interface.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/visual_odometry_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/localize_ackerman_core.h>

#include "fused_odometry_kalman.h"
#include "xsens_xyz_handler.h"
#include "car_odometry_handler.h"
#include <carmen/fused_odometry_interface.h>

#include <tf.h>


static int argc_g;
static char **argv_g;

// Message handlers state
static xsens_xyz_handler *xsens_xyz_h = NULL;
static car_odometry_handler *car_odom_h = NULL;

static int use_viso_odometry;
static int use_car_odometry;

static carmen_vector_3D_t gps_initial_pos;

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
	
	carmen_fused_odometry_message fused_odometry_message;

	err = IPC_publishData(CARMEN_FUSED_ODOMETRY_NAME, &fused_odometry_message); 	
	carmen_test_ipc_exit(err, "Could not publish fused odometry", CARMEN_FUSED_ODOMETRY_NAME);
}


static void 
register_ipc_messages(void)
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




void
carmen_fused_odometry_initialize(carmen_fused_odometry_parameters *fused_odometry_parameters)
{	
	if (xsens_xyz_h == NULL)
		xsens_xyz_h = create_xsens_xyz_handler(argc_g, argv_g, fused_odometry_parameters);
	else
		reset_xsens_xyz_handler(xsens_xyz_h);

	if (car_odom_h == NULL)
		car_odom_h = create_car_odometry_handler(xsens_xyz_h, fused_odometry_parameters);
	else
		reset_car_odometry_handler(car_odom_h);

	gps_initial_pos.x = 0.0;
	gps_initial_pos.y = 0.0;
	gps_initial_pos.z = 0.0;

}


void
finalize_fused_odometry()
{
	destroy_xsens_xyz_handler(xsens_xyz_h);
	
	if (use_car_odometry)
	{
		destroy_car_odometry_handler(car_odom_h);
	}
}

static void 
initialize_carmen_parameters(int argc, char** argv, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	int num_items;

	carmen_param_t param_list[] = 
	{
		{(char *) "fused_odometry", 	(char *) "use_viso_odometry", CARMEN_PARAM_INT, &use_viso_odometry, 0, NULL},
		{(char *) "fused_odometry", 	(char *) "use_car_odometry", CARMEN_PARAM_INT, &use_car_odometry, 0, NULL},	

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

	register_ipc_messages();

	carmen_fused_odometry_initialize(&fused_odometry_parameters);

	carmen_ipc_dispatch();

	return 0;
}
