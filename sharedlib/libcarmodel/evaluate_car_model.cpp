#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/task_manager_interface.h>
#include "car_model.h"


carmen_robot_ackerman_config_t robot_config;
carmen_semi_trailers_config_t   semi_trailer_config;


static void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "robot", (char *) "max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
		{(char *) "robot", (char *) "max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
		{(char *) "robot", (char *) "length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
		{(char *) "robot", (char *) "width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
		{(char *) "robot", (char *) "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
		{(char *) "robot", (char *) "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward, 1, NULL},
		{(char *) "robot", (char *) "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_wheels, 1,NULL},
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
		{(char *) "robot", (char *) "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
		{(char *) "robot", (char *) "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels, 1, NULL},

		{(char *) "robot", (char *) "desired_decelaration_forward",	 CARMEN_PARAM_DOUBLE, &robot_config.desired_decelaration_forward,					1, NULL},
		{(char *) "robot", (char *) "desired_decelaration_reverse",	 CARMEN_PARAM_DOUBLE, &robot_config.desired_decelaration_reverse,					1, NULL},
		{(char *) "robot", (char *) "desired_acceleration",			 CARMEN_PARAM_DOUBLE, &robot_config.desired_acceleration,							1, NULL},
		{(char *) "robot", (char *) "desired_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.desired_steering_command_rate,					1, NULL},
		{(char *) "robot", (char *) "understeer_coeficient",		 CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient,							1, NULL},
		{(char *) "robot", (char *) "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate, 					1, NULL},

		{(char *) "semi_trailer",	   (char *) "initial_type", CARMEN_PARAM_INT,	 &semi_trailer_config.semi_trailers.type,								 0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));

	if (semi_trailer_config.semi_trailers.type > 0)
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc, argv, semi_trailer_config.semi_trailers.type);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	carmen_robot_and_trailers_traj_point_t robot_state = {};
//	robot_state.v = 1.0;
	double v = 1.0;
	double phi = 0.2;
	double delta_t = 0.01;
	double full_time_interval = 0.1;
	double distance_traveled = 0.0;

	double time0 = carmen_get_time();
	for (int i = 0; i < 10000; i++)
		robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, v, phi, full_time_interval, &distance_traveled, delta_t,
				robot_config, semi_trailer_config);
	double total_time = carmen_get_time() - time0;

	printf("total_time %lf, pose (%lf %lf %lf %lf %lf %lf %lf)\n", total_time,
			robot_state.x, robot_state.y, robot_state.theta, robot_state.phi, robot_state.trailer_theta[0], robot_state.v, distance_traveled);


	robot_state = {};
//	robot_state.v = 1.0;
	distance_traveled = 0.0;
	time0 = carmen_get_time();
	for (int i = 0; i < 10000; i++)
		robot_state = carmen_libcarmodel_recalc_pos_ackerman_new(robot_state, v, phi, full_time_interval, &distance_traveled, delta_t,
				robot_config, semi_trailer_config);
	total_time = carmen_get_time() - time0;

	printf("total_time %lf, pose (%lf %lf %lf %lf %lf %lf %lf)\n", total_time,
			robot_state.x, robot_state.y, robot_state.theta, robot_state.phi, robot_state.trailer_theta[0], robot_state.v, distance_traveled);

	return (0);
}
