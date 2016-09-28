#include <carmen/carmen.h>
#include "../stehs_planner/stehs_planner.h"


stehs_planner_config_t stehs_planner_config;


void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
		{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.length,								 			1, NULL},
		{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.width,								 			1, NULL},
		{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.distance_between_rear_wheels,			 		1, NULL},
		{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.distance_between_front_and_rear_axles, 			1, NULL},
		{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
		{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
		{(char *) "robot", 	(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.max_v,									 		1, NULL},
		{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.max_phi,								 		1, NULL},
		{(char *) "robot", 	(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.maximum_acceleration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.maximum_acceleration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.maximum_deceleration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.maximum_deceleration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.maximum_steering_command_rate,					1, NULL},
		{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &stehs_planner_config.robot_config.understeer_coeficient,							1, NULL},

		{(char *) "rrt",	(char *) "show_debug_info",								CARMEN_PARAM_ONOFF,	 &stehs_planner_config.show_debug_info,												1, NULL},
		//{(char *) "rrt",	(char *) "use_obstacle_avoider", 						CARMEN_PARAM_ONOFF,	 &GlobalState::use_obstacle_avoider, 												1, NULL},
		//{(char *) "rrt",	(char *) "use_mpc",										CARMEN_PARAM_ONOFF,	 &GlobalState::use_mpc, 															0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	//register_handlers();

	carmen_ipc_dispatch();
}
