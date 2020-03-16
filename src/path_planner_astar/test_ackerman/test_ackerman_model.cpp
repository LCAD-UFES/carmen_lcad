#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <algorithm>
#include <car_model.h>

#include <queue>
#include <list>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DELTA_T 0.01                      // Size of step for the ackerman Euler method


carmen_robot_ackerman_config_t robot_config;


static void
carmen_rddf_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
			{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &robot_config.length,							 			1, NULL},
			{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &robot_config.width,								 			1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_wheels,			 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
			{(char *) "robot", 	(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &robot_config.max_v,									 		1, NULL},
			{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &robot_config.max_phi,								 		1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate,					1, NULL},
			{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient,							1, NULL}
		};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}

int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	carmen_rddf_play_get_parameters(argc, argv);

	double target_phi, distance_traveled = 0.0;
	distance_traveled = 2.0;
	double steering_acceleration[3] = {-0.25, 0.0, 0.25}; //TODO ler velocidade angular do volante do carmen.ini
	double target_v[2]   = {2.0, 0.0};

	carmen_ackerman_traj_point_t *current_state= (carmen_ackerman_traj_point_t*) malloc(sizeof(carmen_ackerman_traj_point_t));
	current_state->x = 0.0;
	current_state->y = 0.0;
	current_state->theta = 0.0;
	current_state->phi = 0.0;
	current_state->v = 0.0;

	for (int z = 0 ; z < 100 ; z++){
		for (int i = 0; i < sizeof(target_v)/sizeof(target_v[0]); ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
	//			carmen_ackerman_traj_point_t *new_state= (carmen_ackerman_traj_point_t*) malloc(sizeof(carmen_ackerman_traj_point_t));

				target_phi = carmen_clamp(-robot_config.max_phi, (current_state->phi + steering_acceleration[j]), robot_config.max_phi);

				*current_state = carmen_libcarmodel_recalc_pos_ackerman(*current_state, target_v[i], target_phi, 0.25, &distance_traveled, DELTA_T, robot_config);
				printf("%lf %lf %lf %lf %lf\n", current_state->x, current_state->y, current_state->theta, current_state->v, current_state->phi);

			}
		}
	}


	return (0);
}
