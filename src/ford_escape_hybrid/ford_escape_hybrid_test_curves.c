#include <carmen/carmen.h>

static carmen_robot_and_trailer_traj_point_t points_vector[10000000];
carmen_robot_and_trailer_motion_command_t motion_command_vector[10000000];
static int points_vector_size = 0;
static carmen_localize_ackerman_globalpos_message globalpos;
static carmen_robot_ackerman_config_t car_config;
static double delta_time = 0.05;

static carmen_robot_and_trailer_traj_point_t
predict_new_robot_position(carmen_robot_and_trailer_traj_point_t current_robot_position, double v, double phi, double time, carmen_robot_ackerman_config_t *car_config)
{
	carmen_robot_and_trailer_traj_point_t new_robot_position;

	new_robot_position = current_robot_position;
	new_robot_position.x = new_robot_position.x + v * cos(new_robot_position.theta) * time;
	new_robot_position.y = new_robot_position.y + v * sin(new_robot_position.theta) * time;
	new_robot_position.theta = new_robot_position.theta + (v * tan(phi) / car_config->distance_between_front_and_rear_axles) * time;
	new_robot_position.v = v;
	new_robot_position.phi = phi;

	return (new_robot_position);
}

static int
build_points_vector(carmen_robot_and_trailer_traj_point_t *points_vector, int initial_index, double phi, double time, carmen_robot_ackerman_config_t *car_config)
{
	double t;
	int size = 0;
	for (t = 0; t <= time; t += delta_time, size++)
	{
		points_vector[size + initial_index] = predict_new_robot_position(points_vector[size + initial_index - 1], 3, phi, delta_time, car_config);
	}

	return size;
}

void
publish_navigator_ackerman_plan_message(carmen_robot_and_trailer_traj_point_t *points_vector, int size)
{
	carmen_navigator_ackerman_plan_message msg;
	static int		first_time = 1;
	IPC_RETURN_TYPE err;

	msg.path = points_vector;
	msg.path_length = size;
	msg.host = carmen_get_host();
	msg.timestamp = carmen_get_time();

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_NAVIGATOR_ACKERMAN_PLAN_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);
		first_time = 0;
	}

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME, &msg);
	carmen_test_ipc(err, "Could not publish",
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);
}

#include <carmen/motion_planner_messages.h>
static void
carmen_motion_planner_publish_path_message(
		carmen_robot_and_trailer_traj_point_t *path, int size, int algorithm)
{
	IPC_RETURN_TYPE err;
	static int firsttime = 1;
	static carmen_motion_planner_path_message msg;

	if(firsttime)
	{
		IPC_RETURN_TYPE err;

		err = IPC_defineMsg(CARMEN_MOTION_PLANNER_PATH_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_MOTION_PLANNER_PATH_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_MOTION_PLANNER_PATH_NAME);
		msg.host = carmen_get_host();
		firsttime = 0;
	}


	msg.timestamp = carmen_get_time();
	msg.path = path;
	msg.path_size = size;
	msg.algorithm = algorithm;

	err = IPC_publishData(CARMEN_MOTION_PLANNER_PATH_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_MOTION_PLANNER_PATH_NAME);
}

void
send_base_ackerman_command(carmen_ackerman_traj_point_t *path, int size)
{
	int i;
	for (i = 0; i < size; i++)
	{
		motion_command_vector[i].v = path[i].v;
		motion_command_vector[i].phi = path[i].phi;
		motion_command_vector[i].time = delta_time;
	}

	carmen_base_ackerman_publish_motion_command(motion_command_vector, size, carmen_get_time());
}

static void
build_points_vector_handler()
{
	static int firstime = 1;

	if (firstime)
	{
		points_vector[0].x = globalpos.globalpos.x;
		points_vector[0].y = globalpos.globalpos.y;
		points_vector[0].theta = globalpos.globalpos.theta;
		points_vector_size = 1;

		points_vector_size += build_points_vector(points_vector, points_vector_size, carmen_degrees_to_radians(10), 3.0, &car_config);
		points_vector_size += build_points_vector(points_vector, points_vector_size, carmen_degrees_to_radians(-10), 3.0, &car_config);
		points_vector_size += build_points_vector(points_vector, points_vector_size, carmen_degrees_to_radians(0), 3.0, &car_config);

		printf("enviando %d, pose %f %f %f\n", points_vector_size, globalpos.globalpos.x, globalpos.globalpos.y, globalpos.globalpos.theta);

		carmen_motion_planner_publish_path_message(points_vector, points_vector_size, 0);

//		publish_navigator_ackerman_plan_message(points_vector, points_vector_size);
//		send_base_ackerman_command(points_vector, points_vector_size);
		carmen_ipc_sleep(0.5);
		firstime = 0;
	}

}

static void
read_parameters(int argc, char **argv, carmen_robot_ackerman_config_t *car_config)
{
	carmen_param_t param_list[] = {
			{"robot",	"length",								  		CARMEN_PARAM_DOUBLE, &car_config->length,								 			1, NULL},
			{"robot",	"width",								  		CARMEN_PARAM_DOUBLE, &car_config->width,								 			1, NULL},
			{"robot", 	"distance_between_rear_wheels",		  			CARMEN_PARAM_DOUBLE, &car_config->distance_between_rear_wheels,			 		1, NULL},
			{"robot", 	"distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &car_config->distance_between_front_and_rear_axles, 			1, NULL},
			{"robot", 	"distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &car_config->distance_between_front_car_and_front_wheels,	1, NULL},
			{"robot", 	"distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &car_config->distance_between_rear_car_and_rear_wheels,		1, NULL},
			{"robot", 	"max_velocity",						  			CARMEN_PARAM_DOUBLE, &car_config->max_v,								 		1, NULL},
			{"robot", 	"max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &car_config->max_phi,								 		1, NULL},
			{"robot", 	"maximum_acceleration_forward",					CARMEN_PARAM_DOUBLE, &car_config->maximum_acceleration_forward,					1, NULL},
			{"robot", 	"maximum_acceleration_reverse",					CARMEN_PARAM_DOUBLE, &car_config->maximum_acceleration_reverse,					1, NULL},
			{"robot", 	"maximum_deceleration_forward",					CARMEN_PARAM_DOUBLE, &car_config->maximum_deceleration_forward,					1, NULL},
			{"robot", 	"maximum_deceleration_reverse",					CARMEN_PARAM_DOUBLE, &car_config->maximum_deceleration_reverse,					1, NULL},
			{"robot", 	"understeer_coeficient",						CARMEN_PARAM_DOUBLE, &car_config->understeer_coeficient,							1, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv, &car_config);


	carmen_localize_ackerman_subscribe_globalpos_message(&globalpos, NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_addPeriodicTimer(0.1, (TIMER_HANDLER_TYPE) build_points_vector_handler, NULL);

	carmen_ipc_dispatch();

	return 0;
}




