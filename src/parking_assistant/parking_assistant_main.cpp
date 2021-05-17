/*********************************************************
 ---   Parking Assistant Module ---
 **********************************************************/

#include <carmen/carmen.h>
#include <carmen/parking_assistant_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/global_graphics.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/playback_interface.h>
#include <carmen/navigator_gui2_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/voice_interface_interface.h>

#include "message_interpolation.cpp"

#define OFFSET_X_CAR_POSE		-0.8//importante: esse parametro tem relacao com o tempo de resposta do motorista ao primeiro sinal de pare.
#define OFFSET_Y_CAR_POSE		0.4
#define DISTANCE_NEIGHBOR_CAR	1.531

MessageInterpolation<carmen_localize_ackerman_globalpos_message, carmen_mapper_map_message> interpolator(1); //3

static carmen_robot_ackerman_config_t car_config;

static int is_first_pose_of_parking_space;
static carmen_point_t first_pose_parking_space;
static carmen_point_t last_pose_parking_space;
double parking_space;
carmen_parking_assistant_goal_message goal;
static int hit;
static carmen_map_p carmen_map = (carmen_map_t*) malloc(sizeof(carmen_map_t));
static int goal_is_set = 0;
static double robot_max_steering_angle = 0;
static int step = 1;
static carmen_point_t path[3];

void parking_assistant_initialize()
{
	first_pose_parking_space.x = 0;
	first_pose_parking_space.y = 0;
	first_pose_parking_space.theta = 0;

	last_pose_parking_space.x = 0;
	last_pose_parking_space.y = 0;
	last_pose_parking_space.theta = 0;

	is_first_pose_of_parking_space = 1;
	hit = 1;

	goal_is_set = 0;
}

void navigator_set_goal(double x, double y, double theta)
{
	carmen_verbose("Set goal to %.1f %.1f\n", x, y);
	carmen_navigator_ackerman_set_goal(x, y, theta);
	IPC_listen(50);
}

static void carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	interpolator.AddMessageToInterpolationList(globalpos_message);
}

static int found_possible_parking_space(carmen_point_t car_pose, carmen_map_p map, carmen_robot_ackerman_config_t *car_config)
{
	car_pose.x = car_pose.x - (OFFSET_X_CAR_POSE + car_config->length);
	car_pose.y = car_pose.y - (OFFSET_Y_CAR_POSE + car_config->distance_between_rearview);
	hit = pose_hit_obstacle_ultrasonic(car_pose, map, car_config);

	//print the current map (jpeg)
	/*
	 static int count = 1;
	 char mapname[256];
	 sprintf(mapname, "map%d.jpg", count);
	 printf("saving map...\n");
	 carmen_graphics_write_map_as_jpeg(mapname,carmen_map, 0);
	 printf("saved map...\n");
	 count++;
	 */

	return !hit;
}

/*********************************************************
 --- Publishers ---
 **********************************************************/

void publish_carmen_parking_assistant_goal_message(carmen_parking_assistant_goal_message *message)
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_PARKING_ASSISTANT_GOAL_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_PARKING_ASSISTANT_GOAL_FMT);
}

void publish_carmen_parking_assistant_parking_space_message(carmen_parking_assistant_parking_space_message *message)
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_PARKING_ASSISTANT_PARKING_SPACE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_PARKING_ASSISTANT_PARKING_SPACE_FMT);
}

void publish_carmen_selector_state_message(carmen_behavior_selector_state_message *message)
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT);
}

static void found_parking_space(carmen_localize_ackerman_globalpos_message car)
{

	goal.pose = first_pose_parking_space;
	//goal.pose.x = first_pose_parking_space.x + ((last_pose_parking_space.x-first_pose_parking_space.x) / 2.0);
	goal.timestamp = car.timestamp;
	goal.host = carmen_get_host();
	publish_carmen_parking_assistant_goal_message(&goal);

	goal_is_set = 1;

	path[0].x = car.globalpos.x;
	path[0].y = car.globalpos.y;
	//printf("path[0].x: %lf\n",path[0].x);
	//printf("path[0].y: %lf\n",path[0].y);

	path[2].x = goal.pose.x;
	path[2].y = goal.pose.y;
	path[2].theta = goal.pose.theta;
	//printf("path[2].x: %lf\n",path[2].x);
	//printf("path[2].y: %lf\n",path[2].y);

	double r = car_config.distance_between_front_and_rear_axles / tan(robot_max_steering_angle);
	//printf("r: %lf\n",r);

	carmen_point_t c1, c2;

	c1.x = path[0].x;
	c1.y = path[0].y - r;

	c2.x = path[2].x;
	c2.y = path[2].y + r;

	//printf("c1.x: %lf\n",c1.x);
	//printf("c1.y: %lf\n",c1.y);
	//printf("c2.x: %lf\n",c2.x);
	//printf("c2.y: %lf\n",c2.y);

	path[1].x = (c1.x + c2.x) / 2.0;
	path[1].y = (c2.y + c1.y) / 2.0;
	path[1].theta = 0.785398163;

	//printf("path[1].x: %lf\n",path[1].x);
	//printf("path[1].y: %lf\n",path[1].y);

	//double d = cos(0.785398163)*r*2.0;

	//printf("distancia da rota no eixo x: %lf\n",path[0].x - path[2].x);
	//printf("distancia ideal: %lf\n",d);

	carmen_parking_assistant_parking_space_message parking_space_msg;
	parking_space_msg.size = parking_space;
	parking_space_msg.p1 = path[0];
	parking_space_msg.p2 = path[2];
	parking_space_msg.p3 = path[1];
	parking_space_msg.c1 = c1;
	parking_space_msg.c2 = c2;
	parking_space_msg.r = r;
	parking_space_msg.timestamp = car.timestamp;
	parking_space_msg.host = carmen_get_host();
	publish_carmen_parking_assistant_parking_space_message(&parking_space_msg);
	/*
	 carmen_behavior_selector_state_message state;
	 state.goal_source = CARMEN_BEHAVIOR_SELECTOR_USER_GOAL;
	 state.algorithm = CARMEN_BEHAVIOR_SELECTOR_A_STAR;
	 state.state = BEHAVIOR_SELECTOR_PARKING;
	 state.parking_algorithm = CARMEN_BEHAVIOR_SELECTOR_A_STAR;
	 state.following_lane_algorithm = CARMEN_BEHAVIOR_SELECTOR_RRT;
	 state.timestamp = carmen_get_time();
	 state.host = carmen_get_host();
	 publish_carmen_selector_state_message(&state);

	 navigator_set_goal(goal.pose.x, goal.pose.y, goal.pose.theta);*/

}

static int step_point_reached(carmen_point_t p, carmen_point_t step_point)
{
	if (fabs(p.x - step_point.x) < 0.4 && fabs(p.y - step_point.y) < 0.4 && fabs(p.theta - step_point.theta) <= 0.174532925)	//10 degrees
		return 1;
	return 0;
}

/*********************************************************
 --- Handlers ---
 **********************************************************/

static void carmen_mapper_map_message_handler(carmen_mapper_map_message *message)
{

	carmen_localize_ackerman_globalpos_message car;
	car = interpolator.InterpolateMessages(message);

	if (!goal_is_set)
	{

		double safe_space_for_parking, max_parking_space;
		safe_space_for_parking = 5.90;	//car_config.length + 2.0*DISTANCE_NEIGHBOR_CAR;
		max_parking_space = 6.10;	//car_config.length + 2.3*DISTANCE_NEIGHBOR_CAR; //o log nao permite usar o tamanho ideal de 7.499 m

		carmen_map->map = (double**) calloc(message->config.x_size, sizeof(double*));

		carmen_map->complete_map = message->complete_map;
		carmen_map->config = message->config;

		carmen_test_alloc(carmen_map->map);
		for (int x_index = 0; x_index < carmen_map->config.x_size; x_index++)
			carmen_map->map[x_index] = carmen_map->complete_map + x_index * carmen_map->config.y_size;

		if (found_possible_parking_space(car.globalpos, carmen_map, &car_config))
		{
			if (is_first_pose_of_parking_space)
			{
				first_pose_parking_space.x = car.globalpos.x - (OFFSET_X_CAR_POSE + car_config.length);
				first_pose_parking_space.y = car.globalpos.y - (OFFSET_Y_CAR_POSE + car_config.distance_between_rearview);
				first_pose_parking_space.theta = car.globalpos.theta;
				is_first_pose_of_parking_space = 0;

				last_pose_parking_space.x = car.globalpos.x - (OFFSET_X_CAR_POSE + car_config.length);
				last_pose_parking_space.y = car.globalpos.y - (OFFSET_Y_CAR_POSE + car_config.distance_between_rearview);
				last_pose_parking_space.theta = car.globalpos.theta;
			}
			else
			{
				last_pose_parking_space.x = car.globalpos.x - (OFFSET_X_CAR_POSE + car_config.length);
				last_pose_parking_space.y = car.globalpos.y - (OFFSET_Y_CAR_POSE + car_config.distance_between_rearview);
				last_pose_parking_space.theta = car.globalpos.theta;
			}

			parking_space = last_pose_parking_space.x - first_pose_parking_space.x + car_config.length;

			//printf("! parking space: %lf\nmax space: %lf\n",parking_space,max_parking_space);
			if (parking_space >= max_parking_space)
			{
				printf("#### tamanho da vaga mapeada: %lf\n", parking_space);
				found_parking_space(car); //goal_is_set = 1;
			}

		}
		else
		{

			parking_space = last_pose_parking_space.x - first_pose_parking_space.x + car_config.length;

			//printf("! parking space: %lf\nsafe space: %lf\n",parking_space,safe_space_for_parking);
			if (parking_space >= safe_space_for_parking)
			{
				printf("#### parking space: %lf\nsafe space: %lf\n", parking_space, safe_space_for_parking);
				found_parking_space(car); //goal_is_set = 1;
			}
			else
			{
				parking_assistant_initialize();
			}
		}
	}
	else //if goal is set
	{
		if (!step)
		{
			return; //goal reached
		}

		switch (step)
		{
		case 1:
			printf("(1) Pare.\n\n");
			//carmen_voice_send_alert((char *) "Pare");
			system("espeak -v pt-br 'Pare' 2>/dev/null");
			step = 2;
			break;
		case 2:
			if (car.v < 0.001 && step_point_reached(car.globalpos, path[0]))
			{
				printf("(2) Gire todo o volante para direita.\n\n");
				//carmen_voice_send_alert((char *) "Todo o volante para direita");
				system("espeak -v pt-br 'Gire todo o volante para direita' 2>/dev/null");
				step = 3;
			}
			break;
		case 3:
			if (DIST2D(car.globalpos,path[1]) < 0.4)
			{
				printf("(3) Pare.\n\n");
				//carmen_voice_send_alert((char *) "Pare");
				system("espeak -v pt-br 'Pare' 2>/dev/null");
				step = 4;
			}
			break;
		case 4:
			if (car.v < 0.001 && step_point_reached(car.globalpos, path[1]))
			{
				printf("(4) Gire todo o volante para esquerda.\n\n");
				//carmen_voice_send_alert((char *) "Todo o volante para esquerda");
				system("espeak -v pt-br 'Gire todo o volante para esquerda' 2>/dev/null");
				step = 5;
			}
			break;
		case 5:
			if (fabs(car.globalpos.x - path[2].x) < 0.4)
			{
				printf("(5) Pare.\n\n");
				//carmen_voice_send_alert((char *) "Pare");
				system("espeak -v pt-br 'Pare' 2>/dev/null");
				step = 6;
			}
			break;
		case 6:
			if (car.v < 0.001)
			{ // && DIST2D(car.globalpos,path[2]) < 0.8 && fabs(car.globalpos.theta-path[2].theta) < 0.0872664626) {
				//carmen_voice_send_alert((char *) "Agora é com você.");
				system("espeak -v pt-br 'Agora é com você.' 2>/dev/null");
				step = 0;
			}
			break;

		}

	}

}

/*
 static void
 carmen_ultrasonic_sensor_message_handler(carmen_ultrasonic_sonar_sensor_message *message)
 {

 }
 */

void shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("parking_assistant: disconnected.\n");
		exit(0);
	}
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = { { (char*) "robot", (char*) "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE,
			&(car_config.distance_between_front_car_and_front_wheels), 1, NULL }, { (char*) "robot", (char*) "distance_between_front_and_rear_axles",
			CARMEN_PARAM_DOUBLE, &(car_config.distance_between_front_and_rear_axles), 1, NULL }, { (char*) "robot",
			(char*) "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rear_car_and_rear_wheels), 1, NULL }, {
			(char*) "robot", (char*) "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rear_wheels), 1, NULL }, {
			(char*) "robot", (char*) "distance_between_rearview", CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rearview), 1, NULL }, { (char*) "robot",
			(char*) "max_steering_angle", CARMEN_PARAM_DOUBLE, &(robot_max_steering_angle), 0, NULL }, { (char*) "robot", (char*) "length", CARMEN_PARAM_DOUBLE,
			&(car_config.length), 0, NULL }, { (char*) "robot", (char*) "width", CARMEN_PARAM_DOUBLE, &(car_config.width), 0, NULL }

	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

static void carmen_subscribe_to_relevant_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) carmen_mapper_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
}

/*
 static void
 carmen_subscribe_to_ultrasonic_relevant_messages()
 {
 carmen_ultrasonic_sonar_sensor_subscribe(NULL, (carmen_handler_t) carmen_ultrasonic_sensor_message_handler, CARMEN_SUBSCRIBE_LATEST);
 }
 */

int main(int argc, char **argv)
{

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Read application specific parameters (Optional) */
	read_parameters(argc, argv);

	//Initialize relevant variables
	parking_assistant_initialize();

	carmen_subscribe_to_relevant_messages();

	/* Define published messages by your module */
	carmen_parking_assistant_define_messages();
	/*
	 carmen_behavior_selector_state_message state;
	 state.goal_source = CARMEN_BEHAVIOR_SELECTOR_USER_GOAL;
	 state.algorithm = CARMEN_BEHAVIOR_SELECTOR_A_STAR;
	 state.state = BEHAVIOR_SELECTOR_PARKING;
	 state.parking_algorithm = CARMEN_BEHAVIOR_SELECTOR_A_STAR;
	 state.following_lane_algorithm = CARMEN_BEHAVIOR_SELECTOR_RRT;
	 state.timestamp = carmen_get_time();
	 state.host = carmen_get_host();

	 publish_carmen_selector_state_message(&state);

	 */
	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	carmen_ipc_disconnect();
	return 0;
}
