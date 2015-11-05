/*
 * main.c
 *
 *  Created on: 10/10/2011
 *      Author: rradaelli
 */
#include <carmen/carmen.h>
#include "dynamic_window.h"

void shutdown_module(int signal);

static int cheat;

/*HANDLERS FUNCTION*/
void globalpos_handler(carmen_localize_ackerman_globalpos_message *globalpos) {
	pose = globalpos->globalpos;
}

void truepos_handler(carmen_simulator_ackerman_truepos_message *truepos) {
	pose = truepos->truepose;
}

void map_handler(void) {

}

void set_goal_handler(carmen_navigator_ackerman_set_goal_message  *goal_pose) {
	is_target_set = 1;
	printf("foi");
	target_pose.x = goal_pose->x;
	target_pose.y = goal_pose->y;
	target_pose.theta = goal_pose->theta;
	
	print_goal(goal_pose);
}

void print_goal(carmen_navigator_ackerman_set_goal_message  *goal_pose){
      
      IPC_RETURN_TYPE err = IPC_OK;  
      static int first_time = 0;

      if(first_time) {
      err = IPC_defineMsg(
      (char*)CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME,
      IPC_VARIABLE_LENGTH,
      (char*)CARMEN_NAVIGATOR_ACKERMAN_STATUS_FMT);
      carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME);

      err = IPC_defineMsg(
      (char*)CARMEN_NAVIGATOR_STATUS_NAME,
      IPC_VARIABLE_LENGTH,
      (char*)CARMEN_NAVIGATOR_ACKERMAN_STATUS_FMT);
      carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_STATUS_NAME);
      first_time = 1;
      }

      carmen_navigator_ackerman_status_message msg;
      msg.autonomous = 1;
      msg.goal_set = 1;
     
      msg.goal.x = goal_pose->x;
      msg.goal.y = goal_pose->y;
      msg.goal.theta = goal_pose->theta;
      
      msg.host = carmen_get_host();
      msg.robot.x = 0;
      msg.robot.y = 0;
      msg.robot.theta = 0;
      msg.timestamp = carmen_get_time();

      err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME, &msg);

      carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME);

}

void odometry_handler(carmen_base_ackerman_odometry_message* msg) {
	current_state = *msg;
}

void dynamic_window_handler(void) {

	struct timeval t1, t2;
	double time_h;

	gettimeofday(&t1, 0);
	execute();
	gettimeofday(&t2, 0);

	time_h = (t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec;
	printf("tempo de execucao: %f\n", time_h);
}

/**
 * Register subscribers
 */
void register_handlers() {
	IPC_defineMsg(
			CARMEN_BASE_ACKERMAN_VELOCITY_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_BASE_ACKERMAN_VELOCITY_FMT);

	carmen_map_get_gridmap(&map);
	carmen_map_subscribe_gridmap_update_message(&map,
			(carmen_handler_t)map_handler,
			CARMEN_SUBSCRIBE_LATEST);

	if(!cheat) {
		carmen_localize_ackerman_subscribe_globalpos_message(
				NULL,
				(carmen_handler_t)globalpos_handler,
				CARMEN_SUBSCRIBE_LATEST);
	} else {
		carmen_simulator_ackerman_subscribe_truepos_message(
				NULL,
				(carmen_handler_t)truepos_handler,
				CARMEN_SUBSCRIBE_LATEST);
	}

	carmen_base_ackerman_subscribe_odometry_message(
			NULL,
			(carmen_handler_t)odometry_handler,
			CARMEN_SUBSCRIBE_LATEST);

// 	carmen_navigator_ackerman_subscribe_status_message(
// 			NULL,
// 			(carmen_handler_t)navigator_status_handler,
// 			CARMEN_SUBSCRIBE_LATEST);
	
	carmen_subscribe_message(
                       (char*)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
                       (char*)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
                       NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
                       (carmen_handler_t)set_goal_handler,
                       CARMEN_SUBSCRIBE_LATEST);
	
	
}

void read_parameters(int argc, char** argv) {
	int num_items;

	carmen_param_t param_list[]= {
			{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &distance_between_rear_wheels, 1,NULL},
			{"robot", "distance_between_front_and_rear_axles",CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 1, NULL},
			{"robot", "acceleration", CARMEN_PARAM_DOUBLE, &acceleration,1,NULL},
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &max_phi, 1, NULL},
			{"robot", "deceleration", CARMEN_PARAM_DOUBLE, &deceleration, 1, NULL},
			{"robot", "length", CARMEN_PARAM_DOUBLE, &height, 1, NULL},
			{"robot", "width", CARMEN_PARAM_DOUBLE, &width, 1, NULL},
	};


	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	carmen_param_t param_list_dynamic_window[] = {
			{"dynamic_window", "interval_time", CARMEN_PARAM_DOUBLE, &interval_time, 1, NULL},
			{"dynamic_window", "alfa", CARMEN_PARAM_DOUBLE, &alfa, 1, NULL},
			{"dynamic_window", "beta", CARMEN_PARAM_DOUBLE, &beta, 1, NULL},
			{"dynamic_window", "gama", CARMEN_PARAM_DOUBLE, &gama, 1, NULL},
			{"dynamic_window", "max_distance", CARMEN_PARAM_DOUBLE, &max_distance, 1, NULL},
			{"dynamic_window", "cheat", CARMEN_PARAM_ONOFF, &cheat, 1, NULL},
			{"dynamic_window", "max_vel", CARMEN_PARAM_DOUBLE, &max_vel, 1, NULL},
			{"dynamic_window", "phi_acceleration", CARMEN_PARAM_DOUBLE, &phi_acceleration, 1, NULL}
	};

	num_items = sizeof(param_list_dynamic_window)/sizeof(param_list_dynamic_window[0]);
	carmen_param_install_params(argc, argv, param_list_dynamic_window, num_items);

	//interval_time = 0.1;
	//max_vel = 0.45;
	//ajustar peso de acordo com o mapa
	//alfa = 4.0;
	//beta = 0.8;
	//gama = 0.2;
	//max_distance = 7;
	//cheat = 1;
	//distance_between_front_and_rear_axles= 0.52;
}

int main(int argc, char** argv) {
	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	register_handlers();

	carmen_ipc_addPeriodicTimer(interval_time, (TIMER_HANDLER_TYPE)
			dynamic_window_handler, NULL);

	carmen_ipc_dispatch();

	return 0;
}

void shutdown_module(int signal) {
	if(signal){}
	signal = 0;
	carmen_ipc_disconnect();
	exit(0);
}
