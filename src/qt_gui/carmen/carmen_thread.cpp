/*
 * Carmen_Thread.cpp
 *
 *  Created on: 16/09/2011
 *      Author: rradaelli
 */
#include <time.h>
#include "carmen_thread.h"
#include <stdio.h>
#include <carmen/grid_mapping_interface.h>
#include <carmen/grid_mapping_messages.h>
#include <carmen/global_graphics_qt.h>
#include "map_config.h"
#include "robot_config.h"
#include "carmen_state.h"

Carmen_Thread* singleton = NULL;
carmen_base_ackerman_velocity_message velocity_message;
carmen_localize_globalpos_message globalpos_diff;
carmen_simulator_truepos_message truepos_diff;

void send_command(double v, double phi) {
	velocity_message.v = v;
	velocity_message.phi = phi;
	velocity_message.host = carmen_get_host();
	velocity_message.timestamp = carmen_get_time();

	IPC_publishData(CARMEN_BASE_ACKERMAN_VELOCITY_NAME, &velocity_message);
}

Carmen_Thread* Carmen_Thread::getInstance(int argc, char **argv) {
	if(singleton == NULL) {
		singleton = new Carmen_Thread(argc, argv);
	}

	return singleton;
}

Carmen_Thread* Carmen_Thread::getInstance() {
	return singleton;
}

Carmen_Thread::Carmen_Thread(int argc, char **argv) {
	this->argc = argc;
	this->argv = argv;

	if(singleton == NULL) {
		singleton = this;
	}

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);
	declare_messages();
	register_meta_types();
}

void Carmen_Thread::run() {
	carmen_ipc_dispatch();
}

int Carmen_Thread::updateIPC(int i)
{
	carmen_ipc_sleep(0.01);
	carmen_graphics_update_ipc_callbacks_qt(this, SLOT(updateIPC(int)));
	i = 1; // only to make the compiler happy

	return i;
}

void Carmen_Thread::set_simulator_position(carmen_point_t pose) {
	carmen_simulator_ackerman_set_truepose(&pose);
	carmen_simulator_set_truepose(&pose);
}

void Carmen_Thread::set_localize_position(carmen_point_t pose) {
	carmen_point_t std = (carmen_point_t) {0.2, 0.2, carmen_degrees_to_radians(0.0)/*, -pose.theta*/};

	carmen_localize_ackerman_initialize_gaussian_command(pose, std);
	carmen_localize_initialize_gaussian_command(pose, std);
}

void Carmen_Thread::set_goal_position(carmen_point_t goal) {
	carmen_rrt_planner_set_goal(goal);
	carmen_navigator_ackerman_set_goal(goal.x, goal.y, goal.theta);
	carmen_navigator_set_goal(goal.x, goal.y);
}

void Carmen_Thread::send_robot_command(double dist, double theta) {
	dist /=140;

	send_command(dist, -theta);
}

void Carmen_Thread::go() {
	carmen_rrt_planner_go();
	carmen_navigator_ackerman_go();
	carmen_navigator_go();
}

void Carmen_Thread::stop() {
	carmen_rrt_planner_stop();
	carmen_navigator_ackerman_stop();
	carmen_navigator_stop();
}

void Carmen_Thread::start_global_localization() {
	carmen_localize_ackerman_initialize_uniform_command();
}

Carmen_Thread::~Carmen_Thread() {
	carmen_ipc_disconnect();
}

void Carmen_Thread::register_meta_types() {
	qRegisterMetaType<carmen_rrt_planner_tree_message>("carmen_rrt_planner_tree_message");
	qRegisterMetaType<carmen_map_t>("carmen_map_t");
	qRegisterMetaType<carmen_point_t>("carmen_point_t");
	qRegisterMetaType<carmen_localize_ackerman_particle_message>("carmen_localize_ackerman_particle_message");
	qRegisterMetaType<carmen_laser_laser_message>("carmen_laser_laser_message");
	qRegisterMetaType<carmen_localize_ackerman_sensor_message>("carmen_localize_ackerman_sensor_message");
	qRegisterMetaType<carmen_navigator_ackerman_plan_message>("carmen_navigator_ackerman_plan_message");
	qRegisterMetaType<carmen_navigator_ackerman_status_message>("carmen_navigator_ackerman_status_message");
	qRegisterMetaType<carmen_navigator_ackerman_status_message>("carmen_rrt_planner_status_message");
}

/*
 * ---HANDLERS---
 */

void rrt_planner_robot_tree_handler(carmen_rrt_planner_tree_message *message) {
	singleton->emit_rrt_planner_robot_tree_signal(*message);
}

void rrt_planner_goal_tree_handler(carmen_rrt_planner_tree_message *message) {
	singleton->emit_rrt_planner_goal_tree_signal(*message);
}

void rrt_status_handler(carmen_rrt_planner_status_message *msg) {
	singleton->emit_rrt_planner_status_signal(*msg);
}

void plan_handler(carmen_navigator_ackerman_plan_message *msg) {
	singleton->emit_plan_changed_signal(*msg);
}

void navigator_status_handler(carmen_navigator_ackerman_status_message *msg) {
	singleton->emit_navigator_status_signal(*msg);
}

void map_handler(carmen_map_t *new_map) {
	if(strcmp(new_map->config.map_name, "GridSlam Map")==0) {
		singleton->emit_occupancy_grid_changed_signal(*new_map);
	} else {
		singleton->set_map(new_map);
		singleton->emit_map_changed_signal(*new_map);
	}
}

void grid_map_handler(carmen_grid_mapping_message *new_grid_map) {
	carmen_map_t m;
	m.map = NULL;

	m.complete_map = new_grid_map->complete_map;
	m.config = new_grid_map->config;
	singleton->emit_occupancy_grid_changed_signal(m);
}

void globalpos_handler(carmen_localize_ackerman_globalpos_message *msg) {
	singleton->emit_globalpos_changed_signal(*msg);
}

void globalpos_handler_diff(carmen_localize_globalpos_message *msg) {
	singleton->emit_globalpos_changed_signal(*msg);
}

void truepos_handler(carmen_simulator_ackerman_truepos_message *msg) {
	singleton->emit_truepos_changed_signal(*msg);
}

void truepos_handler_diff(carmen_simulator_truepos_message *msg) {
	singleton->emit_truepos_changed_signal(*msg);
}

void particle_handler(carmen_localize_ackerman_particle_message *msg) {
	singleton->emit_particle_changed_signal(*msg);
}

void laser_handler(carmen_laser_laser_message *msg) {
	singleton->emit_laser_changed_signal(*msg);
}

void localize_laser_handler(carmen_localize_ackerman_sensor_message *msg) {
	singleton->emit_localize_laser_changed_signal(*msg);
}

/**
 * END HANDLERS
 */

void Carmen_Thread::declare_messages() {
	IPC_RETURN_TYPE err;


	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_VELOCITY_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_BASE_ACKERMAN_VELOCITY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ACKERMAN_VELOCITY_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);

	carmen_laser_define_laser_message(1);

	carmen_grid_mapping_define_messages();

	carmen_rrt_planner_define_robot_tree_message();

	carmen_rrt_planner_define_status_message();
}

void Carmen_Thread::register_handlers() {

	carmen_map_t map;


	carmen_map_get_gridmap(&map);
	set_map(&map);

	emit_map_changed_signal(map);

	carmen_map_subscribe_gridmap_update_message(
			NULL,
			(carmen_handler_t)map_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_grid_mapping_subscribe_message(NULL,
			(carmen_handler_t)grid_map_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(
			NULL,
			(carmen_handler_t)globalpos_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_simulator_ackerman_subscribe_truepos_message(
			NULL,
			(carmen_handler_t)truepos_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_simulator_subscribe_truepos_message(
			NULL,
			(carmen_handler_t)truepos_handler_diff,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_particle_message(
			NULL,
			(carmen_handler_t)particle_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_subscribe_globalpos_message(
			NULL,
			(carmen_handler_t)globalpos_handler_diff,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_laser_subscribe_frontlaser_message(
			NULL,
			(carmen_handler_t)laser_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_sensor_message(
			NULL,
			(carmen_handler_t)localize_laser_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_ackerman_subscribe_plan_message(
			NULL,
			(carmen_handler_t)plan_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_subscribe_plan_message(
				NULL,
				(carmen_handler_t)plan_handler,
				CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_ackerman_subscribe_status_message(
			NULL,
			(carmen_handler_t) navigator_status_handler,
			CARMEN_SUBSCRIBE_LATEST);

	IPC_setMsgQueueLength(CARMEN_GRID_MAPPING_MESSAGE_NAME, 1);

	carmen_rrt_planner_subscribe_robot_tree_message(
			NULL,
			(carmen_handler_t)rrt_planner_robot_tree_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_rrt_planner_subscribe_goal_tree_message(
			NULL,
			(carmen_handler_t)rrt_planner_goal_tree_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_rrt_planner_subscribe_status_message(
			NULL,
			(carmen_handler_t)rrt_status_handler,
			CARMEN_SUBSCRIBE_LATEST);
}

void Carmen_Thread::read_parameters(int argc, char *argv[]) {
	Robot_Config* robot_config = Carmen_State::get_instance()->robot_config;
	int num_items;
	carmen_param_t param_list[] = {
			{(char*)"robot", (char*)"length", CARMEN_PARAM_DOUBLE, &robot_config->length, 1, NULL},
			{(char*)"robot", (char*)"width", CARMEN_PARAM_DOUBLE, &robot_config->width, 1, NULL},
			{(char*)"robot", (char*)"distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config->distance_between_rear_wheels, 1,NULL},
			{(char*)"robot", (char*)"distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config->distance_between_front_and_rear_axles, 1, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);

	carmen_param_install_params(argc, argv, param_list, num_items);

}

void Carmen_Thread::set_map(carmen_map_p new_map) {
	Map_Config* map_config = Carmen_State::get_instance()->map_config;

	map_config->map_name = new_map->config.map_name;
	map_config->resolution = new_map->config.resolution;
	map_config->x_size = new_map->config.x_size;
	map_config->y_size = new_map->config.y_size;
}
