/*
 * Carmen_Thread.h
 *
 *  Created on: 16/09/2011
 *      Author: rradaelli
 */

#include <QThread>
#include <carmen/carmen.h>
#include <carmen/localize_traf_module_filter_interface.h>
#include <carmen/localize_traf_module_filter_messages.h>
#include <qmetatype.h>
#include <carmen/rrt_planner_interface.h>

#ifndef CARMENTHREAD_H_
#define CARMENTHREAD_H_

class Carmen_Thread : public QThread
{
	Q_OBJECT

private:
	int argc;
	char **argv;

	void declare_messages();

	signals:
	void map_changed(carmen_map_t map);
	void occupancy_grid_changed(carmen_map_t map);
	void globalpos_changed(carmen_point_t pos);
	void truepos_changed(carmen_point_t pos);
	void particle_changed(carmen_localize_ackerman_particle_message particles);
	void laser_changed(carmen_laser_laser_message laser);
	void localize_laser_changed(carmen_localize_ackerman_sensor_message localize_laser);
	void plan_changed(carmen_navigator_ackerman_plan_message plan_message);
	void navigator_status_changed(carmen_navigator_ackerman_status_message navigator_status);
	void rrt_planner_robot_tree_changed(carmen_rrt_planner_tree_message msg);
	void rrt_planner_goal_tree_changed(carmen_rrt_planner_tree_message msg);
	void rrt_planner_status_changed(carmen_rrt_planner_status_message msg);

public slots:
void set_simulator_position(carmen_point_t pose);
void set_localize_position(carmen_point_t pose);
void set_goal_position(carmen_point_t pose);
void start_global_localization();
void send_robot_command(double dist, double theta);
void go();
void stop();
int updateIPC(int i);

public:
Carmen_Thread(int argc, char **argv);
static Carmen_Thread* getInstance(int argc, char **argv);
static Carmen_Thread* getInstance();
virtual ~Carmen_Thread();
void run();
void set_map(carmen_map_p new_map);
void register_handlers();

void emit_map_changed_signal(carmen_map_t msg) {
	emit map_changed(msg);
}

void emit_occupancy_grid_changed_signal(carmen_map_t msg) {
	emit occupancy_grid_changed(msg);
}

void emit_globalpos_changed_signal(carmen_localize_ackerman_globalpos_message msg) {
	emit globalpos_changed(msg.globalpos);
}

void emit_globalpos_changed_signal(carmen_localize_globalpos_message globalpos_diff) {
	carmen_localize_ackerman_globalpos_message globalpos;

	globalpos.globalpos = globalpos_diff.globalpos;
	emit_globalpos_changed_signal(globalpos);
}

void emit_truepos_changed_signal(carmen_simulator_ackerman_truepos_message msg) {
	emit truepos_changed(msg.truepose);
}

void emit_truepos_changed_signal(carmen_simulator_truepos_message truepos_diff) {
	carmen_simulator_ackerman_truepos_message truepos;
	truepos.truepose = truepos_diff.truepose;
	emit truepos_changed(truepos.truepose);
}

void emit_particle_changed_signal(carmen_localize_ackerman_particle_message msg) {
	emit particle_changed(msg);
}

void emit_laser_changed_signal(carmen_laser_laser_message msg) {
	emit laser_changed(msg);
}

void emit_localize_laser_changed_signal(carmen_localize_ackerman_sensor_message msg) {
	emit localize_laser_changed(msg);
}

void emit_plan_changed_signal(carmen_navigator_ackerman_plan_message msg) {
	emit plan_changed(msg);
}
void emit_navigator_status_signal(carmen_navigator_ackerman_status_message msg) {
	emit navigator_status_changed(msg);
}

void emit_rrt_planner_robot_tree_signal(carmen_rrt_planner_tree_message msg) {
	emit rrt_planner_robot_tree_changed(msg);
}

void emit_rrt_planner_goal_tree_signal(carmen_rrt_planner_tree_message msg) {
	emit rrt_planner_goal_tree_changed(msg);
}

void emit_rrt_planner_status_signal(carmen_rrt_planner_status_message msg) {
	emit rrt_planner_status_changed(msg);
}

private:
void register_meta_types();
void read_parameters(int argc, char *argv[]);
};

#endif /* CARMENTHREAD_H_ */
