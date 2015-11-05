/*
 * goal_component.cpp
 *
 *  Created on: 30/01/2012
 *      Author: rradaelli
 */

#include "goal_component.h"

Goal_Component::Goal_Component() {
	car_component = new Car_Component(Qt::darkYellow);
	car_component->set_color(248, 167, 10, 255);
	goal_set = 0;

	robot_config = Carmen_State::get_instance()->robot_config;
}

void Goal_Component::set_goal(carmen_navigator_ackerman_status_message msg) {
	goal_set = msg.goal_set;
	carmen_point_t goal;
	goal.x = msg.goal.x;
	goal.y = msg.goal.y;
	goal.theta = msg.goal.theta;

	set_goal(goal);
}

void Goal_Component::set_goal(carmen_rrt_planner_status_message msg) {
	goal_set = msg.goal_set;
	set_goal(msg.goal);
}

void Goal_Component::set_goal(carmen_point_t goal) {
	car_component->set_pose(goal);

	prepareGeometryChange();

	brect.setX(goal.x/get_resolution()-robot_config->length/get_resolution());
	brect.setY((get_heigth()-(goal.y/get_resolution()))-robot_config->length/get_resolution());
	brect.setWidth(robot_config->length/get_resolution()*2);
	brect.setHeight(robot_config->length/get_resolution()*2);

	update();
}

void Goal_Component::paint(QPainter *painter) {
	if(goal_set)
	{
		car_component->paint(painter);
	}
}

Goal_Component::~Goal_Component() {
	// TODO Auto-generated destructor stub
}
