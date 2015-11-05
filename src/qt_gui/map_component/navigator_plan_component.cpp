/*
 * navigator_plan_component.cpp
 *
 *  Created on: 13/10/2011
 *      Author: rradaelli
 */

#include "navigator_plan_component.h"
#include "car_component.h"
#include <QPainter>

Navigator_Plan_Component::Navigator_Plan_Component()
: Map_Component() {
	plan.path_length = 0;

	robot_config = Carmen_State::get_instance()->robot_config;
}

void Navigator_Plan_Component::paint(QPainter *painter) {

	Car_Component *car = new Car_Component(Qt::yellow);

	for(int i=0; i<plan.path_length-1; i++) {
		if(plan.path[i].v<0) {
			painter->setPen(Qt::red);
		} else {
			painter->setPen(Qt::blue);
		}

		painter->drawLine(plan.path[i].x/get_resolution(), (get_heigth()-plan.path[i].y/get_resolution()),
				plan.path[i+1].x/get_resolution(), (get_heigth()-plan.path[i+1].y/get_resolution()));
	}

	if(waypoint_visible) {
		for(int i=0; i<plan.path_length; i++) {
			if(plan.path[i].v<0) {
				car->set_color(255, 0, 0, 0.2*255);
			} else {
				car->set_color(89, 0, 107, 0.2*255);
			}

			carmen_point_t p;
			p.theta = plan.path[i].theta;
			p.x = plan.path[i].x;
			p.y = plan.path[i].y;
			car->set_pose(p);
			car->paint(painter);

		}
	}
}

void Navigator_Plan_Component::set_waypoint_visible(bool visible) {
	waypoint_visible = visible;
}

void Navigator_Plan_Component::set_plan(carmen_navigator_ackerman_plan_message plan) {
	this->plan = plan;
	double min_x, max_x, min_y, max_y;
	min_x = min_y = DBL_MAX;
	max_x = max_y = 0;

	for(int i=0; i<plan.path_length; i++) {
		if(plan.path[i].x/get_resolution() > max_x) {
			max_x = plan.path[i].x/get_resolution();
		}

		if((get_heigth()-(plan.path[i].y/get_resolution())) > max_y) {
			max_y = get_heigth()-(plan.path[i].y/get_resolution());
		}

		if(plan.path[i].x/get_resolution() < min_x) {
			min_x = plan.path[i].x/get_resolution();
		}

		if((get_heigth()-(plan.path[i].y/get_resolution())) < min_y) {
			min_y = get_heigth()-(plan.path[i].y/get_resolution());
		}
	}

	prepareGeometryChange();

	brect.setX(min_x-robot_config->length/get_resolution());
	brect.setY(min_y-robot_config->length/get_resolution());
	brect.setWidth(max_x - min_x + robot_config->length/get_resolution()*2);
	brect.setHeight(max_y - min_y +robot_config->length/get_resolution()*2);

	update();
}

Navigator_Plan_Component::~Navigator_Plan_Component() {
}
