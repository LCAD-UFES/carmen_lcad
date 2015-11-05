/*
 * RRT_Component.cpp
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#include "rrt_component.h"
#include <math.h>
#include "car_component.h"

RRT_Component::RRT_Component(QColor color) {
	this->color = color;
	rrt_msg.num_edges = 0;
	brect.setX(0);
	brect.setY(0);
	brect.setWidth(200);
	brect.setHeight(200);
	draw_car = false;
}

RRT_Component::~RRT_Component() {
}

void RRT_Component::paint(QPainter *painter) {
	int max_x=0, max_y=0;
	int min_x = get_heigth() * 2, min_y = get_heigth() * 2;

	Car_Component car(Qt::blue);


	QPainterPath *path = new QPainterPath();
	QPainterPath *path_nodes = new QPainterPath();
	for(int i=0; i<rrt_msg.num_edges; i++) {
		int x1, x2;
		int y1, y2;
		double theta1, theta2;

		x1 = rrt_msg.edges[i].p1.x;
		x2 = rrt_msg.edges[i].p2.x;
		theta1 = -rrt_msg.edges[i].p1.theta;
		theta2 = -rrt_msg.edges[i].p2.theta;

		y1 = get_heigth()-rrt_msg.edges[i].p1.y;
		y2 = get_heigth()-rrt_msg.edges[i].p2.y;

		path->moveTo(x1, y1);
		path->lineTo(x2, y2);

		path_nodes->moveTo(x1, y1);
		path_nodes->lineTo(x1 + cos(theta1)*0.2, y1+ sin(theta1)*0.2);

		path_nodes->moveTo(x2, y2);
		path_nodes->lineTo(x2+ cos(theta2)*0.2, y2+ sin(theta2)*0.2);

		if(draw_car) {
			painter->setOpacity(0.3);
			carmen_point_t point;
			{
				point.theta = rrt_msg.edges[i].p1.theta;
				point.x = rrt_msg.edges[i].p1.x * get_resolution();
				point.y = rrt_msg.edges[i].p1.y * get_resolution();
				car.set_pose(point);
				car.paint(painter);
			}
			{
				point.theta = rrt_msg.edges[i].p2.theta;
				point.x = rrt_msg.edges[i].p2.x * get_resolution();
				point.y = rrt_msg.edges[i].p2.y * get_resolution();
				car.set_pose(point);
				car.paint(painter);
			}
			painter->setOpacity(1);
		}




		max_x = x1>max_x?x1:max_x;
		max_x = x2>max_x?x2:max_x;
		max_y = y1>max_y?y1:max_y;
		max_y = y2>max_y?y2:max_y;

		min_x = x1<min_x?x1:min_x;
		min_x = x2<min_x?x2:min_x;
		min_y = y1<min_y?y1:min_y;
		min_y = y2<min_y?y2:min_y;
	}

	painter->setPen(color);
	painter->drawPath(*path);

	painter->setPen(Qt::red);
	painter->drawPath(*path_nodes);

	prepareGeometryChange();
	brect.setX(min_x);
	brect.setY(min_y);
	brect.setWidth(max_x - min_x);
	brect.setHeight(max_y - min_y);

	delete path;
	delete path_nodes;
}

void RRT_Component::set_rrt(carmen_rrt_planner_tree_message rrt_msg) {
	this->rrt_msg = rrt_msg;
	update();
}

void RRT_Component::set_draw_car(bool b) {
	draw_car = b;
	update();
}
