/*
 * Car_View.cpp
 *
 *  Created on: 20/09/2011
 *      Author: rradaelli
 */

#include "car_component.h"
#include <QPainter>
#include <math.h>

Car_Component::Car_Component(Qt::GlobalColor color)
: Map_Component() {
	pose = (carmen_point_t*)malloc(sizeof(carmen_point_t));
	pose->theta = 0;
	pose->x = 0;
	pose->y = 0;
	this->color = new QColor(color);

	robot_config = Carmen_State::get_instance()->robot_config;
}

Car_Component::~Car_Component() {
	free(pose);
	delete color;
}

void Car_Component::paint(QPainter *painter) {

	painter->setPen(Qt::red);

	if(pose != NULL) {
		double length = robot_config->length/get_resolution();
		double width = robot_config->width/get_resolution();

		double x, y;
		double pose_x, pose_y;
		double delta_x, delta_y;
		double alfa;

		QPainterPath *path = new QPainterPath();

		pose_x = pose->x/get_resolution();
		pose_y = get_heigth()-(pose->y/get_resolution());

		alfa = M_PI/2 +pose->theta;

		delta_x = cos(alfa)*width/2;
		delta_y = sin(alfa)*width/2;

		x = pose_x + delta_x;
		y = pose_y - delta_y;
		path->moveTo(x, y);

		x = pose_x - delta_x;
		y = pose_y + delta_y;
		path->lineTo(x, y);

		delta_x = cos(-pose->theta)*length;
		delta_y = sin(-pose->theta)*length;
		pose_x += delta_x;
		pose_y += delta_y;

		delta_x = cos(alfa)*width/2;
		delta_y = sin(alfa)*width/2;

		x = pose_x - delta_x;
		y = pose_y + delta_y;
		path->lineTo(x, y);

		x = pose_x + delta_x;
		y = pose_y - delta_y;
		path->lineTo(x, y);

		QBrush brush(*color);
		painter->fillPath(*path, brush);

		painter->setPen(Qt::black);

		painter->drawLine(
				pose_x,
				pose_y,
				pose_x - cos(-pose->theta)*length/2,
				pose_y - sin(-pose->theta)*length/2
		);
		delete path;
	}
}

void Car_Component::set_color(int r, int g, int b, int a) {
	delete color;
	color = new QColor(r, g, b, a);
}

carmen_point_t* Car_Component::get_pose() {
	return pose;
}

void Car_Component::set_pose(carmen_point_t pose) {

	*this->pose = pose;
	prepareGeometryChange();

	this->brect.setX(pose.x/get_resolution()-robot_config->length/get_resolution());
	this->brect.setY((get_heigth()-(pose.y/get_resolution()))-robot_config->length/get_resolution());
	this->brect.setWidth(robot_config->length/get_resolution()*2);
	this->brect.setHeight(robot_config->length/get_resolution()*2);
	update();
}
