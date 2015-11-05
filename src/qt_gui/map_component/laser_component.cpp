/*
 * laser_component.cpp
 *
 *  Created on: 06/10/2011
 *      Author: rradaelli
 */

#include "laser_component.h"
#include <QPainter>
#include <math.h>

Laser_Component::Laser_Component(Qt::GlobalColor color)
: Map_Component() {
	this->color = color;
	num_readings = 0;
	start_angle = 0;
	angular_resolution = 0;
	range = NULL;
	pose.x = pose.y = pose.theta = 0;
	skip = 20;
	set_visible(false);
}

void Laser_Component::paint(QPainter *painter) {
	double min_x, min_y, max_x, max_y;
	double x_initial = pose.x/get_resolution();
	double y_initial = (get_heigth()-(pose.y/get_resolution()));
	double x_final = 0;
	double y_final = 0;
	double laser_angle = -start_angle - pose.theta;
	double increment = -angular_resolution;

	min_x = min_y = DBL_MAX;
	max_x = max_y = 0;

	painter->setPen(color);

	for(int i=0; i<num_readings; i+=skip, laser_angle+=increment*skip) {
		double distance = range[i]/get_resolution();

		x_final = x_initial + (cos(laser_angle) * distance);
		y_final = y_initial + (sin(laser_angle) * distance);
		painter->drawLine(x_initial, y_initial, x_final, y_final);

		min_x = x_final < min_x ? x_final : min_x;
		min_y = y_final < min_y ? y_final : min_y;
		max_x = x_final > max_x ? x_final : max_x;
		max_y = y_final > max_y ? y_final : max_y;

		brect.setX(min_x - 2);
		brect.setY(min_y - 2);
		brect.setWidth(max_x - min_x);
		brect.setHeight(max_y - min_y);
		prepareGeometryChange();
	}
}

void Laser_Component::set_laser(carmen_laser_laser_message laser) {
	num_readings = laser.num_readings;
	start_angle = laser.config.start_angle;
	angular_resolution = laser.config.angular_resolution;
	range = laser.range;
	update();
}

void Laser_Component::set_laser(carmen_localize_ackerman_sensor_message laser) {
	num_readings = laser.num_readings;
	start_angle = laser.config.start_angle;
	angular_resolution = laser.config.angular_resolution;
	range = laser.range;
	update();
}

void Laser_Component::set_skip(int skip) {
	this->skip = skip;
}

void Laser_Component::set_pose(carmen_point_t pose) {
	this->pose = pose;
}

carmen_point_t Laser_Component::get_pose() {
	return pose;
}

Laser_Component::~Laser_Component() {
	// TODO Auto-generated destructor stub
}
