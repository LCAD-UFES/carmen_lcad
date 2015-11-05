/*
 * laser_component.h
 *
 *  Created on: 06/10/2011
 *      Author: rradaelli
 */

#ifndef LASER_COMPONENT_H_
#define LASER_COMPONENT_H_

#include <carmen/carmen.h>
#include "map_component.h"

class Laser_Component : public Map_Component {
	Q_OBJECT
public:
	Laser_Component(Qt::GlobalColor color);
	virtual ~Laser_Component();
	void paint(QPainter *painter);
	carmen_point_t get_pose();

public slots:
void set_pose(carmen_point_t pose);
void set_laser(carmen_laser_laser_message laser);
void set_laser(carmen_localize_ackerman_sensor_message laser);
void set_skip(int);

private:
carmen_point_t pose;
int num_readings;
double start_angle;
double angular_resolution;
float* range;
int skip;
Qt::GlobalColor color;
};

#endif /* LASER_COMPONENT_H_ */
