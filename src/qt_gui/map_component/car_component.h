/*
 * Car_View.h
 *
 *  Created on: 20/09/2011
 *      Author: rradaelli
 */

#ifndef CAR_VIEW_H_
#define CAR_VIEW_H_

#include <carmen/carmen.h>
#include "map_component.h"
#include "../carmen/carmen_state.h"


class Car_Component : public Map_Component {
	Q_OBJECT

public:
	Car_Component(Qt::GlobalColor color);
	virtual ~Car_Component();
	void paint(QPainter *painter);
	carmen_point_t* get_pose();
	void set_color(int r, int g, int b, int a=0);

public slots:
void set_pose(carmen_point_t pose);

private:
carmen_point_t *pose;
QColor *color;
Robot_Config *robot_config;
};

#endif /* CAR_VIEW_H_ */
