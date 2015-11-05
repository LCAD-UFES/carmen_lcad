/*
 * navigator_plan_component.h
 *
 *  Created on: 13/10/2011
 *      Author: rradaelli
 */

#ifndef NAVIGATOR_PLAN_COMPONENT_H_
#define NAVIGATOR_PLAN_COMPONENT_H_

#include <carmen/carmen.h>
#include "map_component.h"
#include "../carmen/robot_config.h"

class Navigator_Plan_Component : public Map_Component {
	Q_OBJECT
public:
	Navigator_Plan_Component();
	virtual ~Navigator_Plan_Component();
	void paint(QPainter *painter);

public slots:
	void set_plan(carmen_navigator_ackerman_plan_message plan);
	void set_waypoint_visible(bool visible);

private:
	carmen_navigator_ackerman_plan_message plan;
	bool waypoint_visible;
	Robot_Config *robot_config;
};

#endif /* NAVIGATOR_PLAN_COMPONENT_H_ */
