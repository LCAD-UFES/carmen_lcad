/*
 * goal_component.h
 *
 *  Created on: 30/01/2012
 *      Author: rradaelli
 */

#ifndef GOAL_COMPONENT_H_
#define GOAL_COMPONENT_H_

#include "map_component.h"
#include <carmen/carmen.h>
#include <carmen/rrt_planner_message.h>
#include "car_component.h"

class Goal_Component : public Map_Component {
	Q_OBJECT
public:
	Goal_Component();
	virtual ~Goal_Component();
	void paint(QPainter *painter);
	void set_goal(carmen_point_t goal);

public slots:
	void set_goal(carmen_navigator_ackerman_status_message msg);
	void set_goal(carmen_rrt_planner_status_message msg);

private:
	Car_Component *car_component;
	int goal_set;
	Robot_Config *robot_config;

};

#endif /* GOAL_COMPONENT_H_ */
