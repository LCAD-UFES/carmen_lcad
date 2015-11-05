/*
 * RRT_Component.h
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#ifndef RRT_COMPONENT_H_
#define RRT_COMPONENT_H_

#include "map_component.h"
#include <carmen/rrt_planner_message.h>

class RRT_Component: public Map_Component {
	Q_OBJECT
public:
	RRT_Component(QColor color = Qt::yellow);
	virtual ~RRT_Component();
	void paint(QPainter *painter);

public slots:
void set_rrt(carmen_rrt_planner_tree_message rrt_msg);
void set_draw_car(bool b);

private:
carmen_rrt_planner_tree_message rrt_msg;
QColor color;
bool draw_car;

};

#endif /* RRT_COMPONENT_H_ */
