/*
 * map_item.cpp
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#include "map_item.h"
#include <qpainter.h>
#include <math.h>
#include <qgraphicsscene.h>
#include "carmen/carmen_state.h"
#include "carmen/carmen_thread.h"
#include <qgraphicsview.h>

inline double min(double a, double b) {
	return a<b?a:b;
}

Map_Item::Map_Item() {
	pixmap_map = NULL;
	pixmap_occupancy_grid = NULL;

	altura = 0;
	map_event = -1;
	fator = 1;

	brect.setX(0);
	brect.setY(0);
	brect.setWidth(200);
	brect.setHeight(200);

	map_config = Carmen_State::get_instance()->map_config;

	occupancy_grid_visibility = false;
	occupancy_grid_overlap = false;
	temporary_car = new Car_Component(Qt::red);
	temporary_car->setVisible(false);
	temporary_car->set_visible(false);
	temporary_car->setZValue(999);
}

void Map_Item::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
	if(option && widget) {

	}

	if(occupancy_grid_visibility) {
		double f;
		if(occupancy_grid_overlap) {
			f = fator;
		} else {
			f = 1;
		}

		if(pixmap_occupancy_grid && !pixmap_occupancy_grid->isNull()) {
			painter->drawPixmap(0,
					altura,
					pixmap_occupancy_grid->width()/f,
					pixmap_occupancy_grid->height()/f,
					*pixmap_occupancy_grid);
		}

		if(occupancy_grid_overlap) {
			painter->setOpacity(0.5);
		}
	}
	if( ( (occupancy_grid_visibility && occupancy_grid_overlap) || !occupancy_grid_visibility ) && pixmap_map && !pixmap_map->isNull()) {
		painter->drawPixmap(0, 0, *pixmap_map);
	}
}

void Map_Item::set_map(carmen_map_t carmen_map) {
	double new_scale = 0;
	create_map_pixmap(carmen_map, &pixmap_map);

	prepareGeometryChange();
	brect.setWidth(pixmap_map->width());
	brect.setHeight(pixmap_map->height());
	this->scene()->setSceneRect(brect);

	QGraphicsView *view = (QGraphicsView *) parent();

	new_scale = fmin((float)view->size().width()/carmen_map.config.x_size, (float)view->size().height()/carmen_map.config.y_size);

	view->resetTransform();

	view->scale(new_scale, new_scale);


	update();
}

void Map_Item::set_occupancy_grid_map(carmen_map_t carmen_map) {
	create_map_pixmap(carmen_map, &pixmap_occupancy_grid);

	if(strcmp(carmen_map.config.map_name, "GridSlam Map") == 0 ) {
		altura = 0;
	} else {
		altura = (map_config->y_size - pixmap_occupancy_grid->height());
	}

	if(occupancy_grid_visibility && !occupancy_grid_overlap) {
		prepareGeometryChange();
		brect.setWidth(pixmap_occupancy_grid->width());
		brect.setHeight(pixmap_occupancy_grid->height());
		this->scene()->setSceneRect(brect);
	}

	update();
}

void Map_Item::set_occupancy_grid_visible(bool b) {
	occupancy_grid_visibility = b;
}

void Map_Item::set_occupancy_grid_overlap(bool b) {
	occupancy_grid_overlap= b;
}

void Map_Item::create_map_pixmap(carmen_map_t carmen_map, QPixmap **pixmap) {
	int cor = 0;

	if((*pixmap)==NULL || (*pixmap)->width() != carmen_map.config.x_size || (*pixmap)->height() != carmen_map.config.y_size) {
		delete (*pixmap);
		(*pixmap) = new QPixmap(carmen_map.config.x_size, carmen_map.config.y_size);
		(*pixmap)->fill(Qt::gray);
	}

	QPainter painter((*pixmap));

	fator = map_config->resolution/carmen_map.config.resolution;

	for(int i=0; i<carmen_map.config.x_size; i++) {
		for(int j=0; j<carmen_map.config.y_size; j++) {
			QColor color(35,142,104);

			if(carmen_map.complete_map[i*carmen_map.config.y_size+j]>=0) {
				color.setRgb(cor, cor, cor);
				cor = min(255-(carmen_map.complete_map[i*carmen_map.config.y_size+j]*255), 255);
			}
			painter.setPen(color);
			painter.drawPoint(i, (carmen_map.config.y_size-j));
		}
	}
}

void Map_Item::mousePressEvent ( QGraphicsSceneMouseEvent * event ) {
	last_mouse_click = event->pos();


	if(event->button() == Qt::LeftButton) {
		map_event = PLACING_ROBOT;
	} else {
		map_event = PLACING_GOAL;
	}

	temporary_car->setVisible(true);
	temporary_car->set_visible(true);
}

void Map_Item::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
	if(event) {}
	if(map_event == -1) {
		return;
	}
	double resolution = Carmen_State::get_instance()->map_config->resolution;

	carmen_point_t pose;
	pose.x = last_mouse_click.x()*resolution;
	pose.y = (pixmap_map->height() - last_mouse_click.y())*resolution;
	pose.theta = atan2(last_mouse_click.x()-event->pos().x(), last_mouse_click.y()-event->pos().y()) + M_PI/2;

	if(map_event == PLACING_ROBOT) {
		temporary_car->set_color(0, 0, 255, 150);
	}
	else {
		temporary_car->set_color(255, 165, 0, 150);
	}

	temporary_car->set_pose(pose);
}

void Map_Item::mouseReleaseEvent ( QGraphicsSceneMouseEvent * event ) {
	double resolution = Carmen_State::get_instance()->map_config->resolution;

	carmen_point_t pose;
	pose.x = last_mouse_click.x()*resolution;
	pose.y = (pixmap_map->height() - last_mouse_click.y())*resolution;
	pose.theta = atan2(last_mouse_click.x()-event->pos().x(), last_mouse_click.y()-event->pos().y()) + M_PI/2;

	if(map_event == PLACING_ROBOT) {
		Carmen_Thread::getInstance()->set_simulator_position(pose);
		Carmen_Thread::getInstance()->set_localize_position(pose);
		carmen_ipc_sleep(0.01);
		Carmen_Thread::getInstance()->set_simulator_position(pose);
		Carmen_Thread::getInstance()->set_localize_position(pose);
	} else {
		Carmen_Thread::getInstance()->set_goal_position(pose);
	}

	temporary_car->setVisible(false);
	temporary_car->set_visible(false);
	map_event = -1;
}

QRectF Map_Item::boundingRect() const {
	return brect;
}

Map_Item::~Map_Item() {
	// TODO Auto-generated destructor stub
}
