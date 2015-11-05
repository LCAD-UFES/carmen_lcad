/*
 * map_item.h
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#ifndef MAP_ITEM_H_
#define MAP_ITEM_H_

#include <qgraphicsitem.h>
#include <carmen/carmen.h>
#include "carmen/map_config.h"
#include <qpoint.h>
#include <qgraphicssceneevent.h>
#include "map_component/car_component.h"

class Map_Item: public QObject, public QGraphicsItem {
	Q_OBJECT
	Q_INTERFACES(QGraphicsItem)

public slots:
	void set_map(carmen_map_t carmen_map);
	void set_occupancy_grid_map(carmen_map_t carmen_map);
	void set_occupancy_grid_visible(bool b);
	void set_occupancy_grid_overlap(bool b);

public:
	typedef enum {
		PLACING_ROBOT, PLACING_GOAL
	} MAP_EVENT;

	Map_Item();
	virtual ~Map_Item();
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
	QRectF boundingRect() const;

private:
	void create_map_pixmap(carmen_map_t carmen_map, QPixmap **pixmap_map);

protected:
	void mousePressEvent ( QGraphicsSceneMouseEvent * event );
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent ( QGraphicsSceneMouseEvent * event );

private:
	QPixmap *pixmap_map;
	QPixmap *pixmap_occupancy_grid;
	QRectF brect;
	Map_Config *map_config;
	double fator;
	double altura;
	bool occupancy_grid_visibility;
	bool occupancy_grid_overlap;
	QPointF last_mouse_click;
	int map_event;

public:
	Car_Component *temporary_car;
};

#endif /* MAP_ITEM_H_ */
