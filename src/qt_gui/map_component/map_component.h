/*
 * Map_Component.h
 *
 *  Created on: 05/10/2011
 *      Author: rradaelli
 */

#ifndef MAP_COMPONENT_H_
#define MAP_COMPONENT_H_

#include <qpainter.h>
#include <qgraphicsitem.h>

/**
 * Components that can be added and showed inside a map
 */
class Map_Component : public QObject, public QGraphicsItem {
	Q_OBJECT
	Q_INTERFACES(QGraphicsItem)

public:
	Map_Component();
	virtual ~Map_Component();
	double get_heigth();
	double get_resolution();
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
	virtual QRectF boundingRect() const;

public slots:
virtual void set_visible(bool b);


protected:
virtual void paint(QPainter *painter) = 0;
QRectF brect;
};

#endif /* MAP_COMPONENT_H_ */
