#ifndef DISPLAY_H
#define DISPLAY_H

#include "canvas.h"
#include "types.h"

#include<boost/shared_ptr.hpp>

#include "CircleGraphicsItem.h"
#include <CGAL/Qt/PointsGraphicsItem.h>
#include <CGAL/Qt/PolygonGraphicsItem.h>

#include <QGraphicsScene>
#include <QGraphicsView>

namespace g2d
{

namespace Qt
{

typedef CGAL::Qt::CircleGraphicsItem<g2d::Kernel> CircleGraphicsItem;

typedef boost::shared_ptr<CircleGraphicsItem> CircleGraphicsItemPtr;

typedef std::vector<CircleGraphicsItemPtr> CirclesGraphicsItem;

typedef CGAL::Qt::PointsGraphicsItem<g2d::Points> PointsGraphicsItem;

typedef CGAL::Qt::PolygonGraphicsItem<g2d::Polygon> PolygonGraphicsItem;

typedef boost::shared_ptr<PolygonGraphicsItem> PolygonGraphicsItemPtr;

typedef std::vector<PolygonGraphicsItemPtr> PolygonsGraphicsItem;

}

class Display
{
public:
	/*
	 * Attributes
	 */

	Canvas canvas;

	QGraphicsScene *scene;

	Qt::CirclesGraphicsItem circles;

	Qt::PointsGraphicsItem points;

	Qt::PolygonsGraphicsItem polygons;

	/*
	 * Constructors
	 */

	Display(QGraphicsScene *scene);

	/*
	 * Methods
	 */

	Circle &circle(Field x, Field y, Field r2);

	Point &point(Field x, Field y);

	Polygon &rectangle(Field x, Field y, Field w_2, Field h_2, Field t);

	void update();
};

}

#endif // DISPLAY_H
