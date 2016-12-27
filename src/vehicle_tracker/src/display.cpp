#include "display.h"

namespace g2d
{

Display::Display(QGraphicsScene *scene):
	points(&canvas.points)
{
	this->scene = scene;
	scene->addItem(&points);
}

Circle &Display::circle(Field x, Field y, Field r2)
{
	Circle &C = canvas.circle(x, y, r2);
	Qt::CircleGraphicsItemPtr item(new Qt::CircleGraphicsItem(&C));
	scene->addItem(item.get());
	circles.push_back(item);
	update();

	return C;
}

Point &Display::point(Field x, Field y)
{
	Point &P = canvas.point(x, y);
	points.modelChanged();

	return P;
}

Polygon &Display::rectangle(Field x, Field y, Field w_2, Field h_2, Field t)
{
	Polygon &R = canvas.rectangle(x, y, w_2, h_2, t);
	Qt::PolygonGraphicsItemPtr item(new Qt::PolygonGraphicsItem(&R));
	scene->addItem(item.get());
	polygons.push_back(item);
	points.update();
	update();

	return R;
}

void Display::update()
{
	scene->views().at(0)->viewport()->update();
}

}
