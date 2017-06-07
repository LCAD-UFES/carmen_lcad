#include "display.h"

namespace g2d
{

Display::Display(QGraphicsScene *scene):
	points(&canvas.points),
	pen(::Qt::black, 0, ::Qt::SolidLine)
{
	this->scene = scene;
	scene->addItem(&points);
}

Circle &Display::circle(Field x, Field y, Field r)
{
	Circle &C = canvas.circle(x, y, r);
	Qt::CircleGraphicsItemPtr item(new Qt::CircleGraphicsItem(&C));
	item->setPen(pen);

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

Polygon &Display::polygon(const Points &points)
{
	Polygon &R = canvas.polygon(points);
	Qt::PolygonGraphicsItemPtr item(new Qt::PolygonGraphicsItem(&R));
	item->setVerticesPen(pen);
	item->setEdgesPen(pen);

	scene->addItem(item.get());
	polygons.push_back(item);
	this->points.update();
	update();

	return R;
}

Polygon &Display::rectangle(Field x, Field y, Field w_2, Field h_2, Field t)
{
	Polygon &R = canvas.rectangle(x, y, w_2, h_2, t);
	Qt::PolygonGraphicsItemPtr item(new Qt::PolygonGraphicsItem(&R));
	item->setVerticesPen(pen);
	item->setEdgesPen(pen);

	scene->addItem(item.get());
	polygons.push_back(item);
	points.update();
	update();

	return R;
}

template<class T>
void clearItems(QGraphicsScene *scene, T &items)
{
	while (items.size() > 0)
	{
		auto item = items.back();
		scene->removeItem(item.get());
		items.pop_back();
	}
}

void Display::clear()
{
	clearItems(scene, circles);
	clearItems(scene, polygons);
	canvas.points.clear();
}

void Display::update()
{
	scene->views().at(0)->viewport()->update();
}

}
