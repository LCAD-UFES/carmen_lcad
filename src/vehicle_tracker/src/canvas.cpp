#include "canvas.h"

namespace g2d
{

Circle &Canvas::circle(Field x, Field y, Field r)
{
	circles.push_back(Circle(Point(x, y), r * r));
	return circles.back();
}

Point &Canvas::point(Field x, Field y)
{
	points.push_back(Point(x, y));
	return points.back();
}

Polygon &Canvas::rectangle(Field x, Field y, Field w_2, Field h_2, Field t)
{
	Affine T = Affine(CGAL::TRANSLATION, Vector(x, y)) * Affine(CGAL::ROTATION, sin(t), cos(t));
	Point points[] = {Point(-w_2, -h_2), Point(w_2, -h_2), Point(w_2, h_2), Point(-w_2, h_2)};
	Polygon R = Polygon(points, points + 4);
	polygons.push_back(CGAL::transform(T, R));
	return polygons.back();
}

}
