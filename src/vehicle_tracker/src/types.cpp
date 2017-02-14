#include "types.h"

namespace g2d
{

Point centroid(Polygon polygon)
{
	Vertices begin = polygon.vertices_circulator();
	Vertices curr = begin;
	Vertices next = curr;
	++next;

	Field total = 0.0;
	Vector center(0, 0);

	do
	{
		Field a = curr->x() * next->y() - next->x() * curr->y();
		center += a * ((*curr - CGAL::ORIGIN) + (*next - CGAL::ORIGIN));
		total += a;
		curr = next;
		++next;
	}
	while (curr != begin);

	center /= (3 * total);
	return CGAL::ORIGIN + center;
}

Field distance(const Point &a, const Point &b)
{
	return CGAL::kth_root(2, distance2(a, b));
}

Field distance2(const Point &a, const Point &b)
{
	Vector d = a - b;
	return d.squared_length();
}

}
