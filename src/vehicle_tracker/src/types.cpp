#include "types.h"

namespace g2d
{

Point centroid(Polygon polygon)
{
	Vertices begin = polygon.vertices_circulator();
	Vertices curr = begin;
	Vertices next = curr;
	++next;

	double total = 0.0;
	Vector center(0, 0);

	do
	{
		double a = curr->x() * next->y() - next->x() * curr->y();
		center += a * ((*curr - CGAL::ORIGIN) + (*next - CGAL::ORIGIN));
		total += a;
		curr = next;
		++next;
	}
	while (curr != begin);

	center /= (3 * total);
	return CGAL::ORIGIN + center;
}

}
