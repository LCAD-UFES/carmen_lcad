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

bool is_inside(const g2d::Point &point, const g2d::Polygon &polygon)
{
	CGAL::Bounded_side side = CGAL::bounded_side_2(
		polygon.vertices_begin(),
		polygon.vertices_end(),
		point,
		g2d::Kernel()
	);

	return (side == CGAL::ON_BOUNDED_SIDE);
}

std::string relation(const g2d::Point &point, const g2d::Polygon &rect)
{
	CGAL::Bounded_side side = CGAL::bounded_side_2(
		rect.vertices_begin(),
		rect.vertices_end(),
		point,
		g2d::Kernel()
	);

	if (side == CGAL::ON_BOUNDED_SIDE)
		return "inside";

	if (side == CGAL::ON_BOUNDARY)
		return "on";

	g2d::Point center = g2d::centroid(rect);
	g2d::Field d_center = (center - CGAL::ORIGIN).squared_length();
	g2d::Field d_point = (point - CGAL::ORIGIN).squared_length();

	if (d_point < d_center)
		return "before";

	return "after";
}

}
