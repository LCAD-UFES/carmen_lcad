#include "point.h"

#include <cmath>

namespace virtual_scan
{


PointXY::PointXY():
	x(0),
	y(0)
{
	// Nothing to do.
}


PointXY::PointXY(double x, double y):
	x(x),
	y(y)
{
	// Nothing to do.
}


PointXY PointXY::operator + (const PointXY &that) const
{
	return PointXY(this->x + that.x, this->y + that.y);
}


double distance2(const PointXY &a, const PointXY &b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return dx * dx + dy * dy;
}


double distance(const PointXY &p)
{
	return sqrt(p.x * p.x + p.y * p.y);
}


double distance(const PointXY &a, const PointXY &b)
{
	return sqrt(distance2(a, b));
}


bool ComparatorXY::operator () (const PointXY &a, const PointXY &b) const
{
	if (a.x < b.x)
		return true;
	else if (a.x > b.x)
		return false;
	else
		return (a.y < b.y);
}


PointOD::PointOD():
	o(0),
	d(0)
{
	// Nothing to do.
}


PointOD::PointOD(double o, double d):
	o(o),
	d(d)
{
	// Nothing to do.
}


PointOD::PointOD(const PointXY &p):
	o(angle(p)),
	d(distance(p))
{
	// Nothing to do
}


bool ComparatorOD::operator () (const PointOD &a, const PointOD &b) const
{
	if (a.o < b.o)
		return true;
	else if (a.o > b.o)
		return false;
	else
		return (a.d < b.d);
}


Point2D::Point2D():
	PointXY(),
	PointOD()
{
	// Nothing to do.
}


Point2D::Point2D(const PointXY &p):
	PointXY(p),
	PointOD(p)
{
	// Nothing to do.
}


Point2D::Point2D(const PointOD &p):
	PointXY(
		std::cos(p.o) * p.d,
		std::sin(p.o) * p.d
	),
	PointOD(p)
{
	// Nothing to do.
}


Pose::Pose():
	PointXY(),
	o(0)
{
	// Nothing to do.
}


Pose::Pose(double x, double y, double o):
	PointXY(x, y),
	o(o)
{
	// Nothing to do.
}


Pose::Pose(const PointXY &position, double o):
	PointXY(position),
	o(o)
{
	// Nothing to do.
}


Pose &Pose::operator += (const Pose &that)
{
	this->x += that.x;
	this->y += that.y;
	this->o = carmen_normalize_theta(this->o + that.o);

	return *this;
}


PointXY Pose::project_global(const PointXY &point) const
{
	return shift(rotate(point, o), *this);
}


Pose Pose::project_global(const Pose &pose) const
{
	Pose global = shift(rotate(pose, o), *this);
	global.o = carmen_normalize_theta(global.o + o);
	return global;
}


PointXY Pose::project_local(const PointXY &point) const
{
	return rotate(shift(point, PointXY(-x, -y)), -o);
}


} // namespace virtual_scan
