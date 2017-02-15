#include "virtual_scan.h"

VirtualScan::VirtualScan():
	center(g2d::Point(0, 0)),
	angle(0)
{
	// Nothing to do.
}

VirtualScan::VirtualScan(carmen_localize_ackerman_globalpos_message *globalpos, carmen_virtual_scan_message *virtual_scan):
	center(g2d::Point(0, 0)),
	angle(0)
{
	if (globalpos != NULL)
	{
		carmen_point_t &pose = globalpos->odometrypos;
		center = g2d::Point(pose.x, pose.y);
		angle = pose.theta;
	}

	if (virtual_scan == NULL)
		return;

	int n = virtual_scan->num_rays;
	cartesian.reserve(n);
	polar.reserve(n);

	double *t = virtual_scan->angles;
	double *d = virtual_scan->ranges;
	for (int i = 0; i < n; i++, t++, d++)
		append(*t, *d);
}

VirtualScan::VirtualScan(const VirtualScan &a, const VirtualScan &b):
	center(b.center),
	angle(b.angle)
{
	g2d::Field dx = b.center.x() - a.center.x();
	g2d::Field dy = b.center.y() - a.center.y();
	g2d::Field dt = b.angle - a.angle;

	//g2d::Affine transform = g2d::Affine(CGAL::TRANSLATION, g2d::Vector(dx, dy)) * g2d::Affine(CGAL::ROTATION, sin(dt), cos(dt));

	if (a.size() == 0 || b.size() == 0)
		return;

	std::cout << dx << ", " << dy << ", " << dt << std::endl;

	int n = std::min(a.size(), b.size());
	cartesian.reserve(n);
	polar.reserve(n);

	for (int i = 0; i < n; i++)
	{
		//g2d::Point pa = transform(a.cartesian[i]);
		const g2d::Point &pa = a.cartesian[i];
		const g2d::Point &pb = b.cartesian[i];
		if (g2d::distance2(pa, pb) > 0.5)
			append(pb);
	}
}

void VirtualScan::append(const g2d::Point &p)
{
	cartesian.push_back(p);

	g2d::Field x = p.x();
	g2d::Field y = p.y();

	g2d::Field t = atan2(y, x);
	g2d::Field d = sqrt(x*x + y*y);
	polar.push_back(g2d::Point(t, d));
}

void VirtualScan::append(g2d::Field t, g2d::Field d)
{
	polar.push_back(g2d::Point(t, d));

	g2d::Field x = cos(t) * d;
	g2d::Field y = sin(t) * d;
	cartesian.push_back(g2d::Point(x, y));
}

size_t VirtualScan::size() const
{
	return polar.size();
}
