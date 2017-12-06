#ifndef VIRTUAL_SCAN_POINT_H
#define VIRTUAL_SCAN_POINT_H

#include <carmen/carmen.h>

#include <cmath>

namespace virtual_scan
{

/**
 * @brief Compute the angle of the line segment between the given point and the origin.
 */
template<class Point> double angle(const Point &p)
{
	return std::atan2(p.y, p.x);
}

/**
 * @brief Rotate the point around the origin by the given angle.
 */
template<class Point> Point rotate(const Point &p, double o)
{
	double x = p.x;
	double y = p.y;

	double cos_o = std::cos(o);
	double sin_o = std::sin(o);

	Point p_o = p;
	p_o.x = x * cos_o - y * sin_o;
	p_o.y = x * sin_o + y * cos_o;

	return p_o;
}

/**
 * @brief Shift the given point by the given 2D shift.
 */
template<class Point, class Shift> Point shift(const Point &p, const Shift &s)
{
	Point p_s = p;
	p_s.x += s.x;
	p_s.y += s.y;
	return p_s;
}

/**
 * @brief Project the given pose to the reference frame described by the given origin pose.
 */
template<class Pose, class Origin> carmen_point_t project_pose(const Pose &pose, const Origin &origin)
{
    carmen_point_t projected = {
        pose.x - origin.x,
        pose.y - origin.y,
        carmen_normalize_theta(pose.theta - origin.theta)
    };

    return rotate(projected, -origin.theta);
}

} // namespace virtual_scan

#endif
