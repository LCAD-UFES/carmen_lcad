#ifndef VIRTUAL_SCAN_POINT_H
#define VIRTUAL_SCAN_POINT_H

#include <carmen/carmen.h>

#include <cmath>

namespace virtual_scan
{

/**
 * @brief A 2D point represented in cartesian notation.
 */
struct PointXY
{
	/** @brief Horizontal coordinate. */
	double x;

	/** @brief Vertical coordinate. */
	double y;

	/**
	 * @brief Default constructor.
	 */
	PointXY();

	/**
	 * @brief Create a new 2D point at given coordinates.
	 */
	PointXY(double x, double y);

	/**
	 * @brief Create a new 2D point from any point-like object.
	 */
	template<class P>
	PointXY(const P &point):
		PointXY(point.x, point.y)
	{
		// Nothing to do.
	}

	/**
	 * @brief Point addition operator.
	 */
	PointXY operator + (const PointXY &that) const;
};

/**
 * @brief A comparator for cartesian points.
 *
 * This can be used in sorted containers (e.g. `std::set`) to provide the necessary
 * ordering.
 */
struct ComparatorXY
{
	bool operator () (const PointXY &a, const PointXY &b) const;
};

/**
 * @brief A 2D point represented in polar notation.
 */
struct PointOD
{
	/** @brief Angle from the horizontal axis. */
	double o;

	/** @brief Distance from the origin. */
	double d;

	/**
	 * @brief Default constructor.
	 */
	PointOD();

	/**
	 * @brief Create a new 2D point at given coordinates.
	 */
	PointOD(double o, double d);

	/**
	 * @brief Create a new polar point from a cartesian point.
	 */
	PointOD(const PointXY &p);
};

/**
 * @brief A comparator for polar points.
 *
 * This can be used in sorted containers (e.g. `std::set`) to provide the necessary
 * ordering.
 */
struct ComparatorOD
{
	bool operator () (const PointOD &a, const PointOD &b) const;
};

/**
 * @brief A 2D point combining cartesian and polar representations.
 */
struct Point2D: PointXY, PointOD
{
	/**
	 * @brief Default constructor.
	 */
	Point2D();

	/**
	 * @brief Create a new composite point from a cartesian point.
	 */
	Point2D(const PointXY &p);

	/**
	 * @brief Create a new composite point from a polar point.
	 */
	Point2D(const PointOD &p);
};

/**
 * @brief Pose in a 2D plane.
 */
struct Pose: PointXY
{
	/** @brief Angle relative to the horizontal axis. */
	double o;

	/**
	 * @brief Default constructor.
	 */
	Pose();

	/**
	 * @brief Create a new pose from the given parameters.
	 */
	Pose(double x, double y, double o);

	/**
	 * @brief Create a new pose from the given parameters.
	 */
	Pose(const PointXY &position, double o = 0.0);

	/**
	 * @brief Implementation of the increment operator.
	 */
	Pose &operator += (const Pose &that);

	/**
	 * @brief Project the given cartesian point (assumed relative to this pose) to the global reference frame.
	 */
	PointXY project_global(const PointXY &point) const;

	/**
	 * @brief Project the given pose (assumed relative to this pose) to the global reference frame.
	 */
	Pose project_global(const Pose &pose) const;

	/**
	 * @brief Project the given cartesian point (assumed global) to this pose's reference frame.
	 */
	PointXY project_local(const PointXY &point) const;
};

/**
 * @brief Compute the angle of the line segment between the given point and the origin.
 */
template<class Point> double angle(const Point &p)
{
	return std::atan2(p.y, p.x);
}

/**
 * @brief Compute the angle of the line that passes between the given two points.
 */
template<class Point> double angle(const Point &a, const Point &b)
{
	return std::atan2(b.y - a.y, b.x - a.x);
}

/**
 * @brief Return the Euclidean distance between a cartesian point and the origin.
 */
double distance(const PointXY &p);

/**
 * @brief Return the Euclidean distance between two cartesian points.
 */
double distance(const PointXY &a, const PointXY &b);

/**
 * @brief Return the squared Euclidean distance between two cartesian points.
 */
double distance2(const PointXY &a, const PointXY &b);

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
