#ifndef CANVAS_H
#define CANVAS_H

#include "types.h"

namespace g2d
{

/**
 * @brief Collection of geometric objects.
 *
 * The `Canvas` class combines a simple API for instantiating a variety of geometric
 * objects with centralized storage. It makes possible to manage heterogeneous
 * collections as a single entity, and provides the basis for transparent visualization
 * (through the `Display` class).
 *
 * **Notes**
 *
 * Care must be taken when keeping references returned by instantiation methods.
 * Because of how dynamic resizing works for the `std::vector` class, a reference may
 * be invalidated after new objects of the same type are created. For example:
 *
 *		g2d::Canvas canvas;
 *		const g2d::Point &p1 = canvas.point(0, 0);
 *
 *		// This call causes the point buffer to be reallocated; p1 may now point to unallocated memory
 *		const g2d::Point &p2 = canvas.point(1, 1);
 *
 * The solution is to hold a copy of the object, for example:
 *
 *		g2d::Canvas canvas;
 *		g2d::Point p1 = canvas.point(0, 0);
 *
 *		// This will not affect the locally allocated p1 above
 *		g2d::Point p2 = canvas.point(1, 1);
 */
struct Canvas
{
	/*
	 * Attributes
	 */

	/** \brief Circles drawn on this canvas. */
	Circles circles;

	/** \brief Points drawn on this canvas. */
	Points points;

	/** \brief Polygons drawn on this canvas. */
	Polygons polygons;

	/*
	 * Methods
	 */

	/**
	 * \brief Draws a circle of center `(x, y)` and squared `r` on this canvas.
	 *
	 * The drawn circle is appended to this object's `circles` sequence.
	 *
	 * \return A reference to the drawn circle.
	 */
	Circle &circle(Field x, Field y, Field r);

	/**
	 * \brief Draws a point at coordinates `(x, y)`.
	 *
	 * The drawn point is appended to this object's `points` sequence.
	 *
	 * \return A reference to the drawn point.
	 */
	Point &point(Field x, Field y);

	/**
	 * \brief Draws a rectangle of center `(x, y)`, half-width `w_2`, half-height `h_2`, and counter-clockwise rotation `t` around it center.
	 *
	 * The drawn rectangle is appended to this object's `polygons` sequence.
	 *
	 * \return A reference to the drawn rectangle.
	 */
	Polygon &rectangle(Field x, Field y, Field w_2, Field h_2, Field t);
};

}

#endif
