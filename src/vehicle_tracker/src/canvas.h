#ifndef CANVAS_H
#define CANVAS_H

#include "types.h"

namespace g2d
{

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

#endif // SCENE_H
