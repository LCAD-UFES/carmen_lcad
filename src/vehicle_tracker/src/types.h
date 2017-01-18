#ifndef TYPES_H
#define TYPES_H

#include <iostream>

#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Algebraic_kernel_for_circles_2_2.h>
#include <CGAL/Circular_kernel_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Polygon_2.h>

#include <vector>

namespace g2d
{

/** \brief Geometric kernel used by the application. */
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

/** \brief Field (scalar) type. */
typedef Kernel::FT Field;

/** \brief 2D point type. */
typedef Kernel::Point_2 Point;

/** \brief Sequence of points. */
typedef std::vector<Point> Points;

/** \brief 2D line segment type. */
typedef Kernel::Segment_2 Segment;

/** \brief Sequence of segments. */
typedef std::vector<Segment> Segments;

/** \brief 2D line type. */
typedef Kernel::Line_2 Line;

/** \brief Sequence of lines. */
typedef std::vector<Line> Lines;

/** \brief 2D vector type. */
typedef CGAL::Vector_2<Kernel> Vector;

/** \brief Sequence of vectors. */
typedef std::vector<Vector> Vectors;

/** \brief 2D circle type. */
typedef Kernel::Circle_2 Circle;

/** \brief Sequence of circles. */
typedef std::vector<Circle> Circles;

/** \brief 2D polygon type. */
typedef CGAL::Polygon_2<Kernel, std::vector< Point> > Polygon;

/** \brief Circular buffer of polygon vertices. */
typedef Polygon::Vertex_circulator Vertices;

/** \brief Sequence of polygons. */
typedef std::vector<Polygon> Polygons;

/** \brief 2D affine transform type. */
typedef CGAL::Aff_transformation_2<Kernel> Affine;

/**
 * @brief Returns the centroid point of the given polygon.
 */
Point centroid(Polygon polygon);

}

#endif // CGAL_KERNEL_H
