#ifndef CARMEN_CPP_POINT_H
#define CARMEN_CPP_POINT_H

#include <carmen/cpp_genericpoint.h>

typedef point2d<int> IntPoint;

class Point : public point2d<double> {
 public:
  Point() : point2d<double>() {};
  Point(double _x, double _y) : point2d<double>(_x, _y) {};
  Point(const Point& p) : point2d<double>(p.x, p.y) {};
  Point(const carmen_point_t& p) : point2d<double>(p.x, p.y) {};
  virtual ~Point()  {};

  carmen_inline operator carmen_point_t() const { carmen_point_t pt; pt.x = x; pt.y = y; return pt;}
  carmen_inline Point& operator= (const carmen_point_t& pt) { x=pt.x; y=pt.y; return *this;}
};

class OrientedPoint : public orientedpoint2d<double,double> {
 public:
  OrientedPoint() : orientedpoint2d<double,double>() {};
  OrientedPoint(double _x, double _y, double _t) : orientedpoint2d<double,double>(_x, _y, _t) {};
  OrientedPoint(const OrientedPoint& p) : orientedpoint2d<double,double>(p.x, p.y, p.theta) {};
  OrientedPoint(const carmen_point_t& p) : orientedpoint2d<double,double>(p.x, p.y, p.theta) {};
  virtual ~OrientedPoint() {};

  carmen_inline operator carmen_point_t() const { carmen_point_t pt; pt.x = x; pt.y = y; pt.theta=theta; return pt;}
  carmen_inline OrientedPoint& operator= (const carmen_point_t& pt) { x=pt.x; y=pt.y; theta=pt.theta; return *this;}
 
};

#endif
