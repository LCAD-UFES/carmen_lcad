/*!@file Util/Geometry2DFunctions.H Miscellaneous planar geometry 
  functions */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Geometry2DFunctions.H $
// $Id: $
//

#include "Image/Point2D.H"

#ifndef GEOMETRY2DFUNCTIONS_H_DEFINED
#define GEOMETRY2DFUNCTIONS_H_DEFINED

//! return intersection point between two segments
//! defined by p1 and p2, and p3 and p4
Point2D<float> intersectPoint
(Point2D<float> p1, Point2D<float> p2,Point2D<float> p3,Point2D<float> p4);

//! return intersection point between two lines
//! defined by p1 and p2, and p3 and p4
Point2D<float> lineIntersectPoint
(Point2D<float> p1, Point2D<float> p2,Point2D<float> p3,Point2D<float> p4);

//! get the distance between a line (defined by pt1 and pt2) 
//! and point pt
template<typename T> 
double distance
(Point2D<T> pt1, Point2D<T> pt2, Point2D<T> pt);

//! get the distance between a line (defined by pt1 and pt2) 
//! and point pt, return the distance to the line and intersect point midPt
template<typename T> 
double distance
(Point2D<T> pt1, Point2D<T> pt2, Point2D<T> pt, Point2D<T>& midPt);
//! get which side of a vector (defined from pt1 to pt2) 
//! point pt falls onto 
//! NOTE: 0 means on the line
//!       positive value means to the left 
//!       negative value means to the right
template<typename T>
T side
(Point2D<T> pt1, Point2D<T> pt2, Point2D<T> pt);

//! get the usual y = mx + b representation of a line 
void getLineEquation
(Point2D<int> pt1, Point2D<int> pt2,float& slope, float& y_intercept);

//! get a polar representation of a line
void getPolarLine
(Point2D<int> pt1, Point2D<int> pt2,float& theta, float& radius);


//! find closest distance to line between linePt1 to linePt2  
template<typename T> 
double distanceFromPoly(const Point2D<T> p, const std::vector<Point2D<T> > poly);


//! find closest distance from a point to a polygon 
template<typename T> 
double pointDistOnLine(Point2D<T> lpt1, Point2D<T> lpt2, Point2D<T> pt);

template<typename T> 
double pointDistOnLine(Point2D<T> lpt1, Point2D<T> lpt2, Point2D<T> pt, Point2D<T>& midPt);

template<typename T> 
double lineDist(Point2D<T> p1, Point2D<T> p2);
// ######################################################################
template<typename T> 
T side(Point2D<T> pt1, Point2D<T> pt2, Point2D<T> pt)
{
  T i = pt.i;
  T j = pt.j;

  T Ax = pt1.i;  T Ay = pt1.j;
  T Bx = pt2.i;  T By = pt2.j;

  // 0 means on the line
  // positive value means to the left 
  // negative value means to the right
  return (Bx-Ax)*(j-Ay) - (By-Ay)*(i-Ax);
}

// ######################################################################
template<typename T> 
double distance(Point2D<T> pt1, Point2D<T> pt2, Point2D<T> pt)
{
 Point2D<T> midPt;
 return distance(pt1,pt2,pt,midPt);
}
// ######################################################################

template<typename T> 
double distance(Point2D<T> pt1, Point2D<T> pt2, Point2D<T> pt,Point2D<T>& midPt)
{
  double x1 = pt1.i;
  double y1 = pt1.j;
  
  double x2 = pt2.i;
  double y2 = pt2.j;

  double x3 = pt.i;
  double y3 = pt.j;
  
  double xi;  double yi;

  if     (x1 == x2){ xi = x1; yi = y3; }
  else if(y1 == y2){ xi = x3; yi = y1; }
  else
    {
      double x2m1 = x2 - x1;
      double y2m1 = y2 - y1;

      double x3m1 = x3 - x1;
      double y3m1 = y3 - y1;

      double distSq = x2m1*x2m1+y2m1*y2m1;
      double u = (x3m1*x2m1 + y3m1*y2m1)/distSq;

      xi = x1 + u*x2m1;
      yi = y1 + u*y2m1;
    }

  midPt = Point2D<T>(xi,yi);

  double dx = xi - x3;
  double dy = yi - y3;
  return pow(dx*dx+dy*dy, .5F);
}

// ######################################################################
template<typename T> 
double lineDist(Point2D<T> p1, Point2D<T> p2)
{

  Point2D<double> d = Point2D<double>(p1-p2);
  return sqrt(d.i * d.i + d.j * d.j); 
}
// ######################################################################
// find closest distance to line between linePt1 to linePt2  
template<typename T> 
double pointDistOnLine(Point2D<T> lpt1, Point2D<T> lpt2, Point2D<T> pt)
{
  Point2D<T> midPt;
  return pointDistOnLine(lpt1,lpt2,pt,midPt);
}
// ######################################################################
// find closest distance to line between linePt1 to linePt2,also return intersect point on line
template<typename T> 
double pointDistOnLine(Point2D<T> lpt1, Point2D<T> lpt2, Point2D<T> pt, Point2D<T>& midPt)
{
  double dx = lpt2.i - lpt1.i;
  double dy = lpt2.j - lpt1.j;

  double u = ((pt.i - lpt1.i) * dx + (pt.j - lpt1.j)*dy) / (dx * dx + dy * dy);

  Point2D<double> closestPointOnLine;
  if (u < 0.0) {
    closestPointOnLine = Point2D<double> (lpt1.i,lpt1.j);
  } else if (u > 1.0) {
    closestPointOnLine = Point2D<double> (lpt2.i,lpt2.j);
  } else {
    closestPointOnLine = 
      Point2D<double>(double(lpt1.i) + u * dx, double(lpt1.j) + u * dy);
  }

  midPt = Point2D<T>(closestPointOnLine);
  Point2D<double> d = Point2D<double>(pt.i,pt.j) - closestPointOnLine;
  return sqrt(d.i * d.i + d.j * d.j); 
}

// ######################################################################
// find closest distance from a point to a polygon 
template<typename T> 
double distanceFromPoly(const Point2D<T> p, const std::vector<Point2D<T> > poly)
{
  double result = -1.0;
    
  // check each line
  // the polygon here is defined as pt0,pt1,pt2,pt3,...ptn,pt0.
  // repeat first one to last one.
  // so we start to check from pt0->pt1
  for(int i = 1; i < (int)poly.size(); i++)
    {
      Point2D<T> currentPoint  = poly[i];
      Point2D<T> previousPoint = poly[i-1];
        				
      double segmentDistance = pointDistOnLine(previousPoint, currentPoint,p);

      if(segmentDistance < result||result == -1.0)
        result = segmentDistance;      
    }
   	 
    return result;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
