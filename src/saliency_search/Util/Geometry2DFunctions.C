/*!@file Util/Geometry2DFunctions.C Miscellaneous planar geometry 
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Geometry2DFunctions.C $
// $Id: $
//

#include "Util/Geometry2DFunctions.H"

// ######################################################################
Point2D<float> intersectPoint
(Point2D<float> p1, Point2D<float> p2,Point2D<float> p3,Point2D<float> p4)
{
 //Find intersection point Algorithm can be find here :
 //http://paulbourke.net/geometry/lineline2d/

  double mua,mub;
  double denom,numera,numerb;
  double x,y;
  double EPS = 0.0001;//Epsilon : a small number to enough to be insignificant
  

  denom  = (p4.j-p3.j) * (p2.i-p1.i) - (p4.i-p3.i) * (p2.j-p1.j);
  numera = (p4.i-p3.i) * (p1.j-p3.j) - (p4.j-p3.j) * (p1.i-p3.i);
  numerb = (p2.i-p1.i) * (p1.j-p3.j) - (p2.j-p1.j) * (p1.i-p3.i);

  /* Are the lines coincident? */
  if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS) 
    {
      x = (p1.i + p2.i) / 2;
      y = (p1.j + p2.j) / 2;
      return Point2D<float>(x,y);
    }

  /* Are the lines parallel */
  if (fabs(denom) < EPS) {
    x = 0;
    y = 0;
    return Point2D<float>(x,y);
  }

  /* Is the intersection along the the segments */
  mua = numera / denom;
  mub = numerb / denom;
  if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
    x = 0;
    y = 0;
    
  }
  x = p1.i + mua * (p2.i - p1.i);
  y = p1.j + mua * (p2.j - p1.j);

  return Point2D<float>(x,y);
}

// ######################################################################
Point2D<float> lineIntersectPoint
(Point2D<float> p1, Point2D<float> p2,Point2D<float> p3,Point2D<float> p4)
{
 //Find intersection point Algorithm can be find here :
 //http://paulbourke.net/geometry/lineline2d/

  double mua,mub;
  double denom,numera,numerb;
  double x,y;
  double EPS = 0.0001;//Epsilon : a small number to enough to be insignificant
  

  denom  = (p4.j-p3.j) * (p2.i-p1.i) - (p4.i-p3.i) * (p2.j-p1.j);
  numera = (p4.i-p3.i) * (p1.j-p3.j) - (p4.j-p3.j) * (p1.i-p3.i);
  numerb = (p2.i-p1.i) * (p1.j-p3.j) - (p2.j-p1.j) * (p1.i-p3.i);

  /* Are the lines coincident? */
  if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS) 
    {
      x = (p1.i + p2.i) / 2;
      y = (p1.j + p2.j) / 2;
      return Point2D<float>(x,y);
    }

  /* Are the lines parallel */
  if (fabs(denom) < EPS) {
    x = 0;
    y = 0;
    return Point2D<float>(x,y);
  }

  /* Is the intersection along the the segments */
  mua = numera / denom;
  mub = numerb / denom;
  // if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
  //   x = 0;
  //   y = 0;
    
  // }

  x = p1.i + mua * (p2.i - p1.i);
  y = p1.j + mua * (p2.j - p1.j);

  return Point2D<float>(x,y);
}

// ######################################################################
void getLineEquation
(Point2D<int> pt1, Point2D<int> pt2,float& slope, float& y_intercept)
{
  float x1 = pt1.i;
  float y1 = pt1.j;
  
  float x2 = pt2.i;
  float y2 = pt2.j;

  slope = (x2-x1)/(y2-y1);
  y_intercept = y1 - slope*x1;
}


// ######################################################################
void getPolarLine
(Point2D<int> pt1, Point2D<int> pt2,float& theta, float& radius)
{
  // dotLineIntersection = function(x, y, x0, y0, x1, y1)

  float x1 = pt1.i;
  float y1 = pt1.j;
  
  float x2 = pt2.i;
  float y2 = pt2.j;

  float x3 = 0;
  float y3 = 0;
  
  float xi;  float yi;

  if     (x1 == x2){ xi = x1; yi = y3; }
  else if(y1 == y2){ xi = x3; yi = y1; }
  else
    {
      float x2m1 = x2 - x1;
      float y2m1 = y2 - y1;

      float x3m1 = x3 - x1;
      float y3m1 = y3 - y1;

      float distSq = x2m1*x2m1+y2m1*y2m1;
      float u = (x3m1*x2m1 + y3m1*y2m1)/distSq;

      xi = x1 + u*x2m1;
      yi = y1 + u*y2m1;
    }

  theta = atan2(yi,xi);
  radius = pow(xi*xi+yi*yi, .5F);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
