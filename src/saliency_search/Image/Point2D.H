/*!@file Image/Point2D.H A basic 2D point class */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Point2D.H $
// $Id: Point2D.H 14734 2011-04-14 18:59:24Z lior $
//

#ifndef POINT2D_H_DEFINED
#define POINT2D_H_DEFINED

#include "Image/Dims.H"
#include "Util/Promotions.H"
#include "Util/log.H"
#include <cmath>
#include <string> // for string conversions
#include <vector>

//! This is a basic class to encode 2D integer coordinates
/*! This is a completely public class whose goal is just to provide a
 shorthand notation for 2D integer coordinates.
*/
template <class T>
class Point2D
{
public:
  //! The default constructor initializes the coordinates to (0,0)
  inline Point2D();

  //! Initialize the Point2D from horizontal & vertical coordinates
  inline Point2D(const T ii, const T jj);

  //! Initialize from a Dims object; i=w, j=h
  explicit inline Point2D(const Dims& d);

  //! Explicit conversion from type T to another type U
  /*! Note that this simply uses clamped_convert, so it will clamp
      coordinates to the available range of T, and may round down. */
  template <class U>
  explicit inline Point2D(const Point2D<U>& a);

  //! += operator
  inline Point2D<T>& operator+=(const Point2D<T> &p);
  //! -= operator
  inline Point2D<T>& operator-=(const Point2D<T> &p);
  //! *= operator
  inline Point2D<T>& operator*=(const Point2D<T> &p);
  //! /= operator
  inline Point2D<T>& operator/=(const Point2D<T> &p);

  //! + operator
  template <class U>
  inline Point2D<typename promote_trait<T,U>::TP>
  operator+(const Point2D<U> &p) const;
  //! - operator
  template <class U>
  inline Point2D<typename promote_trait<T,U>::TP>
  operator-(const Point2D<U> &p) const;
  //! * operator
  template <class U>
  inline Point2D<typename promote_trait<T,U>::TP>
  operator*(const Point2D<U> &p) const;
  //! / operator
  template <class U>
  inline Point2D<typename promote_trait<T,U>::TP>
  operator/(const Point2D<U> &p) const;

  //! += operator
  inline Point2D<T>& operator+=(const T val);
  //! -= operator
  inline Point2D<T>& operator-=(const T val);
  //! *= operator
  inline Point2D<T>& operator*=(const T val);
  //! /= operator
  inline Point2D<T>& operator/=(const T val);

  //! + operator
  template <class U>
  inline Point2D<typename promote_trait<T,U>::TP> operator+(const U val) const;

  //! - operator
  template <class U>
  inline Point2D<typename promote_trait<T,U>::TP> operator-(const U val) const;

  //! * operator
  template <class U>
  inline Point2D<typename promote_trait<T,U>::TP> operator*(const U val) const;

  //! / operator
  template <class U>
  inline Point2D<typename promote_trait<T,U>::TP> operator/(const U val) const;

  //! test whether i and j are both positive
  inline bool isValid() const;

  //! the square of the euclidean distance
  inline typename promote_trait<T,float>::TP
  squdist(const Point2D<T>& p) const;

  //! the euclidean distance from p
  inline typename promote_trait<T,float>::TP
  distance(const Point2D<T>& p) const;

  //! the Magnitude 
  inline typename promote_trait<T,float>::TP
  magnitude() const;

  //! the euclidean distance from line pq
  inline typename promote_trait<T,float>::TP
  distanceToLine(const Point2D<T>& p, const Point2D<T>& q, 
                 const bool getsigned = false) const;

  //! the euclidean distance from segment pq
  inline typename promote_trait<T,float>::TP
  distanceToSegment(const Point2D<T>& p, const Point2D<T>& q) const;

  //! clamp the coords such that the point fits into image of dimensions dims
  inline void clampToDims(const Dims& dims);

  //! 2D coordinates
  T i, j;
};


//! == operator
template <class T, class U>
inline bool operator==(const Point2D<T>& p1, const Point2D<U>& p2);

//! != operator
template <class T, class U>
inline bool operator!=(const Point2D<T>& p1, const Point2D<U>& p2);

//! > operator
template <class T, class U>
inline bool operator>(const Point2D<T>& p1, const Point2D<U>& p2);

//! < operator
template <class T, class U>
inline bool operator<(const Point2D<T>& p1, const Point2D<U>& p2);


//! Point2D<T> overload: format is "<i>,<j>"
template <class T>
std::string convertToString(const Point2D<T>& val);

//! Point2D<T> overload: format is "<i>,<j>"
template <class T>
void convertFromString(const std::string& str, Point2D<T>& val);

//! Is a point inside a polygon?
template <class T>
inline bool pnpoly(const std::vector<Point2D<T> > &polygon,
                   const Point2D<T>& p);

//! Where is the center of mass of a polygon?
template <class T>
inline Point2D<T> centroid(const std::vector<Point2D<T> > &polygon);

//! What is the area of a polygon?
template <class T>
inline double area(const std::vector<Point2D<T> > &polygon,
                   const bool getsigned = false);

//! What is the measure (in radians, signed -pi to pi) of angle BAC?
template <class T>
inline double angleMeasure(const Point2D<T> & A,
                           const Point2D<T> & B,
                           const Point2D<T> & C);
 
//! determine if a point P is inside a triangle with vertices A,B,C
template <class T>
inline bool pnTriangle(const Point2D<T>& A,
                       const Point2D<T>& B,
                       const Point2D<T>& C,
                       const Point2D<T>& P);


// ######################################################################
template <class T>
inline Point2D<T>::Point2D()
{ i = 0; j = 0; }

// ######################################################################
template <class T>
inline Point2D<T>::Point2D(const T ii, const T jj)
{ i = ii; j = jj; }

// ######################################################################
template <class T>
inline Point2D<T>::Point2D(const Dims& d)
{ i = d.w(); j = d.h(); }

// #######################################################################
template <class T>
template <class U>
inline Point2D<T>::Point2D(const Point2D<U>& a)
  : i(clamped_convert<T>(a.i)), j(clamped_convert<T>(a.j))
{ }

// ######################################################################
template <class T>
inline Point2D<T>& Point2D<T>::operator+=(const Point2D<T> &p)
{ i += p.i; j += p.j; return *this; }

// ######################################################################
template <class T>
inline Point2D<T>& Point2D<T>::operator-=(const Point2D<T> &p)
{ i -= p.i; j -= p.j; return *this; }

// ######################################################################
template <class T>
inline Point2D<T>& Point2D<T>::operator*=(const Point2D<T> &p)
{ i *= p.i; j *= p.j; return *this; }

// ######################################################################
template <class T>
inline Point2D<T>& Point2D<T>::operator/=(const Point2D<T> &p)
{ i /= p.i; j /= p.j; return *this; }

// ######################################################################
template <class T>
template <class U>
inline Point2D<typename promote_trait<T,U>::TP>
Point2D<T>::operator+(const Point2D<U> &p) const
{ return Point2D<typename promote_trait<T,U>::TP>(i + p.i, j + p.j); }

// ######################################################################
template <class T>
template <class U>
inline Point2D<typename promote_trait<T,U>::TP>
Point2D<T>::operator-(const Point2D<U> &p) const
{ return Point2D<typename promote_trait<T,U>::TP>(i - p.i, j - p.j); }

// ######################################################################
template <class T>
template <class U>
inline Point2D<typename promote_trait<T,U>::TP>
Point2D<T>::operator*(const Point2D<U> &p) const
{ return Point2D<typename promote_trait<T,U>::TP>(i * p.i, j * p.j); }

// ######################################################################
template <class T>
template <class U>
inline Point2D<typename promote_trait<T,U>::TP>
Point2D<T>::operator/(const Point2D<U> &p) const
{ return Point2D<typename promote_trait<T,U>::TP>(i / p.i, j / p.j); }

// ######################################################################
template <class T>
inline Point2D<T>& Point2D<T>::operator+=(const T val)
{ i += val; j += val; return *this; }

// ######################################################################
template <class T>
inline Point2D<T>& Point2D<T>::operator-=(const T val)
{ i -= val; j -= val; return *this; }

// ######################################################################
template <class T>
inline Point2D<T>& Point2D<T>::operator*=(const T val)
{ i *= val; j *= val; return *this; }

// ######################################################################
template <class T>
inline Point2D<T>& Point2D<T>::operator/=(const T val)
{ i /= val; j /= val; return *this; }

// ######################################################################
template <class T>
template <class U>
inline Point2D<typename promote_trait<T,U>::TP>
Point2D<T>::operator+(const U val) const
{ return Point2D<typename promote_trait<T,U>::TP>(this->i+val, this->j+val); }

// ######################################################################
template <class T>
template <class U>
inline Point2D<typename promote_trait<T,U>::TP>
Point2D<T>::operator-(const U val) const
{ return Point2D<typename promote_trait<T,U>::TP>(this->i-val, this->j-val); }

// ######################################################################
template <class T>
template <class U>
inline Point2D<typename promote_trait<T,U>::TP>
Point2D<T>::operator*(const U val) const
{ return Point2D<typename promote_trait<T,U>::TP>(this->i*val, this->j*val); }

// ######################################################################
template <class T>
template <class U>
inline Point2D<typename promote_trait<T,U>::TP>
Point2D<T>::operator/(const U val) const
{ return Point2D<typename promote_trait<T,U>::TP>(this->i/val, this->j/val); }

// ######################################################################
template <class T, class U>
inline bool operator==(const Point2D<T>& p1, const Point2D<U>& p2)
{ return p1.i == p2.i && p1.j == p2.j; }

// ######################################################################
template <class T, class U>
inline bool operator!=(const Point2D<T>& p1, const Point2D<U>& p2)
{ return p1.i != p2.i || p1.j != p2.j; }

// ######################################################################
template <class T, class U>
inline bool operator>(const Point2D<T>& p1, const Point2D<U>& p2)
{ return p1.i > p2.i && p1.j > p2.j; }

// ######################################################################
template <class T, class U>
inline bool operator<(const Point2D<T>& p1, const Point2D<U>& p2)
{ return p1.i < p2.i && p1.j < p2.j; }

// ######################################################################
template <class T>
inline bool Point2D<T>::isValid() const
{ return ((i >= 0) && (j >= 0)); }

// ######################################################################
template <class T>
inline typename promote_trait<T,float>::TP
Point2D<T>::squdist(const Point2D<T>& p) const
{
  typedef typename promote_trait<T,float>::TP TF;
  TF d1 = p.i - i, d2 = p.j - j;
  return (d1 * d1 + d2 * d2);
}

// ######################################################################
template <class T>
inline typename promote_trait<T,float>::TP
Point2D<T>::distance(const Point2D<T>& p) const
{ return sqrt(squdist(p)); }

// ######################################################################
template <class T>
inline typename promote_trait<T,float>::TP
Point2D<T>::magnitude() const
{ return sqrt((i*i) + (j*j)); }

// ######################################################################
template <class T>
inline typename promote_trait<T,float>::TP
Point2D<T>::distanceToLine(const Point2D<T>& p, const Point2D<T>& q, 
                           const bool getsigned) const
{
  // standard geometry
  typedef typename promote_trait<T,float>::TP TF;
  TF a = q.j - p.j, b = q.i - p.i, c = q.i * p.j - q.j * p.i;
  if (getsigned) return (a*i-b*j+c)/sqrt(a*a + b*b);
  else return fabs(a*i-b*j+c)/sqrt(a*a + b*b);
}

// ######################################################################
template <class T>
inline typename promote_trait<T,float>::TP
Point2D<T>::distanceToSegment(const Point2D<T>& p, const Point2D<T>& q) const
{
  // test if point is "past" segment
  Point2D<T> V0 = q-p, V1 = (*this)-q, V2 = (*this)-p; //vectors
  if (V0.i*V1.i + V0.j*V1.j>0) return distance(q); // is "past" q
  if (V0.i*V2.i + V0.j*V2.j<0) return distance(p); // is "past" p
  return distanceToLine(p,q);
}

// ######################################################################
template <class T>
inline void Point2D<T>::clampToDims(const Dims& dims)
{
  if (dims.isEmpty())
    LFATAL("clampToDims() requires non-empty Dims");

  if (i < 0) i = 0; else if (i >= dims.w()) i = dims.w()-1;
  if (j < 0) j = 0; else if (j >= dims.h()) j = dims.h()-1;
}

// ######################################################################
// Original algo by Randolph Franklin,
//   http://astronomy.swin.edu.au/~pbourke/geometry/insidepoly/
template <class T>
inline bool pnpoly(const std::vector<Point2D<T> > &polygon,
                   const Point2D<T>& p)
{
  bool c = false; const uint ps = polygon.size();
  for (uint i = 0, j = ps-1; i < ps; j = i++)
    if ((((polygon[i].j <= p.j) && (p.j < polygon[j].j)) ||
         ((polygon[j].j <= p.j) && (p.j < polygon[i].j))) &&
        (p.i < (polygon[j].i - polygon[i].i) *
         (p.j - polygon[i].j) / (polygon[j].j - polygon[i].j)
         + polygon[i].i))
      c = !c;

  return c;
}

// ######################################################################
template <class T>
inline Point2D<T> centroid(const std::vector<Point2D<T> > &polygon)
{
  Point2D<T> CM(0,0);
  const uint ps = polygon.size();
  for (uint i = 0; i < ps; i++) CM += polygon[i];
  return CM / ps;
}

// ######################################################################
template <class T>
inline double area(const std::vector<Point2D<T> > &polygon, const bool getsigned)
{
  double area = 0.0;
  const uint ps = polygon.size();
  for (uint i = 0, j = ps-1; i < ps; j = i++)
      area += polygon[i].i*polygon[j].j - polygon[i].j*polygon[j].i;
  if (getsigned) return (0.5 * area);
  else return fabs(0.5 * area);
}

// ######################################################################
template <class T>
inline double angleMeasure(const Point2D<T> & A,
                           const Point2D<T> & B,
                           const Point2D<T> & C)
{
  const double eps = 0.00001;
  double Sab = A.distance(B), Sac = A.distance(C), Sbc = B.distance(C);

  if (Sab * Sac * Sbc == 0 || fabs(Sab - Sac + Sbc) < eps
      || fabs(- Sab + Sac + Sbc) < eps) return 0;
  else if(fabs(Sab + Sac - Sbc) < eps) return 3.1415;

  int sgn = (A.i*(B.j-C.j) + B.i*(C.j-A.j) * C.i*(A.j-B.j)) < 0 ? 1 : -1;
  return sgn * acos((Sac*Sac+Sab*Sab-Sbc*Sbc)/(2*Sab*Sac));
}

// ######################################################################
template <class T>
inline bool pnTriangle(const Point2D<T>& A,
                       const Point2D<T>& B,
                       const Point2D<T>& C,
                       const Point2D<T>& P)
{


  T ax = C.i - B.i;  T ay = C.j - B.j;
  T bx = A.i - C.i;  T by = A.j - C.j;
  T cx = B.i - A.i;  T cy = B.j - A.j;
  T apx= P.i - A.i;  T apy= P.j - A.j;
  T bpx= P.i - B.i;  T bpy= P.j - B.j;
  T cpx= P.i - C.i;  T cpy= P.j - C.j;

  T aCROSSbp = ax*bpy - ay*bpx;
  T cCROSSap = cx*apy - cy*apx;
  T bCROSScp = bx*cpy - by*cpx;

  return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
}




// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
