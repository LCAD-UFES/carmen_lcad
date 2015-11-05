/*!@file Image/Point3D.H A basic 3D point class */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Point3D.H $
// $Id: Point3D.H 14628 2011-03-23 16:49:36Z lior $
//

#ifndef POINT3D_H_DEFINED
#define POINT3D_H_DEFINED

#include "Image/Dims.H"
#include "Image/Image.H"
#include "Util/Promotions.H"
#include "Util/log.H"
#include <cmath>
#include <string> // for string conversions
#include <vector>

//! This is a basic class to encode 3D coordinates
/*! This is a completely public class whose goal is just to provide a
 shorthand notation for 3D coordinates.
*/
template <class T>
class Point3D
{
public:
  //! The default constructor initializes the coordinates to (0,0)
  inline Point3D();

  //! Initialize the Point3D from horizontal & vertical coordinates
  inline Point3D(const T xx, const T yy, const T zz);

  //! Initialize the Point3D from an image
  inline Point3D(const Image<T>& mat);

  //! Explicit conversion from type T to another type U
  /*! Note that this simply uses clamped_convert, so it will clamp
      coordinates to the available range of T, and may round down. */
  template <class U>
  explicit inline Point3D(const Point3D<U>& a);



  //! += operator
  inline Point3D<T>& operator+=(const Point3D<T> &p);
  //! -= operator
  inline Point3D<T>& operator-=(const Point3D<T> &p);
  //! *= operator
  inline Point3D<T>& operator*=(const Point3D<T> &p);
  //! /= operator
  inline Point3D<T>& operator/=(const Point3D<T> &p);

  //! + operator
  template <class U>
  inline Point3D<typename promote_trait<T,U>::TP>
  operator+(const Point3D<U> &p) const;
  //! - operator
  template <class U>
  inline Point3D<typename promote_trait<T,U>::TP>
  operator-(const Point3D<U> &p) const;
  //! * operator
  template <class U>
  inline Point3D<typename promote_trait<T,U>::TP>
  operator*(const Point3D<U> &p) const;
  //! / operator
  template <class U>
  inline Point3D<typename promote_trait<T,U>::TP>
  operator/(const Point3D<U> &p) const;

  //! += operator
  inline Point3D<T>& operator+=(const T val);
  //! -= operator
  inline Point3D<T>& operator-=(const T val);
  //! *= operator
  inline Point3D<T>& operator*=(const T val);
  //! /= operator
  inline Point3D<T>& operator/=(const T val);

  //! + operator
  template <class U>
  inline Point3D<typename promote_trait<T,U>::TP> operator+(const U val) const;

  //! - operator
  template <class U>
  inline Point3D<typename promote_trait<T,U>::TP> operator-(const U val) const;

  //! * operator
  template <class U>
  inline Point3D<typename promote_trait<T,U>::TP> operator*(const U val) const;

  //! / operator
  template <class U>
  inline Point3D<typename promote_trait<T,U>::TP> operator/(const U val) const;

  //! test whether x and y and z are all positive
  inline bool isValid() const;

  //! the square of the euclidean distance
  inline typename promote_trait<T,float>::TP
  squdist(const Point3D<T>& p) const;

  //! the euclidean distance from p
  inline typename promote_trait<T,float>::TP
  distance(const Point3D<T>& p) const;

  //! the magnitude
  inline typename promote_trait<T,float>::TP
  magnitude() const;

  //! Get an image representation used for matrix operations [x,y,z]
  inline Image<T> getImage();

  //! 3D coordinates
  T x, y, z;
};


//! == operator
template <class T, class U>
inline bool operator==(const Point3D<T>& p1, const Point3D<U>& p2);

//! != operator
template <class T, class U>
inline bool operator!=(const Point3D<T>& p1, const Point3D<U>& p2);

//! > operator
template <class T, class U>
inline bool operator>(const Point3D<T>& p1, const Point3D<U>& p2);

//! < operator
template <class T, class U>
inline bool operator<(const Point3D<T>& p1, const Point3D<U>& p2);


//! Point3D<T> overload: format is "<i>,<j>"
template <class T>
std::string convertToString(const Point3D<T>& val);

//! Point3D<T> overload: format is "<i>,<j>"
template <class T>
void convertFromString(const std::string& str, Point3D<T>& val);

// ######################################################################
template <class T>
inline Point3D<T>::Point3D()
{ z = 0; y = 0; z = 0;}

// ######################################################################
template <class T>
inline Point3D<T>::Point3D(const T xx, const T yy, const T zz)
{ x = xx; y = yy; z= zz;}

// ######################################################################
template <class T>
inline Point3D<T>::Point3D(const Image<T>& mat) 
{
  ASSERT(mat.size() == 3);
  x = mat[0]; y = mat[1]; z= mat[2];
}

// #######################################################################
template <class T>
template <class U>
inline Point3D<T>::Point3D(const Point3D<U>& a)
  : x(clamped_convert<T>(a.x)), y(clamped_convert<T>(a.y)), z(clamped_convert<T>(a.z))
{ }

// ######################################################################
template <class T>
inline Point3D<T>& Point3D<T>::operator+=(const Point3D<T> &p)
{ x += p.x; y += p.y; z += p.z; return *this; }

// ######################################################################
template <class T>
inline Point3D<T>& Point3D<T>::operator-=(const Point3D<T> &p)
{ x -= p.x; y -= p.y; z -= p.z; return *this; }

// ######################################################################
template <class T>
inline Point3D<T>& Point3D<T>::operator*=(const Point3D<T> &p)
{ x *= p.x; y *= p.y; z *= p.z; return *this; }

// ######################################################################
template <class T>
inline Point3D<T>& Point3D<T>::operator/=(const Point3D<T> &p)
{ x /= p.x; y /= p.y; z /- p.z; return *this; }

// ######################################################################
template <class T>
template <class U>
inline Point3D<typename promote_trait<T,U>::TP>
Point3D<T>::operator+(const Point3D<U> &p) const
{ return Point3D<typename promote_trait<T,U>::TP>(x + p.x, y + p.y, z + p.z); }

// ######################################################################
template <class T>
template <class U>
inline Point3D<typename promote_trait<T,U>::TP>
Point3D<T>::operator-(const Point3D<U> &p) const
{ return Point3D<typename promote_trait<T,U>::TP>(x - p.x, y - p.y, z - p.z); }

// ######################################################################
template <class T>
template <class U>
inline Point3D<typename promote_trait<T,U>::TP>
Point3D<T>::operator*(const Point3D<U> &p) const
{ return Point3D<typename promote_trait<T,U>::TP>(x * p.x, y * p.y, z * p.z); }

// ######################################################################
template <class T>
template <class U>
inline Point3D<typename promote_trait<T,U>::TP>
Point3D<T>::operator/(const Point3D<U> &p) const
{ return Point3D<typename promote_trait<T,U>::TP>(x / p.x, y / p.y, z / p.z); }

// ######################################################################
template <class T>
inline Point3D<T>& Point3D<T>::operator+=(const T val)
{ x += val; y += val; z += val; return *this; }

// ######################################################################
template <class T>
inline Point3D<T>& Point3D<T>::operator-=(const T val)
{ x -= val; y -= val; z -= val; return *this; }

// ######################################################################
template <class T>
inline Point3D<T>& Point3D<T>::operator*=(const T val)
{ x *= val; y *= val; z *= val; return *this; }

// ######################################################################
template <class T>
inline Point3D<T>& Point3D<T>::operator/=(const T val)
{ x /= val; y /= val; z /= val; return *this; }

// ######################################################################
template <class T>
template <class U>
inline Point3D<typename promote_trait<T,U>::TP>
Point3D<T>::operator+(const U val) const
{ return Point3D<typename promote_trait<T,U>::TP>(this->x+val, this->y+val, this->z+val); }

// ######################################################################
template <class T>
template <class U>
inline Point3D<typename promote_trait<T,U>::TP>
Point3D<T>::operator-(const U val) const
{ return Point3D<typename promote_trait<T,U>::TP>(this->x-val, this->y-val, this->z-val); }

// ######################################################################
template <class T>
template <class U>
inline Point3D<typename promote_trait<T,U>::TP>
Point3D<T>::operator*(const U val) const
{ return Point3D<typename promote_trait<T,U>::TP>(this->x*val, this->y*val, this->z*val); }

// ######################################################################
template <class T>
template <class U>
inline Point3D<typename promote_trait<T,U>::TP>
Point3D<T>::operator/(const U val) const
{ return Point3D<typename promote_trait<T,U>::TP>(this->x/val, this->y/val, this->z/val); }

// ######################################################################
template <class T, class U>
inline bool operator==(const Point3D<T>& p1, const Point3D<U>& p2)
{ return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z; }

// ######################################################################
template <class T, class U>
inline bool operator!=(const Point3D<T>& p1, const Point3D<U>& p2)
{ return p1.x != p2.x || p1.y != p2.y || p1.z != p2.z; }

// ######################################################################
template <class T, class U>
inline bool operator>(const Point3D<T>& p1, const Point3D<U>& p2)
{ return p1.x > p2.x && p1.y > p2.y && p1.z > p2.z; }

// ######################################################################
template <class T, class U>
inline bool operator<(const Point3D<T>& p1, const Point3D<U>& p2)
{ return p1.x < p2.x && p1.y < p2.y && p1.z < p2.z; }

// ######################################################################
template <class T>
inline bool Point3D<T>::isValid() const
{ return ((x >= 0) && (y >= 0) &&  (z >= 0)); }

// ######################################################################
template <class T>
inline typename promote_trait<T,float>::TP
Point3D<T>::squdist(const Point3D<T>& p) const
{
  typedef typename promote_trait<T,float>::TP TF;
  TF d1 = p.x - x, d2 = p.y - y, d3 = p.z - z;
  return (d1 * d1 + d2 * d2 + d3 * d3);
}

// ######################################################################
template <class T>
inline typename promote_trait<T,float>::TP
Point3D<T>::distance(const Point3D<T>& p) const
{ return sqrt(squdist(p)); }

// ######################################################################
template <class T>
inline typename promote_trait<T,float>::TP
Point3D<T>::magnitude() const
{ return sqrt((x*x) + (y*y) + (z*z)); }


template <class T>
inline Image<T> Point3D<T>::getImage()
{
  Image<T> mat(1,3,NO_INIT);
  mat[0] = x;
  mat[1] = y;
  mat[2] = z;

  return mat;
}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
