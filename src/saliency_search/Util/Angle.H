/*!@file Util/Angle.H A simple class to embody an angular value */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Angle.H $
// $Id: Angle.H 10221 2008-09-07 17:45:52Z lior $
//

#ifndef ANGLE_H_DEFINED
#define ANGLE_H_DEFINED

#include <cmath>
#include <deque>

//! Embodiment of an angular value
/*! This class provides functions to manipulate and do maths on
  angles. The internal representation is a double value in degrees in
  the range ]-180.0 .. 180.0]. This is an all-inlined class, so there
  is no Angle.C file. */
class Angle
{
public:
  // ######################################################################
  /*! @name Constructors, destructor, assigments, and conversions */
  //@{

  //! Constructor. Initial value should be in degrees.
  inline Angle(const double val = 0.0);

  //! Copy constructor
  inline Angle(const Angle& ang);

  //! Destructor
  inline ~Angle();

  //! Assignment
  inline Angle& operator=(const Angle& ang);

  //! Assignment
  inline Angle& operator=(const double ang);

  //! Set a new value in degrees
  inline void setVal(const double val);

  //! Get current value in degrees in ]-180.0 .. 180.0]
  inline double getVal() const;

  //! Conversion from radians
  inline void setRadians(const double val);

  //! Get value in radians (in [-pi .. pi])
  inline double getRadians() const;

  //@}

  // ######################################################################
  /*! @name Arithmetic operators */
  //@{

  //! Addition. Result is taken modulo 360 and remains in ]-180.0 .. 180.0]
  inline Angle& operator+=(const Angle& other);

  //! Subtraction. Result is taken modulo 360 and remains in ]-180.0 .. 180.0]
  inline Angle& operator-=(const Angle& other);

  //! Multiplication by a constant. Result remains in ]-180.0 .. 180.0]
  inline Angle& operator*=(const double fac);

  //! Division by a constant. Result remains in ]-180.0 .. 180.0]
  inline Angle& operator/=(const double fac);

  //! Addition of two angles. Result remains in ]-180.0 .. 180.0]
  inline Angle operator+(const Angle& other) const;

  //! Subtraction of two angles. Result remains in ]-180.0 .. 180.0]
  inline Angle operator-(const Angle& other) const;

  //! Combine with another angle, in vector style
  /*! This computes the sum of a unit vector with our angle and a
    vector with other's angle and optionally a given length. The
    result is the angle of the combined vector. Special cases (e.g.,
    other is the exact opposite of us) are handled like atan2()
    would. */
  inline void vectorAdd(const Angle& other, const double otherlen = 1.0);

  //! Comparison. USE WITH CAUTION!
  inline bool operator<(const Angle& other) const;

  //! Comparison. USE WITH CAUTION!
  inline bool operator>(const Angle& other) const;

  //! Comparison. USE WITH CAUTION!
  inline bool operator!=(const Angle& other) const;

  //! Comparison. USE WITH CAUTION!
  inline bool operator<(const double other) const;

  //! Comparison. USE WITH CAUTION!
  inline bool operator>(const double other) const;

  //! Comparison. USE WITH CAUTION!
  inline bool operator!=(const double other) const;

  //@}

private:
  inline void fixValue(); // apply modulo to enforce ]-180.0 .. 180.0]

  double angle;
};

// ######################################################################
// ##### Free functions that manipulate angles:
// ######################################################################

//! Compute the vector average of a bunch of angles
/*! Each angle is assumed to be associated with a vector of unit
  length, and the angle of the vector sum of all unit vectors is
  resturned. */
inline Angle averageVectorAngle(std::deque<Angle>::const_iterator astart,
                                std::deque<Angle>::const_iterator aend);

//! Compute the weighted vector average of a bunch of angles
/*! The length of each vector starts with 1.0 and then is multiplied
  by factor each at each iteration, thus providing an exponential
  length decay. Each angle is assumed to be associated with a vector
  of given length length, and the angle of the vector sum of all
  vectors is resturned. */
inline Angle averageVectorAngle(std::deque<Angle>::const_iterator astart,
                                std::deque<Angle>::const_iterator aend,
                                const double factor);



// ######################################################################
// ########## Inlined methods:
// ######################################################################
inline Angle::Angle(const double val) :
  angle(val)
{ fixValue(); }

// ######################################################################
inline Angle::Angle(const Angle& ang) :
  angle(ang.angle)
{  }

// ######################################################################
inline Angle::~Angle()
{  }

// ######################################################################
inline Angle& Angle::operator=(const Angle& ang)
{ angle = ang.angle; return *this; }

// ######################################################################
inline Angle& Angle::operator=(const double ang)
{ angle = ang; return *this; }

// ######################################################################
inline void Angle::setVal(const double val)
{ angle = val; fixValue(); }

// ######################################################################
inline double Angle::getVal() const
{ return angle; }

// ######################################################################
inline void Angle::setRadians(const double val)
{ angle = val * 180.0 / M_PI; fixValue(); }

// ######################################################################
inline double Angle::getRadians() const
{ return angle * M_PI / 180.0; }

// ######################################################################
inline Angle& Angle::operator+=(const Angle& other)
{ angle += other.angle; fixValue(); return *this; }

// ######################################################################
inline Angle& Angle::operator-=(const Angle& other)
{ angle -= other.angle; fixValue(); return *this; }

// ######################################################################
inline Angle& Angle::operator*=(const double fac)
{ angle *= fac; fixValue(); return *this; }

// ######################################################################
inline Angle& Angle::operator/=(const double fac)
{ angle /= fac; fixValue(); return *this; }

// ######################################################################
inline Angle Angle::operator+(const Angle& other) const
{ Angle a(*this); a += other; a.fixValue(); return a; }

// ######################################################################
inline Angle Angle::operator-(const Angle& other) const
{ Angle a(*this); a -= other; a.fixValue(); return a; }

// ######################################################################
inline void Angle::vectorAdd(const Angle& other, const double otherlen)
{
  // compute the vector sum:
  double x = cos(angle * M_PI / 180.0);
  double y = sin(angle * M_PI / 180.0);
  x += cos(other.angle * M_PI / 180.0) * otherlen;
  y += sin(other.angle * M_PI / 180.0) * otherlen;

  angle = atan2(y, x) * 180.0 / M_PI;
  fixValue(); // atan2 returns value between -PI and PI inclusive
}

// ######################################################################
inline bool Angle::operator<(const Angle& other) const
{ return (angle < other.angle); }

// ######################################################################
inline bool Angle::operator>(const Angle& other) const
{ return (angle > other.angle); }

// ######################################################################
inline bool Angle::operator!=(const Angle& other) const
{ return (angle != other.angle); }

// ######################################################################
inline bool Angle::operator<(const double other) const
{ return (angle < other); }

// ######################################################################
inline bool Angle::operator>(const double other) const
{ return (angle > other); }

// ######################################################################
inline bool Angle::operator!=(const double other) const
{ return (angle != other); }

// ######################################################################
inline void Angle::fixValue()
{
  while (angle <= -180.0) angle += 360.0;
  while (angle > 180.0) angle -= 360.0;
}




// ######################################################################
// ##### Free functions that manipulate angles:
// ######################################################################
inline Angle averageVectorAngle(std::deque<Angle>::const_iterator astart,
                                std::deque<Angle>::const_iterator aend)
{
  double x = 0.0, y = 0.0;
  for (std::deque<Angle>::const_iterator i = astart; i != aend; i ++)
    {
      double a = i->getVal();
      x += cos(a * M_PI / 180.0);
      y += sin(a * M_PI / 180.0);
    }
  return Angle(atan2(y, x) * 180.0 / M_PI);
}

// ######################################################################
inline Angle averageVectorAngle(std::deque<Angle>::const_iterator astart,
                                std::deque<Angle>::const_iterator aend,
                                const double factor)
{
  double x = 0.0, y = 0.0; double len = 1.0;
  for (std::deque<Angle>::const_iterator i = astart; i != aend; i ++)
    {
      double a = i->getVal();
      x += cos(a * M_PI / 180.0) * len;
      y += sin(a * M_PI / 180.0) * len;
      len *= factor;
    }
  return Angle(atan2(y, x) * 180.0 / M_PI);
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
