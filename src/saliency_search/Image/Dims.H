/*!@file Image/Dims.H A simple struct to hold a pair of width/height dimensions.
 */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Dims.H $
// $Id: Dims.H 5755 2005-10-19 20:57:27Z rjpeters $
//

#ifndef DIMS_H_DEFINED
#define DIMS_H_DEFINED

#include "Util/Assert.H"
#include "Util/Types.H"

#include <string> // for string conversions

//! A simple struct to hold a pair of width/height dimensions.
/*! We guarantee that the width and height are always non-negative, though not
    necessarily strictly positive. The default Dims are the 0-by-0
    dimensions. Dims is strictly a const class; the width/height can only be
    set at construction. */
class Dims
{
public:
  //! Default construct with width and height == 0.
  inline Dims() throw();

  //! Construct from a given width + height, which must be non-negative.
  inline Dims(int width, int height) throw();

  //! Copy construct.
  inline Dims(const Dims& other) throw();

  // Default copy-ctor and assignment-operator OK.

  //! Returns the width, will always be non-negative.
  inline int w() const throw();

  //! Returns the height, will always be non-negative.
  inline int h() const throw();

  //! Returns the size of the 2-D area specified by the dimensions.
  /*! (i.e. width * height). */
  inline int sz() const throw();

  //! Returns the larger of the width and height.
  inline int max() const throw();

  //! Returns the smaller of the width and height.
  inline int min() const throw();

  //! Query whether the 2-D array is empty
  /*! (empty means that at least one of the width or height is 0). */
  inline bool isEmpty() const throw();

  //! Query whether the 2-D array is non-empty
  /*! (i.e. both width and height are positive, non-zero). */
  inline bool isNonEmpty() const throw();

  //! See if two Dims are the same.
  inline bool operator==(const Dims& x) const throw();

  //! See if two Dims are different.
  inline bool operator!=(const Dims& x) const throw();

private:
  int ww;  //!< The width.
  int hh;  //!< The height.
};


//! Dims overload: format is "<int>x<int>"
std::string convertToString(const Dims& val);

//! Dims overload: format is "<int>x<int>"
void convertFromString(const std::string& str, Dims& val);


// ######################################################################
// ######################################################################
// INLINE free functions for Dims:
// ######################################################################
// ######################################################################

/// Dims+Dims addition
inline Dims operator+(const Dims& d1, const Dims& d2)
{
  return Dims(d1.w()+d2.w(), d1.h()+d2.h());
}

/// Dims+int addition
inline Dims operator+(const Dims& d, int a)
{
  return Dims(d.w()+a, d.h()+a);
}

/// int+Dims addition
inline Dims operator+(int a, const Dims& d)
{
  return Dims(d.w()+a, d.h()+a);
}

/// Dims-int subtraction
inline Dims operator-(const Dims& d, int a)
{
  return Dims(d.w()-a, d.h()-a);
}

/// Dims*int multiplication
inline Dims operator*(const Dims& d, int factor)
{
  return Dims(d.w()*factor, d.h()*factor);
}

/// Dims*double multiplication
inline Dims operator*(const Dims& d, double factor)
{
  return Dims(int(d.w()*factor), int(d.h()*factor));
}

/// int*Dims multiplication
inline Dims operator*(int factor, const Dims& d)
{
  return Dims(d.w()*factor, d.h()*factor);
}

/// double*Dims multiplication
inline Dims operator*(double factor, const Dims& d)
{
  return Dims(int(d.w()*factor), int(d.h()*factor));
}

/// Dims/int division
inline Dims operator/(const Dims& d, int factor)
{
  return Dims(d.w()/factor, d.h()/factor);
}

/// Dims/double division
inline Dims operator/(const Dims& d, double factor)
{
  return Dims(int(d.w()/factor), int(d.h()/factor));
}

// ######################################################################
// ######################################################################
// INLINE member functions for Dims:
// ######################################################################
// ######################################################################

// ######################################################################
inline Dims::Dims() throw() :
  ww(0), hh(0)
{}

// ######################################################################
inline Dims::Dims(int width, int height) throw() :
  ww(width), hh(height)
{
  ASSERT(ww >= 0 && hh >= 0);
}

// ######################################################################
inline Dims::Dims(const Dims& other) throw() :
  ww(other.ww), hh(other.hh)
{}

// ######################################################################
inline int Dims::w() const throw()
{ return ww; }

// ######################################################################
inline int Dims::h() const throw()
{ return hh; }

// ######################################################################
inline int Dims::sz() const throw()
{ return ww*hh; }

// ######################################################################
inline int Dims::max() const throw()
{ return (ww > hh) ? ww : hh; }

// ######################################################################
inline int Dims::min() const throw()
{ return (ww < hh) ? ww : hh; }

// ######################################################################
inline bool Dims::isEmpty() const throw()
{ return sz() == 0; }

// ######################################################################
inline bool Dims::isNonEmpty() const throw() { return sz() > 0; }

// ######################################################################
inline bool Dims::operator==(const Dims& x) const throw()
{ return ww == x.ww && hh == x.hh; }

// ######################################################################
inline bool Dims::operator!=(const Dims& x) const throw()
{ return ww != x.ww || hh != x.hh; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
