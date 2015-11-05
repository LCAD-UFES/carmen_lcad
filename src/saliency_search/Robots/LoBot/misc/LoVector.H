/**
   \file Robots/LoBot/misc/LoVector.H

   \brief Some utility types and functions related to vectors and images
   of vectors.
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoVector.H $
// $Id: LoVector.H 13811 2010-08-21 02:00:08Z mviswana $
//

#ifndef LOBOT_VECTOR_DOT_H
#define LOBOT_VECTOR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/LoMath.H"

// INVT image support
#include "Image/Image.H"
#include "Image/Point2D.H"

// Standard C headers
#include <math.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//----------------------------- TYPEDEFS --------------------------------

/// Some parts of Robolocust rely on the notion of a vector (here, we are
/// talking about the mathematical variety, not the container type).
/// Although vectors and points are quite distinct concepts, we simply
/// piggyback off the INVT Point2D implementation for our vectors.
typedef Point2D<float> Vector ;

/// Some quick operators for vectors
//@{
inline Vector operator*(float k, const Vector& V)
{
   return Vector(k * V.i, k * V.j) ;
}

inline Vector operator*(const Vector& V, float k)
{
   return operator*(k, V) ;
}

inline Vector operator/(const Vector& V, float k)
{
   return is_zero(k) ? Vector() : operator*(1.0f/k, V) ;
}
//@}

/// Return the radial direction of the supplied vector.
inline float direction(const Vector& v)
{
   return atan(v.j, v.i) ;
}

/// Return the unit vector associated with the given radial direction.
inline Vector unit_vector(float dir)
{
   return Vector(cos(dir), sin(dir)) ;
}

/// Return the magnitude of the supplied vector.
inline float magnitude(const Vector& v)
{
   return sqrtf((v.i * v.i) + (v.j * v.j)) ;
}

/// Normalize the supplied vector and return the result.
inline Vector normalized(const Vector& v)
{
   return v/magnitude(v) ;
}

/// Return the dot product of two 2D vectors.
inline float dot(const Vector& a, const Vector& b)
{
   return a.i * b.i + a.j * b.j ;
}

/// Some parts of Robolocust need the notion of a collection of vectors
/// arranged in a 2D grid, i.e., an image of vectors.
typedef Image<Vector> VectorImage ;

/// From an image of vectors, extract the vector at pixel location (x,y).
inline const Vector& get_vector(const VectorImage& I, int x, int y)
{
   return I.getVal(x, y) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
