/**
   \file  Robots/LoBot/util/LoMath.H
   \brief Math functions.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoMath.H $
// $Id: LoMath.H 13628 2010-06-28 23:48:02Z mviswana $
//

#ifndef LOBOT_MATH_UTILITIES_DOT_H
#define LOBOT_MATH_UTILITIES_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <algorithm>
#include <limits>

// Standard C headers
#include <math.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//---------------------------- BASIC STUFF ------------------------------

/// The sign of a number
template<typename T>
int sign(T t)
{
   return (t < 0) ? -1 : +1 ;
}

/// The sign of a number: -1 for -ve, 0 for zero, +1 for +ve
template<typename T>
int sign0(T t)
{
   if (t < 0)
      return -1 ;
   if (t > 0)
      return +1 ;
   return 0 ;
}

/// Absolute value for different numeric types
template<typename T>
T abs(T t)
{
   return (t < 0) ? -t : t ;
}

/// Square of a number
template<typename T>
inline T sqr(T t)
{
   return t * t ;
}

/// Return the next power of two >= given number
int next_power_of_two(int n) ;

//-------------------------- FLOATING POINT -----------------------------

/// Check if a floating point number is near zero
inline bool is_zero(float t)
{
   return t > -std::numeric_limits<float>::epsilon()
       && t <  std::numeric_limits<float>::epsilon() ;
}

/// Check if a double precision floating point number is near zero
inline bool is_zero(double t)
{
   return t > -std::numeric_limits<double>::epsilon()
       && t <  std::numeric_limits<double>::epsilon() ;
}

/// Round a floating-point number to the nearest integer
int round(float t) ;

//----------------------------- CLAMPING --------------------------------

/// Clamp a given value to the specified range
template<typename T>
T clamp(T value, T min, T max)
{
   if (value < min)
      return min ;
   if (value > max)
      return max ;
   return value ;
}

/// Clamp a given value to the specified range
template<typename T>
inline T clamp(T value, const range<T>& R)
{
   return clamp(value, R.min(), R.max()) ;
}

/// Clamp the input range's min and max to the bounds range's min and max
template<typename T>
inline range<T> clamp(const range<T>& input, const range<T>& bounds)
{
   return range<T>(std::max(input.min(), bounds.min()),
                   std::min(input.max(), bounds.max())) ;
}

//------------------------------ ANGLES ---------------------------------

/// Clamp an angle (specified in degrees) to lie in [0, 360]
float clamp_angle(float angle) ;

/// Returns the quadrant to which the given angle belongs.
inline int quadrant(float angle)
{
   return static_cast<int>(clamp_angle(angle)/90) + 1 ;
}

// Returns the octant to which the given angle belongs.
//
// NOTE: Octants are counted starting at one (not zero). Octant #1 goes
// from zero to 44 degrees; octant #2 from 45 to 89 degrees; so on and so
// forth.
inline int octant(float angle)
{
   return static_cast<int>(clamp_angle(angle)/45) + 1 ;
}

//-------------------------- RANDOM NUMBERS -----------------------------

/// Return a random integer in the specified range.
///
/// NOTE: The range is inclusive, i.e., this function will return a
/// number in the range [min, max].
int random(int min, int max) ;

/// Return a random float in the specified range.
///
/// NOTE: The range is inclusive, i.e., this function will return a
/// number in the range [min, max].
float randomf(float min, float max) ;

//------------------------------- TRIG ----------------------------------

/// Trig functions for degrees rather than radians
//@{

// Sine
inline float sin(float angle)
{
   return sinf(angle * 0.01745329f) ; // angle * PI/180
}

inline double sin(double angle)
{
   return ::sin(angle * 0.0174532925199432) ; // angle * PI/180
}

inline float sin(int angle)
{
   return sin(static_cast<float>(angle)) ;
}

// Cosine
inline float cos(float angle)
{
   return cosf(angle * 0.01745329f) ; // angle * PI/180
}

inline double cos(double angle)
{
   return ::cos(angle * 0.0174532925199432) ; // angle * PI/180
}

inline float cos(int angle)
{
   return cos(static_cast<float>(angle)) ;
}

// Inverse sine
inline float asin(float x)
{
   return asinf(x) * 57.29577976f ; // 180/PI
}

inline double asin(double x)
{
   return ::asin(x) * 57.2957795130823215 ; // 180/PI
}

inline float asin(int x)
{
   return asin(static_cast<float>(x)) ;
}

// Inverse cosine
inline float acos(float x)
{
   return acosf(x) * 57.29577976f ; // 180/PI
}

inline double acos(double x)
{
   return ::acos(x) * 57.2957795130823215 ; // 180/PI
}

inline float acos(int x)
{
   return acos(static_cast<float>(x)) ;
}

// Inverse tangent
inline float atan(float x)
{
   return atanf(x) * 57.29577976f ; // 180/PI
}

inline double atan(double x)
{
   return ::atan(x) * 57.2957795130823215 ; // 180/PI
}

inline float atan(int x)
{
   return atan(static_cast<float>(x)) ;
}

// Inverse tangent considering signs of both components
inline float atan(float y, float x)
{
   return atan2f(y, x) * 57.29577976f ; // 180/PI
}

inline double atan(double y, double x)
{
   return ::atan2(y, x) * 57.2957795130823215 ; // 180/PI
}

inline float atan(int y, int x)
{
   return atan(static_cast<float>(y), static_cast<float>(x)) ;
}
//@}

//---------------------------- LOG AND EXP ------------------------------

/// Log functions for base 10
//@{
inline float log(float n)
{
   return log10f(n) ;
}

inline double log(double n)
{
   return log10(n) ;
}
//@}

/// Natural logarithms
//@{
inline float ln(float n)
{
   return logf(n) ;
}

inline double ln(double n)
{
   return ::log(n) ;
}
//@}

/// Exponential functions
//@{
inline float exp(float n)
{
   return expf(n) ;
}

inline double exp(double n)
{
   return ::exp(n) ;
}
//@}

/// These functions convert from normal [0,1] probabilities to log-odds
/// and vice versa.
//@{
float prob_to_log_odds(float p) ;
float log_odds_to_prob(float log_odds) ;
//@}

//--------------------------- MISCELLANEOUS -----------------------------

/// A function to evaluate the Gaussian
float gaussian(float x, float mu, float sigma) ;

/// A function object to perform Gaussian weighting
//@{
template<typename T = float>
class gaussian_weight {
   T mean ;
   T two_sigma_sqr, sqrt_2pi_sigma ;
public:
   gaussian_weight(T mean, T sigma) ;
   T operator()(T x, T w) const {
      return w * exp(-sqr(x - mean)/two_sigma_sqr)/sqrt_2pi_sigma ;
   }
} ;

template<typename T>
gaussian_weight<T>::gaussian_weight(T m, T sigma)
   : mean(m),
     two_sigma_sqr(2 * sqr(sigma)),
     sqrt_2pi_sigma(T(2.5066282746310004) * sigma)
{}
//@}

/// This function returns a random number sampled from a triangular
/// distribution centered at sigma.
float sample_tri(float sigma) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
