/*!@file Util/MathFunctions.H Miscellaneous math functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/MathFunctions.H $
// $Id: MathFunctions.H 15310 2012-06-01 02:29:24Z itti $
//

#ifndef MATHFUNCTIONS_H_DEFINED
#define MATHFUNCTIONS_H_DEFINED

#include "Util/Assert.H"
#include "Image/Image.H"
#include "Image/Point2D.H"
#include "Util/FastMathFunctions.H"
#include "Util/Promotions.H"
#include "rutz/compat_cmath.h" // for M_PI, M_E if not present in <cmath>

#include <algorithm> // for std::swap()
#include <cmath>

//! Miscellaneous math functions

// ######################################################################
/*! @Precomputed Constants 128 bits IEEE quad */
/*! Each constant is 128 bits IEEE quad. On a 64 bit system,
  a double is 128 bits. Thus, each number is a double on a modern system

  # define M_El                2.7182818284590452353602874713526625L

  is given for the value of 'e'. We follow this standard.

  Note:
*/
//@{
#define D_PI               3.1415926535897932384626433832795029
//! e
#define D_E                2.7182818284590452353602874713526625
//! Log Root of 2 Pi
#define D_LOG_SQRT_2_PI    0.9189385332046727417803297364056176
//! Root of 2*Pi
#define D_SQRT_2_PI        2.5066282746310002416123552393401041
//! Root of Pi
#define D_SQRT_PI          1.7724538509055160272981674833411451
//! Root of 2 DEFINED in cmath as M_SQRT2, but is not 128 bit
#define D_SQRT_2           1.4142135623730950488016887242096981
//! Natural Log of 2*Pi
#define D_LOG_2_PI         1.8378770664093454835606594728112353
//! Natural Log of Pi
#define D_LOG_PI           1.1447298858494001741434273513530587
//! Natural Log of 2 DEFINED in cmath as M_LN2, but is not 128 bit
#define D_LOG_2            0.6931471805599453094172321214581766
//! Natural Log of sqrt of 2
#define D_LOG_SQRT_2       0.3465735902799726547086160607290883
//! 2*Pi
#define D_2_PI             6.2831853071795862319959269370883703
//! The Euler-Mascheroni Constant (Not the same as e)
#define D_EULER_GAMMA      0.5772156649015328606065120900824024
//! Number of radians in one degree
#define D_DEGREE           0.0174532925199432957692369076848861
//! The golden ratio
#define D_GOLDEN_RATIO     1.6180339887498948482045868343656381
//! Catalan's Constant
#define D_CATALAN          0.9159655941772190150546035149323841
//! Glaisher-Kinkelin Constant
#define D_GLAISHER         1.2824271291006226368753425688697917
//! Khinchin's Constant
#define D_KHINCHIN         2.6854520010653064453097148354817956
//! Ramanujan-Soldner's Constant
#define D_SOLDNER          1.4513692348833810502839684858920274
//! Apery's Constant
#define D_APERY            1.2020569031595942853997381615114499
//! Gauss-Kuzmin-Wirsing constant
#define D_GAUSS_KUZMAN     0.3036630028987326585974481219015562
//! Fransén-Robinson constant
#define D_FRANSEN          2.8077702420285193652215011865577729
//! Sierpin'ski's constant
#define D_SERPINSKY        2.5849817595792532170658935873831711
//! Mills' constant
#define D_MILL             1.3063778838630806904686144926026057
//! Omega constant
#define D_OMEGA            0.5671432904097838729999686622103555
//! Golomb-Dickman constant
#define D_GOLOMB_DICKMAN   0.6243299885435508709929363831008372

// ######################################################################
/*! @Precomputed Constants 256 Bit*/
/*! Each constant is 256 bits, This makes it a double long floating point on any modern 64 bit
  CPU.
*/
//@{
//! Pi
#define L_PI            3.14159265358979323846264338327950288419716939937510582097494459230781L // ... 640628620899862803482534
//! e
#define L_E             2.71828182845904523536028747135266249775724709369995957496696762772407L // ... 663035354759457138
//! Log Root of 2 Pi
#define L_LOG_SQRT_2_PI 0.91893853320467274178032973640561763986139747363778341281715154048276L
//! Root of 2*Pi
#define L_SQRT_2_PI     2.50662827463100050241576528481104525300698674060993831662992357634229L
//! Root of Pi
#define L_SQRT_PI       1.77245385090551602729816748334114518279754945612238712821380778985291L
//! Root of 2 DEFINED in cmath as M_SQRT2, but is not > 128 bit
#define L_SQRT_2        1.41421356237309514547462185873882845044136047363281250000000000000000L
//! Natural Log of 2*Pi
#define L_LOG_2_PI      1.83787706640934548356065947281123527972279494727556682563430308096553L
//! Natural Log of Pi
#define L_LOG_PI        1.14472988584940017414342735135305871164729481291531157151362307147214L
//! Natural Log of 2 DEFINED in cmath as M_LN2, but is not > 128 bit
#define L_LOG_2         0.69314718055994528622676398299518041312694549560546875000000000000000L
//! 2*Pi
#define L_2_PI          6.28318530717958647692528676655900576839433879875021164194988918461560L
//! Number of radians in one degree
#define L_DEGREE        0.01745329251994329576923690768488612713442871888541725456097191440171L // ... 009114603449443682
//! The Euler-Mascheroni Constant (Not the same as e)
#define L_EULER_GAMMA   0.57721566490153286060651209008240243104215933593992359880576723488486L
//! The golden ratio
#define L_GOLDEN_RATIO  1.61803398874989484820458683436563811772030917980576286213544862270526L // ... 046281890244970721
//! Catalan's Constant
#define L_CATALAN       0.91596559417721901505460351493238411077414937428167213426649811962176L // ... 301977625476947936
//! Glaisher-Kinkelin Constant
#define L_GLAISHER      1.28242712910062263687534256886979172776768892732500119206374002174040L // ... 630885882646112974
//! Khinchin's Constant
#define L_KHINCHIN      2.68545200106530644530971483548179569382038229399446295305115234555721L // ... 885953715200280114
//@}

// ######################################################################
/*! @name Basic numerical operations */
//@{

//! Returns -1 if \a x is negative, or 1 if \a x is positve.
template <class T>
inline int signOf(const T& x);

//! Returns the square of \a x.
template <class T>
inline T squareOf(const T& x);

//! Returns the number closest to \a val and between \a bound1 and \a bound2.
/*! The bounds may be specified in either order (i.e., either (lo, hi) or
  (hi, lo). */
template <class T, class T2>
inline T clampValue(const T& val, const T2& bound1, const T2& bound2);

//@}

// ######################################################################
/*! @name Numerical range and finiteness testing */
//@{

//! Function to check for finite value
bool isFinite(const byte& arg);

//! Function to check for finite value
bool isFinite(const int16& arg);

//! Function to check for finite value
bool isFinite(const int32& arg);

//! Function to check for finite value
bool isFinite(const float& arg);

//! Function to check for finite value
bool isFinite(const double& arg);

//@}

// ######################################################################
/*! @name Random number generation */
//@{

//! Initialize random seeds from pseudo-random sources (e.g. time(), getpid())
void initRandomNumbers();

//! Initialize random seeds with zero, to produce deterministic pseudo-random sequences
void initRandomNumbersZero();

//! Generate a random double in [0.0,1.0)
/*! This is a thin wrapper around drand48() or rand(), depending on
    which is available on the build platform. */
double randomDouble();

//! Generate a random double from a normal distribution with mean zero and standard deviation s
/*! This function call randomDouble() 12 times to sample from normal distribution  */
double randomDoubleFromNormal(const double s);

//! Generate a random integer in [0,n] (i.e., including n)
/*! This is a thin wrapper around lrand48() or rand(), depending on
    which is available on the build platform. */
int randomUpToIncluding(const int n);

//! Generate a random integer in [0,n) (i.e., not including n)
/*! This is a thin wrapper around lrand48() or rand(), depending on
    which is available on the build platform. */
int randomUpToNotIncluding(const int n);

//! Randomly shuffle given array
template <class T> inline
void randShuffle(T* array, const uint size);

//! Random number generator from Numerical Recipes in C book
/*! Long period >2x10^18 random number generator of L Ecuyer with
    Bays-Durham shuffle. Return a uniform deviate in the interval
    (0,1).  Call with int variable as argument set to a negative
    value, and don't change this variable between calls unless you
    want to reseed the generator. You can get ain initial seed using
    the function getIdum() */
double ran2(int& idum);

//! Get a random seed for ran2
/*! If useRandom is true, a random negative number will be return,
  otherwise a negative number that is always the same will be
  returned. */
int getIdum(const bool useRandom = true);

//! Randomly distributed Gaussian deviate with zero mean and unit variance
double gasdev(int& idum);

//! Randomly distributed Exponential deviate
double expdev(int& idum);

//@}

// ######################################################################
/*! @name Special math functions with iterative implementations */
//@{

//! Compute probability of drawing a value k from a Poisson of mean mu
double poisson(const unsigned int k, const double mu);

//! Compute log of Gamma
double lngamma(double x);

//! Compute the psi (derivarive of log of gamma) function (requires x > 0.0)
double psi(const double x);


//@}

// ######################################################################
/*! @name Scalar math functions corresponding with Pixel math functions in Pixels.h */
//@{

template <class T> inline T maxmerge(T A, const T B)
{
  if (A < B) A = B;
  return A;
}

// ######################################################################
template <class T> inline T minmerge(T A, const T B)
{
  if (A > B) A = B;
  return A;
}

// ######################################################################
//! Secant
template <class T> inline T sec(T A)
{
  return clamped_convert<T>(1.0/cos(A));
}

// ######################################################################
//! Cosecant
template <class T> inline T cosec(T A)
{
  return clamped_convert<T>(1.0/sin(A));
}

// ######################################################################
//! Cotangent
template <class T> inline T cotan(T A)
{
  return clamped_convert<T>(1.0/tan(A));
}

// ######################################################################
//! inverse secant
template <class T> inline T asec(T A)
{
  ASSERT(A >= 1.0);
  return clamped_convert<T>(acos(1.0/A));
}

// ######################################################################
//! inverse cosecant
template <class T> inline T acosec(T A)
{
  ASSERT(A >= 1.0);
  return clamped_convert<T>(asin(1.0/A));
}

// ######################################################################
//! inverse cotangent
template <class T> inline T acotan(T A)
{
  return clamped_convert<T>(atan(1.0/A));
}

// ######################################################################
//! Hyperbolic Secant
template <class T> inline T sech(T A)
{
  return clamped_convert<T>(1.0/cosh(A));
}

// ######################################################################
//! Hyperbolic Cosecant
template <class T> inline T cosech(T A)
{
  return clamped_convert<T>(1.0/sinh(A));
}

// ######################################################################
//! Hyperbolic Cotangent
template <class T> inline T cotanh(T A)
{
  return clamped_convert<T>(1.0/tanh(A));
}

// ######################################################################
//! Inverse Hyperbolic Secant
template <class T> inline T asech(T A)
{
  return clamped_convert<T>(acosh(1.0/A));
}

// ######################################################################
//! Inverse Hyperbolic Cosecant
template <class T> inline T acosech(T A)
{
  return clamped_convert<T>(asinh(1.0/A));
}

// ######################################################################
//! Inverse Hyperbolic Cotangent
template <class T> inline T acotanh(T A)
{
  return clamped_convert<T>(atanh(1.0/A));
}

// ######################################################################
//! return the sign of this item
template <class T> inline T sign(T A)
{
  if (A >= 0) A = 1;
  else A = -1;
  return A;
}

// ######################################################################
//! logN for pixel
template <class T> inline T logN(T A, const T n)
{
  A = clamped_convert<T>(
      log(static_cast<double>(A))/
      log(static_cast<double>(n)));
  return A;
}

// ######################################################################
//! log sigmoid for pixel
template <class T> inline T logsig(T A,const T a, const T b)
{
  A = clamped_convert<T>(1.0/(1+exp(static_cast<double>(- (b * (A - a))))));
  return A;
}

// ######################################################################
//! log sigmoid for pixel with offset
/*! Values like a = 10 and b = 5 create a nice sigmoid for x = 0 to 1 since
    the center of the sigmoid is at b/a.
    @param a The slope for the sigmoid
    @param b The offset for the sigmoid
*/
template <class T> inline T logsig2(T x, const T a, const T b)
{
  x = clamped_convert<T>(1.0/(1+exp(static_cast<double>(-x*a + b))));
  return x;
}

// ######################################################################
//! tan sigmoid for pixel
template <class T> inline T tansig(T A)
{
  A = clamped_convert<T>(2/1+exp(-2*static_cast<double>(A))-1);
  return A;
}

// ######################################################################
//! Angle difference, which is the minimum circular difference between two angles. Angles are in rad. Only for angles from 0 to diffWRAP
template <class T> inline T angDiff(const T A,const T B, const double diffWRAP=M_PI)
{
  double diff = fabs(static_cast<double>(A) - static_cast<double>(B));
  return std::min(diff, fabs(diff-diffWRAP));
}

//! Find the cosine of the angle between two vectors (p0,p1) and (p0,p2)
template <class T> inline double angle(const Point2D<T> p0, const Point2D<T> p1, const Point2D<T> p2)
{

  double dx1 = p1.i - p0.i;
  double dy1 = p1.j - p0.j;
  double dx2 = p2.i - p0.i;
  double dy2 = p2.j - p0.j;
  
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}



// ######################################################################
//! saturate pixel at some value
template <class T> inline T saturate(T A, const T n)
{
  if(A >= n) A = n;
  else A = A;
  return A;
}

// ######################################################################
//! find p from a Gaussian distribution
template <class T> inline T gauss(T A, const T Mu, const T Std)
{
  return clamped_convert<T>(
      (1/(fastSqrt(D_2_PI*pow(static_cast<double>(Std),2))))
      *exp(-1*(pow((static_cast<double>(A) -
                    static_cast<double>(Mu)),2)
               /(2*pow(static_cast<double>(Std),2)))));
}

//! find p from a Gaussian distribution
//! (instead of the mu and A, the difference is supplied)
template <class T> inline T gauss(T diff, const T Std)
{
  return clamped_convert<T>(
      (1/(fastSqrt(D_2_PI*pow(static_cast<double>(Std),2))))
      *exp(-1*(pow((static_cast<double>(diff)),2)
               /(2*pow(static_cast<double>(Std),2)))));
}

// ######################################################################
//! find p from a gamma distrabution
template <class T> inline T gamma(T X, const T A, const T B)
{
  return clamped_convert<T>((
                             pow(static_cast<double>(X),static_cast<double>(A) -1) *
                             pow(static_cast<double>(B),static_cast<double>(A))    *
                             exp(static_cast<double>(-1.0*B*X)))/
                            lgamma(static_cast<double>(A)));
}

// ######################################################################
//! this nonsense operator is for compatability
template <class T> inline T max(T A)
{
  return A;
}

// ######################################################################
//! this nonsense operator is for compatability
template <class T> inline T min(T A)
{
  return A;
}

// ######################################################################
//! find if this value or pixel is bounded by the other two
template <class T> inline bool isBounded(T A, const T upper, const T lower)
{
  return ((A >= lower) && (A <= upper));
}

// ######################################################################
//! this nonsense operator is for compatability
template <class T> inline T sum(T A)
{
  return A;
}

// ######################################################################
//! this nonsense operator is for compatability
template <class T> inline T mean(T A)
{
  return A;
}

//@}


// ######################################################################
//compute the stddev and mean of each feature
//This algorithm is due to Knuth (The Art of Computer Programming, volume 2:
//  Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.a
//
//! Compute the mean online by supplying the current mean and a new value
//! mu is the previous mean, A is the current value and n is the number of values proceses so far
template <class T> inline T onlineMean(T mu, T A, int n)
{
  const T delta = A - mu;
  return mu + (delta/n);
}

template <class T> inline T onlineStdev(T mu, T stdev, T A, int n)
{
  T ret = stdev;
  if (n > 2) //watch for divide by 0
  {
    const T delta = A - mu;
    ret = (stdev * (n-2)) + delta*delta;
    ret /= T(n-1);
  }

  return ret;
}

// ######################################################################
// ######################################################################
// ######################################################################
// ##### Implementation of inlined functions
// ######################################################################
// ######################################################################
// ######################################################################

template <class T> inline int signOf(const T& x)
{ return (x < 0) ? -1 : 1; }

template <class T> inline T squareOf(const T& x)
{ return (x * x); }

template <class T, class T2>
inline T clampValue(const T& val, const T2& bound1, const T2& bound2)
{
  if (bound1 < bound2)
    {
      if (val < bound1) return bound1;
      if (val > bound2) return bound2;
      return val;
    }
  else
    {
      if (val < bound2) return bound2;
      if (val > bound1) return bound1;
    }
  return val;
}

template <class T> inline void randShuffle(T* array, const uint size)
{
  for (uint i = 0; i < size; ++i)
    {
      const uint idx = i + randomUpToNotIncluding(size-i);
      std::swap(array[i], array[idx]);
    }
}

// ######################################################################
// ##### Digamma (psi) function
// ######################################################################
// Code for computation of psi function (derivative of log gamma)
// ripped from GSL, the GNU scientific library
//
/* Copyright (C) 1996, 1997, 1998, 1999, 2000 Gerard Jungman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

// ######################################################################
/* data for a Chebyshev series over a given interval */
struct cheb_series_struct {
  double * c;   /* coefficients                */
  int order;    /* order of expansion          */
  double a;     /* lower interval point        */
  double b;     /* upper interval point        */
  int order_sp; /* effective single precision order */
};
typedef struct cheb_series_struct cheb_series;

// ######################################################################
/* Chebyshev fits from SLATEC code for psi(x) */
static double psics_data[23] = {
  -.038057080835217922,
   .491415393029387130,
  -.056815747821244730,
   .008357821225914313,
  -.001333232857994342,
   .000220313287069308,
  -.000037040238178456,
   .000006283793654854,
  -.000001071263908506,
   .000000183128394654,
  -.000000031353509361,
   .000000005372808776,
  -.000000000921168141,
   .000000000157981265,
  -.000000000027098646,
   .000000000004648722,
  -.000000000000797527,
   .000000000000136827,
  -.000000000000023475,
   .000000000000004027,
  -.000000000000000691,
   .000000000000000118,
  -.000000000000000020
};
const static cheb_series psi_cs = { psics_data, 22, -1, 1, 17 };

// ######################################################################
static double apsics_data[16] = {
  -.0204749044678185,
  -.0101801271534859,
   .0000559718725387,
  -.0000012917176570,
   .0000000572858606,
  -.0000000038213539,
   .0000000003397434,
  -.0000000000374838,
   .0000000000048990,
  -.0000000000007344,
   .0000000000001233,
  -.0000000000000228,
   .0000000000000045,
  -.0000000000000009,
   .0000000000000002,
  -.0000000000000000
};
const static cheb_series apsi_cs = { apsics_data, 15, -1, 1, 9 };

// ######################################################################
// Ripped from GSL, the GNU scientific library, function
// cheb_eval_e(); modified to eliminate the error computation and just
// return the result:

inline double cheb_eval(const cheb_series* cs, const double x)
{
  double d  = 0.0, dd = 0.0;

  const double y  = (2.0*x - cs->a - cs->b) * (1.0/(cs->b - cs->a));
  //Was: const double y  = (2.0*x - cs->a - cs->b) / (cs->b - cs->a);
  const double y2 = 2.0 * y;

  for (uint j = cs->order; j >= 1; j--)
    {
      const double temp = d;
      d = y2*d - dd + cs->c[j];
      dd = temp;
    }

  return y*d - dd + 0.5 * cs->c[0];
}

// ######################################################################
// This is the Chebyshev evaluation manually speciallized. This is *probably*
// faster than the non specialized version since no structures are used and
// there are no pointers. This depends on the compiler and flags used. At
// worst it's just as fast as the non specialized version

inline double cheb_eval_psi(const double x)
{
  double d  = 0.0, dd = 0.0;

  const double y2 = 2.0 * x;

  for (uint j = 22; j >= 1; j--)
    {
      const double temp = d;
      d = y2*d - dd + psics_data[j];
      dd = temp;
    }

  return x*d - dd + 0.5 * -.038057080835217922;
}

inline double cheb_eval_apsi(const double x)
{
  double d  = 0.0, dd = 0.0;

  const double y2 = 2.0 * x;

  for (uint j = 15; j >= 1; j--)
    {
      const double temp = d;
      d = y2*d - dd + apsics_data[j];
      dd = temp;
    }

  return x*d - dd + 0.5 * -.0204749044678185;
}

// ######################################################################
// Ripped from GSL, the GNU scientific library, function psi_x();
// modified to eliminate the error computation and just return the
// result:
inline double psi(const double x)
{
  if (x >= 2.0)
    {
      /*
      const double d    = 1.0 / x;
      const double t    = 8.0 * d * d - 1.0;
      const double cheb = cheb_eval(&apsi_cs, t);
      return fastLog(x) - 0.5 * d + cheb;
      */
      const double t    = 8.0 / (x*x) - 1.0;
      const double cheb = cheb_eval(&apsi_cs, t);
      return fastLog(x) - 0.5 / x + cheb;
    }
  else if (x < 1.0)
    {
      if( x <= 0.0)
        LFATAL("Digamma (Psi) Only defined for x > 0.0");

      const double t1   = 1.0 / x;
      const double cheb = cheb_eval(&psi_cs, 2.0*x-1.0);
      return cheb - t1;
    }
  else
    {
      const double v = x - 1.0;
      return cheb_eval(&psi_cs, 2.0*v-1.0);

    }
}

// ######################################################################
// Somewhat faster psi. dumps error checking and structure usage.
inline double fast_psi(const double x)
{
  if (x >= 2.0)
    {
      const double t    = 8.0 / (x*x) - 1.0;
      const double cheb = cheb_eval_apsi(t);
      return fastLog(x) - 0.5 / x + cheb;
    }
  else if (x < 1.0)
    {
      const double t1   = 1.0 / x;
      const double cheb = cheb_eval_psi(2.0*x-1.0);
      return cheb - t1;
    }
  else
    {
      const double v = x - 1.0;
      return cheb_eval_psi(2.0*v-1.0);
    }
}

// ######################################################################
// ##### Kullback-Leibler (KL) distances
// ######################################################################
//! Convert a number into Wows, for instance a KL distance
/*! Wow's in this computation is the same as the conversion
  from Nats to Bits in KL
*/
template <class T> inline T wow(const T& k)
{
  // surprise should always be positive but rounding may mess that up:
  // This is OK since surprise is negative with equality if and only
  // if two distribution functions are equal. Thus, we gain no insite
  // with negative surprise.

  if (k < 0.0) return 0.0;
  // return the surprise, in units of wows (KL Bits):
  return k * (1.0F/D_LOG_2);
}

// ######################################################################
//! Compute the KL distance for a joint gamma PDF
/*!
  @param ai New Alpha in model i
  @param aj New Alpha in model j
  @param bi New Beta  in model i
  @param bj New Beta  in model j
  @param Ai Current Alpha in model i
  @param Aj Current Alpha in model j
  @param Bi Current Beta  in model i
  @param Bj Current Beta  in model j
  @param doWow is true then convert output to Wows

  This formula derived by T. Nathan Mundhenk 2007
*/
template <class T> inline T KLjointGammaGamma(const T ai,const T aj,
                                              const T bi,const T bj,
                                              const T Ai,const T Aj,
                                              const T Bi,const T Bj,
                                              const bool doWow = false,
                                              const T gammaBiasi = 1.0F,
                                              const T gammaBiasj = 1.0F)
{

  /* We will compute the joint KL for P(x), P(X), P(y) and P(Y) where
     they are gamma distributions.

                               P(X) P(Y)
  KL = Integrate[P(x) P(y) log[---------] ]
                               P(x) P(y)

     ai - New Alpha in model i
     aj - New Alpha in model j
     bi - New Beta  in model i
     bj - New Beta  in model j

     Ai - Current Alpha in model i
     Aj - Current Alpha in model j
     Bi - Current Beta  in model i
     Bj - Current Beta  in model j

                     ai Bi   aj Bj          bi           bj
  KL =    -ai - aj + ----- + ----- - Ai Log[--] - Aj Log[--] +
                      bi      bj            Bi           Bj

         Gamma[Ai] Gamma[Aj]
>    Log[-------------------] - (-ai + Ai) PolyGamma[0, ai] -
         Gamma[ai] Gamma[aj]

>    (-aj + Aj) PolyGamma[0, aj]


  Note: Since the pure Gamma function can get large very quickly, we avoid
  large numbers by computing the log gamma directly as lgamma.

  */

  // Simplified a little from the above:
  const T k = gammaBiasi * KLgamma(ai,bi,Ai,Bi) + gammaBiasj * KLgamma(aj,bj,Aj,Bj);

  if(doWow) return wow(k);
  else      return k;
}

// ######################################################################
//! Compute the KL distance for a single gamma PDF
/*!
  @param a New Alpha
  @param b New Beta
  @param A Current Alpha
  @param B Current Beta
  @param doWow is true then convert output to Wows
*/
template <class T> inline T KLgamma(const T a,const T b,
                                    const T A,const T B,
                                    const bool doWow = false)
{
  /* We will compute the joint KL for P(x), P(X), P(y) and P(Y)
     where one is a gamma distribution and the other gaussian as:

     surprise is KL(new || old):

                          P(X)
  KL = Integrate[P(x) log[----] ]
                          P(x)

     a - New Alpha
     b - New Beta
     A - Current Alpha
     B - Current Beta

               a B         b        Gamma[A]
  KL    = -a + --- + A Log[-] + Log[--------] + a PolyGamma[0, a] -
                b          B        Gamma[a]

>    A * PolyGamma[0, a]

  Note: Since the pure Gamma function can get large very quickly, we avoid
  large numbers by computing the log gamma directly as lgamma.
  */

  const T k =  - a + a*B/b + A*fastLog(b/B) + lgamma(A) - lgamma(a)
               + (a - A)*fast_psi(a);

  if(doWow) return wow(k);
  else      return k;
}


// ######################################################################
//! Compute the KL distance for a single gauss PDF
/*!
  @param u New Mean
  @param s New Std.
  @param U Current Mean
  @param S Current Std.
  @param doWow is true then convert output to Wows
*/
template <class T> inline T KLgauss(const T u,const T s,
                                    const T U,const T S,
                                    const bool doWow = false)
{
  /* Compute the KL distance between two Gaussian PDFs as:

                          P(X)
  KL = Integrate[P(x) log[----] ]
                          P(x)

    u - New Mean
    s - New Std.
    U - Current Mean
    S - Current Std.

                      2            2
             -1     s      (u - U)        s
  KL =    - (--- + ----- + -------- - Log[-])
              2        2         2        S
                    2 S       2 S
  */

  const T iS = 1.0 / 2*S*S;
  const T x  = s*s * iS;
  const T mm = (u - U)*(u - U);
  const T k  = -1.0 * (x - 0.5 - log(s/S) + mm * iS);

  if(doWow)
    return wow(k);
  else
    return k;
}

// ######################################################################
//! Compute the KL distance for a joint gamma gauss PDF
/*!
  @param a  New Alpha in model 1
  @param b  New Beta  in model 1
  @param u  New Mean  in model 2
  @param s  New Std.  in model 2
  @param A  Current Alpha in model 1
  @param B  Current Beta  in model 1
  @param U  Current Mean  in model 2
  @param S  Current Std.  in model 2
  @param doWow is true then convert output to Wows

  This formula derived by T. Nathan Mundhenk 2007
*/
template <class T> inline T KLjointGammaGauss(const T a,const T u,
                                              const T b,const T s,
                                              const T A,const T U,
                                              const T B,const T S,
                                              const bool doWow = false,
                                              const T gammaBias = 1.0F,
                                              const T gaussBias = 1.0F)
{

  /* We will compute the joint KL for P(x), P(X), P(y) and P(Y)
     where one is a gamma distribution and the other gaussian as:

                               P(X) P(Y)
  KL = Integrate[P(x) P(y) log[---------] ]
                               P(x) P(y)

     a - New Alpha in model 1
     b - New Beta  in model 1
     u - New Mean  in model 2
     s - New Std.  in model 2

     A - Current Alpha in model 1
     B - Current Beta  in model 1
     U - Current Mean  in model 2
     S - Current Std.  in model 2

                              2           2
            1        a B    s     (u - U)          B        s
        = -(-) + a - --- + ---- + -------- + A Log[-] - Log[-] +
            2         b       2        2           b        S
                           2 S      2 S

         Gamma[a]
>    Log[--------] - (a - A) PolyGamma[0, a]
         Gamma[A]
*/

  const T k = gammaBias * KLgamma(a,b,A,B) + gaussBias * KLgauss(u,s,U,S);

  if(doWow) return wow(k);
  else      return k;
}

// ######################################################################
//! Compute the KL distance for a single gamma PDF
/*!
  This method produces the best results for usage with RSVP data as measured
  by the MAG when using the example values for C1 and C2. It is also faster
  than the basic KL computation since we eliminate two divisions and one log
  computation.

  @param a New Alpha
  @param A Current Alpha
  @param C1 the const standin for B/b - 1
  @param C2 the const standin for log(b/B)
  @param doWow is true then convert output to Wows
*/
template <class T> inline T KLgammaConst(const T a,  const T A,
                                         const T C1, const T C2,
                                         const bool doWow = false)
{
  /* We will compute the joint KL for P(x), P(X), P(y) and P(Y)
     where one is a gamma distribution and the other gaussian as:

     surprise is KL(new || old):

                          P(X)
  KL = Integrate[P(x) log[----] ]
                          P(x)

     a - New Alpha
     A - Current Alpha

                                     Gamma[A]
  KL    = -a + a * C1 + A * C2 + Log[--------] + a PolyGamma[0, a] -
                                     Gamma[a]

>    A * PolyGamma[0, a]

  This simplifies a little to:

         k =  a*C1 + A*C2 + lgamma(A) - lgamma(a) + (a - A)*psi(a)

  NOTES: (1) Psi is the same as the first order PolyGamma
         (2) lgamma(A) - lgamma(a) is a simplification of the Log gamma part

  Note: Since the pure Gamma function can get large very quickly, we avoid
  large numbers by computing the log gamma directly as lgamma.

  Examples of good values for C1 and C2 are:

         C1 = D_SQRT_2/2.0F - 1.0F  (i.e. sqrt(2)/2 - 1 )
         C2 = D_LOG_SQRT_2          (i.e. log(sqrt(2))  )

  Also notice, if a = A then:

         k = a*(C1 + C2)
  */

  const T k =  a*C1 + A*C2 + lgamma(A) - lgamma(a) + (a - A)*fast_psi(a);

  if(doWow) return wow(k);
  else      return k;
}

// ######################################################################
//! Compute the KL distance for a single gamma PDF
/*!
  Here we fix beta to be 1. This is a special case, but if we have it
  we can compute KL much faster. We can also use this if beta1 = beta2
  The Chi Square is one such case where beta is 1/2 .

  @param a New Alpha
  @param A Current Alpha
  @param doWow is true then convert output to Wows
*/
template <class T> inline T KLgamma(const T a,const T A,
                                    const bool doWow = false)
{
  /*
  Note: Since the pure Gamma function can get large very quickly, we avoid
  large numbers by computing the log gamma directly as lgamma.
  */

  const T k = lgamma(A) - lgamma(a) + (a - A)*fast_psi(a);

  if(doWow) return wow(k);
  else      return k;
}

// ######################################################################
//! Definition for a gaussian used for GMM

struct GaussianDef
{
  double weight;
  double mu;
  double sigma;

  GaussianDef(double w, double m, double s) :
    weight(w), mu(m), sigma(s)
  {}

};

// ######################################################################
//! Compute the L2 distance between two mixtures of Gaussians 
/*!
  @param f GMM1
  @param g GMM2
*/
inline double L2GMM(const std::vector<GaussianDef>& f,
                    const std::vector<GaussianDef>& g)
{
  /* Compute Closed-form expression for the L2 distance between two mixtures of
     Gaussians.
    From: B. Jian and B. Vemuri, “A robust algorithm for point set registration
    using mixtures of guassians,” in International Conference on Com-
    puter Vision, ICCV’05, Beijing, China, 2005.

    L2(f,g) = integ(f(x)-g(x))^2 dx 

    where f(x) and g(x) are GMM

  */


  double distff = 0;
  for(uint i=0; i<f.size(); i++)
    for(uint j=0; j<f.size(); j++)
      distff += f[i].weight*f[j].weight * gauss(0.0, f[i].mu-f[j].mu, f[i].sigma+f[j].sigma);

  double distgg = 0;
  for(uint i=0; i<g.size(); i++)
    for(uint j=0; j<g.size(); j++)
      distgg += g[i].weight*g[j].weight * gauss(0.0, g[i].mu-g[j].mu, g[i].sigma+g[j].sigma);

  double distfg = 0;
  for(uint i=0; i<f.size(); i++)
    for(uint j=0; j<g.size(); j++)
      distfg += f[i].weight*g[j].weight * gauss(0.0, f[i].mu-g[j].mu, f[i].sigma+g[j].sigma);


  return distff + distgg - 2*distfg;
}

// return the AUC of two vectors
double AUC(const float* model, const float* rand,
           size_t sm, size_t sr,
           const double step = 0.1);


///// Superquadrics helper functions
template <class T> inline Point2D<T> ellipsoid(const T a, const T b, const T e, const T theta)
{

  T x = clamped_convert<T>(a * sign(cos(theta)) * pow(fabs(static_cast<double>(cos(theta))), e));
  T y = clamped_convert<T>(b * sign(sin(theta)) * pow(fabs(static_cast<double>(sin(theta))), e));

  return Point2D<T>(x,y);

}

//! Convert from a covariance matrix to the major/minor axis of an ellipse
Point2D<float> getEllipseFromCov(const Image<double>& cov);


//! A table for returning the chi2inv at 95% (should be expanded as needed)
double inline getChi2Inv(int k)
{
  switch (k)
  {
    case 2: return 5.991464547107980e+00;
    case 3: return 7.814727903251160e+00;
    case 6: return 1.259158724374398e+01;
    default : LFATAL("Unknown value for k=%i", k);
  }

  return 0;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
