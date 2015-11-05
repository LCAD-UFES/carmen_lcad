/*!@file Image/Pixels.H Basic pixel types version 2.0 */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Pixels.H $
// $Id: Pixels.H 12598 2010-01-19 03:03:03Z dberg $
//

#ifndef PIXELS_H_DEFINED
#define PIXELS_H_DEFINED

#include "Util/MathFunctions.H"
#include "Image/PixelsTypes.H"
#include <cmath>
#include <string> // for string conversions

// ######################################################################
// ######################################################################
// FREE FUNCTION PIXEL MATH METHODS
// ######################################################################
// ######################################################################

namespace pix_helper
{
  typedef double (unary_math_func)(double);

  template <class P>
  inline P do_unary_func(P pixel, unary_math_func* func)
  {
    /* We use static_cast to make sure the compiler goes ahead and
       figures out how to cast this data type to a double at compile
       time. This should be trivial in almost all instances.  */

    for (size_t i = 0; i < pixel.myDim; ++i)
      pixel.p[i] =
        clamped_convert<typename P::ScalarType>
        (func(static_cast<double>(pixel.p[i])));

    return pixel;
  }
}

#define PIX_FREE_MATH_FUNCTIONS(PIX, TPARAMS)                           \
/* ############################################################ */      \
/* element-wise unary math functions */                                 \
template <TPARAMS> inline PIX                                           \
abs(PIX A) { return pix_helper::do_unary_func(A, &::fabs); }            \
                                                                        \
template <TPARAMS> inline PIX                                           \
sqrt(PIX A) { return pix_helper::do_unary_func(A, &::sqrt); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
floor(PIX A) { return pix_helper::do_unary_func(A, &::floor); }         \
                                                                        \
template <TPARAMS> inline PIX                                           \
ceil(PIX A) { return pix_helper::do_unary_func(A, &::ceil); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
round(PIX A) { return pix_helper::do_unary_func(A, &::round); }         \
                                                                        \
template <TPARAMS> inline PIX                                           \
log(const PIX A) { return pix_helper::do_unary_func(A, &::log); }       \
                                                                        \
template <TPARAMS> inline PIX                                           \
log10(PIX A) { return pix_helper::do_unary_func(A, &::log10); }         \
                                                                        \
template <TPARAMS> inline PIX                                           \
exp(PIX A) { return pix_helper::do_unary_func(A, &::exp); }             \
                                                                        \
template <TPARAMS> inline PIX                                           \
erf(PIX A) { return pix_helper::do_unary_func(A, &::erf); }             \
                                                                        \
template <TPARAMS> inline PIX                                           \
erfc(PIX A) { return pix_helper::do_unary_func(A, &::erfc); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
sign(PIX A) { return pix_helper::do_unary_func(A, &::sign); }           \
                                                                        \
/* ############################################################ */      \
/* element-cumulative unary math functions */                           \
template <TPARAMS>                                                      \
inline T max(PIX A)                                                     \
{                                                                       \
  T a = A.p[0];                                                         \
  for (size_t i = 1; i < A.myDim; i++)                                  \
    if (A.p[i] > a) a = A.p[i];                                         \
                                                                        \
  return a;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS>                                                      \
inline T min(PIX A)                                                     \
{                                                                       \
  T a = A.p[0];                                                         \
  for (size_t i = 1; i < A.myDim; i++)                                  \
    if (A.p[i] < a) a = A.p[i];                                         \
                                                                        \
  return a;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS>                                                      \
inline bool isFinite(PIX A)                                             \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    if (!isFinite(A.p[i])) return false;                                \
                                                                        \
  return true;                                                          \
}                                                                       \
                                                                        \
template <TPARAMS>                                                      \
inline T sum(PIX A)                                                     \
{                                                                       \
  T sum = 0;                                                            \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    sum = A.p[i] + sum;                                                 \
                                                                        \
  return sum;                                                           \
}                                                                       \
                                                                        \
template <TPARAMS>                                                      \
inline T mean(PIX A)                                                    \
{                                                                       \
  T sum = 0;                                                            \
  for (size_t i = 0; i < A.myDim; i++)                                  \
  {                                                                     \
    sum = A.p[i] + sum;                                                 \
  }                                                                     \
                                                                        \
  return clamped_convert<T>(sum/double(A.myDim));                       \
}                                                                       \
                                                                        \
/* ############################################################ */      \
/* basic trig functions */                                              \
template <TPARAMS> inline PIX                                           \
sin(PIX A) { return pix_helper::do_unary_func(A, &::sin); }             \
                                                                        \
template <TPARAMS> inline PIX                                           \
cos(PIX A) { return pix_helper::do_unary_func(A, &::cos); }             \
                                                                        \
template <TPARAMS> inline PIX                                           \
tan(PIX A) { return pix_helper::do_unary_func(A, &::tan); }             \
                                                                        \
template <TPARAMS> inline PIX                                           \
sec(PIX A) { return pix_helper::do_unary_func(A, &::sec); }             \
                                                                        \
template <TPARAMS> inline PIX                                           \
cosec(PIX A) { return pix_helper::do_unary_func(A, &::cosec); }         \
                                                                        \
template <TPARAMS> inline PIX                                           \
cotan(PIX A) { return pix_helper::do_unary_func(A, &::cotan); }         \
                                                                        \
/* ############################################################ */      \
/* inverse trig functions */                                            \
template <TPARAMS> inline PIX                                           \
asin(PIX A) { return pix_helper::do_unary_func(A, &::asin); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
acos(PIX A) { return pix_helper::do_unary_func(A, &::acos); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
atan(PIX A) { return pix_helper::do_unary_func(A, &::atan); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
asec(PIX A) { return pix_helper::do_unary_func(A, &::asec); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
acosec(PIX A) { return pix_helper::do_unary_func(A, &::acosec); }       \
                                                                        \
template <TPARAMS> inline PIX                                           \
acotan(PIX A) { return pix_helper::do_unary_func(A, &::acotan); }       \
                                                                        \
/* ############################################################ */      \
/* hyperbolic trig functions */                                         \
template <TPARAMS> inline PIX                                           \
sinh(PIX A) { return pix_helper::do_unary_func(A, &::sinh); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
cosh(PIX A) { return pix_helper::do_unary_func(A, &::cosh); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
tanh(PIX A) { return pix_helper::do_unary_func(A, &::tanh); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
sech(PIX A) { return pix_helper::do_unary_func(A, &::sech); }           \
                                                                        \
template <TPARAMS> inline PIX                                           \
cosech(PIX A) { return pix_helper::do_unary_func(A, &::cosech); }       \
                                                                        \
template <TPARAMS> inline PIX                                           \
cotanh(PIX A) { return pix_helper::do_unary_func(A, &::cotanh); }       \
                                                                        \
/* ############################################################ */      \
/* inverse hyperbolic trig functions */                                 \
template <TPARAMS> inline PIX                                           \
asinh(PIX A) { return pix_helper::do_unary_func(A, &::asinh); }         \
                                                                        \
template <TPARAMS> inline PIX                                           \
acosh(PIX A) { return pix_helper::do_unary_func(A, &::acosh); }         \
                                                                        \
template <TPARAMS> inline PIX                                           \
atanh(PIX A) { return pix_helper::do_unary_func(A, &::atanh); }         \
                                                                        \
template <TPARAMS> inline PIX                                           \
asech(PIX A) { return pix_helper::do_unary_func(A, &::asech); }         \
                                                                        \
template <TPARAMS> inline PIX                                           \
acosech(PIX A) { return pix_helper::do_unary_func(A, &::acosech); }     \
                                                                        \
template <TPARAMS> inline PIX                                           \
acotanh(PIX A) { return pix_helper::do_unary_func(A, &::acotanh); }     \
                                                                        \
/* ############################################################ */      \
/* other binary or ternary math functions */                            \
template <TPARAMS> inline PIX                                           \
maxmerge(PIX A, const PIX B)                                            \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    if (A.p[i] < B.p[i]) A.p[i] = B.p[i];                               \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
minmerge(PIX A, const PIX B)                                            \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    if (A.p[i] > B.p[i]) A.p[i] = B.p[i];                               \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
pow(PIX A, const T val)                                                 \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = clamped_convert<T>(pow(A.p[i], val));                      \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
pow(PIX A, const PIX B)                                                 \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = clamped_convert<T>(pow(A.p[i], B.p[i]));                   \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
rand(PIX A,const int low, const int high)                               \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = clamped_convert<T>(1+(high*rand()                          \
                               /(RAND_MAX+low)));                       \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
logN(PIX A, const T n)                                                  \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = logN(A.p[i], n);                                           \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
logsig(PIX A, const T a, const T b)                                     \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = logsig(A.p[i], a, b);                                      \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
tansig(PIX A)                                                           \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = tansig(A.p[i]);                                            \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
saturate(PIX A, const T n)                                              \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = saturate(A.p[i], n);                                       \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
poly(PIX A, const T* pn, const size_t n)                                \
{                                                                       \
  PIX B(0);                                                             \
  for (size_t i = 0; i < A.myDim; ++i)                                  \
    for (size_t j = 0; j < n; ++j)                                      \
      B.p[i] += clamped_convert<T>(double(pn[j]) * pow(A.p[i],j));      \
                                                                        \
  return B;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
gauss(PIX A, const PIX Mu, const PIX Std)                               \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = gauss(A.p[i], Mu.p[i], Std.p[i]);                          \
                                                                        \
  return A;                                                             \
}                                                                       \
                                                                        \
template <TPARAMS> inline PIX                                           \
gauss(PIX A, const T Mu, const T Std)                                   \
{                                                                       \
  for (size_t i = 0; i < A.myDim; i++)                                  \
    A.p[i] = gauss(A.p[i], Mu, Std);                                    \
                                                                        \
  return A;                                                             \
}


PIX_FREE_MATH_FUNCTIONS(PixRGB<T>, class T)
PIX_FREE_MATH_FUNCTIONS(PixHSV<T>, class T)
PIX_FREE_MATH_FUNCTIONS(PixJpegYUV<T>, class T)
PIX_FREE_MATH_FUNCTIONS(PixVideoYUV<T>, class T)
PIX_FREE_MATH_FUNCTIONS(PixYIQ<T>, class T)
PIX_FREE_MATH_FUNCTIONS(PixH2SV1<T>, class T)
PIX_FREE_MATH_FUNCTIONS(PixH2SV2<T>, class T)
PIX_FREE_MATH_FUNCTIONS(PixH2SV3<T>, class T)

#define LIST_2(p1, p2) p1, p2
PIX_FREE_MATH_FUNCTIONS(PixHyper<LIST_2(T, dim)>, LIST_2(class T, size_t dim))
#undef LIST_2

#undef PIX_FREE_MATH_FUNCTIONS


//! PixRGB<byte> overload
/*! Format is "<byte>,<byte>,<byte>" for R, G, B */
std::string convertToString(const PixRGB<byte>& val);
//! PixRGB<byte> overload
/*! Format  is "<byte>,<byte>,<byte>" for  R, G, B, or  "#RRGGBB" with
  the RR, GG and BB values in hexadecimal. */
void convertFromString(const std::string& str, PixRGB<byte>& val);

//! PixHSV<byte> overload
/*! Format is "<byte>,<byte>,<byte>" for H, S, V */
std::string convertToString(const PixHSV<byte>& val);
//! PixHSV<byte> overload
/*! Format  is "<byte>,<byte>,<byte>" for  H, S, V */
void convertFromString(const std::string& str, PixHSV<byte>& val);

//! PixDKL<byte> overload
/*! Format is "<byte>,<byte>,<byte>" for D, K, L */
std::string convertToString(const PixDKL<byte>& val);
//! PixDKL<byte> overload
/*! Format  is "<byte>,<byte>,<byte>" for  D, K, L*/
void convertFromString(const std::string& str, PixDKL<byte>& val);

#endif
