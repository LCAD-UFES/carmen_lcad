/*!@file Util/Promotions.H basic definitions for template type promotions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Promotions.H $
// $Id: Promotions.H 4731 2005-06-29 19:33:51Z rjpeters $
//

#ifndef PROMOTIONS_H_DEFINED
#define PROMOTIONS_H_DEFINED

#include "Util/Types.H"
#include "Util/log.H"
#include <limits>


/*! The idea here is to create a mechanism by which a given template
type can be promoted to a more capable type (e.g., byte to float, or
PixRGB<byte> to PixRGB<float>), typically to hold the result of some
operation that may exceed the capacity of the original type (e.g., the
result of adding two bytes may not fit within the range of possible
values for byte). See http://www.oonumerics.org/blitz/traits.html for
further details.  The way to use this is as follows: if you have a
template type T (which may be scalar or not) that you wish to promote
to a more capable type that would have "similar characteristics" to
another type TT, then use the type promote_trait<T,TT>::TP. For
example: promote_trait<T,float>::TP is a float if T was a byte, and is
a PixRGB<float> if T was a PixRGB<byte> */


//! Promote from T1 and T2 to a type than can hold T1 * T2
/*! Basic promotion mechanism: given T1 and T2, TP provides the
 appropriate type to hold the result of an operation involving T1 and
 T2.  Default is to promote to type T1, that is, no change (this is
 used when T2 is "weaker" than T1; when T2 is "stronger" that T1, the
 explicit specialized rules below are used). */
template <class T1, class T2>
struct promote_trait { typedef T1 TP; enum { willPromote = 0 }; };

// Here go the various specializations:

// NOTE: we follow the basic C++ integral promotions, promoting to int
// from byte, int16 and int32
template <> struct promote_trait<byte, byte>          { typedef int         TP; enum { willPromote = 1 }; };
template <> struct promote_trait<byte, int16>         { typedef int         TP; enum { willPromote = 1 }; };
template <> struct promote_trait<byte, int32>         { typedef int         TP; enum { willPromote = 1 }; };
template <> struct promote_trait<byte, float>         { typedef float       TP; enum { willPromote = 1 }; };
template <> struct promote_trait<byte, double>        { typedef double      TP; enum { willPromote = 1 }; };
template <> struct promote_trait<byte, long double>   { typedef long double TP; enum { willPromote = 1 }; };

template <> struct promote_trait<int16, byte>         { typedef int         TP; enum { willPromote = 1 }; };
template <> struct promote_trait<int16, int16>        { typedef int         TP; enum { willPromote = 1 }; };
template <> struct promote_trait<int16, int32>        { typedef int         TP; enum { willPromote = 1 }; };
template <> struct promote_trait<int16, float>        { typedef float       TP; enum { willPromote = 1 }; };
template <> struct promote_trait<int16, double>       { typedef double      TP; enum { willPromote = 1 }; };
template <> struct promote_trait<int16, long double>  { typedef long double TP; enum { willPromote = 1 }; };

template <> struct promote_trait<int32, float>        { typedef float       TP; enum { willPromote = 1 }; };
template <> struct promote_trait<int32, double>       { typedef double      TP; enum { willPromote = 1 }; };
template <> struct promote_trait<int32, long double>  { typedef long double TP; enum { willPromote = 1 }; };

template <> struct promote_trait<float, double>       { typedef double      TP; enum { willPromote = 1 }; };
template <> struct promote_trait<float, long double>  { typedef long double TP; enum { willPromote = 1 }; };

template <> struct promote_trait<double, long double> { typedef long double TP; enum { willPromote = 1 }; };

//! A macro for specializing promote_trait for template types.
/*! See e.g. usage in PixelsTypes.H. */
#define SPECIALIZE_PROMOTE_TRAIT(T_TYPE, T_PARAMS, T_ARGS)                                                                                                                 \
template <T_PARAMS> class T_TYPE;                                                                                                                               \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , byte>          { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , int16>         { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , int32>         { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , float>         { typedef T_TYPE<float T_ARGS>       TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , double>        { typedef T_TYPE<double T_ARGS>      TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , long double>   { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };                     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , byte>         { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , int16>        { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , int32>        { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , float>        { typedef T_TYPE<float T_ARGS>       TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , double>       { typedef T_TYPE<double T_ARGS>      TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , long double>  { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };                     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<int32 T_ARGS> , float>        { typedef T_TYPE<float T_ARGS>       TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<int32 T_ARGS> , double>       { typedef T_TYPE<double T_ARGS>      TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<int32 T_ARGS> , long double>  { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };                     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<float T_ARGS> , double>       { typedef T_TYPE<double T_ARGS>      TP; enum { willPromote = 1 }; };                     \
template <> struct promote_trait<T_TYPE<float T_ARGS> , long double>  { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };                     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<double T_ARGS> , long double> { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };                     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , T_TYPE<byte T_ARGS> >          { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , T_TYPE<int16 T_ARGS> >         { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , T_TYPE<int32 T_ARGS> >         { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , T_TYPE<float T_ARGS> >         { typedef T_TYPE<float T_ARGS>       TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , T_TYPE<double T_ARGS> >        { typedef T_TYPE<double T_ARGS>      TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<byte T_ARGS> , T_TYPE<long double T_ARGS> >   { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , T_TYPE<byte T_ARGS> >         { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , T_TYPE<int16 T_ARGS> >        { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , T_TYPE<int32 T_ARGS> >        { typedef T_TYPE<int T_ARGS>         TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , T_TYPE<float T_ARGS> >        { typedef T_TYPE<float T_ARGS>       TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , T_TYPE<double T_ARGS> >       { typedef T_TYPE<double T_ARGS>      TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<int16 T_ARGS> , T_TYPE<long double T_ARGS> >  { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<int32 T_ARGS> , T_TYPE<float T_ARGS> >        { typedef T_TYPE<float T_ARGS>       TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<int32 T_ARGS> , T_TYPE<double T_ARGS> >       { typedef T_TYPE<double T_ARGS>      TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<int32 T_ARGS> , T_TYPE<long double T_ARGS> >  { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<float T_ARGS> , T_TYPE<double T_ARGS> >       { typedef T_TYPE<double T_ARGS>      TP; enum { willPromote = 1 }; };     \
template <> struct promote_trait<T_TYPE<float T_ARGS> , T_TYPE<long double T_ARGS> >  { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };     \
                                                                                                                                                                \
template <> struct promote_trait<T_TYPE<double T_ARGS> , T_TYPE<long double T_ARGS> > { typedef T_TYPE<long double T_ARGS> TP; enum { willPromote = 1 }; };

// ######################################################################

// The following defines rules to safely convert from type TT to type
// T, clamping the values to the destination type's range if
// necessary. The function to use is clamped_convert<T>(xx), which
// will convert xx from type TT to type T, clamping its value to the
// acceptable range for T if necessary.

// Basically there's a promoteFromTo<T1,T2> struct that exposes an
// enum is_preserving which is set to 1 or 0 depending on whether T1
// --> T2 is value-preserving or not. Then defined a convert_helper
// struct that has a convert() function that does the real work,
// except the struct can be specialized on whether the conversion is
// preserving or not. Then the actual clamped_convert function just
// forwards to convert_helper<T1,T2>::convert(), which picks the right
// implementation. This way all of the important information is
// specified by how we define promoteFromTo<T1,T2>::is_preserving, and
// this specification is cleanly separated from the implementation
// that uses it.

// by default, promotion is considered range-preserving (will not
// require clamping by us in clamped_convert). The rationale for this
// weirdness is that for unknown types we will leave it to the
// converting operator=() of that type to deal with the clamping.
template <class T1, class T2>
struct promoteFromTo { enum { is_preserving = 1 }; };

// promote from T to itself is a range-preserving no-op:
template <class T>
struct promoteFromTo<T, T> { enum { is_preserving = 1 }; };

// these rules declare which conversions are not range-preserving and
// will require clamping within clamped_convert; they are used to
// enforce clamping of builtin types:

template <> struct promoteFromTo<int16      , byte>   { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<int32      , byte>   { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<float      , byte>   { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<double     , byte>   { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<long double, byte>   { enum { is_preserving = 0 }; };

template <> struct promoteFromTo<int32      , int16>  { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<float      , int16>  { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<double     , int16>  { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<long double, int16>  { enum { is_preserving = 0 }; };

template <> struct promoteFromTo<float      , int32>  { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<double     , int32>  { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<long double, int32>  { enum { is_preserving = 0 }; };

template <> struct promoteFromTo<double     , float>  { enum { is_preserving = 0 }; };
template <> struct promoteFromTo<long double, float>  { enum { is_preserving = 0 }; };

template <> struct promoteFromTo<long double, double> { enum { is_preserving = 0 }; };

//! Helper struct to handle rounding between different types
/*! Note that we aren't worried about clamping here, that is handled
    separately by convert_helper (see below). We have different
    specializations of round_helper depending on whether the src and
    dest types are integral or floating-point. The only case where we
    have to do any actual rounding is when we're going from
    floating->integral. */
template <class dst_type, class src_type,
          bool need_rounding =
          (std::numeric_limits<dst_type>::is_integer
           && !std::numeric_limits<src_type>::is_integer)>
struct round_helper;

template <class dst_type, class src_type>
struct round_helper<dst_type, src_type, false> // no rounding needed
{
  inline static dst_type round(src_type x)
  { return dst_type(x); }
};

template <class dst_type, class src_type>
struct round_helper<dst_type, src_type, true> // rounding needed
{
  inline static dst_type round(src_type x)
  {
    if      (x > 0) return dst_type(x+0.5); // round toward +Inf
    else if (x < 0) return dst_type(x-0.5); // round toward -Inf
    else            return dst_type(0);
  }
};

//! Helper struct to handle type conversions at compile time.
/*! Helps decide whether to clamp or not at compile time so that we
    don't waste time in a test at run time; because
    std::numeric_limits is a pile of trash, we also need to
    parameterize the helper by a boolean indicating whether the type
    we convert to is integer or not. This will determine which numeric
    limits we use to do the clamping: */
template <class dst_type, class src_type,
          bool is_preserving =
                  promoteFromTo<src_type, dst_type>::is_preserving,
          bool dst_type_is_integer =
                  std::numeric_limits<dst_type>::is_integer>
struct convert_helper;

// clamp and convert if non-preserving; dest type is integral:
template <class dst_type, class src_type>
struct convert_helper<dst_type, src_type,
                      /* is_preserving = */ false,
                      /* dst_type_is_integer = */ true>
{
  inline static dst_type convert(src_type x)
  {
    if (x < std::numeric_limits<dst_type>::min())
      return std::numeric_limits<dst_type>::min();
    if (x > std::numeric_limits<dst_type>::max())
      return std::numeric_limits<dst_type>::max();
    return dst_type(x);
  }

  inline static dst_type rounded_convert(src_type x)
  {
    if (x < std::numeric_limits<dst_type>::min())
      return std::numeric_limits<dst_type>::min();
    if (x > std::numeric_limits<dst_type>::max())
      return std::numeric_limits<dst_type>::max();

    return round_helper<dst_type, src_type>::round(x);
  }
};

// clamp and convert if non-preserving; dest type is non-integral:
template <class dst_type, class src_type>
struct convert_helper<dst_type, src_type,
                      /* is_preserving = */ false,
                      /* dst_type_is_integer = */ false>
{
  inline static dst_type convert(src_type x)
  {
    if (x < -std::numeric_limits<dst_type>::max())
      return -std::numeric_limits<dst_type>::max();
    if (x > std::numeric_limits<dst_type>::max())
      return std::numeric_limits<dst_type>::max();
    return dst_type(x);
  }

  inline static dst_type rounded_convert(src_type x)
  {
    if (x < -std::numeric_limits<dst_type>::max())
      return -std::numeric_limits<dst_type>::max();
    if (x > std::numeric_limits<dst_type>::max())
      return std::numeric_limits<dst_type>::max();

    return round_helper<dst_type, src_type>::round(x);
  }
};

// cast without checking range if preserving; for builtin types, this
// means that we know that we can safely cast (the destination types
// can represent all values of the source type); for other types, we
// here assume that the conversion constructor will take care of the
// clamping:
template <class dst_type, class src_type>
struct convert_helper<dst_type, src_type,
                      /* is_preserving = */ true,
                      /* dst_type_is_integer = */ true>
{
  inline static dst_type convert(src_type x)
  {
    return dst_type(x);
  }

  inline static dst_type rounded_convert(src_type x)
  {
    return round_helper<dst_type, src_type>::round(x);
  }
};

template <class dst_type, class src_type>
struct convert_helper<dst_type, src_type,
                      /* is_preserving = */ true,
                      /* dst_type_is_integer = */ false>
{
  inline static dst_type convert(src_type x)
  {
    return dst_type(x);
  }

  inline static dst_type rounded_convert(src_type x)
  {
    return round_helper<dst_type, src_type>::round(x);
  }
};

//! Convert safely from src_type to type dst_type, clamping if necessary.
template <class dst_type, class src_type>
inline
dst_type clamped_convert(src_type x)
{
  return convert_helper<dst_type, src_type>::convert(x);
}

//! Convert safely from src_type to type dst_type, clamping and rounding as needed.
template <class dst_type, class src_type>
inline
dst_type clamped_rounded_convert(src_type x)
{
  return convert_helper<dst_type, src_type>::rounded_convert(x);
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
