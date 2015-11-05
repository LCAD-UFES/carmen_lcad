/*!@file Image/PixelsTypes.H Basic pixel types version 2.0 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PixelsTypes.H $
// $Id: PixelsTypes.H 15059 2011-11-11 00:55:58Z dberg $
//

#ifndef PIXELSTYPES_H_DEFINED
#define PIXELSTYPES_H_DEFINED

#include "Util/MathFunctions.H"  // for clampValue()
#include "Image/PixelsBase.H"
#include "Image/PixelsCommonDef.H"
#include "Util/Promotions.H"
#include "Util/TypeTraits.H"
#include "Util/Assert.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Image/colorDefs.H"

#include <cmath>

// ######################################################################
// ######################################################################
// Derived pixel classes that hold type conversion information
// ######################################################################
// ######################################################################

template <class T>             class PixRGB;
template <class T>             class PixHSV;
template <class T>             class PixJpegYUV;
template <class T>             class PixVideoYUV;
template <class T>             class PixYIQ;
template <class T>             class PixHSL;
template <class T>             class PixLab;
template <class T>             class PixXYZ;
template <class T>             class PixH2SV1;
template <class T>             class PixH2SV2;
template <class T>             class PixH2SV3;
template <class T, size_t dim> class PixHyper;
template <class T>             class PixDKL;
template <class T>             class PixDKLM;

//! Specialize TypeTraits to indicate that PixRGB is a "trivial" type.
/*! That is, it is a type that can be safely initialized with memset(), and
    safely copied with memcpy(). In practical terms, this means we're
    relying on PixRGB not to do anything significant in its copy
    constructor or destructor. */

template <class T> struct TypeTraits<PixRGB<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixHSV is a "trivial" type.
template <class T> struct TypeTraits<PixHSV<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixJpegYUV is a "trivial" type.
template <class T> struct TypeTraits<PixJpegYUV<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixVideoYUV is a "trivial" type.
template <class T> struct TypeTraits<PixVideoYUV<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixYIQ is a "trivial" type.
template <class T> struct TypeTraits<PixYIQ<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixHSL is a "trivial" type.
template <class T> struct TypeTraits<PixHSL<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixLab is a "trivial" type.
template <class T> struct TypeTraits<PixLab<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixXYZ is a "trivial" type.
template <class T> struct TypeTraits<PixXYZ<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixH2SV1 is a "trivial" type.
template <class T> struct TypeTraits<PixH2SV1<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixH2SV2 is a "trivial" type.
template <class T> struct TypeTraits<PixH2SV2<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixH2SV3 is a "trivial" type.
template <class T> struct TypeTraits<PixH2SV3<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixHyper is a "trivial" type.
template <class T, size_t dim> struct TypeTraits<PixHyper<T,dim> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixDKL is a "trivial" type.
template <class T> struct TypeTraits<PixDKL<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};
//! Specialize TypeTraits to indicate that PixDKLM is a "trivial" type.
template <class T> struct TypeTraits<PixDKLM<T> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};


namespace ops
{
  struct plus
  {
    template <class T, class U>
    typename promote_trait<T,U>::TP
    operator()(const T& t, const U& u) { return t+u; }
  };

  struct minus
  {
    template <class T, class U>
    typename promote_trait<T,U>::TP
    operator()(const T& t, const U& u) { return t-u; }
  };

  struct multiplies
  {
    template <class T, class U>
    typename promote_trait<T,U>::TP
    operator()(const T& t, const U& u) { return t*u; }
  };

  struct divides
  {
    template <class T, class U>
    typename promote_trait<T,U>::TP
    operator()(const T& t, const U& u) { return t/u; }
  };

  struct modulo
  {
    template <class T, class U>
    typename promote_trait<T,U>::TP
    operator()(const T& t, const U& u) { return t%u; }
  };

  struct bitwise_and
  {
    template <class T, class U>
    typename promote_trait<T,U>::TP
    operator()(const T& t, const U& u) { return t&u; }
  };

  struct bitwise_or
  {
    template <class T, class U>
    typename promote_trait<T,U>::TP
    operator()(const T& t, const U& u) { return t|u; }
  };

  struct bitwise_xor
  {
    template <class T, class U>
    typename promote_trait<T,U>::TP
    operator()(const T& t, const U& u) { return t^u; }
  };

}


//! Helper struct to provide a generic way to "rebind" template parameters.
/*! That is, this gives us a way to get Foo<T2> if we are given
    Foo<T>, or to get Bar<T2, U> if we are given Bar<T, U>. In
    particular, this lets us write generic functions that can deal
    with pixel types that have either a single template parameter
    (e.g. PixRGB<T>, PixHSV<T>), or more than one template parameter
    (e.g. PixHyper<T, dim>). */
template <class C, class T>
struct rebind
{};

template <template <class> class C,
          class T, class U>
struct rebind<C<T>, U>
{
  typedef C<U> type;
};

template <template <class, size_t> class C,
          class T1, size_t T2, class U1>
struct rebind<C<T1, T2>, U1>
{
  typedef C<U1, T2> type;
};


namespace pix_helper
{
  // ######################################################################
  // ######################################################################
  // clamp and convert helpers
  // ######################################################################
  // ######################################################################

  /* These inline functions are used when setting Pix1 = Pix2 where the
     type for Pix1 is the same as Pix2, but its base type (float, int etc)
     is different.

     As examples:
     - PixRGB<float> A = PixRGB<byte> B            would use these.
     - PixRGB<float> A = PixHSV<int>  B            would NOT use these.

     Since the typing is uncertain in this usege we use clamped_convert for
     all copies
  */

  template <class pix1, class pix2>
  inline void clamped_assign_3(pix1& lhs, const pix2& rhs)
  {
    lhs.p[0] = clamped_convert<typename pix1::ScalarType>(rhs.p[0]);
    lhs.p[1] = clamped_convert<typename pix1::ScalarType>(rhs.p[1]);
    lhs.p[2] = clamped_convert<typename pix1::ScalarType>(rhs.p[2]);
  }

  template <class pix1, class pix2>
  inline void clamped_assign_4(pix1& lhs, const pix2& rhs)
  {
    lhs.p[0] = clamped_convert<typename pix1::ScalarType>(rhs.p[0]);
    lhs.p[1] = clamped_convert<typename pix1::ScalarType>(rhs.p[1]);
    lhs.p[2] = clamped_convert<typename pix1::ScalarType>(rhs.p[2]);
    lhs.p[3] = clamped_convert<typename pix1::ScalarType>(rhs.p[3]);
  }

  template <class pix1, class pix2>
  inline void clamped_assign_n(pix1& lhs, const pix2& rhs)
  {
    for (size_t i = 0; i < pix1::myDim; ++i)
      lhs.p[i] = clamped_convert<typename pix1::ScalarType>(rhs.p[i]);
  }


  // ######################################################################
  // ######################################################################
  // helper functions for defining operators
  // ######################################################################
  // ######################################################################

  // ######################################################################
  //! helper for adding a base type with a pixel where basic type T != T2
  /*! EXAMPLE: PixRGB<float> += int */

  template <class insttype, class T2, class functype>
  inline insttype& pix_scalar_op_eq(insttype& lhs,
                                    const T2 rhs,
                                    functype func)
  {
    typedef typename insttype::ScalarType T;

    for (size_t i = 0; i < insttype::myDim; i++)
      lhs.p[i] = clamped_convert<T>(func(lhs.p[i], rhs));

    return lhs;
  }

  // ######################################################################
  //! helper for adding a base type with a pixel where basic type T != T2
  /*! EXAMPLE: PixRGB<float> + int */

  template <class T, class insttype, class T2, class functype>
  inline typename rebind<insttype,
                         typename promote_trait<T, T2>::TP>::type
  pix_scalar_op(const insttype& lhs, const T2 rhs, functype func)
  {
    typedef typename promote_trait<T, T2>::TP TP;
    typename rebind<insttype, TP>::type result;
    for (size_t i = 0; i < insttype::myDim; i++)
      result.p[i] = clamped_convert<TP>(func(lhs.p[i], rhs));
    return result;
  }

  // ######################################################################
  //! Apply a x= operation between two pixels where the base type are the same
  /*! 4 operations can be performed here. The conditionals are constants so the compiler will pre-compute them
    (1) If a pixel type is the same and we will promote the trait and clamp         (e.g. PixRGB<byte>  += PixRGB<byte>  )
    (2) If a pixel type is not the same and we will promote the trait and clamp     (e.g. PixHSV<byte>  += PixRGB<byte>  )
    (3) If a pixel type is the same and we will not promote the trait and clamp     (e.g. PixRGB<float> += PixRGB<float> )
    (4) If a pixel type is not the same and we not will promote the trait and clamp (e.g. PixHSV<float> += PixRGB<float> )
  */

  template <class lhstype, class rhstype, class functype>
  inline lhstype& pix_pix_op_eq(lhstype& lhs,
                                const rhstype& rhs,
                                functype func)
  {
    ASSERT(lhs.pType == rhs.pType); // since both pixels are pixtype<>

    for (size_t i = 0; i < lhstype::myDim; i++)
      lhs.p[i] = clamped_convert<typename lhstype::ScalarType>
        (func(lhs.p[i], rhs.p[i]));

    return lhs;
  }

  // ######################################################################
  //! Apply a x operation between two pixels where the base type are the same
  /*! 2 operations can be performed here. The conditionals are constants so the compiler will pre-compute them
    (1) If a pixel type is the same and we will promote the trait      (e.g. PixRGB<byte> + PixRGB<byte> )
    (2) If a pixel type is not the same and we will promote the trait  (e.g. PixRGB<byte> + PixHSV<byte> )
  */

  template <template <class> class pixtype, class T, class T2, class functype>
  inline pixtype<typename promote_trait<T,T2>::TP>
  pix_pix_op(const pixtype<T>& lhs, const pixtype<T2>& rhs, functype func)
  {
    ASSERT(lhs.pType == rhs.pType); // since both pixels are pixtype<>

    typedef typename promote_trait<T,T2>::TP TP;
    pixtype<TP> result;
    for (size_t i = 0; i < pixtype<T>::myDim; i++)
      result.p[i] = clamped_convert<TP>(func(lhs.p[i], rhs.p[i]));
    return result;
  }

  template <template <class, size_t> class pixtype,
            class T, class T2, size_t dim, class functype>
  inline pixtype<typename promote_trait<T,T2>::TP, dim>
  pix_pix_op(const pixtype<T, dim>& lhs, const pixtype<T2, dim>& rhs, functype func)
  {
    ASSERT(lhs.pType == rhs.pType); // since both pixels are pixtype<>

    typedef typename promote_trait<T,T2>::TP TP;
    pixtype<TP, dim> result;
    for (size_t i = 0; i < pixtype<T, dim>::myDim; i++)
      result.p[i] = clamped_convert<TP>(func(lhs.p[i], rhs.p[i]));
    return result;
  }

}

//! A "mix-in" class that injects all arithmetic operators into the class that inherits it.
/*! This is an example of "the curiously recurring template pattern",
    in which some class, say Foo, inherits from a template class
    instantiated on Foo, say Mixin<Foo>. In our case, we have the
    various pixel classes (e.g. PixRGB<T>) inheriting from this mixin
    class (e.g. pix_ops_mixin<PixRGB<T>, T>). */

template <template <class> class pixtype, class T>
struct pix_ops_mixin
{

#define PIX_MATH_OPS_EQ(OPSYM, FUNCTYPE)    \
  template <class T2> inline pixtype<T>& operator OPSYM (const pixtype<T2>& A) { return pix_helper::pix_pix_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); }

  PIX_MATH_OPS_EQ(+=, ops::plus)
  PIX_MATH_OPS_EQ(-=, ops::minus)
  PIX_MATH_OPS_EQ(*=, ops::multiplies)
  PIX_MATH_OPS_EQ(/=, ops::divides)
  PIX_MATH_OPS_EQ(%=, ops::modulo)
  PIX_MATH_OPS_EQ(&=, ops::bitwise_and)
  PIX_MATH_OPS_EQ(|=, ops::bitwise_or)
  PIX_MATH_OPS_EQ(^=, ops::bitwise_xor)

#undef PIX_MATH_OPS_EQ

#define PIX_OPERATOR_EQ(OPSYM, FUNCTYPE)                                                                                    \
  inline pixtype<T>& operator OPSYM (const bool           A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const char           A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const short          A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const int            A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const long           A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const float          A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const double         A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const long double    A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const unsigned char  A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const unsigned short A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const unsigned int   A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T>& operator OPSYM (const unsigned long  A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T>&>(*this), A, FUNCTYPE()); }

  PIX_OPERATOR_EQ(+=, ops::plus)
  PIX_OPERATOR_EQ(-=, ops::minus)
  PIX_OPERATOR_EQ(*=, ops::multiplies)
  PIX_OPERATOR_EQ(/=, ops::divides)
  PIX_OPERATOR_EQ(%=, ops::modulo)
  PIX_OPERATOR_EQ(&=, ops::bitwise_and)
  PIX_OPERATOR_EQ(|=, ops::bitwise_or)
  PIX_OPERATOR_EQ(^=, ops::bitwise_xor)

#undef PIX_OPERATOR_EQ

#define PIX_MATH_OPS(OPSYM, FUNCTYPE)                                                           \
  template <class T2>                                                                           \
  inline typename rebind<pixtype<T>, typename promote_trait<T,T2>::TP>::type                    \
  operator OPSYM (const pixtype<T2>& rhs) const                                                 \
  { return pix_helper::pix_pix_op(static_cast<const pixtype<T>&>(*this), rhs, FUNCTYPE()); }

  PIX_MATH_OPS(+, ops::plus)
  PIX_MATH_OPS(-, ops::minus)
  PIX_MATH_OPS(*, ops::multiplies)
  PIX_MATH_OPS(/, ops::divides)
  PIX_MATH_OPS(%, ops::modulo)
  PIX_MATH_OPS(&, ops::bitwise_and)
  PIX_MATH_OPS(|, ops::bitwise_or)
  PIX_MATH_OPS(^, ops::bitwise_xor)

#undef PIX_MATH_OPS

#define PIX_OP(T2, OPSYM, FUNC)                                                                                                 \
  inline typename rebind<pixtype<T>, typename promote_trait<T, T2>::TP>::type                                                   \
  operator OPSYM (const T2 A) const { return pix_helper::pix_scalar_op<T>(static_cast<const pixtype<T>&>(*this), A, FUNC); }

#define PIX_OPERATOR(OPSYM, FUNCTYPE)           \
  PIX_OP(bool, OPSYM, FUNCTYPE())               \
  PIX_OP(char, OPSYM, FUNCTYPE())               \
  PIX_OP(short, OPSYM, FUNCTYPE())              \
  PIX_OP(int, OPSYM, FUNCTYPE())                \
  PIX_OP(long, OPSYM, FUNCTYPE())               \
  PIX_OP(float, OPSYM, FUNCTYPE())              \
  PIX_OP(double, OPSYM, FUNCTYPE())             \
  PIX_OP(long double, OPSYM, FUNCTYPE())        \
  PIX_OP(unsigned char, OPSYM, FUNCTYPE())      \
  PIX_OP(unsigned short, OPSYM, FUNCTYPE())     \
  PIX_OP(unsigned int, OPSYM, FUNCTYPE())       \
  PIX_OP(unsigned long, OPSYM, FUNCTYPE())

  PIX_OPERATOR(+, ops::plus)
  PIX_OPERATOR(-, ops::minus)
  PIX_OPERATOR(*, ops::multiplies)
  PIX_OPERATOR(/, ops::divides)
  PIX_OPERATOR(%, ops::modulo)
  PIX_OPERATOR(&, ops::bitwise_and)
  PIX_OPERATOR(|, ops::bitwise_or)
  PIX_OPERATOR(^, ops::bitwise_xor)

#undef PIX_OPERATOR
#undef PIX_OP
};

template <template <class, size_t> class pixtype, class T, size_t dim>
struct pix_ops_mixin_hyper
{

#define PIX_MATH_OPS_EQ(OPSYM, FUNCTYPE)                                                                                                                                            \
  template <class T2> inline pixtype<T, dim>& operator OPSYM (const pixtype<T2, dim>& A) { return pix_helper::pix_pix_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); }

  PIX_MATH_OPS_EQ(+=, ops::plus)
  PIX_MATH_OPS_EQ(-=, ops::minus)
  PIX_MATH_OPS_EQ(*=, ops::multiplies)
  PIX_MATH_OPS_EQ(/=, ops::divides)
  PIX_MATH_OPS_EQ(%=, ops::modulo)
  PIX_MATH_OPS_EQ(&=, ops::bitwise_and)
  PIX_MATH_OPS_EQ(|=, ops::bitwise_or)
  PIX_MATH_OPS_EQ(^=, ops::bitwise_xor)

#undef PIX_MATH_OPS_EQ

#define PIX_OPERATOR_EQ(OPSYM, FUNCTYPE)                                                                                    \
  inline pixtype<T, dim>& operator OPSYM (const bool           A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const char           A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const short          A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const int            A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const long           A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const float          A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const double         A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const long double    A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const unsigned char  A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const unsigned short A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const unsigned int   A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); } \
  inline pixtype<T, dim>& operator OPSYM (const unsigned long  A) { return pix_helper::pix_scalar_op_eq(static_cast<pixtype<T, dim>&>(*this), A, FUNCTYPE()); }

  PIX_OPERATOR_EQ(+=, ops::plus)
  PIX_OPERATOR_EQ(-=, ops::minus)
  PIX_OPERATOR_EQ(*=, ops::multiplies)
  PIX_OPERATOR_EQ(/=, ops::divides)
  PIX_OPERATOR_EQ(%=, ops::modulo)
  PIX_OPERATOR_EQ(&=, ops::bitwise_and)
  PIX_OPERATOR_EQ(|=, ops::bitwise_or)
  PIX_OPERATOR_EQ(^=, ops::bitwise_xor)

#undef PIX_OPERATOR_EQ

#define PIX_MATH_OPS(OPSYM, FUNCTYPE)                                                                   \
  template <class T2>                                                                                   \
  inline pixtype<typename promote_trait<T,T2>::TP, dim>                                                 \
  operator OPSYM (const pixtype<T2, dim>& rhs) const                                                    \
  { return pix_helper::pix_pix_op(static_cast<const pixtype<T, dim>&>(*this), rhs, FUNCTYPE()); }

  PIX_MATH_OPS(+, ops::plus)
  PIX_MATH_OPS(-, ops::minus)
  PIX_MATH_OPS(*, ops::multiplies)
  PIX_MATH_OPS(/, ops::divides)
  PIX_MATH_OPS(%, ops::modulo)
  PIX_MATH_OPS(&, ops::bitwise_and)
  PIX_MATH_OPS(|, ops::bitwise_or)
  PIX_MATH_OPS(^, ops::bitwise_xor)

#undef PIX_MATH_OPS

#define PIX_OP(T2, OPSYM, FUNC)                                                                                                         \
  inline typename rebind<pixtype<T, dim>, typename promote_trait<T, T2>::TP>::type                                                      \
  operator OPSYM (const T2 A) const { return pix_helper::pix_scalar_op<T>(static_cast<const pixtype<T, dim>&>(*this), A, FUNC); }

#define PIX_OPERATOR(OPSYM, FUNCTYPE)           \
  PIX_OP(bool, OPSYM, FUNCTYPE())               \
  PIX_OP(char, OPSYM, FUNCTYPE())               \
  PIX_OP(short, OPSYM, FUNCTYPE())              \
  PIX_OP(int, OPSYM, FUNCTYPE())                \
  PIX_OP(long, OPSYM, FUNCTYPE())               \
  PIX_OP(float, OPSYM, FUNCTYPE())              \
  PIX_OP(double, OPSYM, FUNCTYPE())             \
  PIX_OP(long double, OPSYM, FUNCTYPE())        \
  PIX_OP(unsigned char, OPSYM, FUNCTYPE())      \
  PIX_OP(unsigned short, OPSYM, FUNCTYPE())     \
  PIX_OP(unsigned int, OPSYM, FUNCTYPE())       \
  PIX_OP(unsigned long, OPSYM, FUNCTYPE())

  PIX_OPERATOR(+, ops::plus)
  PIX_OPERATOR(-, ops::minus)
  PIX_OPERATOR(*, ops::multiplies)
  PIX_OPERATOR(/, ops::divides)
  PIX_OPERATOR(%, ops::modulo)
  PIX_OPERATOR(&, ops::bitwise_and)
  PIX_OPERATOR(|, ops::bitwise_or)
  PIX_OPERATOR(^, ops::bitwise_xor)

#undef PIX_OPERATOR
#undef PIX_OP
};

// #################################
// #################################
// ######################################################################
// ######################################################################
// --> PIXEL TYPE DECLARATIONS <--
// ######################################################################
// ######################################################################
// #################################
// #################################

// ######################################################################
// ######################################################################
//! This is the basic RGB pixel class
// ######################################################################
// ######################################################################

//! Derived RGB pixel class from Pixels
template <class T> class PixRGB :
  public Pixels<T,3>,
  public pix_ops_mixin<PixRGB, T>
{
private:
public:
  const static enum PixType pType = PixTypeRGB;
  const static size_t myDim = 3;

  using Pixels<T,3>::p;

  inline PixRGB();
  inline ~PixRGB();
  explicit inline PixRGB(const T val);

  template <class T2>
  explicit inline PixRGB(const T2 val);

  template <class T2> explicit inline PixRGB(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixRGB(const PixHSV<T2>&     A);
  template <class T2> explicit inline PixRGB(const PixXYZ<T2>&     A);
  template <class T2> explicit inline PixRGB(const PixLab<T2>&     A);
  template <class T2> explicit inline PixRGB(const PixJpegYUV<T2>& A);
  template <class T2> explicit inline PixRGB(const PixVideoYUV<T2>& A);
  template <class T2> explicit inline PixRGB(const PixYIQ<T2>&     A);
  template <class T2> explicit inline PixRGB(const PixHSL<T2>&     A);
  template <class T2> explicit inline PixRGB(const PixH2SV1<T2>&   A);
  template <class T2> explicit inline PixRGB(const PixH2SV2<T2>&   A);
  template <class T2> explicit inline PixRGB(const PixHyper<T2,3>& A);

  // NOTE: default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixRGB(const PixRGB<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixRGB(const T redval, const T greenval, const T blueval);
  //! Construct from given values, clamping & convertPixing as needed
  template <class T2, class T3, class T4> inline
  PixRGB(const T2 redval, const T3 greenval, const T4 blueval);

  // special local access methods
  inline void getRGB(T& r, T& g, T& b) const;

  //! Access red channel
  inline T red() const;
  //! Access green channel
  inline T green() const;
  //! Access blue channel
  inline T blue() const;
  //! Set red channel value, clamping & convertPixing as necessary
  template <class T2> inline void setRed(const T2 redval);
  //! Set green channel value, clamping & convertPixing as necessary
  template <class T2> inline void setGreen(const T2 greenval);
  //! Set blue channel value, clamping & convertPixing as necessary
  template <class T2> inline void setBlue(const T2 blueval);
  //! Set all channel values, clamping & convertPixing as necessary
  template <class T2, class T3, class T4> inline void
  set(const T2 redval, const T3 greenval, const T4 blueval);
  //! Compute luminance as (r + g + b) / 3
  inline T luminance() const;
  //! special methods added for backwards compat.
  inline void setHSV(const T h1, const T h2, const T s, const T v);
  //! special methods added for backwards compat.
  inline void getHSV(T& h1, T& h2, T& s, T& v) const;

  // Any overrides to Pixels methods

};

// ######################################################################
// ######################################################################
//! This is the basic HSV pixel class
// ######################################################################
// ######################################################################

//! Derived HSV pixel class from Pixels
template <class T> class PixHSV :
  public Pixels<T,3>,
  public pix_ops_mixin<PixHSV, T>
{
private:
public:
  const static enum PixType pType = PixTypeHSV;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixHSV();
  inline ~PixHSV();
  explicit inline PixHSV(const T val);

  template <class T2> explicit inline PixHSV(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixHSV(const PixHSV<T2>&     A);
  template <class T2> explicit inline PixHSV(const PixYIQ<T2>&     A);
  template <class T2> explicit inline PixHSV(const PixHSL<T2>&     A);
  template <class T2> explicit inline PixHSV(const PixH2SV1<T2>&   A);
  template <class T2> explicit inline PixHSV(const PixH2SV2<T2>&   A);
  template <class T2> explicit inline PixHSV(const PixH2SV3<T2>&   A);
  template <class T2> explicit inline PixHSV(const PixHyper<T2,3>& A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixHSV(const PixHSV<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixHSV(const T Hval, const T Sval, const T Vval);
  //! Construct from given values, clamping & convertPixing as needed
  template <class T2, class T3, class T4> inline
  PixHSV(const T2 Hval, const T3 Sval, const T4 Vval);

  // special local methods

  inline void getHSV(T& h, T& s, T& v) const;

  //! Access hue channel
  inline T H() const;
  //! Access sat channel
  inline T S() const;
  //! Access val channel
  inline T V() const;
  //! Set hue channel value, clamping & converting as necessary
  template <class T2> inline void setH(const T2 H);
  //! Set sat channel value, clamping & converting as necessary
  template <class T2> inline void setS(const T2 S);
  //! Set val channel value, clamping & converting as necessary
  template <class T2> inline void setV(const T2 V);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the basic jpeg-YUV pixel class
// ######################################################################
// ######################################################################

//! Derived YUV pixel class from Pixels
template <class T> class PixJpegYUV :
  public Pixels<T,3>,
  public pix_ops_mixin<PixJpegYUV, T>
{
private:
public:
  const static enum PixType pType = PixTypeYUV;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixJpegYUV();
  inline ~PixJpegYUV();
  explicit inline PixJpegYUV(const T val);

  template <class T2> explicit inline PixJpegYUV(const PixJpegYUV<T2>& A);
  template <class T2> explicit inline PixJpegYUV(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixJpegYUV(const PixHyper<T2,3>& A);

  // default operator=() OK

  inline PixJpegYUV(const PixJpegYUV<T>& pix);

  //! Construct an RGB pixel with given values red, green, blue
  inline PixJpegYUV(const T Yval, const T Uval, const T Vval);
  //! Construct from given values, clamping & converting as needed
  template <class T2, class T3, class T4> inline
  PixJpegYUV(const T2 Yval, const T3 Uval, const T4 Vval);

  //! Access Y channel
  inline T Y() const;
  //! Access U channel
  inline T U() const;
  //! Access V channel
  inline T V() const;
  //! Set Y channel value, clamping & converting as necessary
  template <class T2> inline void setY(const T2 Y);
  //! Set U channel value, clamping & converting as necessary
  template <class T2> inline void setU(const T2 U);
  //! Set V channel value, clamping & converting as necessary
  template <class T2> inline void setV(const T2 V);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the basic video-YUV pixel class
// ######################################################################
// ######################################################################

//! Derived YUV pixel class from Pixels
template <class T> class PixVideoYUV :
  public Pixels<T,3>,
  public pix_ops_mixin<PixVideoYUV, T>
{
private:
public:
  const static enum PixType pType = PixTypeYUV;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixVideoYUV();
  inline ~PixVideoYUV();
  explicit inline PixVideoYUV(const T val);

  template <class T2> explicit inline PixVideoYUV(const PixVideoYUV<T2>& A);
  template <class T2> explicit inline PixVideoYUV(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixVideoYUV(const PixHyper<T2,3>& A);

  // default operator=() OK

  inline PixVideoYUV(const PixVideoYUV<T>& pix);

  //! Construct an RGB pixel with given values red, green, blue
  inline PixVideoYUV(const T Yval, const T Uval, const T Vval);
  //! Construct from given values, clamping & converting as needed
  template <class T2, class T3, class T4> inline
  PixVideoYUV(const T2 Yval, const T3 Uval, const T4 Vval);

  // special local methods

  inline void getVideoYUV(T& y, T& u, T& v) const;

  //! Access Y channel
  inline T Y() const;
  //! Access U channel
  inline T U() const;
  //! Access V channel
  inline T V() const;
  //! Set Y channel value, clamping & converting as necessary
  template <class T2> inline void setY(const T2 Y);
  //! Set U channel value, clamping & converting as necessary
  template <class T2> inline void setU(const T2 U);
  //! Set V channel value, clamping & converting as necessary
  template <class T2> inline void setV(const T2 V);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the basic YIQ pixel class
// ######################################################################
// ######################################################################

//! Derived YIQ pixel class from Pixels
template <class T> class PixYIQ :
  public Pixels<T,3>,
  public pix_ops_mixin<PixYIQ, T>
{
private:
public:
  const static enum PixType pType = PixTypeYIQ;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixYIQ();
  inline ~PixYIQ();
  explicit inline PixYIQ(const T val);

  template <class T2> explicit inline PixYIQ(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixYIQ(const PixHSV<T2>&     A);
  template <class T2> explicit inline PixYIQ(const PixYIQ<T2>&     A);
  template <class T2> explicit inline PixYIQ(const PixHSL<T2>&     A);
  template <class T2> explicit inline PixYIQ(const PixH2SV1<T2>&   A);
  template <class T2> explicit inline PixYIQ(const PixH2SV2<T2>&   A);
  template <class T2> explicit inline PixYIQ(const PixH2SV3<T2>&   A);
  template <class T2> explicit inline PixYIQ(const PixHyper<T2,3>& A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixYIQ(const PixYIQ<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixYIQ(const T Yval, const T Ival, const T Qval);
  //! Construct from given values, clamping & converting as needed
  template <class T2, class T3, class T4> inline
  PixYIQ(const T2 Yval, const T3 Uval, const T4 Vval);

  // special local methods

  inline void getYIQ(T& y, T& i, T& q) const;

  //! Access Y channel
  inline T Y() const;
  //! Access I channel
  inline T I() const;
  //! Access Q channel
  inline T Q() const;
  //! Set Y channel value, clamping & converting as necessary
  template <class T2> inline void setY(const T2 Y);
  //! Set I channel value, clamping & converting as necessary
  template <class T2> inline void setI(const T2 I);
  //! Set Q channel value, clamping & converting as necessary
  template <class T2> inline void setQ(const T2 Q);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the basic HSL pixel class
// ######################################################################
// ######################################################################

//! Derived HSL pixel class from Pixels
template <class T> class PixHSL :
  public Pixels<T,3>,
  public pix_ops_mixin<PixHSL, T>
{
private:
public:
  const static enum PixType pType = PixTypeHSL;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixHSL();
  inline ~PixHSL();
  explicit inline PixHSL(const T val);

  template <class T2> explicit inline PixHSL(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixHSL(const PixHSV<T2>&     A);
  template <class T2> explicit inline PixHSL(const PixYIQ<T2>&     A);
  template <class T2> explicit inline PixHSL(const PixHSL<T2>&     A);
  template <class T2> explicit inline PixHSL(const PixH2SV1<T2>&   A);
  template <class T2> explicit inline PixHSL(const PixH2SV2<T2>&   A);
  template <class T2> explicit inline PixHSL(const PixH2SV3<T2>&   A);
  template <class T2> explicit inline PixHSL(const PixHyper<T2,3>& A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixHSL(const PixHSL<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixHSL(const T Hval, const T Sval, const T Lval);
  //! Construct from given values, clamping & convertPixing as needed
  template <class T2, class T3, class T4> inline
  PixHSL(const T2 Hval, const T3 Sval, const T4 Lval);

  // special local methods

  inline void getHSL(T& h, T& s, T& l) const;

  //! Access hue channel
  inline T H() const;
  //! Access sat channel
  inline T S() const;
  //! Access val channel
  inline T L() const;
  //! Set hue channel value, clamping & converting as necessary
  template <class T2> inline void setH(const T2 H);
  //! Set sat channel value, clamping & converting as necessary
  template <class T2> inline void setS(const T2 S);
  //! Set val channel value, clamping & converting as necessary
  template <class T2> inline void setL(const T2 L);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the basic Lab pixel class
// ######################################################################
// ######################################################################

//! Derived Lab pixel class from Pixels
template <class T> class PixLab :
  public Pixels<T,3>,
  public pix_ops_mixin<PixLab, T>
{
private:
public:
  const static enum PixType pType = PixTypeLab;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixLab();
  inline ~PixLab();
  explicit inline PixLab(const T val);

  template <class T2> explicit inline PixLab(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixLab(const PixLab<T2>&     A);
  template <class T2> explicit inline PixLab(const PixHyper<T2,3>& A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixLab(const PixLab<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixLab(const T Lval, const T Aval, const T Bval);
  //! Construct from given values, clamping & convertPixing as needed
  template <class T2, class T3, class T4> inline
  PixLab(const T2 Lval, const T3 Aval, const T4 Bval);

  // special local methods

  inline void getLab(T& l, T& a, T& b) const;

  //! Access hue channel
  inline T L() const;
  //! Access sat channel
  inline T A() const;
  //! Access val channel
  inline T B() const;
  //! Set hue channel value, clamping & converting as necessary
  template <class T2> inline void setL(const T2 L);
  //! Set sat channel value, clamping & converting as necessary
  template <class T2> inline void setA(const T2 A);
  //! Set val channel value, clamping & converting as necessary
  template <class T2> inline void setB(const T2 B);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the basic XYZ pixel class
// ######################################################################
// ######################################################################

//! Derived XYZ pixel class from Pixels
template <class T> class PixXYZ :
  public Pixels<T,3>,
  public pix_ops_mixin<PixXYZ, T>
{
private:
public:
  const static enum PixType pType = PixTypeXYZ;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixXYZ();
  inline ~PixXYZ();
  explicit inline PixXYZ(const T val);

  template <class T2> explicit inline PixXYZ(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixXYZ(const PixXYZ<T2>&     A);
  template <class T2> explicit inline PixXYZ(const PixHyper<T2,3>& A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixXYZ(const PixXYZ<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixXYZ(const T Xval, const T Yval, const T Zval);
  //! Construct from given values, clamping & convertPixing as needed
  template <class T2, class T3, class T4> inline
  PixXYZ(const T2 Xval, const T3 Yval, const T4 Zval);

  // special local methods

  inline void getXYZ(T& x, T& y, T& z) const;

  //! Access hue channel
  inline T X() const;
  //! Access sat channel
  inline T Y() const;
  //! Access val channel
  inline T Z() const;
  //! Set hue channel value, clamping & converting as necessary
  template <class T2> inline void setX(const T2 X);
  //! Set sat channel value, clamping & converting as necessary
  template <class T2> inline void setY(const T2 Y);
  //! Set val channel value, clamping & converting as necessary
  template <class T2> inline void setZ(const T2 Z);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the H2SV1 pixel class
/*! This is the HSV color space normalized and de-modulized into
    components H1 H2 S and V. This is used since the modulus nature
    of hue makes it hard to deal with in many types of computation.
    PixH2SV1 is a very linear transforamtion.
    PixH2SV2 is linear but is B/Y R/G opponent in nature
    PixH2SV3 is as bio as this can get but is less linear
*/
// ######################################################################
// ######################################################################

//! Derived H2SV1 pixel class from Pixels
template <class T> class PixH2SV1 :
  public Pixels<T,4>,
  public pix_ops_mixin<PixH2SV1, T>
{
private:
public:
  const static enum PixType pType = PixTypeH2SV1;
  const static size_t myDim = 4;
  using Pixels<T,4>::p;

  inline PixH2SV1();
  inline ~PixH2SV1();
  explicit inline PixH2SV1(const T val);

  template <class T2> explicit inline PixH2SV1(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixH2SV1(const PixHSV<T2>&     A);
  template <class T2> explicit inline PixH2SV1(const PixYIQ<T2>&     A);
  template <class T2> explicit inline PixH2SV1(const PixHSL<T2>&     A);
  template <class T2> explicit inline PixH2SV1(const PixH2SV1<T2>&   A);
  template <class T2> explicit inline PixH2SV1(const PixH2SV2<T2>&   A);
  template <class T2> explicit inline PixH2SV1(const PixH2SV3<T2>&   A);
  template <class T2> explicit inline PixH2SV1(const PixHyper<T2,4>& A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixH2SV1(const PixH2SV1<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixH2SV1(const T H1val, const T H2val, const T Sval, const T Vval);
  //! Construct from given values, clamping & converting as needed
  template <class T2, class T3, class T4, class T5> inline
  PixH2SV1(const T2 H1val, const T3 H2val, const T4 Sval, const T5 Vval );

  // special local methods
  inline void getH2SV1(T& h1, T& h2, T& s, T& v) const;

  //! Access H1 channel
  inline T H1() const;
  //! Access H2 channel
  inline T H2() const;
  //! Access S channel
  inline T S() const;
  //! Access V channel
  inline T V() const;
  //! Set H1 channel value, clamping & converting as necessary
  template <class T2> inline void setH1(const T2 H1);
  //! Set H2 channel value, clamping & converting as necessary
  template <class T2> inline void setH2(const T2 H2);
  //! Set S channel value, clamping & converting as necessary
  template <class T2> inline void setS(const T2 S);
  //! Set V channel value, clamping & converting as necessary
  template <class T2> inline void setV(const T2 V);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the H2SV2 pixel class
/*! This is the HSV color space normalized and de-modulized into
    components H1 H2 S and V. This is used since the modulus nature
    of hue makes it hard to deal with in many types of computation.
    PixH2SV1 is a very linear transforamtion.
    PixH2SV2 is linear but is B/Y R/G opponent in nature
    PixH2SV3 is as bio as this can get but is less linear
*/
// ######################################################################
// ######################################################################

//! Derived H2SV2 pixel class from Pixels
template <class T> class PixH2SV2 :
  public Pixels<T,4>,
  public pix_ops_mixin<PixH2SV2, T>
{
private:
public:
  const static enum PixType pType = PixTypeH2SV2;
  const static size_t myDim = 4;
  using Pixels<T,4>::p;

  inline PixH2SV2();
  inline ~PixH2SV2();
  explicit inline PixH2SV2(const T val);

  template <class T2> explicit inline PixH2SV2(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixH2SV2(const PixHSV<T2>&     A);
  template <class T2> explicit inline PixH2SV2(const PixYIQ<T2>&     A);
  template <class T2> explicit inline PixH2SV2(const PixHSL<T2>&     A);
  template <class T2> explicit inline PixH2SV2(const PixH2SV1<T2>&   A);
  template <class T2> explicit inline PixH2SV2(const PixH2SV2<T2>&   A);
  template <class T2> explicit inline PixH2SV2(const PixH2SV3<T2>&   A);
  template <class T2> explicit inline PixH2SV2(const PixHyper<T2,4>& A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixH2SV2(const PixH2SV2<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixH2SV2(const T H1val, const T H2val, const T Sval, const T Vval);
  //! Construct from given values, clamping & converting as needed
  template <class T2, class T3, class T4, class T5> inline
  PixH2SV2(const T2 H1val, const T3 H2val, const T4 Sval, const T5 Vval );

  // special local methods

  inline void getH2SV2(T& h1, T& h2, T& s, T& v) const;

  //! Access H1 channel
  inline T H1() const;
  //! Access H2 channel
  inline T H2() const;
  //! Access S channel
  inline T S() const;
  //! Access V channel
  inline T V() const;
  //! Set H1 channel value, clamping & converting as necessary
  template <class T2> inline void setH1(const T2 H1);
  //! Set H2 channel value, clamping & converting as necessary
  template <class T2> inline void setH2(const T2 H2);
  //! Set S channel value, clamping & converting as necessary
  template <class T2> inline void setS(const T2 S);
  //! Set V channel value, clamping & converting as necessary
  template <class T2> inline void setV(const T2 V);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the H2SV3 pixel class
/*! This is the HSV color space normalized and de-modulized into
    components H1 H2 S and V. This is used since the modulus nature
    of hue makes it hard to deal with in many types of computation.
    PixH2SV1 is a very linear transforamtion.
    PixH2SV2 is linear but is B/Y R/G opponent in nature
    PixH2SV3 is as bio as this can get but is less linear
*/
// ######################################################################
// ######################################################################

//! Derived H2SV3 pixel class from Pixels
template <class T> class PixH2SV3 :
  public Pixels<T,4>,
  public pix_ops_mixin<PixH2SV3, T>
{
private:
public:
  const static enum PixType pType = PixTypeH2SV3;
  const static size_t myDim = 4;
  using Pixels<T,4>::p;

  inline PixH2SV3();
  inline ~PixH2SV3();
  explicit inline PixH2SV3(const T val);

  template <class T2> explicit inline PixH2SV3(const PixYIQ<T2>&     A);
  template <class T2> explicit inline PixH2SV3(const PixH2SV1<T2>&   A);
  template <class T2> explicit inline PixH2SV3(const PixH2SV2<T2>&   A);
  template <class T2> explicit inline PixH2SV3(const PixH2SV3<T2>&   A);
  template <class T2> explicit inline PixH2SV3(const PixHyper<T2,4>& A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixH2SV3(const PixH2SV3<T>& pix);
  //! Construct an RGB pixel with given values red, green, blue
  inline PixH2SV3(const T H1val, const T H2val, const T Sval, const T Vval);
  //! Construct from given values, clamping & converting as needed
  template <class T2, class T3, class T4, class T5> inline
  PixH2SV3(const T2 H1val, const T3 H2val, const T4 Sval, const T5 Vval );

  // special local methods

  inline void getH2SV3(T& h1, T& h2, T& s, T& v) const;

  //! Access H1 channel
  inline T H1() const;
  //! Access H2 channel
  inline T H2() const;
  //! Access S channel
  inline T S() const;
  //! Access V channel
  inline T V() const;
  //! Set H1 channel value, clamping & converting as necessary
  template <class T2> inline void setH1(const T2 H1);
  //! Set H2 channel value, clamping & converting as necessary
  template <class T2> inline void setH2(const T2 H2);
  //! Set S channel value, clamping & converting as necessary
  template <class T2> inline void setS(const T2 S);
  //! Set V channel value, clamping & converting as necessary
  template <class T2> inline void setV(const T2 V);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the hyper pixel class
// ######################################################################
// ######################################################################

//! Derived Hyper spectral pixel class from Pixels
template <class T, size_t dim> class PixHyper :
  public Pixels<T,dim>,
  public pix_ops_mixin_hyper<PixHyper, T, dim>
{
private:
public:
  const static enum PixType pType = PixTypeHyper;
  const static size_t myDim = dim;

  using Pixels<T,dim>::p;

  inline PixHyper();
  inline ~PixHyper();
  explicit inline PixHyper(const T val);

  template <class T2> explicit inline PixHyper(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixHyper(const PixHSV<T2>&     A);
  template <class T2> explicit inline PixHyper(const PixJpegYUV<T2>& A);
  template <class T2> explicit inline PixHyper(const PixVideoYUV<T2>& A);
  template <class T2> explicit inline PixHyper(const PixYIQ<T2>&     A);
  template <class T2> explicit inline PixHyper(const PixHSL<T2>&     A);
  template <class T2> explicit inline PixHyper(const PixH2SV1<T2>&   A);
  template <class T2> explicit inline PixHyper(const PixH2SV2<T2>&   A);
  template <class T2> explicit inline PixHyper(const PixH2SV3<T2>&   A);
  template <class T2> explicit inline PixHyper(const PixHyper<T2,dim>& A);

  //! construct pixel from another. Defined here for tech reasons
  inline PixHyper(const PixHyper<T,dim>& pix);
};

// ######################################################################
// ######################################################################
//! This is the basic DKL pixel class
// ######################################################################
// ######################################################################

//! Derived DKL pixel class from Pixels
template <class T> class PixDKL :
  public Pixels<T,3>,
  public pix_ops_mixin<PixDKL, T>
{
private:
public:
  const static enum PixType pType = PixTypeDKL;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixDKL();
  inline ~PixDKL();
  explicit inline PixDKL(const T val);

  template <class T2> explicit inline PixDKL(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixDKL(const PixDKL<T2>&     A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixDKL(const PixDKL<T>& pix);
  //! Construct a pixel with given values for D, K, L
  inline PixDKL(const T Dval, const T Kval, const T Lval);
  //! Construct from given values, clamping & convertPixing as needed
  template <class T2, class T3, class T4> inline
  PixDKL(const T2 Dval, const T3 Kval, const T4 Lval);

  // special local methods

  inline void getDKL(T& d, T& k, T& l) const;

  //! Access hue channel
  inline T D() const;
  //! Access sat channel
  inline T K() const;
  //! Access val channel
  inline T L() const;
  //! Set hue channel value, clamping & converting as necessary
  template <class T2> inline void setD(const T2 D);
  //! Set sat channel value, clamping & converting as necessary
  template <class T2> inline void setK(const T2 K);
  //! Set val channel value, clamping & converting as necessary
  template <class T2> inline void setL(const T2 L);

  // Any overrides to Pixel methods
};

// ######################################################################
// ######################################################################
//! This is the basic DKLM pixel class
// ######################################################################
// ######################################################################

//! Derived DKLM pixel class from Pixels
template <class T> class PixDKLM :
  public Pixels<T,3>,
  public pix_ops_mixin<PixDKLM, T>
{
private:
public:
  const static enum PixType pType = PixTypeDKLM;
  const static size_t myDim = 3;
  using Pixels<T,3>::p;

  inline PixDKLM();
  inline ~PixDKLM();
  explicit inline PixDKLM(const T val);

  template <class T2> explicit inline PixDKLM(const PixRGB<T2>&     A);
  template <class T2> explicit inline PixDKLM(const PixDKLM<T2>&     A);

  // default operator=() OK

  //! construct pixel from another. Defined here for tech reasons
  inline PixDKLM(const PixDKLM<T>& pix);
  //! Construct a pixel with given values for D, K, L
  inline PixDKLM(const T RGval, const T BYval, const T LUMval);
  //! Construct from given values, clamping & convertPixing as needed
  template <class T2, class T3, class T4> inline
  PixDKLM(const T2 RGval, const T3 BYval, const T4 LUMval);

  // special local methods

  inline void getDKLM(T& rg, T& by, T& lum) const;

  //! Access hue channel
  inline T RG() const;
  //! Access sat channel
  inline T BY() const;
  //! Access val channel
  inline T LUM() const;
  //! Set hue channel value, clamping & converting as necessary
  template <class T2> inline void setRG(const T2 D);
  //! Set sat channel value, clamping & converting as necessary
  template <class T2> inline void setBY(const T2 K);
  //! Set val channel value, clamping & converting as necessary
  template <class T2> inline void setLUM(const T2 L);

  // Any overrides to Pixel methods
};

// #################################
// #################################
// ######################################################################
// ######################################################################
// --> PIXEL METHODS <--
// ######################################################################
// ######################################################################
// #################################
// #################################

// ######################################################################
// ######################################################################
// Pixels RGB
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixRGB<T2>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixHSV<T2>& A1)
  : Pixels<T,3>()
{
  const PixHSV<double> A( A1 );

  // p[0] = r : p[1] = g : p[2] = b
  // A[0] = h : A[1] = s : A[2] = v

  const double vf = A[2] / HSV_V_UPPER;
  const double sf = A[1] / HSV_S_UPPER;

// SEE PixelsCommonDef.H
PIX_HSV_TO_RGB_COMMON(A[0],sf,vf,p[0],p[1],p[2])

}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixXYZ<T2>& A1)
  : Pixels<T,3>()
{
  const PixXYZ<double> A( A1 );

  // p[0] = r : p[1] = g : p[2] = b
  // A[0] = x : A[1] = y : A[2] = z

// SEE PixelsCommonDef.H
  double r,g,b;

PIX_XYZ_TO_RGB_OPENCV_COMMON(A[0],A[1],A[2],r,g,b)

  p[0] = clamped_convert<T>(r);
  p[1] = clamped_convert<T>(g);
  p[2] = clamped_convert<T>(b);
}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixLab<T2>& A1)
  : Pixels<T,3>()
{
  const PixLab<double> A( A1 );

  // p[0] = r : p[1] = g : p[2] = b
  // A[0] = l : A[1] = a : A[2] = b

  double X,Y,Z,r,g,b;

// SEE PixelsCommonDef.H
PIX_LAB_TO_XYZ_OPENCV_COMMON(A[0],A[1],A[2],X,Y,Z)
// SEE PixelsCommonDef.H
PIX_XYZ_TO_RGB_OPENCV_COMMON(X,Y,Z,r,g,b)

  p[0] = clamped_convert<T>(r);
  p[1] = clamped_convert<T>(g);
  p[2] = clamped_convert<T>(b);
}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixJpegYUV<T2>& A)
  : Pixels<T,3>()
{
  // p[0] = r : p[1] = g : p[2] = b
  // A[0] = y : A[1] = u : A[2] = v
  const double yf = double(A[0]) - JPEGYUV_Y_OFFSET;
  const double uf = double(A[1]) - JPEGYUV_UV_OFFSET;
  const double vf = double(A[2]) - JPEGYUV_UV_OFFSET;

  const double rf = yf                  + JPEGYUV_R_V*vf;
  const double gf = yf + JPEGYUV_G_U*uf + JPEGYUV_G_V*vf;
  const double bf = yf + JPEGYUV_B_U*uf;

  this->p[0] = clamped_rounded_convert<T>(clampValue(rf, 0.0, 255.0));
  this->p[1] = clamped_rounded_convert<T>(clampValue(gf, 0.0, 255.0));
  this->p[2] = clamped_rounded_convert<T>(clampValue(bf, 0.0, 255.0));
}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixVideoYUV<T2>& A)
  : Pixels<T,3>()
{
  // p[0] = r : p[1] = g : p[2] = b
  // A[0] = y : A[1] = u : A[2] = v
  const double yf = double(A[0]) - VIDEOYUV_Y_OFFSET;
  const double uf = double(A[1]) - VIDEOYUV_UV_OFFSET;
  const double vf = double(A[2]) - VIDEOYUV_UV_OFFSET;

  const double rf = VIDEOYUV_RGB_Y*yf                   + VIDEOYUV_R_V*vf;
  const double gf = VIDEOYUV_RGB_Y*yf + VIDEOYUV_G_U*uf + VIDEOYUV_G_V*vf;
  const double bf = VIDEOYUV_RGB_Y*yf + VIDEOYUV_B_U*uf;

  this->p[0] = clamped_rounded_convert<T>(clampValue(rf, 0.0, 255.0));
  this->p[1] = clamped_rounded_convert<T>(clampValue(gf, 0.0, 255.0));
  this->p[2] = clamped_rounded_convert<T>(clampValue(bf, 0.0, 255.0));
}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixYIQ<T2>& A1)
  : Pixels<T,3>()
{
  const PixYIQ<double> A( A1 );

  // p[0] = r : p[1] = g : p[2] = b
  // A[0] = y : A[1] = i : A[2] = q
  p[0] = clamped_rounded_convert<T>(1.0 * A[0] + 0.956 * A[1] + 0.621 * A[2]);
  p[1] = clamped_rounded_convert<T>(1.0 * A[0] - 0.272 * A[1] - 0.647 * A[2]);
  p[2] = clamped_rounded_convert<T>(1.0 * A[0] - 1.105 * A[1] + 1.702 * A[2]);
}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixHSL<T2>& A1)
  : Pixels<T,3>()
{
  const PixHSL<double> A( A1 );

  // p[0] = r : p[1] = g : p[2] = b
  // A[0] = h : A[1] = s : A[2] = l
  const double H = A[0];
  const double S = A[1];
  const double L = A[2];

  if(S == 0)
  {
    p[0] = clamped_convert<T>(L*RGB_R_UPPER);
    p[1] = clamped_convert<T>(L*RGB_G_UPPER);
    p[2] = clamped_convert<T>(L*RGB_B_UPPER);
  }
  else
  {
    double r,g,b;
    double t2;
    if(L < 0.5) t2 = L * (1.0 + S);
    else        t2 = L + S - (L * S);
    const double t1 = 2.0 * L - t2;
    const double hk = H/360;

    // Set RED
    const double tr = hk + 1.0/3.0;
    if      (tr < 0) tr = tr + 1;
    else if (tr > 1) tr = tr - 1;
    if      (tr < 1.0/6.0) r = t1 + ((t2 - t1) * 6.0 * tr);
    else if (tr < 1.0/2.0) r = t2;
    else if (tr < 2.0/3.0) r = t1 + ((t2 - t1) * 6.0 * (2.0/3.0 - tr));
    else                   r = t1;
    r *= RGB_R_UPPER; p[0] = clamped_convert<T>(r);

    // set GREEN
    const double tg = hk;
    if      (tg < 0) tg = tr + 1;
    else if (tg > 1) tg = tg - 1;
    if      (tg < 1.0/6.0) g = t1 + ((t2 - t1) * 6.0 * tg);
    else if (tg < 1.0/2.0) g = t2;
    else if (tg < 2.0/3.0) g = t1 + ((t2 - t1) * 6.0 * (2.0/3.0 - tg));
    else                   g = t1;
    g *= RGB_G_UPPER; p[1] = clamped_convert<T>(g);

    // set BLUE
    const double tb = hk - 1.0/3.0;
    if      (tb < 0) tb = tb + 1;
    else if (tb > 1) tb = tb - 1;
    if      (tb < 1.0/6.0) b = t1 + ((t2 - t1) * 6.0 * tb);
    else if (tb < 1.0/2.0) b = t2;
    else if (tb < 2.0/3.0) b = t1 + ((t2 - t1) * 6.0 * (2.0/3.0 - tb));
    else                   b = t1;
    b *= RGB_B_UPPER; p[2] = clamped_convert<T>(b);
  }


}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixH2SV1<T2>& A1)
  : Pixels<T,3>()
{
  const PixH2SV1<double> A( A1 );

  // p[0] = r  : p[1] = g  : p[2] = b
  // A[0] = h1 : A[1] = h2 : A[2] = s : A[3] = v

  double h = 0;

// SEE PixelsCommonDef.H
PIX_H2SV1_TO_HSV_SIMPLE_COMMON(A[0],A[1],h)
// SEE PixelsCommonDef.H
PIX_HSV_TO_RGB_COMMON(h,A[2],A[3],p[0],p[1],p[2])
}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixH2SV2<T2>& A1)
  : Pixels<T,3>()
{

  const PixH2SV2<double> A( A1 );

  // p[0] = r  : p[1] = g  : p[2] = b
  // A[0] = h1 : A[1] = h2 : A[2] = s : A[3] = v

  double h  = 0;
  double sf = 0;

// SEE PixelsCommonDef.H
PIX_H2SV2_TO_HSV_ROBUST_COMMON(A[0],A[1],A[2],h,sf)
// SEE PixelsCommonDef.H
PIX_HSV_TO_RGB_COMMON(h,sf,A[3],p[0],p[1],p[2])
}

// ######################################################################
template <class T>
template <class T2> inline PixRGB<T>::PixRGB(const PixHyper<T2,3>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixRGB<T>::PixRGB() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixRGB<T>::~PixRGB()
{}

// ######################################################################
template <class T>
inline PixRGB<T>::PixRGB(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
template <class T2>
inline PixRGB<T>::PixRGB(const T2 val)
  :
  Pixels<T,3>(clamped_convert<T>(val))
{}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixRGB<T>::getRGB(T& r, T& g, T& b) const
{
  r = p[0]; g = p[1]; b = p[2];
}

template <class T>
inline PixRGB<T>::PixRGB(const PixRGB<T>& pix) : Pixels<T,3>(pix)
{}

// ######################################################################
template <class T>
inline PixRGB<T>::PixRGB(const T redval, const T greenval, const T blueval) : Pixels<T,3>()
{
  p[0] = redval; p[1] = greenval; p[2] = blueval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixRGB<T>::PixRGB(const T2 redval, const T3 greenval, const T4 blueval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(redval);
  p[1] = clamped_convert<T>(greenval);
  p[2] = clamped_convert<T>(blueval);
}

// ######################################################################
template <class T> inline T PixRGB<T>::red() const
{
  return p[0];
}

// ######################################################################
template <class T> inline T PixRGB<T>::green() const
{
  return p[1];
}

// ######################################################################
template <class T> inline T PixRGB<T>::blue() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixRGB<T>::setRed(const T2 redval)
{
  p[0] = clamped_convert<T>(redval);
}

// ######################################################################
template <class T>
template <class T2> inline void PixRGB<T>::setGreen(const T2 greenval)
{
  p[1] = clamped_convert<T>(greenval);
}

// ######################################################################
template <class T>
template <class T2> inline void PixRGB<T>::setBlue(const T2 blueval)
{
  p[2] = clamped_convert<T>(blueval);
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline void
PixRGB<T>::set(const T2 redval, const T3 greenval, const T4 blueval)
{
  p[0] = clamped_convert<T>(redval);
  p[1] = clamped_convert<T>(greenval);
  p[2] = clamped_convert<T>(blueval);
}

// ######################################################################
template <class T> inline T PixRGB<T>::luminance() const
{
  return clamped_convert<T>( (double(p[0]) + double(p[1]) + double(p[2]))
                             / 3.0 );
}

// ######################################################################
template <class T>
inline void PixRGB<T>::getHSV(T& h1, T& h2, T& s, T& v) const
{
  const PixH2SV1<double> temph2sv1(*this);
  h1 = temph2sv1.p[0];
  h2 = temph2sv1.p[1];
  s  = temph2sv1.p[2];
  v  = temph2sv1.p[3];
}

// ######################################################################
template <class T>
inline void PixRGB<T>::setHSV(const T h1, const T h2, const T s, const T v)
{
  T H1 = h1; T H2 = h2; T S = s; T V = v;
  PixH2SV1<T> A(H1,H2,S,V);
  *this = A;
}

// ######################################################################
// ######################################################################
// Pixels HSV
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T>
template <class T2> inline PixHSV<T>::PixHSV(const PixRGB<T2>& A1)
  : Pixels<T,3>()
{
  const PixRGB<double> A( A1 );

  // p[0] = h : p[1] = s : p[2] = v
  // A[0] = r : A[1] = g : A[2] = b

PIX_RGB_TO_HSV_COMMON(A[0],A[1],A[2],p[0],p[1],p[2],true)

}

// ######################################################################
template <class T>
template <class T2> inline PixHSV<T>::PixHSV(const PixHSV<T2>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSV<T>::PixHSV(const PixYIQ<T2>& A1)
  : Pixels<T,3>()
{
  const PixYIQ<double> A( A1 );
  const PixRGB<double> B( A );
  const PixHSV<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSV<T>::PixHSV(const PixHSL<T2>& A1)
  : Pixels<T,3>()
{
  const PixHSL<double> A( A1 );
  const PixRGB<double> B( A );
  const PixHSV<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSV<T>::PixHSV(const PixH2SV1<T2>& A1)
  : Pixels<T,3>()
{
  const PixH2SV1<double> A( A1 );
  // A[0] = H1, A[1] = H2

// SEE PixelsCommonDef.H
PIX_H2SV1_TO_HSV_SIMPLE_COMMON(A[0],A[1],p[0])

  p[1] = clamped_convert<T>(A[2]*HSV_S_UPPER);
  p[2] = clamped_convert<T>(A[3]*HSV_V_UPPER);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSV<T>::PixHSV(const PixH2SV2<T2>& A1)
  : Pixels<T,3>()
{
  const PixH2SV2<double> A( A1 );

  double h  = 0;
  double sf = 0;

// SEE PixelsCommonDef.H
PIX_H2SV2_TO_HSV_ROBUST_COMMON(A[0],A[1],A[2],p[0],sf)

  p[1] = clamped_convert<T>(sf*HSV_S_UPPER);
  p[2] = clamped_convert<T>(A[3]*HSV_V_UPPER);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSV<T>::PixHSV(const PixHyper<T2,3>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixHSV<T>::PixHSV() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixHSV<T>::~PixHSV()
{}

// ######################################################################
template <class T>
inline PixHSV<T>::PixHSV(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixHSV<T>::PixHSV(const PixHSV<T>& pix) : Pixels<T,3>(pix)
{}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixHSV<T>::getHSV(T& h, T& s, T& v) const
{
  h = p[0]; s = p[1]; v = p[2];
}

// ######################################################################
template <class T>
inline PixHSV<T>::PixHSV(const T Hval, const T Sval, const T Vval) : Pixels<T,3>()
{
  p[0] = Hval; p[1] = Sval; p[2] = Vval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixHSV<T>::PixHSV(const T2 Hval, const T3 Sval, const T4 Vval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(Hval);
  p[1] = clamped_convert<T>(Sval);
  p[2] = clamped_convert<T>(Vval);
}

// ######################################################################
template <class T>
inline T PixHSV<T>::H() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixHSV<T>::S() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixHSV<T>::V() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixHSV<T>::setH(const T2 hh)
{
  p[0] = clamped_convert<T>(hh);
}

// ######################################################################
template <class T>
template <class T2> inline void PixHSV<T>::setS(const T2 ss)
{
  p[1] = clamped_convert<T>(ss);
}

// ######################################################################
template <class T>
template <class T2> inline void PixHSV<T>::setV(const T2 vv)
{
  p[2] = clamped_convert<T>(vv);
}

// ######################################################################
// ######################################################################
// Pixels YUV
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

// ######################################################################
template <class T>
template <class T2> inline PixJpegYUV<T>::PixJpegYUV(const PixJpegYUV<T2>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

template <class T>
template <class T2> inline PixJpegYUV<T>::PixJpegYUV(const PixRGB<T2>& A)
  : Pixels<T,3>()
{
  // p[0] = y : p[1] = u : p[2] = v
  // A[0] = r : A[1] = g : A[2] = b
  const double yf =
    JPEGYUV_Y_R*A[0] + JPEGYUV_Y_G*A[1] + JPEGYUV_Y_B*A[2];
  const double uf =
    JPEGYUV_U_R*A[0] + JPEGYUV_U_G*A[1] + JPEGYUV_U_B*A[2]
    + JPEGYUV_UV_OFFSET;
  const double vf =
    JPEGYUV_V_R*A[0] + JPEGYUV_V_G*A[1] + JPEGYUV_V_B*A[2]
    + JPEGYUV_UV_OFFSET;

  p[0] = clamped_rounded_convert<T>(clampValue(yf, 0.0, 255.0));
  p[1] = clamped_rounded_convert<T>(clampValue(uf, 0.0, 255.0));
  p[2] = clamped_rounded_convert<T>(clampValue(vf, 0.0, 255.0));
}

// ######################################################################
template <class T>
template <class T2> inline PixJpegYUV<T>::PixJpegYUV(const PixHyper<T2,3>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixJpegYUV<T>::PixJpegYUV() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixJpegYUV<T>::~PixJpegYUV()
{}

// ######################################################################
template <class T>
inline PixJpegYUV<T>::PixJpegYUV(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixJpegYUV<T>::PixJpegYUV(const PixJpegYUV<T>& pix)
  : Pixels<T,3>(pix)
{}


// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline PixJpegYUV<T>::PixJpegYUV(const T Yval, const T Uval, const T Vval)
  : Pixels<T,3>()
{
  p[0] = Yval; p[1] = Uval; p[2] = Vval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixJpegYUV<T>::PixJpegYUV(const T2 Yval, const T3 Uval, const T4 Vval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(Yval);
  p[1] = clamped_convert<T>(Uval);
  p[2] = clamped_convert<T>(Vval);
}

// ######################################################################
template <class T>
inline T PixJpegYUV<T>::Y() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixJpegYUV<T>::U() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixJpegYUV<T>::V() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixJpegYUV<T>::setY(const T2 yy)
{
  p[0] = clamped_convert<T>(yy);
}

// ######################################################################
template <class T>
template <class T2> inline void PixJpegYUV<T>::setU(const T2 uu)
{
  p[1] = clamped_convert<T>(uu);
}

// ######################################################################
template <class T>
template <class T2> inline void PixJpegYUV<T>::setV(const T2 vv)
{
  p[2] = clamped_convert<T>(vv);
}

// ######################################################################
// ######################################################################
// Pixels YUV
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

// ######################################################################
template <class T>
template <class T2> inline PixVideoYUV<T>::PixVideoYUV(const PixVideoYUV<T2>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

template <class T>
template <class T2> inline PixVideoYUV<T>::PixVideoYUV(const PixRGB<T2>& A)
  : Pixels<T,3>()
{
  // p[0] = y : p[1] = u : p[2] = v
  // A[0] = r : A[1] = g : A[2] = b

  const double yf =
    VIDEOYUV_Y_R*A[0] + VIDEOYUV_Y_G*A[1] + VIDEOYUV_Y_B*A[2]
    + double(VIDEOYUV_Y_OFFSET);
  const double uf =
    VIDEOYUV_U_R*A[0] + VIDEOYUV_U_G*A[1] + VIDEOYUV_U_B*A[2]
    + double(VIDEOYUV_UV_OFFSET);
  const double vf =
    VIDEOYUV_V_R*A[0] + VIDEOYUV_V_G*A[1] + VIDEOYUV_V_B*A[2]
    + double(VIDEOYUV_UV_OFFSET);

  p[0] = clamped_rounded_convert<T>(clampValue(yf, 0.0, 255.0));
  p[1] = clamped_rounded_convert<T>(clampValue(uf, 0.0, 255.0));
  p[2] = clamped_rounded_convert<T>(clampValue(vf, 0.0, 255.0));
}

// ######################################################################
template <class T>
template <class T2> inline PixVideoYUV<T>::PixVideoYUV(const PixHyper<T2,3>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixVideoYUV<T>::PixVideoYUV() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixVideoYUV<T>::~PixVideoYUV()
{}

// ######################################################################
template <class T>
inline PixVideoYUV<T>::PixVideoYUV(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixVideoYUV<T>::PixVideoYUV(const PixVideoYUV<T>& pix)
  : Pixels<T,3>(pix)
{}


// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixVideoYUV<T>::getVideoYUV(T& y, T& u, T& v) const
{
  y = p[0]; u = p[1]; v = p[2];
}

template <class T>
inline PixVideoYUV<T>::PixVideoYUV(const T Yval, const T Uval, const T Vval)
  : Pixels<T,3>()
{
  p[0] = Yval; p[1] = Uval; p[2] = Vval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixVideoYUV<T>::PixVideoYUV(const T2 Yval, const T3 Uval, const T4 Vval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(Yval);
  p[1] = clamped_convert<T>(Uval);
  p[2] = clamped_convert<T>(Vval);
}

// ######################################################################
template <class T>
inline T PixVideoYUV<T>::Y() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixVideoYUV<T>::U() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixVideoYUV<T>::V() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixVideoYUV<T>::setY(const T2 yy)
{
  p[0] = clamped_convert<T>(yy);
}

// ######################################################################
template <class T>
template <class T2> inline void PixVideoYUV<T>::setU(const T2 uu)
{
  p[1] = clamped_convert<T>(uu);
}

// ######################################################################
template <class T>
template <class T2> inline void PixVideoYUV<T>::setV(const T2 vv)
{
  p[2] = clamped_convert<T>(vv);
}

// ######################################################################
// ######################################################################
// Pixels YIQ
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T>
template <class T2> inline PixYIQ<T>::PixYIQ(const PixRGB<T2>& A1)
  : Pixels<T,3>()
{
  const PixRGB<double> A( A1 );
  // coeffs from Foley & Van Dam
  // p[0] = y   : p[1] = i   : p[2] = q
  // A[0] = r : A[1] = g : A[2] = b
  p[0] = clamped_rounded_convert<T>(0.299 * A[0] + 0.587 * A[1] + 0.114 * A[2]);
  p[1] = clamped_rounded_convert<T>(0.596 * A[0] - 0.275 * A[1] - 0.321 * A[2]);
  p[2] = clamped_rounded_convert<T>(0.212 * A[0] - 0.523 * A[1] + 0.311 * A[2]);
}

// ######################################################################
template <class T>
template <class T2> inline PixYIQ<T>::PixYIQ(const PixHSV<T2>& A1)
  : Pixels<T,3>()
{
  const PixHSV<double> A( A1 );
  const PixRGB<double> B( A );
  const PixYIQ<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixYIQ<T>::PixYIQ(const PixYIQ<T2>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixYIQ<T>::PixYIQ(const PixHSL<T2>& A1)
  : Pixels<T,3>()
{
  const PixHSL<double> A( A1 );
  const PixRGB<double> B( A );
  const PixYIQ<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixYIQ<T>::PixYIQ(const PixH2SV1<T2>& A1)
  : Pixels<T,3>()
{
  const PixH2SV1<double> A( A1 );
  const PixRGB<double> B( A );
  const PixYIQ<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixYIQ<T>::PixYIQ(const PixH2SV2<T2>& A1)
  : Pixels<T,3>()
{
  const PixH2SV2<double> A( A1 );
  const PixRGB<double> B( A );
  const PixYIQ<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixYIQ<T>::PixYIQ(const PixH2SV3<T2>& A1)
  : Pixels<T,3>()
{
  const PixH2SV3<double> A( A1 );
  const PixRGB<double> B( A );
  const PixYIQ<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixYIQ<T>::PixYIQ(const PixHyper<T2,3>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
// Standard Constructors / Destructor
// ######################################################################

template <class T>
inline PixYIQ<T>::PixYIQ() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixYIQ<T>::~PixYIQ()
{}

// ######################################################################
template <class T>
inline PixYIQ<T>::PixYIQ(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixYIQ<T>::PixYIQ(const PixYIQ<T>& pix) : Pixels<T,3>(pix)
{}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixYIQ<T>::getYIQ(T& y, T& i, T& q) const
{
  y = p[0]; i = p[1]; q = p[2];
}

// ######################################################################
template <class T>
inline PixYIQ<T>::PixYIQ(const T Yval, const T Ival, const T Qval) : Pixels<T,3>()
{
    p[0] = Yval; p[1] = Ival; p[2] = Qval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixYIQ<T>::PixYIQ(const T2 Yval, const T3 Ival, const T4 Qval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(Yval);
  p[1] = clamped_convert<T>(Ival);
  p[2] = clamped_convert<T>(Qval);
}

// ######################################################################
template <class T>
inline T PixYIQ<T>::Y() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixYIQ<T>::I() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixYIQ<T>::Q() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixYIQ<T>::setY(const T2 yy)
{
  p[0] = clamped_convert<T>(yy);
}

// ######################################################################
template <class T>
template <class T2> inline void PixYIQ<T>::setI(const T2 ii)
{
  p[1] = clamped_convert<T>(ii);
}

// ######################################################################
template <class T>
template <class T2> inline void PixYIQ<T>::setQ(const T2 qq)
{
  p[2] = clamped_convert<T>(qq);
}


// ######################################################################
// ######################################################################
// Pixels HSL
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T>
template <class T2> inline PixHSL<T>::PixHSL(const PixRGB<T2>& A1)
  : Pixels<T,3>()
{
  const PixRGB<double> A( A1 );

  // p[0] = h : p[1] = s : p[2] = l
  // A[0] = r : A[1] = g : A[2] = b

  const double R = A[0]/RGB_R_UPPER;
  const double G = A[1]/RGB_G_UPPER;
  const double B = A[2]/RGB_B_UPPER;

  double h,s,l;

  //Note: HSL is very much like HSV. For instance hue is computed in
  //      exactly the same way.

  if((B > G) && (B > R))
  {
    const double max = B;
    double min;
    if(R > G) min = G;
    else      min = R;
    const double posDelta = max + min;
    const double negDelta = max - min;
    l = posDelta/2;
    if(l > 0.5)       s = negDelta/(2 - posDelta);
    else              s = negDelta/posDelta;
    if(negDelta != 0) h = 4 + (R - G) / negDelta;
    else              h = 4 + (R - G);
    h *= 60; if(h < 0) h += 360;
  }
  else if(G > R)
  {
    const double max = G;
    double min;
    if(R > B) min = B;
    else      min = R;
    const double posDelta = max + min;
    const double negDelta = max - min;
    l = posDelta/2;
    if(l > 0.5)       s = negDelta/(2 - posDelta);
    else              s = negDelta/posDelta;
    if(negDelta != 0) h = 2 + (B - R) / negDelta;
    else              h = 2 + (B - R);
    h *= 60; if(h < 0) h += 360;
  }
  else
  {
    const double max = R;
    double min;
    if(G > B) min = B;
    else      min = G;
    const double posDelta = max + min;
    const double negDelta = max - min;
    l = posDelta/2;
    if(l > 0.5)       s = negDelta/(2 - posDelta);
    else              s = negDelta/posDelta;
    if(negDelta != 0) h = (G - B) / negDelta;
    else              h = (G - B);
    h *= 60; if(h < 0) h += 360;
  }

  p[0] = clamped_convert<T>(h);
  p[1] = clamped_convert<T>(s);
  p[2] = clamped_convert<T>(l);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSL<T>::PixHSL(const PixHSV<T2>& A1)
  : Pixels<T,3>()
{
  const PixHSV<double> A( A1 );
  const PixRGB<double> B( A );
  const PixHSL<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSL<T>::PixHSL(const PixYIQ<T2>& A1)
  : Pixels<T,3>()
{
  const PixYIQ<double> A( A1 );
  const PixRGB<double> B( A );
  const PixHSL<double> C( B );
  pix_helper::clamped_assign_3(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSL<T>::PixHSL(const PixHSL<T2>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSL<T>::PixHSL(const PixH2SV1<T2>& A1)
  : Pixels<T,3>()
{
  const PixH2SV1<double> A( A1 );
  // A[0] = H1, A[1] = H2

PIX_H2SV1_TO_HSV_SIMPLE_COMMON(A[0],A[1],p[0])

  p[1] = clamped_convert<T>(A[2]*HSL_S_UPPER);
  p[2] = clamped_convert<T>(A[3]*HSL_L_UPPER);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSL<T>::PixHSL(const PixH2SV2<T2>& A1)
  : Pixels<T,3>()
{
  const PixH2SV2<double> A( A1 );

  double h  = 0;
  double sf = 0;

// SEE PixelsCommonDef.H
PIX_H2SV2_TO_HSV_ROBUST_COMMON(A[0],A[1],A[2],p[0],sf)

  p[1] = clamped_convert<T>(sf*HSL_S_UPPER);
  p[2] = clamped_convert<T>(A[3]*HSL_L_UPPER);
}

// ######################################################################
template <class T>
template <class T2> inline PixHSL<T>::PixHSL(const PixHyper<T2,3>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixHSL<T>::PixHSL() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixHSL<T>::~PixHSL()
{}

// ######################################################################
template <class T>
inline PixHSL<T>::PixHSL(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixHSL<T>::PixHSL(const PixHSL<T>& pix) : Pixels<T,3>(pix)
{}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixHSL<T>::getHSL(T& h, T& s, T& l) const
{
  h = p[0]; s = p[1]; l = p[2];
}

// ######################################################################
template <class T>
inline PixHSL<T>::PixHSL(const T Hval, const T Sval, const T Lval) : Pixels<T,3>()
{
  p[0] = Hval; p[1] = Sval; p[2] = Lval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixHSL<T>::PixHSL(const T2 Hval, const T3 Sval, const T4 Lval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(Hval);
  p[1] = clamped_convert<T>(Sval);
  p[2] = clamped_convert<T>(Lval);
}

// ######################################################################
template <class T>
inline T PixHSL<T>::H() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixHSL<T>::S() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixHSL<T>::L() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixHSL<T>::setH(const T2 hh)
{
  p[0] = clamped_convert<T>(hh);
}

// ######################################################################
template <class T>
template <class T2> inline void PixHSL<T>::setS(const T2 ss)
{
  p[1] = clamped_convert<T>(ss);
}

// ######################################################################
template <class T>
template <class T2> inline void PixHSL<T>::setL(const T2 ll)
{
  p[2] = clamped_convert<T>(ll);
}
// ######################################################################
// ######################################################################
// Pixels Lab
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T>
template <class T2> inline PixLab<T>::PixLab(const PixRGB<T2>& A1)
  : Pixels<T,3>()
{
  const PixRGB<double> A( A1 );

  // p[0] = l : p[1] = a : p[2] = b
  // A[0] = r : A[1] = g : A[2] = b

  double X,Y,Z,L,a,b;

// SEE PixelsCommonDef.H
PIX_RGB_TO_XYZ_OPENCV_COMMON(A[0],A[1],A[2],X,Y,Z)
// SEE PixelsCommonDef.H
PIX_XYZ_TO_LAB_OPENCV_COMMON(X,Y,Z,L,a,b)
//PIX_XYZ_TO_LAB_AUGMENT_COMMON(X,Y,Z,L,a,b)

  p[0] = clamped_convert<T>(L);
  p[1] = clamped_convert<T>(a);
  p[2] = clamped_convert<T>(b);
}

// ######################################################################
template <class T>
template <class T2> inline PixLab<T>::PixLab(const PixLab<T2>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixLab<T>::PixLab(const PixHyper<T2,3>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixLab<T>::PixLab() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixLab<T>::~PixLab()
{}

// ######################################################################
template <class T>
inline PixLab<T>::PixLab(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixLab<T>::PixLab(const PixLab<T>& pix) : Pixels<T,3>(pix)
{}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixLab<T>::getLab(T& l, T& a, T& b) const
{
  l = p[0]; a = p[1]; b = p[2];
}

// ######################################################################
template <class T>
inline PixLab<T>::PixLab(const T Lval, const T Aval, const T Bval) : Pixels<T,3>()
{
  p[0] = Lval; p[1] = Aval; p[2] = Bval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixLab<T>::PixLab(const T2 Lval, const T3 Aval, const T4 Bval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(Lval);
  p[1] = clamped_convert<T>(Aval);
  p[2] = clamped_convert<T>(Bval);
}

// ######################################################################
template <class T>
inline T PixLab<T>::L() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixLab<T>::A() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixLab<T>::B() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixLab<T>::setL(const T2 L)
{
  p[0] = clamped_convert<T>(L);
}

// ######################################################################
template <class T>
template <class T2> inline void PixLab<T>::setA(const T2 A)
{
  p[1] = clamped_convert<T>(A);
}

// ######################################################################
template <class T>
template <class T2> inline void PixLab<T>::setB(const T2 B)
{
  p[2] = clamped_convert<T>(B);
}

// ######################################################################
// ######################################################################
// Pixels XYZ
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T>
template <class T2> inline PixXYZ<T>::PixXYZ(const PixRGB<T2>& A1)
  : Pixels<T,3>()
{
  const PixRGB<double> A( A1 );

  // p[0] = l : p[1] = a : p[2] = b
  // A[0] = r : A[1] = g : A[2] = b

  double X,Y,Z;

// SEE PixelsCommonDef.H
PIX_RGB_TO_XYZ_OPENCV_COMMON(A[0],A[1],A[2],X,Y,Z)

  p[0] = clamped_convert<T>(X);
  p[1] = clamped_convert<T>(Y);
  p[2] = clamped_convert<T>(Z);
}

// ######################################################################
template <class T>
template <class T2> inline PixXYZ<T>::PixXYZ(const PixXYZ<T2>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixXYZ<T>::PixXYZ(const PixHyper<T2,3>& A)
  : Pixels<T,3>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixXYZ<T>::PixXYZ() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixXYZ<T>::~PixXYZ()
{}

// ######################################################################
template <class T>
inline PixXYZ<T>::PixXYZ(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixXYZ<T>::PixXYZ(const PixXYZ<T>& pix) : Pixels<T,3>(pix)
{}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixXYZ<T>::getXYZ(T& x, T& y, T& z) const
{
  x = p[0]; y = p[1]; z = p[2];
}

// ######################################################################
template <class T>
inline PixXYZ<T>::PixXYZ(const T Xval, const T Yval, const T Zval) : Pixels<T,3>()
{
  p[0] = Xval; p[1] = Yval; p[2] = Zval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixXYZ<T>::PixXYZ(const T2 Xval, const T3 Yval, const T4 Zval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(Xval);
  p[1] = clamped_convert<T>(Yval);
  p[2] = clamped_convert<T>(Zval);
}

// ######################################################################
template <class T>
inline T PixXYZ<T>::X() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixXYZ<T>::Y() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixXYZ<T>::Z() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixXYZ<T>::setX(const T2 X)
{
  p[0] = clamped_convert<T>(X);
}

// ######################################################################
template <class T>
template <class T2> inline void PixXYZ<T>::setY(const T2 Y)
{
  p[1] = clamped_convert<T>(Y);
}

// ######################################################################
template <class T>
template <class T2> inline void PixXYZ<T>::setZ(const T2 Z)
{
  p[2] = clamped_convert<T>(Z);
}

// ######################################################################
// ######################################################################
// Pixels H2SV1
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T>
template <class T2> inline PixH2SV1<T>::PixH2SV1(const PixRGB<T2>& A1)
  : Pixels<T,4>()
{
  const PixRGB<double> A( A1 );

  // p[0] = h1  : p[1] = h2  : p[2] = s  : p[3] = v
  // A[0] = r : A[1] = g : A[2] = b

  double h = 0;

// SEE PixelsCommonDef.H
PIX_RGB_TO_HSV_COMMON(A[0],A[1],A[2],h,p[2],p[3],false)
// SEE PixelsCommonDef.H
PIX_HSV_TO_H2SV1_COMMON(h,p[2],p[0],p[1])

}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV1<T>::PixH2SV1(const PixHSV<T2>& A1)
  : Pixels<T,4>()
{
  const PixHSV<double> A( A1 );

  // p[0] = h1  : p[1] = h2  : p[2] = s  : p[3] = v
  // A[0] = h : A[1] = s : A[2] = v

  // demoduloize hue into two components

// SEE PixelsCommonDef.H
  PIX_HSV_TO_H2SV1_COMMON(A[0],A[1],p[0],p[1])

  p[2] = clamped_convert<T>(A[1]/HSV_S_UPPER);
  p[3] = clamped_convert<T>(A[2]/HSV_V_UPPER);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV1<T>::PixH2SV1(const PixYIQ<T2>& A1)
  : Pixels<T,4>()
{
  const PixYIQ<double>   A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV1<double> C( B );
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV1<T>::PixH2SV1(const PixHSL<T2>& A1)
  : Pixels<T,4>()
{
  const PixHSL<double> A( A1 );

  // p[0] = h1  : p[1] = h2  : p[2] = s  : p[3] = v
  // A[0] = h : A[1] = s : A[2] = l

  // demoduloize hue into two components

// SEE PixelsCommonDef.H
PIX_HSV_TO_H2SV1_COMMON(A[0],A[1],p[0],p[1])

  p[2] = clamped_convert<T>(A[1]/HSL_S_UPPER);
  p[3] = clamped_convert<T>(A[2]/HSL_L_UPPER);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV1<T>::PixH2SV1(const PixH2SV1<T2>& A)
  : Pixels<T,4>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV1<T>::PixH2SV1(const PixH2SV2<T2>& A1)
  : Pixels<T,4>()
{
  const PixH2SV2<double> A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV1<double> C( B );
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV1<T>::PixH2SV1(const PixH2SV3<T2>& A1)
  : Pixels<T,4>()
{
  const PixH2SV3<double> A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV1<double> C( B );
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV1<T>::PixH2SV1(const PixHyper<T2,4>& A)
  : Pixels<T,4>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
// Standard Constructors / Destructor
// ######################################################################

template <class T>
inline PixH2SV1<T>::PixH2SV1() : Pixels<T,4>()
{}

// ######################################################################
template <class T>
inline PixH2SV1<T>::~PixH2SV1()
{}

// ######################################################################
template <class T>
inline PixH2SV1<T>::PixH2SV1(T val) : Pixels<T,4>(val)
{}

// ######################################################################
template <class T>
inline PixH2SV1<T>::PixH2SV1(const PixH2SV1<T>& pix) : Pixels<T,4>(pix)
{}


// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixH2SV1<T>::getH2SV1(T& h1, T& h2, T& s, T& v) const
{
  h1 = p[0]; h2 = p[1]; s = p[2]; v = p[3];
}

// ######################################################################
template <class T>
inline PixH2SV1<T>::PixH2SV1(const T H1val, const T H2val, const T Sval, const T Vval) : Pixels<T,4>()
{
  p[0] = H1val; p[1] = H2val; p[2] = Sval; p[3] = Vval;
}

// ######################################################################
template <class T>
template <class T2, class T3, class T4, class T5> inline
PixH2SV1<T>::PixH2SV1(const T2 H1val, const T3 H2val, const T4 Sval, const T5 Vval ) : Pixels<T,4>()
{
  p[0] = clamped_convert<T>(H1val);
  p[1] = clamped_convert<T>(H2val);
  p[2] = clamped_convert<T>(Sval);
  p[3] = clamped_convert<T>(Vval);
}

// ######################################################################
template <class T>
inline T PixH2SV1<T>::H1() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixH2SV1<T>::H2() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixH2SV1<T>::S() const
{
  return p[2];
}

// ######################################################################
template <class T>
inline T PixH2SV1<T>::V() const
{
  return p[3];
}

// ######################################################################
template <class T>
template <class T2> inline void PixH2SV1<T>::setH1(const T2 hh1)
{
  p[0] = clamped_convert<T>(hh1);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV1<T>::setH2(const T2 hh2)
{
  p[1] = clamped_convert<T>(hh2);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV1<T>::setS(const T2 ss)
{
  p[2] = clamped_convert<T>(ss);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV1<T>::setV(const T2 vv)
{
  p[3] = clamped_convert<T>(vv);
}

// ######################################################################
// ######################################################################
// Pixels H2SV2
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T>
template <class T2> inline PixH2SV2<T>::PixH2SV2(const PixRGB<T2>& A1)
  : Pixels<T,4>()
{

  const PixRGB<double> A( A1 );

  // p[0] = h1  : p[1] = h2  : p[2] = s  : p[3] = v
  // A[0] = r : A[1] = g : A[2] = b
  double h = 0;

// SEE PixelsCommonDef.H
PIX_RGB_TO_HSV_COMMON(A[0],A[1],A[2],h,p[2],p[3],false)
// SEE PixelsCommonDef.H
PIX_HSV_TO_H2SV2_COMMON(h,p[2],p[0],p[1])
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV2<T>::PixH2SV2(const PixHSV<T2>& A1)
  : Pixels<T,4>()
{
  const PixH2SV2<double> A( A1 );

  // p[0] = h1  : p[1] = h2  : p[2] = s  : p[3] = v
  // A[0] = h : A[1] = s : A[2] = v

  // demoduloize hue into two components B/Y and R/G like

// SEE PixelsCommonDef.H
PIX_HSV_TO_H2SV2_COMMON(A[0],A[1],p[0],p[1])

  p[2] = clamped_convert<T>(A[1]/HSV_S_UPPER);
  p[3] = clamped_convert<T>(A[2]/HSV_V_UPPER);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV2<T>::PixH2SV2(const PixYIQ<T2>& A1)
  : Pixels<T,4>()
{
  const PixYIQ<double>   A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV2<double> C( B ); // FIXME shouldn't this be PixH2SV2???
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV2<T>::PixH2SV2(const PixHSL<T2>& A1)
  : Pixels<T,4>()
{
  const PixH2SV2<double> A( A1 );

  // p[0] = h1  : p[1] = h2  : p[2] = s  : p[3] = v
  // A[0] = h : A[1] = s : A[2] = l

  // demoduloize hue into two components B/Y and R/G like

// SEE PixelsCommonDef.H
PIX_HSV_TO_H2SV2_COMMON(A[0],A[1],p[0],p[1])

  p[2] = clamped_convert<T>(A[1]/HSL_S_UPPER);
  p[3] = clamped_convert<T>(A[2]/HSL_L_UPPER);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV2<T>::PixH2SV2(const PixH2SV1<T2>& A1)
  : Pixels<T,4>()
{
  const PixH2SV1<double> A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV2<double> C( B ); // FIXME shouldn't this be PixH2SV2???
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV2<T>::PixH2SV2(const PixH2SV2<T2>& A)
  : Pixels<T,4>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV2<T>::PixH2SV2(const PixH2SV3<T2>& A1)
  : Pixels<T,4>()
{
  const PixH2SV3<double> A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV2<double> C( B ); // FIXME shouldn't this be PixH2SV2???
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV2<T>::PixH2SV2(const PixHyper<T2,4>& A)
  : Pixels<T,4>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
// Standard Constructors / Destructor
// ######################################################################

template <class T>
inline PixH2SV2<T>::PixH2SV2() : Pixels<T,4>()
{}

// ######################################################################
template <class T>
inline PixH2SV2<T>::~PixH2SV2()
{}

// ######################################################################
template <class T>
inline PixH2SV2<T>::PixH2SV2(const T val) : Pixels<T,4>(val)
{}

// ######################################################################
template <class T>
inline PixH2SV2<T>::PixH2SV2(const PixH2SV2<T>& pix) : Pixels<T,4>(pix)
{}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixH2SV2<T>::getH2SV2(T& h1, T& h2, T& s, T& v) const
{
  h1 = p[0]; h2 = p[1]; s = p[2]; v = p[3];
}

template <class T>
inline PixH2SV2<T>::PixH2SV2(const T H1val, const T H2val, const T Sval, const T Vval) : Pixels<T,4>()
{
  p[0] = H1val; p[1] = H2val; p[2] = Sval; p[3] = Vval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4, class T5> inline
PixH2SV2<T>::PixH2SV2(const T2 H1val, const T3 H2val, const T4 Sval, const T5 Vval ) : Pixels<T,4>()
{
  p[0] = clamped_convert<T>(H1val);
  p[1] = clamped_convert<T>(H2val);
  p[2] = clamped_convert<T>(Sval);
  p[3] = clamped_convert<T>(Vval);
}

// ######################################################################
template <class T>
inline T PixH2SV2<T>::H1() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixH2SV2<T>::H2() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixH2SV2<T>::S() const
{
  return p[2];
}

// ######################################################################
template <class T>
inline T PixH2SV2<T>::V() const
{
  return p[3];
}

// ######################################################################
template <class T>
template <class T2> inline void PixH2SV2<T>::setH1(const T2 hh1)
{
  p[0] = clamped_convert<T>(hh1);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV2<T>::setH2(const T2 hh2)
{
  p[1] = clamped_convert<T>(hh2);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV2<T>::setS(const T2 ss)
{
  p[2] = clamped_convert<T>(ss);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV2<T>::setV(const T2 vv)
{
  p[3] = clamped_convert<T>(vv);
}

// ######################################################################
// ######################################################################
// Pixels H2SV3
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

// ######################################################################
template <class T>
template <class T2> inline PixH2SV3<T>::PixH2SV3(const PixYIQ<T2>& A1)
  : Pixels<T,4>()
{
  const PixYIQ<double> A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV3<double> C( B );
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV3<T>::PixH2SV3(const PixH2SV1<T2>& A1)
  : Pixels<T,4>()
{
  const PixH2SV1<double> A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV3<double> C( B );
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV3<T>::PixH2SV3(const PixH2SV2<T2>& A1)
  : Pixels<T,4>()
{
  const PixH2SV2<double> A( A1 );
  const PixRGB<double>   B( A );
  const PixH2SV3<double> C( B );
  pix_helper::clamped_assign_4(*this, C);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV3<T>::PixH2SV3(const PixH2SV3<T2>& A)
  : Pixels<T,4>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
template <class T>
template <class T2> inline PixH2SV3<T>::PixH2SV3(const PixHyper<T2,4>& A)
  : Pixels<T,4>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
// Standard Constructors / Destructor
// ######################################################################

template <class T>
inline PixH2SV3<T>::PixH2SV3() : Pixels<T,4>()
{}

// ######################################################################
template <class T>
inline PixH2SV3<T>::~PixH2SV3()
{}

// ######################################################################
template <class T>
inline PixH2SV3<T>::PixH2SV3(const T val) : Pixels<T,4>(val)
{}

// ######################################################################
template <class T>
inline PixH2SV3<T>::PixH2SV3(const PixH2SV3<T>& pix) : Pixels<T,4>(pix)
{}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixH2SV3<T>::getH2SV3(T& h1, T& h2, T& s, T& v) const
{
  h1 = p[0]; h2 = p[1]; s = p[2]; v = p[3];
}

template <class T>
inline PixH2SV3<T>::PixH2SV3(const T H1val, const T H2val, const T Sval, const T Vval) : Pixels<T,4>()
{
  p[0] = H1val; p[1] = H2val; p[2] = Sval; p[3] = Vval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4, class T5> inline
PixH2SV3<T>::PixH2SV3(const T2 H1val, const T3 H2val, const T4 Sval, const T5 Vval ) : Pixels<T,4>()
{
  p[0] = clamped_convert<T>(H1val);
  p[1] = clamped_convert<T>(H2val);
  p[2] = clamped_convert<T>(Sval);
  p[3] = clamped_convert<T>(Vval);
}

// ######################################################################
template <class T>
inline T PixH2SV3<T>::H1() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixH2SV3<T>::H2() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixH2SV3<T>::S() const
{
  return p[2];
}

// ######################################################################
template <class T>
inline T PixH2SV3<T>::V() const
{
  return p[3];
}

// ######################################################################
template <class T>
template <class T2> inline void PixH2SV3<T>::setH1(const T2 hh1)
{
  p[0] = clamped_convert<T>(hh1);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV3<T>::setH2(const T2 hh2)
{
  p[1] = clamped_convert<T>(hh2);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV3<T>::setS(const T2 ss)
{
  p[2] = clamped_convert<T>(ss);
}
// ######################################################################
template <class T>
template <class T2> inline void PixH2SV3<T>::setV(const T2 vv)
{
  p[3] = clamped_convert<T>(vv);
}

// ######################################################################
// ######################################################################
// Pixels Hyper
// ######################################################################
// ######################################################################

// ######################################################################
// constructors for where Pix<T> != Pix<T2> - with conversion
// ######################################################################

template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixRGB<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixHSV<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixJpegYUV<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixVideoYUV<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixYIQ<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixHSL<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_3(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixH2SV1<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixH2SV2<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixH2SV3<T2>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_4(*this, A);
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline PixHyper<T,dim>::PixHyper(const PixHyper<T2,dim>& A)
  : Pixels<T,dim>()
{
  pix_helper::clamped_assign_n(*this, A);
}

// ######################################################################
// Standard Constructors / Destructor
// ######################################################################

template <class T, size_t dim>
inline PixHyper<T,dim>::PixHyper() : Pixels<T,dim>()
{}

// ######################################################################
template <class T, size_t dim>
inline PixHyper<T,dim>::~PixHyper()
{}

// ######################################################################
template <class T, size_t dim>
inline PixHyper<T,dim>::PixHyper(const T val) : Pixels<T,dim>(val)
{}

// ######################################################################
template <class T, size_t dim>
inline PixHyper<T,dim>::PixHyper(const PixHyper<T,dim>& pix) : Pixels<T,dim>(pix)
{}


// ######################################################################
// ######################################################################
// Pixels DKL
// ######################################################################
// ######################################################################

template <class T>
template <class T2> inline PixDKL<T>::PixDKL(const PixRGB<T2>& A1)
  : Pixels<T,3>()
{
  // bunch of constants used for RGB -> DKL conversion:
  static const double lut_rgb[256*3]  = {
    0.024935, 0.0076954, 0.042291,
    0.024974, 0.0077395, 0.042346,
    0.025013, 0.0077836, 0.042401,
    0.025052, 0.0078277, 0.042456,
    0.025091, 0.0078717, 0.042511,
    0.02513, 0.0079158, 0.042566,
    0.025234, 0.007992, 0.042621,
    0.025338, 0.0080681, 0.042676,
    0.025442, 0.0081443, 0.042731,
    0.025545, 0.0082204, 0.042786,
    0.025649, 0.0082966, 0.042841,
    0.025747, 0.0084168, 0.042952,
    0.025844, 0.0085371, 0.043062,
    0.025942, 0.0086573, 0.043172,
    0.026039, 0.0087776, 0.043282,
    0.026136, 0.0088978, 0.043392,
    0.026234, 0.0090581, 0.043502,
    0.026331, 0.0092184, 0.043612,
    0.026429, 0.0093788, 0.043722,
    0.026526, 0.0095391, 0.043833,
    0.026623, 0.0096994, 0.043943,
    0.026818, 0.0099198, 0.044141,
    0.027013, 0.01014, 0.044339,
    0.027208, 0.010361, 0.044537,
    0.027403, 0.010581, 0.044736,
    0.027597, 0.010802, 0.044934,
    0.027857, 0.010994, 0.04522,
    0.028117, 0.011186, 0.045507,
    0.028377, 0.011379, 0.045793,
    0.028636, 0.011571, 0.046079,
    0.028896, 0.011764, 0.046366,
    0.029104, 0.012068, 0.046652,
    0.029312, 0.012373, 0.046938,
    0.029519, 0.012677, 0.047225,
    0.029727, 0.012982, 0.047511,
    0.029935, 0.013287, 0.047797,
    0.030273, 0.013663, 0.048326,
    0.03061, 0.01404, 0.048855,
    0.030948, 0.014417, 0.049383,
    0.031286, 0.014794, 0.049912,
    0.031623, 0.01517, 0.050441,
    0.032156, 0.015707, 0.051035,
    0.032688, 0.016244, 0.05163,
    0.033221, 0.016782, 0.052225,
    0.033753, 0.017319, 0.052819,
    0.034286, 0.017856, 0.053414,
    0.034961, 0.018693, 0.054493,
    0.035636, 0.019531, 0.055573,
    0.036312, 0.020369, 0.056652,
    0.036987, 0.021206, 0.057731,
    0.037662, 0.022044, 0.058811,
    0.038623, 0.023246, 0.060044,
    0.039584, 0.024449, 0.061278,
    0.040545, 0.025651, 0.062511,
    0.041506, 0.026854, 0.063744,
    0.042468, 0.028056, 0.064978,
    0.043857, 0.029659, 0.066806,
    0.045247, 0.031263, 0.068634,
    0.046636, 0.032866, 0.070463,
    0.048026, 0.034469, 0.072291,
    0.049416, 0.036072, 0.074119,
    0.051221, 0.038156, 0.076476,
    0.053026, 0.04024, 0.078833,
    0.054831, 0.042325, 0.081189,
    0.056636, 0.044409, 0.083546,
    0.058442, 0.046493, 0.085903,
    0.06039, 0.048737, 0.087996,
    0.062338, 0.050982, 0.090088,
    0.064286, 0.053226, 0.092181,
    0.066234, 0.055471, 0.094273,
    0.068182, 0.057715, 0.096366,
    0.070519, 0.06012, 0.098921,
    0.072857, 0.062525, 0.10148,
    0.075195, 0.06493, 0.10403,
    0.077532, 0.067335, 0.10659,
    0.07987, 0.069739, 0.10914,
    0.082208, 0.072345, 0.11176,
    0.084545, 0.07495, 0.11438,
    0.086883, 0.077555, 0.117,
    0.089221, 0.08016, 0.11963,
    0.091558, 0.082766, 0.12225,
    0.094026, 0.085611, 0.12533,
    0.096494, 0.088457, 0.12841,
    0.098961, 0.091303, 0.1315,
    0.10143, 0.094148, 0.13458,
    0.1039, 0.096994, 0.13767,
    0.10688, 0.10028, 0.14119,
    0.10987, 0.10357, 0.14471,
    0.11286, 0.10685, 0.14824,
    0.11584, 0.11014, 0.15176,
    0.11883, 0.11343, 0.15529,
    0.12208, 0.11695, 0.15903,
    0.12532, 0.12048, 0.16278,
    0.12857, 0.12401, 0.16652,
    0.13182, 0.12754, 0.17026,
    0.13506, 0.13106, 0.17401,
    0.1387, 0.13499, 0.17819,
    0.14234, 0.13892, 0.18238,
    0.14597, 0.14285, 0.18656,
    0.14961, 0.14677, 0.19075,
    0.15325, 0.1507, 0.19493,
    0.15727, 0.15519, 0.19956,
    0.1613, 0.15968, 0.20419,
    0.16532, 0.16417, 0.20881,
    0.16935, 0.16866, 0.21344,
    0.17338, 0.17315, 0.21806,
    0.17805, 0.17796, 0.22291,
    0.18273, 0.18277, 0.22775,
    0.1874, 0.18758, 0.2326,
    0.19208, 0.19238, 0.23744,
    0.19675, 0.19719, 0.24229,
    0.20156, 0.20224, 0.24758,
    0.20636, 0.20729, 0.25286,
    0.21117, 0.21234, 0.25815,
    0.21597, 0.21739, 0.26344,
    0.22078, 0.22244, 0.26872,
    0.2261, 0.22806, 0.27423,
    0.23143, 0.23367, 0.27974,
    0.23675, 0.23928, 0.28524,
    0.24208, 0.24489, 0.29075,
    0.2474, 0.2505, 0.29626,
    0.25299, 0.25651, 0.3022,
    0.25857, 0.26253, 0.30815,
    0.26416, 0.26854, 0.3141,
    0.26974, 0.27455, 0.32004,
    0.27532, 0.28056, 0.32599,
    0.28156, 0.28697, 0.33238,
    0.28779, 0.29339, 0.33877,
    0.29403, 0.2998, 0.34515,
    0.30026, 0.30621, 0.35154,
    0.30649, 0.31263, 0.35793,
    0.3126, 0.31904, 0.36388,
    0.3187, 0.32545, 0.36982,
    0.32481, 0.33186, 0.37577,
    0.33091, 0.33828, 0.38172,
    0.33701, 0.34469, 0.38767,
    0.34325, 0.3511, 0.39361,
    0.34948, 0.35752, 0.39956,
    0.35571, 0.36393, 0.40551,
    0.36195, 0.37034, 0.41145,
    0.36818, 0.37675, 0.4174,
    0.37429, 0.38317, 0.42313,
    0.38039, 0.38958, 0.42885,
    0.38649, 0.39599, 0.43458,
    0.3926, 0.4024, 0.44031,
    0.3987, 0.40882, 0.44604,
    0.40494, 0.41523, 0.45198,
    0.41117, 0.42164, 0.45793,
    0.4174, 0.42806, 0.46388,
    0.42364, 0.43447, 0.46982,
    0.42987, 0.44088, 0.47577,
    0.43623, 0.44689, 0.48128,
    0.4426, 0.45291, 0.48678,
    0.44896, 0.45892, 0.49229,
    0.45532, 0.46493, 0.4978,
    0.46169, 0.47094, 0.5033,
    0.46792, 0.47695, 0.50837,
    0.47416, 0.48297, 0.51344,
    0.48039, 0.48898, 0.5185,
    0.48662, 0.49499, 0.52357,
    0.49286, 0.501, 0.52863,
    0.49805, 0.50701, 0.53392,
    0.50325, 0.51303, 0.53921,
    0.50844, 0.51904, 0.54449,
    0.51364, 0.52505, 0.54978,
    0.51883, 0.53106, 0.55507,
    0.52442, 0.53667, 0.55969,
    0.53, 0.54228, 0.56432,
    0.53558, 0.5479, 0.56894,
    0.54117, 0.55351, 0.57357,
    0.54675, 0.55912, 0.57819,
    0.55182, 0.56433, 0.58304,
    0.55688, 0.56954, 0.58789,
    0.56195, 0.57475, 0.59273,
    0.56701, 0.57996, 0.59758,
    0.57208, 0.58517, 0.60242,
    0.57675, 0.58998, 0.60639,
    0.58143, 0.59479, 0.61035,
    0.5861, 0.5996, 0.61432,
    0.59078, 0.60441, 0.61828,
    0.59545, 0.60922, 0.62225,
    0.60065, 0.61403, 0.62709,
    0.60584, 0.61884, 0.63194,
    0.61104, 0.62365, 0.63678,
    0.61623, 0.62846, 0.64163,
    0.62143, 0.63327, 0.64648,
    0.62584, 0.63808, 0.65088,
    0.63026, 0.64289, 0.65529,
    0.63468, 0.6477, 0.65969,
    0.63909, 0.65251, 0.6641,
    0.64351, 0.65731, 0.6685,
    0.64857, 0.66132, 0.67269,
    0.65364, 0.66533, 0.67687,
    0.6587, 0.66934, 0.68106,
    0.66377, 0.67335, 0.68524,
    0.66883, 0.67735, 0.68943,
    0.67273, 0.68136, 0.69361,
    0.67662, 0.68537, 0.6978,
    0.68052, 0.68938, 0.70198,
    0.68442, 0.69339, 0.70617,
    0.68831, 0.69739, 0.71035,
    0.69221, 0.7022, 0.7141,
    0.6961, 0.70701, 0.71784,
    0.7, 0.71182, 0.72159,
    0.7039, 0.71663, 0.72533,
    0.70779, 0.72144, 0.72907,
    0.71169, 0.72505, 0.73348,
    0.71558, 0.72866, 0.73789,
    0.71948, 0.73226, 0.74229,
    0.72338, 0.73587, 0.7467,
    0.72727, 0.73948, 0.7511,
    0.73247, 0.74349, 0.75507,
    0.73766, 0.74749, 0.75903,
    0.74286, 0.7515, 0.763,
    0.74805, 0.75551, 0.76696,
    0.75325, 0.75952, 0.77093,
    0.75714, 0.76393, 0.77599,
    0.76104, 0.76834, 0.78106,
    0.76494, 0.77275, 0.78612,
    0.76883, 0.77715, 0.79119,
    0.77273, 0.78156, 0.79626,
    0.77792, 0.78677, 0.80132,
    0.78312, 0.79198, 0.80639,
    0.78831, 0.79719, 0.81145,
    0.79351, 0.8024, 0.81652,
    0.7987, 0.80762, 0.82159,
    0.80519, 0.81283, 0.82687,
    0.81169, 0.81804, 0.83216,
    0.81818, 0.82325, 0.83744,
    0.82468, 0.82846, 0.84273,
    0.83117, 0.83367, 0.84802,
    0.83636, 0.83888, 0.85286,
    0.84156, 0.84409, 0.85771,
    0.84675, 0.8493, 0.86256,
    0.85195, 0.85451, 0.8674,
    0.85714, 0.85972, 0.87225,
    0.86364, 0.86613, 0.87819,
    0.87013, 0.87255, 0.88414,
    0.87662, 0.87896, 0.89009,
    0.88312, 0.88537, 0.89604,
    0.88961, 0.89178, 0.90198,
    0.8961, 0.8986, 0.90947,
    0.9026, 0.90541, 0.91696,
    0.90909, 0.91222, 0.92445,
    0.91558, 0.91904, 0.93194,
    0.92208, 0.92585, 0.93943,
    0.92857, 0.93307, 0.94493,
    0.93506, 0.94028, 0.95044,
    0.94156, 0.94749, 0.95595,
    0.94805, 0.95471, 0.96145,
    0.95455, 0.96192, 0.96696,
    0.96364, 0.96954, 0.97357,
    0.97273, 0.97715, 0.98018,
    0.98182, 0.98477, 0.98678,
    0.99091, 0.99238, 0.99339,
    1, 1, 1 };

  static const double lms0[3] = { 34.918538957799996, 19.314796676499999, 0.585610818500000 };

  static const double m[9] = { 18.32535,  44.60077,   7.46216,
                               4.09544,  28.20135,   6.66066,
                               0.02114,   0.10325,   1.05258 };

  static const double fac = 1.0 / (lms0[0] + lms0[1]);

  static const double mm[9] = {
    sqrt(3.0)*fac, sqrt(3.0)*fac, 0.0,
    sqrt(lms0[0]*lms0[0]+lms0[1]*lms0[1])/lms0[0]*fac, -sqrt(lms0[0]*lms0[0]+lms0[1]*lms0[1])/lms0[1]*fac, 0.0,
    -fac, -fac, (lms0[0] + lms0[1]) / lms0[2] * fac };

  // convert RGB values to bytes, with clamping:
  const PixRGB<byte> A(A1);

  // do a lookup RGB -> rgb:
  const double aa1 = lut_rgb[A[0] * 3];
  const double aa2 = lut_rgb[A[1] * 3 + 1];
  const double aa3 = lut_rgb[A[2] * 3 + 2];

  // now convert to LMS:
  const double lms1 = m[0] * aa1 + m[1] * aa2 + m[2] * aa3 - lms0[0];
  const double lms2 = m[3] * aa1 + m[4] * aa2 + m[5] * aa3 - lms0[1];
  const double lms3 = m[6] * aa1 + m[7] * aa2 + m[8] * aa3 - lms0[2];

  // finally to DKL:
  const double dkl1 = mm[0] * lms1 + mm[1] * lms2 + mm[2] * lms3;
  const double dkl2 = mm[3] * lms1 + mm[4] * lms2 + mm[5] * lms3;
  const double dkl3 = mm[6] * lms1 + mm[7] * lms2 + mm[8] * lms3;

  // finally to DKLn:
  p[0] = clamped_convert<T>(dkl1 * 0.5774);
  p[1] = clamped_convert<T>(dkl2 * 2.7525);
  p[2] = clamped_convert<T>(dkl3 * 0.4526);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixDKL<T>::PixDKL() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixDKL<T>::~PixDKL()
{}

// ######################################################################
template <class T>
inline PixDKL<T>::PixDKL(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixDKL<T>::PixDKL(const PixDKL<T>& pix) : Pixels<T,3>(pix)
{}

// ######################################################################
template <class T>
template <class T2> inline PixDKL<T>::PixDKL(const PixDKL<T2>& A1)
  : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(A1[0]);
  p[1] = clamped_convert<T>(A1[1]);
  p[2] = clamped_convert<T>(A1[2]);
}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixDKL<T>::getDKL(T& d, T& k, T& l) const
{
  d = p[0]; k = p[1]; l = p[2];
}

// ######################################################################
template <class T>
inline PixDKL<T>::PixDKL(const T Dval, const T Kval, const T Lval) : Pixels<T,3>()
{
  p[0] = Dval; p[1] = Kval; p[2] = Lval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixDKL<T>::PixDKL(const T2 Dval, const T3 Kval, const T4 Lval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(Dval);
  p[1] = clamped_convert<T>(Kval);
  p[2] = clamped_convert<T>(Lval);
}

// ######################################################################
template <class T>
inline T PixDKL<T>::D() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixDKL<T>::K() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixDKL<T>::L() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixDKL<T>::setD(const T2 dd)
{
  p[0] = clamped_convert<T>(dd);
}

// ######################################################################
template <class T>
template <class T2> inline void PixDKL<T>::setK(const T2 kk)
{
  p[1] = clamped_convert<T>(kk);
}

// ######################################################################
template <class T>
template <class T2> inline void PixDKL<T>::setL(const T2 ll)
{
  p[2] = clamped_convert<T>(ll);
}

// ######################################################################
// ######################################################################
// Pixels DKLM
// ######################################################################
// ######################################################################

template <class T>
template <class T2> inline PixDKLM<T>::PixDKLM(const PixRGB<T2>& A1)
  : Pixels<T,3>()
{
  // bunch of constants used for RGB -> DKL conversion:
  static const double lut_rgb[256*3]  = 
    {0.0099042,0.021611,0,
     0.046316,0.0552,0,
     0.067066,0.076415,0.058219,
     0.083651,0.093499,0.083037,
     0.097974,0.10827,0.10217,
     0.11081,0.1215,0.11835,
     0.12258,0.13361,0.13263,
     0.13351,0.14485,0.14558,
     0.14379,0.1554,0.1575,
     0.15351,0.16538,0.16861,
     0.16278,0.17487,0.17905,
     0.17165,0.18394,0.18895,
     0.18017,0.19265,0.19837,
     0.18839,0.20104,0.20737,
     0.19633,0.20914,0.21602,
     0.20403,0.21698,0.22435,
     0.21151,0.22459,0.23239,
     0.21879,0.23199,0.24017,
     0.22588,0.2392,0.24771,
     0.2328,0.24622,0.25504,
     0.23956,0.25308,0.26218,
     0.24618,0.25979,0.26913,
     0.25266,0.26635,0.27591,
     0.25901,0.27278,0.28254,
     0.26524,0.27908,0.28902,
     0.27136,0.28526,0.29537,
     0.27737,0.29133,0.30159,
     0.28328,0.2973,0.30768,
     0.2891,0.30317,0.31366,
     0.29483,0.30894,0.31954,
     0.30047,0.31462,0.32531,
     0.30603,0.32022,0.33099,
     0.31151,0.32574,0.33657,
     0.31691,0.33118,0.34207,
     0.32224,0.33654,0.34748,
     0.32751,0.34183,0.35281,
     0.33271,0.34705,0.35807,
     0.33785,0.35221,0.36325,
     0.34292,0.35731,0.36836,
     0.34794,0.36234,0.37341,
     0.3529,0.36732,0.37839,
     0.35781,0.37223,0.38331,
     0.36266,0.3771,0.38817,
     0.36747,0.38191,0.39297,
     0.37223,0.38667,0.39772,
     0.37693,0.39138,0.40241,
     0.3816,0.39605,0.40705,
     0.38622,0.40067,0.41164,
     0.39079,0.40524,0.41618,
     0.39533,0.40977,0.42068,
     0.39982,0.41426,0.42512,
     0.40427,0.41871,0.42953,
     0.40869,0.42312,0.43389,
     0.41307,0.42749,0.43821,
     0.41741,0.43182,0.44249,
     0.42172,0.43611,0.44673,
     0.42599,0.44037,0.45093,
     0.43023,0.4446,0.45509,
     0.43444,0.44879,0.45922,
     0.43862,0.45295,0.46331,
     0.44276,0.45708,0.46737,
     0.44687,0.46117,0.4714,
     0.45096,0.46524,0.47539,
     0.45502,0.46927,0.47935,
     0.45904,0.47328,0.48328,
     0.46304,0.47726,0.48718,
     0.46702,0.48121,0.49104,
     0.47096,0.48513,0.49488,
     0.47489,0.48902,0.4987,
     0.47878,0.49289,0.50248,
     0.48265,0.49674,0.50624,
     0.4865,0.50056,0.50996,
     0.49032,0.50435,0.51367,
     0.49412,0.50812,0.51735,
     0.4979,0.51187,0.521,
     0.50166,0.51559,0.52463,
     0.50539,0.51929,0.52823,
     0.5091,0.52297,0.53181,
     0.51279,0.52663,0.53537,
     0.51646,0.53027,0.53891,
     0.52011,0.53388,0.54242,
     0.52374,0.53748,0.54591,
     0.52735,0.54105,0.54938,
     0.53094,0.5446,0.55283,
     0.53451,0.54814,0.55626,
     0.53806,0.55165,0.55967,
     0.5416,0.55515,0.56306,
     0.54511,0.55863,0.56643,
     0.54861,0.56209,0.56978,
     0.55209,0.56553,0.57311,
     0.55556,0.56896,0.57643,
     0.559,0.57236,0.57972,
     0.56243,0.57575,0.583,
     0.56585,0.57913,0.58626,
     0.56925,0.58248,0.5895,
     0.57263,0.58582,0.59273,
     0.576,0.58915,0.59593,
     0.57935,0.59246,0.59913,
     0.58268,0.59575,0.6023,
     0.58601,0.59903,0.60546,
     0.58931,0.60229,0.6086,
     0.5926,0.60554,0.61173,
     0.59588,0.60877,0.61485,
     0.59915,0.61199,0.61794,
     0.60239,0.61519,0.62103,
     0.60563,0.61838,0.6241,
     0.60885,0.62156,0.62715,
     0.61206,0.62472,0.63019,
     0.61526,0.62787,0.63322,
     0.61844,0.631,0.63623,
     0.62161,0.63413,0.63923,
     0.62477,0.63724,0.64221,
     0.62791,0.64033,0.64518,
     0.63105,0.64342,0.64814,
     0.63417,0.64649,0.65109,
     0.63728,0.64955,0.65402,
     0.64037,0.6526,0.65694,
     0.64346,0.65563,0.65985,
     0.64653,0.65866,0.66274,
     0.64959,0.66167,0.66563,
     0.65265,0.66467,0.6685,
     0.65569,0.66766,0.67136,
     0.65871,0.67063,0.67421,
     0.66173,0.6736,0.67704,
     0.66474,0.67655,0.67987,
     0.66773,0.6795,0.68268,
     0.67072,0.68243,0.68549,
     0.6737,0.68536,0.68828,
     0.67666,0.68827,0.69106,
     0.67962,0.69117,0.69383,
     0.68256,0.69406,0.69659,
     0.6855,0.69694,0.69934,
     0.68842,0.69982,0.70208,
     0.69134,0.70268,0.70481,
     0.69424,0.70553,0.70753,
     0.69714,0.70837,0.71023,
     0.70003,0.7112,0.71293,
     0.7029,0.71403,0.71562,
     0.70577,0.71684,0.7183,
     0.70863,0.71964,0.72097,
     0.71148,0.72244,0.72363,
     0.71432,0.72523,0.72628,
     0.71715,0.728,0.72892,
     0.71998,0.73077,0.73156,
     0.72279,0.73353,0.73418,
     0.7256,0.73628,0.7368,
     0.7284,0.73902,0.7394,
     0.73119,0.74176,0.742,
     0.73397,0.74448,0.74459,
     0.73674,0.7472,0.74717,
     0.73951,0.7499,0.74974,
     0.74226,0.7526,0.7523,
     0.74501,0.75529,0.75485,
     0.74775,0.75798,0.7574,
     0.75049,0.76065,0.75994,
     0.75321,0.76332,0.76247,
     0.75593,0.76598,0.76499,
     0.75864,0.76863,0.7675,
     0.76134,0.77128,0.77001,
     0.76403,0.77391,0.77251,
     0.76672,0.77654,0.775,
     0.7694,0.77916,0.77748,
     0.77207,0.78178,0.77995,
     0.77474,0.78438,0.78242,
     0.7774,0.78698,0.78488,
     0.78005,0.78957,0.78733,
     0.78269,0.79216,0.78978,
     0.78533,0.79474,0.79222,
     0.78796,0.79731,0.79465,
     0.79058,0.79987,0.79707,
     0.7932,0.80243,0.79949,
     0.79581,0.80498,0.8019,
     0.79841,0.80752,0.8043,
     0.80101,0.81006,0.8067,
     0.8036,0.81259,0.80909,
     0.80618,0.81511,0.81147,
     0.80876,0.81762,0.81384,
     0.81133,0.82013,0.81621,
     0.81389,0.82264,0.81858,
     0.81645,0.82513,0.82093,
     0.819,0.82762,0.82328,
     0.82154,0.83011,0.82562,
     0.82408,0.83259,0.82796,
     0.82662,0.83506,0.83029,
     0.82914,0.83752,0.83262,
     0.83166,0.83998,0.83493,
     0.83418,0.84244,0.83725,
     0.83669,0.84488,0.83955,
     0.83919,0.84732,0.84185,
     0.84169,0.84976,0.84414,
     0.84418,0.85219,0.84643,
     0.84666,0.85461,0.84871,
     0.84914,0.85703,0.85099,
     0.85162,0.85944,0.85326,
     0.85408,0.86185,0.85552,
     0.85655,0.86425,0.85778,
     0.859,0.86664,0.86003,
     0.86146,0.86903,0.86228,
     0.8639,0.87142,0.86452,
     0.86634,0.87379,0.86676,
     0.86878,0.87617,0.86899,
     0.87121,0.87854,0.87121,
     0.87363,0.8809,0.87343,
     0.87605,0.88325,0.87565,
     0.87847,0.8856,0.87786,
     0.88088,0.88795,0.88006,
     0.88328,0.89029,0.88226,
     0.88568,0.89263,0.88445,
     0.88807,0.89496,0.88664,
     0.89046,0.89728,0.88882,
     0.89285,0.8996,0.891,
     0.89522,0.90192,0.89317,
     0.8976,0.90423,0.89534,
     0.89997,0.90653,0.8975,
     0.90233,0.90883,0.89965,
     0.90469,0.91113,0.90181,
     0.90704,0.91342,0.90395,
     0.90939,0.91571,0.9061,
     0.91174,0.91799,0.90823,
     0.91408,0.92026,0.91037,
     0.91641,0.92253,0.91249,
     0.91874,0.9248,0.91462,
     0.92107,0.92706,0.91674,
     0.92339,0.92932,0.91885,
     0.92571,0.93157,0.92096,
     0.92802,0.93382,0.92306,
     0.93033,0.93606,0.92516,
     0.93263,0.9383,0.92726,
     0.93493,0.94054,0.92935,
     0.93722,0.94277,0.93144,
     0.93951,0.94499,0.93352,
     0.9418,0.94721,0.93559,
     0.94408,0.94943,0.93767,
     0.94636,0.95164,0.93974,
     0.94863,0.95385,0.9418,
     0.9509,0.95605,0.94386,
     0.95316,0.95825,0.94591,
     0.95542,0.96045,0.94797,
     0.95768,0.96264,0.95001,
     0.95993,0.96482,0.95206,
     0.96218,0.96701,0.95409,
     0.96442,0.96918,0.95613,
     0.96666,0.97136,0.95816,
     0.9689,0.97353,0.96018,
     0.97113,0.97569,0.96221,
     0.97335,0.97786,0.96422,
     0.97558,0.98001,0.96624,
     0.9778,0.98217,0.96825,
     0.98001,0.98432,0.97025,
     0.98222,0.98646,0.97226,
     0.98443,0.9886,0.97425,
     0.98663,0.99074,0.97625,
     0.98883,0.99288,0.97824,
     0.99103,0.99501,0.98022,
     0.99322,0.99713,0.98221,
     1,1,1};

  static const double XYZrgb[9] = {65.6238, 49.1126, 37.8203,
                                   34.0351, 99.2207, 16.5735,
                                   2.2731, 17.5876, 200.9760};

  static const double LMSxyz[9] = {0.7328, 0.4296, -0.1624, 
                                   -0.7036, 1.6975, 0.0061,
                                   0.0030, 0.0136, 0.9834};

  // convert RGB values to bytes, with clamping:
  const PixRGB<byte> A(A1);
  
  // do a lookup RGB -> gamma corrected and normalized [0-1]:
  const double ar = lut_rgb[A[0] * 3];
  const double ag = lut_rgb[A[1] * 3 + 1];
  const double ab = lut_rgb[A[2] * 3 + 2];
  
  //convert RGB -> XYZ
  const double X = XYZrgb[0]*ar + XYZrgb[1]*ag + XYZrgb[2]*ab; 
  const double Y = XYZrgb[3]*ar + XYZrgb[4]*ag + XYZrgb[5]*ab; 
  const double Z = XYZrgb[6]*ar + XYZrgb[7]*ag + XYZrgb[8]*ab; 

  //convert XYZ -> LMS
  const double L = LMSxyz[0]*X + LMSxyz[1]*Y + LMSxyz[2]*Z; 
  const double M = LMSxyz[3]*X + LMSxyz[4]*Y + LMSxyz[5]*Z; 
  const double S = LMSxyz[6]*X + LMSxyz[7]*Y + LMSxyz[8]*Z;

  //convert LMS -> DKL
  const double RG   = (M - L + 1) / 2;
  const double BY   = ((M + L) / 2 - S + 1) / 2;
  const double LUM  = (M + L + S) / 3;

  p[0] = clamped_convert<T>(RG);
  p[1] = clamped_convert<T>(BY);
  p[2] = clamped_convert<T>(LUM);
}

// ######################################################################
// Standard Constructors / Destrcutor
// ######################################################################

template <class T>
inline PixDKLM<T>::PixDKLM() : Pixels<T,3>()
{}

// ######################################################################
template <class T>
inline PixDKLM<T>::~PixDKLM()
{}

// ######################################################################
template <class T>
inline PixDKLM<T>::PixDKLM(const T val) : Pixels<T,3>(val)
{}

// ######################################################################
template <class T>
inline PixDKLM<T>::PixDKLM(const PixDKLM<T>& pix) : Pixels<T,3>(pix)
{}

// ######################################################################
template <class T>
template <class T2> inline PixDKLM<T>::PixDKLM(const PixDKLM<T2>& A1)
  : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(A1[0]);
  p[1] = clamped_convert<T>(A1[1]);
  p[2] = clamped_convert<T>(A1[2]);
}

// ######################################################################
// Special methods for this class
// ######################################################################

template <class T>
inline void PixDKLM<T>::getDKLM(T& rg, T& by, T& lum) const
{
  rg = p[0]; by = p[1]; lum = p[2];
}

// ######################################################################
template <class T>
inline PixDKLM<T>::PixDKLM(const T RGval, const T BYval, const T LUMval) : Pixels<T,3>()
{
  p[0] = RGval; p[1] = BYval; p[2] = LUMval;
}

// ######################################################################
template <class T> template <class T2, class T3, class T4> inline
PixDKLM<T>::PixDKLM(const T2 RGval, const T3 BYval, const T4 LUMval) : Pixels<T,3>()
{
  p[0] = clamped_convert<T>(RGval);
  p[1] = clamped_convert<T>(BYval);
  p[2] = clamped_convert<T>(LUMval);
}

// ######################################################################
template <class T>
inline T PixDKLM<T>::RG() const
{
  return p[0];
}

// ######################################################################
template <class T>
inline T PixDKLM<T>::BY() const
{
  return p[1];
}

// ######################################################################
template <class T>
inline T PixDKLM<T>::LUM() const
{
  return p[2];
}

// ######################################################################
template <class T>
template <class T2> inline void PixDKLM<T>::setRG(const T2 rg)
{
  p[0] = clamped_convert<T>(rg);
}

// ######################################################################
template <class T>
template <class T2> inline void PixDKLM<T>::setBY(const T2 by)
{
  p[1] = clamped_convert<T>(by);
}

// ######################################################################
template <class T>
template <class T2> inline void PixDKLM<T>::setLUM(const T2 ll)
{
  p[2] = clamped_convert<T>(ll);
}

// ######################################################################
// Specialize promote_trait for pixel types
// ######################################################################

/* The SPECIALIZE_PROMOTE_TRAIT macro is defined in
   Util/Promotions.H. Note that these SPECIALIZE_PROMOTE_TRAIT() macro
   calls should NOT go in Util/Promotions.H, since the Util/ directory
   is supposed to be independent of pixels, images, etc. */

//! Quickie macro for escaping the comma out of a macro argument.
#define LIST_2(p1, p2) p1, p2

/* If you need to add a dim to a hyper pixel you can add it here */
SPECIALIZE_PROMOTE_TRAIT(PixHyper, LIST_2(class T, size_t dim), LIST_2(, 3))
SPECIALIZE_PROMOTE_TRAIT(PixHyper, LIST_2(class T, size_t dim), LIST_2(, 4))

#undef LIST_2

SPECIALIZE_PROMOTE_TRAIT(PixRGB, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixHSV, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixJpegYUV, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixVideoYUV, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixYIQ, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixHSL, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixH2SV1, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixH2SV2, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixH2SV3, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixDKL, class T, )
SPECIALIZE_PROMOTE_TRAIT(PixDKLM, class T, )

#endif
