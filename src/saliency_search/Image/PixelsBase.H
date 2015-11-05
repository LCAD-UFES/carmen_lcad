/*!@file Image/PixelsBase.H Basic pixel types version 2.0 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PixelsBase.H $
// $Id: PixelsBase.H 15059 2011-11-11 00:55:58Z dberg $
//

#ifndef PIXELSBASE_H_DEFINED
#define PIXELSBASE_H_DEFINED

#include "Util/Types.H"
#include "Util/TypeTraits.H"
#include "Util/log.H"
#include "Util/Promotions.H"
#include <cmath>
#include <cstdlib> // for size_t

// ######################################################################
// ######################################################################
// Macros for overloading on base types
// ######################################################################
// ######################################################################

//! Enumerate pixel types as a convenience
enum PixType
{
  PixTypeBase,
  PixTypeRGB,
  PixTypeHSV,
  PixTypeYUV,
  PixTypeYIQ,
  PixTypeHSL,
  PixTypeLab,
  PixTypeXYZ,
  PixTypeH2SV1,
  PixTypeH2SV2,
  PixTypeH2SV3,
  PixTypeHyper,
  PixTypeDKL,
  PixTypeDKLM,
};

// ######################################################################
// ######################################################################
// Range values for certain pixel types
// ######################################################################
// ######################################################################

//! RGB ranges R Upper bound
const int RGB_R_UPPER = 255;
//! RGB ranges R Lower bound
const int RGB_R_LOWER = 0;
//! RGB ranges G Upper bound
const int RGB_G_UPPER = 255;
//! RGB ranges G Lower bound
const int RGB_G_LOWER = 0;
//! RGB ranges B Upper bound
const int RGB_B_UPPER = 255;
//! RGB ranges B Lower bound
const int RGB_B_LOWER = 0;

//! HSV Ranges H Upper bound
const int HSV_H_UPPER = 360;
//! HSV Ranges H Lower bound
const int HSV_H_LOWER = 0;
//! HSV Ranges S Upper bound
const int HSV_S_UPPER = 100;
//! HSV Ranges S Lower bound
const int HSV_S_LOWER = 0;
//! HSV Ranges V Upper bound
const int HSV_V_UPPER = 255;
//! HSV Ranges V Lower bound
const int HSV_V_LOWER = 0;

//! HSL Ranges H Upper bound
const int HSL_H_UPPER = 360;
//! HSL Ranges H Lower bound
const int HSL_H_LOWER = 0;
//! HSL Ranges S Upper bound
const int HSL_S_UPPER = 1;
//! HSL Ranges S Lower bound
const int HSL_S_LOWER = 0;
//! HSL Ranges V Upper bound
const int HSL_L_UPPER = 1;
//! HSL Ranges V Lower bound
const int HSL_L_LOWER = 0;

//! YUV ranges Y Upper bound
const int YUV_Y_UPPER = 255;
//! YUV ranges Y Lower bound
const int YUV_Y_LOWER = 0;
//! YUV ranges U Upper bound
const int YUV_U_UPPER = 255;
//! YUV ranges U Lower bound
const int YUV_U_LOWER = 0;
//! YUV ranges V Upper bound
const int YUV_V_UPPER = 255;
//! YUV ranges V Lower bound
const int YUV_V_LOWER = 0;

//! LAB ranges L Upper bound (Normalized to 0 - 255)
const int LAB_L_UPPER = 255;
//! LAB ranges L Lower bound
const int LAB_L_LOWER = 0;
//! LAB ranges A Upper bound (normalized from +/-128)
const int LAB_A_UPPER = 255;
//! LAB ranges A Lower bound
const int LAB_A_LOWER = 0;
//! LAB ranges B Upper bound (normalized from +/-128)
const int LAB_B_UPPER = 255;
//! LAB ranges B Lower bound
const int LAB_B_LOWER = 0;

//! H2SV ranges (all types) H1 Upper Bound
const double H2SV_H1_UPPER = 1.0;
//! H2SV ranges (all types) H1 Lower Bound
const double H2SV_H1_LOWER = 0.0;
//! H2SV ranges (all types) H2 Upper Bound
const double H2SV_H2_UPPER = 1.0;
//! H2SV ranges (all types) H2 Lower Bound
const double H2SV_H2_LOWER = 0.0;
//! H2SV ranges (all types) S Upper Bound
const double H2SV_S_UPPER =  1.0;
//! H2SV ranges (all types) S Lower Bound
const double H2SV_S_LOWER =  0.0;
//! H2SV ranges (all types) V Upper Bound
const double H2SV_V_UPPER =  1.0;
//! H2SV ranges (all types) V Lower Bound
const double H2SV_V_LOWER =  0.0;

// ######################################################################
// ######################################################################
// BASE PIXELS CLASS
// ######################################################################
// ######################################################################

template <class T, size_t dim> class Pixels;

//! Specialize TypeTraits to indicate that Pixels is a "trivial" type.
template <class T, size_t dim>
struct TypeTraits<Pixels<T,dim> >
{
  enum { isTrivial = TypeTraits<T>::isTrivial };
};

//! This is the base Pixel class for a pixel of any size
/*! This class contains all base operators and constructors to create
    a pixel of any size. To create specific pixel types (i.e. RGB HSV YUV)
    derive from this class then add the translation functions for typing
    and constructors.

    The base class is mainly used to contain the data and access it raw.
    Since data from one pixel to another needs to be converted and translated
    into a different space, all operations between pixels is handled at the
    level of the derived class to force compile time binding and to make sure
    that all bindings are handeled correctly.

    ********************************************
    All pixels have several constant attributes:

    const static enum PixType pType - This is an enumeration for the
                                      type of pixel this is

    const static size_t myDim - This is how many dimensions (dim) I
                                have for the base class and hyper
                                spectral pixels, this is contained in
                                the template parameter dim at
                                declaration.

    <size_t dim> - This is applied to Pixels and PixHyper at
                   declaration and contains the diminsion of this
                   pixel

    ******
    Notes:

    (1) size_t is used to hold dimension of the Pixel class. on a
        32-bit machine this means that any hyper spectral image is
        limited to 2^32 pixel componenets (which should be plenty!)
    (2) Any custom free function for pixels must be accompanied by another
        free form function for a base type to keep compatability.
        Thus, if you write a function getBent(Pixels<T>) you must also make
        a getBent(T) so that it can apply to either a basic type or a pixel
        type.
    (3) All pixel types must convert to and from PixRGB so that it can be
        used as a universal translation pixel type. That is, if you want
        to convert from type XYZ to ABC you do not need to know the direct
        translation so long as you can convert from XYZ to RGB and then
        to ABC.
    (4) All arithmetic between pixels must convert between color spaces
        while performing such operations. As such PixRGB = PixRGB + PixHSV
        will convert the third pixel to a PixRGB type then add it with
        second pixel this formula. Thus, conversion is implicite in the
        math. Additionally, all pixels must support such operations
        with basic types. So PixRGB = PixRGB + float must be supported
        as well.

   ******
   Files:

   (1) Pixels.H - This file contains the free math functions such as
                       sqrt(Pixel<T>). It also contains free functions
                       for the non-sense relational operators. If you
                       add a pixel type you will only need to make a
                       small change at then end by adding macros for
                       free math functions for your pixel with the
                       PIX_FREE_MATH_FUNCTIONS.

   (2) PixelsBase.H - This file. Contains the base class and its
                       methods.  If you add a pixel, you will not need
                       to change anything in this file.

   (3) PixelsTypes.H - This file contains the derived pixel classes as
                       well as other derived pixel type specific
                       information such as TypeTraits information for
                       each pixel and the static def for each pixels
                       temporary. This file as well as
                       PixelsTypesDefine will require the most change
                       if you add a new type of pixel

  (4) Promotions.H - This file contains a macro for creating the
                     proper promotions for basic1 types. Take a look at
                     it. If you add your own pixel type you may want
                     to add the promotions and type trait info
                     here. This is simple and recomended.

  (5) test-new-Pixels - This is a very simple binary to test to pixels
                        and make sure they work as they are supposed
                        to.

  ******************************
  Current supported Pixel Types:

  PixRGB<T> - This is a basic RGB pixel of 3 dimensions. All pixels
              must be able to convert to and from this type of pixel
              at the very minumum.

  PixHSV<T> - This is a standard HSV pixel type of three dimensions.

  PixJpegYUV<T> - This is a YUV pixel of three dimensions, using
                  YUV-for-JPEG conversion conventions.

  PixVideoYUV<T> - This is a YUV pixel of three dimensions, using
                   YUV-for-video (MPEG) conversion conventions.

  PixYIQ<T> - This is a YIQ pixel of three dimensions.

  PixHSL<T> - This is a standard HSL pixel (similar to HSV).

  PixH2SV1<T> - This is a pixel derived from HSV with the hue
                demodulized.  It has four dimensions.

  PixH2SV2<T> - This is a pixel derived from HSV with the hue
                demodulized.  It has four dimensions. It has a closer
                resemblance, but a linear one to Red/Green Blue/Yellow
                opponencie than H2SV1.

  PixH2SV3<T> - This is a pixel derived from HSV with the hue
                demodulized.  It has four dimensions.  It has a closer
                resemblance, but a non-linear one to Red/Green
                Blue/Yellow opponencie than H2SV1.  It is currently in
                the works.

  PixHyper<T,dim> - This is a hyper spectral pixel as large as 16^2
                    dimensions. Some operators are not supported on
                    this pixel type, but will be at a future date.

*/
template <class T, size_t dim> class Pixels
{
private:
protected:
public:

  // ######################################################################
  // typedefs

  typedef T                                 ScalarType;
  typedef typename promote_trait<T,T>::TP   SelfPromoteType;

  // ######################################################################
  // Constuctors

  //! initalize an empty pixel
  inline Pixels();
  //! generic destructor
  inline ~Pixels();
  //! initalize a pixel with all cell values T
  inline Pixels(const Pixels<T,dim>& pix);
  //! initalize a pixel with all cell values T
  template <class T2> explicit inline Pixels(const Pixels<T2,dim>& pix);
  //! initalize a pixel with all cell values T
  explicit inline Pixels(const T* pix);

  //! initalize a pixel with all cell values T
  explicit inline Pixels(const T val);

  // ######################################################################
  /* DATA */
  //! pixel info array pointer
  T p[dim];

  const static enum PixType pType = PixTypeBase;
  const static size_t myDim = dim;

  // ######################################################################
  // Pixel Data Access Methods

  //! get the diminsion of this pixel
  inline size_t getDim();
  //! set the value of this pixel from another pixel, with no conversion
  template <class T2> inline void setVal(Pixels<T2,dim>& A);
  //! get a const array element in this pixel
  inline const T& operator[](const size_t item) const;
  //! get an array element in this pixel
  inline T& operator[](const size_t item);
  //! get functional access to a pixel element
  inline T& at(const size_t item);

  // ######################################################################
  // Pixel operators ==, !=

  //! Pixel equality operator
  inline bool operator==(const Pixels<T,dim>& A) const;
  //! Pixel inequality operator
  inline bool operator!=(const Pixels<T,dim>& A) const;

  // ######################################################################
  // Pixel operators ++, --

  //! Pixels ++ operator
  inline Pixels<T,dim> operator++();
  //! Pixels -- operator
  inline Pixels<T,dim> operator--();
};

// ######################################################################
// ######################################################################
// Actual Pixel class that holds pixel data
// ######################################################################
// ######################################################################
// Class Pixels
// ######################################################################
// ######################################################################

// ######################################################################
// CONSTRUCTORS
// ######################################################################
template <class T, size_t dim>
inline Pixels<T,dim>::Pixels()
{
  for (size_t i = 0; i < dim; i++)
    p[i] = 0;
}

// ######################################################################
template <class T, size_t dim>
inline Pixels<T,dim>::~Pixels()
{}

// ######################################################################
template <class T, size_t dim>
inline Pixels<T,dim>::Pixels(const Pixels<T,dim>& pix)
{
  for (size_t i = 0; i < dim; i++)
    p[i] = pix.p[i];
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline Pixels<T,dim>::Pixels(const Pixels<T2,dim>& pix)
{
  for (size_t i = 0; i < dim; i++)
    p[i] = clamped_convert<T>(pix.p[i]);
}

// ######################################################################
template <class T, size_t dim>
inline Pixels<T,dim>::Pixels(const T* pix)
{
  for (size_t i = 0; i < dim; i++)
    p[i] = pix[i];
}

// ######################################################################
template <class T, size_t dim>
inline Pixels<T,dim>::Pixels(const T val)
{
  for (size_t i = 0; i < dim; i++)
    p[i] = val;
}

// ######################################################################
// ACCESS METHODS
// ######################################################################

template <class T, size_t dim>
inline size_t Pixels<T,dim>::getDim()
{
  return dim;
}

// ######################################################################
template <class T, size_t dim>
template <class T2> inline void Pixels<T,dim>::setVal(Pixels<T2,dim>& A)
{
  for (size_t i = 0; i < dim; i++)
    p[i] =  clamped_convert<T>(A[i]);
}

// ######################################################################
template <class T, size_t dim>
inline const T& Pixels<T,dim>::operator[](const size_t item) const
{
  return p[item];
}

// ######################################################################
template <class T, size_t dim>
inline T& Pixels<T,dim>::operator[](const size_t item)
{
  return p[item];
}

// ######################################################################
template <class T, size_t dim>
inline T& Pixels<T,dim>::at(const size_t item)
{
  return p[item];
}

// ######################################################################
// OPERATORS
// ######################################################################

template <class T, size_t dim>
inline bool Pixels<T,dim>::operator==(const Pixels<T,dim>& A) const
{
  bool ok = true;
  for (size_t i = 0; i < dim; i++)
    if(p[i] != A.p[i]) ok = false;

  return ok;
}

// ######################################################################
template <class T, size_t dim>
inline bool Pixels<T,dim>::operator!=(const Pixels<T,dim>& A) const
{
  bool ok = false;
  for (size_t i = 0; i < dim; i++)
    if(p[i] != A.p[i]) ok = true;
  return ok;
}

// ######################################################################
// INCR. OPERATORS
// ######################################################################

template <class T, size_t dim>
inline Pixels<T,dim> Pixels<T,dim>::operator++()
{
  for (size_t i = 0; i < dim; i++)
    p[i] = p[i] + 1;
  return *this;
}

// ######################################################################
template <class T, size_t dim>
inline Pixels<T,dim> Pixels<T,dim>::operator--()
{
  for (size_t i = 0; i < dim; i++)
    p[i] = p[i] - 1;
  return *this;
}

namespace pix_helper
{
  //! Transform this data into something more printable.
  /*! For example, convert char->int so that the char gets printed as
    a integer, rather than an alphabetic character. */
  inline bool printable(bool val) { return val;}
  inline int printable(char val) { return static_cast<int>(val);}
  inline short printable(short val) { return val;}
  inline int printable(int val) { return val;}
  inline long printable(long val) { return val;}
  inline float printable(float val) { return val;}
  inline double printable(double val) { return val;}
  inline long double printable(long double val) { return val;}
  inline int printable(unsigned char val) { return static_cast<int>(val);}
  inline int printable(unsigned short val) { return static_cast<int>(val);}
  inline long printable(unsigned int val) { return static_cast<long>(val);}
  inline unsigned long printable(unsigned long val) { return val;}
}


// ######################################################################
// ######################################################################
// inline free functions for stream input/output
// ######################################################################
// ######################################################################

// ######################################################################
//! << operator for pixels with use of Image_IO.C
template <class T, size_t dim, class Ostream>
inline Ostream&
operator<<(Ostream& os, const Pixels<T,dim>& pixel)
{
  os << '(';

  for (size_t i = 0; i < dim - 1; i++)
    os << pix_helper::printable(pixel.p[i]) << ", ";

  os << pix_helper::printable(pixel.p[dim - 1]) << ')';

  return os;
}

// ######################################################################
//! >> operator for pixels
template <class T, size_t dim, class Istream>
inline Istream&
operator>>(Istream& is, Pixels<T,dim>& pixel)
{
  for (size_t i = 0; i < dim; i++)
    is >> pixel.p[i];

  return is;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
