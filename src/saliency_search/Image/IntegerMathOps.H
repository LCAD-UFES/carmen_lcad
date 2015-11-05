/*!@file Image/IntegerMathOps.H Fixed-point integer math versions of some of our floating-point Image functions */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/IntegerMathOps.H $
// $Id: IntegerMathOps.H 10983 2009-03-05 07:19:14Z itti $
//

#ifndef IMAGE_INTEGERMATHOPS_H_DEFINED
#define IMAGE_INTEGERMATHOPS_H_DEFINED

#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/PyrBuilder.H"
#include "Image/fancynorm.H"
#include "Image/integer_math.h"

#include <deque>

template <int tabsiz_, int nbits_>
class IntgTrigTable
{
public:
  enum { tabsiz = tabsiz_ };
  enum { nbits = nbits_ };

  IntgTrigTable()
  {
    for (int i = 0; i < tabsiz; ++i)
      {
        const double arg = (2.0*M_PI*i) / double(tabsiz);
#ifdef HAVE_SINCOS
        double sinarg, cosarg;
        sincos(arg, &sinarg, &cosarg);
#else
        double sinarg = sin(arg);
        double cosarg = cos(arg);
#endif
        sintab[i] = int(sinarg * (1 << nbits));
        costab[i] = int(cosarg * (1 << nbits));
      }
  }

  int indexDegrees(const double deg)
  {
    int idx = int((deg * tabsiz) / 360.0) % tabsiz;
    if (idx < 0) idx += tabsiz;
    return idx;
  }

  int indexRadians(const double rad)
  {
    int idx = int((rad * tabsiz) / (2.0*M_PI)) % tabsiz;
    if (idx < 0) idx += tabsiz;
    return idx;
  }

  int sintab[tabsiz];
  int costab[tabsiz];

private:

  bool inited;
};

/// shift left if nbits > 8; shift right if nbits < 8
/** src is a pointer (rather than a reference) to prevent unwanted
    implicit conversions */
int intgScaleFromByte(const byte* src, const uint nbits);

/// assume input range is [0..255]; shift left if nbits > 8; shift right if nbits < 8
/** src is a pointer (rather than a reference) to prevent unwanted
    implicit conversions */
int intgScaleFromFloat(const float* src, const uint nbits);

/// shift left if nbits > 8; shift right if nbits < 8
/** src is a pointer (rather than a reference) to prevent unwanted
    implicit conversions */
Image<int> intgScaleFromByte(const Image<byte>* src, const uint nbits);

/// assume input range is [0..255]; shift left if nbits > 8; shift right if nbits < 8
/** src is a pointer (rather than a reference) to prevent unwanted
    implicit conversions */
Image<int> intgScaleFromFloat(const Image<float>* src, const uint nbits);

/// shift left if nbits > 8; shift right if nbits < 8
/** src is a pointer (rather than a reference) to prevent unwanted
    implicit conversions */
Image<PixRGB<int> > intgScaleFromByte(const Image<PixRGB<byte> >* src, const uint nbits);

/// get the luminance with nbits of precision of the input image
Image<int> intgScaleLuminanceFromByte(const Image<PixRGB<byte> >* src, const uint nbits);

/// assume input range is [0..255]; shift left if nbits > 8; shift right if nbits < 8
/** src is a pointer (rather than a reference) to prevent unwanted
    implicit conversions */
Image<PixRGB<int> > intgScaleFromFloat(const Image<PixRGB<float> >* src, const uint nbits);

/// Convert from out integer representation to floats
/** src is a pointer (rather than a reference) to prevent unwanted
    implicit conversions */
Image<float> intgScaleToFloat(const Image<int>* src, const uint nbits);

Image<int> intgLowPass5xDecX(const Image<int>& src,
                             const integer_math* imath);

Image<int> intgLowPass5yDecY(const Image<int>& src,
                             const integer_math* imath);

Image<int> intgXFilterClean(const Image<int>& src,
                            const int* hFilt, const int hfs,
                            const int shiftbits,
                            const integer_math* imath);

Image<int> intgYFilterClean(const Image<int>& src,
                            const int* vFilt, const int vfs,
                            const int shiftbits,
                            const integer_math* imath);

Image<int> intgLowPass9x(const Image<int>& src,
                         const integer_math* imath);
Image<int> intgLowPass9y(const Image<int>& src,
                         const integer_math* imath);
Image<int> intgLowPassX(int filterSize, const Image<int>& src,
                        const integer_math* imath);
Image<int> intgLowPassY(int filterSize, const Image<int>& src,
                        const integer_math* imath);
Image<int> intgLowPass(int filterSize, const Image<int>& src,
                       const integer_math* imath);
Image<int> intgQuadEnergy(const Image<int>& img1, const Image<int>& img2);
Image<int> intgOrientedFilter(const Image<int>& src,
                              const float k, const float theta,
                              const integer_math* imath);
void intgInplaceAttenuateBorders(Image<int>& a, int size);
ImageSet<int> intgBuildPyrLaplacian(const Image<int>& image,
                                    int firstlevel, int depth,
                                    int filterSize,
                                    const integer_math* imath);
ImageSet<int> intgBuildPyrOrientedFromLaplacian(const ImageSet<int>& lplc,
                                                const int filterSize,
                                                const float theta,
                                                const integer_math* imath);
ImageSet<int> intgBuildPyrOriented(const Image<int>& image,
                                   int firstlevel, int depth,
                                   int filterSize, float theta,
                                   const integer_math* imath);
ImageSet<int> intgBuildPyrGaussian(const Image<int>& image,
                                   int depth, int filterSize,
                                   const integer_math* imath);

Image<int> intgDownSize(const Image<int>& src, const Dims& dims,
                        const int filterWidth,
                        const integer_math* imath);
Image<int> intgRescale(const Image<int>& src, const Dims& dims);
void intgInplaceAddBGnoise(Image<int>& src, int max);
Image<int> intgMaxNormalize(const Image<int>& src, int min, int max,
                           MaxNormType typ);
Image<int> intgMaxNormalizeNone(const Image<int>& src, int min, int max);
Image<int> intgMaxNormalizeStd(const Image<int>& src, int min, int max);
Image<int> intgCenterSurround(const ImageSet<int>& pyr,
                             int lev1, int lev2, bool absol,
                             const ImageSet<int>* clipPyr);

void intgDoLowThresh(ImageSet<int>& pyr, int thresh, int newval = 0);

void intgDoLowThreshAbs(ImageSet<int>& pyr, int thresh, int newval = 0);

void intgDoRectify(ImageSet<int>& pyr);

void intgInplaceNormalize(Image<int>& dst,
                          const int nmin, const int nmax,
                          int* actualmin, int* actualmax);

Image<int> intgCenterSurround(const Image<int>& center,
                              const Image<int>& surround,
                              const bool absol);

/// Pixels with luminance less than MAX/threshfactor will have rg=by=0
/** Note that in the floating-point version getRGBY(), we take a
    luminance threshold and test

    ((r+g+b)/3 < thresh)

    where thresh is usually set to 25.5, i.e. 255/10. Here, to avoid
    loss of precision, we want to avoid any divisions, so instead we
    test

    (threshfactor*(r+g+b) < 3*MAX)

    which is equivalent to, but more precise than,

    ((r+g+b)/3 < MAX/threshfactor).
*/
void intgGetRGBY(const Image<PixRGB<byte> >& src,
                 Image<int>& rg,
                 Image<int>& by, const int threshfactor,
                 const uint inputbits);

Image<int> intgShiftImage(const Image<int>& srcImg,
                          const int dxnumer, const int dynumer,
                          const uint denombits);

// ######################################################################
//! Builds pyramids based on Gaussian filters.
class IntgGaussianPyrBuilder : public PyrBuilder<int>
{
public:
  //! Constructor.
  IntgGaussianPyrBuilder(const int filter_size,
                         const integer_math* imath);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual IntgGaussianPyrBuilder* clone() const;

  //! Builds a Gaussian pyramid based on \a img.
  virtual ImageSet<int> build(const Image<int>& img,
                              const int firstlevel, const int depth,
                              PyramidCache<int>* cache = 0);

private:
  const int itsFiltSize;
  const integer_math* const itsImath;
};

// ######################################################################
//! Builds pyramid based on oriented Laplacian filters.
/*! This is a fast approximation to Gabor convolution. */
class IntgOrientedPyrBuilder : public PyrBuilder<int>
{
public:
  //! Construct with a given gabor filter orientation and intensity.
  IntgOrientedPyrBuilder(const int filter_size, const float theta,
                         const integer_math* imath);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual IntgOrientedPyrBuilder* clone() const;

  //! Builds an oriented pyramid based on \a img.
  virtual ImageSet<int> build(const Image<int>& img,
                              const int firstlevel, const int depth,
                              PyramidCache<int>* cache = 0);

private:
  const int itsFiltSize;
  const float itsAngle;
  const integer_math* const itsImath;
};

// ######################################################################
//! This class implements Reichardt motion detections
/*! The class uses the filter results of pyramids of other types
  to implement Reichardt motion detection. The pyramids of the previous
  time step are stored and used with the pyramid of this time step and
  a version of the current pyramid, in which each level is shifted by the
  same amount dx and dy. The results are stored in the Reichardt pyramid and
  represent motion at different speeds and spatial resolutions. */
class IntgReichardtPyrBuilder : public PyrBuilder<int>
{
public:
  // ######################################################################
  // #### Constructors
  // ######################################################################
  //! build with depth, dx, dy and type, but no input image yet
  /*! @param dx the number of pixels to shift into the x-direction
    (can be fractional value)
    @param dy the number of pixels to shift into the y-direction
  */
  IntgReichardtPyrBuilder(const int dxnumer, const int dynumer,
                          const uint denombits,
                          const integer_math* imath);

  // default copy constructor and assignment are okay
  // std::deque handles the copying of the individual images

  //! cloning constructor
  virtual IntgReichardtPyrBuilder* clone() const;

  //! Builds a Reichardt pyramid based on a image
  virtual ImageSet<int> build(const Image<int>& image,
                              const int firstlevel, const int depth,
                              PyramidCache<int>* cache = 0);

  //! reset the stored queues - overrides PyrBuilder<T>::reset()
  virtual void reset();

private:
  const int itsDXnumer;
  const int itsDYnumer;
  const uint itsDenomBits;
  const integer_math* const itsImath;
  std::deque< ImageSet<int> > shifted, unshifted;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_INTEGERMATHOPS_H_DEFINED
