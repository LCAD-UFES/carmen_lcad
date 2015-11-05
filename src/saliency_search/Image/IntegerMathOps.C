/*!@file Image/IntegerMathOps.C Fixed-point integer math versions of some of our floating-point Image functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/IntegerMathOps.C $
// $Id: IntegerMathOps.C 10983 2009-03-05 07:19:14Z itti $
//

#ifndef IMAGE_INTEGERMATHOPS_C_DEFINED
#define IMAGE_INTEGERMATHOPS_C_DEFINED

#include "Image/IntegerMathOps.H"

#include "Image/CutPaste.H" // for shiftClean()
#include "Image/ImageSetOps.H" // for operator/=()
#include "Image/MathOps.H" // for inplaceRectify()
#include "Image/Pixels.H"
#include "Image/PyramidCache.H"
#include "Image/ShapeOps.H" // for decXY(), decX(), decY()
#include "rutz/rand.h"

// ######################################################################
int intgScaleFromByte(const byte* src, const uint nbits)
{
  if (nbits > 8)
    return (int(*src) << (nbits - 8));
  else if (nbits < 8)
    return (int(*src) >> (8 - nbits));
  // else nbits == 8
  return int(*src);
}

// ######################################################################
int intgScaleFromFloat(const float* src, const uint nbits)
{
  if (nbits > 8)
    return (int(*src * (1 << (nbits - 8))));
  else if (nbits < 8)
    return (int(*src / (1 << (8 - nbits))));
  // else nbits == 8
  return int(*src);
}

// ######################################################################
Image<int> intgScaleFromByte(const Image<byte>* src, const uint nbits)
{
  if (nbits > 8)
    return (Image<int>(*src) * (1 << (nbits - 8)));
  else if (nbits < 8)
    return (Image<int>(*src) / (1 << (8 - nbits)));
  // else nbits == 8
  return Image<int>(*src);
}

// ######################################################################
Image<int> intgScaleFromFloat(const Image<float>* src, const uint nbits)
{
  if (nbits > 8)
    return (Image<int>(*src * (1 << (nbits - 8))));
  else if (nbits < 8)
    return (Image<int>(*src / (1 << (8 - nbits))));
  // else nbits == 8
  return Image<int>(*src);
}

// ######################################################################
Image<PixRGB<int> > intgScaleFromByte(const Image<PixRGB<byte> >* src, const uint nbits)
{
  Image<PixRGB<int> > result(src->getDims(), NO_INIT);
  Image<PixRGB<byte> >::const_iterator sptr = src->begin();
  Image<PixRGB<int> >::iterator dptr = result.beginw();
  Image<PixRGB<int> >::iterator const stop = result.endw();

  if (nbits > 8)
    {
      const int lshift = nbits - 8;
      while (dptr != stop)
        {
          *dptr++ = PixRGB<int>(int(sptr->p[0]) << lshift,
                                int(sptr->p[1]) << lshift,
                                int(sptr->p[2]) << lshift);
          ++sptr;
        }
    }
  else if (nbits < 8)
    {
      const int rshift = 8 - nbits;
      while (dptr != stop)
        {
          *dptr++ = PixRGB<int>(int(sptr->p[0]) >> rshift,
                                int(sptr->p[1]) >> rshift,
                                int(sptr->p[2]) >> rshift);
          ++sptr;
        }
    }
  else // nbits == 8
    {
      while (dptr != stop)
        {
          *dptr++ = PixRGB<int>(int(sptr->p[0]),
                                int(sptr->p[1]),
                                int(sptr->p[2]));
          ++sptr;
        }
    }

  return result;
}

// ######################################################################
Image<int> intgScaleLuminanceFromByte(const Image<PixRGB<byte> >* src, const uint nbits)
{
  Image<int> result(src->getDims(), NO_INIT);
  Image<PixRGB<byte> >::const_iterator sptr = src->begin();
  Image<int>::iterator dptr = result.beginw();
  Image<int>::iterator const stop = result.endw();

  if (nbits > 8)
    while (dptr != stop)
      { *dptr++ = ((sptr->p[0] + sptr->p[1] + sptr->p[2]) / 3) << (nbits - 8); ++sptr; }
  else if (nbits < 8)
    while (dptr != stop)
      { *dptr++ = ((sptr->p[0] + sptr->p[1] + sptr->p[2]) / 3) >> (8 - nbits); ++sptr; }
  else // nbits == 8
    while (dptr != stop)
      { *dptr++ = (sptr->p[0] + sptr->p[1] + sptr->p[2]) / 3; ++sptr; }

  return result;
}

// ######################################################################
Image<PixRGB<int> > intgScaleFromFloat(const Image<PixRGB<float> >* src, const uint nbits)
{
  if (nbits > 8)
    return (Image<PixRGB<int> >(*src * (1 << (nbits - 8))));
  else if (nbits < 8)
    return (Image<PixRGB<int> >(*src / (1 << (8 - nbits))));
  // else nbits == 8
  return Image<PixRGB<int> >(*src);
}

// ######################################################################
Image<float> intgScaleToFloat(const Image<int>* src, const uint nbits)
{
  if (nbits > 8)
    return (Image<float>(*src) / float(1 << (nbits - 8)));
  else if (nbits < 8)
    return (Image<float>(*src) * (1 << (8 - nbits)));
  // else nbits == 8
  return Image<float>(*src);
}

// ######################################################################
// Anderson's separable kernel: 1/16 * [1 4 6 4 1]
Image<int> intgLowPass5xDecX(const Image<int>& src,
                             const integer_math* imath)
{
  const int w = src.getWidth(), h = src.getHeight();

  if (w < 2) return src; // nothing to smooth

  const int w2 = w / 2;
  ASSERT(w2 > 0);

  Image<int> result(w2, h, NO_INIT);

  const int filterbits = 4; // log2(16)
  const int accumbits = 3; // ceil(log2(5))

  if ((imath->nbits + filterbits + accumbits) >= (8*sizeof(int) - 1))
    (*imath->low_pass_5_x_dec_x_manybits)(src.getArrayPtr(), w, h,
                                          result.getArrayPtr(), w2);
  else
    (*imath->low_pass_5_x_dec_x_fewbits)(src.getArrayPtr(), w, h,
                                         result.getArrayPtr(), w2);

  return result;
}

// ######################################################################
// Anderson's separable kernel: 1/16 * [1 4 6 4 1]
Image<int> intgLowPass5yDecY(const Image<int>& src,
                             const integer_math* imath)
{
  const int w = src.getWidth(), h = src.getHeight();

  if (h < 2) return src; // nothing to smooth

  const int h2 = h / 2;
  ASSERT(h2 > 0);

  Image<int> result(w, h2, NO_INIT);

  const int filterbits = 4; // log2(16)
  const int accumbits = 3; // ceil(log2(5))

  if ((imath->nbits + filterbits + accumbits) >= (8*sizeof(int) - 1))
    (*imath->low_pass_5_y_dec_y_manybits)(src.getArrayPtr(), w, h,
                                          result.getArrayPtr(), h2);
  else
    (*imath->low_pass_5_y_dec_y_fewbits)(src.getArrayPtr(), w, h,
                                         result.getArrayPtr(), h2);

  return result;
}

// ######################################################################
Image<int> intgXFilterClean(const Image<int>& src,
                            const int* hFilt, const int hfs,
                            const int shiftbits,
                            const integer_math* imath)
{
  if (hfs == 0)
    return src;

  int hFiltFlipped[hfs]; // flipped filter to accelerate convolution:
  int sumh = 0;
  for (int x = 0; x < hfs; x ++)
    { sumh += hFilt[x]; hFiltFlipped[hfs-1-x] = hFilt[x]; }

  ASSERT(sumh == (1 << shiftbits));

  const int accumbits = int(ceil(log2(double(hfs))));

  Image<int> result(src.getDims(), NO_INIT);

  if ((imath->nbits + shiftbits + accumbits) >= (8*sizeof(int) - 1))
    {
      if (src.getWidth() < hfs)
        (*imath->x_filter_clean_small_manybits)
          (src.getArrayPtr(), src.getWidth(), src.getHeight(),
           hFiltFlipped, hfs, shiftbits,
           result.getArrayPtr());
      else
        (*imath->x_filter_clean_manybits)
          (src.getArrayPtr(), src.getWidth(), src.getHeight(),
           hFiltFlipped, hfs, shiftbits,
           result.getArrayPtr());
    }
  else
    {
      if (src.getWidth() < hfs)
        (*imath->x_filter_clean_small_fewbits)
          (src.getArrayPtr(), src.getWidth(), src.getHeight(),
           hFiltFlipped, hfs, shiftbits,
           result.getArrayPtr());
      else
        (*imath->x_filter_clean_fewbits)
          (src.getArrayPtr(), src.getWidth(), src.getHeight(),
           hFiltFlipped, hfs, shiftbits,
           result.getArrayPtr());
    }

  return result;
}

// ######################################################################
Image<int> intgYFilterClean(const Image<int>& src,
                            const int* vFilt, const int vfs,
                            const int shiftbits,
                            const integer_math* imath)
{
  if (vfs == 0)
    return src;

  int vFiltFlipped[vfs]; // flipped filter to accelerate convolution
  int sumv = 0;
  for (int x = 0; x < vfs; x ++)
    { sumv += vFilt[x]; vFiltFlipped[vfs-1-x] = vFilt[x]; }

  ASSERT(sumv == (1 << shiftbits));

  const int accumbits = int(ceil(log2(double(vfs))));

  Image<int> result(src.getDims(), NO_INIT);

  if ((imath->nbits + shiftbits + accumbits) >= (8*sizeof(int) - 1))
    {
      if (src.getHeight() < vfs)
        (*imath->y_filter_clean_small_manybits)
          (src.getArrayPtr(), src.getWidth(), src.getHeight(),
           vFiltFlipped, vfs, shiftbits,
           result.getArrayPtr());
      else
        (*imath->y_filter_clean_manybits)
          (src.getArrayPtr(), src.getWidth(), src.getHeight(),
           vFiltFlipped, vfs, shiftbits,
           result.getArrayPtr());
    }
  else
    {
      if (src.getHeight() < vfs)
        (*imath->y_filter_clean_small_fewbits)
          (src.getArrayPtr(), src.getWidth(), src.getHeight(),
           vFiltFlipped, vfs, shiftbits,
           result.getArrayPtr());
      else
        (*imath->y_filter_clean_fewbits)
          (src.getArrayPtr(), src.getWidth(), src.getHeight(),
           vFiltFlipped, vfs, shiftbits,
           result.getArrayPtr());
    }

  return result;
}

// ######################################################################
Image<int> intgLowPass9x(const Image<int>& src,
                         const integer_math* imath)
{
  const int w = src.getWidth(), h = src.getHeight();

  if (w < 2) return src; // nothing to smooth

  if (w < 9)  // use inefficient implementation for small images
    {
      const int kernel[9] = { 1, 8, 28, 56, 70, 56, 28, 8, 1 };
      const int shiftbits = 8; // 256 = (1<<8)
      return intgXFilterClean(src, kernel, 9, shiftbits, imath);
    }

  Image<int> result(w, h, NO_INIT);

  const int filterbits = 8; // log2(256)
  const int accumbits = 4; // ceil(log2(9))

  if ((imath->nbits + filterbits + accumbits) >= (8*sizeof(int) - 1))
    (*imath->low_pass_9_x_manybits)(src.getArrayPtr(), w, h,
                                    result.getArrayPtr());
  else
    (*imath->low_pass_9_x_fewbits)(src.getArrayPtr(), w, h,
                                   result.getArrayPtr());

  return result;
}

// ######################################################################
Image<int> intgLowPass9y(const Image<int>& src,
                         const integer_math* imath)
{
  const int w = src.getWidth(), h = src.getHeight();

  if (h < 2) return src; // nothing to smooth

  if (h < 9)  // use inefficient implementation for small images
    {
      const int kernel[9] = { 1, 8, 28, 56, 70, 56, 28, 8, 1 };
      const int shiftbits = 8; // 256 = (1<<8)
      return intgYFilterClean(src, kernel, 9, shiftbits, imath);
    }

  Image<int> result(w, h, NO_INIT);

  const int filterbits = 8; // log2(256)
  const int accumbits = 4; // ceil(log2(9))

  if ((imath->nbits + filterbits + accumbits) >= (8*sizeof(int) - 1))
    (*imath->low_pass_9_y_manybits)(src.getArrayPtr(), w, h,
                                    result.getArrayPtr());
  else
    (*imath->low_pass_9_y_fewbits)(src.getArrayPtr(), w, h,
                                   result.getArrayPtr());

  return result;
}

// ######################################################################
Image<int> intgLowPassX(int filterSize, const Image<int>& src,
                        const integer_math* imath)
{
  switch (filterSize)
    {
    case 9: return intgLowPass9x(src, imath);
    default:
      LFATAL("Filter size %d is not supported", filterSize);
    }

  /* can't happen */ return Image<int>();
}

// ######################################################################
Image<int> intgLowPassY(int filterSize, const Image<int>& src,
                        const integer_math* imath)
{
  switch (filterSize)
    {
    case 9: return intgLowPass9y(src, imath);
    default:
      LFATAL("Filter size %d is not supported", filterSize);
    }

  /* can't happen */ return Image<int>();
}

// ######################################################################
Image<int> intgLowPass(int filterSize, const Image<int>& src,
                       const integer_math* imath)
{
  return intgLowPassY(filterSize,
                      intgLowPassX(filterSize, src, imath),
                      imath);
}

// ######################################################################
Image<int> intgQuadEnergy(const Image<int>& img1, const Image<int>& img2)
{
  ASSERT(img1.isSameSize(img2));

  Image<int> result(img1.getDims(), NO_INIT);

  Image<int>::const_iterator s1ptr = img1.begin();
  Image<int>::const_iterator s2ptr = img2.begin();
  Image<int>::iterator dptr = result.beginw();
  Image<int>::iterator stop = result.endw();

  while (dptr != stop)
    {
      const int s1 = abs(*s1ptr++);
      const int s2 = abs(*s2ptr++);

      /* "A Fast Approximation to the Hypotenuse" by Alan Paeth, from
         "Graphics Gems", Academic Press, 1990

         http://www.acm.org/pubs/tog/GraphicsGems/gems/HypotApprox.c

         gives approximate value of sqrt(s1*s1+s2*s2) with only
         overestimations, and then never by more than (9/8) + one bit
         uncertainty
      */
      *dptr++ = (s1 > s2) ? (s1 + (s2 >> 1)) : ((s1 >> 1) + s2);
    }

  return result;
}

// ######################################################################
Image<int> intgOrientedFilter(const Image<int>& src,
                              const float k, const float theta,
                              const integer_math* imath)
{

  static IntgTrigTable<256, 8> trig;

  const int KBITS = 8;

  const int thetaidx = trig.indexDegrees(theta + 90.0);

  const double kx = double(k) * trig.costab[thetaidx] / double(1<<trig.nbits);
  const double ky = double(k) * trig.sintab[thetaidx] / double(1<<trig.nbits);

  const int kxnum = int((kx * (1 << KBITS) * trig.tabsiz) / (2.0*M_PI));
  const int kynum = int((ky * (1 << KBITS) * trig.tabsiz) / (2.0*M_PI));

  Image<int> re(src.getDims(), NO_INIT), im(src.getDims(), NO_INIT);
  Image<int>::const_iterator sptr = src.begin();
  Image<int>::iterator reptr = re.beginw(), imptr = im.beginw();

  // (x,y) = (0,0) at center of image:
  const int w2l = src.getWidth() / 2;
  const int w2r = src.getWidth() - w2l;
  const int h2l = src.getHeight() / 2;
  const int h2r = src.getHeight() - h2l;

  // let's do a conservative check to make sure that we won't overflow
  // when we compute "arg" later on -- as a very rough estimate, kxnum
  // and kynum are on the order of 2^16 (8 bits from KBITS=8, 8 bits
  // from trig.tabsiz=256), which gives room for w+h to be up to about
  // 2^15
  ASSERT((std::numeric_limits<int>::max() / (abs(kxnum) + abs(kynum)))
         > (w2r + h2r));

  ASSERT((2 * trig.nbits + 1) < 8*sizeof(int));

  const int mdcutoff =
    std::numeric_limits<int>::max() >> (trig.nbits+1);

  for (int j = -h2l; j < h2r; ++j)
    for (int i = -w2l; i < w2r; ++i)
      {
        const int arg = (i * kxnum + j * kynum) >> KBITS;

        int idx = arg % trig.tabsiz;
        if (idx < 0) idx += trig.tabsiz;

        const int sval = *sptr++;

        if (sval == 0)
          {
            *reptr++ = 0;
            *imptr++ = 0;
          }
        else if (abs(sval) < mdcutoff)
          {
            *reptr++ = (sval * trig.costab[idx]) >> (trig.nbits+1);
            *imptr++ = (sval * trig.sintab[idx]) >> (trig.nbits+1);
          }
        else
          {
            const int val = (sval >> (trig.nbits+1));
            *reptr++ = val * trig.costab[idx];
            *imptr++ = val * trig.sintab[idx];
          }
      }

  re = intgLowPass(9, re, imath);
  im = intgLowPass(9, im, imath);

  return intgQuadEnergy(re, im);
}

// ######################################################################
void intgInplaceAttenuateBorders(Image<int>& a, int size)
{
  ASSERT(a.initialized());

  Dims dims = a.getDims();

  if (size * 2 > dims.w()) size = dims.w() / 2;
  if (size * 2 > dims.h()) size = dims.h() / 2;
  if (size < 1) return;  // forget it

  // top lines:
  int coeff = 1;
  Image<int>::iterator aptr = a.beginw();
  for (int y = 0; y < size; y ++)
    {
      for (int x = 0; x < dims.w(); x ++)
        {
          *aptr = ((*aptr) / (size+1)) * coeff;
          ++aptr;
        }
      ++coeff;
    }
  // normal lines: start again from beginning to attenuate corners twice:
  aptr = a.beginw();
  for (int y = 0; y < dims.h(); y ++)
    {
      coeff = 1;
      for (int x = 0; x < size; x ++)
        {
          *(aptr + dims.w() - 1 - x * 2) =
            (*(aptr + dims.w() - 1 - x * 2) / (size+1)) * coeff;

          *aptr = ((*aptr) / (size+1)) * coeff;
          ++aptr;
          ++coeff;
        }
      aptr += dims.w() - size;
    }
  // bottom lines
  aptr = a.beginw() + (dims.h() - size) * dims.w();
  coeff = size;
  for (int y = dims.h() - size; y < dims.h(); y ++)
    {
      for (int x = 0; x < dims.w(); ++x)
        {
          *aptr = ((*aptr) / (size+1)) * coeff;
          ++aptr;
        }
      --coeff;
    }
}

// ######################################################################
ImageSet<int> intgBuildPyrLaplacian(const Image<int>& image,
                                    int firstlevel, int depth,
                                    int filterSize,
                                    const integer_math* imath)
{
  ASSERT(image.initialized());

  ImageSet<int> result(depth);

  // compute laplacian as image - LPF(image), then compute oriented
  // filter:

  Image<int> lpfima;

  for (int lev = 0; lev < depth; ++lev)
    {
      const Image<int> dec =
        lev == 0 ? image : decXY(lpfima);
      lpfima = intgLowPass(filterSize, dec, imath);

      if (lev >= firstlevel)
        result[lev] = dec-lpfima;
    }

  return result;
}

// ######################################################################
ImageSet<int> intgBuildPyrOrientedFromLaplacian(const ImageSet<int>& lplc,
                                                const int filterSize,
                                                const float theta,
                                                const integer_math* imath)
{
  int attenuation_width = -1;
  float spatial_freq = -1.0;

  switch (filterSize)
    {
    case 9: attenuation_width = 5; spatial_freq = 2.6; break;
    default:
      LFATAL("Filter size %d is not supported", filterSize);
    }

  ImageSet<int> result(lplc.size());

  for (uint lev = 0; lev < lplc.size(); ++lev)
    {
      // if the laplacian is empty at a given level, then just leave
      // the output empty at that level, too
      if (!lplc[lev].initialized())
        continue;

      result[lev] = intgOrientedFilter(lplc[lev], spatial_freq, theta,
                                       imath);
      // attenuate borders that are overestimated due to filter trunctation:
      intgInplaceAttenuateBorders(result[lev], attenuation_width);
    }

  return result;
}

// ######################################################################
ImageSet<int> intgBuildPyrOriented(const Image<int>& image,
                                   int firstlevel, int depth,
                                   int filterSize, float theta,
                                   const integer_math* imath)
{
  const ImageSet<int> lplc =
    intgBuildPyrLaplacian(image, firstlevel, depth, filterSize, imath);

  return intgBuildPyrOrientedFromLaplacian(lplc, filterSize,
                                           theta, imath);
}

// ######################################################################
ImageSet<int> intgBuildPyrGaussian(const Image<int>& image,
                                   int depth, int filterSize,
                                   const integer_math* imath)
{
  ASSERT(image.initialized());

  ImageSet<int> result(depth);

  result[0] = image;
  for (int lev = 1; lev < depth; ++lev)
    {
      Image<int> a = result.getImage(lev-1);
      if (filterSize == 5)
        {
          a = intgLowPass5xDecX(a,imath);
          a = intgLowPass5yDecY(a,imath);
        }
      else
        {
          a = decX(intgLowPassX(filterSize, a, imath));
          a = decY(intgLowPassY(filterSize, a, imath));
        }
      result[lev] = a;
    }

  return result;
}

// ######################################################################
Image<int> intgDownSize(const Image<int>& src, const Dims& dims,
                        const int filterWidth,
                        const integer_math* imath)
{
  const int new_w = dims.w();
  const int new_h = dims.h();

  if (src.getWidth() == new_w && src.getHeight() == new_h) return src;

  ASSERT(src.getWidth() / new_w > 1 && src.getHeight() / new_h > 1);

  const int wdepth = int(0.5+log(double(src.getWidth() / new_w)) / M_LN2);
  const int hdepth = int(0.5+log(double(src.getHeight() / new_h)) / M_LN2);

  if (wdepth != hdepth)
    LFATAL("arrays must have same proportions");

  Image<int> result = src;

  for (int i = 0; i < wdepth; ++i)
    {
      result = decX(intgLowPassX(filterWidth, result, imath));
      result = decY(intgLowPassY(filterWidth, result, imath));
    }

  return result;
}

// ######################################################################
Image<int> intgRescale(const Image<int>& src, const Dims& dims)
{
  const int new_w = dims.w();
  const int new_h = dims.h();

  ASSERT(src.initialized()); ASSERT(new_w > 0 && new_h > 0);

  const int orig_w = src.getWidth();
  const int orig_h = src.getHeight();

  // check if same size already
  if (new_w == orig_w && new_h == orig_h) return src;

  Image<int> result(new_w, new_h, NO_INIT);
  Image<int>::iterator dptr = result.beginw();
  Image<int>::const_iterator const sptr = src.begin();

  // code inspired from one of the Graphics Gems book:
  /*
    (1) (x,y) are the original coords corresponding to scaled coords (i,j)
    (2) (x0,y0) are the greatest lower bound integral coords from (x,y)
    (3) (x1,y1) are the least upper bound integral coords from (x,y)
    (4) d00, d10, d01, d11 are the values of the original image at the corners
    of the rect (x0,y0),(x1,y1)
    (5) the value in the scaled image is computed from bilinear interpolation
    among d00,d10,d01,d11
  */
  for (int j = 0; j < new_h; ++j)
    {
      const int y_numer = std::max(0, j*2*orig_h+orig_h-new_h);
      const int y_denom = 2*new_h;

      const int y0 = y_numer / y_denom;
      const int y1 = std::min(y0 + 1, orig_h - 1);

      const int fy_numer = y_numer - y0 * y_denom;
      const int fy_denom = y_denom;
      ASSERT(fy_numer == (y_numer % y_denom));

      const int wy0 = orig_w * y0;
      const int wy1 = orig_w * y1;

      for (int i = 0; i < new_w; ++i)
        {
          const int x_numer = std::max(0, i*2*orig_w+orig_w-new_w);
          const int x_denom = 2*new_w;

          const int x0 = x_numer / x_denom;
          const int x1 = std::min(x0 + 1, orig_w - 1);

          const int fx_numer = x_numer - x0 * x_denom;
          const int fx_denom = x_denom;
          ASSERT(fx_numer == (x_numer % x_denom));

          const int
            d00( sptr[x0 + wy0] ), d10( sptr[x1 + wy0] ),
            d01( sptr[x0 + wy1] ), d11( sptr[x1 + wy1] ),
            dx0( d00 + ((d10 - d00) / fx_denom) * fx_numer ),
            dx1( d01 + ((d11 - d01) / fx_denom) * fx_numer );

          *dptr++ = dx0 + ((dx1 - dx0) / fy_denom) * fy_numer;  // no need to clamp
        }
    }
  return result;
}

// ######################################################################
void intgInplaceAddBGnoise(Image<int>& src, int max)
{
  const int range = max / 100000;

  ASSERT(src.initialized());
  const int w = src.getWidth(), h = src.getHeight();

  // do not put noise very close to image borders:
  int siz = std::min(w, h) / 10;

  Image<int>::iterator sptr = src.beginw() + siz + siz * w;

  static rutz::urand rnd;

  for (int j = siz; j < h - siz; ++j)
    {
      for (int i = siz; i < w - siz; ++i)
        *sptr++ += rnd.idraw(range);
      sptr += siz + siz;
    }
}

// ######################################################################
Image<int> intgMaxNormalize(const Image<int>& src,
                            const int mi, const int ma, const MaxNormType normtyp)
{

  Image<int> result;

  // do normalization depending on desired type:
  switch(normtyp)
    {
    case VCXNORM_NONE:
      result = intgMaxNormalizeNone(src, mi, ma);
      break;
    case VCXNORM_MAXNORM:
      result = intgMaxNormalizeStd(src, mi, ma);
      break;
//     case VCXNORM_FANCYFAST:
//       result = maxNormalizeFancyFast(src, mi, ma, nbiter);
//       break;
//     case VCXNORM_FANCYONE:
//       nbiter = 1;
//     case VCXNORM_FANCY:
//       result = maxNormalizeFancy(src, mi, ma, nbiter, 1.0, lrexcit);
//       break;
//     case VCXNORM_FANCYLANDMARK:
//       result = maxNormalizeFancyLandmark(src, mi, ma, nbiter);
//       break;
//     case VCXNORM_LANDMARK:
//       result = maxNormalizeLandmark(src, mi, ma);
//       break;
//     case VCXNORM_FANCYWEAK:
//       result = maxNormalizeFancy(src, mi, ma, nbiter, 0.5);
//       break;
//     case VCXNORM_FANCYVWEAK:
//       result = maxNormalizeFancy(src, mi, ma, nbiter, 0.1);
//       break;
//     case VCXNORM_IGNORE:
//       result = src;
//       break;
//     case VCXNORM_SURPRISE:
//       result = src;
//       break;
    default:
      LFATAL("Unknown normalization type: %d", int(normtyp));
    }

  return result;
}

// ######################################################################
Image<int> intgMaxNormalizeNone(const Image<int>& src,
                                const int nmi, const int nma)
{
  Image<int> result = src;

  // first clamp negative values to zero
  inplaceRectify(result);

  // then, normalize between mi and ma if not zero
  int mi = nmi;
  int ma = nma;
  if (mi != 0 || ma != 0)
    intgInplaceNormalize(result, nmi, nma, &mi, &ma);

  return result;
}

// ######################################################################
Image<int> intgMaxNormalizeStd(const Image<int>& src,
                               const int nmi, const int nma)
{
  ASSERT(src.initialized());

  Image<int> result = src;

  // first clamp negative values to zero
  inplaceRectify(result);

  // then, normalize between mi and ma if not zero
  int mi = nmi;
  int ma = nma;
  if (nmi != 0 || nma != 0)
    intgInplaceNormalize(result, nmi, nma, &mi, &ma);

  const int w = result.getWidth();
  const int h = result.getHeight();

  // normalize between mi and ma and multiply by (max - mean)^2

  // we want to detect quickly local maxes, but avoid getting local mins
  const int thresh = mi + (ma - mi) / 10;

  // then get the mean value of the local maxima:
  std::vector<int> vals;
  Image<int>::const_iterator const dptr = result.begin();
  for (int j = 1; j < h - 1; ++j)
    for (int i = 1; i < w - 1; ++i)
      {
        const int index = i + w * j;
        const int val = dptr[index];
        if (val >= thresh &&
            val >= dptr[index - w] &&
            val >= dptr[index + w] &&
            val >= dptr[index - 1] &&
            val >= dptr[index + 1])  // local max
          {
            vals.push_back(val);
          }
      }

  int lm_mean = 0;
  for (size_t i = 0; i < vals.size(); ++i)
    lm_mean += (vals[i] / vals.size());

  ASSERT(ma >= lm_mean);

  // scale factor is (max - mean_local_max)^2:
  if (vals.size() > 1)
    {
      // make sure that (ma - lm_mean)^2 won't overflow:
      ASSERT((ma == lm_mean) ||
             ((std::numeric_limits<int>::max() / (ma - lm_mean))
              > (ma - lm_mean)));

      const int factor = ((ma - lm_mean) * (ma - lm_mean)) / ma;
      result *= factor;
    }
  else if (vals.size() == 1)  // a single narrow peak
    {
      const int factor = ma;
      result *= factor;
    }
  else
    {
      /* LERROR("No local maxes found !!"); */
    }

  return result;
}

// ######################################################################
Image<int> intgCenterSurround(const ImageSet<int>& pyr,
                              int lev1, int lev2, bool absol,
                              const ImageSet<int>* clipPyr)
{
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < pyr.size() && uint(lev2) < pyr.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      LFATAL("clipping pyramids not supported");

      ASSERT((*clipPyr)[largeLev].getDims() == pyr[largeLev].getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == pyr[smallLev].getDims());

      // FIXME: this will overflow:
      return
        intgCenterSurround(Image<int>(pyr[largeLev] * (*clipPyr)[largeLev]),
                           Image<int>(pyr[smallLev] * (*clipPyr)[smallLev]),
                           absol);
    }
  else
    return intgCenterSurround(pyr[largeLev], pyr[smallLev], absol);
}

// ######################################################################
void intgDoLowThresh(ImageSet<int>& x, int threshold, int newval)
{
  for (uint i = 0; i < x.size(); ++i)
    inplaceLowThresh(x[i], threshold, newval);
}

// ######################################################################
void intgDoLowThreshAbs(ImageSet<int>& x, int threshold, int newval)
{
  for (uint i = 0; i < x.size(); ++i)
    inplaceLowThreshAbs(x[i], threshold, newval);
}

// ######################################################################
void intgDoRectify(ImageSet<int>& x)
{
  for (uint i = 0; i < x.size(); ++i)
    inplaceRectify(x[i]);
}

// ######################################################################
void intgInplaceNormalize(Image<int>& dst, const int nmin, const int nmax,
                          int* actualmin_p, int* actualmax_p)
{
  ASSERT(dst.initialized());
  ASSERT(nmax >= nmin);

  int mi, ma; getMinMax(dst, mi, ma);
  const int scale = ma - mi;
  if (scale == 0) { dst.clear(0); return; } // input image is uniform
  const int nscale = nmax - nmin;

  if (nscale == 0) { dst.clear(nmin); return; } // output range is uniform

  Image<int>::iterator aptr = dst.beginw();
  Image<int>::iterator stop = dst.endw();

  int actualmin, actualmax;

  if (scale == nscale)
    {
      const int add = nmin - mi;
      while (aptr != stop)
        {
          *aptr += add;
          ++aptr;
        }

      actualmin = nmin;
      actualmax = nmax;
    }
  else if (scale > nscale)
    {
      const int div = scale / nscale;

      while (aptr != stop)
        {
          *aptr = nmin + ((*aptr - mi) / div);
          ++aptr;
        }

      // FIXME sometimes we we will end up with actualmax>nmax, which
      // is not really what the user wants; yet, we can't do arbitrary
      // precision arithmetic without overflowing -- maybe we can find
      // a rational number with small numerator and denominator that
      // approximates nscale/scale without exceeding it?

      actualmin = nmin;
      actualmax = nmin + (scale / div);
    }
  else // (scale < nscale)
    {
      const int mul = nscale / scale;
      while (aptr != stop)
        {
          *aptr = nmin + ((*aptr - mi) * mul);
          ++aptr;
        }

      actualmin = nmin;
      actualmax = nmin + (scale * mul);
    }

  // Don't assign to the pointers until the very end, in case the user
  // passes pointers to nmin,nmax as the actualmin/actualmax pointers
  ASSERT(actualmin_p != NULL);
  ASSERT(actualmax_p != NULL);
  *actualmin_p = actualmin;
  *actualmax_p = actualmax;
}

// ######################################################################
Image<int> intgCenterSurround(const Image<int>& center,
                              const Image<int>& surround,
                              const bool absol)
{
  const int lw = center.getWidth(), lh = center.getHeight();
  const int sw = surround.getWidth(), sh = surround.getHeight();

  if (sw > lw || sh > lh) LFATAL("center must be larger than surround");

  int scalex = lw / sw, remx = lw - 1 - (lw % sw);
  int scaley = lh / sh, remy = lh - 1 - (lh % sh);

  // result has the size of the larger image:
  Image<int> result(center.getDims(), NO_INIT);

  // scan large image and subtract corresponding pixel from small image:
  int ci = 0, cj = 0;
  Image<int>::const_iterator lptr = center.begin();
  Image<int>::const_iterator sptr = surround.begin();
  Image<int>::iterator dptr = result.beginw();

  if (absol == true)  // compute abs(hires - lowres):
    {
      for (int j = 0; j < lh; ++j)
        {
          for (int i = 0; i < lw; ++i)
            {
              if (*lptr > *sptr)
                *dptr++ = (*lptr++ - *sptr);
              else
                *dptr++ = (*sptr - *lptr++);

              if ((++ci) == scalex && i != remx) { ci = 0; ++sptr; }
            }
          if (ci) { ci = 0; ++sptr; }  // in case the reduction is not round
          if ((++cj) == scaley && j != remy) cj = 0; else sptr -= sw;
        }
    }
  else  // compute hires - lowres, clamped to 0:
    {
      for (int j = 0; j < lh; ++j)
        {
          for (int i = 0; i < lw; ++i)
            {
              if (*lptr > *sptr)
                *dptr++ = (*lptr++ - *sptr);
              else
                { *dptr++ = 0; lptr++; }

              if ((++ci) == scalex && i != remx) { ci = 0; ++sptr; }
            }
          if (ci) { ci = 0; ++sptr; } // in case the reduction is not round
          if ((++cj) == scaley && j != remy) cj = 0; else sptr -= sw;
        }
    }

  // attenuate borders:
  intgInplaceAttenuateBorders(result, result.getDims().max() / 20);

  return result;
}

// ######################################################################
void intgGetRGBY(const Image<PixRGB<byte> >& src,
                 Image<int>& rg,
                 Image<int>& by, const int threshfactor,
                 const uint inputbits)
{
  // red = [r - (g+b)/2]        [.] = clamp between 0 and 255
  // green = [g - (r+b)/2]
  // blue = [b - (r+g)/2]
  // yellow = [2*((r+g)/2 - |r-g| - b)]

  ASSERT(src.initialized());

  rg = Image<int>(src.getDims(), NO_INIT);
  by = Image<int>(src.getDims(), NO_INIT);

  Image<PixRGB<byte> >::const_iterator aptr = src.begin();
  Image<PixRGB<byte> >::const_iterator stop = src.end();

  Image<int>::iterator rgptr = rg.beginw(), byptr = by.beginw();

  const int bitsaftershift = 8*sizeof(int) - 2;    // e.g. 30
  const int upshift = bitsaftershift - 14;         // e.g. 16
  const int bitsafterdiv = bitsaftershift - 10;    // e.g. 20
  const int finalshift = inputbits - bitsafterdiv;

  ASSERT(upshift > 0);

  while (aptr != stop)
    {
      const int r = aptr->p[0]; // 8 bits
      const int g = aptr->p[1];
      const int b = aptr->p[2];
      ++aptr;

      if (threshfactor*(r+g+b) < 3*255)  // too dark... no response from anybody
        { *rgptr++ = 0; *byptr++ = 0; }
      else
        {
          // first do the luminanceNormalization:
          const int lum = r + g + b; // range up to 10 bits

          // now compute color opponencies:
          int red = 2*r - g - b;             // range up to 10 bits
          int green = 2*g - r - b;
          int blue = 2*b - r - g;
          int yellow = -2*blue - 4*abs(r-g); // range up to 11 bits

          if (red < 0) red = 0;
          if (green < 0) green = 0;
          if (blue < 0) blue = 0;
          if (yellow < 0) yellow = 0;

          // compute differences and normalize chroma by luminance:

          // bit counts: the (red-green) and (blue-yellow) differences
          // can range up to 12 bits, and after the multiplication by
          // 3 can range up to 14 bits, thus the use of 14 in
          // computing the upshift value above

          if (finalshift > 0)
            {
              *rgptr++ = ((3*(red - green) << upshift) / lum) << finalshift;
              *byptr++ = ((3*(blue - yellow) << upshift) / lum) << finalshift;
            }
          else if (finalshift < 0)
            {
              *rgptr++ = ((3*(red - green) << upshift) / lum) >> (-finalshift);
              *byptr++ = ((3*(blue - yellow) << upshift) / lum) >> (-finalshift);
            }
          else
            {
              *rgptr++ = (3*(red - green) << upshift) / lum;
              *byptr++ = (3*(blue - yellow) << upshift) / lum;
            }
        }
    }
}

// ######################################################################
Image<int> intgShiftImage(const Image<int>& srcImg,
                          const int dxnumer, const int dynumer,
                          const uint denombits)
{
  // make sure the source image is valid
  ASSERT(srcImg.initialized());

  ASSERT(denombits < 8*sizeof(int));

  const int denom = (1 << denombits);

  // create and clear the return image
  Dims dim(srcImg.getDims());
  int w = dim.w(), h = dim.h();
  Image<int> retImg(dim, ZEROS);

  // prepare a couple of variable for the x direction
  int xt = dxnumer >= 0
    ? (dxnumer >> denombits)
    : - ((-dxnumer + denom-1) >> denombits);
  int xfrac_numer = dxnumer - (xt << denombits);
  int startx = std::max(0,xt);
  int endx = std::min(0,xt) + w;
  if (xfrac_numer != 0 && abs(denom)/abs(xfrac_numer) > (1 << 30))
    xfrac_numer = 0;
  else endx--;

  // prepare a couple of variable for the y direction
  int yt = dynumer >= 0
    ? (dynumer >> denombits)
    : - ((-dynumer + denom-1) >> denombits);
  int yfrac_numer = dynumer - (yt << denombits);
  int starty = std::max(0,yt);
  int endy = std::min(0,yt) + h;
  if (yfrac_numer != 0 && abs(denom)/abs(yfrac_numer) > (1 << 30))
    yfrac_numer = 0;
  else endy--;

  // dispatch to faster shiftClean() if displacements are roughly
  // integer:
  if (xfrac_numer == 0 && yfrac_numer == 0)
    return shiftClean(srcImg, xt, yt);

  if (xfrac_numer > 0)
    {
      xfrac_numer = denom - xfrac_numer;
      xt++;
    }

  if (yfrac_numer > 0)
    {
      yfrac_numer = denom - yfrac_numer;
      yt++;
    }

  // prepare the pointers
  Image<int>::const_iterator src, src2 = srcImg.begin();
  Image<int>::iterator ret, ret2 = retImg.beginw();
  if (xt > 0) ret2 += xt;
  if (xt < 0) src2 -= xt;
  if (yt > 0) ret2 += yt * w;
  if (yt < 0) src2 -= yt * w;

  // now loop over the images
  for (int y = starty; y < endy; ++y)
    {
      src = src2; ret = ret2;
      for (int x = startx; x < endx; ++x)
        {
          *ret = (((src[0] >> denombits) * (denom - xfrac_numer)) >> denombits) * (denom - yfrac_numer);
          *ret += (((src[1] >> denombits) * xfrac_numer) >> denombits) * (denom - yfrac_numer);
          *ret += (((src[w] >> denombits) * (denom - xfrac_numer)) >> denombits) * yfrac_numer;
          *ret += (((src[w+1] >> denombits) * xfrac_numer) >> denombits) * yfrac_numer;
          ++src; ++ret;
        }
      src2 += w; ret2 += w;
    }
  return retImg;
}

// ######################################################################
// ##### IntgGaussianPyrBuilder Functions:
// ######################################################################

IntgGaussianPyrBuilder::IntgGaussianPyrBuilder(const int filter_size,
                                               const integer_math* imath) :
  PyrBuilder<int>(),
  itsFiltSize(filter_size),
  itsImath(imath)
{}

ImageSet<int> IntgGaussianPyrBuilder::build(const Image<int>& img,
                                            const int firstlevel,
                                            const int depth,
                                            PyramidCache<int>* cache)
{
  ASSERT(cache != 0); // FIXME remove this

  const ImageSet<int>* const cached =
    (cache != 0 && itsFiltSize == 5)
    ? cache->gaussian5.get(img) // may be null if there is no cached pyramid
    : 0;

  return (cached != 0)
    ? *cached
    : intgBuildPyrGaussian(img, depth, itsFiltSize, itsImath);
}

IntgGaussianPyrBuilder* IntgGaussianPyrBuilder::clone() const
{
  return new IntgGaussianPyrBuilder(*this);
}

// ######################################################################
// ##### IntgOrientedPyrBuilder Functions:
// ######################################################################

IntgOrientedPyrBuilder::IntgOrientedPyrBuilder(const int filter_size,
                                               const float theta,
                                               const integer_math* imath) :
  PyrBuilder<int>(),
  itsFiltSize(filter_size),
  itsAngle(theta),
  itsImath(imath)
{}

ImageSet<int> IntgOrientedPyrBuilder::build(const Image<int>& img,
                                            const int firstlevel,
                                            const int depth,
                                            PyramidCache<int>* cache)
{
  ASSERT(cache != 0); // FIXME remove this

  const ImageSet<int>* const lplc =
    (cache != 0 && itsFiltSize == 9)
    ? cache->laplacian9.get(img) // may be null if there is no cached pyramid
    : 0;

  return lplc != 0
    ? intgBuildPyrOrientedFromLaplacian(*lplc, itsFiltSize, itsAngle,
                                        itsImath)
    : intgBuildPyrOriented(img, firstlevel, depth,
                           itsFiltSize, itsAngle, itsImath);
}

IntgOrientedPyrBuilder* IntgOrientedPyrBuilder::clone() const
{
  return new IntgOrientedPyrBuilder(*this);
}

// ######################################################################
// ##### IntgReichardtPyrBuilder Functions:
// ######################################################################
IntgReichardtPyrBuilder::
IntgReichardtPyrBuilder(const int dxnumer, const int dynumer,
                        const uint denombits,
                        const integer_math* imath) :
  PyrBuilder<int>(),
  itsDXnumer(dxnumer), itsDYnumer(dynumer), itsDenomBits(denombits),
  itsImath(imath)
{}

ImageSet<int> IntgReichardtPyrBuilder::build(const Image<int>& image,
                                             const int firstlevel,
                                             const int depth,
                                             PyramidCache<int>* cache)
{
  const ImageSet<int>* const cached =
    cache != 0
    ? cache->gaussian5.get(image) // may be null if there is no cached pyramid
    : 0;

  // create a pyramid with the input image
  ImageSet<int> upyr =
    cached != 0
    ? *cached
    : intgBuildPyrGaussian(image, depth, 5, itsImath);

  // create an empty pyramid
  ImageSet<int> spyr(depth);

  // fill the empty pyramid with the shifted version
  for (int i = firstlevel; i < depth; ++i)
    spyr[i] = intgShiftImage(upyr[i], itsDXnumer, itsDYnumer,
                             itsDenomBits);

  // store both pyramids in the deques
  unshifted.push_back(upyr);
  shifted.push_back(spyr);

  ImageSet<int> result(depth);

  // so, it's our first time? Pretend the pyramid before this was
  // the same as the current one ...
  if (unshifted.size() == 1)
    {
      unshifted.push_back(upyr);
      shifted.push_back(spyr);
    }

  // need to pop off old pyramid?
  if (unshifted.size() == 3)
    {
      unshifted.pop_front();
      shifted.pop_front();
    }

  // compute the Reichardt maps
  for (int i = firstlevel; i < depth; i++)
    {
      result[i] = Image<int>(unshifted.back()[i].getDims(), NO_INIT);
      const int sz = result[i].getSize();

      Image<int>::iterator rptr = result[i].beginw();
      Image<int>::const_iterator ubptr = unshifted.back()[i].begin();
      Image<int>::const_iterator ufptr = unshifted.front()[i].begin();
      Image<int>::const_iterator sbptr = shifted.back()[i].begin();
      Image<int>::const_iterator sfptr = shifted.front()[i].begin();

      // scale bit depth down to avoid overflow when we multiply
      const uint nshift = (itsImath->nbits+1)/2;

      for (int j = 0; j < sz; ++j)
        rptr[j] =
          ((ubptr[j] >> nshift) * (sfptr[j] >> nshift)) -
          ((ufptr[j] >> nshift) * (sbptr[j] >> nshift));
    }

  return result;
}

// ######################################################################
IntgReichardtPyrBuilder* IntgReichardtPyrBuilder::clone() const
{
  return new IntgReichardtPyrBuilder(*this);
}

// ######################################################################
void IntgReichardtPyrBuilder::reset()
{
  shifted.clear();
  unshifted.clear();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_INTEGERMATHOPS_C_DEFINED
