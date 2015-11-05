/*!@file Image/FilterOps.C Filtering operations on Image */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/FilterOps.C $
// $Id: FilterOps.C 13815 2010-08-22 17:58:48Z lior $
//

#include "Image/FilterOps.H"

#include "Image/Image.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/Promotions.H"
#include "Util/log.H"
#include "rutz/trace.h"

#include <algorithm>
#include <cmath>
#include <limits>

// ######################################################################
template <class T>
  Image<typename promote_trait<T, float>::TP>
correlation(const Image<T>& src, const Image<float>& filter)
{
  ASSERT(src.initialized());
  ASSERT(filter.initialized());
  const int src_w = src.getWidth();
  const int src_h = src.getHeight();

  Image<float>::const_iterator fptrBegin = filter.begin();
  const int fil_w = filter.getWidth();
  const int fil_h = filter.getHeight();
  ASSERT((fil_w & 1) && (fil_h & 1)); //check if the filter is odd size

  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:

  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  Image<TF> result(src_w, src_h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  //const int fil_end = fil_w * fil_h - 1;
  // const int Nx2 = (fil_w - 1) / 2;
  // const int Ny2 = (fil_h - 1) / 2;

  const int srow_skip = src_w-fil_w;


  for (int dst_y = 0; dst_y < src_h-fil_h; dst_y++) {

         for (int dst_x = 0; dst_x < src_w-fil_w; dst_x++, dptr++) {

                float dst_val = 0.0f;

                Image<float>::const_iterator fptr = fptrBegin;

                // Image<float>::const_iterator srow_ptr =
                //          sptr + src_w*(dst_y-Ny2) + (dst_x - Nx2);

                Image<float>::const_iterator srow_ptr =
                  sptr + (src_w*dst_y) + dst_x;

                // LINFO("DD:%ix%i %i", dst_x,dst_y, src_w*(dst_y) + (dst_x));


                for (int f_y = 0; f_y < fil_h; ++f_y)
                {
                  for (int f_x = 0; f_x < fil_w; ++f_x){
                         dst_val += fabs((*srow_ptr++) - (*fptr++));
                  }

                  srow_ptr += srow_skip;

                }
                *dptr = dst_val;
                continue;
         }
  }

  return result;

}

// ######################################################################
template <class T>
  Image<typename promote_trait<T, float>::TP>
templMatch(const Image<T>& src, const Image<float>& filter, int method)
{
  ASSERT(src.initialized());
  ASSERT(filter.initialized());
  const int src_w = src.getWidth();
  const int src_h = src.getHeight();


  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:

  const int fil_w = filter.getWidth();
  const int fil_h = filter.getHeight();

  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  Image<TF> result(src_w-fil_w+1, src_h-fil_h+1, NO_INIT);

  result = correlation(source, filter);

  return result;
}



// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
spatialPoolMax(const Image<T>& src, const int si, const int sj,
               const int sti, const int stj)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  Image<TF> result((int)ceil((float)w / (float)si),
                  (int)ceil((float)h / (float)sj),
                  NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  // very inefficient implementation, ha ha
  for (int j = 0; j < h; j += sj)
    {
      int jmax = std::min(j + stj, h);
      for (int i = 0; i < w; i += si)
        {
          TF val = std::numeric_limits<TF>::min();
          int imax = std::min(i + sti, w);

          for (int jj = j; jj < jmax; jj ++)
            for (int ii = i; ii < imax; ii ++)
              if (sptr[jj * w + ii] > val) val = sptr[jj * w + ii];
          *dptr++ = val;
        }
    }
  return result;
}

// ######################################################################
template<class T>
float featurePoolHmax(const Image<T>& img1, const Image<T>& img2,
                      const Image<T>& img3, const Image<T>& img4,
                      const int si, const int sj, const float s2t)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img1.isSameSize(img2)&&img1.isSameSize(img3)&&img1.isSameSize(img4));

  float res = std::numeric_limits<float>::min();
  int w = img1.getWidth(), h = img1.getHeight();
  typename Image<T>::const_iterator i1 = img1.begin() + si + w * sj;
  typename Image<T>::const_iterator i2 = img2.begin() + si;
  typename Image<T>::const_iterator i3 = img3.begin() + w * sj;
  typename Image<T>::const_iterator i4 = img4.begin();

  for (int y = sj; y < h; y++)
    {
      for (int x = si; x < w; x++)
        {
          float s2r = (*i1 - s2t) * (*i1 - s2t) + (*i2 - s2t) * (*i2 - s2t) +
            (*i3 - s2t) * (*i3 - s2t) + (*i4 - s2t) * (*i4 - s2t);
          if (s2r > res) res = s2r;
          i1++; i2++; i3++; i4++;
        }
      i1 += si; i2 += si; i3 += si; i4 += si;
    }
  return res;
}

namespace
{
  pthread_once_t trigtab_init_once = PTHREAD_ONCE_INIT;

  const int TRIGTAB_SIZE = 256;

  double sintab[TRIGTAB_SIZE];
  double costab[TRIGTAB_SIZE];

  void trigtab_init()
  {
    for (int i = 0; i < 256; ++i)
      {
        const double arg = (2.0*M_PI*i) / 256.0;

#if defined(HAVE_ASM_FSINCOS)
        asm("fsincos"
            :"=t"(costab[i]), "=u"(sintab[i])
            :"0"(arg));
#elif defined(HAVE_SINCOS)
        sincos(arg, &sintab[i], &costab[i]);
#else
        sintab[i] = sin(arg);
        costab[i] = cos(arg);
#endif
      }
  }
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
orientedFilter(const Image<T>& src, const float k,
               const float theta, const float intensity,
               const bool usetab)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  double kx = double(k) * cos((theta + 90.0) * M_PI / 180.0);
  double ky = double(k) * sin((theta + 90.0) * M_PI / 180.0);

  typedef typename promote_trait<T, float>::TP TF;
  Image<TF> re(src.getDims(), NO_INIT), im(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<TF>::iterator reptr = re.beginw(), imptr = im.beginw();

  // (x,y) = (0,0) at center of image:
  int w2l = src.getWidth() / 2, w2r = src.getWidth() - w2l;
  int h2l = src.getHeight() / 2, h2r = src.getHeight() - h2l;

  if (usetab)
    {
      pthread_once(&trigtab_init_once, &trigtab_init);

      const double kx2 = (256.0 * kx) / (2.0*M_PI);
      const double ky2 = (256.0 * ky) / (2.0*M_PI);

      for (int j = -h2l; j < h2r; ++j)
        for (int i = -w2l; i < w2r; ++i)
          {
            const double arg2 = kx2 * i + ky2 * j;
            int idx = int(arg2) % 256;
            if (idx < 0) idx += 256;
            const TF val = (*sptr++) * intensity;

            const double sinarg = sintab[idx];
            const double cosarg = costab[idx];

            *reptr++ = TF(val * cosarg);
            *imptr++ = TF(val * sinarg);
          }
    }
  else
    {
      for (int j = -h2l; j < h2r; ++j)
        for (int i = -w2l; i < w2r; ++i)
          {
            const double arg = kx * i + ky * j;
            const TF val = (*sptr++) * intensity;

#if defined(HAVE_ASM_FSINCOS)
            // If we the asm "fsincos" instruction is available, then
            // that will be the fastest way to compute paired sin and
            // cos calls.
            double sinarg, cosarg;
            asm("fsincos"
                :"=t"(cosarg), "=u"(sinarg)
                :"0"(arg));

#elif defined(HAVE_SINCOS)
            // Otherwise we use a sincos() library function if it is
            // available as an extension (it's present at least in GNU
            // libc, maybe on other systems as well) then it is faster
            // (~30% overall speedup of orientedFilter()) to compute
            // both sin+cos at once rather than doing separate calls
            // -- this allows the use of the x87 assembly instruction
            // fsincos (see /usr/include/bits/mathinline.h for the
            // glibc sincos()).
            double sinarg, cosarg;
            sincos(arg, &sinarg, &cosarg);

#else
            // Otherwise we resort to explicit separate sin() and
            // cos() calls.
            double sinarg = sin(arg);
            double cosarg = cos(arg);
#endif

            *reptr++ = TF(val * cosarg);
            *imptr++ = TF(val * sinarg);
          }
    }

  re = ::lowPass9(re);
  im = ::lowPass9(im);

  return quadEnergy(re, im);
}

// ######################################################################
template <class T>
Image<T> centerSurround(const Image<T>& center,
                        const Image<T>& surround,
                        const bool absol)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int lw = center.getWidth(), lh = center.getHeight();
  const int sw = surround.getWidth(), sh = surround.getHeight();

  if (sw > lw || sh > lh) LFATAL("center must be larger than surround");

  int scalex = lw / sw, remx = lw - 1 - (lw % sw);
  int scaley = lh / sh, remy = lh - 1 - (lh % sh);

  // result has the size of the larger image:
  Image<T> result(center.getDims(), NO_INIT);

  // scan large image and subtract corresponding pixel from small image:
  int ci = 0, cj = 0;
  typename Image<T>::const_iterator lptr = center.begin();
  typename Image<T>::const_iterator sptr = surround.begin();
  typename Image<T>::iterator dptr = result.beginw();
  T zero = T();

  if (absol == true)  // compute abs(hires - lowres):
    {
      for (int j = 0; j < lh; ++j)
        {
          for (int i = 0; i < lw; ++i)
            {
              if (*lptr > *sptr)
                *dptr++ = T(*lptr++ - *sptr);
              else
                *dptr++ = T(*sptr - *lptr++);

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
                *dptr++ = T(*lptr++ - *sptr);
              else
                { *dptr++ = zero; lptr++; }

              if ((++ci) == scalex && i != remx) { ci = 0; ++sptr; }
            }
          if (ci) { ci = 0; ++sptr; } // in case the reduction is not round
          if ((++cj) == scaley && j != remy) cj = 0; else sptr -= sw;
        }
    }

  // attenuate borders:
  inplaceAttenuateBorders(result, result.getDims().max() / 20);

  return result;
}

// ######################################################################
template <class T>
void centerSurround(const Image<T>& center, const Image<T>& surround,
                    Image<T>& pos, Image<T>& neg)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int lw = center.getWidth(), lh = center.getHeight();
  const int sw = surround.getWidth(), sh = surround.getHeight();

  if (sw > lw || sh > lh) LFATAL("center must be larger than surround");

  int scalex = lw / sw, remx = lw - 1 - (lw % sw);
  int scaley = lh / sh, remy = lh - 1 - (lh % sh);

  // result has the size of the larger image:
  pos.resize(center.getDims(), NO_INIT);
  neg.resize(center.getDims(), NO_INIT);

  // scan large image and subtract corresponding pixel from small image:
  int ci = 0, cj = 0;
  typename Image<T>::const_iterator lptr = center.begin();
  typename Image<T>::const_iterator sptr = surround.begin();
  typename Image<T>::iterator pptr = pos.beginw(), nptr = neg.beginw();
  T zero = T();

  for (int j = 0; j < lh; ++j)
    {
      for (int i = 0; i < lw; ++i)
        {
          if (*lptr > *sptr) { *pptr++ = T(*lptr++ - *sptr); *nptr++ = zero; }
          else { *pptr++ = zero; *nptr++ = T(*sptr - *lptr++); }

          if ((++ci) == scalex && i != remx) { ci = 0; ++sptr; }
        }
      if (ci) { ci = 0; ++sptr; }  // in case the reduction is not round
      if ((++cj) == scaley && j != remy) cj = 0; else sptr -= sw;
    }

  // attenuate borders:
  inplaceAttenuateBorders(pos, pos.getDims().max() / 20);
  inplaceAttenuateBorders(neg, neg.getDims().max() / 20);
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
doubleOpp(const Image<T>& cplus, const Image<T>& cminus,
          const Image<T>& splus, const Image<T>& sminus)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(cplus.isSameSize(cminus)); ASSERT(splus.isSameSize(sminus));

  typedef typename promote_trait<T, float>::TP TF;

  // compute difference between both center arrays
  Image<TF> cdiff = cplus; cdiff -= cminus;

  // compute difference between both surround arrays
  Image<TF> sdiff = splus; sdiff -= sminus;

  // compute center-surround cdiff - sdiff = (cp - cm) [-] (sp - sm)
  return centerSurround(cdiff, sdiff, true);  // take abs
}

// ######################################################################
template <class T>
void avgOrient(const Image<T>& src,
               Image<float>& orient, Image<float>& strength)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  float Gf1[9] = { 0.0094, 0.1148, 0.3964, -0.0601, -0.9213,  -0.0601,
                   0.3964, 0.1148, 0.0094 };
  float Gf2[9] = { 0.0008, 0.0176, 0.1660, 0.6383, 1.0, 0.6383, 0.1660,
                   0.0176, 0.0008 };
  float Gf3[9] = { -0.0028, -0.0480, -0.3020, -0.5806, 0.0, 0.5806, 0.3020,
                   0.0480, 0.0028 };
  float Hf1[9] = { -0.0098, -0.0618, 0.0998, 0.7551, 0.0, -0.7551, -0.0998,
                   0.0618, 0.0098 };
  float Hf2[9] = { 0.0008, 0.0176, 0.1660, 0.6383, 1.0, 0.6383, 0.1660,
                   0.0176, 0.0008 };
  float Hf3[9] = { -0.0020, -0.0354, -0.2225, -0.4277, 0.0, 0.4277, 0.2225,
                   0.0354, 0.0020 };
  float Hf4[9] = { 0.0048, 0.0566, 0.1695, -0.1889, -0.7349, -0.1889, 0.1695,
                   0.0566, 0.0048 };

  Image<float> fima = src;

  // apply Nine-Tap filters to create the separable bases of G2 and H2
  Image<float> G2a = sepFilter(fima, Gf1, Gf2, 9, 9, CONV_BOUNDARY_ZERO);
  Image<float> G2b = sepFilter(fima, Gf3, Gf3, 9, 9, CONV_BOUNDARY_ZERO);
  Image<float> G2c = sepFilter(fima, Gf2, Gf1, 9, 9, CONV_BOUNDARY_ZERO);
  Image<float> H2a = sepFilter(fima, Hf1, Hf2, 9, 9, CONV_BOUNDARY_ZERO);
  Image<float> H2b = sepFilter(fima, Hf4, Hf3, 9, 9, CONV_BOUNDARY_ZERO);
  Image<float> H2c = sepFilter(fima, Hf3, Hf4, 9, 9, CONV_BOUNDARY_ZERO);
  Image<float> H2d = sepFilter(fima, Hf2, Hf1, 9, 9, CONV_BOUNDARY_ZERO);

  // combine the basis-filtered images:
  orient = Image<float>(src.getDims(), NO_INIT);
  strength = Image<float>(src.getDims(), NO_INIT);

  const int siz = src.getSize();

  typename Image<float>::const_iterator
    pG2a = G2a.begin(),
    pG2b = G2b.begin(),
    pG2c = G2c.begin(),
    pH2a = H2a.begin(),
    pH2b = H2b.begin(),
    pH2c = H2c.begin(),
    pH2d = H2d.begin();

  Image<float>::iterator porient = orient.beginw();
  Image<float>::iterator pstrength = strength.beginw();

  for (int k = 0; k < siz; ++k)
    {
      const float c2 = 0.5 * (squareOf(pG2a[k]) - squareOf(pG2c[k]))
        + 0.46875 * (squareOf(pH2a[k]) - squareOf(pH2d[k]))
        + 0.28125 * (squareOf(pH2b[k]) - squareOf(pH2c[k]))
        + 0.1875 * (pH2a[k] * pH2c[k] - pH2b[k] * pH2d[k]);
      const float c3 =
        - pG2a[k] * pG2b[k]
        - pG2b[k] * pG2c[k]
        - 0.9375 * (pH2c[k] * pH2d[k] + pH2a[k] * pH2b[k])
        - 1.6875 * pH2b[k] * pH2c[k]
        - 0.1875 * pH2a[k] * pH2d[k];
      porient[k] = (atan2(c3, c2) + M_PI)/ 2.0;  // between 0 and PI
      pstrength[k] = sqrt(squareOf(c2) + squareOf(c3));
    }
}

// ######################################################################
template <class T>
Image<T> energyNorm(const Image<T>& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // Need to bump everything above zero here so that we don't have
  // divide-by-zero problems when we later divide by the energy. FIXME
  // should be a better way???
  Image<float> result = img + 1.0f;

  // Compute the local image energy
  const Image<float> energy = sqrt(lowPass9(result * result));

  // Normalize by the local image energy
  result = result / energy; // watch divbyzero here!!!

  // Set to zero-mean
  result -= mean(result);

  return result;
}

// ######################################################################
template <class T>
Image<T> junctionFilterFull(const Image<T>& i0,  const Image<T>& i45,
                            const Image<T>& i90, const Image<T>& i135,
                            const bool r[8],     const int dx = 6,
                            const int dy = 6,
                            const bool useEuclidDiag = true)
{
GVX_TRACE(__PRETTY_FUNCTION__);

 ASSERT(i0.isSameSize(i45));
 ASSERT(i0.isSameSize(i90));
 ASSERT(i0.isSameSize(i135));

  Image<T> result(i0.getDims(), ZEROS);

  const int w = i0.getWidth(), h = i0.getHeight();

  /* example L junction: upright L(x,y) = |(x, y-1) and (NOT
     /(x+1,y-1)) and -(x+1, y) and (NOT \(x+1,y+1)) and (NOT |(x,
     y+1)) and (NOT /(x-1,y+1)) and (NOT -(x-1, y)) and (NOT
     \(x-1,y-1))

     To find NOT of an irrelevant feature, just consider how strong it
     is in comparison to the weakest relevant feature, i.e. d =
     min(all Rel responses)/2 - Irr. response

     d is high <=> the absence of this Irr. feature
     d is low <=> the presence of this Irr. feature

     *******

     3rd party comment:

     Nathan: This filter basically will give either a proportional
             response to a proper feature set or should return
             zero given 1 or more irr features. That is, notice
             that for any feature <= min rel feature is tacked
             into the multiplication. Thus, weak irr features still
             add to the strength. So this filter is sensitive to noise.
             However, it is simple and will brutally weed out any
             response given a strong irr feature.
 */

  // pixel offsets for (x+dx, y), (x+dx, y-dy), (x, y-dy), etc. going
  // counterclockwise:

  int dx_diag, dy_diag;

  // Compute the diagonal offsets if needed as a euclidian distance
  // from the center in dx,dy or should we just put it on a "grid"
  if(useEuclidDiag)
  {
    dx_diag = (int)round(sqrt(pow(dx,2)/2.0f));
    dy_diag = (int)round(sqrt(pow(dy,2)/2.0f));
  }
  else
  {
    dx_diag = dx;
    dy_diag = dy;
  }

  // non diagonal elements
  const int o0 =  dx;
  const int o2 = -dy*w;
  const int o4 = -dx;
  const int o6 =  dy*w;

  // diagonal elements
  const int o1 =  dx_diag - dy_diag*w;
  const int o3 = -dx_diag - dy_diag*w;
  const int o5 = -dx_diag + dy_diag*w;
  const int o7 =  dx_diag + dy_diag*w;

  // get pointers to the image pixel data:
  typename Image<T>::const_iterator
    ip0  = i0.begin()  + dx + dy*w, ip45  = i45.begin()  + dx + dy*w,
    ip90 = i90.begin() + dx + dy*w, ip135 = i135.begin() + dx + dy*w;
  typename Image<T>::iterator rp = result.beginw() + dx + dy*w;

  // loop over the bulk of the images (i.e., all except a border of
  // width dx and height dy):

  const T max = std::numeric_limits<T>::max();

  for (int j = (dy*2); j < h; j ++)
  {
    for (int i = (dx*2); i < w; i ++)
    {

      // get the various feature values:
      const T r0 = ip0[o0],   r4 = ip0[o4];   ++ip0;
      const T r1 = ip45[o1],  r5 = ip45[o5];  ++ip45;
      const T r2 = ip90[o2],  r6 = ip90[o6];  ++ip90;
      const T r3 = ip135[o3], r7 = ip135[o7]; ++ip135;

      // for this full implementation, let's start by finding out the
      // minimum relevant response, so that we can then check the
      // irrelevant responses against that baseline:
      T minRel = max;
      if (r[0] && r0 < minRel) minRel = r0;
      if (r[1] && r1 < minRel) minRel = r1;
      if (r[2] && r2 < minRel) minRel = r2;
      if (r[3] && r3 < minRel) minRel = r3;
      if (r[4] && r4 < minRel) minRel = r4;
      if (r[5] && r5 < minRel) minRel = r5;
      if (r[6] && r6 < minRel) minRel = r6;
      if (r[7] && r7 < minRel) minRel = r7;
      minRel /= 2;

      // now let's compute the response of the junction:
      float resp = 1.0f;

      // One at a time we test to see if an irr feature is stronger
      // than one of our rel features. If so, we return a zero reponse
      // and quit. Otherwise we keep going and normalize the final response.
      // Notice we only get a non-zero reponse of all irr features are
      // less than 1/2 of the minimum rel feature.

      if (r[0]) resp *= r0; else resp *= (minRel - r0);
      if (resp <= 0.0f) resp = 0.0f;
      else
      {
        if (r[1]) resp *= r1; else resp *= (minRel - r1);
        if (resp <= 0.0f) resp = 0.0f;
        else
        {
          if (r[2]) resp *= r2; else resp *= (minRel - r2);
          if (resp <= 0.0f) resp = 0.0f;
          else
          {
            if (r[3]) resp *= r3; else resp *= (minRel - r3);
            if (resp <= 0.0f) resp = 0.0f;
            else
            {
              if (r[4]) resp *= r4; else resp *= (minRel - r4);
              if (resp <= 0.0f) resp = 0.0f;
              else
              {
                if (r[5]) resp *= r5; else resp *= (minRel - r5);
                if (resp <= 0.0f) resp = 0.0f;
                else
                {
                  if (r[6]) resp *= r6; else resp *= (minRel - r6);
                  if (resp <= 0.0f) resp = 0.0f;
                  else
                  {
                    if (r[7]) resp *= r7; else resp *= (minRel - r7);
                    if (resp <= 0.0f) resp = 0.0f;
                    else
                    {
                      // normalize it by the number of features:
                      resp = pow(resp, 1.0/8.0);
                    }
                  }
                }
              }
            }
          }
        }
      }

      // store the response
      *rp++ = T(resp);
    }
    // skip border and go to next row of pixels:
    ip0 += dx*2; ip45 += dx*2; ip90 += dx*2; ip135 += dx*2; rp += dx*2;
  }

  return result;
}

// ######################################################################
namespace
{
  // Trivial helper function for junctionFilterPartial()
  template <class DstItr, class SrcItr>
  inline void JFILT(DstItr dptr, SrcItr sptr,
                    const int w, const int jmax, const int imax)
  {
    for(int j = 0; j < jmax; ++j)
    {
      for (int i = 0; i < imax; ++i)
        dptr[i] *= sptr[i];

      sptr += w; dptr += w;
    }
  }
}

template <class T>
Image<T> junctionFilterPartial(const Image<T>& i0,  const Image<T>& i45,
                               const Image<T>& i90, const Image<T>& i135,
                               const bool r[8],     const int dx = 6,
                               const int dy = 6,
                               const bool useEuclidDiag = false)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // the preamble is identical to junctionFilterFull:

  ASSERT(i0.isSameSize(i45) && i0.isSameSize(i90) && i0.isSameSize(i135));
  Image<T> result(i0.getDims(), ZEROS);

  const int w = i0.getWidth(), h = i0.getHeight();

  int dx_diag, dy_diag;

  // Compute the diagonal offsets if needed as a euclidian distance
  // from the center in dx,dy or should we just put it on a "grid"
  if(useEuclidDiag)
  {
    dx_diag = (int)round(fastSqrt((dx*dx)/2.0f));
    dy_diag = (int)round(fastSqrt((dy*dy)/2.0f));
  }
  else
  {
    dx_diag = dx;
    dy_diag = dy;
  }

  // non diagonal elements
  const int o0 =  dx;
  const int o2 = -dy*w;
  const int o4 = -dx;
  const int o6 =  dy*w;

  // diagonal elements
  const int o1 =  dx_diag - dy_diag*w;
  const int o3 = -dx_diag - dy_diag*w;
  const int o5 = -dx_diag + dy_diag*w;
  const int o7 =  dx_diag + dy_diag*w;

  const int offset = dx + dy*w;

  typename Image<T>::iterator const rpp = result.beginw() + offset;

  // compute the number of relevant features (for normalization):
  int nr = 0; for (int i = 0; i < 8; i++) if (r[i]) nr++;

  const int imax = w - dx*2;
  const int jmax = h - dy*2;

  // initialize the valid portion of the response array to 1.0's
  {
    typename Image<T>::iterator rp = rpp;
    for (int j = 0; j < jmax; ++j)
    {
      for (int i = 0; i < imax; ++i)
        rp[i] = T(1.0);
      rp += w;
    }
  }

  // loop over the bulk of the images, computing the responses of
  // the junction filter:
  {
    if (r[0]) JFILT(rpp, i0.begin()   + o0 + offset, w, jmax, imax);
    if (r[1]) JFILT(rpp, i45.begin()  + o1 + offset, w, jmax, imax);
    if (r[2]) JFILT(rpp, i90.begin()  + o2 + offset, w, jmax, imax);
    if (r[3]) JFILT(rpp, i135.begin() + o3 + offset, w, jmax, imax);
    if (r[4]) JFILT(rpp, i0.begin()   + o4 + offset, w, jmax, imax);
    if (r[5]) JFILT(rpp, i45.begin()  + o5 + offset, w, jmax, imax);
    if (r[6]) JFILT(rpp, i90.begin()  + o6 + offset, w, jmax, imax);
    if (r[7]) JFILT(rpp, i135.begin() + o7 + offset, w, jmax, imax);
  }

  // normalize the responses by the number of relevant features,
  // optimizing by using sqrt() or cbrt() where possible (this gives
  // an average speedup of ~3x across all junction filter types):
  {
    typename Image<T>::iterator rp = rpp;
    if (nr == 1)
    {
      // nothing to do here
    }
    else if (nr == 2)
    {
      for (int j = 0; j < jmax; ++j)
      {
        for (int i = 0; i < imax; ++i)
          rp[i] = T(fastSqrt(rp[i]));
        rp += w;
      }
    }
    else if (nr == 3)
    {
      for (int j = 0; j < jmax; ++j)
      {
        for (int i = 0; i < imax; ++i)
          rp[i] = T(cbrt(rp[i]));
        rp += w;
      }
    }
    else if (nr == 4)
    {
      for (int j = 0; j < jmax; ++j)
      {
        for (int i = 0; i < imax; ++i)
          rp[i] = T(fastSqrt(fastSqrt(rp[i])));
        rp += w;
      }
    }
    else
    {
      const double power = 1.0 / nr;
      for (int j = 0; j < jmax; ++j)
      {
        for (int i = 0; i < imax; ++i)
          rp[i] = T(pow(rp[i], power));
        rp += w;
      }
    }
  }
  return result;
}

// ######################################################################

// ######################################################################
template <class T>
Image<T> MSTFilterFull(const Image<T>& i0,  const Image<T>& i45,
                            const Image<T>& i90, const Image<T>& i135,
                            const bool r[8],     const int dx = 6,
                            const int dy = 6,
                            const bool useEuclidDiag = true)
{
GVX_TRACE(__PRETTY_FUNCTION__);

 ASSERT(i0.isSameSize(i45));
 ASSERT(i0.isSameSize(i90));
 ASSERT(i0.isSameSize(i135));

  Image<T> result(i0.getDims(), ZEROS);

  const int w = i0.getWidth(), h = i0.getHeight();

  /* example star MST: star star(x,y) = |(x, y-1) and (
     /(x+1,y-1)) and -(x+1, y) and  \(x+1,y+1) and ( |(x,
     y+1)) and ( /(x-1,y+1)) and ( -(x-1, y)) and (
     \(x-1,y-1))

     To find NOT of an irrelevant feature, just consider how strong it
     is in comparison to the weakest relevant feature, i.e. d =
     min(all Rel responses)/2 - Irr. response

     d is high <=> the absence of this Irr. feature
     d is low <=> the presence of this Irr. feature

     *******

 */

  // pixel offsets for (x+dx, y), (x+dx, y-dy), (x, y-dy), etc. going
  // counterclockwise:

  int dx_diag, dy_diag;

  // Compute the diagonal offsets if needed as a euclidian distance
  // from the center in dx,dy or should we just put it on a "grid"
  if(useEuclidDiag)
  {
    dx_diag = (int)round(sqrt(pow(dx,2)/2.0f));
    dy_diag = (int)round(sqrt(pow(dy,2)/2.0f));
  }
  else
  {
    dx_diag = dx;
    dy_diag = dy;
  }

  // non diagonal elements
  const int o0 =  dx;
  const int o2 = -dy*w;
  const int o4 = -dx;
  const int o6 =  dy*w;

  const int o8 =  dx*2;
  const int o10 = -dy*w*2;
  const int o12 = -dx*2;
  const int o14 =  dy*w*2;

  // diagonal elements
  const int o1 =  dx_diag - dy_diag*w;
  const int o3 = -dx_diag - dy_diag*w;
  const int o5 = -dx_diag + dy_diag*w;
  const int o7 =  dx_diag + dy_diag*w;

  const int o9 =  (dx_diag - dy_diag*w)*2;
  const int o11 = (-dx_diag - dy_diag*w)*2;
  const int o13 = (-dx_diag + dy_diag*w)*2;
  const int o15 =  (dx_diag + dy_diag*w)*2;

  // get pointers to the image pixel data:
  typename Image<T>::const_iterator
    ip0  = i0.begin()  + dx + dy*w, ip45  = i45.begin()  + dx + dy*w,
    ip90 = i90.begin() + dx + dy*w, ip135 = i135.begin() + dx + dy*w;
  typename Image<T>::iterator rp = result.beginw() + dx + dy*w;

  // loop over the bulk of the images (i.e., all except a border of
  // width dx and height dy):

  const T max = std::numeric_limits<T>::max();

  for (int j = (dy*2); j < h; j ++)
  {
    for (int i = (dx*2); i < w; i ++)
    {

      // get the various feature values:
      const T r0 = ip0[o0],   r4 = ip0[o4], r8 = ip0[o8],   r12 = ip0[o12];  ++ip0;
      const T r1 = ip45[o1],  r5 = ip45[o5], r9 = ip45[o9],  r13 = ip45[o13];  ++ip45;
      const T r2 = ip90[o2],  r6 = ip90[o6], r10 = ip90[o10],  r14 = ip90[o14];  ++ip90;
      const T r3 = ip135[o3], r7 = ip135[o7], r11 = ip135[o11], r15 = ip135[o15]; ++ip135;

      // for this full implementation, let's start by finding out the
      // minimum relevant response, so that we can then check the
      // irrelevant responses against that baseline:
      T minRel = max;
      if (r[0] && r0 < minRel) minRel = r0;
      if (r[1] && r1 < minRel) minRel = r1;
      if (r[2] && r2 < minRel) minRel = r2;
      if (r[3] && r3 < minRel) minRel = r3;
      if (r[4] && r4 < minRel) minRel = r4;
      if (r[5] && r5 < minRel) minRel = r5;
      if (r[6] && r6 < minRel) minRel = r6;
      if (r[7] && r7 < minRel) minRel = r7;

      if (r[0] && r8 < minRel) minRel = r8;
      if (r[1] && r9 < minRel) minRel = r9;
      if (r[2] && r10 < minRel) minRel = r10;
      if (r[3] && r11 < minRel) minRel = r11;
      if (r[4] && r12 < minRel) minRel = r12;
      if (r[5] && r13 < minRel) minRel = r13;
      if (r[6] && r14 < minRel) minRel = r14;
      if (r[7] && r15 < minRel) minRel = r15;
      minRel /= 2;

      // now let's compute the response of the MST:
      float resp = 1.0f;

      // One at a time we test to see if an irr feature is stronger
      // than one of our rel features. If so, we return a zero reponse
      // and quit. Otherwise we keep going and normalize the final response.
      // Notice we only get a non-zero reponse of all irr features are
      // less than 1/2 of the minimum rel feature.

      if (r[0]) resp *= r0; else resp *= (minRel - r0);
      if (resp <= 0.0f)
        resp = 0.0f;
      else
      {
        if (r[1]) resp *= r1; else resp *= (minRel - r1);
        if (resp <= 0.0f)
          resp = 0.0f;
        else
        {
          if (r[2]) resp *= r2;  else resp *= (minRel - r2);
          if (resp <= 0.0f)
            resp = 0.0f;
          else
          {
            if (r[3]) resp *= r3;  else resp *= (minRel - r3);
            if (resp <= 0.0f)
              resp = 0.0f;
            else
            {
              if (r[4]) resp *= r4; else resp *= (minRel - r4);
              if (resp <= 0.0f) resp = 0.0f;
              else
              {
                if (r[5]) resp *= r5; else resp *= (minRel - r5);
                if (resp <= 0.0f) resp = 0.0f;
                else
                {
                  if (r[6]) resp *= r6; else resp *= (minRel - r6);
                  if (resp <= 0.0f) resp = 0.0f;
                  else
                  {
                    if (r[7]) resp *= r7; else resp *= (minRel - r7);
                    if (resp <= 0.0f) resp = 0.0f;
                    else
                    {
                      if (r[0]) resp *= r8; else resp *= (minRel - r8);
                      if (resp <= 0.0f && r8<r0)
                        resp = 0.0f;
                      else
                      {
                          if (r[1]) resp *= r9; else resp *= (minRel - r9);
                          if (resp <= 0.0f && r9<r1)
                            resp = 0.0f;
                          else
                          {
                            if (r[2]) resp *= r10;  else resp *= (minRel - r10);
                            if (resp <= 0.0f && r10<r2)
                              resp = 0.0f;
                            else
                            {
                              if (r[3]) resp *= r11;  else resp *= (minRel - r11);
                              if (resp <= 0.0f && r11<r3)
                                resp = 0.0f;
                              else
                              {
                                  if (r[4]) resp *= r12; else resp *= (minRel - r12);
                                  if (resp <= 0.0f && r12<r4) resp = 0.0f;
                                  else
                                  {
                                    if (r[5]) resp *= r13; else resp *= (minRel - r13);
                                    if (resp <= 0.0f && r13<r5) resp = 0.0f;
                                    else
                                      {
                                        if (r[6]) resp *= r14; else resp *= (minRel - r14);
                                        if (resp <= 0.0f && r14<r6) resp = 0.0f;
                                        else
                                          {
                                            if (r[7]) resp *= r15; else resp *= (minRel - r15);
                                            if (resp <= 0.0f && r15<r7) resp = 0.0f;
                                            else
                                              {
                                                // normalize it by the number of features:
                                                resp = pow(resp, 1.0/8.0);
                                              }
                                          }
                                      }
                                  }
                              }
                            }
                          }
                      }
                      // normalize it by the number of features:
                      // resp = pow(resp, 1.0/8.0);
                    }
                  }
                }
              }
            }
          }
        }
      }

      // store the response
      *rp++ = T(resp);
    }
    // skip border and go to next row of pixels:
    ip0 += dx*2; ip45 += dx*2; ip90 += dx*2; ip135 += dx*2; rp += dx*2;
  }

  return result;
}

// ######################################################################
namespace
{
  // Trivial helper function for MSTFilterPartial()
  template <class DstItr, class SrcItr>
  inline void MSTFILT(DstItr dptr, SrcItr sptr,
                    const int w, const int jmax, const int imax)
  {
    for(int j = 0; j < jmax; ++j)
    {
      for (int i = 0; i < imax; ++i)
        dptr[i] *= sptr[i];

      sptr += w; dptr += w;
    }
  }
}

template <class T>
Image<T> MSTFilterPartial(const Image<T>& i0,  const Image<T>& i45,
                               const Image<T>& i90, const Image<T>& i135,
                               const bool r[8],     const int dx = 6,
                               const int dy = 6,
                               const bool useEuclidDiag = false)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // the preamble is identical to MSTFilterFull:

  ASSERT(i0.isSameSize(i45) && i0.isSameSize(i90) && i0.isSameSize(i135));
  Image<T> result(i0.getDims(), ZEROS);

  const int w = i0.getWidth(), h = i0.getHeight();

  int dx_diag, dy_diag;

  // Compute the diagonal offsets if needed as a euclidian distance
  // from the center in dx,dy or should we just put it on a "grid"
  if(useEuclidDiag)
  {
    dx_diag = (int)round(fastSqrt((dx*dx)/2.0f));
    dy_diag = (int)round(fastSqrt((dy*dy)/2.0f));
  }
  else
  {
    dx_diag = dx;
    dy_diag = dy;
  }

 // non diagonal elements
  const int o0 =  dx;
  const int o2 = -dy*w;
  const int o4 = -dx;
  const int o6 =  dy*w;

  const int o8 =  dx*2;
  const int o10 = -dy*w*2;
  const int o12 = -dx*2;
  const int o14 =  dy*w*2;

  // diagonal elements
  const int o1 =  dx_diag - dy_diag*w;
  const int o3 = -dx_diag - dy_diag*w;
  const int o5 = -dx_diag + dy_diag*w;
  const int o7 =  dx_diag + dy_diag*w;

  const int o9 =  (dx_diag - dy_diag*w)*2;
  const int o11 = (-dx_diag - dy_diag*w)*2;
  const int o13 = (-dx_diag + dy_diag*w)*2;
  const int o15 =  (dx_diag + dy_diag*w)*2;

  const int offset = dx + dy*w;

  typename Image<T>::iterator const rpp = result.beginw() + offset;

  // compute the number of relevant features (for normalization):
  int nr = 0; for (int i = 0; i < 8; i++) if (r[i]) nr++;

  const int imax = w - dx*2;
  const int jmax = h - dy*2;

  // initialize the valid portion of the response array to 1.0's
  {
    typename Image<T>::iterator rp = rpp;
    for (int j = 0; j < jmax; ++j)
    {
      for (int i = 0; i < imax; ++i)
        rp[i] = T(1.0);
      rp += w;
    }
  }

  // loop over the bulk of the images, computing the responses of
  // the MST filter:
  {
    if (r[0]) MSTFILT(rpp, i0.begin()   + o0 + offset, w, jmax, imax);
    if (r[1]) MSTFILT(rpp, i45.begin()  + o1 + offset, w, jmax, imax);
    if (r[2]) MSTFILT(rpp, i90.begin()  + o2 + offset, w, jmax, imax);
    if (r[3]) MSTFILT(rpp, i135.begin() + o3 + offset, w, jmax, imax);
    if (r[4]) MSTFILT(rpp, i0.begin()   + o4 + offset, w, jmax, imax);
    if (r[5]) MSTFILT(rpp, i45.begin()  + o5 + offset, w, jmax, imax);
    if (r[6]) MSTFILT(rpp, i90.begin()  + o6 + offset, w, jmax, imax);
    if (r[7]) MSTFILT(rpp, i135.begin() + o7 + offset, w, jmax, imax);

    if (r[0]) MSTFILT(rpp, i0.begin()   + o8 + offset, w, jmax, imax);
    if (r[1]) MSTFILT(rpp, i45.begin()  + o9 + offset, w, jmax, imax);
    if (r[2]) MSTFILT(rpp, i90.begin()  + o10 + offset, w, jmax, imax);
    if (r[3]) MSTFILT(rpp, i135.begin() + o11 + offset, w, jmax, imax);
    if (r[4]) MSTFILT(rpp, i0.begin()   + o12 + offset, w, jmax, imax);
    if (r[5]) MSTFILT(rpp, i45.begin()  + o13 + offset, w, jmax, imax);
    if (r[6]) MSTFILT(rpp, i90.begin()  + o14 + offset, w, jmax, imax);
    if (r[7]) MSTFILT(rpp, i135.begin() + o15 + offset, w, jmax, imax);

  }

  // normalize the responses by the number of relevant features,
  // optimizing by using sqrt() or cbrt() where possible (this gives
  // an average speedup of ~3x across all MST filter types):
  {
    typename Image<T>::iterator rp = rpp;
    if (nr == 1)
    {
      // nothing to do here
    }
    else if (nr == 2)
    {
      for (int j = 0; j < jmax; ++j)
      {
        for (int i = 0; i < imax; ++i)
          rp[i] = T(fastSqrt(rp[i]));
        rp += w;
      }
    }
    else if (nr == 3)
    {
      for (int j = 0; j < jmax; ++j)
      {
        for (int i = 0; i < imax; ++i)
          rp[i] = T(cbrt(rp[i]));
        rp += w;
      }
    }
    else if (nr == 4)
    {
      for (int j = 0; j < jmax; ++j)
      {
        for (int i = 0; i < imax; ++i)
          rp[i] = T(fastSqrt(fastSqrt(rp[i])));
        rp += w;
      }
    }
    else
    {
      const double power = 1.0 / nr;
      for (int j = 0; j < jmax; ++j)
      {
        for (int i = 0; i < imax; ++i)
          rp[i] = T(pow(rp[i], power));
        rp += w;
      }
    }
  }
  return result;
}


// ######################################################################

template <class T>
Image<typename promote_trait<T, float>::TP> gradientmag(const Image<T>& input)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typedef typename promote_trait<T, float>::TP TF;

  Image<TF> result(input.getDims(), NO_INIT);
  typename Image<T>::const_iterator src = input.begin();
  typename Image<TF>::iterator dst = result.beginw();
  const int w = input.getWidth(), h = input.getHeight();
  TF zero = TF();

  // first row is all zeros:
  for (int i = 0; i < w; i ++) *dst ++ = zero;
  src += w;

  // loop over inner rows:
  for (int j = 1; j < h-1; j ++)
    {
      // leftmost pixel is zero:
      *dst ++ = zero; ++ src;

      // loop over inner columns:
      for (int i = 1; i < w-1; i ++)
        {
          TF valx = src[1] - src[-1];
          TF valy = src[w] - src[-w];

          *dst++ = sqrt(valx * valx + valy * valy);
          ++ src;
        }

      // rightmost pixel is zero:
      *dst ++ = zero; ++ src;
    }

  // last row is all zeros:
  for (int i = 0; i < w; i ++) *dst ++ = zero;

  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP> gradientori(const Image<T>& input)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typedef typename promote_trait<T, float>::TP TF;

  Image<TF> result(input.getDims(), NO_INIT);
  typename Image<T>::const_iterator src = input.begin();
  typename Image<TF>::iterator dst = result.beginw();
  const int w = input.getWidth(), h = input.getHeight();
  TF zero = TF();

  // first row is all zeros:
  for (int i = 0; i < w; i ++) *dst ++ = zero;
  src += w;

  // loop over inner rows:
  for (int j = 1; j < h-1; j ++)
    {
      // leftmost pixel is zero:
      *dst ++ = zero; ++ src;

      // loop over inner columns:
      for (int i = 1; i < w-1; i ++)
        {
          TF valx = src[1] - src[-1];
          TF valy = src[w] - src[-w];

          *dst++ = atan2(valy, valx);
          ++ src;
        }

      // rightmost pixel is zero:
      *dst ++ = zero; ++ src;
    }

  // last row is all zeros:
  for (int i = 0; i < w; i ++) *dst ++ = zero;

  return result;
}

// ######################################################################
template <class T>
void gradient(const Image<T>& input,
              Image<typename promote_trait<T, float>::TP>& mag,
              Image<typename promote_trait<T, float>::TP>& ori)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typedef typename promote_trait<T, float>::TP TF;

  mag.resize(input.getDims()); ori.resize(input.getDims());
  typename Image<T>::const_iterator src = input.begin();
  typename Image<TF>::iterator m = mag.beginw(), o = ori.beginw();
  const int w = input.getWidth(), h = input.getHeight();
  TF zero = TF();

  // first row is all zeros:
  for (int i = 0; i < w; i ++) { *m ++ = zero; *o ++ = zero; }
  src += w;

  // loop over inner rows:
  for (int j = 1; j < h-1; j ++)
    {
      // leftmost pixel is zero:
      *m ++ = zero; *o ++ = zero; ++ src;

      // loop over inner columns:
      for (int i = 1; i < w-1; i ++)
        {
          TF valx = src[1] - src[-1];
          TF valy = src[w] - src[-w];

          *m++ = sqrt(valx * valx + valy * valy);
          *o++ = atan2(valy, valx);
          ++ src;
        }

      // rightmost pixel is zero:
      *m ++ = zero; *o ++ = zero; ++ src;
    }

  // last row is all zeros:
  for (int i = 0; i < w; i ++) { *m ++ = zero; *o ++ = zero; }
}

// ######################################################################
template <class T>
void gradientSobel(const Image<T>& input,
              Image<typename promote_trait<T, float>::TP>& mag,
              Image<typename promote_trait<T, float>::TP>& ori,
                                  int kernelSize)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typedef typename promote_trait<T, float>::TP TF;

  ASSERT( (kernelSize == 3 ) | (kernelSize == 5));
  mag.resize(input.getDims(), true); ori.resize(input.getDims(), true);
  typename Image<T>::const_iterator src = input.begin();
  typename Image<TF>::iterator m = mag.beginw(), o = ori.beginw();
  const int w = input.getWidth(), h = input.getHeight();
  TF zero = TF();

  // first rows are all zeros:
  switch (kernelSize) {
    case 3:
      for (int i = 0; i < w; i ++) { *m ++ = zero; *o ++ = zero; }
      src += w;
      break;
    case 5:
      for (int i = 0; i < w*2; i ++) { *m ++ = zero; *o ++ = zero; }
      src += w*2;
      break;
  }

  switch (kernelSize){
    case 3:
      // loop over inner rows:
      for (int j = 1; j < h-1; j ++)
      {
        // leftmost pixel is zero:
        *m ++ = zero; *o ++ = zero; ++ src;
        // loop over inner columns:
        for (int i = 1; i < w-1; i ++)
        {
          TF valx = -1*src[-1*w + -1] + 0*src[-1*w + 0] + 1*src[-1*w + 1]
            + -2*src[ 0*w + -1] + 0*src[ 0*w + 0] + 2*src[ 0*w + 1]
            + -1*src[ 1*w + -1] + 0*src[ 1*w + 0] + 1*src[ 1*w + 1];

          TF valy =  1*src[-1*w + -1] +  2*src[-1*w + 0] +  1*src[-1*w + 1]
            +  0*src[ 0*w + -1] +  0*src[ 0*w + 0] +  0*src[ 0*w + 1]
            + -1*src[ 1*w + -1] + -2*src[ 1*w + 0] + -1*src[ 1*w + 1];

          *m++ = sqrt(valx * valx + valy * valy);
          *o++ = atan2(valy, valx);
          ++ src;
        }
        // rightmost pixel is zero:
        *m ++ = zero; *o ++ = zero; ++ src;
      }
      break;

    case 5:
      // loop over inner rows:
      for (int j = 2; j < h-2; j ++)
      {
        // leftmost pixel is zero:
        *m ++ = zero; *o ++ = zero; ++ src;
        *m ++ = zero; *o ++ = zero; ++ src;
        // loop over inner columns:
        for (int i = 2; i < w-2; i ++)
        {
          TF valx = -1*src[-2*w + -2] +  -2*src[-2*w + -1] + 0*src[-2*w + 0] +  2*src[-2*w + 1] + 1*src[-2*w + 2]
            + -4*src[-1*w + -2] +  -8*src[-1*w + -1] + 0*src[-1*w + 0] +  8*src[-1*w + 1] + 4*src[-1*w + 2]
            + -6*src[ 0*w + -2] + -12*src[ 0*w + -1] + 0*src[ 0*w + 0] + 12*src[ 0*w + 1] + 6*src[ 0*w + 2]
            + -4*src[ 1*w + -2] +  -8*src[ 1*w + -1] + 0*src[ 1*w + 0] +  8*src[ 1*w + 1] + 4*src[ 1*w + 2]
            + -1*src[ 2*w + -2] +  -2*src[ 2*w + -1] + 0*src[ 2*w + 0] +  2*src[ 2*w + 1] + 1*src[ 2*w + 2];

          TF valy =  1*src[-2*w + -2] +  4*src[-2*w + -1] +   6*src[-2*w + 0] +  4*src[-2*w + 1] +  1*src[-2*w + 2]
            +  2*src[-1*w + -2] +  8*src[-1*w + -1] +  12*src[-1*w + 0] +  8*src[-1*w + 1] +  2*src[-1*w + 2]
            +  0*src[ 0*w + -2] +  0*src[ 0*w + -1] +   0*src[ 0*w + 0] +  0*src[ 0*w + 1] +  0*src[ 0*w + 2]
            + -2*src[ 1*w + -2] + -8*src[ 1*w + -1] + -12*src[ 1*w + 0] + -8*src[ 1*w + 1] + -2*src[ 1*w + 2]
            + -1*src[ 2*w + -2] + -4*src[ 2*w + -1] +  -6*src[ 2*w + 0] + -4*src[ 2*w + 1] + -1*src[ 2*w + 2];

          *m++ = sqrt(valx * valx + valy * valy);
          *o++ = atan2(valy, valx);
          ++ src;
        }
        // rightmost pixel is zero:
        *m ++ = zero; *o ++ = zero; ++ src;
      }
      break;
  }


  // last rows are all zeros:
  switch (kernelSize){
    case 3:
      for (int i = 0; i < w; i ++) { *m ++ = zero; *o ++ = zero; }
      break;
    case 5:
      for (int i = 0; i < w*2; i ++) { *m ++ = zero; *o ++ = zero; }
      break;
  }
}

// ######################################################################
Image<float> nonMaxSuppr(const Image<float>& mag, const Image<float>& ori)
{
  //Non maximal suppersion
  Image<float> outImg = mag; //(mag.getDims(), ZEROS);
  for(int y=0; y<mag.getHeight(); y++)
    for(int x=0; x<mag.getWidth(); x++)
    {
      if (mag.getVal(x,y) > 0)
      {
        float t = ori.getVal(x,y);
        //if (t<0.57 && t > -0.57)
        //{
        //  if (mag.getVal(x,y) >= mag.getVal(x,y+1) &&
        //      mag.getVal(x,y) >  mag.getVal(x,y-1))
        //    outImg.setVal(x,y, mag.getVal(x,y));
        //} else if (t > 1.04 || t < -1.04)
        //{
        //  if (mag.getVal(x,y) >= mag.getVal(x+1,y) &&
        //      mag.getVal(x,y) >  mag.getVal(x-1,y))
        //    outImg.setVal(x,y, mag.getVal(x,y));
        //} else if (t>0.57)
        //{
        //  if (mag.getVal(x,y) >= mag.getVal(x-1,y+1) &&
        //      mag.getVal(x,y) >  mag.getVal(x+1,y-1))
        //    outImg.setVal(x,y, mag.getVal(x,y));
        //} else {
        //  if (mag.getVal(x,y) >= mag.getVal(x-1,y-1) &&
        //      mag.getVal(x,y) >  mag.getVal(x+1,y+1))
        //    outImg.setVal(x,y, mag.getVal(x,y));
        //}

        
        int dx = int(1.5*cos(t));
        int dy = int(1.5*sin(t));

        if (mag.coordsOk(x+dx, y-dy) &&
            mag.coordsOk(x-dx, y+dy) )
        {
          //Remove the edge if its not a local maxima
          if (mag.getVal(x,y) < mag.getVal(x+dx, y-dy) ||
              mag.getVal(x,y) <= mag.getVal(x-dx, y+dy) )
            outImg.setVal(x,y,0);
          
        }

      }
    }

  return outImg;
}

// ######################################################################
template <class T>
Image<T> shuffleImage(const Image<T> &img)
{
  //result image
  Image<T> result(img.getDims(), NO_INIT);

  uint imgSize = img.getSize();
  for(uint i=0; i < imgSize; i++)
  {
    uint j = i + rand() / (RAND_MAX / (imgSize-i) + 1);
    result[j] = img[i];
    result[i] = img[j];
  }

  return result;
}


// Include the explicit instantiations
#include "inst/Image/FilterOps.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
