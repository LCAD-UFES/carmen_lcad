/*!@file Image/Convolutions.C basic 1-D and 2-D filtering operations */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Convolutions.C $
// $Id: Convolutions.C 10532 2008-12-16 20:02:46Z dparks $
//

#ifndef IMAGE_CONVOLUTIONS_C_DEFINED
#define IMAGE_CONVOLUTIONS_C_DEFINED

#include "Image/Convolutions.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "rutz/trace.h"

// ######################################################################
template <class T> static
Image<typename promote_trait<T, float>::TP>
convolveZeroHelper(const Image<T>& src, const float* filter,
                   const int Nx, const int Ny)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized()); //ASSERT((Nx & 1)  && (Ny & 1));
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  const int kkk = Nx * Ny - 1;
  const int Nx2 = (Nx - 1) / 2;
  const int Ny2 = (Ny - 1) / 2;

  // very inefficient implementation; one has to be crazy to use non
  // separable filters anyway...
  for (int j = 0; j < h; ++j) // LOOP rows of src
    {
      const int j_offset = j-Ny2;

      for (int i = 0; i < w; ++i) // LOOP columns of src
        {
          TF sum = TF();
          const int i_offset = i-Nx2;
          const int kj_bgn = std::max(- j_offset, 0);
          const int kj_end = std::min(h - j_offset, Ny);

          for (int kj = kj_bgn; kj < kj_end; ++kj) // LOOP rows of filter
            {
              const int kjj = kj + j_offset; // row of src
              const int src_offset = w * kjj + i_offset;
              const int filt_offset = kkk - Nx*kj;

              const int ki_bgn = std::max(-i_offset, 0);
              const int ki_end = std::min(w-i_offset, Nx);

              for (int ki = ki_bgn; ki < ki_end; ++ki) // LOOP cols of filt
                sum += sptr[ki + src_offset] * filter[filt_offset - ki];
            }
          *dptr++ = sum;
        }
    }
  return result;
}

// ######################################################################
template <class T> static
Image<typename promote_trait<T, float>::TP>
convolveCleanHelper(const Image<T>& src, const float* filter,
                   const int Nx, const int Ny)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized()); //ASSERT((Nx & 1)  && (Ny & 1));
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  const int kkk = Nx * Ny - 1;
  const int Nx2 = (Nx - 1) / 2;
  const int Ny2 = (Ny - 1) / 2;

  float sumf = 0.0f;
  for (int i = 0; i < Nx*Ny; ++i)
    sumf += filter[i];

  // very inefficient implementation; one has to be crazy to use non
  // separable filters anyway...
  for (int j = 0; j < h; ++j) // LOOP rows of src
    {
      const int j_offset = j-Ny2;

      for (int i = 0; i < w; ++i) // LOOP columns of src
        {
          TF sum = TF();
          float sumw = 0.0f;
          const int i_offset = i-Nx2;
          const int kj_bgn = std::max(- j_offset, 0);
          const int kj_end = std::min(h - j_offset, Ny);

          for (int kj = kj_bgn; kj < kj_end; ++kj) // LOOP rows of filter
            {
              const int kjj = kj + j_offset; // row of src
              const int src_offset = w * kjj + i_offset;
              const int filt_offset = kkk - Nx*kj;

              const int ki_bgn = std::max(-i_offset, 0);
              const int ki_end = std::min(w-i_offset, Nx);

              for (int ki = ki_bgn; ki < ki_end; ++ki) // LOOP cols of filt
                {
                  sum += sptr[ki + src_offset] * filter[filt_offset - ki];
                  sumw += filter[filt_offset - ki];
                }
            }
          *dptr++ = sum * sumf / sumw;
        }
    }
  return result;
}

// ######################################################################
template <class T> static
Image<typename promote_trait<T, float>::TP>
convolve(const Image<T>& src, const float* filter,
         const int Nx, const int Ny,
         ConvolutionBoundaryStrategy boundary)
{
  switch (boundary)
    {
    case CONV_BOUNDARY_ZERO:
      return convolveZeroHelper(src, filter, Nx, Ny);
    case CONV_BOUNDARY_CLEAN:
      return convolveCleanHelper(src, filter, Nx, Ny);
    case CONV_BOUNDARY_REPLICATE:
      // not implemented yet
      break;
    }

  LFATAL("convolution boundary strategy %d not supported",
         (int) boundary);
  /* can't happen */ return Image<typename promote_trait<T, float>::TP>();
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
optConvolve(const Image<T>& src, const Image<float>& f)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  const int src_w = src.getWidth();
  const int src_h = src.getHeight();

  Image<float>::const_iterator filter = f.begin();
  const int fil_w = f.getWidth();
  const int fil_h = f.getHeight();

  ASSERT((fil_w & 1) && (fil_h & 1));

  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  Image<TF> result(src_w, src_h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  const int fil_end = fil_w * fil_h - 1;
  const int Nx2 = (fil_w - 1) / 2;
  const int Ny2 = (fil_h - 1) / 2;

  const int srow_skip = src_w-fil_w;

  for (int dst_y = 0; dst_y < src_h; ++dst_y)
    {
      // Determine if we're safely inside the image in the y-direction:
      const bool y_clean = dst_y >= Ny2 && dst_y <  (src_h - Nx2);

      for (int dst_x = 0; dst_x < src_w; ++dst_x, ++dptr)
        {
          // Determine if we're safely inside the image in the x-direction:
          const bool x_clean = dst_x >= Nx2 && dst_x <  (src_w - Nx2);

          // Here is where we pick whether we can use the optimized inner
          // loop (in cases where the filter and image patch overlap
          // completely) or whether we must use the inner loop that can
          // handle boundary conditions.

          if (x_clean && y_clean)
            {
              float dst_val = 0.0f;

              Image<float>::const_iterator fptr = filter+fil_end;

              Image<float>::const_iterator srow_ptr =
                sptr + src_w*(dst_y-Nx2) + dst_x - Nx2;

              for (int f_y = 0; f_y < fil_h; ++f_y)
                {
                  for (int f_x = 0; f_x < fil_w; ++f_x)
                    dst_val += (*srow_ptr++) * (*fptr--);

                  srow_ptr += srow_skip;
                }
              *dptr = dst_val;
              continue;
            }
          else
            {
              // OK, we're at an image boundary, so what do we do to make
              // up for the missing pixels? The approach here is to
              // compute the average value of the non-missing pixels, and
              // proceed as if the missing pixels had that value. This
              // minimizes the introduction of edge artifacts e.g. when
              // convolving with an oriented filter.

              float dst_val = 0.0f;
              float src_sum = 0.0f;
              int src_cnt = 0;
              float fil_sum_skipped = 0.0f;

              for (int f_y = 0; f_y < fil_h; ++f_y)
                {
                  const int src_y = f_y + dst_y - Ny2;
                  if (src_y >= 0 && src_y < src_h)
                    {
                      for (int f_x = 0; f_x < fil_w; ++f_x)
                        {
                          const float fil = filter[fil_end - f_x - fil_w*f_y];

                          const int src_x = f_x + dst_x - Nx2;
                          if (src_x >= 0 && src_x < src_w)
                            {
                              const float src_val = sptr[src_x + src_w * src_y];
                              dst_val += src_val * fil;
                              src_sum += src_val;
                              ++src_cnt;
                            }
                          else
                            {
                              fil_sum_skipped += fil;
                            }
                        }
                    }
                  else
                    {
                      for (int f_x = 0; f_x < fil_w; ++f_x)
                        fil_sum_skipped += filter[fil_end - f_x - fil_w*f_y];
                    }
                }
              const float src_avg = src_sum / src_cnt;
              *dptr = dst_val + (fil_sum_skipped * src_avg);
            }
        }
    }
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
convolveHmax(const Image<T>& src, const Image<float>& filter)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  const int Nx = filter.getWidth(), Ny = filter.getHeight();
  ASSERT((Nx & 1) && (Ny & 1));

  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();
  typename Image<float>::const_iterator const filt = filter.begin();

  int kkk = Nx * Ny - 1;
  int Nx2 = (Nx - 1) / 2, Ny2 = (Ny - 1) / 2;
  // very inefficient implementation; one has to be crazy to use non
  // separable filters anyway...
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      {
        TF sum = TF(), sumw = TF();
        for (int kj = 0; kj < Ny; ++kj)
          {
            int kjj = kj + j - Ny2;
            if (kjj >= 0 && kjj < h)
              for (int ki = 0; ki < Nx; ++ki)
                {
                  int kii = ki + i - Nx2;
                  if (kii >= 0 && kii < w)
                    {
                      float fil = filt[kkk - ki - Nx*kj];
                      TF img = sptr[kii + w * kjj];
                      sum += img * fil; sumw += img * img;
                    }
                }
          }
        //if (sumw > 0.0F) sum /= fastSqrt(sumw);
        if (sumw > 0.0F) sum = fabs(sum)/sqrt(sumw);
        *dptr++ = sum;
      }
  return result;
}

// ######################################################################
template <class T>
static Image<typename promote_trait<T, float>::TP>
xFilter(const Image<T>& src, const float* hFilt, const int hfs,
        ConvolutionBoundaryStrategy boundary)
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
  if (hfs == 0) return source;  // no filter
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  const int hfs2 = (hfs - 1) / 2;

  // flip the filter to accelerate convolution:
  float *hFilter = new float[hfs]; float sumh = 0.0f;
  for (int x = 0; x < hfs; x ++)
    { sumh += hFilt[x]; hFilter[hfs-1-x] = hFilt[x]; }

  // *** horizontal pass ***
  if (w < hfs)  // special function for very small images
    {
      switch (boundary)
        {
        case CONV_BOUNDARY_ZERO:
          {
            for (int j = 0; j < h; j ++)
              for (int i = 0; i < w; i ++)
                {
                  TF val = TF();
                  for (int k = 0; k < hfs; k ++)
                    if (i + k - hfs2 >= 0 && i + k - hfs2 < w)
                      {
                        val += sptr[k - hfs2] * hFilter[k];
                      }
                  *dptr++ = val; sptr ++;
                }
          }
          break;
        case CONV_BOUNDARY_CLEAN:
          {
            for (int j = 0; j < h; j ++)
              for (int i = 0; i < w; i ++)
                {
                  TF val = TF(); float sum = 0.0F;
                  for (int k = 0; k < hfs; k ++)
                    if (i + k - hfs2 >= 0 && i + k - hfs2 < w)
                      {
                        val += sptr[k - hfs2] * hFilter[k];
                        sum += hFilter[k];
                      }
                  *dptr++ = val * sumh / sum; sptr ++;
                }
          }
          break;
        case CONV_BOUNDARY_REPLICATE:
          {
            for (int j = 0; j < h; j ++)
              for (int i = 0; i < w; i ++)
                {
                  TF val = TF();
                  for (int k = 0; k < hfs; k ++)
                    if (i + k - hfs2 < 0)
                      {
                        val += sptr[-i] * hFilter[k];
                      }
                    else if (i + k - hfs2 >= w)
                      {
                        val += sptr[w-1-i] * hFilter[k];
                      }
                    else
                      {
                        val += sptr[k - hfs2] * hFilter[k];
                      }
                  *dptr++ = val; sptr ++;
                }
          }
          break;
        default:
          LFATAL("invalid convolution boundary strategy %d",
                 (int) boundary);
        }
    }
  else  // function for reasonably large images
    for (int jj = 0; jj < h; jj ++)
      {
        // leftmost points:
        switch (boundary)
          {
          case CONV_BOUNDARY_ZERO:
            {
              for (int j = hfs2; j > 0; --j)
                {
                  TF val = TF();
                  for (int k = j; k < hfs; ++k)
                    {
                      val += sptr[k - j] * hFilter[k];
                    }
                  *dptr++ = val;
                }
            }
            break;
          case CONV_BOUNDARY_CLEAN:
            {
              for (int j = hfs2; j > 0; --j)
                {
                  TF val = TF(); float sum = 0.0F;
                  for (int k = j; k < hfs; ++k)
                    {
                      val += sptr[k - j] * hFilter[k];
                      sum += hFilter[k];
                    }
                  *dptr++ = val * sumh / sum;
                }
            }
            break;
          case CONV_BOUNDARY_REPLICATE:
            {
              for (int j = hfs2; j > 0; --j)
                {
                  TF val = TF();
                  for (int k = 0; k < j; ++k)
                    {
                      val += sptr[0] * hFilter[k];
                    }
                  for (int k = j; k < hfs; ++k)
                    {
                      val += sptr[k - j] * hFilter[k];
                    }
                  *dptr++ = val;
                }
            }
            break;
//           case CONV_BOUNDARY_REFLECT:
//             {
//               for (int j = hfs2; j > 0; --j)
//                 {
//                   TF val = TF();
//                   for (int k = 0; k < j; ++k)
//                     {
//                       val += sptr[j - k] * hFilter[k];
//                     }
//                   for (int k = j; k < hfs; ++k)
//                     {
//                       val += sptr[k - j] * hFilter[k];
//                     }
//                   *dptr++ = val;
//                 }
//             }
//             break;
          default:
            LFATAL("invalid convolution boundary strategy %d",
                   (int) boundary);
          }
        // bulk (far from the borders):
        for (int i = 0; i < w - hfs + 1; i ++)
          {
            TF val = TF();
            for (int k = 0; k < hfs; k ++) val += sptr[k] * hFilter[k];
            *dptr++ = val; sptr++;
          }
        // rightmost points:
        switch (boundary)
          {
          case CONV_BOUNDARY_ZERO:
            {
              for (int j = hfs - 1; j > hfs2; --j)
                {
                  TF val = TF();
                  for (int k = 0; k < j; ++k)
                    {
                      val += sptr[k] * hFilter[k];
                    }
                  *dptr++ = val;
                  sptr++;
                }
              }
            break;
          case CONV_BOUNDARY_CLEAN:
            {
              for (int j = hfs - 1; j > hfs2; --j)
                {
                  TF val = TF(); float sum = 0.0F;
                  for (int k = 0; k < j; ++k)
                    {
                      val += sptr[k] * hFilter[k];
                      sum += hFilter[k];
                    }
                  *dptr++ = val * sumh / sum;
                  sptr++;
                }
            }
            break;
          case CONV_BOUNDARY_REPLICATE:
            {
              for (int j = hfs - 1; j > hfs2; --j)
                {
                  TF val = TF();
                  for (int k = 0; k < j; ++k)
                    {
                      val += sptr[k] * hFilter[k];
                    }
                  for (int k = j; k < hfs; ++k)
                    {
                      val += sptr[j-1] * hFilter[k];
                    }
                  *dptr++ = val;
                  sptr++;
                }
              }
            break;
//           case CONV_BOUNDARY_REFLECT:
//             {
//               for (int j = hfs - 1; j > hfs2; --j)
//                 {
//                   TF val = TF();
//                   for (int k = 0; k < j; ++k)
//                     {
//                       val += sptr[k] * hFilter[k];
//                     }
//                   for (int k = j; k < hfs; ++k)
//                     {
//                       val += sptr[2*j-k-1] * hFilter[k];
//                     }
//                   *dptr++ = val;
//                   sptr++;
//                 }
//             }
//             break;
          default:
            LFATAL("invalid convolution boundary strategy %d",
                   (int) boundary);
          }
        sptr += hfs2;  // sptr back to same as dptr (start of next line)
      }
  delete [] hFilter;
  return result;
}

// ######################################################################
template <class T>
static Image<typename promote_trait<T, float>::TP>
yFilter(const Image<T>& src, const float* vFilt, const int vfs,
        ConvolutionBoundaryStrategy boundary)
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
  if (vfs == 0) return source;  // no filter
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  const int vfs2 = (vfs - 1) / 2;

  // flip the filter to accelerate convolution:
  float *vFilter = new float[vfs]; float sumv = 0.0f;
  for (int x = 0; x < vfs; x ++)
    { sumv += vFilt[x]; vFilter[vfs-1-x] = vFilt[x]; }

  int ww[vfs];
  for (int i = 0; i < vfs; i ++) ww[i] = w * i; // speedup precompute

  if (h < vfs)  // special function for very small images
    {
      switch (boundary)
        {
        case CONV_BOUNDARY_ZERO:
          {
            for (int j = 0; j < h; j ++)
              for (int i = 0; i < w; i ++)
                {
                  TF val = TF();
                  for (int k = 0; k < vfs; k ++)
                    if (j + k - vfs2 >= 0 && j + k - vfs2 < h)
                      {
                        val += sptr[w * (k - vfs2)] * vFilter[k];
                      }
                  *dptr++ = val; sptr ++;
                }
          }
          break;
        case CONV_BOUNDARY_CLEAN:
          {
            for (int j = 0; j < h; j ++)
              for (int i = 0; i < w; i ++)
                {
                  TF val = TF(); float sum = 0.0;
                  for (int k = 0; k < vfs; k ++)
                    if (j + k - vfs2 >= 0 && j + k - vfs2 < h)
                      {
                        val += sptr[w * (k - vfs2)] * vFilter[k];
                        sum += vFilter[k];
                      }
                  *dptr++ = val * sumv / sum; sptr ++;
                }
          }
          break;
        case CONV_BOUNDARY_REPLICATE:
          {
            for (int j = 0; j < h; j ++)
              for (int i = 0; i < w; i ++)
                {
                  TF val = TF();
                  for (int k = 0; k < vfs; k ++)
                    if (j + k - vfs2 < 0)
                      {
                        val += sptr[w * (-j)] * vFilter[k];
                      }
                    else if (j + k - vfs2 >= h)
                      {
                        val += sptr[w * (h-1-j)] * vFilter[k];
                      }
                    else
                      {
                        val += sptr[w * (k - vfs2)] * vFilter[k];
                      }
                  *dptr++ = val; sptr ++;
                }
          }
          break;
        default:
          LFATAL("invalid convolution boundary strategy %d",
                 (int) boundary);
        }
    }
  else  // function for reasonably large images
    {
      // top points:
      switch (boundary)
        {
        case CONV_BOUNDARY_ZERO:
          {
            for (int j = vfs2; j > 0; --j)
              {
                for (int i = 0; i < w; ++i)  // scan all points of given horiz
                  {
                    TF val = TF();
                    for (int k = j; k < vfs; ++k)
                      {
                        val += sptr[ww[k - j]] * vFilter[k];
                      }
                    *dptr++ = val;
                    sptr++;
                  }
                sptr -= w;   // back to top-left corner
              }
          }
          break;
        case CONV_BOUNDARY_CLEAN:
          {
            for (int j = vfs2; j > 0; --j)
              {
                for (int i = 0; i < w; ++i)  // scan all points of given horiz
                  {
                    TF val = TF(); float sum = 0.0;
                    for (int k = j; k < vfs; ++k)
                      {
                        val += sptr[ww[k - j]] * vFilter[k];
                        sum += vFilter[k];
                      }
                    *dptr++ = val * sumv / sum;
                    sptr++;
                  }
                sptr -= w;   // back to top-left corner
              }
          }
          break;
        case CONV_BOUNDARY_REPLICATE:
          {
            for (int j = vfs2; j > 0; --j)
              {
                for (int i = 0; i < w; ++i)  // scan all points of given horiz
                  {
                    TF val = TF();
                    for (int k = 0; k < j; ++k)
                      {
                        val += sptr[ww[0]] * vFilter[k];
                      }
                    for (int k = j; k < vfs; ++k)
                      {
                        val += sptr[ww[k - j]] * vFilter[k];
                      }
                    *dptr++ = val;
                    sptr++;
                  }
                sptr -= w;   // back to top-left corner
              }
          }
          break;
        default:
          LFATAL("invalid convolution boundary strategy %d",
                 (int) boundary);
        }
      // bulk (far from edges):
      for (int j = 0; j < h - vfs + 1; j ++)
        for (int i = 0; i < w; i ++)
          {
            TF val = TF();
            for (int k = 0; k < vfs; k ++) val += sptr[ww[k]] * vFilter[k];
            *dptr++ = val; sptr ++;
          }
      // bottommost points:
      switch (boundary)
        {
        case CONV_BOUNDARY_ZERO:
          {
            for (int j = vfs - 1; j > vfs2; --j)
              for (int i = 0; i < w; ++i)
                {
                  TF val = TF();
                  for (int k = 0; k < j; ++k)
                    {
                      val += sptr[ww[k]] * vFilter[k];
                    }
                  *dptr++ = val;
                  sptr ++;
                }
          }
          break;
        case CONV_BOUNDARY_CLEAN:
          {
            for (int j = vfs - 1; j > vfs2; --j)
              for (int i = 0; i < w; ++i)
                {
                  TF val = TF(); float sum = 0.0;
                  for (int k = 0; k < j; ++k)
                    {
                      val += sptr[ww[k]] * vFilter[k];
                      sum += vFilter[k];
                    }
                  *dptr++ = val * sumv / sum;
                  sptr ++;
                }
          }
          break;
        case CONV_BOUNDARY_REPLICATE:
          {
            for (int j = vfs - 1; j > vfs2; --j)
              for (int i = 0; i < w; ++i)
                {
                  TF val = TF();
                  for (int k = 0; k < j; ++k)
                    {
                      val += sptr[ww[k]] * vFilter[k];
                    }
                  for (int k = j; k < vfs; ++k)
                    {
                      val += sptr[ww[j-1]] * vFilter[k];
                    }
                  *dptr++ = val;
                  sptr ++;
                }
          }
          break;
        default:
          LFATAL("invalid convolution boundary strategy %d",
                 (int) boundary);
        }
    }
  delete [] vFilter;
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
sepFilter(const Image<T>& src, const Image<float>& hFilter,
          const Image<float>& vFilter,
          ConvolutionBoundaryStrategy boundary)
{
  ASSERT(hFilter.is1D() || hFilter.getSize() == 0);
  ASSERT(vFilter.is1D() || vFilter.getSize() == 0);
  return sepFilter(src, hFilter.getArrayPtr(), vFilter.getArrayPtr(),
                   hFilter.getSize(), vFilter.getSize(), boundary);
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
sepFilter(const Image<T>& src, const float* hFilt, const float* vFilt,
          const int hfs, const int vfs,
          ConvolutionBoundaryStrategy boundary)
{
  Image<typename promote_trait<T, float>::TP> result = src;

  if (hfs > 0) result = xFilter(result, hFilt, hfs, boundary);
  if (vfs > 0) result = yFilter(result, vFilt, vfs, boundary);

  return result;
}

// Include the explicit instantiations
#include "inst/Image/Convolutions.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_CONVOLUTIONS_C_DEFINED
