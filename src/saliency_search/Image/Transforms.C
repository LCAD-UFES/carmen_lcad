/*!@file Image/Transforms.C Transformations on Image
 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Transforms.C $
// $Id: Transforms.C 14592 2011-03-11 23:19:12Z jshen $
//

#ifndef IMAGE_TRANSFORMS_C_DEFINED
#define IMAGE_TRANSFORMS_C_DEFINED

#include "Image/Transforms.H"

#include "Image/CutPaste.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/Rectangle.H"
#include "Util/Assert.H"
#include "Util/log.H"

#include <algorithm>
#include <cmath>
#include <vector>
#include <cstdio>  // for printf()

#define SEGOBJ_START 0.9
#define SEGOBJ_STEP  0.05
#define SEGOBJ_MINA  0.005
#define SEGOBJ_EXPL  1.05

// ######################################################################
template <class T>
Image<byte> segmentObject(const Image<T>& src,
                          const Point2D<int>& seed)
{
  ASSERT(src.initialized());
  Image<T> tmp;
  bool explode = false; int prev = 0; double thresh = 1.0;
  double step = double(src.getVal(seed.i, seed.j)) * SEGOBJ_STEP;
  int minadiff = int(float(src.getSize()) * SEGOBJ_MINA);
  double efac = SEGOBJ_EXPL;
  int iter;

  while(explode == false && minadiff > 0 && efac > 1)
    {
      thresh = double(src.getVal(seed.i, seed.j)) * SEGOBJ_START;
      iter = 0;
      while(explode == false && thresh > 0)
        {
          int area = flood(src, tmp, seed, (T)thresh, (T)255.0);
          LDEBUG("prev=%d area=%d minadiff=%d thresh=%f efac=%f\n",
                 prev, area, minadiff, thresh, efac);
          if (iter > 2 && area > prev + minadiff &&
              area > int(double(prev)*efac))
            explode = true;
          else
            { prev = area; thresh -= step; }
          iter ++;
        }
      minadiff -= minadiff / 4; efac *= 0.95;
    }
  // write out segmented object: 255 in object, zero otherwise
  thresh += step;  // back 1 step before explosion
  flood(src, tmp, seed, (T)thresh, (T)255.0);
  return Image<byte>(tmp);
}

// ######################################################################
template <class T>
int segmentLandmark(const Image<T>& src,
                    const Point2D<int>& seed, Image<byte>& target,
                    double& activity, double& standout,
                    float& area_percentage)
{
  ASSERT(src.initialized());
  Image<T> tmp;

  bool explode = false; int prev = 0;
  double thresh = 1.0;
  double step = double(src.getVal(seed.i, seed.j)) * SEGOBJ_STEP;
  int minadiff = int(float(src.getSize()) * SEGOBJ_MINA);
  double efac = SEGOBJ_EXPL;
  double valdiff = 0.0, maxvaldiff = 0.0, sumvaldiff = 0.0,
    prev_thresh = 0.0, maxthresh = 0.0;
  int iter = 0; int flood_count = 0;
  bool gradual_descent = false;

  thresh = double(src.getVal(seed.i, seed.j)) * SEGOBJ_START;
  prev_thresh = thresh; activity = 0.0;
  while(minadiff > 0 && thresh > 0)
    {
      explode = false;
      while(explode == false && thresh > 0)
        {
          int area = flood(src, tmp, seed, (T)thresh, (T)255.0);

          // NOTE: g++4.3 warns about precedence of && over ||; should
          // this expression be (A&&B)||C or A&&(B||C)?
          if ((area > prev + minadiff &&
               area > int(double(prev)*efac))
              || area > 2 * prev)
            {
              explode = true;
              gradual_descent = false;
              flood_count++;
              valdiff = prev_thresh - thresh;
              sumvaldiff += valdiff;

              if(maxvaldiff < valdiff)
                {
                  maxthresh = thresh + step;
                  maxvaldiff = valdiff;
                }
              prev_thresh = thresh;
            }
          else if(area > prev + minadiff/4) gradual_descent = true;
          prev = area; thresh -= step;
          iter ++;
        }
    }
  // write out segmented object: 255.0 inside object, zero otherwise
  if(maxvaldiff == 0.0)
    {
      // flooded the first time and then no more floods

      // is this because of gradual descent?
      if(gradual_descent == true)
        standout = 0.0;
      // this is the only component and the rest is empty
      else standout = double(src.getVal(seed.i, seed.j)) * SEGOBJ_START;
      maxthresh = step; // back one step
    }
  else standout = maxvaldiff;

  if(maxthresh <= 0) return -1;
  // maxthresh is the threshold where valdiff is max

  if(flood_count != 0)
    if(maxvaldiff <= sumvaldiff/(double)flood_count)
      thresh = step;
    else thresh = maxthresh;

  else thresh = step;

  int area = flood(src, tmp, seed, (T)thresh, (T)255.0);
  if (area == -1) return -1;
  target = (Image<byte>)tmp;

  // find activity (measure of mean activity in this component)
  double threshold = thresh; double sum = 0.0;
  thresh = double(src.getVal(seed.i, seed.j)) * SEGOBJ_START;
  prev = 0; int sum_area = 0;
  activity = thresh;

  while(thresh >= threshold)
    {
      area = flood(src, tmp, seed, (T)thresh, (T)255.0);
      if(area > prev)
        activity = thresh;

      sum += activity * (area - prev);
      sum_area += area - prev;
      thresh -= step; prev = area;
    }

  if( sum_area != 0)activity = sum/(double)sum_area;
  else activity = threshold;

  if(standout > activity)
    standout = activity;

  area_percentage = (area  * 100.0f)/ (float)src.getSize();

  return area;
}

// ######################################################################
template <class T>
Image<byte> contour2D(const Image<T>& src, const byte onval, const byte offval)
{
  ASSERT(src.initialized());

  Image<byte> result(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator sptr = src.begin();
  Image<byte>::iterator dptr = result.beginw();
  T z = T(); const int h = src.getHeight(), w = src.getWidth();

  // This is a 4-connected contour algorithm. We are on the contour if we are not zero but at least: either 1) one of
  // our 4-neighbors is zero, or 2) we are on an edge of the image. Here we unfold the algo for max speed, i.e., we
  // treat the borders of the image explicitly and then the bulk:

  // first row:
  for (int i = 0; i < w; ++i) if (*sptr++) *dptr++ = onval; else *dptr++ = offval;

  // done if the image only has 1 row:
  if (h == 1) return result;

  // bulk, will only run if image has >2 rows:
  for (int j = 1; j < h-1; ++j)
    {
      // leftmost pixel of current row:
      if (*sptr++) *dptr++ = onval; else *dptr++ = offval;

      // done if image has only 1 column:
      if (w == 1) continue;

      // bulk of current row, will run only if image has >2 columns:
      for (int i = 1; i < w-1; ++i) {
        if (*sptr && (sptr[-1] == z || sptr[1] == z || sptr[-w] == z || sptr[w] == z))
          *dptr++ = onval; else *dptr++ = offval;
        ++sptr;
      }

      // rightmost pixel of current row, we know it exists since image has at least 2 columns:
      if (*sptr++) *dptr++ = onval; else *dptr++ = offval;
    }

  // last row (we know it exists since image has at least 2 rows):
  for (int i = 0; i < w; ++i) if (*sptr++) *dptr++ = onval; else *dptr++ = offval;

  return result;
}

// ######################################################################
template <class T>
Image<T> chamfer34(const Image<T>& src, const T dmax)
{

  ASSERT(src.initialized());
  ASSERT(src.getWidth() >= 3 && src.getHeight() >= 3);

  Image<T> result = src;     // init dest array and copy input to it

  const Dims dims = src.getDims();
  const int size = src.getSize();
  const int w = dims.w();
  const T zero = T();

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();

  // first, set distance to 0 inside object and max distance outside:
  for (int i = 0; i < size; i ++)
    if (*sptr++) *dptr++ = zero; else *dptr++ = dmax;
  dptr = result.beginw();

  const float tmaxf = float(dmax);

  // now let's propagate the distance first forward, and then backwards:
  float d1, d2, d3, d4;

  /* first point is untouched ... */
  ++ dptr;

  /* first line : y=0 x=1..MAXX-1 */
  for (int i = 1; i < w; ++i)
    {
      d1 = std::min(dptr[-1] + 3.0F, dptr[-1 + w] + 4.0F);
      d2 = std::min(float(*dptr), d1);
      *dptr++ = clamped_convert<T>(std::min(d2, tmaxf));
    }

  /* y=1..MAXY-2, x=0..MAXX-1 */
  for (int j = 1; j < dims.h() - 1; ++j)
    {
      /* x=0 point on each line : x=0 y=1..MAXY-2 */
      d1 = std::min(dptr[-w] + 3.0F, float(*dptr));
      *dptr++ = clamped_convert<T>(std::min(d1, tmaxf));

      /* here : x=1..MAXX-1 y=1..MAXY-2 */
      for (int k = 1; k < w; k++)
        {
          d1 = std::min(dptr[-w - 1] + 4.0F, dptr[-w] + 3.0F);
          d2 = std::min(dptr[-1] + 3.0F, float(*dptr));
          d3 = std::min(dptr[-1 + w] + 4.0F, d1);
          d4 = std::min(d2, d3);
          *dptr++ = clamped_convert<T>(std::min(d4, tmaxf));
        }
    }

  /* last line : x=0..MAXX-1 y=MAXY-1 */
  /* first point */
  d1 = std::min(dptr[-w] + 3.0F, float(*dptr));
  *dptr++ = clamped_convert<T>(std::min(d1, tmaxf));

  /* x=1..MAXX-1 y=MAXY-1 */
  for (int j = 1; j < w; ++j)
    {
      d1 = std::min(dptr[-w - 1] + 4.0F, dptr[-w] + 3.0F);
      d2 = std::min(dptr[-1] + 3.0F, float(*dptr));
      d3 = std::min(d1, d2);
      *dptr++ = clamped_convert<T>(std::min(d3, tmaxf));
    }
  --dptr; // back to last point

  /******************* backward */
  /* last point is untouched ... */
  --dptr;

  /* last line : y=MAXY-1 x=MAXX-2..0 */
  for (int j = w-2; j >= 0; --j)
    {
      d1 = std::min(dptr[1] + 3.0F, dptr[-w + 1] + 4.0F);
      d2 = std::min(float(*dptr), d1);
      *dptr-- = clamped_convert<T>(std::min(d2, tmaxf));
    }

  /* y=MAXY-2..1, x=MAXX-1..0 */
  for (int j = dims.h() - 2; j >= 1; --j)
    {
      /* x=MAXX-1 point on each line : y=MAXY-2..1 */
      d1 = std::min(dptr[w] + 3.0F, float(*dptr));
      *dptr-- = clamped_convert<T>(std::min(d1, tmaxf));

      /* here : x=MAXX-2..0 y=MAXY-2..1 z=MAXZ-1 */
      for (int k = w - 2; k >= 0; k--)
        {
          d1 = std::min(float(*dptr), dptr[-w + 1] + 4.0F);
          d2 = std::min(dptr[1] + 3.0F, dptr[w] + 3.0F);
          d3 = std::min(dptr[w + 1] + 4.0F, d1);
          d4 = std::min(d2, d3);
          *dptr-- = clamped_convert<T>(std::min(d4, tmaxf));
        }
    }

  /* first line : x=MAXX-1..0 y=0 */
  /* last point */
  d1 = std::min(dptr[w] + 3.0F, float(*dptr));
  *dptr-- = clamped_convert<T>(std::min(d1, tmaxf));

  /* x=MAXX-2..0 y=0 z=MAXZ-1 */
  for (int j = w-2; j >= 0; --j)
    {
      d1 = std::min(dptr[1] + 3.0F, float(*dptr));
      d2 = std::min(dptr[w] + 3.0F, dptr[w + 1] + 4.0F);
      d3 = std::min(d1, d2);
      *dptr-- = clamped_convert<T>(std::min(d3, tmaxf));
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> saliencyChamfer34(const Image<T>& src, const T dmax)
{

  ASSERT(src.initialized());
  ASSERT(src.getWidth() >= 3 && src.getHeight() >= 3);

  Image<T> result = src;     // init dest array and copy input to it

  const Dims dims = src.getDims();
  const int size = src.getSize();
  const int w = dims.w();
  const int h = dims.h();
  const T zero = T();

  Image<T> magImg = src;

#define ALGORITHM CHAMFER34
#if ALGORITHM == CHAMFER34
  int MASK_SIZE = 5;
  const int x_offset[] = { -1,  0,  1, -1,  0 };
  const int y_offset[] = { -1, -1, -1,  0,  0 };
  const int i_offset[] = {  4,  3,  4,  3,  0 };
#elif ALGORITHM == CHAMFER5711
  int MASK_SIZE = 9;
  int x_offset[] = { -1,  1, -2, -1,  0,  1,  2, -1,  0 };
  int y_offset[] = { -2, -2, -1, -1, -1, -1, -1,  0,  0 };
  int i_offset[] = { 11, 11, 11,  7,  5,  7, 11,  5,  0 };
#endif

  typename Image<T>::iterator sptr = magImg.beginw();
  typename Image<T>::iterator dptr = result.beginw();

  float K = 1.0F;

  // first, set distance to 0 inside object and max distance outside:
  for (int i = 0; i < size; i ++)
    if (*sptr++ > 0) *dptr++ = zero; else *dptr++ = dmax;

  double prev_measure = 0, min_measure = 0, measure = 0;
  double min_dist,min_mag;
  int d[MASK_SIZE],m[MASK_SIZE];

  int max_iter = 10;

  //Reset the pointers
  sptr = magImg.beginw();
  dptr = result.beginw();

  /* propagate distance etc */
  int j = 0;
  bool  change = false;
  do {
    j++;
    change = false;

    /* forward pass */
    for (int y = 1; y < h-1; y++) {
      for (int x = 1; x < w-1; x++) {

         prev_measure = (double) (dptr[x + w*y]+K) /
          (double) ((unsigned int)sptr[x + w*y]+K);
        for (int i = 0; i < MASK_SIZE; i++) {
          int xn = x + x_offset[i]; int yn = y + y_offset[i];
          d[i] = (int)(dptr[xn + w*yn] + i_offset[i]);
          m[i] = (unsigned int)sptr[xn + yn*w];
        }

        min_dist = d[0];
        min_mag = m[0];
        min_measure = (double) (d[0]+K) / (double) (m[0]+K);
        for (int i = 1; i < MASK_SIZE; i++) {
          measure = (double) (d[i]+K) / (double) (m[i]+K);
          if (measure < min_measure) {
            min_measure = measure;
            min_dist = d[i];
            min_mag = m[i];
          }
        }
        dptr[x + y*w] = (T)min_dist;
        sptr[x + y*w] = (T)min_mag; //TODO ?

        if (prev_measure != min_measure)
          change = true;
      }
    }

    /* backward pass */
    for (int y = h-2; y >= 1; y--) {
      for (int x = w-2; x >= 1; x--) {
        prev_measure = (double) (dptr[x + y*w]+K) /
          (double) ((unsigned int)sptr[x + y*w]+K);
        for (int i = 0; i < MASK_SIZE; i++) {
          int xn = x - x_offset[i]; int yn = y - y_offset[i];
          d[i] = (int)( dptr[xn + yn*w] + i_offset[i]);
          m[i] = (unsigned int)sptr[xn + yn*w];
        }

        min_dist = d[0];
        min_mag = m[0];
        min_measure = (double) (d[0]+K) / (double) (m[0]+K);
        for (int i = 1; i < MASK_SIZE; i++) {
          measure = (double) (d[i]+K) / (double) (m[i]+K);
          if (measure < min_measure) {
            min_measure = measure;
            min_dist = d[i];
            min_mag = m[i];
          }
        }
        dptr[x + y*w] = (T)min_dist;
        sptr[x + y*w] = (T)min_mag; //TODO ?

        if (prev_measure != min_measure)
          change = true;
      }
    }
  } while (change && j < max_iter);

  /* calculate measure values */
  for (int y = 1; y < h-1; y++) {
    for (int x = 1; x < w-1; x++) {
      dptr[x + y*w] = (T) ((double) (dptr[x +y*w]+K) /
        (double) ((unsigned int)sptr[x + y*w]+K));
    }
  }

  /* rescale */
  T max_val = dptr[0];
  for (int y = 1; y < h-1; y++)
    for (int x = 1; x < w-1; x++)
      if (dptr[x + y*w] > max_val)
        max_val = dptr[x + y*w];

  for (int y = 1; y < h-1; y++)
    for (int x = 1; x < w-1; x++)
      dptr[x + y*w] = (unsigned char)(dptr[x + y*w] * 255 / max_val);

  ///* reset borders */
  //for (y = 0; y < height; y++)
  //  tmp[0][y] = tmp[width-1][y] = 255;
  //for (x = 0; x < width; x++)
  //  tmp[x][0] = tmp[x][height-1] = 255;


  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
dct(const Image<T>& src, const int offx, const int offy, const int size)
{
  ASSERT(src.initialized());
  ASSERT(offx + size <= src.getWidth() && offy + size <= src.getHeight());

  int m = (int)(log((double)size) / log(2.0) + 0.001);
  ASSERT((1 << m) == size);  // size must be a power of two
  int N = 1 << m;

  float x[512][512], ct[512], s[512][512];
  for (int ii = 0; ii < N; ii ++)
    for (int jj = 0; jj < N; jj ++)
      x[ii][jj] = (float)src.getVal(ii+offx, jj+offy);

  /* Implementation of 2D FCT through the 1D decimation in
     frequency (DIF) split radix algorithm from paper:

     1)Skodras A.N., and Christopoulos C.A., Split-radix fast cosine
     transform, Int. J. Electronics, Vol. 74, No. 4, pp. 513-522, 1993.

     2)Christopoulos C.A., Skodras A.N. and Cornelis J., Comparative
     performance evaluation of algorithms for fast computation of the
     two-dimensional DCT, Proceedings of the IEEE Benelux & ProRISC
     Workshop on Circuits, Systems and Signal Processing, Papendal,
     Arnhem (The Netherlands), March 24, 1994, pp. 75-79.

     3)Christopoulos C.A., and Skodras A.N., On the computation of the fast
     cosine transform, Proceedings of the European Conference on Circuit
     Theory and Design (ECCTD' 93), Davos, Switzerland, Aug.30 - Sept.3,
     1993, pp. 1037-1042.

     First written:     September 1994
     Last modified:             January 1996.

     Author:
     Charilaos Christopoulos
     Ericsson Telecom AB
     Compression Lab, HF/ETX/ML
     126 24 Stockholm, Sweden

     ch.christopoulos@clab.ericsson.se

     (c) Please notice that you are allowed to use the algorithms for your
     research provided that you always give a reference to the corresponding
     paper (which is not always written by myself as you will see) and a
     reference to the author of the algorithms. You can do any modifications,
     (and add your name in the list of authors that did modifications),
     improvements, etc in the programs and you can distribute the programs to
     other researchers provided that the name of the authors of the first
     version will remain in the software.
     */
  {
    int is,id,i0,i1,i2,i3;

    //makecosinetable() /* 1D cosine table for N points */
    {
      int n2,n4;
      double e;
      float *p;

      n2 = N << 1; p = ct;

      for (int k = 1; k < m; k++) {
        e  = M_PI / n2;
        n2 = n2 >> 1;
        n4 = n2 >> 2;

        for (int j = 0; j < n4; j++) {
          *p++ = cos(((j<<2)+1)*e);
          *p++ = sin(((j<<2)+1)*e);
          *p++ = cos((((j<<2)+1)<<1)*e);

        }
      }
      *p = cos(M_PI/4.0);
    }

    // rowsinputmapping()
    {
      int n1,n2,n;

      for(n1=0; n1 <= N; n1++)
        for (n2=0; n2<N; n2++ )
          s[n1][n2] = x[n1][n2];

      for (int cols=0; cols<N; cols++) {
        for(n=0; n < N/2; n++) {
          x[cols][n]     = s[cols][2*n];
          x[cols][N-n-1] = s[cols][2*n+1];
        }
      }
    }

    for (int rows=0; rows<N; rows++) {
      int p=0;
      int n2 = N << 1;
      for (int k = 1; k < m; k++) {
        n2 = n2 >> 1;
        const int n4 = n2 >> 2;

        for (int j = 0; j < n4; j++) {
          is = j;
          id = n2 << 1;

        again1:
          for (i0 = is; i0 < N; i0 +=id) {
            i1 = i0 + n4;
            i2 = i1 + n4;
            i3 = i2 + n4;

            float t1 = 2 * (x[rows][i0] - x[rows][i2]) *ct[p] ;
            float t2 = (-2) * (x[rows][i1] - x[rows][i3]) * ct[p+1];

            x[rows][i0] = x[rows][i0] + x[rows][i2];
            x[rows][i1] = x[rows][i1] + x[rows][i3];
            x[rows][i2] = t1 + t2;
            x[rows][i3] = 2 * (t1 - t2) * ct[p+2];

            /* if at the m-1 stage, divide by 2 */
            if (k==m-1) {
              x[rows][i2] = x[rows][i2]/2;
              x[rows][i3] = x[rows][i3]/2;
            }
          }
          is = (id << 1) - n2 + j;
          id = id << 2;
          if ( is < N ) goto again1;
          p+=3;
        }
      }
      /* last stage */
      is = 0;
      id = 4;
    l_again1:
      for( i0 = is; i0 < N; i0 += id) {
        i1 = i0 + 1;
        float t1 = x[rows][i0];
        x[rows][i0] = t1 + x[rows][i1];
        /* divide the upper element by 2 */
        if (i0 != 0) x[rows][i0] = x[rows][i0]/2;
        x[rows][i1] = (t1 - x[rows][i1]) * ct[p];
      }
      is = (id << 1) - 2;
      id =  id << 2;
      if( is < N ) goto l_again1;
    }/* end of for rows */

    //rowsbitreversal() /* 2d bit reversal */
    {
      int v1, v2, v3,i,k;
      float xt;

      /* revesre rows */
      for (int cols =0; cols<N; cols ++) {
        v1 = (m+1)/2;
        v2 = 1 << v1;
        v3 = N-1-v2;
        int j=0;
        for(i=1; i<=v3; i++){
          k= N>>1;
          while(k<=j){
            j=j-k;
            k=k>>1;
          }
          j +=k;
          if(i<j){
            xt=x[cols][j];
            x[cols][j]=x[cols][i];
            x[cols][i]=xt;
          }
        }
      }
    }
    //rowspostadditions()
    {
      int step,loops,ep,i,l;

      for (int rows=0; rows<N; rows++) {
        step =N;
        loops = 1;
        for (int k=1; k<m; k++)  {
          step = step>>1;
          ep = step>>1;
          loops = loops <<1;
          for (int j=0; j<(step>>1); j++) {
            l=ep;
            for (i=1; i<loops;i++)  {
              x[rows][l+step] = x[rows][l+step]-x[rows][l];
              l =l+step;
            }
            ep +=1;
          }
        }
      }
    }
    //  columnsinputmapping() /* 2d input mapping */
    {
      int n1,n2,n;

      for(n1=0; n1 <= N; n1++)
        for (n2=0; n2<N; n2++ )
          s[n1][n2] = x[n1][n2];

      for (int rows=0; rows<N; rows++) {
        for(n=0; n < N/2; n++) {
          x[n][rows]     = s[2*n][rows];
          x[N-n-1][rows] = s[2*n+1][rows];
        }
      }
    }

    for (int cols=0; cols<N; cols++) {
      int p=0;

      int n2 = N << 1;
      for (int k = 1; k < m; k++) {
        n2 = n2 >> 1;
        const int n4 = n2 >> 2;

        for (int j = 0; j < n4; j++) {
          is = j;
          id = n2 << 1;
        again2:
          for (i0 = is; i0 < N; i0 +=id) {
            i1 = i0 + n4;
            i2 = i1 + n4;
            i3 = i2 + n4;

            float t1 = 2 * (x[i0][cols] - x[i2][cols]) *ct[p] ;
            float t2 = (-2) * (x[i1][cols]-x[i3][cols])* ct[p+1];

            x[i0][cols] = x[i0][cols] + x[i2][cols];
            x[i1][cols] = x[i1][cols] + x[i3][cols];
            x[i2][cols] = t1 + t2;
            x[i3][cols] = 2 * (t1 - t2) * ct[p+2];

            /* if at the m-1 stage, divide by 2 */
            if (k==m-1) {
              x[i2][cols] = x[i2][cols]/2;
              x[i3][cols] = x[i3][cols]/2;
            }
          }

          is = (id << 1) - n2 + j;
          id = id << 2;
          if ( is < N ) goto again2;
          p+=3;
        }
      }

      /* last stage */
      is = 0;
      id = 4;
    l_again2:
      for( i0 = is; i0 < N; i0 += id) {
        i1 = i0 + 1;

        float t1 = x[i0][cols];
        x[i0][cols] = t1 + x[i1][cols];
        /* divide the upper element by 2 */
        if (i0 != 0) x[i0][cols] = x[i0][cols]/2;
        x[i1][cols] = (t1 - x[i1][cols]) * ct[p];
      }

      is = (id << 1) - 2;
      id =  id << 2;
      if( is < N ) goto l_again2;

    }/* end of for cols */

    //  columnsbitreversal()
    {
      int v1, v2, v3,i,k;
      float xt;
      /* reverse columns */
      for (int rows =0; rows<N; rows ++) {
        v1 = (m+1)/2;
        v2 = 1 << v1;
        v3 = N-1-v2;
        int j=0;
        for(i=1; i<=v3; i++){
          k= N>>1;
          while(k<=j){
            j=j-k;
            k=k>>1;
          }
          j +=k;
          if(i<j){
            xt=x[j][rows];
            x[j][rows]=x[i][rows];
            x[i][rows]=xt;
          }
        }
      }
    }

    //columnspostadditions()
    {
      int step,loops,ep,i,l;

      for (int cols=0; cols<N; cols++) {
        step =N;
        loops = 1;
        for (int k=1; k<m; k++)  {
          step = step>>1;
          ep = step>>1;
          loops = loops <<1;
          for (int j=0; j<(step>>1); j++) {
            l=ep;
            for (i=1; i<loops; i++)  {
              x[l+step][cols] = x[l+step][cols] - x[l][cols];
              l =l+step;
            }
            ep +=1;
          }
        }
      }
    } /* end of rowspostadditions */

  }  /*    end of SR_FCT    */
  typedef typename promote_trait<T, float>::TP TF;
  Image<TF> result(N, N, NO_INIT);
  for (int ii = 0; ii < N; ii ++)
    for (int jj = 0; jj < N; jj ++)
      result.setVal(ii, jj, x[ii][jj] * 4.0F);

  return result;
}

// ######################################################################
template <class T>
float infoFFT(const Image<T>& src, const float eps, const Rectangle& rect)
{
  Image<float> tmp = crop(src, rect);
  Image<float> tmp2 = infoFFT(tmp, eps);
  float mini, maxi;
  getMinMax(tmp2, mini, maxi);
  return maxi;
}

// ######################################################################
template <class T>
Image<float> infoFFT(const Image<T>& src, const float eps)
{
  Image<float> result(src.getWidth() / 4, src.getHeight() / 4, NO_INIT);
  // look at 16x16 patches every 4 pix in x and y

  const Dims dims = src.getDims();

  for (int cx = 0; cx < dims.w(); cx += 4)      // center of patch
    for (int cy = 0; cy < dims.h(); cy += 4)
      {
        int offx = cx - 8, offy = cy - 8;
        if (dims.w() > 32 && dims.h() > 32)
          {
            // kill last column:
            if (offx == dims.w() - 16) offx = dims.w();

            // last line;for symmetry of display:
            if (offy == dims.h() - 16) offy = dims.h();
          }

        if (offx >= 0 &&
            offy >= 0 &&
            offx+16 <= dims.w() &&
            offy+16 <= dims.h())
          {
            // take 2D fft of a 16x16 patch starting at offx,
            // offy, compute abs, and compute nb coeffs > epsn
            float data[16][33];
            for (int row = 0; row < 16; row ++)
              for (int col = 0; col < 16; col ++)
                {
                  data[row][col*2+2] = 0.0;
                  data[row][col*2+1] = (float) src.getVal(offx+col, offy+row);
                }

            for (int row = 0; row < 16; row ++)
              {
                // 1D FFT routine in x
                // takes abs fft of data[row][1..32]
                int j = 1;
                for (int i = 1; i < 32; i += 2)  // bit reversal procedure
                  {
                    if (j > i)
                      { float tmp = data[row][i]; data[row][i] = data[row][j];
                      data[row][j] = tmp; tmp = data[row][i+1];
                      data[row][i+1] = data[row][j+1]; data[row][j+1] = tmp; }
                    int m = 16; while (m >= 2 && j > m) { j -= m; m >>= 1; }
                    j += m;
                  }
                int mmax = 2;
                while (32 > mmax)
                  {
                    int istep = mmax << 1; float theta = -2*M_PI/(float)mmax;
                    float wtemp = sin(0.5*theta); float wpr = -2.0*wtemp*wtemp;
                    float wpi = sin(theta); float wr = 1.0; float wi = 0.0;
                    for (int m = 1; m < mmax; m += 2)
                      {
                        for (int i = m; i <= 32; i += istep)
                          {
                            j = i + mmax;
                            float tempr=wr*data[row][j]-wi*data[row][j+1];
                            float tempi = wr*data[row][j+1]+wi*data[row][j];
                            data[row][j] = data[row][i]-tempr;
                            data[row][j+1] = data[row][i+1]-tempi;
                            data[row][i]+=tempr;
                            data[row][i+1]+=tempi;
                          }
                        wr=(wtemp=wr)*wpr-wi*wpi+wr; wi=wi*wpr+wtemp*wpi+wi;
                      }
                    mmax = istep;
                  }
              }

            // transpose to do the same thing in y
            for (int row = 0; row < 16; row ++)
              for (int col = 0; col < row; col ++)
                {
                  // transpose real and imaginary parts
                  float tt = data[row][col*2+1];
                  data[row][col*2+1] = data[col][row*2+1];
                  data[col][row*2+1] = tt;
                  tt = data[row][col*2+2];
                  data[row][col*2+2] = data[col][row*2+2];
                  data[col][row*2+2] = tt;
                }

            // redo the same stuff
            for (int row = 0; row < 16; row ++)
              {
                // 1D FFT routine in x
                // takes abs fft of data[row][1..32]
                int j = 1;
                for (int i = 1; i < 32; i += 2)  // bit reversal procedure
                  {
                    if (j > i)
                      { float tmp = data[row][i]; data[row][i] = data[row][j];
                      data[row][j] = tmp; tmp = data[row][i+1];
                      data[row][i+1] = data[row][j+1]; data[row][j+1] = tmp; }
                    int m = 16; while (m >= 2 && j > m) { j -= m; m >>= 1; }
                    j += m;
                  }
                int mmax = 2;
                while (32 > mmax)
                  {
                    int istep = mmax << 1; float theta = -2*M_PI/(float)mmax;
                    float wtemp = sin(0.5*theta); float wpr = -2.0*wtemp*wtemp;
                    float wpi = sin(theta); float wr = 1.0; float wi = 0.0;
                    for (int m = 1; m < mmax; m += 2)
                      {
                        for (int i = m; i <= 32; i += istep)
                          {
                            j = i + mmax;
                            float tempr=wr*data[row][j]-wi*data[row][j+1];
                            float tempi = wr*data[row][j+1]+wi*data[row][j];
                            data[row][j] = data[row][i]-tempr;
                            data[row][j+1] = data[row][i+1]-tempi;
                            data[row][i]+=tempr;
                            data[row][i+1]+=tempi;
                          }
                        wr=(wtemp=wr)*wpr-wi*wpi+wr; wi=wi*wpr+wtemp*wpi+wi;
                      }
                    mmax = istep;
                  }
              }

            // WARNING: at this point we have the transpose of the fft

            // count # coeffs > epsn
            float nbsup = 0.0;
            float epsn = (eps * 256.0 * 127.5)*(eps * 256.0 * 127.5);
            // epsn is normalized by patch size and ^2
            for (int row = 0; row < 16; row ++)
              for (int col = 1; col <= 32; col += 2)
                {
                  float val = data[row][col]*data[row][col] +
                    data[row][col+1]*data[row][col+1];  // this is |..|^2
                  if (val > epsn) nbsup += 1.0;
                }
            result.setVal(cx/4, cy/4, nbsup / 256.0);  // ratio>eps, in [0..1]
          }
        else
          result.setVal(cx/4, cy/4, 0.0);
      }

  return result;
}

// ######################################################################
double learningCoeff(const Image<float>& featureMap,
                     const Image<byte>& targetDMap,
                     const bool softmask,
                     const int in_thresh, const int out_thresh)
{
  ASSERT(featureMap.initialized()); ASSERT(targetDMap.initialized());
  ASSERT(featureMap.isSameSize(targetDMap));

  const bool havein = (in_thresh >= 0 && in_thresh <= 255);
  const bool haveou = (out_thresh >= 0 && out_thresh <= 255);

  if (softmask)
    {
      // remember that in the dmap zero means in the target and 255
      // outside. In the soft version, we just take the product of the
      // feature map by the mask (or 255-mask) and look for the avg
      // resulting value:
      float av = mean(featureMap) / float(featureMap.getSize());
      float oav = sum(featureMap * targetDMap) / (255.0f * sum(targetDMap));
      Image<float> mask = binaryReverse(targetDMap, byte(255));
      float iav = sum(featureMap * mask) / (255.0f * sum(mask));

      if (!havein && !haveou)
        { LFATAL("At least one threshold must be valid"); return 0.0;}
      else if (!havein && haveou)  // bg only: strong is bad
        return double((av - oav) / av);
      else if (havein && !haveou)  // target only: strong good
        return double((iav - av) / av);
      else // target+bg: higher inside than outside is good (>0), opposite bad
        return double((iav - oav) / av);
    }
  else
    {
      // Hard mask approach: examine local maxs in/out of targets and
      // compute learning weight convention: in the given distance
      // map, distance 0 is inside targets; is considered target
      // everything for which map value <= in_thresh; is considered
      // non-target everything for which map value >= out_thresh; the
      // transition between target and non-target is not used for
      // computation

      Image<byte>::const_iterator dmapptr = targetDMap.begin();
      Image<float>::const_iterator sptr = featureMap.begin(),
        stop = featureMap.end();

      float glob_min = *sptr; float glob_max = *sptr;
      float in_max = 0.0f; float out_max = 0.0f;

      while (sptr != stop)
        {
          const int mapval = *dmapptr++;
          const float val = *sptr++;
          if (mapval <= in_thresh && val > in_max) in_max = val;
          if (mapval >= out_thresh && val > out_max) out_max = val;
          if (val < glob_min) glob_min = val;
          if (val > glob_max) glob_max = val;
        }

      if (!havein && !haveou)
        { LFATAL("At least one threshold must be valid"); return 0.0;}
      else if (!havein && haveou)  // bg only: strong is bad
        return -double(in_max) / (double(glob_max) - double(glob_min));
      else if (havein && !haveou)  // target only: strong good
        return double(in_max) / (double(glob_max) - double(glob_min));
      else // target+bg: higher inside than outside is good (>0), opposite bad
        return (double(in_max) - double(out_max)) /
          (double(glob_max) - double(glob_min));
    }
}

// ######################################################################
template <class T>
Image<T> scaleBlock(const Image<T>& src, const Dims newDims)
{
  int new_w = newDims.w(), new_h = newDims.h();
  std::vector<int> col_idx(new_w), row_idx(new_h);
  int w = src.getWidth(), h = src.getHeight();

  for (int x = 0; x < new_w; x++)
    col_idx[x] = (int)((float)x / (float)new_w * (float)w);
  for (int y = 0; y < new_h; y++)
    row_idx[y] = (int)((float)y / (float)new_h * (float)h);

  Image<T> result(new_w, new_h, NO_INIT);
  typename Image<T>::iterator rptr = result.beginw();
  for (int y = 0; y < new_h; y++)
    for (int x = 0; x < new_w; x++)
      {
        *rptr++ = src.getVal(col_idx[x],row_idx[y]);
      }
  return result;
}

// ######################################################################
template <class T>
int floodClean(const Image<T>& src, Image<T>& dest, const Point2D<int>& seed,
               const T thresh, const T val, int numConn)
{
  ASSERT(src.initialized()); ASSERT(src.coordsOk(seed));

  // if dest is not initialized, initilize it to ZEROS
  // ATTENTION: the old behavior is initialization to src
  // If you want to have this behavior, you have to explicitely
  // initialize dest to src BEFORE calling floodClean
  if (!dest.initialized()) dest.resize(src.getDims(),true);

  if (src.getVal(seed) < thresh) return -1; // starting point not in object

  // Allocate space for the recursion stack
  std::vector<Point2D<int> > stk;
  const int STACKSIZE = 256 * 256 + 20;
  stk.reserve(STACKSIZE);
  stk.push_back(seed);

  // relative directions
  std::vector<Point2D<int> > dirs(8);
  dirs[0] = Point2D<int>(1,0);   dirs[1] = Point2D<int>(0,1);
  dirs[2] = Point2D<int>(0,-1);  dirs[3] = Point2D<int>(-1,0);
  dirs[4] = Point2D<int>(1,1);   dirs[5] = Point2D<int>(1,-1);
  dirs[6] = Point2D<int>(-1,-1); dirs[7] = Point2D<int>(-1,1);

  // mask for the region already visited
  Image<byte> visited(src.getDims(), ZEROS);
  int nbpix = 0;

  while (! stk.empty())
    {
      Point2D<int> index = stk.back(); stk.pop_back();
      if (!visited.getVal(index))  // we have not yet visited this point
        {
          // we know that the current point IS in our object:
          dest.setVal(index, val);  // flood output
          visited.setVal(index,1);  // don't count this one again
          ++nbpix;                  // one more flooded pixel

          for (int i = 0; i < numConn; i++)
            {
              Point2D<int> idx2 = index + dirs[i];
              if ((src.coordsOk(idx2)) &&
                  (!visited.getVal(idx2)) &&
                  (src.getVal(idx2) >= thresh)
                 )
                stk.push_back(idx2);
            }
        }
    }
  return nbpix;
}

// ######################################################################
template <class T>
int floodCleanBB(const Image<T>& src, Image<T>& dest, const Point2D<int>& seed,
                 const T thresh, const T val, Rectangle& bbox)
{
  ASSERT(src.initialized()); ASSERT(src.coordsOk(seed));

  int ll = src.getWidth(), rr = -1, tt = src.getHeight(), bb = -1;

  // if dest is not initialized, initilize it to ZEROS
  // ATTENTION: the old behavior is initialization to src
  // If you want to have this behavior, you have to explicitely
  // initialize dest to src BEFORE calling floodClean
  if (!dest.initialized()) dest.resize(src.getDims(),true);

  if (src.getVal(seed) < thresh) return -1; // starting point not in object

  // Allocate space for the recursion stack
  std::vector<Point2D<int> > stk;
  const int STACKSIZE = 256 * 256 + 20;
  stk.reserve(STACKSIZE);
  stk.push_back(seed);

  // relativ directions
  std::vector<Point2D<int> > dirs(8);
  dirs[0] = Point2D<int>(1,0);   dirs[1] = Point2D<int>(0,1);
  dirs[2] = Point2D<int>(1,1);   dirs[3] = Point2D<int>(1,-1);
  dirs[4] = Point2D<int>(0,-1);  dirs[5] = Point2D<int>(-1,0);
  dirs[6] = Point2D<int>(-1,-1); dirs[7] = Point2D<int>(-1,1);

  // mask for the region already visited
  Image<byte> visited(src.getDims(), ZEROS);
  int nbpix = 0;

  while (! stk.empty())
    {
      Point2D<int> index = stk.back(); stk.pop_back();
      if (visited.getVal(index) == 0)  // we have not yet visited this point
        {
          // we know that the current point IS in our object:
          dest.setVal(index, val);  // flood output
          visited.setVal(index,1);  // don't count this one again
          ++nbpix;                  // one more flooded pixel

          // set the bbox values if necessary
          if (index.i < ll) ll = index.i;
          if (index.i > rr) rr = index.i;
          if (index.j < tt) tt = index.j;
          if (index.j > bb) bb = index.j;

          for (int i = 0; i < 8; i++)
            {
              Point2D<int> idx2 = index + dirs[i];
              if ((src.coordsOk(idx2)) &&
                  (src.getVal(idx2) >= thresh) &&
                  (visited.getVal(idx2) == 0))
                stk.push_back(idx2);
            }
        }
    }
  if (nbpix > 0) bbox = Rectangle::tlbrI(tt,ll,bb,rr);
  return nbpix;
}

// ######################################################################
template <class T>
Image<float> distDegrade(const Image<T>& src, const Point2D<int>& foa, const float area)
{
  float s2 = area / 100.0;
  float dist2,dx,dy;
  const T zero = T();
  Image<float> result(src.getDims(),ZEROS);
  typename Image<float>::iterator rptr = result.beginw();
  typename Image<T>::const_iterator sptr = src.begin();

  for (int y = 0; y < src.getHeight(); y++)
    for (int x = 0; x < src.getWidth(); x++)
      {
        if (*sptr > zero)
          {
            dx = x - foa.i;
            dy = y - foa.j;
            dist2 = dx * dx + dy * dy;
            if (dist2 == 0.0)
              *rptr = 1.0;
            else
              *rptr = 1.0 - exp( - s2 / dist2);
          }
        sptr++; rptr++;
      }
  return result;
}

// ######################################################################
template <class T>
Image<byte> segmentObjectClean(const Image<T>& src,
                               const Point2D<int>& seed, int numConn)
{
  float start = 0.5, pstep = 0.05, mina = 0.05, expl = 1.05;
  float stop = 0.1;
  ASSERT(src.initialized());
  Image<T> tmp;
  bool explode = false; int prev = 0; double thresh = 1.0, minThresh = 0.0;
  double step = double(src.getVal(seed.i, seed.j)) * pstep;
  int minadiff = int(float(src.getSize()) * mina);
  double efac = expl;
  int iter = 0;

  // while still not explode and growth factor still above 1.0
  while(!explode && minadiff > 0 && efac > 1.0)
    {
      // update current threshold and minimum threshold
      thresh = double(src.getVal(seed.i, seed.j)) * start;
      minThresh = double(src.getVal(seed.i, seed.j)) * stop;
      iter = 0;

      // while still not explode and threshold still above minimum
      while(!explode && thresh > minThresh)
        {
          tmp.freeMem();
          int area = floodClean(src, tmp, seed, (T)thresh, (T)255.0, numConn);
          LDEBUG("%d->%d[%d] t=%.3f efac=%.3f",
                 prev, area, minadiff, thresh, efac);

          // make sure at least 1 iteration occur
          // make sure that the increase in area
          // does not go beyond "expl"
          if (iter > 1 && area > prev + minadiff &&
              area > int(double(prev)*efac))
              explode = true;
          else
            { prev = area; thresh -= step; }
          iter ++;
        }

      minadiff -= minadiff / 4; efac *= 0.95;
    }

  // write out segmented object: 255 in object, zero otherwise
  thresh += step;  // back 1 step before explosion
  tmp.freeMem();
  floodClean(src, tmp, seed, (T)thresh, (T)255.0, numConn);

  return Image<byte>(tmp);
}

// ######################################################################
template <class T>
Image<T> highThresh(const Image<T>& src, const T thresh, const T val)
{
  ASSERT(src.initialized());
  Image<T> res(src);
  typename Image<T>::iterator aptr = res.beginw(), stop = res.endw();

  while (aptr != stop)
    {
      if (*aptr >= thresh) *aptr = val;
      ++aptr;
    }
  return res;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> replaceVals(const Image<T_or_RGB>& src,
                            const T_or_RGB from,
                            const T_or_RGB to,
                            const T_or_RGB other)
{
  Image<T_or_RGB> result(src.getDims(), NO_INIT);
  typename Image<T_or_RGB>::const_iterator sptr = src.begin();
  typename Image<T_or_RGB>::iterator rptr = result.beginw();
  while (rptr != result.endw())
    *rptr++ = (*sptr++ == from) ? to : other;
  return result;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> replaceVals(const Image<T_or_RGB>& src,
                            const T_or_RGB from,
                            const T_or_RGB to)
{
  if (from == to) return src;
  Image<T_or_RGB> result(src.getDims(), NO_INIT);
  typename Image<T_or_RGB>::const_iterator sptr = src.begin();
  typename Image<T_or_RGB>::iterator rptr;
  for (rptr = result.beginw(); rptr != result.endw(); ++rptr, ++sptr)
    *rptr = (*sptr == from) ? to : *sptr;
  return result;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> composite(const Image<T_or_RGB>& fg,
                          const Image<T_or_RGB>& bg,
                          const T_or_RGB transparent)
{
  ASSERT(fg.isSameSize(bg));

  Image<T_or_RGB> result(fg.getDims(), NO_INIT);
  typename Image<T_or_RGB>::const_iterator
    fgptr = fg.begin(), bgptr = bg.begin(), stop = fg.end();
  typename Image<T_or_RGB>::iterator rptr = result.beginw();

  while(fgptr != stop)
    {
      if (*fgptr == transparent) *rptr++ = *bgptr;
      else *rptr++ = *fgptr;

      ++fgptr; ++bgptr;
    }

  return result;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> createMask(const Image<T_or_RGB>& fg,
                           const Image<bool> mask,
                           const T_or_RGB transparent = T_or_RGB())
{
  ASSERT(fg.isSameSize(mask));
  Image<T_or_RGB> result(fg.getDims(), NO_INIT);
  typename Image<T_or_RGB>::const_iterator
    fgptr = fg.begin(), stop = fg.end();
  typename Image<bool>::const_iterator
    maskptr = mask.begin();
  typename Image<T_or_RGB>::iterator rptr = result.beginw();

  while(fgptr != stop)
    {
      if (*maskptr) *rptr++ = *fgptr; //non-zero
      else *rptr++ = transparent;

      ++fgptr; ++maskptr;
    }

  return result;
    
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> mosaic(const Image<T_or_RGB>& fg,
                       const Image<T_or_RGB>* bg,
                       const T_or_RGB* transvalues,
                       const uint Nimages)
{
  uint i;
  for(i = 0; i < Nimages; i++)
    ASSERT(fg.isSameSize(bg[i]));

  Image<T_or_RGB> result(fg.getDims(), NO_INIT);
  typename Image<T_or_RGB>::const_iterator
    fgptr = fg.begin(), stop = fg.end();

  typename Image<T_or_RGB>::const_iterator bgptr[Nimages];
  for(i = 0; i < Nimages; i++)
    bgptr[i] = bg[i].begin();

  typename Image<T_or_RGB>::iterator rptr = result.beginw();

  while(fgptr != stop)
    {
      i = 0;
      while(*fgptr != transvalues[i] && i < Nimages) i++;
      if (i < Nimages) 
        *rptr++ = *bgptr[i];
      else 
        *rptr++ = *fgptr;

      ++fgptr; 
      for(i = 0; i < Nimages; i++)
        bgptr[i]++;
    }

  return result;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> alphaBlend(const Image<T_or_RGB>& fg,
                           const Image<T_or_RGB>& bg,
                           const double alpha,
                           const T_or_RGB transparent)
{
  ASSERT(fg.isSameSize(bg));
  ASSERT(alpha>=0 && alpha<=1);

  Image<T_or_RGB> result(fg.getDims(), NO_INIT);
  typename Image<T_or_RGB>::const_iterator
    fgptr = fg.begin(), bgptr = bg.begin(), stop = fg.end();
  typename Image<T_or_RGB>::iterator rptr = result.beginw();

  while(fgptr != stop)
    {
      if (*fgptr == transparent) *rptr++ = *bgptr;
      else *rptr++ = T_or_RGB((*fgptr) * (1-alpha) + T_or_RGB(*bgptr) * alpha);

      ++fgptr; ++bgptr;
    }

  return result;
}

// ######################################################################
template <class T>
Image<byte> makeBinary(const Image<T>& src, const T& threshold,
                       const byte lowVal, const byte highVal)
{
  Image<byte> result(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator sptr = src.begin();
  Image<byte>::iterator rptr = result.beginw(), stop = result.endw();
  while (rptr != stop)
    *rptr++ = (*sptr++ <= threshold) ? lowVal : highVal;
  return result;
}

// ######################################################################
template <class T>
Image<byte> makeBinary2(const Image<T>& src,
    const T& lowThresh, const T& highThresh,
    const byte lowVal, const byte highVal)
{
  Image<byte> result(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator sptr = src.begin();
  Image<byte>::iterator rptr = result.beginw(), stop = result.endw();
  while (rptr != stop)
  {
    *rptr++ = (*sptr >= lowThresh && *sptr <= highThresh) ? highVal : lowVal;
    sptr++;
  }
  return result;
}

// ######################################################################
template <class T_or_RGB>
void pasteImage(Image<T_or_RGB>& background,
                const Image<T_or_RGB>& foreground,
                const T_or_RGB& transparent,
                const Point2D<int> location,
                float opacity)
{
  float op2 = 1.0F - opacity;
  int bw = background.getWidth();
  int bh = background.getHeight();
  int fw = foreground.getWidth();
  int fh = foreground.getHeight();

  Point2D<int> bStart = location;
  Point2D<int> fStart(0,0);
  Point2D<int> bEnd(location.i + fw, location.j + fh);
  Point2D<int> fEnd(fw,fh);

  if (bStart.i < 0) { fStart.i -= bStart.i; bStart.i = 0; }
  if (bStart.j < 0) { fStart.j -= bStart.j; bStart.j = 0; }
  if (bEnd.i > bw) { fEnd.i -= (bEnd.i - bw); bEnd.i = bw; }
  if (bEnd.j > bh) { fEnd.j -= (bEnd.j - bh); bEnd.j = bh; }

  // initialize the iterators
  typename Image<T_or_RGB>::iterator bPtr1, bPtr2;
  typename Image<T_or_RGB>::const_iterator fPtr1, fPtr2;
  bPtr1 = background.beginw() + bStart.j * bw + bStart.i;
  fPtr1 = foreground.begin()  + fStart.j * fw + fStart.i;

  // loop over the image patch and paste it in
  for (int y = bStart.j; y < bEnd.j; ++y)
    {
      bPtr2 = bPtr1; fPtr2 = fPtr1;
      for (int x = bStart.i; x < bEnd.i; ++x)
        {
          if (*fPtr2 != transparent)
            *bPtr2 = T_or_RGB(*bPtr2 * op2 + *fPtr2 * opacity);
          ++bPtr2; ++fPtr2;
        }
      bPtr1 += bw; fPtr1 += fw;
    }
}

// ######################################################################
template <class T>
void inplacePasteGabor(Image<T>& dst,
                       const Image<T>& gabor,
                       const Point2D<int>& pos, const T background)
{
  int w = dst.getWidth(), h = dst.getHeight();
  int iw = gabor.getWidth(), ih = gabor.getHeight();
  ASSERT(pos.i + iw <= w && pos.j + ih <= h);
  T dist, idist;

  typename Image<T>::const_iterator sptr = gabor.begin();
  typename Image<T>::iterator dptr = dst.beginw() + pos.i + pos.j * w;
  for (int j = 0; j < ih; j ++)
    {
      for (int i = 0; i < iw; i ++)
        {
          dist = T(*dptr - background);
          if (dist < 0.0F) dist = T(dist * -1);
          if (*sptr < 0.0F) idist = T(*sptr * -1);
          else idist = *sptr;
          //take the value furthest from grey (background)
          if (dist < idist)
            {
              *dptr = T(*sptr+background);
            }
          dptr++;
          sptr++;
        }
      dptr += w - iw;
    }
}

// ######################################################################
template <class T>
int flood(const Image<T>& src,
          Image<T>& dest, const Point2D<int>& seed,
          const T thresh, const T val)
{
  ASSERT(src.initialized());
  ASSERT(src.coordsOk(seed));

  // clear destination image if necessary
  if (!dest.initialized()) dest.resize(src.getDims(), true);

  if (src.getVal(seed) < thresh) return -1; // starting point not in object

  const T zero = T();

  const int size = src.getSize();
  const int w = src.getWidth();

  // Allocate space for the recursion stack
  std::vector<int> stk;

  const int STACKSIZE = 256 * 256 + 20;
  stk.reserve(STACKSIZE);

  stk.push_back(seed.i + w * seed.j);

  Image<T> input = src;  // copy input because we will modify it
  typename Image<T>::iterator const inptr = input.beginw();

  int nbpix = 0;

  while (! stk.empty())
    {
      int index = stk.back(); stk.pop_back();
      if (inptr[index] != zero)  // we have not already visited this point
        {
          // we know that the current point IS in our object:
          dest.setVal(index, val);  // flood output
          inptr[index] = zero;      // clear this point in input
          ++nbpix;                 // one more flooded pixel

          // explore recursively the 8-connected neighbors:
          if ((index+1 < size) && (inptr[index+1] >= thresh))
            stk.push_back( index+1 );

          if ((index+w < size) && (inptr[index+w] >= thresh))
            stk.push_back( index+w );

          if ((index+w+1 < size) && (inptr[index+w+1] >= thresh))
            stk.push_back( index+w+1 );

          if ((index+w-1 < size) && (inptr[index+w-1] >= thresh))
            stk.push_back( index+w-1 );

          if ((index-1 >= 0) && (inptr[index-1] >= thresh))
            stk.push_back( index-1 );

          if ((index-w >= 0) && (inptr[index-w] >= thresh))
            stk.push_back( index-w );

          if ((index-w-1 >= 0) && (inptr[index-w-1] >= thresh))
            stk.push_back( index-w-1 );

          if ((index-w+1 >= 0) && (inptr[index-w+1] >= thresh))
            stk.push_back( index-w+1 );
        }
    }
  return nbpix;
}

// ######################################################################
template <class T>
int countParticles(const Image<T>& src, const T thresh)
{
  Point2D<int> p;
  int nbpart = 0;
  Image<T> tmp = src;
  T fill = thresh;
  const T zero = T();
  if (fill >= zero + (T)1.0) fill -= (T)1.0;  // fill with val below thresh
  else LFATAL("Cannot count particles with such low threshold");

  for (p.i = 0; p.i < src.getWidth(); ++p.i)
    for (p.j = 0; p.j < src.getHeight(); ++p.j)
      if (tmp.getVal(p) >= thresh)  // found one particle
        {
          ++nbpart;
          Image<T> tmp2 = tmp;
          flood(tmp2, tmp, p, thresh, fill);  // discard it
        }
  return nbpart;
}

// ######################################################################
template <class T>
void inplaceAddBGnoise(Image<T>& src, const float range)
{
// background noise level: as coeff of map full dynamic range:
#define BGNOISELEVEL 0.00001

  inplaceAddBGnoise2(src, range * BGNOISELEVEL);
}

// ######################################################################
template <class T>
void inplaceAddBGnoise2(Image<T>& src, const float range)
{
  ASSERT(src.initialized());
  const int w = src.getWidth(), h = src.getHeight();

  // do not put noise very close to image borders:
  int siz = std::min(w, h) / 10;

  typename Image<T>::iterator sptr = src.beginw() + siz + siz * w;

  for (int j = siz; j < h - siz; ++j)
    {
      for (int i = siz; i < w - siz; ++i)
        *sptr++ += clamped_convert<T>(range * randomDouble());
      sptr += siz + siz;
    }
}

// Include the explicit instantiations
#include "inst/Image/Transforms.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_TRANSFORMS_C_DEFINED
