/*!@file Image/Retinex.C Retinex color-correction algorithm */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Retinex.C $
// $Id: Retinex.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef IMAGE_RETINEX_C_DEFINED
#define IMAGE_RETINEX_C_DEFINED

#include "Image/Retinex.H"

#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/MathOps.H"
#include "Image/PyramidOps.H"
#include "Image/Range.H"
#include "Image/ShapeOps.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/safecopy.H"

#include <vector>

// ######################################################################
template <class T> static
Image<T> retinexCompareNeighbors(const Image<T>& src,
                                 const Image<T>& radiance,
                                 const Point2D<int> shift,
                                 const T maxi,
                                 const Rectangle& bbox)
{
  ASSERT(src.getDims() == radiance.getDims());

  Image<T> result(src.getDims(), NO_INIT);

  const int w = result.getWidth();
  const int h = result.getHeight();

  const int offset = shift.i + shift.j * w;

  const int top = bbox.top();
  const int bottom = bbox.bottomO();
  const int left = bbox.left();
  const int right = bbox.rightO();

  // the number of pixels that we need to skip after each row because
  // they are outside the range of our output rect:
  const int wskip = w - (right - left);

  typename Image<T>::const_iterator sptr = src.begin() + top * w + bbox.left();
  typename Image<T>::const_iterator ritr = radiance.begin() + top * w + bbox.left();
  typename Image<T>::iterator dptr = result.beginw() + top * w + bbox.left();

  for (int y = top; y < bottom; ++y)
    {
      for (int x = left; x < right; ++x, ++sptr, ++ritr, ++dptr)
        {
          T val;

          if (y+shift.j < 0 || y+shift.j >= h
              || x+shift.i < 0 || x+shift.i >= w)
            {
              val = sptr[0];
            }
          else
            {
              val = sptr[offset] + ritr[0] - ritr[offset];
            }

          if (val > maxi) val = maxi;

          *dptr = T(0.5F * (val + sptr[0]));
        }

      sptr += wskip;
      ritr += wskip;
      dptr += wskip;
    }

  return result;
}

// ######################################################################
size_t retinexDepth(const Dims& dims)
{
  size_t power = 0;

  while ((dims.w() >> power) > 1 &&  (dims.h() >> power) > 1)
    ++power;

  return power + 1;
}

// ######################################################################
template <class T>
Image<T> intXYWithPad(const Image<T>& src, const bool padx, const bool pady)
{
  ASSERT(src.initialized());   // works only if there is an image

  const int w = src.getWidth();
  const int h = src.getHeight();

  const int w2 = w * 2 + int(padx);
  const int h2 = h * 2 + int(pady);

  Image<T> result(w2, h2, NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();
  for (int j = 0; j < h; ++j)
    {
      for (int i = 0; i < w; ++i)
        {
          *dptr++ = *sptr;     // copy one point
          *dptr++ = *sptr;     // again
          if (padx && i == w-1)
            *dptr++ = *sptr;   // and one more time if padx and we're at the end of the line
          ++sptr;
        }
      // duplicate line we just wrote:
      safecopy(dptr, dptr - w2, w2);
      dptr += w2;
      if (pady && j == h-1)
        {
          // duplicate the line one more time if pady and we're at the last line
          safecopy(dptr, dptr - w2, w2);
          dptr += w2;
        }
    }

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrRetinexLog(const Image<T>& L,
                               const size_t depth, const int niter,
                               const Rectangle& outrect)
{
  ASSERT(depth >= 1);
  ASSERT(niter >= 1);

  const ImageSet<T> radiancePyr = buildPyrLocalAvg2x2(L, depth);

  if (radiancePyr[depth - 1].getSize() > 25)
    LFATAL("invalid toplevel image size %dx%d",
           radiancePyr[depth - 1].getWidth(),
           radiancePyr[depth - 1].getHeight());

  const Range<T> rng = rangeOf(radiancePyr[0]);

  Image<T> OP(radiancePyr[depth - 1].getDims(), NO_INIT);
  OP.clear(rng.max());

  ImageSet<T> result(depth);

  std::vector<Rectangle> bboxes(depth);
  bboxes[0] = L.getBounds().getOverlap
    (Rectangle::tlbrO(outrect.top() - niter - 1,
                      outrect.left() - niter - 1,
                      outrect.bottomO() + niter + 1,
                      outrect.rightO() + niter + 1));
  for (size_t i = 1; i < bboxes.size(); ++i)
    {
      bboxes[i] = radiancePyr[i].getBounds().getOverlap
        (Rectangle::tlbrO(bboxes[i-1].top() / 2 - niter - 1,
                          bboxes[i-1].left() / 2 - niter - 1,
                          bboxes[i-1].bottomO() / 2 + niter,
                          bboxes[i-1].rightO() / 2 + niter));
    }

  for (size_t i = 0; i < bboxes.size(); ++i)
    {
      LINFO("pyr[%" ZU "] dims=%dx%d, rect=%dx%d @ (%d,%d)",
            i, radiancePyr[i].getWidth(), radiancePyr[i].getHeight(),
            bboxes[i].width(), bboxes[i].height(),
            bboxes[i].left(), bboxes[i].top());
    }

  for (int lev = depth - 1; lev >= 0; --lev)
    {
      const Image<T> radiance = radiancePyr[lev];

      ASSERT(OP.getDims() == radiance.getDims());

      for (int iter = 0; iter < niter; ++iter)
        {
          OP = retinexCompareNeighbors(OP, radiance, Point2D<int>( 0, -1), rng.max(), bboxes[lev]);
          OP = retinexCompareNeighbors(OP, radiance, Point2D<int>( 1, -1), rng.max(), bboxes[lev]);
          OP = retinexCompareNeighbors(OP, radiance, Point2D<int>( 1,  0), rng.max(), bboxes[lev]);
          OP = retinexCompareNeighbors(OP, radiance, Point2D<int>( 1,  1), rng.max(), bboxes[lev]);
          OP = retinexCompareNeighbors(OP, radiance, Point2D<int>( 0,  1), rng.max(), bboxes[lev]);
          OP = retinexCompareNeighbors(OP, radiance, Point2D<int>(-1,  1), rng.max(), bboxes[lev]);
          OP = retinexCompareNeighbors(OP, radiance, Point2D<int>(-1,  0), rng.max(), bboxes[lev]);
          OP = retinexCompareNeighbors(OP, radiance, Point2D<int>(-1, -1), rng.max(), bboxes[lev]);
        }

      result[lev] = OP;

      if (lev > 0)
        {
          const int padx = radiancePyr[lev-1].getWidth() - OP.getWidth() * 2;
          ASSERT(padx == 0 || padx == 1);
          const int pady = radiancePyr[lev-1].getHeight() - OP.getHeight() * 2;
          ASSERT(pady == 0 || pady == 1);

          LINFO("padx = %d, pady = %d", padx, pady);

          if (!padx && !pady)
            OP = intXY(OP, true);
          else
            OP = intXYWithPad(OP, bool(padx), bool(pady));
        }
      else
        // we are going to be exiting the loop, so we don't need OP
        // anymore:
        OP = Image<T>();
    }

  return result;
}

template ImageSet<float> buildPyrRetinexLog(const Image<float>&, size_t, int,
                                            const Rectangle&);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_RETINEX_C_DEFINED
