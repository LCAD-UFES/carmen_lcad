/*!@file Image/CutPaste.C Cut+paste operations from/to Image subregions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/CutPaste.C $
// $Id: CutPaste.C 15107 2011-12-10 04:23:35Z kai $
//

#include "Image/CutPaste.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/Assert.H"
#include "Util/safecopy.H"
#include "rutz/trace.h"
#include "Image/vec2.h"

#include <algorithm>

// ######################################################################
template <class T>
Image<T> concatX(const Image<T>& left, const Image<T>& right)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!left.initialized()) return right;
  else if (!right.initialized()) return left;

  ASSERT(left.getHeight() == right.getHeight());
  const int h = left.getHeight();
  const int wl = left.getWidth();
  const int wr = right.getWidth();
  Image<T> result(wl + wr, left.getHeight(), NO_INIT);
  typename Image<T>::const_iterator lptr = left.begin();
  typename Image<T>::const_iterator rptr = right.begin();
  typename Image<T>::iterator dptr = result.beginw();
  for (int j = 0; j < h; ++j)
    {
      for (int i1 = 0; i1 < wl; i1 ++) *dptr++ = *lptr++;
      for (int i2 = 0; i2 < wr; i2 ++) *dptr++ = *rptr++;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> concatY(const Image<T>& top, const Image<T>& bottom)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!top.initialized()) return bottom;
  else if (!bottom.initialized()) return top;

  ASSERT(top.getWidth() == bottom.getWidth());
  const int w = top.getWidth();
  const int ht = top.getHeight();
  const int hb = bottom.getHeight();
  Image<T> result(w, ht + hb, NO_INIT);
  typename Image<T>::const_iterator tptr = top.begin();
  typename Image<T>::const_iterator bptr = bottom.begin();
  typename Image<T>::iterator dptr = result.beginw();
  for (int i = 0; i < ht * w; ++i) *dptr++ = *tptr++;
  for (int i2 = 0; i2 < hb * w; i2 ++) *dptr++ = *bptr++;
  return result;
}

// ######################################################################
template <class T>
Image<T> concatLooseY(const Image<T>& topImg,
                      const Image<T>& bottomImg,
                      const T& bgColor)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const int dsth = topImg.getHeight() + bottomImg.getHeight();
  const int dstw = std::max(topImg.getWidth(), bottomImg.getWidth());

  const int topw = topImg.getWidth();
  const int toph = topImg.getHeight();
  const int botw = bottomImg.getWidth();
  const int both = bottomImg.getHeight();

  Image<T> result = Image<T>(dstw, dsth, NO_INIT);

  result.clear(bgColor);

  T* const dptr = result.getArrayPtr();
  const T* const topptr = topImg.getArrayPtr();
  const T* const botptr = bottomImg.getArrayPtr();

  for (int j=0; j < toph; ++j)
    for (int i=0; i < topw; ++i)
      dptr[i + j * dstw] = topptr[i + j * topw];

  for (int j=0; j < both; ++j)
    for (int i=0; i < botw; ++i)
      dptr[i + (j+toph) * dstw] = botptr[i + j * botw];

  return result;
}

// ######################################################################
template <class T>
Image<T> concatLooseX(const Image<T>& leftImg,
                      const Image<T>& rightImg,
                      const T& bgColor)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const int dstw = leftImg.getWidth() + rightImg.getWidth();
  const int dsth = std::max(leftImg.getHeight(), rightImg.getHeight());

  const int lw = leftImg.getWidth();
  const int lh = leftImg.getHeight();
  const int rw = rightImg.getWidth();
  const int rh = rightImg.getHeight();

  Image<T> result = Image<T>(dstw, dsth, NO_INIT);

  result.clear(bgColor);

  T* const dptr = result.getArrayPtr();
  const T* const lptr = leftImg.getArrayPtr();
  const T* const rptr = rightImg.getArrayPtr();

  for (int j=0; j < lh; ++j)
    for (int i=0; i < lw; ++i)
      dptr[i + j * dstw] = lptr[i + j * lw];

  for (int j=0; j < rh; ++j)
    for (int i=0; i < rw; ++i)
      dptr[(i + lw) + j * dstw] = rptr[i + j * rw];

  return result;
}

// ######################################################################
template <class T>
Image<T> crop(const Image<T>& src, const Point2D<int>& pt, const Dims& dims,
              const bool zerofill)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (pt == Point2D<int>(0,0) && dims == src.getDims())
    return src;

  Image<T> result(dims, NO_INIT);

  typename Image<T>::iterator dptr = result.beginw();

  if (!zerofill)
    {
      ASSERT(src.coordsOk(pt));
      ASSERT(src.coordsOk(pt.i + dims.w() - 1, pt.j + dims.h() - 1));

      typename Image<T>::const_iterator sptr =
        src.begin()
        + pt.j * src.getWidth()   // Skip downward over starting rows
        + pt.i;                   // Skip rightward over starting columns

      const int src_catchup = src.getWidth() - result.getWidth();

      for (int y = 0; y < dims.h(); ++y)
        {
          for (int x = 0; x < dims.w(); ++x)
            {
              *dptr++ = *sptr++;
            }
          sptr += src_catchup;
        }
    }
  else
    {
      const T zero = T();

      const int ymax = pt.j + dims.h();
      const int xmax = pt.i + dims.w();

      for (int y = pt.j; y < ymax; ++y)
        for (int x = pt.i; x < xmax; ++x)
          {
            if (src.coordsOk(x, y)) *dptr = src.getVal(x, y);
            else                    *dptr = zero;
            ++dptr;
          }
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> crop(const Image<T>& src, const Rectangle& rect, const bool zerofill)
{
  return crop(src,
              Point2D<int>(rect.left(), rect.top()),
              Dims(rect.width(), rect.height()),
              zerofill);
}

// ######################################################################
template <class T>
Image<T> shift(const Image<T>& srcImg, const int dx, const int dy)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // make sure the source image is valid
  ASSERT(srcImg.initialized());

  // create and clear the return image
  Dims dim(srcImg.getDims());
  Image<T> retImg(srcImg.getDims(), ZEROS);

  // create the source and destination iterators
  typename Image<T>::const_iterator srcBegPtr = srcImg.begin();
  typename Image<T>::const_iterator srcEndPtr = srcImg.end();
  typename Image<T>::iterator retBegPtr = retImg.beginw();
  typename Image<T>::iterator retEndPtr = retImg.endw();

  // adjust the pointers according to the direction of image shift
  int shiftAmt = dy * dim.w() + dx;
  if(shiftAmt < 0)
    srcBegPtr -= shiftAmt;
  else
    retBegPtr += shiftAmt;

  // determine width of the actual data to be transferred
  //  int newW = dim.w() - abs(dx);
  //  int quad = dx*dy;

  // transfer from source to destination image
  //  for(int i=0; (srcBegPtr != srcEndPtr) && (retBegPtr != retEndPtr);
  //      ++i, ++srcBegPtr, ++retBegPtr) {
    //    if(((quad < 0) && (i%dim.w() < abs(dx))) ||
    //       ((quad > 0) && (i%dim.w() >= newW)))
    //      *retBegPtr = T(0);
    //    else
  //      *retBegPtr = *srcEndPtr;
  //  } // for block

  // transfer from source to destination image
  while((srcBegPtr != srcEndPtr) && (retBegPtr != retEndPtr))
    *retBegPtr++ = *srcBegPtr++;

  return retImg;
}

// ######################################################################
template <class T>
Image<T> shiftImage(const Image<T>& srcImg, const float dx, const float dy)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // make sure the source image is valid
  ASSERT(srcImg.initialized());

  // create and clear the return image
  Dims dim(srcImg.getDims());
  int w = dim.w(), h = dim.h();
  Image<T> retImg(dim, ZEROS);

  // prepare a couple of variable for the x direction
  int xt = (int)floor(dx);
  float xfrac = dx - xt;
  int startx = std::max(0,xt);
  int endx = std::min(0,xt) + w;
  if (fabs(xfrac) < 1.0e-10) xfrac = 0.0;
  else endx--;

  // prepare a couple of variable for the y direction
  int yt = (int)floor(dy);
  float yfrac = dy - yt;
  int starty = std::max(0,yt);
  int endy = std::min(0,yt) + h;
  if (fabs(yfrac) < 1.0e-10) yfrac = 0.0;
  else endy--;

  // dispatch to faster shiftClean() if displacements are roughly integer:
  if (fabs(xfrac) < 1.0e-10 && fabs(yfrac) < 1.0e-10)
    return shiftClean(srcImg, xt, yt);

  if (xfrac > 0.0)
  {
    xfrac = 1.0 - xfrac;
    xt++;
  }

  if (yfrac > 0.0)
  {
    yfrac = 1.0 - yfrac;
    yt++;
  }

  // prepare the coefficients
  float tl = (1.0 - xfrac) * (1.0 - yfrac);
  float tr = xfrac * (1.0 - yfrac);
  float bl = (1.0 - xfrac) * yfrac;
  float br = xfrac * yfrac;

  // prepare the pointers
  typename Image<T>::const_iterator src, src2 = srcImg.begin();
  typename Image<T>::iterator ret, ret2 = retImg.beginw();
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
          (*ret) = (T)((*src) * tl);
          if (tr > 0.0) (*ret) += (T)((*(src + 1)) * tr);
          if (bl > 0.0) (*ret) += (T)((*(src + w)) * bl);
          if (br > 0.0) (*ret) += (T)((*(src + w + 1)) * br);
          ++src; ++ret;
        }
      src2 += w; ret2 += w;
    }
  return retImg;
}

// ######################################################################
template <class T>
Image<T> shiftClean(const Image<T>& srcImg, const int dx, const int dy,
                    const T bgval)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // make sure the source image is valid
  ASSERT(srcImg.initialized());

  // create and clear the return image
  int w = srcImg.getWidth(), h = srcImg.getHeight();
  Image<T> retImg(w, h, NO_INIT); retImg.clear(bgval);

  // create the source and destination iterators
  typename Image<T>::const_iterator src = srcImg.begin();
  typename Image<T>::iterator dst = retImg.beginw();

  // find range of pixels to copy:
  int startx = std::max(0, -dx), endx = std::min(w - 1, w - 1 - dx);
  if (startx >= w || endx < 0) return retImg; // empty result
  int starty = std::max(0, -dy), endy = std::min(h - 1, h - 1 - dy);
  if (starty >= h || endy < 0) return retImg; // empty result

  int dstx = std::max(0, std::min(w - 1, dx));
  int dsty = std::max(0, std::min(h - 1, dy));

  src += startx + starty * w;
  dst += dstx + dsty * w;

  int skip = w - endx + startx - 1;

  // do the copy:
  for (int j = starty; j <= endy; j ++)
    {
      for (int i = startx; i <= endx; i ++) *dst++ = *src++;

      // ready for next row of pixels:
      src += skip; dst += skip;
    }

  return retImg;
}

// ######################################################################
template <class T>
void inplacePaste(Image<T>& dst,
                  const Image<T>& img, const Point2D<int>& pos)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  int w = dst.getWidth(), h = dst.getHeight();
  int iw = img.getWidth(), ih=img.getHeight();

  ASSERT(pos.i + iw <= w && pos.j + ih <= h);

  typename Image<T>::const_iterator sptr = img.begin();
  typename Image<T>::iterator dptr = dst.beginw() + pos.i + pos.j * w;
  for (int j = 0; j < ih; j ++)
    {
      safecopy(dptr, sptr, iw);
      dptr += w;
      sptr += iw;
    }
}

// ######################################################################
template <class T>
void inplaceClearRegion(Image<T>& dst,
                        const Rectangle& region1, const T& val)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const Rectangle region = region1.getOverlap(dst.getBounds());

  if (!region.isValid() || !dst.initialized())
    return;

  const int w = dst.getWidth();
  const int iw = region.width(), ih = region.height();

  ASSERT(dst.rectangleOk(region));

  typename Image<T>::iterator dptr = dst.beginw() + region.left() + region.top() * w;
  for (int j = 0; j < ih; ++j)
    {
      for (int i = 0; i < iw; ++i)
        dptr[i] = val;
      dptr += w;
    }
}

// ######################################################################
template <class T>
void inplaceEmbed(Image<T>& dst,
                  const Image<T>& img, const Rectangle& r,
                  const T background, const bool keep_aspect)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(dst.initialized() && img.initialized());
  ASSERT(dst.rectangleOk(r));
  int ww = r.width(), hh = r.height();
  int iw = img.getWidth(), ih = img.getHeight();
  int myw = dst.getWidth();
  float sw = float(iw) / float(ww), sh = float(ih) / float(hh);

  // take larger of scale factors if keep aspect ratio:
  if (keep_aspect)
    {
      if (sw > sh) sh = sw;
      if (sh > sw) sw = sh;
    }

  // determine bounds of actual embedded image, within rectangle:
  const int imin = (ww - int(float(iw) / sw)) / 2;
  const int imax = ww - imin;
  const int jmin = (hh - int(float(ih) / sh)) / 2;
  const int jmax = hh - jmin;

  typename Image<T>::iterator aptr = dst.beginw();
  typename Image<T>::const_iterator sptr = img.begin();
  aptr += r.top() * myw + r.left();

  // top empty lines:
  for (int j = 0; j < jmin; j ++)
    {
      for (int i = 0; i < ww; i ++) *aptr++ = background;
      aptr += myw - ww;
    }

  // bulk of embedded image:
  for (int j = jmin; j < jmax; j ++)
    {
      for (int i = 0; i < imin; i ++) *aptr++ = background;

      const float y = std::max(0.0F, float(j - jmin) * sh);
      const int y0 = int(y);
      const float fy = y - float(y0);
      const int y1 = std::min(y0 + 1, ih - 1);
      const int wy0 = iw * y0, wy1 = iw * y1;

      for (int i = imin; i < imax; i ++)
        {
          const float x = std::max(0.0F, float(i - imin) * sw);
          const int x0 = int(x);
          const float fx = x - float(x0);
          const int x1 = std::min(x0 + 1, iw - 1);

          typename promote_trait<T, float>::TP
            d00( sptr[x0 + wy0] ), d10( sptr[x1 + wy0] ),
            d01( sptr[x0 + wy1] ), d11( sptr[x1 + wy1] ),
            dx0( d00 + (d10 - d00) * fx ),
            dx1( d01 + (d11 - d01) * fx );

          *aptr++ = T( dx0 + (dx1 - dx0) * fy );  // no need to clamp
        }
      for (int i = imax; i < ww; i ++) *aptr++ = background;
      aptr += myw - ww;
    }

  // bottom empty lines:
  for (int j = jmax; j < hh; j ++)
    {
      for (int i = 0; i < ww; i ++) *aptr++ = background;
      aptr += myw - ww;
    }
}

// ######################################################################
// NOTE: can only have one mask in the image
template <class T>
Rectangle findBoundingRect(const Image<T>& src,
                           const T threshold)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized());

  int x1 = -1;  int y1 = -1; int x2 = -1; int y2 = -1;
  int w = src.getWidth(); int h = src.getHeight();

  // find x offset
  for(int i = 0; i < src.getWidth(); i++)
    for (int j = 0; j < src.getHeight(); j++)
      if (src.getVal(i,j) >= threshold){ x1 = i; i = w; break; }
  LDEBUG("x1 = %d", x1);

  // find y
  for(int j = 0; j < src.getHeight(); j++)
    for (int i = 0; i < src.getWidth(); i++)
      if (src.getVal(i,j) >= threshold) { y1 = j; j = h; break; }
  LDEBUG("y1 = %d", y1);

  // find width
  for(int i = src.getWidth()-1; i >= 0; i--)
    for (int j = 0; j < src.getHeight(); j++)
      if (src.getVal(i,j) >= threshold) { x2 = i; i =-1; break; }
  LDEBUG("x2 = %d", x2);

  // find height
  for(int j = src.getHeight()-1; j >= 0; j--)
    for (int i = 0; i < src.getWidth(); i++)
      if (src.getVal(i,j) >= threshold) { y2 = j; j =-1; break; }
  LDEBUG("y2 = %d", y2);

  return Rectangle::tlbrI(y1,x1,y2,x2);
}

// ######################################################################
Rectangle findBoundingRect(const std::vector<Point2D<int> >& poly, const Dims imgDims)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  Point2D<int> upperLeft = poly[0];
  Point2D<int> bottomRight = poly[0];

  for(uint i=0; i<poly.size(); i++)
  {
    if (poly[i].i < upperLeft.i)
      upperLeft.i = poly[i].i;

    if (poly[i].j < upperLeft.j)
      upperLeft.j = poly[i].j;

    if (poly[i].i > bottomRight.i)
      bottomRight.i = poly[i].i;

    if (poly[i].j > bottomRight.j)
      bottomRight.j = poly[i].j;
  }
  Dims size(bottomRight.i-upperLeft.i,
      bottomRight.j - upperLeft.j);

  int width = bottomRight.i-upperLeft.i;
  int height = bottomRight.j - upperLeft.j;

  if (imgDims.w() > 0 && imgDims.h() > 0)
  {
    //Fix any bounding problems
    if (upperLeft.i < 0) upperLeft.i = 0;
    if (upperLeft.j < 0) upperLeft.j = 0;
    if (upperLeft.i + width  > imgDims.w())
      width = imgDims.w() - upperLeft.i;
    if (upperLeft.j + height  > imgDims.h())
      height = imgDims.h() - upperLeft.j;
  }

  return Rectangle(upperLeft, Dims(width, height));
}


// Include the explicit instantiations (color instantiations are now
// requested by using "T_or_RGB" for the template formal parameter name in
// the declarations in the .H file).
#include "inst/Image/CutPaste.I"

template Image<double>
crop(const Image<double>&, const Point2D<int>&, const Dims&, bool);

template Image<PixRGB<unsigned short> >
crop(Image<PixRGB<unsigned short> > const&, Point2D<int> const&, Dims const&, bool);

template Image<unsigned short>
concatX(Image<unsigned short> const&, Image<unsigned short> const&);

template void
inplacePaste(Image<double>&, const Image<double>&, const Point2D<int>&);

template Image<int>
shiftClean(const Image<int>& srcImg, const int dx, const int dy,
           const int bgval);

template Image<double>
shiftClean(const Image<double>& srcImg, const int dx, const int dy,
           const double bgval);

template Image<geom::vec2f>
shiftClean(const Image<geom::vec2f>& srcImg, const int dx, const int dy,
           const geom::vec2f bgval);

template void
inplacePaste(Image<geom::vec2f>&, const Image<geom::vec2f>&, const Point2D<int>&);

template void inplaceEmbed(Image<uint16>&, const Image<uint16>&, const Rectangle&,
                           const uint16 background, const bool);

template Image<uint16> crop(const Image<uint16>&, const Rectangle&, const bool);
template Image<double> crop(const Image<double>&, const Rectangle&, const bool);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
