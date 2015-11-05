/*!@file Image/DrawOps.C functions for drawing on images
 */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; filed July 23, 2001, following provisional applications     //
// No. 60/274,674 filed March 8, 2001 and 60/288,724 filed May 4, 2001).//
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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/DrawOps.C $
// $Id: DrawOps.C 15383 2012-08-09 01:18:01Z kai $
//

#include <deque>
#include "Image/DrawOps.H"

#include "Image/ColorOps.H"    // for makeRGB()
#include "Image/ColorMap.H"
#include "Image/CutPaste.H"    // for concatX() etc.
#include "Image/FilterOps.H"   // for lowPass9()
#include "Image/Image.H"
#include "Image/MathOps.H"     // for inplaceNormalize() etc.
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "Image/Range.H"
#include "Image/ShapeOps.H"    // for rescale() etc.
#include "Image/Transforms.H"  // for chamfer34() etc.
#include "Image/MatrixOps.H" //for transpose
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/sformat.H"
#include "rutz/trace.h"
#include <cmath>
// ######################################################################
void drawTraj(Image< PixRGB<byte> >& img,
              const Point2D<int>* foa, const Point2D<int>* foas_end)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const Point2D<int>* foa_prev = foa;
  for (; foa != foas_end; ++foa)
    {
      drawCircle(img, *foa, 10, PixRGB<byte>(255,255,0), 1);
      if (foa != foa_prev)
        drawArrow(img, *foa_prev, *foa, PixRGB<byte>(255,0,0), 1);
      foa_prev = foa;
    }
}

// ######################################################################
Image< PixRGB<byte> > colGreyCombo(const Image< PixRGB<byte> >& colimg,
                                   const Image<float>& bwimg,
                                   const bool xcombo, const bool interp)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(colimg.initialized());

  // will convert and clamp to byte value range as necessary:
  const Image<byte> tmp =
    bwimg.initialized()
    ? Image<byte>(rescaleOpt(bwimg, colimg.getDims(), interp))
    : Image<byte>(colimg.getDims(), ZEROS);

  const Image< PixRGB<byte> > tmp2 = makeRGB(tmp, tmp, tmp);

  if (xcombo) return concatX(colimg, tmp2);
  else        return concatY(colimg, tmp2);
}

// ######################################################################
Image< PixRGB<byte> > colColCombo(const Image< PixRGB<byte> >& colimg1,
                                   const Image< PixRGB<byte> >& colimg2,
                                   const bool xcombo, const bool interp)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(colimg1.initialized());
  ASSERT(colimg2.initialized());

  Image< PixRGB<byte> > tmp;
  if (xcombo)
    {
      if (colimg1.getHeight() != colimg2.getHeight())
        tmp = rescaleOpt(colimg2, colimg1.getWidth() / 2,
                      colimg1.getHeight(), interp);
      else tmp = colimg2;
      return concatX(colimg1, tmp);
    }
  else
    {
      if (colimg1.getWidth() != colimg2.getWidth())
        tmp = rescaleOpt(colimg2, colimg1.getWidth(),
                      colimg1.getHeight() / 2, interp);
      else tmp = colimg2;
      return concatY(colimg1, tmp);
    }
}

// ######################################################################
Image< PixRGB<byte> > highlightRegions(const Image< PixRGB<byte> >& img,
                                       const Image<byte>& mask,
                                       const int maxval)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> tmp = chamfer34(mask);
  tmp = binaryReverse(tmp, float(maxval));
  tmp.setVal(0, 0, 0.0);
  inplaceNormalize(tmp, 0.0f, 1.0f);
  return img * tmp;
}

// ######################################################################
Image< PixRGB<byte> > warp3Dmap(const Image< PixRGB<byte> >& img,
                                const Image<float>& hmap,
                                const float pitch, const float yaw,
                                Dims& imdims)
{
GVX_TRACE(__PRETTY_FUNCTION__);
    // smooth linear interpolation of the height map:
    Image<float> ftmp = lowPass9(rescale(hmap,img.getWidth(),img.getHeight()));
    inplaceClamp(ftmp, 0.0f, 255.0f);

    // first call to this fct will compute size of output:
    return warp3D(img, ftmp, pitch, yaw, 255.0, imdims);
}

// ######################################################################
void inplaceSetValMask(Image<float>& dest,
                       const Image<byte>& mask, const float val)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dest.isSameSize(mask));

  Image<byte>::const_iterator mptr = mask.begin();

  Image<float>::iterator
    dptr = dest.beginw(),
    endptr = dest.endw();

  while (dptr != endptr)
    {
      if (*mptr) *dptr = val;
      ++mptr;
      ++dptr;
    }
}

// ######################################################################
template <class T>
void drawDisk(Image<T>& dst, const Point2D<int>& center,
              const int radius, const T value, const float alpha)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  if (radius == 1)
    {
      if (dst.coordsOk(center)) dst.setVal(center.i + dst.getWidth() * center.j, value);
      return;
    }

  for (int y = -radius; y <= radius; ++y)
    {
      int bound = int(sqrtf(float(squareOf(radius) - squareOf(y))));
      for (int x = -bound; x <= bound; ++x)
        if (dst.coordsOk(x + center.i, y + center.j))
				{
					if(alpha >= 1.0|| alpha < 0.0)
						dst.setVal(x + center.i, y + center.j, value);
					else
					{
						T bgColor = dst.getVal( x + center.i,y + center.j);
						T color = T(value*alpha + bgColor*(1.0-alpha));
						dst.setVal(x + center.i, y + center.j, color);

					
					}
				}
    }
}

// ######################################################################
template <class T>
void drawHalfDisk(Image<T>& dst, const Point2D<int>& center,
              const int radius, const T value, const float ori)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  if (radius == 1)
    {
      if (dst.coordsOk(center)) dst.setVal(center.i + dst.getWidth() * center.j, value);
      return;
    }
	float cost = cos(ori);
	float sint = sin(ori);
	int rotx,roty;
  for (int y = 0; y <= radius; ++y)
    {
      int bound = int(sqrtf(float(squareOf(radius) - squareOf(y))));
			for (int x = -bound; x <= bound; ++x)
			{
				rotx = int(x*cost - y*sint);
				roty = int(x*sint + y*cost);
				if (dst.coordsOk(rotx + center.i, roty + center.j))
					dst.setVal(rotx + center.i, roty + center.j, value);
			}
    }
}

// ######################################################################
template <class T>
void drawLine(Image<T>& dst,
    const Point2D<int>& pos, float ori, float len, const T col,
    const int rad = 1)
{

GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());

  int x1 = int(cos(ori)*len/2);
  int y1 = int(sin(ori)*len/2);

  Point2D<int> p1(pos.i-x1, pos.j+y1);
  Point2D<int> p2(pos.i+x1, pos.j-y1);

  drawLine(dst, p1, p2, col, rad);

}


// ######################################################################
template <class T>
void drawLine(Image<T>& dst, const Point2D<int>& p1, const Point2D<int>& p2,
              const T col, const int rad, const float alpha)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  // from Graphics Gems / Paul Heckbert
  int dx = p2.i - p1.i, ax = abs(dx) << 1, sx = signOf(dx);
  int dy = p2.j - p1.j, ay = abs(dy) << 1, sy = signOf(dy);
  int x = p1.i, y = p1.j;

  const int w = dst.getWidth();
  const int h = dst.getHeight();

  T* const dptr = dst.getArrayPtr();

  if (ax > ay)
    {
      int d = ay - (ax >> 1);
      for (;;)
        {
          if (rad == 1)
            {
              // avoid a function call to drawDisk() if rad==1
              if (x >= 0 && x < w && y >= 0 && y < h)
                dptr[x + w * y] = col;
            }
          else
            drawDisk(dst, Point2D<int>(x,y), rad, col,alpha);
          if (x == p2.i) return;
          if (d >= 0) { y += sy; d -= ax; }
          x += sx; d += ay;
        }
    }
  else
    {
      int d = ax - (ay >> 1);
      for (;;)
        {
          if (rad == 1)
            {
              // avoid a function call to drawDisk() if rad==1
              if (x >= 0 && x < w && y >= 0 && y < h)
                dptr[x + w * y] = col;
            }
          else
            drawDisk(dst, Point2D<int>(x,y), rad, col,alpha);
          if (y == p2.j) return;
          if (d >= 0) { x += sx; d -= ay; }
          y += sy; d += ax;
        }
    }
}

// ######################################################################
template <class T>
void drawCorner(Image<T>& dst,
    const Point2D<int>& pos, float ori, float ang, float len, const T col,
    const int rad = 1)
{

GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());

  int x1 = int(cos(ori - ang/2)*len/2);
  int y1 = int(sin(ori - ang/2)*len/2);

  int x2 = int(cos(ori + ang/2)*len/2);
  int y2 = int(sin(ori + ang/2)*len/2);

  Point2D<int> p1(pos.i, pos.j);
  Point2D<int> p2(pos.i+x1, pos.j-y1);
  Point2D<int> p3(pos.i+x2, pos.j-y2);

  drawLine(dst, p1, p2, col, rad);
  drawLine(dst, p1, p3, col, rad);

}

// ######################################################################
template <class T>
void drawRect(Image<T>& dst,
              const Rectangle& r, const T color, const int rad)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  ASSERT(r.isValid());
  ASSERT(dst.rectangleOk(r));

  Point2D<int> p1(r.left(), r.top());
  Point2D<int> p2(r.rightI(), r.top());
  Point2D<int> p3(r.left(), r.bottomI());
  Point2D<int> p4(r.rightI(), r.bottomI());

  drawLine(dst, p1, p2, color, rad);
  drawLine(dst, p1, p3, color, rad);
  drawLine(dst, p3, p4, color, rad);
  drawLine(dst, p2, p4, color, rad);
}

// ######################################################################
template <class T>
void drawRectSquareCorners(Image<T>& dst, const Rectangle& r1,
                           const T color, const int linewidth)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  ASSERT(r1.isValid());

  const Rectangle r = r1.getOverlap(dst.getBounds());

  ASSERT(dst.rectangleOk(r));

  const int minrad = (linewidth - 1) / 2;

  for (int i = 0; i < linewidth; ++i)
    {
      const int offset = i - minrad;

      const Point2D<int> p1(r.left() + offset, r.top() + offset);
      const Point2D<int> p2(r.rightI() - offset, r.top() + offset);
      const Point2D<int> p3(r.left() + offset, r.bottomI() - offset);
      const Point2D<int> p4(r.rightI() - offset, r.bottomI() - offset);

      drawLine(dst, p1, p2, color, 1);
      drawLine(dst, p1, p3, color, 1);
      drawLine(dst, p3, p4, color, 1);
      drawLine(dst, p2, p4, color, 1);
    }
}

// ######################################################################
template <class T>
void drawRectOR(Image<T>& dst,
              const Rectangle& r, const T color, const int rad,
              const float ori)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  //ASSERT(dst.rectangleOk(r));

  // compute the center of the rect
  const float centerX = r.rightI()  - r.left();
  const float centerY = r.bottomI() - r.top();

  // compute the distance from the center to the corners
  //const float distX  = r.rightI()  - centerX;
  //const float distY  = r.bottomI() - centerY;
  const float distX  = centerX/2;
  const float distY  = centerY/2;
  const float radius = sqrt(pow(distX,2) + pow(distY,2));

  // compute the transformed coordinates
  const float thetaO  = atan(distX/distY);
  const float thetaN1 = thetaO + ori;
  const float thetaN2 = ori - thetaO;
  const float Xnew1   = radius * sin(thetaN1);
  const float Ynew1   = radius * cos(thetaN1);
  const float Xnew2   = radius * sin(thetaN2);
  const float Ynew2   = radius * cos(thetaN2);

  // compute new coords based on rotation
  const float Xbr    = r.left() + distX + Xnew1; // bottom-right
  const float Ybr    = r.top()  + distY + Ynew1;
  const float Xtl    = r.left() + distX - Xnew1; // top-left
  const float Ytl    = r.top()  + distY - Ynew1;
  const float Xbl    = r.left() + distX + Xnew2; // bottom-left
  const float Ybl    = r.top()  + distY + Ynew2;
  const float Xtr    = r.left() + distX - Xnew2; // top-right
  const float Ytr    = r.top()  + distY - Ynew2;

  // set up points
  Point2D<int> p1((int)round(Xbr), (int)round(Ybr));
  Point2D<int> p2((int)round(Xtl), (int)round(Ytl));
  Point2D<int> p3((int)round(Xbl), (int)round(Ybl));
  Point2D<int> p4((int)round(Xtr), (int)round(Ytr));

  // draw lines
  drawLine(dst, p1, p3, color, rad);
  drawLine(dst, p1, p4, color, rad);
  drawLine(dst, p2, p3, color, rad);
  drawLine(dst, p2, p4, color, rad);
}

// ######################################################################
template <class T>
void drawRectEZ(Image<T>& dst,
                const Rectangle& r, const T color, const int rad)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!r.isValid()) return;

  ASSERT(dst.initialized());

  if (dst.rectangleOk(r))
  {
    Point2D<int> p1(r.left(), r.top());
    Point2D<int> p2(r.rightI(), r.top());
    Point2D<int> p3(r.left(), r.bottomI());
    Point2D<int> p4(r.rightI(), r.bottomI());

    drawLine(dst, p1, p2, color, rad);
    drawLine(dst, p1, p3, color, rad);
    drawLine(dst, p3, p4, color, rad);
    drawLine(dst, p2, p4, color, rad);
  }
  else
  {
    const Rectangle r2 = r.getOverlap(dst.getBounds());

    int rl = r2.left();  int rt = r2.top();
    int rr = r2.rightI(); int rb = r2.bottomI();

    Point2D<int> p1(rl, rt); Point2D<int> p2(rr, rt);
    Point2D<int> p3(rl, rb); Point2D<int> p4(rr, rb);

    drawLine(dst, p1, p2, color, rad); drawLine(dst, p1, p3, color, rad);
    drawLine(dst, p3, p4, color, rad); drawLine(dst, p2, p4, color, rad);
  }

}

// ######################################################################
template <class T>
void drawFilledRect(Image<T>& dst, const Rectangle& r, const T val)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.rectangleOk(r));

  const int w = dst.getWidth();

  const int rw = r.width();
  const int rh = r.height();

  const int rowskip = w - rw;

  typename Image<T>::iterator dptr = dst.beginw() + r.left() + r.top() * w;

  for (int y = 0; y < rh; ++y)
    {
      for (int x = 0; x < rw; ++x)
        *dptr++ = val;
      dptr += rowskip;
    }
}

// ######################################################################
template <class T>
Image<T> drawHistogram(std::vector<float> hist, int width, int height, T lineVal, T fillVal)
{
  Image<T> ret = Image<T>(width,height,ZEROS);
  if(hist.size() == 0)
    return ret;
  float recWidth = width/float(hist.size()+1);
  float peak = *std::max_element(hist.begin(),hist.end());
  float recScale = height/float(peak);
  for(uint h=0; h< hist.size(); h++)
    {
      Dims d = Dims(floor(recWidth),std::max(int(floor(recScale*(hist[h]))),1));
      Point2D<int> p = Point2D<int>(floor(recWidth*h+recWidth/2.0),0);
      Rectangle rec = Rectangle(p,d);
      if(!ret.rectangleOk(rec))
        {
          LINFO("H[%d] failed with p %s and Dims %s",h,convertToString(p).c_str(),convertToString(d).c_str());
          LFATAL("Histogram rectangle invalid");
        }
      drawFilledRect(ret,rec,fillVal);
      drawRectSquareCorners(ret,rec,lineVal,1);
    }
  return ret;
}

// ######################################################################
template <class T>
void drawCross(Image<T>& dst,
               const Point2D<int>& p, const T col, const int siz,
               const int rad)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  Point2D<int> p1, p2;

  p1.i = clampValue(p.i - siz, 0, dst.getWidth() - 1);
  p1.j = clampValue(p.j, 0, dst.getHeight() - 1);
  p2.i = clampValue(p.i + siz, 0, dst.getWidth() - 1);
  p2.j = clampValue(p.j, 0, dst.getHeight() - 1);

  drawLine(dst, p1, p2, col, rad);

  p1.i = clampValue(p.i, 0, dst.getWidth() - 1);
  p1.j = clampValue(p.j - siz, 0, dst.getHeight() - 1);
  p2.i = clampValue(p.i, 0, dst.getWidth() - 1);
  p2.j = clampValue(p.j + siz, 0, dst.getHeight() - 1);

  drawLine(dst, p1, p2, col, rad);
}

// ######################################################################
template <class T>
void drawCrossOR(Image<T>& dst,
                 const Point2D<int>& p, const T col, const int siz,
                 const int rad, const float ori)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  Point2D<int> p1, p2;

  // compute new X and Y given rotation
  const float newX = siz * sin(ori);
  const float newY = siz * cos(ori);

  p1.i = clampValue(int(p.i - newY), 0, dst.getWidth() - 1);
  p1.j = clampValue(int(p.j + newX), 0, dst.getHeight() - 1);

  p2.i = clampValue(int(p.i + newY), 0, dst.getWidth() - 1);
  p2.j = clampValue(int(p.j - newX), 0, dst.getHeight() - 1);

  drawLine(dst, p1, p2, col, rad);

  p1.i = clampValue(int(p.i - newX), 0, dst.getWidth() - 1);
  p1.j = clampValue(int(p.j - newY), 0, dst.getHeight() - 1);

  p2.i = clampValue(int(p.i + newX), 0, dst.getWidth() - 1);
  p2.j = clampValue(int(p.j + newY), 0, dst.getHeight() - 1);

  drawLine(dst, p1, p2, col, rad);
}

// ######################################################################
template <class T>
void drawPatch(Image<T>& dst,
               const Point2D<int>& p, const int siz, const T col)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  for (int i = -siz; i <= siz; ++i)
    for (int j = -siz; j <= siz; ++j)
      {
        const int xx = clampValue(p.i + i, 0, dst.getWidth() - 1);
        const int yy = clampValue(p.j + j, 0, dst.getHeight() - 1);
        dst.setVal(xx, yy, col);
      }
}

// ######################################################################
template <class T>
void drawPatchBB(Image<T>& dst, const Point2D<int>& p, const int siz, const T col, const T bgcol)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());

  for (int i = -siz-2; i <= siz+2; ++i)
    for (int j = -siz-2; j <= siz+2; ++j)
      {
        const int xx = clampValue(p.i + i, 0, dst.getWidth() - 1);
        const int yy = clampValue(p.j + j, 0, dst.getHeight() - 1);
        if (i < -siz || i > siz || j < -siz || j > siz) dst.setVal(xx, yy, bgcol);
        else dst.setVal(xx, yy, col);
      }
}

// ######################################################################
template <class T>
void drawCircle(Image<T>& dst,
                const Point2D<int>& p, const int radius,
                const T col, const int rad)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  if (radius == 1)
    {
      if (dst.coordsOk(p)) dst.setVal(p.i, p.j, col);
      return;
    }

  Point2D<int> pp(p);
  pp.i -= radius; drawDisk(dst, pp, rad, col);
  pp.i += radius + radius; drawDisk(dst, pp, rad, col);
  int bound1 = radius, bound2 = radius;

  for (int y = 1; y <= radius; y ++)
    {
      bound2 = bound1;
      bound1 = int(sqrtf(float( squareOf(radius) - squareOf(y) )));
      for (int i = bound1; i <= bound2; i ++)
        {
          pp.j = p.j - y;
          pp.i = p.i - i; drawDisk(dst, pp, rad, col);
          pp.i = p.i + i; drawDisk(dst, pp, rad, col);
          pp.j = p.j + y; drawDisk(dst, pp, rad, col);
          pp.i = p.i - i; drawDisk(dst, pp, rad, col);
        }
    }
}

// ######################################################################
template <class T>
void drawArc(Image<T>& dst,
                const Point2D<int>& center, const int radius,
                const T col, 
								const float startAng,const float endAng, const int rad)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
	//compute radius
  if (radius == 1)
    {
      if (dst.coordsOk(center)) dst.setVal(center.i, center.j, col);
      return;
    }

	//Midpoint Circle algorithm
	//
	//      90
	//   45    135
	//
	// 0----+----180
	//
	//  315    225
	//     270
	int p = (5 - radius * 4) / 4;

	float ang90  = M_PI/2.0F;
	float ang180 = M_PI;
	float ang270 = ang180 + ang90;
	float ang360 = M_PI*2.0F;
	int cx = center.i,cy = center.j;
	int x = 0,y = radius;
	while(x <= y)
	{
		x++;
		if(p < 0){
			p += 2 * x + 1;
		}else{
			y--;
			p += 2*(x - y) + 1;
		}
		
		float angle = atan2(y,x);
		if(x < y)
		{
			// draw point in range 0 to 45 degrees
			if(ang90 - angle >= startAng && ang90 - angle <= endAng)
          drawDisk(dst, Point2D<int>(cx - y,cy - x), rad, col);					
			// draw point in range 45 to 90 degrees
			if( angle >= startAng && angle <= endAng)
          drawDisk(dst, Point2D<int>(cx - x,cy - y), rad, col);					
			// draw point in range 90 to 135 degrees
			if(ang180 - angle >= startAng && ang180 - angle <= endAng)
          drawDisk(dst, Point2D<int>(cx + x,cy - y), rad, col);					
			// draw point in range 135 to 180 degrees
			if(ang90 + angle >= startAng && ang90 + angle <= endAng)
          drawDisk(dst, Point2D<int>(cx + y,cy - x), rad, col);					
			// draw point in range 180 to 225 degrees
			if(ang270 - angle >= startAng && ang270 - angle <= endAng)
          drawDisk(dst, Point2D<int>(cx + y,cy + x), rad, col);					
			// draw point in range 225 to 270 degrees
			if(ang180 + angle >= startAng && ang180 + angle <= endAng)
          drawDisk(dst, Point2D<int>(cx + x,cy + y), rad, col);					
			// draw point in range 270 to 315 degrees
			if(ang360 - angle >= startAng && ang360 - angle <= endAng)
          drawDisk(dst, Point2D<int>(cx - x,cy + y), rad, col);					
			// draw point in range 315 to 360 degrees
			if(ang270 + angle >= startAng && ang270 + angle <= endAng)
          drawDisk(dst, Point2D<int>(cx - y,cy + x), rad, col);					

		}
	}

}

// ######################################################################
template <class T>
void drawEllipse(Image<T>& dst,
                const Point2D<int>& p, const int radiusx, const int radiusy,
                const T col, const int rad)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());

  int steps = 36;
  double a = double(M_PI*2.0)/double(steps);
  double sa = (double)sin(a);
  double ca = (double)cos(a);

  double ny=1;
  double nz=0;
  Point2D<int> lastPoint(p.i + radiusx, p.j);
  for(int i=1; i<=steps; i++)
  {
    double tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
    Point2D<int> newPoint(p.i+(ny*radiusx),p.j+(nz*radiusy));
    drawLine(dst, lastPoint, newPoint, col, rad);
    lastPoint = newPoint;
  }


}

// ######################################################################
template <class T_or_RGB>
void drawSuperquadric(Image<T_or_RGB>& dst,
                const Point2D<int>& p,
                const float a, const float b, const float e, const T_or_RGB col,
                const float rot, const float k1, const float k2,
                const float thetai,
                const float thetaf,
                const int rad,
                const int nSeg)

{
  const float dTheta = (thetaf-thetai) / (float)nSeg;

  for (float theta=thetai; theta < thetaf; theta += dTheta)
  {
    Point2D<float> p1 = ellipsoid(a,b, e, theta);
    Point2D<float> p2 = ellipsoid(a,b, e, theta + dTheta);

    Point2D<float> tmpPos1;
    Point2D<float> tmpPos2;
    //Sheer
    tmpPos1.i = p1.i + p1.j*k1;
    tmpPos1.j = p1.i*k2 + p1.j;

    tmpPos2.i = p2.i + p2.j*k1;
    tmpPos2.j = p2.i*k2 + p2.j;

    //Rotate and move to p
    p1.i = (cos(rot)*tmpPos1.i - sin(rot)*tmpPos1.j) + p.i;
    p1.j = (sin(rot)*tmpPos1.i + cos(rot)*tmpPos1.j) + p.j;

    p2.i = (cos(rot)*tmpPos2.i - sin(rot)*tmpPos2.j) + p.i;
    p2.j = (sin(rot)*tmpPos2.i + cos(rot)*tmpPos2.j) + p.j;

    drawLine(dst, (Point2D<int>)p1, (Point2D<int>)p2, col, rad);
  }

}


// ######################################################################
template <class T>
void drawArrow(Image<T>& dst,
               const Point2D<int>& p1, const Point2D<int>& p2,
               const T col, const int rad, const int len)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // first draw a line connecting p1 to p2:
  drawLine(dst, p1, p2, col, rad);

  // second, draw an arrow:
  Point2D<int> pm; pm.i = (p1.i + p2.i) / 2; pm.j = (p1.j + p2.j) / 2;

  // arrow size
  float norm = len;
  if(len == -1)
    norm = float(std::max(dst.getWidth(), dst.getHeight())) / 30.0F; 

  float angle = atan2((float)(p2.j - p1.j), (float)(p2.i - p1.i));
  float arrow_angle = 20.0F * M_PI / 180.0F;

  Point2D<int> pp;
  pp.i = pm.i - int(norm * cos(angle + arrow_angle));
  pp.j = pm.j - int(norm * sin(angle + arrow_angle));
  drawLine(dst, pm, pp, col, rad);

  pp.i = pm.i - int(norm * cos(angle - arrow_angle));
  pp.j = pm.j - int(norm * sin(angle - arrow_angle));
  drawLine(dst, pm, pp, col, rad);
}

// ######################################################################
template <class T>
void drawGrid(Image<T>& dst,
              const int spacingX, const int spacingY,
              const int thickX, const int thickY,
              const T col)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  for (int i = thickX / 2; i < dst.getWidth() - thickX / 2; i += spacingX)
    for (int j = 0; j < dst.getHeight(); j ++)
      for (int t = -thickX / 2; t <= thickX / 2; t ++)
        dst.setVal(i + t, j, col);

  for (int j = thickY / 2; j < dst.getHeight() - thickY / 2; j += spacingY)
    for (int i = 0; i < dst.getWidth(); i ++)
      for (int t = -thickY / 2; t <= thickY / 2; t ++)
        dst.setVal(i, t + j, col);
}

// ######################################################################
template <class T, class TT>
void drawContour2D(const Image<T>& src, Image<TT>& dst, const TT& col, const byte rad)
{
  // this code should be in sync with contour2D() in Image/Transforms.C
  ASSERT(src.initialized());
  ASSERT(dst.initialized());
  ASSERT(src.isSameSize(dst));

  typename Image<T>::const_iterator sptr = src.begin();
  T z = T(); const int h = src.getHeight(), w = src.getWidth();

  // This is a 4-connected contour algorithm. We are on the contour if we are not zero but at least: either 1) one of
  // our 4-neighbors is zero, or 2) we are on an edge of the image. Here we unfold the algo for max speed, i.e., we
  // treat the borders of the image explicitly and then the bulk:
  Point2D<int> p(0, 0);

  // first row:
  for (p.i = 0; p.i < w; ++p.i) if (*sptr++) drawDisk(dst, p, rad, col);

  // done if the image only has 1 row:
  if (h == 1) return;

  // bulk, will only run if image has >2 rows:
  for (p.j = 1; p.j < h-1; ++p.j)
    {
      // leftmost pixel of current row:
      p.i = 0; if (*sptr++) drawDisk(dst, p, rad, col);

      // done if image has only 1 column:
      if (w == 1) continue;

      // bulk of current row, will run only if image has >2 columns:
      for (p.i = 1; p.i < w-1; ++p.i) {
        if (*sptr && (sptr[-1] == z || sptr[1] == z || sptr[-w] == z || sptr[w] == z)) drawDisk(dst, p, rad, col);
        ++sptr;
      }

      // rightmost pixel of current row, we know it exists since image has at least 2 columns:
      p.i = w-1; if (*sptr++) drawDisk(dst, p, rad, col);
    }

  // last row (we know it exists since image has at least 2 rows):
  p.j = h-1; for (p.i = 0; p.i < w; ++p.i) if (*sptr++) drawDisk(dst, p, rad, col);
}

// ######################################################################
template <class T>
void drawGrid(Image<T>& dst, const uint nx, const uint ny, const int thick, const T col)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());

  const float wx = float(dst.getWidth()) / float(nx), hy = float(dst.getHeight()) / float(ny);
  for (uint j = 0; j < ny; ++j)
    {
      const int py1 = int(hy*(j)+0.49999F);
      const int py2 = int(hy*(j+1)+0.49999F);
      for (uint i = 0; i < nx; ++i)
        {
          const int px1 = int(wx*(i)+0.49999F);
          const int px2 = int(wx*(i+1)+0.49999F);
          drawLine(dst, Point2D<int>(px1, py2-1), Point2D<int>(px2-1, py2-1), col, thick);
          drawLine(dst, Point2D<int>(px2-1, py1), Point2D<int>(px2-1, py2-1), col, thick);
      }
    }
  drawLine(dst, Point2D<int>(0, 0), Point2D<int>(dst.getWidth()-1, 0), col, thick);
  drawLine(dst, Point2D<int>(0, 0), Point2D<int>(0, dst.getHeight()-1), col, thick);
}

// ######################################################################
template <class T>
void writeText(Image<T>& dst,
               const Point2D<int>& pt, const char* text,
               const T col, const T bgcol, const SimpleFont& f,
               const bool transparent_bg,
               const TextAnchor anchor)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const int textwidth = strlen(text) * f.w();
  const int textheight = f.h();

  const Point2D<int> top_left
    =    anchor == ANCHOR_BOTTOM_RIGHT ?    pt - Point2D<int>(textwidth, textheight)
    :    anchor == ANCHOR_BOTTOM_LEFT  ?    pt - Point2D<int>(0, textheight)
    :    anchor == ANCHOR_TOP_RIGHT    ?    pt - Point2D<int>(textwidth, 0)
    :    anchor == ANCHOR_CENTER			 ?    pt - Point2D<int>(textwidth/2, textheight/2)
    : /* anchor == ANCHOR_TOP_LEFT     ? */ pt;

  Point2D<int> p = top_left; // copy for modif
  const int ww = dst.getWidth(), hh = dst.getHeight();
  const int len = int(strlen(text));

  for (int i = 0; i < len; i ++)
    {
      const unsigned char *ptr = f.charptr(text[i]);

      for (int y = 0; y < int(f.h()); y ++)
        for (int x = 0; x < int(f.w()); x ++)
          if (p.i + x >= 0 && p.i + x < ww && p.j + y >= 0 && p.j + y < hh)
            {
              if (!ptr[y * f.w() + x])
                dst.setVal(p.i + x, p.j + y, col);
              else if (!transparent_bg)
                dst.setVal(p.i + x, p.j + y, bgcol);
            }
      p.i += f.w();
    }
}

// ######################################################################
template <class T>
Image<T> makeMultilineTextBox(const int w,
                              const std::string* lines,
                              const size_t nlines,
                              const T col,
                              const T bg,
                              const size_t max_chars_per_line_hint,
                              const int fontwidth)
{
  size_t maxchars = 0;

  if (max_chars_per_line_hint > 0)
    maxchars = max_chars_per_line_hint;
  else
    for (size_t i = 0; i < nlines; ++i)
      if (lines[i].length() > maxchars)
        maxchars = lines[i].length();

  if (maxchars > 0)
    {
      const SimpleFont font =
        SimpleFont::fixedMaxWidth(fontwidth ? fontwidth : size_t(w)/maxchars);

      Image<T> textarea(w, 4 + nlines*(font.h()+2), ZEROS);

      for (size_t i = 0; i < nlines; ++i)
        writeText(textarea, Point2D<int>(1,1+i*(font.h()+2)),
                  lines[i].c_str(),
                  col, bg,
                  font);

      return textarea;
    }

  return Image<T>();
}

// ######################################################################
template <class T>
void drawPoint(Image<T>& dst,
               int X,int Y,T pix)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  dst.setVal(X,Y,pix);
  if(X > 0) dst.setVal((X-1),Y,pix);
  if(Y > 0) dst.setVal(X,(Y-1),pix);
  if(X < (dst.getWidth()-1)) dst.setVal((X+1),Y,pix);
  if(Y < (dst.getHeight()-1)) dst.setVal(X,(Y+1),pix);
}

// ######################################################################
template <class T>
int drawDiskCheckTarget(Image<T>& dst,
                        Image<T>& mask,
                        const Point2D<int>& center,
                        const int radius,
                        const T value,
                        const T targetvalue,
                        const T floodvalue)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
  ASSERT(floodvalue < targetvalue);
  int nbhit = 0; Point2D<int> hitpt;
  for (int y = -radius; y <= radius; y ++)
    {
      int bound = (int)(sqrt( (double)( squareOf(radius) - squareOf(y) ) ));
      for (int x = -bound; x <= bound; x ++)
        if (dst.coordsOk(x + center.i, y + center.j))
          {
            dst.setVal(x + center.i, y + center.j, value);
            if (mask.getVal(x + center.i, y + center.j) == targetvalue)
              {
                nbhit ++;
                hitpt.i = x + center.i, hitpt.j = y + center.j;
                // flood the hit target:
                Image<T> tmp = mask;
                flood(tmp, mask, hitpt, targetvalue, floodvalue);
              }
          }
    }
  return nbhit;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > warp3D(const Image<PixRGB<T> >& ima,
                         const Image<float>& zmap,
                         const float pitch, const float yaw,
                         const float zMax, Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(ima.isSameSize(zmap));

  float cp = float(cos(double(pitch) * M_PI / 180.0));
  float sp = float(sin(double(pitch) * M_PI / 180.0));
  float cy = float(cos(double(yaw) * M_PI / 180.0));
  float sy = float(sin(double(yaw) * M_PI / 180.0));
  int iw = ima.getWidth(), ih = ima.getHeight();

  // compute coords of bounding box to determine 3D image size
  int xmin = 0, zmin = 0, xmax = 0, zmax = 0, xx, zz;
  // (0, 0, 0) is implicitly done: yields (0,0)
  // (0, 0, zMax)
  zz = int(-cp * zMax);
  zmin = std::min(zmin, zz);
  zmax = std::max(zmax, zz);
  // (0,h,0)
  xx = int(sy * ih);
  xmin = std::min(xmin, xx);
  xmax = std::max(xmax, xx);
  zz = int(- cy * sp * ih);
  zmin = std::min(zmin, zz);
  zmax = std::max(zmax, zz);
  // (0, h, zMax)
  zz = int(- cy * sp * ih - cp * zMax);
  zmin = std::min(zmin, zz);
  zmax = std::max(zmax, zz);
  // (w, 0, 0)
  xx = int(cy * iw);
  xmin = std::min(xmin, xx);
  xmax = std::max(xmax, xx);
  zz = int(sy * sp * iw);
  zmin = std::min(zmin, zz);
  zmax = std::max(zmax, zz);
  // (w, 0, zMax)
  zz = int(sy * sp * iw - cp * zMax);
  zmin = std::min(zmin, zz);
  zmax = std::max(zmax, zz);
  // (w, h, 0)
  xx = int(cy * iw + sy * ih);
  xmin = std::min(xmin, xx);
  xmax = std::max(xmax, xx);
  zz = int(sy * sp * iw - cy * sp * ih);
  zmin = std::min(zmin, zz);
  zmax = std::max(zmax, zz);
  // (w, h, zMax)
  zz = int(sy * sp * iw - cy * sp * ih - cp * zMax);
  zmin = std::min(zmin, zz);
  zmax = std::max(zmax, zz);

  int nw, nh;
  if (dims.isEmpty())  // need to compute image size
    {
      nw = ((xmax - xmin) * 12) / 10;
      nh = ((zmax - zmin) * 16) / 10;  // height is oversized: place_image_3D
      //LDEBUG("image Width=%d, Height=%d", nw, nh);
      dims = Dims(nw, nh);
    }
  else  // image size known; just place the 3D in it
    {
      nw = dims.w(); nh = dims.h();
    }
  Image<PixRGB<T> > result(nw, nh, ZEROS);  // set background to black

  int xoff = -xmin + nw / 10;  // center image
  int zoff = -zmin + nh / 10;

  // store previously drawn points, for gap filling
  int *prev_zz = new int[nw]; memset(prev_zz, 0, nw * sizeof(int));
  int *prev_xx = new int[nw]; memset(prev_xx, 0, nw * sizeof(int));
  int ppzz = 0, ppxx = 0;
  for (int j = 0; j < ih; j ++)
    for (int i = 0; i < iw; i ++)
      {
        float x = float(i);
        float y = float(j);
        float z = - zmap.getVal(i, j);   // z-axis pointing downwards

        // rotate and project
        xx = xoff + int(cy * x + sy * y);
        zz = zoff + int(sy * sp * x - cy * sp * y + cp * z);
        xx = clampValue(xx, 0, nw-1);
        zz = clampValue(zz, 0, nh-1);

        // check if some gap-filling is needed due to high slopes
        PixRGB<T> pix, col;
        bool drawn = false;
        ima.getVal(i, j, col);  // color of current pixel
        if (j > 0 && zz > prev_zz[i] + 1)  // we are much lower than (i, j-1)
          {
            result.getVal(prev_xx[i], prev_zz[i], pix);  // save this pixel
            drawLine(result, Point2D<int>(xx, zz),
                     Point2D<int>(prev_xx[i], prev_zz[i]),
                     col);
            result.setVal(prev_xx[i], prev_zz[i], pix);
            drawn = true;
          }
        if (i > 0 && j > 0 && zz > ppzz + 1) // much lower than (i-1, j-1)
          {
            result.getVal(ppxx, ppzz, pix);  // save this pixel
            drawLine(result, Point2D<int>(xx, zz),
                     Point2D<int>(ppxx, ppzz),
                     col);
            result.setVal(ppxx, ppzz, pix);
            drawn = true;
          }
        if (i > 0 && (zz > prev_zz[i-1] + 1||   // much lower than (i-1, j)
                      zz < prev_zz[i-1] - 1))    // much higher than (i-1, j)
          {
            result.getVal(prev_xx[i-1], prev_zz[i-1], pix);  // save this pixel
            drawLine(result, Point2D<int>(xx, zz),
                     Point2D<int>(prev_xx[i-1], prev_zz[i-1]),
                     col);
            result.setVal(prev_xx[i-1], prev_zz[i-1], pix);
            drawn = true;
          }
        if (drawn == false)     // no line to draw, only one point
          result.setVal(xx, zz, col);

        ppzz = prev_zz[i]; ppxx = prev_xx[i];
        prev_zz[i] = zz; prev_xx[i] = xx;
      }
  delete [] prev_zz; delete [] prev_xx;

  return result;
}

// ######################################################################
template <class T>
void drawStar(Image<T>& dst,
               const Point2D<int>& p, const T col, const int siz,
               const int rad)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dst.initialized());
	
	double sqrt3 = 1.73205;
	double s100 = siz;
	double s50 = s100/2.0;
	double s50Dsq3 = s50 / sqrt3;
  double s50Tsq3 = s50 * sqrt3;
  double s100Dsq3 = s100/sqrt3;

	/*
	//              S0
	//              /\
	//             /  \
	//     S2_____/    \_____ S10 
	//       \	  S1  S11   /
	//        \            /
	//        /S3   O    S9\
	//     S4/___S5   S7____\ S8
	//            \    /
	//             \  /
	//              \/
	//              S6
	*/
	Point2D<int> s0(0				 ,-s100) ;//tip
	Point2D<int> s1(-s50Dsq3 , -s50) ;
	Point2D<int> s2(-s50Tsq3 , -s50) ;
	Point2D<int> s3(-s100Dsq3,	  0) ;
	Point2D<int> s4(-s50Tsq3 ,	s50) ;
	Point2D<int> s5(-s50Dsq3 ,	s50) ;
	Point2D<int> s6(0				 , s100) ;//tip
	Point2D<int> s7(s50Dsq3	 ,	s50) ;//s5
	Point2D<int> s8(s50Tsq3	 ,	s50) ;//s4
	Point2D<int> s9(s100Dsq3 ,		0) ;//s3
	Point2D<int> s10(s50Tsq3 , -s50) ;//s2
	Point2D<int> s11(s50Dsq3 , -s50) ;//s1

	std::vector<Point2D<int> > polygon;
	polygon.push_back(s0 );
	polygon.push_back(s1 );
	polygon.push_back(s2 );
	polygon.push_back(s3 );
	polygon.push_back(s4 );
	polygon.push_back(s5 );
	polygon.push_back(s6 );
	polygon.push_back(s7 );
	polygon.push_back(s8 );
	polygon.push_back(s9 );
	polygon.push_back(s10);
	polygon.push_back(s11);
	polygon.push_back(s0 );

	drawOutlinedPolygon(dst,polygon,col,p,0,1.0,0,0,rad);	
}
// ######################################################################
template <class T>
void drawStar2(Image<T>& dst,
		const Point2D<int>& p, const T col, const int siz,
		const int rad)
{
	GVX_TRACE(__PRETTY_FUNCTION__);
	ASSERT(dst.initialized());

	double ang36 = M_PI / 5.0;   // 36° x PI/180
	double ang72 = 2.0 * ang36;  // 72° x PI/180
	float sin36 = sin(ang36);
	float sin72 = sin(ang72);
	float cos36 = cos(ang36);
	float cos72 = cos(ang72);

	Point2D<int> s0(0,-siz);
	Point2D<int> s2(siz*sin72,-siz*cos72);//0:12
	Point2D<int> s4(siz*sin36, siz*cos36);//0:24
	Point2D<int> s6(-s4.i,s4.j);//mirror of s4
	Point2D<int> s8(-s2.i,s2.j);//mirror of s2

	drawLine(dst, s0 + p, s4 + p, col , rad);
	drawLine(dst, s4 + p, s8 + p, col , rad);
	drawLine(dst, s8 + p, s2 + p, col , rad);
	drawLine(dst, s2 + p, s6 + p, col , rad);
	drawLine(dst, s6 + p, s0 + p, col , rad);

}
// ######################################################################
Image< PixRGB<byte> >
formatMapForDisplay(const Image<float>& img, const float factor,
                    const Dims& newdims, const bool useInterp,
                    const ColorMap& cmap, const bool showColorScale,
                    const char *label)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<float> ftmp = img;

  // if the image is empty, make it black:
  if (ftmp.initialized() == false) ftmp.resize(newdims, true);

  // start by normalizing the map values:
  float omi, oma;
  if (factor != 0.0F) { getMinMax(ftmp, omi, oma); ftmp *= factor; }
  else inplaceNormalize(ftmp, 0.0F, 255.0F, omi, oma);

  // convert to byte, which may clamp out-of-range values:
  Image<byte> btmp = ftmp;

  // colorize using the desired colormap:
  Image< PixRGB<byte> > ctmp = colorize(btmp, cmap);

  // rescale the image to the desired new dims:
  ctmp = rescaleOpt(ctmp, newdims, useInterp);
  const uint w = ctmp.getWidth(), h = ctmp.getHeight();
  const SimpleFont f = SimpleFont::FIXED(6);
  int ty = h - f.h() - 1;

  // draw a border:
  drawRect(ctmp, Rectangle::tlbrI(0, 0, h-1, w-1), PixRGB<byte>(64, 64, 255), 1);

  // display color scale if desired:
  if (showColorScale)
    {
      const float fac = float(cmap.getWidth()) / float(w);
      for (uint i = 0; i < w; i ++)
        drawLine(ctmp, Point2D<int>(i, h-4), Point2D<int>(i, h-1), cmap.getVal(int(i * fac)), 1);
      ty -= 4;
    }

  // Write label using large font, if we have one:
  if (label) writeText(ctmp, Point2D<int>(2,2), label, PixRGB<byte>(128,255,128),
                       PixRGB<byte>(0), SimpleFont::FIXED(10), true);

  // write original value range using tiny font:
  std::string smin = sformat("%0.02g", omi);
  std::string smax = sformat("%0.02g", oma);

  // if we used a factor, clamping may have occurred, hence let's
  // indicate that by adding "<" and ">" before the range values:
  if (factor)
    {
      smin = std::string("<") + smin;
      smax = std::string(">") + smax;
    }

  // add space before and after to avoid clutter:
  smin = std::string(" ") + smin + std::string(" ");
  smax = std::string(" ") + smax + std::string(" ");

  // do we have room to write all that?
  if (w >= f.w() * (smin.length() + smax.length()))
    {
      const PixRGB<byte> tcol(255, 255, 128);
      const PixRGB<byte> bcol(1);

      writeText(ctmp, Point2D<int>(1, ty), smin.c_str(), tcol, bcol, f);
      writeText(ctmp, Point2D<int>(w - smax.length()*f.w() - 1, ty), smax.c_str(), tcol, bcol, f);
    }

  return ctmp;
}

// ###############################################################
template <typename T, class U>
Image<PixRGB<byte> > linePlot(const U& points, const int w,const int h, 
                              const T& minVal, const T& maxVal,
                              const char *title,
                              const char *ylabel,
                              const char *xlabel,
                              const PixRGB<byte>& linecol,
                              const PixRGB<byte>& bckcol,
                              const int numticks,
                              const bool axisonright)
{


  const int vecLength = points.size();
  T maxv(maxVal), minv(minVal);

  //find our min and max range
  if (minVal == 0 && maxVal == 0)
    {
      typename U::const_iterator iter = points.begin();
      while (iter != points.end())
        {
          if (*iter > maxv) 
            maxv = *iter;
          if (*iter < minv) 
            minv = *iter;
          ++iter;
        }
    }
  //fonts
  SimpleFont sf = SimpleFont::fixedMaxHeight(uint(0.125F*h)); //axis labels 
  SimpleFont sn = SimpleFont::fixedMaxHeight(uint(0.075F*h)); //tick labels
  std::string ymaxstr = sformat("%1.2f",(float)maxv);
  std::string yminstr = sformat("%1.2f",(float)minv);
  
  //border between image corner and y-axis
  int labelsize = ymaxstr.size() > yminstr.size() ? ymaxstr.size() : yminstr.size();
  
  //make our border has room for at least two sig figs, and four
  //positions for value and sign
  if (labelsize < 7)
    labelsize = 7;

  int hborder = (int)(labelsize * sn.w() + sf.w() * 1.5F);
  int vborder = (int)(sn.h() + sf.h() * 1.5F);

  //line thickness 
  const int thickness =  (h / 90) > 0 ? (h / 90) : 1;

  //create image with opposite axis as we will transpose after we draw
  //the y axis
  Image<PixRGB<byte> > result(h, w, ZEROS);
  result += bckcol;

  //lets make the axes
  Point2D<float> origin(axisonright ? w - hborder : hborder, h - vborder);
  Point2D<float> originraw(hborder, h - vborder);
  Point2D<float> xlim(!axisonright ? w - hborder : hborder, h - vborder);
  Point2D<float> ylim(axisonright ? w - hborder : hborder, vborder);
  float xpos, ypos;
  //write labels
  if (strlen(ylabel) > 0)
    {
      xpos = origin.j + (ylim.j - origin.j) / 2.0F - 
        sf.w() * strlen(ylabel) / 2.0F;
      if (axisonright)
        ypos = ylim.i  + sn.h() + sn.h() / 2.0F;
      else
        ypos = vborder - sn.h() - sn.h() / 2.0F;
      writeText(result,Point2D<int>((int)xpos, (int)ypos), ylabel, 
                linecol, bckcol, sf);//ylabel
      result = flipVertic(transpose(result));
    }
     
  if (strlen(title) > 0)
    {
      xpos = origin.i + (xlim.i - origin.i)/2.0F - 
        sf.w() * strlen(title) / 2.0F;
      ypos = vborder - 1.25 * sf.h();
      writeText(result, Point2D<int>((int)xpos, (int)ypos),
                title, linecol, bckcol, sf);//title 
    }

  if(strlen(xlabel) > 0)
    {
      xpos = origin.i + (xlim.i - origin.i) / 2.0F - 
        sf.w() * strlen(xlabel) / 2.0F;
      ypos = origin.j + sn.h() + sn.h() / 2.0F;
      writeText(result,Point2D<int>( (int)xpos,(int)ypos),
                xlabel, linecol, bckcol, sf);//xlabel
    }
  
  if (numticks >= 0)  
    {
      if (axisonright)
        xpos = ylim.i + sn.w() * 2.0F;
      else
        xpos = hborder - (sn.w() * ymaxstr.size() + sn.w() * 2.0F);
      ypos = ylim.j;
      writeText(result, Point2D<int>((int)xpos,(int)ypos),
                ymaxstr.c_str(), linecol, bckcol, sn);//ymax

      if (axisonright)
        xpos = ylim.i + sn.w() * 2.0F;
      else
        xpos = hborder - (sn.w() * yminstr.size() + sn.w() * 2.0F);
      ypos = ylim.j;      
      ypos = origin.j - sn.h();
      writeText(result, Point2D<int>((int)xpos, (int)ypos),
                yminstr.c_str(), linecol, bckcol, sn);//ymin
      
      //Fill the x-axis with 'numticks' equally spaced tick marks. 
      float tspace = (xlim.i - origin.i)/(float)numticks;//tick spacing (pixel)
      for (int ii = 0; ii <= numticks; ++ii)
        {
          //tickmark
          xpos =  tspace * ii + origin.i; //xpos
          Point2D<int> tick((int)xpos, (int)xlim.j), 
            ticke((int)xpos, int(tick.j - sn.h()));
          drawLine(result, tick,  ticke, linecol, thickness);
          
          //draw label
          float xval = (float)vecLength / (float)numticks * (float)ii;
          std::string ticklabel(sformat("%1.2f", xval));
          int offset = strlen(ticklabel.c_str())*(int)sn.w()/2;
          tick.i -= offset; tick.j += sn.h() / 2;
          //time point
          writeText(result,tick,ticklabel.c_str(), linecol, bckcol, sn);
        }
    }
  //draw axis
  Point2D<int> origin_int((int)origin.i,(int)origin.j);
  Point2D<int> originraw_int((int)originraw.i,(int)originraw.j);
  Point2D<int> xlim_int((int)xlim.i,(int)xlim.j);
  Point2D<int> ylim_int((int)ylim.i,(int)ylim.j);
  
  drawLine(result, origin_int, xlim_int, linecol, thickness);
  drawLine(result, origin_int, ylim_int, linecol , thickness);

  //generate and rescale vector of points to be plotted
  T   newMaxY = h - vborder, newMinY = vborder;
  T   newMaxX = w - hborder, newMinX = hborder;
  int   xPoints[vecLength], yPoints[vecLength];
  
  for (int i = 0; i < vecLength; i++)
    {
      xPoints[i] = (int)((newMaxX - newMinX) * (float)i / vecLength + newMinX);

      T p = points[i];
      if (points[i] < minv) p = minv;
      else if (points[i] > maxv) p = maxv;
      
      yPoints[i] = (int)((newMaxY - newMinY) * (p - minv)/(maxv - minv) + newMinY + .5);
      
      if (i == 0)
        drawLine(result, originraw_int, 
                 Point2D<int>(xPoints[i],(int)(newMaxY - yPoints[i] + vborder)), linecol, thickness);
      else
        drawLine(result,
                 Point2D<int>(xPoints[i-1],(int)(newMaxY - yPoints[i-1] + vborder)),
                 Point2D<int>(xPoints[i], (int)(newMaxY - yPoints[i] + vborder)),
                 linecol, thickness);
    }
  return result;
}

// ###############################################################
template <typename T>
Image<PixRGB<byte> > multilinePlot(const std::vector<std::vector<T> >& lines, const int w,
                              const int h,T minVal,  T maxVal,
                              const char *title,
                              const char *ylabel,
                              const char *xlabel,
                              const std::vector<PixRGB<byte> >& linescolor,
                              const PixRGB<byte>& gridcolor,
                              const PixRGB<byte>& bckcolor)
{

	//        std::vector<T> points = lines[0];
	//find our min and max range
	int defvecLength = 0;

	if(lines.size() !=0){

		defvecLength = lines[0].size();
		for(size_t i=0; i<lines.size(); i++)
			defvecLength = std::min(defvecLength, (int)lines[i].size());

		if (defvecLength != 0 && minVal == 0 && maxVal == 0)
		{
			maxVal = lines[0][0];
			for(int l = 0; l < (int)lines.size();l++)
			{

				std::vector<T> points = lines[l];
				const int vecLength = points.size();
				for(int i = 0; i < vecLength ; i++)
					if(points[i] > maxVal)
						maxVal = points[i];

				minVal    = points[0];
				for(int i = 0; i < vecLength ; i++)
					if(points[i] < minVal)
						minVal = points[i];
			}
		}
	}



	//fonts
	SimpleFont sf = SimpleFont::fixedMaxHeight(int(.1*h));
	SimpleFont sn = SimpleFont::fixedMaxHeight(int(.01*h));
	std::string ymaxstr = sformat("%1.2f",(float)maxVal);
	std::string yminstr = sformat("%1.2f",(float)minVal);

	//border between image corner and y-axis
	int hborder;
	ymaxstr.size() > yminstr.size() ?
		hborder = ymaxstr.size() : hborder = yminstr.size();
	//make our border has room for at least two sig figs, and four
	//positions for value and sign
	if (hborder < 7)
		hborder = 7;
	hborder = (int)(hborder * sn.w() + sf.w()*1.5);
	int vborder = (int)(sf.h() * 1.5);

	//create image with opposite axis as we will transpose after we draw
	//the y axis
	Image<PixRGB<byte> > result(h, w, ZEROS);
	result += bckcolor;

	//lets make the axes
	Point2D<float> origin(hborder, h - vborder);
	Point2D<float> xlim(w-hborder, h - vborder);
	Point2D<float> ylim(hborder, vborder);

	//write labels
	float pos = origin.j + (ylim.j - origin.j)/2.0F -
		float(sf.w()*strlen(ylabel))/2.0F;
	writeText(result,Point2D<int>((int)pos, (int)(vborder - 1.25*sf.h())),
			ylabel, gridcolor, bckcolor, sf);//ylabel
	result = flipVertic(transpose(result));

	writeText(result,Point2D<int>(hborder-sn.w()*ymaxstr.size()-1,
				(int)ylim.j-sn.h()),
			ymaxstr.c_str(), gridcolor, bckcolor, sn);//ymax

	writeText(result,Point2D<int>(hborder - sn.w() * yminstr.size() -1,
				(int)origin.j),
			yminstr.c_str(), gridcolor, bckcolor, sn);//ymin

	pos = origin.i + (xlim.i - origin.i)/2.0F - float(sf.w()*strlen(title))/2;
	writeText(result, Point2D<int>( (int)pos, (int)(vborder - 1.25*sf.h())),
			title, gridcolor, bckcolor, sf);//title

	pos = origin.i + (xlim.i - origin.i)/2.0F - float(sf.w()*strlen(xlabel))/2.0F;
	writeText(result,Point2D<int>( (int)pos,(int)( origin.j + 1.1 * sf.h())),
			xlabel, gridcolor, bckcolor, sf);//xlabel

	//draw axis
	Point2D<int> origin_int((int)origin.i,(int)origin.j),
		xlim_int((int)xlim.i,(int)xlim.j),
		ylim_int((int)ylim.i,(int)ylim.j);

	//generate and rescale vector of points to be plotted
	T   newMaxY = h - vborder, newMinY = vborder;
	T   newMaxX = w - hborder, newMinX = hborder;
	int   xPoints[defvecLength], yPoints[defvecLength];

	for(int j = 0; j < (int)lines.size(); j++)
	{

		//Check line color vector is available
		PixRGB<byte> linecolor(255,255,255) ;
		if(j < (int)linescolor.size())
			linecolor = linescolor[j];

		for (int i = 0; i < defvecLength; i++)
		{
			xPoints[i] = (int)((newMaxX - newMinX) * (float)i / defvecLength + newMinX);
			yPoints[i] = (int)((newMaxY - newMinY) *
					(lines[j][i] - minVal)/(maxVal - minVal) + newMinY + .5);

			if (i == 0)
				drawLine(result, origin_int,
						Point2D<int>(xPoints[i],(int)( newMaxY - yPoints[i] + vborder)),
						linecolor, 1);
			else
				drawLine(result,
						Point2D<int>(xPoints[i-1],(int)(newMaxY - yPoints[i-1] + vborder)),
						Point2D<int>(xPoints[i], (int)(newMaxY - yPoints[i] + vborder)),
						linecolor, 1);
		}
	}
	//draw axis
	drawLine(result, origin_int, xlim_int, gridcolor, 1);
	drawLine(result, origin_int, ylim_int, gridcolor , 1);

	return result;
}


// ######################################################################
template <class T>
void drawOutlinedPolygon(Image<T>& img, const std::vector<Point2D<int> >& polygon,
                         const T col,
                         const Point2D<int> trans,
                         const float rot,
                         const float scale,
                         const float k1,
                         const float k2,
                         const int rad)
{
GVX_TRACE(__PRETTY_FUNCTION__);
 for(uint i = 0; i<polygon.size(); i++)
   {
     Point2D<float> p1 = (Point2D<float>)polygon[i];
     Point2D<float> p2 = (Point2D<float>)polygon[(i+1)%polygon.size()];


     Point2D<float> tmpPos1;
     Point2D<float> tmpPos2;
     //Sheer
     tmpPos1.i = p1.i + p1.j*k1;
     tmpPos1.j = p1.i*k2 + p1.j;

     tmpPos2.i = p2.i + p2.j*k1;
     tmpPos2.j = p2.i*k2 + p2.j;

     //Rotate and move to p
     p1.i = scale*(cos(rot)*tmpPos1.i - sin(rot)*tmpPos1.j) + trans.i;
     p1.j = scale*(sin(rot)*tmpPos1.i + cos(rot)*tmpPos1.j) + trans.j;

     p2.i = scale*(cos(rot)*tmpPos2.i - sin(rot)*tmpPos2.j) + trans.i;
     p2.j = scale*(sin(rot)*tmpPos2.i + cos(rot)*tmpPos2.j) + trans.j;

     drawLine(img, (Point2D<int>)p1,(Point2D<int>)p2, col, rad);
   }
}

// ######################################################################
template <class T>
void drawFilledPolygon(Image<T>& img, const std::vector<Point2D<int> >& polygon,
                       const T col)
{
  //OLD Slow Method which iterates over all pixels
  const int w = static_cast<int>(img.getWidth());
  const int h = static_cast<int>(img.getHeight());
  typename Image<T>::iterator iptr = img.beginw();
  //Point2D<int> p;

  //for (p.j = 0; p.j < h; p.j ++)
  //  for (p.i = 0; p.i < w; p.i ++)
  //    if (pnpoly(polygon, p)) *iptr++ = col; else iptr ++;

  //New method derived from pascal code by achalfin@uceng.uc.edu
  //http://www.bsdg.org/SWAG/EGAVGA/0170.PAS.html

  if (polygon.size() <= 0 ) return;

  int edgeCount = polygon.size();
  int num = polygon.size();
  int minY = polygon[0].j; 
  int startV1 = 0;
  for(uint c= 1; c < polygon.size(); c++) //Find Top Vertex
  {
    if (polygon[c].j < minY)
    {
      minY = polygon[c].j;
      startV1 = c;
    }
  }
  int startV2 = startV1;
  int endV1 = startV1 - 1;
  if(endV1 < 0) endV1 = (num-1);

  int endV2 = startV2 + 1;
  if (endV2 >= num) endV2 = 0;

  minY = polygon[startV1].j;
  int x1 = polygon[startV1].i; int y1 = polygon[startV1].j;
  int x2 = polygon[endV1].i; int y2 = polygon[endV1].j;

  int dx1 = ((x2 - x1) << 8) / (y2 - y1 + 1);
  int count1 = y2-y1;
  int xVal1 = x1 << 8;

  int x11 = polygon[startV2].i; int y11 = polygon[startV2].j;
  int x22 = polygon[endV2].i; int y22 = polygon[endV2].j;

  int dx2 = ((x22 - x11) << 8) / (y22 - y11 + 1);
  int count2 = y22-y11;
  int xVal2 = x11 << 8;

  while (edgeCount > 1)
  {
    while ( (count1 > 0) &&  (count2 > 0) )
    {
      //Draw the horizontal line
      for(int x = xVal1>>8; x <= xVal2>>8; x++)
        if (x >= 0 && x < w && minY >= 0 && minY < h)
          iptr[x + w * minY] = col;

      xVal1 += dx1; xVal2 += dx2;
      count1--; count2--;
      minY++;
    }
    if (count1 == 0)
    {
      edgeCount--;
      startV1 = endV1;
      endV1--;
      if (endV1 < 0) endV1 = num-1;

      minY = polygon[startV1].j;
      x1 = polygon[startV1].i; y1 = polygon[startV1].j;
      x2 = polygon[endV1].i; y2 = polygon[endV1].j;
      dx1 = ((x2 - x1) << 8) / (abs(y2 - y1) + 1);
      count1 = y2-y1;
      xVal1 = x1 << 8;
    }
    if (count2 == 0)
    {
      edgeCount--;
      startV2 = endV2;
      endV2++;
      if(endV2 >= num) endV2 = 0;
      minY = polygon[startV2].j;
      x11 = polygon[startV2].i; y11 = polygon[startV2].j;
      x22 = polygon[endV2].i; y22 = polygon[endV2].j;
      dx2 = ((x22 - x11) << 8) / (abs(y22 - y11) + 1);
      count2 = y22-y11;
      xVal2 = x11 << 8;
    }
  }
}

// ######################################################################
Image<PixRGB<byte> > drawMeters(const MeterInfo* infos, const size_t ninfo,
                                const size_t nx, const Dims& meterdims)
{
  if (ninfo == 0) return Image<PixRGB<byte> >();

  ASSERT(meterdims.w() > 0 && meterdims.h() > 0);

  size_t maxlabelsize = infos[0].label.size();
  for (size_t i = 1; i < ninfo; ++i)
    if (infos[i].label.size() > maxlabelsize) maxlabelsize = infos[i].label.size();

  const SimpleFont f = SimpleFont::fixedMaxHeight(meterdims.h());

  const int meterx = f.w() * (maxlabelsize + 10);
  const int maxmeterlen = meterdims.w() - meterx;

  const size_t ny = (ninfo + nx-1) / nx;

  Image<PixRGB<byte> > result(meterdims.w() * nx, meterdims.h() * ny, ZEROS);

  for (size_t i = 0; i < ninfo; ++i) {
    const size_t ay = i % ny, ax = i / ny;

    const std::string txt = sformat("%*s %6.2e", int(maxlabelsize), infos[i].label.c_str(), infos[i].val);

    writeText(result, Point2D<int>(meterdims.w() * ax, meterdims.h() * ay),
              txt.c_str(), PixRGB<byte>(255), PixRGB<byte>(0), f);

    const int meterlen = clampValue(int(maxmeterlen * infos[i].val / infos[i].valmax), 1, maxmeterlen);

    const int threshlen =
      (infos[i].thresh < 0.0 || infos[i].thresh > infos[i].valmax) ?
      -1 : int(maxmeterlen * infos[i].thresh / infos[i].valmax);

    Image<PixRGB<byte> >::iterator itr = result.beginw()
      + meterdims.w()*ax + meterx + ay*meterdims.h()*result.getWidth();

    const int rowskip = result.getWidth() - maxmeterlen;

    const PixRGB<byte> c1(infos[i].color);
    const PixRGB<byte> c2(c1/2);
    const PixRGB<byte> c3(c2/3);
    const PixRGB<byte> c4(c3/2);

    for (int y = 0; y < meterdims.h()-1; ++y) {
      for (int x = 0; x < meterlen; ++x)
        *itr++ = (x == threshlen) ? PixRGB<byte>(255,255,255) : (x & 1) ? c2 : c1;
      for (int x = meterlen; x < maxmeterlen; ++x)
        *itr++ = (x == threshlen) ? PixRGB<byte>(255,255,255) : (x & 1) ? c4 : c3;
      itr += rowskip;
    }
  }
  return result;
}


// Include the explicit instantiations (color instantiations are now
// requested by using "T_or_RGB" for the template formal parameter name in
// the declarations in the .H file).
#include "inst/Image/DrawOps.I"

//explicit instantiations for line plot
template
Image<PixRGB<byte> > linePlot(const std::vector<double>& points, const int w,
                              const int h, const double& minVal, const double& maxVal,
                              const char *title,
                              const char *ylabel,
                              const char *xlabel,
                              const PixRGB<byte>& linecol,
                              const PixRGB<byte>& bckcol, 
                              const int numticks, const bool axisonright);

template
Image<PixRGB<byte> > linePlot(const std::deque<double>& points, const int w,
                              const int h, const double& minVal, const double& maxVal,
                              const char *title,
                              const char *ylabel,
                              const char *xlabel,
                              const PixRGB<byte>& linecol,
                              const PixRGB<byte>& bckcol, 
                              const int numticks, const bool axisonright);

template
Image<PixRGB<byte> > linePlot(const std::deque<float>& points, const int w,
                              const int h, const float& minVal, const float& maxVal,
                              const char *title,
                              const char *ylabel,
                              const char *xlabel,
                              const PixRGB<byte>& linecol,
                              const PixRGB<byte>& bckcol, 
                              const int numticks, const bool axisonright);

template
Image<PixRGB<byte> > linePlot(const std::vector<float>& points, const int w,
                              const int h, const float& minVal, const float& maxVal,
                              const char *title,
                              const char *ylabel,
                              const char *xlabel,
                              const PixRGB<byte>& linecol,
                              const PixRGB<byte>& bckcol, 
                              const int numticks, const bool axisonright);

template
Image<PixRGB<byte> > linePlot(const std::vector<int>& points, const int w,
                              const int h, const int& minVal, const int& maxVal,
                              const char *title,
                              const char *ylabel,
                              const char *xlabel,
                              const PixRGB<byte>& linecol,
                              const PixRGB<byte>& bckcol, 
                              const int numticks, const bool axisonright);

// explicit instantiations for drawContour2D:
template
void drawContour2D(const Image<byte>& src, Image< PixRGB<byte> >& dst, const PixRGB<byte> &col, const byte rad);

template
void drawContour2D(const Image<byte>& src, Image<byte>& dst, const byte &col, const byte rad);

template 
Image<PixRGB<byte> > drawHistogram(std::vector<float> hist, int width, int height, PixRGB<byte> lineVal, PixRGB<byte> fillVal);
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
