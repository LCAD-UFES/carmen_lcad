/*!@file Image/OmniOps.C Operations for omnidirectional correction
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/OmniOps.C $
// $Id: OmniOps.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef IMAGE_OMNI_C_DEFINED
#define IMAGE_OMNI_C_DEFINED

#include "Image/OmniOps.H"

#include "Image/Image.H"
#include "Image/Pixels.H"

#include <cmath>
#include <iostream>

// ######################################################################
template <class T>
void lowPassPixel(Image<T>& src,
                  const int x, const int y, const int radius)
{
  int startX = x - radius, endX = x + radius;
  int startY = y - radius, endY = y + radius;
  T sum = T(); int n = 0;
  if (startX < 0) startX = 0;
  if (endX > src.getWidth() - 1) endX = src.getWidth() - 1;
  if (startY < 0) startY = 0;
  if (endY > src.getHeight() - 1) endY = src.getHeight() - 1;
  for (int xx = startX; xx < endX ; xx++)
    for(int yy = startY; yy < endY; yy++)
      { sum += src.getVal(xx, yy); n ++; }
  sum /= n;
  src.setVal(x, y, sum);
}

// ######################################################################
template <class T>
Image<PixRGB<T> > omniCorrectGen(const Image<PixRGB<T> >& src,
                                 const int Rx, const int Ry,
                                 const int Xc, const int Yc,
                                 const int RA)
{
  Point2D<int> oldPoint, point;
  float newX = 0.0F, newY = 0.0F, absX, absY;
  bool negX, negY, noPix, stop;

  PixRGB<T> value;
  // result has same size as input:
  Image<PixRGB<T> > result(src.getDims(), NO_INIT);

  Dims dims = src.getDims();

  for (int x = 0; x < dims.w(); x++)
    {
      std::cout << ".";
      for(int y = 0; y < dims.h(); y++)
        {
          noPix = false; stop = false; oldPoint.i = x; oldPoint.j = y;
          // Fix circular boundary:
          absX = fabs(Rx - x); //find distance from center in X
          absY = fabs(Ry - y); //find distance from center in Y
          if (x < Xc) negX = true; else negX = false;
          if (y < Yc) negY = true; else negY = false;
          // find ratio factor for x:
          if ((absY <= Ry) && (absY != 0))
            newX = sqrt(1.0 - pow((absY / (Ry + RA)), 2.0));
          else
            noPix = true;
          //find ratio factor for y:
          if((absX <= Rx) && (!noPix) && (absX != 0))
            newY = sqrt(1.0 - pow((absX / (Rx + RA)), 2));
          else
            noPix = true;

          if(noPix)
            {
              value = src.getVal(oldPoint);
              result.setVal(oldPoint, value);
            }
          else
            {
              if ((x != 0) && (newX != 0))
                newX = absX / newX; // finish X pixel
              else
                {
                  value = src.getVal(oldPoint);
                  result.setVal(oldPoint,value);
                  stop = true;
                }
              if ((y != 0) && (newY != 0))
                newY = absY / newY; // finish Y pixel
              else
                {
                  value = src.getVal(oldPoint);
                  result.setVal(oldPoint,value);
                  stop = true;
                }
              if (stop == false)
                {
                  if (negX) newX = Xc - newX; else newX = Xc + newX;
                  if (negY) newY = Yc - newY; else newY = Yc + newY;
                  point.i = (int)newX; point.j = (int)newY;
                  value = src.getVal(oldPoint); // get value of old pixel

                  // assure circular bound:
                  if (point.i < dims.w() && point.j < dims.h())
                    {
                      // translate value to new pixel coordinate
                      if (point.i >= 0 && point.j >= 0)
                        result.setVal(point, value);
                    }
                }
            }
        }
    }

  return result;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > omniCorrectSp(const Image<PixRGB<T> >& src,
                                const float r,
                                const float hh, const float kk,
                                const int /*Rx*/, const int /*Ry*/,
                                const int Xc, const int Yc)
{
  // bool noPix = false; // FIXME noPix is unused...?

  // result has same size as input image:
  Image<PixRGB<T> > result(src.getDims(), NO_INIT);

  for(int x = 0; x < src.getWidth(); ++x)
    for(int y = 0; y < src.getHeight(); ++y)
      {
        // FIXME the absX/absY variables are unused...?
        // int absX = abs(Rx - x); // find distance from center in X
        // int absY = abs(Ry - y); // find distance from center in Y
        bool negX = (x < Xc);
        bool negY = (y < Yc);

        // FIXME things may be bungled here because the 'hh' parameter used to
        // be called 'h', which conflicted with the data member 'h'
        int newX = int(-(sqrt(-pow((x - kk), 2.0) + pow(r, 2.0)) - hh));
        int newY = int(-(sqrt(-pow((y - kk), 2.0) + pow(r, 2.0)) - hh));

        if (negX) newX = -newX;
        if (negY) newY = -newY;

        PixRGB<T> value = src.getVal(x, y); // get value of old pixel
        result.setVal(newX, newY, value); // translate to new pixel coordinate
      }

  return result;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > omniDenebulize(const Image< PixRGB<T> >& src,
                                 const int radius)
{
  Image<PixRGB<T> > result = src;

  for (int x = 0; x < src.getWidth(); ++x)
    for (int y = 0; y < src.getHeight(); ++y)
      {
        PixRGB<T> pix = src.getVal(x,y);
        if((pix.red() < T(1)) && // FIXME why 1 here but 0 in the monochrome
                                 // version??
           (pix.green() < T(1)) &&
           (pix.blue() < T(1)))
          lowPassPixel(result, x, y, radius);
      }

  return result;
}

// ######################################################################
template <class T>
Image<T> omniDenebulize(const Image<T>& src, const int radius)
{
  Image<T> result = src;

  for (int x = 0; x < src.getWidth(); ++x)
    for (int y = 0; y < src.getHeight(); ++y)
      if (src.getVal(x, y) == T(0))
        lowPassPixel(result, x, y, radius);

  return result;
}

// Include the explicit instantiations
#include "inst/Image/OmniOps.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_OMNI_C_DEFINED
