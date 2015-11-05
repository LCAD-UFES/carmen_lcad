/*!@file GUI/DebugWin.H A window class to show images  */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/DebugWin.H $
// $Id: DebugWin.H 13898 2010-09-09 01:19:08Z lior $
//
//

#ifndef DEBUGWIN_H
#define DEBUGWIN_H

#include "GUI/XWinManaged.H"
#include "Image/Point2D.H"
#include "Image/Pixels.H"
#include "Image/Transforms.H"
#include "Image/ColorOps.H"
#include "Image/ShapeOps.H"
#include "Image/MathOps.H"
#include "Image/DrawOps.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"
#include "Envision/env_image.h"

//debuging macros
inline void SHOWIMG(Image<float> img, bool mag = false) {
  XWinManaged debug_xwin( img.getDims(), -1, -1, "DebugWin");
  Image<float> tmpImg = img;
  debug_xwin.drawImage(tmpImg);
  int key = -1;
  while(key != 24) //q to quite
  {
    key = debug_xwin.getLastKeyPress();
    Point2D<int>  p = debug_xwin.getLastMouseClick();
    if (p.isValid())
    {
      LINFO("Loc %ix%i pix value: %f", p.i, p.j, img.getVal(p));

      if (mag)
      {
        Dims dims(9,9);
        int tl_i = p.i - dims.w()/2;
        int tl_j = p.j - dims.h()/2;

        Image<float> c = crop(img, Point2D<int>(tl_i, tl_j), dims);
        SHOWIMG(rescaleNI(c, 256, 256));
      }
    }

    if (key == 58)  //m
    {
      Point2D<int> loc;
      float maxVal;
      findMax(img, loc, maxVal);
      LINFO("Max at %ix%i val=%f", loc.i, loc.j, maxVal);
      //img.setVal(loc, 0);
      drawDisk(img, loc, 6, 0.0F);
      drawCircle(tmpImg, loc, 6, maxVal);
      debug_xwin.drawImage(tmpImg);
    }
    usleep(10000);
  }
}

inline void SHOWIMG(Image<byte> img, bool mag = false) {
  XWinManaged debug_xwin( img.getDims(), -1, -1, "DebugWin");
  debug_xwin.drawImage(img);
  int key = -1;
  while(key != 24) //q to quite
  {
    key = debug_xwin.getLastKeyPress();
    Point2D<int>  p = debug_xwin.getLastMouseClick();
    if (p.isValid())
    {
      LINFO("Loc %ix%i pix value: %i", p.i, p.j, img.getVal(p));

      if (mag)
      {
        Dims dims(9,9);
        int tl_i = p.i - dims.w()/2;
        int tl_j = p.j - dims.h()/2;

        Image<float> c = crop(img, Point2D<int>(tl_i, tl_j), dims);
        SHOWIMG(rescaleNI(c, 256, 256));
      }


    }
    usleep(10000);
  }
}

inline void SHOWIMG(Image<PixRGB<byte> > img) {

  if (img.initialized())
  {
    XWinManaged debug_xwin( img.getDims(), -1, -1, "DebugWin");
    debug_xwin.drawImage(img);
    int key = -1;
    while(key != 24) //q to quit
    {
      key = debug_xwin.getLastKeyPress();
      Point2D<int>  p = debug_xwin.getLastMouseClick();
      if (p.isValid())
      {
        PixRGB<byte> pix = img.getVal(p);
        LINFO("Loc %ix%i pix value: (%i,%i,%i)", p.i, p.j, pix[0], pix[1], pix[2]);
      }
      if (key == 25) //write image
        Raster::WriteRGB(img, "DebugWin.ppm");
      usleep(10000);
    }
  }
}

inline void ENV_SHOWIMG(struct env_image *img, int w=-1, int h=-1) {

  if (w == -1 || h == -1)
  {
    w = img->dims.w;
    h = img->dims.h;
  }


  XWinManaged debug_xwin(Dims(w,h), -1, -1, "DebugWin");

  Image<float> outImg(img->dims.w, img->dims.h, NO_INIT);
  const intg32* const src = img->pixels;
  const env_size_t sz = env_img_size(img);
  float* outImgPtr = outImg.getArrayPtr();

  for (env_size_t i = 0; i < sz; ++i)
   outImgPtr[i] = (float) src[i];

  outImg = rescale(outImg, w, h);

  debug_xwin.drawImage(outImg);
  int key = -1;
  while(key != 24) //q to quite
  {
    key = debug_xwin.getLastKeyPress();
    Point2D<int>  p = debug_xwin.getLastMouseClick();
    if (p.isValid())
    {
      LINFO("Loc %ix%i pix value: %f", p.i, p.j, outImg.getVal(p));
    }
    usleep(10000);
  }
}




#endif
