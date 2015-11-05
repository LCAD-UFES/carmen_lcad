/*!@file Raster/DebayerREG.C is the debayer class without use sse */

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
// Primary maintainer for this file: Zhicheng Li <zhicheng@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/DeBayerREG.C $
// $Id: DeBayerREG.C 10794 2009-02-08 06:21:09Z itti $
//
#include "Raster/DeBayerREG.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Util/StringUtil.H"
#include "Util/FileUtil.H"

template <class T> Image<PixRGB<T> >
debayerREG (Image<T> src,
             BayerFormat format)
{
  int w = src.getWidth();
  int h = src.getHeight();
  ASSERT(w % 2 == 0);
  ASSERT(h % 2 == 0);

  Image<PixRGB<float> > res(src.getDims(), NO_INIT);
  Image<float> srcF = src;
  Image<float>::const_iterator sptr = srcF.begin();
  Image<PixRGB<float> >::iterator dptr = res.beginw();

  if(format == BAYER_GBRG || format == BAYER_GBRG12)
    for (int y = 0; y < h; y += 2){
      for (int x = 0; x < w; x += 2){
        if (x < 2 || x >= w-2 || y < 2 || y >= h-2)
          {
            const float r = sptr[w];
            const float g = 0.5f * (sptr[0] + sptr[w+1]);
            const float b = sptr[1];

            dptr[0].p[0] = r;
            dptr[1].p[0] = r;
            dptr[w].p[0] = r;
            dptr[w+1].p[0] = r;

            dptr[0].p[1] = g;
            dptr[1].p[1] = g;
            dptr[w].p[1] = g;
            dptr[w+1].p[1] = g;

            dptr[0].p[2] = b;
            dptr[1].p[2] = b;
            dptr[w].p[2] = b;
            dptr[w+1].p[2] = b;
          }
        else
          {
            /*
                    +------+------+------+------+
                    :-w-1  :-w    :-w+1  :-w+2  :
                    :      :      :      :      :
                    :    G :    R :    G :    R :
                    +------+======+======+------+
                    :-1    |0     |1     |2     :
                    :      |      |      |      :
                    :    B |    G |    B |    G :
                    +------+======+======+------+
                    :w-1   |w     |w+1   |w+2   :
                    :      |      |      |      :
                    :    G |    R |    G |    R :
                    +------+======+======+------+
                    :w*2-1 :w*2   :w*2+1 :w*2+2 :
                    :      :      :      :      :
                    :    B :    G :    B :    G :
                    +------+------+------+------+
            */

            dptr[0].p[0] = 0.5f * (sptr[-w] + sptr[w]);
            dptr[1].p[0] = 0.25f * (sptr[-w] + sptr[-w+2] + sptr[w] + sptr[w+2]);
            dptr[w].p[0] = sptr[w];
            dptr[w+1].p[0] = 0.5f * (sptr[w] + sptr[w+2]);

            dptr[0].p[1] = 0.5f * sptr[0] + 0.125f * (sptr[-w-1] + sptr[-w+1] + sptr[w-1] + sptr[w+1]);
            dptr[1].p[1] = 0.25f * (sptr[-w+1] + sptr[0] + sptr[2] + sptr[w+1]);
            dptr[w].p[1] = 0.25f * (sptr[0] + sptr[w-1] + sptr[w+1] + sptr[w*2]);
            dptr[w+1].p[1] = 0.5f * sptr[w+1] + 0.125f * (sptr[0] + sptr[2] + sptr[w*2] + sptr[w*2+2]);

            dptr[0].p[2] = 0.5f * (sptr[-1] + sptr[1]);
            dptr[1].p[2] = sptr[1];
            dptr[w].p[2] = 0.25f * (sptr[-1] + sptr[1] + sptr[w*2-1] + sptr[w*2+1]);
            dptr[w+1].p[2] = 0.5f * (sptr[1] + sptr[w*2+1]);
          }

        sptr += 2;
        dptr += 2;
      }

      sptr += w;
      dptr += w;
    }

    // case BAYER_GRBG:
  else if(format == BAYER_GRBG || format == BAYER_GRBG12)
    for (int y = 0; y < h; y += 2){
      for (int x = 0; x < w; x += 2){
        if (x < 2 || x >= w-2 || y < 2 || y >= h-2)
          {
            const float r = sptr[1];
            const float g = 0.5f * (sptr[0] + sptr[w+1]);
            const float b = sptr[w];

            dptr[0].p[0] = r;
            dptr[1].p[0] = r;
            dptr[w].p[0] = r;
            dptr[w+1].p[0] = r;

            dptr[0].p[1] = g;
            dptr[1].p[1] = g;
            dptr[w].p[1] = g;
            dptr[w+1].p[1] = g;

            dptr[0].p[2] = b;
            dptr[1].p[2] = b;
            dptr[w].p[2] = b;
            dptr[w+1].p[2] = b;
          }
        else
          {
            /*
                    +------+------+------+------+
                    :-w-1  :-w    :-w+1  :-w+2  :
                    :      :      :      :      :
                    :    G :    B :    G :    B :
                    +------+======+======+------+
                    :-1    |0     |1     |2     :
                    :      |      |      |      :
                    :    R |    G |    R |    G :
                    +------+======+======+------+
                    :w-1   |w     |w+1   |w+2   :
                    :      |      |      |      :
                    :    G |    B |    G |    B :
                    +------+======+======+------+
                    :w*2-1 :w*2   :w*2+1 :w*2+2 :
                    :      :      :      :      :
                    :    R :    G :    R :    G :
                    +------+------+------+------+
            */

            dptr[0].p[0] = 0.5f * (sptr[-1] + sptr[1]);
            dptr[1].p[0] = sptr[1];
            dptr[w].p[0] = 0.25f * (sptr[-1] + sptr[1] + sptr[w*2-1] + sptr[w*2+1]);
            dptr[w+1].p[0] = 0.5f * (sptr[1] + sptr[w*2+1]);

            dptr[0].p[1] = 0.5f * sptr[0] + 0.125f * (sptr[-w-1] + sptr[-w+1] + sptr[w-1] + sptr[w+1]);
            dptr[1].p[1] = 0.25f * (sptr[-w+1] + sptr[0] + sptr[2] + sptr[w+1]);
            dptr[w].p[1] = 0.25f * (sptr[0] + sptr[w-1] + sptr[w+1] + sptr[w*2]);
            dptr[w+1].p[1] = 0.5f * sptr[w+1] + 0.125f * (sptr[0] + sptr[2] + sptr[w*2] + sptr[w*2+2]);

            dptr[0].p[2] = 0.5f * (sptr[-w] + sptr[w]);
            dptr[1].p[2] = 0.25f * (sptr[-w] + sptr[-w+2] + sptr[w] + sptr[w+2]);
            dptr[w].p[2] = sptr[w];
            dptr[w+1].p[2] = 0.5f * (sptr[w] + sptr[w+2]);
          }

        sptr += 2;
        dptr += 2;
      }

      sptr += w;
      dptr += w;
     }


  //case BAYER_RGGB:
  else if(format == BAYER_RGGB || format == BAYER_RGGB12)
    for (int y = 0; y < h; y += 2){
      for (int x = 0; x < w; x += 2){
        if (x < 2 || x >= w-2 || y < 2 || y >= h-2)
          {
            const float r = sptr[0];
            const float g = 0.5f * (sptr[1] + sptr[w]);
            const float b = sptr[w+1];

            dptr[0].p[0] = r;
            dptr[1].p[0] = r;
            dptr[w].p[0] = r;
            dptr[w+1].p[0] = r;

            dptr[0].p[1] = g;
            dptr[1].p[1] = g;
            dptr[w].p[1] = g;
            dptr[w+1].p[1] = g;

            dptr[0].p[2] = b;
            dptr[1].p[2] = b;
            dptr[w].p[2] = b;
            dptr[w+1].p[2] = b;
          }
        else
          {
            /*
                    +------+------+------+------+
                    :-w-1  :-w    :-w+1  :-w+2  :
                    :      :      :      :      :
                    :    B :    G :    B :    G :
                    +------+======+======+------+
                    :-1    |0     |1     |2     :
                    :      |      |      |      :
                    :    G |    R |    G |    R :
                    +------+======+======+------+
                    :w-1   |w     |w+1   |w+2   :
                    :      |      |      |      :
                    :    B |    G |    B |    G :
                    +------+======+======+------+
                    :w*2-1 :w*2   :w*2+1 :w*2+2 :
                    :      :      :      :      :
                    :    G :    R :    G :    R :
                    +------+------+------+------+
            */

            dptr[0].p[0] = sptr[0];
            dptr[1].p[0] = 0.5f * (sptr[0] + sptr[2]);
            dptr[w].p[0] = 0.5f * (sptr[0] + sptr[w*2]);
            dptr[w+1].p[0] = 0.25f * (sptr[0] + sptr[2] + sptr[w*2] + sptr[w*2+2]);

            dptr[0].p[1] = 0.25f * (sptr[-w] + sptr[-1] + sptr[1] + sptr[w]);
            dptr[1].p[1] = 0.5f * sptr[1] + 0.125f * (sptr[-w] + sptr[-w+2] + sptr[w] + sptr[w+2]);
            dptr[w].p[1] = 0.5f * sptr[w] + 0.125f * (sptr[-1] + sptr[1] + sptr[w*2-1] + sptr[w*2+1]);
            dptr[w+1].p[1] = 0.25f * (sptr[1] + sptr[w] + sptr[w+2] + sptr[w*2+1]);

            dptr[0].p[2] = 0.25f * (sptr[-w-1] + sptr[-w+1] + sptr[w-1] + sptr[w+1]);
            dptr[1].p[2] = 0.5f * (sptr[-w+1] + sptr[w+1]);
            dptr[w].p[2] = 0.5f * (sptr[w-1] + sptr[w+1]);
            dptr[w+1].p[2] = sptr[w+1];
          }

        sptr += 2;
        dptr += 2;
      }

      sptr += w;
      dptr += w;
    }

  //case BAYER_BGGR:
  else if(format == BAYER_BGGR || format == BAYER_BGGR)
    for (int y = 0; y < h; y += 2){
      for (int x = 0; x < w; x += 2){
        if (x < 2 || x >= w-2 || y < 2 || y >= h-2)
          {
            const float r = sptr[w+1];
            const float g = 0.5f * (sptr[1] + sptr[w]);
            const float b = sptr[0];

            dptr[0].p[0] = r;
            dptr[1].p[0] = r;
            dptr[w].p[0] = r;
            dptr[w+1].p[0] = r;

            dptr[0].p[1] = g;
            dptr[1].p[1] = g;
            dptr[w].p[1] = g;
            dptr[w+1].p[1] = g;

            dptr[0].p[2] = b;
            dptr[1].p[2] = b;
            dptr[w].p[2] = b;
            dptr[w+1].p[2] = b;
          }
        else
          {
            /*
                    +------+------+------+------+
                    :-w-1  :-w    :-w+1  :-w+2  :
                    :      :      :      :      :
                    :    R :    G :    R :    G :
                    +------+======+======+------+
                    :-1    |0     |1     |2     :
                    :      |      |      |      :
                    :    G |    B |    G |    B :
                    +------+======+======+------+
                    :w-1   |w     |w+1   |w+2   :
                    :      |      |      |      :
                    :    R |    G |    R |    G :
                    +------+======+======+------+
                    :w*2-1 :w*2   :w*2+1 :w*2+2 :
                    :      :      :      :      :
                    :    G :    B :    G :    B :
                    +------+------+------+------+
            */

            dptr[0].p[0] = 0.25f * (sptr[-w-1] + sptr[-w+1] + sptr[w-1] + sptr[w+1]);
            dptr[1].p[0] = 0.5f * (sptr[-w+1] + sptr[w+1]);
            dptr[w].p[0] = 0.5f * (sptr[w-1] + sptr[w+1]);
            dptr[w+1].p[0] = sptr[w+1];

            dptr[0].p[1] = 0.25f * (sptr[-w] + sptr[-1] + sptr[1] + sptr[w]);
            dptr[1].p[1] = 0.5f * sptr[1] + 0.125f * (sptr[-w] + sptr[-w+2] + sptr[w] + sptr[w+2]);
            dptr[w].p[1] = 0.5f * sptr[w] + 0.125f * (sptr[-1] + sptr[1] + sptr[w*2-1] + sptr[w*2+1]);
            dptr[w+1].p[1] = 0.25f * (sptr[1] + sptr[w] + sptr[w+2] + sptr[w*2+1]);

            dptr[0].p[2] = sptr[0];
            dptr[1].p[2] = 0.5f * (sptr[0] + sptr[2]);
            dptr[w].p[2] = 0.5f * (sptr[0] + sptr[w*2]);
            dptr[w+1].p[2] = 0.25f * (sptr[0] + sptr[2] + sptr[w*2] + sptr[w*2+2]);
          }

        sptr += 2;
        dptr += 2;
      }

      sptr += w;
      dptr += w;
    }
  else
    {
      LFATAL("error in bayer format math");
    }

  return res;
}

template Image<PixRGB<byte> > debayerREG(Image<byte> src, BayerFormat format);
template Image<PixRGB<uint16> > debayerREG (Image<uint16> src, BayerFormat format);

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
