/*!@file Image/ColorMap.C Simple colormaps */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ColorMap.C $
// $Id: ColorMap.C 13690 2010-07-23 19:50:36Z lior $
//

#ifndef IMAGE_COLORMAP_C_DEFINED
#define IMAGE_COLORMAP_C_DEFINED

#include "Image/ColorMap.H"

#include "Image/MatrixOps.H"
#include "Util/log.H"

// ######################################################################
ColorMap::ColorMap() :
  Image< PixRGB<byte> >()
{ }

// ######################################################################
ColorMap::ColorMap(const int n) :
  Image< PixRGB<byte> >(n, 1, NO_INIT)
{ }

// ######################################################################
ColorMap::ColorMap(const Image< PixRGB<byte> >& cmap) :
  Image< PixRGB<byte> >(cmap.getHeight() != 1 ? transpose(cmap) : cmap)
{
  if (this->getHeight() != 1) LFATAL("Invalid colormap image height != 1");
}

// ######################################################################
ColorMap::~ColorMap()
{ }

// ######################################################################
ColorMap ColorMap::GREY(const int n)
{
  Image< PixRGB<byte> > cmap(n, 1, NO_INIT);
  Image< PixRGB<byte> >::iterator ptr = cmap.beginw();
  for (int i = 0; i < n; i++)
    *ptr++ = PixRGB<byte>(byte((i * 256) / n));
  return ColorMap(cmap);
}

// ######################################################################
ColorMap ColorMap::GRADIENT(const PixRGB<byte>& from,
                            const PixRGB<byte>& to,
                            const int n)
{
  Image< PixRGB<byte> > cmap(n, 1, NO_INIT);
  Image< PixRGB<byte> >::iterator ptr = cmap.beginw();

  for (int i = 0; i < n; i++)
    *ptr++ = PixRGB<byte>(PixRGB<float>(from) +
                          (PixRGB<float>(to) - PixRGB<float>(from)) *
                          float(i) / float(n));
  return ColorMap(cmap);
}

// ######################################################################
ColorMap ColorMap::JET(const int m)
{
  Image<PixRGB<byte> > cmap(m, 1, NO_INIT);
  const Image<PixRGB<byte> >::iterator ptr = cmap.beginw();

  const int n = m/4;          // e.g., 64 if cmapsize=256
  const int mid1 = n/2;       // e.g., 32 if cmapsize=256
  const int mid2 = mid1 + n;  // e.g., 96 if cmapsize=256
  const int mid3 = mid2 + n;  // e.g., 160 if cmapsize=256
  const int mid4 = mid3 + n;  // e.g., 224 if cmapsize=256

#define LINMAP(i, a_in, b_in, a_out, b_out)     \
  byte(255.0 * ((a_out)                         \
                + double((i) - (a_in))          \
                * ((b_out)-(a_out))             \
                / ((b_in) - (a_in))))

  for (int i = 0; i < mid1; ++i)
    ptr[i] = PixRGB<byte>(0,
                          0,
                          LINMAP(i, 0, mid1, 0.5, 1.0));

  for (int i = mid1; i < mid2; ++i)
    ptr[i] = PixRGB<byte>(0,
                          LINMAP(i, mid1, mid2, 0.0, 1.0),
                          255);

  for (int i = mid2; i < mid3; ++i)
    ptr[i] = PixRGB<byte>(LINMAP(i, mid2, mid3, 0.0, 1.0),
                          255,
                          LINMAP(i, mid2, mid3, 1.0, 0.0));

  for (int i = mid3; i < mid4; ++i)
    ptr[i] = PixRGB<byte>(255,
                          LINMAP(i, mid3, mid4, 1.0, 0.0),
                          0);

  for (int i = mid4; i < m; ++i)
    ptr[i] = PixRGB<byte>(LINMAP(i, mid4, m, 1.0, 0.5),
                          0,
                          0);

#undef LINMAP

  return ColorMap(cmap);
}

ColorMap ColorMap::LINES(const int m)
{
  Image<PixRGB<byte> > cmap(m, 1, NO_INIT);
  const Image<PixRGB<byte> >::iterator ptr = cmap.beginw();

  byte RValues[7] = {0,   0,   255, 0,   191, 191, 64};
  byte GValues[7] = {0,   128, 0,   191, 0,   191, 64};
  byte BValues[7] = {255, 0,   0,   191, 191, 0,   64};

  if (m > 7*7*7)
    LFATAL("Only %i values are allowed for max", 7*7*7);

  int i=0;
  for(int b=0; b<7; b++) 
    for(int g=0; g<7; g++) 
      for(int r=0; r<7; r++) 
      {
        if (i >= m)
          break;
        ptr[i] = PixRGB<byte>(RValues[r],GValues[g],BValues[b]);
        i++;
      }

  return ColorMap(cmap);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_COLORMAP_C_DEFINED
