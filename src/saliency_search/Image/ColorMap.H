/*!@file Image/ColorMap.H Simple colormaps with N colors */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ColorMap.H $
// $Id: ColorMap.H 13687 2010-07-23 02:26:37Z lior $
//

#ifndef IMAGE_COLORMAP_H_DEFINED
#define IMAGE_COLORMAP_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"

//! Simple colormaps with N colors
/*! ColorMap derives from Image< PixRGB<byte> > and hence inherits all
  the associated functions. Here we just provide a bunch of
  constructors. A ColorMap then is merely an Image< PixRGB<byte> > of
  width N and height 1. */
class ColorMap : public Image< PixRGB<byte> > {
public:

  //! Default constructor (empty colormap)
  ColorMap();

  //! Default constructor (uninitialized color values, n entries)
  ColorMap(const int n);

  //! Build from an Image of pixels (used by the pseudo-constructors)
  /*! Will throw a fatal error if cmap has height != 1 */
  ColorMap(const Image< PixRGB<byte> >& cmap);

  //! Destructor
  virtual ~ColorMap();

  //! Pseudo-constructor for a greyscale with n entries
  static ColorMap GREY(const int n = 256);

  //! Pseudo-constructor for a transition from 'from' to 'to' with n entries
  static ColorMap GRADIENT(const PixRGB<byte>& from,
                           const PixRGB<byte>& to,
                           const int n = 256);

  //! Pseudo-constructor for "jet" colormap a la matlab
  static ColorMap JET(const int n = 256);

  //! Pseudo-constructor for "lines" colormap a la matlab
  static ColorMap LINES(const int n = 256);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_COLORMAP_H_DEFINED
