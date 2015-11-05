/*! @file Image/SimpleFont.H simple raster font definition */

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


// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/SimpleFont.H $
// $Id: SimpleFont.H 8117 2007-03-15 21:06:28Z rjpeters $

#ifndef IMAGE_SIMPLEFONT_H_DEFINED
#define IMAGE_SIMPLEFONT_H_DEFINED

#include "Util/Types.H" // for uint

#include <cstddef> // for size_t

//! A simple font class
/*! This is very impoverished by today's standards... Our font
  definitions only support fixed width and consist of the 95
  characters (the first one is a space):

  \pre
 !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
  \endpre

  as created by AppMedia/app-font2c.C from an image of these
  characters. See writeText() in Image/DrawOps.H for a function that
  uses fonts. */
class SimpleFont {
public:
  //! Constructor
  SimpleFont(const uint cw, const uint ch, const unsigned char *cdata);

  //! Destructor
  ~SimpleFont();

  //! Pseudo-constructor to get a font with fixed width
  /*! Supported widths include: 6, 7, 8, 9, 10, 14 and 20. */
  static SimpleFont FIXED(const uint width);

  //! Get a standard font with width <= maxwidth
  /*! If no font satisfies this criterion, return the smallest
      available font */
  static SimpleFont fixedMaxWidth(const uint maxwidth);

  //! Get a standard font with height <= maxheight
  /*! If no font satisfies this criterion, return the smallest
      available font */
  static SimpleFont fixedMaxHeight(const uint maxheight);

  //! Return the n'th standard font (n must be less than numStdFonts())
  static SimpleFont stdFont(const size_t n);

  //! Return the number of standard fonts
  static size_t numStdFonts();

  //! Helper function to get a pointer to a specific char
  /*! a Pointer to 'space' is returned if c < 32 or c > 126 */
  const unsigned char* charptr(const int c) const;

  //! Width of each char, in pixels
  uint w() const { return ww; }

  //! Height of each char, in pixels
  uint h() const { return hh; }

  //! Equality operator; return true if the two fonts share the same data
  bool operator==(const SimpleFont& f) const
  { return this->ww == f.ww && this->hh == f.hh && this->data == f.data; }

  //! Inequality operator
  bool operator!=(const SimpleFont& f) const
  { return !(*this == f); }

private:
  //! Width of each char, in pixels
  uint ww;

  //! Height of each char, in pixels
  uint hh;

  //! Pointer to the data
  const unsigned char* data;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
