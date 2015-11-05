/*!@file Image/Normalize.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Normalize.H $
// $Id: Normalize.H 8029 2007-03-02 01:28:22Z rjpeters $
//

#ifndef IMAGE_NORMALIZE_H_DEFINED
#define IMAGE_NORMALIZE_H_DEFINED

template <class T> class Image;
template <class T> class PixRGB;

/*! @name Flags for float normalization */
//@{

//! Normalize a float image into the range 0..255
const int FLOAT_NORM_0_255      = 1 << 0;

//! Display the pre-normalization float range to the user somehow
/*! The mode of this display may depend on the exact FrameOstream
    subclass: it might display the original range on the console, or
    it draw the original range into the new image, or it might display
    the range in a GUI text box, etc. */
const int FLOAT_NORM_WITH_SCALE = 1 << 1;

//! Try to preserve the full floating-point precision, if possible
/*! For example, in the case of raster files, this flag would be
    expected to cause the image to be saved in some format with
    greater than 8-bit precision. */
const int FLOAT_NORM_PRESERVE   = 1 << 2;

//@}

//! Normalize a grayscale float image, respecting the passed flags.
Image<float> normalizeFloat(const Image<float>& image, const int flags);

//! Normalize an rgb float image, respecting the passed flags.
Image<PixRGB<float> >
normalizeFloatRgb(const Image<PixRGB<float> >& image,
                  const int flags);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_NORMALIZE_H_DEFINED
