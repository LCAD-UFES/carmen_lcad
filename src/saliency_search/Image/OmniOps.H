/*!@file Image/OmniOps.H Operations for omnidirectional correction
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/OmniOps.H $
// $Id: OmniOps.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef IMAGE_OMNI_H_DEFINED
#define IMAGE_OMNI_H_DEFINED

#include "Util/Types.H"

template <class T> class Image;
template <class T> class PixRGB;


//! Perform a low pass filter about a specific pixel
/*! @param x The x coord of the pixel
  @param y The y coord of the pixel
  @param radius The radius of the filter
*/
template <class T>
void lowPassPixel(Image<T>& src,
                  const int x, const int y, const int radius);

//! First step of omni directional correction
/*! @param image The image your are going to process
  @param Rx The Radius in X - May be equal to Ry
  @param Ry The Radius in Y - May be equal to Rx
  @param Xc The center pixel x
  @param Yc The center pixel y
  @param RA Manual radius adjustment, this is used only for distortion
  adjustment.
  @note omniCorrectSp must be used next for full correction unless
  image is a true pinhole image. */
template <class T>
Image<PixRGB<T> > omniCorrectGen(const Image<PixRGB<T> >& src,
                                 const int Rx, const int Ry,
                                 const int Xc, const int Yc,
                                 const int RA = 0);

//! Second step of omni-directional correcton
/*! Values for r,h,k must be obtained via measurement and derived via
  circular regression (see Mundhenk et al SPIE 2000)
  Note: run omniCorrectGen first.
  @param image The image your are going to process
  @param r The radius calculated by circular regression
  @param hh The Circle center coord h (i.e. x)
  @param kk The Circle center coord k (i.e. y)
  @param Rx The Radius in X - May be equal to Ry
  @param Ry The Radius in Y - May be equal to Rx
  @param Xc The center pixel x
  @param Yc The center pixel y
*/
template <class T>
Image<PixRGB<T> > omniCorrectSp(const Image<PixRGB<T> >& src,
                                const float r, const float hh,
                                const float kk, const int Rx, const int Ry,
                                const int Xc, const int Yc);

//! Denebulize a color Image using targeted low pass filtering
/*! @param image a standard Image
  @param radius The size of the filter
*/
template <class T>
Image<PixRGB<T> > omniDenebulize(const Image< PixRGB<T> >& src,
                                 const int radius);

//! Denebulize a byte Image using targeted low pass filtering
/*! @param image a standard Image
  @param radius The size of the filter
*/
template <class T>
Image<T> omniDenebulize(const Image<T>& image, const int radius);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_OMNI_H_DEFINED
