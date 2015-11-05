/*!@file Image/LowPassLpt.H low-pass filtering and smoothing functions */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LowPassLpt.H $
//

#ifndef IMAGE_LOWPASSLPT_H_DEFINED
#define IMAGE_LOWPASSLPT_H_DEFINED

#include "Util/Promotions.H"

class Dims;
template <class T> class Image;

//There are three possible border conditions for space variant images
//based on radial transforms. Most such transforms will store the
//image in 'rings' (distance from center of image) by 'wedges'
//(angle). It is often convenient to rearange this output image so
//that the fovia is at the center, with angles -PI/2->PI/2 (right
//hemifield) on the right and the other half of space accordingly on
//the left. This arrangement would allow for three reasonable ways
//convolution could be performed.
//
//1) SEPARATE_HEMI
     //No hemifield interactions. Clean (normalized energy) border
     //conditions apply at each hemifild in both directions.
//2) CROSS_HEMI
     //Convolutions will cross hemifields in both the R (ring) and W
     //(wedge) dirctions.
//3) CROSS_ANGLE
     //Convolutions will only cross hemifields in the W (wedge)
     //direction. Clean (normalized energy) border condition will be
     //applied in the R (ring) dircection.
enum BorderPolicy {NONE = 0, SEPARATE_HEMI = 1, CROSS_HEMI = 2,
                   CROSS_ANGLE = 3};


//! Low-pass filter using a 3 or 5 tap binomial
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt(const Image<T_or_RGB>& src, const uint taps = 5,
            const BorderPolicy policy = CROSS_HEMI);


//! Low-pass filter, coeff * [0.25 0.5 0.25], applied separably in R
//('rings') and W ('wedges'). The BorderPolicy argument will determine
//how to handle the borders and hemifields of an lpt image.
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt3(const Image<T_or_RGB>& src,
            const BorderPolicy policy = CROSS_HEMI);

//! Low-pass filter an Lpt image, coeff * [0.25 0.5 0.25], applied in
//! R. Here we don't allow the filter to cross hemidfilds along the R
//! direction. Use LowPass3x in LowPass.H if you want hemifields to
//! interact.
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt3r(const Image<T_or_RGB>& src);

//! Low-pass filter an Lpt image, coeff * [0.25 0.5 0.25], applied in
//! W. Here we allow the filter to cross to the other hemifield when
//! filtering in the W direction so space connects at the vertical
//! meridian. Use LowPass3y in LowPass.H if you dont want to cross the
//! meridian.
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt3w(const Image<T_or_RGB>& src);


//! Low-pass filter an lpt image, coeff * [1 4 6 4 1]/16, applied
//separably in R and W. The policy will determine how to handle the
//borders and hemifields of an lpt image.
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt5(const Image<T_or_RGB>& src,
            const BorderPolicy policy = CROSS_HEMI);

//! Low-pass filter an Lpt image, coef * [1 4 6 4 1]/16  applied in
//! R. Here we don't allow the filter to cross hemidfilds along the R
//! direction. Use LowPass5x in LowPass.H if you want hemifield to
//! interact.
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt5r(const Image<T_or_RGB>& src);

//! Low-pass filter an Lpt image, coef * [1 4 6 4 1]/16 applied in
//! W. Here we allow the filter to cross to the other hemifield when
//! filtering in the W direction so space connects at the vertical
//! meridian. Use LowPass5y in LowPass.H if you dont want to cross the
//! meridian.
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt5w(const Image<T_or_RGB>& src);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LOWPASSLPT_H_DEFINED
