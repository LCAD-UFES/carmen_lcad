/*!@file SpaceVariant/ScaleSpaceOps.H */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/ScaleSpaceOps.H $

#ifndef IMAGE_SCALESPACEOPS_H_DEFINED
#define IMAGE_SCALESPACEOPS_H_DEFINED

#include "Image/Image.H"
#include "Util/Promotions.H"

#include <utility> //for pair

//forward declare 
template <class T> class ImageSet;

// ######################################################################
/*! A class to represent a template of a local edge at a specific orientaton
  and length and with a specific dinsity of sub regions. 
  The spaceing is set for difference of gaussians responses where the 
  centers are allowed to be offset if smult < 1.0 else
  the spacing is set suitable for sub regions which are made of 
  center-surround LGN type resonses. Orientation should be in 
  radians, and all other values in pixels. Its best, but not reqired if 
  length is odd. 

The optimal offset for a given std_center and smult was determined by fitting
difference of Gaussians (with different offsets) to 1 period of a sin function 
for different frequencies. A function offset=c*std_center fit this function quite nicely

For the edge made of two DoG's things are a little more complicated and the best
parameters to a sin function form a surface in 3D (std_center, smult, offset). 
First for each surround_multiplier we fit a linear function to find the best offset as above. Then 
fit a function of surround_multiplier over 'c' variable as the sum of two exponentials. Once 
some initial paramters were estimated we fit the whole surface to obtain 
:offset=(a*exp(surround_mult*b) + c*exp(surround_mult*d) + e * std_center) where a-e are constants
*/
// ######################################################################
struct LocalEdge
{
  LocalEdge(const Dims& inp_dims, const float& std_center, const float& smult, const float& orientation, 
            const uint& length, const uint density);
  
  LocalEdge();
  
  ~LocalEdge();
  
  typedef std::vector<std::pair<float,float> > EdgeTemplate;
  EdgeTemplate onReg, offReg;
};

// ######################################################################
// ######################################################################
// scale space functions
// ######################################################################
// ######################################################################
// create a scale space up to the desired blur parameterized by the 
// standard deviation of the gaussian filter. From Crowley, '03. 
// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
getScaleSpace(const Image<T_or_RGB>& input, const float& max_std);

// ######################################################################
template <class T_or_RGB>
void addScale(ImageSet<T_or_RGB>& pyr, const float& max_std);

// ######################################################################
//! Get value of a pixel from a dyadic pyramid with trilinear interpolation
// ######################################################################
template <class T_or_RGB>
T_or_RGB getScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, const float& std);

// ######################################################################
//! Get value of a difference of scales pixel from a dyadic pyramid with trilinear interpolation
// ######################################################################
template <class T_or_RGB>
typename promote_trait<T_or_RGB, float>::TP
getDiffScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, const float& std_center, const float& std_surround, const bool on_center = true);

// ######################################################################
//! Get value of a division of scales pixel from a dyadic pyramid with trilinear interpolation
// ######################################################################
template <class T_or_RGB>
typename promote_trait<T_or_RGB, float>::TP
getDivScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, const float& std_center, const float& std_surround);

// ######################################################################
/*! Get value of a local edge by pooling nearby on only and off only
    responses, but allowing the centers to differ. The tree shrew 
    apparently creates edge detecting cells in this fashion see
    Hirsh and Martiniz, Trends Neuro Sci, 2006 for a review.
    The result is squared.

    Note: the std should be the same as was used to construct the LocalEdge object
*/
// ######################################################################
template <class T_or_RGB>
typename promote_trait<T_or_RGB, float>::TP
getEdgeScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, const float& std, const LocalEdge& edgemap);

// ######################################################################
/*! Get value of a local edge by pooling nearby difference of scales pixel 
  from a pyramid with trilinear interpolation. The wiring diagram is inspired by: 
  Hirsh and Martiniz, Trends Neuro Sci, 2006 (for a reviw).
  We model the orientation of a pixel as an on-center DiffScaleSpacePixel 
  adjacent to an off-center one, creating a local orientation estimation. 
  When the parameters are chosen correctly this looks simmilar to a odd gabor or
  a third derivitive Gaussian An edge is then modeled as a length of local co-oriented local 
  orientations at a specific density. The result is squared.

  Note: the std and surround_mult should be the same as was used to construct the LocalEdge object
*/
// ######################################################################
template <class T_or_RGB>
typename promote_trait<T_or_RGB, float>::TP
getEdgeScaleSpacePixel(const ImageSet<T_or_RGB>& pyr, const float& x, const float& y, const float& std_center, const float& std_surround, const LocalEdge& edgemap);

#endif // IMAGE_SCALESPACEOPS_H_DEFINED
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */


