/*!@file Image/fancynorm.C Intrafeature competition with maxNormalize(). */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/fancynorm.C $
// $Id: fancynorm.C 9993 2008-07-29 00:04:18Z lior $
//

#include "ModelNeuron/SCNorm.H"

#include "Util/Assert.H"
#include "Image/Image.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "ModelNeuron/SVFilter.H"


// ######################################################################
// ##### fancyNorm from Itti et al, JEI, 2001 -- FULL implementation:
template <typename T>
Image<T> SCmaxNormalizeFancy(const Image<T>& src, const double esigma, const double isigma, const double weakness)
{
  // Normalize using fancy normalization: multiple iterations of
  // filtering by a DoG
  
  const double FANCYCOEX = 0.5; //!< excitatory coefficient (strength)
  const double FANCYCOIN = 1.5; //!< inhibitory coefficient (strength)
  const double FANCYINHI = 2.0; //!< strength of global inhibition

  ASSERT(src.initialized());
 
  Image<T> result = src;

  if (weakness <= 0.0)
    return result;
 
  // first clamp negative values to zero
  inplaceRectify(result);
 
  const int w = result.getWidth();
  const int h = result.getHeight();
 
  int siz = std::max(w, h);
  int maxhw = std::max(0, std::min(w, h) / 2 - 1);

  // build separable Gaussians for DoG computation:
  float esig = (float(siz) * esigma) * 0.01F;
  float isig = (float(siz) * isigma) * 0.01F;
  Image<float> gExc =
    gaussian<float>(FANCYCOEX/(esig*sqrt(2.0*M_PI)) * weakness, esig, maxhw);
  Image<float> gInh =
    gaussian<float>(FANCYCOIN/(isig*sqrt(2.0*M_PI)) * weakness, isig, maxhw);
 

  Image<T> excit = sepFilterSV(result, gExc, gExc); // excitatory part
  Image<T> inhib = sepFilterSV(result, gInh, gInh); // inhibitory part
  excit -= inhib;
  
  T minim, maxim; getMinMax(result, minim, maxim);
  T globinhi = (T)(0.01 * FANCYINHI * maxim);
  
  result += excit;        // we get broad inhibition from everywhere
  result += -globinhi;    // we get fixed global inhibition
  inplaceRectify(result);
  
  return result;
}

// ######################################################################
#include "inst/ModelNeuron/SCNorm.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
