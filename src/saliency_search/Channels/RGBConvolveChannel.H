/*!@file Channels/RGBConvolveChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/RGBConvolveChannel.H $
// $Id: RGBConvolveChannel.H 8643 2007-07-27 18:15:43Z rjpeters $
//

#ifndef RGBCONVOLVECHANNEL_H_DEFINED
#define RGBCONVOLVECHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"
#include "Image/Convolutions.H"

// ######################################################################
//! A generic convolution channel using color filters.
class RGBConvolveChannel : public SingleChannel
{
public:
  //! Construct with standard params. Will need to set the 3 filters.
  RGBConvolveChannel(OptionManager& mgr);

  //! Set the 3 filters to use for convolutions
  void setFilters(const Image<float>& rfilter,
                  const Image<float>& gfilter,
                  const Image<float>& bfilter,
                  ConvolutionBoundaryStrategy boundary);

  //! Destructor
  virtual ~RGBConvolveChannel();

protected:
  virtual void doInput(const InputFrame& inframe);

  //! compute from a color image
  virtual ImageSet<float> computePyramid(const Image< PixRGB<float> >& img,
                                         const rutz::shared_ptr<PyramidCache<float> >& cache);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // RGBCONVOLVECHANNEL_H_DEFINED
