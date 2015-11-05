/*!@file Channels/HueChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/HueChannel.H $
// $Id: HueChannel.H 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef HUECHANNEL_H_DEFINED
#define HUECHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"

// ######################################################################
//! A hue similarity channel
/*! Hue is represented as a bivariate Gaussian distibution in
  the C.I.E. color space, which is R/I vs. G/I.*/
class HueChannel : public SingleChannel
{
public:

  //! Constructor with parameters of the bivariate Gaussian
  /*!@param mgr the ModelManager to assign this channel to
    @param muR the mean of the distribution in the R/I coordinate
    @param muG the mean in the G/I coordinate of the C.I.E. color space
    @param sigR the std in R/I
    @param sigG the std in G/I
    @param rho the correlation between R/I and G/I */
  HueChannel(OptionManager& mgr,
             float muR, float muG,
             float sigR, float sigG, float rho);

  //! Constuctor with sample pixel and std
  /*! Simplified version of the constructor that derives
    muR and muG from the pixel value hue, and it assumes
    that sigR=sigG=sig, and rho = 0.*/
  HueChannel(OptionManager& mgr,
             const PixRGB<byte> hue,
             float sig = 0.04);

  //! virtual destructor for inheritance
  virtual ~HueChannel();

  //! Combine all feature maps into a single output map
  /*! This overrides SingleChannel::combineSubMaps.
    Instead of doing center-surround and combining across scales,
    we just return the hue-filtered image at the map
    resolution.*/
  Image<float> combineSubMaps();

  //! always returns 1
  virtual uint numSubmaps() const;

  //! This is just a caching wrapper around computeSubmap()
  virtual Image<float> getSubmap(const uint index) const;

  //! Always returns "Hue"
  virtual std::string getSubmapName(const uint index) const;


 protected:
  //! special input function to compute the hue distance map
  virtual void doInput(const InputFrame& inframe);

  float itsMuR, itsMuG, itsSigR, itsSigG, itsRho;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // HUECHANNEL_H_DEFINED
