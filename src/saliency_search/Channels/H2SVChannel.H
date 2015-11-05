/*!@file Channels/H2SVChannel.H */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/H2SVChannel.H $
// $Id: H2SVChannel.H 11094 2009-04-07 05:59:42Z mundhenk $
//

#ifndef H2SVCHANNEL_H_DEFINED
#define H2SVCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Channels/ColorComputeTypes.H"

class Hue1Channel;
class Hue2Channel;
class SaturationChannel;
class ValueIntensityChannel;

// ######################################################################
//! A double opponent color channel class that combines r/g, b/y subchannels.
/*! This is an example of an heterogeneous complex channel where the
  various subchannels have distinct visual features. Thus, the JetSpec
  representation for such complex channel will be a series of
  independent single-channel representations, one for each
  subchannel.

  Additionally, Saturation and intensity are extracted. Extracting color in
  this way creates an othogonal set of values that are invertable. Thus,
  we can recover R,G and B from H2SV which contains Intensity, Saturation and
  color combined.
 */
class H2SVChannel : public ComplexChannel
{
public:
  //! Construct with standard params.
  H2SVChannel(OptionManager& mgr);

  //! Destructor
  virtual ~H2SVChannel();

  //! Get access the RedGreen Hue1 subchannel.
  virtual Hue1Channel& H1() const;

  //! Get access to the BlueYellow Hue2 subchannel.
  virtual Hue2Channel& H2() const;

  //! Get access the RedGreen Hue1 subchannel.
  virtual SaturationChannel& S() const;

  //! Get access to the BlueYellow Hue2 subchannel.
  virtual ValueIntensityChannel& V() const;

protected:
  //! ColorChannel requires only color input.
  virtual void doInput(const InputFrame& inframe);

  /// Install a different H1 red/green channel
  void setH1(nub::ref<Hue1Channel> h1);

  /// Install a different H2 blue/yellow channel
  void setH2(nub::ref<Hue2Channel> h2);

  /// Install a different saturation channel
  void setS(nub::ref<SaturationChannel> s);

  /// Install a different value intensity channel
  void setV(nub::ref<ValueIntensityChannel> v);

  OModelParam<bool> itsUseH2SV1;

private:
  Image<float> itsH1img, itsH2img, itsSimg, itsVimg;
  nub::ref<Hue1Channel>           itsH1;
  nub::ref<Hue2Channel>           itsH2;
  nub::ref<SaturationChannel>     itsS;
  nub::ref<ValueIntensityChannel> itsV;
  //! Use a sigmoid transfer function to calm color by saturation
  bool                            itsUseSigmoid;
  float                           itsSigAlpha;
  float                           itsSigBeta;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // H2SVCHANNEL_H_DEFINED
