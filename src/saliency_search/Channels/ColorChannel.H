/*!@file Channels/ColorChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ColorChannel.H $
// $Id: ColorChannel.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef COLORCHANNEL_H_DEFINED
#define COLORCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Channels/ColorComputeTypes.H"

class RedGreenChannel;
class BlueYellowChannel;
class GreenRedChannel;
class YellowBlueChannel;


// ######################################################################
//! A double opponent color channel class that combines r/g, b/y subchannels.
/*! This is an example of an heterogeneous complex channel where the
  various subchannels have distinct visual features. Thus, the JetSpec
  representation for such complex channel will be a series of
  independent single-channel representations, one for each
  subchannel. */
class ColorChannel : public ComplexChannel
{
public:
  //! Construct with standard params.
  ColorChannel(OptionManager& mgr);

  //! Destructor
  virtual ~ColorChannel();

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Get access the RedGreenChannel subchannel.
  virtual RedGreenChannel& rg() const;

  virtual GreenRedChannel& gr() const;

  //! Get access to the BlueYellowChannel subchannel.
  virtual BlueYellowChannel& by() const;

  virtual YellowBlueChannel& yb() const;

protected:
  NModelParam<float> itsLumThresh; //!< Luminance threshold (see getRGBY())
  OModelParam<ColorComputeType> itsMethod;

  //! ColorChannel requires only color input.
  virtual void doInput(const InputFrame& inframe);

  /// Install a different red/green channel
  void setRG(nub::ref<RedGreenChannel> rg);

  /// Install a different red/green channel
  void setGR(nub::ref<GreenRedChannel> gr);

  /// Install a different blue/yellow channel
  void setBY(nub::ref<BlueYellowChannel> by);

  /// Install a different blue/yellow channel
  void setYB(nub::ref<YellowBlueChannel> yb);

private:
  nub::soft_ref<RedGreenChannel> itsRG;
  nub::soft_ref<BlueYellowChannel> itsBY;
  nub::soft_ref<GreenRedChannel> itsGR;
  nub::soft_ref<YellowBlueChannel> itsYB;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COLORCHANNEL_H_DEFINED
