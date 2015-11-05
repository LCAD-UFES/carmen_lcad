/*!@file Channels/SOColorChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SOColorChannel.H $
// $Id: SOColorChannel.H 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef SOCOLORCHANNEL_H_DEFINED
#define SOCOLORCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"

class SORedGreenChannel;
class SOGreenRedChannel;
class SOBlueYellowChannel;
class SOYellowBlueChannel;
class RedChannel;
class GreenChannel;
class BlueChannel;
class YellowChannel;

//######################################################################
//! A single opponent color channel class that combines r/g, g/r, b/y
//! and y/b subchannels.
/*! This is an example of an heterogeneous complex channel where the
  various subchannels have distinct visual features. Thus, the JetSpec
  representation for such complex channel will be a series of
  independent single-channel representations, one for each
  subchannel. */
class SOColorChannel : public ComplexChannel
{
public:
  //! Construct with standard params.
  SOColorChannel(OptionManager& mgr);

  //! Destructor
  virtual ~SOColorChannel();

  //! Get access the single opponent RedGreenChannel subchannel.
  virtual SORedGreenChannel& rg() const;

  //! Get access the single opponent GreenRedChannel subchannel.
  virtual SOGreenRedChannel& gr() const;

  //! Get access to the single opponent BlueYellowChannel subchannel.
  virtual SOBlueYellowChannel& by() const;

  //! Get access to the single opponent YellowBlueChannel subchannel.
  virtual SOYellowBlueChannel& yb() const;

  //! Get access the RedChannel subchannel.
  virtual RedChannel& r() const;

  //! Get access the  GreenChannel subchannel.
  virtual GreenChannel& g() const;

  //! Get access to the BlueChannel subchannel.
  virtual BlueChannel& b() const;

  //! Get access to the YellowChannel subchannel.
  virtual YellowChannel& y() const;

protected:
  NModelParam<float> itsLumThresh; //!< Luminance threshold (see getRGBY())

  //! SOColorChannel requires only color input.
  virtual void doInput(const InputFrame& inframe);

  void setRG(nub::ref<SORedGreenChannel> RG);
  void setGR(nub::ref<SOGreenRedChannel> GR);
  void setBY(nub::ref<SOBlueYellowChannel> BY);
  void setYB(nub::ref<SOYellowBlueChannel> YB);
  void setR(nub::ref<RedChannel> R);
  void setG(nub::ref<GreenChannel> G);
  void setB(nub::ref<BlueChannel> B);
  void setY(nub::ref<YellowChannel> Y);

private:
  nub::ref<SORedGreenChannel> itsRG;
  nub::ref<SOGreenRedChannel> itsGR;
  nub::ref<SOBlueYellowChannel> itsBY;
  nub::ref<SOYellowBlueChannel> itsYB;
  nub::ref<RedChannel> itsR;
  nub::ref<GreenChannel> itsG;
  nub::ref<BlueChannel> itsB;
  nub::ref<YellowChannel> itsY;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SOCOLORCHANNEL_H_DEFINED
