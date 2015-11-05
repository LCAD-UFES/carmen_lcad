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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/CIELabChannel.H $
// $Id: CIELabChannel.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef CIELABCHANNEL_H_DEFINED
#define CIELABCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Channels/ColorComputeTypes.H"

class Hue1Channel;
class Hue2Channel;
class ValueIntensityChannel;

// ######################################################################
//! A double opponent color channel class that combines r/g, b/y subchannels.
/*! This is an example of an heterogeneous complex channel where the
  various subchannels have distinct visual features. Thus, the JetSpec
  representation for such complex channel will be a series of
  independent single-channel representations, one for each
  subchannel.

 */
class CIELabChannel : public ComplexChannel
{
public:
  //! Construct with standard params.
  CIELabChannel(OptionManager& mgr);

  //! Destructor
  virtual ~CIELabChannel();

  //! Get access the Luminance subchannel.
  virtual ValueIntensityChannel& L() const;

  //! Get access to the Red-Green Hue subchannel.
  virtual Hue1Channel& A() const;

  //! Get access to the Blue-Yellow Hue subchannel.
  virtual Hue2Channel& B() const;

protected:
  //! ColorChannel requires only color input.
  virtual void doInput(const InputFrame& inframe);

  /// Install a different value intensity channel
  void setL(nub::ref<ValueIntensityChannel> L);

  /// Install a different H1 red/green channel
  void setA(nub::ref<Hue1Channel> A);

  /// Install a different H2 blue/yellow channel
  void setB(nub::ref<Hue2Channel> B);

private:
  Image<float> itsLimg, itsAimg, itsBimg;
  nub::ref<ValueIntensityChannel> itsL;
  nub::ref<Hue1Channel>           itsA;
  nub::ref<Hue2Channel>           itsB;

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // H2SVCHANNEL_H_DEFINED
