/*!@file Channels/DKLcolorChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DKLcolorChannel.H $
// $Id: DKLcolorChannel.H 15059 2011-11-11 00:55:58Z dberg $
//

#ifndef DKLCOLORCHANNEL_H_DEFINED
#define DKLCOLORCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Channels/DKLTypes.H"

class DcolorChannel;
class KcolorChannel;
class LcolorChannel;

// ######################################################################
//! A DKL color channel
/*! This is an example of an heterogeneous complex channel where the
  various subchannels have distinct visual features. Thus, the JetSpec
  representation for such complex channel will be a series of
  independent single-channel representations, one for each
  subchannel. */
class DKLcolorChannel : public ComplexChannel
{
public:
  //! Construct with standard params.
  DKLcolorChannel(OptionManager& mgr);

  //! Destructor
  virtual ~DKLcolorChannel();

  //! Get access the Dcolor subchannel
  virtual DcolorChannel& Dcolor() const;

  //! Get access the Kcolor subchannel
  virtual KcolorChannel& Kcolor() const;

  //! Get access the Lcolor subchannel
  virtual LcolorChannel& Lcolor() const;

protected:
  //! DKLcolorChannel requires only color input.
  virtual void doInput(const InputFrame& inframe);

  OModelParam<float> itsDweight;
  OModelParam<float> itsKweight;
  OModelParam<float> itsLweight;
  OModelParam<DKLType> itsDKLTypeParam;

  //! get started
  void start1();

private:
  nub::ref<DcolorChannel> itsD;
  nub::ref<KcolorChannel> itsK;
  nub::ref<LcolorChannel> itsL;

  DKLType itsDKLType;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DKLCOLORCHANNEL_H_DEFINED
