/*!@file Channels/FlickerNBackChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/FlickerNBackChannel.H $

#ifndef FLICKERNBACKCHANNEL_H_DEFINED
#define FLICKERNBACKCHANNEL_H_DEFINED

#include "Channels/GaussianPyrChannel.H"
#include "Util/SimTime.H"
#include <deque>

// ######################################################################
//! A temporal flicker channel.
/*! This channel just keeps a memory trace of the previous N frames
  and then does a basic center-surround processing on the difference
  between current and last memorized input. The time constant
  determines how the memorized input should be updated when a new
  input arrives. */
class FlickerNBackChannel : public GaussianPyrChannel
{
public:
  //! Construct with standard specs.
  FlickerNBackChannel(OptionManager& mgr);

  //! Destructor
  virtual ~FlickerNBackChannel();

  //! Reset FlickerNBackChannel
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

protected:
  //! FlickerNBackChannel requires only luminance input.
  virtual void doInput(const InputFrame& inframe);

  /*! Queue length of the channel, in seconds. This
    is used to determine the length of our memory. */
  NModelParam<SimTime> itsQueueLength;

private:
  class ImageHolder
  {
  public:
    ImageHolder(const Image<float>& input, const SimTime& inputtime) :
      itsImage(input), itsTime(inputtime) { };
    const SimTime time() const {return itsTime; };
    const Image<float> image() const {return itsImage; };
  private:
    const Image<float> itsImage;
    const SimTime      itsTime;
  };
  
  std::deque<ImageHolder> itsQueue;
};
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // FLICKERCHANNEL_H_DEFINED
