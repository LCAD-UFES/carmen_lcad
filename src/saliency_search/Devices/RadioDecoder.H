/*!@file Devices/RadioDecoder.H Decode radio pulse-width-modulated signals */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/RadioDecoder.H $
// $Id: RadioDecoder.H 7183 2006-09-20 00:02:57Z rjpeters $
//

#ifndef RADIODECODER_H_DEFINED
#define RADIODECODER_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"

#include <pthread.h>
#include <unistd.h>

class AudioGrabber;

//! This class runs a thread which records servo position from sound card
/*! Radio-control servomechanisms are controlled by a
    pulse-width-modulated protocol in which a 1-2ms pulse (from 0V to
    4.8V) is generated every 18-25ms. The width of the pulse (between
    1 and 2ms) determines the position of the servo. This class uses
    an AudioGrabber to record pulses, for example coming from an R/C
    receiver. It then computes the pulse width and converts it to a
    number between -1.0 and 1.0. The current pulse values are
    available via getVal(), which is a non-blocking function call. */

class RadioDecoder : public ModelComponent {
public:
  //! Constructor
  RadioDecoder(OptionManager& mgr,
               const std::string& descrName = "R/C Radio Decoder",
               const std::string& tagName = "RadioDecoder");

  //! Destructor
  ~RadioDecoder();

  //! Calibrate the zero (will block until done)
  void zeroCalibrate(const int nbiter = 20);

  //! Calibrate the positive and negative ranges to +/-1.0
  void rangeCalibrate(const int nbiter = 200);

  //! Access function; values guaranteed within ]-1.0 .. 1.0[
  float getVal(const int channel) const;

  //! This method should not be called (implements thread code)
  void run();

protected:
  rutz::shared_ptr<NModelParam<float> >* zero;    //!< array of zero calibration values
  rutz::shared_ptr<NModelParam<float> >* posmult; //!< array of positive multiplier calibration values
  rutz::shared_ptr<NModelParam<float> >* negmult; //!< array of negative multiplier calibration values

  //! get started (see ModelComponent.H)
  void start1();

  //! get started (see ModelComponent.H)
  void start2();

  //! get stopped (see ModelComponent.H)
  void stop1();

private:
#ifdef HAVE_SYS_SOUNDCARD_H
  nub::soft_ref<AudioGrabber> agb;    // keep explicit pointer to our AudioGrabber
#endif
  int nch;
  mutable float *servoval;
  pthread_t runner;
  mutable bool running;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
