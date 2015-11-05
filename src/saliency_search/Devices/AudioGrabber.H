/*!@file Devices/AudioGrabber.H Grab audio samples from /dev/dsp */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/AudioGrabber.H $
// $Id: AudioGrabber.H 14971 2011-10-08 15:30:08Z farhan $
//

#ifndef AUDIOGRABBER_H_DEFINED
#define AUDIOGRABBER_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"
#include "Audio/AudioBuffer.H"

//! Class to grab audio samples from an audio device
/*! For now, grabbing is done using a blocking read() onto the DSP device.*/

class AudioGrabber : public ModelComponent
{
public:
  //! Constructor
  AudioGrabber(OptionManager& mgr,
               const std::string& descrName = "Audio Grabber Driver",
               const std::string& tagName = "AudioGrabber");

  //! Destructor
  ~AudioGrabber();

  //! Grab a buffer
  /* A fatal error will arise if T has a different number of bits than
     itsBits. Typically, T should be byte, int16, etc. If buf was not
     empty its contents will be destroyed. */
  template <class T>
  void grab(AudioBuffer<T>& buf) const;

  //! Play sound from buffer 
  /*  Make sure your device is not just a grabber otherwise it obviously 
      will not play the sound. Maybe we need to move the playing somewhere
      else later but for now its here */
  template <class T>
  void playBuffer(AudioBuffer<T>& buf) const;


protected:
  OModelParam<std::string> itsDevName;    //!< audio device name
  OModelParam<uint>        itsBits;       //!< bits per sample (8 or 16)
  OModelParam<uint>        itsFreq;       //!< sampling frequency (Hz)
  OModelParam<uint>        itsBufsamples; //!< samples per buffer
  OModelParam<uint>        itsChans;     //!<  number of channels 
  OModelParam<bool>        itsInterleaved; //!< channel data interleaved?

  //! get started (see ModelComponent.H)
  void start2();

  //! get stopped (see ModelComponent.H)
  void stop1();

private:
  int itsFd;                               // file descriptor for device
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
