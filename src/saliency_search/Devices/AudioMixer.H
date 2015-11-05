/*!@file Devices/AudioMixer.H A simple interface to select audio recording source */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/AudioMixer.H $
// $Id: AudioMixer.H 7183 2006-09-20 00:02:57Z rjpeters $
//

#ifndef AUDIOMIXER_H_DEFINED
#define AUDIOMIXER_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

//! Simple interface to the Linux audio mixer
/*! To be used to set the recording input. */
class AudioMixer : public ModelComponent {
public:
  //! Constructor
  AudioMixer(OptionManager& mgr,
             const std::string& descrName = "Audio Mixer Driver",
             const std::string& tagName = "AudioMixer");

  //! Destructor
  ~AudioMixer();

protected:
  OModelParam<std::string> DevName; //!< mixer device name
  OModelParam<bool> InpLine;        //!< line input on
  OModelParam<bool> InpCD;          //!< cd input on
  OModelParam<bool> InpMic;         //!< mic input on

  //! get started (see ModelComponent.H)
  void start2();

  //! get stopped (see ModelComponent.H)
  void stop1();

private:
  int fd;                          // file descriptor for audio device
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
