/*!@file Devices/AudioMixer.C A simple interface to select audio recording source */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/AudioMixer.C $
// $Id: AudioMixer.C 7183 2006-09-20 00:02:57Z rjpeters $
//

#include "Devices/AudioMixer.H"

#include "Component/OptionManager.H"
#include "Devices/DeviceOpts.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#ifdef HAVE_SYS_SOUNDCARD_H
#  include <sys/soundcard.h>
#endif
#include <unistd.h>

// ######################################################################
AudioMixer::AudioMixer(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  DevName(&OPT_AudioMixerDevice, this), // see Devices/DeviceOpts.{H,C}
  InpLine(&OPT_AudioMixerLineIn, this), // see Devices/DeviceOpts.{H,C}
  InpCD(&OPT_AudioMixerCdIn, this), // see Devices/DeviceOpts.{H,C}
  InpMic(&OPT_AudioMixerMicIn, this), // see Devices/DeviceOpts.{H,C}
  fd(-1)
{  }

// ######################################################################
void AudioMixer::start2()
{
#ifndef HAVE_SYS_SOUNDCARD_H
  LFATAL("I need to have <sys/soundcard.h>");
#else
  // open the mixer device
  fd = open(DevName.getVal().c_str(), O_RDWR);
  if (fd == -1) PLFATAL("Cannot open %s", DevName.getVal().c_str());

  // select the recording input
  int mask = 0;
  if (InpLine.getVal()) mask |= SOUND_MASK_LINE;
  if (InpCD.getVal()) mask |= SOUND_MASK_CD;
  if (InpMic.getVal()) mask |= SOUND_MASK_MIC;
  if (ioctl(fd, SOUND_MIXER_WRITE_RECSRC, &mask) == -1)
    PLERROR("Cannot select recording source");
  else
    LINFO("Recording sources: Line: %s, CD: %s, Mic: %s",
          InpLine.getVal() ? "on" : "off",
          InpCD.getVal() ? "on" : "off",
          InpMic.getVal() ? "on" : "off");
#endif // HAVE_SYS_SOUNDCARD_H
}

// ######################################################################
void AudioMixer::stop1()
{
  close(fd); fd = -1;
}

// ######################################################################
AudioMixer::~AudioMixer()
{  }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
