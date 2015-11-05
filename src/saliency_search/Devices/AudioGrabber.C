/*!@file Devices/AudioGrabber.C Grab audio samples from /dev/dsp */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/AudioGrabber.C $
// $Id: AudioGrabber.C 14970 2011-10-08 15:17:42Z farhan $
//

#include "Devices/AudioGrabber.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Devices/DeviceOpts.H"
#include "Image/Image.H"
#include "Image/MatrixOps.H"
#include "Util/Assert.H"
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
AudioGrabber::AudioGrabber(OptionManager& mgr, const std::string& descrName,
                           const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsDevName(&OPT_AudioGrabberDevice, this), // see Devices/DeviceOpts.{H,C}
  itsBits(&OPT_AudioGrabberBits, this), // see Devices/DeviceOpts.{H,C}
  itsFreq(&OPT_AudioGrabberFreq, this), // see Devices/DeviceOpts.{H,C}
  itsBufsamples(&OPT_AudioGrabberBufSamples, this), // idem
  itsChans(&OPT_AudioGrabberChans, this), // see Devices/DeviceOpts.{H,C}
  itsInterleaved(&OPT_AudioGrabberInterleaved,this), 
  itsFd(-1)
{  }

// ######################################################################
void AudioGrabber::start2()
{
#ifndef HAVE_SYS_SOUNDCARD_H
  LFATAL("Oops! I need to have <sys/soundcard.h>");
#else
  // open and reset device:
  const char *device = itsDevName.getVal().c_str();
  itsFd = open(device, O_RDWR);
  if (itsFd == -1) LFATAL("Cannot open device %s", device);

  // setup grab buffer
  uint bufsiz = itsBufsamples.getVal() * (itsBits.getVal() / 8);
  uint numChans = itsChans.getVal();
  bufsiz *= numChans;
  
  if (bufsiz > 65534) LFATAL("Maximum buffer size of 64k bytes exceeded");

  // configure the device:
  int32 x = (bufsiz << 16) | 1;
  if (ioctl(itsFd, SNDCTL_DSP_SETFRAGMENT, &x) == -1)
    PLFATAL("Cannot SetFragment to %d buffers of %d samples",
            1, itsBufsamples.getVal());
  if (ioctl(itsFd, SOUND_PCM_RESET) == -1)
    PLFATAL("Cannot reset device %s", device);
  if (ioctl(itsFd, SOUND_PCM_SYNC) == -1)
    PLFATAL("Cannot sync device %s", device);

  const int bits = itsBits.getVal();
  if (ioctl(itsFd, SOUND_PCM_WRITE_BITS, &bits) == -1)
    PLFATAL("Cannot set bits to %d", bits);
  
  if (ioctl(itsFd, SOUND_PCM_WRITE_CHANNELS, &numChans) == -1)
    LFATAL("Cannot set number of channels to %d ", numChans);

  if (ioctl(itsFd, SOUND_PCM_SYNC) == -1)
    PLFATAL("Cannot sync device %s", device);
  
  const int freq = itsFreq.getVal();
  if (ioctl(itsFd, SOUND_PCM_WRITE_RATE, &freq) == -1)
    PLFATAL("Cannot set write rate to %d", freq);

  // print some info about the device:
  int rate, channels, nbbits, blocksize;
  if (ioctl(itsFd, SOUND_PCM_READ_CHANNELS, &channels) == -1)
    PLERROR("Cannot read nb channels");
  if (ioctl(itsFd, SOUND_PCM_READ_BITS, &nbbits) == -1)
    PLERROR("Cannot read nb bits");
  if (ioctl(itsFd, SOUND_PCM_READ_RATE, &rate) == -1)
    PLERROR("Cannot read sampling rate");
  if (ioctl(itsFd, SNDCTL_DSP_GETBLKSIZE, &blocksize) == -1)
    PLERROR("Cannot read blocksize");
  LDEBUG("%s: %d Hz, %d ch, %d bits, %db blocks", device, rate,
         channels, nbbits, blocksize);
  LINFO("its rate: %d, its channels: %d, its bits: %d, its block: %d",
        rate, channels, nbbits, blocksize);
  LINFO("Ready to grab...");
#endif // HAVE_SYS_SOUNDCARD_H
}

// ######################################################################
void AudioGrabber::stop1()
{
#ifdef HAVE_SYS_SOUNDCARD_H
  if (ioctl(itsFd, SOUND_PCM_SYNC) == -1)
    PLFATAL("Cannot sync audio device");
  if (ioctl(itsFd, SOUND_PCM_RESET) == -1)
    PLFATAL("Cannot reset audio device");
#endif

  if (itsFd > -1) { close(itsFd); itsFd = -1; }
}

// ######################################################################
AudioGrabber::~AudioGrabber()
{  }

// ######################################################################
template <class T>
void AudioGrabber::grab(AudioBuffer<T>& buf) const
{
  ASSERT(itsFd != -1);
  ASSERT(itsBits.getVal() == sizeof(T) * 8);

  int myBufSamp = itsBufsamples.getVal();
  int myChans  = itsChans.getVal();

  AudioBuffer<T> b(myBufSamp,
                   myChans,
                   float(itsFreq.getVal()),
                   NO_INIT);

  int got = read(itsFd, b.getDataPtr(), b.sizeBytes());
  if (got != int(b.sizeBytes()))
    PLERROR("Error reading from device: got %d of %u requested bytes",
            got, b.sizeBytes());
 
  if(itsInterleaved.getVal())
    {
      ASSERT(myChans > 1);
     
       // data comes in as c1s0 c2s0 ... cNs0, c1s1 c1s2 ... cNs2, etc but we want it transposed for storage into
       // AudioBuffer: c1s0 ... c1sN, c2s0 ... c2sN, etc

      // reorganize data in buf
      Image<T> interleaved;
      interleaved.attach(b.getDataPtr(), myChans, myBufSamp);
      Image<T> chunked = transpose(interleaved); 
          
      AudioBuffer<T> tposed(chunked.getArrayPtr(), myBufSamp, myChans, itsFreq.getVal());
          
      buf = tposed;
      interleaved.detach();

    }
  else
    buf=b;
  
}

// ######################################################################
template <class T>
void AudioGrabber::playBuffer(AudioBuffer<T>& buf) const
{
    ASSERT(itsFd != -1);
    ASSERT(itsBits.getVal() == sizeof(T) * 8);
    
    int status = write(itsFd, buf.getDataPtr(), buf.sizeBytes());
    if (status != int (buf.sizeBytes()))
      PLERROR("Error writing to device!");
    
    /*wait for playback to complete before recording again*/
    status = ioctl(itsFd,SOUND_PCM_SYNC,0);
    if (status == -1)
      PLERROR("SOUND_PCM_SYNC ioctl failed!");
    
}
// template instantiations:
template void AudioGrabber::grab(AudioBuffer<byte>& buf) const;
template void AudioGrabber::grab(AudioBuffer<uint16>& buf) const;
template void AudioGrabber::grab(AudioBuffer<int16>& buf) const;

template void AudioGrabber::playBuffer(AudioBuffer<byte>& buf) const;
template void AudioGrabber::playBuffer(AudioBuffer<uint16>& buf) const;
template void AudioGrabber::playBuffer(AudioBuffer<int16>& buf) const;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
