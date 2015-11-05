/*!@file Devices/RadioDecoder.C Decode radio pulse-width-modulated signals */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/RadioDecoder.C $
// $Id: RadioDecoder.C 14697 2011-04-08 21:34:48Z farhan $
//

#include "Devices/RadioDecoder.H"

#include "Component/ParamMap.H"
#include "Devices/AudioGrabber.H"
#include "Devices/AudioMixer.H"
#include "Devices/DeviceOpts.H"
#include "Util/Assert.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <cstdio>
#include <cmath>

// half window size, in samples:
#define RD_WIN 2

// values higher than this are considered one:
#define RD_THRESH (255 - 20)

// min and max radio pulse width, in samples:
#define MINRDPULSE 30
#define MAXRDPULSE 150

void* RadioDecoder_run(void *r0); // will live in a separate thread

// ######################################################################
void* RadioDecoder_run(void *r0)
{
  RadioDecoder *r = (RadioDecoder *)r0;
  r->run(); return NULL;
}

// ######################################################################
RadioDecoder::RadioDecoder(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName)
{
#ifndef HAVE_SYS_SOUNDCARD_H
  CLFATAL("Oops! I can't run without <sys/soundcard.h>");
#else
  running = false;

  agb = nub::soft_ref<AudioGrabber>
    (new AudioGrabber(mgr, "RadioDecoder Grabber", "RadioDecoderGrabber"));

  // Feed our own default values to the grabber, to replace its defaults
  agb->setModelParamVal("AudioGrabberFreq", uint(48000));
  agb->setModelParamVal("AudioGrabberChans", 1);
  agb->setModelParamVal("AudioGrabberBufSamples", uint(1000));

  // Create and attach our mixer and grabber as subcomponents:
  nub::soft_ref<AudioMixer>
    mix(new AudioMixer(mgr, "RadioDecoder Mixer", "RadioDecoderMixer"));
  addSubComponent(mix);
  addSubComponent(agb);

  // By default, all of the command-line options of our newly-minted
  // AudioGrabber and RadioDecoder will be exported. However, in this
  // case we don't want that to happen, as we need the grabber and
  // mixer to be in a specific state for us to work, and we don't want
  // people messing with them; so, we explicit forget those guys as
  // command-line options, and then we will a hand-pick a few options
  // (typically, device names and recording inputs, but not recording
  // frequency and others) to export that can be safely twiddled by
  // the user.
  agb->forgetExports();
  mix->forgetExports();

  agb->doRequestOption(&OPT_AudioGrabberDevice);
  agb->doRequestOption(&OPT_AudioGrabberChans, true);
  mix->doRequestOption(&OPT_AudioMixerDevice);
  mix->doRequestOption(&OPT_AudioMixerLineIn);
  mix->doRequestOption(&OPT_AudioMixerCdIn);
  mix->doRequestOption(&OPT_AudioMixerMicIn);
#endif // HAVE_SYS_SOUNDCARD_H
}

// ######################################################################
void RadioDecoder::start1()
{
#ifndef HAVE_SYS_SOUNDCARD_H
  CLFATAL("Oops! I can't run without <sys/soundcard.h>");
#else
  const bool stereo = agb->getModelParamVal<uint>("AudioGrabberChans");
  nch = stereo ? 2 : 1;

  // setup our internals:
  zero = new rutz::shared_ptr<NModelParam<float> >[nch];
  posmult = new rutz::shared_ptr<NModelParam<float> >[nch];
  negmult = new rutz::shared_ptr<NModelParam<float> >[nch];
  servoval = new float[nch];
  for (int i = 0; i < nch; i ++)
    {
      servoval[i] = 0.0F; char buf[20];

      sprintf(buf, "Zero%d", i);
      zero[i] = NModelParam<float>::make(buf, this, 0.0F);

      sprintf(buf, "PosMult%d", i);
      posmult[i] = NModelParam<float>::make(buf, this, 1.0F);

      sprintf(buf, "NegMult%d", i);
      negmult[i] = NModelParam<float>::make(buf, this, 1.0F);
    }
#endif // HAVE_SYS_SOUNDCARD_H
}

// ######################################################################
void RadioDecoder::start2()
{
  // start thread for run():
  pthread_create(&runner, NULL, &RadioDecoder_run, (void *)this);
}

// ######################################################################
void RadioDecoder::stop1()
{
  // stop our thread:
  running = false; while(running == false) usleep(5);
  usleep(50); running = false;

  delete [] servoval; servoval = 0;
  delete [] zero;     zero = 0;
  delete [] posmult;  posmult = 0;
  delete [] negmult;  negmult = 0;
}

// ######################################################################
RadioDecoder::~RadioDecoder()
{ }

// ######################################################################
float RadioDecoder::getVal(const int channel) const
{
  ASSERT(channel >= 0 && channel < nch);
  float val;
  if (servoval[channel] >= zero[channel]->getVal())
    {
      val = (servoval[channel] - zero[channel]->getVal()) *
        posmult[channel]->getVal();
      if (val >= 1.0F) val = 0.999999F;
    }
  else
    {
      val = (servoval[channel] - zero[channel]->getVal()) *
        negmult[channel]->getVal();
      if (val <= -1.0F) val = -0.999999F;
    }
  return val;
}

// ######################################################################
void RadioDecoder::zeroCalibrate(const int nbiter)
{
  LINFO("Starting zero calibration. Put controls to rest.");
  float avg[nch]; for (int i = 0; i < nch; i ++) avg[i] = 0.0F;

  // somehow it takes some time for the signal to be good; maybe some
  // automatic gain control in the sound card?
  usleep(2000000);

  for (int i = 0; i < nbiter; i ++)
    {
      for (int j = 0; j < nch; j ++) avg[j] += servoval[j];
      usleep(30000);
    }
  for (int i = 0; i < nch; i ++) zero[i]->setVal(avg[i] / ((float)nbiter));
  LINFO("Zero calibration done.");
}

// ######################################################################
void RadioDecoder::rangeCalibrate(const int nbiter)
{
  LINFO("Range calibration: Full-swing all controls now!");
  float pmax[nch], nmax[nch];
  for (int i = 0; i < nch; i ++) { pmax[i] = -10000.0; nmax[i] = 10000.0; }

  for (int i = 0; i < nbiter; i ++)
    {
      for (int j = 0; j < nch; j ++)
        {
          float x = servoval[j];  // avoid value change while we work on it
          if (x > pmax[j]) pmax[j] = x;
          if (x < nmax[j]) nmax[j] = x;
        }
      usleep(20000);
    }
  for (int i = 0; i < nch; i ++)
    {
      if (fabs(pmax[i] - zero[i]->getVal()) < 0.001F)
        {
          LERROR("ZERO positive range? Setting multiplier to 1");
          posmult[i]->setVal(1.0F);
        }
      else
        posmult[i]->setVal(0.99999F / (pmax[i] - zero[i]->getVal()));

      if (fabs(zero[i]->getVal() - nmax[i]) < 0.001F)
        {
          LERROR("ZERO negative range? Setting multiplier to 1");
          negmult[i]->setVal(1.0F);
        }
      else
        negmult[i]->setVal(-0.99999F / (nmax[i] - zero[i]->getVal()));
    }
  LINFO("Range calibration done.");
}

// ######################################################################
void RadioDecoder::run()
{
#ifndef HAVE_SYS_SOUNDCARD_H
  CLFATAL("Oops! I can't run without <sys/soundcard.h>");
#else
  running = true;
  int scoreStart[nch], idxUp[nch], scoreEnd[nch], idxDown[nch],
    first[nch], middle[nch], last[nch], bestUp[nch], bestDown[nch];

  while(running)
    {
      // grab some data (blocking):
      AudioBuffer<byte> buffer;
      agb->grab(buffer);
      int nsamples = int(buffer.nsamples());

      // slide a window and count how many positive and negative samples
      // are found around its center:
      for (int i = 0; i < nch; i ++) {
        scoreStart[i] = 0; scoreEnd[i] = 0; idxUp[i] = -1; idxDown[i] = -1;
        bestUp[i] = 0; bestDown[i] = 0;
      }
      const byte *bptr = buffer.getDataPtr();

      // initialize scores for initial window:
      for (int i = 0; i < RD_WIN; i ++)
        for (int j = 0; j < nch; j ++)
          {
            if (*bptr > RD_THRESH) scoreStart[j] ++;
            if (bptr[RD_WIN] > RD_THRESH) scoreEnd[j] ++;
            bptr ++;
          }

      int mid = RD_WIN * nch, end = (2 * RD_WIN - 1) * nch;
      bptr = buffer.getDataPtr();
      for (int i = 0; i < nch; i ++)
        {
          if (*bptr > RD_THRESH) first[i] = 1; else first[i] = 0;
          if (bptr[mid] > RD_THRESH) middle[i] = 1; else middle[i] = 0;
          if (bptr[end] > RD_THRESH) last[i] = 1; else last[i] = 0;
          bptr ++;
        }

      // now slide the window and update the scores, subtracting the first
      // value, sliding the middle value, and adding the last value:
      bptr = buffer.getDataPtr() + nch;
      for (int i = RD_WIN + 1; i < nsamples - RD_WIN; i ++)
        for (int j = 0; j < nch; j ++)
          {
            if (idxUp[j] != -1 && scoreStart[j] - scoreEnd[j] > bestDown[j]) {
              bestDown[j] = scoreStart[j] - scoreEnd[j];
              idxDown[j] = i;
              //LDEBUG("down(%d): %d %d %d %d",j,scoreStart[j],scoreEnd[j],
              //     bestDown[j], idxDown[j]);
            }
            if (idxDown[j] == -1 && scoreEnd[j] - scoreStart[j] > bestUp[j]) {
              bestUp[j] = scoreEnd[j] - scoreStart[j];
              idxUp[j] = i;
              //LDEBUG("up(%d): %d %d %d %d",j,scoreStart[j],scoreEnd[j],
              //             bestUp[j], idxUp[j]);
            }

            scoreStart[j] += middle[j] - first[j];
            scoreEnd[j] += last[j] - middle[j];

            if (*bptr > RD_THRESH) first[j] = 1; else first[j] = 0;
            if (bptr[mid] > RD_THRESH) middle[j] = 1; else middle[j] = 0;
            if (bptr[end] > RD_THRESH) last[j] = 1; else last[j] = 0;
            bptr ++;
          }

      // compute pulse width, in samples:
      for (int i = 0; i < nch; i ++)
        {
          //LDEBUG("pulse(%d): %d--%d",idxUp[i],idxDown[i]);
          if (idxDown[i] != -1 && idxUp[i] != -1 && idxUp[i] < idxDown[i] &&
              idxDown[i] - idxUp[i] > MINRDPULSE &&
              idxDown[i] - idxUp[i] < MAXRDPULSE)
            servoval[i] = (float)(idxDown[i] - idxUp[i] + 1);
        }
      // sleep a little:
      usleep(5000);
    }

  // we got an order to stop:
  running = true;  // FIXME: bogus! other thread may unlock too soon
  pthread_exit(0);
#endif // HAVE_SYS_SOUNDCARD_H
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
