/*!@file Psycho/MPlayerWrapper.C Wrapper class for playing videos in MPlayer */

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
// Primary maintainer for this file: John Shen <shenjohn at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/MPlayerWrapper.C $
// $Id: MPlayerWrapper.C 13712 2010-07-28 21:00:40Z itti $
//

#ifndef PSYCHO_MPLAYERWRAPPER_C_DEFINED
#define PSYCHO_MPLAYERWRAPPER_C_DEFINED

#include "Psycho/MPlayerWrapper.H"
#include "Component/ComponentOpts.H"
#include "Component/ModelManager.H"
#include "Util/Pause.H"
#include "Util/csignals.H"
#include "Util/Types.H"
#include "GUI/GUIOpts.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "rutz/pipe.h"
#include "nub/ref.h"
#include "Component/ModelOptionDef.H"
#include <stdlib.h>
#include "Psycho/PsychoDisplay.H"
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <vector>
#include <unistd.h>

const ModelOptionCateg MOC_MPLAYER = {
  MOC_SORTPRI_2, "MPlayer-Related Options" };

const ModelOptionDef OPT_MPlayerPath =
  { MODOPT_ARG_STRING, "MPlayerPath", &MOC_MPLAYER, OPTEXP_CORE,
    "Path for the mplayer application.",
    "mplayer-path", '\0', "<std::string>", "/usr/bin/mplayer" };

const ModelOptionDef OPT_MPlayerSound =
  { MODOPT_FLAG, "MPlayerSound", &MOC_MPLAYER, OPTEXP_CORE,
    "Play sound.", "sound", '\0', "", "true" };

const ModelOptionDef OPT_MPlayerAudioDriver =
  { MODOPT_ARG_STRING, "MPlayerAudioDriver", &MOC_MPLAYER, OPTEXP_CORE,
    "Audio driver to send to mplayer.  If sound does not work try alsa/oss.  Other valid options are esd/pulse/jack/nas/sdl/mpegpes/v4l2/pcm.  Setting to null is equivalent to --nosound.", "ao", '\0', "<std::string>", "alsa" };

const ModelOptionDef OPT_MPlayerCacheSize =
  { MODOPT_ARG(uint), "MPlayerCacheSize", &MOC_MPLAYER, OPTEXP_CORE,
    "Amount of memory for precaching the video, in kB.",
    "cachesize", '\0', "<uint>", "16384"};

const ModelOptionDef OPT_MPlayerCachePercent =
  { MODOPT_ARG(double), "MPlayerCachePercent", &MOC_MPLAYER, OPTEXP_CORE,
    "Playback begins when the cache is filled to this percent of the total.",
    "cachepercent", '\0', "<0-99>", "50" };


MPlayerWrapper::MPlayerWrapper(OptionManager &mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsEventLog(),
        itsPlayingVideo(false),
  itsIsFullScreen(&OPT_SDLdisplayFullscreen, this),
  itsMPlayerPath(&OPT_MPlayerPath, this),
  itsIsSound(&OPT_MPlayerSound, this),
  itsAudioDriver(&OPT_MPlayerAudioDriver, this),
  itsCacheSize(&OPT_MPlayerCacheSize, this),
  itsCachePercent(&OPT_MPlayerCachePercent, this)
{
}

// ######################################################################
MPlayerWrapper::~MPlayerWrapper()
{}

// ######################################################################
void MPlayerWrapper::setSourceVideo(std::string fname)
{
  itsFileName = fname;
}

// ######################################################################
void MPlayerWrapper::pushEvent(const std::string& msg, const bool& useLinfo)
{
  if (useLinfo)
    LINFO("%s", msg.c_str());
  if (itsEventLog.isValid())
    itsEventLog->pushEvent(msg);
}

// ######################################################################

//display a movie in
int MPlayerWrapper::run(const int argc, const char** argv,
                        const char* extraArgsDescription,
                        const int minExtraArgs, const int maxExtraArgs)
{
  try
    {
      return this->tryRun(argc, argv, extraArgsDescription,
                          minExtraArgs, maxExtraArgs);
    }
  catch (...)
    {
      REPORT_CURRENT_EXCEPTION;
    }

  return 1;
}


// ######################################################################

//display a movie in mplayer - meant to call through run()
int MPlayerWrapper::tryRun(const int argc, const char** argv,
                           const char* extraArgsDescription,
                           const int minExtraArgs, const int maxExtraArgs)
{
  volatile int signum = 0;
  catchsignals(&signum);

  try
    {
      //playing in mplayer
      pushEvent(std::string("===== Playing movie: ") +
                itsFileName + " =====");

      std::string mp_out;
      rutz::exec_pipe mphandle("r", itsMPlayerPath.getVal().c_str(), "-fs", itsFileName.c_str(), (const char*) 0);

      bool movie_begun = false;
      while(std::getline(mphandle.stream(), mp_out, '\r')) //Read the output from mplayer word by word
        {
          //when the video starts, the status line reads as
          //A:   # V:   # A-V: # ct: #   #/  #   ??% ??% ??,?% 0 0

          movie_begun |= (mp_out.find("A:")!=std::string::npos);

          //send messages through parser
          if(movie_begun)
            parseStatusLine(mp_out);
        }
    }
  catch (...)
    {
      fprintf(stderr,"Mplayer initializer failed.\n");
    }
  //itsManager->stop();

  return 0;
}

// ######################################################################

//run a movie while closing an SDL display
//the movie should display seamlessly after the SDL stops
//in reality, mplayer will rush the first few frames regardless.
//also, the timing is inconsistent

int MPlayerWrapper::runfromSDL(nub::soft_ref<PsychoDisplay> D)
{
  try
    {
      //play movie in mplayer
      //create a pipe which sends mplayer output to a stream
      //NB: mplayer process inherits SDLpriority
      nub::ref<rutz::exec_pipe> mphandle = createMPlayerPipe();
      std::string mp_out;

      ModelComponent * displayManager(D->getParent()); //for some reason a nub_ref crashes the program

      bool status_displayed = false, movie_begun = false, display_closed=false;

      //this is the last line before screen displays, e.g.
      //VO: [vdpau] 1280x720 => 1920x1080 Planar YV12  [fs]
      const std::string firstline = "VO: [";
      while(std::getline(mphandle->stream(), mp_out, '\r'))
        //Read the output from mplayer line by line
        {
          status_displayed = (mp_out.find("A:")!=std::string::npos);
          movie_begun |= (mp_out.find(firstline)!=std::string::npos);

          //if the status line begins stop the SDL display
          if(!display_closed && movie_begun)
            {
              //there may be a rescaling: need to wait before closing SDL display
              pushEvent("===== Movie starting =====");
              //const uint delay = 250000; //in microseconds
              //usleep(delay);

              //remove D from its manager or close it independently
              while(displayManager->hasSubComponent(D) && displayManager != NULL)
                {
                  LINFO("removing subcomponent from %s", D->getParent()->tagName().c_str());
                  displayManager->removeSubComponent(*D);
                  displayManager = D->getParent();
                }
              pushEvent("===== Closing PsychoDisplay: " +
                        D->descriptiveName() + " =====");
              D->stop();
              display_closed = true;
            }

                                        //loop that runs while movie plays
          //send messages through parser
          if(status_displayed && movie_begun)
            {
              //itsEventLog->pushEvent(mp_out.c_str());
              parseStatusLine(mp_out);	
	      itsPlayingVideo = true;
	    }
        }
      //after video restart the PsychoDisplay
      //displayManager->addSubComponent(D);
      itsPlayingVideo = false;
      D->start();
      D->clearScreen();
    }
  catch (...)
    {
      LFATAL("MPlayer initializer failed.");
    }

  return 0;
}

nub::ref<rutz::exec_pipe> MPlayerWrapper::createMPlayerPipe()
{
  const uint maxargs = 20;
  char** arglist = new(char* [maxargs]);
  for (uint i = 0; i < maxargs; i++)
    arglist[i] = new char(50);

  int numargs = 0;
  //explicit casts b/c the pipe only takes char*

  arglist[numargs++] = (char*) (itsMPlayerPath.getVal()).c_str();
  arglist[numargs++] = (char*) itsFileName.c_str();

  //these arguments allow for full screening
  if(itsIsFullScreen.getVal())
    {
      arglist[numargs++] = (char*) "-fs";
      arglist[numargs++] = (char*) "-xineramascreen";
      arglist[numargs++] = (char*) "0";
    }

  if(itsIsSound.getVal()) // sets correct audio output driver
    {
      arglist[numargs++] = (char*) "-ao";
      arglist[numargs++] = (char*) (itsAudioDriver.getVal()).c_str();
    }
  else //no sound
    {
      arglist[numargs++] = (char*) "-nosound";
    }

  //this argument *SHOULD* get rid of tearing artifacts
  //arglist[numargs++] = (char*) "-vsync";

  //trying to get better initial A-V sync
  if(0)
    {
      arglist[numargs++] = (char*) "-autosync";
      arglist[numargs++] = (char*) "30";
    }
  if(0) //allowing dropped frames
    {
      //this allows framedropping for slower displays
      arglist[numargs++] = (char*) "-framedrop";
    }

  //&OPT_InputMPEGStreamPreload is the OModelParam<bool> for this
  if(itsCacheSize.getVal() > 0) //caching
    {
      //this sets the cache size
      arglist[numargs++] = (char*) "-cache";
      std::string cacheStr = toStr<int>(itsCacheSize.getVal());
      arglist[numargs++] = (char*) cacheStr.c_str();

      //this precaches a certain % of the video
      arglist[numargs++] = (char*) "-cache-min";
      std::string percentStr = toStr<double>(itsCachePercent.getVal());
      arglist[numargs++] = (char*) percentStr.c_str();

   }

  arglist[numargs] = (char*) 0; //null-terminate
  for(int i = 0; arglist[i]!=NULL; i++)
       LINFO("arg %d: \"%s\"", i, arglist[i]);
   return nub::ref<rutz::exec_pipe>(new rutz::exec_pipe("r", arglist));
}

void MPlayerWrapper::parseStatusLine(const std::string stat_line)
{
  //parse status line with sscanf...
  //A:   # V:   # A-V: # ct: #   #/  #   ??% ??% ??.?% 0 0
  unsigned int num_frames = 1, num_dropped = 0;
  static unsigned int num_prev_dropped = 0;
  double t_video = 0;

  if(sscanf(stat_line.c_str(),"A: %*f V: %lf A-V: %*f ct: %*f %u/%*u %*s %*s %*s %u %*u",
            &t_video, &num_frames, &num_dropped)==EOF)
    pushEvent("Status line error");
  else //if status line is correctly read
    {
      //for the start of each video, restart dropped frame count
      if(num_frames == 1) num_prev_dropped = 0;

      //see if the number of dropped frames so far increased
      if(num_dropped > num_prev_dropped)
        {
          pushEvent("MPlayerWrapper - " + toStr<int>(num_dropped - num_prev_dropped) + " frames dropped");
          num_prev_dropped = num_dropped;
        }
      //account for drop frames, start frame index from 0
                        //NB: sometimes counter will report back to frame 0 - we prevent this from happening here:
                        if(!(num_frames==1 && itsPlayingVideo))
                        pushEvent("MPlayerWrapper - frame " + toStr<int>(num_frames + num_dropped-1)
                + " at time " + toStr<double>(t_video));
    }

}

#endif
// PSYCHO_MPLAYERWRAPPER_C_DEFINED
