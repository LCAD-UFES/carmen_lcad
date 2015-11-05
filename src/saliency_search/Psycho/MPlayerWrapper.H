/*!@file Psycho/MPlayerWrapper.H Wrapper class for playing videos in MPlayer */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/MPlayerWrapper.H $
// $Id: MPlayerWrapper.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef PSYCHO_MPLAYERWRAPPER_H_DEFINED
#define PSYCHO_MPLAYERWRAPPER_H_DEFINED

#include "nub/ref.h"
#include "Component/EventLog.H"
#include "Psycho/PsychoDisplay.H"
#include "Component/ModelComponent.H"
#include "Component/ModelManager.H"
#include <vector>
#include <string>
#include "rutz/pipe.h"

/// Base class for simple input-output streaming applications
class MPlayerWrapper : public ModelComponent
{
public:

  MPlayerWrapper(OptionManager &mgr,
                const std::string &descrName="Psycho Display",
                const std::string &tagName="PsychoDisplay");

  virtual ~MPlayerWrapper();

  int run(const int argc, const char** argv,
          const char* extraArgsDescription = "",
          const int minExtraArgs = 0, const int maxExtraArgs = 1);

  int runfromSDL(nub::soft_ref<PsychoDisplay> D);

  void setSourceVideo(std::string fname);

  void setOverDisplay(nub::soft_ref<PsychoDisplay> pd);

  //! push an event to the log, if uselinfo is true also display to the screen
  void pushEvent(const std::string& msg, const bool& uselinfo = false);

  void setEventLog(nub::soft_ref<EventLog> log) {itsEventLog = log;}
protected:

private:
  MPlayerWrapper(const MPlayerWrapper&); // not implemented
  MPlayerWrapper& operator=(const MPlayerWrapper&); // not implemented

  int tryRun(const int argc, const char** argv,
             const char* extraArgsDescription,
             const int minExtraArgs, const int maxExtraArgs);

  nub::ref<rutz::exec_pipe> createMPlayerPipe();

  void parseStatusLine(const std::string stat_line);

  nub::soft_ref<EventLog> itsEventLog;
  std::string itsFileName;
  bool itsPlayingVideo;

  OModelParam<bool> itsIsFullScreen;
  OModelParam<std::string> itsMPlayerPath;
  OModelParam<bool> itsIsSound;
  OModelParam<std::string> itsAudioDriver;
  OModelParam<uint> itsCacheSize;
  OModelParam<double> itsCachePercent;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // AUDIOPROJ_MPLAYERWRAPPER_H_DEFINED
