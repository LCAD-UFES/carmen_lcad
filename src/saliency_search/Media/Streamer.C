/*!@file Media/Streamer.C Base class for simple input-output streaming applications */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/Streamer.C $
// $Id: Streamer.C 9841 2008-06-20 22:05:12Z lior $
//

#ifndef MEDIA_STREAMER_C_DEFINED
#define MEDIA_STREAMER_C_DEFINED

#include "Media/Streamer.H"

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Util/Pause.H"
#include "Util/csignals.H"

// ######################################################################
Streamer::Streamer(const char* name)
  :
  itsManager(new ModelManager(name)),
  itsIfs(new InputFrameSeries(*itsManager)),
  itsOfs(new OutputFrameSeries(*itsManager))
{
  itsManager->addSubComponent(itsIfs);
  itsManager->addSubComponent(itsOfs);
}

// ######################################################################
Streamer::~Streamer()
{}

// ######################################################################
int Streamer::run(const int argc, const char** argv,
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
int Streamer::tryRun(const int argc, const char** argv,
                     const char* extraArgsDescription,
                     const int minExtraArgs, const int maxExtraArgs)
{
  volatile int signum = 0;
  catchsignals(&signum);

  if (itsManager->parseCommandLine(argc, argv, extraArgsDescription,
                                   minExtraArgs, maxExtraArgs) == false)
    return(1);

  this->handleExtraArgs(*itsManager);

  itsManager->start();

  itsIfs->startStream();

  int c = 0;

  PauseWaiter p;

  while (true)
    {
      if (signum != 0)
        {
          LINFO("quitting because %s was caught", signame(signum));
          return -1;
        }

      if (itsOfs->becameVoid())
        {
          LINFO("quitting because output stream was closed or became void");
          return 0;
        }

      if (p.checkPause())
        continue;

      const FrameState is = itsIfs->updateNext();
      if (is == FRAME_COMPLETE)
        break;

      GenericFrame input = itsIfs->readFrame();
      if (!input.initialized())
        break;

      const FrameState os = itsOfs->updateNext();

      this->onFrame(input, *itsOfs, itsIfs->frame());

      if (os == FRAME_FINAL)
        break;

      LDEBUG("frame %d", c++);

      if (itsIfs->shouldWait() || itsOfs->shouldWait())
        Raster::waitForKey();
    }

  itsManager->stop();

  return 0;
}

// ######################################################################
void Streamer::addComponent(const nub::ref<ModelComponent>& c)
{
  itsManager->addSubComponent(c);
}

// ######################################################################
OptionManager& Streamer::getManager()
{
  return *itsManager;
}

// ######################################################################
void Streamer::handleExtraArgs(const ModelManager& mgr)
{
  // default implementation is a no-op
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_STREAMER_C_DEFINED
