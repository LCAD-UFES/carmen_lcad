/*!@file Media/SimFrameSeries.C a series of frames as SimModule */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@pollux.usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/SimFrameSeries.C $
// $Id: SimFrameSeries.C 15333 2012-07-18 22:14:20Z dberg $
//

#include "Media/SimFrameSeries.H"

#include "Media/FrameSeries.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimulationOpts.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"



// ######################################################################
// ######################################################################
// ######################################################################

//Used for dynamaicly loaded modules
SIMMODULEINSTFUNC(SimInputFrameSeries);

SimInputFrameSeries::SimInputFrameSeries(OptionManager& mgr,
                                         const std::string& descrName,
                                         const std::string& tagName) :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  itsIFS(new InputFrameSeries(mgr))
{
  addSubComponent(itsIFS);
}

// ######################################################################
SimInputFrameSeries::~SimInputFrameSeries()
{ }

// ######################################################################
void SimInputFrameSeries::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
  // update our FrameSeries:
  const FrameState fs = itsIFS->update(q.now());

  // do we have a new frame?
  if (fs == FRAME_NEXT || fs == FRAME_FINAL)
    {
      // read the frame and post it to our SimEventQueue:
      GenericFrame f = itsIFS->readFrame();
      if (f.initialized())
        {
          rutz::shared_ptr<SimEventInputFrame>
            ev(new SimEventInputFrame(this, f, itsIFS->frame()));
          q.post(ev);

          // now is a great time to display memory usage stats if the
          // user wishes to see them:
          rutz::shared_ptr<SimEventShowMemStats>
            ee(new SimEventShowMemStats(this, itsIFS->frame(),
                                        f.frameSpec().dims.sz()));
          q.post(ee);
        }
    }
  
  // input exhausted?
  if (fs == FRAME_COMPLETE)
    {
      // post a break event:
      rutz::shared_ptr<SimEventBreak>
        ev(new SimEventBreak(this, "Input frames exhausted"));
      q.post(ev);
    }

  // does our FrameSeries wish to wait for user?
  if (itsIFS->shouldWait())
    {
      // post a userwait event:
      rutz::shared_ptr<SimEventUserWait> ev(new SimEventUserWait(this));
      q.post(ev);
    }
}

// ######################################################################
GenericFrameSpec SimInputFrameSeries::peekFrameSpec()
{ return itsIFS->peekFrameSpec(); }

// ######################################################################
void SimInputFrameSeries::startStream()
{ itsIFS->startStream(); }

// ######################################################################
void SimInputFrameSeries::setFrameSource(const std::string& source)
{ itsIFS->setFrameSource(source); }

// ######################################################################
int SimInputFrameSeries::frame() const
{ return itsIFS->frame(); }



// ######################################################################
// ######################################################################
// ######################################################################

//Used for dynamaicly loaded modules
SIMMODULEINSTFUNC(SimOutputFrameSeries);

SimOutputFrameSeries::SimOutputFrameSeries(OptionManager& mgr,
                                           const std::string& descrName,
                                           const std::string& tagName) :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventRequestSaveOutput),
  itsTimeStep(&OPT_SimulationTimeStep, this),
  itsOFS(new OutputFrameSeries(mgr)),
  itsSaveRequested(false)
{
  addSubComponent(itsOFS);
}

// ######################################################################
SimOutputFrameSeries::~SimOutputFrameSeries()
{ }

// ######################################################################
void SimOutputFrameSeries::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& ect)
{
  update(q, itsSaveRequested);
}

// ######################################################################
void SimOutputFrameSeries::
onSimEventRequestSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventRequestSaveOutput>& e)
{
  update(q, true);
}

// ######################################################################
void SimOutputFrameSeries::update(SimEventQueue& q, const bool saveRequested)
{
  // ok, we have a request to save; see whether we should save now:
  const FrameState fs = itsOFS->update(q.now(), saveRequested);

  //Go though our windows and send any keyboard or mouse click events
  const nub::soft_ref<ImageDisplayStream> ids = itsOFS->findFrameDestType<ImageDisplayStream>();
  if (ids.is_valid()) //Do we have a xwins
  {
    std::vector<rutz::shared_ptr<XWinManaged> > windows = ids->getWindows();

    for(uint i=0; i<windows.size(); i++)
    {
      const rutz::shared_ptr<XWinManaged> uiwin = windows[i];
      if (uiwin.is_valid())
      {
        Point2D<int> loc = uiwin->getLastMouseClick();
        int key = uiwin->getLastKeyPress();

        if (loc.isValid() || key != -1)
        {
          rutz::shared_ptr<SimEventUserInput> e(new SimEventUserInput(this, uiwin->getTitle(), loc, key));
          q.post(e);
        }
      }
    }
  }

  // should we save results immediately?
  if (fs == FRAME_NEXT || fs == FRAME_FINAL)
    {
      // request that everybody save their outputs to our OFS:
      rutz::shared_ptr<SimModuleSaveInfo> sinfo(new SimModuleSaveInfo(itsOFS, q));
      rutz::shared_ptr<SimEventSaveOutput> e(new SimEventSaveOutput(this, sinfo));
      q.post(e);

      // if we had some save requests, clear them since we just saved now:
      itsSaveRequested = false;
    }
  else
    // otherwise, just note that we should save them next time we write an output frame:
    if (saveRequested) itsSaveRequested = true;

  // does our FrameSeries wish to wait for user?
  if (itsOFS->shouldWait())
    {
      // post a userwait event which our SimEventQueue will catch at
      // the turn of the current time step:
      rutz::shared_ptr<SimEventUserWait> e(new SimEventUserWait(this));
      q.post(e);
    }

  // is this the end of our outputs?
  if (fs == FRAME_FINAL)
    {
      rutz::shared_ptr<SimEventBreak> e(new SimEventBreak(this, "Output frames complete"));
      q.post(e);
    }
}

// ######################################################################
bool SimOutputFrameSeries::isVoid() const
{ return itsOFS->isVoid(); }

// ######################################################################
void SimOutputFrameSeries::addFrameDest(const std::string& dest)
{ itsOFS->addFrameDest(dest); }

// ######################################################################
int SimOutputFrameSeries::frame() const
{ return itsOFS->frame(); }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
