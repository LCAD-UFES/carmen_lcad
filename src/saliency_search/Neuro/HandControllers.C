/*!@file Neuro/HandControllers.C Hand controllers */

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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL:
// $Id:
//

#include "Neuro/HandControllers.H"
//#include "Neuro/SaccadeControllers.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SpatialMetrics.H"
#include "Component/OptionManager.H"
#include "Psycho/HandTrace.H"
#include "Simulation/SimEventQueue.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/sformat.H"

#include <vector>

// ######################################################################
// ## Stub Hand Controller
// ######################################################################
StubHandController::StubHandController(OptionManager& mgr) :
  HandController(mgr, "Stub Hand Controller",
                 "StubHandController")
{  }

// ######################################################################
StubHandController::~StubHandController()
{  }



// ######################################################################
// ## Tracker Hand Controller
// ######################################################################
TrackerHandController::TrackerHandController(OptionManager& mgr) :
  HandController(mgr, "Tracker Hand Controller",
                    "TrackerHandController"),
  SIMCALLBACK_INIT(SimEventClockTick),
  itsConfig(&OPT_HandConfig, this),
  itsHandTrace(), itsHandSample()
{  }

// ######################################################################
TrackerHandController::~TrackerHandController()
{  }

// ######################################################################
void TrackerHandController::start1()
{
  // parse our config string and instantiate all our trackers:
  std::vector<std::string> tok;
  split(itsConfig.getVal(), ",", std::back_inserter(tok));
  if (tok.empty()) LFATAL("I cannot run without at least one handtrace file.");

  for (uint i = 0; i < tok.size(); i ++) {
    std::vector<std::string> tt;
    split(tok[i], ":", std::back_inserter(tt));
    if (tt.empty()) LFATAL("Invalid empty eye-tracker filename");
    
    std::string fname = tt[0];
    std::string extras = join(tt.begin() + 1, tt.end(), ":");
    
    LINFO("Instantiating Tracker %03d with file '%s', extras '%s'",
          i, fname.c_str(), extras.c_str());
    
    // the only extra we support for now is a color:
    //PixRGB<byte> color(128, 255, 255); // cyan color
    PixRGB<byte> color(255, 255, 128); // light yellow color
    if (tt.size() > 1) convertFromString(tt[1], color);
    
    // instantiate a new HandTrace object:
    rutz::shared_ptr<HandTrace> et(new HandTrace(fname, color));
    
    itsHandTrace.push_back(et);
    itsHandSample.push_back(0);
  }
  
  HandController::start1();
}

// ######################################################################
void TrackerHandController::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& ect)
{
  const SimTime t = q.now();

  // evolve all our babies and post their data:
  bool keepgoing = true;
  while(keepgoing) { // will go on until no tracker has any new data
    keepgoing = false;

    // loop over our trackers:
    for (uint i = 0; i < itsHandTrace.size(); i ++) {
      if (itsHandTrace[i]->hasData(itsHandSample[i], t)) {
        // ok, this warrants that we continue our while() loop:
        keepgoing = true;
        
        // get the next data sample:
        rutz::shared_ptr<HandData> data = itsHandTrace[i]->data(itsHandSample[i]);

        LDEBUG("Human hand %03u [%07" ZU "] (%03d, %03d| %03" ZU ") at %.1fms",
               i, itsHandSample[i], data->getWheel(), data->getAcclBrake(), data->numButton(),
               itsHandSample[i] * itsHandTrace[i]->period().msecs());

        // Event posting
        q.post(rutz::make_shared
               (new SimEventHandTrackerData
                (this, data, i, itsHandTrace[i]->filename(),
                 itsHandTrace[i]->color(), itsHandTrace[i]->period())));
        
        // ready for next eye movement sample:
        ++ itsHandSample[i];
      }
    }
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
