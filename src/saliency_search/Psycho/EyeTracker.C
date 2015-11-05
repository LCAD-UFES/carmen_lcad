/*!@file Psycho/EyeTracker.C Abstraction of an eye tracker device */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTracker.C $
// $Id: EyeTracker.C 14176 2010-10-28 04:28:19Z ilink $
//

#ifndef PSYCHO_EYETRACKER_C_DEFINED
#define PSYCHO_EYETRACKER_C_DEFINED

#include "Psycho/EyeTracker.H"
#include "Component/EventLog.H"
#include "Util/sformat.H"

// ######################################################################
EyeTracker::EyeTracker(OptionManager& mgr, const std::string& descrName,
                       const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsEventLog(), itsIsTracking(false)
{  }

// ######################################################################
EyeTracker::~EyeTracker()
{  }

// ######################################################################
void EyeTracker::setEventLog(nub::soft_ref<EventLog> elog)
{ itsEventLog = elog; }

// ######################################################################
void EyeTracker::start1()
{
  // reset session number:
  itsSession = 0;
}

// ######################################################################
int EyeTracker::getSession() const
{ return itsSession; }

// ######################################################################
void EyeTracker::calibrate(nub::soft_ref<PsychoDisplay> d)
{
  LINFO("Calibration not supported in this tracker.");
}

// ######################################################################
void EyeTracker::setBackgroundColor(nub::soft_ref<PsychoDisplay> d)
{
  LINFO("SetBackgroundColor not supported in this tracker.");
}

// ######################################################################
void EyeTracker::manualDriftCorrection(Point2D<double> eyepos, 
								                       Point2D<double> targetpos)
{
  LINFO("Recalibration not supported in this tracker.");
}

// ######################################################################
void EyeTracker::recalibrate(nub::soft_ref<PsychoDisplay> d, int repeats)
{
  LINFO("Recalibration not supported in this tracker.");
}

// ######################################################################
void EyeTracker::calibrateOnline(nub::soft_ref<PsychoDisplay> d)
{
  LFATAL("online calibration not supported on this tracker");
}

// ######################################################################
void EyeTracker::closeSDL()
{
  LFATAL("online calibration not supported on this tracker");
}

// ######################################################################
void EyeTracker::openSDL()
{
  LFATAL("online calibration not supported on this tracker");
}

// ######################################################################
void EyeTracker::track(const bool startstop)
{
  if (startstop)
    {
      if (itsIsTracking)
        LERROR("Request to start tracking ignored, already tracking...");
      else
        {
          this->startTracking(); // derived classes implement this
          if (itsEventLog.isValid())
            itsEventLog->
              pushEvent(sformat("----- Eye Tracker Start session %d -----",
                                itsSession));
          itsIsTracking = true;
        }
    }
  else
    {
      if (itsIsTracking == false)
        LERROR("Request to stop tracking ignored, not tracking...");
      else
        {
          this->stopTracking();  // derived classes implement this
          if (itsEventLog.isValid())
            itsEventLog->
              pushEvent(sformat("----- Eye Tracker Stop session %d -----",
                                itsSession));
          ++itsSession;   // ready for next session:
          itsIsTracking = false;
        }
    }
}

// ######################################################################
bool EyeTracker::isTracking() const
{ return itsIsTracking; }

// ######################################################################
void EyeTracker::clearEyeStatus()
{
  while (isFixating()) ;
  while (isSaccade()) ;
}

// ######################################################################
Point2D<int> EyeTracker::getCalibEyePos()
{        LFATAL("not supported");
        return Point2D<int>(-1,-1);}

// #####################################################################
void EyeTracker::requestQuickEyeS()
{
    LFATAL("can't save eyeS only eyetrackers with online calibration support this");
}

// ####################################################################

void EyeTracker::setCurrentStimFile(std::string filename)
{
    itsCurrentStimFile = filename;
}

// ####################################################################

std::string EyeTracker::getCurrentStimFile()
{
    if(!itsCurrentStimFile.empty())
            return itsCurrentStimFile;
        else
        {  LERROR("itsCurrentStimFile not set");
            return NULL;
        }


}



/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKER_C_DEFINED
