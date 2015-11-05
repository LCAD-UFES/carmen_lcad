/*!@file Psycho/EyeTrackerTIL.C Eye tracker in Tadashi Isa's lab */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerTIL.C $
// $Id: EyeTrackerTIL.C 14159 2010-10-22 04:04:17Z ilink $
//

#ifndef PSYCHO_EYETRACKERTIL_C_DEFINED
#define PSYCHO_EYETRACKERTIL_C_DEFINED

#include "Component/OptionManager.H"
#include "Psycho/EyeTrackerTIL.H"
#include "Psycho/PsychoOpts.H"
#include "Util/sformat.H"
#include "Devices/ParPort.H"

// ######################################################################
EyeTrackerTIL::EyeTrackerTIL(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName) :
  EyeTracker(mgr, descrName, tagName),
  itsParDev(&OPT_EyeTrackerParDev, this),
  itsParPort(new ParPort(mgr, "TIL Parallel Port", "TILParPort"))
{
  addSubComponent(itsParPort);
}

// ######################################################################
EyeTrackerTIL::~EyeTrackerTIL()
{  }

// ######################################################################
void EyeTrackerTIL::start1()
{
  // configure our parallel port:
  itsParPort->setModelParamVal("TILParPortDevName", itsParDev.getVal());

  EyeTracker::start1();
}

// ######################################################################
void EyeTrackerTIL::start2()
{
  // make sure we start with the tracker off:
  itsParPort->WriteData(128, 0);    // reset pin D7
}

// ######################################################################
void EyeTrackerTIL::startTracking()
{
  itsParPort->WriteData(127, getSession()); // set the data pins D0-D6
  itsParPort->WriteData(128, 128);          // assert pin D7
}

// ######################################################################
void EyeTrackerTIL::stopTracking()
{
  itsParPort->WriteData(128, 0);    // reset pin D7
}

// ######################################################################
bool EyeTrackerTIL::isFixating()
{
  if (itsParPort->ReadStatusPaperout()) return true;
  else return false;
}

// ######################################################################
bool EyeTrackerTIL::isSaccade()
{
  LINFO("Unimplemented on TIL eye traker");
  return false;
}

// ######################################################################
Point2D<int> EyeTrackerTIL::getEyePos() const
{
  LFATAL("Unavailable on TIL tracker, sorry.");
  return Point2D<int>(0, 0);
}

// ######################################################################
Point2D<int> EyeTrackerTIL::getFixationPos() const
{
  LFATAL("Unavailable on TIL tracker, sorry.");
  return Point2D<int>(0, 0);
}

// ######################################################################
CalibrationTransform::Data EyeTrackerTIL::getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const
{
    LINFO("\n getting calibration set...");
    CalibrationTransform::Data dummy;
    dummy.addData(Point2D<double>(-1.0,-1.0),Point2D<double>(-1.0,-1.0));
    return dummy;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKERTIL_C_DEFINED
