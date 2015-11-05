/*!@file Psycho/EyeTrackerStub.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerStub.C $
// $Id: EyeTrackerStub.C 14159 2010-10-22 04:04:17Z ilink $
//

#ifndef PSYCHO_EYETRACKERSTUB_C_DEFINED
#define PSYCHO_EYETRACKERSTUB_C_DEFINED

#include "Psycho/EyeTrackerStub.H"

// ######################################################################
EyeTrackerStub::EyeTrackerStub(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName)
  :
  EyeTracker(mgr, descrName, tagName)
{}

// ######################################################################
EyeTrackerStub::~EyeTrackerStub()
{}

// ######################################################################
bool EyeTrackerStub::isFixating()
{ return false; }

// ######################################################################
bool EyeTrackerStub::isSaccade()
{ return false; }

// ######################################################################
Point2D<int> EyeTrackerStub::getEyePos() const
{ return Point2D<int>(-1,-1); }

// ######################################################################
Point2D<int> EyeTrackerStub::getFixationPos() const
{ return Point2D<int>(-1,-1); }

// ######################################################################
void EyeTrackerStub::startTracking()
{ /* stub; do nothing */ }

// ######################################################################
void EyeTrackerStub::stopTracking()
{ /* stub; do nothing */ }

// ######################################################################
CalibrationTransform::Data EyeTrackerStub::getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const
{
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

#endif // PSYCHO_EYETRACKERSTUB_C_DEFINED
