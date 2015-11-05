/*!@file Psycho/EyeTrackerTIL.H Eye tracker in Tadashi Isa's lab*/

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerTIL.H $
// $Id: EyeTrackerTIL.H 14159 2010-10-22 04:04:17Z ilink $
//

#ifndef PSYCHO_EYETRACKERTIL_H_DEFINED
#define PSYCHO_EYETRACKERTIL_H_DEFINED

#include "Psycho/EyeTracker.H"
#include "Component/ModelParam.H"
class ParPort;

//! Interface to eye-tracker used in Tadashi Isa's lab
/*! This is a monkey scleral search coil type, with trigger and
  fixation detection over the parallel port. */
class EyeTrackerTIL : public EyeTracker {
public:
  //! Constructor
  EyeTrackerTIL(OptionManager& mgr,
                const std::string& descrName = "Eye Tracker TIL",
                const std::string& tagName = "EyeTrackerTIL");
  //! Destructor
  virtual ~EyeTrackerTIL();

  //! Is the subject fixating?
  /*! When fixation is achieved PAPEROUT goes to 1 on the parallel port. */
  virtual bool isFixating();

  //! Is the subject in a saccde?
  /*! Not availible for TIL eye tracker*/
  virtual bool isSaccade();

  //! Get current eye position
  /*! Not supported by this tracker. */
  virtual Point2D<int> getEyePos() const;

  //! Get current fixation position
  /*! Not supported by this tracker. */
  virtual Point2D<int> getFixationPos() const;

  //!Get the calibration set
  virtual CalibrationTransform::Data getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const;

protected:
  //! Start the tracker
  /*! Tracking is started by setting the session number on bits D0-D6
    of the parallel port then asserting D7 until stopped. */
  virtual void startTracking();

  //! Stop the tracker
  /*! Tracking is stopped by resetting D7 on the parallel port. */
  virtual void stopTracking();

  OModelParam<std::string> itsParDev; //! device filename for parallel port

  virtual void start1(); //!< get started
  virtual void start2(); //!< get started

private:
  nub::soft_ref<ParPort> itsParPort;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKERTIL_H_DEFINED
