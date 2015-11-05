/*!@file Psycho/EyeTracker.H Abstraction of an eye tracker device */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTracker.H $
// $Id: EyeTracker.H 14176 2010-10-28 04:28:19Z ilink $
//

#ifndef PSYCHO_EYETRACKER_H_DEFINED
#define PSYCHO_EYETRACKER_H_DEFINED
#include "Psycho/PsychoDisplay.H"
#include "Component/ModelComponent.H"
#include "Image/Point2D.H"
#include "Image/CalibrationTransform.H"
#include "Image/AffineTransform.H"
#include <string>
class EventLog;
class PsychoDisplay;

//! Abstraction of an eye tracker device, virtual base class
/*! This class specifies the common interface to all EyeTracker
  derivatives and provides a number of shared basic functions. This
  class cannot be instantiated directly since it contains some pure
  virtual methods. Instead, instantiate one of the EyeTrackerXXX
  objects which derive from the present EyeTracker common base. */

class EyeTracker : public ModelComponent
{
public:
  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  //! Constructor
  EyeTracker(OptionManager& mgr,
             const std::string& descrName = "Eye Tracker",
             const std::string& tagName = "EyeTracker");

  //! destructor
  virtual ~EyeTracker();

  //! Set an EventLog to use to keep a trace of what happens
  /*! If an EventLog component is registered with us, we will send it
    lots of event messages, each time we start or stop tracking, etc. */
  virtual void setEventLog(nub::soft_ref<EventLog> elog);

  //@}

  // ######################################################################
  /*! @name Eye-tracker control functions */
  //@{

  //! Calibrate the tracker, full calibration
  /*! This function will use the provided PsychoDisplay to display
    calibration images and wait for user input. Default implementation is
    to do nothing. */
  virtual void calibrate(nub::soft_ref<PsychoDisplay> d);

  //! Calibrate the tracker, quick re-calibration
  /*! This function will use the provided PsychoDisplay to display
    calibration images and wait for user input. Default implementation is
    to do nothing. */
  virtual void recalibrate(nub::soft_ref<PsychoDisplay> d,int repeats=5);

  virtual void setBackgroundColor(nub::soft_ref<PsychoDisplay> d);

	virtual void manualDriftCorrection(Point2D<double> eyepos, Point2D<double> targetpos);

	//! Run online calibration routine if available on the eyetracker
  virtual void calibrateOnline(nub::soft_ref<PsychoDisplay> d);

        /*! Close SDL from EyeLink library.  This function is designed when using
                 EyeLink eye tracker.  Ignore the function when using other eye tracker.
                The purpose of opening and closing SDL from EyeLink library is to take back
                full control of SDL from our own code.  Therefore, movies can be played
          smoothly (no slow frames).        */
  virtual void closeSDL();

        /*! Open SDL from EyeLink library.  This function is designed when using
                 EyeLink eye tracker.  Ignore the function when using other eye tracker.
                The purpose of opening and closing SDL from EyeLink library is to take back
                full control of SDL from our own code.  Therefore, movies can be played
          smoothly (no slow frames).        */
  virtual void openSDL();

  //! Start/stop tracking depending on a boolean parameter
  /*! If startstop is true, then StartTracking() will be called,
    otherwise StopTracking() will. Note that requests to start tracking
    will be ignored here (no call to startTracking()) if we are
    already tracking; idem for stopping. */
  virtual void track(const bool startstop);

  //! Are we tracking?
  virtual bool isTracking() const;

  //! Get current eye-tracking session number
  virtual int getSession() const;

  //!Get the calibration set
  virtual CalibrationTransform::Data getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const = 0;
  //@}



  // ######################################################################
  /*! @name Eye-tracker monitoring functions */
  //@{

  //! Has the eye-tracker detected that the subject is fixating?
  /*! Depending on hardware, this may either rely on a built-in
    fixation detection in the eye-tracking device (e.g.,
    EyeTrackerDML, EyeTrackerUDP), or on monitoring getEyePos() over
    time (e.g., EyeTrackerISCAN). NOTE: this should be a non-blocking
    call as typically callers will also want to check for alternative
    triggers like keypresses while polling isFixating(). */
  virtual bool isFixating() = 0;

  //! Has the eye-tracker detected that the subject initiated a
  //! saccade?
  /*! Depending on hardware, this may either rely on a built-in
    saccade detection in the eye-tracking device (e.g. EyeTrackerUDP),
    or on monitoring getEyePos() over time. NOTE: this should be a
    non-blocking call as typically callers will also want to check for
    alternative triggers like keypresses while polling isSaccade(). */
  virtual bool isSaccade() = 0;

  //! Clear previosly triggered fixations and saccades. 
  /*! Clear all our eye status flags so that isFixating and isSaccade
    will not evaluate to true because of a previosly triggered but
    unchecked for fixation or saccde. Default behavior is to loop over
    isFixating and isSaccade, gobbling up any extra
    fixations. Depending on hardware, some eye trackers may have
    different requirements. */
  virtual void clearEyeStatus();

  //@}

  // ######################################################################
  /*! @name Eye position data streaming functions */
  //@{

  //! Get current eye position
  /*! Note: not all trackers support this! */
  virtual Point2D<int> getEyePos() const = 0;

  //! Get current fixation position (eliminates blink, saccade, etc.)
  /*! Note: not all trackers support this! */
  virtual Point2D<int> getFixationPos() const = 0;

  //@}

  //!get the current calibrated eye position
  virtual Point2D<int> getCalibEyePos();

  //!save a direct eyeS file using online calib
  virtual void requestQuickEyeS();

//getter setter function for current stimulus filename used in online
// calib type psycho progs
    virtual void setCurrentStimFile(std::string filename);
    virtual std::string getCurrentStimFile();


protected:
  //! Start tracking
  /*! Derived classes need to implement this in a hardware-dependent manner. */
  virtual void startTracking() = 0;

  //! Stop tracking
  /*! Derived classes need to implement this in a hardware-dependent manner. */
  virtual void stopTracking() = 0;

  void start1(); //!< get started

  nub::soft_ref<EventLog> itsEventLog; //!< log stuff if desired

private:
  bool itsIsTracking;  // true if we are currently tracking
  int itsSession; // tracking session number
  std::string itsCurrentStimFile; //name of current stimulus being shown

};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKER_H_DEFINED
