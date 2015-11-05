/*!@file Psycho/EyeTrackerISCAN.H Abstraction for an ISCAN eye tracker */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerISCAN.H $
// $Id: EyeTrackerISCAN.H 14159 2010-10-22 04:04:17Z ilink $
//

#ifndef PSYCHO_EYETRACKERISCAN_H_DEFINED
#define PSYCHO_EYETRACKERISCAN_H_DEFINED

#include "Psycho/EyeTracker.H"
#include "Component/ModelParam.H"
#include "rutz/atomic.h"
class Serial;
class ParPort;


//! Interface to an ISCAN RK-464 eye-tracker
/*! There are two modes of operation which can be used with the ISCAN
  tracker: trigger via serial port and no data streaming, or trigger
  via parallel port and data streaming over serial. */
class EyeTrackerISCAN : public EyeTracker
{
public:
  //! Constructor
  EyeTrackerISCAN(OptionManager& mgr,
                  const std::string& descrName = "Eye Tracker ISCAN",
                  const std::string& tagName = "EyeTrackerISCAN");
  //! Destructor
  virtual ~EyeTrackerISCAN();

  //! Calibrate the tracker, full calibration
  /*! Here we do a 9-point calibration. */
  virtual void calibrate(nub::soft_ref<PsychoDisplay> d);

  //! Calibrate the tracker, quick re-calibration
  /*! Since there is no quick calibration for ISCAN, this is like a
    full calibration. */
  virtual void recalibrate(nub::soft_ref<PsychoDisplay> d, int repeats);

  //! Perform an online calibration

  virtual void calibrateOnline(nub::soft_ref<PsychoDisplay> d);

  //! Is the subject fixating?
  /*! TODO: This only works in parallel trigger mode. We check that eye
    position remains within a tolerance over a period of time (see
    model parameters). */
  virtual bool isFixating();

  //! Is the subject in a saccade?
  /*! TODO: give this some functionality */
  virtual bool isSaccade();

  //! Get current eye position
  /*! TODO: we need to think about calibration */
  virtual Point2D<int> getEyePos() const;

  //! Get current fixation position
  /*! TODO: we need to think about calibration */
  virtual Point2D<int> getFixationPos() const;

  //! Get the current calibrated eye position
  virtual Point2D<int> getCalibEyePos();

  //!Get the calibration set
  virtual CalibrationTransform::Data getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const;

  //i want a quick calibrated eyeS file in the end
   void requestQuickEyeS();


protected:
  //! Start the tracker
  /*! If serial trigger is selected, we send a single byte of value
    132 to start the ISCAN tracker. Otherwise, we violently set all
    pins of the parallel port to 0 to start the tracker. */
  virtual void startTracking();

  //! Stop the tracker
  /*! If serial trigger is selected, we send a single byte of value
    136 to stop the ISCAN tracker. Otherwise, we violently set all
    pins of the parallel port to 1 to stop the tracker. */
  virtual void stopTracking();

  OModelParam<bool> itsParTrig; //!< Use parallel-port start/stop trigger
  OModelParam<std::string> itsSerDev; //! device filename for serial port
  OModelParam<std::string> itsParDev; //! device filename for parallel port

  virtual void start1(); //!< get started
  virtual void start2(); //!< get started
  bool itsRequestQuickEyeS;
private:
  nub::soft_ref<Serial> itsSerial;
  nub::soft_ref<ParPort> itsParPort;
  int itsRecalibCount;
  AffineTransform itsAffine;

  //live eye position polling stuff
  static void* eyePosPollThread(void* p);

  struct EyePosEvent
  {
    int64 tim;
    Point2D<int16> pt;
  };

  std::vector<EyePosEvent> itsEyePosEvents;
  pthread_t itsEyePosPollThread;
  rutz::atomic_int_t itsCurrentRawEyePos;
  rutz::atomic_int_t itsCurrentCalibEyePos;
  rutz::atomic_int_t itsCurrentCalibEyePosX;
  rutz::atomic_int_t itsCurrentCalibEyePosY;
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKERISCAN_H_DEFINED
