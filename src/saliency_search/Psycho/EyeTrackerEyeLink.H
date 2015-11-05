/*!@file Psycho/EyeTrackerEyeLink.H Abstraction for an EyeLink eye tracker */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerEyeLink.H $
// $Id: EyeTrackerEyeLink.H 15105 2011-12-10 01:59:29Z pohetsn $
//

#ifndef PSYCHO_EYETRACKEREYELINK_H_DEFINED
#define PSYCHO_EYETRACKEREYELINK_H_DEFINED

#include "Psycho/EyeTracker.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"

//! Interface to an EyeLink-II eye-tracker
/*! This requires the EyeLink development kit for Linux, available
  only from SR-Research to select registered users. NOTE: If you
  encounter an error message like "Cannot open connection, check
  TCP/IP settings", this may be due to the way the eyelink library
  figures out your host's IP address, which is through resolving your
  hostname (nasty!). Since typically you will have two ethernet cards
  in your box, one connected to the EyeLink tracker (100.1.1.1) via a
  crossover cable, you need your hostname to be associated with your
  IP on that network (e.g., your IP will typically be 100.1.1.2).
  Check your /etc/nsswich.conf and /etc/hosts to make sure this is the
  case. */
class EyeTrackerEyeLink : public EyeTracker
{
public:
  //! Constructor
  EyeTrackerEyeLink(OptionManager& mgr,
                    const std::string& descrName = "Eye Tracker EyeLink",
                    const std::string& tagName = "EyeTrackerEyeLink");
  //! Destructor
  virtual ~EyeTrackerEyeLink();

  //! Calibrate the tracker, full calibration
  /*! Here we do a 9-point calibration. */
  virtual void calibrate(nub::soft_ref<PsychoDisplay> d);

	virtual void calibrate2(nub::soft_ref<PsychoDisplay> d);

	virtual void setBackgroundColor(nub::soft_ref<PsychoDisplay> d); 

  //! Calibrate the tracker, quick re-calibration
  /*! Since there is no quick calibration for ISCAN, this is like a
    full calibration. */
  virtual void recalibrate(nub::soft_ref<PsychoDisplay> d, int repeats);

  //! Open SDL from eyelink library
  virtual void openSDL();

  //! Close SDL from eyelink library
  virtual void closeSDL();

  //! Is the subject fixating?
  /*! TODO: implement this functionality*/
  virtual bool isFixating();

  //! Is the subject in a saccade?
  /*! TODO: implement this functionality*/
  virtual bool isSaccade();

  //! Get current eye position
  virtual Point2D<int> getEyePos() const;

  //! Get current fixation position (eliminates blink, saccades, etc.)
  virtual Point2D<int> getFixationPos() const;

	//! mannual drift correction
	virtual void manualDriftCorrection(Point2D<double> eyepos, Point2D<double> targetpos);

  //!Get the calibration set
  virtual CalibrationTransform::Data getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const;

protected:
  OModelParam<std::string> itsEDFfname;
  OModelParam<Dims> itsDims;      //!< SDL screen resolution

	OModelParam<PixRGB<byte> > itsEyelinkBackgroundColor; //!< eyelink background grey color for calibration
	OModelParam<PixRGB<byte> > itsEyelinkForegroundColor; //!< eyelink foreground color for calibration

	//! Start the tracker
  virtual void startTracking();

  //! Stop the tracker
  virtual void stopTracking();

  virtual void start1(); //!< get started
  virtual void stop1();  //!< get stopped
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKEREYELINK_H_DEFINED
