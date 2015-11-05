/*!@file Devices/HMR3300.H class for interfacing with a Honeywell 3300 compass */

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
// Primary maintainer for this file: Nitin Dhavale <dhavale@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/HMR3300.H $
// $Id: HMR3300.H 7880 2007-02-09 02:34:07Z itti $
//

#ifndef DEVICES_HMR3300_H_DEFINED
#define DEVICES_HMR3300_H_DEFINED

#include "Component/OptionManager.H"
#include "Devices/Serial.H"
#include "Util/Angle.H"
#include <pthread.h>

//! A hook which will be called when a new compass reading is received
class HMR3300Listener {
public:
  //! Destructor
  virtual ~HMR3300Listener();

  //! New data was received
  virtual void newData(const Angle heading, const Angle pitch,
                       const Angle roll) = 0;
};

//! Class for interfacing with a Honeywell HMR-3300 compass
/*! This is a 3-axis compass. The compass communicates with a host via
  RS-232 at 19,200 bauds. The compass sends the 3 angular measurements
  at regular intervals.

  In our BeoSub, Pitch is approx 0.0 when horizontal; tilting the sub
  nose down makes the pitch more negative.

  For Heading, 0.0 is north, turning left decreases the heading angle.

  For Roll, rolling leftwards (so that the left side of the sub goes
  down while the right side goes up) increases the roll angle.
*/
class HMR3300 : public ModelComponent
{
public:
  //! Initialize
  HMR3300(OptionManager& mgr,
          const std::string& descrName = "HMR3300",
          const std::string& tagName = "HMR3300",
          const char *dev = "/dev/ttyS0");

  //! Destructor
  ~HMR3300();

  //! Install a callback (listener)
  /*! This callback will be called with the corresponding data each
    time new data is received from the device. */
  void setListener(rutz::shared_ptr<HMR3300Listener>& listener);

  //! Get the current filtered heading, pitch, and roll from the compass
  void get(Angle& heading, Angle& pitch, Angle& roll);

  //! Get the current filtered heading
  Angle getHeading();

  //! Get the current filtered pitch
  Angle getPitch();

  //! Get the current filtered roll
  Angle getRoll();

  //! The user need not concern himself with this method.
  /*! Should have been protected, but is not because
    of the pthread hack. */
  void run();

protected:
  void start2();  //!< get started
  void stop1();   //!< get stopped

private:
  nub::soft_ref<Serial> itsSerial;
  bool itsKeepgoing;
  rutz::shared_ptr<HMR3300Listener> itsListener;
  Angle itsHeading, itsPitch, itsRoll;
  pthread_mutex_t itsLock;
  pthread_t itsRunner;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
