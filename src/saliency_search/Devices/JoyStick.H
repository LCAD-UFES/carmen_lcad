/*!@file Devices/JoyStick.H Driver for a linux joystick */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/JoyStick.H $
// $Id: JoyStick.H 13071 2010-03-28 16:24:18Z beobot $
//

#ifndef JOYSTICK_H_DEFINED
#define JOYSTICK_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"

#include <pthread.h>
#include <vector>

//! Simple listener for JoyStick events
/*! The JoyStickListener gets called each time an event is received
  from the JoyStick.  By default, there is no listener, and users can
  just asynchronously query the JoyStick for its current internal
  state. If those queries come in too slowly, however, they may miss
  some events (e.g., a briefly pressed button). So, production code
  should define a derived class for JoyStickListener and register it
  with the JoyStick object, so that it will take action as soon as an
  event is received. */
class JoyStickListener {
public:
  //! Destructor
  virtual ~JoyStickListener();

  //! An axis changed
  /*! @param num axis number
    @param val new axis value */
  virtual void axis(const uint num, const int16 val) = 0;

  //! A button changed
  /*! @param num button number
    @param state new button state (true = pressed) */
  virtual void button(const uint num, const bool state) = 0;
};


#ifdef HAVE_LINUX_JOYSTICK_H

//! Driver for a Linux joystick
/*! This is a simple driver for a Linux joystick. There are basically
  two modes of operation: you can either get the current JoyStick
  status at any time using getAxisValue() and getButtonState(), or you
  can register a JoyStickListener using setListener, and the listener
  will be called each time some change occurs in any of the axes or
  buttons. Beware that your listener will be called from a thread
  running in parallel with your main thread. */
class JoyStick : public ModelComponent
{
public:
  //! Constructor
  JoyStick(OptionManager& mgr,
           const std::string& descrName = "JoyStick",
           const std::string& tagName = "JoyStick",
           const char *dev = "/dev/input/js0");

  //! Add a listener
  void setListener(rutz::shared_ptr<JoyStickListener>& listener);

  //! Destructor
  ~JoyStick();

  //! Get number of axes
  uint getNumAxes() const;

  //! Get number of buttons
  uint getNumButtons() const;

  //! Get value of a given axis
  int16 getAxisValue(const uint num) const;

  //! Get state of a given button
  bool getButtonState(const uint num) const;

  //! Should not call this directly -- running in a thread
  void run();

  bool joystickPlugged()
  {
	  return itsPlugged;
  }

protected:
  NModelParam<std::string> itsDevName; //!< Name of our device

  void start1();  //!< get started
  void stop1();   //!< get stopped

  int itsFd;         //!< file descriptor for our device
  bool itsPlugged;
  bool itsKeepGoing; //!< tell our thread to keep going
  std::vector<int16> itsAxes;    //!< data from our axes
  std::vector<bool> itsButtons;  //!< data from our buttons

private:
  rutz::shared_ptr<JoyStickListener> itsListener;
  pthread_mutex_t itsMutex;  // mutex to protect access to the data
  pthread_t itsRunner;       // thread to poll the joystick
};

#endif // HAVE_LINUX_JOYSTICK_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
