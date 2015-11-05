/*!@file Devices/JoyStick.C simple driver for a Linux joystick */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/JoyStick.C $
// $Id: JoyStick.C 15310 2012-06-01 02:29:24Z itti $
//

#ifdef HAVE_LINUX_JOYSTICK_H

#include "Devices/JoyStick.H"
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <sys/types.h>

// ######################################################################
JoyStickListener::~JoyStickListener()
{  }

// ######################################################################
void *JoyStick_run(void *c);

// ######################################################################
void *JoyStick_run(void *c)
{
  JoyStick *d = (JoyStick *)c;
  d->run();
  return NULL;
}

// ######################################################################
JoyStick::JoyStick(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName, const char *dev) :
  ModelComponent(mgr, descrName, tagName),
  itsDevName(tagName+"DevName", this, dev), itsFd(-1), itsPlugged(false), itsKeepGoing(false), itsAxes(), itsButtons(), itsListener(NULL)
{
  pthread_mutex_init(&itsMutex, NULL);
  //itsFd = open(itsDevName.getVal().c_str(), O_RDONLY);
  //itsPlugged = (itsFd >= 0);
  //if (itsFd >= 0) { close(itsFd); itsFd = -1; }
}

// ######################################################################
void JoyStick::setListener(rutz::shared_ptr<JoyStickListener>& listener)
{ itsListener = listener; }

// ######################################################################
JoyStick::~JoyStick()
{ pthread_mutex_destroy(&itsMutex); }

// ######################################################################
void JoyStick::start1()
{
  // Open joystick device:
  itsFd = open(itsDevName.getVal().c_str(), O_RDONLY);
  if (itsFd < 0) { LINFO("Cannot open %s", itsDevName.getVal().c_str()); return;}
  //LINFO("Joystick plugged in");
  itsPlugged = true;
  // Get number of joystick axes:
  char c;
  int ret = ioctl(itsFd, JSIOCGAXES, &c);
  if (ret < 0) PLFATAL("ioctl(JSIOCGAXES) failed");
  itsAxes.resize(int(c));

  // Get number of joystick buttons:
  ret = ioctl(itsFd, JSIOCGBUTTONS, &c);
  if (ret < 0) PLFATAL("ioctl(JSIOCGBUTTONS) failed");
  itsButtons.resize(c);

  // get our thread going:
  itsKeepGoing = true;
  pthread_create(&itsRunner, NULL, &JoyStick_run, (void *)this);
}

// ######################################################################
void JoyStick::stop1()
{
  itsKeepGoing = false;
  usleep(300000); // make sure thread exits
  if (itsFd >= 0) { close(itsFd); itsFd = -1; }
}

// ######################################################################
uint JoyStick::getNumAxes() const
{ return itsAxes.size(); }

// ######################################################################
uint JoyStick::getNumButtons() const
{ return itsButtons.size(); }

// ######################################################################
int16 JoyStick::getAxisValue(const uint num) const
{
  if (num >= itsAxes.size())
    LFATAL("Axis number %d out of range [0..%" ZU "]", num, itsAxes.size()-1);

  pthread_mutex_lock(const_cast<pthread_mutex_t*>(&itsMutex));
  int16 val = itsAxes[num];
  pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&itsMutex));

  return val;
}

// ######################################################################
bool JoyStick::getButtonState(const uint num) const
{
  if (num >= itsButtons.size())
    LFATAL("Button number %d out of range [0..%" ZU "]", num, itsButtons.size()-1);

  pthread_mutex_lock(const_cast<pthread_mutex_t*>(&itsMutex));
  bool val = itsButtons[num];
  pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&itsMutex));

  return val;
}

// ######################################################################
void JoyStick::run()
{
  js_event event;

  while(itsKeepGoing)
    {
      // get the next event:
      int bytes_read = read(itsFd, &event, sizeof(js_event));
      if (bytes_read < 0) PLERROR("Read error");

      // decode the event:
      if (event.type & JS_EVENT_AXIS)
        {
          if (event.number >= itsAxes.size())
            LERROR("Bogus axis event received -- IGNORED");
          else
            {
              pthread_mutex_lock(&itsMutex);
              itsAxes[event.number] = event.value;
              pthread_mutex_unlock(&itsMutex);

              // activate our listener if we have one:
              if (itsListener.get())
                itsListener->axis(event.number, event.value);
            }
        }
      else if (event.type & JS_EVENT_BUTTON)
        {
          if (event.number >= itsButtons.size())
            LERROR("Bogus button event received -- IGNORED");
          else
            {
              pthread_mutex_lock(&itsMutex);
              itsButtons[event.number] = (event.value == 1);
              pthread_mutex_unlock(&itsMutex);

              // activate our listener if we have one:
              if (itsListener.get())
                itsListener->button(event.number, (event.value == 1));
            }
        }
    }
}

#endif // HAVE_LINUX_JOYSTICK_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
