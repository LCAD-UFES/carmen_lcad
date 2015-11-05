/*!@file Devices/HMR3300.C class for interfacing with a Honeywell 3300 compass */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/HMR3300.C $
// $Id: HMR3300.C 7880 2007-02-09 02:34:07Z itti $
//

#include "Devices/HMR3300.H"
#include <sstream>
#include <string>

void *Compass_run(void *c);

// ######################################################################
void *Compass_run(void *c)
{
  HMR3300 *d = (HMR3300 *)c;
  d->run();
  return NULL;
}

// ######################################################################
HMR3300Listener::~HMR3300Listener()
{ }

// ######################################################################
HMR3300::HMR3300(OptionManager& mgr, const std::string& descrName,
                 const std::string& tagName, const char *dev) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr, descrName+" Serial Port", tagName+"SerialPort")),
  itsKeepgoing(true), itsListener(), itsHeading(), itsPitch(), itsRoll()
{
  itsSerial->configure(dev, 19200, "8N1", false, false, 0);
  addSubComponent(itsSerial);
  pthread_mutex_init(&itsLock, NULL);
}

// ######################################################################
void HMR3300::setListener(rutz::shared_ptr<HMR3300Listener>& listener)
{ itsListener = listener; }

// ######################################################################
void HMR3300::start2()
{ pthread_create(&itsRunner, NULL, &Compass_run, (void *)this); }

// ######################################################################
void HMR3300::stop1()
{
  itsKeepgoing = false;
  usleep(300000); // make sure thread has exited
}

// ######################################################################
HMR3300::~HMR3300()
{
  pthread_mutex_destroy(&itsLock);
}

// ######################################################################
void HMR3300::run()
{
  unsigned char c = 255;
  while(itsKeepgoing)
    {
      // skip to next LF:
      while(c != '\n') c = itsSerial->read();

      // get data until next LF:
      std::string str;
      while( (c = itsSerial->read() ) != '\n')
         if (isdigit(c) || c == '.' || c == '-') str += c;
          else str += ' ';
      // convert to doubles:
        std::stringstream strs(str);
       double hh, pp, rr; strs >> hh >> rr >> pp;

      // update our internal data:
      pthread_mutex_lock(&itsLock);
      itsHeading = hh; itsPitch = pp; itsRoll = rr;
      pthread_mutex_unlock(&itsLock);

      // if we have a listener, let it know:
      if (itsListener.is_valid()) itsListener->newData(hh, pp, rr);
    }

  pthread_exit(0);
}

// ######################################################################
void HMR3300::get(Angle& heading, Angle& pitch, Angle& roll)
{
  pthread_mutex_lock(&itsLock);
  heading = itsHeading; pitch = itsPitch; roll = itsRoll;
  pthread_mutex_unlock(&itsLock);
}

// ######################################################################
Angle HMR3300::getHeading()
{
  pthread_mutex_lock(&itsLock);
  const Angle ret = itsHeading;
  pthread_mutex_unlock(&itsLock);
  return ret;
}

// ######################################################################
Angle HMR3300::getPitch()
{
  pthread_mutex_lock(&itsLock);
  const Angle ret = itsPitch;
  pthread_mutex_unlock(&itsLock);
  return ret;
}

// ######################################################################
Angle HMR3300::getRoll()
{
  pthread_mutex_lock(&itsLock);
  const Angle ret = itsRoll;
  pthread_mutex_unlock(&itsLock);
  return ret;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
