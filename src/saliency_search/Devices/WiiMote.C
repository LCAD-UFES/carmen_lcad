/*!@file Devices/WiiMote.C read wiimote data  */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/WiiMote.C $
// $Id: WiiMote.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Devices/WiiMote.H"

#include "Component/ParamMap.H"
#include "Util/Assert.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <cmath>
#include <unistd.h>

void* WiiMote_run(void *r0); // will live in a separate thread

// ######################################################################
void* WiiMote_run(void *r0)
{
  WiiMote *r = (WiiMote *)r0;
  r->run(); return NULL;
}

// ######################################################################
WiiMote::WiiMote(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName)
{
  running = false;
        //itsWiimote = wiimote_t;
  pthread_mutex_init(&itsLock, NULL);
}

// ######################################################################
void WiiMote::start1()
{
}

// ######################################################################
void WiiMote::start2()
{
}

void WiiMote::init()
{
#ifdef HAVE_LIBWIIMOTE
  LINFO("Press buttons 1 and 2 on the wiimote now to connect.");
  int nmotes = wiimote_discover(&itsWiimote, 1);
  if (nmotes == 0)
    LFATAL("no wiimotes were found");

  LINFO("found: %s\n", itsWiimote.link.r_addr);

  if (wiimote_connect(&itsWiimote, itsWiimote.link.r_addr) < 0)
    LFATAL("Unable to connect to wiimote");

  /* Activate the first led on the wiimote. It will take effect on the
     next call to wiimote_update. */

  itsWiimote.led.one  = 1;
  itsWiimote.mode.ir = 1;
  itsWiimote.mode.acc = 1;

  // start thread for run():
  pthread_create(&runner, NULL, &WiiMote_run, (void *)this);
#else
  LFATAL("Need the libwiimote");
#endif

}

// ######################################################################
void WiiMote::stop1()
{
  // stop our thread:
  running = false; while(running == false) usleep(5);
  usleep(50); running = false;
}

// ######################################################################
WiiMote::~WiiMote()
{
  pthread_mutex_destroy(&itsLock);
}

// ######################################################################
WiiMote::SensorData WiiMote::getSensorData()
{
  SensorData sensorData;
#ifdef HAVE_LIBWIIMOTE
  //pthread_mutex_lock(&itsLock);
  sensorData.tilt.x = itsWiimote.tilt.x;
  sensorData.tilt.y = itsWiimote.tilt.y;
  sensorData.tilt.z = itsWiimote.tilt.z;

  sensorData.force.x = itsWiimote.force.x;
  sensorData.force.y = itsWiimote.force.y;
  sensorData.force.z = itsWiimote.force.z;

  sensorData.IR1 = Point2D<int>(itsWiimote.ir1.x, itsWiimote.ir1.y);
  sensorData.ir1Size = itsWiimote.ir1.size;

  sensorData.IR2 = Point2D<int>(itsWiimote.ir2.x, itsWiimote.ir2.y);
  sensorData.ir2Size = itsWiimote.ir2.size;

  sensorData.IR3 = Point2D<int>(itsWiimote.ir3.x, itsWiimote.ir3.y);
  sensorData.ir3Size = itsWiimote.ir3.size;

  sensorData.IR4 = Point2D<int>(itsWiimote.ir4.x, itsWiimote.ir4.y);
  sensorData.ir4Size = itsWiimote.ir4.size;
  //pthread_mutex_unlock(&itsLock);
#endif

  return sensorData;
}

// ######################################################################
void WiiMote::run()
{
#ifdef HAVE_LIBWIIMOTE
  running = true;

  while(wiimote_is_open(&itsWiimote) && running)
  {
    pthread_mutex_lock(&itsLock);
    int rc = wiimote_update(&itsWiimote);
    pthread_mutex_unlock(&itsLock);

    if (rc < 0) {
      wiimote_disconnect(&itsWiimote);
      break;
    }
    //// sleep a little:
    //usleep(5000);
  }

  // we got an order to stop:
  //running = f;  // FIXME: bogus! other thread may unlock too soon
  pthread_exit(0);
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
