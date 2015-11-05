/*!@file Devices/WiiMote.H read wiimote data  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/WiiMote.H $
// $Id: WiiMote.H 13901 2010-09-09 15:12:26Z lior $
//

#ifndef WIIMOTE_H_DEFINED
#define WIIMOTE_H_DEFINED


#ifdef HAVE_LIBWIIMOTE
extern "C" {
#define _ENABLE_TILT
#define _ENABLE_FORCE
#undef HAVE_CONFIG_H
#include <libcwiimote/wiimote.h>
#include <libcwiimote/wiimote_api.h>
}
#endif

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"
#include "Image/Point3D.H"
#include "Image/Point2D.H"

#include <pthread.h>
#include <unistd.h>

class WiiMote : public ModelComponent {
public:

  struct SensorData {
    Point3D<float> tilt;
    Point3D<float> force;
    Point2D<int> IR1;
    int ir1Size;
    Point2D<int> IR2;
    int ir2Size;
    Point2D<int> IR3;
    int ir3Size;
    Point2D<int> IR4;
    int ir4Size;
  };



  //! Constructor
  WiiMote(OptionManager& mgr,
               const std::string& descrName = "WiiMote",
               const std::string& tagName = "WiiMote");

  //! Destructor
  ~WiiMote();

  //! This method should not be called (implements thread code)
  void run();

  //! init the thread and connect to the wiimote
  void init();

  //! Get sensor data
  SensorData getSensorData();

protected:
  //! get started (see ModelComponent.H)
  void start1();

  //! get started (see ModelComponent.H)
  void start2();

  //! get stopped (see ModelComponent.H)
  void stop1();

private:
#ifdef HAVE_LIBWIIMOTE
  wiimote_t itsWiimote;
#endif
  pthread_t runner;
  pthread_mutex_t itsLock;
  mutable bool running;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
