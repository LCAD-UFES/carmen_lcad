/*!@file Devices/BeeStemTiny.H Simple interface to beestem */


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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeeStemTiny.H $
// $Id: BeeStemTiny.H 8623 2007-07-25 17:57:51Z rjpeters $
//

#ifndef BeeStemTiny_H_DEFINED
#define BeeStemTiny_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"      // for byte
#include <string>

//Motor max values (Warning: can blow fuses if set too high)
#define MOTOR_MAX 75

class Serial;

class BeeStemTiny : public ModelComponent
{
public:
  //! Default constructor; see ModelComponent.H
  BeeStemTiny(OptionManager& mgr,
      const std::string& descrName = "BeeStemTiny Driver",
      const std::string& tagName = "BeeStemTiny",
      const char *defdev = "/dev/ttyS1",
      const char *defdev2 = "/dev/ttyUSB0");

  //! Destructor
  ~BeeStemTiny();

  bool setThrusters(int &m1, int &m2, int &m3,
                 int &m4, int &m5);

  bool getSensors(int &heading, int &pitch, int &roll, int &ext_pressure, int &int_pressure);

  bool setHeartBeat();

  bool calibrateCrappyCompass();

protected:
  nub::soft_ref<Serial> itsPort;   //!< Serial port to use
  pthread_mutex_t itsSerialLock;
  std::vector<int> crappyCompassRaw;
  nub::soft_ref<Serial> itsCompassPort;

  std::list<int> itsAvgDepth;

  int itsLastMotorCmd[5];

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
