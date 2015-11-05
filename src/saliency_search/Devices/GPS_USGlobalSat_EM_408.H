/*!@file Devices/GPS_USGlobalSat_EM_408.H class
 for interfacing with the USGlobalSat EM-408 GPS */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/GPS_USGlobalSat_EM_408.H $
// $Id: GPS_USGlobalSat_EM_408.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef GPS_USGLOBALSAT_EM_408_H_DEFINED
#define GPS_USGLOBALSAT_EM_408_H_DEFINED

#include "Component/OptionManager.H"
#include "Devices/Serial.H"
#include "Util/Angle.H"

typedef struct {
  double lat,lon;
        int precision;
        int satNum;
} GpsData;

//! Class for interfacing with the IMU
//! It sends X,Y,Z acceleration and X,Y,Z angular velocity
class GPS_USGlobalSat_EM_408: public ModelComponent
{
public:
  //! Initialize
  GPS_USGlobalSat_EM_408
  (OptionManager& mgr,
   const std::string& descrName = "GPS_USGlobalSat_EM_408",
   const std::string& tagName   = "GPS_USGlobalSat_EM_408",
   const std::string& dev       = "/dev/ttyUSB0");

  //! Destructor
  ~GPS_USGlobalSat_EM_408();

  //! to select which serial port to use
  void configureSerial(std::string dev);

  //!
  void run();

  //! is there a new data since it is last accessed
  bool newData();

protected:
  void start1(); //!< get started
  void stop1();  //!< get stopped

private:

  nub::soft_ref<Serial> itsSerial;

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
