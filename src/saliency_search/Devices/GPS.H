/*!@file Devices/GPS.H Serial interface to an NMEA 0183 GPS unit (Garmin Geko 301) */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/GPS.H $
// $Id: GPS.H 14967 2011-10-07 21:02:21Z kai $
//

#ifndef GPS_H_DEFINED
#define GPS_H_DEFINED

#include "Component/ModelComponent.H"
#include "Devices/Serial.H"
#include "Util/Types.H" // for byte

//! Data received from the GPS unit
class GPSdata {
public:
  //! Constructor with empty data (indicated by fix type 0)
  GPSdata();

  byte fixtype;     //!< fix type (0=none/invalid data, 1=2D, 2=3D)

  byte nsat;        //!< Number of satellites in use (0..12)

  byte fixho;       //!< UTC time of fix, hours
  byte fixmi;       //!< UTC time of fix, minutes
  byte fixse;       //!< UTC time of fix, seconds
  byte fixda;       //!< UTC date of fix, day
  byte fixmo;       //!< UTC date of fix, month
  int fixye;        //!< UTC date of fix, year

  double latitude;  //!< latitude (>=0 for North)
  double longitude; //!< longitude (>=0 for East)
  float galtitude;  //!< GPS-derived altitude, in meters
  float altitude;   //!< pressure-sensor derived altitude, in meters
  float speed;      //!< course speed, in km/h
  float heading;    //!< heading, true, in deg
  float magvar;     //!< magnetic variation, deg (add to true for mag heading)

  float pdil;       //!< Precision dilution on position (smaller is better)
  float hdil;       //!< Horiz precision dilution (smaller is better)
  float vdil;       //!< Vertic precision dilution (smaller is better)

  float epe;        //!< Estimated position error, in meters
  float hpe;        //!< Estimated horiz position error, in meters
  float vpe;        //!< Estimated vertic position error, in meters

	int latDD;				//!< latitude minutes	
	int lonDD;				//!< longitude minutes

  //! Convert to string
  /*! At least siz bytes should have been pre-allocated to str */
  void toString(char *str, const int siz) const;

  //! Convert from string
  /*! Returns true on success */
  bool fromString(const char *str);
};

//! GPS event listener
/*! The GPSlistener is called each time a new GPS fix data block is
  received. This is a virtual base class and users should define their
  own derived classes that will do something when a frame is
  received. */
class GPSlistener {
public:
  //! Destructor
  virtual ~GPSlistener();

  //! New GPS data available
  virtual void newData(const GPSdata& data) = 0;
};

//! Interface with a GPS unit (tested with Garmin Geko 301) using NMEA 0183
/*! This is a multi-threaded class which continuously polls the unit
  for data and updates an internal copy of a GPSdata struct. Users can
  get the most up-to-date data at any time using getData(), or can
  setup a GPSlistener that will be called each time new data is
  available. Note that communication with the GPS unit operates at
  4800 bauds, so the actual rate of data refresh is quite low (2s) and
  querying for data more often than once every two seconds is useless.

  The Geko 301 has a true compass and altimeter/barometer. These need
  to be queried separately from the standard GPS summary query. This
  driver achieves that.

  The Geko 301 is stated to support the following NMEA 0183 sentences:
  GPGGA, GPGLL, GPGSA, GPGSV, GPRMB, GPRMC, GPTRE, GPVTG, GPWPL,
  GPBOD, PGRME, PGRMM, PGRMZ, PSLIB.  Not all of the sentences may be
  used by this driver, however (in particular, waypoint functionality
  can be more easily achieved in the main C++ code than in the GPS
  unit).

  Here is a typical sequence emitted by the Geko 301 when set in NMEA
  output mode:

  $GPRMC,082158,A,3403.4155,N,11815.0936,W,0.0,13.9,211104,13.5,E,A*05
  $GPRMB,A,,,,,,,,,,,,A,A*0B
  $GPGGA,082158,3403.4155,N,11815.0936,W,1,08,1.1,127.6,M,-31.8,M,,*7A
  $GPGSA,A,3,01,03,,14,15,18,19,21,22,,,,2.2,1.1,1.9*3B
  $GPGSV,3,1,10,01,27,216,36,03,40,254,41,09,19,058,00,14,75,207,39*7B
  $GPGSV,3,2,10,15,74,090,42,18,35,058,42,19,34,296,37,21,27,123,43*72
  $GPGSV,3,3,10,22,60,020,35,25,02,184,00*72
  $GPGLL,3403.4155,N,11815.0936,W,082158,A,A*52
  $GPBOD,,T,,M,,*47
  $PGRME,4.5,M,6.7,M,8.0,M*26
  $PGRMZ,424,f*06
  $HCHDG,92.6,,,13.5,E*23
  $GPRTE,1,1,c,*37
 */
class GPS : public ModelComponent {
public:
  //! Constructor
  /*! You can set which serial device to use by setting the
    ModelParameter GPSDevName */
  GPS(OptionManager& mgr, const std::string& descrName = "GPS",
      const std::string& tagName = "GPS");

  //! setup a listener that will be called each time new data is received
  void setListener(rutz::shared_ptr<GPSlistener>& listener);

  //! destructor
  virtual ~GPS();

  //! Get current data from the GPS unit
  /*! returns true if returned data comes from a completed new GPS fix
    compared to last time we were called. Note that some of the data
    may still differ even if the return value is false, if we have
    partially received and parsed new data from a new fix. */
  bool getData(GPSdata& data);

  //! Running thread
  /*! Do not call directly, this is called by our GPS polling thread */
  void run();

protected:
  void start2(); //!< get started
  void stop1();  //! get stopped
  nub::soft_ref<Serial> itsSerial;  //!< our serial port

private:
  double str2d(const char *s, const int nchar) const;

  GPSdata itsData;              // our internal copy of the data
  bool itsKeepgoing;            // regulates our running thread
  bool itsGotnew;               // true if new data since last getData()
  rutz::shared_ptr<GPSlistener> itsListener; // our listener
  pthread_t itsRunner;          // our GPS polling thread
  pthread_mutex_t itsLock;      // mutex to access our data
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
