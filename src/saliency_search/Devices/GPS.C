/*!@file Devices/GPS.C Serial interface to an NMEA 0183 GPS unit (Garmin Geko 301) */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/GPS.C $
// $Id: GPS.C 14967 2011-10-07 21:02:21Z kai $
//

#include "Devices/GPS.H"
#include "Component/OptionManager.H"
#include "rutz/compat_snprintf.h"
#include <cstdlib>
#include <pthread.h>

// ######################################################################
GPSlistener::~GPSlistener()
{  }

// ######################################################################
GPSdata::GPSdata()
{ fixtype = 0; }

// ######################################################################
void GPSdata::toString(char *str, const int siz) const
{
  snprintf(str, siz, "Fix=%d/%d*%04d/%02d/%02d*%02d:%02d:%02d "
           "Lat=%.6f Lon=%.6f Alt=%.2f Galt=%.2f Sp=%.2f Heading=%.2f "
           "Mvar=%.2f Dil=%.1f/%.1f/%.1f Err=%.1f/%.1f/%.1f",
           fixtype, nsat, fixye, fixmo, fixda, fixho, fixmi,
           fixse, latitude, longitude, altitude, galtitude, speed, heading,
           magvar, pdil, hdil, vdil, epe, hpe, vpe);
}

// ######################################################################
bool GPSdata::fromString(const char *str)
{
  int ifixtype, insat, ifixho, ifixmi, ifixse, ifixda, ifixmo;
  if (sscanf(str, "Fix=%d/%d*%d/%d/%d*%d:%d:%d "
             "Lat=%lf Lon=%lf Alt=%f Galt=%f Sp=%f Heading=%f "
             "Mvar=%f Dil=%f/%f/%f Err=%f/%f/%f",
             &ifixtype, &insat, &fixye, &ifixmo, &ifixda, &ifixho, &ifixmi,
             &ifixse, &latitude, &longitude, &altitude, &galtitude, &speed,
             &heading, &magvar, &pdil, &hdil, &vdil, &epe, &hpe, &vpe) != 20)
    {
      LERROR("Conversion failed -- MARKING AS BAD");
      fixtype = 0;
      return false;
    }
  fixtype = byte(ifixtype); nsat = byte(insat); fixmo = byte(ifixmo);
  fixda = byte(ifixda); fixho = byte(ifixho); fixmi = byte(ifixmi);
  fixse = byte(ifixse);
  return true;
}

// ######################################################################
void *gps_run(void *c)
{
  GPS *cc = (GPS *)c;
  cc->run();
  return NULL;
}

// ######################################################################
GPS::GPS(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr, "GPS Serial Port", "GPSserial")),
  itsData(), itsKeepgoing(false), itsGotnew(false), itsListener(NULL)
{
  // set a default config for our serial port:
  itsSerial->configure("/dev/rfcomm4", 4800, "8N1", false, false, 20000);

  // adopt our serial port as a subcomponent:
  addSubComponent(itsSerial);

  // initialize our mutex:
  pthread_mutex_init(&itsLock, NULL);
}

// ######################################################################
void GPS::setListener(rutz::shared_ptr<GPSlistener>& listener)
{ itsListener = listener; }

// ######################################################################
GPS::~GPS()
{
  pthread_mutex_destroy(&itsLock);
}

// ######################################################################
void GPS::start2()
{
  // since we are in start2(), we know our serial is up already. So
  // here we just need to get our thread going:
  itsKeepgoing = true;
  pthread_create(&itsRunner, NULL, &gps_run, (void *)this);
}

// ######################################################################
void GPS::stop1()
{
  itsKeepgoing = false;
  usleep(300000); // make sure thread has exited
}

// ######################################################################
bool GPS::getData(GPSdata& data)
{
  pthread_mutex_lock(&itsLock);
  data = itsData; bool ret = itsGotnew; itsGotnew = false;
  pthread_mutex_unlock(&itsLock);
  return ret;
}

// ######################################################################
void GPS::run()
{
  char buf[100]; // NMEA 0183 limits sentences to 80 chars
  int i = 0, retry = 10, idx[50];
  float heading = 0.0f; // will decide btw GPS and compass based on speed
  bool gotstart = false;

  while(itsKeepgoing)
    {
      //if (retry < 0) LINFO("Too many serial errors");

      // let's receive the data, byte per byte:
      int ret = itsSerial->read(&buf[i], 1);
      if (ret == 0)
        { LDEBUG("Timeout on read() -- WAITING MORE");
        --retry; gotstart = false; continue; }
      if (ret != 1)
        { LDEBUG("read() error -- DROP"); buf[0] = 'X';
        --retry; gotstart = false; continue; }

      // read went well
      ++i; retry = 10;

      // buffer overflow?
      if (i >= 100)
        { LERROR("Serial buffer overflow -- TRASHING");
        i = 0; gotstart = false; continue; }

      // complete sentence received?
      if (buf[i-1] != '\n') continue;

      // data is ready for decoding:
      i -= 2; buf[i] = '\0';
      LDEBUG("Received: %s", buf);

      // check the checksum:
      if (i < 4) { LERROR("Short sentence -- DROP");
      i = 0; gotstart = false; continue; }
      byte chksum = 0; for (int j = 1; j < i - 3; j ++) chksum ^= buf[j];
      if (chksum != strtol(&buf[i-2], NULL, 16))
        { LERROR("Wrong checksum -- DROP"); i=0; gotstart = false; continue; }
      i -= 3; buf[i] = '\0';

      // start the decoding by splitting it into an array of strings:
      int n = 0; idx[n++] = 0;
      for (int j = 0; j < i; j ++)
        if (buf[j] == ',') { idx[n++] = j+1; buf[j] = '\0'; }

      // let's now fill up our data structure based on the sentence:
      bool triggerlistener = false;
      pthread_mutex_lock(&itsLock);
      if (strcmp(buf, "$GPGGA") == 0 && n == 15)
        {
          itsData.fixho = byte(str2d(&buf[idx[1]], 2));
          itsData.fixmi = byte(str2d(&buf[idx[1]+2], 2));
          itsData.fixse = byte(str2d(&buf[idx[1]+4], 2));

          itsData.latitude = str2d(&buf[idx[2]], 2) +
            str2d(&buf[idx[2]+2], 7) / 60.0;
          if (buf[idx[3]] == 'S') itsData.latitude = -itsData.latitude;

          itsData.longitude = str2d(&buf[idx[4]], 3) +
            str2d(&buf[idx[4]+3], 7) / 60.0;
          if (buf[idx[5]] == 'W') itsData.longitude = -itsData.longitude;

          itsData.nsat = byte(atoi(&buf[idx[7]]));

          if (buf[idx[10]] == 'M')
            itsData.galtitude = atof(&buf[idx[9]]);

          // we got our first sentence for a new fix:
          gotstart = true;
        }

      if (strcmp(buf, "$GPRMC") == 0 && n == 13)
        {
          itsData.speed = atof(&buf[idx[7]]) / 1.85200f; // knot to km/h
          heading = atof(&buf[idx[8]]);
          itsData.magvar = strtod(&buf[idx[10]], NULL);
          if (buf[idx[11]] == 'E') itsData.magvar = - itsData.magvar;
          itsData.fixda = byte(str2d(&buf[idx[9]], 2));
          itsData.fixmo = byte(str2d(&buf[idx[9]+2], 2));
          itsData.fixye = 2000 + byte(str2d(&buf[idx[9]+4], 2));
        }

      if (strcmp(buf, "$GPGSA") == 0 && n == 18)
        {
          itsData.fixtype = byte(atoi(&buf[idx[2]]) - 1);
          itsData.pdil = atof(&buf[idx[15]]);
          itsData.hdil = atof(&buf[idx[16]]);
          itsData.vdil = atof(&buf[idx[17]]);
        }

      if (strcmp(buf, "$PGRME") == 0 && n == 7)
        {
          if (buf[idx[2]] == 'M') itsData.hpe = atof(&buf[idx[1]]);
          if (buf[idx[4]] == 'M') itsData.vpe = atof(&buf[idx[3]]);
          if (buf[idx[6]] == 'M') itsData.epe = atof(&buf[idx[5]]);
        }

      if (strcmp(buf, "$PGRMZ") == 0 && n == 3)
        {
          if (buf[idx[2]] == 'f')
            itsData.altitude = atof(&buf[idx[1]]) * 0.3048;
        }

      if (strcmp(buf, "$HCHDG") == 0 && n == 6)
        {
          // this sentence comes after the GPRMC one which provides
          // GPS-derived true heading, valid when we are moving. If we
          // are not moving, we will use compass magnetic heading (and
          // convert it to true heading) instead:
          if (itsData.speed < 5.0)
            itsData.heading = atof(&buf[idx[1]]) - itsData.magvar;
          else
            itsData.heading = heading;

          // this is our last sentence, so once we have it we indicate
          // we have completed receiving a new fix:
          if (gotstart) { itsGotnew = true; triggerlistener = true; }
          gotstart = false;
        }
      itsGotnew = true; triggerlistener = true;
      pthread_mutex_unlock(&itsLock);

      // trigger our listener if a completed new block has arrived:
      if (triggerlistener && itsListener.get())
      {
        itsListener->newData(itsData);
      }

      // ready for next sentence:
      i = 0;
    }
}

// ######################################################################
double GPS::str2d(const char *s, const int nchar) const
{
  char tmp[nchar + 1]; tmp[nchar] = '\0';
  for (int i = 0; i < nchar; i ++) tmp[i] = s[i];
  return strtod(tmp, NULL);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
