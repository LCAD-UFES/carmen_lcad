/*!@file Robots2/Beobot2/Hardware/BeoGPS.C Ice Module for GPS           */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoGPS.C
// $ $Id: BeoGPS.C 15081 2011-11-21 20:56:30Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Hardware/BeoGPS.H"
#include "Ice/BeobotEvents.ice.H"


// const ModelOptionCateg MOC_BeoGPS = {
//         MOC_SORTPRI_3, "Beobot GPS Related Options" };

// const ModelOptionDef OPT_SerialDev =
// { MODOPT_ARG(std::string), "SerialDev", &MOC_BeoGPS, OPTEXP_CORE,
//         "The serial device file",
//         "serial-dev", '\0', "/dev/ttyUSBX", "/dev/ttyUSB1"};

// #####################################################################
BeoGPS::BeoGPS(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr)),
  itsOfs(new OutputFrameSeries(mgr)),
  itsDisplayUpdateRate(.05),
  //  itsSerialDev(&OPT_SerialDev, this, 0),
  itsPosition(0.0,0.0,0.0),
  itsInitLat(-1.0),
  itsInitLon(-1.0),
  itsDispImage(512,512,ZEROS),
  itsCurrMessageID(0)
{
  addSubComponent(itsSerial);
  addSubComponent(itsOfs);
	itsData.latDD = -1;
	itsData.lonDD = -1;
	itsBufI = 0; 
	itsData.lat = -1;
	itsData.lon = -1;
	itsNewGPSdata = false;
}

// ######################################################################
BeoGPS::~BeoGPS()
{

}

// ######################################################################
void BeoGPS::start1()
{
  //itsSerial->configure("/dev/ttyUSB0", 115200);

	//To start SPP bluetooth gps, run $sudo rfcomm connect 4
  itsSerial->configure("/dev/rfcomm4", 115200);//bluetooth gps

//   itsSerial->configure
//     (itsSerialDev.getVal().c_str(), 115200, "8N1", false, false, 0);

  // Check to ensure that our ttyUSB devices are set up correctly
  // by polling their IDs
//   LINFO("Checking for Sensor board");
//   unsigned char cmd = 0;
//   std::vector<unsigned char> propIDvec = itsSerial->readFrame(cmd, 1, .5);

//   std::string propID(propIDvec.begin(), propIDvec.end());

//   if(propID == "")
//     LFATAL("ERROR! Unrecognized device on %s",
//            itsSerialDev.getVal().c_str());
//   else if(propID != "sensorboard")
//     LFATAL("ERROR! Incorrect device on %s: %s",
//            itsSerialDev.getVal().c_str(), propID.c_str());

//   LINFO("%s found on %s", propID.c_str(), itsSerialDev.getVal().c_str());
}

// ######################################################################
void BeoGPS::registerTopics()
{
  registerPublisher("GPSMessageTopic");
}

// ######################################################################
void BeoGPS::evolve()
{
  
	//getPropGPS();
	getStdGPS();
	if(((itsData.latDD == 34 && itsData.lonDD == -118)) && itsNewGPSdata )
	{
		itsNewGPSdata = false;
		if(itsInitLat == -1.0 )
		{

			LINFO("Set init GPS point  Lat: %f Lon %f",itsData.lat,itsData.lon);
			itsInitLat = itsData.lat;
			itsInitLon = itsData.lon;
		}
		updatePosition(itsData.lat,itsData.lon);

		BeobotEvents::GPSMessagePtr msg = new BeobotEvents::GPSMessage;
		msg->latitude  = itsData.lat;
		msg->longitude = itsData.lon;
                msg->x = itsPosition.x;
                msg->y = itsPosition.y;
		msg->precision = itsData.precision;
		msg->satNum    = itsData.satNum;

		msg->RequestID = itsCurrMessageID;

		//LINFO("initLat %f initLon%f",itsInitLat,itsInitLon);
		LINFO("[%6d]Publishing GPS report Lat: %f Lon %f Precision %d SatNum %d",
				itsCurrMessageID, 
				itsData.lat, itsData.lon,itsData.precision,itsData.satNum);
		this->publish("GPSMessageTopic", msg);

		itsCurrMessageID++;
	}
	//else
	//	LINFO("Waiting GPS... Lat: %f Lon %f Precision %d SatNum %d", 
	//			itsData.lat, itsData.lon,itsData.precision,itsData.satNum);


	// Plot the location
	plotGPS();
  usleep(100);
}
// ######################################################################
void BeoGPS::updatePosition(double lat,double lon)
{
  //Find Y movement
  double R = 6378.7 ; //Earth Radians (KM)
  double theta = DEG2RAD(itsInitLon - lon);
  double d1 = sin(DEG2RAD(itsInitLat)) * sin(DEG2RAD(itsInitLat));
  double d2 = cos(DEG2RAD(itsInitLat)) * cos(DEG2RAD(itsInitLat)) * cos(theta);
  double y = acos(d1 + d2) * R * 1000.0;//meter

  //Find X movement
  d1 = sin(DEG2RAD(itsInitLat)) * sin(DEG2RAD(lat));
  d2 = cos(DEG2RAD(itsInitLat)) * cos(DEG2RAD(lat));
  double x = acos(d1 + d2) * R * 1000.0;//meter

  if(x != 0.0 && y != 0.0)
    {
      itsPosition.x = (itsInitLat > lat) ? x : -x;
      itsPosition.y = (itsInitLon > lon) ? y : -y;
    }

  LINFO("Publishing GPS report Lat: %f Lon: %f  X: %f Y: %f",
        lat, lon, itsPosition.x, itsPosition.y);
}

// ######################################################################
void BeoGPS::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                           const Ice::Current&)
{ }

// ######################################################################
void BeoGPS::plotGPS()
{
   if(itsDisplayTimer.getSecs() > itsDisplayUpdateRate )
     {
       itsDisplayTimer.reset();

       //Draw The Map
       double mapScale = 1.0;
       Point2D<int> drawPos
         (int(itsPosition.x*mapScale + itsDispImage.getWidth()/2),
          int(itsPosition.y*mapScale + itsDispImage.getHeight()/2));
       if(itsDispImage.coordsOk(drawPos))
         itsDispImage.setVal(drawPos, PixRGB<byte>(0,255,0));


       //itsDispImage.setVal(Point2D<int>(100,100), PixRGB<byte>(0,255,255));

       Image<PixRGB<byte> > itsDrawitsDispImage(itsDispImage);
       drawCircle(itsDrawitsDispImage, drawPos, 7, PixRGB<byte>(255,255,255));

                         char buffer[128];
                         sprintf(buffer, "X=%2.2f Y=%2.2f Lon: %1.5f Lat: %1.5f PCSN: %-3d SatNum: %-3d",
                                         itsPosition.x,itsPosition.y,
                                         itsData.lat,itsData.lon,
                                         itsData.precision,itsData.satNum);
                         writeText(itsDispImage, Point2D<int>(0,0), buffer, PixRGB<byte>(255,255,255),
                                         PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

			 //LDEBUG("%s",buffer);
       itsOfs->writeRGB(itsDispImage, "GPSData", FrameInfo("GPSData",SRC_POS));
       itsOfs->updateNext();
     }
}
// ######################################################################
void BeoGPS::getPropGPS()
{
	
  // Request a set of GPS readings from the sensor board
  //std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 1, .2);
  unsigned char cmd = 97;
  itsSerial->write(&cmd, 1);

  unsigned char buffer[256];
  itsSerial->read(&buffer,20);

  std::vector<unsigned char> frame(buffer+2, buffer+12);
  // If we recieved the ex./pected 20 bytes back
  // (4 bytes for each of the 5 gpss) then let's parse them.
  // The resulting values are the number of clock ticks
  // at 80Mhz that it took for the gps pulse to leave the gps,
  // bounce off of the nearest object, and return
  if(frame.size() == 10)
    {
      int latDD = frame[0];
      int latMM = frame[1];

      int latMMMM = ((0x0FF & frame[2])  << 8)  |
                                ((0x0FF & frame[3])  << 0);

      int lonDD = frame[4];
      int lonMM = frame[5];

      int lonMMMM = ((0x0FF & frame[6])  << 8)  |
                                ((0x0FF & frame[7])  << 0);

      int precision = frame[8];
      int satNum = frame[9];

      double lat = latDD + (latMM * 60.0 + latMMMM * 60.0 / 10000.0) / 3600.0;
      double lon = lonDD + (lonMM * 60.0 + lonMMMM * 60.0 / 10000.0) / 3600.0;

			itsData.lat = lat;
			itsData.lon = lon;
			itsData.precision = precision;
			itsData.satNum = satNum;
			itsData.latDD = latDD;
			itsData.lonDD = -lonDD;

}
  else
    {
      LERROR("Invalid Frame Size Received from GPS!");
			itsData.lat = -1;
			itsData.lon = -1;
			itsData.latDD = -1;
			itsData.lonDD = -1;
    }
}

// ######################################################################
void BeoGPS::getStdGPS()
{
	float heading = 0.0f; // will decide btw GPS and compass based on speed

      //if (retry < 0) LINFO("Too many serial errors");

      // let's receive the data, byte per byte:
      int ret = itsSerial->read(&itsBuf[itsBufI], 1);
      if (ret == 0)
        { 
				LDEBUG("Timeout on read() -- WAITING MORE");
				return;
				}
      if (ret != 1)
        { LDEBUG("read() error -- DROP"); itsBuf[0] = 'X';
					return;
				}

      // read went well
			++itsBufI;

      // buffer overflow?
      if (itsBufI >= 100)
        { LERROR("Serial buffer overflow -- TRASHING");
				itsBufI = 0; return;}

      // complete sentence received?
      if (itsBuf[itsBufI-1] != '\n') return;

      // data is ready for decoding:
      itsBufI -= 2; itsBuf[itsBufI] = '\0';
      LDEBUG("Received: %s", itsBuf);

      // check the checksum:
      if (itsBufI < 4) { LERROR("Short sentence -- DROP");
      itsBufI = 0; return;}
      byte chksum = 0; for (int j = 1; j < itsBufI - 3; j ++) chksum ^= itsBuf[j];
      if (chksum != strtol(&itsBuf[itsBufI-2], NULL, 16))
        { LERROR("Wrong checksum -- DROP"); itsBufI = 0; return; }
      itsBufI -= 3; itsBuf[itsBufI] = '\0';

      // start the decoding by splitting it into an array of strings:
      int n = 0; itsIdx[n++] = 0;
      for (int j = 0; j < itsBufI; j ++)
        if (itsBuf[j] == ',') { itsIdx[n++] = j+1; itsBuf[j] = '\0'; }

      // let's now fill up our data structure based on the sentence:
      bool gotAllData= false;
      if (strcmp(itsBuf, "$GPGGA") == 0 && n == 15)
        {
          itsGPSData.fixho = byte(str2d(&itsBuf[itsIdx[1]], 2));
          itsGPSData.fixmi = byte(str2d(&itsBuf[itsIdx[1]+2], 2));
          itsGPSData.fixse = byte(str2d(&itsBuf[itsIdx[1]+4], 2));

          itsGPSData.latitude = str2d(&itsBuf[itsIdx[2]], 2) +
            str2d(&itsBuf[itsIdx[2]+2], 7) / 60.0;
					itsGPSData.latDD = 	str2d(&itsBuf[itsIdx[2]], 2);					
          if (itsBuf[itsIdx[3]] == 'S') itsGPSData.latitude = -itsGPSData.latitude;
          if (itsBuf[itsIdx[3]] == 'S') itsGPSData.latDD = -itsGPSData.latDD;

          itsGPSData.longitude = str2d(&itsBuf[itsIdx[4]], 3) +
            str2d(&itsBuf[itsIdx[4]+3], 7) / 60.0;
          itsGPSData.lonDD = str2d(&itsBuf[itsIdx[4]], 3); 
          if (itsBuf[itsIdx[5]] == 'W') itsGPSData.longitude = -itsGPSData.longitude;
          if (itsBuf[itsIdx[5]] == 'W') itsGPSData.lonDD = -itsGPSData.lonDD;

          itsGPSData.nsat = byte(atoi(&itsBuf[itsIdx[7]]));

          if (itsBuf[itsIdx[10]] == 'M')
            itsGPSData.galtitude = atof(&itsBuf[itsIdx[9]]);

          // we got our first sentence for a new fix:
          gotAllData= true; 
          //gotstart = true;
					LDEBUG("lon %f lat %f lonDD %d latDD %d numSat %d",
						itsGPSData.longitude,
						itsGPSData.latitude,
						itsGPSData.lonDD,
						itsGPSData.latDD,
						itsGPSData.nsat);

        }

      if (strcmp(itsBuf, "$GPRMC") == 0 && n == 13)
        {
          itsGPSData.speed = atof(&itsBuf[itsIdx[7]]) / 1.85200f; // knot to km/h
          heading = atof(&itsBuf[itsIdx[8]]);
          itsGPSData.magvar = strtod(&itsBuf[itsIdx[10]], NULL);
          if (itsBuf[itsIdx[11]] == 'E') itsGPSData.magvar = - itsGPSData.magvar;
          itsGPSData.fixda = byte(str2d(&itsBuf[itsIdx[9]], 2));
          itsGPSData.fixmo = byte(str2d(&itsBuf[itsIdx[9]+2], 2));
          itsGPSData.fixye = 2000 + byte(str2d(&itsBuf[itsIdx[9]+4], 2));


					LDEBUG("Current Date YMD  %d/%d/%d ",
					itsGPSData.fixye,
					itsGPSData.fixmo,
					itsGPSData.fixda
					);


        }

			//Satellites in view
			//signal not acquired : n = 7
			//signal acquired: n = 20
      if (strcmp(itsBuf, "$GPGSV") == 0 && (n == 20|| n ==7))
        {
					LDEBUG("GOT $GPGSV n = %d",n);
        }
      if (strcmp(itsBuf, "$GPGSA") == 0 && n == 18)
        {
          itsGPSData.fixtype = byte(atoi(&itsBuf[itsIdx[2]]) - 1);
          itsGPSData.pdil = atof(&itsBuf[itsIdx[15]]);
          itsGPSData.hdil = atof(&itsBuf[itsIdx[16]]);
          itsGPSData.vdil = atof(&itsBuf[itsIdx[17]]);
        }

      if (strcmp(itsBuf, "$PGRME") == 0 && n == 7)
        {
          if (itsBuf[itsIdx[2]] == 'M') itsGPSData.hpe = atof(&itsBuf[itsIdx[1]]);
          if (itsBuf[itsIdx[4]] == 'M') itsGPSData.vpe = atof(&itsBuf[itsIdx[3]]);
          if (itsBuf[itsIdx[6]] == 'M') itsGPSData.epe = atof(&itsBuf[itsIdx[5]]);
        }

      if (strcmp(itsBuf, "$PGRMZ") == 0 && n == 3)
        {
          if (itsBuf[itsIdx[2]] == 'f')
            itsGPSData.altitude = atof(&itsBuf[itsIdx[1]]) * 0.3048;
        }

      if (strcmp(itsBuf, "$HCHDG") == 0 && n == 6)
        {
          // this sentence comes after the GPRMC one which provides
          // GPS-derived true heading, valid when we are moving. If we
          // are not moving, we will use compass magnetic heading (and
          // convert it to true heading) instead:
          if (itsGPSData.speed < 5.0)
            itsGPSData.heading = atof(&itsBuf[itsIdx[1]]) - itsGPSData.magvar;
          else
            itsGPSData.heading = heading;

          // this is our last sentence, so once we have it we indicate
          // we have completed receiving a new fix:
        }
      // output data if a completed new block has arrived:
      if (gotAllData)
      {
				itsData.lat 			= itsGPSData.latitude;
				itsData.lon 			= itsGPSData.longitude;
				itsData.precision = itsGPSData.pdil;
				itsData.satNum 		= itsGPSData.nsat;
				itsData.latDD 		= itsGPSData.latDD;
				itsData.lonDD 		= itsGPSData.lonDD;
				itsNewGPSdata = true;
      }

      // ready for next sentence:
      itsBufI = 0;
    
}

// ######################################################################
double BeoGPS::str2d(const char *s, const int nchar) const
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
