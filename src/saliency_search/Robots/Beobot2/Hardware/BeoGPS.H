/*!@file Robots2/Beobot2/Hardware/BeoGPS.H Ice Module for GPS           */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoGPS.H
// $Id: BeoGPS.H 14968 2011-10-07 22:22:47Z kai $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/Point3D.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Devices/GPS.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include <vector>

#include "Devices/Serial.H"

#ifndef BEOGPSI_H
#define BEOGPSI_H

#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))
typedef struct {
	double lat,lon;
	int precision;
	int satNum;
	int latDD,lonDD;
} GpsData;

class BeoGPS : public RobotBrainComponent
{
public:

  BeoGPS(OptionManager& mgr,
         const std::string& descrName = "BeoGPS",
         const std::string& tagName = "BeoGPS");
  ~BeoGPS();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();

  //!
  void getData();

  //!
  void plotGPS();

	//! Beobot Propeller GPS
	void getPropGPS();

	//! Standard GPS such as Garmin,Holux...
	void getStdGPS();
  //!
  void start1();

	double str2d(const char *s, const int nchar) const;
private:
  nub::soft_ref<Serial> itsSerial;

  GpsData itsData;
	GPSdata itsGPSData; //from Laurent's code
  nub::soft_ref<OutputFrameSeries> itsOfs;
  Timer itsDisplayTimer;
  float itsDisplayUpdateRate;

  //  OModelParam<std::string> itsSerialDev;
  Point3D<double> itsPosition;
  void updatePosition(double lat,double lon);
  double itsInitLat,itsInitLon;
  Image<PixRGB<byte> > itsDispImage;

  int itsCurrMessageID;

	bool itsNewGPSdata;


	char itsBuf[100]; // NMEA 0183 limits sentences to 80 chars
	int itsBufI; 
	int itsIdx[50];


};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
