/*!@file Robots2/Beobot2/Hardware/BeoTiltLRF.H Ice Module for an LRF        */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoTiltLRF.H
// $Id: BeoTiltLRF.H 15336 2012-07-19 21:26:31Z kai $
//
#include "Image/DrawOps.H"

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "Raster/Raster.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"


#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/BeobotEvents.ice.H"
#include <IceUtil/Thread.h>

#include <vector>
#include "Robots/LoBot/io/LoLaserRangeFinder.H"
#include "Devices/Dynamixel.H"

#ifndef BEOTILTLRFI_H
#define BEOTILTLRFI_H

class BeoTiltLRF : public RobotBrainComponent
{
public:

  BeoTiltLRF(OptionManager& mgr,
         const std::string& descrName = "BeoTiltLRF",
         const std::string& tagName = "BeoTiltLRF");

  ~BeoTiltLRF();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();
	virtual void start3();

  //! set draw image flag
  void setDrawImage(bool onoff);
	void drawRobotBody();
	void drawRobotBody(Point2D<int>pt,int count,int dist);

  void writePCDheader(std::string fname);
  void writePCD(Image<int> dists,float pitch);
private:
  //nub::soft_ref<lobot::LaserRangeFinder> itsLRF;
  lobot::LaserRangeFinder* itsLRF;


  nub::soft_ref<Dynamixel> itsServo;

  //std::vector<LRFData> itsData;
  nub::soft_ref<OutputFrameSeries> itsOfs;

  // stored raw previous LRF reading
  std::vector<int> itsPreviousRawLRFreading;

  int itsServoMax;
  int itsServoMin;
  int itsServoCmd;
  int itsPointsTotal;
  int itsScanSpeed;
  std::vector<std::string> itsPointCloud;

  //!
  Timer itsTimer;
  std::vector<float> itsRecentTimes;
  float itsFrameRate;
  float itsPreviousTime;


  Image<PixRGB<byte> > itsDrawImage;
  int itsCurrMessageID;
  Image<int> itsDists;
  //std::string itsDevice;

  //!
  OModelParam<std::string> itsDevice;

  //! label for device
  OModelParam<int> itsDeviceLabel;

  //! servo serial port 
  OModelParam<int> itsServoSerialPort;

  //! turn GUI on/off
  OModelParam<bool> itsGUIstatus;

  FILE * itsPCDfile;
  pcl::PointCloud<pcl::PointXYZ>::Ptr itsCloud;
  //pcl::visualization::CloudViewer itsCloudViewer;
  //pcl::visualization::PCLVisualizer itsViewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> itsViewer;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
