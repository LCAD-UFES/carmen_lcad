/*!@file Robots2/Beobot2/Hardware/BeoLRF.H Ice Module for an LRF        */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoLRF.H
// $Id: BeoLRF.H 15336 2012-07-19 21:26:31Z kai $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "Raster/Raster.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/DrawOps.H"
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

#ifndef BEOLRFI_H
#define BEOLRFI_H

class BeoLRF : public RobotBrainComponent
{
public:

  BeoLRF(OptionManager& mgr,
         const std::string& descrName = "BeoLRF",
         const std::string& tagName = "BeoLRF");

  ~BeoLRF();

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

private:
  //nub::soft_ref<lobot::LaserRangeFinder> itsLRF;
  lobot::LaserRangeFinder* itsLRF;
  //std::vector<LRFData> itsData;
  nub::soft_ref<OutputFrameSeries> itsOfs;

  // stored raw previous LRF reading
  std::vector<int> itsPreviousRawLRFreading;

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

  //! turn GUI on/off
  OModelParam<bool> itsGUIstatus;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
