/*!@file Robots2/Beobot2/Hardware/BeoCamera.H Ice Module for camera     */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoCamera.H
// $Id: BeoCamera.H 15431 2012-11-06 05:17:38Z kai $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

//#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Media/FrameSeries.H"
//#include "Transport/FrameIstream.H"
#include "Robots/Beobot2/BeoCommon.H"


#include <vector>

#ifndef BEOCAMERAI_H
#define BEOCAMERAI_H

class BeoCamera : public RobotBrainComponent
{
public:

  BeoCamera(OptionManager& mgr,
         const std::string& descrName = "BeoCamera",
         const std::string& tagName = "BeoCamera");

  ~BeoCamera();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  //! register topics
  virtual void registerTopics();

  //! to set the log mode and folder
  void start3();

private:

  bool initLogFolder();

  nub::soft_ref<InputFrameSeries> itsIfs;
  nub::soft_ref<OutputFrameSeries> itsOfs;

  std::vector<uint16> itsGamma;


  Timer itsTimer;
  Timer itsDisplayTimer;
  std::vector<float> itsRecentTimes;
  float itsFrameRate;
  float itsPreviousTime;


  int itsCurrMessageID;
  int itsPrevDisplayMessageID;

  //! The display refresh rate, default is 30 fps
  OModelParam<float>       itsDisplayFps;

  //! label for device
  OModelParam<int> itsDeviceLabel;

  OModelParam<bool>   itsLogMode;
  OModelParam<std::string> itsLogFolderName;
  IceUtil::Mutex its_logFolderName_mutex;  //!< locking log folder name
  //std::string itsLogFoldername; 

  float itsDisplayPeriod;

  bool itsSaveImage;
  bool itsSendImage;

};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
