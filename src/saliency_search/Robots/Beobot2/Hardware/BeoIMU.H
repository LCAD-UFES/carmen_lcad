/*!@file Robots2/Beobot2/Hardware/BeoIMU.H Ice Module for IMU           */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoIMU.H
// $Id: BeoIMU.H 12962 2010-03-06 02:13:53Z irock $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include <vector>
#include "Devices/IMU_MicroStrain_3DM_GX2.H"

#ifndef BEOIMUI_H
#define BEOIMUI_H

class BeoIMU : public RobotBrainComponent
{
public:

  BeoIMU(OptionManager& mgr,
         const std::string& descrName = "BeoIMU",
         const std::string& tagName = "BeoIMU");

  ~BeoIMU();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  //! register topics
  virtual void registerTopics();

  //! to select which serial port to use
  void configureSerial(std::string dev);

  //! set the requested data from the IMU
  bool setDataRequested(DataRequested dataRequested);

private:
  nub::soft_ref<IMU_MicroStrain_3DM_GX2> itsIMU;

  //std::vector<IMUData> itsData;
  //nub::soft_ref<OutputFrameSeries> itsOfs;
  Timer itsTimer;
  int itsCurrMessageID;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
