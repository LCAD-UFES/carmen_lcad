/*!@file PrimarySomatosensoryCortex.C get sensors the actual robot */


//////////////////////////////////////////////////////////////////// //
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
// Primary maintainer for this file: Lior Elazary <lelazary@yahoo.com>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/PrimarySomatosensoryCortexI.H $
// $Id: PrimarySomatosensoryCortexI.H 12281 2009-12-17 09:00:36Z itti $
//

#ifndef PrimarySomatosensoryCortex_H_DEFINED
#define PrimarySomatosensoryCortex_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"

#include <Ice/Ice.h>
#include <IceUtil/Thread.h>
#include "Ice/IRobot.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"

class PrimarySomatosensoryCortexI : public ModelComponent, public RobotBrainObjects::PrimarySomatosensoryCortex, public IceUtil::Thread
{

public:

  PrimarySomatosensoryCortexI(OptionManager& mgr,
      const std::string& descrName = "PrimarySomatosensoryCortex",
      const std::string& tagName = "PrimarySomatosensoryCortex");
  ~PrimarySomatosensoryCortexI();

  virtual void run();

  //! Attache to the IRobot
  void init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter);

  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);
private:
  Ice::ObjectPrx            itsObjectPrx;
  Robots::IRobotPrx         itsRobot;
  RobotSimEvents::EventsPrx itsGPSPublisher;
  RobotSimEvents::EventsPrx itsMotionPublisher;
};

#endif
