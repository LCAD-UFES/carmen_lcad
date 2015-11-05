/*!@file src/Robots/RobotBrain/StartsThe primary motor cortex service.C */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/RobotBrainService.C $
// $Id: RobotBrainService.C 12281 2009-12-17 09:00:36Z itti $
//

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/RobotBrain/PrimaryMotorCortexI.H"
#include "Robots/RobotBrain/HippocampusI.H"
#include "Robots/RobotBrain/PrimarySomatosensoryCortexI.H"
#include "Robots/RobotBrain/SupplementaryMotorAreaI.H"
#include "Robots/RobotBrain/InferotemporalCortexI.H"
#include "Robots/RobotBrain/PrefrontalCortexI.H"
#include "Robots/RobotBrain/LateralGeniculateNucleusI.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"


class RobotBrainServiceService : public Ice::Service {
  protected:
    virtual bool start(int, char* argv[]);
    virtual bool stop() {
      if (itsMgr)
        delete itsMgr;
      return true;
    }

  private:
    Ice::ObjectAdapterPtr itsAdapter;
    ModelManager *itsMgr;
};

bool RobotBrainServiceService::start(int argc, char* argv[])
{
  char adapterStr[255];

  //Create the topics
  SimEventsUtils::createTopic(communicator(), "ActionMessageTopic");
  SimEventsUtils::createTopic(communicator(), "GPSMessageTopic");
  SimEventsUtils::createTopic(communicator(), "MotionMessageTopic");
  SimEventsUtils::createTopic(communicator(), "GoalStateMessageTopic");
  SimEventsUtils::createTopic(communicator(), "GoalProgressMessageTopic");
  SimEventsUtils::createTopic(communicator(), "AttendedRegionMessageTopic");
  SimEventsUtils::createTopic(communicator(), "ObjectMessageTopic");
  SimEventsUtils::createTopic(communicator(), "LandmarksMessageTopic");

  //Create the adapter
  sprintf(adapterStr, "default -p %i", RobotBrainObjects::RobotBrainPort);
        itsAdapter = communicator()->createObjectAdapterWithEndpoints("RobotBrainPort",
      adapterStr);

  //Create the manager and its objects
        itsMgr = new ModelManager("RobotBrainService");

  LINFO("PMC");
  nub::ref<PrimaryMotorCortexI> pmc(new PrimaryMotorCortexI(*itsMgr));
        itsMgr->addSubComponent(pmc);
  pmc->init(communicator(), itsAdapter);

  LINFO("PSC");
  nub::ref<PrimarySomatosensoryCortexI> psc(new PrimarySomatosensoryCortexI(*itsMgr));
        itsMgr->addSubComponent(psc);
  psc->init(communicator(), itsAdapter);

  LINFO("LGN");
  nub::ref<LateralGeniculateNucleusI> lgn(new LateralGeniculateNucleusI(*itsMgr));
        itsMgr->addSubComponent(lgn);
  lgn->init(communicator(), itsAdapter);

  LINFO("Hipp");
  nub::ref<HippocampusI> hipp(new HippocampusI(*itsMgr));
        itsMgr->addSubComponent(hipp);
  hipp->init(communicator(), itsAdapter);

  //LINFO("SMA");
  //nub::ref<SupplementaryMotorAreaI> sma(new SupplementaryMotorAreaI(*itsMgr));
        //itsMgr->addSubComponent(sma);
  //sma->init(communicator(), itsAdapter);

  LINFO("IT");
  nub::ref<InferotemporalCortexI> itc(new InferotemporalCortexI(*itsMgr));
  itsMgr->addSubComponent(itc);
  itc->init(communicator(), itsAdapter);


  LINFO("PFC");
  nub::ref<PrefrontalCortexI> pfc(new PrefrontalCortexI(*itsMgr));
  itsMgr->addSubComponent(pfc);
  pfc->init(communicator(), itsAdapter);

        itsMgr->parseCommandLine((const int)argc, (const char**)argv, "", 0, 0);


        itsAdapter->activate();

  itsMgr->start();

        return true;
}

// ######################################################################
int main(int argc, char** argv) {

  RobotBrainServiceService svc;
  return svc.main(argc, argv);
}


