/*!@file Robots2/Beobot2/app-Beobot2GistSalMaster.C
  Robot navigation using a combination saliency and gist.
  Run app-Beobot2GistSalMaster at X1 to run Gist-Saliency model
  Run app-LandmarkDBWorker     at X[2 ... 8] to run SIFT recognition    */
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
// $HeadURL:
// svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/app-BeobotGistSalMaster.C
// $ $Id: app-Beobot2GistSalMaster.C 15441 2012-11-14 21:28:03Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"

#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"

#include "Robots/Beobot2/Localization/Beobot2GistSalMaster.H"
#include "Beobot/beobot-GSnav-def.H"
#include "Beobot/GSnavResult.H"

// ######################################################################
// ######################################################################
class RobotBrainServiceService : public Ice::Service {
  protected:
#if ICE_INT_VERSION >= 30402
    virtual bool start(int, char* argv[],int &);
#else
    virtual bool start(int, char* argv[]);
#endif	
    virtual bool stop() {
      if (itsMgr)
        delete itsMgr;
      return true;
    }

  private:
    Ice::ObjectAdapterPtr itsAdapter;
    ModelManager *itsMgr;
};

// ######################################################################
#if ICE_INT_VERSION >= 30402
bool RobotBrainServiceService::start(int argc, char* argv[],int& status)
#else
bool RobotBrainServiceService::start(int argc, char* argv[])
#endif	
{
  MYLOGVERB = LOG_INFO;

  char adapterStr[255];

  LINFO("Creating Topic!");

  // Create the topics
  // SimEventsUtils::createTopic
  // (communicator(), "Beobot2GistSalMasterMessageTopic");

  // Create the adapter
  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;

  // try to connect to ports until successful
  LDEBUG("Opening Connection");
  while(!connected)
  {
    try
    {
      LINFO("Trying Port:%d", port);
      sprintf(adapterStr, "default -p %i", port);
      itsAdapter = communicator()->createObjectAdapterWithEndpoints
        ("Beobot2GistSalMaster", adapterStr);
      connected = true;
    }
    catch(Ice::SocketException)
    {
      port++;
    }
  }

  //Create the manager and its objects
  itsMgr = new ModelManager("Beobot2GistSalMasterService");

  // Note the string was Beobot2GistSalMaster1 & 2
  LINFO("Starting Beobot2GistSalMaster");
  nub::ref<Beobot2GistSalMasterI> GSMaster
    (new Beobot2GistSalMasterI(*itsMgr, "Beobot2GistSalMaster",
                               "Beobot2GistSalMaster"));
  LINFO("Beobot2GistSalMaster Created");
  itsMgr->addSubComponent(GSMaster);
  LINFO("Beobot2GistSalMaster Added As Sub Component");
  GSMaster->init(communicator(), itsAdapter);
  LINFO("Beobot2GistSalMaster Initiated");

  // check command line inputs/options
  if (itsMgr->parseCommandLine((const int)argc, (const char**)argv,
                               "[input frame rate]", 0, 1)
      == false) return(1);

  int w = GSMaster->getIfs()->getWidth(), h = GSMaster->getIfs()->getHeight();
  std::string dims = convertToString(Dims(w, h));
  LINFO("image size: [%dx%d]", w, h);
  itsMgr->setOptionValString(&OPT_InputFrameDims, dims);
  itsMgr->setModelParamVal("InputFrameDims", Dims(w, h),
                           MC_RECURSE | MC_IGNORE_MISSING);

  // input frame rate
  uint64 inputFrameRate = 0;
  if(itsMgr->numExtraArgs() >  0)
    inputFrameRate = itsMgr->getExtraArgAs<uint>(0);
  LINFO("frame frequency: %f ms/frame", inputFrameRate/1000.0F);
  GSMaster->setInputFrameRate(inputFrameRate);

  // activate manager and adapter
  itsAdapter->activate();
  itsMgr->start();

  return true;
}


// ######################################################################
int main(int argc, char** argv) {

  LINFO("Creating Service...");
  RobotBrainServiceService svc;
  LINFO("Service Created...");
  return svc.main(argc, argv);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
