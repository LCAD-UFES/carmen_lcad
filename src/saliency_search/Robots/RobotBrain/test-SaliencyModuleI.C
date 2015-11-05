
#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/SeaBeeIII/SaliencyModuleI.H"
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
  SimEventsUtils::createTopic(communicator(), "SalientPointMessageTopic");

  //Create the adapter
  sprintf(adapterStr, "default -p %i", RobotBrainObjects::RobotBrainPort);
  itsAdapter = communicator()->createObjectAdapterWithEndpoints("SaliencyModule",
                                                                adapterStr);

  //Create the manager and its objects
  itsMgr = new ModelManager("SaliencyModuleService");

  LINFO("Starting SaliencyModule");
  nub::ref<SaliencyModuleI> ret(new SaliencyModuleI(*itsMgr));
  itsMgr->addSubComponent(ret);

  ret->init(communicator(), itsAdapter);

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


