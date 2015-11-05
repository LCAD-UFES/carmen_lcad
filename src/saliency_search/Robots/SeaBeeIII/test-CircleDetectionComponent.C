#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/SeaBeeIII/CircleDetectionComponent.H"
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

LINFO("Starting the Circle Detection Component");
  //Create the topics
 // SimEventsUtils::createTopic(communicator(), "CircleDetectionComponentMessageTopic");

  //Create the adapter
  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;

  while(!connected)
    {
      try
        {
          LINFO("Trying Port:%d", port);
          sprintf(adapterStr, "default -p %i", port);
          itsAdapter = communicator()->createObjectAdapterWithEndpoints("CircleDetectionComponent",
                                                                        adapterStr);
          connected = true;
        }
      catch(Ice::SocketException)
        {
          port++;
        }
    }

  //Create the manager and its objects
  itsMgr = new ModelManager("CircleDetectionComponentService");

  LINFO("Starting CircleDetectionComponent");
  nub::ref<CircleDetectionComponent> ret(new CircleDetectionComponent(0, *itsMgr, "CircleDetectionComponent1", "CircleDetectionComponent2"));
  LINFO("CircleDetectionComponent Created");
  itsMgr->addSubComponent(ret);
  LINFO("CircleDetectionComponent Added As Sub Component");
  ret->init(communicator(), itsAdapter);
  LINFO("CircleDetectionComponent Inited");

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


