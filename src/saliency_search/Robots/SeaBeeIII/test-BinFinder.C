
#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/SeaBeeIII/BinFinder.H"
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

  LINFO("Creating Topic!");
  //Create the topics
//  SimEventsUtils::createTopic(communicator(), "BinFinderMessageTopic");

  //Create the adapter
  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;
  LDEBUG("Opening Connection");

  while(!connected)
  {
    try
    {
      LINFO("Trying Port:%d", port);
      sprintf(adapterStr, "default -p %i", port);
      itsAdapter = communicator()->createObjectAdapterWithEndpoints
        ("BinFinder",
          adapterStr);
      connected = true;
    }
    catch(Ice::SocketException)
    {
      port++;
    }
  }

  //Create the manager and its objects
        itsMgr = new ModelManager("BinFinderService");

  LINFO("Starting BinFinder");
  nub::ref<BinFinder> ret
    (new BinFinder(*itsMgr, "BinFinder1", "BinFinder2"));
  LINFO("BinFinder Created");
  itsMgr->addSubComponent(ret);
  LINFO("BinFinder Added As Sub Component");
  ret->init(communicator(), itsAdapter);
  LINFO("BinFinder Inited");

  itsMgr->parseCommandLine((const int)argc, (const char**)argv, "", 0, 0);

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


