
#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"
#include "Robots/SeaBeeIII/ColorSegmenterI.H"


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

  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;

  while(!connected)
  {
    try
    {
      LINFO("Trying Port: %d", port);
      sprintf(adapterStr, "default -p %i", port);
      itsAdapter = communicator()->createObjectAdapterWithEndpoints("ColorSegmenter", adapterStr);
      connected = true;
    } catch(Ice::SocketException)
    {
      port++;
    }
  }

  //Create the manager and its objects
  itsMgr = new ModelManager("ColorSegmenterService");

  LINFO("Starting ColorSegmenter");
  nub::ref<ColorSegmenterI> ret(new ColorSegmenterI(*itsMgr));
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
