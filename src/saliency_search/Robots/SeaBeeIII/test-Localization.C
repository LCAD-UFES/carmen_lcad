
#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/RobotBrain/RetinaI.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"

// My stuff
#include "Robots/SeaBeeIII/Localization.H"



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
  SimEventsUtils::createTopic(communicator(), "RetinaMessageTopic");

  //Create the adapter
  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;

  while(!connected)
  {
    try
    {
      LINFO("Trying Port:%d", port);
      sprintf(adapterStr, "default -p %i", port);
      itsAdapter = communicator()->createObjectAdapterWithEndpoints("Retina",
          adapterStr);
      connected = true;
    }
    catch(Ice::SocketException)
    {
      port++;
    }
  }

  //Create the manager and its objects
        itsMgr = new ModelManager("LocalizationService");

  LINFO("Starting Retina");
  nub::ref<Localization> ret(new Localization(0, *itsMgr));
  LINFO("Localization Created");
  itsMgr->addSubComponent(ret);
  LINFO("Localization Added As Sub Component");
  ret->init(communicator(), itsAdapter);
  LINFO("Localization Inited");

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


