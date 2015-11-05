/*
 * test-ObjectFinder.C
 */

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/BeoHawk/vision/ObjectFinder.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"


class ObjectFinderService : public Ice::Service {
protected:
        virtual bool start(int, char * argv[]);
        virtual bool stop() {
                if (itsMgr)
                        delete itsMgr;
                return true;
        }

private:
        Ice::ObjectAdapterPtr itsAdapter;
        ModelManager *itsMgr;
};

bool ObjectFinderService::start (int argc, char * argv[]) {

        char adapterStr[255];

        int port = RobotBrainObjects::RobotBrainPort;
        bool connected = false;
        LDEBUG("Opening Connection");

        while (!connected) {
                try {
                        LINFO("Trying Port: %d", port);
                        sprintf(adapterStr, "default -p %i", port);
                        itsAdapter = communicator()->createObjectAdapterWithEndpoints(
                                        "ObjectFinder", adapterStr);
                        connected = true;
                } catch (Ice::SocketException) {
                        port++;
                }
        }

        itsMgr = new ModelManager("ObjectFinderService");

        LINFO("Creating ObjectFinder");
        nub::ref<ObjectFinder> ret (new ObjectFinder(*itsMgr, "ObjectFinder", "ObjectFinder"));
        LINFO("ObjectFinder Created");
        itsMgr->addSubComponent(ret);
        ret->init(communicator(), itsAdapter);
        itsMgr->parseCommandLine((const int) argc, (const char **) argv, "", 0, 0);
        ret->buildFeatureDB();
        LINFO("ObjectFinder Initialized, DB Built");

        itsAdapter->activate();
        itsMgr->start();

        return true;
}


//#############################################################
int main(int argc, char ** argv) {
        LINFO("Creating Object Finder Service...");
        ObjectFinderService svc;
        LINFO("Service Created...");
        return svc.main(argc, argv);
}
