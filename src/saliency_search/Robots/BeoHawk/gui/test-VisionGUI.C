/*
 * test-VisionGUI.C
 *
 *                Description: Basically just runs the Vision GUI (VisionGUI.qt.C).
 */

#include <QtGui/QApplication>

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/BeoHawk/gui/VisionGUI.qt.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"


class VisionGUIService : public Ice::Service {
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


bool VisionGUIService::start (int argc, char * argv[]) {

        char adapterStr[255];

        int port = RobotBrainObjects::RobotBrainPort;
        bool connected = false;
        LDEBUG("Opening Connection");

        while (!connected) {
                try {
                        LINFO("Trying Port: %d", port);
                        sprintf(adapterStr, "default -p %i", port);
                        itsAdapter = communicator()->createObjectAdapterWithEndpoints(
                                        "VisionGUI", adapterStr);
                        connected = true;
                } catch (Ice::SocketException) {
                        port++;
                }
        }

        itsMgr = new ModelManager("VisionGUIService");

        LINFO("Creating VisionGUI");
        QApplication gui(argc, argv);
        nub::ref<VisionGUI> ret (new VisionGUI(*itsMgr, "VisionGUI", "VisionGUI"));
        LINFO("VisionGUI Created");
        itsMgr->addSubComponent(ret);
        ret->init(communicator(), itsAdapter);

        itsMgr->parseCommandLine((const int) argc, (const char **) argv, "", 0, 0);
        itsAdapter->activate();
        itsMgr->start();

        //load up the vision gui
        LINFO("VisionGUI Initialized");
        ret->show();
        gui.connect(&gui, SIGNAL(lastWindowClosed()), &gui, SLOT(quit()));

        return gui.exec();
}


int main (int argc, char ** argv) {
        VisionGUIService svc;
        return svc.main(argc, argv);
        return 0;
}
