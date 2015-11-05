#include <QtGui/QtGui>

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Media/FrameSeries.H"
#include "Image/Image.H"
#include "Image/PixelsTypes.H"
#include "Robots/SeaBeeIII/GUI/MainWindow.qt.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>

class SeaBee3GUIService : public Ice::Service {
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

bool SeaBee3GUIService::start(int argc, char* argv[])
{
  QApplication app(argc, argv);

  itsMgr = new ModelManager("SeaBeeIIIGUIManager");

  char adapterStr[255];
  LINFO("Creating Adapter");
  sprintf(adapterStr, "default -p %i", 12345);
  itsAdapter = communicator()->createObjectAdapterWithEndpoints("RobotBrainPort",
      adapterStr);

  nub::ref<MainWindow> mainWindow(new MainWindow(*itsMgr));
  itsMgr->addSubComponent(mainWindow);

  LINFO("Starting Up GUI Comm");
  mainWindow->initIce(communicator(), itsAdapter);

  itsMgr->parseCommandLine(argc, argv, "", 0, 0);

  itsAdapter->activate();

  itsMgr->start();

  mainWindow->setGeometry(100,100,900,800);
  mainWindow->show();

  return app.exec();
}

// ######################################################################
int main(int argc, char** argv) {

  SeaBee3GUIService svc;
  return svc.main(argc, argv);
}

