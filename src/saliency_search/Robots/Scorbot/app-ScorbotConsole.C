#include <stdio.h>
#include <Qt/qapplication.h>
#include "Component/ModelManager.H"
#include "Robots/Scorbot/ScorbotConsole.qt.H"

int main(int argc, char* argv[])
{
  int argc_fake = 1;
  char* argv_fake = new char[128];
  sprintf(argv_fake, "app-ScorbotConsole");
  QApplication app(argc_fake, &argv_fake);

  ModelManager mgr("Scorbot Console");

  nub::ref<ScorbotConsole> console(new ScorbotConsole(mgr));
  mgr.addSubComponent(console);
  if(mgr.parseCommandLine(argc, argv, "", 0, 0) == false) return -1;
  mgr.start();


  //console->setGeometry(100,100, 600, 600);
  console->show();
  return app.exec();
}
