#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/SeaBeeIII/SeaBee3Simulator.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"
#include "Image/MatrixOps.H"
#include "Media/FrameSeries.H"



void handle_keys(nub::soft_ref<OutputFrameSeries> ofs, nub::soft_ref<SeaBee3Simulator> subSim)
{
    //handle keyboard input
    const nub::soft_ref<ImageDisplayStream> ids =
      ofs->findFrameDestType<ImageDisplayStream>();

    const rutz::shared_ptr<XWinManaged> uiwin =
      ids.is_valid()
      ? ids->getWindow("subSim")
      : rutz::shared_ptr<XWinManaged>();

    int key = uiwin->getLastKeyPress();
    if (key != -1)
    {
      float yaw = 0;
      float forward = 0;
      float up = 0;

      switch(key)
      {
        case 8: //mac 8
        case 38: up = -100.0; break; //a
        case 14: //mac z
        case 52: up =  100.0; break; //z
        case 43: //mac p
        case 33: yaw = 100.0; break; //p
        case 39: //mac o
        case 32: yaw = -100.0; break; //o
        case 10: //mac d
        case 40: forward = 100.0; break; //d
        case 16: //mac c
        case 54: forward = -100.0; break; //c
        case 65: //space bar : stop
                 yaw = 0;
                 forward = 0;
                 up = 0;
                 break; // space
        case 20: //mac q
        case 24: //q
                 exit(0);
                 break;
      }

      float forwardLeftThruster = forward;
      float forwardRightThruster = forward;
      float verticalLeftThruster = up;
      float verticalRightThruster = up;
      float forwardStrafeThruster = yaw;
      float backStrafeThruster = -yaw;

      subSim->setThrusters(forwardLeftThruster, forwardRightThruster, verticalLeftThruster, verticalRightThruster,
          forwardStrafeThruster, backStrafeThruster);

      LINFO("Key is %i\n", key);
    }
}


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

  //Create the adapter
  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;

  while(!connected)
    {
      try
        {
          LINFO("Trying Port:%d", port);
          sprintf(adapterStr, "default -p %i", port);
          itsAdapter = communicator()->createObjectAdapterWithEndpoints("SeaBee3SimulatorAdapter",
                                                                        adapterStr);
          connected = true;
        }
      catch(Ice::SocketException)
        {
          port++;
        }
    }

  //Create the manager and its objects
  itsMgr = new ModelManager("SeaBee3SimulatorServiceManager");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*itsMgr));
  itsMgr->addSubComponent(ofs);


  LINFO("Starting SeaBee3 Simulator");
  nub::ref<SeaBee3Simulator> subSim(new SeaBee3Simulator(*itsMgr, "SeaBee3Simulator", "SeaBee3Simulator"));
  itsMgr->addSubComponent(subSim);
  subSim->init(communicator(), itsAdapter);

  itsMgr->parseCommandLine((const int)argc, (const char**)argv, "", 0, 0);

  itsAdapter->activate();

  itsMgr->start();

  while(1){
    Layout<PixRGB<byte> > outDisp;

    subSim->simLoop();
    Image<PixRGB<byte> > forwardCam = flipVertic(subSim->getFrame(1));
    Image<PixRGB<byte> > downwardCam = flipVertic(subSim->getFrame(2));

    outDisp = vcat(outDisp, hcat(forwardCam, downwardCam));

    ofs->writeRgbLayout(outDisp, "subSim", FrameInfo("subSim", SRC_POS));

    handle_keys(ofs, subSim);
  }


  return true;
}

// ######################################################################
int main(int argc, char** argv) {

  RobotBrainServiceService svc;
  return svc.main(argc, argv);
}



