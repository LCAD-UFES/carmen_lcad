#ifndef PWIICONTROLLER_C_DEFINED
#define PWIICONTROLLER_C_DEFINED

#include "Robots/PWiiBot/PWiiController.H"
#include "Component/ModelOptionDef.H"
#include "Devices/DeviceOpts.H"
#include "Util/Assert.H"
#include "Image/DrawOps.H"
#include "Util/Timer.H"

enum DIRECTION {FORWARD, BACKWARD, RELEASE};



namespace
{
  class WiimoteLoop : public JobWithSemaphore
  {
  public:
    WiimoteLoop(PWiiController* controller)
      :
      itsController(controller),
      itsPriority(1),
      itsJobType("Wiimote Controller Loop"),
      itsTimer()
    {}

    virtual ~WiimoteLoop() {}

    virtual void run()
    {
      int breakTime, updateTime;
      ASSERT(itsController);
      while(1)
        {
         if(itsController->getConnectionStatus()) {
                 breakTime = itsTimer.getReset();
                 itsController->updateWiimote();
                 updateTime = itsTimer.getReset();
                 LINFO("Update took %dms with %dms break time  TOTAL: %dms" , updateTime, breakTime, breakTime+updateTime);
                 usleep(10000);
         }

        }
    }

    virtual const char* jobType() const
    { return itsJobType.c_str(); }

    virtual int priority() const
    { return itsPriority; }

  private:
    PWiiController* itsController;
    const int itsPriority;
    const std::string itsJobType;
    Timer itsTimer;
  };
}








PWiiController::PWiiController(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName):
                             ModelComponent(mgr, descrName, tagName),
                             itsMotor1Speed(0), itsMotor2Speed(0),itsMotor1Dir(0), itsMotor2Dir(0),
                             itsTransVel(0), itsRotVel(0),
                             itsXAccel(0), itsYAccel(0), itsZAccel(0),
                             itsIR1X(0), itsIR1Y(0), itsIR1Size(0),
                             itsIR2X(0), itsIR2Y(0), itsIR2Size(0),
                             itsIR3X(0), itsIR3Y(0), itsIR3Size(0),
                             itsIR4X(0), itsIR4Y(0), itsIR4Size(0),
                             itsBatteryStatus(0), itsConnectionStatus(false),
                             itsIRImage(255, 255, ZEROS),
                             paramCurrentTransVel("Translational Velocity", this, 0, ALLOW_ONLINE_CHANGES),
                               paramCurrentRotVel("Rotational Velocity", this, 0, ALLOW_ONLINE_CHANGES),
                               paramMotor1Speed("Motor1 Speed", this, 0, ALLOW_ONLINE_CHANGES),
                               paramMotor2Speed("Motor2 Speed", this, 0, ALLOW_ONLINE_CHANGES),
                                paramMotor1Dir("Motor1 Direction", this, 0, ALLOW_ONLINE_CHANGES),
                               paramMotor2Dir("Motor2 Direction", this, 0, ALLOW_ONLINE_CHANGES),
                               paramDrawIR("Draw IR", this, 0, ALLOW_ONLINE_CHANGES),
                               paramConnected("Wiimote Connection", this, 0, ALLOW_ONLINE_CHANGES) {

}

PWiiController::~PWiiController() {

                    wiimote_write_byte(&itsWiimote, 0x04a40001, 0);
                    wiimote_write_byte(&itsWiimote, 0x04a40002, 0);
                    wiimote_write_byte(&itsWiimote, 0x04a40003, 0);
                    wiimote_write_byte(&itsWiimote, 0x04a40004, 0);


                wiimote_disconnect(&itsWiimote);
}


bool PWiiController::setTransVel(int transVel) {
        itsTransVel = transVel;

        if(itsTransVel > 100) {
                itsTransVel = 100;
        }
        else if(itsTransVel < -100) {
                itsTransVel = -100;
        }

        itsMotor1Speed = transVel + itsRotVel;
        itsMotor2Speed = transVel - itsRotVel;
        return true;

}

bool PWiiController::setRotVel(int rotVel) {
        itsRotVel = rotVel;

        if(itsRotVel > 100) {
                itsRotVel = 100;

        }
        else if(itsRotVel < -100) {
                itsRotVel = -100;
        }

        //Switch signs?
        itsMotor1Speed = itsTransVel + itsRotVel;
        itsMotor2Speed = itsTransVel - itsRotVel;
        return true;

}

bool PWiiController::setMotor1Speed(int speed) {
        itsMotor1Speed = speed;
        if(itsMotor1Speed < 0) {
                itsMotor1Dir = BACKWARD;
                itsMotor1Speed = -itsMotor1Speed;
        }
        else if(itsMotor1Speed > 0)
                itsMotor1Dir = FORWARD;
        else if(itsMotor1Speed == 0)
                itsMotor1Dir = RELEASE;

        return true;
}

bool PWiiController::setMotor2Speed(int speed) {
        itsMotor2Speed = speed;

        if(itsMotor2Speed < 0) {
                itsMotor2Dir = BACKWARD;
                itsMotor2Speed = -itsMotor2Speed;
        }
        else if(itsMotor2Speed > 0)
                itsMotor2Dir = FORWARD;
        else if(itsMotor2Speed == 0)
                itsMotor2Dir = RELEASE;

        return true;
}

bool PWiiController::setMotor1Dir(int dir) {
        itsMotor1Dir = dir;
        return true;
}

bool PWiiController::setMotor2Dir(int dir) {
        itsMotor2Dir = dir;
        return true;
}

void PWiiController::paramChanged(ModelParamBase* const param, const bool valueChanged, ParamClient::ChangeStatus* status) {
        if (param == &paramMotor1Speed && valueChanged){
                setMotor1Speed(paramMotor1Speed.getVal());
        }
        else if (param == &paramMotor2Speed && valueChanged){
                setMotor2Speed(paramMotor2Speed.getVal());
        }
        else if (param == &paramMotor1Dir && valueChanged){
                setMotor1Dir(paramMotor1Dir.getVal());
        }
        else if (param == &paramMotor2Dir && valueChanged){
                setMotor2Dir(paramMotor2Dir.getVal());
        }
        else if (param == &paramCurrentTransVel && valueChanged){
                setTransVel(paramCurrentTransVel.getVal());
        }
        else if (param == &paramCurrentRotVel && valueChanged){
                setRotVel(paramCurrentRotVel.getVal());
        }
        else if(param == &paramDrawIR && valueChanged){

                if(paramDrawIR.getVal() == false) {
                  itsIRImage.clear(PixRGB<byte>(0,0,0));
                }
        }
        else if(param == &paramConnected && valueChanged) {
        //TODO: Fix bugs here... If the wiimote fails to connect, we need to set the paramConnected to false
                if(paramConnected.getVal() == true) {
                        if(!connectWiimote()) {
                                itsConnectionStatus = false;
                        }
                }
                if(paramConnected.getVal() == false) {
                        itsConnectionStatus = false;
                }
        }


}


void PWiiController::start2()
{

  //setup wiimote update thread
  itsThreadServer.reset(new WorkThreadServer("PWiiController",1)); //start a single worker thread
  itsThreadServer->setFlushBeforeStopping(false);
  rutz::shared_ptr<WiimoteLoop> j(new WiimoteLoop(this));
  itsThreadServer->enqueueJob(j);

}

bool PWiiController::connectWiimote() {
        //itsWiimote = WIIMOTE_INIT;



        LINFO("Press buttons 1 and 2 on the wiimote now to connect.");
        int nmotes = wiimote_discover(&itsWiimote, 1);

  if (nmotes == 0)  {
    LINFO("no wiimotes were found");
    itsConnectionStatus = false;
    return false;
    }

  LINFO("found: %s\n", itsWiimote.link.r_addr);

  if (wiimote_connect(&itsWiimote, itsWiimote.link.r_addr) < 0) {
     LINFO("Unable to connect to wiimote");
     itsConnectionStatus = false;
     return false;
    }


    //Enable acceleration and IR data from the wiimote as well as nunchuck data
    itsWiimote.mode.acc = 1;
    itsWiimote.mode.ir  = 1;
    itsWiimote.mode.ext = 1;

  LINFO("Status %i", wiimote_is_open(&itsWiimote));

  itsConnectionStatus = true;
  return true;
}

bool PWiiController::updateWiimote() {


        if(!itsConnectionStatus) {
                LINFO("Connection Status is False");
                paramConnected.setVal(false);
                return false;
        }

        if(!wiimote_is_open(&itsWiimote)) {
                LINFO("Could not communicate with wiimote");
                itsConnectionStatus = false;
                paramConnected.setVal(false);
                return false;
        }



        /*wiimote_write_byte(&itsWiimote, 0x04a40001, itsMotor1Dir);
            wiimote_write_byte(&itsWiimote, 0x04a40002, itsMotor1Speed);
            wiimote_write_byte(&itsWiimote, 0x04a40003, itsMotor2Dir);
           wiimote_write_byte(&itsWiimote, 0x04a40004, itsMotor2Speed);*/
           uint8_t buffer[4] = {itsMotor1Dir, itsMotor1Speed, itsMotor2Dir, itsMotor2Speed};
           wiimote_write(&itsWiimote, 0x04a40001, buffer, 4);
           usleep(10000);

        if (wiimote_update(&itsWiimote) < 0) {
                LINFO("Could not update wiimote");
                itsConnectionStatus = false;
                paramConnected.setVal(false);
                return false;
        }

        itsBatteryStatus = itsWiimote.battery;

        itsXAccel = itsWiimote.axis.x;
        itsYAccel = itsWiimote.axis.y;
        itsZAccel = itsWiimote.axis.z;

        itsIR1X = itsWiimote.ir1.x;
        itsIR1Y = itsWiimote.ir1.y;
        itsIR1Size = itsWiimote.ir1.size;

        itsIR2X = itsWiimote.ir2.x;
        itsIR2Y = itsWiimote.ir2.y;
        itsIR2Size = itsWiimote.ir2.size;

        itsIR3X = itsWiimote.ir3.x;
        itsIR3Y = itsWiimote.ir3.y;
        itsIR3Size = itsWiimote.ir3.size;

        itsIR4X = itsWiimote.ir4.x;
        itsIR4Y = itsWiimote.ir4.y;
        itsIR4Size = itsWiimote.ir4.size;






        if(paramDrawIR.getVal())
                updateIRImage();

        return true;
}

//Redraw the IR Image
void PWiiController::updateIRImage() {
        itsIRImage.clear(PixRGB<byte>(20,20,20));

        Point2D<int> IRPoint;


        if(itsIR1Size < 15) {

                IRPoint = Point2D<int>(float(itsIR1X)/1023.0*itsIRImage.getWidth(),float(itsIR1Y)/767.0*itsIRImage.getHeight());

                drawDisk(itsIRImage, IRPoint, itsIR1Size*2, PixRGB<byte>(255,255,255));

                writeText(itsIRImage, IRPoint, "1", PixRGB<byte>(255, 0, 0), PixRGB<byte>(0,0,0), SimpleFont::FIXED(8), true);

        }

        if(itsIR2Size < 15) {
                IRPoint = Point2D<int>(float(itsIR2X)/1023.0*itsIRImage.getWidth(),float(itsIR2Y)/767.0*itsIRImage.getHeight());

                drawDisk(itsIRImage,
                IRPoint, itsIR2Size*2, PixRGB<byte>(255,255,255));

                writeText(itsIRImage, IRPoint, "2", PixRGB<byte>(255, 0, 0), PixRGB<byte>(0,0,0), SimpleFont::FIXED(8), true);
        }

        if(itsIR3Size < 15) {
                IRPoint = Point2D<int>(float(itsIR3X)/1023.0*itsIRImage.getWidth(),float(itsIR3Y)/767.0*itsIRImage.getHeight());

                drawDisk(itsIRImage,
                IRPoint, itsIR3Size*2, PixRGB<byte>(255,255,255));

                writeText(itsIRImage, IRPoint, "3", PixRGB<byte>(255, 0, 0), PixRGB<byte>(0,0,0), SimpleFont::FIXED(8), true);
        }

        if(itsIR4Size < 15) {
                IRPoint = Point2D<int>(float(itsIR4X)/1023.0*itsIRImage.getWidth(),float(itsIR4Y)/767.0*itsIRImage.getHeight());
                drawDisk(itsIRImage,
                IRPoint, itsIR4Size*2, PixRGB<byte>(255,255,255));

                writeText(itsIRImage, IRPoint, "4", PixRGB<byte>(255, 0, 0), PixRGB<byte>(0,0,0), SimpleFont::FIXED(8), true);
        }




}
#endif



