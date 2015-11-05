#ifndef PWIICONTROLLER_H_DEFINED
#define PWIICONTROLLER_H_DEFINED

#ifdef HAVE_LIBWIIMOTE
  extern "C" {
  #define _ENABLE_TILT
  #define _ENABLE_FORCE
  #include <wiimote.h>
  #include <wiimote_api.h>
}
#endif

#include <pthread.h>
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "rutz/shared_ptr.h"
#include "Util/Types.H"
#include "Util/WorkThreadServer.H"
#include "Util/JobWithSemaphore.H"
#include "Controllers/PID.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/Angle.H"










class PWiiController : public ModelComponent
{
public:
  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  //! Constructor
  PWiiController(OptionManager& mgr,
             const std::string& descrName = "PWiiController",
             const std::string& tagName = "PWiiController");

  //! Destructor
  ~PWiiController();

  //Connect to a wiimote
  bool connectWiimote();

  //Get Wiimote values, and send controls
  bool updateWiimote();

  bool setMotor1Speed(int speed);
  bool setMotor2Speed(int speed);
  bool setMotor1Dir(int dir);
  bool setMotor2Dir(int dir);
  bool setTransVel(int transVel);
  bool setRotVel(int rotVel);



  int* getMotor1SpeedPtr() { return &itsMotor1Speed; }
  int* getMotor2SpeedPtr() { return &itsMotor2Speed; }
  int* getMotor1DirPtr() { return &itsMotor1Dir; }
  int* getMotor2DirPtr() { return &itsMotor2Dir; }
  int* getTransVelPtr() { return &itsTransVel; }
  int* getRotVelPtr() { return &itsRotVel; }
  int* getXAccelPtr() { return &itsXAccel; }
  int* getYAccelPtr() { return &itsYAccel; }
  int* getZAccelPtr() { return &itsZAccel; }
  int* getBatteryPtr() { return &itsBatteryStatus; }
  Image<PixRGB<byte> >* getIRImagePtr() { return &itsIRImage ; }

  bool getConnectionStatus() { return itsConnectionStatus; }

  void start2();

private:
  void paramChanged(ModelParamBase* const param, const bool valueChanged, ParamClient::ChangeStatus* status);
  void updateIRImage();


  int itsMotor1Speed;
  int itsMotor2Speed;
  int itsMotor1Dir;
  int itsMotor2Dir;
  int itsTransVel;
  int itsRotVel;

  int itsXAccel;
  int itsYAccel;
  int itsZAccel;

  int itsIR1X;
  int itsIR1Y;
  int itsIR1Size;

  int itsIR2X;
  int itsIR2Y;
  int itsIR2Size;

  int itsIR3X;
  int itsIR3Y;
  int itsIR3Size;

  int itsIR4X;
  int itsIR4Y;
  int itsIR4Size;

  int itsBatteryStatus;
  bool itsConnectionStatus;

  Image<PixRGB<byte> > itsIRImage;

  NModelParam<int> paramCurrentTransVel;
  NModelParam<int> paramCurrentRotVel;

  NModelParam<int> paramMotor1Speed;
  NModelParam<int> paramMotor2Speed;
  NModelParam<int> paramMotor1Dir;
  NModelParam<int> paramMotor2Dir;
  NModelParam<bool> paramDrawIR;
  NModelParam<bool> paramConnected;

  wiimote_t itsWiimote;

  rutz::shared_ptr<WorkThreadServer> itsThreadServer;




};


#endif
