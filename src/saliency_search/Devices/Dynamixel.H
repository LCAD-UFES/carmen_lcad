#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"      // for byte
#include "Devices/DynamixelLib.h"

#ifndef DYNAMIXEL_HEADER
#define DYNAMIXEL_HEADER
class Dynamixel : public ModelComponent
{
public:
  //! Default constructor; see ModelComponent.H
  Dynamixel(OptionManager& mgr,
      const std::string& descrName = "Dynamixel Servo Driver",
      const std::string& tagName = "Dynamixeldriver",
      const char *defdev = "/dev/ttyUSB0");

  //! Destructor
  ~Dynamixel();

  bool init(int baudnum,int deviceIndex);
  bool commStatus();

  //! Moves servo # to given position in [0.0 .. 300.0] degree
  /*! Returns true on success, false if some serial error occurred. */
  bool move(const int servo, const float deg);

  //! Moves servo # to given RAW (uncalibrated) position in [0..1023]
  /*! Returns true on success, false if some serial error occurred. */
  bool moveRaw(const int servo, const int rawpos);

  //! Gets the current position of given servo in degree
  float getPosition(const int servo) const;

  //! Gets the current RAW (uncalibrated) position [0..1023] of given servo
  int getPositionRaw(const int servo) const;

  //! Set the speed in RPM form 0~114rpm
  bool setSpeed(const int servo, const int rpm);

  //! Set the speed at which the servo moves 0~1023, 0 is max rpm without control 
  bool setSpeedRaw(const int servo, const int speed);

  //! set the neutral position
  bool setNeutral(const int servo, const short int pos);

  //! set servo param
  bool setParam(const int servo, bool on_off, bool direction, char range);

  bool  isMoving(const int servo);
  float getTemperature(const int servo); 
  float getVoltage(const int servo);
  int   getSpeedRaw(const int servo);
  float getSpeed(const int servo);
  bool  setLed(const int servo, bool on);
  bool  setTorque(const int servo, bool hold);
  void  PrintCommStatus(int CommStatus);
  void  PrintErrorCode();
protected:
  rutz::shared_ptr<NModelParam<float> >* zero;     //!< zero calibration values
  rutz::shared_ptr<NModelParam<float> >* posmult;  //!< positive multiplier calib
  rutz::shared_ptr<NModelParam<float> >* negmult;  //!< negative multiplier calib
  byte *pos;                   //!< raw servo positions [0..255]

  ////! Convert from raw (0..255) to calibrated (-1.0..1.0) position
  //float rawToCalib(const int servo, const byte rawpos) const;

  ////! Convert from calibrated (-1.0..1.0) to raw (0..255) position
  //byte calibToRaw(const int servo, const float position) const;
};

#define DYNAMIXEL_LIB_H
#endif

