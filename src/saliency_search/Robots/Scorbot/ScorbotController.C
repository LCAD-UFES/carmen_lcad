#include "Robots/Scorbot/ScorbotController.H"

const ModelOptionCateg MOC_ScorbotComponent = {
    MOC_SORTPRI_3, "Scorbot Related Options" };

const ModelOptionDef OPT_SerialDev =
{ MODOPT_ARG(std::string), "SerialDev", &MOC_ScorbotComponent, OPTEXP_CORE,
    "Serial Device",
    "serial-dev", '\0', "<string>", "/dev/ttyUSB0" };


//############################################################
ScorbotController::ScorbotController(int id, OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsSerialDev(&OPT_SerialDev, this, 0),
  itsSerial(new Serial(mgr))
{
  addSubComponent(itsSerial)
}

//############################################################
ScorbotController::start1()
{
  //Configure the serial port
  itsSerial->configure(itsSerialDev.getVal(), 115200, "8N1", false, false, 0);
}

//############################################################
void ScorbotController::registerTopics()
{
  LINFO("Registering Scorbot Messages");
//  this->registerPublisher("RetinaMessageTopic");
  this->registerSubscription("ScorbotControl");
}

//############################################################
void ScorbotController::evolve()
{

}

//############################################################
void ScorbotController::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  //Get a retina message
  if(eMsg->ice_isA("::RobotSimEvents::CameraConfigMessage"))
  {
  }
}


//############################################################
void ScorbotController::setMotor(JOINT m, int pwm)
{
  unsigned char cmd[6];

        if (pwm > MAX_PWM) pwm = MAX_PWM;
        if (pwm < -MAX_PWM) pwm = -MAX_PWM;

        cmd[0] = 10; //move motor

  if (m == WRIST_ROLL || m == WRIST_PITCH)
  {
    cmd[1] = WRIST1; //move motor
    cmd[3] = 10; //move motor
    cmd[4] = WRIST2;

    if (pwm >= 0)
      cmd[2] = (short int)pwm&0x7F;
    else
      cmd[2] = (abs((short int)pwm)&0x7F) | 0x80;

    if (m == WRIST_ROLL)
    {
      if (pwm >= 0)
        cmd[5] = (short int)pwm&0x7F;
      else
        cmd[5] = (abs((short int)pwm)&0x7F) | 0x80;
    } else {
      if (pwm <= 0)
        cmd[5] = (short int)pwm&0x7F;
      else
        cmd[5] = (abs((short int)pwm)&0x7F) | 0x80;
    }


    for(int i=0; i<3; i++)
    {
      itsSerial->write(cmd, 6);
      usleep(1000);
    }

  } else {
    cmd[1] = m; //motor
    if (pwm >= 0)
      cmd[2] = (short int)pwm&0x7F;
    else
      cmd[2] = (abs((short int)pwm)&0x7F) | 0x80;

    for(int i=0; i<3; i++)
    {
      itsSerial->write(cmd, 3);
      usleep(1000);
    }
  }

}

//############################################################
void ScorbotController::stopAllMotors()
{
  unsigned char cmd[1];
  cmd[0] = 11; //stop all motors
  itsSerial->write(cmd, 1);
}

//############################################################
int Scorbot::getMicroSwitch()
{
  unsigned char cmd;
  unsigned char buf;
  cmd = 15; //get all ms

        itsSerial->write(&cmd, 1);

        usleep(1000);
        itsSerial->read(&buf, 1);
  return buf;
}

//############################################################
int ScorbotController::getEncoder(JOINT m)
{
  unsigned char cmd[2];
  unsigned char buf[2];

  if (m == WRIST_ROLL || m == WRIST_PITCH)
  {
    int wrist1 = getEncoder(WRIST1);
    int wrist2 = getEncoder(WRIST2);

    if (m==WRIST_PITCH)
      return wrist1-wrist2;
    else
      return wrist1+wrist2;

  } else {
    cmd[0] = 12; //get encoder
    cmd[1] = m; //motor

    short val = 0;
    itsSerial->write(cmd, 2);

    usleep(1000);
    int i = itsSerial->read(buf, 2);
    if (i == 1) //Only read one byte, wait for another
      i += itsSerial->read(buf+1, 1);

    if (i < 2)
    {
      LFATAL("Error reading encoder value read(%i)", i);
    }

    val = (buf[0] << 8) + buf[1];
    return val;
  }
  return 0;
}

//############################################################
void ScorbotController::setSafety(bool val)
{
  unsigned char cmd[2];
  cmd[0] = SAFE_MODE;
        if (val)
                cmd[1] = 1;
        else
                cmd[1] = 0;

        itsSerial->write(cmd, 2);
}

//############################################################
int ScorbotController::getPWM(JOINT m)
{
  unsigned char cmd[2];
  unsigned char buf[2];
  cmd[0] = GET_PWM;
  cmd[1] = m; //motor
  itsSerial->write(cmd, 2);

  int i = itsSerial->read(buf, 1);
  if (i < 1) return -1;
  return buf[0];
}





