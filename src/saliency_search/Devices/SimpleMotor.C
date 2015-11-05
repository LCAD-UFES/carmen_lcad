#ifndef SIMPLEMOTOR_C
#define SIMPLEMOTOR_C

#include "Component/OptionManager.H"
#include "Devices/SimpleMotor.H"

SimpleMotor::SimpleMotor(OptionManager& mgr,
    const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new SerialAdvanced(mgr))
{
  addSubComponent(itsSerial);
}

SimpleMotor::~SimpleMotor()
{
}

void SimpleMotor::setMotor(int speed)
{
  speed = std::min(100, speed);
  speed = std::max(-100, speed);

  speed += 128;

  unsigned char buffer[1];
  buffer[0] = (unsigned char) speed;

  if(itsSerial->write(buffer, 1) < 1)
    itsSerial->perror();
}

#endif

