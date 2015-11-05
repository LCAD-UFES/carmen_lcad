#ifndef SIMPLELED_C
#define SIMPLELED_C

#include "Component/OptionManager.H"
#include "Devices/SimpleLED.H"
#include "Devices/SerialAdvanced.H"

SimpleLED::SimpleLED(OptionManager& mgr,
    const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new SerialAdvanced(mgr))
{
  addSubComponent(itsSerial);
}

void SimpleLED::turnOn()
{
  unsigned char onBuff[1] = {'1'};
        if(itsSerial->write(onBuff, 1) < 1)
    itsSerial->perror();
}

void SimpleLED::turnOff()
{
  unsigned char onBuff[1] = {'0'};
  if(itsSerial->write(onBuff, 1) < 1)
    itsSerial->perror();
}

#endif

