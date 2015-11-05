#ifndef SIMPLEMOTOR_H
#define SIMPLEMOTOR_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Devices/SerialAdvanced.H"

class SimpleMotor : public ModelComponent {
  public:
    SimpleMotor(OptionManager& mgr, const std::string& descrName = "SimpleMotor",
         const std::string& tagName = "SimpleMotor");

    ~SimpleMotor();

    //Set a motor speed and direction, -100 <= speed <= 100
    void setMotor(int speed);

  private:
    nub::soft_ref<SerialAdvanced>          itsSerial;
};

#endif
