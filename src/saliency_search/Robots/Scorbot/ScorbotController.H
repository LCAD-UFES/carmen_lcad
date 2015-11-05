#ifndef SCORBOTCONTROLLER_H
#define SCORBOTCONTROLLER_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>
#include "Component/ModelOptionDef.H"

#include "Devices/Serial.H"
#include "Ice/Scorbot.ice.H"


class ScorbotController : public RobotBrainComponent
{
  public:
    ScorbotController(OptionManager& mgr,
        const std::string& descrName = "ScorbotController",
        const std::string& tagName = "ScorbotController");

    ~ScorbotController();

    void start1();

    void evolve();

    void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
        const Ice::Current&);

    void registerTopics();

  private:
    OModelParam<std::string> itsSerialDev;

    nub::soft_ref<Serial> itsSerial; //our serial controller

    void setMotor(JOINT m, int pwm);
    void stopAllMotors();
    int getMicroSwitch();
    int getEncoder(JOINT m);
    void setSafety(bool val);
    int getPWM(JOINT m);
    void resetEncoders();
    void stopAllMotors();
    void setSafety(bool val);
    int getPWM(JOINT m);
}

#endif

