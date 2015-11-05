#ifndef SIMPLELED_H
#define SIMPLELED_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Devices/SerialAdvanced.H"

class SimpleLED : public ModelComponent {
  public:
    SimpleLED(OptionManager& mgr, const std::string& descrName = "SimpleLED",
         const std::string& tagName = "SimpleLED");

  // Turn the LED on or off
  void turnOn();
  void turnOff();

        protected:
                void stop2() {turnOff();} //turn the LED off when closing down

        private:
    nub::soft_ref<SerialAdvanced> itsSerial;
};

#endif
