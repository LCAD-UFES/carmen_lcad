// File: HawkSimulator.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include "Robots/BeoHawk/computer/HawkSimulator.H"
//#include <GL/glut.h>
#include <GL/gl.h>

HawkSimulator::HawkSimulator(std::string myName,int argc, char* argv[])
  : HawkAgent(myName, argc, argv) {
  state = LOOP;
}

void HawkSimulator::registerTopics() {
  registerPublisher("SensorDataMessage");
  //registerSubscription("ExampleMessage");
}

bool HawkSimulator::scheduler() {
  if(state == LOOP) {
    sendSensorDataMessage();
    return true;
  }
  return false;
}

void HawkSimulator::catchMessage(const HawkMessages::MessagePtr& hawkMessage,
                                 const Ice::Current&) {
  /*if(hawkMessage->ice_isA("::HawkMessages::Example"))
    {
    HawkMessages::ExamplePtr msg = HawkMessages::ExamplePtr::dynamicCast(hawkMessage);
    std::cout << "Caught a message! " << msg->name << " says: " << msg->chatter << std::endl;

  // Update State / Wake Up Agent
  if(msg->name != itsName && msg->chatter == "Hello?") {
  state = RECEIVED_FIRST_MESSAGE;
  wakeUp();
  }
  }*/

  // if we've received a message to move, then move the robot.
  if(hawkMessage->ice_isA("::HawkMessages::ControlMoveMessage")) {
    HawkMessages::ControlMoveMessagePtr message = HawkMessages::ControlMoveMessagePtr::dynamicCast(hawkMessage);
    std::cout << "Caught a ControlMove message." << std::endl;
  }
  // if we've received a message to land, then land the robot.
  else if(hawkMessage->ice_isA("HawkMessages::ControlLandMessage")) {
    HawkMessages::ControlLandMessagePtr message = HawkMessages::ControlLandMessagePtr::dynamicCast(hawkMessage);
    std::cout << "Caught a ControlLand message." << std::endl;
  }
  // if we've received a message to take off, then make the robot take flight.
  else if(hawkMessage->ice_isA("HawkMessages::ControlTakeOffMessage")) {
    HawkMessages::ControlTakeOffMessagePtr message = HawkMessages::ControlTakeOffMessagePtr::dynamicCast(hawkMessage);
    std::cout << "Caught a ControlTakeOff message." << std::endl;
  }

}

void HawkSimulator::sendSensorDataMessage() {
  // Create Message
  HawkMessages::SensorDataMessagePtr msg = new HawkMessages::SensorDataMessage;
  //msg->name = itsName;

  // Send Message
  publish("SensorDataMessage", msg);

  // Update State
  state = LOOP;

  // Wait (this should be scheduled in a separate thread later...)
  usleep(5000000);
}

int main (int argc, char* argv[]) {
  std::cout << "HawkSimulator: starting..." << std::endl;

  HawkSimulator agent("HawkSimulator", argc, argv);
  agent.start();

  std::cout << "HawkSimulator: all done!" << std::endl;
}
