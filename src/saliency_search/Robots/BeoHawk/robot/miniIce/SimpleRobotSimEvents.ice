#include <ImageIce.ice>

#ifndef ROBOTSIM_EVENTS
#define ROBOTSIM_EVENTS

module RobotSimEvents {

  class EventMessage {
  };

  interface Events {
    void updateMessage (EventMessage eMsg);
  };

  class RetinaMessage extends EventMessage {
    ImageIceMod::ImageIce img;
    string cameraID;
  };

  class CameraConfigMessage extends EventMessage {
    string cameraID;
    bool active;
  };

};
 #endif
