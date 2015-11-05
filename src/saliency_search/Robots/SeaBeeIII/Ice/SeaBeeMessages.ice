#include <Ice/ImageIce.ice> 
#include <Ice/RobotSimEvents.ice>

#ifndef SEABEEMESSAGES_ICE
#define SEABEEMESSAGES_ICE

module SeaBeeSimEvents {

  class CameraConfigMessage extends RobotSimEvents::EventMessage {
    int cameraID;
    bool active;
  };


};

#endif
