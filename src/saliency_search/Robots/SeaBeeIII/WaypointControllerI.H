#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include "Util/Timer.H"

#include <IceUtil/Thread.h>


#ifndef WAYPOINTCONTROLLERI_H
#define WAYPOINTCONTROLLERI_H

class WaypointControllerI : public RobotBrainComponent
{
public:

  WaypointControllerI(OptionManager& mgr,
      const std::string& descrName = "WaypointController",
      const std::string& tagName = "WaypointController");

  void evolve();

  //!Get a message
  void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  void registerTopics();

  // The various possible sources for waypoints to follow
  enum WaypointSource { BUOY, BIN, PATH, PINGER, NAV };

  struct Waypoint
  {
    int heading;
    int depth;
    int speed;
    Timer lastUpdate; // the last time the waypoint recieved update
  };

private:
  void sendHeadingUpdate(int heading);
  void sendDepthUpdate(int depth);
  void sendSpeedUpdate(int speed);
  void sendThrusterUpdate(string thruster, int val);

  int itsCurrentHeading, itsCurrentDepth;
  int itsImgWidth, itsImgHeight;
  WaypointSource itsWaypointSource;
  Waypoint itsNavigationWaypoint, itsTrackingWaypoint;

        Timer itsBuoyTimer;

};

#endif

