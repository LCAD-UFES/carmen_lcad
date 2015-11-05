#include "Robots/SeaBeeIII/WaypointControllerI.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Image/MathOps.H"

#ifndef WAYPOINTCONTROLLERI_C
#define WAYPOINTCONTROLLERI_C

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

#define MAX_TURN_ANGLE 35
#define MAX_TURN_SPEED 100
#define MAX_SPEED 70

// ######################################################################
WaypointControllerI::WaypointControllerI(OptionManager& mgr,
                                         const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsCurrentHeading(0),
  itsCurrentDepth(0),
  itsImgWidth(IMG_WIDTH),
  itsImgHeight(IMG_HEIGHT)
{
  itsNavigationWaypoint.heading = 0;
  itsNavigationWaypoint.depth = 0;
  itsNavigationWaypoint.speed = 0;

  itsTrackingWaypoint.heading = 0;
  itsTrackingWaypoint.depth = 0;
  itsTrackingWaypoint.speed = 0;

  itsWaypointSource = BUOY;
        itsBuoyTimer.reset();
}

// ######################################################################
void WaypointControllerI::registerTopics()
{
  LINFO("Registering Waypoint Controller Message");

  registerSubscription("BeeStemMessageTopic");
  registerSubscription("BuoyColorSegmentMessageTopic");

  registerPublisher("BeeStemConfigTopic");
  registerPublisher("XBox360RemoteControlMessageTopic");

}

// ######################################################################
void WaypointControllerI::evolve()
{
  if(itsWaypointSource == NAV)
    {
      LINFO("Nav");
    }
  else
    {
      if(itsBuoyTimer.getSecs() > .5)
        {
          itsTrackingWaypoint.speed = 0;
          itsTrackingWaypoint.heading = 0;
        }

      LINFO("Heading: %d",itsTrackingWaypoint.heading);
      LINFO("Depth: %d",itsTrackingWaypoint.depth);
      LINFO("Speed: %d",itsTrackingWaypoint.speed);
      LINFO("=========");
      sendThrusterUpdate("HEADING_AXIS",itsTrackingWaypoint.heading *-1);
      sendThrusterUpdate("SPEED_AXIS",itsTrackingWaypoint.speed * -1);

      //sendHeadingUpdate(itsTrackingWaypoint.heading);
      //sendSpeedUpdate(itsTrackingWaypoint.speed);
      sendDepthUpdate(itsTrackingWaypoint.depth);
    }
}

// ######################################################################
void WaypointControllerI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                                        const Ice::Current&)
{
        if(eMsg->ice_isA("::RobotSimEvents::BeeStemMessage"))
        {
                RobotSimEvents::BeeStemMessagePtr msg = RobotSimEvents::BeeStemMessagePtr::dynamicCast(eMsg);
                itsCurrentHeading = msg->compassHeading;
                itsCurrentDepth = msg->externalPressure;
        }
        else if(eMsg->ice_isA("::RobotSimEvents::BuoyColorSegmentMessage") && itsWaypointSource == BUOY)
        {
                RobotSimEvents::BuoyColorSegmentMessagePtr msg = RobotSimEvents::BuoyColorSegmentMessagePtr::dynamicCast(eMsg);
                //LINFO("msg->x %f,itsCurrentHeading: %d", msg->size, itsCurrentHeading);
                //LINFO("Size: %f", msg->size);
                if(msg->size > 100.0)

                {
                        //LINFO("OBJECT FOUND!!");
                        float headingError = msg->x - 0.5;
                        itsTrackingWaypoint.heading = (int)(headingError * MAX_TURN_SPEED);//(int)(headingError*MAX_TURN_ANGLE + itsCurrentHeading);

                        //       if(itsTrackingWaypoint.heading < 0)
                        //         itsTrackingWaypoint.heading += 360;
                        //       else if(itsTrackingWaypoint.heading >= 360)
                        //         itsTrackingWaypoint.heading -= 360;

                        itsTrackingWaypoint.depth = 950;
                        itsTrackingWaypoint.speed = MAX_SPEED*(1 - abs(headingError));
                        itsBuoyTimer.reset();
                }
                else
                {
                        //LINFO("No Object Found");
                        //itsTrackingWaypoint.heading = 0;
                        //itsTrackingWaypoint.depth = 950;
                        //itsTrackingWaypoint.speed = 0;
                }
        }
}

// ######################################################################
void WaypointControllerI::sendDepthUpdate(int depth)
{
  RobotSimEvents::BeeStemConfigMessagePtr msg = new RobotSimEvents::BeeStemConfigMessage;

  msg->headingK = 0.0;
  msg->headingP = 0.0;
  msg->headingI = 0.0;
  msg->headingD = 0.0;
  msg->updateHeadingPID = false;
  msg->depthK = 0.0;
  msg->depthP = 0.0;
  msg->depthI = 0.0;
  msg->depthD = 0.0;
  msg->updateDepthPID = false;
  msg->enablePID = 0;
  msg->enableVal = 0;

  msg->updateDesiredValue = 2;
  msg->desiredDepth = depth;
  this->publish("BeeStemConfigTopic", msg);
}

// ######################################################################
void WaypointControllerI::sendSpeedUpdate(int speed)
{
  RobotSimEvents::BeeStemConfigMessagePtr msg = new RobotSimEvents::BeeStemConfigMessage;

  msg->headingK = 0.0;
  msg->headingP = 0.0;
  msg->headingI = 0.0;
  msg->headingD = 0.0;
  msg->updateHeadingPID = false;
  msg->depthK = 0.0;
  msg->depthP = 0.0;
  msg->depthI = 0.0;
  msg->depthD = 0.0;
  msg->updateDepthPID = false;
  msg->enablePID = 0;
  msg->enableVal = 0;

  msg->updateDesiredValue = 3;
  msg->desiredSpeed = speed;

  this->publish("BeeStemConfigTopic", msg);
}

// ######################################################################
void WaypointControllerI::sendHeadingUpdate(int heading)
{
  RobotSimEvents::BeeStemConfigMessagePtr msg = new RobotSimEvents::BeeStemConfigMessage;

  msg->headingK = 0.0;
  msg->headingP = 0.0;
  msg->headingI = 0.0;
  msg->headingD = 0.0;
  msg->updateHeadingPID = false;
  msg->depthK = 0.0;
  msg->depthP = 0.0;
  msg->depthI = 0.0;
  msg->depthD = 0.0;
  msg->updateDepthPID = false;
  msg->enablePID = 0;
  msg->enableVal = 0;

  msg->updateDesiredValue = 1;
  msg->desiredHeading = heading;
  this->publish("BeeStemConfigTopic", msg);
}

// ######################################################################
void WaypointControllerI::sendThrusterUpdate(string thruster, int val)
{
  RobotSimEvents::JoyStickControlMessagePtr msg = new RobotSimEvents::JoyStickControlMessage;
  msg->axisName = thruster;
  msg->axis = 0;
  msg->axisVal = val;
  msg->button = -1;
  msg->butVal = 0;
  this->publish("XBox360RemoteControlMessageTopic", msg);
}

#endif

