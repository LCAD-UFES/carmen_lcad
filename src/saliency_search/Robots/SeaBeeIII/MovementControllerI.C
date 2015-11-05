#include "Robots/SeaBeeIII/MovementControllerI.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"


#ifndef MOVEMENTCONTROLLERI_C
#define MOVEMENTCONTROLLERI_C

// max speed sub can go
#define MAX_SPEED              50.0

// pose error must be below GATE_POSE_ERR_THRESH
// in order to sleep for fwd gate time
#define GATE_POSE_ERR_THRESH   10

// quick hack to calculate barbwire position based on
// percentage of img dims
#define IMG_WIDTH              320.0
#define IMG_HEIGHT             240.0

// minimum diff needed between current pose and last pose
// in order for beestem  message to be sent out
#define MIN_POSE_DIFF         3.0

/* Intialize Command-line parameters */
const ModelOptionCateg MOC_SeaBeeIIIMovementController = {
    MOC_SORTPRI_3, "SeaBeeIII Movement Controller Related Options" };

const ModelOptionDef OPT_GateFwdTime =
{ MODOPT_ARG(float), "GateFwdTime", &MOC_SeaBeeIIIMovementController, OPTEXP_CORE,
  "How long, in seconds, the SeaBeeIII should go forward to go through the gate.",
   "gate-time", '\0', "<float>", "40.0" };

const ModelOptionDef OPT_GateDepth =
{ MODOPT_ARG(float), "GateDepth", &MOC_SeaBeeIIIMovementController, OPTEXP_CORE,
  "How deep the SeaBeeIII should go in order to pass underneath the gate.",
   "gate-depth", '\0', "<float>", "85.0" };

const ModelOptionDef OPT_SaliencyHeadingCorrScale =
{ MODOPT_ARG(float), "SaliencyHeadingCorrScale", &MOC_SeaBeeIIIMovementController, OPTEXP_CORE,
  "The rate at which the SeaBeeIII should correct heading based on target point input",
  "headingcorr-scale", '\0', "<float>", "125.0" };

const ModelOptionDef OPT_SaliencyDepthCorrScale =
{ MODOPT_ARG(float), "SaliencyDepthCorrScale", &MOC_SeaBeeIIIMovementController, OPTEXP_CORE,
  "The rate at which the SeaBeeIII should correct depth based on target point input",
  "depthcorr-scale", '\0', "<float>", "100.0" };

const ModelOptionDef OPT_SpeedCorrScale =
{ MODOPT_ARG(float), "SpeedCorrScale", &MOC_SeaBeeIIIMovementController, OPTEXP_CORE,
  "The rate at which the SeaBeeIII should correct speed based on target point input",
  "speedcorr-scale", '\0', "<float>", "1.0" };

// ######################################################################
MovementControllerI::MovementControllerI(int id, OptionManager& mgr,
                const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsGateFwdTime(&OPT_GateFwdTime, this, 0),
  itsGateDepth(&OPT_GateDepth, this, 0),
  itsSaliencyHeadingCorrScale(&OPT_SaliencyHeadingCorrScale, this, 0),
  itsSaliencyDepthCorrScale(&OPT_SaliencyDepthCorrScale, this, 0),
  itsSpeedCorrScale(&OPT_SpeedCorrScale, this, 0),
  itsCurrentState(STATE_INIT),
  itsKillSwitchState(1),
  itsPoseError(-1.0),
  itsLastCompositeHeading(0.0),
  itsLastCompositeDepth(0.0),
  itsLastSpeed(0.0),
  isInit(false),
  itsDiveVal(-1),
  itsDiveCount(0),
  itsHeadingVal(-1),
  itsPIDEnabled(false),
  itsSpeedEnabled(false)
{
  // initialize SensorVotes
  initSensorVotes();

  // variables so we know which of the four paths we are following
  checker_path_follow_1 = 0;
  checker_path_follow_2 = 0;
  checker_path_follow_3 = 0;
}

// ######################################################################
MovementControllerI::~MovementControllerI()
{

}

// ######################################################################
void MovementControllerI::registerTopics()
{
  registerPublisher("BeeStemConfigTopic");
  registerPublisher("MovementControllerMessageTopic");
  registerSubscription("SeaBeePositionMessageTopic");
  registerSubscription("SeaBeeStateConditionMessageTopic");
  registerSubscription("XBox360RemoteControlMessageTopic");
  registerSubscription("BeeStemMessageTopic");
  registerSubscription("SalientPointMessageTopic");
  registerSubscription("VisionRectangleMessageTopic");
  registerSubscription("StraightEdgeMessageTopic");
}

// ######################################################################
void MovementControllerI::initSensorVotes()
{
  SensorVote path;
  path.type = PATH;
  path.heading.val = 0.0;
  path.heading.weight = 0.0;
  path.heading.decay = 0.0;
  path.init = false;

  SensorVote saliency;
  saliency.type = SALIENCY;
  saliency.heading.val = 0.0;
  saliency.heading.weight = 0.0;
  saliency.heading.decay = 0.0;
  saliency.init = false;

  SensorVote pinger;
  pinger.type = PINGER;
  pinger.heading.val = 0.0;
  pinger.heading.weight = 0.0;
  pinger.heading.decay = 0.0;
  pinger.init = false;

  SensorVote barbwire;
  barbwire.type = BARBWIRE;
  barbwire.heading.val = 0.0;
  barbwire.heading.weight = 0.0;
  barbwire.heading.decay = 0.0;
  barbwire.init = false;

  itsSensorVotes.push_back(path);
  itsSensorVotes.push_back(saliency);
  itsSensorVotes.push_back(pinger);
  itsSensorVotes.push_back(barbwire);
}

// ######################################################################
void MovementControllerI::enablePID()
{
  RobotSimEvents::BeeStemConfigMessagePtr msg = new RobotSimEvents::BeeStemConfigMessage;
  msg->desiredHeading = 0.0;
  msg->desiredDepth = 0.0;
  msg->desiredSpeed = 0.0;
  msg->updateDesiredValue = 0;

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

  msg->enablePID = 1;
  msg->enableVal = 1;

  this->publish("BeeStemConfigTopic", msg);
}

// ######################################################################
void MovementControllerI::disablePID()
{
  set_speed(0);
  RobotSimEvents::BeeStemConfigMessagePtr msg = new RobotSimEvents::BeeStemConfigMessage;
  msg->desiredHeading = 0.0;
  msg->desiredDepth = 0.0;
  msg->desiredSpeed = 0.0;
  msg->updateDesiredValue = 0;

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

  msg->enablePID = 1;
  msg->enableVal = 0;
  this->publish("BeeStemConfigTopic", msg);

}

void MovementControllerI::evolve()
{
  its_current_pose_mutex.lock();

  // If this is the first evolve() loop, disable PID
  if(!isInit)
    {
      disablePID();
      itsPIDEnabled = false;
      isInit = true;
    }

  // If the kill switch is active and pid is enabled,
  // disable pid and reset the sub's state
  if(itsKillSwitchState == 1 && itsPIDEnabled)
    {
      itsPIDEnabled = false;
      disablePID();

      itsDiveVal = -1;
      itsHeadingVal = -1;
      itsCurrentState = STATE_INIT;
    }

  // If the kill switch is active, perform the movement control function
  // based on the sub's current state.
  if(itsKillSwitchState == 0)
    {
      switch(itsCurrentState)
        {
        case STATE_INIT:
          state_init();
          break;
        case STATE_DO_GATE:
          state_do_gate();
          break;
        case STATE_FIND_FLARE:
          state_find_flare();
          break;
        case STATE_DO_FLARE:
          state_do_flare();
          break;
        case STATE_FIND_BARBWIRE:
          state_find_barbwire();
          break;
        case STATE_DO_BARBWIRE:
          state_do_barbwire();
          break;
        case STATE_FIND_BOMBING:
          state_find_bombing();
          break;
        case STATE_DO_BOMBING:
          state_do_bombing();
          break;
        case STATE_FIND_BRIEFCASE:
          state_find_briefcase();
          break;
        case STATE_DO_BRIEFCASE:
          state_do_briefcase();
          break;
        case STATE_PATH_FOLLOW:
          state_path_follow();
          break;
        }
    }
  else
    LINFO("Waiting for kill switch...");


  // Check whether any of our SensorVotes has been init (i.e. a val has been set)
  bool votesInit = false;

  for(uint i = 0; i < itsSensorVotes.size(); i++)
    {
      if(itsSensorVotes[i].init)
        votesInit = true;
    }


  // If at least one val has been set and the kill switch is off
  if(votesInit && itsKillSwitchState == 0)
    {
      // Decay the weights of all SensorVotes
      for(uint i = 0; i < itsSensorVotes.size(); i++)
        {
          SensorVote sv = itsSensorVotes[i];

          if(sv.heading.decay >= 0.0 && sv.heading.decay <= 1.0)
            sv.heading.weight *= 1.0 - sv.heading.decay;

          if(sv.depth.decay >= 0.0 && sv.depth.decay <= 1.0)
            sv.depth.weight *= 1.0 - sv.depth.decay;

          itsSensorVotes[i] = sv;
        }

      // Calculate the composite heading and depth based on the SensorPose
      // values and weights of our SensorVotes
      float compositeHeading = 0.0;
      int totalHeadingWeight = 0;
      float compositeDepth = 0.0;
      float totalDepthWeight = 0.0;

      for(uint i = 0; i < itsSensorVotes.size(); i++)
        {
          SensorVote sv = itsSensorVotes[i];

          compositeHeading += sv.heading.val * sv.heading.weight;
          totalHeadingWeight += sv.heading.weight;

          compositeDepth += sv.depth.val * sv.depth.weight;
          totalDepthWeight += sv.depth.weight;
        }

      // If at lease one heading weight is set,
      // calculate final heading and send it to BeeStemI
      if(totalHeadingWeight != 0.0)
        {

          compositeHeading /= totalHeadingWeight;

          // make sure current heading differs from last heading by MIN_POSE_DIFF
          if(abs(itsLastCompositeHeading - compositeHeading) > MIN_POSE_DIFF)
            set_heading(compositeHeading);
        }

      // If at lease one depth weight is set,
      // calculate final depth and send it to BeeStemI
      if(totalDepthWeight != 0.0)
        {
          compositeDepth /= totalDepthWeight;

          // make sure current depth differs from last depth by MIN_POSE_DIFF
          if(abs(itsLastCompositeDepth - compositeDepth) > MIN_POSE_DIFF)
            set_depth(compositeDepth);
        }


      // Calculate the speed we should be going based on heading and depth error
      int headingErr = compositeHeading - its_current_heading;
      int depthErr = compositeDepth - its_current_ex_pressure;

      itsPoseError = sqrt((float)(headingErr*headingErr + depthErr*depthErr));

      float speedCorr = itsPoseError * itsSpeedCorrScale.getVal();

      // Make sure correction doesn't go above max speed
      if(speedCorr > MAX_SPEED)
        speedCorr = MAX_SPEED;

      // If setting speed is enabled, send corrected speed to BeeStemI
      if(itsSpeedEnabled)
        set_speed(MAX_SPEED - speedCorr);

      itsLastCompositeHeading = compositeHeading;
      itsLastCompositeDepth = compositeDepth;
      itsLastSpeed = MAX_SPEED - speedCorr;

      // Send out MovementControllerMessage to display in GUI
      RobotSimEvents::MovementControllerMessagePtr msg = new RobotSimEvents::MovementControllerMessage;

      for(uint i = 0; i < itsSensorVotes.size(); i++)
        {
          ImageIceMod::SensorVote sv = ImageIceMod::SensorVote();
          sv.type = (ImageIceMod::SensorType)i;
          sv.heading.val = itsSensorVotes[i].heading.val;
          sv.heading.weight = itsSensorVotes[i].heading.weight;
          sv.heading.decay = itsSensorVotes[i].heading.decay;

          sv.depth.val = itsSensorVotes[i].depth.val;
          sv.depth.weight = itsSensorVotes[i].depth.weight;
          sv.depth.decay = itsSensorVotes[i].depth.decay;

          msg->votes.push_back(sv);
        }

      msg->compositeHeading = compositeHeading;
      msg->compositeDepth = compositeDepth;

      this->publish("MovementControllerMessageTopic",msg);
    }

  its_current_pose_mutex.unlock();

  // Sleep on each loop to prevent MovementController from overloading BeeStemI with messages
  usleep(100000);
}

// ######################################################################
void MovementControllerI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                const Ice::Current&)
{
  if(eMsg->ice_isA("::RobotSimEvents::SeaBeePositionMessage"))
    {
      RobotSimEvents::SeaBeePositionMessagePtr msg = RobotSimEvents::SeaBeePositionMessagePtr::dynamicCast(eMsg);
      printf("Got A Position Message! x: %f y: %f ori: %f y_var: %f x_var: %f", msg->x, msg->y, msg->orientation, msg->xVar, msg->yVar);
    }

  else if(eMsg->ice_isA("::RobotSimEvents::XBox360RemoteControlMessage"))
    {
      RobotSimEvents::JoyStickControlMessagePtr msg = RobotSimEvents::JoyStickControlMessagePtr::dynamicCast(eMsg);
      printf("Got A XBox Message! Axis: %d AxisVal: %d Button: %d ButtonVal: %d", msg->axis, msg->axisVal, msg->button, msg->butVal);
    }

  else if(eMsg->ice_isA("::RobotSimEvents::SeaBeeStateConditionMessage"))
    {
      RobotSimEvents::SeaBeeStateConditionMessagePtr msg = RobotSimEvents::SeaBeeStateConditionMessagePtr::dynamicCast(eMsg);

      // Message: Start over
      if (msg->StartOver==1)
        {
          itsCurrentState = STATE_INIT;
          checker_path_follow_1 = 0;
          checker_path_follow_2 = 0;
          checker_path_follow_3 = 0;
          printf("\nGOT MESSAGE: Starting Over\n");
        }
      // Message: Gate Done
      else if (msg->GateDone==1)
        {
          itsCurrentState = STATE_PATH_FOLLOW;
          checker_path_follow_1 = 0;
          checker_path_follow_2 = 0;
          checker_path_follow_3 = 0;
          printf("\nGOT MESSAGE: Gate Done... Starting Path Follow\n");
        }
      // Message: Flare Done
      else if (msg->FlareDone==1)
        {
          itsCurrentState = STATE_PATH_FOLLOW;
          checker_path_follow_1 = 1;
          checker_path_follow_2 = 0;
          checker_path_follow_3 = 0;
          printf("\nGOT MESSAGE: Flare Done... Starting Path Follow\n");
        }
      // Message: Barbwire Done
      else if (msg->BarbwireDone==1)
        {
          itsCurrentState = STATE_PATH_FOLLOW;
          checker_path_follow_1 = 1;
          checker_path_follow_2 = 1;
          checker_path_follow_3 = 0;
          printf("\nGOT MESSAGE: Barbwire Done... Starting Path Follow\n");
        }
      // Message: Bombing Run Done
      else if (msg->BombingRunDone==1)
        {
          itsCurrentState = STATE_PATH_FOLLOW;
          checker_path_follow_1 = 1;
          checker_path_follow_2 = 1;
          checker_path_follow_3 = 1;
          printf("\nGOT MESSAGE: Bombing Done... Starting Path Follow\n");
        }
      // Message: Briefcase Found
      else if (msg->BriefcaseFound==1)
        {
          itsCurrentState = STATE_DO_BRIEFCASE;
          checker_path_follow_1 = 1;
          checker_path_follow_2 = 1;
          checker_path_follow_3 = 1;
          printf("\nGOT MESSAGE: Briefcase Found... Doing Briefcase\n");
        }
    }
  else if(eMsg->ice_isA("::RobotSimEvents::BeeStemMessage"))
    {

      RobotSimEvents::BeeStemMessagePtr msg = RobotSimEvents::BeeStemMessagePtr::dynamicCast(eMsg);
      its_current_pose_mutex.lock();

      its_current_heading = msg->compassHeading;
      its_current_ex_pressure = msg->externalPressure;
      its_current_int_pressure = msg->internalPressure;
      itsKillSwitchState = msg->killSwitch;

      if(itsKillSwitchState)
        itsCurrentState =  STATE_INIT;

      its_current_pose_mutex.unlock();
    }
  else if(eMsg->ice_isA("::RobotSimEvents::SalientPointMessage"))
    {
      RobotSimEvents::SalientPointMessagePtr msg = RobotSimEvents::SalientPointMessagePtr::dynamicCast(eMsg);

      SensorVote saliency = itsSensorVotes[SALIENCY];

      its_current_pose_mutex.lock();

      // if we haven't set a saliency heading yet, set one
      if(!saliency.init)
        {
          saliency.heading.val = ((msg->x - 0.5) * itsSaliencyHeadingCorrScale.getVal()) + its_current_heading;
          saliency.depth.val = ((msg->y - 0.5) * itsSaliencyDepthCorrScale.getVal()) + its_current_ex_pressure;
          saliency.init = true;
        }
      // otherwise, add the new salient point heading into
      // the running average
      else
        {

          saliency.heading.val =
            (0.0 * ( saliency.heading.val ) +
             1.0 * ((msg->x - 0.5) * itsSaliencyHeadingCorrScale.getVal()
                   + its_current_heading));

          saliency.heading.val = (int)(saliency.heading.val) % 360;

          if(saliency.heading.val < 0)
            saliency.heading.val += 360.0;

          saliency.depth.val =
            (.6 * ( saliency.depth.val ) +
             .4 * ((msg->y - 0.5) * itsSaliencyDepthCorrScale.getVal())
             + its_current_ex_pressure);

        }

      itsSensorVotes[SALIENCY] = saliency;
      its_current_pose_mutex.unlock();
    }
  else if(eMsg->ice_isA("::RobotSimEvents::StraightEdgeMessage"))
    {
      RobotSimEvents::StraightEdgeMessagePtr msg =
        RobotSimEvents::StraightEdgeMessagePtr::dynamicCast(eMsg);
      its_current_pose_mutex.lock();

      // don't override the path if we are trying to go through the gate
      if(itsCurrentState != STATE_INIT && itsCurrentState != STATE_DO_GATE)
        {
          SensorVote path = itsSensorVotes[PATH];

          LINFO("Setting path heading weight");
          path.heading.val = msg->line.angle + its_current_heading;
          path.heading.weight = 2.0;
          itsSensorVotes[PATH] = path;
        }



      its_current_pose_mutex.unlock();

    }
  else if(eMsg->ice_isA("::RobotSimEvents::VisionRectangleMessage"))
    {
      RobotSimEvents::VisionRectangleMessagePtr msg = RobotSimEvents::VisionRectangleMessagePtr::dynamicCast(eMsg);

      its_current_pose_mutex.lock();

      if(msg->isFwdCamera)
        {
          SensorVote barbwire = itsSensorVotes[BARBWIRE];

          for(uint i = 0; i < msg->quads.size(); i++)
            {
              // if we haven't set a barbwire heading, set one
              if(!barbwire.init)
                {
                  barbwire.heading.val = ((((float)(msg->quads[i].center.i)/IMG_WIDTH) - 0.5) *
                                          itsSaliencyHeadingCorrScale.getVal()) + its_current_heading;
                  barbwire.depth.val = ((((float)(msg->quads[i].center.j)/IMG_HEIGHT) - 0.5) *
                                        itsSaliencyDepthCorrScale.getVal()) + its_current_ex_pressure;
                  barbwire.init = true;
                }
              //otherwise, add it to running average
              else
                {
                  barbwire.heading.val =
                    .8 * ( barbwire.heading.val ) +
                    .2 * (((msg->quads[i].center.i/IMG_WIDTH) - 0.5) *
                          itsSaliencyHeadingCorrScale.getVal()) + its_current_heading;

                  barbwire.depth.val =
                    .8 * ( barbwire.depth.val ) +
                    .2 * (((msg->quads[i].center.j/IMG_HEIGHT) - 0.5) *
                          itsSaliencyDepthCorrScale.getVal()) + its_current_ex_pressure;
                }

              barbwire.heading.weight = 1.0;
            }

          itsSensorVotes[BARBWIRE] = barbwire;
        }
      else
        {
          SensorVote path = itsSensorVotes[PATH];

          // don't override the path if we are trying to go through the gate
          if(itsCurrentState != STATE_INIT && itsCurrentState != STATE_DO_GATE)
            {
              for(uint i = 0; i < msg->quads.size(); i++)
                {
                  // if we haven't set a path heading, set one
                  if(!path.init)
                    {
                      path.heading.val = its_current_heading + msg->quads[i].angle;
                      path.init = true;
                    }
                  //otherwise, add it to running average
                  else
                    {
                      path.heading.val += its_current_heading + msg->quads[i].angle;
                      path.heading.val /= 2;
                    }

                  path.heading.weight = 1.0;
                }
            }

          itsSensorVotes[PATH] = path;
        }

      its_current_pose_mutex.unlock();
    }
}

// ######################################################################
void MovementControllerI::state_init()
{

  // First stage of dive, should only happen once
  if(itsDiveVal == -1 && itsDiveCount < 4)
    {
      LINFO("Doing initial dive");

      // Give diver 5 seconds to point sub at gate
      its_current_pose_mutex.unlock();
      sleep(5);
      its_current_pose_mutex.lock();

      // Make sure other SensorVotes do not have weights
      // before going through gate
      itsSensorVotes[SALIENCY].heading.weight = 0.0;
      itsSensorVotes[SALIENCY].depth.weight = 0.0;

      itsSensorVotes[PINGER].heading.weight = 0.0;
      itsSensorVotes[PINGER].depth.weight = 0.0;

      itsSensorVotes[BARBWIRE].heading.weight = 0.0;
      itsSensorVotes[BARBWIRE].depth.weight = 0.0;

      // Set our desired heading to our current heading
      // and set a heading weight
      itsHeadingVal = its_current_heading;
      itsSensorVotes[PATH].heading.val = itsHeadingVal;
      itsSensorVotes[PATH].heading.weight = 1.0;


      // Set our desired depth to a fourth of itsGateDepth.
      // This is done because we are diving in 4 stages.
      // Also set a depth weight.
      itsDiveVal = its_current_ex_pressure + itsGateDepth.getVal() / 4;
      itsSensorVotes[PATH].depth.val = itsDiveVal;
      itsSensorVotes[PATH].depth.weight = 1.0;

      // Increment the dive stage we are on
      itsDiveCount++;

      // Indicate that PATH SensorVote vals have been set
      itsSensorVotes[PATH].init = true;

      // Enable PID
      enablePID();

    }
  // Dive States 2 through 4
  else if(itsDiveVal != -1 &&
          abs(itsDiveVal - its_current_ex_pressure) <= 4 && itsDiveCount < 4)
    {
      // Add an additional fourth of itsGateDepth to our desired depth
      itsDiveVal = its_current_ex_pressure + itsGateDepth.getVal() / 4;
      itsSensorVotes[PATH].depth.val = itsDiveVal;
      itsSensorVotes[PATH].depth.weight = 1.0;

      // increment the dive state
      itsDiveCount++;

      LINFO("Diving stage %d", itsDiveCount);

      LINFO("itsDiveVal: %d, itsErr: %d", itsDiveVal, abs(itsDiveVal - its_current_ex_pressure));
    }
  // After going through all 4 stages of diving, go forward through the gate
  else if(itsDiveVal != -1 &&
          abs(itsDiveVal - its_current_ex_pressure) <= 4 && itsDiveCount >= 4)
    {
      itsCurrentState = STATE_DO_GATE;

      LINFO("State Init -> State Do Gate");
    }
}


// ######################################################################
void MovementControllerI::state_do_gate()
{
  LINFO("Moving towards gate...");

  // Set speed to MAX_SPEED
  set_speed(MAX_SPEED);
  its_current_pose_mutex.unlock();
  // Sleep for itsGateFwdTime
  sleep(itsGateFwdTime.getVal());
  its_current_pose_mutex.lock();

  LINFO("Finished going through gate...");

  // Start following saliency to go towards flare
  itsCurrentState = STATE_DO_FLARE;
}

// ######################################################################
void MovementControllerI::state_find_flare()
{
  printf("\n. . . Looking for Flare . . .\n");

        /*

                 looking for flare code

         */

        sleep(2);
        printf("\n. . . Found Flare . . .\n");
        itsCurrentState = STATE_DO_FLARE;

}

// ######################################################################
void MovementControllerI::state_do_flare()
{
  LINFO("Doing Flare...");

  // Enable speed based on heading and depth error
  if(!itsSpeedEnabled)
    itsSpeedEnabled = true;

  // Set SALIENCY SensorVote's heading weight to 1.0
  // so that we begin to follow saliency
  itsSensorVotes[SALIENCY].heading.weight = 1.0;

  // Decay the weight we place on PATH SensorVote's heading
  itsSensorVotes[PATH].heading.decay = 0.005;
  LINFO("heading weight: %f",itsSensorVotes[PATH].heading.weight);
}

// ######################################################################
void MovementControllerI::state_find_barbwire()
{
        printf("\n. . . Looking for Barbwire . . .\n");

        /*

                 keep going straight
                 until the barbwire comes into view

         */

        sleep(2);
        printf("\n. . . Found Barbwire . . .\n");
        itsCurrentState = STATE_DO_BARBWIRE;
}

// ######################################################################
void MovementControllerI::state_do_barbwire()
{
        printf("\n. . . Doing Barbwire . . .\n");

        /*
                 keep going until the green bar is xxx size
                 across the screen
                 make sure we are aligned properly
                 then go downwards, and then go forward for
                 xxx time
         */

        sleep(2);
        printf("\n. . . Finished Barbwire . . .\n");
        itsCurrentState = STATE_PATH_FOLLOW;
}

// ######################################################################
void MovementControllerI::state_find_bombing()
{
        printf("\n. . . Looking for Bombing Run . . .\n");

        /*
                 look at compass and make sure we are not going
                 towards the machine gun nest
                 keep going straight until sam's module
                 finds the boxes
         */

        sleep(2);
                printf("\n. . . Found Bombing Run . . .\n");
        itsCurrentState = STATE_DO_BOMBING;
}

// ######################################################################
void MovementControllerI::state_do_bombing()
{
        printf("\n. . . Doing Bombing Run . . .\n");

        /*
                 keep going forward
                 make sure that we are xxx depth above the bottom
                 or the boxes are in the bottom of the screen
                 keep going until the boxes are recognized on
                 the bottom camera
                 drop bombs
         */

        sleep(2);
        printf("\n. . . Finished Bombing Run . . .\n");
        itsCurrentState = STATE_PATH_FOLLOW;
}

// ######################################################################
void MovementControllerI::state_find_briefcase()
{
        printf("\n. . . Looking for Briefcase . . .\n");

        /*
                 follow pinger
         */
        sleep(2);
        printf("\n. . . Found Briefcase . . .\n");
        itsCurrentState = STATE_DO_BRIEFCASE;
}

// ######################################################################
void MovementControllerI::state_do_briefcase()
{
        printf("\n. . . Doing Briefcase . . .\n");

        /*
                 once pinger has gotten us close enough
                 keep going in that direction until we can see it
                 keep going over it until bottom camera can see it
                 hop across it and pick it up
                 surface straight up
         */
        sleep(2);
        printf("\n. . . Finished Briefcase . . .\n");
        exit(0);
}

// ######################################################################
void MovementControllerI::state_path_follow()
{
  /*  its_current_pose_mutex.lock();
  for(uint i = 0; i < itsQuads.size(); i++)
    {
      set_heading(its_current_heading + itsQuads[i].angle);
      //      set_speed(PATH_SPEED);

      its_current_pose_mutex.unlock();
      sleep(PATH_SLEEP_TIME);
      its_current_pose_mutex.lock();
    }
  itsQuads.clear();
  its_current_pose_mutex.unlock();

  if(checker_path_follow_1 == 0)
    {
      // comment this out for now so we keep
      // executing this block every time we find a path
      //checker_path_follow_1 = 1;
      printf("\n. . . Following Orange Path (1st) . . .\n");



      printf("\n. . . Finished Orange Path (1st) . . .\n");
      itsCurrentState = STATE_FIND_FLARE;
    }
  else if(checker_path_follow_1 == 1 && checker_path_follow_2 == 0)
    {
      checker_path_follow_2 = 1;
      printf("\n. . . Following Orange Path (2nd) . . .\n");
      sleep(2);
      printf("\n. . . Finished Orange Path (2nd) . . .\n");
      itsCurrentState = STATE_FIND_BARBWIRE;
    }
  else if(checker_path_follow_2 == 1 && checker_path_follow_3 == 0)
    {
      checker_path_follow_3 = 1;
      printf("\n. . . Following Orange Path (3rd) . . .\n");
      sleep(2);
      printf("\n. . . Finished Orange Path (3rd) . . .\n");
      itsCurrentState = STATE_FIND_BOMBING;
        }
  else if(checker_path_follow_3 == 1)
    {
      printf("\n. . . Following Orange Path (4th) . . .\n");
      sleep(2);
      printf("\n. . . Finished Orange Path (4th) . . .\n");
      itsCurrentState = STATE_FIND_BRIEFCASE;
    }



    Path follow code here

  */
}

// ######################################################################
void MovementControllerI::set_depth(int depth)
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

void MovementControllerI::set_speed(int speed)
{
  // make sure we don't go too fast
  if(speed > MAX_SPEED)
    speed = MAX_SPEED;

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

void MovementControllerI::set_heading(int heading)
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

#endif
