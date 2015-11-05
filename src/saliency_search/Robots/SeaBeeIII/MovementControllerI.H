#include "Component/ModelManager.H"
#include "Component/OptionManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#ifndef MOVEMENTCONTROLLERI_H
#define MOVEMENTCONTROLLERI_H

/* Movement Controller States */
#define STATE_INIT                0
#define STATE_DO_GATE                1
#define STATE_FIND_FLARE        2
#define STATE_DO_FLARE                3
#define STATE_FIND_BARBWIRE        4
#define STATE_DO_BARBWIRE        5
#define STATE_FIND_BOMBING        6
#define STATE_DO_BOMBING        7
#define STATE_FIND_BRIEFCASE        8
#define STATE_DO_BRIEFCASE        9
#define STATE_PATH_FOLLOW        10


class MovementControllerI : public RobotBrainComponent
{
public:

  MovementControllerI(int id, OptionManager& mgr,
                      const std::string& descrName = "MovementController",
                      const std::string& tagName = "MovementController");

  ~MovementControllerI();

  virtual void evolve();

  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                             const Ice::Current&);

  virtual void registerTopics();


  // Represents a pose that a SensorVote can set
  struct SensorPose
  {
    float val; // value of pose
    float weight; // weight of pose
    float decay; // how fast the weight should decay (0 = none)
  };

  // The various types of SensorVotes we use
  enum SensorType { PATH, SALIENCY, PINGER, BARBWIRE };

  // A vote for what the pose of the sub should be according to
  // one of our sensors (i.e. salient point or path found in bottom cam)
  struct SensorVote
  {
    enum SensorType type; // sensor type
    SensorPose heading; // sensor's vote for absolute heading
    SensorPose depth; // sensor's vote for relative depth

    bool init; // whether or not the SensorVote has a val set
  };

private:

  // Different function for each state
  void state_init();
  void state_do_gate();
  void state_find_flare();
  void state_do_flare();
  void state_find_barbwire();
  void state_do_barbwire();
  void state_find_bombing();
  void state_do_bombing();
  void state_find_briefcase();
  void state_do_briefcase();
  void state_path_follow();

  // Setter functions for the pose of the submarine.
  // Each function sends a message to the BeeStemI
  void set_heading(int heading);
  void set_depth(int depth);
  void set_speed(int speed);

  // Enable/Disable PID by sending message to BeeStemI
  void enablePID();
  void disablePID();

  // Initialize all the SensorVotes
  void initSensorVotes();


  /**Start Command-line Options**/

  // how many secs we should sleep while going through gate
  OModelParam<float> itsGateFwdTime;
  // how deep we should dive (relative to current depth) for gate
  OModelParam<float> itsGateDepth;

  // how much we should correct heading based on salient point x position
  OModelParam<float> itsSaliencyHeadingCorrScale;
  // how much we should correct depth based on salient point y position
  OModelParam<float> itsSaliencyDepthCorrScale;
  // scales how fast we should go based on how far we are from desired heading and depth
  OModelParam<float> itsSpeedCorrScale;

  /**End Command-line Options**/


  // Stores the sub's current mission state
  unsigned int itsCurrentState;

  // checkers so we don't do states twice
  int checker_path_follow_1;
  int checker_path_follow_2;
  int checker_path_follow_3;

  // Variables to store the sub's current pose according to messages
  // received from the BeeStemI
  int its_current_heading;
  int its_current_ex_pressure;
  int its_current_int_pressure;

  // lock for current pose variables
  IceUtil::Mutex its_current_pose_mutex;

  // Stores the current state of the kill switch according to BeeStemI
  char itsKillSwitchState;

  // Vector that stores all of the sub's SensorVotes
  std::vector<SensorVote> itsSensorVotes;

  // The combined error of our current heading and depth from our desired heading and depth
  float itsPoseError;

  // Stores the last values we set for our heading, depth, and speed.
  // These are used to limit the number of messages sent to the BeeStemI
  // by ensuring that we only send a message if the current pose differs
  // from the last pose by MIN_POSE_DIFF amount
  float itsLastCompositeHeading, itsLastCompositeDepth, itsLastSpeed;

  // Used to perform intialization steps on first evolve() loop
  bool isInit;

  // Stores our currently desired dive value. Used to store intermediate dive values when
  // as we dive in 4 stages in the beginning.
  int itsDiveVal;

  // The current dive state we are on (1-4)
  char itsDiveCount;

  // Used to store the heading we want to use to go through the gate at the beginning of the mission.
  int itsHeadingVal;

  // Stores whether or not PID is currently enabled. Prevents us from sending multiple disable PID messages
  // to the BeeStem in the evolve() loop.
  bool itsPIDEnabled;

  // Whether or not we should set a new speed in the control loop
  bool itsSpeedEnabled;
};

#endif
