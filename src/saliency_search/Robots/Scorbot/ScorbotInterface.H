#ifndef ROBOTS_SCORBOT_INTERFACE_H
#define ROBOTS_SCORBOT_INTERFACE_H

#include "Component/ModelComponent.H"
#include "Devices/Serial.H"
#include <pthread.h>
#include <map>

class ScorbotInterface : public ModelComponent
{
  public: 

    enum Joint_t {Base=0, Shoulder=1, Elbow=2, Wrist1=3, Wrist2=4, Gripper=5, Slider=6};
		typedef std::map<Joint_t, int32> encoderVals_t;
		typedef std::map<Joint_t, float> pwmVals_t;

    ScorbotInterface(OptionManager& mgr,
        const std::string& descrName = "ScorbotInterface",
        const std::string& tagName   = "ScorbotInterface");

		//! Disables the motor interface
    ~ScorbotInterface();

		//! Sets all desired joint positions to their current positions at startup
		void start2();

		//! Set the joint to move to the given encoder position in the given amount of time
    void setJoint(Joint_t joint, int32 encoderPos, int32 time_ms);

		//! A convenience function to set multiple joint positions
		/*! This function just loops over all positions in pos and sets them using setJointPos() !*/
		void setJoints(encoderVals_t pos, int32 time_ms);

		//! Get the position of a joint
    int32 getEncoder(Joint_t joint);

		//! Get the position of all joints
		encoderVals_t getEncoders();

		//! Turn on/off the motors at the Scorpower box
    void setEnabled(bool enabled);

		//! Reset all encoders to 0, and set the desired position of all joints to 0
    void resetEncoders();

		//! Get the current PWM value of a joint
    float getPWM(Joint_t joint);

		//! Get the current PWM values of all joints
		pwmVals_t getPWMs();

		//! Set the PID control params for a joint
    void setControlParams(Joint_t joint, 
        float pGain, float iGain, float dGain, float maxI, float maxPWM, float pwmOffset);

		//! Get the PID values for a joint from the microcontroller
    void getPIDVals(Joint_t joint,
        float &pGain, float &iGain, float &dGain, float &maxI, float &maxPWM, float &pwmOffset);

		//! Get the current target position and velocity
		void getTuningVals(Joint_t joint,
			  int32 &targetPos, int32 &targetVel, float& gravityCompensation);

		//! Set the gravity compensation parameters
		void setGravityParameters(int32 upperArmMass, int32 foreArmMass, float compensationScale);

		//! Get the current gravity compensation parameters
		void getGravityParameters(int32 &upperArmMass, int32 &foreArmMass, float &compensationScale);

  private:
    nub::soft_ref<Serial> itsSerial;
    pthread_mutex_t itsSerialMutex;
};

#endif // ROBOTS_SCORBOT_INTERFACE_H

