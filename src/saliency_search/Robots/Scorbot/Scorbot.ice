

module ScorbotIce
{
	enum JointType {Base, Shoulder, Elbow, Wrist1, Wrist2, Gripper, Slider};
	dictionary<JointType, int> encoderValsType;
	dictionary<JointType, float> pwmValsType;

	interface Scorbot
	{

		//! Set the joint to move to the given encoder position in the given amount of time
    void setJoint(JointType joint, int encoderPos, int timeMS);

		//! A convenience function to set multiple joint positions
		/*! This function just loops over all positions in pos and sets them using setJointPos() !*/
		void setJoints(encoderValsType pos, int timeMS);

		//! Get the position of a joint
    int getEncoder(JointType joint);

		//! Get the position of all joints
		encoderValsType getEncoders();

		//! Turn on/off the motors at the Scorpower box
    void setEnabled(bool enabled);

		//! Reset all encoders to 0, and set the desired position of all joints to 0
    void resetEncoders();

		//! Get the current PWM value of a joint
    float getPWM(JointType joint);

		//! Get the current PWM values of all joints
		pwmValsType getPWMs();

		//! Set the PID control params for a joint
    void setControlParams(JointType joint, 
        float pGain, float iGain, float dGain, float maxI, float maxPWM, float pwmOffset);

		//! Get the PID values for a joint from the microcontroller
    void getPIDVals(JointType joint, out float pGain, out float iGain, out float dGain, out float maxI, out float maxPWM, out float pwmOffset);

		//! Get the current target position and velocity
		void getTuningVals(JointType joint,
			  out int targetPos, out int targetVel, out float gravityCompensation);

		//! Set the gravity compensation parameters
		void setGravityParameters(int upperArmMass, int foreArmMass, float compensationScale);

		//! Get the current gravity compensation parameters
		void getGravityParameters(out int upperArmMass, out int foreArmMass, out float compensationScale);
	};
};

