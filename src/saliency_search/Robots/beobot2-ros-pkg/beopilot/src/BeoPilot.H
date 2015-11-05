#include <vector>
#include <deque>
#include <list>
#include "Serial.H"
#include <control_toolbox/pid.h>
#ifndef __BEOPILOT_H__
#define __BEOPILOT_H__


typedef struct{
                float time,transVel,rotVel,encoderX,encoderY,encoderOri,rcTransVel,rcRotVel;
                int rcMode;
} motorData;
typedef struct{
	double x;
	double y;
	double z;

}Point3D;
//! The BeoPilot module is responsible for actually driving the BeoBot2.  All motor
//! requests are serviced to hardware by this module through MotorRequest messages.
class BeoPilot {
public:

  BeoPilot();
  ~BeoPilot();

        //! Evolve keeps the motor driver propeller up to date on the latest motor speed requests.
        //! If the evolve loop ever hangs for more than 1/4 second, then the propeller will shut down
        //! the motors until it recieves a new message.
        void start();

        //Send the motor speeds ([-1 .. 1]) to the motordriver/propeller
        void SetMotors(float motor1speed, float motor2speed);

	
        void SetMotorsPid(float motor1speed, float motor2speed);

        //Get the RC transmitter status
        unsigned char getRCStatus();

        //Get the status of an RC channel
        unsigned int getRCChannel(int channel);

        //Get the Enabled status from RC
        unsigned char getRCEnabled();

        //Get the Mix speed status from MotorBoard
        unsigned int getRCSpeed();

        //Get all of the RC status data in one frame
        void UpdateRCStatus();



	//Update position after get odometry data
        void UpdatePosition(void);

	void resetEncoder();
	Serial* itsSerial;
        //////////////////////////////////////////
        //Motor Data Structures
        //////////////////////////////////////////

        std::vector<int> itsRCChannels;
        int itsEmergencyMode;
        int itsRemoteMode;
        float itsMotor1Speed;
        float itsMotor2Speed;
        int itsLeftEncoder;
        int itsRightEncoder;
	ros::Time itsPreviousTime;
        Point3D itsPosition;
        Point3D itsDiffPosition;
        Point3D itsVelocity;
	int itsLastRightEncoder;
	int itsLastLeftEncoder;
	double itsLeftMotorVelocity;
	double itsRightMotorVelocity;
	control_toolbox::Pid itsPidSpeed;
	control_toolbox::Pid itsPidRot;
	ros::Duration dt ;

};
#endif
