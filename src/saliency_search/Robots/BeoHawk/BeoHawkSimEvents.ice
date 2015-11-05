#include <Ice/ImageIce.ice>
#include <Ice/RobotSimEvents.ice>
#include <Ice/RobotSimEvents.ice>

module RobotSimEvents {

        struct Location
        {
                double x;
                double y;
                double z;
                double theta;
        };

        struct Lrf
        {
                double angle;
                double distance;
        };

        struct Sonar
        {
                string sonarID;
                double distance;
        };


	enum BeoHawkEyeSpyType {SECURITYSIGN, USBKEY, RECTANGLE}; 

	class BeoHawkEyeSpyMessage extends EventMessage {
		BeoHawkEyeSpyType foundType;	
		string cameraID; //is it fwd or down?
		// If orientation does not matter as in the case of RECTANGLE
		// it does not matter which corner is assigned to which of the 
		// below points. (PS for some reason ice doesn't like more
		// than one entry on a line)
		ImageIceMod::Point3DIce topLeft;
		ImageIceMod::Point3DIce topRight;
		ImageIceMod::Point3DIce bottomLeft;
		ImageIceMod::Point3DIce bottomRight; 
		float certaintyLevel; //0 to 1
	};


	
	class ControlCameraMessage extends EventMessage { //Controls camera agents
		string cameraID;
		string compression;
		int fps;
		bool cameraOn;
	};
	
	class CameraImageMessage extends EventMessage { // Contains an image from a camera
		string cameraID;
		string compression;
		ImageIceMod::ImageIce img;
		
	};
	
	class ControlDriveVisionMessage extends EventMessage { //Controls the flash drive vision agent 
		bool drivevisionOn;
	};
	
	class ExecuteMissionMessage extends EventMessage { //Tells the navigator to execute mission form the mission list 
		string mission;
	};

	class SlamDataMessage extends EventMessage { //The output of Slam 
		Location lctn;
		// Slam map data here
	};
	
	class ControlLandMessage extends EventMessage { //Tell quadrotor to land in place
		
	};
	
	class ControlMoveMessage extends EventMessage {
		Location move;
	};

	sequence<Lrf> LrfSeq;
	sequence<Sonar> SonarSeq;

	class SensorDataMessage extends EventMessage { //all sensor data from craf		
		//Movement
		//BeoHawkData::Location desiredMove;
		double motorSpeeds;
		
		//IMU
		bool validRollPitchYaw;
		double pitch;
		double yaw;

		//Magnetomoeter
		double absouteHeading;

		//Laser Scanner
		LrfSeq lrf;
		
		//Sonars
		SonarSeq sonars;
		
	};
			
};



