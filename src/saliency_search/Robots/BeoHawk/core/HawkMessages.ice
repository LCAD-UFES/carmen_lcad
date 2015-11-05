#include "Ice/ImageIce.ice"

module HawkMessages {

	////////////////////
	// ICE Basics
	////////////////////

	class Message {
	};

	interface MessageAgent {
		void catchMessage (Message msg);
	};
	
	////////////////////
	// Data Constructs
	////////////////////

    struct Pose
    {
		// Location
		double x;
		double y;
		double z;
		
		// Orientation
		double theta; // theta = 0 along x axis in world frame of reference
    };

    struct Sonar
    {
        string sonarID;
        double distance;
    };

	sequence<Sonar> SonarSeq;

	sequence<string> StringSeq;

	sequence<long> LongSeq;
	
	////////////////////
	// Messages
	////////////////////
	
	// Contains an image from a camera
	class CameraImageMessage extends Message {
		string cameraID;
		string compression;
		ImageIceMod::ImageIce image;
		
	};
	
	//Controls camera agents
	class ControlCameraMessage extends Message {
		string cameraID;
		string compression;
		int fps;
		bool cameraOn;
	};
	
	// Controls the flash drive vision agent
	class ControlDriveVisionMessage extends Message {
		bool driveVisionOn;
	};
	
	// Tell quadrotor to land in place
	class ControlLandMessage extends Message {
		
	};
	
	// Tell quadrotor to go someplace
	class ControlMoveMessage extends Message {
		Pose move;
	};
	
	// Controls the flash drive vision agent
	class ControlRoomVisionMessage extends Message {
		bool roomVisionOn;
	};
	
	// Tell quadrotor to takeoff in place
	class ControlTakeOffMessage extends Message {
		double altitude;
	};
	
	// The output of DriveFinder
	class DriveFinderMessage extends Message {
		bool driveFound;
		Pose drivePose;
	};
	
	// HawkExample Message
	class ExampleMessage extends Message {
		string name;
		string chatter;
	};
	
	// Tells the navigator to execute mission form the mission list 
	class ExecuteMissionMessage extends Message {
		string mission;
	};
	
	// Tells the gui controler what mission are available
	class MissionListMessage extends Message {
		StringSeq missions;
	};
	
	// The output of RoomFinder
	class RoomFinderMessage extends Message {
		bool roomFound;
		Pose roomPose;
	};

	// Senor data from craft
	class SensorDataMessage extends Message {
		// General Info
		Pose attemptedMove; // in the robot's frame of reference
		double motorSpeeds;
		double batterVoltage;
		
		// Raw IMU
		bool validAccel;
		double accelX;
		double accelY;
		double accelZ;
		bool validGyro;
		double gyroX;
		double gyroY;
		double gyroZ;
		bool validMag;
		double magX;
		double magY;
		double magZ;
		
		// Processed IMU (altitude and heading reference)
		bool validAhr;
		double roll;
		double pitch;
		double yaw;
		double heading;

		// Laser Scanner
		double angularResolution;
		LongSeq scannerData;
		
		// Sonars
		SonarSeq sonarData;
	};
	
	// The output of Slam 
	class SlamDataMessage extends Message {
		Pose hawkPose;
		//Image< PixRGB<byte> > map;
		//int mapResolution;
	};
			
};

