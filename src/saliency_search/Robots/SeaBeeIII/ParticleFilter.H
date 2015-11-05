#include <IceUtil/Thread.h>

#include "Robots/RobotBrain/RobotBrainComponent.H"

#include "Component/ModelManager.H"
#include "Component/OptionManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/SeaBeeSimEvents.ice.H"
#include "Ice/IceImageUtils.H"

#include "Image/PixelsTypes.H"
#include "Image/Image.H"
#include "Image/Pixels.H"

#include "Media/FrameSeries.H"

#include "Raster/GenericFrame.H"

#include "Transport/FrameInfo.H"

#include "Controller.h"

#include "LocalizationParticle.h"
#include "LocalizationUtil.h"
#include "LocalizationSensorReader.h"
#include "LocalizationMapEntity.h"
#include "LocalizationMap.h"
#include "IMUDataServer/IMUDataServer.H"
#include "BeeStemI.H"

#ifndef ParticleFilter_H
#define ParticleFilter_H

using namespace std;

enum EXCEPTION_ID
{
	OUT_OF_RANGE
};

const int numSteps = 1; //number of steps to make each update cycle for visualization
//static float angDrift = 2.5; //degrees/sec
//static float transDrift = 1.5 / 12.0; //ft/sec
//static int numParticles = 2500;
const int resampleDelay = 6; //number of iterations to wait before resampling
const int reWeighDelay = 3;
const int saveStateDelay = 10;

class LocalizationWaypointController
{
public:
	const static float rotateThresholdRadius = 2; // min offset before going to TRANSLATE
	const static float rotateThresholdRadius2 = 10; //max offset before stopping to adjust angle during TRANSLATE
	const static float translateThresholdRadius = 1; // radius to stop; feet?
	const static float translateThresholdRadius2 = 3; //radius to slow down; feet?
	const static float depthThresholdRadius = 2; // arbitrary units, handled by BeeStem's PID

	float desiredAngle;
	float distance;

	struct State
	{
		const static int idle = 0;
		const static int move_to_waypoint = 1;
		const static int at_waypoint = 2;
		const static int remote = 3;
	};
	struct LoopState
	{
		const static int disabled = -1;
		const static int stopped = 0;
		const static int started = 1;
		const static int running = 2;
	};
	struct MovementState
	{
		const static int idle = 0;
		const static int set_depth = 1;
		const static int rotate = 2; //rotate to face the waypoint
		const static int rotate_moving = 3; //rotate while moving
		const static int translate = 4;
		const static int rotate2 = 5; //rotate into position at the waypoint
	};

	struct Waypoint
	{
		Waypoint()
		{
			mPoint = Point2D<float>(0, 0);
			mDepth = 0;
			mOrientation = 0;
			mRadius = 0;
		}
		Waypoint(Point2D<float> point, float depth, float orientation, float radius)
		{
			mPoint = point;
			mDepth = depth;
			mOrientation = orientation;
			mRadius = radius;
		}
		Point2D<float> mPoint; //(x on map, y on map)
		float mDepth;
		float mOrientation; //orientation angle -> UP; N = 90
		float mRadius;
	};

	LocalizationWaypointController()
	{
		mLoopState = LoopState::stopped;
		mState = State::idle;
		mMovementState = MovementState::idle;
		mCurrentWaypointIndex = -1;
	};

	Waypoint operator[](int i)
	{
		if(mWaypoints.size() == 0)
		{
			Waypoint result;
			return result;
		}
		if(i == -1)
		{
			return mWaypoints[mCurrentWaypointIndex];
		}
		if(i == -2)
		{
			return mWaypoints[mWaypoints.size() - 1];
		}
		return mWaypoints[i];
	}

	void addWaypoint(Waypoint wp)
	{
		mWaypoints.resize(mWaypoints.size() + 1);
		mWaypoints[mWaypoints.size() - 1] = wp;
	}

	void resume()
	{
		if(mLoopState == LoopState::started)
		{
			mLoopState = LoopState::running;
			mState = State::move_to_waypoint;
			mMovementState = MovementState::rotate;
		}
	}

	void start()
	{
		if(mLoopState == LoopState::stopped)
		{
			mLoopState = LoopState::started;
			resume();
			mCurrentWaypointIndex = 0;
		}
	}

	void pause()
	{
		if(mLoopState == LoopState::running)
		{
			mLoopState = LoopState::started;
			mState = State::idle;
		}
	}

	void stop()
	{
		pause();
		if(mLoopState != LoopState::stopped)
		{
			mLoopState = LoopState::stopped;
			mMovementState = MovementState::idle;
		}
	}

	void disable()
	{
		mLoopState = LoopState::disabled;
	}

	void enable()
	{
		mLoopState = LoopState::stopped;
	}

	bool step(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, RobotSimEvents::BeeStemConfigMessagePtr & depthMsg, Waypoint currentPose)
	{
		if(mState == State::idle)
		{
			return false;
		}
		else if(mState == State::at_waypoint)
		{
			if(mCurrentWaypointIndex < mWaypoints.size())
			{
				mState = State::move_to_waypoint;
				mMovementState = MovementState::set_depth;
			}
			else
			{
				stop();
				disable();
				return false;
			}
		}

		if(mMovementState == MovementState::set_depth)
		{
			//make sure to do this only once per waypoint; the PID loop will handle the rest
			//set depth in pid
			if(operator[](-1).mDepth != 0)
			{
				depthMsg = new RobotSimEvents::BeeStemConfigMessage;
				depthMsg->desiredDepth = operator[](-1).mDepth;
				depthMsg->updateDepthPID = 1;
				depthMsg->depthK;
				depthMsg->depthP;
				depthMsg->depthI;
				depthMsg->depthD;
			}
			mMovementState = MovementState::rotate;
			return false;
		}

		//step as long as mState == move_to_waypoint

		//do checks to see if mode needs to be stepped down to correct angle or translate
		desiredAngle = LocalizationUtil::vectorTo(currentPose.mPoint, mWaypoints[mCurrentWaypointIndex].mPoint).j;
		distance = sqrt(pow(currentPose.mPoint.i - mWaypoints[mCurrentWaypointIndex].mPoint.i, 2) + pow(currentPose.mPoint.j - mWaypoints[mCurrentWaypointIndex].mPoint.j, 2)) - operator[](-1).mRadius;
		if(distance < 0)
		{
			distance = 0;
		}
		float angleDiff = 180.0 * LocalizationUtil::linearAngleDiffRatio(currentPose.mOrientation, desiredAngle);
		if(mMovementState == MovementState::rotate || mMovementState == MovementState::translate || mMovementState == MovementState::rotate_moving)
		{
			//cout << "desired angle to waypoint: " << desiredAngle << endl;
			//cout << "current heading: " << currentPose.mOrientation << endl;
			if(distance <= translateThresholdRadius || (mMovementState == MovementState::translate && angleDiff < rotateThresholdRadius2) || !getMsgToFaceAngle(msg, currentPose.mOrientation, desiredAngle))
			{
				mMovementState = MovementState::translate;
			}
			else if(mMovementState == MovementState::translate || mMovementState == MovementState::rotate_moving)
			{
				mMovementState = MovementState::rotate_moving;
			}
			else
			{
				mMovementState = MovementState::rotate;
			}
		}
		if(mMovementState == MovementState::translate || mMovementState == MovementState::rotate_moving)
		{
			if(distance <= translateThresholdRadius)
			{
				mMovementState = MovementState::rotate2;
			}
			else
			{
				int val = -85;
				//msg->axisName = XBox360RemoteControl::Keys::Actions::toString[XBox360RemoteControl::Keys::Actions::SPEED];
				//msg->axisVal = -85;
				if(distance <= translateThresholdRadius2)
				{
					val *= (distance - translateThresholdRadius) / (translateThresholdRadius2 - translateThresholdRadius);
				}
				val -= 15;

				setMotorVal(msg, BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER, -val);
				setMotorVal(msg, BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER, val);
			}
		}
		if(mMovementState == MovementState::rotate2)
		{
			//cout << "desired angle (of waypoint): " << mWaypoints[mCurrentWaypointIndex].mOrientation << endl;
			//cout << "current heading: " << currentPose.mOrientation << endl;
			if(!getMsgToFaceAngle(msg, currentPose.mOrientation, mWaypoints[mCurrentWaypointIndex].mOrientation))
			{
				mMovementState = MovementState::idle;
				mState = State::at_waypoint; //allow everyone to see that we're at the waypoint; this will be changed to idle during the next step()
				mCurrentWaypointIndex ++; //get ready to go to the next waypoint
				return false;
			}
			else
			{
				setMotorVal(msg, BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER, 0);
				setMotorVal(msg, BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER, 0);
			}
		}

		return true;
	}

	void setMotorVal(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, int i, int val)
	{
		msg->values[i] = val;
		msg->mask[i] = 1;
	}

	bool getMsgToFaceAngle(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, float currentAngle, float desiredAngle)
	{
		float tempAngle = desiredAngle - currentAngle;
		if(abs(tempAngle) <= rotateThresholdRadius) //we're already close enough to the specified angle
		{
			return false;
		}
		else
		{
			int val = round(95.0f * LocalizationUtil::linearAngleDiffRatio(currentAngle, desiredAngle)) + 5;
			//msg->axisName = XBox360RemoteControl::Keys::Actions::toString[XBox360RemoteControl::Keys::Actions::HEADING];
			//msg->axisVal = round(95.0f * LocalizationUtil::linearAngleDiffRatio(currentAngle, desiredAngle)) + 5;
			//cout << ">>" << tempAngle << "|" << msg->axisVal << endl;
			if((abs(tempAngle) < 180 && tempAngle >= 0) || (abs(tempAngle) >= 180 && tempAngle < 0)) //conditions for turning left
			{
				val *= -1;
			}
			val *= -1;
			setMotorVal(msg, BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER, -val);
			setMotorVal(msg, BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER, val);
			//cout << ">>>" << msg->axisVal << endl;
		}
		return true;
	}

	int mLoopState;
	int mID;
	int mState;
	int mMovementState;
	int mCurrentWaypointIndex;
	vector<Waypoint> mWaypoints;
};

class ParticleFilter: public RobotBrainComponent
{

public:
	struct SimulationState
	{
		const static int disabled = -1;
		const static int idle = 0;
		const static int running = 1;
	};
	struct InitializationState
	{
		const static int uninitialized = 0;
		const static int controllerChecked = 1;
		const static int initialized = 2;
	};
	struct GraphicsFlags
	{
		struct SnapState
		{
			const static int disable = 0;
			const static int snapToAvgParticle = 1;
			const static int snapToRealParticle = 2;
			const static int snapToCurrentWaypoint = 3;
		};

		bool drawLines;
		int mSnapState;

		GraphicsFlags()
		{
			drawLines = true;
			mSnapState = SnapState::snapToCurrentWaypoint;
		}
	};
	struct SensorFlags //signals whether these sensors are enabled; set automatically based on detection of related ICE messages
	{
		bool BinFinder;
		bool BuoyFinder;
		bool CompassSensor;
		bool Hydrohpones;
		bool PipeSensor;
		bool RectangleSensor;

		SensorFlags()
		{
			BinFinder = false;
			BuoyFinder = false;
			CompassSensor = false;
			Hydrohpones = false;
			PipeSensor = false;
			RectangleSensor = false;
		}
	};

	//!Constructor
	ParticleFilter(int id, OptionManager& mgr, XBox360RemoteControlI * controller, const std::string& descrName =
			"ParticleFilter", const std::string& tagName = "ParticleFilter");

	//!Destructor
	~ParticleFilter();

	//!Main run loop
	virtual void evolve();

	//!Get a message
	virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
			const Ice::Current&);

	//!Component registers itself as publisher/subscriber to topic(s)
	virtual void registerTopics();

	void simulateMovement(int mode, int movementMode, RobotSimEvents::BeeStemMotorControllerMessagePtr & msg);
	void startSimulation()
	{
		if(mSimulationState == SimulationState::idle)
		{
			mSimulationState = SimulationState::running;
		}
	}

	void stopSimulation()
	{
		if(mSimulationState == SimulationState::running)
		{
			mSimulationState = SimulationState::idle;
		}
	}

	void disableSimulation()
	{
		mSimulationState = SimulationState::disabled;
	}

	static void getMotorControllerMsg(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, int mc0, int mc1, int mc2, int mc3, int mc4, int mc5, int mc6, int mc7, int mc8);

private:
	void Init();
	bool tryToRecoverState(string uri, LocalizationParticle::State &s);
	bool tryToSaveState(string uri, LocalizationParticle::State s);
	bool readDefaultConfigFile(bool firstAttempt);
	bool writeDefaultConfigFile(bool firstAttempt);
	void step();
	void updateGraphics();
	void calculateAvgParticle();
	void resetMotorsExceptFor(XBox360RemoteControl::Keys::Actions::Ids actionId);
	void resetMotorsExceptFor(XBox360RemoteControl::Keys::Actions::Ids actionId, RobotSimEvents::BeeStemMotorControllerMessagePtr & msg);
	bool dynamicsEnabled();
	vector<LocalizationParticle> resampleParticles(
			vector<LocalizationParticle> oldParticles);
	void reweighParticles(vector<LocalizationParticle> &p);
	void processControllerInput(ParamData pd);
	void stop2();

	int mSimulationState;
	int mInitializationState;

	GraphicsFlags mGraphicsFlags;
	SensorFlags mSensorFlags;

	ParamData mParamData; //for configuration
	Camera mCam;
	ColorSpectrum particleColor;

	LocalizationMap mLocMap;
	LocalizationParticle realParticle;
	LocalizationParticle avgParticle;
	LocalizationWaypointController::Waypoint mCurrentPose;
	LocalizationWaypointController mWaypointController;

	vector<LocalizationParticle> particles;

	CompassSensor * mCompassSensor;
	PipeSensor * mPipeSensor;
	RectangleSensor * mRectangleSensor;
	BuoySensor * mBuoySensor;

	long lastTimeTemp;
	float dt;
	float lastTime;
	int motorSpeedX;
	int strafeValue;
	int motorSpeedY;
	bool mNeedsResampling;

	//int timescaleExp; // timescale for visualization
	float timescale;

	XBox360RemoteControlI * mController;
	IceUtil::Mutex itsJSMutex;
	vector<int> itsJSValues;
	vector<int> itsButValues;

	nub::ref<OutputFrameSeries> itsOfs;
	Image<PixRGB<byte> > itsMap;
	OModelParam<int> mDisableGraphics;
};

#endif
