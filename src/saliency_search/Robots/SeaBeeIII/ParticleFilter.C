#include "ParticleFilter.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Image/DrawOps.H"
#include "LocalizationUtil.h"
//#include "Raster/Raster.H"
#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

#ifndef ParticleFilter_C
#define ParticleFilter_C

using namespace std;

const string mConfigFileURI = "/home/uscr/.seabee/ParticleFilterParams.txt";
const string mSavedStateURI = "/home/uscr/.seabee/ParticleFilterState.txt";

const ModelOptionCateg MOC_SeaBeeIIIParticleFilter = {
    MOC_SORTPRI_3, "Options" };

const ModelOptionDef OPT_DisableGraphics =
{ MODOPT_ARG(int), "DisableGraphics", &MOC_SeaBeeIIIParticleFilter, OPTEXP_CORE,
  "Prevent the particle filter from performing graphics calculations",
   "disable-graphics", '\0', "<int>", "0" };

// ######################################################################
ParticleFilter::ParticleFilter(int id, OptionManager& mgr, XBox360RemoteControlI * controller,
		const std::string& descrName, const std::string& tagName) :
	RobotBrainComponent(mgr, descrName, tagName), itsOfs(new OutputFrameSeries(
			mgr)),
		mDisableGraphics(&OPT_DisableGraphics, this, 0)
{
	addSubComponent(itsOfs);

	itsJSValues.resize(8);
	itsButValues.resize(20);

	mSimulationState = SimulationState::idle;
	mInitializationState = InitializationState::uninitialized;

	mController = controller;
}

ParticleFilter::~ParticleFilter()
{
	delete mController;
	delete mBuoySensor;
	delete mCompassSensor;
	delete mPipeSensor;
	delete mRectangleSensor;
}

void ParticleFilter::stop2()
{
	resetMotorsExceptFor(XBox360RemoteControl::Keys::Actions::NONE);
}

bool ParticleFilter::tryToRecoverState(string uri, LocalizationParticle::State &state)
{
	ifstream ifs(uri.c_str(), ifstream::in);
	if (!ifs.good())
	{
		LERROR("Couldn't find file.\n");
		return false;
	}
	else
	{
		map<std::string, float> theParams;

		bool go = ifs.good();
		int pos2 = 0;
		int pos3 = -1;

		while (go)
		{
			float theValue;
			string theTag;
			stringstream ss;
			string s;
			ifs >> s;
			if (ifs.good())
			{
				int pos1 = pos3 + 1;
				pos2 = s.find("=", pos1);
				pos3 = s.find("\n", pos2);
				theTag = s.substr(pos1, pos2 - pos1);
				pos2 ++;
				ss << s.substr(pos2, pos3 - pos2);
				ss >> theValue;
				theParams[theTag] = theValue;
			}
			else
			{
				go = false;
			}
		}

		ifs.close();

		state.mAngle = theParams["heading"];
		state.mPoint.i = theParams["p_x"];
		state.mPoint.j = theParams["p_y"];
		state.mWeight = 0;
		mWaypointController.mCurrentWaypointIndex = theParams["wp_current_index"];
		mWaypointController.mState = theParams["wp_state"];
		mWaypointController.mLoopState = theParams["wp_loopstate"];
		mWaypointController.mMovementState = theParams["wp_movstate"];
		int numWaypoints = theParams["num_waypoints"];
		stringstream ss;
		string s0, s1, s2, s3, s4;
		for(int i = 0; i < numWaypoints; i ++)
		{
			ss << "wp" << i << "_p_x"; s0 = ss.str(); ss.str("");
			ss << "wp" << i << "_p_y"; s1 = ss.str(); ss.str("");
			ss << "wp" << i << "_depth"; s2 = ss.str(); ss.str("");
			ss << "wp" << i << "_heading"; s3 = ss.str(); ss.str("");
			ss << "wp" << i << "_rad"; s4 = ss.str(); ss.str("");
			LocalizationWaypointController::Waypoint wp(Point2D<float> (theParams[s0], theParams[s1]), theParams[s2], theParams[s3], theParams[s4]);
			mWaypointController.addWaypoint(wp);
		}
	}
	return true;
}

bool ParticleFilter::tryToSaveState(string uri, LocalizationParticle::State s)
{
	ofstream ofs(uri.c_str());
	if (ofs.good())
	{
		map<std::string, float> state;

		state["heading"] = s.mAngle;
		state["p_x"] = s.mPoint.i;
		state["p_y"] = s.mPoint.j;
		state["wp_current_index"] = mWaypointController.mCurrentWaypointIndex;
		state["wp_state"] = mWaypointController.mState;
		state["wp_loopstate"] = mWaypointController.mLoopState;
		state["wp_movstate"] = mWaypointController.mMovementState;
		state["num_waypoints"] = mWaypointController.mWaypoints.size();
		stringstream ss;
		string s0, s1, s2, s3, s4;
		for(unsigned int i = 0; i < mWaypointController.mWaypoints.size(); i ++)
		{
			ss << "wp" << i << "_p_x"; s0 = ss.str(); ss.str("");
			ss << "wp" << i << "_p_y"; s1 = ss.str(); ss.str("");
			ss << "wp" << i << "_depth"; s2 = ss.str(); ss.str("");
			ss << "wp" << i << "_heading"; s3 = ss.str(); ss.str("");
			ss << "wp" << i << "_rad"; s4 = ss.str(); ss.str("");
			state[s0] = mWaypointController[i].mPoint.i;
			state[s1] = mWaypointController[i].mPoint.j;
			state[s2] = mWaypointController[i].mDepth;
			state[s3] = mWaypointController[i].mOrientation;
			state[s4] = mWaypointController[i].mRadius;
		}

		map<std::string, float>::iterator it;
		for(it = state.begin(); it != state.end(); it ++)
		{
			ofs << it->first << "=" << it->second << endl;
		}
	}
	else
	{
		LERROR("Couldn't write to file.\n");
		return false;
	}
	ofs.close();
	return true;
}



void ParticleFilter::Init()
{
	if(mInitializationState == InitializationState::initialized)
		return;

	/*RobotSimEvents::BeeStemMotorControllerMessagePtr mcMsg = new RobotSimEvents::BeeStemMotorControllerMessage;
	getMotorControllerMsg(mcMsg, 0,0,0,0,0,0,100,0,0);
	publish("BeeStemMotorControllerMessageTopic", mcMsg);
	sleep(1);
	getMotorControllerMsg(mcMsg, 0,0,0,0,0,0,0,100,0);
	publish("BeeStemMotorControllerMessageTopic", mcMsg);
	sleep(1);
	getMotorControllerMsg(mcMsg, 0,0,0,0,0,0,0,0,100);
	publish("BeeStemMotorControllerMessageTopic", mcMsg);
	sleep(1);
	getMotorControllerMsg(mcMsg, 0,0,0,0,0,0,0,0,0);
	publish("BeeStemMotorControllerMessageTopic", mcMsg);*/

	cout << "Initing..." << endl;

	ParamData defaultParamData = LocalizationUtil::getDefaultParamData(mController);

	bool readAttempt1, writeAttempt1, readAttempt2 = false;
	readAttempt1 = LocalizationUtil::readConfigFile(mConfigFileURI, mParamData);

	if(!readAttempt1)
	{
		writeAttempt1 = LocalizationUtil::writeConfigFile(mConfigFileURI, defaultParamData);

		if(writeAttempt1)
		{
			readAttempt2 = LocalizationUtil::readConfigFile(mConfigFileURI, mParamData);
		}

		if(!readAttempt2)
		{
			LERROR("Using default hard-coded parameters!\n");
			mParamData = defaultParamData;
		}
	}

	LocalizationParticle::State startingState;

	if(tryToRecoverState(mSavedStateURI, startingState))
	{
		cout << "Successfully recovered state" << endl;
	}
	else
	{
		LERROR("Unable to recover state! Using default state...\n");

		startingState.mPoint = Point2D<float> (mParamData["startingstate_x"], mParamData["startingstate_y"]);
		startingState.mAngle = mParamData["startingstate_angle"];

		LocalizationWaypointController::Waypoint wp1 (Point2D<float>(-15, -30), 0, 90, 0);
		LocalizationWaypointController::Waypoint wp2 (Point2D<float>(-30, -15), 0, 45, 0);
		LocalizationWaypointController::Waypoint wp3 (Point2D<float>(-15, -15), 0, 270, 0);
		LocalizationWaypointController::Waypoint wp4 (Point2D<float>(-30, -30), 0, 90, 0);
		mWaypointController.addWaypoint(wp1);
		mWaypointController.addWaypoint(wp2);
		mWaypointController.addWaypoint(wp3);
		mWaypointController.addWaypoint(wp4);
		mWaypointController.start();
	}

	startingState.mWeight = 1.0f / (float) mParamData["num_particles"];

	//printf("%f|%f|%f|%d|%d\n", startingState.mAngle, startingState.mPoint.i, startingState.mPoint.j, mWaypointController.mMode, mWaypointController.mMovementMode);

	mBuoySensor = new BuoySensor(1.0f);
	mCompassSensor = new CompassSensor(1.0f);
	mPipeSensor = new PipeSensor(1.0f);
	mRectangleSensor = new RectangleSensor(1.0f);

	itsMap = Image<PixRGB<byte> > ((int)mParamData["display_width"], (int)mParamData["display_height"], ZEROS);

	mCam.mCenter = Point2D<float> (mParamData["cam_center_x"], mParamData["cam_center_y"]);
	mCam.setZoom(mParamData["cam_zoom"]);

	LocalizationParticle::State indicatorState = {Point2D<float>(-50, 0), 0, 0};
	realParticle = LocalizationParticle(indicatorState);

	avgParticle = LocalizationParticle(startingState);

	particles.resize((int)mParamData["num_particles"]);
	for (int i = 0; i < (int)mParamData["num_particles"]; i++)
	{
		particles[i] = LocalizationParticle(startingState);
	}

	if(mController && mController->isEnabled())
	{
		stopSimulation();
	}
	else
	{
		startSimulation();
	}

	lastTime = clock();
	motorSpeedX = 0;
	motorSpeedY = 0;
	dt = 0.01;
	timescale = 1.0;

	LocalizationMapEntity theBoundary(Point2D<float> (0.0, 0.0), 90.0, Point2D<
			float> (mParamData["pool_dim_x"], mParamData["pool_dim_y"]), //25 yd by 25 yd
			LocalizationMapEntity::ShapeType::rect, PixRGB<byte>(255, 255, 255), LocalizationMapEntity::ObjectType::boundary, LocalizationMapEntity::InteractionType::external);

	LocalizationMapEntity pipe1 (Point2D<float> (-22.5, -15), 0.0, Point2D<float> (0.5, 2.0), LocalizationMapEntity::ShapeType::rect, PixRGB<byte>(255, 127, 0), LocalizationMapEntity::ObjectType::pipe);
	LocalizationMapEntity pipe2 (Point2D<float> (-22.5, -22.5), 135.0, Point2D<float> (0.5, 2.0), LocalizationMapEntity::ShapeType::rect, PixRGB<byte>(255, 127, 0), LocalizationMapEntity::ObjectType::pipe);
	LocalizationMapEntity pipe3 (Point2D<float> (-22.5, -30), 0.0, Point2D<float> (0.5, 2.0), LocalizationMapEntity::ShapeType::rect, PixRGB<byte>(255, 127, 0), LocalizationMapEntity::ObjectType::pipe);

	mCurrentPose = LocalizationWaypointController::Waypoint(avgParticle.mState.mPoint, avgParticle.mState.mAngle, 0, 0);

	mLocMap = LocalizationMap(theBoundary);
	mLocMap.addMapEntity(pipe1);
	mLocMap.addMapEntity(pipe2);
	mLocMap.addMapEntity(pipe3);

	mInitializationState = InitializationState::initialized;
	cout << "Init finished." << endl;
}

// ######################################################################
// ######################################################################
void ParticleFilter::registerTopics()
{
	registerPublisher("LocalizationMessageTopic");
	registerPublisher("XBox360RemoteControlMessageTopic");
	registerPublisher("BeeStemConfigMessageTopic");
	if(mDisableGraphics.getVal() != 0) //if graphics are disabled, display our messages instead
	{
		//registerSubscription("LocalizationMessageTopic");
	}
	registerPublisher("BeeStemMotorControllerMessageTopic");

	registerSubscription("XBox360RemoteControlMessageTopic");
	registerSubscription("IMUDataServerMessageTopic");
	registerSubscription("BeeStemMotorControllerMessageTopic");
	registerSubscription("VisionRectangleMessageTopic");
	registerSubscription("PipeColorSegmentMessageTopic");
}

void ParticleFilter::getMotorControllerMsg(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, int mc0, int mc1, int mc2, int mc3, int mc4, int mc5, int mc6, int mc7, int mc8)
{
	vector<int> values;
	values.resize(BeeStem3::NUM_MOTOR_CONTROLLERS);
	values[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = mc0;
	values[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = mc1;
	values[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = mc2;
	values[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = mc3;
	values[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = mc4;
	values[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = mc5;
	values[BeeStem3::MotorControllerIDs::SHOOTER] = mc6;
	values[BeeStem3::MotorControllerIDs::DROPPER_STAGE1] = mc7;
	values[BeeStem3::MotorControllerIDs::DROPPER_STAGE2] = mc8;
	msg->values = values;
	vector<int> mask;
	mask.assign(9, 1);
	msg->mask = mask;
}

void ParticleFilter::simulateMovement(int state, int movementState, RobotSimEvents::BeeStemMotorControllerMessagePtr & msg)
{
	if(state != LocalizationWaypointController::State::remote)
	{
		if(state == LocalizationWaypointController::State::idle)
		{
			movementState = state;
		}
		else
		{
			cout << "{ " << state << "|" << movementState << " } ";
		}
		switch(movementState)
		{
		case LocalizationWaypointController::MovementState::rotate:
			motorSpeedX = -msg->values[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER];//-value;
			motorSpeedY = 0;
			strafeValue = 0;
			break;
		case LocalizationWaypointController::MovementState::rotate_moving:
			motorSpeedX = -msg->values[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER];//-value;
			motorSpeedY = msg->values[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] * msg->mask[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER];
			strafeValue = 0;
			break;
		case LocalizationWaypointController::MovementState::translate:
			motorSpeedX = -msg->values[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER];//-value;
			motorSpeedY = msg->values[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] * msg->mask[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER];
			strafeValue = 0;
			break;
		case LocalizationWaypointController::MovementState::rotate2:
			motorSpeedX = -msg->values[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER];//-value;
			motorSpeedY = 0;
			strafeValue = 0;
			break;
		case LocalizationWaypointController::MovementState::idle:
			motorSpeedX = 0;
			motorSpeedY = 0;
			strafeValue = 0;
			break;
		}
	}
	int speedValue = -motorSpeedY;
	int theAngle = 0;

	if(abs(motorSpeedY) < abs(strafeValue))
	{
		speedValue = -strafeValue;
		theAngle += 90;
	}

	if(speedValue < 0)
	{
		theAngle -= 180;
	}

	//float da = LocalizationParticle::getChangeInRotation((float)motorSpeedX / 100.0, timescale * dt / (float)numSteps, mParamData);

	/*float m = LocalizationParticle::getDistance(
			(float) speedValue / 100.0,
			timescale * dt / (float) numSteps,
			mParamData);*/

	particles = LocalizationParticle::stepParticles(particles, timescale * dt / (float) numSteps, motorSpeedX, speedValue, theAngle, mParamData);
}

void ParticleFilter::step()
{
	if(mInitializationState != InitializationState::initialized)
	{
		cout << "waiting for initialization" << endl;
		return;
	}
	static int myTimer1 = -1; //reweigh
	static int myTimer2 = -1; //resample
	static int myTimer3 = -1; //save state

	//update loop

	for (int i = 0; i < (int) ceil(timescale); i++)
	{
		if(reWeighDelay > 0)
		{
			myTimer1++;
		}
		if(resampleDelay > 0)
		{
			myTimer2++;
		}

		if (myTimer1 >= reWeighDelay)
		{
			reweighParticles(particles);

			myTimer1 = -1;
		}
		if (myTimer2 >= resampleDelay)
		{
			particles = resampleParticles(particles);

			myTimer2 = -1;
		}

		/*if(mController && mController->isEnabled())
		{
			simulateMovement(LocalizationWaypointController::MovementState::full, 0);
		}
		else if(mWaypointController.mMode == LocalizationWaypointController::State::idle)
		{
			//just sit there and drift
			simulateMovement(LocalizationWaypointController::MovementState::idle, 0);
		}*/

		/*int speedValue = -motorSpeedY;
		int theAngle = 0;

		if(abs(motorSpeedY) < abs(strafeValue))
		{
			speedValue = -strafeValue;
			theAngle += 90;
		}

		if(speedValue < 0)
		{
			theAngle -= 180;
		}

		float da = LocalizationParticle::getChangeInRotation((float)motorSpeedX / 100.0, timescale * dt / (float)numSteps, mParamData);

		float m = LocalizationParticle::getDistance(
				(float) speedValue / 100.0, timescale * dt / (float) numSteps,
				mParamData);
		*/
		//realParticle.updatePosition(m, da + theAngle + realParticle.mState.mAngle);
		//realParticle.updatePosition(m, da + theAngle + realParticle.mState.mAngle);
		//--->realParticle.mState.mAngle += da;
		float theCompassAngle = realParticle.mState.mAngle;
		if(mSensorFlags.CompassSensor)
		{
			theCompassAngle = RobotSimEvents::IMUDataServerMessagePtr::dynamicCast(mCompassSensor->mRealMsg)->orientation[2] + 90;
			if(theCompassAngle < 0)
			{
				theCompassAngle += 360;
			}
		}
		realParticle.mState.mAngle = theCompassAngle;
		//particles = LocalizationParticle::stepParticles(particles, timescale * dt / (float) numSteps, motorSpeedX, speedValue, theAngle, mParamData);

		//realParticle.mState.mAngle = IMUCompass.mData[0] + mParamData["startingstate_angle"];

		//printf("%f, %f, %f\n", avgParticle.mState.mPoint.i, avgParticle.mState.mPoint.j, avgParticle.mState.mAngle);
		calculateAvgParticle();

		if(saveStateDelay > 0)
		{
			myTimer3++;
		}
		if(myTimer3 >= saveStateDelay)
		{
			cout << "saving state...";
			if(tryToSaveState(mSavedStateURI, avgParticle.mState))
			{
				cout << "success" << endl;
			}
			else
			{
				cout << "failed" << endl;
			}

			myTimer3 = -1;
		}
	}
}

void ParticleFilter::updateGraphics()
{
	static int display_width = (int)mParamData["display_width"];
	static int display_height = (int)mParamData["display_height"];
	itsMap = Image<PixRGB<byte> > (display_width, display_height, ZEROS);

	mLocMap.drawMe(itsMap, mCam);

	switch(mGraphicsFlags.mSnapState)
	{
	case GraphicsFlags::SnapState::disable:
		break;
	case GraphicsFlags::SnapState::snapToRealParticle:
		mCam.mCenter.i = realParticle.mState.mPoint.i;
		mCam.mCenter.j = -realParticle.mState.mPoint.j;
		break;
	case GraphicsFlags::SnapState::snapToAvgParticle:
		mCam.mCenter.i = avgParticle.mState.mPoint.i;
		mCam.mCenter.j = -avgParticle.mState.mPoint.j;
		break;
	case GraphicsFlags::SnapState::snapToCurrentWaypoint:
		if(mWaypointController.mState > LocalizationWaypointController::State::idle)
		{
			mCam.mCenter.i = mWaypointController[-1].mPoint.i;
			mCam.mCenter.j = -mWaypointController[-1].mPoint.j;
		}
		break;
	}

	Point2D<int> thePoint(0, 0);
	static PixRGB<byte> theColor;
	static PixRGB<byte> theActiveWaypointColor(0, 255, 0);
	float base_rad;

	for (unsigned int i = 0; i < particles.size(); i++)
	{
		thePoint = Point2D<int> (round((particles[i].mState.mPoint.i - mCam.mCenter.i) * mCam.mScale), -round((particles[i].mState.mPoint.j + mCam.mCenter.j) * mCam.mScale));
		thePoint += Point2D<int> (itsMap.getWidth() / 2, itsMap.getHeight() / 2);

		theColor = LocalizationUtil::RGBFromHSV(0.33f * 360.0f * (1.0 / particleColor.spectrum) * (particles[i].mState.mWeight - particleColor.minValue), 1.0, 1.0);

		if (mGraphicsFlags.drawLines)
		{
			drawLine(itsMap, thePoint, particles[i].mState.mAngle * D_DEGREE, (6.0 / 12.0) * mCam.mScale, theColor, 1);
		}
		else
		{
			Dims theDim = Dims(mCam.mScale * (12.0 / 12.0), mCam.mScale * (24.0 / 12.0));
			Rectangle theRect = Rectangle(Point2D<int> (thePoint.i - theDim.w() / 2, thePoint.j - theDim.h() / 2), theDim);
			drawRectOR(itsMap, theRect, theColor, 1, D_DEGREE * (particles[i].mState.mAngle + 90));
		}
	}

	theColor = PixRGB<byte>(255, 255, 255);

	for(unsigned int i = 0; i < mWaypointController.mWaypoints.size(); i ++)
	{
		PixRGB<byte> color = theColor;
		if(i == mWaypointController.mCurrentWaypointIndex)
		{
			color = theActiveWaypointColor;
		}
		base_rad = mWaypointController[i].mRadius;
		if(base_rad <= 0)
		{
			base_rad = 1;
		}
		thePoint = Point2D<int> (round((mWaypointController[i].mPoint.i - mCam.mCenter.i) * mCam.mScale), -round((mWaypointController[i].mPoint.j + mCam.mCenter.j) * mCam.mScale));
		thePoint += Point2D<int> (itsMap.getWidth() / 2, itsMap.getHeight() / 2);

		drawLine(itsMap, thePoint, mWaypointController[i].mOrientation * D_DEGREE, (12.0 / 12.0) * (2 * base_rad) * mCam.mScale, color, 1);

		drawCircle(itsMap, thePoint, (12 / 12.0) * (mWaypointController[i].mRadius + LocalizationWaypointController::translateThresholdRadius) * mCam.mScale, PixRGB<byte> (255, 0, 0), 1);
		drawCircle(itsMap, thePoint, (12 / 12.0) * (mWaypointController[i].mRadius + LocalizationWaypointController::translateThresholdRadius2) * mCam.mScale, PixRGB<byte> (255, 255, 0), 1);

		drawCircle(itsMap, thePoint, (12 / 12.0) * base_rad * mCam.mScale, color, 1);

		thePoint += Point2D<int> ((12.0 / 12.0) * base_rad * mCam.mScale * cos(D_DEGREE * mWaypointController[i].mOrientation), -(12.0 / 12.0) * base_rad * mCam.mScale * sin(D_DEGREE * mWaypointController[i].mOrientation));
		drawCircle(itsMap, thePoint, (3.0 / 12.0) * mCam.mScale, color, 1);
	}
	base_rad = mWaypointController[-1].mRadius;
	if(base_rad <= 0)
	{
		base_rad = 1;
	}
	thePoint = Point2D<int> (round((mCurrentPose.mPoint.i - mCam.mCenter.i) * mCam.mScale), -round((mCurrentPose.mPoint.j + mCam.mCenter.j) * mCam.mScale));
	thePoint += Point2D<int> (itsMap.getWidth() / 2, itsMap.getHeight() / 2);
	thePoint += Point2D<int> ((12.0 / 12.0) * 0.5f * (mWaypointController.distance - base_rad) * mCam.mScale * cos(D_DEGREE * mCurrentPose.mOrientation), -(12.0 / 12.0) * 0.5f * (mWaypointController.distance - base_rad) * mCam.mScale * sin(D_DEGREE * mCurrentPose.mOrientation));
	drawLine(itsMap, thePoint, mCurrentPose.mOrientation * D_DEGREE, (12.0 / 12.0) * (mWaypointController.distance - base_rad) * mCam.mScale, theActiveWaypointColor, 1);

	thePoint = Point2D<int> (round((avgParticle.mState.mPoint.i - mCam.mCenter.i) * mCam.mScale), -round((avgParticle.mState.mPoint.j + mCam.mCenter.j) * mCam.mScale));
	thePoint += Point2D<int> (itsMap.getWidth() / 2, itsMap.getHeight() / 2);

	drawLine(itsMap, thePoint, avgParticle.mState.mAngle * D_DEGREE, (24.0 / 12.0) * mCam.mScale, PixRGB<byte> (0, 0, 255), 1);

	drawCircle(itsMap, thePoint, (12.0 / 12.0) * mCam.mScale, PixRGB<byte> (0, 0, 255), 1);
	thePoint += Point2D<int> ((12.0 / 12.0) * mCam.mScale * cos(D_DEGREE * avgParticle.mState.mAngle), -(12.0 / 12.0) * mCam.mScale * sin(D_DEGREE * avgParticle.mState.mAngle));
	drawCircle(itsMap, thePoint, (3.0 / 12.0) * mCam.mScale, PixRGB<byte> (0, 0, 255), 1);

	thePoint = Point2D<int> (round((realParticle.mState.mPoint.i - mCam.mCenter.i) * mCam.mScale), -round((realParticle.mState.mPoint.j + mCam.mCenter.j) * mCam.mScale));
	thePoint += Point2D<int> (itsMap.getWidth() / 2, itsMap.getHeight() / 2);

	drawLine(itsMap, thePoint, realParticle.mState.mAngle * D_DEGREE, (24.0 / 12.0) * mCam.mScale, PixRGB<byte> (0, 255, 0), 2);
	drawCircle(itsMap, thePoint, (12.0 / 12.0) * mCam.mScale, PixRGB<byte> (0, 255, 0), 1);

	itsOfs->writeRGB(itsMap, "Particle Map");
}

void ParticleFilter::calculateAvgParticle()
{
	avgParticle.mState.mPoint.i = 0.0;
	avgParticle.mState.mPoint.j = 0.0;
	avgParticle.mState.mAngle = 0.0;

	for (unsigned int i = 0; i < particles.size(); i++)
	{
		avgParticle.mState.mPoint.i += particles[i].mState.mPoint.i * particles[i].mState.mWeight;
		avgParticle.mState.mPoint.j += particles[i].mState.mPoint.j * particles[i].mState.mWeight;
		avgParticle.mState.mAngle += particles[i].mState.mAngle * particles[i].mState.mWeight;
	}

	avgParticle.mState.mAngle = LocalizationParticle::normalizeAngle(avgParticle.mState.mAngle);

	mCurrentPose.mPoint = avgParticle.mState.mPoint;
	mCurrentPose.mOrientation = avgParticle.mState.mAngle;
}

bool ParticleFilter::dynamicsEnabled()
{
	return mInitializationState == InitializationState::initialized && (mSensorFlags.BinFinder || mSensorFlags.BuoyFinder || mSensorFlags.CompassSensor || mSensorFlags.Hydrohpones || mSensorFlags.PipeSensor || mSensorFlags.RectangleSensor);
}

// Use a roulette wheel to probabilistically duplicate particles with high weights,
// and discard those with low weights. A ‘Particle’ is some structure that has
// a weight element w. The sum of all w’s in oldParticles should equal 1.
vector<LocalizationParticle> ParticleFilter::resampleParticles(vector<
		LocalizationParticle> oldParticles)
{
	if(!mNeedsResampling)
		return oldParticles;

	LINFO("\n\n\nresampling...\n\n\n");

	vector<LocalizationParticle> newParticles;
	newParticles.resize(oldParticles.size());
	/// Calculate a Cumulative Distribution Function for our particle weights
	vector<double> CDF;

	CDF.resize(oldParticles.size());

	CDF[0] = oldParticles[0].mState.mWeight;

	for (uint i = 1; i < CDF.size(); i++)
	{
		CDF[i] = CDF[i - 1] + oldParticles[i].mState.mWeight;
	}

	// Loop through our particles as if spinning a roulette wheel.
	// The random u that we start with determines the initial offset
	// and each particle will have a probability of getting landed on and
	// saved proportional to it’s posterior probability. If a particle has a very large
	// posterior, then it may get landed on many times. If a particle has a very low
	// posterior, then it may not get landed on at all. By incrementing by
	// 1/(NUM_PARTICLES) we ensure that we don’t change the number of particles in our
	// returned set.
	uint i = 0;
	double u = randomDouble() * 1.0 / double(oldParticles.size());

	for (uint j = 0; j < oldParticles.size(); j++)
	{
		while (u > CDF[i])
		{
			i++;
		}
		LocalizationParticle p = oldParticles[i];
		p.mState.mWeight = 1.0 / double(oldParticles.size()); //all resampled particles will have the same weight...
		newParticles[j] = p;
		u += 1.0 / double(oldParticles.size());
	}
	mNeedsResampling = false;
	return newParticles;
}

//use sensor data here to adjust particle weights
void ParticleFilter::reweighParticles(vector<LocalizationParticle> &p)
{
	if(!dynamicsEnabled())
		return;

	//printf("\n\nreweighing :D\n\n");

	for (uint i = 0; i < p.size(); i++)
	{
		float totalScale = 0.0;
		//cout << p.size() << endl;
		p[i].mState.mWeight = 0.0;

		//if(mSensorFlags.BinFinder && mBinSensor->inited);
		//--do this for each sensor
		if(mSensorFlags.BuoyFinder && mBuoySensor->inited)
		{
			VirtualSensorMessage::BuoyColorSegmentMessage * msgptr = static_cast<VirtualSensorMessage::BuoyColorSegmentMessage*>(BuoySensor::generatetVirtualSensorMessage(p[i], mLocMap));
			mBuoySensor->takeVirtualReading(msgptr);
			p[i].mState.mWeight += mBuoySensor->getScaledVote();
			totalScale += mBuoySensor->mScale;
			delete msgptr;
		}
		//--
		if(mSensorFlags.CompassSensor && mCompassSensor->inited)
		{
			VirtualSensorMessage::IMUDataServerMessage * msgptr = static_cast<VirtualSensorMessage::IMUDataServerMessage*>(CompassSensor::generatetVirtualSensorMessage(p[i], mLocMap));
			mCompassSensor->takeVirtualReading(msgptr);
			p[i].mState.mWeight += mCompassSensor->getScaledVote();
			totalScale += mCompassSensor->mScale;
			delete msgptr;
		}
		//if(mSensorFlags.Hydrohpones && mHydrophonesSensor->inited);
		if(mSensorFlags.PipeSensor && mPipeSensor->inited && mSensorFlags.RectangleSensor && mRectangleSensor->inited)
		{
			printf("pipe and rectangle sensors enabled\n");
			VirtualSensorMessage::PipeColorSegmentMessage * msgptr1 = static_cast<VirtualSensorMessage::PipeColorSegmentMessage*>(PipeSensor::generatetVirtualSensorMessage(p[i], mLocMap));
			VirtualSensorMessage::VisionRectangleMessage * msgptr2 = static_cast<VirtualSensorMessage::VisionRectangleMessage*>(RectangleSensor::generatetVirtualSensorMessage(p[i], mLocMap));
			mPipeSensor->takeVirtualReading(msgptr1);
			//p[i].mState.mWeight = mPipeSensor->getScaledVote();
			//totalScale += mPipeSensor->mScale;
			mRectangleSensor->takeVirtualReading(msgptr2);
			float temp = mPipeSensor->getScaledVote() * mRectangleSensor->getScaledVote(); //we need to see orange AND see a rectangle
			if(temp != 0) //only change it if it's not zero, otherwise we trigger resampling
			{
				printf("\n\n\n\n\nparticle can see an orange rectangle\n\n\n\n\n");
				p[i].mState.mWeight = 2 * temp;
				totalScale += mPipeSensor->mScale + mRectangleSensor->mScale;
			}
			delete msgptr1;
			//delete msgptr2;
		}
		/*if(mSensorFlags.PipeSensor && mPipeSensor->inited)
		{
			VirtualSensorMessage::PipeColorSegmentMessage * msgptr1 = static_cast<VirtualSensorMessage::PipeColorSegmentMessage*>(PipeSensor::generatetVirtualSensorMessage(p[i], mLocMap));
			mPipeSensor->takeVirtualReading(msgptr1);
			p[i].mState.mWeight += mPipeSensor->getScaledVote();
			totalScale += mPipeSensor->mScale;
			delete msgptr1;
		}
		if(mSensorFlags.RectangleSensor && mRectangleSensor->inited)
		{
			VirtualSensorMessage::VisionRectangleMessage * msgptr2 = static_cast<VirtualSensorMessage::VisionRectangleMessage*>(RectangleSensor::generatetVirtualSensorMessage(p[i], mLocMap));
			mRectangleSensor->takeVirtualReading(msgptr2);
			p[i].mState.mWeight += mRectangleSensor->getScaledVote();
			totalScale += mRectangleSensor->mScale;
			delete msgptr2;
		}*/
		if(abs(totalScale) >  0)
		{
			p[i].mState.mWeight /= totalScale;
		}

		//sum += p[i].mState.mWeight;

	}

	float largest = p[0].mState.mWeight;
	float smallest = p[0].mState.mWeight;

	//calculate the largest and smallest values
	for (uint i = 0; i < p.size(); i++)
	{
		if (p[i].mState.mWeight > largest)
		{
			largest = p[i].mState.mWeight;
		}
		if (p[i].mState.mWeight < smallest)
		{
			smallest = p[i].mState.mWeight;
		}
	}

	float sum = 0.0f;

	if(smallest == largest) //likely all zeros; regardless, prepare to set all weights to 1/p.size()
	{
		sum = p.size();
	}
	/*else //set sum to what it will be after stretching the data
	{
		sum =  (sum - (smallest * p.size())) / (largest - smallest);
	}*/

	//reorganize particles so that small changes can be amplified
	//stretch data so that smallest->0 and largest->1
	//calculate sum if necessary
	for(uint i = 0; i < p.size(); i ++)
	{
		if(smallest == largest)
		{
			p[i].mState.mWeight = 1.0f / sum;
		}
		else
		{
			p[i].mState.mWeight -= smallest;
			p[i].mState.mWeight /= (largest - smallest);
			p[i].mState.mWeight = pow(p[i].mState.mWeight, 1);
			sum += p[i].mState.mWeight;
		}
		//p[i].mState.mWeight /= sum;
	}

	//normalize if necessary
	if(smallest != largest)
	{
		for(uint i = 0; i < p.size(); i ++)
		{
			p[i].mState.mWeight /= sum;
		}
	}

	//float largestValue = 1.0f / sum;
	//float smallestValue = 0;
	mNeedsResampling = true;
	if(smallest == largest) //no resampling needs to get done on particles that all have the same weight
	{
		mNeedsResampling = false;
	}
	if(mDisableGraphics.getVal() == 0)
	{
		//this will suffice regardless of the situation
		particleColor.setSpectrum(1.0f / sum);
		particleColor.minValue = 0;
		if(smallest == largest)
		{
			particleColor.minValue = 1.0f / sum;
		}
	}
}

void ParticleFilter::processControllerInput(ParamData pd)
{
	itsJSMutex.lock();

	static int jsvals_r_x = pd["jsvals_r_x"];
	static int jsvals_l_x = pd["jsvals_l_x"];
	static int jsvals_l_y = pd["jsvals_l_y"];
	static int jsvals_dpad_x = pd["jsvals_dpad_x"];
	static int jsvals_dpad_y = pd["jsvals_dpad_y"];
	static int bvals_back = pd["bvals_back"];
	static int bvals_start = pd["bvals_start"];
	static int bvals_x = pd["bvals_x"];
	static int bvals_a = pd["bvals_a"];
	static int bvals_b = pd["bvals_b"];
	static int bvals_y = pd["bvals_y"];
	static int bvals_l_bumper = pd["bvals_l_bumper"];
	static int bvals_r_bumper = pd["bvals_r_bumper"];

	motorSpeedX = -itsJSValues[jsvals_r_x]; //speed
	strafeValue = -itsJSValues[jsvals_l_x]; //strafe
	motorSpeedY = itsJSValues[jsvals_l_y]; //heading

	mCam.mCenter.i += 0.25 * (float) itsJSValues[jsvals_dpad_x]
			/ mCam.mScale;
	mCam.mCenter.j += 0.25 * (float) itsJSValues[jsvals_dpad_y]
			/ mCam.mScale;

	static int last_bvals_start = -1;
	static int last_bvals_back = -1;
	static int last_bvals_b = -1;
	static int last_bvals_y = -1;
	static int last_bvals_x = -1;
	static int last_bvals_a = -1;
	static int last_bvals_l_bumper = -1;
	static int last_bvals_r_bumper = -1;

	if (itsButValues[bvals_back] == 1)
	{
		if (last_bvals_back == 0)
		{
			mGraphicsFlags.drawLines = !mGraphicsFlags.drawLines;
		}
	}

	if (itsButValues[bvals_start] == 1)
	{
		if (last_bvals_start == 0)
		{
			if(mSimulationState == SimulationState::running)
			{
				stopSimulation();
			}
			else
			{
				startSimulation();
			}
		}
	}

	/*if (itsButValues[pd.bvals_b] == 1)
	{
		if (last_bvals_b == 0)
		{
			timescaleExp -= 1;
			timescale = pow(1.25f, (float) timescaleExp);
			cout << "a^[k] dt: " << timescale << endl;
		}
	}*/

	/*if (itsButValues[pd.bvals_y] == 1)
	{
		if (last_bvals_y == 0)
		{
			timescaleExp += 1;
			timescale = pow(1.25f, (float) timescaleExp);
			cout << "a^[k] dt: " << timescale << endl;
		}
	}*/

	if (itsButValues[bvals_x] == 1) //use this to cycle through snapping options
	{
		if (last_bvals_x == 0)
		{
			if(mGraphicsFlags.mSnapState < GraphicsFlags::SnapState::snapToCurrentWaypoint)
			{
				mGraphicsFlags.mSnapState ++;
			}
			else
			{
				mGraphicsFlags.mSnapState = GraphicsFlags::SnapState::disable;
			}
		}
	}

	/*if (itsButValues[bvals_a] == 1)
	{
		if (last_bvals_a == 0)
		{
			toggleFollowRealParticle = !toggleFollowRealParticle;
			toggleFollowAvgParticle = false;
		}
	}*/

	if (itsButValues[bvals_l_bumper] == 1)
	{
		if (last_bvals_l_bumper == 0)
		{
			mCam.offsetZoom(-50);
		}
	}

	if (itsButValues[bvals_r_bumper] == 1)
	{
		if (last_bvals_r_bumper == 0)
		{
			mCam.offsetZoom(50);
		}
	}

	if (abs(motorSpeedX) < 1)
	{
		motorSpeedX = 0;
	}
	if (abs(motorSpeedY) < 1)
	{
		motorSpeedY = 0;
	}

	last_bvals_start = itsButValues[bvals_start];
	last_bvals_back = itsButValues[bvals_back];
	last_bvals_b = itsButValues[bvals_b];
	last_bvals_y = itsButValues[bvals_y];
	last_bvals_x = itsButValues[bvals_x];
	last_bvals_a = itsButValues[bvals_a];
	last_bvals_l_bumper = itsButValues[bvals_l_bumper];
	last_bvals_r_bumper = itsButValues[bvals_r_bumper];

	itsJSMutex.unlock();
}

// ######################################################################
void ParticleFilter::evolve()
{
	if(mInitializationState < InitializationState::controllerChecked)
	{
		if(mController && mController->isEnabled())
		{
			cout << "controller enabled" << endl;
		}
		else
		{
			cout << "controller disabled" << endl;
		}
		mInitializationState = InitializationState::controllerChecked;
	}

	/*
	 if (askToCalibrate){
	 int choice = 0;
	 cout << "Enter 1 To Calibrate, 0 for default: ";
	 cint >> choice;
	 if (choice == 1)
	 controller.calibrate();
	 askToCalibrate = false;
	 controller.setCalibration(askToCalibrate);
	 }*/

	lastTimeTemp = clock();
	dt = (float) (lastTimeTemp - lastTime) / (float) CLOCKS_PER_SEC;
	lastTime = lastTimeTemp;



	if(mInitializationState < InitializationState::initialized)
	{
		if( !(mController) || (mController && mController->isEnabled() && mController->isCalibrated()) )
		{
			Init();
		}
		else
		{
			return;
		}
	}

	if(mController && mController->isEnabled())
	{
		processControllerInput(mParamData);
		//simulateMovement(LocalizationWaypointController::State::remote, 0, 0);
	}

	if (mSimulationState == SimulationState::running)
	{
		step();

		static RobotSimEvents::BeeStemMotorControllerMessagePtr motorMsg = new RobotSimEvents::BeeStemMotorControllerMessage;
		resetMotorsExceptFor(XBox360RemoteControl::Keys::Actions::SPEED, motorMsg);
		//motorMsg->values.assign(BeeStem3::NUM_MOTOR_CONTROLLERS, 0);
		//motorMsg->mask.assign(BeeStem3::NUM_MOTOR_CONTROLLERS, 0);
		//resetMotorsExceptFor("", motorMsg);
		//joyStickMsg->axis = -1; //we only want BeeStemI to use this, so make everything else think it's an invalid message
		//joyStickMsg->button = -1; //we don't care about the buttons
		//joyStickMsg->axisVal = 0;

		RobotSimEvents::BeeStemConfigMessagePtr depthMsg;// = new RobotSimEvents::BeeStemMotorControllerMessage;

		if(mWaypointController.step(motorMsg, depthMsg, mCurrentPose))
		{
			//cout << "trying to publish 360 message...";
			//resetMotorsExceptFor(motorMsg->axisName);
			publish("BeeStemMotorControllerMessageTopic", motorMsg);
			//cout << "done" << endl;
		}
		else
		{
			if(depthMsg && depthMsg->updateDepthPID && depthMsg->updateDepthPID == 1)
			{
				//publish("BeeStemConfigMessageTopic", depthMsg);
				depthMsg->updateDepthPID = 0;
				depthMsg->enablePID = 1;
				//publish("BeeStemConfigMessageTopic", depthMsg);
			}
			//joyStickMsg->axisVal = 0;
			//resetMotorsExceptFor("");
		}

		cout << "simulating movement...";
		//simulateMovement(LocalizationWaypointController::State::move_to_waypoint, LocalizationWaypointController::MovementMode::rotate2, 25);
		simulateMovement(mWaypointController.mState, mWaypointController.mMovementState, motorMsg);
		cout << "done" << endl;
	}

	if(mDisableGraphics.getVal() == 0)
	{
		updateGraphics();
	}

	RobotSimEvents::LocalizationMessagePtr msg = new RobotSimEvents::LocalizationMessage;
	msg->mode = mWaypointController.mState;
	msg->currentWaypoint.x = mWaypointController[-1].mPoint.i;
	msg->currentWaypoint.y = mWaypointController[-1].mPoint.j;
	msg->currentWaypoint.depth = mWaypointController[-1].mDepth;
	msg->currentWaypoint.heading = mWaypointController[-1].mOrientation;
	msg->pos.i = avgParticle.mState.mPoint.i;
	msg->pos.j = avgParticle.mState.mPoint.j;
	msg->heading = avgParticle.mState.mAngle;

	publish("LocalizationMessageTopic", msg);
}

void ParticleFilter::resetMotorsExceptFor(XBox360RemoteControl::Keys::Actions::Ids actionId)
{
	static RobotSimEvents::BeeStemMotorControllerMessagePtr msg = new RobotSimEvents::BeeStemMotorControllerMessage;
	resetMotorsExceptFor(actionId, msg);
	publish("BeeStemMotorControllerMessageTopic", msg);
}

void ParticleFilter::resetMotorsExceptFor(XBox360RemoteControl::Keys::Actions::Ids actionId, RobotSimEvents::BeeStemMotorControllerMessagePtr & msg)
{
	msg->values.assign(BeeStem3::NUM_MOTOR_CONTROLLERS, 0);
	msg->mask.assign(BeeStem3::NUM_MOTOR_CONTROLLERS, 0);

	msg->mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = 1;
	msg->mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = 1;
	msg->mask[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = 1;
	msg->mask[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = 1;
	if(actionId == XBox360RemoteControl::Keys::Actions::SPEED)
	{
		msg->mask[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = 0;
		msg->mask[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = 0;
	}
	else if(actionId == XBox360RemoteControl::Keys::Actions::HEADING)
	{
		msg->mask[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = 0;
		msg->mask[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = 0;
	}
	//msg->mask[BeeStem3::MotorControllerIDs::DROPPER_STAGE1] = 0;
	//msg->mask[BeeStem3::MotorControllerIDs::DROPPER_STAGE2] = 0;
	//msg->mask[BeeStem3::MotorControllerIDs::SHOOTER] = 0;
	//msg->mask[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = 0;
	//msg->mask[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = 0;
}

// ######################################################################
void ParticleFilter::updateMessage(
		const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
	static int BinFinderCounter = 0;
	static int BuoyFinderCounter = 0;
	static int CompassSensorCounter = 0;
	static int HydrophonesCounter = 0;
	static int PipeSensorCounter = 0;
	static int RectangleSensorCounter = 0;
	const static int sensorMsgTimeout = 5; //timeout if no msg after this many updates

	if(mSensorFlags.BinFinder)
		BinFinderCounter ++;
	if(mSensorFlags.BuoyFinder)
		BuoyFinderCounter ++;
	if(mSensorFlags.CompassSensor)
		CompassSensorCounter ++;
	if(mSensorFlags.Hydrohpones)
		HydrophonesCounter ++;
	if(mSensorFlags.PipeSensor)
		PipeSensorCounter ++;
	if(mSensorFlags.RectangleSensor)
		RectangleSensorCounter ++;

	if(BinFinderCounter > sensorMsgTimeout)
		mSensorFlags.BinFinder = false;
	if(BuoyFinderCounter > sensorMsgTimeout)
		mSensorFlags.BuoyFinder = false;
	if(CompassSensorCounter > sensorMsgTimeout)
		mSensorFlags.CompassSensor = false;
	if(HydrophonesCounter > sensorMsgTimeout)
		mSensorFlags.Hydrohpones = false;
	if(PipeSensorCounter > sensorMsgTimeout)
		mSensorFlags.PipeSensor = false;
	if(RectangleSensorCounter > sensorMsgTimeout)
		mSensorFlags.RectangleSensor = false;

	if (eMsg->ice_isA("::RobotSimEvents::JoyStickControlMessage"))
	{
		RobotSimEvents::JoyStickControlMessagePtr msg = RobotSimEvents::JoyStickControlMessagePtr::dynamicCast(eMsg);

		itsJSMutex.lock();
		if (msg->axis >= 0)
		{
			itsJSValues[msg->axis] = msg->axisVal;
		}
		if (msg->button >= 0)
			itsButValues[msg->button] = msg->butVal;

		itsJSMutex.unlock();
	}

	/* need working bin finder first
	else if (eMsg->ice_isA("::RobotSimEvents::BinFinderMessage"))
	{
		mSensorFlags.BinFinder = true;
		BinFinderCounter = 0;
	}*/

	else if (eMsg->ice_isA("::RobotSimEvents::BuoyColorSegmentMessage"))
	{
		mSensorFlags.BuoyFinder = true;
		BuoyFinderCounter = 0;
		RobotSimEvents::BuoyColorSegmentMessagePtr msg = RobotSimEvents::BuoyColorSegmentMessagePtr::dynamicCast(eMsg);
		mBuoySensor->takeReading(eMsg);
	}

	else if (eMsg->ice_isA("::RobotSimEvents::IMUDataServerMessage"))
	{
		mSensorFlags.CompassSensor = true;
		CompassSensorCounter = 0;
		//cout << "IMU" << endl;
		RobotSimEvents::IMUDataServerMessagePtr msg = RobotSimEvents::IMUDataServerMessagePtr::dynamicCast(eMsg);
		if(msg->angleMode == IMUDataServer::Mode::euler)
		{
			mCompassSensor->takeReading(eMsg);
		}
	}

	/* need working hydrophones first
	else if (eMsg->ice_isA("::RobotSimEvents::HydrophoneMessage"))
	{
		mSensorFlags.Hydrohpones = true;
		HydrophonesSensorCounter = 0;
	}*/

	else if (eMsg->ice_isA("::RobotSimEvents::PipeColorSegmentMessage"))
	{
		mSensorFlags.PipeSensor = true;
		PipeSensorCounter = 0;
		//printf("got a pipe color segmenter message!\n");
		RobotSimEvents::PipeColorSegmentMessagePtr msg = RobotSimEvents::PipeColorSegmentMessagePtr::dynamicCast(eMsg);
		if(msg->size > 1500)
		{
			//printf("message contains large orange blob xD\n");
			mPipeSensor->takeReading(eMsg);
		}
	}

	else if (eMsg->ice_isA("::RobotSimEvents::VisionRectangleMessage"))
	{
		//printf("got a rectangle message!\n");
		mSensorFlags.RectangleSensor = true;
		RectangleSensorCounter = 0;
		RobotSimEvents::VisionRectangleMessagePtr msg = RobotSimEvents::VisionRectangleMessagePtr::dynamicCast(eMsg);
		mRectangleSensor->takeReading(eMsg);
	}

	else if(eMsg->ice_isA("::RobotSimEvents::BeeStemMotorControllerMessage"))
	{
		cout << "MotorController" << endl;
		RobotSimEvents::BeeStemMotorControllerMessagePtr msg = RobotSimEvents::BeeStemMotorControllerMessagePtr::dynamicCast(eMsg);
		cout << "{[" << msg->mask[0] << "]" << msg->values[0];
		for(unsigned int i = 1; i < msg->values.size(); i ++)
		{
			cout << ",[" << msg->mask[i] << "]" << msg->values[i];
		}
		cout << "}" << endl;
	}

	else if(eMsg->ice_isA("::RobotSimEvents::LocalizationMessage"))
	{
		cout << "localization" << endl;
		RobotSimEvents::LocalizationMessagePtr msg = RobotSimEvents::LocalizationMessagePtr::dynamicCast(eMsg);
		cout << "----------" << endl;
		cout << "(" << msg->pos.i << "," << msg->pos.j << ")" << endl;
		cout << msg->heading << endl;
		cout << msg->mode << endl;
		if(msg->mode == LocalizationWaypointController::State::move_to_waypoint || msg->mode == LocalizationWaypointController::State::at_waypoint)
		{
			cout << "<" << msg->currentWaypoint.x << "," << msg->currentWaypoint.y << "," << msg->currentWaypoint.depth << ">" << endl;
			cout << "@ " << msg->currentWaypoint.heading << endl;
		}
		/*cout << "(" << avgParticle.mState.mPoint.i << "," << avgParticle.mState.mPoint.j << ")" << endl;
		cout << avgParticle.mState.mAngle << endl;
		cout << mWaypointController.mMode << endl;
		if(mWaypointController.mMode == LocalizationWaypointController::State::move_to_waypoint || mWaypointController.mMode == LocalizationWaypointController::State::at_waypoint)
		{
			cout << "<" << mWaypointController.mCurrentWaypoint.mPoint.x << "," << mWaypointController.mCurrentWaypoint.mPoint.y << "," << mWaypointController.mCurrentWaypoint.mPoint.z << ">" << endl;
			cout << mWaypointController.mCurrentWaypoint.mOrientation << endl;
		}*/
	}
}

#endif
