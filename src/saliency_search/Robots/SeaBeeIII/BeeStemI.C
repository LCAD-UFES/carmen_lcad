#include "Robots/SeaBeeIII/BeeStemI.H"
#include "Util/Timer.H"
#include "Robots/SeaBeeIII/XBox360RemoteControlI.H"

// ######################################################################
BeeStemI::BeeStemI(int id, OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
	RobotBrainComponent(mgr, descrName, tagName), itsStem(new BeeStem3(mgr, "BeeStem3", "BeeStem3", "/dev/ttyUSB0")),
	itsUpdateHeading(0),
	itsUpdateDepth(0),
	itsUpdateSpeed(0),
	itsLastUpdateHeading(0),
	itsLastUpdateDepth(0),
	itsLastUpdateSpeed(0),
	mShooterState(ShooterState::Idle),
	mDropperState(DropperState::AllIdle),
	mFiringDeviceID(FiringDeviceID::Null)
{
	addSubComponent(itsStem);
	//  addSubComponent(itsKillSwitch);

	//itsKillSwitch->configure("/dev/ttyUSB1",57600);
	itsEvolveSleepTime = 0;
	mFlags.initFlag = false;
	//itsJSValues.resize(8);
	itsButValues.resize(20);
	// init trigger values
	itsJSMappings[XBox360RemoteControl::Keys::Actions::SURFACE] = -100;
	itsJSMappings[XBox360RemoteControl::Keys::Actions::DIVE] = -100;
	//itsJSValues[4] = -100; //this only works if those axes are properly mapped
	//itsJSValues[5] = -100;
}

// ######################################################################
BeeStemI::~BeeStemI()
{
}

// ######################################################################
void BeeStemI::initPose()
{
	mFlags.initFlag = true;

	itsStemMutex.lock();
	itsStem->setPID(0, HEADING_K, HEADING_P, HEADING_I, HEADING_D);
	itsStem->setPID(1, DEPTH_K, DEPTH_P, DEPTH_I, DEPTH_D);

	int accelX, accelY, accelZ;
	int compassHeading, compassPitch, compassRoll;
	int internalPressure, externalPressure;
	int headingK, headingP, headingD, headingI, headingOutput;
	int depthK, depthP, depthD, depthI, depthOutput;
	char killSwitch;

	bool successful = itsStem->getSensors(accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, internalPressure, externalPressure, itsDesiredHeading, itsDesiredDepth, itsDesiredSpeed, headingK, headingP, headingD, headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, killSwitch);

	if (successful)
	{
		itsUpdateHeading = compassHeading;
		itsLastUpdateHeading = itsUpdateHeading;
		itsStem->setDesiredHeading(compassHeading);
		itsUpdateDepth = externalPressure;
		itsLastUpdateDepth = itsUpdateDepth;
		itsStem->setDesiredDepth(externalPressure);
	}

	itsStemMutex.unlock();
}

void BeeStemI::registerTopics()
{
	LINFO("Registering BeeStem Message");
	this->registerPublisher("BeeStemMessageTopic");
	registerSubscription("BeeStemMotorControllerMessageTopic");
	registerSubscription("XBox360RemoteControlMessageTopic");
	registerSubscription("BeeStemConfigTopic");
}

void BeeStemI::setValuesFromJoystick()
{
	int rightVal = itsJSMappings[XBox360RemoteControl::Keys::Actions::SPEED] * -1;//itsJSValues[1]*-1;
	int leftVal = itsJSMappings[XBox360RemoteControl::Keys::Actions::SPEED];//itsJSValues[1];

	itsStem->setThruster(BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER, rightVal);
	itsStem->setThruster(BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER, leftVal);

	//  itsJSValues[5], itsJSValues[4]
	//cout << itsJSMappings[XBox360RemoteControlI::Keys::Axes::Actions::SURFACE] << endl;
	//cout << itsJSMappings[XBox360RemoteControlI::Keys::Axes::Actions::DIVE] << endl;
	int depthVal = int((float(itsJSMappings[XBox360RemoteControl::Keys::Actions::SURFACE]) + 100.0) / 2.0 - (float(itsJSMappings[XBox360RemoteControl::Keys::Actions::DIVE]) + 100.0) / 2.0);
	//cout << depthVal << endl;
	//cout << "-----" << endl;

	int depthValRight = depthVal;
	int depthValLeft = depthVal;

	//   if(itsButValues[5])
	//     {
	//       depthValRight = 75;
	//       depthValLeft = -75;
	//     }
	//   else if(itsButValues[4])
	//     {
	//       depthValRight = -75;
	//       depthValLeft = 75;
	//     }

	itsStem->setThruster(BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER, depthValRight);
	itsStem->setThruster(BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER, depthValLeft);

	//itsJSValues[2], itsJSValues[0]
	int frontVal = itsJSMappings[XBox360RemoteControl::Keys::Actions::HEADING] + itsJSMappings[XBox360RemoteControl::Keys::Actions::STRAFE] * -1;
	int backVal = itsJSMappings[XBox360RemoteControl::Keys::Actions::HEADING] * -1 + itsJSMappings[XBox360RemoteControl::Keys::Actions::STRAFE] * -1;

	itsStem->setThruster(BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER, frontVal);
	itsStem->setThruster(BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER, backVal);
}

void BeeStemI::evolve()
{
	if (!mFlags.initFlag)
		initPose();

	if(mFlags.needsUpdateFromJoystick)
	{
		setValuesFromJoystick();
		mFlags.needsUpdateFromJoystick = false;
	}

	/****************
	** arm devices **
	****************/

	//first, put all devices at last idle state
	mShooterState = mShooterState == ShooterState::Armed ? ShooterState::Idle : mShooterState;
	mDropperState = mDropperState == DropperState::Stage1Armed ? DropperState::AllIdle : mDropperState;
	mDropperState = mDropperState == DropperState::Stage2Armed ? DropperState::Stage1Idle : mDropperState;

	//then, arm the currently selected device
	switch (mFiringDeviceID)
	{
	case FiringDeviceID::Shooter:
		mShooterState = mShooterState == ShooterState::Idle ? ShooterState::Armed : mShooterState;
		break;
	case FiringDeviceID::DropperStage1:
		mDropperState = mDropperState == DropperState::AllIdle ? DropperState::Stage1Armed : mDropperState;
		break;
	case FiringDeviceID::DropperStage2:
		mDropperState = mDropperState == DropperState::Stage1Idle ? DropperState::Stage2Armed : mDropperState;
		break;
	}

	/*******************************
	** set firing mode on devices **
	*******************************/
	if(abs(itsJSMappings[XBox360RemoteControl::Keys::Actions::FIRE_DEV]) > 0) //fire key pressed
	{
		if (mShooterState == ShooterState::Armed)
		{
			cout << "Shooter set to fire!" << endl;
			mShooterState = ShooterState::Firing;
		}
		else if(mDropperState == DropperState::Stage1Armed)
		{
			cout << "Dropper stage1 set to drop!" << endl;
			mDropperState = DropperState::Stage1Dropping;
		}
		else if(mDropperState == DropperState::Stage2Armed)
		{
			cout << "Dropper stage2 set to drop!" << endl;
			mDropperState = DropperState::Stage2Dropping;
		}
	}
	else if(abs(itsJSMappings[XBox360RemoteControl::Keys::Actions::FIRE_DEV]) == 0) //fire key released
	{
		if (mShooterState == ShooterState::Reset)
		{
			cout << "Shooter reset" << endl;
			mShooterState = ShooterState::Idle;
		}
		else if(mDropperState == DropperState::Stage1Reset)
		{
			cout << "Dropper stage1 reset" << endl;
			mDropperState = DropperState::Stage1Idle;
			/* //uncomment to auto-advance between stage1 and stage2
			 *
			 * mFiringDeviceID = FiringDeviceID::DropperStage2
			 */
		}
		else if(mDropperState == DropperState::Stage2Reset)
		{
			cout << "Dropper stage2 reset" << endl;
			mDropperState = DropperState::AllIdle;
		}
	}

	/*****************************************
	** fire devices, then reset firing mode **
	*****************************************/
	if (mShooterState == ShooterState::Firing)
	{
		fireDevice(FiringDeviceID::Shooter);
		mShooterState = ShooterState::Reset;
	}
	else if(mDropperState == DropperState::Stage1Dropping)
	{
		fireDevice(FiringDeviceID::DropperStage1);
		mDropperState = DropperState::Stage1Reset;
	}
	else if(mDropperState == DropperState::Stage2Dropping)
	{
		fireDevice(FiringDeviceID::DropperStage2);
		mDropperState = DropperState::Stage2Reset;
	}

	//done

	int accelX, accelY, accelZ;
	int compassHeading, compassPitch, compassRoll;
	int internalPressure, externalPressure;
	int headingK, headingP, headingD, headingI, headingOutput;
	int depthK, depthP, depthD, depthI, depthOutput;
	char killSwitch;
	//  int thruster1,  thruster2,  thruster3,  thruster4,  thruster5,  thruster6;

	itsStemMutex.lock();
	bool successful = itsStem->getSensors(accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, internalPressure, externalPressure, itsDesiredHeading, itsDesiredDepth, itsDesiredSpeed, headingK, headingP, headingD, headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, killSwitch);

	itsUpdateMutex.lock();
	int tempHeading = itsUpdateHeading;
	int tempDepth = itsUpdateDepth;
	int tempSpeed = itsUpdateSpeed;
	itsUpdateMutex.unlock();

	if (itsLastUpdateHeading != tempHeading)
	{
		itsLastUpdateHeading = tempHeading;
		itsStem->setDesiredHeading(tempHeading);
	}
	if (itsLastUpdateDepth != tempDepth)
	{
		itsLastUpdateDepth = tempDepth;
		itsStem->setDesiredDepth(tempDepth);
	}
	if (itsLastUpdateSpeed != tempSpeed)
	{
		itsLastUpdateSpeed = tempSpeed;
		itsStem->setDesiredSpeed(tempSpeed);
	}

	itsStemMutex.unlock();
	if (successful)
	{

		RobotSimEvents::BeeStemMessagePtr msg = new RobotSimEvents::BeeStemMessage;

		msg->accelX = accelX;
		msg->accelY = accelY;
		msg->accelZ = accelZ;
		msg->compassHeading = compassHeading;
		msg->compassPitch = compassPitch;
		msg->compassRoll = compassRoll;
		msg->internalPressure = internalPressure;
		msg->externalPressure = externalPressure;
		msg->desiredHeading = itsDesiredHeading;
		msg->desiredDepth = itsDesiredDepth;
		msg->desiredSpeed = itsDesiredSpeed;
		msg->headingK = headingK;
		msg->headingP = headingP;
		msg->headingD = headingD;
		msg->headingI = headingI;
		msg->headingOutput = headingOutput;
		msg->depthK = depthK;
		msg->depthP = depthP;
		msg->depthD = depthD;
		msg->depthI = depthI;
		msg->depthOutput = depthOutput;
		msg->killSwitch = killSwitch;

		this->publish("BeeStemMessageTopic", msg);

		//       char* str = new char[32];
		//       sprintf(str, "\n%04d", externalPressure);

		//       itsKillSwitch->write(str,32);

		//       delete str;
	}
	else if (!successful)
		LINFO("Error reading from BeeStem");

}

void BeeStemI::fireDevice(int deviceID)
{
	switch(deviceID)
	{
	case FiringDeviceID::Shooter:
		cout << "Firing torpedo!" << endl;
		itsStem->setThruster(BeeStem3::MotorControllerIDs::SHOOTER, 95);
		usleep(50 * 1000);
		itsStem->setThruster(BeeStem3::MotorControllerIDs::SHOOTER, 0);
		mShooterState = ShooterState::Reset;
		break;
	case FiringDeviceID::DropperStage1:
		cout << "Dropping first marker!" << endl;
		itsStem->setThruster(BeeStem3::MotorControllerIDs::DROPPER_STAGE1, 95);
		usleep(50 * 1000);
		itsStem->setThruster(BeeStem3::MotorControllerIDs::DROPPER_STAGE1, 0);
		mDropperState = DropperState::Stage1Reset;
		break;
	case FiringDeviceID::DropperStage2:
		cout << "Dropping second marker!" << endl;
		itsStem->setThruster(BeeStem3::MotorControllerIDs::DROPPER_STAGE2, 95);
		usleep(50 * 1000);
		itsStem->setThruster(BeeStem3::MotorControllerIDs::DROPPER_STAGE2, 0);
		mDropperState = DropperState::Stage2Reset;
		break;
	}
}

// ######################################################################
void BeeStemI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
	// Get a XBox360RemoteControl Message
	if (eMsg->ice_isA("::RobotSimEvents::JoyStickControlMessage"))
	{
		RobotSimEvents::JoyStickControlMessagePtr msg = RobotSimEvents::JoyStickControlMessagePtr::dynamicCast(eMsg);
		LINFO("Got message %d %s %d",msg->axis,msg->axisName.c_str(), msg->axisVal);

		itsJSMappings[XBox360RemoteControl::Keys::Actions::toInt[msg->axisName]] = msg->axisVal;

		static bool deviceCycleBtnWaitingReset = false; //prevent it from cycling really f-ing fast through devices
		if(XBox360RemoteControl::Keys::Actions::toInt[msg->axisName] == XBox360RemoteControl::Keys::Actions::ARM_NEXT_DEV)
		{
			if(msg->axisVal > 0 && !deviceCycleBtnWaitingReset)
			{
				mFiringDeviceID = mFiringDeviceID >= FiringDeviceID::MAX ? FiringDeviceID::Null : mFiringDeviceID + 1;
				deviceCycleBtnWaitingReset = true;
			}
			else if(msg->axisVal < 0 && !deviceCycleBtnWaitingReset)
			{
				mFiringDeviceID = mFiringDeviceID <= FiringDeviceID::Null ? FiringDeviceID::MAX : mFiringDeviceID - 1;
				deviceCycleBtnWaitingReset = true;
			}
			else if(msg->axisVal == 0 && deviceCycleBtnWaitingReset)
			{
				deviceCycleBtnWaitingReset = false;
				printf(mFiringDeviceID == FiringDeviceID::Null ? "All devices set to be disarmed\n" : "Device %d set to be armed\n", mFiringDeviceID);
			}
		}

		if (msg->button >= 0)
		{
			itsButValues[msg->button] = msg->butVal;
		}

		mFlags.needsUpdateFromJoystick = true;
	}
	else if (eMsg->ice_isA("::RobotSimEvents::BeeStemMotorControllerMessage"))
	{
		RobotSimEvents::BeeStemMotorControllerMessagePtr msg = RobotSimEvents::BeeStemMotorControllerMessagePtr::dynamicCast(eMsg);
		//cout << "setting motor values..." << endl;
		for (unsigned int i = 0; i < msg->values.size(); i++)
		{
			// printf("%d|%d : ", i, msg->values[i]);
			if(msg->mask[i] == 1)
				itsStem->setThruster(i, msg->values[i]);
		}
		//cout << endl;
	}
	// Get a BeeStemConfig Message
	else if (eMsg->ice_isA("::RobotSimEvents::BeeStemConfigMessage"))
	{
		RobotSimEvents::BeeStemConfigMessagePtr msg = RobotSimEvents::BeeStemConfigMessagePtr::dynamicCast(eMsg);

		//      int h,d,s;
		if (msg->deviceToFire && msg->deviceToFire != FiringDeviceID::Null) //so...yeah we can't fire a device and update PID at the same time, NBD
		{
			fireDevice(msg->deviceToFire);
		}
		else if (msg->enablePID == 1)
		{
			itsStemMutex.lock();
			if (msg->enableVal == 1)
				itsStem->setPID(3, msg->headingK, msg->headingP, msg->headingI, msg->headingD);
			else
				itsStem->setPID(2, msg->headingK, msg->headingP, msg->headingI, msg->headingD);
			itsStemMutex.unlock();
		}
		else if (msg->updateHeadingPID == 0 && msg->updateDepthPID == 0)
		{
			itsUpdateMutex.lock();

			LINFO("Update Pose: %d\n",msg->updateDesiredValue);
			if (msg->updateDesiredValue == 1)
				itsUpdateHeading = msg->desiredHeading;
			else if (msg->updateDesiredValue == 2)
				itsUpdateDepth = msg->desiredDepth;
			else if (msg->updateDesiredValue == 3)
				itsUpdateSpeed = msg->desiredSpeed;

			itsUpdateMutex.unlock();
		}
		else if (msg->updateDepthPID == 1)
		{
			itsStemMutex.lock();
			itsStem->setPID(0, msg->depthK, msg->depthP, msg->depthI, msg->depthD);
			itsStemMutex.unlock();
		}
		else if (msg->updateHeadingPID == 1)
		{
			itsStemMutex.lock();
			itsStem->setPID(1, msg->headingK, msg->headingP, msg->headingI, msg->headingD);
			itsStemMutex.unlock();
		}
	}
}

void BeeStemI::getMotorControllerMsg(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, int mc0, int mc1, int mc2, int mc3, int mc4, int mc5, int mc6, int mc7, int mc8)
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
}

