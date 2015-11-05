#include "Robots/Scorbot/ScorbotInterface.H"

// ######################################################################
ScorbotInterface::ScorbotInterface(OptionManager& mgr,
        const std::string& descrName, const std::string& tagName) :
	ModelComponent(mgr, descrName, tagName),
	itsSerial(new Serial(mgr))
{
  addSubComponent(itsSerial);
  itsSerial->configure("/dev/ttyACM0", 115200, "8N1", false, false, 100000);

  pthread_mutex_init(&itsSerialMutex, NULL);
}

// ######################################################################
void ScorbotInterface::start2()
{
	itsSerial->flush();
	//Set the default gains
	
	setControlParams(ScorbotInterface::Base, 
			0.018,   //pGain
			.00015,      //iGain
			0.0000, //dGain 
			200,    //maxI
			1,      //maxPWM
			0      //pwmOffset
			);

	setControlParams(ScorbotInterface::Shoulder, 
			0.009,  //pGain
			0.0004, //iGain
			0,      //dGain 
			200,    //maxI
			1,      //maxPWM
			0      //pwmOffset
			);

	setControlParams(ScorbotInterface::Elbow, 
			0.015,  //pGain
			0.0002, //iGain
			0,      //dGain 
			200,    //maxI
			1,      //maxPWM
			0      //pwmOffset
			);

	setControlParams(ScorbotInterface::Wrist1, 
			0.010,  //pGain
			0.0002, //iGain
			0,      //dGain 
			200,    //maxI
			1,      //maxPWM
			0      //pwmOffset
			);

	setControlParams(ScorbotInterface::Wrist2, 
			0.015,  //pGain
			0.0008, //iGain
			0,      //dGain 
			200,    //maxI
			1,      //maxPWM
			0      //pwmOffset
			);

	setControlParams(ScorbotInterface::Slider, 
			0.0015,  //pGain
			0.00011, //iGain
			0.00011, //dGain 
			2000,    //maxI
			1,       //maxPWM
			0       //pwmOffset
			);

	//Set all desired joint positions to their current positions
	setJoints(getEncoders(), 1000);
}

// ######################################################################
ScorbotInterface::~ScorbotInterface()
{
	setEnabled(false);
}

// ######################################################################
void ScorbotInterface::setJoint(ScorbotInterface::Joint_t joint, int32 encoderPos, int32 time_ms)
{
  pthread_mutex_lock(&itsSerialMutex);
  byte cmd[11];

  cmd[0]  = 20;
  cmd[1]  = (byte)joint;

  cmd[2]  = 0x0FF & ( encoderPos >> 24 );
  cmd[3]  = 0x0FF & ( encoderPos >> 16 );
  cmd[4]  = 0x0FF & ( encoderPos >> 8 );
  cmd[5]  = 0x0FF & ( encoderPos >> 0 );

  cmd[6]  = 0x0FF & ( time_ms >> 24 );
  cmd[7]  = 0x0FF & ( time_ms >> 16 );
  cmd[8]  = 0x0FF & ( time_ms >> 8 );
  cmd[9]  = 0x0FF & ( time_ms >> 0 );
	
  cmd[10] = 255;

  itsSerial->write(cmd, 11);

  // Check for success
  byte retVal = 0;
  int retNum = itsSerial->read(&retVal, 1);
  if(retNum != 1 || retVal != 255)
    LERROR("Error Disabling Scorbot");
	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);
}

// ######################################################################
void ScorbotInterface::setJoints(ScorbotInterface::encoderVals_t pos, int32 time_ms)
{
	ScorbotInterface::encoderVals_t::iterator posIt;
	for(posIt = pos.begin(); posIt != pos.end(); ++posIt)
		setJoint(posIt->first, posIt->second, time_ms);
}

// ######################################################################
ScorbotInterface::encoderVals_t ScorbotInterface::getEncoders()
{
	ScorbotInterface::encoderVals_t encoders;

	pthread_mutex_lock(&itsSerialMutex);
	byte cmd[2];
	cmd[0] = 22;
	cmd[1] = 255;
	itsSerial->flush();
	itsSerial->write(cmd, 2);

	std::vector<byte> res = itsSerial->readFrame(cmd[0], 255, -1, .01);

	if(res.size() == 0)
		LINFO("Could Not Read Encoder Vals");
	else
	{
		int idx = 0;
		encoders[ScorbotInterface::Base]  = res[idx++] << 24;
		encoders[ScorbotInterface::Base] |= res[idx++] << 16;
		encoders[ScorbotInterface::Base] |= res[idx++] <<  8;
		encoders[ScorbotInterface::Base] |= res[idx++] <<  0;

		encoders[ScorbotInterface::Shoulder]  = res[idx++] << 24;
		encoders[ScorbotInterface::Shoulder] |= res[idx++] << 16;
		encoders[ScorbotInterface::Shoulder] |= res[idx++] <<  8;
		encoders[ScorbotInterface::Shoulder] |= res[idx++] <<  0;

		encoders[ScorbotInterface::Elbow]  = res[idx++] << 24;
		encoders[ScorbotInterface::Elbow] |= res[idx++] << 16;
		encoders[ScorbotInterface::Elbow] |= res[idx++] <<  8;
		encoders[ScorbotInterface::Elbow] |= res[idx++] <<  0;

		encoders[ScorbotInterface::Wrist1]  = res[idx++] << 24;
		encoders[ScorbotInterface::Wrist1] |= res[idx++] << 16;
		encoders[ScorbotInterface::Wrist1] |= res[idx++] <<  8;
		encoders[ScorbotInterface::Wrist1] |= res[idx++] <<  0;

		encoders[ScorbotInterface::Wrist2]  = res[idx++] << 24;
		encoders[ScorbotInterface::Wrist2] |= res[idx++] << 16;
		encoders[ScorbotInterface::Wrist2] |= res[idx++] <<  8;
		encoders[ScorbotInterface::Wrist2] |= res[idx++] <<  0;

		encoders[ScorbotInterface::Gripper]  = res[idx++] << 24;
		encoders[ScorbotInterface::Gripper] |= res[idx++] << 16;
		encoders[ScorbotInterface::Gripper] |= res[idx++] <<  8;
		encoders[ScorbotInterface::Gripper] |= res[idx++] <<  0;

		encoders[ScorbotInterface::Slider]  = res[idx++] << 24;
		encoders[ScorbotInterface::Slider] |= res[idx++] << 16;
		encoders[ScorbotInterface::Slider] |= res[idx++] <<  8;
		encoders[ScorbotInterface::Slider] |= res[idx++] <<  0;
	}

	itsSerial->flush();
	pthread_mutex_unlock(&itsSerialMutex);

	return encoders;
};


// ######################################################################
int32 ScorbotInterface::getEncoder(ScorbotInterface::Joint_t joint)
{
  pthread_mutex_lock(&itsSerialMutex);

  byte cmd[3];
	cmd[0] = 10;
	cmd[1] = (byte)joint;
	cmd[2] = 255;
  itsSerial->write(cmd, 3);

  // Check for success
  byte retVal[5];
	retVal[4] = 0;
  int retNum = itsSerial->read(retVal, 5);
  if(retNum != 5 || retVal[4] != 255)
    LERROR("Error Getting Joint Position (Joint %d): Recieved %d bytes: %d", (byte)joint, retNum, retVal[4]);

  int32 encoder = 0;
  encoder |= (0x0FF & retVal[0]) << 24;
  encoder |= (0x0FF & retVal[1]) << 16;
  encoder |= (0x0FF & retVal[2]) << 8;
  encoder |= (0x0FF & retVal[3]) << 0;

	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);

  return encoder;
}

// ######################################################################
float ScorbotInterface::getPWM(ScorbotInterface::Joint_t joint)
{
  pthread_mutex_lock(&itsSerialMutex);

  byte cmd[3];
	cmd[0] = 12;
	cmd[1] = (byte)joint;
	cmd[2] = 255;
  itsSerial->write(cmd, 3);

  // Check for success
  byte retVal[5];
	retVal[4] = 0;
  int retNum = itsSerial->read(retVal, 5);
	if(retNum != 5 || retVal[4] != 255)
		LERROR("Error Getting Joint PWM (Joint %d): Recieved %d bytes: %d", (byte)joint, retNum, retVal[4]);

  int32 pwm_fp = 0;
  pwm_fp |= (0x0FF & retVal[0]) << 24;
  pwm_fp |= (0x0FF & retVal[1]) << 16;
  pwm_fp |= (0x0FF & retVal[2]) << 8;
  pwm_fp |= (0x0FF & retVal[3]) << 0;
  
	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);

  return float(pwm_fp)/100000.0;
}

// ######################################################################
ScorbotInterface::pwmVals_t ScorbotInterface::getPWMs()
{
	ScorbotInterface::pwmVals_t PWMs;

	pthread_mutex_lock(&itsSerialMutex);
	byte cmd[2];
	cmd[0] = 23;
	cmd[1] = 255;
	itsSerial->flush();
	itsSerial->write(cmd, 2);

	std::vector<byte> res = itsSerial->readFrame(cmd[0], 255, -1, .01);

	if(res.size() == 0)
		LINFO("Could Not Read Encoder Vals");
	else
	{
		int32 tmp_fp = 0;

		int idx = 0;
		tmp_fp  = res[idx++] << 24;
		tmp_fp |= res[idx++] << 16;
		tmp_fp |= res[idx++] <<  8;
		tmp_fp |= res[idx++] <<  0;
		PWMs[ScorbotInterface::Base]     = float(tmp_fp)/100000.0;

		tmp_fp  = res[idx++] << 24;
		tmp_fp |= res[idx++] << 16;
		tmp_fp |= res[idx++] <<  8;
		tmp_fp |= res[idx++] <<  0;
		PWMs[ScorbotInterface::Shoulder] = float(tmp_fp)/100000.0;

		tmp_fp  = res[idx++] << 24;
		tmp_fp |= res[idx++] << 16;
		tmp_fp |= res[idx++] <<  8;
		tmp_fp |= res[idx++] <<  0;
		PWMs[ScorbotInterface::Elbow]    = float(tmp_fp)/100000.0;

		tmp_fp  = res[idx++] << 24;
		tmp_fp |= res[idx++] << 16;
		tmp_fp |= res[idx++] <<  8;
		tmp_fp |= res[idx++] <<  0;
		PWMs[ScorbotInterface::Wrist1]   = float(tmp_fp)/100000.0;

		tmp_fp  = res[idx++] << 24;
		tmp_fp |= res[idx++] << 16;
		tmp_fp |= res[idx++] <<  8;
		tmp_fp |= res[idx++] <<  0;
		PWMs[ScorbotInterface::Wrist2]   = float(tmp_fp)/100000.0;

		tmp_fp  = res[idx++] << 24;
		tmp_fp |= res[idx++] << 16;
		tmp_fp |= res[idx++] <<  8;
		tmp_fp |= res[idx++] <<  0;
		PWMs[ScorbotInterface::Gripper]  = float(tmp_fp)/100000.0;

		tmp_fp  = res[idx++] << 24;
		tmp_fp |= res[idx++] << 16;
		tmp_fp |= res[idx++] <<  8;
		tmp_fp |= res[idx++] <<  0;
		PWMs[ScorbotInterface::Slider]   = float(tmp_fp)/100000.0;
	}

	itsSerial->flush();
	pthread_mutex_unlock(&itsSerialMutex);

	return PWMs;
};

// ######################################################################
void ScorbotInterface::setEnabled(bool enabled)
{
  pthread_mutex_lock(&itsSerialMutex);
	

  if(enabled)
  {
		byte cmd[2];
		cmd[0] = 30;
  	cmd[1] = 255;
		itsSerial->write(cmd, 2);
		usleep(10000);

		// Check for success
		byte retVal;
		int retNum = itsSerial->read(&retVal, 1);

		if(retNum != 1 || retVal != 255)
			LERROR("Error Enabling Scorbot");
			
  }
  else
  {
    byte cmd[2];
		cmd[0] = 31;
		cmd[1] = 255;
    itsSerial->write(cmd, 2);

    // Check for success
    byte retVal;
    byte retNum = itsSerial->read(&retVal, 1);

		if(retNum != 1 || retVal != 255)
			LERROR("Error Disabling Scorbot");
  }
	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);
}

// ######################################################################
void ScorbotInterface::resetEncoders()
{

	setEnabled(false);
	usleep(10000);

  pthread_mutex_lock(&itsSerialMutex);

  byte cmd[2];
  cmd[0] = 21;
	cmd[1] = 255;
  itsSerial->write(cmd, 2);

  // Check for success
  byte retVal = 0;
  int retNum = itsSerial->read(&retVal, 1);
  if(retNum != 1 || retVal != 255)
    LERROR("Error Disabling Scorbot");

	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);

	//Reset all joints to 0
	ScorbotInterface::encoderVals_t encoders;
	encoders[ScorbotInterface::Base]     = 0;
	encoders[ScorbotInterface::Shoulder] = 0;
	encoders[ScorbotInterface::Elbow]    = 0;
	encoders[ScorbotInterface::Wrist1]   = 0;
	encoders[ScorbotInterface::Wrist2]   = 0;
	encoders[ScorbotInterface::Gripper]  = 0;
	encoders[ScorbotInterface::Slider]   = 0;
	setJoints(encoders, 1000);

	usleep(10000);
}

// ######################################################################
void ScorbotInterface::setControlParams(ScorbotInterface::Joint_t joint, 
    float pGain, float iGain, float dGain, float maxI, float maxPWM, float pwmOffset)
{
  pthread_mutex_lock(&itsSerialMutex);

  //Multiply all of our params by 100,000 to convert them to fixed point before we send them
  int32 pGain_fp     = int32(pGain     * 100000.0);
  int32 iGain_fp     = int32(iGain     * 100000.0);
  int32 dGain_fp     = int32(dGain     * 100000.0);
  int32 maxI_fp      = int32(maxI      * 100000.0);
  int32 maxPWM_fp    = int32(maxPWM    * 100000.0);
  int32 pwmOffset_fp = int32(pwmOffset * 100000.0);

  byte cmd[28];
  cmd[0] = 25;
  cmd[1] = (byte)joint;

  cmd[2] = 24; //Number of bytes to follow

  cmd[3] = 0x0FF & (pGain_fp >> 24);
  cmd[4] = 0x0FF & (pGain_fp >> 16);
  cmd[5] = 0x0FF & (pGain_fp >> 8);
  cmd[6] = 0x0FF & (pGain_fp >> 0);

  cmd[7]  = 0x0FF & (iGain_fp >> 24);
  cmd[8]  = 0x0FF & (iGain_fp >> 16);
  cmd[9]  = 0x0FF & (iGain_fp >> 8);
  cmd[10] = 0x0FF & (iGain_fp >> 0);

  cmd[11] = 0x0FF & (dGain_fp >> 24);
  cmd[12] = 0x0FF & (dGain_fp >> 16);
  cmd[13] = 0x0FF & (dGain_fp >> 8);
  cmd[14] = 0x0FF & (dGain_fp >> 0);

  cmd[15] = 0x0FF & (maxI_fp >> 24);
  cmd[16] = 0x0FF & (maxI_fp >> 16);
  cmd[17] = 0x0FF & (maxI_fp >> 8);
  cmd[18] = 0x0FF & (maxI_fp >> 0);

  cmd[19] = 0x0FF & (maxPWM_fp >> 24);
  cmd[20] = 0x0FF & (maxPWM_fp >> 16);
  cmd[21] = 0x0FF & (maxPWM_fp >> 8);
  cmd[22] = 0x0FF & (maxPWM_fp >> 0);

  cmd[23] = 0x0FF & (pwmOffset_fp >> 24);
  cmd[24] = 0x0FF & (pwmOffset_fp >> 16);
  cmd[25] = 0x0FF & (pwmOffset_fp >> 8);
  cmd[26] = 0x0FF & (pwmOffset_fp >> 0);

	cmd[27] = 255;

  itsSerial->write(cmd, 28);
	usleep(1000);

  // Check for success
  byte retVal = 0;
  int retNum = itsSerial->read(&retVal, 1);
  if(retNum != 1 || retVal != 255)
    LERROR("Error Setting Control Params (retNum: %d, retVal: %d)", retNum, retVal);

	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);
}

// ######################################################################
void ScorbotInterface::getPIDVals(ScorbotInterface::Joint_t joint,
		float &pGain, float &iGain, float &dGain, float &maxI, float &maxPWM, float &pwmOffset)
{
  pthread_mutex_lock(&itsSerialMutex);
  byte cmd[3];
  cmd[0] = 15;
  cmd[1] = (byte)joint;
  cmd[2] = 255;
  itsSerial->write(cmd, 3);

	byte numVals;
	int retNum = itsSerial->read(&numVals, 1);
	if(retNum != 1 || numVals != 24)
		LERROR("Could Not Read PID Values retNum: %d numVals %d", retNum, numVals);
	else
	{
		usleep(10000);
		byte retVal[numVals+1];
		retNum = itsSerial->read(retVal, numVals+1);
		if(retNum != numVals+1 || retVal[numVals] != 255)
			LERROR("Could Not Read PID Values (Error 2) retNum: %d, retVal[numVals]: %d", retNum, retVal[numVals]);
		else
		{
			int numIdx = 0;

			int32 pGain_fp = 0;
			pGain_fp |= retVal[numIdx++] << 24;
			pGain_fp |= retVal[numIdx++] << 16;
			pGain_fp |= retVal[numIdx++] <<  8;
			pGain_fp |= retVal[numIdx++] <<  0;

			int32 iGain_fp = 0;
			iGain_fp |= retVal[numIdx++] << 24;
			iGain_fp |= retVal[numIdx++] << 16;
			iGain_fp |= retVal[numIdx++] <<  8;
			iGain_fp |= retVal[numIdx++] <<  0;

			int32 dGain_fp = 0;
			dGain_fp |= retVal[numIdx++] << 24;
			dGain_fp |= retVal[numIdx++] << 16;
			dGain_fp |= retVal[numIdx++] <<  8;
			dGain_fp |= retVal[numIdx++] <<  0;

			int32 maxI_fp = 0;
			maxI_fp |= retVal[numIdx++] << 24;
			maxI_fp |= retVal[numIdx++] << 16;
			maxI_fp |= retVal[numIdx++] <<  8;
			maxI_fp |= retVal[numIdx++] <<  0;

			int32 maxPWM_fp = 0;
			maxPWM_fp |= retVal[numIdx++] << 24;
			maxPWM_fp |= retVal[numIdx++] << 16;
			maxPWM_fp |= retVal[numIdx++] <<  8;
			maxPWM_fp |= retVal[numIdx++] <<  0;

			int32 pwmOffset_fp = 0;
			pwmOffset_fp |= retVal[numIdx++] << 24;
			pwmOffset_fp |= retVal[numIdx++] << 16;
			pwmOffset_fp |= retVal[numIdx++] <<  8;
			pwmOffset_fp |= retVal[numIdx++] <<  0;

			pGain     = pGain_fp/100000.0;
			iGain     = iGain_fp/100000.0;
			dGain     = dGain_fp/100000.0;
			maxI      = maxI_fp/100000.0;
			maxPWM    = maxPWM_fp/100000.0;
			pwmOffset = pwmOffset_fp/100000.0;
		}
	}

	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);
}

// ######################################################################
void ScorbotInterface::getTuningVals(ScorbotInterface::Joint_t joint,
	int32 &targetPos, int32 &targetVel, float &gravityCompensation)
{
  pthread_mutex_lock(&itsSerialMutex);

  byte cmd[3];
  cmd[0] = 13;
  cmd[1] = (byte)joint;
	cmd[2] = 255;
  itsSerial->write(cmd, 3);

	std::vector<byte> ret = itsSerial->readFrame(cmd[0], 255, -1, .3);

	if(ret.size() == 0)
	{
		LERROR("Could Not Read Tuning Values");
		targetPos = 0;
		targetVel = 0;
		gravityCompensation = 0;
	}
	else
	{
		int idx=0;
		targetPos = 0;
		targetPos |= ret[idx++] << 24;
		targetPos |= ret[idx++] << 16;
		targetPos |= ret[idx++] <<  8;
		targetPos |= ret[idx++] <<  0;

		targetVel = 0;
		targetVel |= ret[idx++] << 24;
		targetVel |= ret[idx++] << 16;
		targetVel |= ret[idx++] <<  8;
		targetVel |= ret[idx++] <<  0;

		long gravityCompensationLong = 0;
		gravityCompensationLong  = ret[idx++] << 24;
		gravityCompensationLong |= ret[idx++] << 16;
		gravityCompensationLong |= ret[idx++] <<  8;
		gravityCompensationLong |= ret[idx++] <<  0;
		gravityCompensation = (float) ((float) gravityCompensationLong) / 100000.0;
	}

	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);
}

// ######################################################################
void ScorbotInterface::setGravityParameters(int32 upperArmMass, int32 foreArmMass,
  float compensationScale)
{
  pthread_mutex_lock(&itsSerialMutex);

  byte cmd[14];
  cmd[0] = 40;
	
  cmd[1] = 0x0FF & (upperArmMass >> 24);
  cmd[2] = 0x0FF & (upperArmMass >> 16);
  cmd[3] = 0x0FF & (upperArmMass >> 8);
  cmd[4] = 0x0FF & (upperArmMass >> 0);

  cmd[5]  = 0x0FF & (foreArmMass >> 24);
  cmd[6]  = 0x0FF & (foreArmMass >> 16);
  cmd[7]  = 0x0FF & (foreArmMass >> 8);
  cmd[8]  = 0x0FF & (foreArmMass >> 0);

  long compensationScale_fp = (long) (compensationScale * 100000.0);
  cmd[9]  = 0x0FF & (compensationScale_fp >> 24);
  cmd[10] = 0x0FF & (compensationScale_fp >> 16);
  cmd[11] = 0x0FF & (compensationScale_fp >> 8);
  cmd[12] = 0x0FF & (compensationScale_fp >> 0);

	cmd[13] = 255;
  itsSerial->write(cmd, 14);

	byte retVal;
	int retNum = itsSerial->read(&retVal, 1);
	if(retNum != 1 || retVal != 255)
		LERROR("Could Not Set Gravity Values retNum: %d retVal[0]: %d", retNum, retVal);

	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);
}

// ######################################################################
void ScorbotInterface::getGravityParameters(int32 &upperArmMass, int32 &foreArmMass, float &compensationScale)
{
  pthread_mutex_lock(&itsSerialMutex);

  byte cmd[3];
  cmd[0] = 41;
	cmd[1] = 255;
  itsSerial->write(cmd, 2);

	std::vector<byte> ret = itsSerial->readFrame(cmd[0], 255, -1, .3);

	if(ret.size() == 0)
	{
		LERROR("Could Not Read Gravity Values");
	}
	else
	{
		int idx=0;
		upperArmMass = 0;
		upperArmMass |= ret[idx++] << 24;
		upperArmMass |= ret[idx++] << 16;
		upperArmMass |= ret[idx++] <<  8;
		upperArmMass |= ret[idx++] <<  0;

		foreArmMass = 0;
		foreArmMass |= ret[idx++] << 24;
		foreArmMass |= ret[idx++] << 16;
		foreArmMass |= ret[idx++] <<  8;
		foreArmMass |= ret[idx++] <<  0;

		long compensationScale_fp = 0;
		compensationScale_fp  = ret[idx++] << 24;
		compensationScale_fp |= ret[idx++] << 16;
		compensationScale_fp |= ret[idx++] <<  8;
		compensationScale_fp |= ret[idx++] <<  0;
		compensationScale = (float) ((float) compensationScale_fp) / 100000.0;
	}

	itsSerial->flush();
  pthread_mutex_unlock(&itsSerialMutex);
}
