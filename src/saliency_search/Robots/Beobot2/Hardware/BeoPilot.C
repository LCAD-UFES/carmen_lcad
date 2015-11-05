#include "Raster/Raster.H"
#include "Robots/Beobot2/Hardware/BeoPilot.H"
#include "Ice/BeobotEvents.ice.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Util/sformat.H"
#include "Util/MathFunctions.H"
#include "Image/ShapeOps.H" // for rescale()
#include "Image/MatrixOps.H"
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#define LOG_FOLDER "../data/logs"
#define WINDOW_W 800 
#define WINDOW_H 600 
#define MAPWINDOW_H WINDOW_H-266 
#define MAPWINDOW_W 480
#define BATTERY_MIN 22.0
#define BATTERY_MAX 30.0
#define RC2VEL 3.0 // convert from -1.0~1.0 rc value to velocity m/s
const ModelOptionCateg MOC_BeoPilot = {
	MOC_SORTPRI_3, "Beobot Pilot Related Options" };

const ModelOptionDef OPT_MinimumSafeDistance =
{ MODOPT_ARG(float), "MinimumSafeDistance", &MOC_BeoPilot, OPTEXP_CORE,
	"The minimum distance (in meters) to objects we can allow before cutting off the motors.",
	"min-distance", '\0', "float", ".5"};

const ModelOptionDef OPT_MaxSpeed =
{ MODOPT_ARG(float), "MaxSpeed", &MOC_BeoPilot, OPTEXP_CORE,
	"The maximum speed of the robot (from [0 .. 1]).",
	"max-speed", '\0', "float", "1.0"};

#define ORI_OFFSET (230.0/180.0)*M_PI
const double cosOri = cos(ORI_OFFSET);
const double sinOri = sin(ORI_OFFSET);
// ######################################################################
BeoPilot::BeoPilot(int id, OptionManager& mgr,
		const std::string& descrName, const std::string& tagName) :
	RobotBrainComponent(mgr, descrName, tagName),
	itsSerial(new Serial(mgr)),
	itsTimer(1000000),
	itsSerialTimer(1000000),
	itsSerialTimeOutCounter(0),
	itsCurrMessageID(0),
	itsLastMessageID(-1),
	itsRcTransSpeed(0.0),
	itsRcRotSpeed(0.0),
	itsRcTransSpeedCapped(0.0),
	itsRcRotSpeedCapped(0.0),
	itsTransCap(0.0),
	itsRotCap(0.0),
	itsBatteryVotage(0.0),
	itsRCChannels(7, -1),
	itsEmergencyMode(-1),
	itsPosition(0.0,0.0,0.0),
	itsDiffPosition(0.0,0.0,0.0),
	itsIMUDiffPosition(0.0,0.0,0.0),
	itsMapImage(MAPWINDOW_W,MAPWINDOW_H,ZEROS),
	itsMapCenter(WINDOW_W/20,(MAPWINDOW_H)/2),
	itsEncoderTimer(1000000),
	itsLastDt(0.0),
	itsLeftMotorVelocity(0.0),
	itsRightMotorVelocity(0.0),
	itsTransVelocityCurrent(0.0),
	itsRotVelocityCurrent (0.0), 
	itsTransAccelCurrent  (0.0), 
	itsRotAccelCurrent    (0.0), 
	itsTravelDistance(0.0),
	itsTravelDistanceAuto(0.0),
	itsTravelDistanceManual(0.0),
	itsStraightDistance(0.0),
	itsPidRot(0.1,0.04,0.05,0.0,0.0,0,0,0,1.0,-1.0),
	itsPidTrans(0.1,0.10,0.05,//pid gain
			0.0,1.0,//imin,imax
			0,//errthresh,not in use
			0,0//pos,neg thresh,not in use
			,1.0,-1.0),//max,min motor

	//    nomovethresh,
	//    runpid,
	//    speed,
	//    posstaticerrthresh,
	//    negstaticerrthresh
	itsIMUheading(10.0),
	itsIMUheadingInit(10.0),
	itsIMURoll(0.0),
	itsIMUPitch(0.0),
	itsIMUon(false),
	itsIMUPosition(0.0,0.0,0.0),
	itsControlImage(512,256,ZEROS),
	itsDispImage(WINDOW_W,WINDOW_H,ZEROS),
	itsDispCameraImage(320,240,ZEROS),
	itsWaitScreen(false),
	itsMeterXwin(itsDispImage,"Pilot Control"),
	itsDisplayUpdateRate(.05),
	itsMapScale(100.0),
	itsChannelMeters(7, SimpleMeter(20, 60, 1040,1921)),
	itsBatteryMeter(SimpleMeter(20, 60, 0,3000)),
	itsVelocityTransQue(50),
	itsVelocityRotQue(50),
	itsVelocityTransCmdQue(50),
	itsVelocityRotCmdQue(50),
	itsAccelTransQue(50),
	itsAccelRotQue(50),
	itsMinimumSafeDistance(&OPT_MinimumSafeDistance, this, 0),
	itsMaxSpeed(&OPT_MaxSpeed, this, 0)
{
	addSubComponent(itsSerial);
	//Connect to the serial motor board
	//itsSerial->configureSearch ("motorboard", 115200,"ttyUSB");
	itsSerial->configureSearch ("motorboard", 115200,"ttyUSB","8N1",false,false,5);//set timout to 5ms
	//ttyUSB for USB or rfcomm from Bluetooth
	//itsSerial->configureSearch ("motorboard", 115200,"rfcomm");//ttyUSB for USB or rfcomm from Bluetooth
	//itsSerial->configure("/dev/rfcomm1", 115200);
}

// ######################################################################
// ######################################################################
void BeoPilot::start3()
{
	if(itsSerial->isSerialOk())
		LINFO("%s found on %s", itsSerial->getDeviceDescriptor().c_str(), itsSerial->getDevName().c_str());

	int code = itsSerial->getSerialErrno();
	if(code==0||code==15)//FIXXX 15 is serialErrReadTimedOut
	{

		LINFO("reserEncoder,serial code is %d",code);
		resetEncoder();
	}
	//Check our command line options to ensure that they are sane
	if(itsMinimumSafeDistance.getVal() < 0)
		LFATAL("Invalid Minimum Safe Distance (%f). Must be > 0!", itsMinimumSafeDistance.getVal());
	if(itsMaxSpeed.getVal() > 1.0 || itsMaxSpeed.getVal() < 0)
		LFATAL("Invalid Maximum Speed Distance (%f). Must be > 0 && < 1.0!", itsMaxSpeed.getVal());
}

// ######################################################################
BeoPilot::~BeoPilot()
{
	//Kill the motors on exit
	SetMotors(0,0);
}

// ######################################################################

void BeoPilot::registerTopics()
{
	registerSubscription("MotorRequestTopic");
	registerPublisher("MotorMessageTopic");
	registerSubscription("IMUMessageTopic");

	registerPublisher("ResetRequestTopic");
	registerSubscription("ResetRequestTopic");
}


// ######################################################################
void BeoPilot::evolve()
{

  //Recieve telemetry from the propeller
  LDEBUG("Evolve Start");

  if(UpdateRCStatus())
  {
    itsVelocityTransCmdQue.add(itsRcTransSpeedCapped * RC2VEL);
    itsVelocityRotCmdQue.add(itsRcRotSpeedCapped * RC2VEL);


    //Local copies of all shared data that we deal with so that we can
    //hold onto our locks for as short a time as possible
    float loc_rotVel;
    float loc_transVel;

    //Set the current speed of the robot by either clamping it to the requested speed, or killing it
    //if we are in a dangerous situation
    itsSpeedMutex.lock();
    {
      itsRotationalSpeed = itsRotationalSpeedReq;
      itsForwardSpeed = itsForwardSpeedReq;
      //Clamp the latest movement requests to the maximum speed as
      //specified by the command line
      if(itsRotationalSpeed > itsMaxSpeed.getVal())
        itsRotationalSpeed = itsMaxSpeed.getVal();
      else if(itsRotationalSpeed < -itsMaxSpeed.getVal())
        itsRotationalSpeed = -itsMaxSpeed.getVal();

      if(itsForwardSpeed > itsMaxSpeed.getVal())
        itsForwardSpeed = itsMaxSpeed.getVal();
      else if(itsForwardSpeed < -itsMaxSpeed.getVal())
        itsForwardSpeed = -itsMaxSpeed.getVal();

      //Set our local copies to the newly clamped requested speeds
      loc_rotVel   = itsRotationalSpeed;
      loc_transVel = itsForwardSpeed;
      //LINFO("Set Local Rot/TranVel (%f, %f)", loc_rotVel, loc_transVel);
    }
    itsSpeedMutex.unlock();
    //Move motor in manual mode
    if(itsRemoteMode == 1 ||itsRemoteMode == 3)
    {
      //Send the actual motor commands to the motor driver/propeller
      SetMotors(loc_rotVel, loc_transVel);
    }
    else if(itsRemoteMode == 2)
    {

      float rot = itsPidRot.update(itsRcRotSpeedCapped*3,itsRotVelocityCurrent);
      float trans = itsPidTrans.update(itsRcTransSpeedCapped*3,itsTransVelocityCurrent);
      //LINFO("Trans %4.4f,Target %4.4f,Current %4.4f,Error %4.4f,Ierror %4.4f",trans,itsTransVelocityTarget, 
      //      itsTransVelocityCurrent,itsPidTrans.getErr());
      //LINFO("Pid Rot CMD %4.4f,err %4.4f,Trans CMD %4.4f,val %4.4f,err %4.4f",rot,itsPidRot.getErr(),trans,itsPidTrans.getVal(),itsPidTrans.getErr());
      SetMotors(rot, trans);
    }

    //Report the current motor speeds to IceStorm
    BeobotEvents::MotorMessagePtr msg = new BeobotEvents::MotorMessage;

    msg->RequestID = itsCurrMessageID;

    msg->transVel = loc_transVel;
    msg->rotVel   = loc_rotVel;
    msg->motor1   = Ice::Int(itsMotor1Speed);
    msg->motor2   = Ice::Int(itsMotor2Speed);

    double loc_IMUheading = 10.0;
    itsIMUMutex.lock();
    {
      //	LINFO("locIMU %f, itsIMU %f",loc_IMUheading,itsIMUheading);
      loc_IMUheading = itsIMUheading;
    }
    itsIMUMutex.unlock();

    msg->rawEncoderX = itsDiffPosition.x;
    msg->rawEncoderY = itsDiffPosition.y;
    msg->rawEncoderOri = itsPosition.z;

    msg->battery = itsBatteryVotage;


    if(loc_IMUheading <= M_PI && loc_IMUheading >= -M_PI && loc_IMUheading < 10.0)
    {
      msg->encoderX   = itsIMUDiffPosition.x;
      msg->encoderY   = itsIMUDiffPosition.y;
      msg->encoderOri = itsIMUDiffPosition.z;

      msg->imuHeading = loc_IMUheading;//absolute imu heading in global 

      Point3DwithColor<double> f = itsIMUDiffPosition;
      //LINFO("[%6d]MotorMessageTopic: Using IMU %9.6f %9.6f %9.6f",itsCurrMessageID,f.x,f.y,f.z);
    }
    else
    {
      msg->encoderX   = itsDiffPosition.x;
      msg->encoderY   = itsDiffPosition.y;
      msg->encoderOri = itsPosition.z;// accumulate heading

      msg->imuHeading = -2*M_PI;// imu not available, sending invalid value	
    } 

    msg->rcTransVel = itsRcTransSpeed;
    msg->rcRotVel   = itsRcRotSpeed;
    msg->rcTransCap = itsTransCap;
    msg->rcRotCap   = itsRotCap;
    msg->rcMode     = itsRemoteMode;

    msg->robotTransVel = itsTransVelocityCurrent;
    msg->robotRotVel   = itsRotVelocityCurrent;	

    msg->trajLen    = itsTravelDistance;//total move length
    msg->dist       = itsStraightDistance;//distance between orgin and current
    this->publish("MotorMessageTopic", msg);
    //LINFO("MotorMessageTopic: %d timer: %f: [%f %f]", 
    //	  itsCurrMessageID, itsTimer.get()/1000.0F/itsCurrMessageID,
    //	  msg->robotTransVel, msg->robotRotVel);


    itsCurrMessageID++;
  }//if RC update OK

  usleep(10000);
  double locSerialTime = (double) itsSerialTimer.get()/1000000.0;//sec
  int code = itsSerial->getSerialErrno();
  //LINFO("Test Serial 1");
  if(code==0||code==15){//FIXXX 15 is serialErrReadTimedOut
    itsSerialTimer.reset();
  }else if(locSerialTime > 0.5){
    LINFO("Try reconnect Timer:%f",locSerialTime);
    itsSerialTimer.reset();
    itsSerial->searchDevice ("ttyUSB","motorboard");
    itsSerial->enablePort(itsSerial->getDevName().c_str());///dev/ttyUSB0

    LINFO("Tried Reconnect to %s",itsSerial->getDevName().c_str());
    //LINFO("Tried Reconnect to %s,please run \"sudo rfcomm connect 1\" for bluetooth serial",itsSerial->getDevName().c_str());
    usleep(100000);


  }else{
    LINFO("Serial Timer:%f",locSerialTime);
    LINFO("Serial Error Number:[%d]",itsSerial->getSerialErrno());
  }
  //								LINFO("Test Serial 2");

  //								LINFO("Test Serial 3");

  //Draw the GUI to the screen
  updateGUI();
  handleUserEvent();

}
// ######################################################################
void BeoPilot::reset()
{
	itsIMUheadingInit = 10.0;
	itsIMUon = false;
	resetEncoder();
	itsIMUPosition     = Point3DwithColor<double>(0.0,0.0,0.0);
	itsPosition        = Point3DwithColor<double>(0.0,0.0,0.0);
	itsDiffPosition    = Point3DwithColor<double>(0.0,0.0,0.0);  
	itsIMUDiffPosition = Point3DwithColor<double>(0.0,0.0,0.0);
	itsMapScale =100.0;
	reDrawMap(itsMapScale);
	
}
// ######################################################################
void BeoPilot::handleUserEvent()
{
	//handle clicks
	//	const nub::soft_ref<ImageDisplayStream> ids =
	//		itsOfs->findFrameDestType<ImageDisplayStream>();
	//

	int key = itsMeterXwin.getLastKeyPress();

	BeobotEvents::ResetRequestPtr msg = new BeobotEvents::ResetRequest;
	switch(key)
	{
		case -1:
			break;
		case 27: //r reset pilot

			//reset pilot first then reset rest app
			reset();
			//Send Reset Request to all other Beo App
			msg->RequestAppID = BEO_PILOT;
			msg->ResetID = BEO_ALL;
			this->publish("ResetRequestTopic", msg);
			break;
		case 13: //g (MAC)                               
		case 42: //g 
			break;
		case 11: //f (mac)
		case 41: //f 
			break;
		case 10: //d (mac) 
		case 40: //d 
			break;
		case 9:  //s (mac) 
		case 39: //s 
			break;
		case  8: //a (mac) 
		case 38: //a 
			break;
		case 12: //h (mac) 
		case 43: //h
			break;
	//case 12: //j (mac) 
		case 44: //j
			break;
		case 57: //space (mac)(n on PC) 
		case 65: //space pause/resume
			itsWaitScreen =! itsWaitScreen;
			break;
		case 14: //z (mac)
		case 52: //z switch to Anf
			break;
		case 15: //x switch to SSL 
		case 53: //x switch to SSL 
			break;
		case 16: //c (mac) 
		case 54: //c switch to Equad
			break;
		case 17: //v (mac)
		case 55: //v switch to HNB 
			break;
		case 19: //b (mac) 
		case 56: //b switch to RoadGenerator 
			break;
		case 20: //q (mac) 
		case 24: //q Turn on/off VPD 
			break;
		case 21: //w (mac) 
		case 25: //w Turn on/off CSH 
			break;
		case 111://up key
			itsMapCenter.j -=10;
			break;
		case 116://down key
			itsMapCenter.j +=10;
			break;
		case 113://left key
			itsMapCenter.i -=10;
			break;
		case 114://right key
			itsMapCenter.i +=10;
			break;
		default:		
			LINFO("key %i", key);
			break;
	}

	Point2D<int> pos = itsMeterXwin.getLastMouseClick();
	if (pos.isValid())
	{
		PixRGB<byte> pixColor = itsDispImage.getVal(pos);
		LINFO("Mouse Click (%d %d) Color (%d,%d,%d)", 
				pos.i,pos.j,pixColor.red(),pixColor.green(),pixColor.blue());
	} 
}	
// ######################################################################
bool BeoPilot::UpdateRCStatus()
{
	unsigned char cmd = 107;
	LDEBUG("Send cmd to motorboard %d",cmd);
	itsSerial->write(cmd);

	std::vector<unsigned char> frame = 
    itsSerial->readFrame(cmd, 255, -1, .2);//FIXXX Serial Crash Here

	if(frame.size() == 30|| frame.size()==26)
	{
		//LINFO("Receive motor package");
		itsRCChannels[0] = ((0x0FF & frame[ 0]) << 8) |
			((0x0FF & frame[ 1]) << 0);

		itsRCChannels[1] = ((0x0FF & frame[ 2]) << 8) |
			((0x0FF & frame[ 3]) << 0);

		itsRCChannels[2] = ((0x0FF & frame[ 4]) << 8) |
			((0x0FF & frame[ 5]) << 0);

		itsRCChannels[3] = ((0x0FF & frame[ 6]) << 8) |
			((0x0FF & frame[ 7]) << 0);

		itsRCChannels[4] = ((0x0FF & frame[ 8]) << 8) |
			((0x0FF & frame[ 9]) << 0);

		itsRCChannels[5] = ((0x0FF & frame[10]) << 8) |
			((0x0FF & frame[11]) << 0);

		itsRCChannels[6] = ((0x0FF & frame[12]) << 8) |
			((0x0FF & frame[13]) << 0);

		itsEmergencyMode = frame[14];
		if(itsRCChannels[2] > 1800)
			itsEmergencyMode = 255;
		itsRemoteMode    = frame[15];
		itsMotor1Speed   = (frame[16] - 64.0) / 64.0;
		itsMotor2Speed   = (frame[17] - 64.0) / 64.0;


		itsLastLeftEncoder  = itsLeftEncoder;
		itsLeftEncoder   = ((0x0FF & frame[18]) << 24) |
			((0x0FF & frame[19]) << 16) |
			((0x0FF & frame[20]) <<  8) |
			((0x0FF & frame[21]) <<  0);

		itsLastRightEncoder = itsRightEncoder;
		itsRightEncoder  =
			((0x0FF & frame[22]) << 24) |
			((0x0FF & frame[23]) << 16) |
			((0x0FF & frame[24]) <<  8) |
			((0x0FF & frame[25]) <<  0);

		if(frame.size() == 30)
		{
			itsBatteryVotage =
				((0x0FF & frame[26]) << 24) |
				((0x0FF & frame[27]) << 16) |
				((0x0FF & frame[28]) <<  8) |
				((0x0FF & frame[29]) <<  0);
			itsBatteryVotage/= 1000000.0;	
			//LINFO("battery %f",itsBatteryVotage);
		}
		//Compute Speed Cap and Rotational Cap
		double speed_cap = 100 - ((itsRCChannels[2] - 1041) * 100)/(1750 -1041);
		double rot_vel_cap = ((itsRCChannels[5] -900)* 100)/(2073 -900);
		if(speed_cap >  100)speed_cap = 100;
		if(speed_cap <    0)speed_cap = 0;
		if(rot_vel_cap   >  100)rot_vel_cap= 100;
		if(rot_vel_cap   <    0)rot_vel_cap= 0;
		itsTransCap = speed_cap;
		itsRotCap = rot_vel_cap;

		if(itsRCChannels[2] < 1800){
			itsRcTransSpeed = (200 - ((itsRCChannels[1] - 1041) * 200)/(1874 - 1041)) -100;
			//if it's mode 1
			if( itsRCChannels[6] > 1800)
				itsRcRotSpeed =         (200 - ((itsRCChannels[0] - 963) * 200)/(1775 -963)) -100;
			else
				itsRcRotSpeed =         (200 - ((itsRCChannels[0] - 1105) * 200)/(1928 -1105)) -100;

			if(itsRcTransSpeed >  100)itsRcTransSpeed = 100;
			if(itsRcTransSpeed < -100)itsRcTransSpeed = -100;
			if(itsRcRotSpeed   >  100)itsRcRotSpeed= 100;
			if(itsRcRotSpeed   < -100)itsRcRotSpeed= -100;

			itsRcTransSpeedCapped =itsRcTransSpeed* speed_cap/100;
			itsRcRotSpeedCapped = itsRcRotSpeed * rot_vel_cap/100;
			itsRcTransSpeed /=100.0;
			itsRcRotSpeed /= 100.0;
			itsRcTransSpeedCapped /= 100.0;
			itsRcRotSpeedCapped /= 100.0;
			LDEBUG("RC cap %f Rc rot cap %f", speed_cap,rot_vel_cap);
			LDEBUG("RC trans%f Rc rot  %f",  itsRcTransSpeed,itsRcRotSpeed);
		}else{
			itsRcTransSpeed = 0;
			itsRcRotSpeed = 0;
			itsRcTransSpeedCapped = 0;
			itsRcRotSpeedCapped = 0;
			itsRotCap = 0;
			itsTransCap = 0;
		}

		//After Get RC update,we update encoder value
		UpdatePosition();
    return true;
	}
	else
	{
		//LERROR("\n\n\nBAD FRAME RECIEVED!\n\n");
	}
  return false;

}
// ######################################################################

// We have 10 inch diameter wheel
// Motor Shaft will turn 21 times for one wheel spin
// The encoder have 4000 tick per revolution
// When the wheel have one revolution, it travels 10 * pi inch
//
// 10* pi inch  = 0.797964534 meters
// Which have 1000 * 21 ticks
//  (10 * pi)/84,000 * 0.0254 =  9.49957779 * 10^(-6) = 0.000009.49957779 meter
// Which means for every tick,
// the wheel will move  meter
//
// Width between two wheel W = 510 mm = 0.51m
// To convert odometry data to position, we need do some math again...
// Ot+1 = Ot + (Dr - Dl) / W
// Dt,t+1 = (Dr + Dl) / 2
// Xt+1 = Xt + Dt,t+1 * cos(Ot+1)
// Yt+1 = Yt + Dt,t+1 * sin(Ot+1)
void BeoPilot::UpdatePosition(void)
{
	if(itsLastRightEncoder == 0.0 && itsLastLeftEncoder == 0.0)
		itsEncoderTimer.reset();

	double time = (double) itsEncoderTimer.getReset()/1000000.0;//sec

	//        double meterPerTick = 0.00000949957779 ;
	double ticksPerMeter = 105267.836;
	double dr = (itsLastRightEncoder - itsRightEncoder) / ticksPerMeter;//FIXXX
	double dl = (itsLastLeftEncoder - itsLeftEncoder)  / ticksPerMeter;//FIXXX
	double w  = BEOBOT2_ROBOT_WHEEL_DIST/1000.0;//0.51m
	double ot = itsPosition.z + ((dr - dl) / w )  ;
	double dt = (dr + dl)/2;
	double xt = itsPosition.x + dt * cos(ot);
	double yt = itsPosition.y + dt * sin(ot);

	// compute motor velocity
	if(time != 0.0){
		itsLeftMotorVelocity = (dl/time);
		itsRightMotorVelocity = (dr/time);
		double lastTransVel = itsTransVelocityCurrent;
		double lastRotVel = itsRotVelocityCurrent;
		itsRotVelocityCurrent = itsRightMotorVelocity - itsLeftMotorVelocity;
		itsTransVelocityCurrent = (itsLeftMotorVelocity+itsRightMotorVelocity)/2;



		double tranAcc = (itsTransVelocityCurrent - lastTransVel)/time;
		double rotAcc  = (itsRotVelocityCurrent - lastRotVel)/time;
		if(fabs(tranAcc) < 100.0) itsTransAccelCurrent = tranAcc;
		if(fabs(rotAcc)  < 100.0) itsRotAccelCurrent= rotAcc;

		itsAccelTransQue.add(itsTransAccelCurrent);
		itsAccelRotQue.add(itsRotAccelCurrent);

		//if(fabs(itsTransVelocityCurrent) > 5.0) itsTransVelocityCurrent = lastTransVel;
		//if(fabs(itsRotVelocityCurrent) > 5.0) itsRotVelocityCurrent = lastRotVel;
		itsVelocityTransQue.add(itsTransVelocityCurrent);
		itsVelocityRotQue.add(itsRotVelocityCurrent);

	}
	LDEBUG("Time[%f] Left V %f Right V %f,dl %f,dr %f",time,itsLeftMotorVelocity,
			itsRightMotorVelocity,dl,dr);

	// the difference in position between the current and previous time step
	itsDiffPosition.x = dt * cos(ot);
	itsDiffPosition.y = dt * sin(ot);
	itsDiffPosition.z = ((dr - dl) / w ); //use z as orientation data

	itsPosition.x = xt;
	itsPosition.y = yt;
	//     0
	// 90     -90
	//   180,-180
	//190 -> -170

	//if(ot > M_PI)  ot -= 2*M_PI;
	//else if(ot < -M_PI) ot+=2*M_PI;
	itsPosition.z = ot;//use z as orientation data


	if(itsRemoteMode == 1)
		itsPosition.color = PixRGB<byte>(255,0,0);
	else
		itsPosition.color = PixRGB<byte>(0,255,0);

	updateIMUPosition(dt);
}
//####################################################################
void BeoPilot::updateIMUPosition(double dt)
{
	double loc_IMUheading = 10.0;
	itsIMUMutex.lock();
	{
		//	LINFO("locIMU %f, itsIMU %f",loc_IMUheading,itsIMUheading);
		loc_IMUheading = itsIMUheading;
	}
	itsIMUMutex.unlock();
	if(loc_IMUheading <= M_PI && loc_IMUheading >= -M_PI && loc_IMUheading < 10.0)
	{
		//LINFO("I have locIMU %f, itsIMU %f",loc_IMUheading,itsIMUheading);
		//initialized first imu heading
		if(itsIMUheadingInit == 10.0)
		{
			itsIMUheadingInit = loc_IMUheading;	
			itsIMUon = true;
			LDEBUG("initial imu is %f",itsIMUheadingInit);
		}
		//LINFO("Got IMU Reading, Current Heading is %f",loc_IMUheading);	
		double imu_ot = (loc_IMUheading - itsIMUheadingInit);//+M_PI; 
		//double dt = (dr + dl)/2;
		double imu_diff_x = dt * cos(imu_ot);
		double imu_diff_y = dt * sin(imu_ot);
		double imu_xt = itsIMUPosition.x + imu_diff_x;
		double imu_yt = itsIMUPosition.y + imu_diff_y;
		//LDEBUG("ENC ot %3.2f,xt %3.2f,yt %3.2f,IMU ot %3.2f,xt %3.2f,yt %3.2f",ot,xt,yt,imu_ot,imu_xt,imu_yt);
		itsIMUPosition.x = imu_xt;
		itsIMUPosition.y = imu_yt;
		itsIMUPosition.z = imu_ot;//use z as orientation data
		itsIMUPosition.color = PixRGB<byte>(0,0,255);//IMU use blue

		itsIMUDiffPosition.x = imu_diff_x; 
		itsIMUDiffPosition.y = imu_diff_y; 
		itsIMUDiffPosition.z = imu_ot; 
    //LINFO("[%6d][%6d][%2d] DX %9.6f DY %9.6f O %9.6f",itsCurrMessageID,itsLastMessageID,
    //                                              itsCurrMessageID-itsLastMessageID,
    //                                              imu_diff_x,imu_diff_y,imu_ot);
    itsLastMessageID = itsCurrMessageID;
		itsTravelDistance += sqrt(imu_diff_x*imu_diff_x + imu_diff_y*imu_diff_y);
		itsStraightDistance = sqrt(imu_xt*imu_xt +imu_yt*imu_yt); 

	}else{
    //LINFO("No Imu, using encoder for orientation instead");
		//itsIMUheading = itsPosition.z;//when imu is not available, juse use wheel heading
		itsIMUPosition.z = itsPosition.z;//
		double dx = itsDiffPosition.x;
		double dy = itsDiffPosition.y;
		double xt = itsPosition.x;
		double yt = itsPosition.y;
		itsTravelDistance += sqrt(dx*dx +dy*dy); 
		itsStraightDistance = sqrt(xt*xt + yt*yt);

	}
}		
//#####################################################3
unsigned int BeoPilot::getRCChannel(int channel)
{
	if(channel < 0 || channel > 7)
		return 0;

	unsigned char cmd[2] = {104, (char)channel};

	itsSerial->write(cmd, 2);
	std::vector<unsigned char> frame = itsSerial->readFrame(cmd[0], 255);

	if(frame.size() == 4)
	{
		unsigned int val = ((0x0FF & frame[0]) << 24) |
			((0x0FF & frame[1]) << 16) |
			((0x0FF & frame[2]) <<  8) |
			((0x0FF & frame[3]) <<  0);
		return val;
	}
	else
		LINFO("Bad Frame Size! (%lu)", frame.size());

	return 0;

}
//#####################################################3
unsigned char BeoPilot::getRCStatus()
{
	unsigned char cmd = 103;

	itsSerial->write(cmd);
	std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 255);

	if(frame.size() == 1)
		return frame[0];

	else
		LERROR("Bad Frame Size! (%lu)", frame.size());
	return 0;
}

// ######################################################################
unsigned char BeoPilot::getRCEnabled()
{
	unsigned char cmd = 105;

	itsSerial->write(cmd);
	std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 255);

	if(frame.size() == 1)
		return frame[0];

	else
		LERROR("Bad Frame Size! (%lu)", frame.size());
	return 0;
}

// ######################################################################
unsigned int BeoPilot::getRCSpeed()
{
	unsigned char cmd = 106;

	itsSerial->write(cmd);
	std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 255);

	if(frame.size() == 4)
	{
		unsigned int val = ((0x0FF & frame[0]) << 24) |
			((0x0FF & frame[1]) << 16) |
			((0x0FF & frame[2]) <<  8) |
			((0x0FF & frame[3]) <<  0);
		return val;
	}
	else
		LINFO("Bad Frame Size! (%lu)", frame.size());

	return 0;

}
// ######################################################################
void BeoPilot::resetEncoder()
{
	LINFO("Trying to reset encoder...Send cmd 108");
	unsigned char cmd = 108;
	itsSerial->write(cmd);
	LINFO("Reset encoder Done...");

	itsPosition.x = 0;
	itsPosition.y = 0;
	itsPosition.z = 0;
	itsDiffPosition.x = 0;
	itsDiffPosition.y = 0;
	itsDiffPosition.z = 0;
	itsLastLeftEncoder = 0;
	itsLastRightEncoder = 0;
	itsLeftEncoder = 0;
	itsRightEncoder = 0;
}
// ######################################################################
double BeoPilot::getBatteryVoltage()
{
	LINFO("Read Battery Voltage");
	unsigned char cmd = 109;

	itsSerial->write(cmd);
	std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 255);
	LINFO("Got Battery Voltage");
	if(frame.size() == 4)
	{
		double val = ((0x0FF & frame[0]) << 24) |
			((0x0FF & frame[1]) << 16) |
			((0x0FF & frame[2]) <<  8) |
			((0x0FF & frame[3]) <<  0);
		val /= 1000000.0;	
		LINFO("Battery Voltage %f",val);
		return val; 
	}
	else
		LINFO("Bad Frame Size! (%lu)", frame.size());


	return 0;
}
// ######################################################################
double BeoPilot::adc2volt(unsigned char adc)
{	
		float R1 = 21400.0;    // !! resistance of R1 !!
		float R2 = 2145.0;     // !! resistance of R2 !!
		double vout = (adc * 4.869689)/1024.0;
		double vin = vout / (R2/(R1+R2));
		//LINFO("Voltage is %f",vin);
		return vin;
}
// ######################################################################
void BeoPilot::SetMotors(float rotationalSpeed, float translationalSpeed)
{
	//Clamp the motor speeds to [-1 .. 1]
	if(rotationalSpeed > 1.0)
		rotationalSpeed = 1.0;
	if(rotationalSpeed < -1.0)
		rotationalSpeed = -1.0;
	if(translationalSpeed > 1.0)
		translationalSpeed = 1.0;
	if(translationalSpeed < -1.0)
		translationalSpeed = -1.0;

	// if(itsBatteryVotage > BATTERY_MIN && itsBatteryVotage < BATTERY_MAX+5.0)
	// {
	// 	double powerRatio = itsBatteryVotage/BATTERY_MAX;//current voltage 22V~30v/30.0V
	// 	if(powerRatio > 1.0 || powerRatio <= 0.0) powerRatio = 1.0;
	// 	double compensateT = 0.0,compensateR = 0.0;
	// 	if(itsTransCap)
	// 		compensateT = ((translationalSpeed/itsTransCap)/powerRatio)*itsTransCap;
	// 	if(itsRotCap)
	// 		compensateR = ((rotationalSpeed/itsRotCap)/powerRatio)*itsRotCap;
	// 	translationalSpeed = compensateT;
	// 	rotationalSpeed = compensateR;

	// 	//LINFO("BAT:%4.2f Ratio:%4.2f T %4.2f -> cT %4.2f,R %4.2f ->cR %4.2f",
	// 	//itsBatteryVotage, powerRatio,translationalSpeed,compensateT,rotationalSpeed,compensateR);
	// }else{
	// 	//LINFO("Batery info not available %4.2fV",itsBatteryVotage);	
	// }
	char rotSpeed  = char(rotationalSpeed    * 100.0);
	char tranSpeed = char(translationalSpeed * 100.0);

	//Send the command for the first motor
	unsigned char cmd[3];
	cmd[0] = 100;             //Command: Set Motor 1
	cmd[1] = rotSpeed;        //Command: Rotational Speed
	itsSerial->write(cmd, 2);
	usleep(10000);

	//Send the command for the second motor
	cmd[0] = 101;             //Command: Set Motor 2
	cmd[1] = tranSpeed;       //Command: Translational Speed
	itsSerial->write(cmd, 2);
	usleep(10000);
}



// ######################################################################
void BeoPilot::updateGUI()
{
	if(itsDisplayTimer.getSecs() > itsDisplayUpdateRate)
	{
		itsDisplayTimer.reset();
		//Grab the last key pressed by the user
		// int key = itsMeterXwin.getLastKeyPress();

		float loc_forwardSpeed    = 0;
		float loc_rotationalSpeed = 0;
		itsSpeedMutex.lock();
		{
      loc_forwardSpeed    = itsForwardSpeed;
      loc_rotationalSpeed = itsRotationalSpeed;
		}
		itsSpeedMutex.unlock();

		//Update the display
		itsControlImage = Image<PixRGB<byte> >(512,256,ZEROS);

		char buffer[128];
		int speedHeight = 0;
		sprintf(buffer, "Fwr: %6.3f:%6.3f", loc_forwardSpeed,loc_forwardSpeed*itsTransCap/100.0);
		writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

		speedHeight+=20;
		sprintf(buffer, "Rot: %6.3f:%6.3f", loc_rotationalSpeed, loc_rotationalSpeed *itsRotCap/100.0);
		writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

		speedHeight+=20;
		sprintf(buffer, "TransCap: %3d  RotatCap %3d", (int)itsTransCap,(int)itsRotCap);
		writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

		speedHeight+=12;
		sprintf(buffer, "Vel T:%5.2f R:%5.2f MT:%4.2f mT:%4.2f MR:%4.2f",
			itsTransVelocityCurrent,itsRotVelocityCurrent,itsVelocityTransQue.getMax(),itsVelocityTransQue.getMin(),
			itsVelocityRotQue.getMax());
		writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(10));

		speedHeight+=16;
		float maxAccT = itsAccelTransQue.getMax();//FIXXXX
		sprintf(buffer, "Acc T:%5.2f R:%5.2f MT:%4.2f mT:%4.2f", 
			itsTransAccelCurrent,itsRotAccelCurrent,maxAccT,itsAccelTransQue.getMin());
		writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(10));

		//sprintf(buffer, "Right: %1.3f m/s Left: %1.3f m/s", itsRightMotorVelocity,itsLeftMotorVelocity);
		//writeText(itsControlImage, Point2D<int>(0,60), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

		sprintf(buffer, "Right Encoder: %d Left Encoder: %d", itsRightEncoder,itsLeftEncoder);
		writeText(itsControlImage, Point2D<int>(0,90), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

		int encHeightStart = 98;
		int encHeightSize = 12;
		int encHeightCurrent = encHeightStart;
		int encFontSize = 8;
		
		sprintf(buffer, "Enc X: %f m",itsPosition.x );
		writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize));

		encHeightCurrent+= encHeightSize;
		sprintf(buffer, "Enc Y: %f m",itsPosition.y );
		writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize+4));


		encHeightCurrent+= encHeightSize+8;
		sprintf(buffer, "Enc R: %f deg",(itsPosition.z/M_PI)*180.0f );
		writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize));

		encHeightCurrent+= encHeightSize;
		sprintf(buffer, "IMU X: %f m",itsIMUPosition.x );
		writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize));

		encHeightCurrent+= encHeightSize;
		sprintf(buffer, "IMU Y: %f m",itsIMUPosition.y );
		writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

		//draw compass from imu yaw reading
		PixRGB<byte> imuRingColor(0,128,255);
		drawCircle(itsControlImage, Point2D<int>(340,200), 45,    imuRingColor);
		drawHalfDisk(itsControlImage, Point2D<int>(340,200), 43,  PixRGB<byte>(0,255,0),itsIMURoll);
		drawLine(itsControlImage, Point2D<int>(340,200),itsIMUPosition.z, 90, PixRGB<byte>(255,0,0),3);
		drawLine(itsControlImage, Point2D<int>(340-55,200),0, 20, imuRingColor);
		drawLine(itsControlImage, Point2D<int>(340+55,200),0, 20, imuRingColor);
		drawLine(itsControlImage, Point2D<int>(340,200-55),M_PI/2, 20, imuRingColor);
		drawLine(itsControlImage, Point2D<int>(340,200+55),M_PI/2, 20, imuRingColor);
		sprintf(buffer, "%6.2f deg",(itsIMUPosition.z/M_PI)*180.0f);
		writeText(itsControlImage, Point2D<int>(300,195), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		if(itsIMUon){
			sprintf(buffer, "IMU:ON");
			writeText(itsControlImage, Point2D<int>(375,240), buffer, PixRGB<byte>(0,255,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		}else{
			sprintf(buffer, "IMU:Off");
			writeText(itsControlImage, Point2D<int>(375,240), buffer, PixRGB<byte>(255,0,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		}

    //show current serial status
		PixRGB<byte> serialOkColor = PixRGB<byte>(20,128,20);
		double locSerialTime = (double) itsSerialTimer.get()/1000.0;//ms
		int code = itsSerial->getSerialErrno();
		if(code==0 && itsSerialTimeOutCounter == 0)
		{
			sprintf(buffer, "Serial OK %2d",code);
			if(locSerialTime < 3.0)
				serialOkColor = PixRGB<byte>(20,128,20);
			else
				serialOkColor = PixRGB<byte>(20,200,128);
		}else if(code==15 || itsSerialTimeOutCounter > 0){
			if(code==15) itsSerialTimeOutCounter += 10; else itsSerialTimeOutCounter--;
			sprintf(buffer, "TimeOut %2d",code);
			serialOkColor = PixRGB<byte>(255,0,128);
		}else
		{
			sprintf(buffer, "Error!%2d",code);
			if((int(locSerialTime))%10 < 5)
				serialOkColor = PixRGB<byte>(255,0,0);
			else	
				serialOkColor = PixRGB<byte>(128,0,128);
		}
		writeText(itsControlImage, Point2D<int>(246,30), buffer, serialOkColor, PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));
		sprintf(buffer, "Serial %6.2fms",locSerialTime);
		writeText(itsControlImage, Point2D<int>(246,10), buffer, serialOkColor, PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

    // show RC status
		for(size_t i=0; i<itsChannelMeters.size(); i++)
		{
			sprintf(buffer, "CH%lu: %d", i+1, itsRCChannels[i]);

			Point2D<int> pastePoint = Point2D<int>(
					int(itsChannelMeters[0].getWidth()*itsChannelMeters.size()*1.05+20),
					itsControlImage.getHeight() - itsChannelMeters[i].getHeight() - 10 + i*8
					);

			writeText(itsControlImage, pastePoint, buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

			pastePoint = Point2D<int>(
					int(10+itsChannelMeters[i].getWidth()*i*1.05),
					itsControlImage.getHeight() - itsChannelMeters[i].getHeight() - 10
					);
			inplacePaste(itsControlImage, itsChannelMeters[i].render(itsRCChannels[i]), pastePoint );
		}

		sprintf(buffer, "Emergency Mode: %d", itsEmergencyMode);
		writeText(itsControlImage, Point2D<int>(256,88), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

		sprintf(buffer, "RemoteMode: %d", itsRemoteMode);
		writeText(itsControlImage, Point2D<int>(256,96), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

		sprintf(buffer, "Motor 1 Speed: %0.2f", itsMotor1Speed);
		writeText(itsControlImage, Point2D<int>(256,104), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

		sprintf(buffer, "Motor 2 Speed: %0.2f", itsMotor2Speed);
		writeText(itsControlImage, Point2D<int>(256,112), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

		sprintf(buffer, "RC Speed   : %0.2f -> %0.2f", itsRcTransSpeed,itsRcTransSpeedCapped);
		writeText(itsControlImage, Point2D<int>(256,120), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

		sprintf(buffer, "RC Rotation: %0.2f -> %0.2f", itsRcRotSpeed,itsRcRotSpeedCapped);
		writeText(itsControlImage, Point2D<int>(256,128), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));


    int battH = itsControlImage.getHeight()-itsBatteryMeter.getHeight() - 10;
		sprintf(buffer, "Battery:%4.2fV",itsBatteryVotage);
		writeText(itsControlImage, Point2D<int>(174,itsControlImage.getHeight()-10), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
		inplacePaste(itsControlImage, itsBatteryMeter.render(itsBatteryVotage*100,true), Point2D<int>(226,battH));

		//Draw the encoder-based trajectory map in red and green
		//float replayspeed = itsReplaySpeed.getVal();
		//int w = itsMapImage.getWidth();
		//int h = itsMapImage.getHeight();
		double scaleRate = 0.9;
		Point2D<int> drawPos(
				int(itsPosition.x*itsMapScale + itsMapCenter.i),
				int(itsPosition.y*itsMapScale +itsMapCenter.j)
				);
		if(itsMapImage.coordsOk(drawPos))
		{
			itsPositionMap.push_back(itsPosition);
		}else {
			//Find New Scale
			do{
				itsMapScale *= scaleRate;
				drawPos = Point2D<int>(
						int(itsPosition.x*itsMapScale + itsMapCenter.i),
						int(itsPosition.y*itsMapScale + itsMapCenter.j)
						);
			}while(!itsMapImage.coordsOk(drawPos) );
			reDrawMap(itsMapScale);																
		}
		if(itsMapImage.coordsOk(drawPos))
			itsMapImage.setVal(drawPos, itsPosition.color);


		itsDrawMapImage = Image<PixRGB<byte> >(itsMapImage);
		drawCircle(itsDrawMapImage, drawPos, 9, PixRGB<byte>(255,255,255));
		drawLine(itsDrawMapImage, drawPos,-itsPosition.z, 9, PixRGB<byte>(255,255,255));

		//draw IMU odometry			
		Point2D<int> drawPosIMU(
				int(itsIMUPosition.x*itsMapScale + itsMapCenter.i),
				int(itsIMUPosition.y*itsMapScale +itsMapCenter.j)
				);
		if(itsMapImage.coordsOk(drawPosIMU))
		{
			itsIMUPositionMap.push_back(itsIMUPosition);
		}else
		{
			//Find New Scale
			do{
				itsMapScale *= scaleRate;
				drawPosIMU = Point2D<int>(
						int(itsIMUPosition.x*itsMapScale + itsMapCenter.i),
						int(itsIMUPosition.y*itsMapScale + itsMapCenter.j)
						);
			}while(!itsMapImage.coordsOk(drawPosIMU) );
			reDrawMap(itsMapScale);																
		}
		if(itsMapImage.coordsOk(drawPosIMU))
		{
			itsMapImage.setVal(drawPosIMU, itsIMUPosition.color);
		}
		//itsDrawMapImage = Image<PixRGB<byte> >(itsMapImage);
		drawCircle(itsDrawMapImage, drawPosIMU, 9, PixRGB<byte>(255,0,255));
		drawLine(itsDrawMapImage, drawPosIMU,-itsIMUPosition.z, 9, PixRGB<byte>(255,0,255));

		inplacePaste(itsDispImage,itsControlImage,Point2D<int>(0,0));

    //LINFO("dispImage %d x %d    drawmapImage %d x %d",itsDispImage.getWidth(),itsDispImage.getHeight(),
    //        itsDrawMapImage.getWidth(),itsDrawMapImage.getHeight());

		inplacePaste(itsDispImage,itsDrawMapImage,Point2D<int>(0,256));

		std::vector<std::vector<float> > lines;
		lines.push_back(itsVelocityTransQue.getVector());
		lines.push_back(itsVelocityTransCmdQue.getVector());

		std::vector<PixRGB<byte> > linesColor;
		linesColor.push_back(PixRGB<byte>(255,0,0));
		linesColor.push_back(PixRGB<byte>(255,165,0));

		int plotPosW = 512-70;	
		int plotPosH = 0;	
		int plotWidth =358;
		int plotHeight=128;
		float plotRange=3.0f;
		Image<PixRGB<byte> > plotImage = multilinePlot(
				lines,
				plotWidth,plotHeight,
				-plotRange,plotRange,
				"Trans velocity","m/s","",
				linesColor,
				PixRGB<byte>(255,255,255),
				PixRGB<byte>(0,0,0)
				);
		inplacePaste(itsDispImage,plotImage,Point2D<int>(plotPosW,plotPosH));

		//plotPosW+=plotWidth;
		plotPosH+=plotHeight;
		std::vector<std::vector<float> > rotlines;
		rotlines.push_back(itsVelocityRotQue.getVector());
		rotlines.push_back(itsVelocityRotCmdQue.getVector());
		Image<PixRGB<byte> > rotWimage = multilinePlot(
				rotlines,
				plotWidth,plotHeight,
				-plotRange,plotRange,
				"Rot velocity(w)","rad/s","",
				linesColor,
				PixRGB<byte>(255,255,255),
				PixRGB<byte>(0,0,0)
				);

		inplacePaste(itsDispImage,rotWimage,Point2D<int>(plotPosW,plotPosH));

		plotPosH += plotHeight;
		std::vector<std::vector<float> > acclines;
		acclines.push_back(itsAccelTransQue.getVector());
		acclines.push_back(itsAccelRotQue.getVector());
		Image<PixRGB<byte> > accelImage = multilinePlot(
				acclines,
				plotWidth,plotHeight,
				-10.0f,10.0f,
				"Acceleration","m/ss","",
				linesColor,
				PixRGB<byte>(255,255,255),
				PixRGB<byte>(0,0,0)
				);
		inplacePaste(itsDispImage,accelImage,Point2D<int>(plotPosW,plotPosH));
		//Draw Divide Line
		drawLine(itsDispImage,Point2D<int>(0,255),Point2D<int>(itsDispImage.getWidth(),255),PixRGB<byte>(255,255,255));//HR
		drawLine(itsDispImage,Point2D<int>(512-70,0),Point2D<int>(512-70,255),PixRGB<byte>(255,255,255));//VT

		itsMeterXwin.drawImage(itsDispImage);

	}
}
// ######################################################################
void BeoPilot::reDrawMap(double scale)
{
	//Clean up the old images
	itsMapImage = Image<PixRGB<byte> >(MAPWINDOW_W,MAPWINDOW_H,ZEROS);

	//int w = itsMapImage.getWidth();
	//int h = itsMapImage.getHeight();

	//Redraw all points with new scale
	LINFO("itsPositionMap.size is %d",(int)itsPositionMap.size());
	for(int i = 0;i < (int)itsPositionMap.size();i++)
	{
		Point2D<int> tmp(
				int(itsPositionMap[i].x*scale + itsMapCenter.i),
				int(itsPositionMap[i].y*scale + itsMapCenter.j)
				);
		if(itsMapImage.coordsOk(tmp))
			itsMapImage.setVal(tmp, itsPositionMap[i].color);
	}
	for(int i = 0;i < (int)itsIMUPositionMap.size();i++)
	{
		Point2D<int> tmp(
				int(itsIMUPositionMap[i].x*scale + itsMapCenter.i),
				int(itsIMUPositionMap[i].y*scale + itsMapCenter.j)
				);
		itsMapImage.setVal(tmp, itsIMUPositionMap[i].color);
	}

}
// ######################################################################
void BeoPilot::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
		const Ice::Current&)
{
  if(eMsg->ice_isA("::BeobotEvents::ResetRequest"))
    {
			BeobotEvents::ResetRequestPtr resetMsg =
				BeobotEvents::ResetRequestPtr::dynamicCast(eMsg);
			LINFO("Got Reset Request %d",resetMsg->ResetID);

			//if message not sent from itself
			if(resetMsg->RequestAppID != BEO_PILOT)
			{
				if(resetMsg->ResetID == BEO_ALL||resetMsg->ResetID == BEO_PILOT)
				{
					reset();
				}
			}
		}
	else if(eMsg->ice_isA("::BeobotEvents::MotorRequest"))
	{
		itsSpeedMutex.lock();
		{
			BeobotEvents::MotorRequestPtr mrMsg = BeobotEvents::MotorRequestPtr::dynamicCast(eMsg);
			itsForwardSpeedReq    = mrMsg->transVel;
			itsRotationalSpeedReq = mrMsg->rotVel;
		}
		itsSpeedMutex.unlock();
		LINFO("MR: (%f, %f)",itsForwardSpeedReq,itsRotationalSpeedReq);
	}
	// IMU message
	else if(eMsg->ice_isA("::BeobotEvents::IMUMessage"))
	{
		// store the IMU data
		BeobotEvents::IMUMessagePtr imuMsg =
			BeobotEvents::IMUMessagePtr::dynamicCast(eMsg);

		int currRequestID = imuMsg->RequestID;
		LDEBUG("Got a IMUMessage with Request ID = %d", currRequestID);

		// get roll, pitch, and yaw
		if(imuMsg->validRollPitchYaw)
		{
			float roll  = imuMsg->roll;
			float pitch = imuMsg->pitch;
			float yaw   = imuMsg->yaw;

			std::string line =
				sformat("IMU Euler Angle  r:%15.6f p:%15.6f y:%15.6f",
						roll, pitch, yaw);
			line += std::string("\n");

			LDEBUG("%s",line.c_str());

			itsIMUMutex.lock();
			{
				itsIMUheading = yaw;//pi to -pi 3.14 ~ -3.14
				itsIMURoll = roll;
				itsIMUPitch = pitch;
			}
			itsIMUMutex.unlock();
		}
		// get magnetometer
		else if(imuMsg->validMagnetometer)
		{
			float magX  = imuMsg->magX;
			float magY  = imuMsg->magY;
			float magZ  = imuMsg->magZ;

			std::string line =
				sformat("IMU Magnetometer  X:%15.6f Y:%15.6f Z:%15.6f",
						magX, magY, magZ);
			LDEBUG("%s",line.c_str());

		}
		// get magnetometer
		else if(imuMsg->validAccAndAng)
		{
      float accelX   = imuMsg->accelX;
      float accelY   = imuMsg->accelY;
      float accelZ   = imuMsg->accelZ;

      float angRateX = imuMsg->angRateX;
      float angRateY = imuMsg->angRateY;
      float angRateZ = imuMsg->angRateZ;

			std::string line =
				sformat("IMU Acc X:%15.6f Y:%15.6f Z:%15.6f Ang X:%15.6f Y:%15.6f Z:%15.6f",
						accelX, accelY, accelZ, angRateX, angRateY, angRateZ);
			LDEBUG("%s",line.c_str());
		}
	}
}
// ######################################################################
Point3D<double> BeoPilot::tf(Point3D<double> p,double x,double y,double z,double roll,double pitch,double yaw)
{

  double a = pitch,b = yaw,c = roll;

  Image<double> R = Image<double>(3,3,ZEROS);
  R.setVal(0,0, cos(a)*cos(b)); R.setVal(1,0,cos(a)*sin(b)*c-sin(a)*cos(c));  R.setVal(2,0,cos(a)*sin(b)*cos(c)+sin(a)*sin(c));
  R.setVal(0,1, sin(a)*cos(b)); R.setVal(1,1,sin(a)*sin(b)*c+cos(a)*cos(c));  R.setVal(2,1,sin(a)*sin(b)*cos(c)-cos(a)*sin(c));
  R.setVal(0,2,-sin(b));        R.setVal(1,2,cos(b)*sin(c));                  R.setVal(2,2,cos(b)*cos(c));

	 Image<double> pt  = Image<double>(1,3,ZEROS);
  pt.setVal(0,0,x);
  pt.setVal(0,1,y);
  pt.setVal(0,2,z);

  //Image<double> pt = pt2matrix(p);//convert Point3D to Matrix format
  Image<double> ptr = matrixMult(R,pt);
  Point3D<double> T = Point3D<double>(x,y,z);
	  Point3D<double> result = Point3D<double>(
      ptr.getVal(0,0),
      ptr.getVal(0,1),
      ptr.getVal(0,2))+T;

  return result;
}

// ######################################################################
Point2D<double> BeoPilot::rotZ(double x,double y,double ori)
{
	Point3D<double> result = tf(Point3D<double>(x,y,0),0,0,0,0.0,ori,0.0);
	LINFO("%f,%f rot %f become +> %f,%f",x,y,ori,result.x,result.y);

	return Point2D<double>(result.x,result.y);
}


