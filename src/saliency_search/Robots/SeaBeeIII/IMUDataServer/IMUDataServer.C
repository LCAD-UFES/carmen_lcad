#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>

#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"

#include "IMUDataServer.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include <iostream>

#ifndef IMUDATASERVER_C
#define IMUDATASERVER_C

using namespace xsens;

const ModelOptionCateg MOC_SeaBeeIIIIMUDataServer = {
    MOC_SORTPRI_3, "Options" };

const ModelOptionDef OPT_EnableConfig =
{ MODOPT_ARG(int), "EnableConfig", &MOC_SeaBeeIIIIMUDataServer, OPTEXP_CORE,
  "Enable configuration of the IMU",
   "enable-config", '\0', "<float>", "0" };

// ######################################################################
IMUDataServer::IMUDataServer(int id, OptionManager& mgr,
		const std::string& descrName, const std::string& tagName) :
	RobotBrainComponent(mgr, descrName, tagName),
	mEnableConfig(&OPT_EnableConfig, this, 0)
	//itsPort(new Serial(mgr))
{
	//itsPort->configure("/dev/ttyUSB2", 57600, "8N2", false, false, 1);
	//addSubComponent(itsPort);
	res = XRV_OK;
	inited = false;
}

// ######################################################################
IMUDataServer::~IMUDataServer()
{
	delete packet;
	//cmt3.closePort();
}

void IMUDataServer::stop2()
{
	cout << "closing port: ";
	LFATAL("Testing to see if this fixes the port issues");
	return;
	/*cmt3.closePort();
	if(cmt3.isPortOpen())
	{
		cout << "failed" << endl;
	}
	else
	{
		cout << "success" << endl;
	}*/
}

bool IMUDataServer::initMe()
{
	unsigned long mtCount = doHardwareScan(cmt3, deviceIds);

	if (mtCount == 0)
	{
		std::cout << "no MTs, quitting." << std::endl;
		cmt3.closePort();
		inited = true;
		return false;
	}

	if(mEnableConfig.getVal() == 1)
	{
		getUserInputs(mode, settings);
	}
	else
	{
		mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
	}

	doMtSettings(cmt3, mode, settings, deviceIds);

	// Initialize packet for data
	packet = new Packet((unsigned short) mtCount, cmt3.isXm());
	msg = new RobotSimEvents::IMUDataServerMessage();

	inited = true;
	return true;
}

// ######################################################################
void IMUDataServer::registerTopics()
{
	registerPublisher("IMUDataServerMessageTopic");
	//registerSubscription("IMUDataServerMessageTopic");
}

// ######################################################################
void IMUDataServer::evolve()
{
	static int frames = 0;

	if (!inited)
	{
		cout << "initing..." << endl;
		if(!initMe())
		{
			LFATAL("No motion trackers found; can't continue");
			//cout << "No motion trackers found; can't continue" << endl;
			return;
		}
	}

	if (res != XRV_OK)
	{
		return;
	}

	frames ++;

	cmt3.waitForDataMessage(packet);

	if(frames > 6) //drop frames
	{
		frames = 0;
		//LFATAL("Temporary call to quit");
	}
	else
	{
		return;
	}

	msg->angleMode = -1;

	// Output Temperature
	if ((mode & CMT_OUTPUTMODE_TEMP) != 0)
	{
		tdata = packet->getTemp(0);

		msg->temp = tdata;
	}

	if ((mode & CMT_OUTPUTMODE_CALIB) != 0)
	{
		caldata = packet->getCalData(0);

		msg->accel.x = caldata.m_acc.m_data[0];
		msg->accel.y = caldata.m_acc.m_data[1];
		msg->accel.z = caldata.m_acc.m_data[2];

		msg->gyro.x = caldata.m_gyr.m_data[0];
		msg->gyro.y = caldata.m_gyr.m_data[1];
		msg->gyro.z = caldata.m_gyr.m_data[2];

		msg->mag.x = caldata.m_mag.m_data[0];
		msg->mag.y = caldata.m_mag.m_data[1];
		msg->mag.z = caldata.m_mag.m_data[2];

	}

	if ((mode & CMT_OUTPUTMODE_ORIENT) != 0)
	{

		switch (settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK)
		{
		case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
			// Output: quaternion
			msg->angleMode = Mode::quat;
			qat_data = packet->getOriQuat(0);

			msg->orientation.resize(4);
			msg->orientation[0] = qat_data.m_data[0];
			msg->orientation[1] = qat_data.m_data[1];
			msg->orientation[2] = qat_data.m_data[2];
			msg->orientation[3] = qat_data.m_data[3];

			break;

		case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
			// Output: Euler
			msg->angleMode = Mode::euler;
			euler_data = packet->getOriEuler(0);

			msg->orientation.resize(3);
			msg->orientation[0] = euler_data.m_roll;
			msg->orientation[1] = euler_data.m_pitch;
			msg->orientation[2] = euler_data.m_yaw;

			break;

		case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
			// Output: Cosine Matrix
			msg->angleMode = Mode::cos_mat;
			matrix_data = packet->getOriMatrix(0);

			msg->orientation.resize(9);
			msg->orientation[0] = matrix_data.m_data[0][0];
			msg->orientation[1] = matrix_data.m_data[0][1];
			msg->orientation[2] = matrix_data.m_data[0][2];
			msg->orientation[3] = matrix_data.m_data[1][0];
			msg->orientation[4] = matrix_data.m_data[1][1];
			msg->orientation[5] = matrix_data.m_data[1][2];
			msg->orientation[6] = matrix_data.m_data[2][0];
			msg->orientation[7] = matrix_data.m_data[2][1];
			msg->orientation[8] = matrix_data.m_data[2][2];

			break;
		}

		if ((mode & CMT_OUTPUTMODE_POSITION) != 0)
		{

			if (packet->containsPositionLLA())
			{
				/* output position */
/*				CmtVector positionLLA = packet->getPositionLLA();
				if (res != XRV_OK)
				{

				}

				for (int i = 0; i < 2; i++)
				{
					double deg = positionLLA.m_data[0];
					double min = (deg - (int) deg) * 60;
					double sec = (min - (int) min) * 60;

				}*/

			}
			else
			{

			}
		}
	}

	publish("IMUDataServerMessageTopic", msg);
}

// ######################################################################
void IMUDataServer::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
		const Ice::Current&)
{
	if (eMsg->ice_isA("::RobotSimEvents::IMUDataServerMessage") && false)
	{
		RobotSimEvents::IMUDataServerMessagePtr msg = RobotSimEvents::IMUDataServerMessagePtr::dynamicCast(eMsg);
		cout << "temperature" << endl;
		cout << msg->temp << endl;
		cout << "acceleration" << endl;
		cout << msg->accel.x << "," << msg->accel.y << "," << msg->accel.z << endl;
		cout << "gyroscope" << endl;
		cout << msg->gyro.x << "," << msg->gyro.y << "," << msg->gyro.z << endl;
		cout << "magnetometer" << endl;
		cout << msg->mag.x << "," << msg->mag.y << "," << msg->mag.z << endl;
		switch(msg->angleMode)
		{
		case 0:
			cout << "quaternian" << endl;
			cout << msg->orientation[0] << "," << msg->orientation[1] << "," << msg->orientation[2] << "," << msg->orientation[3] << endl;
			break;
		case 1:
			cout << "euler" << endl;
			cout << msg->orientation[0] << "," << msg->orientation[1] << "," << msg->orientation[2] << endl;
			break;
		case 2:
			cout << "cosine matrix" << endl;
			cout << msg->orientation[0] << "," << msg->orientation[1] << "," << msg->orientation[2] << endl;
			cout << msg->orientation[3] << "," << msg->orientation[4] << "," << msg->orientation[5] << endl;
			cout << msg->orientation[6] << "," << msg->orientation[7] << "," << msg->orientation[8] << endl;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
int IMUDataServer::doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
{
	XsensResultValue res;
	//List<CmtPortInfo> portInfo;// = {0,0,0,""};
	CmtPortInfo port = {0,0,0,"/dev/ttyUSB0"};

	unsigned long portCount = 0;
	int mtCount;

	std::cout << "Scanning for connected Xsens devices..." << std::endl;
	//xsens::cmtScanPorts(portInfo);
	//portCount = portInfo.length();
	portCount = (xsens::cmtScanForIMU(port));
	std::cout << "done" << std::endl;

	if (portCount == 0)
	{
		std::cout << "No MotionTrackers found" << std::endl;
		return 0;
	}

	for (int i = 0; i < (int) portCount; i++)
	{
		//std::cout << portInfo[i].m_baudrate << "|" << portInfo[i].m_deviceId << "|" << portInfo[i].m_portName << "|" << portInfo[i].m_portNr << std::endl;
		std::cout << "Using COM port at " << port.m_portName;

		switch (port.m_baudrate)
		{
		case B9600:
			std::cout << "9k6";
			break;
		case B19200:
			std::cout << "19k2";
			break;
		case B38400:
			std::cout << "38k4";
			break;
		case B57600:
			std::cout << "57k6";
			break;
		case B115200:
			std::cout << "115k2";
			break;
		case B230400:
			std::cout << "230k4";
			break;
		case B460800:
			std::cout << "460k8";
			break;
		case B921600:
			std::cout << "921k6";
			break;
		default:
			std::cout << port.m_baudrate;
		}
		std::cout << "baud" << std::endl;
	}

	std::cout << "Opening ports...";
	//open the port which the device is connected to and connect at the device's baudrate.
	//for (int p = 0; p < (int) portCount; p++)
	//{
		res = cmt3.openPort(port.m_portName, port.m_baudrate);
		//                EXIT_ON_ERROR(res,"cmtOpenPort");
	//}
	std::cout << "done" << std::endl;

	//get the Mt sensor count.
	std::cout
			<< "Retrieving MotionTracker count (excluding attached Xbus Master(s))"
			<< std::endl;
	//mtCount = cmt3.getMtCount();
	mtCount = portCount;
	std::cout << "MotionTracker count: " << mtCount << std::endl;

	// retrieve the device IDs
	std::cout << "Retrieving MotionTrackers device ID(s)" << std::endl;
	/*for (int j = 0; j < mtCount; j++)
	{
		res = cmt3.getDeviceId((unsigned char) (j + 1), deviceIds[j]);
		//                EXIT_ON_ERROR(res,"getDeviceId");
		std::cout << "Device ID at busId " << j + 1 << ","
				<< (long) deviceIds[j] << std::endl;
	}*/

	deviceIds[0] = port.m_deviceId;

	return mtCount;
	//return 0;
}

//////////////////////////////////////////////////////////////////////////
// getUserInputs
//
// Request user for output data
void IMUDataServer::getUserInputs(CmtOutputMode &mode,
		CmtOutputSettings &settings)
{
	mode = 0;

	std::cout << "Select desired output:" << std::endl;
	std::cout << "1 - Calibrated data" << std::endl;
	std::cout << "2 - Orientation data and GPS Position (MTi-G only)"
			<< std::endl;
	std::cout << "3 - Both Calibrated and Orientation data" << std::endl;
	std::cout << "4 - Temperature and Calibrated data" << std::endl;
	std::cout << "5 - Temperature and Orientation data" << std::endl;
	std::cout << "6 - Temperature, Calibrated and Orientation data"
			<< std::endl;
	std::cout << "Enter your choice: ";

	std::cin >> mode;

	//nope
	/*if (mode < 1 || mode > 6) {
	 std::cout << "\n\nPlease enter a valid output mode\n");
	 }*/

	switch (mode)
	{
	case 1:
		mode = CMT_OUTPUTMODE_CALIB;
		break;
	case 2:
		mode = CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_POSITION;
		break;
	case 3:
		mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		break;
	case 4:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB;
		break;
	case 5:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_ORIENT;
		break;
	case 6:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB
				| CMT_OUTPUTMODE_ORIENT;
		break;
	}

	if ((mode & CMT_OUTPUTMODE_ORIENT) != 0)
	{
		settings = 0;

		std::cout << std::endl;
		std::cout << "Select desired output format" << std::endl;
		std::cout << "1 - Quaternions" << std::endl;
		std::cout << "2 - Euler angles" << std::endl;
		std::cout << "3 - Matrix" << std::endl;
		std::cout << "Enter your choice: ";

		std::cin >> settings;

		//nope
		/*if (settings < 1  || settings > 3) {
		 std::cout << "\n\nPlease enter a valid output format\n");
		 }*/

		// Update outputSettings to match data specs of SetOutputSettings
		switch (settings)
		{
		case 1:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
			break;
		case 2:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
			break;
		case 3:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX;
			break;
		}
	}
	else
	{
		settings = 0;
	}
	settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
void IMUDataServer::doMtSettings(xsens::Cmt3 &cmt3, CmtOutputMode &mode,
		CmtOutputSettings &settings, CmtDeviceId deviceIds[])
{
	XsensResultValue res;
	unsigned long mtCount = cmt3.getMtCount();

	// set sensor to config sate
	res = cmt3.gotoConfig();
	//        EXIT_ON_ERROR(res,"gotoConfig");

	unsigned short sampleFreq;
	sampleFreq = cmt3.getSampleFrequency();

	// set the device output mode for the device(s)
	std::cout << "Configuring your mode selection" << std::endl;

	for (unsigned int i = 0; i < mtCount; i++)
	{
		CmtDeviceMode deviceMode(mode, settings, sampleFreq);
		if ((deviceIds[i] & 0xFFF00000) != 0x00500000)
		{
			// not an MTi-G, remove all GPS related stuff
			deviceMode.m_outputMode &= 0xFF0F;
		}
		res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
		//                EXIT_ON_ERROR(res,"setDeviceMode");
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	//        EXIT_ON_ERROR(res,"gotoMeasurement");
}
#endif
