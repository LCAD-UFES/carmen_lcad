 /*********************************************************
	---   Skeleton Module Application ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/xsens_mtig_interface.h>
#include <carmen/xsens_messages.h>
#include <carmen/xsens_interface.h>

#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"
#include "example_linux.h"

using namespace xsens;

static xsens::Cmt3 cmt3;
static unsigned long mtCount = 0;
static CmtDeviceId deviceIds[256];
static CmtOutputMode mode;
static CmtOutputSettings settings;

static int xsens_scenario;
static CmtVector gps_position;
static char *xsens_dev;
static int xsens_type = 0;
// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printf("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

void 
shutdown_module(int signo)
{
  if(signo == SIGINT)
  {
     carmen_ipc_disconnect();
     printf("xsens_mtig: disconnected.\n");
     exit(0);
  }
}


static int 
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
		{(char *) "xsens", (char *) "scenario", CARMEN_PARAM_INT, &xsens_scenario, 0, NULL},
		{(char *) "xsens", (char *) "gps_x", CARMEN_PARAM_DOUBLE, &(gps_position.m_data[0]), 0, NULL},
		{(char *) "xsens", (char *) "gps_y", CARMEN_PARAM_DOUBLE, &(gps_position.m_data[1]), 0, NULL},
		{(char *) "xsens", (char *) "gps_z", CARMEN_PARAM_DOUBLE, &(gps_position.m_data[2]), 0, NULL},
		{(char *) "xsens", (char *) "dev",   CARMEN_PARAM_STRING, &(xsens_dev), 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
void 
doMtSettings(xsens::Cmt3 &cmt3, CmtOutputMode &mode,
		     CmtOutputSettings &settings, CmtDeviceId deviceIds[], bool reset_orientation)
{
	XsensResultValue res;
	unsigned long mtCount = cmt3.getMtCount();

	// set sensor to config sate
	res = cmt3.gotoConfig();
	EXIT_ON_ERROR(res,"gotoConfig");

	cmt3.setBaudrate(CMT_BAUD_RATE_460K8);


	// set the device output mode for the device(s)
	printf("Configuring your mode selection\n");

	for (unsigned int i = 0; i < mtCount; i++)
	{
		CmtScenario scenarios[CMT_MAX_SCENARIOS_IN_MT];
		cmt3.getAvailableScenarios(scenarios, deviceIds[i]);

		bool scenario_available = false;
		printf("\nAvailable scenarios:\n");
		for (int j = 0; j < CMT_MAX_SCENARIOS_IN_MT; j++)
		{
			printf("Scenario %d: %.20s\n", scenarios[j].m_type, scenarios[j].m_label);

			if (scenarios[j].m_type == xsens_scenario)
				scenario_available = true;
		}

		if (scenario_available)
		{
			printf("\nSetting scenario %d\n", xsens_scenario);
			cmt3.setScenario(xsens_scenario, deviceIds[i]);
		}
		else
			printf("\nError, scenario %d not available\n", xsens_scenario);

		uint8_t scenarioType = 0;
		uint8_t scenarioVersion = 0;
		cmt3.getScenario(scenarioType, scenarioVersion, deviceIds[i]);

		printf("\nScenario in use is %d, version %d\n", scenarioType, scenarioVersion);

		cmt3.setGpsLeverArm(gps_position, deviceIds[i]);

		unsigned short sampleFreq;
		sampleFreq = cmt3.getSampleFrequency();

		CmtDeviceMode deviceMode(mode, settings, sampleFreq);

		if ((deviceIds[i] & 0xFFF00000) != 0x01500000) 		// This was modified!!! Original was 0x00500000 but our sensor gives this ID
		{
			// not an MTi-G, remove all GPS related stuff
			xsens_type = 0;
			deviceMode.m_outputMode &= 0xFF0F;
			printf("ID %x: not an MTi-G, removing GPS related stuff\n", deviceIds[i]);
		}
		else
		{
			printf("ID %x: is an MTi-G\n", deviceIds[i]);
			xsens_type = 1;
		}
		res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
		EXIT_ON_ERROR(res,"setDeviceMode");
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	EXIT_ON_ERROR(res,"gotoMeasurement");

	if (reset_orientation)
	{
		cmt3.resetOrientation(CMT_RESETORIENTATION_OBJECT, deviceIds[0]);

		res = cmt3.gotoConfig();
		EXIT_ON_ERROR(res,"gotoConfig");
		cmt3.resetOrientation(CMT_RESETORIENTATION_STORE, deviceIds[0]);

		res = cmt3.gotoMeasurement();
		EXIT_ON_ERROR(res,"gotoMeasurement");

		printf("\nReseting the orientation\n");
	}
	printf("\nStarting to receive data\n");
}


int
doHardwareScan_old(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	int mtCount;
	
	printf("Opening ports...");

	res = cmt3.openPort(xsens_dev, B460800);
	EXIT_ON_ERROR(res,"cmtOpenPort");

	if(res == XRV_OK)
		mtCount = 1;

	printf("done\n\n");

	 //set the measurement timeout to 100ms (default is 16ms)
	int timeOut = 100;
	res = cmt3.setTimeoutMeasurement(timeOut);
	EXIT_ON_ERROR(res, "set measurment timeout");
	printf("Measurement timeout set to %d ms\n", timeOut);


	// retrieve the device IDs 
	printf("Retrieving MotionTrackers device ID(s)\n");
	for(int j = 0; j < mtCount; j++){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		printf("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
	}
	
	return mtCount;
}


int
doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	unsigned long portCount = 0;
	int mtCount;

	printf("Scanning for connected Xsens devices...");
	xsens::cmtScanPorts(portInfo);
	portCount = portInfo.length();
	printf("done\n");

	if (portCount == 0) {
		printf("No MotionTrackers found\n\n");
		return 0;
	}

	for(int i = 0; i < (int)portCount; i++) {
		printf("Using COM port %s at ", portInfo[i].m_portName);

		switch (portInfo[i].m_baudrate) {
		case B9600  : printf("9k6");   break;
		case B19200 : printf("19k2");  break;
		case B38400 : printf("38k4");  break;
		case B57600 : printf("57k6");  break;
		case B115200: printf("115k2"); break;
		case B230400: printf("230k4"); break;
		case B460800: printf("460k8"); break;
		case B921600: printf("921k6"); break;
		default: printf("0x%x", portInfo[i].m_baudrate);
		}
		printf(" baud\n\n");
	}

	printf("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for(int p = 0; p < (int)portCount; p++){
		res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate);
		EXIT_ON_ERROR(res,"cmtOpenPort");
	}
	printf("done\n\n");

	 //set the measurement timeout to 100ms (default is 16ms)
	int timeOut = 100;
	res = cmt3.setTimeoutMeasurement(timeOut);
	EXIT_ON_ERROR(res, "set measurment timeout");
	printf("Measurement timeout set to %d ms\n", timeOut);

	//get the Mt sensor count.
	printf("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	mtCount = cmt3.getMtCount();
	printf("MotionTracker count: %d\n\n", mtCount);

	// retrieve the device IDs
	printf("Retrieving MotionTrackers device ID(s)\n");
	for(int j = 0; j < mtCount; j++){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		printf("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
	}

	return mtCount;
}


void 
publish_mtig_message(carmen_xsens_mtig_message message)
{
   IPC_RETURN_TYPE err;

   err = IPC_publishData(CARMEN_XSENS_MTIG_NAME, &message);
   carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_MTIG_FMT);
}


void
publish_mti_quat_message(carmen_xsens_global_quat_message message)
{
   IPC_RETURN_TYPE err;
      
   err = IPC_publishData(CARMEN_XSENS_GLOBAL_QUAT_NAME, &message);
   carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_QUAT_FMT);
}


carmen_xsens_mtig_raw_gps_message 
make_xsens_mtig_raw_gps_message(int sensor_ID, CmtQuat qat_data, CmtCalData caldata, CmtGpsPvtData gpsPvtData, uint8_t status)
{
	carmen_xsens_mtig_raw_gps_message message;

	message.quat.q0 = qat_data.m_data[0];
	message.quat.q1 = qat_data.m_data[1];
	message.quat.q2 = qat_data.m_data[2];
	message.quat.q3 = qat_data.m_data[3];

	message.acc.x = caldata.m_acc.m_data[0];
	message.acc.y = caldata.m_acc.m_data[1];
	message.acc.z = caldata.m_acc.m_data[2];

	message.gyr.x = caldata.m_gyr.m_data[0];
	message.gyr.y = caldata.m_gyr.m_data[1];
	message.gyr.z = caldata.m_gyr.m_data[2];

	message.mag.x = caldata.m_mag.m_data[0];
	message.mag.y = caldata.m_mag.m_data[1];
	message.mag.z = caldata.m_mag.m_data[2];

	message.latitude = gpsPvtData.m_latitude;
	message.longitude = gpsPvtData.m_longitude;
	message.height = gpsPvtData.m_height;
	message.vel_north = gpsPvtData.m_veln;
	message.vel_east = gpsPvtData.m_vele;
	message.vel_down = gpsPvtData.m_veld;
	message.horizontal_accuracy = gpsPvtData.m_hacc;
	message.vertical_accuracy = gpsPvtData.m_vacc;
	message.speed_accuracy = gpsPvtData.m_sacc;

	message.gps_fix = status & 0x04;
	message.xkf_valid = status & 0x02;

	message.sensor_ID = sensor_ID;
	
	message.timestamp = carmen_get_time();
	message.host = carmen_get_host();

	return message;
}


carmen_xsens_mtig_message 
make_xsens_mtig_message(int sensor_ID, CmtQuat qat_data, CmtCalData caldata, CmtVector positionLLA, CmtVector velocity, uint8_t status)
{
	carmen_xsens_mtig_message message;

	message.quat.q0 = qat_data.m_data[0];
	message.quat.q1 = qat_data.m_data[1];
	message.quat.q2 = qat_data.m_data[2];
	message.quat.q3 = qat_data.m_data[3];

	message.acc.x = caldata.m_acc.m_data[0];
	message.acc.y = caldata.m_acc.m_data[1];
	message.acc.z = caldata.m_acc.m_data[2];

	message.gyr.x = caldata.m_gyr.m_data[0];
	message.gyr.y = caldata.m_gyr.m_data[1];
	message.gyr.z = caldata.m_gyr.m_data[2];

	message.mag.x = caldata.m_mag.m_data[0];
	message.mag.y = caldata.m_mag.m_data[1];
	message.mag.z = caldata.m_mag.m_data[2];

	message.velocity.x = velocity.m_data[0];
	message.velocity.y = velocity.m_data[1];
	message.velocity.z = velocity.m_data[2];

	message.latitude = positionLLA.m_data[0];
	message.longitude = positionLLA.m_data[1];
	message.height = positionLLA.m_data[2];
	

	message.gps_fix = status & 0x04;
	message.xkf_valid = status & 0x02;

	message.sensor_ID = sensor_ID;
	
	message.timestamp = carmen_get_time();
	message.host = carmen_get_host();

	return message;
}


carmen_xsens_global_quat_message
make_xsens_mti_quat_message(CmtQuat qat_data, CmtCalData caldata, double tdata)
{
	carmen_xsens_global_quat_message xsens_quat_message;

	//Acceleration
	xsens_quat_message.m_acc.x = caldata.m_acc.m_data[0];
	xsens_quat_message.m_acc.y = caldata.m_acc.m_data[1];
	xsens_quat_message.m_acc.z = caldata.m_acc.m_data[2];

	//Gyro
	xsens_quat_message.m_gyr.x = caldata.m_gyr.m_data[0];
	xsens_quat_message.m_gyr.y = caldata.m_gyr.m_data[1];
	xsens_quat_message.m_gyr.z = caldata.m_gyr.m_data[2];

	//Magnetism
	xsens_quat_message.m_mag.x = caldata.m_mag.m_data[0];
	xsens_quat_message.m_mag.y = caldata.m_mag.m_data[1];
	xsens_quat_message.m_mag.z = caldata.m_mag.m_data[2];

	xsens_quat_message.quat_data.m_data[0] = qat_data.m_data[0];
	xsens_quat_message.quat_data.m_data[1] = qat_data.m_data[1];
	xsens_quat_message.quat_data.m_data[2] = qat_data.m_data[2];
	xsens_quat_message.quat_data.m_data[3] = qat_data.m_data[3];

	//Temperature and Message Count
	xsens_quat_message.m_temp = tdata;
	xsens_quat_message.m_count = 0.0;

	//Timestamp
	xsens_quat_message.timestamp = carmen_get_time();

	//Host
	xsens_quat_message.host = carmen_get_host();

	return xsens_quat_message;
}


int
read_data_from_xsens(void)
{	
	double tdata = 0.0;

	//structs to hold data.
	CmtCalData caldata;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;
	//CmtGpsPvtData gpsPvtData;
	CmtVector positionLLA;
	CmtVector velocity;
	uint8_t status = 0;

	// Initialize packet for data
	Packet *packet = new Packet((unsigned short) mtCount, cmt3.isXm());

	while (1) 
	{
		XsensResultValue result = cmt3.waitForDataMessage(packet);
		if (result != XRV_OK)
		{
			printf("NOK!\n");

			if ((result == XRV_TIMEOUTNODATA) || (result == XRV_TIMEOUT))
				continue;  //Ignore the error and restart the while loop			

			delete packet;
			cmt3.closePort();
			printf("\nError %d occured in waitForDataMessage, can not recover.\n", result);
			return 0;
		}

		for (unsigned int i = 0; i < mtCount; i++) 
		{	
			if ((mode & CMT_OUTPUTMODE_TEMP) != 0)
				tdata = packet->getTemp(i);

			if ((mode & CMT_OUTPUTMODE_CALIB) != 0) 
				caldata = packet->getCalData(i);				

			if ((mode & CMT_OUTPUTMODE_ORIENT) == 0) 
				continue;

			switch (settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) 
			{
				case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
					// Output: quaternion
					qat_data = packet->getOriQuat(i);					
					break;

				case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
					// Output: Euler
					euler_data = packet->getOriEuler(i);					
					break;

				case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
					// Output: Cosine Matrix
					matrix_data = packet->getOriMatrix(i);					
					break;
				default:
					break;
			}

			
			if ((mode & CMT_OUTPUTMODE_POSITION) != 0) 
				positionLLA = packet->getPositionLLA();

			
			if ((mode & CMT_OUTPUTMODE_VELOCITY) != 0) 
				velocity = packet->getVelocity();
			

			//if((mode & CMT_OUTPUTMODE_GPSPVT_PRESSURE) != 0)
			//{
			//	gpsPvtData = packet->getGpsPvtData();

				//printf("LLA: % d % d % d\n", gpsPvtData.m_latitude, gpsPvtData.m_longitude, gpsPvtData.m_height);
			//}


			if ((mode & CMT_OUTPUTMODE_STATUS) != 0)
				status = packet->getStatus();

			switch(xsens_type)
			{
				case 0:
				{
					carmen_xsens_global_quat_message message = make_xsens_mti_quat_message(qat_data, caldata, tdata);
					publish_mti_quat_message(message);
					break;
				}
				case 1:
				{
					carmen_xsens_mtig_message xsens_message = make_xsens_mtig_message(deviceIds[i], qat_data, caldata, positionLLA, velocity, status);
					publish_mtig_message(xsens_message);

					carmen_xsens_global_quat_message message = make_xsens_mti_quat_message(qat_data, caldata, tdata);
					publish_mti_quat_message(message);
					break;
				}
			}
		}
	}
}


int
read_data_from_xsens_without_xsens(void)
{
	while (1)
	{
		carmen_xsens_global_quat_message message = {-0.076347, 0.022741, 9.856972,
													 0.150156, -0.005627, -0.002592,
													 0.988643, -0.278449, -0.101014,
													 0.281111, 0.022601, 0.002073, 0.006112,
													 60.500000, 0,
													 carmen_get_time(),
													 carmen_get_host()};
		publish_mti_quat_message(message);
		carmen_ipc_sleep(0.01);
	}
}


int
init_xsens(bool reset_orientation)
{
	mtCount = doHardwareScan(cmt3, deviceIds);

	if (mtCount == 0)
	{
		printf("xsens not found\n");
		cmt3.closePort();
		return 0;
	}

	mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_STATUS | CMT_OUTPUTMODE_POSITION | CMT_OUTPUTMODE_VELOCITY | CMT_OUTPUTMODE_TEMP;//CMT_OUTPUTMODE_GPSPVT_PRESSURE;
	settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT | CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632;

	doMtSettings(cmt3, mode, settings, deviceIds, reset_orientation);

	return 1;
}


int
init_xsens_new(bool reset_orientation)
{
	return (1);

	mtCount = doHardwareScan(cmt3, deviceIds);

	if (mtCount == 0) 
	{
		printf("xsens not found\n");
		cmt3.closePort();
		return 0;
	}

	mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_STATUS | CMT_OUTPUTMODE_POSITION | CMT_OUTPUTMODE_VELOCITY | CMT_OUTPUTMODE_TEMP;//CMT_OUTPUTMODE_GPSPVT_PRESSURE;
	settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT | CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632;

	doMtSettings(cmt3, mode, settings, deviceIds, reset_orientation);

	return 1;
}


static void 
define_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_XSENS_MTIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_NAME);

    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}


int 
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);
	define_ipc_messages();

	signal(SIGINT, shutdown_module);

	bool reset_orientation = false;
	if ((argc == 2) && (strcmp(argv[1], (char *) "reset_orientation") == 0))
		reset_orientation = true;

	int xsens_initialized = init_xsens(reset_orientation);

	read_data_from_xsens_without_xsens();
	if (xsens_initialized)
		read_data_from_xsens();

	carmen_ipc_disconnect();

	return (0);
}

