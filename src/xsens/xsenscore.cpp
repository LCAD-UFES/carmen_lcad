#include <stdio.h>    
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <curses.h>
#include <carmen/carmen.h>

#include "xsenscore.h"

// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printf("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

using namespace xsens;

//xsens::Cmt3 cmt3;

inline int isnum(int c)
{
  return (c >= '0' && c <= '9');
}                              

//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
/*int doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
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
    printf(" - no MotionTrackers found -\n\n");
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
      default: printf("0x%lx", (unsigned long)portInfo[i].m_baudrate);
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
}*/

int doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[], char* xsens_dev)
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	//unsigned long portCount = 0;
	int mtCount;
	
	printf("Scanning for connected Xsens devices...");
	//xsens::cmtScanPorts(portInfo);
	//portCount = portInfo.length();
	printf("done\n");

	//if (portCount == 0) {
	//	printf("No MotionTrackers found\n\n");
	//	return 0;
	//}

	//for(int i = 0; i < (int)portCount; i++) {	
	//	printf("Using COM port %s at ", portInfo[i].m_portName);
		
	//	switch (portInfo[i].m_baudrate) {
	//	case B9600  : printf("9k6");   break;
	//	case B19200 : printf("19k2");  break;
	//	case B38400 : printf("38k4");  break;
	//	case B57600 : printf("57k6");  break;
	//	case B115200: printf("115k2"); break;
	//	case B230400: printf("230k4"); break;
	//	case B460800: printf("460k8"); break;
	//	case B921600: printf("921k6"); break;
	//	default: printf("0x%lx", portInfo[i].m_baudrate);
	//	}
	//	printf(" baud\n\n");
	//}

	printf("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	//for(int p = 0; p < (int)portCount; p++){
		//res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate);
		res = cmt3.openPort(xsens_dev, B230400);
		EXIT_ON_ERROR(res,"doHardwareScan() call to cmt3.OpenPort (is the baudrate correct?)");  

		if(res == XRV_OK)
			mtCount = 1;
	//}
	printf("done\n\n");

	 //set the measurement timeout to 100ms (default is 16ms)
	int timeOut = 100;
	res = cmt3.setTimeoutMeasurement(timeOut);
	EXIT_ON_ERROR(res, "set measurment timeout");
	printf("Measurement timeout set to %d ms\n", timeOut);

	//get the Mt sensor count.
	//printf("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	//mtCount = cmt3.getMtCount();
	//printf("MotionTracker count: %d\n\n", mtCount);

	// retrieve the device IDs 
	printf("Retrieving MotionTrackers device ID(s)\n");
	for(int j = 0; j < mtCount; j++){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		printf("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
	}
	
	return mtCount;
}

//////////////////////////////////////////////////////////////////////////
// doHardwareScan_Manual
//
// Checa a porta selecionada e scaneia um MotionTracker
/*int doHardwareScan_Manual(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[], char* portName)
{
  XsensResultValue res;
  List<CmtPortInfo> portInfo;
  unsigned long portCount = 0;
  int mtCount;
  
  printf("Scanning for connected Xsens devices...");
  xsens::cmtScanPorts_Manual(portInfo, 0, 1000, 1, portName);
  portCount = portInfo.length();
  
  if (portCount == 0) {
    printf(" - no MotionTrackers found -\n\n");
    return 0;
  }else{
    printf("done\n");
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
      default: printf("0x%lx", (unsigned long)portInfo[i].m_baudrate);
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
}*/


//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
void doMtSettings(xsens::Cmt3 &cmt3, CmtOutputMode &mode, CmtOutputSettings &settings, CmtDeviceId deviceIds[]) 
{
  XsensResultValue res;
  unsigned long mtCount = cmt3.getMtCount();
  
  // set sensor to config sate
  res = cmt3.gotoConfig();
  EXIT_ON_ERROR(res,"gotoConfig");
  
  unsigned short sampleFreq;
  sampleFreq = cmt3.getSampleFrequency();
  
  // set the device output mode for the device(s)
  printf("Configuring your mode selection\n");
  
  for (unsigned int i = 0; i < mtCount; i++) 
  {
    CmtDeviceMode deviceMode(mode, settings, sampleFreq);
    if ((deviceIds[i] & 0xFFF00000) != 0x00500000) 
    {
      // not an MTi-G, remove all GPS related stuff
      deviceMode.m_outputMode &= 0xFF0F;
    }
    res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
    EXIT_ON_ERROR(res,"setDeviceMode");
  }
  
  // start receiving data
  res = cmt3.gotoMeasurement();
  EXIT_ON_ERROR(res,"gotoMeasurement");
}

                  
                  
//////////////////////////////////////////////////////////////////////////
// initializeXsens
//
// Inicializa o Xsens
void initializeXsens(xsens::Cmt3& cmt3, CmtOutputMode& mode, CmtOutputSettings& settings, unsigned long& mtCount, char* portName)
{
  
  CmtDeviceId deviceIds[256];
  XsensResultValue res = XRV_OK;
  
  // Perform hardware scan
  if(portName == NULL){
    mtCount = doHardwareScan(cmt3, deviceIds, portName);
  } else {
    //mtCount = doHardwareScan_Manual(cmt3, deviceIds, portName); //This function is not available at the libcmt that came with MTiG
	printf("port ok!\n");
	mtCount = doHardwareScan(cmt3, deviceIds, portName);
  }
  
  if (mtCount == 0) {
    cmt3.closePort();
    exit(1);
  }
  
  switch(mode)
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
      mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
      break;
    case 7:
      mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_RAW | CMT_OUTPUTMODE_ORIENT;
      break;
    default:
      carmen_die("Unknown mode = %d in initializeXsens()", mode);
  }
  
  if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
    switch(settings) 
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
      default:
        carmen_die("Unknown settings = %lu in initializeXsens()", settings);
    }
    
  }else{
    settings = 0;
  }
  
  settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
  
  
  // Set device to user input settings
  doMtSettings(cmt3, mode, settings, deviceIds);
  
  if (res != XRV_OK) {
    printf("\nerror %ud\n", res);
  }
  
  printf("\nInitialization complete.\n");  
}
                  

//////////////////////////////////////////////////////////////////////////
// getDataFromXsens
//
// Retorna os dados do Xsens
void getDataFromXsens(xsens::Cmt3& cmt3, xsens::Packet* packet, CmtOutputMode& mode, CmtOutputSettings& settings, xsens_global& data) 
{
  
  unsigned short sdata;
  double tdata;
  
  CmtCalData caldata;
  CmtQuat qat_data;
  CmtEuler euler_data;
  CmtMatrix matrix_data;
  
  cmt3.waitForDataMessage(packet);
  //get sample count, goto position & display.
  sdata = packet->getSampleCounter();
  data.count = sdata;
  
  //XsensResultValue res = XRV_OK;
  
  if ((mode & CMT_OUTPUTMODE_TEMP) != 0) {
    tdata = packet->getTemp(0);
    data.temp = tdata;
  }
  
  if ((mode & CMT_OUTPUTMODE_CALIB) != 0) {
    
    caldata = packet->getCalData(0);
    
    data.acc.x = caldata.m_acc.m_data[0];
    data.acc.y = caldata.m_acc.m_data[1];
    data.acc.z = caldata.m_acc.m_data[2];
    
    data.gyr.x = caldata.m_gyr.m_data[0];
    data.gyr.y = caldata.m_gyr.m_data[1];
    data.gyr.z = caldata.m_gyr.m_data[2];
    
    data.mag.x = caldata.m_mag.m_data[0];
    data.mag.y = caldata.m_mag.m_data[1];
    data.mag.z = caldata.m_mag.m_data[2];
  }
  
  if ((mode & CMT_OUTPUTMODE_ORIENT) == 0) {
    return;
  }
  
  switch (settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
    
    case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
      // Output: quaternion
      qat_data = packet->getOriQuat(0);
      
      data.quatData.data[0] = qat_data.m_data[0];
      data.quatData.data[1] = qat_data.m_data[1];
      data.quatData.data[2] = qat_data.m_data[2];
      data.quatData.data[3] = qat_data.m_data[3];
      
      break;
      
    case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
      // Output: Euler
      euler_data = packet->getOriEuler(0);
      
      data.eulerData.pitch = euler_data.m_pitch;
      data.eulerData.roll = euler_data.m_roll;
      data.eulerData.yaw = euler_data.m_yaw;
      
      break;
      
    case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
      // Output: Cosine Matrix
      matrix_data = packet->getOriMatrix(0);
      
      data.matrixData.data[0][0] = matrix_data.m_data[0][0];
      data.matrixData.data[0][1] = matrix_data.m_data[0][1];
      data.matrixData.data[0][2] = matrix_data.m_data[0][2];
      
      data.matrixData.data[1][0] = matrix_data.m_data[1][0];
      data.matrixData.data[1][1] = matrix_data.m_data[1][1];
      data.matrixData.data[1][2] = matrix_data.m_data[1][2];
      
      data.matrixData.data[2][0] = matrix_data.m_data[2][0];
      data.matrixData.data[2][1] = matrix_data.m_data[2][1];
      data.matrixData.data[2][2] = matrix_data.m_data[2][2];

      data.timestamp = packet->m_toa;
      
      break;
      
    default:
      carmen_die("Unknown settings in getDataFromXsens()");
      break;
  }
  
}

//////////////////////////////////////////////////////////////////////////
// shutDownXsens
//
// "Desliga" o Xsens
void shutDownXsens(xsens::Cmt3& cmt3)
{
  
  cmt3.closePort();
  
}
                                                             
                                                             
