/*!@file Devices/Serial.C class for interfacing with a serial port */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Nitin Dhavale <dhavale@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Serial.C $
// $Id: Serial.C 15298 2012-05-24 23:31:23Z kai $
//

#include "Devices/Serial.H"
#include "Util/Timer.H"
#include "Component/OptionManager.H"
#include "Util/log.H"
#include "rutz/unixcall.h" // for rutz::unixcall::get_file_user_pid()
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>

const ModelOptionCateg MOC_Serial= {
	MOC_SORTPRI_3, "Serial Port Related Options" };

const ModelOptionDef OPT_DevName =
{ MODOPT_ARG(std::string), "DevName", &MOC_Serial, OPTEXP_CORE,
	"Device file",
	"serial-dev", '\0', "", "/dev/ttyUSB9" };

// ######################################################################

Serial::Serial(OptionManager& mgr,
		const std::string& descrName,
		const std::string& tagName) :
	ModelComponent(mgr, descrName, tagName),
	itsCmdDevName(&OPT_DevName, this, 0),
	itsDeviceDescription(tagName+"DeviceDescription", this, ""),
	itsSearchPrefix(tagName+"SearchPrefix", this, "ttyUSB"),
	itsBaud(tagName+"Baud", this, 9600),
	itsCharBits(tagName+"CharBits", this, 8),
	itsStopBits(tagName+"StopBits", this, 1),
	itsUseParity(tagName+"UseParity", this, false),
	itsParityOdd(tagName+"ParityOdd", this, false),
	itsFlowHard(tagName+"FlowHard", this, false),
	itsFlowSoft(tagName+"FlowSoft", this, false),
	itsRdTout(tagName+"ReadTimeout", this, 0),
	itsBlocking(tagName+"Blocking", this, true),
	itsSerialOk(tagName+"SerialOk", this, false),
	dev(-1),
	itsDevName(""),
	serialErrno(serialErrSuccess)
{

}

// ######################################################################
void Serial::searchDevice(std::string prefix,std::string deviceDescription,bool dontCheck)
{

	LINFO("Searching for devices");

	DIR *directory_p;
	struct dirent *entry_p;

	//Open the device directory to search for devices whose names match the search prefix
	directory_p = ::opendir ("/dev");
	if (directory_p == NULL)
		LFATAL("Could Not Open /dev Directory!");

	//Iterate through the directory entries
	while ((entry_p = ::readdir (directory_p)))
	{
		std::string entryName(entry_p->d_name);
		if(entryName.find(prefix.c_str()) != std::string::npos)
		{//If the directory entry name matches our search prefix, then let's try configuring a serial
			//port on that device, sending it an identity request command (0x00), and comparing the result
			//with our required device description

			std::string portName("/dev/"+entryName);
			LDEBUG("Cmd %s,Found port %s ,check its description match [%s] or not",itsCmdDevName.getVal().c_str(),portName.c_str(),deviceDescription.c_str());
			if(itsCmdDevName.getVal() == std::string("/dev/"+entryName))
			{
				LDEBUG("Same port,don't need re-enable it");
			}else{
				enablePort("/dev/" + entryName);
			}
			unsigned char cmd[1] = {0};

			if(!dontCheck){
				LDEBUG("Send board description cmd 1 to device");
				write(cmd,1);
				LDEBUG("Read device description");
				std::vector<unsigned char> deviceStringVec = readFrame(cmd[0], 255);
				std::string deviceString(deviceStringVec.begin(), deviceStringVec.end());
				LINFO("Got %s : %s", entryName.c_str(), deviceString.c_str());
				if(deviceString == itsDeviceDescription.getVal())
				{
					itsCmdDevName.setVal("/dev/"+entryName);
					break;
				}
			}else{
				itsCmdDevName.setVal("/dev/"+entryName);
				break;	
			}
		}


	}
	(void) ::closedir (directory_p);
	if(itsCmdDevName.getVal() == "")
	{
		LDEBUG("Could Not Find Serial Device Matching Descriptor (%s)", itsDeviceDescription.getVal().c_str());
	}
}
void Serial::start2()
{
	//Check to see if we have a hardcoded device name. If so, then let's just
	//go ahead and enable that port. If
	LINFO("Looking Device Name [%s]",itsDevName.c_str());
	if (itsDevName == "search") 
	{
		searchDevice(itsSearchPrefix.getVal(),itsDeviceDescription.getVal());
	}else if(itsDevName != "")
	{
		LINFO("Opening %s", itsDevName.c_str());
		enablePort(itsDevName);	
	
	} else {
		LINFO("Opening from cmd line %s", itsCmdDevName.getVal().c_str());
		enablePort(itsCmdDevName.getVal());
	}
}


// ######################################################################
void Serial::enablePort(std::string DeviceName)
{
	closePort();
	serialErrno = serialErrSuccess;      // clear the error flag

	LDEBUG("Open Port %s",DeviceName.c_str());
	openPort(DeviceName);                // open the device
	perror();

	LDEBUG("Set Speed %d",itsBaud.getVal());
	setSpeed(itsBaud.getVal());          // Set the baud rate
	perror();

	setFlowControl(itsFlowHard.getVal(),
			itsFlowSoft.getVal());           // Set flow control
	perror();

	setCharBits(itsCharBits.getVal());   // set no of bits in a char
	perror();

	setParity(itsUseParity.getVal(),
			itsParityOdd.getVal());          // Set even or odd parity
	perror();

	LDEBUG("Set Blocking Mode %d",itsBlocking.getVal());
	setBlocking(itsBlocking.getVal());   // blocking mode?
	perror();

	LDEBUG("enablePort Done");
}


#include <cstdio>
// ######################################################################
void Serial::stop2()
{
	closePort();
}

// ######################################################################
void Serial::closePort()
{
	if(dev >= 0) {
		sendBreak();
		close(dev);
		dev = -1; }
}

// ######################################################################
void Serial::configureSearch(const std::string DeviceDescription, const int speed, const std::string SearchPrefix, const char *format,
		const bool flowSoft, const bool flowHard,
		const int tout)
{
	LINFO("Current DevName is %s,Search for %s in speed %d",itsDevName.c_str(),DeviceDescription.c_str(),speed);
	itsDevName = "search";
	itsDeviceDescription.setVal(DeviceDescription);
	itsSearchPrefix.setVal(SearchPrefix);
	itsBaud.setVal(speed);
	itsFlowSoft.setVal(flowSoft);
	itsFlowHard.setVal(flowHard);
	itsRdTout.setVal(tout);
	//enable serch
	LINFO("Searching for devices");
	configure(itsDevName.c_str(), speed, format, flowSoft, flowHard, tout);
}

// ######################################################################
void Serial::configure(const char *dev, const int speed, const char *format,
		const bool flowSoft, const bool flowHard,
		const int tout)
{
	itsDevName = dev;


	LDEBUG("Serial Port is %s",itsDevName.c_str());
	itsBaud.setVal(speed);
	itsFlowSoft.setVal(flowSoft);
	itsFlowHard.setVal(flowHard);
	itsRdTout.setVal(tout);

	if (strlen(format) != 3) LFATAL("Incorrect format string: %s", format);

	switch(format[0]) {
		case '5': itsCharBits.setVal(5); break;
		case '6': itsCharBits.setVal(6); break;
		case '7': itsCharBits.setVal(7); break;
		case '8': itsCharBits.setVal(8); break;
		default: LFATAL("Invalid charbits: %c (should be 5..8)", format[0]);
	}

	switch(format[1]) {
		case 'N': itsUseParity.setVal(false); break;
		case 'E': itsUseParity.setVal(true); itsParityOdd.setVal(false); break;
		case 'O': itsUseParity.setVal(true); itsParityOdd.setVal(true); break;
		default: LFATAL("Invalid parity: %c (should be N,E,O)", format[1]);
	}

	switch(format[2]) {
		case '1': itsStopBits.setVal(1); break;
		case '2': itsStopBits.setVal(2); break;
		default: LFATAL("Invalid stopbits: %c (should be 1..2)", format[2]);
	}
}

// ######################################################################
serialError Serial::setSpeed(const int speed)
{
	struct termios options;
	unsigned int rate;

	switch(speed)
	{
		case 230400:
			rate = B230400;
			break;
		case 115200:
			rate = B115200;
			break;
		case 57600:
			rate = B57600;
			break;
		case 38400:
			rate = B38400;
			break;
		case 19200:
			rate = B19200;
			break;
		case 9600:
			rate = B9600;
			break;
		case 4800:
			rate = B4800;
			break;
		case 2400:
			rate = B2400;
			break;
		case 1200:
			rate = B1200;
			break;
		case 600:
			rate = B600;
			break;
		case 300:
			rate = B300;
			break;
		case 110:
			rate = B110;
			break;
		case 0:
			rate = B0;
			break;
		default:
			return serialError(serialErrSpeedInvalid);
	}

	// get current options
	if(tcgetattr(dev, &options)==-1){
		serialErrno = serialErrTcGetAttrFailed;
		return serialErrno;
	}

	// set the speed
	cfsetispeed(&options, rate);
	cfsetospeed(&options, rate);

	// change the terminals parameter instantly
	if( tcsetattr(dev, TCSANOW, &options) == -1){
		serialErrno = serialErrTcSetAttrFailed;
		return serialErrno;
	}

	if( tcgetattr(dev, &options) == -1) {
		serialErrno = serialErrTcGetAttrFailed;
		return serialErrno;
	}

	// update our ModelParam:
	itsBaud.setVal(speed);

	return serialErrSuccess;
}


// ######################################################################
serialError Serial::setFlowControl(const bool useHard, const bool useSoft)
{
	termios options;

	if( tcgetattr(dev, &options) == -1){
		serialErrno = serialErrTcGetAttrFailed;
		return serialErrno;
	}

	options.c_cflag &= ~CRTSCTS;
	options.c_iflag &= ~(IXON | IXANY | IXOFF);

	if (useSoft) options.c_iflag |= (IXON | IXANY | IXOFF);
	if (useHard) options.c_cflag |= CRTSCTS;

	if(tcsetattr(dev, TCSANOW, &options) == -1){
		serialErrno = serialErrTcGetAttrFailed;
		return serialErrno;
	}

	// update our ModelParams:
	itsFlowHard.setVal(useHard);
	itsFlowSoft.setVal(useSoft);

	return serialErrSuccess;
}

// ######################################################################
serialError Serial::setCharBits(const int bits)
{
	termios options;

	if(tcgetattr(dev, &options)==-1){
		serialErrno = serialErrTcGetAttrFailed;
		return serialErrno;
	}

	options.c_cflag &= ~CSIZE; // mask off the 'size' bits

	switch(bits)
	{
		case 5: options.c_cflag |= CS5; break;
		case 6: options.c_cflag |= CS6; break;
		case 7: options.c_cflag |= CS7; break;
		case 8: options.c_cflag |= CS8; break;
		default: return serialError(serialErrCharsizeInvalid);
	}

	if( tcsetattr(dev, TCSANOW, &options) == -1 ){
		serialErrno = serialErrTcSetAttrFailed;
		return serialErrno;
	}

	// update our ModelParam:
	itsCharBits.setVal(bits);

	return serialErrSuccess;
}

// ######################################################################
serialError Serial::setParity(const bool useParity, const bool oddParity)
{
	struct termios options;

	if(tcgetattr(dev, &options)==-1){
		serialErrno = serialErrTcGetAttrFailed;
		return serialErrno;
	}

	options.c_cflag &= ~(PARENB | PARODD);
	if (useParity)
	{
		if (oddParity)
			options.c_cflag |= (PARENB | PARODD);
		else
			options.c_cflag |= PARENB;
	}

	if(tcsetattr(dev, TCSANOW, &options) == -1){
		serialErrno = serialErrTcSetAttrFailed;
		return serialErrno;
	}

	// update our ModelParams:
	itsUseParity.setVal(useParity);
	itsParityOdd.setVal(oddParity);

	return serialErrSuccess;
}

// ######################################################################
serialError Serial::setStopBits(const int bits)
{
	struct termios options;

	if(tcgetattr(dev, &options)==-1){
		serialErrno = serialErrTcGetAttrFailed;
		return serialErrno;
	}

	options.c_cflag &= ~CSTOPB;
	if (bits == 2) options.c_cflag |= CSTOPB;
	else if (bits != 1) return serialError(serialErrStopbitsInvalid);

	if(tcsetattr(dev, TCSANOW, &options) == -1){
		serialErrno = serialErrTcSetAttrFailed;
		return serialErrno;
	}

	// update our ModelParam:
	itsStopBits.setVal(bits);

	return serialErrSuccess;
}

// ######################################################################
serialError Serial::setBlocking(const bool blocking)
{
	int flags = fcntl(dev, F_GETFL, 0);
	if (flags == -1) PLERROR("Cannot get flags");
	if (blocking) flags &= (~O_NONBLOCK); else flags |= O_NONBLOCK;
	if (fcntl(dev, F_SETFL, flags) == -1) PLERROR("Cannot set flags");

	itsBlocking.setVal(blocking);

	return serialErrSuccess;
}

// ######################################################################
void Serial::toggleDTR(const time_t ms)
{
	struct termios tty, old;
	if(tcgetattr(dev, &tty) == -1 || tcgetattr(dev, &old) == -1){
		serialErrno = serialErrTcGetAttrFailed;
	}

	cfsetospeed(&tty, B0);
	cfsetispeed(&tty, B0);

	if(tcsetattr(dev, TCSANOW, &tty) == -1){
		serialErrno = serialErrTcSetAttrFailed;
	}

	if(ms)
		usleep(ms*1000);

	if(tcsetattr(dev, TCSANOW, &old) == -1){
		serialErrno = serialErrTcSetAttrFailed;
	}
}

// ######################################################################
void Serial::sendBreak(void)
{
	// Send a Hangup to the port
	tcsendbreak(dev, 0);
}

// ######################################################################
serialError Serial::error(const serialError serialErrorNum)
{
	serialErrno = serialErrorNum;
	return serialErrorNum;
}

// ######################################################################
int Serial::openPort(std::string DeviceName)
{
	const pid_t fuser =
		rutz::unixcall::get_file_user_pid(DeviceName.c_str());

	if (fuser != 0 && DeviceName.compare("/dev/null") != 0)
	{
		LINFO("serial device %s is already in use by process pid=%d;\n"
				"\ttry using /dev/null or a different serial device instead",
				DeviceName.c_str(), int(fuser));
	}

	int flags= O_RDWR | O_NOCTTY;
	termios options;

	// don't set the flag if we are setting a timeout on
	// the descriptor
	if (itsRdTout.getVal() < 0) flags |= O_NDELAY;

	LDEBUG("Opening port %s", DeviceName.c_str());
	dev = ::open(DeviceName.c_str(), flags);
	if (dev == -1) { serialErrno = serialErrOpenFailed; return dev; }
	LDEBUG("Done");

	// Save current state
	if( tcgetattr(dev, &savedState) == -1 ){
		serialErrno = serialErrTcGetAttrFailed;
		return -1;
	}

	// reset all the flags
	if( fcntl(dev, F_SETFL, 0) == -1 ){
		serialErrno = serialErrFcntlFailed;
		return -1;
	}

	if(tcgetattr(dev, &options) == -1 ){
		serialErrno = serialErrTcGetAttrFailed;
		return -1;
	}

	// get raw input from the port
	options.c_cflag |= ( CLOCAL     // ignore modem control lines
			| CREAD ); // enable the receiver

	options.c_iflag &= ~(  IGNBRK    // ignore BREAK condition on input
			| BRKINT  // If IGNBRK is not set, generate SIGINT
			// on BREAK condition, else read BREAK as \0
			| PARMRK
			| ISTRIP  // strip off eighth bit
			| INLCR   // donot translate NL to CR on input
			| IGNCR   // ignore CR
			| ICRNL   // translate CR to newline on input
			| IXON    // disable XON/XOFF flow control on output
			);

	// disable implementation-defined output processing
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO  // dont echo i/p chars
			| ECHONL // do not echo NL under any circumstance
			| ICANON // disable cannonical mode
			| ISIG   // do not signal for INTR, QUIT, SUSP etc
			| IEXTEN // disable platform dependent i/p processing
			);

	// set a timeout on the descriptor
	if(itsRdTout.getVal() > 0) {
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = itsRdTout.getVal();
	}

	if( tcsetattr(dev, TCSANOW, &options) == -1){
		serialErrno = serialErrTcSetAttrFailed;
		return -1;
	}
	LDEBUG("Open Port Done dev=%d",dev);
	return dev;
}


// ######################################################################
int Serial::read(void* buffer, const int nbytes)
{
	int n = ::read(dev, buffer, nbytes);//make sure program not stuck here
	if(n==-1)
		serialErrno = serialErrReadFailed;

	if(n==0)
		serialErrno = serialErrReadTimedOut;
	else
		serialErrno = serialErrSuccess; 

	return n;
}


// ######################################################################
char Serial::read(void)
{
	char ch;

	this->read(&ch, 1);

	return ch;
}

std::vector<unsigned char> Serial::readFrame(const unsigned char frameStart, const unsigned char frameEnd, const int frameLength, const double timeout)
{

	std::vector<unsigned char> data;
	unsigned char buffer[1024];

	Timer timer;

	unsigned int startPos  =  0;
	unsigned int currPos   =  0;
	int frameSize          = -1;
	bool done              = false;
	int startFrameOffset   = 2;

	if(frameLength > 0)
	{
		frameSize = frameLength;
		startFrameOffset = 1;
	}

	timer.reset();
	LDEBUG("Start Reading Data time %f",timer.getSecs());
	while(!done)
	{

		//If we have reached the timeout, then just quit return an empty vector
		if(timeout > 0 && timer.getSecs() > timeout)
			return std::vector<unsigned char>();

		//LDEBUG("Call read() %f",timer.getSecs());
		//Read data into the data buffer
		int bytesRead = read(buffer, 1024);
		//LDEBUG("Read Done %f",timer.getSecs());


	  //LINFO("Got %i bytes", bytesRead);
    //for(int i=0; i<bytesRead; i++)
		//	printf("%i ", buffer[i]);
	  //printf("\n");
	  

		if(bytesRead > 0)
		{
			//If we read some bytes, then let's append them onto the data buffer
			std::vector<unsigned char> newData(buffer, buffer+bytesRead*(sizeof(buffer[0])));
			data.insert(data.end(), newData.begin(), newData.end());

			while(currPos < data.size())
			{
				//If we have reached the timeout, then just quit return an empty vector
				if(timeout > 0 && timer.getSecs() > timeout)
					return std::vector<unsigned char>();

				if(data.at(startPos) != frameStart)
				{ //Get the start byte position settled
					currPos++;
					startPos = currPos;
				}
				else
				{ //Here, we already have our start byte

					if(frameLength == -1 && frameSize == -1 && (startPos+1 < data.size()) )
					{ //Find the frame size if one was not supplied, and we haven't yet found it
						frameSize = data.at(startPos+1);
						currPos++;
					}
					else if(frameSize > -1 && data.size() <= currPos+frameSize+1)
					{ //If we know our frame size, but our data buffer is not large enough,
						//then just break and we will read more data at the top of the while loop
						break;
					}
					else if(frameSize > -1 && data.size() > currPos+frameSize+1 && data.at(currPos+frameSize+1) != frameEnd)
					{ //If we know our frame size, and have enough data, and our frame stop
						//byte is not there, then we need to move our startPos hypothesis one
						//byte forward and start the whole thing over again
						if(frameLength == -1)
							frameSize = -1;
						startPos++;
						currPos = startPos;
					}
					else if(frameSize > -1 && data.size() > currPos+frameSize+1 && data.at(currPos+frameSize+1) == frameEnd)
					{ //All constraints satisfied - we have our frame
						return std::vector<unsigned char>(data.begin()+startPos+startFrameOffset, data.begin()+startPos+startFrameOffset+frameSize);
					}
				}
			}
		}
		else
		{ //If the serial read returned 0 bytes, then just wait for a bit before reading again
			usleep(1000);

			//If the timer has expired, then let's just return an empty vector
			if(timer.getSecs() > timeout && timeout > 0)
				return std::vector<unsigned char>();
		}
	}
	return std::vector<unsigned char>();
}


// ######################################################################
int Serial::write(const void* buffer, const int nbytes)
{
	//char c[3] = { 0xff, 0x01, 0xff };
	//::write(dev, c, 3);

	int n = ::write(dev, buffer, nbytes);

	/*
	//LINFO("----------------- WRITE --------------------");
	for(int i=0; i<n; ++i)
	{
	fprintf(stderr, "%02X ",
	(static_cast<const unsigned char*>(buffer)[i])&0xff);
	}
	fprintf(stderr,"\n");
	//LINFO("-------------------------------------------");
	 */

	if(n==-1)
		serialErrno = serialErrWriteFailed;

	return n;
}

// ######################################################################
int Serial::write(const unsigned char character)
{
	return write(&character, 1);
}


// ######################################################################
void Serial::flush(void)
{
	//Flush the input
	int status = tcflush(dev,TCIFLUSH);
	if(status != 0)
		LERROR("BAD SERIAL FLUSH");

}

// ######################################################################
void Serial::perror(void)
{
	switch(serialErrno)
	{
		case serialErrSuccess:
			LERROR("Serial::OK");
			return;
		case serialErrOpenNoTty:
			LERROR("Serial::Open() No TTY Failed");
			break;
		case serialErrOpenFailed:
			LERROR("Serial::Open() Failed");
			break;
		case serialErrSpeedInvalid:
			LERROR("Serial::setSpeed() Invalid Speed Param");
			break;
		case serialErrFlowInvalid:
			LERROR("Serial::setFlow() Invalid Flow Param");
			break;
		case serialErrParityInvalid:
			LERROR("Serial::setParity() Invalid parity Param");
			break;
		case serialErrCharsizeInvalid:
			LERROR("Serial::setCharSize() Invalid Char Size");
			break;
		case serialErrStopbitsInvalid:
			LERROR("Serial::setStopBits() Invalid Stop Bits");
			break;
		case serialErrOptionInvalid:
			LERROR("Serial::read() invalid option");
			break;
		case serialErrOutput:
			LERROR("Serial::read() err output");
			break;
		case serialErrReadFailed:
			LERROR("Serial::read() Failed");
			break;
		case serialErrReadTimedOut:
			LERROR("Serial::read() Timed Out");
			break;
		case serialErrWriteFailed:
			LERROR("Serial::Write() Failed");
			break;
		case serialErrFcntlFailed:
			LERROR("Serial::Fcntl() Failed");
			break;
		case serialErrTcGetAttrFailed:
			LERROR("Serial::tcgetattr() Failed");
			break;
		case serialErrTcSetAttrFailed:
			LERROR("Serial::tcsetattr() Failed");
			break;
		default:
			LERROR("Unknow error!");
	}
}


// ######################################################################
Serial::~Serial(void)
{  }



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
