/*!@file Devices/SerialAdvanced.C A class for interfacing with a serial port */

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
// Primary maintainer for this file: Randolph Voorhies <voorhies at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/SerialAdvanced.C $
// $Id: SerialAdvanced.C 13712 2010-07-28 21:00:40Z itti $
//

#include "Devices/SerialAdvanced.H"

#include "Component/OptionManager.H"
#include "Util/log.H"
#include "rutz/unixcall.h" // for rutz::unixcall::get_file_user_pid()

#include <iostream>

const ModelOptionCateg MOC_SerialAdvanced = {
    MOC_SORTPRI_3, "Advanced Serial Port Related Options" };

const ModelOptionDef OPT_DevNameAdvanced =
{ MODOPT_ARG(std::string), "DevNameAdvanced", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Device file",
    "serial-dev-advanced", '\0', "", "/dev/ttyS0" };

const ModelOptionDef OPT_Baud =
{ MODOPT_ARG(int), "Baud", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Baud Rate",
    "serial-baud", '\0', "", "9600" };

const ModelOptionDef OPT_CharBits =
{ MODOPT_ARG(int), "CharBits", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Number of bits per character",
    "serial-charbits", '\0', "", "8" };

const ModelOptionDef OPT_StopBits =
{ MODOPT_ARG(int), "StopBits", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Number of stop bits",
    "serial-stopbits", '\0', "", "1" };

const ModelOptionDef OPT_UseParity =
{ MODOPT_ARG(bool), "UseParity", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Use Parity?",
    "serial-parity", '\0', "true | false", "false" };

const ModelOptionDef OPT_ParityOdd =
{ MODOPT_ARG(bool), "ParityOdd", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Use Odd Parity?",
    "serial-oddparity", '\0', "true | false", "false" };

const ModelOptionDef OPT_FlowHard =
{ MODOPT_ARG(bool), "FlowHard", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Use Hardware Flow Control?",
    "serial-flowhard", '\0', "true | false", "false" };

const ModelOptionDef OPT_FlowSoft =
{ MODOPT_ARG(bool), "FlowSoft", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Use Software Flow Control?",
    "serial-flowsoft", '\0', "true | false", "false" };

const ModelOptionDef OPT_RdTout =
{ MODOPT_ARG(int), "RdTout", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Read Timeout",
    "serial-readtout", '\0', "", "-1" };

const ModelOptionDef OPT_Blocking =
{ MODOPT_ARG(bool), "Blocking", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Use Blocking Mode?",
    "serial-blocking", '\0', "true | false", "true" };

const ModelOptionDef OPT_DevSearchDescriptor =
{ MODOPT_ARG(std::string), "DevSearchDesctriptor", &MOC_SerialAdvanced, OPTEXP_CORE,
    "Some of the serial devices in iLab will respond to a command of '0' with "
    "a framed string containing a device description. By setting this commmand line option, "
    "one can specify which such string to search for among all available serial devices. "
    "The DevName will then be set according to which available device with the correct string.",
    "dev-descriptor", '\0', "", "" };

// ######################################################################

SerialAdvanced::SerialAdvanced(OptionManager& mgr,
               const std::string& descrName,
               const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  dev(-1), serialErrno(serialErrSuccess),
  itsDevName(&OPT_DevNameAdvanced, this,0),
  itsBaud(&OPT_Baud, this, 0),
  itsCharBits(&OPT_CharBits, this, 0),
  itsStopBits(&OPT_StopBits, this, 0),
  itsUseParity(&OPT_UseParity, this, 0),
  itsParityOdd(&OPT_ParityOdd, this, 0),
  itsFlowHard(&OPT_FlowHard, this, 0),
  itsFlowSoft(&OPT_FlowSoft, this, 0),
  itsRdTout(&OPT_RdTout, this, 0),
  itsBlocking(&OPT_Blocking, this, 0),
  itsDevSearchDescriptor(&OPT_DevSearchDescriptor, this, 0)
{  }

// ######################################################################
void SerialAdvanced::start2()
{
  serialErrno = serialErrSuccess; // clear the error flag

  openPort();   // open the device
  perror();

  setSpeed(itsBaud.getVal());        // Set the baud rate
  perror();

  setFlowControl(itsFlowHard.getVal(),
                 itsFlowSoft.getVal());   // Set flow control
  perror();

  setCharBits(itsCharBits.getVal());        // set no of bits in a char
  perror();

  setParity(itsUseParity.getVal(),
            itsParityOdd.getVal());      // Set even or odd parity
  perror();

  setBlocking(itsBlocking.getVal()); // blocking mode?
  perror();
}

#include <cstdio>
// ######################################################################
void SerialAdvanced::stop2()
{
  if(dev >= 0) {
    //sendBreak();
    fputs("closing...\n", stderr);
    close(dev);
    dev = -1; }
}

// ######################################################################
void SerialAdvanced::configure(const char *dev, const int speed, const char *format,
                       const bool flowSoft, const bool flowHard,
                       const int tout)
{
        itsDevName.setVal(dev);
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
serialError SerialAdvanced::setSpeed(const int speed)
{
  struct termios options;
  unsigned int rate;

  switch(speed)
    {
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
serialError SerialAdvanced::setFlowControl(const bool useHard, const bool useSoft)
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
serialError SerialAdvanced::setCharBits(const int bits)
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
serialError SerialAdvanced::setParity(const bool useParity, const bool oddParity)
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
serialError SerialAdvanced::setStopBits(const int bits)
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
serialError SerialAdvanced::setBlocking(const bool blocking)
{
  int flags = fcntl(dev, F_GETFL, 0);
  if (flags == -1) PLERROR("Cannot get flags");
  if (blocking) flags &= (~O_NONBLOCK); else flags |= O_NONBLOCK;
  if (fcntl(dev, F_SETFL, flags) == -1) PLERROR("Cannot set flags");

  itsBlocking.setVal(blocking);

  return serialErrSuccess;
}

// ######################################################################
void SerialAdvanced::toggleDTR(const time_t ms)
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
void SerialAdvanced::sendBreak(void)
{
  // Send a Hangup to the port
  tcsendbreak(dev, 0);
}

// ######################################################################
serialError SerialAdvanced::error(const serialError serialErrorNum)
{
  serialErrno = serialErrorNum;
  return serialErrorNum;
}

// ######################################################################
int SerialAdvanced::openPort()
{
  const pid_t fuser =
    rutz::unixcall::get_file_user_pid(itsDevName.getVal().c_str());

  if (fuser != 0 && itsDevName.getVal().compare("/dev/null") != 0)
    {
      LFATAL("serial device %s is already in use by process pid=%d;\n"
             "\ttry using /dev/null or a different serial device instead",
             itsDevName.getVal().c_str(), int(fuser));
    }

  int flags= O_RDWR | O_NOCTTY;
  termios options;

  // don't set the flag if we are setting a timeout on
  // the descriptor
  if (itsRdTout.getVal() < 0) flags |= O_NDELAY;

  CLDEBUG("Opening port %s", itsDevName.getVal().c_str());
  dev = ::open(itsDevName.getVal().c_str(), flags);
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
  return dev;
}


// ######################################################################
int SerialAdvanced::read(void* buffer, const int nbytes)
{
  int n = ::read(dev, buffer, nbytes);

  if(n==-1)
    serialErrno = serialErrReadFailed;

  if(n==0)
    serialErrno = serialErrReadTimedOut;

  return n;
}


// ######################################################################
char SerialAdvanced::read(void)
{
  char ch;

  this->read(&ch, 1);

  return ch;
}


// ######################################################################
int SerialAdvanced::write(const void* buffer, const int nbytes)
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
void SerialAdvanced::perror(void)
{
  const char* s=NULL; // initialize to stop g++ from complaining!
  switch(serialErrno)
    {
    case serialErrSuccess:
      s = "Success";
      return;
    case serialErrOpenNoTty:
      s = "SerialAdvanced::Open() No TTY Failed";
      break;
    case serialErrOpenFailed:
      s = "SerialAdvanced::Open() Failed";
      break;
    case serialErrSpeedInvalid:
      s = "SerialAdvanced::setSpeed() Invalid Speed Param";
      break;
    case serialErrFlowInvalid:
      s = "SerialAdvanced::setFlow() Invalid Flow Param";
      break;
    case serialErrParityInvalid:
      s = "SerialAdvanced::setParity() Invalid parity Param";
      break;
    case serialErrCharsizeInvalid:
      s = "SerialAdvanced::setCharSize() Invalid Char Size";
      break;
    case serialErrStopbitsInvalid:
      s = "SerialAdvanced::setStopBits() Invalid Stop Bits";
      break;
    case serialErrOptionInvalid:
      break;
    case serialErrOutput:
      break;
    case serialErrReadFailed:
      s = "SerialAdvanced::read() Failed";
      break;
    case serialErrReadTimedOut:
      s = "SerialAdvanced::read() Timed Out";
      break;
    case serialErrWriteFailed:
      s = "SerialAdvanced::Write() Failed";
      break;
    case serialErrFcntlFailed:
      s = "SerialAdvanced::Fcntl() Failed";
      break;
    case serialErrTcGetAttrFailed:
      s = "SerialAdvanced::tcgetattr() Failed";
      break;
    case serialErrTcSetAttrFailed:
      s = "SerialAdvanced::tcsetattr() Failed";
      break;
    default:
      s = "Unknow error!";
    }
  LERROR("%s", s);
}


// ######################################################################
SerialAdvanced::~SerialAdvanced(void)
{  }



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
