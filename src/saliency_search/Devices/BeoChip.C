/*!@file Devices/BeoChip.C Interface to Brian Hudson's BeoChip interface device*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeoChip.C $
// $Id: BeoChip.C 13708 2010-07-28 16:55:36Z itti $
//

#include "Devices/BeoChip.H"

#include "Component/OptionManager.H"
#include "Util/Assert.H"
#include "rutz/compat_snprintf.h"

#define NUMSERVO 8
#define NUMPWM 2
#define NUMADC 2


// Recursive template metaprogramming is used here to allow us to
// define binary constants in the code, as we use these a lot in this
// BeoChip program. See http://www.flipcode.com/cgi-bin/msg.cgi?
// showThread=Tip-CPPCompileTimeBinConst&forum=totd&id=-1 for
// additional info. NOTE: we have to be very careful not to have
// leading zeros in our constants, otherwise they will be interpreted
// as octal numbers by the compiler. The macro BIN below fixes that by
// first forcing the interpretation of our constant as a double (which
// may have leading zeros) and then casting it back to int and doing
// the recursion. See Carlo Pescio's "Binary Constants Using Template
// Metaprogramming," C/C++ Users Journal, February 1997 for a version
// of that (which we here bugfixed for syntax).
template<unsigned long int N> class binary
{ public: enum { bit = N % 10, value = bit + (((binary<N/10>::value)<<1)) }; };
template <> class binary<0> { public: enum { bit = 0, value = 0 }; };
#define BIN(N) (((byte(binary<(unsigned long int)(N##.0)>::value))))

// ######################################################################
BeoChipListener::~BeoChipListener()
{ }

// ######################################################################
void *BeoChip_run(void *c)
{
  BeoChip *d = (BeoChip *)c;
  d->run();
  return NULL;
}

// ######################################################################
BeoChip::BeoChip(OptionManager& mgr, const std::string& descrName,
                 const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsDevName("BeoChipDeviceName", this, "/dev/ttyS0"),
  itsLCDrows("BeoChipLCDrows", this, 4),
  itsLCDcols("BeoChipLCDcols", this, 20),
  itsUseRTSCTS("BeoChipUseRTSCTS", this, true)
{
  // Initialize our internals:
  zeroS = new rutz::shared_ptr<NModelParam<int> >[NUMSERVO];
  minS = new rutz::shared_ptr<NModelParam<int> >[NUMSERVO];
  maxS = new rutz::shared_ptr<NModelParam<int> >[NUMSERVO];
  servopos = new byte[NUMSERVO];
  for (uint i = 0; i < NUMSERVO; i ++)
    {
      servopos[i] = 127; char buf[20];
      sprintf(buf, "ZeroS%d", i);
      zeroS[i] = NModelParam<int>::make(buf, this, 127);
      sprintf(buf, "MinS%d", i);
      minS[i] = NModelParam<int>::make(buf, this, 0);
      sprintf(buf, "MaxS%d", i);
      maxS[i] = NModelParam<int>::make(buf, this, 255);
    }

  zeroP = new rutz::shared_ptr<NModelParam<int> >[NUMPWM];
  minP = new rutz::shared_ptr<NModelParam<int> >[NUMPWM];
  maxP = new rutz::shared_ptr<NModelParam<int> >[NUMPWM];
  pulseval = new short int[NUMPWM];
  for (uint i = 0; i < NUMPWM; i ++)
    {
      pulseval[i] = 1023; char buf[20];
      sprintf(buf, "ZeroP%d", i);
      zeroP[i] = NModelParam<int>::make(buf, this, 1023);
      sprintf(buf, "MinP%d", i);
      minP[i] = NModelParam<int>::make(buf, this, 0);
      sprintf(buf, "MaxP%d", i);
      maxP[i] = NModelParam<int>::make(buf, this, 2047);
    }

  zeroA = new rutz::shared_ptr<NModelParam<int> >[NUMADC];
  minA = new rutz::shared_ptr<NModelParam<int> >[NUMADC];
  maxA = new rutz::shared_ptr<NModelParam<int> >[NUMADC];
  adcval = new byte[NUMADC];
  for (uint i = 0; i < NUMADC; i ++)
    {
      adcval[i] = 127; char buf[20];
      sprintf(buf, "ZeroA%d", i);
      zeroA[i] = NModelParam<int>::make(buf, this, 127);
      sprintf(buf, "MinA%d", i);
      minA[i] = NModelParam<int>::make(buf, this, 0);
      sprintf(buf, "MaxA%d", i);
      maxA[i] = NModelParam<int>::make(buf, this, 255);
    }

  itsFd = -1;
  itsListener.reset(NULL);
  keyboard = 0;
  pthread_mutex_init(&lock, NULL);
  pthread_mutex_init(&serlock, NULL);
}

// ######################################################################
void BeoChip::start1()
{
  CLDEBUG("Opening port %s", itsDevName.getVal().c_str());
  itsFd = open(itsDevName.getVal().c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (itsFd < 0)
    PLFATAL("Error opening serial port '%s'", itsDevName.getVal().c_str());
  CLDEBUG("Opened port %s", itsDevName.getVal().c_str());

  // save current port settings:
  if (tcgetattr(itsFd, &itsOldtio) == -1) PLFATAL("tcgetattr() failed");

  // set input mode (non-canonical, no echo, ...):
  struct termios newtio;
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = CS8 | CREAD;
  if (itsUseRTSCTS.getVal())
    newtio.c_cflag |= CRTSCTS;
  else
    {
      newtio.c_cflag |= CLOCAL;
      CLDEBUG("WARNING: RTS/CTS flow and modem control lines not used");
    }
  newtio.c_iflag = IGNBRK | IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;   /* inter-character timer unused */
  newtio.c_cc[VMIN] = 1;   /* blocking read until 1 char received */

  // Clear everything out:
  if (tcflush(itsFd, TCIFLUSH) == -1) PLFATAL("tcflush() failed");

  // Set our new termio device settings:
  if (cfsetispeed(&newtio, B19200) == -1)
    PLFATAL("Cannot set serial in speed");
  if (cfsetospeed(&newtio, B19200) == -1)
    PLFATAL("Cannot set serial out speed");
  if (tcsetattr(itsFd, TCSANOW, &newtio) == -1)
    PLFATAL("tcsetattr() failed");

  // start our thread:
  keepgoing = true;
  pthread_create(&runner, NULL, &BeoChip_run, (void *)this);
}

// ######################################################################
void BeoChip::stop2()
{
  keepgoing = false;
  usleep(300000); // make sure thread has exited

  // restore old port settings:
  if (tcsetattr(itsFd, TCSANOW, &itsOldtio) == -1)
    PLERROR("tcsetaddr() failed");
  close(itsFd);
  itsFd = -1;
}

// ######################################################################
BeoChip::~BeoChip()
{
  delete [] zeroS;
  delete [] minS;
  delete [] maxS;
  delete [] servopos;

  delete [] zeroP;
  delete [] minP;
  delete [] maxP;
  delete [] pulseval;

  delete [] zeroA;
  delete [] minA;
  delete [] maxA;
  delete [] adcval;

  pthread_mutex_destroy(&lock);
  pthread_mutex_destroy(&serlock);
}

// ######################################################################
void BeoChip::setListener(rutz::shared_ptr<BeoChipListener>& listener)
{ itsListener = listener; }

// ######################################################################
bool BeoChip::echoRequest()
{
  // 11111000: Echo request (will send back an echo reply)
  return writeByte(BIN(11111000));
}

// ######################################################################
bool BeoChip::debugMode(const bool on)
{
  // 1111110x: Turn debug mode on/off (x)
  return writeByte(BIN(11111100) | (on?1:0));
}

// ######################################################################
bool BeoChip::resetChip()
{
  // 11111001: Reset the BeoChip
  return writeByte(BIN(11111001));
}

// ######################################################################
bool BeoChip::lcdGotoXY(const int x, const int y)
{
  if (itsLCDrows.getVal() != 4)
    LFATAL("FIXME: need some code to support various numbers of rows!");

  // figure out our starting offset:
  int offset = x;
  switch(y)
    {
    case 0: break;
    case 1: offset += 64; break;
    case 2: offset += itsLCDcols.getVal(); break;
    case 3: offset += 64 + itsLCDcols.getVal(); break;
    default:
      LERROR("Row number %d out of range 0..3 -- IGNORING", y);
      return false;
    }

  return lcdGoto(offset);
}

// ######################################################################
bool BeoChip::lcdGoto(const int i)
{
  if (i < 0 || i >= 128)
    { LERROR("Offset %d out of range 0..127 -- IGNORING", i); return false; }

  // set DDRAM address:
  return lcdSendRaw(BIN(10000000) | i, false);
}

// ######################################################################
bool BeoChip::lcdPrintf(const char *fmt, ...)
{
  // format and write out our message, truncating at our number of columns:
  char txt[itsLCDcols.getVal() + 1];
  va_list a; va_start(a, fmt);
  vsnprintf(txt, itsLCDcols.getVal()+1, fmt, a);
  va_end(a);

  // send it off to the LCD, char by char:
  bool ret = true;
  pthread_mutex_lock(&serlock);
  for (unsigned int i = 0; i < strlen(txt); i ++)
    ret &= lcdSendRaw(txt[i], true, false);
  pthread_mutex_unlock(&serlock);

  return ret;
}

// ######################################################################
bool BeoChip::lcdPrintf(const int x, const int y, const char *fmt, ...)
{
  // first do a goto:
  if (lcdGotoXY(x, y) == false) return false;

  // format and write out our message, truncating at our number of columns:
  char txt[itsLCDcols.getVal() + 1];
  va_list a; va_start(a, fmt);
  vsnprintf(txt, itsLCDcols.getVal()+1, fmt, a);
  va_end(a);

  return lcdPrintf("%s", txt);
}

// ######################################################################
bool BeoChip::lcdClear()
{
  return lcdSendRaw(BIN(00000001), false);
}

// ######################################################################
bool BeoChip::lcdScrollLeft(const int i)
{
  bool ret = true;

  pthread_mutex_lock(&serlock);
  for (int j = 0; j < i; j ++)
    ret &= lcdSendRaw(BIN(00011000), false, false);
  pthread_mutex_unlock(&serlock);

  return ret;
}

// ######################################################################
bool BeoChip::lcdScrollRight(const int i)
{
  bool ret = true;

  pthread_mutex_lock(&serlock);
  for (int j = 0; j < i; j ++)
    ret &= lcdSendRaw(BIN(00011100), false, false);
  pthread_mutex_unlock(&serlock);

  return ret;
}

// ######################################################################
bool BeoChip::lcdMoveCursorLeft(const int i)
{
  bool ret = true;

  pthread_mutex_lock(&serlock);
  for (int j = 0; j < i; j ++)
    ret &= lcdSendRaw(BIN(00010000), false, false);
  pthread_mutex_unlock(&serlock);

  return ret;
}

// ######################################################################
bool BeoChip::lcdMoveCursorRight(const int i)
{
  bool ret = true;

  pthread_mutex_lock(&serlock);
  for (int j = 0; j < i; j ++)
    ret &= lcdSendRaw(BIN(00010100), false, false);
  pthread_mutex_unlock(&serlock);

  return ret;
}

// ######################################################################
bool BeoChip::lcdCursorBlock()
{ return lcdSendRaw(BIN(00001101), false); }

// ######################################################################
bool BeoChip::lcdCursorUnderline()
{ return lcdSendRaw(BIN(00001110), false); }

// ######################################################################
bool BeoChip::lcdCursorInvisible()
{ return lcdSendRaw(BIN(00001100), false); }

// ######################################################################
bool BeoChip::lcdLoadFont(const int font)
{
  if (font < 0 || font > 7)
    {
      LERROR("font value %d out of range [0..7] - IGNORED", font);
      return false;
    }

  // 11100ccc: Load LCD graphics charset ccc
  return writeByte(BIN(11100000) | byte(font));
}

// ######################################################################
bool BeoChip::lcdLoadFont(const byte data[64])
{
  pthread_mutex_lock(&serlock);

  // get to start of CGRAM:
  if (lcdSendRaw(BIN(01000000), false, false) == false) return false;

  // load up the values:
  bool ret = true;
  for (int i = 0; i < 64; i ++) ret &= lcdSendRaw(data[i], true, false);

  pthread_mutex_unlock(&serlock);

  return ret;
}

// ######################################################################
bool BeoChip::lcdSetAnimation(const int anim)
{
  if (anim < 0 || anim > 7)
    {
      LERROR("anim value %d out of range [0..7] - IGNORED", anim);
      return false;
    }

  // 11101ttt: Set LCD animation type to ttt
  return writeByte((BIN(11101000)) | (byte(anim)));
}

// ######################################################################
bool BeoChip::lcdSendRaw(const byte val, const bool RS, const bool uselock)
{
  // we use the most efficient of either:
  //
  // 00xxxxxx: Send (6-bit) char xxxxxx to LCD, 32 will be added to value
  //
  // or:
  //
  // 101rxxxx: Memorize R/S r and 4 LSB xxxx for later send to LCD
  // 1100xxxx: Combine 4 MSB xxxx to memorized and send to the LCD

  if (RS == true && val >= 32 && val < 96)
    return writeByte(byte(val - 32), uselock); //32 will be re-added by BeoChip
  else
    {
      if (uselock) pthread_mutex_lock(&serlock);
      bool ret = writeByte((BIN(10100000)) |
                           (RS == true ? (BIN(00010000)):0) |
                           (byte(val) & (BIN(00001111))), false);
      ret &= writeByte((BIN(11000000)) | byte(val >> 4), false);
      if (uselock) pthread_mutex_unlock(&serlock);
      return ret;
    }
}

// ######################################################################
bool BeoChip::shimServos(const byte shim)
{
  if (shim > 7)
    {
      LERROR("shim value %d out of range [0..7] - IGNORED", shim);
      return false;
    }

  // 11011ddd: Set servos' delay loop shim to ddd
  return writeByte(BIN(11011000) | shim);
}

// ######################################################################
void BeoChip::calibrateServo(const int servo, const byte neutralval,
                             const byte minval, const byte maxval)
{
  ASSERT(servo >= 0 && servo < NUMSERVO);
  zeroS[servo]->setVal(neutralval);
  minS[servo]->setVal(minval);
  maxS[servo]->setVal(maxval);
}

// ######################################################################
bool BeoChip::setServo(const int servo, const float position)
{
  ASSERT(servo >= 0 && servo < NUMSERVO);
  byte raw = calibToRaw(position, zeroS[servo]->getVal(),
                        minS[servo]->getVal(), maxS[servo]->getVal());
  return setServoRaw(servo, raw);
}

// ######################################################################
float BeoChip::getServo(const int servo) const
{
  ASSERT(servo >= 0 && servo < NUMSERVO);

  byte raw = getServoRaw(servo);
  return rawToCalib(raw, zeroS[servo]->getVal(),
                    minS[servo]->getVal(), maxS[servo]->getVal());
}

// ######################################################################
bool BeoChip::setServoRaw(const int servo, const byte val)
{
  ASSERT(servo >= 0 && servo < NUMSERVO);

  // attempt to set it:
  // 01xxxxxx: Memorize 6 LSB xxxxxx to later send to a servo
  // 100sssxx: Combine 2 MSB xx to memorized and send to servo sss
  pthread_mutex_lock(&serlock);
  bool ret = writeByte(BIN(01000000) | (val & BIN(00111111)), false);
  if (ret) ret = writeByte(BIN(10000000) | (servo << 2) | (val >> 6), false);
  pthread_mutex_unlock(&serlock);

  if (ret == false) { LERROR("Set servo failed - keeping old"); return false; }
  servopos[servo] = val;

  return true;
}

// ######################################################################
byte BeoChip::getServoRaw(const int servo) const
{
  ASSERT(servo >= 0 && servo < NUMSERVO);
  return servopos[servo];
}

// ######################################################################
bool BeoChip::capturePulse(const int whichone, const bool on)
{
  ASSERT(whichone >= 0 && whichone <= 1);

  // 110100px: Turn PWM capture p on/off (x)
  return writeByte(BIN(11010000) | (whichone?2:0) | (on?1:0));
}

// ######################################################################
void BeoChip::calibratePulse(const int whichone, const int neutralval,
                             const int minval, const int maxval)
{
  ASSERT(whichone >= 0 && whichone <= 1);
  zeroP[whichone]->setVal(neutralval);
  minP[whichone]->setVal(minval);
  maxP[whichone]->setVal(maxval);
}

// ######################################################################
float BeoChip::getPulse(const int whichone)
{
  ASSERT(whichone >= 0 && whichone <= 1);
  short int raw = getPulseRaw(whichone);
  return rawToCalib(raw, zeroP[whichone]->getVal(),
                    minP[whichone]->getVal(), maxP[whichone]->getVal());
}

// ######################################################################
short int BeoChip::getPulseRaw(const int whichone)
{
  ASSERT(whichone >= 0 && whichone <= 1);
  pthread_mutex_lock(&lock);
  short int val = pulseval[whichone];
  pthread_mutex_unlock(&lock);
  return val;
}

// ######################################################################
bool BeoChip::captureAnalog(const int whichone, const bool on)
{
  ASSERT(whichone >= 0 && whichone <= 1);

  // 110101ax: Turn A/D capture a on/off (x)
  return writeByte(BIN(11010100) | (whichone?2:0) | (on?1:0));
}

// ######################################################################
void BeoChip::calibrateAnalog(const int whichone, const int neutralval,
                              const int minval, const int maxval)
{
  ASSERT(whichone >= 0 && whichone <= 1);
  zeroA[whichone]->setVal(neutralval);
  minA[whichone]->setVal(minval);
  maxA[whichone]->setVal(maxval);
}

// ######################################################################
float BeoChip::getAnalog(const int whichone)
{
  ASSERT(whichone >= 0 && whichone <= 1);
  byte raw = getAnalogRaw(whichone);
  return rawToCalib(raw, zeroS[whichone]->getVal(),
                    minS[whichone]->getVal(), maxS[whichone]->getVal());
}

// ######################################################################
byte BeoChip::getAnalogRaw(const int whichone)
{
  ASSERT(whichone >= 0 && whichone <= 1);
  pthread_mutex_lock(&lock);
  byte val = adcval[whichone];
  pthread_mutex_unlock(&lock);
  return val;
}

// ######################################################################
bool BeoChip::captureKeyboard(const bool on)
{
  // 1111101x: Turn Keyboard capture on/off (x)
  return writeByte(BIN(11111010) | (on?1:0));
}

// ######################################################################
bool BeoChip::debounceKeyboard(const bool on)
{
  // 1111111x: Turn keyboard debounce on/off (x)
  return writeByte(BIN(11111110) | (on?1:0));
}

// ######################################################################
int BeoChip::getKeyboard()
{
  pthread_mutex_lock(&lock);
  int val = keyboard;
  pthread_mutex_unlock(&lock);
  return val;
}

// ######################################################################
bool BeoChip::setDigitalOut(const int outnum, const bool on)
{
  ASSERT(outnum >= 0 && outnum < 4);
  // 11110oox: Set digital output oo to value x
  return writeByte(BIN(11110000) | (outnum << 1) | (on?1:0));
}

// ######################################################################
bool BeoChip::writeByte(const byte val, const bool uselock)
{
  if (MYLOGVERB >= LOG_DEBUG)
    {
      char txt[9];   txt[8] = '\0';
      for (int i = 0; i < 8; i ++) txt[i] = (val >> (7-i)) & 1 ? '1':'0';
      LDEBUG("Sending: %s", txt);
    }

  if (uselock) pthread_mutex_lock(&serlock);
  bool ret = (write(itsFd, &val, 1) == 1);
  if (uselock) pthread_mutex_unlock(&serlock);

  if (ret == false) PLERROR("Write to BeoChip failed");
  return ret;
}

// ######################################################################
void BeoChip::run()
{
  int pwm[2], adc[2], pwmidx = 0, adcidx = 0;
  while(keepgoing)
    {
      byte c;
      if (read(itsFd, &c, 1) != 1)  // blocking read
        PLERROR("Error reading from BeoChip -- IGNORED");
      else
        {
          if (MYLOGVERB >= LOG_DEBUG)
            {
              char txt[9]; txt[8] = '\0';
              for (int i = 0; i < 8; i ++) txt[i] = (c >> (7-i)) & 1 ? '1':'0';
              LDEBUG("Received: %s", txt);
            }

          // 00pxxxxx: PWM value p ready, 5 LSB attached in xxxxx
          // 01xxxxxx: 6 MSB attached in xxxxxx for last-sent PWM value LSB
          // 100kkkkk: Keyboard changed, current status in kkkkk
          // 101axxxx: A/D value a ready, 4 LSB attached in xxxx
          // 1100xxxx: 4 MSB for last-sent A/D value LSB
          // 11010000: Reset occurred, data may have been lost
          // 11010001: Echo reply (in reply to an echo request)
          // 11010010: Input command serial buffer overflow, data was lost
          // 11010011: Framing/overrun error on serial input, data was lost
          // 11010100: Return value buffer overflow, data was lost (panic mode)

          switch(c >> 6)
            {
            case BIN(00):
              pwmidx = ((c & BIN(00100000)) >> 5);
              pwm[pwmidx] = (c & BIN(00011111));
              break;
            case BIN(01):
              pwm[pwmidx] |= ((c & BIN(00111111)) << 5);
              pthread_mutex_lock(&lock);
              pulseval[pwmidx] = pwm[pwmidx];
              pthread_mutex_unlock(&lock);
              if (itsListener.get())
                itsListener->event((pwmidx == 0 ? PWM0:PWM1),
                                   pwm[pwmidx],
                                   rawToCalib(pwm[pwmidx],
                                              zeroP[pwmidx]->getVal(),
                                              minP[pwmidx]->getVal(),
                                              maxP[pwmidx]->getVal()));
              break;
            case BIN(10):
              if ((c & BIN(11100000)) == BIN(10000000))
                {
                  int kbd = (c & BIN(00011111));
                  pthread_mutex_lock(&lock);
                  keyboard = kbd;
                  pthread_mutex_unlock(&lock);
                  if (itsListener.get())
                    itsListener->event(KBD, kbd, 0.0F);
                }
              else
                {
                  adcidx = ((c & BIN(00010000)) >> 4);
                  adc[adcidx] = (c & BIN(00001111));
                }
              break;
            case BIN(11):
              if ( (c & BIN(11110000)) == BIN(11000000) )
                {
                  adc[adcidx] |= ((c & BIN(00001111)) << 4);
                  pthread_mutex_lock(&lock);
                  adcval[adcidx] = adc[adcidx];
                  pthread_mutex_unlock(&lock);
                  if (itsListener.get())
                    itsListener->event((adcidx == 0 ? ADC0:ADC1),
                                       adc[adcidx],
                                       rawToCalib(adc[adcidx],
                                                  zeroA[adcidx]->getVal(),
                                                  minA[adcidx]->getVal(),
                                                  maxA[adcidx]->getVal()));
                }
              else
                switch(c)
                  {
                  case BIN(11010000):
                    if (itsListener.get())
                      itsListener->event(RESET, 0, 0.0F);
                    break;
                  case BIN(11010001):
                    if (itsListener.get())
                      itsListener->event(ECHOREP, 0, 0.0F);
                    break;
                  case BIN(11010010):
                    if (itsListener.get())
                      itsListener->event(INOVERFLOW, 0, 0.0F);
                    break;
                  case BIN(11010011):
                    if (itsListener.get())
                      itsListener->event(SERIALERROR, 0, 0.0F);
                    break;
                  case BIN(11010100):
                    if (itsListener.get())
                      itsListener->event(OUTOVERFLOW, 0, 0.0F);
                    break;
                  default:
                    LERROR("Unknown message 0x%x from BeoChip -- IGNORED", c);
                  }
            }
        }
    }
  pthread_exit(0);
}

// ######################################################################
float BeoChip::rawToCalib(const int raw, const int zero, const int mini,
                          const int maxi) const
{
  if (raw < mini || raw > maxi)
    LERROR("Raw value %d out of range [%d..%d]", raw, mini, maxi);

  if (raw < zero)
    return float(zero - raw) / float(mini - zero);
  else
    return float(raw - zero) / float(maxi - zero);
}

// ######################################################################
int BeoChip::calibToRaw(const float calibrated, const int zero, const int mini,
                        const int maxi, const int bits) const
{
  if (calibrated < -1.0F || calibrated > 1.0F)
    LERROR("Calibrated value %f out of range [-1..1]", calibrated);

  if (calibrated < 0.0F)
    return zero + int(calibrated * float(zero - mini) + 0.4999F);
  else
    return zero + int(calibrated * float(maxi - zero) + 0.4999F);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
