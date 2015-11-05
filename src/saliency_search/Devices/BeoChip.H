/*!@file Devices/BeoChip.H Interface to Brian Hudson's BeoChip interface device.*/

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeoChip.H $
// $Id: BeoChip.H 6990 2006-08-11 18:13:51Z rjpeters $
//

#ifndef BEOCHIP_H_DEFINED
#define BEOCHIP_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/Serial.H"
#include "Util/Types.H"
#include "rutz/shared_ptr.h"

#include <pthread.h>
#include <stdarg.h>

//! BeoChip event types
enum BeoChipEventType {
  NONE = 0, PWM0 = 1, PWM1 = 2, KBD = 3, ADC0 = 4, ADC1 = 5,
  RESET = 6, ECHOREP = 7,
  INOVERFLOW = 8, SERIALERROR = 9, OUTOVERFLOW = 10 };

//! BeoChip event listener
/*! The BeoChipListener gets called each time an event is received
  from the BeoChip. The BeoChipEventType is passed to the listener,
  allowing the user to determine the type of event received and to
  then interpret the event data also passed to the listener. By
  default, there is no listener, and users can just asynchronously
  query the BeoChip for its current internal state. If those queries
  come in too slowly, however, they may miss some events (e.g., a
  briefly pressed key). So, production code should define a derived
  class for BeoChipListener and register it with the BeoChip object,
  so that it will take action as soon as an event is received. */
class BeoChipListener {
public:
  //! Destructor
  virtual ~BeoChipListener();

  //! An event was received
  /*! This function will be called by the BeoChip once you have
    registered your BeoChipListener derivative with the BeoChip using
    BeoChip::setListener(). Beware that the call to event() will ve
    from a thread running in the BeoChip and thus will operate in
    parallel with your main thread and main loop. See test-BeoChip.C
    for an example of how to synchronize a main loop with these
    asynchronous event calls. The following data will be passed along
    with the event type:
    <PRE>

    Type            valint                        valfloat
    ----------------------------------------------------------------------
    NONE         0                             0.0F
    PWM0         pulse 0 width, 1.6us units    calibrated width in [-1..1]
    PWM1         pulse 1 width, 1.6us units    calibrated width in [-1..1]
    KBD          5 LSB show the 5 keys         0.0F
    ADC0         A/D 0 value                   calibrated value in [-1..1]
    ADC1         A/D 0 value                   calibrated value in [-1..1]
    all others   0                             0.0F

    </PRE>
  */
  virtual void event(const BeoChipEventType t, const int valint,
                     const float valfloat) = 0;
};

//! BeoChip.H Interface to Brian Hudson's BeoChip device
/*! Hardware interface Class. Contains interfaces to display to an
LCD, read servo pulse values (2), set servo positions (8), get
A/D conversion values (2), manipulate four digital outs,
and finally, get some intput from 5 pushbuttons. */
class BeoChip : public ModelComponent
{
public:
  // ############################################################
  /*! @name Constructors, destructors and debugging */
  //@{

  //! Default constructor. See ModelComponent.H
  BeoChip(OptionManager& mgr,
          const std::string& descrName = "BeoChip",
          const std::string& tagName = "BeoChip");

  //! Destructor
  ~BeoChip();

  //! Install a callback (listener) for BeoChip events
  /*! This callback will be called with the corresponding
    BeoChipEvenType each time an event is received from the BeoChip. */
  void setListener(rutz::shared_ptr<BeoChipListener>& listener);

  //! Send an echo request
  /*! The BeoChip should reply with an EchoReply event if it is alive */
  bool echoRequest();

  //! Turn debug mode on/off
  /*! When in debug mode, the BeoChip sends an EchoReply for each command
    it receives. */
  bool debugMode(const bool on);

  //! Reset the BeoChip
  bool resetChip();

  //@}

  // ############################################################
  /*! @name LCD functions */
  //@{

  //! Go to given cursor position
  bool lcdGotoXY(const int x = 0, const int y = 0);

  //! Go to given cursor position
  bool lcdGoto(const int i = 0);

  //! Print some text
  /*! syntax is the same as printf(). Returns true on success. */
  bool lcdPrintf(const char *fmt, ...)
    // NOTE: this __attribute__ tells gcc that it should issue
    // printf-style warnings when compiling calls to
    // BeoChip::lcdPrintf(), treating the 2nd argument (fmt) as the
    // format string, and the 3rd and subsequent arguments as the
    // printf-style parameters (SUBNOTE: because this is a member
    // function, there is a hidden 'this' parameter that counts as arg
    // 1, so the listed arguments are counted starting from 2)
    __attribute__((format(__printf__, 2, 3)));
    ;

  //! Print some text starting at given cursor position
  /*! syntax is the same as printf(), plus the x, y
    coordinates. Returns true on success. */
  bool lcdPrintf(const int x, const int y, const char *fmt, ...)
    // NOTE: this __attribute__ tells gcc that it should issue
    // printf-style warnings when compiling calls to
    // BeoChip::lcdPrintf(), treating the 3rd argument (fmt) as the
    // format string, and the 4th and subsequent arguments as the
    // printf-style parameters (SUBNOTE: because this is a member
    // function, there is a hidden 'this' parameter that counts as arg
    // 1, so the listed arguments are counted starting from 2)
    __attribute__((format(__printf__, 4, 5)));
    ;

  //! Clear LCD screen
  bool lcdClear();

  //! Scroll display left by i positions
  bool lcdScrollLeft(const int i = 1);

  //! Scroll display right by i positions
  bool lcdScrollRight(const int i = 1);

  //! Move cursor left by i positions
  bool lcdMoveCursorLeft(const int i = 1);

  //! Move cursor right by i positions
  bool lcdMoveCursorRight(const int i = 1);

  //! Make cursor a blinking block
  bool lcdCursorBlock();

  //! Make cursor a blinking underline
  bool lcdCursorUnderline();

  //! Make cursor invisible (not recommended)
  bool lcdCursorInvisible();

  //! Load one of the pre-programmed custom fonts
  /*! Valid font values are [0..7]. Font 0 is an all-blank font. */
  bool lcdLoadFont(const int font);

  //! Load a new font from an array of 64 bytes
  bool lcdLoadFont(const byte data[64]);

  //! Select an LCD animation and start it
  /*! Valid anim values are [0..7]. Anim 0 is no animation. */
  bool lcdSetAnimation(const int anim = 0);

  //! Send a raw byte to the LCD
  bool lcdSendRaw(const byte val, const bool RS, const bool uselock = true);

  //@}

  // ############################################################
  /*! @name Servo control functions */
  //@{

  //! Shim the BeoChip's servo pulses
  /*! Valid shim values are [0..7]. The default at bootup of the
    BeoChip is 4. Higher values will increase pulse length and lower
    values will decrease them. This may allow you to shim the pulses
    so as to maximally exploit the range of your servos. */
  bool shimServos(const byte shim);

  //! Calibrate a servo
  /*! Calibration will be made so that setServo(servo, 0.0F) will send the
    value 'neutralval' to the servo, setServo(servo, -1.0F) will send
    minval and setServo(servo, 1.0F) will send maxval. */
  void calibrateServo(const int servo, const byte neutralval,
                      const byte minval, const byte maxval);

  //! Moves servo # to given position in [-1.0 .. 1.0]
  /*! Returns true on success, false if some serial error occurred. */
  bool setServo(const int servo, const float position);

  //! Gets the current position of given servo
  float getServo(const int servo) const;

  //! Sets servo number servo to value val
  bool setServoRaw(const int servo, const byte val);

  //! Gets the current raw position of given servo
  byte getServoRaw(const int servo) const;

  //@}

  // ############################################################
  /*! @name Pulse acquisition functions */
  //@{

  //! Turn pulse captures on/off
  bool capturePulse(const int whichone, const bool on);

  //! Calibrate a pulse
  /*! Calibration will be made so that getPulse() returns 0.0F when
    raw pulse width is 'neutralval', returns -1.0F when the raw pulse
    value is minval and returns 1.0F when the raw pulse value is
    maxval. */
  void calibratePulse(const int whichone, const int neutralval,
                      const int minval, const int maxval);

  //! Get current pulse value, between [-1.0..1.0]
  float getPulse(const int whichone);

  //! Get current pulse value, raw uncalibrated
  short int getPulseRaw(const int whichone);

  //@}

  // ############################################################
  /*! @name Analog acquisition functions */
  //@{

  //! Turn A/D captures on/off
  bool captureAnalog(const int whichone, const bool on);

  //! Calibrate an A/D converter
  /*! Calibration will be made so that getAnalog() returns 0.0F when
    raw value width is 'neutralval', returns -1.0F when the raw value
    is minval and returns 1.0F when the raw value is maxval. */
  void calibrateAnalog(const int whichone, const int neutralval,
                       const int minval, const int maxval);

  //! Get current calibrated Analog value
  float getAnalog(const int whichone);

  //! Get current raw Analog value
  byte getAnalogRaw(const int whichone);

  //@}

  // ############################################################
  /*! @name Keyboard acquisition functions */
  //@{

  //! Turn keyboard capture on/off
  bool captureKeyboard(const bool on);

  //! Turn keyboard debouncing on/off
  bool debounceKeyboard(const bool on);

  //! Get current Keyboard value
  int getKeyboard();

  //@}

  // ############################################################
  /*! @name Digital output function */
  //@{

  //! Turn a given digital output on/off
  /*! Valid range for outnum is [0..3] */
  bool setDigitalOut(const int outnum, const bool on);

  //@}

  //! This is our main running thread - don't call directly
  /*! Should have been protected, but is not because of pthread hack. */
  void run();

  //! send a byte to the BeoChip
  /*! You should never have to use this, use the other functions
    instead.  This function is made public just for testing the chip's
    robustness against a flood of random junk hitting it... See
    test-BeoChip.C */
  bool writeByte(const byte val, const bool uselock = true);

protected:
  NModelParam<std::string> itsDevName; //!< name of our serial device
  NModelParam<int> itsLCDrows; //!< number of rows of LCD screen (lines of text)
  NModelParam<int> itsLCDcols; //!< number of columns of LCD screen

  rutz::shared_ptr<NModelParam<int> >* zeroS; //!< zero calibration value, for 0.0F (servo)
  rutz::shared_ptr<NModelParam<int> >* minS;  //!< minimum raw value, for -1.0F (servo)
  rutz::shared_ptr<NModelParam<int> >* maxS;  //!< maximum raw value, for 1.0F (servo)
  byte *servopos;                      //!< raw servo positions [0..255]

  rutz::shared_ptr<NModelParam<int> >* zeroP; //!< zero calibration value, for 0.0F (pulse)
  rutz::shared_ptr<NModelParam<int> >* minP;  //!< minimum raw value, for -1.0F (pulse)
  rutz::shared_ptr<NModelParam<int> >* maxP;  //!< maximum raw value, for 1.0F (pulse)
  short int *pulseval;                 //!< raw pulse values (11bit int)

  rutz::shared_ptr<NModelParam<int> >* zeroA; //!< zero calibration value, for 0.0F (pulse)
  rutz::shared_ptr<NModelParam<int> >* minA;  //!< minimum raw value, for -1.0F (servo)
  rutz::shared_ptr<NModelParam<int> >* maxA;  //!< maximum raw value, for 1.0F (servo)

  NModelParam<bool> itsUseRTSCTS;      //!< Use RTS/CTS flow control

  byte *adcval;                        //!< raw adc values [0..255]

  byte keyboard;                       //!< our current keyboard state

  //! Convert from raw (int) to calibrated (-1.0..1.0) position
  float rawToCalib(const int raw, const int zero, const int mini,
                   const int maxi) const;

  //! Convert from calibrated (-1.0..1.0) to raw (int) position
  int calibToRaw(const float calibrated, const int zero, const int mini,
                 const int maxi, const int bits = 8) const;

  //! open the port and get started
  void start1();

  //! close the port and get stopped
  void stop2();

private:
  struct termios itsOldtio;  //!< our old terminal io settings
  int itsFd;                 //!< our serial port file descriptor
  rutz::shared_ptr<BeoChipListener> itsListener;

  pthread_t runner;
  pthread_mutex_t lock, serlock;
  bool keepgoing;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
