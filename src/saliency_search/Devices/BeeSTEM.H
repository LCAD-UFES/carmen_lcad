/*!@file Devices/BeeSTEM.H Interface to Rand Voorhies' BeeSTEM controller.*/

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
// Primary maintainer for this file: Rand Voorhies <voorhies@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeeSTEM.H $

#ifndef BEESTEM_H_DEFINED
#define BEESTEM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/Serial.H"
#include "Util/Types.H"
#include "rutz/shared_ptr.h"

#include <pthread.h>
#include <stdarg.h>


// Recursive template metaprogramming is used here to allow us to
// define binary constants in the code, as we use these a lot in this
// BeeSTEM program. See http://www.flipcode.com/cgi-bin/msg.cgi?
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

//! BeeSTEM event types
/*!
  This is the EventType that is passed to an external event handler on an event trigger.
  For simplicity, COMPASS_HEADING_EVENT through MOTOR_E_CURR_EVENT are used internally to
  address our data buffer. If any events need to be added later, please just make sure to
  pay attention to how the data buffer size is initialized in the private section of this
  class, and how it is addressed in the accessor functions as well as the run loop.
*/
enum BeeSTEMEventType {
  COMPASS_HEADING_EVENT  = 0,
  COMPASS_PITCH_EVENT    = 1,
  COMPASS_ROLL_EVENT     = 2,
  ACCEL_X_EVENT          = 3,
  ACCEL_Y_EVENT          = 4,
  INT_PRESS_EVENT        = 5,
  EXT_PRESS_EVENT        = 6,
  TEMP1_EVENT            = 7,
  TEMP2_EVENT            = 8,
  TEMP3_EVENT            = 9,
  DIG_IN_EVENT           = 10,
  ADC_IN_EVENT           = 11,
  MOTOR_A_CURR_EVENT     = 12,
  MOTOR_B_CURR_EVENT     = 13,
  MOTOR_C_CURR_EVENT     = 14,
  MOTOR_D_CURR_EVENT     = 15,
  MOTOR_E_CURR_EVENT     = 16,
  ECHO_REPLY_EVENT       = 17,
  RESET_EVENT            = 18,
  SW_OVERFLOW_EVENT      = 19,
  FRAMING_ERR_EVENT      = 20,
  OVR_ERR_EVENT          = 21,
  HMR3300_LOST_EVENT     = 22,
  ACCEL_LOST_EVENT       = 23,
  TEMP1_LOST_EVENT       = 24,
  TEMP2_LOST_EVENT       = 25,
  HMR_LEVELED_EVENT      = 26,
  ESTOP_EVENT            = 27,
  UNRECOGNIZED_EVENT     = 28,
  BAD_IN_CMD_SEQ_EVENT   = 29,
  BAD_OUT_CMD_SEQ_EVENT  = 30,
  RESET_ACK_EVENT        = 31,
  DATA_EVENT             = 32,
  HMR3300_CAL_EVENT      = 33,
  NO_EVENT               = 34
};
#define DATATOP MOTOR_E_CURR_EVENT

//! BeeSTEM Reporting Masks
/*! Quick and dirty way to turn on reporting for a feature or the
  BeeSTEM. Simply pass one of these masks to setReporting with
  a bool to turn reporting for that feature on or off */
enum BeeSTEMReportingMask {
  HMR3300       = BIN(01100010),
  ACCELEROMETER = BIN(01100000),
  INT_PRESS     = BIN(01101010),
  EXT_PRESS     = BIN(01101100),
  TEMP1         = BIN(01100100),
  TEMP2         = BIN(01100110),
  TEMP3         = BIN(01101000),
  DIG_IN        = BIN(01101110),
  ADC_IN        = BIN(01110000),
  MOTOR_CURR    = BIN(01110010)
};


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
class BeeSTEMListener {
public:
  //! Destructor
  virtual ~BeeSTEMListener();

  //! An event was received
  /*! This function will be called by the BeeSTEM once you have
    registered your BeeSTEMListener derivative with the BeeSTEM using
    BeeSTEM::setListener(). Beware that the call to event() will be
    from a thread running in the BeeSTEM and thus will operate in
    parallel with your main thread and main loop. See test-BeeSTEM.C
    for an example of how to synchronize a main loop with these
    asynchronous event calls. The following data will be passed along
    with the event type:
    <PRE>
    _type______________________dat1__________________dat2_
    COMPASS_HEADING             heading               0
    COMPASS_PITCH               pitch                 0
    COMPASS_ROLL                roll                  0
    ACCEL_X                     x-acceleration        0
    ACCEL_Y                     y-acceleration        0
    INT_PRESS                   internal pressure     0
    EXT_PRESS                   external pressure     0
    TEMP1                       temperature 1         0
    TEMP2                       temperature 2         0
    TEMP3                       temperature 3         0
    DIG_IN                      digital input byte    0
    ADC_IN                      spare adc value       0
    MOTOR_CURR                  current               motor number
    All Others                  0                     0
    </PRE>
  */
  virtual void event(const BeeSTEMEventType t, const unsigned char dat1,
                     const unsigned char dat2) = 0;
};

//! BeeSTEM.H Interface to Rand Voorhies' BeeSTEM device
/*! Hardware interface Class. Contains interfaces to display to an
LCD, a 2D Accelerometer/Compass, and extra high speed accelerometer,
3 temperature sensors, internal & external pressure sensors, 8 digital I/O pins
an ADC pin, 5 PWM outputs, and motor current readings. */
class BeeSTEM : public ModelComponent
{
public:
  // ############################################################
  /*! @name Constructors, destructors and debugging */
  //@{

  //! Default constructor. See ModelComponent.H
  BeeSTEM(OptionManager& mgr,
          const std::string& descrName = "BeeSTEM",
          const std::string& tagName = "BeeSTEM",
          const char* dev = "/dev/ttyS0");

  //! Destructor
  ~BeeSTEM();

  //! Install a callback (listener) for BeeSTEM events
  /*! This callback will be called with the corresponding
    BeeSTEMEventType each time an event is received from the BeeSTEM. */
  void setListener(rutz::shared_ptr<BeeSTEMListener>& listener);

  //! Send an echo request
  /*! The BeeSTEM should reply with an Echo_Reply event if it is alive */
  bool echoRequest();

  //! Turn debug mode on/off
  /*! When in debug mode, the BeeSTEM sends an EchoReply for each command
    it receives. */
  bool debugMode(const bool on);

  //! Reset the BeeSTEM
  bool resetChip();

  //! Toggle the calibration mode of the HMR3300
  bool toggleCalibrateHMR3300();

  //! Level the HMR3300
  bool levelHMR3300();

  //@}

  // ############################################################
  /*! @name LCD functions */
  //@{

  //! Print some text
  /*! syntax is the same as printf(). Returns true on success. */
  bool lcdPrintf(const char *fmt, ...)
    // NOTE: this __attribute__ tells gcc that it should issue
    // printf-style warnings when compiling calls to
    // BeeSTEM::lcdPrintf(), treating the 2nd argument (fmt) as the
    // format string, and the 3rd and subsequent arguments as the
    // printf-style parameters (SUBNOTE: because this is a member
    // function, there is a hidden 'this' parameter that counts as arg
    // 1, so the listed arguments are counted starting from 2)
    __attribute__((format(__printf__, 2, 3)));
    ;

  //! Clear LCD screen
  bool lcdClear();
  //@}

  // ############################################################
  /*! @name PWM control functions */
  //@{

  //! Sets the speed of a motor. -100 is full reverse, and +100 is full forwards
  /*! Note that the [-100..+100] is only a percentage of the maximum possible
      PWM allowed by the BeeSTEM. See MotorControl.H of the BeeSTEM code for more. */
  bool setMotor(const int motor, signed char speed);

  //! Gets the current current consumption of a motor
  /*! Note that motor current monitoring must be explicitely turned on*/
  float getMotorCurrent(const int motor) const;

  //@}

  // ############################################################
  /*! @name Data aquisition and reporting functions */
  //! Tell the BeeSTEM to start (or stop) sending a data type over the serial connection.
  /*!   This function takes in a BeeSTEMReportingMask and a bool, and tells the BeeSTEM
      to start or stop monitoring and reporting a resource designated by the mask.
        Note that turning on too many reporting features may significantly slow down
      the device, so please only turn on what's needed.*/
  bool setReporting(const BeeSTEMReportingMask t, const bool on);


  //@}

  // ############################################################
  /*! @name Data Acquisition Functions */
  //@{

  //! Get Compass Heading
  byte getCompassHeading();

  //! Get Compass Pitch
  byte getCompassPitch();

  //! Get Compass Pitch
  byte getCompassRoll();

  //! Get High Speed Accelerometer X
  byte getAccelX();

  //! Get High Speed Accelerometer Y
  byte getAccelY();

  //! Get Internal Pressure
  byte getIntPress();

  //! Get External Pressure
  byte getExtPress();

  //! Get Temperature Zone 1
  byte getTemp1();

  //! Get Temperature Zone 2
  byte getTemp2();

  //! Get Temperature Zone 3
  byte getTemp3();

  //! Get current raw Analog value from the spare ADC pin
  byte getSpareADC();

  //! Get the current from a motor
  byte getMotorCurr(byte whichone);

  //! Get the value of a single digital input pin
  bool getDigitalIn(const int whichone);

  //! Get the value of all 8 digital input pins as a whole byte
  byte getDigitalIn();


  // ############################################################
  /*! @name Digital output function */
  //@{

  //! Turn a given digital output on/off
  /*! Valid range for outnum is [0..7] */
  bool setDigitalOut(const int outnum, const bool on);

  //@}

  //! This is our main running thread - don't call directly
  /*! Should have been protected, but is not because of pthread hack. */
  void run();

  //! send a byte to the BeeSTEM
  /*! You should never have to use this, use the other functions
    instead.  This function is made public just for testing the chip's
    robustness against a flood of random junk hitting it... See
    test-BeeSTEM.C */
  bool writeByte(const byte val, const bool uselock = true);

protected:
  //! open the port and get started
  void start1();

  //! close the port and get stopped
  void stop2();

private:

  nub::soft_ref<Serial> itsSerial;
  bool itsKeepgoing;

  rutz::shared_ptr<BeeSTEMListener> itsListener;

  pthread_t runner;
  pthread_mutex_t lock, serlock;

  //!Buffer for incoming data from the BeeSTEM
  byte* itsData;

  int itsCurrentMotorValues[5];

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
