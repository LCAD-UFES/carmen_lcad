
/*!@file Devices/BeeSTEM.C Interface to Rand Voorhies' BeeSTEM controller*/

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeeSTEM.C $
//
#include "Devices/BeeSTEM.H"
#include "Component/OptionManager.H"
#include "Util/Assert.H"
#include "rutz/compat_snprintf.h"


// ######################################################################
BeeSTEMListener::~BeeSTEMListener()
{ }

// ######################################################################
void *BeeSTEM_run(void *c)
{
        BeeSTEM *d = (BeeSTEM *)c;
        d->run();
        return NULL;
}

// ######################################################################
//BeeSTEM::BeeSTEM(OptionManager& mgr, const std::string& descrName,
//                 const std::string& tagName, const char* dev) :
//  ModelComponent(mgr, descrName, tagName),
//  itsSerial(new Serial(mgr, descrName+" Serial Port", tagName+"SerialPort"))




BeeSTEM::BeeSTEM(OptionManager& mgr, const std::string& descrName,
                const std::string& tagName, const char *dev) :
        ModelComponent(mgr, descrName, tagName),
        itsSerial(new Serial(mgr, descrName+" Serial Port", tagName+"SerialPort")),
        itsKeepgoing(true), itsListener()
{

        //Initialize the serial port
        itsSerial->configure(dev, 38400, "8N1", false, false, 0);
        addSubComponent(itsSerial);

        //Allocate space for the data
        itsData = new byte[DATATOP+1];

        // Initialize all data to zero:
        for (uint i = 0; i <= DATATOP; i ++)
                itsData[i]=0;

        for(uint i=0; i<5; i++)
                itsCurrentMotorValues[i] = 0;

        itsListener.reset(NULL);
        pthread_mutex_init(&lock, NULL);
        pthread_mutex_init(&serlock, NULL);

}

// ######################################################################
void BeeSTEM::start1()
{
        // start our thread:
        itsKeepgoing = true;
        pthread_create(&runner, NULL, &BeeSTEM_run, (void *)this);
}

// ######################################################################
void BeeSTEM::stop2()
{
        itsKeepgoing = false;
        usleep(300000); // make sure thread has exited
}

// ######################################################################
BeeSTEM::~BeeSTEM()
{
        pthread_mutex_destroy(&lock);
        pthread_mutex_destroy(&serlock);
}

// ######################################################################
void BeeSTEM::setListener(rutz::shared_ptr<BeeSTEMListener>& listener)
{ itsListener = listener; }

// ######################################################################
bool BeeSTEM::echoRequest()
{
        // 11111000: Echo request (will send back an echo reply)
        return writeByte(BIN(11111010));
}

// ######################################################################
bool BeeSTEM::debugMode(const bool on)
{
        // 1111110x: Turn debug mode on/off (x)
        return writeByte(BIN(11111100) | (on?1:0));
}

// ######################################################################
bool BeeSTEM::resetChip()
{
        // 11111001: Reset the BeeSTEM
        return writeByte(BIN(11111011));
}


// ######################################################################
bool BeeSTEM::lcdPrintf(const char *fmt, ...)
{

        LERROR("Not Yet Implemented!");
        return false;
        // format and write out our message, truncating at our number of columns:
        /*  char txt[itsLCDcols.getVal() + 1];
                        va_list a; va_start(a, fmt);
                        vsnprintf(txt, itsLCDcols.getVal()+1, fmt, a);
                        va_end(a);

        // send it off to the LCD, char by char:
        bool ret = true;
        pthread_mutex_lock(&serlock);
        for (unsigned int i = 0; i < strlen(txt); i ++)
        ret &= lcdSendRaw(txt[i], true, false);
        pthread_mutex_unlock(&serlock);

        return ret;*/
}

// ######################################################################
bool BeeSTEM::lcdClear()
{
        return writeByte(BIN(11000100), false);
}

bool BeeSTEM::setMotor(const int motor, signed char speed)
{
        if(motor < 0 || motor >4) {
                LERROR("Invalid Motor Number (%d)", motor);
                return false;
        }
        if (speed > 100) speed = 100;
        if (speed < -100) speed = -100;

        if (speed == 0) speed = 1; //hack
        LINFO("speed %i", speed);

        if (abs(itsCurrentMotorValues[motor] - speed) > 60 &&
                        itsCurrentMotorValues[motor]*speed < 0)
        {
                pthread_mutex_lock(&serlock);
                bool ret = writeByte( (0) & BIN(00111111), false);
                if (ret >= 0) ret = writeByte(BIN(01000000) | (motor << 2) | (BIN(00000011) & ((0) >> 6)), false);
                pthread_mutex_unlock(&serlock);
                sleep(1);

        }
        itsCurrentMotorValues[motor] = speed;

        // attempt to set it:
        // 00xxxxxx: Memorize 6 LSB xxxxxx to later send to a motor
        // 010xxxyy: Combine 2 MSB yy to memorized and send to motor xxx
        pthread_mutex_lock(&serlock);
        bool ret = writeByte( (speed/2) & BIN(00111111), false);
        if (ret >= 0) ret = writeByte(BIN(01000000) | (motor << 2) | (BIN(00000011) & ((speed/2) >> 6)), false);
        pthread_mutex_unlock(&serlock);

        if (ret < 0) { LERROR("Set motor failed - keeping old. ret=%d", ret); return false; }

        return true;
}

bool BeeSTEM::toggleCalibrateHMR3300() {
        pthread_mutex_lock(&serlock);
        bool ret = writeByte(BIN(11111110), false);
        pthread_mutex_unlock(&serlock);

        return ret;
}

bool BeeSTEM::levelHMR3300() {
        pthread_mutex_lock(&serlock);
        bool ret = writeByte(BIN(11111111), false);
        pthread_mutex_unlock(&serlock);

        return ret;
}

// ######################################################################

bool BeeSTEM::setReporting(const BeeSTEMReportingMask m, const bool on) {
        return writeByte(m | (on?1:0));
}

// ######################################################################
//! Get Compass Heading
byte BeeSTEM::getCompassHeading() {
        pthread_mutex_lock(&lock);
        byte val = itsData[COMPASS_HEADING_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get Compass Pitch
byte BeeSTEM::getCompassPitch() {
        pthread_mutex_lock(&lock);
        byte val = itsData[COMPASS_PITCH_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get Compass Roll
byte BeeSTEM::getCompassRoll() {
        pthread_mutex_lock(&lock);
        byte val = itsData[COMPASS_ROLL_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get High Speed Accelerometer X
byte BeeSTEM::getAccelX() {
        pthread_mutex_lock(&lock);
        byte val = itsData[ACCEL_X_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get High Speed Accelerometer Y
byte BeeSTEM::getAccelY() {
        pthread_mutex_lock(&lock);
        byte val = itsData[ACCEL_Y_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get Internal Pressure
byte BeeSTEM::getIntPress() {
        pthread_mutex_lock(&lock);
        byte val = itsData[INT_PRESS_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get External Pressure
byte BeeSTEM::getExtPress() {
        pthread_mutex_lock(&lock);
        byte val = itsData[EXT_PRESS_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get Temperature Zone 1
byte BeeSTEM::getTemp1() {
        pthread_mutex_lock(&lock);
        byte val = itsData[TEMP1_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get Temperature Zone 2
byte BeeSTEM::getTemp2() {
        pthread_mutex_lock(&lock);
        byte val = itsData[TEMP2_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get Temperature Zone 3
byte BeeSTEM::getTemp3() {
        pthread_mutex_lock(&lock);
        byte val = itsData[TEMP3_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get current raw Analog value from the spare ADC pin
byte BeeSTEM::getSpareADC() {
        pthread_mutex_lock(&lock);
        byte val = itsData[ADC_IN_EVENT];
        pthread_mutex_unlock(&lock);
        return val;
}

//! Get the current from a motor
/*! 0 for A, ..., 4 for E*/
byte BeeSTEM::getMotorCurr(byte whichone) {
        pthread_mutex_lock(&lock);
        byte val = itsData[MOTOR_A_CURR_EVENT+whichone];
        pthread_mutex_unlock(&lock);
        return val;
}
//! Get the value of a single digital input pin
bool BeeSTEM::getDigitalIn(const int whichone)
{
        ASSERT(whichone >= 0 && whichone < 8);
        return ((itsData[DIG_IN] & (1 << whichone)) != 0);
}

//! Get the whole digital input byte
byte BeeSTEM::getDigitalIn()
{
        return itsData[DIG_IN_EVENT];
}

// ######################################################################
bool BeeSTEM::setDigitalOut(const int outnum, const bool on)
{
        ASSERT(outnum >= 0 && outnum < 8);
        //1110yyyx:        Set digital output yyy to value x
        return writeByte(BIN(11100000) | (outnum << 1) | (on?1:0));
}

// ######################################################################
bool BeeSTEM::writeByte(const byte val, const bool uselock)
{
        if (MYLOGVERB >= LOG_DEBUG)
        {
                char txt[9];   txt[8] = '\0';
                for (int i = 0; i < 8; i ++) txt[i] = (val >> (7-i)) & 1 ? '1':'0';
                LDEBUG("Sending: %s", txt);
        }

        if (uselock) pthread_mutex_lock(&serlock);
        bool ret = (itsSerial->write(&val, 1) <= 0);
        if (uselock) pthread_mutex_unlock(&serlock);

        if (ret) PLERROR("Write to BeeSTEM failed!! -- ret=%d", ret);
        return ret;
}

// ######################################################################
void BeeSTEM::run()
{
        //Because the BeeSTEM data format relies on two seperate bytes to transmit the data and type
        //information, we not only have to keep track of the last bit of data recieved, but also
        //the last command type.
        BeeSTEMEventType last_cmd = NO_EVENT;
        bool last_bit = 0;  //The LSB from the header is the MSB of the data
        bool cmd_found = false;
        byte c;

        while(itsKeepgoing)
        {

                c = itsSerial->read();
                if (MYLOGVERB >= LOG_DEBUG)
                {
                        char txt[9]; txt[8] = '\0';
                        for (int i = 0; i < 8; i ++) txt[i] = (c >> (7-i)) & 1 ? '1':'0';
                        LDEBUG("Received: %s", txt);
                }

                /*
                         BeeSTEM to PC serial command protocol:
                         To avoid framing problems, the following BeeSTEM data is sent in two byte frames.
                         The first (header) byte has an MSB of 0 followed by a 6 bit datacode
                         followed by the MSB of the corresponding data.
                         The second (data) byte has an MSB of 1 followed by the 7 LSBs of the data

0000000x:        Compass Heading
0000001x:        Compass Pitch
0000010x:        Compass Roll
0000011x:        High Speed Accelerometer X
0000100x:        High Speed Accelerometer Y
0000101x:        Internal Pressure
0000110x:        External Pressure
0000111x:        Temperature Zone 1
0001000x:        Temperature Zone 2
0001001x:        Temperature Zone 3
0001010x:        Digital Input
0001011x:        Spare ADC Pin
0010yyyx:        Motor yyy current (A=000,...,E=101)

The following status frames are single byte only:
01110000:        Echo Reply (in reply to an echo request)
01110001:        Reset occured, data may have been lost
01110010:        Input command serial buffer overflow, data was lost (SW Error)
01110011:        Framing error on serial input, data was lost (HW Error)
01110100:        OverRun error on serial input, data was lost (HW Error)
01110101:        Lost communication with HMR3300
01110110:        Lost communication with High Speed Accelerometer
01110111:        Lost communication with Temp1
01111000:        Lost communication with Temp2
01111001:         HMR3300 Leveled
01111010:        EStop has been pulled
01111011:        Invalid / Unrecognized / Unimplemented command -- ignored
01111100:        Bad Command Sequence Recieved -- ignored
01111101:        Reset request acknowledged, attempting reset
01111110:         HMR3300 Calibrating
*/

                //First check to see if the incoming byte is a header or data
                if(c >> 7 == 0) {//If the MSB is a 0, then it is a header
                        last_bit = ( c & (00000001) ); //Grab the LSB off the header, because it will be the MSB of the data

                        //Now, parse the command type

                        //Is it a motor current report?
                        if(c>>4 == BIN(0010)) {
                                switch((c >> 1) & 00000111) {
                                        case BIN(000):
                                                last_cmd = MOTOR_A_CURR_EVENT;
                                                LDEBUG("Got Motor A Current Header");
                                                break;
                                        case BIN(001):
                                                last_cmd = MOTOR_B_CURR_EVENT;
                                                LDEBUG("Got Motor B Current Header");
                                                break;
                                        case BIN(010):
                                                last_cmd = MOTOR_C_CURR_EVENT;
                                                LDEBUG("Got Motor C Current Header");
                                                break;
                                        case BIN(011):
                                                last_cmd = MOTOR_D_CURR_EVENT;
                                                LDEBUG("Got Motor D Current Header");
                                                break;
                                        case BIN(100):
                                                last_cmd = MOTOR_E_CURR_EVENT;
                                                LDEBUG("Got Motor E Current Header");
                                                break;
                                        default:
                                                last_cmd = UNRECOGNIZED_EVENT;
                                                LERROR("Unknown Motor Current Report From BeeSTEM (Motor %d) -- Ignored", (c >> 1) & 00000111);
                                }
                        }
                        else { //If not, then see what it is?
                                cmd_found = false;

                                //First let's check the single byte types (Echo Reply through Invalid Command)
                                switch(c) {
                                        case BIN(01110000): //Echo Reply
                                                LDEBUG("Got Echo Reply");
                                                cmd_found = true;
                                                if (itsListener.get())
                                                        itsListener->event(ECHO_REPLY_EVENT, 0, 0);
                                                break;
                                        case BIN(01110001): //Reset Occured
                                                cmd_found = true;
                                                LDEBUG("Got Reset");
                                                if (itsListener.get())
                                                        itsListener->event(RESET_EVENT, 0, 0);
                                                break;
                                        case BIN(01110010): //Soft Buffer Overflow
                                                cmd_found = true;
                                                LDEBUG("Got Soft Buffer Overflow");
                                                if (itsListener.get())
                                                        itsListener->event(SW_OVERFLOW_EVENT, 0, 0);
                                                break;
                                        case BIN(01110011): //Hard Framing Error
                                                cmd_found = true;
                                                LDEBUG("Got Hard Framing Error");
                                                if (itsListener.get())
                                                        itsListener->event(FRAMING_ERR_EVENT, 0, 0);
                                                break;
                                        case BIN(01110100): //Hard Buffer Overflow
                                                cmd_found = true;
                                                LDEBUG("Got Hard Buffer Overflow");
                                                if (itsListener.get())
                                                        itsListener->event(OVR_ERR_EVENT, 0, 0);
                                                break;
                                        case BIN(01110101): //Lost HMR3300
                                                cmd_found = true;
                                                LDEBUG("Got Lost HMR3300");
                                                if (itsListener.get())
                                                        itsListener->event(HMR3300_LOST_EVENT, 0, 0);
                                                break;
                                        case BIN(01110110): //Lost Accelerometer
                                                cmd_found = true;
                                                LDEBUG("Got Lost Accelerometer");
                                                if (itsListener.get())
                                                        itsListener->event(ACCEL_LOST_EVENT, 0, 0);
                                                break;
                                        case BIN(01110111): //Lost Temperature Sensor 1
                                                cmd_found = true;
                                                LDEBUG("Got Lost Temp1");
                                                if (itsListener.get())
                                                        itsListener->event(TEMP1_LOST_EVENT, 0, 0);
                                                break;
                                        case BIN(01111000): //Lost Temperature Sensor 2
                                                cmd_found = true;
                                                LDEBUG("Got Lost Temp2");
                                                if (itsListener.get())
                                                        itsListener->event(TEMP2_LOST_EVENT, 0, 0);
                                                break;
                                        case BIN(01111001): //The HMR3300 has been leveled
                                                cmd_found = true;
                                                LDEBUG("Got HMR3300 Leveling Complete");
                                                if (itsListener.get())
                                                        itsListener->event(HMR_LEVELED_EVENT, 0, 0);
                                                break;
                                        case BIN(01111010): //ESTOP Was Pulled
                                                cmd_found = true;
                                                LDEBUG("Got ESTOP");
                                                if (itsListener.get())
                                                        itsListener->event(ESTOP_EVENT, 0, 0);
                                                break;
                                        case BIN(01111011): //The command was unrecognized by the BeeSTEM
                                                cmd_found = true;
                                                LDEBUG("Got Unrecognized Command");
                                                if (itsListener.get())
                                                        itsListener->event(UNRECOGNIZED_EVENT, 0, 0);
                                                break;
                                        case BIN(01111100): //The BeeSTEM has recieved a bad command sequence
                                                cmd_found = true;
                                                LDEBUG("Got BeeSTEM Reporting Bad Command Sequence");
                                                if(itsListener.get())
                                                        itsListener->event(BAD_OUT_CMD_SEQ_EVENT, 0, 0);
                                                break;
                                        case BIN(01111101): //The BeeSTEM has acknowledged a reset request, and is attempting to reset
                                                cmd_found = true;
                                                LDEBUG("Got Reset Acknowledge");
                                                if(itsListener.get())
                                                        itsListener->event(RESET_ACK_EVENT, 0, 0);
                                                break;
                                        case BIN(01111110): //The HMR3300 has been put into calibration mode
                                                cmd_found = true;
                                                LDEBUG("Got HMR3300 Calibration Started");
                                                if(itsListener.get())
                                                        itsListener->event(HMR3300_CAL_EVENT, 0, 0);
                                                break;


                                }
                                //If it's not any of the single data bytes, then it must be the header to a data byte
                                if(!cmd_found)
                                        switch(c >> 1)
                                        {
                                                case BIN(0000000):
                                                        LDEBUG("Got Compass Heading Header");
                                                        last_cmd = COMPASS_HEADING_EVENT;
                                                        break;
                                                case BIN(0000001):
                                                        LDEBUG("Got Compass Pitch Header");
                                                        last_cmd = COMPASS_PITCH_EVENT;
                                                        break;
                                                case BIN(0000010):
                                                        LDEBUG("Got Compass Roll Header");
                                                        last_cmd = COMPASS_ROLL_EVENT;
                                                        break;
                                                case BIN(0000011):
                                                        LDEBUG("Got Accelerometer X Header");
                                                        last_cmd = ACCEL_X_EVENT;
                                                        break;
                                                case BIN(0000100):
                                                        LDEBUG("Got Accelerometer Y Header");
                                                        last_cmd = ACCEL_Y_EVENT;
                                                        break;
                                                case BIN(0000101):
                                                        LDEBUG("Got Internal Pressure Header");
                                                        last_cmd = INT_PRESS_EVENT;
                                                        break;
                                                case BIN(0000110):
                                                                LDEBUG("Got External Pressure Header");
                                                                last_cmd = EXT_PRESS_EVENT;
                                                        break;
                                                case BIN(0000111):
                                                        LDEBUG("Got Temperature Zone 1 Header");
                                                        last_cmd = TEMP1_EVENT;
                                                        break;
                                                case BIN(0001000):
                                                        LDEBUG("Got Temperature Zone 2 Header");
                                                        last_cmd = TEMP2_EVENT;
                                                        break;
                                                case BIN(0001001):
                                                        LDEBUG("Got Temperature Zone 3 Header");
                                                        last_cmd = TEMP3_EVENT;
                                                        break;
                                                case BIN(0001010):
                                                        LDEBUG("Got Digital Input Header");
                                                        last_cmd = DIG_IN_EVENT;
                                                        break;
                                                case BIN(0001011):
                                                        LDEBUG("Got ADC Input Header");
                                                        last_cmd = ADC_IN_EVENT;
                                                        break;
                                                default:
                                                        last_cmd = UNRECOGNIZED_EVENT;
                                                        char txt[9];   txt[8] = '\0';
                                                        for (int i = 0; i < 8; i ++) txt[i] = (c >> (7-i)) & 1 ? '1':'0';
                                                        LERROR("Unknown message %s from BeeSTEM -- IGNORED", txt);
                                                        break;
                                        }
                        }
                }
                else if(c >> 7 == 1) {
                        if(last_cmd >=COMPASS_HEADING_EVENT && last_cmd <=MOTOR_E_CURR_EVENT) {//If the MSB is a 1, then it is data

                                byte dat =( (c & BIN(01111111)) | ((last_bit?1:0) << 7));

                                char txt[9]; txt[8] = '\0';
                                for (int i = 0; i < 8; i ++) txt[i] = (dat >> (7-i)) & 1 ? '1':'0';
                                LDEBUG("Data Byte: %s", txt);

                                //Put the data in it's proper place based on the previous command and try to call
                                //the event listener.
                                pthread_mutex_lock(&lock);
                                //I know it's a bit weird to address the data table with event types, but it really cuts down
                                //on code here, and so long as no one messes with the event/data list bindings, this should work great.
                                itsData[last_cmd] = dat;
                                pthread_mutex_unlock(&lock);
                                if (itsListener.get())
                                        itsListener->event(last_cmd, dat, 0);
                                last_cmd = DATA_EVENT;
                        }
                        else {
                                LERROR("Bad Data Ordering From BeeSTEM");
                                if (itsListener.get())
                                        itsListener->event(BAD_IN_CMD_SEQ_EVENT, 0, 0);
                                last_cmd = NO_EVENT;
                        }
                }
        }

        pthread_exit(0);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
