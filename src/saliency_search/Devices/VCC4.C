/*!@file Devices/VCC4.C class for interfacing with the pan-tilt mechanism of a */
//          Canon VC-C4 camera
//
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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/VCC4.C $
// $Id: VCC4.C 6853 2006-07-18 17:20:09Z ibeo $
//

#include "Devices/VCC4.H"
#include "Component/OptionManager.H"
#include "Devices/DeviceOpts.H"
#include "Util/log.H"

#include <cmath>
#include <string.h>

// ######################################################################
VCC4::VCC4(OptionManager& mgr, const std::string& descrName,
           const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr)),
  itsSerialDevice(&OPT_VCC4serialDevice, this),
  itsUnitNo(&OPT_VCC4unitNo, this),
  itsPulseRatio(&OPT_VCC4pulseRatio, this), // was 0.1125F
  itsMaxAngle(&OPT_VCC4maxAngle, this) // was 100.0125F
{
  // adopt our serial port as a subcomponent:
  addSubComponent(itsPort);
}

// ######################################################################
VCC4::~VCC4(void)
{ }

// ######################################################################
void VCC4::start1()
{
  // configure our serial port:
  itsPort->configure(itsSerialDevice.getVal().c_str(), 9600, "8N1", false, true, 1);
}

// ######################################################################
void VCC4::stop1()
{
  PlainCommand (VCC4_EnterLocalMode);
}

// ######################################################################
int VCC4::CameraInitialize (bool goHome)
{
  int err, init_cmd;

  // switch to host control mode and switch camera power on
  PlainCommand (VCC4_EnterHostMode);
  PlainCommand (VCC4_SetCameraPowerON);

  // initialize camera as soon as Power On has finished
  if (goHome) init_cmd = VCC4_PTinit1;
  else init_cmd = VCC4_PTinit2;

  while (true)
    {
      err = PlainCommand (init_cmd, false);
      if ((err & VCC4_MODE_ERR) == 0) break;
      // try again after 10 ms
      usleep (10000);
    }

  // retrieve the pulse ratio if it is unconfigured
  if (itsPulseRatio.getVal() == 0.0F) {
    err = PlainCommand (VCC4_GetPanGRatio);
    if (err != 0) LERROR("Error in VCC4_GetPanGRatio");
    else itsPulseRatio.setVal((float)getAbsNum (4) / 1.0e5F);
  }

  // retrieve the maximum pan angle if it is unconfigured
  if (itsMaxAngle.getVal() == 0.0F) {
    err = PlainCommand (VCC4_GetAngleRatio2);
    if (err != 0)
      {
        LERROR("Error in VCC4_GetAngleRatio2");
      }
    else
      {
        float ang = 0.0F;
        err = getOneAngle(ang); itsMaxAngle.setVal(ang);
        if (err !=0) LERROR("Error in getOneAngle");
      }
  }

  // retrieve unit name (i.e. camera name)
  PlainCommand (VCC4_GetUnitName);
  return (err);
}


// ######################################################################
int VCC4::gotoPosition(float pan, float tilt, bool wait)
{
  int err;
  float max = 0.0F, min = 0.0F;

  PlainCommand (VCC4_GetPTAngle0);
  getTwoAngles (min, max);
  if (pan < min)
    {
      LERROR("VCC4: pan angle too small: %f - go to minimum angle %f instead",
             pan, min);
      pan = min;
    }
  if (pan > max)
    {
      LERROR("VCC4: pan angle too large: %f - go to maximum angle %f instead",
             pan, max);
      pan = max;
    }

  PlainCommand (VCC4_GetPTAngle1);
  getTwoAngles (min, max);
  if (tilt < min)
    {
      LERROR("VCC4: tilt angle too small: %f - go to minimum angle %f instead",
             tilt, min);
      tilt = min;
    }

  if (tilt > max)
    {
      LERROR("VCC4: tilt angle too large: %f - go to maximum angle %f instead",
             tilt, max);
      tilt = max;
    }

  err = AngleCommand (VCC4_SetPTAngle2, pan, tilt);
  err = err | getPosition (pan, tilt, wait);
  return err;
}



// ######################################################################
int VCC4::getPosition (float& pan, float& tilt, bool wait)
{
  int err;
  err = PlainCommand (VCC4_GetPTAngle2, wait);
  getTwoAngles (pan, tilt);
  return err;
}


// ######################################################################
int VCC4::PlainCommand (int cmdno, bool wait)
{
  int i = getIndex (cmdno);
  if (i == VCC4_INVALID_COMMAND) return i;
  if (command[i].cmdtyp != VCC4_EMBED_NONE) return (VCC4_INVALID_COMMAND);

  if (wait) return WaitRawCommand (cmdno);
  else return SendRawCommand (cmdno);
}


// ######################################################################
int VCC4::AbsNumCommand (int cmdno, unsigned int value, bool wait)
{
  int i, err;
  i = getIndex (cmdno);
  if (i == VCC4_INVALID_COMMAND) return i;
  if (command[i].cmdtyp != VCC4_EMBED_NUM) return (VCC4_INVALID_COMMAND);

  err = intToHexString (value, &(paramString[0]), command[i].cmdparamlen);
  if (err != command[i].cmdparamlen) return VCC4_INVALID_NUMBER;

  if (wait) return WaitRawCommand (cmdno, &(paramString[0]));
  else return SendRawCommand (cmdno, &(paramString[0]));
}


// ######################################################################
int VCC4::getOneAngle (float& angle)
{
  int val;
  val = hexStringToInt (&(ReplyString[0]), 4);
  if (val < 0) return val;

  angle = (float)(val - 0x8000) * itsPulseRatio.getVal();
  return VCC4_SUCCESS;
}


// ######################################################################
int VCC4::getTwoAngles (float& angle1, float& angle2)
{
  int val1, val2;
  val1 = hexStringToInt (&(ReplyString[0]), 4);
  val2 = hexStringToInt (&(ReplyString[4]), 4);
  if (val1 < 0) return val1;
  if (val2 < 0) return val2;

  angle1 = (float)(val1 - 0x8000) * itsPulseRatio.getVal();
  angle2 = (float)(val2 - 0x8000) * itsPulseRatio.getVal();

  return VCC4_SUCCESS;
}


// ######################################################################
int VCC4::AngleCommand (int cmdno, float angle1, float angle2, bool wait)
{
  unsigned int code1, code2;

  int i = getIndex (cmdno);
  if (i == VCC4_INVALID_COMMAND) return i;
  if (command[i].cmdtyp != VCC4_EMBED_ANG) return (VCC4_INVALID_COMMAND);

  if ((fabs(angle1) > itsMaxAngle.getVal()) ||
      (fabs(angle2) > itsMaxAngle.getVal()))
    return VCC4_INVALID_ANGLE;

  code1 = (unsigned int)((int)(angle1 / itsPulseRatio.getVal()) + 0x8000);
  code2 = (unsigned int)((int)(angle2 / itsPulseRatio.getVal()) + 0x8000);

  intToHexString (code1, &(paramString[0]), 4);
  intToHexString (code2, &(paramString[4]), 4);

  if (wait) return WaitRawCommand (cmdno, &(paramString[0]));
  else return SendRawCommand (cmdno, &(paramString[0]));
}


// ######################################################################
int VCC4::StringCommand (int cmdno, char* string, bool wait)
{
  int i = getIndex (cmdno);
  if (i == VCC4_INVALID_COMMAND) return i;
  if (command[i].cmdtyp != VCC4_EMBED_STR) return (VCC4_INVALID_COMMAND);

  if (wait) return WaitRawCommand (cmdno, string);
  else return SendRawCommand (cmdno, string);
}



// ######################################################################
// transforms an unsigned int to a string of hex digits
// returns length if everything is okay
// returns VCC4_BUF_OVERFLOW if the buffer paramString is too small
// returns VCC4_NUM_OVERFLOW if the number is too big to fit the string
//
int VCC4::intToHexString (unsigned int value, char* buf, unsigned int length)
{
  int i, j;
  unsigned int p;

  for ((i = length-1, j = 0); i >= 0; (i--, j++))
    {
      p = ((0xF << (4*i)) & value) >> (4*i);
      buf[j] = (p < 0xA) ? ('0' + p):('A' + p - 0xA);
    }

  if ((value >> (4*length)) != 0)
    return VCC4_NUM_OVERFLOW;
  else
    return length;
}


// ######################################################################
int VCC4::hexStringToInt (char* buf, unsigned int length)
{
  int i, p;
  unsigned int val = 0;

  for (i = 0; i < (int)length; i++)
    {
      if (buf[i] == '\x00') return VCC4_BUF_UNDERFLOW;
      p = (int)((buf[i] <= '9') ? (buf[i] - '0'):(buf[i] - 'A' + 0xA));
      val = (val << 4) | (p & 0xF);
    }
  return val;
}


// ######################################################################
int VCC4::WaitRawCommand (int cmdno, char* param)
{
  int err;
  while (true)
    {
      err = SendRawCommand (cmdno, param);
      if ((err & VCC4_BUSY_ERR) == 0) break;
      if ((err & VCC4_MODE_ERR) != 0) break;
      // try again after 10 ms
      usleep (10000);
    }
  return err;
}


// ######################################################################
int VCC4::SendRawCommand (int cmdno, char* param)
{
  char cmd_string[30];

  int i = getIndex (cmdno);
  if (i == VCC4_INVALID_COMMAND)
    {
      LERROR("VCC4::SendRawCommand - invalid command number: %i", cmdno);
      return (i);
    }

  // found - prepare command string
  bzero (cmd_string, sizeof(cmd_string));
  memcpy (cmd_string, command[i].cmdstr, command[i].cmdlen);

  // copy unit number in
  cmd_string[2] = '0' + (char)(itsUnitNo.getVal());

  // if we have to include parameters, include them
  if (command[i].cmdparamlen > 0)
    {
      if (param == NULL)
        {
          LERROR ("VCC4::SendRawCommand - invalid parameter pointer");
          return (VCC4_INVALID_PARAM);
        }

      // now copy the parameters over
      memcpy (&(cmd_string[command[i].cmdparamstart - 1]), param,
              command[i].cmdparamlen);
    }

  // send the command string to the port
  if (!itsPort->write (cmd_string, command[i].cmdlen))
    {
      LERROR("VCC4::SendRawCommand - error writing to the device");
      return (VCC4_IO_ERR);
    }
  return (getReply());
}



// ######################################################################
// retrieve reply
int VCC4::getReply (void)
{
  char rep_string[30];
  bool endfound = false;
  int err, j;

  bzero (rep_string, sizeof(rep_string));
  bzero (ReplyString, sizeof(ReplyString));

  for (j = 0; ((j < (int)sizeof(rep_string)) && !endfound); j++)
    {
      rep_string[j] = itsPort->read();
      endfound = (rep_string[j] == '\xEF');
    }

  if (!endfound)
    {
      LERROR("VCC4::getReply - reply string overflow");
      return (VCC4_REPLY_ERR);
    }

  if (j < 6)
    {
      LERROR("VCC4::getReply - reply string too short: %i", j);
      return (VCC4_REPLY_ERR);
    }

  // return values?
  if (j > 6) memcpy (ReplyString, &(rep_string[5]), (j-6));

  // get errors from reply string
  err = (((int)(rep_string[3] - '0') & 0xF) << 12)
      | (((int)(rep_string[4] - '0') & 0xF) <<  8);

  return (err);
}


// ######################################################################
int VCC4::getIndex (int cmdno)
{
  int i;
  bool found = false;

  for (i = 0; ((i < VCC4_CMDMAX) && !found); i++)
    if (command[i].cmdno == cmdno) found = true;
  i--;
  if (!found) i = VCC4_INVALID_COMMAND;
  return (i);
}



// ######################################################################
/*
 * This array contains all the command codes for controlling the
 * Canon VC-C4 camera's pan and tilt mechanism
 * The structure VC4CMD is defined in vcc4cmddef.h
 *
 *  This array is adapted from the Software Developer's Kit vcc4sdk
 *  provided by Canon for Microsoft Windows programming,
 *  extended by the fourth  and fifth numbers (command[i].cmdparamlen
 *  and command[i].paramstart), which give the length and the start
 *  postion of the embedded parameter section in bytes.
 */
VCC4CMD VCC4::command[VCC4_CMDMAX] = {
{2,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA0','\x30','\xEF'}},// set : power off
{3,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA0','\x31','\xEF'}},// set : power on
{4,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA1','\x30','\xEF'}},// set : focus mode AF
{5,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA1','\x31','\xEF'}},// set : focus mode manual
{6,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA1','\x32','\xEF'}},// set : focus near
{7,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA1','\x33','\xEF'}},// set : focus far
{8,1,10,4,6,{'\xFF','\x30','\x30','\x00','\xB0','*','*','*','*','\xEF'}},// set : focus position
{9,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xB1','\x30','\xEF'}},// request : focus position
{10,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xB1','\x31','\xEF'}},// set : onepush AF
{11,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA2','\x30','\xEF'}},// set : zooming stop
{12,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA2','\x31','\xEF'}},// set : zooming wide
{13,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA2','\x32','\xEF'}},// set : zooming tele
{14,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA2','\x33','\xEF'}},// set : high zooming wide
{15,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA2','\x34','\xEF'}},// set : high zooming tele
{16,1,8,2,6,{'\xFF','\x30','\x30','\x00','\xA3','*','*','\xEF'}},// set : zooming position1
{17,0,6,0,6,{'\xFF','\x30','\x30','\x00','\xA4','\xEF'} },// request : zooming position1
{18,1,10,4,6,{'\xFF','\x30','\x30','\x00','\xB3','*','*','*','*','\xEF'}},// set : zooming position2
{19,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xB4','\x30','\xEF'}},// request : zooming position2
{20,1,8,1,6,{'\xFF','\x30','\x30','\x00','\xB4','\x31','*','\xEF'}},// set : zooming speed
{21,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xB4','\x32','\xEF'}},// request : zooming speed
{22,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x30','\xEF'}},// set : backlight off
{23,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x31','\xEF'}},// set : backlight on
{24,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x32','\xEF'}},// set : exposed auto
{25,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x33','\xEF'}},// set : exposed manual
{26,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA8','\x30','\xEF'}},// set : shutter speed program
{28,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x36','\xEF'}},// request : shutter speed
{29,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA8','\x31','\xEF'}},// set : shutter speed 1/60(PAL:1/50)
{30,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA8','\x32','\xEF'}},// set : shutter speed 1/100(PAL:1/120)
{31,1,9,2,6,{'\xFF','\x30','\x30','\x00','\xA5','\x37','*','*','\xEF'}},// set : AGC gain
{32,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x38','\xEF'}},// request : AGC gain
{33,1,9,2,6,{'\xFF','\x30','\x30','\x00','\xA5','\x39','*','*','\xEF'}},// set : iris
{34,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x3A','\xEF'}},// request : iris
{35,1,9,2,6,{'\xFF','\x30','\x30','\x00','\xA5','\x3B','*','*','\xEF'}},// set : AE target value
{36,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x3C','\xEF'}},// request : AE target value
{37,1,8,1,6,{'\xFF','\x30','\x30','\x00','\xA5','\x3D','*','\xEF'}},// set : gain select
{38,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x3E','\xEF'}},// request : gain select
{39,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA7','\x32','\xEF'}},// set : white balance manual
{40,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA7','\x33','\xEF'}},// set : white balance high speed
{41,1,9,2,6,{'\xFF','\x30','\x30','\x00','\xA7','\x34','*','*','\xEF'}},// set : white balance manual
{42,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA7','\x35','\xEF'}},// request : white balance
{44,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA9','\x30','\xEF'}},// set : fading mode normal
{45,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA9','\x31','\xEF'}},// set : fading mode white
{46,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA9','\x32','\xEF'}},// set : fading mode high speed white
{47,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA9','\x33','\xEF'}},// set : fading mode high speed black
{48,0,6,0,6,{'\xFF','\x30','\x30','\x00','\xAA','\xEF'} },// set : camera reset
{49,0,6,0,6,{'\xFF','\x30','\x30','\x00','\xAB','\xEF'} },// request : zooming ratio
{50,0,6,0,6,{'\xFF','\x30','\x30','\x00','\xAC','\xEF'} },// request : CCD size
{53,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xB4','\x33','\xEF'}},// request : zooming maximum value
{57,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xBE','\x30','\xEF'}},        // request : camera version
{58,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xBE','\x31','\xEF'}},        // request : eeprom version
{59,1,9,3,6,{'\xFF','\x30','\x30','\x00','\x50','*','*','*','\xEF'}},// set : pan motor speed
{60,1,9,3,6,{'\xFF','\x30','\x30','\x00','\x51','*','*','*','\xEF'}},// set : tilt motor speed
{61,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x52','\x30','\xEF'}},// request : pan motor speed
{62,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x52','\x31','\xEF'}},// request : tilt motor speed
{63,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x53','\x30','\xEF'}},// set : pan/tilt stop
{64,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x53','\x31','\xEF'}},// set : pan right start
{65,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x53','\x32','\xEF'}},// set : pan left start
{66,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x53','\x33','\xEF'}},// set : tilt up start
{67,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x53','\x34','\xEF'}},// set : tilt down start
{69,0,6,0,6,{'\xFF','\x30','\x30','\x00','\x57','\xEF'} },// set : goto home position
{70,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x58','\x30','\xEF'}},// set : pan/tilt motor initilaize1
{71,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x58','\x31','\xEF'}},// set : pan/tilt motor initilaize2
{72,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x59','\x30','\xEF'}},// request : pan motor minimum speed
{73,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x59','\x31','\xEF'}},// request : pan motor maximum speed
{74,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x59','\x32','\xEF'}},// request : tilt motor minimum speed
{75,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x59','\x33','\xEF'}},// request : tilt motor maximum speed
{76,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x5B','\x30','\xEF'}},// request : pan gear ratio
{77,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x5B','\x31','\xEF'}},// request : tilt gear ratio
{78,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x5C','\x30','\xEF'}},// request : pan motor minimum angle
{79,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x5C','\x31','\xEF'}},// request : pan motor maximum angle
{80,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x5C','\x32','\xEF'}},// request : tilt motor minimum angle
{81,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x5C','\x33','\xEF'}},// request : tilt motor maximum angle
{82,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x30','\x30','\xEF'}},// set : pan/tilt stop
{83,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x30','\x31','\xEF'}},// set : tilt up start
{84,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x30','\x32','\xEF'}},// set : tilt down start
{85,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x31','\x30','\xEF'}},// set : pan right start
{86,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x32','\x30','\xEF'}},// set : pan left start
{87,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x31','\x31','\xEF'}},// set : pan right and tilt up start
{88,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x31','\x32','\xEF'}},// set : pan right and tilt down start
{89,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x32','\x31','\xEF'}},// set : pan left and tilt up start
{90,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x60','\x32','\x32','\xEF'}},// set : pan left and tilt down start
{92,2,14,8,6,{'\xFF','\x30','\x30','\x00','\x62','*','*','*','*','*','*','*','*','\xEF'}},// set : pan/tilt angle
{93,0,6,0,6,{'\xFF','\x30','\x30','\x00','\x63','\xEF'} },// request : pan/tilt angle
{94,2,15,8,7,{'\xFF','\x30','\x30','\x00','\x64','\x30','*','*','*','*','*','*','*','*','\xEF'}},// set : pan movement angle
{95,2,15,8,7,{'\xFF','\x30','\x30','\x00','\x64','\x31','*','*','*','*','*','*','*','*','\xEF'}},// set : tilt movement angle
{96,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x65','\x30','\xEF'}},// request : pan movement angle
{97,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x65','\x31','\xEF'}},// request : tilt movement angle
{102,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x80','\x30','\xEF'}},// set : remote command on
{103,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x80','\x31','\xEF'}},// set : remote command off
{104,0,6,0,6,{'\xFF','\x30','\x30','\x00','\x86','\xEF'}},// request : movement status
{106,0,6,0,6,{'\xFF','\x30','\x30','\x00','\x87','\xEF'}},// request : unit name
{107,0,6,0,6,{'\xFF','\x30','\x30','\x00','\x88','\xEF'}},// request : rom version
{108,1,7,1,6,{'\xFF','\x30','\x30','\x00','\x89','*','\xEF'}},// set : preset memory
{109,1,7,1,6,{'\xFF','\x30','\x30','\x00','\x8A','*','\xEF'}},// set : movement preset memory
{110,0,6,0,6,{'\xFF','\x30','\x30','\x00','\x8B','\xEF'}},// request : preset status
{113,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8D','\x30','\xEF'}},// set : remote command pass on
{114,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8D','\x31','\xEF'}},// set : remote command pass off
{118,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8F','\x30','\xEF'}},// set : cascade on
{119,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8F','\x31','\xEF'}},// set : cascade off
{120,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x90','\x30','\xEF'}},// set : host mode
{121,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x90','\x31','\xEF'}},// set local mode
{122,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x30','\xEF'}},// set : onscreen off
{123,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x31','\xEF'}},// set : onscreen on
{124,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x32','\xEF'}},// set : screen title display off
{125,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x33','\xEF'}},// set : screen title display on
{126,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x34','\xEF'}},// set : screen time display off
{127,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x35','\xEF'}},// set : screen time display on (mode1)
{128,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x36','\xEF'}},// set : screen time display on (mode2)
{129,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x37','\xEF'}},// set : screen date display off
{130,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x38','\xEF'}},// set : screen date display on (mode1)
{131,0,8,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x30','\x39','\xEF'}},// set : screen date display on (mode2)
{132,3,12,5,7,{'\xFF','\x30','\x30','\x00','\x91','\x31','*','*','*','*','*','\xEF'}},// set : screen title
{133,3,10,3,7,{'\xFF','\x30','\x30','\x00','\x91','\x32','*','*','*','\xEF'}},// request : screen title
{134,3,13,6,7,{'\xFF','\x30','\x30','\x00','\x91','\x33','*','*','*','*','*','*','\xEF'}},// set : screen date
{135,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x34','\xEF'}},// request : screen date
{136,3,13,6,7,{'\xFF','\x30','\x30','\x00','\x91','\x35','*','*','*','*','*','*','\xEF'}},// set : screen time
{137,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x91','\x36','\xEF'}},// request : screen time
{138,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x92','\x30','\xEF'}},// request : camera power on time
{139,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x92','\x31','\xEF'}},// request : pedestal power on time
{140,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x93','\x30','\xEF'}},// set : default reset
{147,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x86','\x30','\xEF'}},// request : extend movement status
{148,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8B','\x30','\xEF'}},// request : extend preset status
{149,1,9,2,7,{'\xFF','\x30','\x30','\x00','\xA5','\x35','*','*','\xEF'}},// set : shutter speed
{150,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA7','\x30','\xEF'}},// set : white balance normal
{151,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xA7','\x31','\xEF'}},// set : white balance lock
{152,0,7,0,6,{'\xFF','\x30','\x30','\x00','\xB1','\x32','\xEF'}},// request : focus range
{155,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x94','\x30','\xEF'}},// set : notify command off
{156,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x94','\x31','\xEF'}},// set : notify command on
{157,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x95','\x30','\xEF'}},// set : cascade global notify off
{158,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x95','\x31','\xEF'}},// set : cascade global notify on
{159,0,8,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x34','\x30','\xEF'}},// set : AE lock off
{160,0,8,0,6,{'\xFF','\x30','\x30','\x00','\xA5','\x34','\x31','\xEF'}},// set : AE lock on
{164,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8E','\x30','\xEF'}},// set : LED normal
{165,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8E','\x31','\xEF'}},// set : LED green on
{166,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8E','\x32','\xEF'}},// set : LED all off
{167,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8E','\x33','\xEF'}},// set : LED red on
{168,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x8E','\x34','\xEF'}},// set : LED orange on
{170,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x9A','\x30','\xEF'}},// request : pedestal model
{171,0,7,0,6,{'\xFF','\x30','\x30','\x00','\x9A','\x31','\xEF'}}  // request : camera model
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
