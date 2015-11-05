/*!@file Devices/VCC4.H  Interface with pan-tilt mechanism of a Canon VC-C4 camera */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/VCC4.H $
// $Id: VCC4.H 6522 2006-04-25 16:41:24Z rjpeters $
//

#ifndef __VCC4_H__
#define __VCC4_H__

#include "Devices/Serial.H"
#include "Devices/VCC4cmddef.H"

#define VCC4_EMBED_NONE 0
#define VCC4_EMBED_NUM  1
#define VCC4_EMBED_ANG  2
#define VCC4_EMBED_STR  3


#define VCC4_SUCCESS          0 //!operation successful

// errors:
#define VCC4_OPEN_ERR         1 //!error while opening the serial device
#define VCC4_INVALID_COMMAND  2 //!invalid command error
#define VCC4_INVALID_PARAM    3 //!invalid parameters error
#define VCC4_IO_ERR           4 //!input / output error
#define VCC4_REPLY_ERR        5 //!some problem with the reply string
#define VCC4_NO_REPLY         6 //!the camera did not send a reply
#define VCC4_INVALID_NUMBER   7 //!an invalid absolute number parameter
#define VCC4_INVALID_ANGLE    8 //!an invalid angle argument
#define VCC4_ERR_MASK      0x1F

//error flags returned by the camera
#define VCC4_SYSTEM_ERR   (1<<8)
#define VCC4_BUSY_ERR     (1<<12)
#define VCC4_COMMAND_ERR  (1<<13)
#define VCC4_PARAM_ERR    (1<<14)
#define VCC4_MODE_ERR     (1<<15)
#define VCC4_CAMERA_ERRS  (VCC4_SYSTEM_ERR|VCC4_BUSY_ERR|VCC4_COMMAND_ERR|\
                           VCC4_PARAM_ERR|VCC4_MODE_ERR)

#define VCC4_NUM_OVERFLOW    -1
#define VCC4_BUF_UNDERFLOW   -2

//!interface with the pan-tilt mechanism of a Canon VC-C4 camera
class VCC4 : public ModelComponent
{
public:
  //! constructor. See ModelComponent.H.
  VCC4(OptionManager& mgr,
       const std::string& descrName = "VCC4 Pan/Tilt Camera Driver",
       const std::string& tagName = "VCC4driver");

  //! destructor - closes serial connection
  ~VCC4(void);

  //! go to the specified position
  /*!@param pan panning angle (in degrees)
    @param tilt tilting angle (in degrees)
    @param wait wait for the command to successfully complete if true
    @return 0 if successful, some error code otherwise*/
  int gotoPosition(float pan, float tilt, bool wait = true);

  //! retrieve the current position
  /*!@param pan panning angle (in degrees)
    @param tilt tilting angle (in degrees)
    @param wait wait for the command to successfully complete if true
    @return 0 if successful, some error code otherwise*/
  int getPosition (float& pan, float& tilt, bool wait = true);

  //! is the pedestal moving?
  inline bool IsMoving (void);

  //! send a command that does not need parameters
  /*!@param cmdno for command numbers see VCC4cmddef.H
    @param wait wait for successful completion if true
    @return 0 if successful, some error code otherwise*/
  int PlainCommand (int cmdno, bool wait = true);

  //! send a command that needs an integer number as a parameter
  /*!@param cmdno for command numbers see VCC4cmddef.H
    @param value the integer value to set
    @param wait wait for successful completion if true
    @return 0 if successful, some error code otherwise*/
  int AbsNumCommand (int cmdno, unsigned int value, bool wait = true);

  //! send a command that needs two angles as parameters
  /*!@param cmdno for command numbers see VCC4cmddef.H
    @param angle1
    @param angle2 the angles (in degrees) to set
    @param wait wait for successful completion if true
    @return 0 if successful, some error code otherwise*/
  int AngleCommand (int cmdno, float angle1, float angle2, bool wait = true);

  //! send a command that needs a string as a parameter
  /*!@param cmdno for command numbers see VCC4cmddef.H
    @param string the string value to set
    @param wait wait for successful completion if true
    @return 0 if successful, some error code otherwise*/
  int StringCommand (int cmdno, char* string, bool wait = true);

  //! retrieve an absolute number from the ReplyString
  /*!@param the length of the string representing the number
    @return the integer value extracted from the ReplyString */
  inline int getAbsNum (unsigned int length);

  //! retrieve one angle from the ReplyString
  /*!@param angle reference to write the extracted angle to
    @return 0 if successful, some error code otherwise*/
  int getOneAngle (float& angle);

  //! retrieve two angle values from ReplyString
  /*!@param angle1
    @param angle2 references to write the extracted angles to (in degrees)
    @return 0 if successful, some error code otherwise*/
  int getTwoAngles (float& angle1, float& angle2);

  //! initialize camera
  /*! after successful completion, ReplyString contains the name
    of the device.
    @param goHome put camera into Home position if true
    @return 0 if successful, some error code otherwise*/
  int CameraInitialize (bool goHome = false);

  //! set the unit number
  inline int setUnitNo (int unum);

  //! get the unit number
  inline int getUnitNo (void);

  //! returns the pulse ratio for the pan and tilt mechanism
  inline float getPulseRatio(void);

  //! reply string from last command (i.e. status reports and other replies)
  char ReplyString[30];

protected:
  nub::soft_ref<Serial> itsPort;          //!< our serial port
  OModelParam<std::string> itsSerialDevice; //!< our serial device
  OModelParam<int> itsUnitNo;          //!< our unit number
  OModelParam<float> itsPulseRatio;    //!< our pulse ratio
  OModelParam<float> itsMaxAngle;      //!< our max angle

  //! send a command with parameters to the device
  int SendRawCommand (int cmdno, char* param = NULL);

  //! send command and keep trying if busy
  int WaitRawCommand (int cmdno, char* param = NULL);

  //! return the index for a command number
  int getIndex (int cmdno);

  //! attempt to retrieve a reply
  int getReply (void);

  //! transforms an unsigned int to a string of hex digits
  int intToHexString (unsigned int value, char* buf, unsigned int length);

  //! transforms a string of hex digits into an int
  int hexStringToInt (char* buf, unsigned int length);

  //! get started (see ModelComponent.H). The serial port gets configured.
  void start1();

  //! get stopped (see ModelComponent.H). We return the unit to home position
  void stop1();

private:
  static VCC4CMD command[VCC4_CMDMAX];          // array with command codes
  char paramString[30];
};

// ######################################################################
// ########  inlined methods
// ######################################################################

// ######################################################################
inline int VCC4::setUnitNo (int unum)
{
  if ((unum < 0) || (unum > 9)) itsUnitNo.setVal(0);
  else itsUnitNo.setVal(unum);
  return unum;
}

// ######################################################################
inline int VCC4::getUnitNo (void)
{
  return itsUnitNo.getVal();
}

// ######################################################################
inline bool VCC4::IsMoving (void)
{
  int err = PlainCommand (VCC4_GetPTAngle2, false);
  return ((err & VCC4_BUSY_ERR) != 0);
}

// ######################################################################
inline int VCC4::getAbsNum (unsigned int length)
{
  return hexStringToInt(&(ReplyString[0]), length);
}

// ######################################################################
inline float VCC4::getPulseRatio(void)
{
  return itsPulseRatio.getVal();
}


#endif



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
