/*!@file Devices/Pololu.C Interface to poloau Serial Servo Controller */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Pololu.C $
// $Id: Pololu.C 8524 2007-06-28 20:37:05Z rjpeters $
//

#include "Devices/Pololu.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"

// ######################################################################
Pololu::Pololu(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr))
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 57600 , "8N1", false, false, 1);

  // attach our port as a subcomponent:
  addSubComponent(itsPort);

  // Initialize our internals:
  zero = new rutz::shared_ptr<NModelParam<float> >[PololuNUMSERVOS];
  posmult = new rutz::shared_ptr<NModelParam<float> >[PololuNUMSERVOS];
  negmult = new rutz::shared_ptr<NModelParam<float> >[PololuNUMSERVOS];
  pos = new byte[PololuNUMSERVOS];
  for (uint i = 0; i < PololuNUMSERVOS; i ++)
    {
      pos[i] = 127; char buf[20];

      sprintf(buf, "Zero%d", i);
      zero[i] = NModelParam<float>::make(buf, this, 0.0F);

      sprintf(buf, "PosMult%d", i);
      posmult[i] = NModelParam<float>::make(buf, this, 1.0F);

      sprintf(buf, "NegMult%d", i);
      negmult[i] = NModelParam<float>::make(buf, this, 1.0F);
    }
}

// ######################################################################
Pololu::~Pololu()
{
  delete [] pos;
  delete [] negmult;
  delete [] posmult;
  delete [] zero;
}

// ######################################################################
bool Pololu::move(const int servo, const float position)
{ return moveRaw(servo, calibToRaw(servo, position)); }

// ######################################################################
float Pololu::getPosition(const int servo) const
{
  if (servo < 0 || servo >= PololuNUMSERVOS)
    LFATAL("Invalid servo number %d", servo);
  return rawToCalib(servo, pos[servo]);
}

// ######################################################################
void Pololu::calibrate(const int servo, const byte neutralval, const byte minval,
                    const byte maxval)
{
  if (servo < 0 || servo >= PololuNUMSERVOS)
    LFATAL("Invalid servo number %d", servo);

  zero[servo]->setVal(float(neutralval));
  negmult[servo]->setVal(1.0F / (float(neutralval) - float(minval)));
  posmult[servo]->setVal(1.0F / (float(maxval) - float(neutralval)));
}

// ######################################################################
bool Pololu::moveRaw(const int servo, const byte rawpos)
{
  char command[5];
  command[0] = 128; //sync
  command[1] = 1; //device #
  command[2] = 2; //set position 7-bit command
  command[3] = servo; //servo num
  command[4] = rawpos; //data


  // write command buffer
  if (itsPort->write(command, 5) == 5)
    {
      // update our position:
      pos[servo] = rawpos;
      return true;
    }
  else
      return false;
}

// ######################################################################
bool Pololu::setSpeed(const int servo, const byte speed)
{
  char command[5];
  command[0] = 128; //sync
  command[1] = 1; //device #
  command[2] = 1; //set speed command
  command[3] = servo; //servo num
  command[4] = speed; //data


  // write command buffer
  if (itsPort->write(command, 5) == 5)
      return true;
  else
      return false;

}

// ######################################################################
bool Pololu::setNeutral(const int servo, const short int pos)
{
  char command[6];
  command[0] = 128; //sync
  command[1] = 1; //device #
  command[2] = 5; //set neutral command
  command[3] = servo; //servo num
  command[4] = pos >> 7; //data MSB
  command[5] = (pos & 0x7F); //data LSB

  // write command buffer
  if (itsPort->write(command, 6) == 6)
      return true;
  else
      return false;

}

// ######################################################################
bool Pololu::setParam(const int servo,
    bool on_off, bool direction, char range)
{
  char command[5];
  command[0] = 128; //sync
  command[1] = 1; //device #
  command[2] = 0; //set param command
  command[3] = servo; //servo num
  command[4] = 0x00;
  command[4] |= (on_off << 6);
  command[4] |= (direction << 5);
  command[4] |= (range & 0x1f);

  // write command buffer
  if (itsPort->write(command, 5) == 5)
      return true;
  else
      return false;

}

// ######################################################################
bool Pololu::moveRawHack(const int servo, const byte rawpos, const int port)
{
  // Command Buffer:
  // [0] is start character
  // [1] is which servo to move
  // [2] is position to move servo to (0 to MAX_POSITION)
  char command[3];
  command[0] = 255;
  command[1] = servo;
  command[2] = rawpos;
  int fd;
  if(port == 1)
    fd = open("/dev/ttyS0", O_RDWR);
  else
    fd = open("/dev/ttyS1", O_RDWR);
  if(fd==-1) LFATAL("open failed");
  if(write(fd, command, 3)==-1){
    perror("open");
    LFATAL("write failed");
  }
  close(fd);
  return true;
}

// ######################################################################
byte Pololu::getPositionRaw(const int servo) const
{
  if (servo < 0 || servo >= PololuNUMSERVOS)
    LFATAL("Invalid servo number %d", servo);

  return pos[servo];
}

// ######################################################################
float Pololu::rawToCalib(const int servo, const byte rawpos) const
{
  if (servo < 0 || servo >= PololuNUMSERVOS)
    LFATAL("Invalid servo number %d", servo);

  float position;
  if (rawpos >= byte(zero[servo]->getVal()))
    position = (float(rawpos) - zero[servo]->getVal()) *
      posmult[servo]->getVal();
  else
    position = (float(rawpos) - zero[servo]->getVal()) *
      negmult[servo]->getVal();

  if (position < -1.0F) position = -1.0F;
  else if (position > 1.0F) position = 1.0F;

  return position;
}

// ######################################################################
byte Pololu::calibToRaw(const int servo, const float position) const
{
  if (servo < 0 || servo >= PololuNUMSERVOS)
    LFATAL("Invalid servo number %d", servo);

  if (position > 1.0F || position < -1.0F)
    LFATAL("Invalid position %f (range -1.0..1.0)", position);

  int rawpos;
  if (position < 0.0F)
    rawpos = int(position / negmult[servo]->getVal() +
                 zero[servo]->getVal() + 0.49999F);
  else
    rawpos = int(position / posmult[servo]->getVal() +
                 zero[servo]->getVal() + 0.49999F);

  if (rawpos < 0) rawpos = 0; else if (rawpos > 255) rawpos = 255;

  return byte(rawpos);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
