/*!@file Devices/ssc.C Interface to a Serial Servo Controller */

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
// Primary maintainer for this file: Jen Ng <jsn@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/ssc.C $
// $Id: ssc.C 6990 2006-08-11 18:13:51Z rjpeters $
//

#include "Devices/ssc.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"

// ######################################################################
SSC::SSC(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr))
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 9600, "8N1", false, false, 1);

  // attach our port as a subcomponent:
  addSubComponent(itsPort);

  // Initialize our internals:
  zero = new rutz::shared_ptr<NModelParam<float> >[SSCNUMSERVOS];
  posmult = new rutz::shared_ptr<NModelParam<float> >[SSCNUMSERVOS];
  negmult = new rutz::shared_ptr<NModelParam<float> >[SSCNUMSERVOS];
  pos = new byte[SSCNUMSERVOS];
  for (uint i = 0; i < SSCNUMSERVOS; i ++)
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
SSC::~SSC()
{ }

// ######################################################################
bool SSC::move(const int servo, const float position)
{ return moveRaw(servo, calibToRaw(servo, position)); }

// ######################################################################
float SSC::getPosition(const int servo) const
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
    LFATAL("Invalid servo number %d", servo);
  return rawToCalib(servo, pos[servo]);
}

// ######################################################################
void SSC::calibrate(const int servo, const byte neutralval, const byte minval,
                    const byte maxval)
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
    LFATAL("Invalid servo number %d", servo);

  zero[servo]->setVal(float(neutralval));
  negmult[servo]->setVal(1.0F / (float(neutralval) - float(minval)));
  posmult[servo]->setVal(1.0F / (float(maxval) - float(neutralval)));
}

// ######################################################################
bool SSC::moveRaw(const int servo, const byte rawpos)
{
  // Command Buffer:
  // [0] is start character
  // [1] is which servo to move
  // [2] is position to move servo to (0 to MAX_POSITION)
  char command[3];
  command[0] = 255;
  command[1] = servo;
  command[2] = rawpos;

  // write command buffer
  if (itsPort->write(command, 3) == 3)
    {
      // update our position:
      pos[servo] = rawpos;
      return true;
    }
  else
      return false;
}

// ######################################################################
bool SSC::moveRawHack(const int servo, const byte rawpos, const int port)
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
byte SSC::getPositionRaw(const int servo) const
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
    LFATAL("Invalid servo number %d", servo);

  return pos[servo];
}

// ######################################################################
float SSC::rawToCalib(const int servo, const byte rawpos) const
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
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
byte SSC::calibToRaw(const int servo, const float position) const
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
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
