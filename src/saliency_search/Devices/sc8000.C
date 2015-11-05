/*!@file sc8000.C Interface to a Serial Servo Controller sc8000
 derived from ssc*/

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
//
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/sc8000.C $
// $Id: sc8000.C 6990 2006-08-11 18:13:51Z rjpeters $
//

#include "Devices/sc8000.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"

// ######################################################################
SC8000::SC8000(OptionManager& mgr, const std::string& descrName,
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
  pos = new short[SSCNUMSERVOS];
  for (uint i = 0; i < SSCNUMSERVOS; i ++)
    {
      pos[i] = 32768; char buf[20];
                sprintf(buf, "Zero%d", i);
      zero[i] = NModelParam<float>::make(buf, this, 0.0F);

      sprintf(buf, "PosMult%d", i);
      posmult[i] = NModelParam<float>::make(buf, this, 1.0F);

      sprintf(buf, "NegMult%d", i);
      negmult[i] = NModelParam<float>::make(buf, this, 1.0F);
    }
}

// ######################################################################
SC8000::~SC8000()
{ }

// ######################################################################
bool SC8000::move(const int servo, const float position)
{ return moveRaw(servo, calibToRaw(servo, position)); }

// ######################################################################
float SC8000::getPosition(const int servo) const
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
    LFATAL("Invalid servo number %d -- ABORT", servo);
  return rawToCalib(servo, pos[servo]);
}

// ######################################################################
void SC8000::calibrate(const int servo, const short neutralval, const short minval,
                    const short maxval)
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
    LFATAL("Invalid servo number %d -- ABORT", servo);

  zero[servo]->setVal(float(neutralval));
  negmult[servo]->setVal(1.0F / (float(neutralval) - float(minval)));
  posmult[servo]->setVal(1.0F / (float(maxval) - float(neutralval)));
}

// ######################################################################
bool SC8000::moveRaw(const int servo, const short rawpos)
{
  // Command Buffer:
  // [0] is start character
  // [1] is start character
  // [2] is which servo to move (servo mask)
  // [3] digital mask
  // [4] is position to move servo to (high order)
  // [5] is position to move servo to (low order)

  char command[6];
  command[0] = 126;
  command[1] = 126;
  command[2] = 1 << (servo-1); //set servo mask to only one servo (1 index base)
  command[3] = 48;        //ascii for 0 (digital ports off)

  //servo positions are 2 bytes
  command[4] = (unsigned char)((rawpos >> 8) & 0xFF);
  command[5] = (unsigned char)(rawpos);


  // write command buffer
  if (itsPort->write(command, 6) == 6)
    {
      // update our position:
      pos[servo] = rawpos;
      return true;
    }
  else
      return false;
}

// ######################################################################
short SC8000::getPositionRaw(const int servo) const
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
    LFATAL("Invalid servo number %d -- ABORT", servo);

  return pos[servo];
}

// ######################################################################
float SC8000::rawToCalib(const int servo, const short rawpos) const
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
    LFATAL("Invalid servo number %d -- ABORT", servo);

  float position;
  if (rawpos >= short(zero[servo]->getVal()))
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
short SC8000::calibToRaw(const int servo, const float position) const
{
  if (servo < 0 || servo >= SSCNUMSERVOS)
    LFATAL("Invalid servo number %d -- ABORT", servo);

  if (position > 1.0F || position < -1.0F)
    LFATAL("Invalid position %f (range -1.0..1.0) -- ABORT", position);

  int rawpos;
  if (position < 0.0F)
    rawpos = int(position / negmult[servo]->getVal() +
                 zero[servo]->getVal() + 0.49999F);
  else
    rawpos = int(position / posmult[servo]->getVal() +
                 zero[servo]->getVal() + 0.49999F);

  if (rawpos < 0) rawpos = 0; else if (rawpos > 65535) rawpos = 65535;

  return short(rawpos);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
