/*!@file Devices/ParPort.C Basic Parallel Port driver */

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
//
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/ParPort.C $
// $Id: ParPort.C 7971 2007-02-22 00:42:23Z itti $
//

#include "Devices/ParPort.H"

#include <cstdlib>
#include <cerrno>
#include <csignal>
#include <iostream>

// ######################################################################
ParPort::ParPort(OptionManager& mgr, const std::string& descrName,
                 const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsDevName(tagName+"DevName", this, "/dev/parport0"),
  itsFd(-1), itsData(0), itsNullMode(false)
{  }

// ######################################################################
ParPort::~ParPort()
{  }

// ######################################################################
void ParPort::start1()
{
  if (itsDevName.getVal().compare("/dev/null") == 0)
    {
      // as a special case, if the user specifies "/dev/null" for the
      // parallel device, then we run in "disabled" mode where we
      // silently ignore attempts to write data
      itsFd = -1;
      itsNullMode = true;
      return;
    }

#ifndef HAVE_LINUX_PARPORT_H
  LFATAL("Oops! I need to have <linux/parport.h>");
#else
  const char *devnam = itsDevName.getVal().c_str();

  // open the port:
  itsFd = open(devnam, O_RDWR);
  if (itsFd == -1)
    { PLERROR("Cannot open parallel port %s", devnam); return; }

  // claim the exclusively for userland access:
  if (ioctl(itsFd, PPEXCL))
    {
      PLERROR("Cannot get exclusive access on %s", devnam);
      close(itsFd);
      itsFd = -1;
      return;
    }

  if (ioctl(itsFd, PPCLAIM))
    {
      PLERROR("Cannot ioctl(PPCLAIM) on %s", devnam);
      close(itsFd);
      itsFd = -1;
      return;
    }

  // set to compatibility mode:
  int mode = IEEE1284_MODE_COMPAT;
  if (ioctl(itsFd, PPSETMODE, &mode))
    {
      PLERROR("Cannot switch to IEEE-1284 mode on %s", devnam);
      ioctl(itsFd, PPRELEASE);
      close(itsFd);
      itsFd = -1;
      return;
    }
#endif // HAVE_LINUX_PARPORT_H
}

// ######################################################################
void ParPort::stop2()
{
#ifdef HAVE_LINUX_PARPORT_H
  // release and close port if needed:
  if (itsFd != -1)
    {
      if (ioctl(itsFd, PPRELEASE))
        PLFATAL("Cannot release parport %s", itsDevName.getVal().c_str());

      close(itsFd);
      itsFd = -1;
    }
#endif

  itsNullMode = false;
}

// ######################################################################
void ParPort::WriteData(const byte mask, const byte newdata)
{
  if (itsNullMode)
    {
      LERROR("Parallel port is disabled;  "
             "attempt to write (data=0x%02x, mask=0x%02x) "
             "has been ignored", newdata, mask);
      return;
    }

  if (itsFd < 0)
    LFATAL("Parallel port is not initialized");

#ifndef HAVE_LINUX_PARPORT_H
  LFATAL("Oops! I need to have <linux/parport.h>");
#else
  // clear all bits in our data that were selected by the mask:
  itsData &= ~mask;

  // now set those bits to their new values specified in newdata:
  itsData |= (newdata & mask);

  // set the port:
  if (ioctl(itsFd, PPWDATA, &itsData))
    PLFATAL("Writing to parport failed");
#endif // HAVE_LINUX_PARPORT_H
}

// ######################################################################
byte ParPort::ReadStatus()
{
  if (itsFd < 0)
    LFATAL("Parallel port is not initialized");

#ifndef HAVE_LINUX_PARPORT_H
  LFATAL("Oops! I need to have <linux/parport.h>");
  return 0;
#else
  byte in;
  if (ioctl(itsFd, PPRSTATUS, &in))
    PLFATAL("Failed reading status of parallel port");
  return in;
#endif // HAVE_LINUX_PARPORT_H
}

// ######################################################################
bool ParPort::ReadStatusPaperout()
{
#ifndef HAVE_LINUX_PARPORT_H
  LFATAL("Oops! I need to have <linux/parport.h>");
  return 0;
#else
  return ( (this->ReadStatus() & PARPORT_STATUS_PAPEROUT) != 0 );
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
