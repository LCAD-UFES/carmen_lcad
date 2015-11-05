/*!@file Devices/ParPort.H Basic Parallel Port driver */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/ParPort.H $
// $Id: ParPort.H 7971 2007-02-22 00:42:23Z itti $
//
#ifndef PARPORT_H_DEFINED
#define PARPORT_H_DEFINED

#include "Util/Types.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#ifdef HAVE_LINUX_PARPORT_H
#  include <linux/parport.h>
#  include <linux/ppdev.h>
#endif
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

/*
#define PARPORT_STATUS_BUSY (1<<7)
#define PARPORT_STATUS_ACK (1<<6)
#define PARPORT_STATUS_PAPER (1<<5)
#define PARPORT_STATUS_SELECT (1<<4)
#define PARPORT_STATUS_ERROR (1<<3)
#define PARPORT_STATUS_IRQ (1<<2)
*/

//! This is a minimalistic interface to a PC parallel port under Linux
class ParPort : public ModelComponent
{
public:
  //! Constructor
  ParPort(OptionManager& mgr,
          const std::string& descrName = "Parallel Port",
          const std::string& tagName = "ParPort");

  //! Destructor
  virtual ~ParPort();

  //! Write some data to the port
  /*! For all the bits which are set in mask, their values will be
    replaced by the values of the corresponding bits in newdata. */
  void WriteData(const byte mask, const byte newdata);

  //! Read status of the port
  byte ReadStatus();

  //! Read the PARPORT_STATUS_PAPEROUT bit of the port status
  bool ReadStatusPaperout();

protected:
  NModelParam<std::string> itsDevName; //!< Name of device to use

  virtual void start1(); //!< get started
  virtual void stop2();  //!< get stopped

private:
  int itsFd;  // file descriptor for the device
  byte itsData;
  bool itsNullMode; // whether we want to run as a disabled "null" device
};

#endif
