/*!@file Devices/Visca.H Interface to ptz cameras via the VISCA protocol */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Visca.H $
// $Id: Visca.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef Visca_H_DEFINED
#define Visca_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"      // for byte

class Serial;

class Visca : public ModelComponent
{
public:
  //! Default constructor; see ModelComponent.H
  Visca(OptionManager& mgr,
      const std::string& descrName = "Visca PTZ Controller",
      const std::string& tagName = "ViscaPTZ",
      const char *defdev = "/dev/ttyUSB1",
  //    const char *defdev = "/dev/ttyS0",
      const int cameraAddr = 1);

  //! Destructor
  ~Visca();

  virtual void start2();

  //! Moves the camera, ptz
  /*! Returns true on success, false if some serial error occurred. */
  bool movePanTilt(const int pan, const int tilt,
      bool relative = false,
      const int panSpeed = 5, const int tiltSpeed = 5);

  //!Reset the pan tilt
  bool resetPanTilt();

  //! zoom to a given value with a given speed
  bool zoom(const int val, bool relative = false);

  //! set focus
  bool setFocus(const int val, bool relative = false);

  //! set Auto focus
  bool setAutoFocus(const bool val);


  //! set the target tracking option in the camera
  bool setTargetTracking();

  int getCurrentZoom() { return itsCurrentZoom; }

  bool getPanTilt(short int &pan, short int &tilt);


protected:

  //!Send the packet to the serial port
  int sendRawCmd(const unsigned char* cmd, const int length);
  nub::soft_ref<Serial> itsPort;   //!< Serial port to use


private:
  int itsCameraAddr;
  int itsCurrentZoom;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
