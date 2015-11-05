/*!@file Robots/Rovio/Rovio.H Interface to Rovio robot */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Rovio/Rovio.H $
// $Id: Rovio.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef Rovio_H_DEFINED
#define Rovio_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/HTTPClient.H"

#include <fstream>

class Rovio : public ModelComponent
{
public:

  enum DRIVE_PARAMS {STOP, FORWARD, BACKWARD, STRAIGHT_LEFT, STRAIGHT_RIGHT, ROTATE_LEFT, ROTATE_RIGHT};


  //Constructor
  Rovio(OptionManager& mgr,
          const std::string& descrName = "Rovio",
          const std::string& tagName = "Rovio");

  ~Rovio();

  void start2();

  //! Stop the robot
  bool stop();


  //! Move the robot forward by speed. Speed is nomalized from 0 to 1
  bool moveForward(float speed);

  //! Move the robot forward by speed. Speed is nomalized from 0 to 1
  bool moveBackward(float speed);

  //! Move the robot forward by speed. Speed is nomalized from 0 to 1
  bool straightLeft(float speed );

  //! Move the robot forward by speed. Speed is nomalized from 0 to 1
  bool straightRight(float speed);

  //! Move the robot forward by speed. Speed is nomalized from 0 to 1
  bool rotateLeft(float speed);

  //! Move the robot forward by speed. Speed is nomalized from 0 to 1
  bool rotateRight(float speed);

  //! Ge the Robot Status
  bool getStatus();

  //!Set the camera position
  bool setCameraPos(int pos);

  //!Play sound
  bool playSound();

protected:
  bool sendDriveRequest(DRIVE_PARAMS dValue, float speed);

  virtual void stop2();

private:
  nub::soft_ref<HTTPClient> itsHttpClient; //our controller

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
