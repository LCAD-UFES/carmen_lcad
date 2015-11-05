/*!@file Devices/BeoHead.H Interface to robot head */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeoHead.H $
// $Id: BeoHead.H 8732 2007-08-30 19:33:05Z beobot $
//

#ifndef BeoHead_H_DEFINED
#define BeoHead_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/Pololu.H"
#include "Controllers/PID.H"
#include "Util/Types.H"
#include <deque>
#include <pthread.h>
#include <stdarg.h>

class BeoHead : public ModelComponent
{
public:
  //Constructor
  BeoHead(OptionManager& mgr,
          const std::string& descrName = "BeoHead",
          const std::string& tagName = "BeoHead");

  bool setLeftEyePan(float pos);
  bool setLeftEyeTilt(float pos);
  bool setRightEyePan(float pos);
  bool setRightEyeTilt(float pos);

  bool setHeadPan(float pos);
  bool setHeadTilt(float pos);
  bool setHeadYaw(float pos);


  float getLeftEyePanPos(){ return  itsLeftEyePanPos;  }
  float getRightEyeTiltPos(){ return  itsRightEyeTiltPos;}
  float getRightEyePanPos(){ return  itsRightEyePanPos; }
  float getLeftEyeTiltPos(){ return  itsLeftEyeTiltPos; }
  float getHeadPanPos(){ return  itsHeadPanPos;     }
  float getHeadTiltPos(){ return  itsHeadTiltPos;    }
  float getHeadYawPos(){ return  itsHeadYawPos;     }


  bool relaxNeck(); //shut off neck servos
  bool relaxHead(); //shut off all servos
  bool moveRestPos(); //move the head to a relaxed position (all servos on)

  //move the head to the desired position based on the current visual pos
  float trackTarget(float desiredVisualPosX, float desiredVisualPosY,
    float currentVisualPosX, float currentVisualPosY);

  //###########################################################
  void start2();

  void initHead();

  //servo definition
  static const byte LEFT_EYE_PAN     = 2;
  static const byte LEFT_EYE_TILT    = 1;
  static const byte RIGHT_EYE_PAN    = 0;
  static const byte RIGHT_EYE_TILT   = 3;

  static const byte HEAD_PAN         = 6;
  static const byte HEAD_LEFT        = 7;
  static const byte HEAD_RIGHT       = 5;

private:

  nub::soft_ref<Pololu> itsPololu; //our servo controller

  float itsLeftEyePanPos;
  float itsRightEyeTiltPos;
  float itsRightEyePanPos;
  float itsLeftEyeTiltPos;
  float itsHeadPanPos;
  float itsHeadTiltPos;
  float itsHeadYawPos;

  PID<float>  pidLeftPan;
  PID<float>  pidLeftTilt;
  PID<float>  pidHeadPan;


  bool isNeckRelaxed;
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
