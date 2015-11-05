/*!@file BeoSub/BeoHawkSim.H Sub Simulator */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/BeoHawk/BeoHawkSim.H $
// $Id: BeoHawkSim.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef BeoHawkSim_H_DEFINED
#define BeoHawkSim_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "GUI/ViewPort.H"
#include "GUI/XWinManaged.H"
#include "Util/Types.H"
#include <deque>
#include <pthread.h>
#include <stdarg.h>
#include <ode/ode.h>
#include <ode/collision.h>

class BeoHawkSim : public ModelComponent
{
public:
  //Constructor
  BeoHawkSim(OptionManager& mgr,
          const std::string& descrName = "BeoHawkSim",
          const std::string& tagName = "BeoHawkSim",
          bool showWorld=true);
  ~BeoHawkSim();

  //###########################################################
  void start2();

  void simLoop();
  void startSim(void);
  void updateSensors(const dReal *pos, const dReal *R);
  Image<PixRGB<byte> > getFrame(int camera);

  void getSensors(float &xPos, float &yPos, float &depth,
       float &roll, float &pitch, float &yaw);

  void setThrusters(float panThruster, float pitchThruster, float rollThruster, float upThruster);

  dWorldID        getWorld() { return world; }
  dJointGroupID getContactgroup() { return contactgroup; }

 // void nearCallback (void *data, dGeomID o1, dGeomID o2);


private:

  void makeBeoHawk();
  void drawBeoHawk();

  dWorldID        world;
  dSpaceID        space;
  dGeomID          ground;
  dJointGroupID contactgroup;

  dBodyID itsBeoHawkBody;
  dGeomID itsBeoHawkGeom;

  double itsBeoHawkLength;
  double itsBeoHawkWidth;
  double itsBeoHawkWeight;

  ViewPort *vp;

  double itsPanThruster;
  double itsPitchThruster;
  double itsRollThruster;
  double itsUpThruster;

  //sensors
  double itsXPos;
  double itsYPos;
  double itsDepth;
  double itsRoll;
  double itsPitch;
  double itsYaw;

  bool itsWorldView;
  bool itsShowWorld;
  XWinManaged *itsWorldDisp;

  pthread_mutex_t itsDispLock;

  ViewPort::DSObject itsBeoHawkObject;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
