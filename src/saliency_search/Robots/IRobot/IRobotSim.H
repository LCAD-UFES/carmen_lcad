/*!@file Robots/IRobot/IRobotSim.C IRobot Simulator */


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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/IRobot/IRobotSim.H $
// $Id: IRobotSim.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef IRobotSim_H_DEFINED
#define IRobotSim_H_DEFINED

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

#include <vector>

class IRobotSim : public ModelComponent
{
public:

#define ROBOT_BITFIELD 1
#define GROUND_BITFIELD 2
#define OBJ_BITFIELD 4

  enum OBJECT_TYPE {BOX, CYLINDER, SPHERE, TREE, DS, TOBJ};
  typedef struct{
    dBodyID body;
    dGeomID geom;
    OBJECT_TYPE type;
    float color[3];
    ViewPort::TEXTURES texture;
    ViewPort::DSObject object;
    Texture *texturePtr;
    int textureID;
  } Object;

  //Constructor
  IRobotSim(OptionManager& mgr,
          const std::string& descrName = "IRobotSim",
          const std::string& tagName = "IRobotSim",
          bool showWorld=false);
  ~IRobotSim();

  //###########################################################
  void start2();
  void handleWinEvents(XEvent& event);

  void simLoop();
  void startSim(void);
  void updateSensors();
  Image<PixRGB<byte> > getFrame(int camera);

  void getSensors(float &xPos, float &yPos, float &ori);

  void setMotors(double rightSpeed, double leftSpeed);

  dWorldID        getWorld() { return world; }
  dJointGroupID getContactgroup() { return contactgroup; }

  void addObject(OBJECT_TYPE objType,double initPos[3],double objSize[3],
      bool addToSpace = true,
      const char* texture=NULL);

 // void nearCallback (void *data, dGeomID o1, dGeomID o2);
 //

  float getXPos() { return itsXPos; }
  float getYPos() { return itsYPos; }
  float getOri() { return itsOri; }

  //INitalize the viewport; This needs to be called from the same thread that callsed getFrame
  void initViewport();


  dGeomID getCasterGeom() { return itsCasterGeom; }
  dGeomID getRobotGeom() { return itsRobotGeom; }
  dGeomID getGroundGeom() { return ground; }

private:

  void makeRobot();
  void drawRobot();

  //World
  void drawWorld();
  void makeWorld();

  dWorldID        world;
  dSpaceID        space;
  dGeomID          ground;
  dJointGroupID contactgroup;

  float   itsRobotStartZ;
  dBodyID itsRobotBody;
  dGeomID itsRobotGeom;

  dBodyID itsRightWheelBody;
  dGeomID itsRightWheelGeom;
  dJointID itsRightWheelJoint;

  dBodyID  itsLeftWheelBody;
  dGeomID  itsLeftWheelGeom;
  dJointID itsLeftWheelJoint;

  dBodyID  itsCasterBody;
  dGeomID  itsCasterGeom;

  double itsRobotRadius;
  double itsRobotHeight;
  double itsRobotWeight;
  double itsRobotWheelRadius;

  ViewPort *itsVP;

  double itsRightWheelSpeed;
  double itsLeftWheelSpeed;

  //sensors
  double itsXPos;
  double itsYPos;
  double itsOri;
  double itsRightEncoder;
  double itsLeftEncoder;
  int itsPrevRightWheelAng;
  int itsPrevLeftWheelAng;

  bool itsWorldView;
  bool itsShowWorld;
  XWinManaged *itsWorldDisp;

  std::vector<Object> itsObjects;

  pthread_mutex_t itsDispLock;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
