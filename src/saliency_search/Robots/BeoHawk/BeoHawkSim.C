/*!@file BeoSub/BeoHawkSim.C Sub Simulator */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
//  See http://iLab.usc.edu for information about this project.          //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/BeoHawk/BeoHawkSim.C $
// $Id: BeoHawkSim.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Robots/BeoHawk/BeoHawkSim.H"
#include "Component/OptionManager.H"
#include "Util/MathFunctions.H"
#include "Util/Assert.H"
#include "rutz/compat_snprintf.h"
#include "Image/MatrixOps.H"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>


namespace {

  void nearCallback (void *data, dGeomID o1, dGeomID o2){
    BeoHawkSim *subSim = (BeoHawkSim *)data;
    const int MaxContacts = 10;

    //create a contact joint to simulate collisions
    dContact contact[MaxContacts];
    int nContacts = dCollide (o1,o2,MaxContacts,
        &contact[0].geom,sizeof(dContact));
    for (int i=0; i<nContacts; i++) {
      contact[i].surface.mode = dContactSoftCFM ; //| dContactBounce; // | dContactSoftCFM;
      contact[i].surface.mu = 0.5;
      contact[i].surface.mu2 = 0.5;
      contact[i].surface.bounce = 0.01;
      contact[i].surface.bounce_vel = 0.01;
      contact[i].surface.soft_cfm = 0.001;

      dJointID c = dJointCreateContact (subSim->getWorld(),
          subSim->getContactgroup(),&contact[i]);
      dJointAttach (c,
          dGeomGetBody(contact[i].geom.g1),
          dGeomGetBody(contact[i].geom.g2));
    }
  }
}

// ######################################################################
BeoHawkSim::BeoHawkSim(OptionManager& mgr, const std::string& descrName,
    const std::string& tagName, bool showWorld) :
  ModelComponent(mgr, descrName, tagName),
  itsBeoHawkLength(7.0), //the length of sub in meters
  itsBeoHawkWidth(4.0), //the radius of the sub in meters
  itsBeoHawkWeight(30), // the weight of the sub in kg
  vp(new ViewPort("BeoHawkSim")),
  itsPanThruster(0),
  itsPitchThruster(0),
  itsRollThruster(0),
  itsUpThruster(0),
  itsXPos(0),
  itsYPos(0),
  itsDepth(0),
  itsRoll(0),
  itsPitch(0),
  itsYaw(0),
  itsWorldView(true),
  itsShowWorld(showWorld),
  itsWorldDisp(NULL)
{

  pthread_mutex_init(&itsDispLock, NULL);


}

BeoHawkSim::~BeoHawkSim()
{
  dSpaceDestroy(space);
  dWorldDestroy(world);

  delete vp;
  delete itsWorldDisp;
  pthread_mutex_destroy(&itsDispLock);
}

void BeoHawkSim::start2()
{
  //        setDrawStuff();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground = dCreatePlane(space, 0, 0, 1, 0);
  dWorldSetGravity(world,0,0,-0.5);

  dWorldSetCFM (world,1e-6);
  dWorldSetERP (world,1);
  //dWorldSetAutoDisableFlag (world,1);
  dWorldSetContactMaxCorrectingVel (world,0.1);
  //set the contact penetration
  dWorldSetContactSurfaceLayer(world, 0.001);


  makeBeoHawk();

  itsBeoHawkObject = vp->load3DSObject("./etc/spaceship.3ds", "./etc/textures/spaceshiptexture.ppm");
  itsBeoHawkObject.scale = 0.1;


  //set the viewpoint
  double xyz[3] = {0 , 25.0, 20};
  double hpr[3] = {-90.0,-35,0.0};
  vp->dsSetViewpoint (xyz,hpr);
  vp->setZoom(1.5);

}

void BeoHawkSim::makeBeoHawk()
{
  dMass mass;
  itsBeoHawkBody = dBodyCreate(world);
  //pos[0] = 0; pos[1] = 1.0*5; pos[2] = 1.80;

  dBodySetPosition(itsBeoHawkBody,0,0.2*5,4.94);

  dMatrix3 R;
  dRFromAxisAndAngle (R,1,0,0,0);
  dBodySetRotation(itsBeoHawkBody, R);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, itsBeoHawkWeight,itsBeoHawkWidth, itsBeoHawkLength, 0.5);
  //dMassSetCappedCylinderTotal(&mass,itsBeoHawkWeight,3,itsBeoHawkWidth,itsBeoHawkLength/2);
  dMassRotate(&mass, R);
  dBodySetMass(itsBeoHawkBody,&mass);
  itsBeoHawkGeom = dCreateBox(space, itsBeoHawkWidth, itsBeoHawkLength, 0.5);
  dGeomSetRotation(itsBeoHawkGeom, R);
  dGeomSetBody(itsBeoHawkGeom, itsBeoHawkBody);
}

void BeoHawkSim::drawBeoHawk()
{

//  vp->dsSetColor(1,1,0);

  vp->dsDraw3DSObject(dBodyGetPosition(itsBeoHawkBody),
      dBodyGetRotation(itsBeoHawkBody),
      itsBeoHawkObject);

//  double sides[3] = {itsBeoHawkWidth, itsBeoHawkLength, 0.5};
//  vp->dsDrawBox(
//      dBodyGetPosition(itsBeoHawkBody),
//      dBodyGetRotation(itsBeoHawkBody),
//      sides);

}

//void BeoHawkSim::updateSensors(const double *pos, const double *R)
void BeoHawkSim::updateSensors(const dReal *pos, const dReal *R)
{
  itsXPos = pos[0];
  itsYPos = pos[1];
  itsDepth = pos[2];
  itsRoll  = atan2(R[9], R[10]) + M_PI/2;     //phi correct for initial rotation
  itsPitch = asin(-R[8]);            //theta
  itsYaw   = atan2(R[4], R[0]);      //greek Y

  if (itsYaw < 0) itsYaw += M_PI*2;
  // LINFO("(%f,%f) Depth %f, roll %f pitch %f yaw %f",
  //     itsXPos, itsYPos, itsDepth,
  //     itsRoll, itsPitch, itsYaw);

}

void BeoHawkSim::simLoop()
{

  //set the trusters
  float thVar = 5.0;
  dBodyAddRelForceAtRelPos(itsBeoHawkBody,0,itsUpThruster+randomDoubleFromNormal(thVar),0, 0,0,itsBeoHawkLength/4);
  dBodyAddRelForceAtRelPos(itsBeoHawkBody,0,itsUpThruster+randomDoubleFromNormal(thVar),0, 0,0,-itsBeoHawkLength/4);
  dBodyAddRelForceAtRelPos(itsBeoHawkBody,0,itsUpThruster+randomDoubleFromNormal(thVar),0, itsBeoHawkLength/4,0,0);
  dBodyAddRelForceAtRelPos(itsBeoHawkBody,0,itsUpThruster+randomDoubleFromNormal(thVar),0, -itsBeoHawkLength/4,0,0);


  dBodyAddRelForceAtRelPos(itsBeoHawkBody,itsPanThruster,0,0,  0,0,itsBeoHawkLength/2);

  dBodyAddRelForceAtRelPos(itsBeoHawkBody,0,itsPitchThruster,0,  0,0,itsBeoHawkLength/2);
  dBodyAddRelForceAtRelPos(itsBeoHawkBody,0,-itsPitchThruster,0,  0,0,-1*itsBeoHawkLength/2);

  dBodyAddRelForceAtRelPos(itsBeoHawkBody,0,itsRollThruster,0,  itsBeoHawkWidth*2,0,0);
  dBodyAddRelForceAtRelPos(itsBeoHawkBody,0,-itsRollThruster,0,  -1*itsBeoHawkWidth*2,0,0);

  //Apply a viscosity water force
  //applyHydrodynamicForces(0.5);

  //Folow the sub with the camera
  //  const double *bodyPos = dBodyGetPosition(itsSubBody);
  //const double *bodyR = dBodyGetRotation(itsSubBody);
  const dReal *bodyPos = dBodyGetPosition(itsBeoHawkBody);
  const dReal *bodyR = dBodyGetRotation(itsBeoHawkBody);

  updateSensors(bodyPos, bodyR);

  dSpaceCollide (space,this,&nearCallback); //check for collisions

  dWorldStep(world,0.1);

  dJointGroupEmpty (contactgroup); //delete the contact joints

}


Image<PixRGB<byte> > BeoHawkSim::getFrame(int camera)
{
  const dReal *bodyPos = dBodyGetPosition(itsBeoHawkBody);
  const dReal *bodyR = dBodyGetRotation(itsBeoHawkBody);

  double cam_xyz[3], cam_hpr[3] = {0.0,0.0,0.0};

  switch (camera)
  {
    case 0: //world camera
      cam_xyz[0] = bodyPos[0];
      cam_xyz[1] =  bodyPos[1]-5;
      cam_xyz[2] =  20;

      cam_hpr[0]   = 90.0;
      cam_hpr[1]   = -45.0;
      cam_hpr[2]   = 0.0;

      break;
    case 1:
      cam_xyz[0] = bodyPos[0];
      cam_xyz[1] = bodyPos[1];
      cam_xyz[2] = bodyPos[2];

      cam_hpr[0]   = (atan2(bodyR[4], bodyR[0])*180/M_PI) + 90; //yaw

      break;
    case 2:
      cam_xyz[0] = bodyPos[0];
      cam_xyz[1] = bodyPos[1];
      cam_xyz[2] = bodyPos[2];

      cam_hpr[0]   = (atan2(bodyR[4], bodyR[0])*180/M_PI) + 90; //yaw
      cam_hpr[1]   = -90; //yaw


      break;
  }
  pthread_mutex_lock(&itsDispLock);
  if (camera != -1)
    vp->dsSetViewpoint (cam_xyz,cam_hpr);
  vp->initFrame();
  drawBeoHawk();
  vp->updateFrame();
  pthread_mutex_unlock(&itsDispLock);


  return vp->getFrame();

}

void BeoHawkSim::getSensors(float &xPos, float &yPos, float &depth,
    float &roll, float &pitch, float &yaw)
{

  xPos = itsXPos;
  yPos = itsYPos;
  depth = itsDepth;
  roll = itsRoll;
  pitch = itsPitch;
  yaw = itsYaw;

}

void BeoHawkSim::setThrusters(float panThruster, float pitchThruster, float rollThruster, float upThruster)
{

  itsPanThruster =      panThruster;
  itsPitchThruster =    pitchThruster;
  itsRollThruster =     rollThruster;
  itsUpThruster =       upThruster;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
