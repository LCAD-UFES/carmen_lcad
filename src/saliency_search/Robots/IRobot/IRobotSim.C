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
// $HeadURL $
// $Id $
//

#include "Robots/IRobot/IRobotSim.H"
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
    IRobotSim *iRobotSim = (IRobotSim *)data;
    const int MaxContacts = 10;

    double surface_mu = dInfinity;

    //Dont colide the robot with the ground
    if ((o1 == iRobotSim->getRobotGeom() && o2 == iRobotSim->getGroundGeom()) ||
        (o2 == iRobotSim->getRobotGeom() && o1 == iRobotSim->getGroundGeom()))
      return;

    //If we are contacted with a castor, then set the contact to slip
    if (o1 == iRobotSim->getCasterGeom() || o2 == iRobotSim->getCasterGeom())
      surface_mu = 0;


    //create a contact joint to simulate collisions
    dContact contact[MaxContacts];
    int nContacts = dCollide (o1,o2,MaxContacts,
        &contact[0].geom,sizeof(dContact));
    for (int i=0; i<nContacts; i++) {
      //contact[i].surface.mode = dContactSoftCFM ; //| dContactBounce; // | dContactSoftCFM;
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
        dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = surface_mu;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.005;
      contact[i].surface.soft_cfm = 0.003;

      dJointID c = dJointCreateContact (iRobotSim->getWorld(),
          iRobotSim->getContactgroup(),&contact[i]);
      dJointAttach (c,
          dGeomGetBody(contact[i].geom.g1),
          dGeomGetBody(contact[i].geom.g2));
    }
  }
}

// ######################################################################
IRobotSim::IRobotSim(OptionManager& mgr, const std::string& descrName,
    const std::string& tagName, bool showWorld) :
  ModelComponent(mgr, descrName, tagName),
  itsRobotStartZ(0.3), //the start loc of the robot
  itsRobotRadius(0.33), //the radius of the sub in meters
  itsRobotHeight(0.06), //the radius of the sub in meters
  itsRobotWeight(1), // the weight of the sub in kg
  itsRobotWheelRadius(0.02), // the wheel size
  itsRightWheelSpeed(0),
  itsLeftWheelSpeed(0),
  itsXPos(0),
  itsYPos(0),
  itsOri(0),
  itsRightEncoder(0),
  itsLeftEncoder(0),
  itsPrevRightWheelAng(0),
  itsPrevLeftWheelAng(0),
  itsWorldView(true),
  itsShowWorld(showWorld),
  itsWorldDisp(NULL)
{
  pthread_mutex_init(&itsDispLock, NULL);

}

IRobotSim::~IRobotSim()
{
  for(uint i=0; i<itsObjects.size(); i++)
  {
    Object obj = itsObjects[i];
    if (obj.texturePtr)
      delete obj.texturePtr;
  }

  dSpaceDestroy(space);
  dWorldDestroy(world);

  delete itsVP;
  delete itsWorldDisp;
  pthread_mutex_destroy(&itsDispLock);
}

void IRobotSim::start2()
{
  //        setDrawStuff();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world,0,0,-0.5);

  ground = dCreatePlane(space, 0, 0, 1, 0);
  dGeomSetCategoryBits(ground, GROUND_BITFIELD);

  //dWorldSetCFM (world,1e-6);
  //dWorldSetERP (world,1);
  //dWorldSetAutoDisableFlag (world,1);
  dWorldSetContactMaxCorrectingVel (world,0.1);
  //set the contact penetration
  dWorldSetContactSurfaceLayer(world, 0.0001);



}

void IRobotSim::initViewport()
{
  itsVP = new ViewPort("IRobotSim", "grass.ppm", "sky.ppm");

  makeRobot();
  makeWorld();

  //Overhead cam
  //double xyz[3] = {0 , 0, 40};
  //double hpr[3] = {90.0,-90,0.0};

  double xyz[3] = {0.6 , -1.5, 2};
  double hpr[3] = {90.0,-25,0.0};
  itsVP->dsSetViewpoint (xyz,hpr);
}

void IRobotSim::makeRobot()
{
  dMass mass;

  //The body
  itsRobotBody = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCylinder(&mass,itsRobotWeight,3,itsRobotRadius,itsRobotHeight);
  dMassAdjust(&mass, itsRobotWeight);
  dBodySetMass(itsRobotBody,&mass);

  dBodySetPosition(itsRobotBody,0,0,itsRobotStartZ);
  itsRobotGeom = dCreateCCylinder(space, itsRobotRadius, itsRobotHeight);
  dGeomSetBody(itsRobotGeom, itsRobotBody);
  //dGeomSetCategoryBits(itsRobotGeom,ROBOT_BITFIELD);
  //dGeomSetCollideBits(itsRobotGeom,!ROBOT_BITFIELD && !GROUND_BITFIELD); //don't colide with the ground

  // wheel bodies
  dQuaternion q;
  dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
  dMassSetSphere (&mass,1,itsRobotWheelRadius);
  dMassAdjust (&mass,0.2);

  itsRightWheelBody = dBodyCreate (world);
  dBodySetQuaternion (itsRightWheelBody,q);
  dBodySetMass (itsRightWheelBody,&mass);
  dGeomID rWheelSphere = dCreateSphere (space,itsRobotWheelRadius);
  dGeomSetBody (rWheelSphere,itsRightWheelBody);
  dBodySetPosition (itsRightWheelBody,-itsRobotRadius/2, itsRobotRadius,itsRobotStartZ-itsRobotHeight-0.01);
  //dGeomSetCategoryBits(rWheelSphere,ROBOT_BITFIELD);
  //dGeomSetCollideBits(rWheelSphere,!ROBOT_BITFIELD); //don't colide with the Robot

  itsLeftWheelBody = dBodyCreate (world);
  dBodySetQuaternion (itsLeftWheelBody,q);
  dBodySetMass (itsLeftWheelBody,&mass);
  dGeomID lWheelSphere = dCreateSphere (space,itsRobotWheelRadius);
  dGeomSetBody (lWheelSphere,itsLeftWheelBody);
  dBodySetPosition (itsLeftWheelBody,-itsRobotRadius/2, -itsRobotRadius,itsRobotStartZ-itsRobotHeight-0.01);
  //dGeomSetCategoryBits(lWheelSphere,ROBOT_BITFIELD);
  //dGeomSetCollideBits(lWheelSphere,!ROBOT_BITFIELD); //don't colide with the Robot

  //Wheel hinges
  itsRightWheelJoint = dJointCreateHinge2 (world,0);
  dJointAttach (itsRightWheelJoint,itsRobotBody,itsRightWheelBody);
  const dReal *rWheelPos = dBodyGetPosition (itsRightWheelBody);
  dJointSetHinge2Anchor (itsRightWheelJoint,rWheelPos[0],rWheelPos[1],rWheelPos[2]);
  dJointSetHinge2Axis1  (itsRightWheelJoint,0,0,1);
  dJointSetHinge2Axis2  (itsRightWheelJoint,0,1,0);
  // set joint suspension
  dJointSetHinge2Param (itsRightWheelJoint,dParamSuspensionERP,0.002);
  dJointSetHinge2Param (itsRightWheelJoint,dParamSuspensionCFM,0.004);

  // lock all wheels along the steering axis
  // set stops to make sure wheels always stay in alignment
  dJointSetHinge2Param (itsRightWheelJoint,dParamLoStop,0);
  dJointSetHinge2Param (itsRightWheelJoint,dParamHiStop,0);

  itsLeftWheelJoint = dJointCreateHinge2 (world,0);
  dJointAttach (itsLeftWheelJoint,itsRobotBody,itsLeftWheelBody);
  const dReal *lWheelPos = dBodyGetPosition (itsLeftWheelBody);
  dJointSetHinge2Anchor (itsLeftWheelJoint,lWheelPos[0],lWheelPos[1],lWheelPos[2]);
  dJointSetHinge2Axis1  (itsLeftWheelJoint,0,0,1);
  dJointSetHinge2Axis2  (itsLeftWheelJoint,0,1,0);
  // set joint suspension
  dJointSetHinge2Param (itsLeftWheelJoint,dParamSuspensionERP,0.002);
  dJointSetHinge2Param (itsLeftWheelJoint,dParamSuspensionCFM,0.004);

  // lock all wheels along the steering axis
  // set stops to make sure wheels always stay in alignment
  dJointSetHinge2Param (itsLeftWheelJoint,dParamLoStop,0);
  dJointSetHinge2Param (itsLeftWheelJoint,dParamHiStop,0);


  //Create the castor
  dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
  dMassSetSphere (&mass,1,itsRobotWheelRadius);
  dMassAdjust (&mass,0.2);

  itsCasterBody = dBodyCreate (world);
  dBodySetQuaternion (itsCasterBody,q);
  dBodySetMass (itsCasterBody,&mass);
  dGeomID itsCasterGeom = dCreateSphere (space,itsRobotWheelRadius);
  dGeomSetBody (itsCasterGeom,itsCasterBody);
  dBodySetPosition (itsCasterBody,itsRobotRadius, 0, itsRobotStartZ-itsRobotHeight-0.01);
  //dGeomSetCategoryBits(itsCasterGeom,ROBOT_BITFIELD);
  //dGeomSetCollideBits(itsCasterGeom,!ROBOT_BITFIELD); //don't colide with the Robot

  dJointID casterJoint = dJointCreateHinge2 (world,0);
  dJointAttach (casterJoint,itsRobotBody,itsCasterBody);
  const dReal *casterPos = dBodyGetPosition (itsCasterBody);
  dJointSetHinge2Anchor (casterJoint,casterPos[0],casterPos[1],casterPos[2]);
  dJointSetHinge2Axis1  (casterJoint,0,0,1);
  dJointSetHinge2Axis2  (casterJoint,0,1,0);
  // set joint suspension
  dJointSetHinge2Param (casterJoint,dParamSuspensionERP,0.00002);
  dJointSetHinge2Param (casterJoint,dParamSuspensionCFM,0.00004);
  dJointSetHinge2Param (casterJoint,dParamLoStop,0);
  dJointSetHinge2Param (casterJoint,dParamHiStop,0);

}

void IRobotSim::drawRobot()
{
  //double r, length;
  dReal r, length;

  dGeomCCylinderGetParams(itsRobotGeom,&r,&length);

  itsVP->dsSetColor(1,1,0);

  //Draw Body
  itsVP->dsDrawCylinder(
      dBodyGetPosition(itsRobotBody),
      dBodyGetRotation(itsRobotBody),
      length,
      r);

  //Draw Wheels
  itsVP->dsSetColor(0,0,0);
  itsVP->dsDrawCylinder (dBodyGetPosition(itsRightWheelBody),
      dBodyGetRotation(itsRightWheelBody),0.02f,itsRobotWheelRadius);
  itsVP->dsDrawCylinder (dBodyGetPosition(itsLeftWheelBody),
      dBodyGetRotation(itsLeftWheelBody),0.02f,itsRobotWheelRadius);

  //Draw Caster
  itsVP->dsDrawSphere (dBodyGetPosition(itsCasterBody),
      dBodyGetRotation(itsCasterBody),itsRobotWheelRadius);

}


void IRobotSim::makeWorld()
{

  initRandomNumbersZero();

  //Draw 50 random trees
  //for(int i=0; i<50; i++)
  //{
  //  double initPos[3] = { 0, 0, 0};
  //  double objSize[3] = { 0.20, 3.5, 0.0};

  //  initPos[0] = (randomDouble()*50-25) + 2;
  //  initPos[1] = (randomDouble()*50-25) + 2;

  //  addObject(TREE, initPos, objSize);
  //}

  double initPos[3] = { -25.0, 25.0, 0.0};
  double objSize[3] = { 2, 2, 2};
  addObject(TOBJ, initPos, objSize, true, "./etc/textures/detail040.ppm");

  initPos[0] = -5; initPos[1] = 16; initPos[2] = 0;
  objSize[0] = 3; objSize[1] = 2;  objSize[2] = 5;
  addObject(TOBJ, initPos, objSize, true, "./etc/textures/build123.ppm");

  initPos[0] = -7; initPos[1] = 7; initPos[2] = 0;
  objSize[0] = 6; objSize[1] = 4;  objSize[2] = 10;
  addObject(TOBJ, initPos, objSize, true, "./etc/textures/build123.ppm");

  initPos[0] = 5; initPos[1] = 10; initPos[2] = 0;
  objSize[0] = 3; objSize[1] = 2;  objSize[2] = 5;
  addObject(TOBJ, initPos, objSize, true, "./etc/textures/build085.ppm");

  initPos[0] = -5; initPos[1] = -5; initPos[2] = 0;
  objSize[0] = 3; objSize[1] = 2;  objSize[2] = 5;
  addObject(TOBJ, initPos, objSize, true, "./etc/textures/build085.ppm");

  initPos[0] = 5; initPos[1] = -10; initPos[2] = 0;
  objSize[0] = 3; objSize[1] = 2;  objSize[2] = 5;
  addObject(TOBJ, initPos, objSize, true, "./etc/textures/brick.ppm");

  //Place trees at waypoints

  objSize[0] = 0.1; objSize[1] = 3.5;  objSize[2] = 0;

  initPos[0] = 0.0; initPos[1] = 0.0; initPos[2] = 0;
  addObject(TREE, initPos, objSize, false);

  initPos[0] = -1800/100; initPos[1] = 200/100; initPos[2] = 0;
  addObject(TREE, initPos, objSize, false);

  initPos[0] = -1800/100; initPos[1] = 2300/100; initPos[2] = 0;
  addObject(TREE, initPos, objSize, false);

  initPos[0] = 400/100; initPos[1] = 1800/100; initPos[2] = 0;
  addObject(TREE, initPos, objSize, false);

}

void IRobotSim::addObject(OBJECT_TYPE objType,
    double initPos[3],double objSize[3],
    bool addToSpace,
    const char* texture)
{

  dMass m;

  Object obj;
  obj.body = dBodyCreate(world);
  obj.texturePtr = NULL;

  dSpaceID        spaceToAdd = 0;
  if (addToSpace)
    spaceToAdd = space;

  switch(objType)
  {
    case BOX:
      dMassSetZero(&m);
      dMassSetBoxTotal(&m, 1, objSize[0], objSize[1], objSize[2]);
      dBodySetMass(obj.body, &m);
      obj.geom = dCreateBox(spaceToAdd, objSize[0], objSize[1], objSize[2]);
      dBodySetPosition(obj.body, initPos[0], initPos[1], initPos[2]);
      dGeomSetBody(obj.geom, obj.body);
      obj.color[0] = 0; obj.color[1] = 196; obj.color[2] = 127; //1 0 0 Red object
      obj.texture = ViewPort::WOOD;
      obj.type = BOX;
      break;
    case TREE:
      obj.geom = dCreateCCylinder(spaceToAdd, objSize[0], objSize[1]);
      dGeomSetBody(obj.geom, 0);
      dGeomSetPosition(obj.geom, initPos[0], initPos[1], initPos[2]);
      obj.texture = ViewPort::WOOD;
      obj.color[0] = 0.543; obj.color[1] = 0.270; obj.color[2] = 0.074; //1 0 0 Red object
      obj.type = TREE;
      break;
    case DS:
      itsVP->load3DSObject(obj.object, "./tests/spaceship.3ds");
      obj.object.scale = 0.01;
      obj.texture = ViewPort::OTHER;
      obj.texturePtr = new Texture("./etc/textures/spaceshiptexture.ppm");
      obj.geom = dCreateBox(spaceToAdd, objSize[0], objSize[1], objSize[2]);
      dGeomSetBody(obj.geom, 0);
      dGeomSetPosition(obj.geom, initPos[0], initPos[1], initPos[2]);
      obj.color[0] = 1.0; obj.color[1] = 1.0; obj.color[2] = 1.0; //1 0 0 Red object
      obj.type = DS;
      break;
    case TOBJ:
      sprintf(obj.object.name, "Textured Object");
      obj.object.vertices_qty = 8;
      obj.object.vertex.resize(8);
      obj.object.vertex[0] = ViewPort::vertex_type(0, 0, objSize[2]); //, // vertex v0
      obj.object.vertex[1] = ViewPort::vertex_type( objSize[0], 0, objSize[2]); // vertex v1
      obj.object.vertex[2] = ViewPort::vertex_type( objSize[0], 0, 0); // vertex v2
      obj.object.vertex[3] = ViewPort::vertex_type( 0, 0, 0); // vertex v3

      obj.object.vertex[4] = ViewPort::vertex_type( 0, objSize[1], objSize[2]); // vertex v4
      obj.object.vertex[5] = ViewPort::vertex_type( objSize[0], objSize[1], objSize[2]); // vertex v5
      obj.object.vertex[6] = ViewPort::vertex_type( objSize[0], objSize[1], 0); // vertex v6
      obj.object.vertex[7] = ViewPort::vertex_type( 0, objSize[1], 0); // vertex v7


      obj.object.polygons_qty = 12;
      obj.object.polygon.resize(12);
      obj.object.polygon[0] = ViewPort::polygon_type(0, 1, 4); // polygon v0,v1,v4
      obj.object.polygon[1] = ViewPort::polygon_type(1, 5, 4); // polygon v1,v5,v4
      obj.object.polygon[2] = ViewPort::polygon_type(1, 2, 5); // polygon v1,v2,v5
      obj.object.polygon[3] = ViewPort::polygon_type(2, 6, 5); // polygon v2,v6,v5
      obj.object.polygon[4] = ViewPort::polygon_type(2, 3, 6); // polygon v2,v3,v6
      obj.object.polygon[5] = ViewPort::polygon_type(3, 7, 6); // polygon v3,v7,v6
      obj.object.polygon[6] = ViewPort::polygon_type(3, 0, 7); // polygon v3,v0,v7
      obj.object.polygon[7] = ViewPort::polygon_type(0, 4, 7); // polygon v0,v4,v7
      obj.object.polygon[8] = ViewPort::polygon_type(4, 5, 7); // polygon v4,v5,v7
      obj.object.polygon[9] = ViewPort::polygon_type(5, 6, 7); // polygon v5,v6,v7
      obj.object.polygon[10] = ViewPort::polygon_type(3, 2, 0); // polygon v3,v2,v0
      obj.object.polygon[11] = ViewPort::polygon_type(2, 1, 0); // polygon v2,v1,v0

      obj.object.mapcoord.resize(8);
      obj.object.mapcoord[0] = ViewPort::mapcoord_type(0.0, 0.0); // mapping coordinates for vertex v0
      obj.object.mapcoord[1] = ViewPort::mapcoord_type(1.0, 0.0); // mapping coordinates for vertex v1
      obj.object.mapcoord[2] = ViewPort::mapcoord_type(1.0, 1.0); // mapping coordinates for vertex v2
      obj.object.mapcoord[3] = ViewPort::mapcoord_type(0.0, 1.0); // mapping coordinates for vertex v3
      obj.object.mapcoord[4] = ViewPort::mapcoord_type(0.0, 0.0); // mapping coordinates for vertex v4
      obj.object.mapcoord[5] = ViewPort::mapcoord_type(0.0, 1.0); // mapping coordinates for vertex v5
      obj.object.mapcoord[6] = ViewPort::mapcoord_type(1.0, 1.0); // mapping coordinates for vertex v6
      obj.object.mapcoord[7] = ViewPort::mapcoord_type(0.0, 1.0); // mapping coordinates for vertex v7


      obj.object.scale = 1;
      if (texture)
      {
        obj.texture = ViewPort::OTHER;
        obj.texturePtr = new Texture(texture);
      }
      obj.geom = dCreateBox(spaceToAdd, objSize[0], objSize[1], objSize[2]);
      dGeomSetBody(obj.geom, 0);
      dGeomSetPosition(obj.geom, initPos[0], initPos[1], initPos[2]);
      obj.color[0] = 256; obj.color[1] = 256; obj.color[2] = 256;
      obj.type = DS;
      break;


    default:
      LINFO("Unknown object");
      break;
  }
  itsObjects.push_back(obj);


}



void IRobotSim::drawWorld()
{

  for(uint i=0; i<itsObjects.size(); i++)
  {
    Object obj = itsObjects[i];

    itsVP->dsSetColor(obj.color[0],obj.color[1],obj.color[2]);

    itsVP->dsSetTexture(obj.texture, obj.texturePtr);

    const dReal *bodyPos = dGeomGetPosition(obj.geom);
    const dReal *bodyRot = dGeomGetRotation(obj.geom);

    switch(obj.type)
    {
      case BOX:
        dReal size[3];
        dGeomBoxGetLengths(obj.geom,size);
        itsVP->dsDrawBox(bodyPos, bodyRot, size);
        break;
      case TREE:
        dReal r, length;
        dGeomCCylinderGetParams(obj.geom,&r,&length);

        //Move the cylinder down by the r since its a capted cylinder
        //The cap cylinder is used wince its is more stable
        dReal pos[3];
        pos[0] = bodyPos[0];
        pos[1] = bodyPos[1];
        pos[2] = bodyPos[2] - r;

        itsVP->dsDrawCylinder(pos, bodyRot, length, r);

        //Draw the top of the tree
        pos[2] = bodyPos[2] + length/2;

        itsVP->dsSetTexture(ViewPort::TREE);
        itsVP->dsSetColor(0,1,0);
        itsVP->dsDrawSphere (pos,
            dBodyGetRotation(obj.body),r*4);

        break;
      case DS:
      case TOBJ:
        itsVP->dsDraw3DSObject(bodyPos, bodyRot, obj.object);
        break;

      default:
        break;
    }
  }
}

void IRobotSim::handleWinEvents(XEvent& event)
{
}

void IRobotSim::updateSensors()
{

  //GPS sensor update
  const dReal *pos = dBodyGetPosition(itsRobotBody);
  const dReal *R = dBodyGetRotation(itsRobotBody);

  itsXPos = pos[0];
  itsYPos = pos[1];
  itsOri   = atan2(R[4], R[0]);      //greek Y
  if (itsOri < 0) itsOri += M_PI*2;



  //Encoder sensors
//itsRoll  = atan2(R[9], R[10]) + M_PI/2;     //phi correct for initial rotation
//  itsPitch = asin(-R[8]);            //theta
//  itsYaw   = atan2(R[4], R[0]);
//  const dReal *rightR = dBodyGetRotation(itsRightWheelBody);
//  //const dReal *leftR = dBodyGetRotation(itsLeftWheelBody);
//
//  int rightWheelAng = (int)((asin(-rightR[8]) + M_PI/2)*180/M_PI);
//  //int leftWheelAng = (int)((asin(-leftR[8]) + M_PI/2)*180/M_PI);
//
//  int tick;
//  if (itsPrevRightWheelAng > rightWheelAng)
//    tick = 1;
//  else
//    tick = 0;
//  LINFO("Ang %i %i %i", rightWheelAng*(tick*-1) + tick*+180, itsPrevRightWheelAng - rightWheelAng, tick);
//
//  itsPrevRightWheelAng = rightWheelAng;

  // LINFO("(%f,%f) Depth %f, roll %f pitch %f yaw %f",
  //     itsXPos, itsYPos, itsDepth,
  //     itsRoll, itsPitch, itsYaw);

}

void IRobotSim::simLoop()
{

  //Move the robot
  dJointSetHinge2Param (itsRightWheelJoint,dParamVel2,itsRightWheelSpeed);
  dJointSetHinge2Param (itsRightWheelJoint,dParamFMax2,0.5);

  dJointSetHinge2Param (itsLeftWheelJoint,dParamVel2,itsLeftWheelSpeed);
  dJointSetHinge2Param (itsLeftWheelJoint,dParamFMax2,0.5);

  //update the sensors
  updateSensors();

  dSpaceCollide (space,this,&nearCallback); //check for collisions

  dWorldStep(world,0.1);

  dJointGroupEmpty (contactgroup); //delete the contact joints

  if (itsShowWorld)
  {
    itsWorldDisp->drawImage(flipVertic(getFrame(0)));
  }
}


Image<PixRGB<byte> > IRobotSim::getFrame(int camera)
{
  const dReal *bodyPos = dBodyGetPosition(itsRobotBody);
  const dReal *bodyR = dBodyGetRotation(itsRobotBody);

  double cam_xyz[3], cam_hpr[3] = {0.0,0.0,0.0};

  switch (camera)
  {
    case 0: //world camera
      cam_xyz[0] = 0;
      cam_xyz[1] = 0;
      cam_xyz[2] = 40;

      cam_hpr[0]   = 90.0;
      cam_hpr[1]   = -90.0;
      cam_hpr[2]   = 0.0;

      break;
    case 1:
      cam_xyz[0] = bodyPos[0];
      cam_xyz[1] = bodyPos[1];
      cam_xyz[2] = bodyPos[2] + 1;

      cam_hpr[0]   = (atan2(bodyR[4], bodyR[0])*180/M_PI) + 180; //yaw

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
    itsVP->dsSetViewpoint (cam_xyz,cam_hpr);
  itsVP->initFrame();
  drawRobot();
  drawWorld();
  itsVP->updateFrame();
  pthread_mutex_unlock(&itsDispLock);


  return itsVP->getFrame();

}

void IRobotSim::getSensors(float &xPos, float &yPos, float &ori)
{
  xPos = itsXPos;
  yPos = itsYPos;
  ori = itsOri;

  if (false)
  {
    //Add some random gauss noise to the sensor
    int idum = getIdum();
    xPos += gasdev(idum)*0.5;
    yPos += gasdev(idum)*0.5;
    ori += gasdev(idum)*0.2;
  }
}

void IRobotSim::setMotors(double rightSpeed, double leftSpeed)
{
  itsRightWheelSpeed = rightSpeed;
  itsLeftWheelSpeed = leftSpeed;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
