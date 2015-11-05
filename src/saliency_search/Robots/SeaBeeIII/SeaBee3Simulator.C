/*!@file Robots/SeaBeeIII/SeaBee3Simulator.C Sub Simulator */

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
// Primary maintainer for this file: Randolph Voorhies <voorhies at usc dot edu>

#include "Robots/SeaBeeIII/SeaBee3Simulator.H"
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
    SeaBee3Simulator *subSim = (SeaBee3Simulator *)data;
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
SeaBee3Simulator::SeaBee3Simulator(OptionManager& mgr, const std::string& descrName,
    const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsPoolDepth(4.5),
  itsPoolLength(70),
  itsPoolWidth(20),
  itsWaterLevel(4.57), //15 feet of water
  itsSubLength(1.5), //the length of sub in meters
  itsSubRadius(0.2), //the radius of the sub in meters
  itsSubWeight(20), // the weight of the sub in kg
  vp(new ViewPort("SeaBee3Simulator", "sea_floor.ppm", "sky.ppm", true, true)),
  itsWorldView(true),
  itsShowWorld(true),
  itsWorldDisp(NULL),
  itsCameraUpdateTime(.0333333),
  itsBeeStemUpdateTime(.1),
  itsSimXPos(0),
  itsSimYPos(0),
  itsSimDepth(0),
  itsSimRoll(0),
  itsSimPitch(0),
  itsSimYaw(0),
  itsPIDsEnabled(false),
  itsDepthPID(new PID<float>(0,0,0,0,100)),
  itsHeadingPID(new PID<Angle>(0,0,0,0,100))
{
  dInitODE();

  Initialized = false;

  //  vp->isSubSim = true;

  itsWorldDisp = new XWinManaged(vp->getDims(), -1, -1, "World View");

  pthread_mutex_init(&itsDispLock, NULL);

  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground = dCreatePlane(space, 0, 0, 1, 0);

  //Initialize the world's gravity
  dWorldSetGravity(world,0,0,-9.81);

  dWorldSetCFM (world,1e-6);
  dWorldSetERP (world,1);
  //dWorldSetAutoDisableFlag (world,1);
  dWorldSetContactMaxCorrectingVel (world,0.1);
  //set the contact penetration
  dWorldSetContactSurfaceLayer(world, 0.001);

  makeSub();

  //set the viewpoint
  double xyz[3] = {0 , -3.0, 15};
  double hpr[3] = {90.0,-45,0.0};
  vp->dsSetViewpoint (xyz,hpr);

  arenaWallTexture    = new Texture("etc/textures/sea_floor.ppm");
  waterSurfaceTexture = new Texture("etc/textures/water.ppm");

  itsJSValues.resize(8);
  itsButValues.resize(20);

  // init trigger values
  itsJSValues[4] = -100;
  itsJSValues[5] = -100;
}

SeaBee3Simulator::~SeaBee3Simulator()
{
  dSpaceDestroy(space);
  dWorldDestroy(world);

  delete vp;
  delete itsWorldDisp;
  pthread_mutex_destroy(&itsDispLock);
}

void SeaBee3Simulator::registerTopics()
{
  registerPublisher("RetinaMessageTopic");
  registerPublisher("BeeStemMessageTopic");
  registerSubscription("XBox360RemoteControlMessageTopic");
  registerSubscription("BeeStemConfigTopic");

}

void SeaBee3Simulator::makeSub()
{
  dMass mass;
  itsSubBody = dBodyCreate(world);
  //pos[0] = 0; pos[1] = 1.0*5; pos[2] = 1.80;

  //dBodySetPosition(itsSubBody,10,5,1.94); //Sub is 10 feet underwater

  //Initialize the position of the submarine to (10, 5) and just below the surface
  dBodySetPosition(itsSubBody,10,5,itsPoolDepth-.1);

  //Turn the submarine towards the long end of the pool
  dMatrix3 R;
  dRFromAxisAndAngle (R,1,0,0,-M_PI/2);
  dBodySetRotation(itsSubBody, R);

  //Set the submarine's mass and geometry
  dMassSetZero(&mass);
  dMassSetCappedCylinderTotal(&mass,itsSubWeight,3,itsSubRadius,itsSubLength);
  dMassRotate(&mass, R);
  dBodySetMass(itsSubBody,&mass);
  itsSubGeom = dCreateCCylinder(space, itsSubRadius, itsSubLength);
  dGeomSetRotation(itsSubGeom, R);
  dGeomSetBody(itsSubGeom, itsSubBody);
}

void SeaBee3Simulator::drawSub()
{
  dReal r, length;

  dGeomCCylinderGetParams(itsSubGeom,&r,&length);

  vp->dsSetColor(1,1,0);
  vp->dsDrawCappedCylinder(
      dBodyGetPosition(itsSubBody),
      dBodyGetRotation(itsSubBody),
      length,
      r);
}

void SeaBee3Simulator::drawArena()
{
  double pos[3];

  drawWaterSurface();
  drawPool();

  pos[0] = 10; pos[1] = 10; pos[2] = 2.80;
  drawGate(pos);

  pos[0] = 11; pos[1] = 17; pos[2] = 1.83;
  drawBuoy(pos);

  pos[0] = 10; pos[1] = 20; pos[2] = 0.6;
  drawPipeline(M_PI/2.0, pos);

  pos[0] = 10; pos[1] = 25; pos[2] = 0.6;
  drawBin(0, pos);

  pos[0] = 10; pos[1] = 27; pos[2] = 0.6;
  drawPipeline(M_PI/4.0, pos);

  pos[0] = 12; pos[1] = 29; pos[2] = 0.6;
  drawPipeline(M_PI/2.0, pos);

  pos[0] = 12; pos[1] = 32; pos[2] = 0.6;
  drawPipeline(-M_PI/4, pos);

  pos[0] = 8; pos[1] = 37; pos[2] = 0.6;
  drawBin(M_PI/2, pos);

  pos[0] = 8; pos[1] = 37; pos[2] = 1.83;
  drawBuoy(pos);

//  pos[0] = 60; pos[1] = 4.33*5; pos[2] = 0.6;
//  drawPipeline(-M_PI/4, pos);
//
//  pos[0] = 70; pos[1] = 3.5*5; pos[2] = 0.6;
//  drawPinger(pos);
}


void SeaBee3Simulator::drawGate(const double *gatePos)
{
  double gateHeight = 3.0; //The length of the gate's side tubes

  double pos[3];
  double R[12];

  //Gate Color
  vp->dsSetColor(0,0,0);

  //Top
  pos[0] = gatePos[0]; pos[1] = gatePos[1]; pos[2] = gatePos[2];
  dRFromAxisAndAngle (R,0,1,0,-M_PI/2);
  vp->dsDrawCappedCylinder(pos, R, 3.05f, 0.1f);

  //Sides
  dRSetIdentity(R);
  pos[0] = gatePos[0]-(3.05/2); pos[1] = gatePos[1]; pos[2] = gatePos[2] - gateHeight/2;
  vp->dsDrawCappedCylinder(pos, R, gateHeight, 0.1);

  pos[0] = gatePos[0]+(3.05/2); pos[1] = gatePos[1]; pos[2] = gatePos[2] - gateHeight/2;
  vp->dsDrawCappedCylinder(pos, R, gateHeight, 0.1);
}


void SeaBee3Simulator::drawBuoy(const double *bouyPos)
{
  static int frame = 0;
  static bool bouyOn = false;
  double pos[3];
  double R[12];

  //start Buoy
  //flash buoy
  if (frame++ > 5) //this sets the frame rate
  {
    bouyOn = !bouyOn;
    frame = 0;
  }

  if (bouyOn)
    vp->dsSetColor(1,0,0);
  else
    vp->dsSetColor(0.5,0,0);
  pos[0] = bouyPos[0]; pos[1] = bouyPos[1]; pos[2] = bouyPos[2];
  dRSetIdentity(R);
  vp->dsDrawSphere(pos, R, 0.20);
  double pos1[3];
  vp->dsSetColor(0,0,0);
  pos1[0] = pos[0]; pos1[1] = pos[1]; pos1[2] = 0;
  vp->dsDrawLine(pos, pos1);
}

void SeaBee3Simulator::drawWaterSurface()
{
  //I haven't the slightest clue why this doesn't work.
  static double texOff=0.0f;
  texOff += 0.1f;

  //Set the water surface texture
  glEnable (GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, waterSurfaceTexture->getName());

  //Draw the bottom water surface
  glBegin(GL_QUADS);
  {
    glNormal3f(0,0,-1);
    glTexCoord2f(0.0f,  0.0f+texOff); glVertex3f( -5.0f,             -5.0f,              itsPoolDepth);
    glTexCoord2f(1.0f,  0.0f+texOff); glVertex3f( itsPoolWidth+5.0f, -5.0f,              itsPoolDepth);
    glTexCoord2f(1.0f,  1.0f+texOff); glVertex3f( itsPoolWidth+5.0f, itsPoolLength+5.0f, itsPoolDepth);
    glTexCoord2f(0.0f,  1.0f+texOff); glVertex3f( -5.0f,             itsPoolLength+5.0f, itsPoolDepth);

    //Draw the top water surface
    glNormal3f(0,0,1);
    glTexCoord2f(0.0f,  0.0f+texOff); glVertex3f( -5.0f,             -5.0f,              itsPoolDepth);
    glTexCoord2f(0.0f,  1.0f+texOff); glVertex3f( -5.0f,             itsPoolLength+5.0f, itsPoolDepth);
    glTexCoord2f(1.0f,  1.0f+texOff); glVertex3f( itsPoolWidth+5.0f, itsPoolLength+5.0f, itsPoolDepth);
    glTexCoord2f(1.0f,  0.0f+texOff); glVertex3f( itsPoolWidth+5.0f, -5.0f,              itsPoolDepth);
  }
  glEnd();

  if(texOff > 1.0f) texOff = 0.0f;
}
void SeaBee3Simulator::drawPool()
{
  double pos[3];
  double R[12];
  double sides[3];

  double WallAngle = M_PI/8;

  double WallHeight = itsPoolDepth / cos(WallAngle);

  //Draw far wall
  pos[0] = itsPoolWidth/2.0; pos[1] = itsPoolLength; pos[2] = itsPoolDepth/2;
  dRFromAxisAndAngle(R, 1,0,0,-WallAngle);
  sides[0] = itsPoolWidth*1.5; sides[1] = 0.5; sides[2] = WallHeight;
  vp->dsSetTexture(ViewPort::OTHER,arenaWallTexture);
  vp->dsDrawBox(pos, R, sides);

  //Draw back wall
  pos[0] = itsPoolWidth/2.0; pos[1] = 0; pos[2] = itsPoolDepth/2;
  dRFromAxisAndAngle(R, 1,0,0,WallAngle);
  sides[0] = itsPoolWidth*1.5; sides[1] = 0.5; sides[2] = WallHeight;
  vp->dsSetTexture(ViewPort::OTHER,arenaWallTexture);
  vp->dsDrawBox(pos, R, sides);

  //Draw right wall
  pos[0] = itsPoolWidth; pos[1] = itsPoolLength/2.0; pos[2] = itsPoolDepth/2;
  dRFromAxisAndAngle(R, 0,1,0,WallAngle);
  sides[0] = 0.5; sides[1] = itsPoolLength*1.5; sides[2] = WallHeight;
  vp->dsSetTexture(ViewPort::OTHER,arenaWallTexture);
  vp->dsDrawBox(pos, R, sides);

  //Draw left wall
  pos[0] = 0; pos[1] = itsPoolLength/2.0; pos[2] = itsPoolDepth/2;
  dRFromAxisAndAngle(R, 0,1,0,-WallAngle);
  sides[0] = 0.5; sides[1] = itsPoolLength*1.5; sides[2] = WallHeight;
  vp->dsSetTexture(ViewPort::OTHER,arenaWallTexture);
  vp->dsDrawBox(pos, R, sides);
}

void SeaBee3Simulator::drawPipeline(const double ori, const double *pipePos)
{

  double sides[3] = {1.2, 0.15, 0.1};
  double R[12];

  dRFromAxisAndAngle (R,0,0,1,ori);

  vp->dsSetColor(1,0.5,0);

  vp->dsDrawBox(pipePos, R, sides);
}

void SeaBee3Simulator::drawBin(const double ori, const double *binPos)
{

  double sides[3];
  double R[12];

  dRFromAxisAndAngle (R,0,0,1,ori);

  vp->dsSetColor(1,1,1);
  sides[0] = 0.6; sides[1] = 0.8; sides[2] = 0.1;
  vp->dsDrawBox(binPos, R, sides);

  vp->dsSetColor(0,0,0);
  sides[0] = 0.3; sides[1] = 0.6; sides[2] = 0.15;
  vp->dsDrawBox(binPos, R, sides);
}

void SeaBee3Simulator::drawPinger(const double *pingerPos)
{
  double pos[3];
  double R[12];

  vp->dsSetColor(1,1,1);
  pos[0] = pingerPos[0]; pos[1] = pingerPos[1]; pos[1] = pingerPos[2] + 1.2/2;
  vp->dsDrawCappedCylinder(pos, R, 1.2, 0.1);

}


void SeaBee3Simulator::handleWinEvents(XEvent& event)
{}

void SeaBee3Simulator::updateSensors(const dReal *pos, const dReal *R)
{
  itsSimXPos  = pos[0];
  itsSimYPos  = pos[1];
  itsSimDepth = 2000 - pos[2] * 100             + randomDoubleFromNormal(3.0);
  itsSimRoll  = atan2(R[9], R[10]) + M_PI/2+ randomDoubleFromNormal(5.0); //phi correct for initial rotation
  itsSimPitch = asin(-R[8])        + randomDoubleFromNormal(5.0);         //theta
  itsSimYaw   = atan2(R[4], R[0]) * 180.0/M_PI  + randomDoubleFromNormal(2.0);         //greek Y

  if (itsSimYaw < 0) itsSimYaw += 360;


  if(itsCameraTimer.getSecs() > itsCameraUpdateTime)
  {
    //Grab an image from the forward camera, and publish it as a Retina message
    Image<PixRGB<byte> > fwdCameraImage = flipVertic(getFrame(1));

    RobotSimEvents::RetinaMessagePtr fwdRetMsg = new RobotSimEvents::RetinaMessage;
    fwdRetMsg->img = Image2Ice(fwdCameraImage);
    fwdRetMsg->cameraID = "DwnCamera";

    this->publish("RetinaMessageTopic", fwdRetMsg);

    //Grab an image from the downward camera, and publish it as a Retina message
    Image<PixRGB<byte> > dwnCameraImage = flipVertic(getFrame(2));

    RobotSimEvents::RetinaMessagePtr dwnRetMsg = new RobotSimEvents::RetinaMessage;
    dwnRetMsg->img = Image2Ice(dwnCameraImage);
    dwnRetMsg->cameraID = "FwdCamera";

    this->publish("RetinaMessageTopic", dwnRetMsg);

    itsCameraTimer.reset();
  }

  if(itsBeeStemTimer.getSecs() > itsBeeStemUpdateTime)
  {
    //Create the BeeStem message, and publish it
    RobotSimEvents::BeeStemMessagePtr BeeStemMsg = new RobotSimEvents::BeeStemMessage;
    BeeStemMsg->accelX           = 0;
    BeeStemMsg->accelY           = 0;
    BeeStemMsg->accelZ           = 0;
    BeeStemMsg->compassHeading   = itsSimYaw;
    BeeStemMsg->compassPitch     = itsSimRoll;
    BeeStemMsg->compassRoll      = itsSimPitch;
    BeeStemMsg->internalPressure = 3000 + randomDoubleFromNormal(3.0);
    BeeStemMsg->externalPressure = itsSimDepth;
    BeeStemMsg->desiredHeading   = itsDesiredHeading;
    BeeStemMsg->desiredDepth     = itsDesiredDepth;
    BeeStemMsg->desiredSpeed     = itsDesiredSpeed;
    BeeStemMsg->headingK         = itsHeadingK;
    BeeStemMsg->headingP         = itsHeadingP;
    BeeStemMsg->headingD         = itsHeadingD;
    BeeStemMsg->headingI         = itsHeadingI;
    BeeStemMsg->headingOutput    = itsHeadingOutput;
    BeeStemMsg->depthK           = itsDepthK;
    BeeStemMsg->depthP           = itsDepthP;
    BeeStemMsg->depthD           = itsDepthD;
    BeeStemMsg->depthI           = itsDepthI;
    BeeStemMsg->depthOutput      = itsDepthOutput;
    BeeStemMsg->killSwitch       = 1;
    BeeStemMsg->thruster1        = itsForwardLeftThrusterVal;
    BeeStemMsg->thruster2        = itsForwardRightThrusterVal;
    BeeStemMsg->thruster3        = itsFrontStrafeThrusterVal;
    BeeStemMsg->thruster4        = itsBackStrafeThrusterVal;
    BeeStemMsg->thruster5        = itsLeftVertThrusterVal;
    BeeStemMsg->thruster6        = itsRightVertThrusterVal;

    this->publish("BeeStemMessageTopic", BeeStemMsg);
    itsBeeStemTimer.reset();
  }
}


void SeaBee3Simulator::evolve() {usleep(10000000);}


void SeaBee3Simulator::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  // Get an XBox360RemoteControl Message
  if(eMsg->ice_isA("::RobotSimEvents::JoyStickControlMessage"))
  {
    RobotSimEvents::JoyStickControlMessagePtr msg = RobotSimEvents::JoyStickControlMessagePtr::dynamicCast(eMsg);

    itsJSMutex.lock();
    if(msg->axis >= 0)
      itsJSValues[msg->axis] = msg->axisVal;
    if(msg->button >= 0)
      itsButValues[msg->button] = msg->butVal;
    itsJSMutex.unlock();
  }
  // Get a BeeStemConfig Message
  else if(eMsg->ice_isA("::RobotSimEvents::BeeStemConfigMessage"))
  {
    RobotSimEvents::BeeStemConfigMessagePtr msg = RobotSimEvents::BeeStemConfigMessagePtr::dynamicCast(eMsg);


    if(msg->enablePID == 1)
    { //Our message contains an instruction to turn on/off the PID

      itsStemMutex.lock();
      {
        if(msg->enableVal == 1)
          itsPIDsEnabled = true;
        else
          itsPIDsEnabled = false;

        LINFO("PIDEnable=%d", itsPIDsEnabled);
      }
      itsStemMutex.unlock();

    }
    else if(msg->updateHeadingPID == 0 && msg->updateDepthPID == 0)
    {

      itsStemMutex.lock();
      {
        LINFO("Update Pose: %d\n",msg->updateDesiredValue);
        if(msg->updateDesiredValue == 1)
          itsTargetHeading = msg->desiredHeading;
        else if(msg->updateDesiredValue == 2)
          itsTargetDepth = msg->desiredDepth;
        else if(msg->updateDesiredValue == 3)
          itsTargetSpeed = msg->desiredSpeed;
      }
      itsStemMutex.unlock();

    }
    else if(msg->updateDepthPID == 1)
    {

      itsStemMutex.lock();
      {
        itsDepthPID->setPIDPgain(float(msg->depthP) / float(msg->depthK));
        itsDepthPID->setPIDIgain(float(msg->depthI) / float(msg->depthK));
        itsDepthPID->setPIDDgain(float(msg->depthD) / float(msg->depthK));
      }
      itsStemMutex.unlock();

    }
    else if(msg->updateHeadingPID == 1)
    {

      itsStemMutex.lock();
      {
        itsHeadingPID->setPIDPgain(float(msg->headingP) / float(msg->headingK));
        itsHeadingPID->setPIDIgain(float(msg->headingI) / float(msg->headingK));
        itsHeadingPID->setPIDDgain(float(msg->headingD) / float(msg->headingK));
      }
      itsStemMutex.unlock();

    }
  }
}

void SeaBee3Simulator::simLoop()
{
  //Add the appropriate forces from the motors
  dBodyAddRelForceAtRelPos(itsSubBody,
      0,0, itsForwardLeftThrusterVal/10.0,  //Force Vector
      -.25, 0, 0);                     //Force Position

  dBodyAddRelForceAtRelPos(itsSubBody,
      0,0, itsForwardRightThrusterVal/10.0, //Force Vector
      +.25, 0, 0);                     //Force Position

  dBodyAddRelForceAtRelPos(itsSubBody,
      itsFrontStrafeThrusterVal/10.0, 0,0,  //Force Vector
      0, 0, +.5);                      //Force Position

  dBodyAddRelForceAtRelPos(itsSubBody,
      itsBackStrafeThrusterVal/10.0, 0,0,   //Force Vector
      0, 0, -.5);                      //Force Position

  dBodyAddRelForceAtRelPos(itsSubBody,
      0, itsLeftVertThrusterVal/10.0, 0,    //Force Vector
      -.25, 0, 0);                     //Force Position

  dBodyAddRelForceAtRelPos(itsSubBody,
      0, itsRightVertThrusterVal/10.0, 0,   //Force Vector
      +.25, 0, 0);                     //Force Position

  //Apply a viscosity water force
  applyHydrodynamicForces(.2);

  //Update the PID controllers if necessary
  updateControllers();

  //Folow the sub with the camera
  //  const double *bodyPos = dBodyGetPosition(itsSubBody);
  //const double *bodyR = dBodyGetRotation(itsSubBody);
  const dReal *bodyPos = dBodyGetPosition(itsSubBody);
  const dReal *bodyR = dBodyGetRotation(itsSubBody);

  updateSensors(bodyPos, bodyR);

  dSpaceCollide (space,this,&nearCallback); //check for collisions

  dWorldStep(world,0.1);

  dJointGroupEmpty (contactgroup); //delete the contact joints

  itsWorldDisp->drawImage(flipVertic(getFrame(0)));
}



void SeaBee3Simulator::updateControllers()
{
  if(itsPIDsEnabled)
    {
      float DepthCommand = itsDepthPID->update(itsTargetDepth, itsSimDepth)*100;
      itsLeftVertThrusterVal = DepthCommand;
      itsRightVertThrusterVal = DepthCommand;
      itsDepthOutput = DepthCommand;

      Angle currHeadingAngle(itsSimYaw);
      Angle desHeadingAngle(itsTargetHeading);

      float HeadingCommand = (itsHeadingPID->update(desHeadingAngle, currHeadingAngle)).getRadians() * (180.0/M_PI)*100.0;
      itsForwardLeftThrusterVal  = -HeadingCommand;
      itsForwardRightThrusterVal = HeadingCommand;
      itsHeadingOutput = HeadingCommand;
    }
  else
    {
      itsJSMutex.lock();
      int rightVal = itsJSValues[1]*-1;
      int leftVal = itsJSValues[1]*-1;

      int depthVal = int((float(itsJSValues[5]) + 100.0)/2.0 - (float(itsJSValues[4]) + 100.0)/2.0);

      int depthValRight = depthVal;
      int depthValLeft = depthVal;

      int frontVal = itsJSValues[2] + itsJSValues[0];
      int backVal = itsJSValues[2]*-1 + itsJSValues[0];

      if(itsButValues[5])
        {
          depthValRight = 75;
          depthValLeft = -75;
        }
      else if(itsButValues[4])
        {
          depthValRight = -75;
          depthValLeft = 75;
        }


      //itsStem->setThruster(FWD_RIGHT_THRUSTER, rightVal);
      //itsStem->setThruster(FWD_LEFT_THRUSTER, leftVal);
      itsForwardRightThrusterVal = rightVal;
      itsForwardLeftThrusterVal = leftVal;

      //itsStem->setThruster(DEPTH_RIGHT_THRUSTER, depthValRight);
      //  itsStem->setThruster(DEPTH_LEFT_THRUSTER, depthValLeft);

      itsLeftVertThrusterVal = depthValLeft;
      itsRightVertThrusterVal = depthValRight;

      //itsStem->setThruster(STRAFE_FRONT_THRUSTER, frontVal);
      //itsStem->setThruster(STRAFE_BACK_THRUSTER, backVal);
      itsFrontStrafeThrusterVal = frontVal;
      itsBackStrafeThrusterVal = backVal;
      itsJSMutex.unlock();
    }
}


//! Calculate the water forces on the object
// Obtained from http://ode.org/pipermail/ode/2005-January/014929.html
void SeaBee3Simulator::applyHydrodynamicForces(dReal viscosity)
{
  const dReal *lvel = dBodyGetLinearVel(itsSubBody);
  const dReal *avel = dBodyGetAngularVel(itsSubBody);
  const dReal *R = dBodyGetRotation(itsSubBody);

  //Should be the area of the sub
  dReal AreaX = 10;
  dReal AreaY = 10;
  dReal AreaZ = 10;

  dReal nx = (R[0] * lvel[0] + R[4] * lvel[1] + R[8] * lvel[2]) *  AreaX;
  dReal ny = (R[1] * lvel[0] + R[5] * lvel[1] + R[9] * lvel[2]) * AreaY;
  dReal nz = (R[2] * lvel[0] + R[6] * lvel[1] + R[10] * lvel[2]) * AreaZ;

  dReal temp = -nx * viscosity;
  dBodyAddForce(itsSubBody, temp * R[0], temp * R[4], temp * R[8]);

  temp = -ny * viscosity;
  dBodyAddForce(itsSubBody, temp * R[1], temp * R[5], temp * R[9]);

  temp =-nz * viscosity;
  dBodyAddForce(itsSubBody, temp * R[2], temp * R[6], temp * R[10]);

  nx = (R[0] * avel[0] + R[4] * avel[1] + R[8] * avel[2]) * AreaZ;
  ny = (R[1] * avel[0] + R[5] * avel[1] + R[9] * avel[2]) * AreaX;
  nz = (R[2] * avel[0] + R[6] * avel[1] + R[10] * avel[2]) * AreaY;

  temp = -nx * viscosity; // * 500; //seems to strong
  dBodyAddTorque(itsSubBody, temp * R[0], temp * R[4], temp * R[8]);

  temp = -ny * viscosity; // * 500;
  dBodyAddTorque(itsSubBody, temp * R[1], temp * R[5], temp * R[9]);

  temp = -nz * viscosity; // * 500;
  dBodyAddTorque(itsSubBody, temp * R[2], temp * R[6], temp * R[10]);


  const dReal *bodyPos = dBodyGetPosition(itsSubBody);
//  LINFO("DEPTH: %f, PoolDepth: %f", bodyPos[2], itsPoolDepth);

  dMass *subMass = new dMass();
  dBodyGetMass(itsSubBody, subMass);

  double TotalBuoyancy = 9.86;

  //Apply buoyancy forces as a piecewise linear function
  if(bodyPos[2] <= itsPoolDepth)
  {
    //The sub is totally in the water, so apply all of the buoyancy
    //dBodyAddForce(itsSubBody, 0,0, subMass->mass * TotalBuoyancy);

    dBodyAddForceAtRelPos(itsSubBody,0,0,subMass->mass * TotalBuoyancy,
                          0,-0.01,0);
  }
  else if(bodyPos[2] <= itsPoolDepth + itsSubRadius)
  {
    //The sub is partially out of the water, so apply only a fraction of the
    //total buoyancy
 //    dBodyAddForce(itsSubBody, 0, 0,
//         subMass->mass *
//         (TotalBuoyancy *
//          (1 - (bodyPos[2] - itsPoolDepth)/(itsSubRadius) )
//         ));

    dBodyAddForceAtRelPos(itsSubBody,0,0,subMass->mass *
                          (TotalBuoyancy *
                           (1 - (bodyPos[2] - itsPoolDepth)/(itsSubRadius))),
                          0,-0.01,0);
  }


}

Image<PixRGB<byte> > SeaBee3Simulator::getFrame(int camera)
{
  const dReal *bodyPos = dBodyGetPosition(itsSubBody);
  const dReal *bodyR = dBodyGetRotation(itsSubBody);

  double cam_xyz[3], cam_hpr[3] = {0.0,0.0,0.0};

  double roll= (atan2(bodyR[9], bodyR[10]) + M_PI/2.0) * 180.0/M_PI;
  double pitch = asin(-bodyR[8]) * 180.0/M_PI;
  double yaw = atan2(bodyR[4], bodyR[0]) * 180.0/M_PI;

  switch (camera)
  {
    case 0: //world camera
      cam_xyz[0] = bodyPos[0];
      cam_xyz[1] =  bodyPos[1]-5;
      cam_xyz[2] =  10;

      cam_hpr[0]   = 90.0;
      cam_hpr[1]   = -45.0;
      cam_hpr[2]   = 0.0;

      break;
    case 1:
      cam_xyz[0] = bodyPos[0];
      cam_xyz[1] = bodyPos[1];
      cam_xyz[2] = bodyPos[2];

      cam_hpr[0]   = yaw + 90;
      cam_hpr[1]   = pitch;
      cam_hpr[2]   = roll;

      break;
    case 2:
      cam_xyz[0] = bodyPos[0];
      cam_xyz[1] = bodyPos[1];
      cam_xyz[2] = bodyPos[2];

      cam_hpr[0]   = yaw + 90;
      cam_hpr[1]   = pitch - 90;
      cam_hpr[2]   = roll;

      break;
  }
  pthread_mutex_lock(&itsDispLock);
  if (camera != -1)
    vp->dsSetViewpoint (cam_xyz,cam_hpr);
  vp->initFrame();
  drawSub();
  drawArena();
  vp->updateFrame();
  pthread_mutex_unlock(&itsDispLock);


  return vp->getFrame();
}

void SeaBee3Simulator::getSensors(float &xPos, float &yPos, float &depth,
    float &roll, float &pitch, float &yaw)
{

  xPos = itsSimXPos;
  yPos = itsSimYPos;
  depth = itsSimDepth;
  roll = itsSimRoll;
  pitch = itsSimPitch;
  yaw = itsSimYaw;

}

void SeaBee3Simulator::setThrusters(double forwardLeftThruster, double forwardRightThruster,
    double verticalLeftThruster, double verticalRightThruster,
    double forwardStrafeThruster, double  backStrafeThruster)
{

  itsForwardLeftThrusterVal    = forwardLeftThruster;
  itsForwardRightThrusterVal   = forwardRightThruster;
  itsFrontStrafeThrusterVal    = forwardStrafeThruster;
  itsBackStrafeThrusterVal     = backStrafeThruster;
  itsLeftVertThrusterVal       = verticalLeftThruster;
  itsRightVertThrusterVal      = verticalRightThruster;
}

