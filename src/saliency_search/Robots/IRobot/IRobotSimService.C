/*!@file Robots/IRobot/IRobotSimService.C Ice interface to the IRobot Simulator */

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

#include "Component/ModelManager.H"
#include "Raster/GenericFrame.H"
#include "Image/Layout.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Image/MatrixOps.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"
#include "Robots/IRobot/IRobotSim.H"
#include "Ice/IceImageUtils.H"
#include "Ice/ImageIce.ice.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/IRobot.ice.H"

#include <stdio.h>
#include <stdlib.h>

#define KEY_UP 98
#define KEY_DOWN 104
#define KEY_LEFT 100
#define KEY_RIGHT 102

class IRobotI : public ModelComponent, public Robots::IRobot, public IceUtil::Thread
{
  public:
    IRobotI(ModelManager& mgr,
        nub::soft_ref<OutputFrameSeries> ofs,
        const std::string& descrName = "IRobotSimService",
        const std::string& tagName = "IRobotSimService") :
      ModelComponent(mgr, descrName, tagName),
      itsOfs(ofs),
      itsWorldView(true),
      itsCurrentSpeed(0),
      itsCurrentSteering(0)
    {
      itsIRobotSim = nub::soft_ref<IRobotSim>(new IRobotSim(mgr));
      addSubComponent(itsIRobotSim);
    }

    virtual void run()
    {
      itsIRobotSim->initViewport();
      while(true)
      {
        itsIRobotSim->simLoop();

        //Image<PixRGB<byte> > camImage;

        float rightWheel = (itsCurrentSpeed*10) + (itsCurrentSteering*10);
        float leftWheel = (itsCurrentSpeed*10) - (itsCurrentSteering*10);

        itsIRobotSim->setMotors(rightWheel, leftWheel);


       // if (itsWorldView)
       //  camImage = flipVertic(itsIRobotSim->getFrame(-1));
       // else
       //   camImage = flipVertic(itsIRobotSim->getFrame(1));
        itsCurrentFrame = flipVertic(itsIRobotSim->getFrame(-1));

       // itsOfs->writeRGB(camImage, "IRobotSim", FrameInfo("IRobotSim", SRC_POS));
      }

    }


    virtual float getSpeed(const Ice::Current&) { return itsCurrentSpeed; }
    virtual short setSpeed(const float speed, const Ice::Current&) { itsCurrentSpeed = speed;  return 0; }
    virtual float getSteering(const Ice::Current&) { return itsCurrentSteering; }
    virtual short setSteering(const float steeringPos, const Ice::Current&) { itsCurrentSteering = steeringPos; return 0;}
    virtual ImageIceMod::ImageIce getImageSensor(const short i, bool color, const Ice::Current&)
    {
      return Image2Ice(itsCurrentFrame);
    }
    virtual ImageIceMod::DimsIce getImageSensorDims(const short i, const Ice::Current&)
    {
      ImageIceMod::DimsIce dims;
      dims.w = 0;
      dims.h = 0;
      return dims;
    }
    virtual float getSensorValue(const short i, const Ice::Current&){
      switch (i)
      {
        case 0: return itsIRobotSim->getXPos();
        case 1: return itsIRobotSim->getYPos();
        case 2: return itsIRobotSim->getOri();
      }

      return 0;
    }

    virtual bool getSensors(float &xPos, float &yPos, float &ori, const Ice::Current&)
    {
      itsIRobotSim->getSensors(xPos, yPos, ori);
      return true;
    }

    virtual bool getDistanceAngle(float &dist, float &ang, const Ice::Current&)
    {
      float xPos, yPos, ori;
      itsIRobotSim->getSensors(xPos, yPos, ori);

      //TODO get dist and ang from pos
      //
      return false;

    }


    virtual void motorsOff(const short i, const Ice::Current&){
      itsCurrentSpeed = 0;
      itsCurrentSteering = 0;
      itsIRobotSim->setMotors(0, 0);
    }
    virtual void setMotor(const short i, const float val, const Ice::Current&){ }
    virtual short sendRawCmd(const std::string& data, const Ice::Current&) { return 0; }
    virtual void playSong(const short song, const Ice::Current&) { }
    virtual void shutdown(const Ice::Current&) { }
    virtual void sendStart(const Ice::Current&) { }
    virtual void setMode(const Robots::IRobotModes demo, const Ice::Current&) { }
    virtual void setDemo(const short demo, const Ice::Current&) { }
    virtual void setLED(const short led, const short color, const short intensity, const Ice::Current&) { }


  private:
    nub::soft_ref<OutputFrameSeries> itsOfs;
    nub::soft_ref<IRobotSim> itsIRobotSim;
    Image<PixRGB<byte> > itsCurrentFrame;
    bool itsWorldView;
    float itsCurrentSpeed;
    float itsCurrentSteering;

};


// ######################################################################
class IRobotSimService : public Ice::Service {
        protected:
                virtual bool start(int, char*[]);
        private:
                Ice::ObjectAdapterPtr itsAdapter;
    ModelManager *itsMgr;
};

//bool IRobotSimService::stop()
//{
//  if (itsMgr)
//    delete itsMgr;
//  return true;
//}


bool IRobotSimService::start(int argc, char* argv[])
{
  // Instantiate a ModelManager:
        itsMgr = new ModelManager("IRobot Simulator Service");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*itsMgr));
  itsMgr->addSubComponent(ofs);

  // Instantiate our various ModelComponents:
  nub::ref<IRobotI> iRobotI(new IRobotI(*itsMgr, ofs));
  itsMgr->addSubComponent(iRobotI);

  // Parse command-line:
  if (itsMgr->parseCommandLine(argc, argv, "", 0, 0) == false)
    return(1);

  char endpointBuff[64];
  sprintf(endpointBuff, "default -p %d", 10000);
        itsAdapter = communicator()->createObjectAdapterWithEndpoints("IRobotSimAdapter", endpointBuff);

        Ice::ObjectPtr object = iRobotI.get();
  Ice::ObjectPrx objectPrx = itsAdapter->add(object, communicator()->stringToIdentity("IRobotService"));
        itsAdapter->activate();

  itsMgr->start();

  //Start the retina evolve thread
  IceUtil::ThreadPtr iRobotIThread = iRobotI.get();
  iRobotIThread->start();
  LINFO("Started");

        return true;

}

// ######################################################################
int main(int argc, char** argv) {
  IRobotSimService svc;
  return svc.main(argc, argv);

}

