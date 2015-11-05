/*!@file Robots/RobotBrain/RobotBrainStimulator.C Stimulator the Robot with recorded data */

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
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define KEY_UP 98
#define KEY_DOWN 104
#define KEY_LEFT 100
#define KEY_RIGHT 102

const ModelOptionCateg MOC_StimulatorData = {
    MOC_SORTPRI_3, "Stimulator Data related options" };

const ModelOptionDef OPT_StimulatorFilename =
{ MODOPT_ARG(std::string), "StimulatorFile", &MOC_StimulatorData, OPTEXP_CORE,
    "Path to the stimulator data input file",
    "stimulator-file", '\0', "", "" };


class RobotBrainStimulator : public ModelComponent, public Robots::IRobot, public IceUtil::Thread
{
  public:
    RobotBrainStimulator(ModelManager& mgr,
        nub::soft_ref<OutputFrameSeries> ofs,
        nub::soft_ref<InputFrameSeries> ifs,
        const std::string& descrName = "RobotBrainStimulator",
        const std::string& tagName = "RobotBrainStimulator") :
      ModelComponent(mgr, descrName, tagName),
      itsOfs(ofs),
      itsIfs(ifs),
      itsFilename(&OPT_StimulatorFilename, this, 0),
      itsCurrentSpeed(0),
      itsCurrentSteering(0)
    {

    }

    virtual void start1()
    {
      if((itsStimulatorFd = fopen(itsFilename.getVal().c_str(), "r")) == NULL)
        LFATAL("Can not open stimulator file %s", itsFilename.getVal().c_str());
    }

    virtual void run()
    {
      while(true)
      {

      }

    }


    virtual float getSpeed(const Ice::Current&) { return itsCurrentSpeed; }
    virtual short setSpeed(const float speed, const Ice::Current&) { itsCurrentSpeed = speed;  return 0; }
    virtual float getSteering(const Ice::Current&) { return itsCurrentSteering; }
    virtual short setSteering(const float steeringPos, const Ice::Current&) { itsCurrentSteering = steeringPos; return 0;}

    virtual ImageIceMod::ImageIce getImageSensor(const short i, const bool color, const Ice::Current&)
    {

      ////grab the images
      GenericFrame input = itsIfs->readFrame();
      Image<PixRGB<byte> > img = input.asRgb();
      if (img.initialized())
        itsCurrentFrame = img;

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
      return 0;
    }

    virtual bool getSensors(float &xPos, float &yPos, float &ori, const Ice::Current&)
    {
      return false; //not implimanted
    }

    virtual bool getDistanceAngle(float &dist, float &ang, const Ice::Current&)
    {
      //Read the sensors from a file and update the image

      char* linePtr=NULL;
      size_t len=0;

      ssize_t ret = getline(&linePtr, &len, itsStimulatorFd);

      if (ret == -1)
        LFATAL("End of input reached");

      std::string line(linePtr);

      vector<string> fields;
      fields.clear();
      if(line.length() > 1) {
        string::size_type start = 0;
        string::size_type end = line.find_first_of(" ");
        while(end != string::npos) {
          fields.push_back(line.substr(start, end-start));
          start = line.find_first_not_of(" ", end+1);
          end = line.find_first_of(" ", start);
        }
        fields.push_back(line.substr(start, line.length()-start));
      }

      if (fields.size() != 6)
        LFATAL("Bad data '%s'", line.c_str());

      dist = atof(fields[2].c_str());
      ang = atof(fields[3].c_str());


      int frame = atoi(fields[5].c_str());

      if (itsIfs->frame() != frame)
        LFATAL("Frame out of sync");

      itsIfs->updateNext(); //Update the frame here so we are synced with the image

      //sleep(2);
      usleep(200000);
     // getchar();

      return true;

    }


    virtual void motorsOff(const short i, const Ice::Current&){
      itsCurrentSpeed = 0;
      itsCurrentSteering = 0;
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
    nub::soft_ref<InputFrameSeries> itsIfs;
    OModelParam<std::string>        itsFilename;
    Image<PixRGB<byte> > itsCurrentFrame;
    float itsCurrentSpeed;
    float itsCurrentSteering;
    FILE*  itsStimulatorFd;

};


// ######################################################################
class RobotBrainStimulatorService : public Ice::Service {
        protected:
                virtual bool start(int, char*[]);
        private:
                Ice::ObjectAdapterPtr itsAdapter;
    ModelManager *itsMgr;
};

bool RobotBrainStimulatorService::start(int argc, char* argv[])
{
  // Instantiate a ModelManager:
        itsMgr = new ModelManager("Robot Brain Stimulator Service");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*itsMgr));
  itsMgr->addSubComponent(ofs);

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(*itsMgr));
  itsMgr->addSubComponent(ifs);

  // Instantiate our various ModelComponents:
  nub::ref<RobotBrainStimulator> robotBrainStimulator(new RobotBrainStimulator(*itsMgr, ofs, ifs));
  itsMgr->addSubComponent(robotBrainStimulator);

  // Parse command-line:
  if (itsMgr->parseCommandLine(argc, argv, "", 0, 0) == false)
    return(1);

  char endpointBuff[64];
  sprintf(endpointBuff, "default -p %d", 10000);
        itsAdapter = communicator()->createObjectAdapterWithEndpoints("RobotBrainStimulatorAdaptor", endpointBuff);

        Ice::ObjectPtr object = robotBrainStimulator.get();
  Ice::ObjectPrx objectPrx = itsAdapter->add(object, communicator()->stringToIdentity("IRobotService"));
        itsAdapter->activate();

  itsMgr->start();

  //IceUtil::ThreadPtr iRobotIThread = iRobotI.get();
  //iRobotIThread->start();
  LINFO("Started");

        return true;

}

// ######################################################################
int main(int argc, char** argv) {
  RobotBrainStimulatorService svc;
  return svc.main(argc, argv);

}

