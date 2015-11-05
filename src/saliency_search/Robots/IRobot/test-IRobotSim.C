/*!@file Robots/IRobot/test-IRobotSim.C Test the IRobot simulator */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/IRobot/test-IRobotSim.C $
// $Id: test-IRobotSim.C 10794 2009-02-08 06:21:09Z itti $
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
#include <stdio.h>
#include <stdlib.h>

#define KEY_UP 98
#define KEY_DOWN 104
#define KEY_LEFT 100
#define KEY_RIGHT 102

bool worldView = true;

void handle_keys(nub::soft_ref<OutputFrameSeries> ofs, nub::soft_ref<IRobotSim> iRobotSim)
{
    //handle keyboard input
    const nub::soft_ref<ImageDisplayStream> ids =
      ofs->findFrameDestType<ImageDisplayStream>();

    const rutz::shared_ptr<XWinManaged> uiwin =
      ids.is_valid()
      ? ids->getWindow("IRobotSim")
      : rutz::shared_ptr<XWinManaged>();


    int key = uiwin->getLastKeyPress();
    if (key != -1)
    {
      switch(key)
      {
        case KEY_UP:
          iRobotSim->setMotors(10, 10);
          break;
        case KEY_DOWN:
          iRobotSim->setMotors(-2.5, -2.5);
          break;
        case KEY_LEFT:
          iRobotSim->setMotors(2.5, -2.5);
          break;
        case KEY_RIGHT:
          iRobotSim->setMotors(-2.5, 2.5);
          break;
        case 65: //space to stop
          iRobotSim->setMotors(0, 0);
          break;
        case 25: //w for world view
          worldView = !worldView;
          break;
        default:
          LINFO("Unkown key %i\n", key);
          break;
      }

    }
}


int main(int argc, char *argv[])
{
  // Instantiate a ModelManager:
  ModelManager manager("IRobot Simulator");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  // Instantiate our various ModelComponents:
  nub::soft_ref<IRobotSim> iRobotSim(new IRobotSim(manager));
  manager.addSubComponent(iRobotSim);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

  LINFO("s");
  // let's get all our ModelComponent instances started:
        manager.start();
  LINFO("s1");

  iRobotSim->initViewport();


        while(1){
    Layout<PixRGB<byte> > outDisp;

                iRobotSim->simLoop();

    Image<PixRGB<byte> > camImage;

    if (worldView)
      camImage = flipVertic(iRobotSim->getFrame(-1));
    else
      camImage = flipVertic(iRobotSim->getFrame(1));

    //Sensors
    float xPos, yPos, ori;
    iRobotSim->getSensors(xPos, yPos, ori);
//    LINFO("pos(%0.2f, %0.2f) ori=%0.2f", xPos, yPos, ori);

                ofs->writeRGB(camImage, "IRobotSim", FrameInfo("IRobotSim", SRC_POS));

    handle_keys(ofs, iRobotSim);

        }
        return 0;

}
