/*!@file test-Gumbot.C a test the Gumbot service */

//////////////////////////////////////////////////////////////////// //
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
// Primary maintainer for this file: Lior Elazary <lelazary@yahoo.com>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Gumbot/test-Gumbot.C $
// $Id: test-Gumbot.C 12281 2009-12-17 09:00:36Z itti $
//

#include <Ice/Ice.h>
#include "Ice/Gumbot.ice.H"
#include "Ice/ImageIce.ice.H"
#include "Ice/IceImageUtils.H"

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"

using namespace std;
using namespace Robots;

#define KEY_UP 98
#define KEY_DOWN 104
#define KEY_LEFT 100
#define KEY_RIGHT 102

int getKey(nub::ref<OutputFrameSeries> &ofs)
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("Output")
    : rutz::shared_ptr<XWinManaged>();
  return uiwin->getLastKeyPress();
}

//////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager manager("test-Gumbot");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);


  int status = 0;
  Ice::CommunicatorPtr ic;
  try {
    ic = Ice::initialize(argc, argv);
    Ice::ObjectPrx base = ic->stringToProxy(
        "GumbotService:default -p 10000 -h 192.168.1.15");
    GumbotPrx gumbot = GumbotPrx::checkedCast(base);
    if(!gumbot)
      throw "Invalid proxy";

    manager.exportOptions(MC_RECURSE);

    if (manager.parseCommandLine((const int)argc, (const char**)argv, "", 0, 0) == false)
      return 1;
    manager.start();

    Image<PixRGB<byte> > gumbotImg = Ice2Image<PixRGB<byte> >(gumbot->getImageSensor(0));
    ofs->writeRGB(gumbotImg, "Output", FrameInfo("Output", SRC_POS));

    gumbot->sendStart();
    gumbot->setMode(Robots::SafeMode);
    while(true)
    {

      //Show the image from the robot camera

      int key = getKey(ofs);


      if (key != -1)
      {
        switch(key)
        {
          case KEY_UP:
            gumbot->setSteering(0);
            gumbot->setSpeed(200);
            break;
          case KEY_DOWN:
            gumbot->setSteering(0);
            gumbot->setSpeed(-200);
            break;
          case KEY_LEFT:
            gumbot->setSteering(10);
            gumbot->setSpeed(100);
            break;
          case KEY_RIGHT:
            gumbot->setSteering(-10);
            gumbot->setSpeed(100);
            break;
          case 65: //space
            gumbot->setMode(Robots::SafeMode);
            gumbot->setSteering(0);
            gumbot->setSpeed(0);
            break;
          case 33: //p for playing the song
            LINFO("Play song");
            gumbot->playSong(0);
            break;
          case 40: //d for dock with base station
            LINFO("Docking");
            gumbot->setMode(Robots::CoverAndDockMode);
            break;
          default:
            LINFO("Unknown key %i\n", key);
            break;
        }
      }
    }

  }
  catch (const Ice::Exception& ex) {
    cerr << ex << endl;
    status = 1;
  }
  catch(const char* msg) {
    cerr << msg << endl;
    status = 1;
  }
  if (ic)
    ic->destroy();
  return status;
}
