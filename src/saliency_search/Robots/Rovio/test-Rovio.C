/*!@file test-Rovio.C a test the Rovio service */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Rovio/test-Rovio.C $
// $Id: test-Rovio.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Util/Timer.H"
#include "Robots/Rovio/Rovio.H"

using namespace std;

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
  if (uiwin.is_valid())
    return uiwin->getLastKeyPress();

  return -1;
}

//////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager manager("test-Rovio");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::ref<Rovio> rovio(new Rovio(manager));
  manager.addSubComponent(rovio);

  Timer localTimer;

  manager.exportOptions(MC_RECURSE);

  if (manager.parseCommandLine((const int)argc, (const char**)argv, "", 0, 0) == false)
    return 1;
  manager.start();


  localTimer.reset();



  while(true)
  {
    localTimer.reset();
    const FrameState is = ifs->updateNext();
    if (is == FRAME_COMPLETE) return 0;
    GenericFrame input = ifs->readFrame();
    Image<PixRGB<byte> > RovioImg = input.asRgb();

    ofs->writeRGB(RovioImg, "Output", FrameInfo("Output", SRC_POS));
    ofs->updateNext();

    int key = getKey(ofs);
    if (key != -1)
    {
      switch(key)
      {
        case KEY_UP:
          rovio->moveForward(0.5);
          break;
        case KEY_DOWN:
          rovio->moveBackward(0.5);
          break;
        case KEY_LEFT:
          rovio->rotateLeft(0.4);
          break;
        case KEY_RIGHT:
          rovio->rotateRight(0.4);
          break;
        case 65: //space
          rovio->stop();
          break;
        case 39: //s for status
            rovio->getStatus();
          break;
        case 24: //q camera up
          rovio->setCameraPos(2);
          break;
        case 38: //a camera mid
          rovio->setCameraPos(1);
          break;
        case 52: //z camera down
          rovio->setCameraPos(0);
          break;
        case 40: //d for play sound
          rovio->playSound();
          break;

        default:
          LINFO("Unknown key %i\n", key);
          break;
      }
      localTimer.reset();
    }
  }

  manager.stop();
  return 0;
}
