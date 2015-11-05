 /*!@file testRoomba.C test the Roomba class  */

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
// Primary maintainer for this file: farhan baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/IRobot/testRoomba.C $
// $Id: testRoomba.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Component/ModelManager.H"
#include "Roomba.H"
#include "Util/log.H"
#include "Util/Types.H"
#include "Util/sformat.H"
#include "Image/DrawOps.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Devices/PingSonar.H"


// ######################################################################

int main(int argc, const char **argv)
{
  // Instantiate a ModelManager:
  ModelManager manager("Test Roomba");

      // nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
      //manager.addSubComponent(ofs);

  // Instantiate our various ModelComponents:
  nub::soft_ref<Roomba> roomba(new Roomba(manager,"Roomba",
                                                   "Roomba","/dev/ttyUSB1"));

   nub::soft_ref<PingSonar> pingSonar(new PingSonar(manager,"PingSonar",                                           "PingSonar","/dev/ttyUSB0",3));
   manager.addSubComponent(pingSonar);
  manager.addSubComponent(roomba);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv,"", 0, 0) == false) return(1);

  // let's get all our ModelComponent instances started:
  manager.start();


  LINFO("-----------testing the roomba class");

  LINFO("starting roomba....");
  roomba->sendStart();
  LINFO("setting to full mode....");
  roomba->setMode(2);


  //std::vector<int> dists = pingSonar->getDists();


  int dist=0,ang=0;
   roomba->getDistanceAngle(dist,ang);
  LINFO("dist = %i,ang=%i",dist,ang);

  //roomba->playSong(0);
  usleep(100*1000);

  roomba->setSpeed(50);
  roomba->setRadius(0);

  usleep(10000*1000);

  roomba->setSpeed(0);
  roomba->getDistanceAngle(dist,ang);
  LINFO("dist = %i,ang=%i",dist,ang);


}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


