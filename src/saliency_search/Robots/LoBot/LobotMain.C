/**
   \file  Robots/LoBot/LobotMain.C
   \brief lobot/Robolocust controller.

   This file defines the main function for the Robolocust controller. The
   goal of the Robolocust project is to design and implement a robot that
   avoids obstacles based on a model of the Lobula Giant Movement
   Detector (LGMD) found in locust brains.

   The LGMD is a visual interneuron that responds with increasing
   frequency to stimuli on a direct collision course with the locust. For
   the Robolocust project, we want to use a computational model of this
   neuron to develop a collision sensor that can then be applied to the
   problem of robotic obstacle avoidance. Additionally, we also want to
   hook up actual locusts to the robot, tap into their LGMDs and use the
   spikes directly to be able to avoid obstacles.

   To be able to perform the above-mentioned tasks, we need to have
   algorithms in place to integrate the LGMD spikes from multiple (real
   or virtual) locusts so as to produce a coherent steering decision for
   the robot. The lobot controller is designed to be a software framework
   for this purpose.

   In terms of hardware, this framework has the ability to interface with
   FireWire cameras, a laser range finder and different robot platforms
   (e.g., the iRobot Create). For high-level control aspects, it provides
   a behaviour-based substrate built atop an implementation of the
   Distributed Architecture for Mobile Navigation (DAMN). Moreover, the
   framework provides for a configuration file that can be used to change
   almost any aspect of the program. This allows us to develop, test and
   compare many different LGMD models and integration algorithms.
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LobotMain.C $
// $Id: LobotMain.C 13780 2010-08-11 22:07:47Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/LoApp.H"

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"
#include "Robots/LoBot/thread/LoThread.H"

#include "Robots/LoBot/misc/LoExcept.H"

// INVT utilities
#include "Util/log.H"

// Standard C++ headers
#include <stdexcept>

//------------------------------- MAIN ----------------------------------

int main(int argc, const char* argv[])
{
   MYLOGVERB = LOG_ERR ; // minimize INVT's incessant chatter
   int ret = 0 ;
   try
   {
      lobot::App& app = lobot::App::create(argc, argv) ;
      app.parse_command_line() ;
      app.run() ;
   }
   catch (lobot::uhoh& e)
   {
      LERROR("%s", e.what()) ;
      ret = e.code() ;
   }
   catch (std::exception& e)
   {
      LERROR("%s", e.what()) ;
      ret = 255 ;
   }
   if (ret != 0) {
      lobot::Shutdown::signal() ;
      lobot::UpdateLock::instance().unlock() ;
   }
   lobot::Thread::wait_all() ;
   return ret ;
}

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
