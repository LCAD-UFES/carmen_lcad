/**
   \file  Robots/LoBot/control/LoBehavior.C
   \brief This file defines the non-inline member functions of the
   lobot::Behavior class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoBehavior.C $
// $Id: LoBehavior.C 13567 2010-06-13 15:58:59Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/ui/LoLaserViz.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/thread/LoPause.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoMath.H"

// INVT utilities
#include "Util/log.H"

// Unix headers
#include <unistd.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Behavior::
Behavior(int update_delay, const std::string& n, const Drawable::Geometry& g)
   : Drawable(n, g),
     m_update_delay(clamp(update_delay, 1, 900000) * 1000)
{}

void Behavior::pre_run(){}

//------------------------ THE THREAD FUNCTION --------------------------

void Behavior::run()
{
   try
   {
      App::wait_for_init() ;

      // Main loop
      pre_run() ;
      while (! Shutdown::signaled())
      {
         if (Pause::is_clear())
            action() ;
         usleep(m_update_delay) ;
      }
      post_run() ;
   }
   catch (uhoh& e)
   {
      LERROR("behaviour %s encountered an error: %s", name.c_str(), e.what()) ;
      return ;
   }
}

//----------------------------- CLEAN-UP --------------------------------

void Behavior::post_run(){}
Behavior::~Behavior(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
