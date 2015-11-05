/**
   \file  Robots/LoBot/control/LoLGMDExtricateEMD.H
   \brief A behaviour for getting the robot unstuck using the raw LGMD
   spikes and an array of elementary motion detectors.

   This file defines a class that implements an array of Reichardt
   Elementary Motion Detectors that are wired into the LGMD's of lobot's
   locusts. The behaviour uses this EMD array to sense the direction in
   which maximal spiking activity is ocurring and to steer the robot away
   from approaching obstacles in that direction.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoLGMDExtricateEMD.H $
// $Id: LoLGMDExtricateEMD.H 13787 2010-08-13 15:30:47Z mviswana $
//

#ifndef LOBOT_LGMD_EXTRICATE_EMD_DOT_H
#define LOBOT_LGMD_EXTRICATE_EMD_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"

#include "Robots/LoBot/lgmd/LocustModel.H"
#include "Robots/LoBot/io/LoEMD.H"

#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LGMDExtricateEMD

   \brief A behaviour for moving the robot away from obstacles by
   running the raw LGMD inputs via an EMD array.

   This class implements a behaviour designed to move lobot away from
   obstacles obstructing its path. It does this by using an array of
   Reichardt Elementary Motion Detectors that are wired into the locusts
   being used to "guide" lobot. The idea is to emulate an insect's
   compound eyes.

   As an obstacle approaches the robot, we expect to see an LGMD "wave"
   travel through the locusts depending on the obstacle's trajectory. For
   example, when an obstacle approaches from the left, we will see the
   LGMD's of the leftmost locusts fire first and then those of the
   locusts towards the right. The EMD will track this LGMD wave and
   point in the direction of maximal spiking activity, which will allow
   the behaviour to steer the robot away.
*/
class LGMDExtricateEMD : public Behavior {
   // Prevent copy and assignment
   LGMDExtricateEMD(const LGMDExtricateEMD&) ;
   LGMDExtricateEMD& operator=(const LGMDExtricateEMD&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<LGMDExtricateEMD, base> ;
   typedef register_factory<LGMDExtricateEMD, base> my_factory ;
   static  my_factory register_me ;

   /// A compound eye is built from an array of elementary motion
   /// detectors. In Robolocust's case, the individual locust LGMDs are
   /// the EMDs.
   typedef EMD<const LocustModel*> LMD ; // LMD = LGMD motion detector
   typedef std::vector<LMD*> EMDs ;
   EMDs m_emds ;

   /// This behaviour works by computing a vector that points in the
   /// direction of maximal LGMD spiking activity and steering the robot
   /// away from that direction. This data member is used to keep track
   /// of the above-mentioned vector for visualization purposes.
   Vector m_emd_vector ;

   /// In each iteration, this behaviour issues both a drive and a turn
   /// command. This structure is a convenient way to hold both these
   /// commands together in one place.
   struct Command {
      int drive ;
      int turn  ;

      Command() ;
   } ;

   /// This data member holds the most recent commands issued by this
   /// behaviour. Useful for visualization.
   Command m_cmd ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   LGMDExtricateEMD() ;

   /// Stuff to take care of before regular action processing starts.
   void pre_run() ;

   /// This method implements the behaviour's extrication strategy, which
   /// involves running all the LGMD spikes through an EMD array.
   void action() ;

   /// Visualization routine to aid with development and debugging.
   void render_me() ;

   /// Clean-up.
   ~LGMDExtricateEMD() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
