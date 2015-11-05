/**
   \file  Robots/LoBot/control/LoTrack.H
   \brief A behaviour for tracking the robot.

   This file implements a class for maintaining a list of lobot's
   positions within the map and visualizing this pose history as a dotted
   trail. Each pose update is also logged via lobot::Metrics. This
   behaviour does not control the robot in any way; it is only meant for
   data collection and visualization.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoTrack.H $
// $Id: LoTrack.H 13608 2010-06-23 04:27:01Z mviswana $
//

#ifndef LOBOT_TRACK_BEHAVIOUR_DOT_H
#define LOBOT_TRACK_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/slam/LoPose.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <deque>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Track
   \brief A behaviour for maintaining pose history and dumping it to a
   file every now and then.

   This class implements a behaviour for tracking the robot's pose
   history and visualizing the recent poses as a dotted trail behind the
   robot's icon on the map drawing area. Each pose update is also logged
   to the metrics log. Additionally, the robot's current speed is also
   logged by this behaviour.

   This behaviour does not control the robot in any way; it is only meant
   for data collection and visualization.
*/
class Track : public Behavior {
   // Prevent copy and assignment
   Track(const Track&) ;
   Track& operator=(const Track&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<Track, base> ;
   typedef register_factory<Track, base> my_factory ;
   static  my_factory register_me ;

   /// The track behaviour maintains a list of poses and saves them to a
   /// file periodically.
   std::deque<Pose> m_history ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   Track() ;

   /// Stuff to do before regular action processing begins.
   void pre_run() ;

   /// Adding pose updates to pose history and logging to metrics log.
   static void add_pose(const Pose&, unsigned long client_data) ;

   /// Most of this behaviour's action takes place in add_pose() and
   /// render_history(). Therefore, this method is not really necessary.
   /// But we define it because it is required (pure virtual). And since
   /// there's no point in having an empty function, we put it to good
   /// use by logging the robot's current speed.
   void action() ;

   /// Visualizing the pose history on lobot's map.
   static void render_history(unsigned long client_data) ;

   /// Clean-up.
   ~Track() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
