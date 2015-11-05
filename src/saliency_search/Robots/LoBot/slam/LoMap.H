/**
   \file  Robots/LoBot/slam/LoMap.H
   \brief An application-wide data structure holding the occupancy grid
   used mainly for visualization and state communication between
   different behaviours.

   This file defines a class that implements an application-wide obstacle
   map for the Robolocust system. This map is an occupancy grid that
   holds probability values rather than simple 0/1 flags indicating the
   absence or presence of obstacles. The probabilities are represented as
   bytes ranging from 0 to 255. High values indicate free space while low
   values indicate obstacles.

   In addition to the obstacle map, this data structure also holds the
   robot's current pose w.r.t. the map.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoMap.H $
// $Id: LoMap.H 13732 2010-07-29 14:16:35Z mviswana $
//

#ifndef LOBOT_MAP_DOT_H
#define LOBOT_MAP_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoPose.H"
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/thread/LoRWLock.H"
#include "Robots/LoBot/thread/LoMutex.H"

// Standard C++ headers
#include <vector>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class OccGrid ;

/**
   \class lobot::Map
   \brief An application-wide obstacle map.

   This class implements an occupancy grid that is used to keep track of
   where obstacles are. The occupancy grid's cells hold probability
   values rather than simple 0/1 flags to indicate the absence or
   presence of an obstacle. The probabilities are bytes in the [0,255]
   range rather than floats in the [0,1] range. Furthermore, high values
   indicate free space while low ones denote the presence of obstacles.

   In addition to holding the occupancy map, this class also keeps track
   of the robot's current pose, which is specified using an (x, y, theta)
   triple.
*/
class Map : public Drawable {
   // Prevent copy and assignment
   Map(const Map&) ;
   Map& operator=(const Map&) ;

   /// The occupancy grid is represented as a single dimensional array
   /// stored in row-major order. Each cell in the array holds a number
   /// in the range [0,255]. Higher values imply greater certainty that a
   /// cell is free; lower values indicate greater certainty that a cell
   /// has an obstacle in it.
   std::vector<unsigned char> m_grid ;

   /// In addition to the occupancy grid, this data structure also keeps
   /// track of the robot's current pose w.r.t. the map.
   Pose m_pose ;

   /// Many behaviours/modules may be interested in hearing about pose
   /// updates. Therefore, the map maintains a list of callback functions
   /// to inform interested parties about pose updates.
   ///
   /// NOTE: Clients should take care that the pose update hooks not
   /// perform lengthy operations so as to not hold up the thread that
   /// originated the pose update.
public:
   //@{
   typedef void (*PoseCB)(const Pose&, unsigned long client_data) ;
   typedef std::pair<PoseCB, unsigned long> PoseHook ;
private:
   std::vector<PoseHook> m_pose_hooks ;
   //@}

   /// Multiple behaviours can access the map at the same time. For
   /// example, a SLAM behaviour can be busy building the map and
   /// updating the pose while a goal behaviour uses the map and pose to
   /// guide the robot towards some specific location. Therefore, we
   /// should protect simultaneous accesses to the map's grid, pose and
   /// related data structures.
   ///
   /// NOTE: More threads read the pose than update it. Therefore, it
   /// makes sense to use a read/write lock for it rather than a mutex.
   //@{
   Mutex  m_grid_mutex ;
   RWLock m_pose_lock  ;
   Mutex  m_pose_hooks_mutex ;
   //@}

public:
   /// Initialization: creates an empty map.
   Map() ;

   /// This method allows clients to add pose update hooks so they can be
   /// informed when the robot's pose gets updated.
   void add_pose_hook(const PoseHook&) ;

   /// This method updates this map object using the data from the
   /// supplied log-odds occupancy grid.
   void update(const OccGrid&) ;

   /// This method updates the pose.
   void update(const Pose&) ;

   /// This method returns the robot's current pose w.r.t. to the map.
   Pose current_pose() const ;

   /// This method returns a square subportion of the occupancy map
   /// centered at the robot's current position. The size of the submap
   /// will 2*w+1, where w is supplied by the client.
   std::vector<unsigned char> submap(int w) const ;

private:
   /// OpenGL initialization.
   void gl_init() ;

   /// Map visualization.
   void render_me() ;

   /// OpenGL clean-up.
   void gl_cleanup() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
