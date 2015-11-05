/**
   \file  Robots/LoBot/control/LoTrack.C
   \brief This file defines the non-inline member functions of the
   lobot::Track class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoTrack.C $
// $Id: LoTrack.C 13619 2010-06-25 01:59:32Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoTrack.H"
#include "Robots/LoBot/control/LoMetrics.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/slam/LoSlamParams.H"
#include "Robots/LoBot/io/LoRobot.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <iomanip>
#include <algorithm>
#include <vector>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from track section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_TRACK, key, default_value) ;
}

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the track behaviour.
class Params : public singleton<Params> {
   /// How much of the pose history should we draw?
   int m_history_size ;

   /// The number of milliseconds between successive iterations of this
   /// behaviour.
   ///
   /// WARNING: The ability to change a behaviour's update frequency is a
   /// very powerful feature whose misuse or abuse can wreak havoc! Be
   /// sure to use reasonable values for this setting.
   int m_update_delay ;

   /// Private constructor because this is a singleton.
   Params() ;
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters
   //@{
   static int history_size() {return instance().m_history_size ;}
   static int update_delay() {return instance().m_update_delay ;}
   //@}
} ;

// Parameter initialization
Params::Params()
   : m_history_size(visualize(LOBE_TRACK)
                       ? clamp(conf("history_size", 25), 3, 100) : 0),
     m_update_delay(clamp(conf("update_delay", 1000), 500, 2500))
{}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

Track::Track()
   : base(Params::update_delay())
{
   start(LOBE_TRACK) ;
}

void Track::pre_run()
{
   if (! App::robot())
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;

   Map* map = App::map() ;
   if (! map)
      throw behavior_error(MAPPING_DISABLED) ;
   map->add_pose_hook(Map::PoseHook(add_pose,
                                    reinterpret_cast<unsigned long>(this))) ;
   if (visualize(LOBE_TRACK))
      map->add_hook(RenderHook(
         render_history, reinterpret_cast<unsigned long>(this))) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// Add latest pose to pose history for visualization. Also send latest
// pose to metrics log.
//
// NOTE: This function is executed by the thread that performs the pose
// update (e.g., the survey behaviour).
void Track::add_pose(const Pose& p, unsigned long client_data)
{
   using namespace std ;
   Metrics::Log() << setw(Metrics::opw()) << left << "tracking pose " << p ;

   Track* T = reinterpret_cast<Track*>(client_data) ;
   T->viz_lock() ;
      T->m_history.push_front(p) ;
      if (static_cast<int>(T->m_history.size()) > Params::history_size())
         T->m_history.pop_back() ;
   T->viz_unlock() ;
}

// All of the action takes place in add_pose(), viz., metrics logging,
// and render_history(). But we still use this method to periodically log
// the robot's current speed.
void Track::action()
{
   UpdateLock::begin_read() ;
      float speed = App::robot()->current_speed() ;
   UpdateLock::end_read() ;

   using namespace std ;
   Metrics::Log() << setw(Metrics::opw()) << left << "tracking speed "
                  << setprecision(3)  << fixed << speed << " m/s" ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

// Helper to generate vertices for the trail joining successive poses.
//
// NOTE: Since the GL coordinate system is rotated ccw by 90 degrees to
// make the robot's notion of "up" coincide with ours, we have to swap
// the x and y coordinates of all vertices.
static void render_pose(const Pose& p)
{
   glVertex2f(p.y(), p.x()) ;
}

// Callback for rendering the pose history on the Robolocust map
void Track::render_history(unsigned long client_data)
{
   // Copy history to avoid holding up pose updating threads
   Track* track = reinterpret_cast<Track*>(client_data) ;
   track->viz_lock() ;
      std::vector<Pose> history(track->m_history.begin(),
                                track->m_history.end()) ;
   track->viz_unlock() ;
   if (history.empty())
      return ;

   // Setup 2D "view volume" to match real/physical coordinate system
   // except that the whole thing is rotated 90 degrees ccw so that our
   // notion of "up" matches that of the robot's.
   float L, R, B, T ; // map extents
   SlamParams::map_extents(&L, &R, &B, &T) ;
   track->setup_view_volume(T, B, L, R) ;

   // Now we're ready to draw the pose history...
   glPushAttrib(GL_POINT_BIT | GL_LINE_BIT) ;
   glEnable(GL_LINE_STIPPLE) ;
   glLineStipple(1, 0xAAAA) ;
   glColor3f(0, 0, 0) ;
   glBegin(GL_LINE_STRIP) ;
      std::for_each(history.begin(), history.end(), render_pose) ;
   glEnd() ;
   glPopAttrib() ;

   // Reset GL transformations so next drawable won't get screwed
   track->restore_view_volume() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

Track::~Track(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
