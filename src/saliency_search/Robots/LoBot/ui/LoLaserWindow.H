/**
   \file Robots/LoBot/ui/LoLaserWindow.H

   \brief The main window for the laser range finder's test program.

   This file defines a class that encapsulates the GLUT-based window for
   the Hokuyo laser range finder's test program. This window uses OpenGL
   for visualizing the laser range finder's measurement data.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserWindow.H $
// $Id: LoLaserWindow.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_LRF_TESTER_WINDOW_DOT_H
#define LOBOT_LRF_TESTER_WINDOW_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserWindowMarkings.H"
#include "Robots/LoBot/ui/LoGLCanvas.H"

#include "Robots/LoBot/io/LoLaserRangeFinder.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <map>
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LaserWindow
   \brief The main window for the laser range finder's test program.

   This class uses GLUT and OpenGL to encapsulate a visualization window
   for the Hokuyo laser range finder's distance measurements. A world
   coordinate system is setup to match the maximum range of the laser
   range finder and the device itself is depicted as a small triangle
   centered at the origin of the world coordinate system. A grid is
   overlaid on the world coordinate system with cells at regular
   intervals and the distance measurements are shown as rays emanating
   from the central triangle in each supported direction.

   The user may tweak various parameters such as grid spacing,
   measurement ray directions, etc. Zoom and pan are also supported.
*/
struct LaserWindow : private singleton<LaserWindow> {
   // Boilerplate code to make generic singleton pattern work
   friend class singleton<LaserWindow> ;

   /// Because of the weird way in which GLUT is structured, clients only
   /// need this one method to use this class. Everything else is handled
   /// internally (meaning everything else in this class is private).
   static void create(const std::string&) ;

private:
   /// The window title. This needs to be static because the singleton
   /// implementation only supports a default constructor. If we want the
   /// create() method to be able to pass the window title it gets from
   /// its client to this class's constructor, we need some means other
   /// than an argument to a constructor.
   static std::string m_title ;

   /// Since the goal of this window is to render a "map" of the distance
   /// measurements made by a Hokuyo laser range finder, it shouldn't be
   /// surprising that we need an instance of the lobot::LaserRangeFinder
   /// class.
   ///
   /// DEVNOTE: A cleaner design would actually have this be part of the
   /// main program or some other module while this one simply takes care
   /// of the visualization details. However, no thanks to GLUT's
   /// architecture, doing the right thing becomes somewhat convoluted
   /// and unintuitive. Since this class is only used in the lolaser test
   /// program, which shouldn't experience much change once it works, we
   /// prefer to keep things as simple as possible and allow this
   /// UI/rendering object manage the data source object.
   LaserRangeFinder* m_lrf ;

   /// This class uses GLUT to help visualize the laser range finder's
   /// measurement data. This member variable holds the window ID of the
   /// GLUT window.
   int m_window ;

   /// We use a helper class to manage the OpenGL window and viewport
   /// transformations.
   GLCanvas* m_canvas ;

   /// We use another helper class to draw markings at regular intervals
   /// to make it easier for users to read/gauge the distances being spit
   /// out by the laser range finder.
   LaserWindowMarkings* m_markings ;

   /// Sometimes, we may want to pause the continuous updates of the
   /// laser range finder. This flag acts as a toggle for the pause
   /// state.
   bool m_paused ;

   /// Private constructor because this class is a singleton.
   LaserWindow() ;

   /// This method retrieves the latest readings from the laser range
   /// finder.
   void update() ;

   /// This method renders the latest measurements obtained from the
   /// laser range finder.
   void render() ;

   /// These methods respond to different key presses. We use a dispatch
   /// table to "route" key presses to the appropriate handler.
   //{@
   void handle_key(unsigned char key) ;

   typedef void (LaserWindow::*KeyHandler)() ;
   typedef std::map<unsigned char, KeyHandler> KeyMap ;
   KeyMap m_keymap ; // dispatch table

   void reset_zoom_pan() ;
   void pause() ;
   void quit() ;
   //@}

   /// These functions and variables take care different mouse events.
   //@{
   int m_drag_button ;
   int m_drag_modifiers ;
   int m_drag_prev[2] ;

   void left_click  (int state, int modifiers, int x, int y) ;
   void middle_click(int state, int modifiers, int x, int y) ;
   void right_click (int state, int modifiers, int x, int y) ;

   void left_drag  (int x, int y) ;
   void middle_drag(int x, int y) ;
   void right_drag (int x, int y) ;
   //@}

   /// GLUT callbacks
   //@{
   static void reshape_callback(int width, int height) ;
   static void render_callback() ;
   static void keyboard_callback(unsigned char key, int mouse_x, int mouse_y) ;
   static void click_callback(int button, int state, int x, int y) ;
   static void drag_callback(int x, int y) ;
   static void timer_callback(int timer_id) ;
   //@}

   /// A helper for setting up the update timer.
   void setup_timer() ;

   /// This method responds to UI window resize events.
   void reshape(int width, int height) ;

   /// Clean-up.
   ~LaserWindow() ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the laser range finder
   /// visualization.
   class Params : public singleton<Params> {
      /// Private constructor because this is a singleton.
      Params() ;
      friend class singleton<Params> ;

      /// The laser range finder can be connected to the host machine via
      /// USB or a plain-old RS-232C serial port. This setting specifies
      /// which device file to use when connecting to the LRF.
      std::string m_device ;

      /// The data transfer rate between host machine and the LRF.
      int m_baud_rate ;

      /// The type of markings that should be used to help users read the
      /// distance measurements being spit out by the laser range finder.
      /// At present, two marking types are supported, viz., grid and
      /// rings. In grid mode, we draw horizontal and vertical lines at
      /// regular intervals. In rings mode, we draw concentric circular
      /// contours to show the distance from the laser range finder.
      std::string m_markings_type ;

      /// The laser range finder's test program continuously retrieves
      /// the latest set of distance readings from the device and paints
      /// them on the screen. This setting specifies the frequency with
      /// which these updates should be performed. It is expected to be a
      /// time expressed in milliseconds. Thus, for some value N, the
      /// update will be performed once every N milliseconds.
      int m_update_frequency ;

      /// The LRF returns distance measurements corresponding to a range
      /// of angles with zero degrees being in front of the device,
      /// negative angles on the right and positive angles on the left.
      /// Drawing the measurements for all the angles in the device's
      /// range can result in a very cluttered and illegible picture.
      /// Thus, we allow users to specify the range of angles to use and
      /// the step size by which to increment the angle as we iterate
      /// through this range.
      //@{
      range<int> m_angles_range ;
      int m_angles_step ;
      //@}

      /// Color for the rays depicting laser range finder distance
      /// measurements.
      GLColor m_measurements_color ;

      /// The LRF itself is depicted as a combination of a rectangle and
      /// a triangle. The triangle serves to let users know which
      /// direction the device is pointed toward. These two settings
      /// specify the size and direction of the LRF.
      ///
      /// The direction is simply an angle in the range 0 to 360 degrees.
      ///
      /// The size is specified in terms of the half-size of the shared
      /// edge between the rectangle and triangle. If this size is R,
      /// then the entire figure will be inscribed within an imaginary
      /// square of size 2R.
      ///
      /// NOTE: As the visualization window's logical/world coordinate
      /// system is setup to match the Hokuyo's specs, the units of the
      /// above-mentioned R will be millimeters. However, these are
      /// "logical" not true physical mm.
      float m_lrf_size ;
      float m_lrf_direction ;

      /// Color for the LRF device (depicted as an upward pointing
      /// equilateral triangle).
      GLColor m_lrf_color ;

      /// We can speed up or slow down the zoom by adjusting this factor.
      /// Higher values will result in amplifying mouse motion so that
      /// even a small movement results in a large zoom in or out; lower
      /// values will damp the mouse motion so that more dragging is
      /// required to achieve the desired zoom level.
      float m_zoom_drag_factor ;

   public:
      // Accessing the various parameters
      static const std::string& device() ;
      static int baud_rate() ;
      static const std::string& markings_type() ;
      static int update_frequency() ;
      static const range<int>& angles_range() ;
      static int angles_step() ;
      static const GLColor& measurements_color() ;
      static float lrf_size() ;
      static float lrf_direction() ;
      static const GLColor& lrf_color() ;
      static float zoom_drag_factor() ;

      // Clean-up
      ~Params() ;
   } ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
