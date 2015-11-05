/**
   \file  Robots/LoBot/ui/LoLaserViz.H
   \brief LRF visualizer.

   This file defines a class that implements a drawable for visualizing
   the laser range finder's measurement data.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserViz.H $
// $Id: LoLaserViz.H 13674 2010-07-18 22:13:22Z mviswana $
//

#ifndef LOBOT_LRF_VISUALIZER_DOT_H
#define LOBOT_LRF_VISUALIZER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/ui/LoLaserWindowMarkings.H"
#include "Robots/LoBot/ui/LoGLCanvas.H"

#include "Robots/LoBot/io/LoLaserRangeFinder.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <memory>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LaserViz
   \brief A drawable for visualizing laser range finder data.

   This class uses OpenGL to visualize the Hokuyo laser range finder's
   distance measurements. A world coordinate system is setup to match the
   maximum range of the laser range finder and the device itself is
   depicted as a small triangle centered at the origin of the world
   coordinate system. A grid is overlaid on the world coordinate system
   with cells at regular intervals and the distance measurements are
   shown as rays emanating from the central triangle in each supported
   direction. Instead of a grid, the visualizer may also be configured to
   display concentric rings to help users gauge the distances being
   measured by the LRF.

   The user may tweak various parameters such as grid spacing,
   measurement ray directions, etc. Zoom and pan are also supported.
*/
class LaserViz : public Drawable {
   /// Since this drawable is used to visualize LRF data, it makes sense
   /// for it to keep handy a reference to an LRF object.
   const LaserRangeFinder* m_lrf ;

   /// We use a helper class to manage the OpenGL window and viewport
   /// transformations.
   std::auto_ptr<GLCanvas> m_canvas ;

   /// We use another helper class to draw markings at regular intervals
   /// to make it easier for users to read/gauge the distances being spit
   /// out by the laser range finder.
   std::auto_ptr<LaserWindowMarkings> m_markings ;

public:
   /// Initialization.
   LaserViz(const LaserRangeFinder*) ;

private:
   /// OpenGL related initialization: we need to configure the
   /// initial projection matrix based on the LRF max distance.
   void gl_init() ;

   /// This method renders the latest measurements obtained from the
   /// laser range finder.
   void render_me() ;

   /// The LRF visualizer supports zoom/pan operations.
   //@{
   void zoom_by(float zoom_factor) ;
   void pan(int x, int y, int prev_x, int prev_y) ;
   void reset_zoom_pan() ;
   //@}

   /// OpenGL related clean-up: the laser window ring markings object
   /// uses a display list that has to be released before the GL
   /// rendering context is destroyed, which may happen before (not
   /// good) or after (okay) the laser visualizer's destructor is called
   /// by the main thread because the rendering context is managed by
   /// the lobot::MainWindow object in a different thread and the OS
   /// scheduler can decide to take down threads however it pleases.
   ///
   /// Therefore, we need to release the ring markings object when the
   /// main window lets us know that the GL rendering context is about
   /// to be nuked, i.e., when it invokes each drawable's gl_cleanup()
   /// method. If we wait to do this in the destructor, we could get a
   /// segfault as the Robolocust controller shuts down.
   virtual void gl_cleanup() ;

public:
   /// Clean-up.
   ~LaserViz() ;

private:
   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the LRF and behaviours'
   /// visualization.
   class Params : public singleton<Params> {
      /// The location and size of the LRF visualizer (within the
      /// Robolocust main window).
      Drawable::Geometry m_geometry ;

      /// The laser range finder's measurements can be depicted either as
      /// rays emanating from the LRF or as hull joining the tips of all
      /// the rays (but without the actual rays themselves). This setting
      /// allows users to change the LRF measurements style.
      std::string m_measurements_style ;

      /// The type of markings that should be used to help users read the
      /// distance measurements being spit out by the laser range finder.
      /// At present, two marking types are supported, viz., grid and
      /// rings. In grid mode, we draw horizontal and vertical lines at
      /// regular intervals. In rings mode, we draw concentric circular
      /// contours to show the distance from the laser range finder.
      std::string m_markings_type ;

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
      //@{
      float m_lrf_size ;
      float m_lrf_direction ;
      //@}

      /// Color for the LRF device (depicted as an upward pointing
      /// equilateral triangle).
      GLColor m_lrf_color ;

      /// Private constructor because this is a singleton.
      Params() ;
      friend class singleton<Params> ;

   public:
      /// Accessing the various parameters
      //@{
      static const Drawable::Geometry& geometry() {
         return instance().m_geometry ;
      }
      static const std::string& measurements_style() {
         return instance().m_measurements_style ;
      }
      static bool draw_rays() {return measurements_style() == "rays" ;}
      static bool draw_hull() {return measurements_style() == "hull" ;}
      static const std::string& markings_type() {
         return instance().m_markings_type ;
      }
      static const range<int>& angles_range() {
         return instance().m_angles_range ;
      }
      static int angles_step() {return instance().m_angles_step ;}
      static const GLColor& measurements_color() {
         return instance().m_measurements_color ;
      }
      static float lrf_size()           {return instance().m_lrf_size ;}
      static float lrf_direction()      {return instance().m_lrf_direction ;}
      static const GLColor& lrf_color() {return instance().m_lrf_color ;}
      //@}

      /// Clean-up
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
