/**
   \file Robots/LoBot/ui/LoLaserWindowMarkings.H

   \brief A helper class for drawing markers to indicate distances on the
   LRF tester's main window.

   This file defines an abstract base class for a helper object that
   draws markers of some sort within the LRF test program's window. These
   markers indicate distances at regular intervals.

   For example, we can have markings every 100, 500, 1000 and 5000 mm.
   Each of these markings can be drawn in a different color. Thus, the
   100 mm markings can be one color, the 500 mm markings another color,
   the 1000 mm markings yet another color, and, finally the 5000 mm
   markings in a fourth color. Furthermore, we can also specify the zoom
   ranges at which the 100, 500, 1000 and 5000 mm markings should be
   visible.

   This helps make the laser range finder's distance measurements more
   readable and easier to interpret.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserWindowMarkings.H $
// $Id: LoLaserWindowMarkings.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_LASER_WINDOW_MARKINGS_DOT_H
#define LOBOT_LASER_WINDOW_MARKINGS_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoGLCanvas.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/triple.hh"
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LaserWindowMarkings
   \brief A helper class for drawing markers indicating distances.

   The laser range finder's test program depicts the distance
   measurements as rays emanating from the origin of a world coordinate
   system that is setup to match the units and range of the LRF device.
   To help make sense of the lengths of these rays, we draw some markers
   at regular intervals. Each marker indicates a particular distance from
   the laser range finder.

   We can have markers of different types (e.g., regularly spaced grid or
   concentric rings). This class provides a common interface for all the
   different marker types.
*/
class LaserWindowMarkings {
   /// The markings are drawn using a collection of marking
   /// specifications.
   //@{
public:
   typedef range<float> ZoomRange ;
   typedef triple<float, GLColor, ZoomRange> Marking ;
   typedef std::vector<Marking> Markings ;
protected:
   Markings m_markings ;
   //@}

   /// To draw the markings properly, we need to know the current zoom
   /// level, which is maintained by the OpenGL canvas.
   const GLCanvas* m_canvas ;

   /// The markings usually span the range of the laser range finder's
   /// maximum distance measurement. For example, if the device's max
   /// reading is 30000 mm, then the markings will cover the area between
   /// -30000 and +30000 units both horizontally and vertically.
   ///
   /// Clients must specify this maximum value.
   float m_max ;

   /// A protected constructor because only derived classes should be
   /// able to call it.
   LaserWindowMarkings() ;

public:
   /// Specify the OpenGL canvas to use.
   void use_canvas(const GLCanvas* C) {m_canvas = C ;}

   /// We draw the concentric rings (which are contours marking distance
   /// from the laser range finder) centered at the canvas's origin and
   /// spanning the entire range of the laser range finder's distance
   /// measurements. This method allows clients to specify the maximum
   /// value in the world coordinate system over which the rings should
   /// extend.
   void set_maximum(float m) {m_max = abs(m) ;}

   /// Draw the markers. Each subclass must implement this method using
   /// its own technique for representing the markers.
   virtual void render() const = 0 ;

   /// Clean-up.
   virtual ~LaserWindowMarkings() ;

protected:
   /// Helper for drawing the main axes.
   void draw_main_axes() const ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of LRF distance markings.
   class Params : public singleton<Params> {
      /// Private constructor because this is a singleton.
      Params() ;
      friend class singleton<Params> ;

      /// Whether or not the main axes should be drawn.
      bool m_draw_main_axes ;

      /// The main axes color.
      GLColor m_main_axes_color ;

   public:
      // Accessing the various parameters
      static bool main_axes_enabled() ;
      static bool main_axes_disabled() {return ! main_axes_enabled() ;}
      static const GLColor& main_axes_color() ;

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
