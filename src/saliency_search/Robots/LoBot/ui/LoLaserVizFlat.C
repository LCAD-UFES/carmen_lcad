/**
   \file Robots/LoBot/ui/LoLaserVizFlat.C

   This file defines the non-inline member functions of the
   lobot::LaserVizFlat class used for visualizing the laser range finder
   measurements.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserVizFlat.C $
// $Id: LoLaserVizFlat.C 13037 2010-03-23 01:00:53Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserVizFlat.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoExcept.H"

#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoString.H"
#include "Robots/LoBot/util/range.hh"
#include "Robots/LoBot/util/triple.hh"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLU
#include <GL/glu.h>
#endif

#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <string>
#include <algorithm>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------- INITIALIZATION AND CLEAN-UP ----------------------

LaserVizFlat::LaserVizFlat(const LaserRangeFinder* lrf)
   : Drawable("laser_viz_flat", Params::geometry()),
     m_lrf(lrf)
{
   if (! lrf)
      throw misc_error(LASER_RANGE_FINDER_MISSING) ;
}

LaserVizFlat::~LaserVizFlat(){}

//----------------------------- RENDERING -------------------------------

#if defined(INVT_HAVE_LIBGLU) && defined(INVT_HAVE_LIBGL)

// Helper to convert the max reading into a string
static std::string max_reading_str(int r)
{
   return std::string("Max: ") + to_string(r) + " mm" ;
}

// This method draws the latest set of distance measurements from the
// laser range finder.
void LaserVizFlat::render_me()
{
   const range<int> A = m_lrf->get_angular_range() ;
   const int M = std::max(abs(A.min()), abs(A.max())) ;
   const int R = m_lrf->max_reading() ;

   glMatrixMode(GL_PROJECTION) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   gluOrtho2D(-M, M, 0, R);

   glMatrixMode(GL_MODELVIEW) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   glRotatef(180, 0, 1, 0) ;

   glPushAttrib(GL_COLOR_BUFFER_BIT) ;
   glBegin(GL_LINE_STRIP) ;
      glColor3fv(Params::color().rgb()) ;
      glVertex2i(-M, 0) ;
      for (int x = A.min(); x <= A.max(); ++x)
      {
         int y = m_lrf->get_distance(x) ;
         if (y < 0) // bad reading
            continue ;
         glVertex2i(x, y) ;
      }
      glVertex2i(M, 0) ;
   glEnd() ;
   glPopAttrib() ;

   glMatrixMode(GL_PROJECTION) ;
   glPopMatrix() ;

   glMatrixMode(GL_MODELVIEW) ;
   glPopMatrix() ;

   text_view_volume() ;
      glColor3f(1, 1, 0) ;
      draw_label(3, 12, "LRF") ;
      draw_label(3, m_geometry.height - 4, max_reading_str(R).c_str()) ;
   restore_view_volume() ;
}

#endif // defined(INVT_HAVE_LIBGLU) && defined(INVT_HAVE_LIBGL)

//-------------------------- KNOB TWIDDLING -----------------------------

// Retrieve settings from laser_viz_flat section of config file
template<typename T>
static inline T flat_conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("laser_viz_flat", key, default_value) ;
}

// For retrieving measurements_color from laser_viz section of config file
static inline triple<int, int, int>
color_conf(const triple<int, int, int>& default_value)
{
   return get_conf<int>("laser_viz", "measurements_color", default_value) ;
}

// Parameters initialization
LaserVizFlat::Params::Params()
   : m_geometry(flat_conf<std::string>("geometry", "0 480 480 80")),
     m_color(color_conf(make_triple(0, 128, 128)))
{}

// Parameters clean-up
LaserVizFlat::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
