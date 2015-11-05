/**
   \file Robots/LoBot/ui/LoLaserViz.C

   This file defines the non-inline member functions of the
   lobot::LaserViz class used for visualizing the laser range finder
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserViz.C $
// $Id: LoLaserViz.C 13674 2010-07-18 22:13:22Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserViz.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/util/LoMath.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

LaserViz::LaserViz(const LaserRangeFinder* lrf)
   : Drawable("laser_viz", Params::geometry()),
     m_lrf(lrf),
     m_canvas(new GLCanvas()),
     m_markings(factory<LaserWindowMarkings>::create(Params::markings_type()))
{
   if (! lrf)
      throw misc_error(LASER_RANGE_FINDER_MISSING) ;

   m_markings->use_canvas(m_canvas.get()) ;
   m_markings->set_maximum(m_lrf->get_distance_range().max()) ;
}

void LaserViz::gl_init()
{
   const int M = m_lrf->get_distance_range().max() ;
   m_canvas->set_window(-M, M, -M, M) ;
}

//----------------------------- RENDERING -------------------------------

#ifdef INVT_HAVE_LIBGL

// This function draws the measurements made by the laser range finder,
// showing them as rays emanating from the origin of the world coordinate
// system (where the laser range finder is positioned; yes, the laser
// range finder is the center of the world).
//
// Since drawing each and every measurement can make the resulting
// picture crowded, this function only draws the distance measurements
// corresponding to angles within the [min, max] range with the specified
// step size.
static void draw_rays(const LaserRangeFinder* lrf,
                      const range<int>& angles, int step, const GLColor& color)
{
   glPushAttrib(GL_COLOR_BUFFER_BIT) ;
   glBegin(GL_LINES) ;
      glColor3fv(color.rgb()) ;

      for (float angle = angles.min(); angle <= angles.max(); angle += step)
      {
         int D = lrf->get_distance(static_cast<int>(angle)) ;
         if (D < 0) // didn't get a valid reading in this direction
            continue ;
         glVertex2i(0, 0) ;
         glVertex2f(D * cos(angle), D * sin(angle)) ;
      }

      // In case the above loop missed zero degrees, i.e., straight in
      // front of the laser range finder...
      int D = lrf->get_distance(0) ;
      if (D > 0) {
         glVertex2i(0, 0) ;
         glVertex2i(0, D) ;
      }
   glEnd() ;
   glPopAttrib() ;
}

// This function draws the measurements made by the laser range finder,
// showing them as an outline hull.
//
// Since drawing each and every measurement can make the resulting
// picture crowded, this function only draws the distance measurements
// corresponding to angles within the [min, max] range with the specified
// step size.
static void draw_hull(const LaserRangeFinder* lrf,
                      const range<int>& angles, int step, const GLColor& color)
{
   glPushAttrib(GL_COLOR_BUFFER_BIT) ;
   glBegin(GL_LINE_LOOP) ;
      glColor3fv(color.rgb()) ;

      glVertex2i(0, 0) ;
      for (float angle = angles.min(); angle <= angles.max(); angle += step)
      {
         int D = lrf->get_distance(static_cast<int>(angle)) ;
         if (D < 0) // didn't get a valid reading in this direction
            continue ;
         glVertex2f(D * cos(angle), D * sin(angle)) ;
      }
   glEnd() ;
   glPopAttrib() ;
}

// The laser range finder is depicted as a rectangle with a triangle on
// it serving to let users know where the front of the device is. This
// function expects to be passed the half-size R of a square inside of
// which the entire rectangle + triangle combo is to inscribed. The
// rectangle is drawn with sides R and 2R; the triangle is drawn with
// height R and base length 2R.
static void draw_lrf(float R, const GLColor& color)
{
   glPushAttrib(GL_COLOR_BUFFER_BIT) ;
   glBegin(GL_TRIANGLES) ;
      glColor3fv(color.rgb()) ;

      // The triangle
      glVertex2f(R,  0) ; // apex
      glVertex2f(0,  R) ; // base
      glVertex2f(0, -R) ;

      // The rectangle (drawn as two triangles)
      glVertex2f( 0,  R) ;
      glVertex2f(-R,  R) ;
      glVertex2f( 0, -R) ;
      glVertex2f(-R,  R) ;
      glVertex2f(-R, -R) ;
      glVertex2f( 0, -R) ;
   glEnd() ;
   glPopAttrib() ;
}

// This method draws the latest set of distance measurements from the
// laser range finder.
void LaserViz::render_me()
{
   glPushMatrix() ;
      glRotatef(Params::lrf_direction(), 0, 0, 1) ;
      m_markings->render() ;
      if (Params::draw_rays())
         draw_rays(m_lrf, Params::angles_range(), Params::angles_step(),
                   Params::measurements_color()) ;
      else
         draw_hull(m_lrf, Params::angles_range(), Params::angles_step(),
                   Params::measurements_color()) ;
      draw_lrf(Params::lrf_size(), Params::lrf_color()) ;
   glPopMatrix() ;
}

#endif // INVT_HAVE_LIBGL

//----------------------------- ZOOM/PAN --------------------------------

void LaserViz::zoom_by(float dz)
{
   m_canvas->zoom_by(dz) ;
}

void LaserViz::pan(int cx, int cy, int px, int py)
{
   double curr_x, curr_y ;
   m_canvas->screen_to_world(cx, cy, &curr_x, &curr_y) ;

   double prev_x, prev_y ;
   m_canvas->screen_to_world(px, py, &prev_x, &prev_y) ;

   const float dx = static_cast<float>(curr_x - prev_x) ;
   const float dy = static_cast<float>(curr_y - prev_y) ;
   m_canvas->pan(-dx, -dy) ;
}

void LaserViz::reset_zoom_pan()
{
   m_canvas->reset_zoom_pan() ;
}

//----------------------------- CLEAN-UP --------------------------------

void LaserViz::gl_cleanup()
{
   // Force auto_ptr to delete the markings object now rather than in
   // destructor to ensure that any GL resources held by it are cleaned
   // up here. Otherwise, depending on the order in which the thread
   // scheduler shuts down different threads, the GL rendering context
   // could disappear before we get around to properly cleaning up...
   m_markings.reset(0) ;

   // Ditto for the GL canvas
   m_canvas.reset(0) ;
}

LaserViz::~LaserViz(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Retrieve settings from laser_viz section of config file
template<typename T>
static inline T viz_conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("laser_viz", key, default_value) ;
}

// Overload of above function for retrieving ranges
template<typename T>
static inline range<T>
viz_conf(const std::string& key, const range<T>& default_value)
{
   return get_conf<T>("laser_viz", key, default_value) ;
}

// Overload for retrieving triples
template<typename T>
static inline triple<T, T, T>
viz_conf(const std::string& key, const triple<T, T, T>& default_value)
{
   return get_conf<T>("laser_viz", key, default_value) ;
}

// Retrieve settings from markings section of config file
template<typename T>
static inline T markings_conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("markings", key, default_value) ;
}

// Parameters initialization
LaserViz::Params::Params()
   : m_geometry(viz_conf<std::string>("geometry", "0 0 480 480")),
     m_measurements_style(downstring(
        viz_conf<std::string>("measurements_style", "rays"))),
     m_markings_type(downstring(markings_conf<std::string>("type", "rings"))),
     m_angles_range(viz_conf<int>("angles_range", make_range(-119, 135))),
     m_angles_step(clamp(viz_conf("angles_step", 5), 1, 30)),
     m_measurements_color(viz_conf<int>("measurements_color",
                                        make_triple(0, 128, 128))),
     m_lrf_size(clamp(viz_conf("lrf_size", 100.0f), 10.0f, 250.0f)),
     m_lrf_direction(clamp_angle(viz_conf("lrf_direction", 90.0f))),
     m_lrf_color(viz_conf<int>("lrf_color", make_triple(242, 13, 26)))
{}

// Parameters clean-up
LaserViz::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
