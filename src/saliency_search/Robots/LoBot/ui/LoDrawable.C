/**
   \file  Robots/LoBot/ui/LoDrawable.C
   \brief A base class for rendering things in the Robolocust UI.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoDrawable.C $
// $Id: LoDrawable.C 13958 2010-09-17 11:45:00Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case OpenGL and/or GLUT are missing
//
// NOTE: Don't really need to check INVT_HAVE_LIBGL and INVT_HAVE_LIBGLU
// as well because it ought to be a pretty rare/broken installation that
// has GLUT but not the OpenGL libraries...
#ifndef INVT_HAVE_LIBGLUT

#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

Drawable::Drawable()
{
   throw missing_libs(MISSING_OPENGL) ;
}

// Empty API
void Drawable::gl_init(){}
void Drawable::add_hook(const Drawable::RenderHook&){}
void Drawable::render(){}
void Drawable::render_me(){}
void Drawable::zoom_by(float){}
void Drawable::pan(int, int, int, int){}
void Drawable::reset_zoom_pan(){}
void Drawable::keypress(unsigned char){}
void Drawable::unit_view_volume() const {}
void Drawable::text_view_volume() const {}
void Drawable::restore_view_volume() const {}

void Drawable::gl_cleanup(){}
Drawable::~Drawable(){}

} // end of namespace encapsulating above empty definition

#else // pthreads, OpenGL and GLUT available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoString.H"

// OpenGL headers
#include <GL/glu.h>
#include <GL/gl.h>

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Drawable::Drawable(const std::string& name, const Geometry& g)
   : m_name(name), m_geometry(g),
     m_border(true), m_border_color(1.0f, 1.0f, 0.0f)
{}

Drawable::Geometry::Geometry(int xx, int yy, int w, int h)
   : x(xx), y(yy), width(w), height(h)
{}

Drawable::Geometry::Geometry(const std::string& geometry)
{
   std::vector<int> g = string_to_vector<int>(geometry) ;
   if (g.size() < 4)
      throw customization_error(BAD_GEOMETRY_SPEC) ;

   x = g[0] ;
   y = g[1] ;
   width  = g[2] ;
   height = g[3] ;
}

void Drawable::gl_init(){}

void Drawable::add_hook(const Drawable::RenderHook& h)
{
   AutoMutex M(m_hooks_mutex) ;
   m_hooks.push_back(h) ;
}

//----------------------------- RENDERING -------------------------------

void Drawable::render()
{
   render_me() ;

   // Render overlays
   AutoMutex M(m_hooks_mutex) ;
   std::for_each(m_hooks.begin(), m_hooks.end(), trigger_hook) ;
}

void Drawable::trigger_hook(const Drawable::RenderHook& h)
{
   h.first(h.second) ;
}

//---------------------------- DEFAULT API ------------------------------

void Drawable::render_me(){}
void Drawable::zoom_by(float){}
void Drawable::pan(int, int, int, int){}
void Drawable::reset_zoom_pan(){}
void Drawable::keypress(unsigned char){}

//------------------------------ HELPERS --------------------------------

// Setup the OpenGL 2D view volume so that it goes from -1 to +1 in both
// x- and y-directions.
void Drawable::unit_view_volume() const
{
   glMatrixMode(GL_PROJECTION) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   gluOrtho2D(-1, 1, -1, 1) ;

   glMatrixMode(GL_MODELVIEW) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   glRotatef(get_conf("laser_viz", "lrf_direction", 90.0f), 0, 0, 1) ;
}

// Setup the OpenGL 2D view volume so that it corresponds to the
// drawable's geometry specification.
void Drawable::text_view_volume() const
{
   glMatrixMode(GL_PROJECTION) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   gluOrtho2D(0, m_geometry.width, 0, m_geometry.height) ;

   glMatrixMode(GL_MODELVIEW) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   glRotatef(180, 0, 0, 1) ;
   glRotatef(180, 0, 1, 0) ;
   glTranslatef(0, -m_geometry.height, 0) ;
}

// Setup the OpenGL 2D view volume using the supplied parameters
void Drawable::setup_view_volume(float L, float R, float B, float T) const
{
   glMatrixMode(GL_PROJECTION) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   gluOrtho2D(L, R, B, T) ;

   glMatrixMode(GL_MODELVIEW) ;
   glPushMatrix() ;
   glLoadIdentity() ;
}

// Undo the effects of the above functions
void Drawable::restore_view_volume() const
{
   glMatrixMode(GL_PROJECTION) ;
   glPopMatrix() ;

   glMatrixMode(GL_MODELVIEW) ;
   glPopMatrix() ;
}

//----------------------------- CLEAN-UP --------------------------------

void Drawable::gl_cleanup(){}
Drawable::~Drawable(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #if !defined(INVT_HAVE_LIBPTHREAD) || !defined(INVT_HAVE_LIBGLUT)

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
