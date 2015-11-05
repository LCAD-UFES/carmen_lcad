/**
   \file Robots/LoBot/ui/LoLaserWindow.C

   This file defines the non-inline member functions of the
   lobot::LaseWindow class used to encapsulate the GLUT-based window for
   the Hokuyo laser range finder's test program, which visualizes the
   laser range finder's measurement data with some simple 2D OpenGL.
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserWindow.C $
// $Id: LoLaserWindow.C 13037 2010-03-23 01:00:53Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case OpenGL and/or GLUT are missing
//
// NOTE: Don't really need to check INVT_HAVE_LIBGL and INVT_HAVE_LIBGLU
// as well because it ought to be a pretty rare/broken installation that
// has GLUT but not the OpenGL libraries...
#ifndef INVT_HAVE_LIBGLUT

#include "Robots/LoBot/ui/LoLaserWindow.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

void LaserWindow::create(const std::string&)
{
   throw missing_libs(MISSING_OPENGL) ;
}

// NOTE: Don't need empty definitions of the remaining member functions
// because they don't get used at all if OpenGL and/or GLUT are missing,
// which means that the linker won't complain about missing functions
// (since the compiler doesn't generate code for these functions).

}

#else // OpenGL and GLUT available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserWindow.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/util/LoMath.H"

// OpenGL headers
#include <GL/glut.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------ STATIC DATA MEMBERS --------------------------

// The window title of the GLUT window created by this class.
std::string LaserWindow::m_title ; // static to allow passage to def. ctor

//-------------------------- INITIALIZATION -----------------------------

// This is the only public method in this class. It is static and merely
// acts as an interface for creating the singleton LaserWindow object.
void LaserWindow::create(const std::string& title)
{
   m_title = title ;
   instance() ;
}

// Constructor sets up the laser range finder device and GLUT UI
LaserWindow::LaserWindow()
   : m_lrf(new LaserRangeFinder(Params::device(), Params::baud_rate())),
     m_window(glutCreateWindow(m_title.c_str())),
     m_canvas(new GLCanvas()),
     m_markings(factory<LaserWindowMarkings>::create(Params::markings_type())),
     m_paused(false),
     m_drag_button(-1),
     m_drag_modifiers(-1)
{
   const int M = m_lrf->get_distance_range().max() ;
   m_canvas->set_window(-M, M, -M, M) ;

   m_markings->use_canvas(m_canvas) ;
   m_markings->set_maximum(M) ;

   glutReshapeFunc(reshape_callback) ;
   glutDisplayFunc(render_callback) ;
   glutKeyboardFunc(keyboard_callback) ;
   glutMouseFunc(click_callback) ;
   glutMotionFunc(drag_callback) ;
   setup_timer() ;

   typedef LaserWindow me ; // typing shortcut
   m_keymap['r'] = & me::reset_zoom_pan ;
   m_keymap['p'] = & me::pause ;

   m_keymap['q'] = & me::quit ;
   m_keymap['Q'] = & me::quit ;
   m_keymap[27]  = & me::quit ; // ESC (assuming ASCII encoding)

   m_drag_prev[0] = m_drag_prev[1] = -1 ;
}

// This method resets the graphics viewport whenever the UI window is
// resized.
void LaserWindow::reshape(int W, int H)
{
   m_canvas->set_viewport(0, W, 0, H) ;
}

//------------------ UPDATING DISTANCE MEASUREMENTS ---------------------

void LaserWindow::update()
{
   m_lrf->update() ;
   glutPostRedisplay() ;
   if (! m_paused)
      setup_timer() ;
}

void LaserWindow::setup_timer()
{
   glutTimerFunc(Params::update_frequency(), timer_callback, 0) ;
}

//------------------ RENDERING DISTANCE MEASUREMENTS --------------------

// Forward declarations
void draw_measurements(const LaserRangeFinder*, const range<int>&, int,
                       const GLColor&) ;
void draw_lrf(float, const GLColor&) ;

// This method draws the latest set of distance measurements from the
// laser range finder.
void LaserWindow::render()
{
   glClear(GL_COLOR_BUFFER_BIT) ;

   glPushMatrix() ;
      glRotatef(Params::lrf_direction(), 0, 0, 1) ;
      m_markings->render() ;
      draw_measurements(m_lrf, Params::angles_range(), Params::angles_step(),
                        Params::measurements_color()) ;
      draw_lrf(Params::lrf_size(), Params::lrf_color()) ;
   glPopMatrix() ;

   glutSwapBuffers() ;
}

// This function draws the measurements made by the laser range finder,
// showing them as rays emanating from the origin of the world coordinate
// system (where the laser range finder is positioned; yes, the laser
// range finder is the center of the world).
//
// Since drawing each and every measurement can make the resulting
// picture crowded, this function only draws the distance measurements
// corresponding to angles within the [min, max] range with the specified
// step size.
void draw_measurements(const LaserRangeFinder* lrf,
                       const range<int>& angles, int step,
                       const GLColor& color)
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

      // In case the above loop missed zero degrees (i.e., straight in
      // front of the laser range finder...
      int D = lrf->get_distance(0) ;
      if (D > 0) {
         glVertex2i(0, 0) ;
         glVertex2i(0, D) ;
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
void draw_lrf(float R, const GLColor& color)
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

//-------------------------- KEYBOARD INPUT -----------------------------

// Use keymap to invoke appropriate handler for key pressed by user
void LaserWindow::handle_key(unsigned char key)
{
   KeyMap::iterator handler = m_keymap.find(key) ;
   if (handler == m_keymap.end())
      return ;
   (this->*(handler->second))() ;
   glutPostRedisplay() ;
}

void LaserWindow::reset_zoom_pan()
{
   m_canvas->reset_zoom_pan() ;
}

void LaserWindow::pause()
{
   m_paused = ! m_paused ;
   if (! m_paused)
      setup_timer() ;
}

void LaserWindow::quit()
{
   exit(0) ;
}

//---------------------------- MOUSE INPUT ------------------------------

void LaserWindow::left_click(int state, int modifiers, int x, int y)
{
   switch (state)
   {
      case GLUT_DOWN:
         m_drag_button    = GLUT_LEFT_BUTTON ;
         m_drag_modifiers = modifiers ;
         m_drag_prev[0]   = x ;
         m_drag_prev[1]   = y ;
         break ;
      case GLUT_UP:
         m_drag_button    = -1 ;
         m_drag_modifiers = -1 ;
         m_drag_prev[0]   = -1 ;
         m_drag_prev[1]   = -1 ;
         break ;
   }
}

void LaserWindow::middle_click(int state, int modifiers, int x, int y)
{
   switch (state)
   {
      case GLUT_DOWN:
         m_drag_button    = GLUT_MIDDLE_BUTTON ;
         m_drag_modifiers = modifiers ;
         m_drag_prev[0]   = x ;
         m_drag_prev[1]   = y ;
         break ;
      case GLUT_UP:
         m_drag_button    = -1 ;
         m_drag_modifiers = -1 ;
         m_drag_prev[0]   = -1 ;
         m_drag_prev[1]   = -1 ;
         break ;
   }
}

void LaserWindow::right_click(int state, int modifiers, int x, int y)
{
   switch (state)
   {
      case GLUT_DOWN:
         m_drag_button    = GLUT_RIGHT_BUTTON ;
         m_drag_modifiers = modifiers ;
         m_drag_prev[0]   = x ;
         m_drag_prev[1]   = y ;
         break ;
      case GLUT_UP:
         m_drag_button    = -1 ;
         m_drag_modifiers = -1 ;
         m_drag_prev[0]   = -1 ;
         m_drag_prev[1]   = -1 ;
         break ;
   }
}

void LaserWindow::left_drag(int x, int y)
{
   if (m_drag_modifiers & GLUT_ACTIVE_SHIFT) // zoom
   {
      const float dy = y - m_drag_prev[1] ;
      m_canvas->zoom_by(-dy * Params::zoom_drag_factor()) ;
   }
   else // pan
   {
      double curr_x, curr_y ;
      m_canvas->screen_to_world(x, y, & curr_x, & curr_y) ;

      double prev_x, prev_y ;
      m_canvas->screen_to_world(m_drag_prev[0], m_drag_prev[1],
                                & prev_x, & prev_y) ;

      const float dx = static_cast<float>(curr_x - prev_x) ;
      const float dy = static_cast<float>(curr_y - prev_y) ;
      m_canvas->pan(-dx, -dy) ;
   }

   m_drag_prev[0] = x ;
   m_drag_prev[1] = y ;

   glutPostRedisplay() ;
}

void LaserWindow::middle_drag(int x, int y)
{
   const float dy = y - m_drag_prev[1] ;
   m_canvas->zoom_by(-dy * Params::zoom_drag_factor()) ;

   m_drag_prev[0] = x ;
   m_drag_prev[1] = y ;

   glutPostRedisplay() ;
}

void LaserWindow::right_drag(int, int)
{
}

//-------------------------- GLUT CALLBACKS -----------------------------

void LaserWindow::reshape_callback(int width, int height)
{
   instance().reshape(width, height) ;
}

void LaserWindow::render_callback()
{
   instance().render() ;
}

void LaserWindow::keyboard_callback(unsigned char key, int, int)
{
   instance().handle_key(key) ;
}

void LaserWindow::click_callback(int button, int state, int x, int y)
{
   switch (button)
   {
      case GLUT_LEFT_BUTTON:
         instance().left_click(state, glutGetModifiers(), x, y) ;
         break ;
      case GLUT_MIDDLE_BUTTON:
         instance().middle_click(state, glutGetModifiers(), x, y) ;
         break ;
      case GLUT_RIGHT_BUTTON:
         instance().right_click(state, glutGetModifiers(), x, y) ;
         break ;
   }
}

void LaserWindow::drag_callback(int x, int y)
{
   switch (instance().m_drag_button)
   {
      case GLUT_LEFT_BUTTON:
         instance().left_drag(x, y) ;
         break ;
      case GLUT_MIDDLE_BUTTON:
         instance().middle_drag(x, y) ;
         break ;
      case GLUT_RIGHT_BUTTON:
         instance().right_drag(x, y) ;
         break ;
   }
}

// We use a timer to continuously update the laser range finder
void LaserWindow::timer_callback(int)
{
   instance().update() ;
}

//----------------------------- CLEAN-UP --------------------------------

LaserWindow::~LaserWindow()
{
   delete m_markings ;
   delete m_canvas ;
   glutDestroyWindow(m_window) ;
   delete m_lrf ;
}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
LaserWindow::Params::Params()
   : m_device(get_conf<std::string>("device", "port", "/dev/ttyACM0")),
     m_baud_rate(get_conf("device", "baud_rate", 115200)),
     m_markings_type(get_conf<std::string>("markings", "type", "rings")),
     m_update_frequency(clamp(get_conf("device", "update_frequency", 250),
                              100, 60000)),
     m_angles_range(get_conf<int>("measurements", "angles_range",
                                  make_range(-135, 135))),
     m_angles_step(clamp(get_conf("measurements", "angles_step", 5), 1, 30)),
     m_measurements_color(get_conf<int>("measurements", "color",
                                        make_triple(0, 128, 128))),
     m_lrf_size(clamp(get_conf("device", "lrf_size", 100.0f), 10.0f, 250.0f)),
     m_lrf_direction(clamp_angle(get_conf("device", "lrf_direction", 90.0f))),
     m_lrf_color(get_conf<int>("device", "lrf_color",
                               make_triple(242, 13, 26))),
     m_zoom_drag_factor(clamp(get_conf("zoom_pan", "zoom_drag_factor", 0.1f),
                              0.1f, 2.5f))
{}

// Parameters clean-up
LaserWindow::Params::~Params(){}

// Parameters access
const std::string& LaserWindow::Params::device()
{
   return instance().m_device ;
}

int LaserWindow::Params::baud_rate()
{
   return instance().m_baud_rate ;
}

const std::string& LaserWindow::Params::markings_type()
{
   return instance().m_markings_type ;
}

int LaserWindow::Params::update_frequency()
{
   return instance().m_update_frequency ;
}

const range<int>& LaserWindow::Params::angles_range()
{
   return instance().m_angles_range ;
}

int LaserWindow::Params::angles_step()
{
   return instance().m_angles_step ;
}

const GLColor& LaserWindow::Params::measurements_color()
{
   return instance().m_measurements_color ;
}

float LaserWindow::Params::lrf_size()
{
   return instance().m_lrf_size ;
}

float LaserWindow::Params::lrf_direction()
{
   return instance().m_lrf_direction ;
}

const GLColor& LaserWindow::Params::lrf_color()
{
   return instance().m_lrf_color ;
}

float LaserWindow::Params::zoom_drag_factor()
{
   return instance().m_zoom_drag_factor ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef INVT_HAVE_LIBGLUT

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
