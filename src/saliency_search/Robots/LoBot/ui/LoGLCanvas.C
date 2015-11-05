/**
   \file Robots/LoBot/ui/LoGLCanvas.C

   \brief This file defines the non-inline member functions of the
   lobot::GLCanvas class, which is used to encapsulate window and
   viewport operations for doing 2D graphics in OpenGL.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoGLCanvas.C $
// $Id: LoGLCanvas.C 13037 2010-03-23 01:00:53Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

#if !defined(INVT_HAVE_LIBGL) || !defined(INVT_HAVE_LIBGLU)

#include "Robots/LoBot/ui/LoGLCanvas.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

GLCanvas::GLCanvas()
{
   throw missing_libs(MISSING_OPENGL) ;
}

// Empty API
void GLCanvas::set_window(float, float, float, float){}
void GLCanvas::set_viewport(int, int, int, int){}
void GLCanvas::screen_to_world(int, int, double*, double*){}
void GLCanvas::zoom_to(float){}
void GLCanvas::pan(float, float){}
void GLCanvas::reset_zoom_pan(){}

GLCanvas::~GLCanvas(){}

#else // OpenGL available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoGLCanvas.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/util/LoMath.H"

// OpenGL headers
#include <GL/glu.h>
#include <GL/gl.h>

// Standard C++ headers
#include <algorithm>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

GLCanvas::GLCanvas()
   : m_zoom_level(1)
{
   m_window[LEFT]   = 0 ;
   m_window[RIGHT]  = 0 ;
   m_window[BOTTOM] = 0 ;
   m_window[TOP]    = 0 ;

   m_viewport[LEFT]   = 0 ;
   m_viewport[RIGHT]  = 0 ;
   m_viewport[BOTTOM] = 0 ;
   m_viewport[TOP]    = 0 ;
}

//-------------- WINDOW AND VIEWPORT RELATED OPERATIONS ----------------

// Setup the world coordinate system (window in graphics speak)
void GLCanvas::set_window(float left, float right, float bottom, float top)
{
   m_window[LEFT]   = left ;
   m_window[RIGHT]  = right ;
   m_window[BOTTOM] = bottom ;
   m_window[TOP]    = top ;

   glMatrixMode(GL_PROJECTION) ;
   glLoadIdentity() ;
   gluOrtho2D(left, right, bottom, top) ;

   glMatrixMode(GL_MODELVIEW) ;
   glLoadIdentity() ;
}

// Setup graphics viewport with aspect ratio matching window aspect ratio
void GLCanvas::set_viewport(int left, int right, int bottom, int top)
{
   const int w = right - left ;
   const int h = top - bottom ;
   if (w <= 0 || h <= 0)
      return ;

   const float R = // window aspect ratio
      (m_window[RIGHT] - m_window[LEFT])/(m_window[TOP] - m_window[BOTTOM]) ;
   const float r = static_cast<float>(w)/h ; // viewport aspect ratio
   if (R < r) // tall thin window ==> adjust viewport width
   {
      const int w_adjusted = static_cast<int>(R * h) ;
      left  = (w - w_adjusted)/2 ;
      right = left + w_adjusted ;
   }
   else // short stout window ==> adjust viewport height
   {
      const int h_adjusted = static_cast<int>(w/R) ;
      bottom = (h - h_adjusted)/2 ;
      top    = bottom + h_adjusted ;
   }

   m_viewport[LEFT]   = left ;
   m_viewport[RIGHT]  = right ;
   m_viewport[BOTTOM] = bottom ;
   m_viewport[TOP]    = top ;

   glViewport(left, bottom, right - left, top - bottom) ;
}

void GLCanvas::get_viewport(int* x, int* y, int* width, int* height)
{
   if (x)
      *x = m_viewport[LEFT] ;
   if (y)
      *y = m_viewport[BOTTOM] ;
   if (width)
      *width = m_viewport[RIGHT] - m_viewport[LEFT] ;
   if (height)
      *height = m_viewport[TOP] - m_viewport[BOTTOM] ;
}

void GLCanvas::screen_to_world(int x, int y, double* wx, double* wy)
{
   if (! wx || ! wy)
      return ;

   GLint viewport[4] ;
   glGetIntegerv(GL_VIEWPORT, viewport) ;

   GLdouble mvmatrix[16] ;
   glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix) ;

   GLdouble projmatrix[16] ;
   glGetDoublev (GL_PROJECTION_MATRIX, projmatrix) ;

   GLdouble wz ;
   gluUnProject(x, viewport[3] - y - 1, 0, mvmatrix, projmatrix, viewport,
                wx, wy, & wz) ;
}

//----------------------------- ZOOM/PAN --------------------------------

void GLCanvas::zoom_to(float zoom_level)
{
   zoom_level = clamp(zoom_level, Params::zoom_range()) ;
   float zoom_factor = zoom_level/m_zoom_level ;
   m_zoom_level = zoom_level ;

   float current_width  = m_window[RIGHT] - m_window[LEFT] ;
   float current_height = m_window[TOP] - m_window[BOTTOM] ;

   float new_width  = current_width /zoom_factor ;
   float new_height = current_height/zoom_factor ;

   float width_incr  = abs((new_width  - current_width )/2) ;
   float height_incr = abs((new_height - current_height)/2) ;

   if (new_width > current_width) // zoom-out
   {
      m_window[LEFT]   -= width_incr ;
      m_window[RIGHT]  += width_incr ;
      m_window[BOTTOM] -= height_incr ;
      m_window[TOP]    += height_incr ;
   }
   else // zoom-in
   {
      m_window[LEFT]   += width_incr ;
      m_window[RIGHT]  -= width_incr ;
      m_window[BOTTOM] += height_incr ;
      m_window[TOP]    -= height_incr ;
   }
   set_window(m_window) ;
}

void GLCanvas::pan(float dx, float dy)
{
   m_window[LEFT]   += dx ;
   m_window[RIGHT]  += dx ;
   m_window[BOTTOM] += dy ;
   m_window[TOP]    += dy ;

   set_window(m_window) ;
}

void GLCanvas::reset_zoom_pan()
{
   zoom_to(1) ;
   pan(-(m_window[LEFT] + m_window[RIGHT])/2,
       -(m_window[BOTTOM] + m_window[TOP])/2) ;
}

//----------------------------- CLEAN-UP --------------------------------

GLCanvas::~GLCanvas()
{}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
GLCanvas::Params::Params()
   : m_zoom_range(get_conf<float>("zoom_pan", "zoom_range",
                                  make_range(0.0001f, 1000.0f)))
{}

// Parameters clean-up
GLCanvas::Params::~Params(){}

// Parameters access
const range<float>& GLCanvas::Params::zoom_range()
{
   return instance().m_zoom_range ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // OpenGL availability check

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
