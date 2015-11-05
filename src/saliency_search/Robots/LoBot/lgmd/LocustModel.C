/**
   \file  Robots/LoBot/lgmd/LocustModel.C
   \brief Abstract base class for different locust models.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/lgmd/LocustModel.C $
// $Id: LocustModel.C 13183 2010-04-08 07:17:49Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/lgmd/LocustModel.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoGL.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLU
#include <GL/glu.h>
#endif

#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <iomanip>
#include <sstream>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

LocustModel::LocustModel(const LocustModel::InitParams& p)
   : Drawable(p.name, p.geometry),
     m_lgmd(0), m_range(p.spike_range),
     m_source(p.source), m_direction(p.direction),
     m_rect(p.rect), m_lrf_range(p.lrf_range),
     m_distance(-1), m_tti(-1)
{
   // DEVNOTE: Ought to be okay to use this variable without the
   // visualization mutex because the visualization and other threads
   // have probably not yet been created at this point. Even if they are
   // up, they're probably waiting for the main thread to signal that the
   // application has been fully initialized...
   m_spikes.resize(p.geometry.width) ;
}

LocustModel::InitParams::InitParams()
   : spike_range(0, 1), direction(0),
     source(0), rect(Rectangle::tlbrI(0, 0, 0, 0)), lrf_range(-119, 135)
{}

//--------------------- SPIKE RATE VISUALIZATION ------------------------

void LocustModel::add_spike(float s)
{
   viz_lock() ;
      m_spikes.pop_front() ;
      m_spikes.push_back(s) ;
   viz_unlock() ;
}

#ifdef INVT_HAVE_LIBGLU

static std::string str(float s)
{
   using namespace std ;

   std::ostringstream label ;
   label << fixed << setprecision(1) << s << " Hz" ;
   return label.str() ;
}

void LocustModel::render_me()
{
   // Make local copy so as to not hold up main thread
   viz_lock() ;
      std::deque<float> spikes = m_spikes ;
   viz_unlock() ;

   // Setup view volume so that x-coordinates match the amount of spike
   // history available and y-coordinates match the spike range.
   //
   // NOTE: We pad the top a little to ensure that the spike trains don't
   // obscure the text label drawn on the top left corner of the
   // visualization.
   glMatrixMode(GL_PROJECTION) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   gluOrtho2D(0, m_geometry.width, 0, 1.25f * m_range.max()) ;

   glMatrixMode(GL_MODELVIEW) ;
   glPushMatrix() ;
   glLoadIdentity() ;

   // Draw the spike train
   glPushAttrib(GL_COLOR_BUFFER_BIT) ;
   glBegin(GL_LINE_STRIP) ;
      glColor3f(0.15f, 0.85f, 0.60f) ;
      for (int x = 0; x < m_geometry.width; ++x)
         glVertex2f(x, spikes[x]) ;
   glEnd() ;
   glPopAttrib() ;

   // Reset view volume for drawing a little "icon" to show the direction
   // in which this locust is looking plus its FOV.
   glMatrixMode(GL_PROJECTION) ;
   glLoadIdentity() ;
   gluOrtho2D(-1, 1, -1, 1) ;

   glMatrixMode(GL_MODELVIEW) ;
   glLoadIdentity() ;
   glTranslatef(0, -0.9f, 0) ;
   glRotatef(get_conf("laser_viz", "lrf_direction", 90.0f), 0, 0, 1) ;
   glScalef(0.375f, 0.375f, 1) ;

   // Show the locust's viewing direction and FOV
   glColor3f(0.75f, 0.45f, 0.05f) ;
   glBegin(GL_LINES) ;
      glVertex2i(0, 0) ;
      glVertex2f(cos(m_direction), sin(m_direction)) ;

      glVertex2i(0, 0) ;
      glVertex2f(cos(m_lrf_range.min()), sin(m_lrf_range.min())) ;

      glVertex2i(0, 0) ;
      glVertex2f(cos(m_lrf_range.max()), sin(m_lrf_range.max())) ;
   glEnd() ;

   // Restore view volume
   glMatrixMode(GL_PROJECTION) ;
   glPopMatrix() ;
   glMatrixMode(GL_MODELVIEW) ;
   glPopMatrix() ;

   // Show latest spike rate in top left corner of drawing area
   text_view_volume() ;
      glColor3f(0, 1, 1) ;
      draw_label(3, 12, str(spikes.back()).c_str()) ;
   restore_view_volume() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

LocustModel::~LocustModel(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
