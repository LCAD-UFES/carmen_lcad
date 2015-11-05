/**
   \file Robots/LoBot/ui/LoLaserWindowRings.C

   \brief This file defines the non-inline member functions of the
   lobot::LaserWindowRings class, which is used to draw contours
   indicating distance from the laser range finder.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserWindowRings.C $
// $Id: LoLaserWindowRings.C 13037 2010-03-23 01:00:53Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case OpenGL is missing...
#ifndef INVT_HAVE_LIBGL

#include "Robots/LoBot/ui/LoLaserWindowRings.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

LaserWindowRings::my_factory LaserWindowRings::register_me("rings") ;

LaserWindowRings::LaserWindowRings()
   : base(), m_display_list(0)
{
   throw missing_libs(MISSING_OPENGL) ;
}

// Empty API
void LaserWindowRings::render() const {}

LaserWindowRings::~LaserWindowRings(){}

}

#else // OpenGL available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserWindowRings.H"
#include "Robots/LoBot/util/LoMath.H"

// OpenGL headers
#include <GL/gl.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

// Factory registration
LaserWindowRings::my_factory LaserWindowRings::register_me("rings") ;

LaserWindowRings::LaserWindowRings()
   : base(), m_display_list(0)
{}

//------------------------- DRAWING THE RINGS ---------------------------

// This function object draws the concentric circular distance contours
// corresponding to a given marking interval, taking care to only draw
// the contours when they are actually active (as determined by the
// current zoom level and the marking's zoom range).
class draw_contours {
   float zoom_level, max ;
   int   display_list ;
public:
   draw_contours(float zoom_level, float max, int display_list) ;
   void operator()(const LaserWindowMarkings::Marking&) const ;
} ;

draw_contours::draw_contours(float Z, float M, int L)
   : zoom_level(Z), max(M), display_list(L)
{}

void draw_contours::operator()(const LaserWindowRings::Marking& M) const
{
   if (! M.third.in(zoom_level)) // current zoom level out of marking's
      return ;                   // zoom range

   glColor3fv(M.second.rgb()) ;
   glPushMatrix() ;
   for (float radius = M.first; radius <= max; radius += M.first) {
      glLoadIdentity() ;
      glScalef(radius, radius, 1) ;
      glCallList(display_list) ;
   }
   glPopMatrix() ;
}

// Draw the contours using the marking specifications
void LaserWindowRings::render() const
{
   if (! m_canvas)
      return ;

   // For the rings, we use an OpenGL display list that draws a unit
   // circle. To draw bigger circles, we apply a scaling transformation.
   //
   // DEVNOTE: It'd be nice to create the display list in the constructor
   // and then not bother performing this check each and every time.
   // Unfortunately, display lists are associated with the GL rendering
   // contexts for which they're created. And GL rendering contexts are
   // associated with the particular thread that created them.
   //
   // Putting this display list creation in the constructor will end up
   // associating it with the Robolocust main thread because that's the
   // beast that creates the laser visualizer object that creates and
   // uses this markings object. Since this application is setup so that
   // only the UI/visualization thread makes GL related rendering calls,
   // the display list, being associated with the main thread, won't work
   // in the context of the UI thread, which is why we need to create the
   // display list in the render function here, thus, ensuring that it is
   // created within the correct GL and thread contexts.
   if (! m_display_list)
   {
      m_display_list = glGenLists(1) ;
      glNewList(m_display_list, GL_COMPILE) ;
      glBegin(GL_LINE_LOOP) ;
         for (float angle = 0; angle < 360; ++angle)
            glVertex2f(cos(angle), sin(angle)) ;
      glEnd() ;
      glEndList() ;
   }

   const float Z = m_canvas->zoom_level() ;
   glPushAttrib(GL_COLOR_BUFFER_BIT) ;
      std::for_each(m_markings.begin(), m_markings.end(),
                    draw_contours(Z, m_max, m_display_list)) ;
      base::draw_main_axes() ;
   glPopAttrib() ;
}

//----------------------------- CLEAN-UP --------------------------------

LaserWindowRings::~LaserWindowRings()
{
   glDeleteLists(m_display_list, 1) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_LIBGL

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
