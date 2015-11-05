/**
   \file Robots/LoBot/ui/LoLaserWindowGrid.C

   \brief This file defines the non-inline member functions of the
   lobot::LaserWindowGrid class that is used to draw the LRF measurements
   grid.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserWindowGrid.C $
// $Id: LoLaserWindowGrid.C 11191 2009-05-16 19:11:59Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case OpenGL is missing...
#ifndef INVT_HAVE_LIBGL

#include "Robots/LoBot/ui/LoLaserWindowGrid.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

LaserWindowGrid::my_factory LaserWindowGrid::register_me("grid") ;

LaserWindowGrid::LaserWindowGrid()
   : base()
{
   throw missing_libs(MISSING_OPENGL) ;
}

// Empty API
void LaserWindowGrid::render() const {}

LaserWindowGrid::~LaserWindowGrid(){}

}

#else // OpenGL available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserWindowGrid.H"

// OpenGL headers
#include <GL/gl.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//----------------------- FACTORY REGISTRATION --------------------------

LaserWindowGrid::my_factory LaserWindowGrid::register_me("grid") ;

//-------------------------- INITIALIZATION -----------------------------

// Constructor
LaserWindowGrid::LaserWindowGrid()
   : base()
{}

//------------------------- DRAWING THE GRID ----------------------------

// This function object draws the grid lines corresponding to a given
// marking interval, taking care to only draw the grid lines when they
// are actually active (as determined by the current zoom level and the
// marking's zoom range).
class draw_grid_lines {
   float zoom_level, max ;
public:
   draw_grid_lines(float zoom_level, float max) ;
   void operator()(const LaserWindowGrid::Marking&) const ;
} ;

draw_grid_lines::draw_grid_lines(float Z, float M)
   : zoom_level(Z), max(M)
{}

void draw_grid_lines::operator()(const LaserWindowGrid::Marking& M) const
{
   if (! M.third.in(zoom_level)) // current zoom level out of marking's
      return ;                   // zoom range

  glColor3fv(M.second.rgb()) ;
  for (float i = M.first; i <= max; i += M.first) {
      glVertex2f(i,  max) ;
      glVertex2f(i, -max) ;
      glVertex2f(-max, i) ;
      glVertex2f( max, i) ;
      glVertex2f(-i,  max) ;
      glVertex2f(-i, -max) ;
      glVertex2f(-max, -i) ;
      glVertex2f( max, -i) ;
  }
}

// Draw the grid using the marking specifications
void LaserWindowGrid::render() const
{
   if (! m_canvas)
      return ;

   glPushAttrib(GL_COLOR_BUFFER_BIT) ;
      glBegin(GL_LINES) ;
         std::for_each(m_markings.begin(), m_markings.end(),
                       draw_grid_lines(m_canvas->zoom_level(), m_max)) ;
      glEnd() ;
      base::draw_main_axes() ;
   glPopAttrib() ;
}

//----------------------------- CLEAN-UP --------------------------------

LaserWindowGrid::~LaserWindowGrid()
{}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_LIBGL

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
