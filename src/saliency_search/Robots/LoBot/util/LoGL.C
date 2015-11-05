/**
   \file  Robots/LoBot/util/LoGL.C
   \brief Some OpenGL related helpers.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoGL.C $
// $Id: LoGL.C 13037 2010-03-23 01:00:53Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/LoGL.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLUT
#include <GL/glut.h>
#endif

#ifdef INVT_HAVE_LIBGLU
#include <GL/glu.h>
#endif

#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- OPENGL HELPERS -----------------------------

#if defined(INVT_HAVE_LIBGL)  && \
    defined(INVT_HAVE_LIBGLU) && \
    defined(INVT_HAVE_LIBGLUT)

// Draw a text label in a GLUT window
void draw_label(float x, float y, const char* label)
{
   glRasterPos2f(x, y) ;
   for (const char* s = label; *s; ++s)
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *s) ;
}

// Draw a vector from the origin
void draw_vector(const Vector& v)
{
   // Draw the vector's "body" with its tail pinned at the origin
   glBegin(GL_LINES) ;
      glVertex2i(0, 0) ;
      glVertex2f(v.i, v.j) ;
   glEnd() ;

   // Draw an arrow head at the vector's head
   Vector n = normalized(v) ;
   Vector p = 0.91f * v ;
   Vector q = p + 0.03f * Vector(-n.j,  n.i) ;
   Vector r = p + 0.03f * Vector( n.j, -n.i) ;
   glBegin(GL_TRIANGLES) ;
      glVertex2f(v.i, v.j) ;
      glVertex2f(q.i, q.j) ;
      glVertex2f(r.i, r.j) ;
   glEnd() ;
}

#else // no OpenGL ==> don't make any GL rendering calls

void draw_label(float, float, const char*){}
void draw_vector(const Vector&){}

#endif

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
