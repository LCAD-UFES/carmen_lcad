/**
   \file Robots/LoBot/misc/LoClipper.C

   This file defines the non-inline member functions of the
   lobot::Clipper class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoClipper.C $
// $Id: LoClipper.C 13445 2010-05-21 06:12:57Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/LoClipper.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- INITIALIZATION -----------------------------

Clipper::Clipper(float left, float right, float bottom, float top)
{
   clip_boundary(left, right, bottom, top) ;
}

Clipper::Clipper(const float boundary[4])
{
   clip_boundary(boundary) ;
}

//------------------ COHEN-SUTHERLAND LINE CLIPPING ---------------------

unsigned char
Clipper::
clip(const float end_points[4], float new_end_points[4]) const
{
   unsigned char return_code = 0 ;

   new_end_points[0] = end_points[0] ;
   new_end_points[1] = end_points[1] ;
   new_end_points[2] = end_points[2] ;
   new_end_points[3] = end_points[3] ;

   float dx = new_end_points[2] - new_end_points[0] ;
   float dy = new_end_points[3] - new_end_points[1] ;

   for(;;)
   {
      unsigned char code1 = cs_code(new_end_points) ;
      unsigned char code2 = cs_code(new_end_points + 2) ;

      if (trivial_accept(code1, code2)) {
         if (return_code == 0)
            return_code = COMPLETELY_INSIDE ;
         return return_code ;
      }

      if (trivial_reject(code1, code2))
         return COMPLETELY_OUTSIDE ;

      if (outside(new_end_points))
      {  // first end-point outside clipping rectangle
         chop(new_end_points, code1, dx, dy) ;
         return_code |= FIRST_POINT_CLIPPED ;
      }
      else // second end-point must be outside clipping rectangle
      {    // or else the line would have been accepted or rejected
         chop(new_end_points + 2, code2, dx, dy) ;
         return_code |= SECOND_POINT_CLIPPED ;
      }
   }

   return return_code ;
}

/// The Cohen-Sutherland code for a point consists of four bit flags as
/// described below:
///    bit 3 = 1 ==> point is to the left of the clipping rectangle
///    bit 2 = 1 ==> point is above the clipping rectangle
///    bit 1 = 1 ==> point is to the right of the clipping rectangle
///    bit 0 = 1 ==> point is below the clipping rectangle
unsigned char Clipper::cs_code(const float point[2]) const
{
   unsigned char code = 0 ; // initially assume point is inside

   if (point[0] < m_clip_boundary[0])
      code |= LEFT_BIT ;

   if (point[1] > m_clip_boundary[3])
      code |= TOP_BIT ;

   if (point[0] > m_clip_boundary[1])
      code |= RIGHT_BIT ;

   if (point[1] < m_clip_boundary[2])
      code |= BOTTOM_BIT ;

   return code ;
}

/*
   If a line can not be trivially accepted or rejected, we break the line
   into two parts at the appropriate clipping boundary and reject the
   portion that lies outside. Then we test the remaining portion against
   the edges and continue until we get an acceptance or rejection.

   The remaining portion of the line is obtained by "moving" that point
   along the line to the intersection of the line with the clipping
   boundary it is being tested against. For eg., consider the situation
   shown below:

                           +--------
                           | P2
                           | /|
                           |/ |
                          I/  |
                          /|  dy
                         / h  |
                        /  |  |
                       /-+-|--+
                      P1 |
                         +--> dx

   We need to compute the intersection of P1-P2 with the left boundary
   of the clipping rectangle and make P1 that intersection. Let the
   intersection point be I. Obviously, the x-coordinate of I is the
   x-coordinate of the left edge. Let the horizontal distance between
   P1 and the left edge be d. From similar triangles, we get:

      h/dy = d/dx ==> h = d * dy/dx

   We can easily compute d:

      d = left - P1.x

   Therefore,

      I.y = P1.y + (left - P1.x) * dy/dx

   And if P1 were to become I, then we'd get:

      P1.y  = P1.y + (left - P1.x) * dy/dx
   or P1.y += (left - P1.x) * dy/dx

   Similar reasoning can be applied for the other cases.
*/
void
Clipper::
chop(float point[2], unsigned char code, float dx, float dy) const
{
   if (code & LEFT_BIT)
   {
      point[1] += (m_clip_boundary[0] - point[0]) * dy/dx ;
      point[0]  =  m_clip_boundary[0] ;
   }
   else if (code & RIGHT_BIT)
   {
      point[1] += (m_clip_boundary[1] - point[0]) * dy/dx ;
      point[0]  =  m_clip_boundary[1] ;
   }
   else if (code & BOTTOM_BIT)
   {
      point[0] += (m_clip_boundary[2] - point[1]) * dx/dy ;
      point[1]  =  m_clip_boundary[2] ;
   }
   else if (code & TOP_BIT)
   {
      point[0] += (m_clip_boundary[3] - point[1]) * dx/dy ;
      point[1]  =  m_clip_boundary[3] ;
   }
}

//----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
