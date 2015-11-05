/**
   \file Robots/LoBot/misc/LoTypes.C

   \brief Some frequently used types across different parts of all the
   Lobot/Robolocust related programs.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoTypes.C $
// $Id: LoTypes.C 13037 2010-03-23 01:00:53Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/LoTypes.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- OpenGL COLORS -----------------------------

// Default constructor sets to black
GLColor::GLColor()
{
   m_color[0] = m_color[1] = m_color[2] = 0 ;
}

// Constructor with bytes converts to clamped floats
GLColor::GLColor(unsigned char r, unsigned char g, unsigned char b)
{
   m_color[0] = r/255.0f ;
   m_color[1] = g/255.0f ;
   m_color[2] = b/255.0f ;
}

// Constructor with floats clamps supplied values
GLColor::GLColor(float r, float g, float b)
{
   m_color[0] = clamp(r, 0.0f, 1.0f) ;
   m_color[1] = clamp(g, 0.0f, 1.0f) ;
   m_color[2] = clamp(b, 0.0f, 1.0f) ;
}

// Copy constructor
GLColor::GLColor(const GLColor& C)
{
   m_color[0] = C.m_color[0] ;
   m_color[1] = C.m_color[1] ;
   m_color[2] = C.m_color[2] ;
}

// Assignment
GLColor& GLColor::operator=(const GLColor& C)
{
   if (&C != this) {
      m_color[0] = C.m_color[0] ;
      m_color[1] = C.m_color[1] ;
      m_color[2] = C.m_color[2] ;
   }
   return *this ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
