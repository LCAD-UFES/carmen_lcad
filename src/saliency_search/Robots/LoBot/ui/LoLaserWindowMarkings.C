/**
   \file Robots/LoBot/ui/LoLaserWindowMarkings.C

   \brief This file defines the non-inline member functions of the
   lobot::LaserWindowMarkings class, which is used to draw markers at
   regular intervals to help indicate distance from the laser range
   finder.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLaserWindowMarkings.C $
// $Id: LoLaserWindowMarkings.C 13905 2010-09-09 21:38:44Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case OpenGL is missing...
#ifndef INVT_HAVE_LIBGL

#include "Robots/LoBot/ui/LoLaserWindowMarkings.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

LaserWindowMarkings::LaserWindowMarkings()
{
   throw missing_libs(MISSING_OPENGL) ;
}

LaserWindowMarkings::~LaserWindowMarkings(){}

// Empty API
void LaserWindowMarkings::draw_main_axes() const {}

}

#else // OpenGL available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// DEVNOTE: The different types of distance markings are created with an
// object factory. Usually, we would register all the subfactories
// together in a separate registry module. However, for the laser range
// finder test program, we don't go to the trouble of doing all that as
// there aren't very many such factory-created objects.
//
// Instead, we put each class's subfactory into its own translation unit.
// That is, some markings type implemented in foo.H and foo.C will have
// its subfactory registration static data member defined in foo.C
// instead of some central registry.C.
//
// Since the INVT build system automatically deduces the dependencies for
// each target, we need to ensure that this module's dependencies list
// includes foo.C. Otherwise, when the program is run and the user
// chooses to use the foo markings type, it will crash because the linker
// would not have linked with foo.o/foo.so.
//
// If all the marking types were in a registry.o/so, we could have
// included registry.H where it was needed and ensured proper linkage.
// However, since we decided not go with a central registry in this case,
// we need to force/trick INVT's dependency calculator into including the
// foo and all other marking modules into this module's dependency list.
// That way, the modules depending on this abstract marker class will
// automatically get linked with the modules containing the subclasses
// and corresponding subfactories.
//
// We force the above-mentioned linkage by simply including all the
// headers for the different marking types over here. It's a little ugly
// w.r.t. how a polymorphic factory should work (i.e., the base class
// shouldn't know about its subclasses), but it gets the job done by
// getting the dependencies calculator to perform the necessary magic.
#include "Robots/LoBot/ui/LoLaserWindowGrid.H"
#include "Robots/LoBot/ui/LoLaserWindowRings.H"

// lobot headers
#include "Robots/LoBot/ui/LoLaserWindowMarkings.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/util/LoString.H"

// OpenGL headers
#include <GL/gl.h>

// Standard C++ headers
#include <algorithm>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

// This function returns a triple consisting of the given interval, its
// corresponding color and zoom range.
static LaserWindowMarkings::Marking make_marking(const float& interval)
{
   std::string interval_str = to_string(interval) ;

   // Retrieve color setting for interval (e.g., color_500 for interval 500)
   PixelType C(get_conf<PixelType>("markings",
                                   std::string("color_") + interval_str,
                                   PixelType(32, 32, 32))) ;
   GLColor color(C.red(), C.green(), C.blue()) ;

   // Retrieve zoom setting for interval (e.g., zoom_500 for interval 500)
   std::vector<float> zoom = string_to_vector<float>(
      get_conf<std::string>("markings",
                            std::string("zoom_") + interval_str,
                            "0.05 100")) ;
   LaserWindowMarkings::ZoomRange zoom_range(zoom[0], zoom[1]) ;

   // Package the interval and corresponding settings together
   return make_triple(interval, color, zoom_range) ;
}

bool compare_markings(const LaserWindowMarkings::Marking& a,
                      const LaserWindowMarkings::Marking& b)
{
   return a.first < b.first ;
}

// On instantiation, we retrieve the intervals specified in the markings
// section of the config file and create marking specs corresponding to
// each of these intervals.
LaserWindowMarkings::LaserWindowMarkings()
   : m_canvas(0), m_max(0)
{
   std::vector<float> intervals = string_to_vector<float>(
      get_conf<std::string>("markings", "intervals", "100 500 1000 5000")) ;

   m_markings.reserve(intervals.size()) ;
   std::transform(intervals.begin(), intervals.end(),
                  std::back_inserter(m_markings), make_marking) ;

   std::sort(m_markings.begin(), m_markings.end(), compare_markings) ;
}

//----------------------------- CLEAN-UP --------------------------------

LaserWindowMarkings::~LaserWindowMarkings()
{}

//------------------------------ HELPERS --------------------------------

void LaserWindowMarkings::draw_main_axes() const
{
   if (Params::main_axes_disabled())
      return ;

   glColor3fv(Params::main_axes_color().rgb()) ;
   glBegin(GL_LINES) ;
      glVertex2f(-m_max, 0) ;
      glVertex2f( m_max, 0) ;
      glVertex2f(0, -m_max) ;
      glVertex2f(0,  m_max) ;
   glEnd() ;
}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
LaserWindowMarkings::Params::Params()
{
   PixelType c(get_conf("markings", "main_axes_color", PixelType())) ;
   if (c.red() == 0 && c.green() == 0 && c.blue() == 0)
      m_draw_main_axes = false ;
   else
   {
      m_draw_main_axes = true ;
      m_main_axes_color = GLColor(c.red(), c.green(), c.blue()) ;
   }
}

// Parameters clean-up
LaserWindowMarkings::Params::~Params(){}

// Parameters access
bool LaserWindowMarkings::Params::main_axes_enabled()
{
   return instance().m_draw_main_axes ;
}

const GLColor& LaserWindowMarkings::Params::main_axes_color()
{
   return instance().m_main_axes_color ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_LIBGL

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
