/**
   \file Robots/LoBot/ui/LoLocustViz.C

   This file defines the non-inline member functions of the
   lobot::LocustViz class used for visualizing the LGMD spike rates of
   all the (virtual) locusts.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoLocustViz.C $
// $Id: LoLocustViz.C 13811 2010-08-21 02:00:08Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLocustViz.H"

#include "Robots/LoBot/lgmd/LocustModel.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/triple.hh"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Boost headers
#include <boost/bind.hpp>

// Standard C++ headers
#include <algorithm>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from locust_viz section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("locust_viz", key, default_value) ;
}

// Overload for retrieving triples
template<typename T>
inline triple<T, T, T>
conf(const std::string& key, const triple<T, T, T>& default_value)
{
   return get_conf<T>("locust_viz", key, default_value) ;
}

// This local helper class encapsulates settings that can be used to
// tweak various aspects of the locust spike rate visualizer.
class Params : public singleton<Params> {
   /// The spike rate visualizer shows each LGMD's current value as a bar
   /// starting at the center of the visualizer window and going out
   /// along the direction vector of that LGMD. The length of this bar
   /// denotes magnitude of the spiking activity for the locust looking
   /// in the bar's direction.
   ///
   /// Thus, for the entire array of locusts, the visualizer will
   /// resemble the depiction of a range sensor such as the laser range
   /// finder with the individual "rays" denoting the "range" measurement
   /// (in this case spike rate) along each of the directions "scanned"
   /// by this sensor.
   ///
   /// This setting specifies the width (in pixels) of the spike rate
   /// rays/bars.
   float m_ray_width ;

   /// This setting specifies the color of the LGMD "range sensor" rays.
   /// It should be a triple of integers, each in the range [0,255].
   GLColor m_ray_color ;

   /// To help gauge each ray's associated numeric value, we draw a
   /// series of concentric rings. This setting specifies the increments
   /// in the radii between two consecutive rings. Its unit is Hertz. Its
   /// value should be a reasonable number between the min and max spike
   /// rate range specified for the locust model in effect.
   ///
   /// For example, if the spike range is 0 to 800 and this increment is
   /// given as 100, the visualizer will setup its internal coordinate
   /// system to go from 0 to 800 in each direction and draw 8 circles,
   /// starting with a radius of 100 units and incrementing thereafter by
   /// 100 units all the way up to 800. By seeing where each ray's end
   /// point falls, the user can get a rough idea of the spike rate
   /// associated with that ray.
   float m_ring_incr ;

   /// This setting specifies the color of concentric rings described
   /// above. It should be a triple of integers, each in the range
   /// [0,255].
   GLColor m_ring_color ;

   /// The location and size (inside the Robolocust main window) of the
   /// locust spike rate visualizer's drawing area.
   Drawable::Geometry m_geometry ;

   /// Private constructor because this is a singleton.
   Params() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters.
   //@{
   static float ray_width() {return instance().m_ray_width ;}
   static float ring_incr() {return instance().m_ring_incr ;}
   static const float* ray_color()  {return instance().m_ray_color.rgb()  ;}
   static const float* ring_color() {return instance().m_ring_color.rgb() ;}
   static Drawable::Geometry geometry() {return instance().m_geometry ;}
   //@}
} ;

// Parameters initialization
Params::Params()
   : m_ray_width(clamp(conf("ray_width", 3.0f), 1.0f, 10.0f)),
     m_ray_color(conf<int>("ray_color", make_triple(96, 96, 96))),
     m_ring_incr(clamp(conf("ring_incr", 100.0f), 1.0f, 1000.0f)),
     m_ring_color(conf<int>("ring_color", make_triple(255, 0, 0))),
     m_geometry(conf<std::string>("geometry", "0 0 10 10"))
{}

} // end of local anonymous namespace encapsulating above helper class

//-------------------------- INITIALIZATION -----------------------------

LocustViz::LocustViz(const std::vector<LocustModel*>& locusts)
   : Drawable("locust_viz", Params::geometry()),
     m_locusts(locusts), m_spike_max(1), m_display_list(0)
{
   if (m_locusts.empty()) // what the hell are we visualizing?
      throw misc_error(NOT_ENOUGH_LOCUSTS) ;

   // Compute the unit vectors associated with the radial directions in
   // which each locust is looking.
   m_directions.reserve(m_locusts.size()) ;
   std::transform(m_locusts.begin(), m_locusts.end(),
                  std::back_inserter(m_directions),
                  boost::bind(unit_vector,
                              boost::bind(&LocustModel::direction, _1))) ;

   // Record the max spike rate so we can setup the visualizer's
   // coordinate properly.
   m_spike_max = m_locusts[0]->get_range().max() ;
}

// Once we know the max spike rate, we know all that we need to know to
// be able to draw the concentric rings used to help the user gauge the
// numeric value associated with each spike rate ray. Therefore, we can
// stuff that bit of rendering code in a display list...
void LocustViz::gl_init()
{
   // We use two display lists for the concentric rings: one for drawing
   // a unit circle and another for scaling this circle and drawing the
   // required number of rings.
   m_display_list = glGenLists(2) ;

   // First, setup the display list for a unit circle
   glNewList(m_display_list + 1, GL_COMPILE) ;
   glBegin(GL_LINE_LOOP) ;
      for (int angle = 0; angle < 360; ++angle)
         glVertex2f(cos(angle), sin(angle)) ;
   glEnd() ;
   glEndList() ;

   // Then, scale the unit circle display list to draw the required
   // number of concentric rings.
   const float R = Params::ring_incr() ;
   glNewList(m_display_list, GL_COMPILE) ;
   for (float radius = R; radius <= m_spike_max; radius += R) {
      glLoadIdentity() ;
      glScalef(radius, radius, 1) ;
      glCallList(m_display_list + 1) ;
   }
   glEndList() ;
}

//----------------------------- RENDERING -------------------------------

#ifdef INVT_HAVE_LIBGL

// Scale a locust's direction vector by its current spike rate
static Vector compute_spike_vector(const LocustModel* L, const Vector& v)
{
   return v * L->get_lgmd() ;
}

// Draw the "ray" associated with a spike rate vector
static void render_spike(const Vector& spike)
{
   glVertex2i(0, 0) ;
   glVertex2f(spike.i, spike.j) ;
}

// Draw the latest set of spike rates so as to make the LGMD array look
// like a range sensor (in fact, this imitates the LRF visualizer's "ray"
// style; see ui/LoLaserViz.C).
void LocustViz::render_me()
{
   // We use a vector to represent each locust's spike rate "ray"
   std::vector<Vector> spikes ;
   spikes.reserve(m_locusts.size()) ;

   // Scale each locust's direction vector by its current spike rate to
   // obtain its spike rate "ray."
   UpdateLock::begin_read() ;
      std::transform(m_locusts.begin(), m_locusts.end(), m_directions.begin(),
                     std::back_inserter(spikes), compute_spike_vector) ;
   UpdateLock::end_read() ;

   setup_view_volume(-m_spike_max, m_spike_max, -m_spike_max, m_spike_max) ;
   glPushAttrib(GL_LINE_BIT | GL_CURRENT_BIT) ;
      // Draw the concentric rings to help user gauge the numeric value
      // associated with each spike rate ray.
      glColor3fv(Params::ring_color()) ;
      glCallList(m_display_list) ;

      // The above display list messes with the modelview matrix. So we
      // reset it and apply a rotation so that the visualizer's notion of
      // "up" (positive x-direction) corresponds with our notion of "up"
      // (positive y-direction).
      glLoadIdentity() ;
      glRotatef(90, 0, 0, 1) ;

      // Draw the spike rate rays
      glLineWidth(Params::ray_width()) ;
      glColor3fv(Params::ray_color()) ;
      glBegin(GL_LINES) ;
         std::for_each(spikes.begin(), spikes.end(), render_spike) ;
      glEnd() ;
   glPopAttrib() ;

   // Label to let user know what this drawable is visualizing
   restore_view_volume() ;
   text_view_volume() ;
      glColor3f(0, 1, 1) ;
      draw_label(3, 12, "LGMD Spikes") ;
   restore_view_volume() ;
}

#endif // INVT_HAVE_LIBGL

//----------------------------- CLEAN-UP --------------------------------

void LocustViz::gl_cleanup()
{
   glDeleteLists(m_display_list, 2) ;
}

LocustViz::~LocustViz(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
