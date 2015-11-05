/**
   \file  Robots/LoBot/slam/LoMap.C
   \brief This file defines the non-inline member functions of the
   lobot::Map class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoMap.C $
// $Id: LoMap.C 13958 2010-09-17 11:45:00Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/slam/LoOccGrid.H"
#include "Robots/LoBot/slam/LoCoords.H"
#include "Robots/LoBot/slam/LoSlamParams.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/triple.hh"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLU
#include <GL/glu.h>
#endif

#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Boost headers
#include <boost/bind.hpp>

// Standard C++ headers
#include <string>
#include <algorithm>
#include <functional>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Quick helper to return settings from "map" section of config file
template<typename T>
inline T conf(const std::string& key, T default_value)
{
   return get_conf<T>("map", key, default_value) ;
}

// Overload for retrieving triples
template<typename T>
inline triple<T, T, T>
conf(const std::string& key, const triple<T, T, T>& default_value)
{
   return get_conf<T>("map", key, default_value) ;
}

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the map.
class MapParams : public singleton<MapParams> {
   /// Generally, when we draw the map, we also draw the robot's current
   /// pose w.r.t. the map. However, in some cases, we may wish to skip
   /// drawing the pose. This setting can be used to do that. By default,
   /// it is true, i.e., the robot pose is drawn.
   bool m_draw_pose ;

   /// In addition to the pose, the map also draws some labels on its top
   /// left corner. This setting can be used to turn those labels off. By
   /// default, they are on.
   bool m_draw_labels ;

   /// To allow users to better make out the map's dimensions and
   /// relative distances between objects, we can draw a grid. This
   /// setting can be used to specify the size of the grid's cells, which
   /// are all square.
   ///
   /// The value of this setting should be a floating point number. Its
   /// units are millimeters. If the grid spacing is a negative number,
   /// the grid will not be drawn. By default, the grid spacing is set to
   /// -1 and the grid is not drawn.
   float m_grid_spacing ;

   /// If the grid is drawn, it will rendered in a light gray color so as
   /// to not interfere visually with the other things on the map. This
   /// setting specifies the gray level to use when rendering the map.
   /// This setting's value is expected to be a unitless number in the
   /// range zero to one. Lower values will result in a darker grid,
   /// higher numbers in a lighter grid.
   float m_grid_color ;

   /// This setting controls whether we want to draw a border around the
   /// map's drawing area within the Robolocust UI. Usually, all
   /// drawables have borders drawn around them so that they are
   /// clearly demarcated from each other. However, in some cases, we may
   /// not want these borders around the map. By default, this flag is
   /// on, i.e., a border is drawn.
   bool m_draw_border ;

   /// This setting allows users to customize the color in which the
   /// map's drawable border is rendered. By default, it is yellow. The
   /// value of this setting should be a triple of integers, each in the
   /// range [0,255].
   GLColor m_border_color ;
   typedef const GLColor& Color ;

   /// Private constructor because this is a singleton.
   MapParams() ;
   friend class singleton<MapParams> ;

public:
   /// Accessing the various parameters
   //@{
   static bool  draw_pose()    {return instance().m_draw_pose    ;}
   static bool  draw_labels()  {return instance().m_draw_labels  ;}
   static bool  draw_border()  {return instance().m_draw_border  ;}
   static Color border_color() {return instance().m_border_color ;}
   static float grid_spacing() {return instance().m_grid_spacing ;}
   static float grid_color()   {return instance().m_grid_color   ;}
   static bool  draw_grid()    {return instance().m_grid_spacing > 0 ;}
   //@}
} ;

// Parameter initialization
MapParams::MapParams()
   : m_draw_pose(conf("draw_pose", true)),
     m_draw_labels(conf("draw_labels", true)),
     m_grid_spacing(conf("grid_spacing", -1.0f)),
     m_grid_color(clamp(conf("grid_color", 0.75f), 0.25f, 0.95f)),
     m_draw_border(conf("draw_border", true)),
     m_border_color(conf<int>("border_color", make_triple(255, 255, 0)))
{
   if (m_grid_spacing > 0)
      m_grid_spacing = clamp(m_grid_spacing, 1.0f, 1000.0f) ;
}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

Map::Map()
   : Drawable("occupancy_grid", SlamParams::map_geometry()),
     m_pose(SlamParams::initial_pose())
{
   const int N = SlamParams::map_width() * SlamParams::map_height() ;
   m_grid.reserve(N) ;
   std::fill_n(std::back_inserter(m_grid), N, 128) ;

   Drawable::m_border = MapParams::draw_border() ;
   Drawable::m_border_color = MapParams::border_color() ;
}

void Map::add_pose_hook(const Map::PoseHook& H)
{
   AutoMutex M(m_pose_hooks_mutex) ;
   m_pose_hooks.push_back(H) ;
}

//---------------------------- MAP UPDATES ------------------------------

// This function converts the log-odds representation of the map held by
// the FastSLAM particle filter into normal probabilities. However, the
// probabilities are expressed as bytes in the range [0,255] rather
// rather than as floats in the range [0,1]. This makes visualization
// using OpenGL particularly straightforward.
//
// Also: the occupancy grid built by FastSLAM uses p = 1 to indicate a
// cell that is occupied and p = 0 for free space. For visualization, we
// prefer to show free space as white and obstacles as black. Therefore,
// we invert the probabilities held by the FastSLAM occupancy grid so
// that empty areas in the map become 255 and blocked areas are 0.
static unsigned char log_odds_to_byte(float log_odds)
{
   return round(255 * (1 - log_odds_to_prob(log_odds))) ;
}

// This method converts the log-odds occupancy grid built by FastSLAM
// into a map that can be visualized easily.
void Map::update(const OccGrid& g)
{
   AutoMutex M(m_grid_mutex) ;
   std::transform(g.begin(), g.end(), m_grid.begin(), log_odds_to_byte) ;
}

// Quick helper to trigger a pose hook
static void trigger_pose_hook(const Map::PoseHook& H, const Pose& p)
{
   H.first(p, H.second) ;
}

// This method records the latest pose returned by FastSLAM
void Map::update(const Pose& p)
{
   // First, record the pose.
   //
   // DEVNOTE: We need to encapsulate this block of code within a pair of
   // braces to ensure that the pose write lock is released once it is no
   // longer required (we do not need to hold on it for the entire
   // duration of this function).
   {
      AutoWriteLock L(m_pose_lock) ;
      m_pose = p ;
   }

   // Next, trigger the pose update hooks.
   //
   // DEVNOTE: We don't need the additional braces here because the pose
   // hooks auto mutex will go out of scope right after the hooks are
   // triggered, thereby releasing the mutex.
   AutoMutex M(m_pose_hooks_mutex) ;
   std::for_each(m_pose_hooks.begin(), m_pose_hooks.end(),
                 boost::bind(trigger_pose_hook, _1, p)) ;
}

//---------------------------- MAP ACCESS -------------------------------

Pose Map::current_pose() const
{
   AutoReadLock L(m_pose_lock) ;
   return m_pose ;
}

// Extract square portion from map centered at robot's current position
std::vector<unsigned char> Map::submap(int w) const
{
   std::vector<unsigned char> M ;
   M.reserve(sqr(w * 2 + 1)) ;

   int X, Y ;
   Pose p = current_pose() ;
   Coords::to_grid(p.x(), p.y(), &X, &Y) ;

   const int W = SlamParams::map_width()  ;
   const int H = SlamParams::map_height() ;

   AutoMutex mutex(m_grid_mutex) ;
   int k = (Y - w) * W + (X - w) ;
   for (int y = Y - w; y <= Y + w; ++y, k += W - 2*w - 1)
   for (int x = X - w; x <= X + w; ++x, ++k)
   {
      if (x < 0 || x >= W || y < 0 || y >= H)
         M.push_back(0) ; // areas outside map are considered occupied
      else
         M.push_back(m_grid[k]) ;
   }

   return M ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGLU

namespace {

// Helper class for encapsulating the GL textures used to visualize the
// the Robolocust map.
class MapTexture {
   unsigned int m_name ;
   int m_width, m_height ;
public:
   MapTexture() ;
   void init(int width, int height) ;
   int  width()  const {return m_width  ;}
   int  height() const {return m_height ;}
   void update(const std::vector<unsigned char>&) ;
   void render(float left, float right, float bottom, float top) const ;
   void cleanup() ;
} ;

MapTexture::MapTexture()
   : m_name(0), m_width(0), m_height(0)
{}

void MapTexture::init(int width, int height)
{
   m_width  = width ;
   m_height = height ;

   glGenTextures(1, &m_name) ;

   glBindTexture(GL_TEXTURE_2D, m_name) ;
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1) ;
   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) ;
   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) ;
   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST) ;
   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST) ;

   const int W = next_power_of_two(m_width)  ;
   const int H = next_power_of_two(m_height) ;
   const int N = W * H ;
   GLubyte* texture = new GLubyte[N] ;
   std::fill_n(texture, N, GLubyte(0)) ;
   glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, W, H, 0,
                GL_LUMINANCE, GL_UNSIGNED_BYTE, texture) ;
   delete[] texture ;
}

// Re-create the GL texture using the latest obstacle map entries
void MapTexture::update(const std::vector<unsigned char>& grid)
{
   typedef std::vector<unsigned char> Texture ;
   std::vector<unsigned char> texture(grid.size()) ;

   // Reverse the grid row-by-row to create GL texture (because GL will
   // draw the texture starting at the bottom, which, in our coordinate
   // system, will look flipped).
   Texture::iterator dst = texture.end() - m_width ;
   Texture::const_iterator src = grid.begin() ;
   for (; src != grid.end(); src += m_width, dst -= m_width)
      std::copy(src, src + m_width, dst) ;

   glBindTexture(GL_TEXTURE_2D, m_name) ;
   glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_width, m_height,
                   GL_LUMINANCE, GL_UNSIGNED_BYTE, &texture[0]) ;
}

// Render the texture representation of the obstacle map
void MapTexture::render(float left, float right, float bottom, float top) const
{
   glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT) ;
   glEnable(GL_TEXTURE_2D) ;
   glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE) ;
   glBindTexture(GL_TEXTURE_2D, m_name) ;

   const float S = static_cast<float>(m_width)/next_power_of_two(m_width) ;
   const float T = static_cast<float>(m_height)/next_power_of_two(m_height) ;
   glBegin(GL_QUADS) ;
      glTexCoord2i(0, 0) ;
      glVertex2f(left, bottom) ;

      glTexCoord2f(S, 0) ;
      glVertex2f(right, bottom) ;

      glTexCoord2f(S, T) ;
      glVertex2f(right, top) ;

      glTexCoord2f(0, T) ;
      glVertex2f(left, top) ;
   glEnd() ;

   glDisable(GL_TEXTURE_2D) ;
   glPopAttrib() ;
}

// Clean-up GL texture object
void MapTexture::cleanup()
{
   glDeleteTextures(1, &m_name) ;
}

// Helper function for overlaying a spacing grid on the map
void draw_grid(float L, float R, float B, float T)
{
   glMatrixMode(GL_PROJECTION) ;
   glLoadIdentity() ;
   gluOrtho2D(T, B, L, R) ;

   glMatrixMode(GL_MODELVIEW) ;
   glLoadIdentity() ;

   const float C = MapParams::grid_color() ;
   const float S = MapParams::grid_spacing() ;

   glPushAttrib(GL_CURRENT_BIT) ;
   glColor3f(C, C, C) ;
   glBegin(GL_LINES) ;
      for (float x = T - S; x > B; x -= S) {
         glVertex2f(x, L) ;
         glVertex2f(x, R) ;
      }
      for (float y = L + S; y < R; y += S) {
         glVertex2f(T, y) ;
         glVertex2f(B, y) ;
      }
   glEnd() ;
   glPopAttrib() ;
}

// Helper function for rendering the robot's current pose
void draw_pose(const Pose& P, const Drawable::Geometry& G, const float ext[])
{
   float x = (P.y() - ext[3])/(ext[2] - ext[3]) * (G.width  - 1) ;
   float y = (P.x() - ext[0])/(ext[1] - ext[0]) * (G.height - 1) ;

   glMatrixMode(GL_PROJECTION) ;
   glLoadIdentity() ;
   gluOrtho2D(0, G.width - 1, 0, G.height - 1) ;

   glMatrixMode(GL_MODELVIEW) ;
   glLoadIdentity() ;
   glTranslatef(x, y, 0) ;
   glRotatef(90 + P.heading(), 0, 0, 1) ;

   glPushAttrib(GL_CURRENT_BIT) ;
   glColor3f(1, 0, 0) ;
   glBegin(GL_TRIANGLES) ;
      glVertex2i(-10, 0) ;
      glVertex2i(  0, 0) ;
      glVertex2f(-17.5f, 5.0f) ;

      glVertex2i(  0, 0) ;
      glVertex2i(-10, 0) ;
      glVertex2f(-17.5f, -5.0f) ;
   glEnd() ;
   glPopAttrib() ;
}

// Quick helper to return a label for the current pose
static std::string pose_label(const Pose& P)
{
   std::ostringstream str ;
   str << "Pose: ("
       << round(P.x()) << ", "
       << round(P.y()) << ", "
       << round((P.t() < 180) ? P.t() : P.t() - 360) << ')' ;
   return str.str() ;
}

// Instantiate a GL texture for the map
MapTexture map_texture ;

} // end of local anonymous namespace encapsulating above helper class

void Map::gl_init()
{
   map_texture.init(SlamParams::map_width(), SlamParams::map_height()) ;
}

// This method renders the GL texture used to visualize the map and draws
// the robot's current pose as well.
void Map::render_me()
{
   // First, render the occupancy grid
   setup_view_volume(0, 1, 0, 1) ;
   glRotatef(90, 0, 0, 1) ;
   glTranslatef(0, -1, 0) ;
   m_grid_mutex.acquire() ;
      std::vector<unsigned char> grid = m_grid ;
   m_grid_mutex.release() ;
   map_texture.update(grid) ;
   map_texture.render(0, 1, 0, 1) ;

   // Next, draw a grid to help user make out map dimensions
   float ext[4] ; // map extents
   SlamParams::map_extents(ext) ;
   if (MapParams::draw_grid())
      draw_grid(ext[0], ext[1], ext[2], ext[3]) ;

   // Then, render the current pose
   Pose P = current_pose() ;
   if (MapParams::draw_pose())
      draw_pose(P, m_geometry, ext) ;

   // Finally, some labels
   if (MapParams::draw_labels()) {
      restore_view_volume() ;
      text_view_volume() ;
      glColor3f(1, 0, 0) ;
      draw_label(3, 12, "Map") ;
      if (MapParams::draw_pose())
         draw_label(3, 24, pose_label(P).c_str()) ;
   }
   restore_view_volume() ;
}

void Map::gl_cleanup()
{
   map_texture.cleanup() ;
}

#endif

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
