/**
   \file  Robots/LoBot/control/LoCalibrateLET.C
   \brief This file defines the non-inline member functions of the
   lobot::CalibrateLET class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoCalibrateLET.C $
// $Id: LoCalibrateLET.C 13732 2010-07-29 14:16:35Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoCalibrateLET.H"

#include "Robots/LoBot/tti/LoTTIEstimator.H"
#include "Robots/LoBot/tti/LoSensorModel.H"
#include "Robots/LoBot/lgmd/gabbiani/LoGabbiani.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoRegistry.H"

#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/range.hh"
#include "Robots/LoBot/util/triple.hh"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLU
#include <GL/glu.h>
#endif

#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <sstream>
#include <algorithm>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------------ GLOBALS --------------------------------

// Robolocust uses two sensor models: one for the LOOMING phase of the
// LGMD signal and the other for the BLANKING phase. This variable is
// used to point at one of the sensor models. The one it points to will
// be the one with the "input focus," i.e., '+' and '-' keypresses will
// result in incrementing/decrementing the sigma associated with that
// particular sensor model. The TAB key is used to switch input focus to
// the other sensor model.
SensorModel* g_sensor_model ;

// Every time the user presses '+' or '-' to increment/decrement the
// sigma associated with the "active" sensor model, we will have to
// update the GL texture object used to visualize that sensor model. This
// flag is used to let the rendering function know that it needs to
// re-create the GL texture using the latest sensor model probabilities.
static bool g_update_flag ;

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from calibrate_lgmd_extricate_tti section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_CALIBRATE_LET, key, default_value) ;
}

//-------------------------- INITIALIZATION -----------------------------

CalibrateLET::CalibrateLET()
   : base(clamp(conf("update_delay", 1500), 1000, 5000),
          LOBE_CALIBRATE_LET,
          conf<std::string>("geometry", "0 0 320 320"))
{
   start(LOBE_CALIBRATE_LET) ;
}

// Setup sensor models
void CalibrateLET::pre_run()
{
   g_sensor_model = &TTIEstimator::looming_sensor_model() ;
   TTIEstimator::blanking_sensor_model() ; // force creation of this object
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// This behaviour is actually an interactive means of generating and
// visualizing the causal probabilities used in the Bayesian
// time-to-impact sensor model. Therefore, it relies on keypress events
// to get its work done rather than regular action processing like
// "normal" behaviours.
void CalibrateLET::action(){}

void CalibrateLET::keypress(unsigned char key)
{
   SensorModel* looming  = &TTIEstimator::looming_sensor_model()  ;
   SensorModel* blanking = &TTIEstimator::blanking_sensor_model() ;

   float dsigma = 0 ;
   switch (key)
   {
      case '\t': // toggle LOOMING/BLANKING sensor model
         viz_lock() ;
            if (g_sensor_model == looming)
               g_sensor_model = blanking ;
            else if (g_sensor_model == blanking)
               g_sensor_model = looming ;
            g_update_flag = false ;
         viz_unlock() ;
         return ;

      case 'u': // up
      case 'i': // increment
      case 'k': // go up one line (vi)
      case '+':
         dsigma = Params::dsigma() ;
         break ;

      case 'd': // down/decrement
      case 'j': // go down one line (vi)
      case '-':
         dsigma = -Params::dsigma() ;
         break ;

      default:
         return ;
   }

   g_sensor_model->update(dsigma) ;
   viz_lock() ;
      g_update_flag = true ;
   viz_unlock() ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGLU

namespace {

// Helper class for encapsulating the GL textures used to visualize the
// probabilities contained in the Robolocust sensor models.
class GLTexture {
   const SensorModel* m_sensor_model ;
   range<float> m_prob_range ;
   unsigned int m_name ;
   int m_width, m_height ;
public:
   GLTexture() ;
   void init(const SensorModel*) ;
   int  width()  const {return m_width  ;}
   int  height() const {return m_height ;}
   range<float> prob_range() const {return m_prob_range ;}
   void update() ;
   void render(float left, float right, float bottom, float top) const ;
   void cleanup() ;
} ;

GLTexture::GLTexture()
   : m_sensor_model(0), m_prob_range(0, 1), m_name(0), m_width(0), m_height(0)
{}

void GLTexture::init(const SensorModel* S)
{
   m_sensor_model = S ;
   m_width  = S->column_size() ;
   m_height = S->row_size() ;

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

   update() ;
}

// Quick helper function object to scale sensor model's probabilities so
// that the min probability corresponds to texel value 0 and max
// probability to texel 255.
class scale {
   float min, scale_factor ;
public:
   scale(const range<float>& prob_range) ;
   GLubyte operator()(float p) const {
      return static_cast<GLubyte>((p - min) * scale_factor) ;
   }
} ;

scale::scale(const range<float>& p)
   : min(p.min()), scale_factor(255/p.size())
{}

// Re-create the GL texture using the latest sensor model probabilities
void GLTexture::update()
{
   // Make a local copy of the causal probabilities for updating the GL
   // texture.
   std::vector<float> prob = m_sensor_model->table() ;

   // Find min and max probability values. These are used to scale the
   // sensor model's probabilities to an appropriate number for
   // visualization. Without the scaling, the probabilities might be much
   // too insignificant (e.g., in the order 10^-8 to 10^-3) so that most
   // of the texture will end up being blank.
   m_prob_range = make_range(*(std::min_element(prob.begin(), prob.end())),
                             *(std::max_element(prob.begin(), prob.end()))) ;

   // Scale sensor model's probabilities to [min, max] range to ensure
   // that the texture map shows something discernible rather than being
   // mostly empty/black.
   const int N = prob.size() ;
   GLubyte* texture = new GLubyte[N] ;
   std::transform(prob.begin(), prob.end(), texture, scale(m_prob_range)) ;

   // The time-to-impact versus LGMD spike rate graph is usually viewed
   // with TTI decreasing from left to right (to zero). The texture
   // created above will have TTI increase from left to right. Therefore,
   // we need to reverse its individual rows.
   for (GLubyte* t = texture; t < texture + N; t += m_width)
      std::reverse(t, t + m_width) ;

   // Finally, update GL texture object and free memory holding texture
   // data (because GL will make a copy).
   glBindTexture(GL_TEXTURE_2D, m_name) ;
   glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_width, m_height,
                   GL_LUMINANCE, GL_UNSIGNED_BYTE, texture) ;
   delete[] texture ;
}

// Render the texture representation of the sensor model
void GLTexture::render(float left, float right, float bottom, float top) const
{
   glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT) ;
   glEnable(GL_TEXTURE_2D) ;
   glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE) ;
   glBindTexture(GL_TEXTURE_2D, m_name) ;

   const float S = static_cast<float>(m_width) /next_power_of_two(m_width) ;
   const float T = static_cast<float>(m_height)/next_power_of_two(m_height);
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
void GLTexture::cleanup()
{
   glDeleteTextures(1, &m_name) ;
}

// Instantiate GL textures for the LOOMING and BLANKING phases of the
// sensor model.
static GLTexture looming_texture  ;
static GLTexture blanking_texture ;

} // end of local anonymous namespace encapsulating above helper class

// Quick helper to return a label for the current sigma value used for
// weighting the sensor model probability bins.
static std::string sigma_label(float sigma)
{
   std::ostringstream str ;
   str << "sigma: " << sigma ;
   return str.str() ;
}

// Quick helper to return a label for the current range of probability
// values used for scaling the texels used to represent the sensor
// model's probabilities.
static std::string prob_label(const range<float>& prob_range)
{
   std::ostringstream str ;
   str << "P-range: ["
       << prob_range.min() << ", " << prob_range.max() << ']' ;
   return str.str() ;
}

void CalibrateLET::gl_init()
{
    looming_texture.init(&TTIEstimator:: looming_sensor_model()) ;
   blanking_texture.init(&TTIEstimator::blanking_sensor_model()) ;
}

// This method renders the GL texture used to visualize the sensor
// model's probability values.
void CalibrateLET::render_me()
{
   const SensorModel* looming  = &TTIEstimator::looming_sensor_model()  ;
   const SensorModel* blanking = &TTIEstimator::blanking_sensor_model() ;

   viz_lock() ;
      if (g_update_flag) {
         if (g_sensor_model == looming)
            looming_texture.update() ;
         else if (g_sensor_model == blanking)
            blanking_texture.update() ;
         g_update_flag = false ;
      }
   viz_unlock() ;

   setup_view_volume(0, looming_texture.width(),
                     -(blanking_texture.height() + 1),
                     looming_texture.height() + 1) ;
   looming_texture.render(0, looming_texture.width()  - 1,
                          1, looming_texture.height() - 1) ;
   blanking_texture.render(0, blanking_texture.width() - 1,
                           -blanking_texture.height() + 1, -1) ;

   glColor3f(1, 1, 0) ;
   glBegin(GL_LINES) ;
      glVertex2i(0, 0) ;
      glVertex2i(looming_texture.width(), 0) ;
   glEnd() ;

   glMatrixMode(GL_PROJECTION) ;
   glPopMatrix() ;
   glMatrixMode(GL_MODELVIEW) ;
   glPopMatrix() ;

   text_view_volume() ;
      if (g_sensor_model == looming)
         glColor3f(1, 0, 0) ;
      else
         glColor3f(0.5f, 0.5f, 0.5f) ;
      draw_label(3, 14, looming->name().c_str()) ;
      draw_label(3, 26, sigma_label(looming->sigma()).c_str()) ;
      draw_label(3, 38, prob_label(looming_texture.prob_range()).c_str()) ;

      const int y = m_geometry.height - 26 ;
      if (g_sensor_model == blanking)
         glColor3f(1, 0, 0) ;
      else
         glColor3f(0.5f, 0.5f, 0.5f) ;
      draw_label(3, y, blanking->name().c_str()) ;
      draw_label(3, y + 10, sigma_label(blanking->sigma()).c_str()) ;
      draw_label(3, y + 20, prob_label(blanking_texture.prob_range()).c_str());
   restore_view_volume() ;
}

void CalibrateLET::gl_cleanup()
{
    looming_texture.cleanup() ;
   blanking_texture.cleanup() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

CalibrateLET::~CalibrateLET(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
CalibrateLET::Params::Params()
   : m_dsigma(clamp(conf("dsigma", 0.01f), 0.001f, 100.00f))
{}

// Parameters clean-up
CalibrateLET::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
