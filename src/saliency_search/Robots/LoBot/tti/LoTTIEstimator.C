/**
   \file  Robots/LoBot/tti/LoTTIEstimator.C
   \brief This file defines the non-inline member functions and static
   data members of the lobot::TTIEstimator class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/tti/LoTTIEstimator.C $
// $Id: LoTTIEstimator.C 14018 2010-09-23 07:10:34Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/tti/LoTTIEstimator.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoString.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/singleton.hh"
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
#include <iomanip>
#include <sstream>
#include <numeric>
#include <algorithm>
#include <functional>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from tti_estimator section of config file
template<typename T>
static inline T conf(const std::string& key, T default_value)
{
   return get_conf<T>("tti_estimator", key, default_value) ;
}

// Overload for retrieving ranges
template<typename T>
static inline range<T> conf(const std::string& key, const range<T>& defval)
{
   return get_conf<T>("tti_estimator", key, defval) ;
}

// To minimize clutter in the LGMD drawable, we render either the TTI
// belief or the actual and predicted times-to-impact or the actual and
// predicted distances. The user picks what should be rendered. This
// enumeration simply names the different possibilities.
enum RenderMode {
   RENDER_OFF,
   RENDER_BELIEF,
   RENDER_TTI,
   RENDER_DISTANCE,
} ;

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the Bayesian time-to-impact estimation.
class TTIParams : public singleton<TTIParams> {
   /// The LGMD spike rate is a function of an approaching object's
   /// time-to-impact. When the object is far away, the LGMD's spike
   /// rate will be fairly low. As it approaches, the LGMD starts
   /// firing very rapidly. Shortly before impact, the LGMD firing
   /// rate reaches a peak and then drops off sharply until impact.
   ///
   /// The peak described above "partitions" the LGMD firing rate vs.
   /// time-to-impact curve into two distinct "phases." We refer to
   /// the first phase, wherein the curve rises to its peak, as
   /// LOOMING because the object is looming large in the LGMD's field
   /// of view. The second phase, we call BLANKING because feedforward
   /// inhibition kicks in after the peak to shutdown the LGMD right
   /// before impact.
   ///
   /// To recognize the transition from LOOMING to BLANKING, we
   /// monitor the second derivative of the LGMD signal to find the
   /// inflection point where the curve becomes concave down (thus
   /// indicating a local maximum near the signal's peak). Since the
   /// derivatives of a signal can be quite noisy, it would be best to
   /// filter the second derivative.
   ///
   /// This setting specifies the size of the low-pass filter used to
   /// filter the second derivative of the LGMD input signal. For
   /// example, if this value is 10, then the previous ten readings
   /// will be used to smooth the second derivative of the input
   /// signal. To disable this filtering (not recommended), use a
   /// filter size of one.
   int m_sder_filter_size ;

   /// As mentioned above, the LGMD signal is partitioned into two
   /// phases, viz., LOOMING and BLANKING. We recognize transitions
   /// from LOOMING to BLANKING using the second derivative of the
   /// LGMD input signal and a spike rate threshold for the rising
   /// portion of the TTI-LGMD curve.
   ///
   /// To recognize the transition from BLANKING back to LOOMING, we
   /// wait for the LGMD signal to fall below some empirically
   /// determined threshold firing rate rather than relying on
   /// higher-order derivatives of the input signal.
   ///
   /// These two settings specify the above-mentioned firing rate
   /// thresholds for recognizing transitions from LOOMING to BLANKING
   /// and then back to LOOMING.
   float m_rising_threshold, m_falling_threshold ;

   /// Once we have a time-to-impact estimate for each locust, we can
   /// combine that estimate with the robot's current motion to
   /// determine the distances to obstacles in each locust's
   /// direction, thus, converting a vision sensor to a range sensor.
   ///
   /// As mentioned earlier, the time-to-impact estimates are made on
   /// the basis of a Bayesian state estimation model that works by
   /// continually updating a probability distribution that specifies
   /// the likelihood of each TTI value. The TTI with the maximum
   /// likelihood is used to compute the corresponding distance.
   ///
   /// However, we only want to use TTI values when their likelihoods
   /// are significant. That is, if the maximum likelihood in the
   /// entire probability distribution is really low (e.g., 1%), then
   /// it might not be a good thing to use that TTI estimate to try
   /// and determine the corresponding distance. Instead, we want to
   /// wait until the confidence level of the most likely
   /// time-to-impact is a reasonably high value before using it.
   ///
   /// This setting specifies the minimum confidence we expect to see
   /// before a TTI estimate will be used to generate distance
   /// readings. It should be a number between zero and one.
   float m_confidence_threshold ;

   /// This setting specifies the min and max bounds for the distance
   /// "readings" computed by the Bayesian time-to-impact estimator.
   /// The units for this setting is mm.
   range<float> m_distance_range ;

   /// There are three visualization modes for the time-to-impact
   /// estimator. In the first mode, the estimator renders its current
   /// belief, i.e., the posterior probability distribution computed
   /// in each iteration of the Bayesian state update loop.
   ///
   /// In the second mode, the estimator shows the history of recent
   /// time-to-impact values predicted by the Bayesian state update.
   /// To help gauge how well the Bayes filter is doing, the estimator
   /// will also render the actual TTI values.
   ///
   /// In the third mode, the estimator shows the history of recent
   /// distance "readings" computed using the predicted
   /// times-to-impact and the actual distances to approaching
   /// obstacles (as reported by the laser range finder).
   ///
   /// This setting specifies which of the above render modes to use.
   /// The value of this setting should be a string. As described
   /// above, the currently supported modes are:
   ///        off ==> don't perform any rendering
   ///        bel ==> render current belief
   ///        tti ==> render time-to-impact histories
   ///        dis ==> render distance histories
   RenderMode m_render_mode ;

   /// The labels used to show the current TTI and other relevant info
   /// can clutter the visualization quite a bit (especially when the
   /// locust model drawables are small). This flag can be used to
   /// turn these labels on/off. By default, the labels are off.
   bool m_render_labels ;

   /// Private constructor because this is a singleton.
   TTIParams() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class singleton<TTIParams> ;

public:
   /// Accessing the various parameters.
   //@{
   static int sder_filter_size() {
      return instance().m_sder_filter_size ;
   }
   static float rising_threshold() {
      return instance().m_rising_threshold ;
   }
   static float falling_threshold() {
      return instance().m_falling_threshold ;
   }
   static float confidence_threshold() {
      return instance().m_confidence_threshold ;
   }
   static const range<float>& distance_range() {
      return instance().m_distance_range ;
   }
   static RenderMode render_mode() {return instance().m_render_mode   ;}
   static bool render_labels()     {return instance().m_render_labels ;}
   //@}
} ;

// Parameters initialization
TTIParams::TTIParams()
   : m_sder_filter_size(clamp(conf("sder_filter_size", 5), 1, 100)),
     m_rising_threshold(clamp(conf("rising_threshold", 700.0f),
                              200.0f, 1000.0f)),
     m_falling_threshold(clamp(conf("falling_threshold", 100.0f),
                               10.0f, 250.0f)),
     m_confidence_threshold(clamp(conf("confidence_threshold", 0.3f),
                                  0.01f, 0.9f)),
     m_distance_range(clamp(conf("distance_range", make_range(50.0f, 5000.0f)),
                            make_range(10.0f, 10000.0f))),
     m_render_mode(RENDER_OFF),
     m_render_labels(conf("render_labels", false))
{
   const std::string render_mode =
      downstring(conf<std::string>("render_mode", "off")) ;
   if (render_mode == "bel")
      m_render_mode = RENDER_BELIEF ;
   else if (render_mode == "tti")
      m_render_mode = RENDER_TTI ;
   else if (render_mode == "dis")
      m_render_mode = RENDER_DISTANCE ;
}

// Shortcut
typedef TTIParams Params ;

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

SensorModel& TTIEstimator::looming_sensor_model()
{
   static SensorModel S("looming") ;
   return S ;
}

SensorModel& TTIEstimator::blanking_sensor_model()
{
   static SensorModel S("blanking") ;
   return S ;
}

// When a Bayesian time-to-impact estimator is created, we initialize the
// belief to a uniform probability distribution (i.e., we use an
// uninformed prior).
TTIEstimator::TTIEstimator(const LocustModel* L)
   : Drawable(L->name(), L->geometry()),
     m_locust(L),
     m_tti(-1), m_confidence(0),
     m_direction(cos(L->direction()), sin(L->direction())),
     m_sder(Params::sder_filter_size()),
     m_distance(-1),
     m_sensor_model(& looming_sensor_model())
{
   m_lgmd[0] = m_lgmd[1] = 0 ;
   m_fder[0] = m_fder[1] = 0 ;

   const int N = m_sensor_model->column_size() ;
   m_belief.reserve(N) ;
   std::fill_n(std::back_inserter(m_belief), N, 1.0f/N) ;

   RenderMode render_mode = Params::render_mode() ;
   if (render_mode != RENDER_OFF)
   {
      if (render_mode == RENDER_TTI || render_mode == RENDER_DISTANCE) {
         m_actual.resize(m_geometry.width) ;
         m_predicted.resize(m_geometry.width) ;
      }
      (const_cast<LocustModel*>(L))->add_hook(
         RenderHook(render_hook, reinterpret_cast<unsigned long>(this))) ;
   }
}

//--------------------------- STATE MACHINE -----------------------------

// Check which phase of the LGMD signal the current firing rate indicates
TTIEstimator::LGMDPhase TTIEstimator::lgmd_phase() const
{
   if (m_sensor_model == & looming_sensor_model())
      return LOOMING ;
   if (m_sensor_model == & blanking_sensor_model())
      return BLANKING ;
   throw misc_error(LOGIC_ERROR) ;
}

// Switch the sensor model to point to the correct likelihood profile
// (based on LGMD firing rate phase as determined in update method).
void TTIEstimator::sensor_model(const SensorModel* S)
{
   viz_lock() ;
      m_sensor_model = S ;
   viz_unlock() ;
}

//------------------- BAYES FILTER UPDATE EQUATIONS ---------------------

void TTIEstimator::copy_lgmd()
{
   // The LGMD signal itself
   m_lgmd[1] = m_lgmd[0] ;
   m_lgmd[0] = m_locust->get_lgmd() ;

   // First derivative of LGMD signal
   m_fder[1] = m_fder[0] ;
   m_fder[0] = m_lgmd[0] - m_lgmd[1] ;

   // Second derivative of LGMD signal (filtered using weighted moving average)
   m_sder.add(m_fder[0] - m_fder[1]) ;

   // Also copy actual time-to-impact or distance so we can later compare
   // with the predicted values.
   viz_lock() ;
      switch (Params::render_mode())
      {
         case RENDER_TTI:
            m_actual.pop_front() ;
            m_actual.push_back(m_locust->tti()) ;
            break ;
         case RENDER_DISTANCE:
            m_actual.pop_front() ;
            m_actual.push_back(m_locust->distance()) ;
            break ;
         default:
            break ;
      }
   viz_unlock() ;
}

// Helper function object to add a uniform distribution to the elements
// of an existing probability distribution.
//
// DEVNOTE: Since both the existing and uniform distributions sum to
// unity, the sum of the combined distribution will be two. Rather than
// perform another normalization step after the addition of the two
// distributions, this function object simply divides each probability it
// computes by two and returns the (already) normalized final result.
// Thus, clients don't have to go through another normalization step
// after applying this function to an existing probability distribution.
class add_uniform_distribution {
   float uniform_probability ;
public:
   add_uniform_distribution(float uniform_probability) ;
   float operator()(float p) const {
      return (p + uniform_probability)/2 ;
   }
} ;

add_uniform_distribution::add_uniform_distribution(float p)
   : uniform_probability(p)
{}

// Recursive Bayesian update to get new belief, i.e., posterior
// probability distribution, using current belief as the prior and the
// latest sensor (i.e., LGMD) value.
//
// DEVNOTE: The client behaviour *must* call TTIEstimator::copy_lgmd()
// before invoking this method.
void TTIEstimator::update()
{
   // Use the correct sensor model for the current phase of the LGMD
   // input signal.
   switch (lgmd_phase())
   {
      case LOOMING:
         if (m_lgmd[0] > Params::rising_threshold() && m_sder.value() < 0)
            sensor_model(& blanking_sensor_model()) ;
         break ;
      case BLANKING:
         if (m_lgmd[0] < Params::falling_threshold()) // blanking over
            sensor_model(& looming_sensor_model()) ;
         break ;
   }

   // Multiply the individual probabilities of the current belief with
   // the corresponding probabilities in the appropriate column vector of
   // the sensor model.
   Belief tmp(m_belief.size(), 0.0f) ;
   std::transform(m_belief.begin(), m_belief.end(),
                  m_sensor_model->column_vector(m_lgmd[0]).begin(),
                  tmp.begin(), std::multiplies<float>()) ;

   // Normalize the intermediate belief obtained above. If the normalizer
   // is out-of-whack, reinit the belief to a uniform probability
   // distribution.
   float normalizer = std::accumulate(tmp.begin(), tmp.end(), 0.0f) ;
   if (normalizer <= 0) // something weird going on!
      std::fill(tmp.begin(), tmp.end(), 1.0f/tmp.size()) ;
   else
      std::transform(tmp.begin(), tmp.end(), tmp.begin(),
                     std::bind2nd(std::multiplies<float>(), 1/normalizer)) ;

   // Add a uniform distribution to the above result so as to model
   // random noise that diffuses the belief. If we don't do this, it
   // often happens that the belief reaches 100% for some state (0% for
   // all others) and then gets stuck in that state.
   //
   // In essence, this step ensures that we don't trust the sensor model
   // completely. After all, no model can be perfect...
   viz_lock() ;
      std::transform(tmp.begin(), tmp.end(), m_belief.begin(),
                     add_uniform_distribution(1.0f/tmp.size())) ;

      Belief::const_iterator max = std::max_element(m_belief.begin(),
                                                    m_belief.end()) ;
      const float min  = m_sensor_model->row_min() ;
      const float step = m_sensor_model->row_step();
      m_tti = min + (max - m_belief.begin() + 1) * step ;
      m_confidence = *max ;
   viz_unlock() ;
}

// Determine distance to obstacle in locust's direction by projecting
// robot's current velocity vector onto locust's direction vector and
// using the TTI estimate with max belief (subject to that estimate's
// confidence being greater than the configured minimum).
void TTIEstimator::compute_distance(const Vector& velocity)
{
   float speed    = magnitude(dot(m_direction, velocity) * m_direction) ;
   float distance = -1 ;
   float time     = -1 ;
   if (m_confidence >= Params::confidence_threshold())
   {
      time = m_tti ;
      if (! is_zero(speed))
         distance = clamp(speed * time * 1000, Params::distance_range()) ;
   }

   viz_lock() ;
      m_distance = distance ;
      switch (Params::render_mode())
      {
         case RENDER_TTI:
            m_predicted.pop_front() ;
            m_predicted.push_back(time) ;
            break ;
         case RENDER_DISTANCE:
            m_predicted.pop_front() ;
            m_predicted.push_back(distance) ;
            break ;
         default:
            break ;
      }
   viz_unlock() ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGLU

// Quick helper to return a label for the current LGMD phase
static std::string phase_label(int lgmd_phase)
{
   std::ostringstream str ;
   switch (lgmd_phase)
   {
      case 0: // LOOMING
         str << "Loom" ;
         break ;
      case 1: // BLANKING
         str << "Blank" ;
         break ;
      default: // should never happen!
         str << "???" ;
         break ;
   }
   return str.str() ;
}

// Helper function to convert a TTI value to an appropriate label
static std::string tti_label(float tti)
{
   using namespace std ;

   std::ostringstream str ;
   if (tti < 0)
      str << "???" ;
   else
      str << fixed << setprecision(1) << tti << 's' ;
   return str.str() ;
}

// Helper function to convert current belief's peak to appropriate label
// showing TTI and confidence level.
static std::string tti_label(float tti, float confidence)
{
   using namespace std ;

   std::ostringstream str ;
   str << fixed << setprecision(1) << tti << "s ("
       << fixed << setprecision(1) << (confidence * 100) << "%)" ;
   return str.str() ;
}

// Quick helper to return a label for current distance estimate based on
// TTI computation.
static std::string distance_label(int distance)
{
   std::ostringstream str ;
   if (distance < 0)
      str << "???" ;
   else
      str << distance << " mm" ;
   return str.str() ;
}

// Helper function to show error in TTI estimate
static std::string error_label(float actual, float predicted)
{
   std::ostringstream str ;
   str << "Err: " ;
   if (actual < 0 || predicted < 0) // bogus TTI values
      str << '?' ;
   else
      str << round(100 * (predicted - actual)/actual) << '%' ;
   return str.str() ;
}

// This function renders the TTI estimator's current state on the
// lobot::LocustModel drawable so that we can see the Bayesian state
// estimation superimposed on the LGMD spike trains.
void TTIEstimator::render_hook(unsigned long client_data)
{
   TTIEstimator* me = reinterpret_cast<TTIEstimator*>(client_data) ;
   switch (Params::render_mode())
   {
      case RENDER_BELIEF:
         me->render_belief() ;
         break ;
      case RENDER_TTI:
         me->render_tti() ;
         break ;
      case RENDER_DISTANCE:
         me->render_distance() ;
         break ;
      default:
         break ;
   }
}

// This function renders the probability distribution showing the
// time-to-impact likelihoods for each TTI estimator.
void TTIEstimator::render_belief()
{
   // Make local copy of current TTI belief so that behaviour's thread
   // isn't held up waiting for visualization to complete.
   viz_lock() ;
      Belief B = m_belief ;
      const SensorModel* S = m_sensor_model ;
      LGMDPhase P = lgmd_phase() ;
      int D = round(m_distance) ;
   viz_unlock() ;

   // Since TTI belief is overlaid on LGMD spikes visualization, we need
   // to reset the view volume before rendering.
   setup_view_volume(S->row_min(), S->row_max(), 0, 1) ;

   // Render TTI belief
   glColor3f(1, 0, 0) ;
   glBegin(GL_LINE_STRIP) ;
      glVertex2f(S->row_min(), 0) ;
      float x = S->row_min() + S->row_step() ;
      for (unsigned int i = 0; i < B.size(); ++i, x += S->row_step())
         glVertex2f(x, B[i]);
      glVertex2f(S->row_max(), 0) ;
   glEnd() ;
   restore_view_volume() ;

   // Label current LGMD phase being used for Bayes filter plus the
   // current belief and distance computation.
   if (Params::render_labels())
   {
      Belief::const_iterator max = std::max_element(B.begin(), B.end()) ;
      float tti = S->row_min() + (max - B.begin() + 1) * S->row_step() ;
      text_view_volume() ;
         glColor3f(0, 1, 1) ;
         draw_label(3, 24, phase_label(P).c_str()) ;
         draw_label(3, 40, tti_label(tti, *max).c_str()) ;
         draw_label(3, 56, distance_label(D).c_str()) ;
      restore_view_volume() ;
   }
}

// Helpers for drawing history
typedef std::deque<float> History ;

static void draw_history(const History& history)
{
   const int N = history.size() ;
   History::const_iterator y = history.begin() ;
   glBegin(GL_LINE_STRIP) ;
      for (int x = 0; x < N; ++x, ++y)
         glVertex2f(x, *y) ;
   glEnd() ;
}

static void draw_history(const History& actual, const History& predicted)
{
   glPushAttrib(GL_COLOR_BUFFER_BIT) ;
      glColor3f(0.0f, 0.15f, 0.85f) ;
      draw_history(actual) ;

      glColor3f(1, 0, 0) ;
      draw_history(predicted) ;
   glPopAttrib() ;
}

// This function renders the actual and predicted time-to-impact values
// so that users can gauge how well the Bayes filter's predictions match
// the actual situation on the ground.
void TTIEstimator::render_tti()
{
   // Make local copies so as to not hold up client behaviour's thread
   viz_lock() ;
      const float max_probability =
         *(std::max_element(m_belief.begin(), m_belief.end())) ;
      History actual = m_actual ;
      History predicted = m_predicted ;
   viz_unlock() ;

   // Setup view volume so that x-coordinates match the amount of TTI
   // history available and y-coordinates match the TTI range.
   setup_view_volume(0, m_geometry.width, 0, 60) ;

   // Draw the actual and predicted TTI histories
   draw_history(actual, predicted) ;
   restore_view_volume() ;

   // Label current values of actual and predicted times-to-impact
   if (Params::render_labels())
   {
      float A = actual.back() ;
      float P = predicted.back() ;
      text_view_volume() ;
         glColor3f(0, 1, 1) ;
         draw_label(3, 24, tti_label(A).c_str()) ;
         draw_label(3, 40, tti_label(P, max_probability).c_str()) ;
         draw_label(3, 56, error_label(A, P).c_str()) ;
      restore_view_volume() ;
   }
}

// This function renders the actual and predicted distances to obstacles
// based on the time-to-impact estimates so that users can gauge how well
// the Bayes filter's predictions match the actual situation on the
// ground.
void TTIEstimator::render_distance()
{
   // Make local copies so as to not hold up client behaviour's thread
   viz_lock() ;
      History actual = m_actual ;
      History predicted = m_predicted ;
   viz_unlock() ;

   // Setup view volume so that x-coordinates match the amount of
   // distance history available and y-coordinates match the distance
   // range.
   setup_view_volume(0, m_geometry.width, 0, Params::distance_range().max()) ;

   // Draw the actual and predicted distance histories
   draw_history(actual, predicted) ;
   restore_view_volume() ;

   // Label current values of actual and predicted times-to-impact
   if (Params::render_labels())
   {
      float A = actual.back() ;
      float P = predicted.back() ;
      text_view_volume() ;
         glColor3f(0, 1, 1) ;
         draw_label(3, 24, distance_label(round(A)).c_str()) ;
         draw_label(3, 40, distance_label(round(P)).c_str()) ;
         draw_label(3, 56, error_label(A, P).c_str()) ;
      restore_view_volume() ;
   }
}

#endif

//----------------------------- CLEAN-UP --------------------------------

TTIEstimator::~TTIEstimator(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
