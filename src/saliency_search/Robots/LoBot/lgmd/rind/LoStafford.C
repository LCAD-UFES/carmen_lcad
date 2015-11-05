/**
   \file  Robots/LoBot/lgmd/rind/LoStafford.C
   \brief Stafford's LGMD model.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/lgmd/rind/LoStafford.C $
// $Id: LoStafford.C 13037 2010-03-23 01:00:53Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/lgmd/rind/LoStafford.H"
#include "Robots/LoBot/config/LoConfig.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoMath.H"

// INVT image support
#include "Image/Convolver.H"
#include "Image/CutPaste.H"
#include "Image/MathOps.H"
#include "Image/Rectangle.H"

// Standard C++ headers
#include <numeric>
#include <algorithm>
#include <functional>
#include <utility>
#include <cmath>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------ STATIC DATA MEMBERS --------------------------

/*
   All instances of the Stafford model share the neural net layers used
   to compute LGMD spikes so as to save on the amount of overall
   computation required.

   Each instance represents a separate (virtual) locust looking in a
   different direction with a limited field of view. Thus, each instance
   is setup to read a subportion of the composited source image. These
   subportions are just the source image divided into consecutive
   vertical strips. For example, if we have a single camera grabbing
   320x240 images and use the default value of 32 pixels for each virtual
   locust's FOV, we will get 320/32 = 10 vertical strips and, therefore,
   10 virtual locusts.

   Each virtual locust "monitors" the strip right in front of it (foveal
   vision) as well as the strips to the left and right (peripheral
   vision). If we were to treat each locust separately, we would perform
   subtraction, convolution, etc. for its strips and then repeat the
   operations for the overlapping strips of the neighbouring locusts.

   This is extremely wasteful. So, to avoid repeating the same
   computations over and over again, we use the following static data
   members to store the layers of the neural net for the entire
   composited input image. When the update() method of the first instance
   of the Stafford model is invoked, we perform the necessary operations
   for the entire composited image. But then, each instance reads only
   its assigned subportion of the S-layer to compute its LGMD membrane
   potential.
*/
StaffordModel::layer StaffordModel::l_layer ;
StaffordModel::layer StaffordModel::p_layer ;
StaffordModel::layer StaffordModel::i_layer ;
StaffordModel::layer StaffordModel::s_layer ;

// To enable the above-mentioned computational savings via neural net
// layer sharing, we have to keep track of the total number of instances
// of the Stafford model and the number for which the layer computations
// have been performed during each instance's update cycle.
int StaffordModel::m_instances ;
int StaffordModel::m_layers_computed ;

//-------------------------- INITIALIZATION -----------------------------

// Whenever a virtual locust based on the Stafford model is created, we
// need to bump up the number of instances to ensure that the sharing
// described above works properly.
StaffordModel::StaffordModel(const LocustModel::InitParams& p)
   : base(p)
{
   ++m_instances ;
}

//------------------------ LAYER COMPUTATIONS ---------------------------

// This method performs the necessary layer computations, taking care to
// do them only once, i.e., for the first instance for which the LGMD
// update cycle is invoked and then skipping them for the rest.
void StaffordModel::compute_layers()
{
   if (m_layers_computed == 0)
   {
      prime_previous_layers() ;

      l_layer.current = m_source->get_grayscale_image() ;

      p_layer.current = absDiff(l_layer.current, l_layer.previous) ;

      GrayImage kernel(3, 3, NO_INIT) ;
      std::fill(kernel.beginw(), kernel.endw(), 1/9.0f) ;
      Convolver C(kernel, i_layer.previous.getDims()) ;
      i_layer.current =
         C.spatialConvolve((p_layer.current + p_layer.previous) * .25) ;

      s_layer.current = p_layer.current - i_layer.previous * 2 ;
      std::transform(s_layer.current.begin(), s_layer.current.end(),
                     s_layer.current.beginw(),
                     std::bind2nd(max<float>(), 0)) ; // discard -ve values
   }
   ++m_layers_computed ;
}

// Once all instances have finished with their LGMD update, we need to
// move the current layers into the previous time-step's slot. And, to
// ensure smooth operation of the sharing described at the start of this
// file, we need to reset the number of instances for which the LGMD
// computations have been done. Without this, the next update cycle will
// fail.
void StaffordModel::reset_layers()
{
   if (m_layers_computed == m_instances)
   {
      l_layer.previous = l_layer.current ;
      p_layer.previous = p_layer.current ;
      i_layer.previous = i_layer.current ;
      s_layer.previous = s_layer.current ;

      m_layers_computed = 0 ;
   }
}

// We need to prime the pump for the very first update cycle. At this
// point, there are no previous time-steps for any of the layers. As a
// starting point, we use the current image as the input for the
// L-layer's previous time-step and just keep all the other layers empty.
void StaffordModel::prime_previous_layers()
{
   if (! l_layer.previous.initialized())
      l_layer.previous = m_source->get_grayscale_image() ;

   if (! p_layer.previous.initialized())
      p_layer.previous.resize(l_layer.previous.getDims(), true) ;

   if (! i_layer.previous.initialized())
      i_layer.previous.resize(l_layer.previous.getDims(), true) ;

   if (! s_layer.previous.initialized())
      s_layer.previous.resize(l_layer.previous.getDims(), true) ;
}

//------------------------- DSMD COMPUTATION ----------------------------

// This helper routine returns the appropriate block size to use given
// the total number of pixels along the desired dimension (i.e., x- or
// y-direction).
static int dsmd_block_size(int dim, int ideal, int alt)
{
   int block_size = ideal ;
   int num_emds = dim/block_size ;
   if (num_emds < 2)
      block_size = alt ; // just hope this works!
   return block_size ;
}

// This method uses three functions or function objects to compute the
// DSMD membrane potential for the desired direction.
//
// The first function/function object returns the appropriate dimension
// of the locust's "viewing" rectangle/window to the world. For
// horizontal DSMDs, this function would return the width of the
// rectangle; for vertical DSMDs, the height.
//
// The second parameter is a function/function object that computes the
// DSMD block along the appropriate dimension. For horizontal DSMDs, it
// should compute the bounds of the ith DSMD block running horizontally
// along the center of the image. For vertical DSMDs, it should compute
// the bounds of the ith DSMD block running vertically along the center
// of the image.
//
// The third parameter computes the membrane potential using the
// rectangular DSMD blocks returned by the second parameter and the
// current and previous S-layer contents of those blocks.
template<typename rect_dim, typename rect_comp, typename pot_comp>
float StaffordModel::compute_dsmd_potential(rect_dim  dimension,
                                             rect_comp compute_dsmd_rect,
                                             pot_comp  compute_potential)
{
   int d = dimension(m_rect) ; // width or height of this locust's FOV
   int b = dsmd_block_size(d, Params::ideal_dsmd_block_size(),
                              Params::alt_dsmd_block_size()) ;
   int n = d/b ; // number of EMDs for the DSMD under consideration

   float P = 0 ; // DSMD membrane potential
   Rectangle R1 = compute_dsmd_rect(0, b, m_rect) ;
   for (int i = 1; i < n; ++i) {
      Rectangle R2 = compute_dsmd_rect(i, b, m_rect) ;
      P += compute_potential(s_layer.current, s_layer.previous, R1, R2) ;
      R1 = R2 ;
   }
   return P ;
}

// rect_dim functions for above template method
static int rect_width (const Rectangle& R) {return R.width() ;}
static int rect_height(const Rectangle& R) {return R.height() ;}

// rect_comp functions for above template method
static Rectangle
compute_horz_dsmd_rect(int i, int block_size, const Rectangle& R)
{
   int left   = std::max(i * block_size, 0) ;
   int right  = std::min(left + block_size, R.width()) ;
   int top    = std::max((R.height() - block_size)/2, 0) ;
   int bottom = std::min(top + block_size, R.height()) ;
   return Rectangle::tlbrI(top, left, bottom, right) ;
}

static Rectangle
compute_vert_dsmd_rect(int i, int block_size, const Rectangle& R)
{
   int left   = std::max((R.width() - block_size)/2, 0) ;
   int right  = std::min(left + block_size, R.width()) ;
   int top    = std::max(i * block_size, 0) ;
   int bottom = std::min(top + block_size, R.height()) ;
   return Rectangle::tlbrI(top, left, bottom, right) ;
}

// A quick helper to compute the membrane potential corresponding to the
// specified rectangular portion of the supplied image (usually, the
// S-layer).
static float membrane_potential(const GrayImage& I, const Rectangle& R)
{
   return sum(crop(I, R.topLeft(), R.dims() - 1)) ;
}

// The following two functions are for use as the third pot_comp
// parameter of the compute_dsmd_potential() template method defined
// above.
static float
compute_dsmd_potential_R1_current(const GrayImage& current,
                                  const GrayImage& previous,
                                  const Rectangle& R1, const Rectangle& R2)
{
   return membrane_potential(current, R1) * membrane_potential(previous, R2) ;
}

static float
compute_dsmd_potential_R2_current(const GrayImage& current,
                                  const GrayImage& previous,
                                  const Rectangle& R1, const Rectangle& R2)
{
   return compute_dsmd_potential_R1_current(current, previous, R2, R1) ;
}

// Given the spike count of some DSMD (e.g., left or up) and its opposite
// (i.e., right or down), this function determines whether there has been
// only lateral motion rather than motion in both directions.
//
// Since the spike counts are either 0 or 1, checking for lateral motion
// is a simple matter of an XOR operation. For example, motion in the
// left DSMD will register as 1 and none in the right DSMD will be zero.
// If XOR these two values, we will get 1, indicating that motion was
// detected in only one direction. However, if there was motion in both
// directions (or in neither), the XOR will return false.
//
// To make it easy to compute the final spike count from the individual
// spike counts of the LGMD and DSMDs and their corresponding weights,
// the counts are stored as floats rather than ints. So before we can
// apply the XOR operator, we convert the supplied counts to ints.
/*
static bool lateral_motion(float d1, float d2)
{
   int a = (d1 > .5) ;
   int b = (d2 > .5) ;
   return a ^ b ;
}

static inline bool uniform_expansion(float d1, float d2)
{
   return ! lateral_motion(d1, d2) ;
}
//*/

//*
static bool lateral_motion(float p1, float p2, float threshold)
{
   float p = p1/p2 ;
   return (p > threshold) || (p < 1/threshold) ;
}
//*/

//---------------------------- LGMD UPDATE ------------------------------

// The following function object uses the sigmoid formula described
// earlier (see comment preceding definition of the AREA_MAGNIFIERS
// array) to scale raw membrane potentials to the [.5,1] range.
namespace {

class sigmoid {
   float area ;
   const float* magnifiers ;
   mutable int i ;
public:
   sigmoid(float, const float*) ;
   float operator()(float) const ;
} ;

sigmoid::sigmoid(float A, const float* M)
   : area(A), magnifiers(M), i(0)
{}

float sigmoid::operator()(float potential) const
{
   return 1/(1 + exp(-potential/(area * magnifiers[i++]))) ;
}

}

// Quick helper macro for debug support
#define LOSTU_DUMP(array) \
           dump(array, array + NUM_NEURONS, "StaffordUpdate", #array)

// This is the main interface routine for updating the LGMD "value" based
// on the latest frame acquired from the input video sources. After doing
// the layer computations as described in the Stafford paper, it computes
// the raw membrane potential of the LGMD by summing all the pixel values
// in the current S-layer. Then, it scales this potential down to [.5, 1]
// by applying a sigmoid function. Whenever this scaled down potential
// exceeds a predetermined (manual) threshold, it is counted as an LGMD
// spike.
//
// In addition to the LGMD spike, we also compute the FFI and DSMD
// potentials, scale them using the same sigmoid and the count them as
// spikes if they exceed their respective spiking thresholds.
//
// The final instantaneous spike rate is computed as a running average of
// a weighted sum of the LGMD and DSMD spike counts.
//
// The running average is computed using the usual formula, i.e.:
//     running average = weight * current + (1 - weight) * previous
//
// But before we can apply the above formula, we need to first normalize
// the previous spike rate value, i.e., scale it down to [0,1], because
// the current spike, being a weighted sum of either zeros or ones, will
// be a number in the range [0,1]. We can't blithely compute a running
// average using one number in [min, max] and another in [0,1]; the
// result will be horribly wrong.
//
// Once the previous spike rate value has been normalized, we compute the
// running average using it and the current spike count. Then, we scale
// the result back to its usual min/max range.
//
// NOTE: The sigmoid thingummy is not described in the Stafford paper.
// Rather it is in the following paper:
//
//     Yue, S., Santer, R. D., Yamawaki, Y., Rind, F. C.
//     Reactive direction control for a mobile robot: A locust-like
//     control of escape direction emerges when a bilateral pair of model
//     locust visual neurons are integrated.
//     Submitted to Autonmous Robots in 2008.
//
// ALSO: This function implements feed-forward and lateral motion
// inhibition as described in the Stafford paper as well as the one
// mentioned above. However, this inhibition does not work very well. So
// we setup the weights and other relevant parameters defined at the
// beginning of this file to ignore the FFI and DSMD neurons. We could
// just as well remove all this unused/dead code. But it's left alone
// just in case a magic wand shows up and enables us to figure out how to
// get this weird and bogus model to actually behave like the LGMD (i.e.,
// ignore lateral motion and respond preferentially to collisions).
void StaffordModel::update()
{
   compute_layers() ;

   float potentials[NUM_NEURONS] = {0} ;
   potentials[LGMD] = membrane_potential(s_layer.current, m_rect) ;
   potentials[FFI] = membrane_potential(p_layer.previous, m_rect) ;
   potentials[DSMD_LEFT] =
      compute_dsmd_potential(rect_width, compute_horz_dsmd_rect,
                             compute_dsmd_potential_R1_current) ;
   potentials[DSMD_RIGHT] =
      compute_dsmd_potential(rect_width, compute_horz_dsmd_rect,
                             compute_dsmd_potential_R2_current) ;
   potentials[DSMD_UP] =
      compute_dsmd_potential(rect_height, compute_vert_dsmd_rect,
                             compute_dsmd_potential_R1_current) ;
   potentials[DSMD_DOWN] =
      compute_dsmd_potential(rect_height, compute_vert_dsmd_rect,
                             compute_dsmd_potential_R2_current) ;
   //LOSTU_DUMP(potentials) ;

   float scaled_potentials[NUM_NEURONS] = {0} ;
   std::transform(potentials, potentials + NUM_NEURONS, scaled_potentials,
                  sigmoid(m_rect.area(), Params::area_magnifiers())) ;
   //LOSTU_DUMP(scaled_potentials) ;

   float spikes[NUM_NEURONS] = {0} ;
   std::transform(scaled_potentials, scaled_potentials + NUM_NEURONS,
                  Params::spike_thresholds(), spikes, std::greater<float>()) ;
   //LOSTU_DUMP(spikes) ;

   if (suppress_lgmd(spikes, potentials)) {
      update_lgmd(0) ;
      std::fill(spikes, spikes + NUM_NEURONS, 0) ;
   }

   // Even if the LGMD has been suppressed, the FFI and DSMD neurons may
   // still be able to produce some spiking from the LGMD network. The
   // following dot product computes a weighted sum of each neuron's
   // spike to produce the final spiking decision.
   //
   // NOTE: This weighted sum procedure is not described in any of the
   // LGMD related papers by Stafford, Blanchard, Yue, Rind and gang.
   // It's just something the author of this class made up to try and get
   // this model to work properly (i.e., suppress lateral motions and
   // respond only to collisional motion). It can be turned off by
   // setting the LGMD spike weight to 1 and the FFI and DSMD spike
   // weights to zero. Then, only the LGMD spike will be considered in
   // the final decision regarding whether or not the LGMD network emits
   // a spike or not (subject, of course, to the suppression signal
   // emitted by the FFI or DSMD).
   float spike = std::inner_product(spikes, spikes + NUM_NEURONS,
                                    Params::spike_weights(), 0.0f) ;

   const float min = get_range().min() ;
   const float max = get_range().max() ;

   float normalized_lgmd = (get_lgmd() - min)/(max - min) ;
   float w = Params::running_average_weight() ;
   float lgmd_avg = w * spike + (1 - w) * normalized_lgmd ;
   update_lgmd(min + lgmd_avg * (max - min)) ;//rescale from [0,1] to [min,max]
   //LINFO("final spike = %g, old spike = %g, new spike = %g",
         //spike, normalized_lgmd, lgmd_avg) ;
   //LINFO("LGMD rate = %g", getLGMD()) ;

   reset_layers() ;
}

// Check if LGMD suppression is on via either the FFI or DSMDs and their
// corresponding suppression tests.
bool StaffordModel::suppress_lgmd(const float spikes[],
                                  const float potentials[])
{
   if (Params::ffi_on()) // check for whole field motion
      return spikes[FFI] > 0.5 ;

   if (Params::dsmd_on()) //chk directionally sensitive neurons for lat. motion
      return lateral_motion(potentials[DSMD_LEFT], potentials[DSMD_RIGHT],
                            Params::horizontal_motion_threshold())
          || lateral_motion(potentials[DSMD_UP], potentials[DSMD_DOWN],
                            Params::vertical_motion_threshold()) ;

   return false ;
}

//----------------------------- CLEAN-UP --------------------------------

StaffordModel::~StaffordModel()
{
   --m_instances ;
   --m_layers_computed ;
   if (m_layers_computed < 0)
      m_layers_computed = 0 ;
}

//-------------------------- KNOB TWIDDLING -----------------------------

// Quick helper to retrieve configuration settings from the stafford
// section of the config file.
template<typename T>
static T conf(const std::string& key, const T& default_value)
{
   return Configuration::get<T>(LOLM_STAFFORD, key, default_value) ;
}

// Quick helper macro to retrieve lists from the stafford section of the
// config file.
#define LOST_GETCONF_ARRAY(key) \
           Configuration::get(LOLM_STAFFORD, #key, m_##key, \
                              default_##key, NUM_NEURONS)

// Parameters initialization
StaffordModel::Params::Params()
   : m_running_average_weight(conf("running_average_weight", 0.25)),
     m_ffi_on(conf("ffi", false)),
     m_dsmd_on(conf("dsmd", false)),
     m_horizontal_motion_threshold(conf("horizontal_motion_threshold", 1.5)),
     m_vertical_motion_threshold(conf("vertical_motion_threshold", 2.5)),
     m_ideal_dsmd_block_size(conf("ideal_dsmd_block_size", 10)),
     m_alt_dsmd_block_size(conf("alt_dsmd_block_size", 4))
{
   float default_spike_thresholds[] = {.99, 1.5, 1.5, 1.5, 1.5, 1.5} ;
   LOST_GETCONF_ARRAY(spike_thresholds) ;

   float default_spike_weights[] = {1, 0, 0, 0, 0, 0} ;
   LOST_GETCONF_ARRAY(spike_weights) ;

   float default_area_magnifiers[] = {2, 5, 500, 500, 500, 500} ;
   LOST_GETCONF_ARRAY(area_magnifiers) ;
}

// Parameters clean-up
StaffordModel::Params::~Params(){}

// Parameters access
const float* StaffordModel::Params::spike_thresholds()
{
   return instance().m_spike_thresholds ;
}

const float* StaffordModel::Params::spike_weights()
{
   return instance().m_spike_weights ;
}

const float* StaffordModel::Params::area_magnifiers()
{
   return instance().m_area_magnifiers ;
}

float StaffordModel::Params::running_average_weight()
{
   return instance().m_running_average_weight ;
}

bool StaffordModel::Params::ffi_on()
{
   return instance().m_ffi_on ;
}

bool StaffordModel::Params::dsmd_on()
{
   return instance().m_dsmd_on ;
}

float StaffordModel::Params::horizontal_motion_threshold()
{
   return instance().m_horizontal_motion_threshold ;
}

float StaffordModel::Params::vertical_motion_threshold()
{
   return instance().m_vertical_motion_threshold ;
}

int StaffordModel::Params::ideal_dsmd_block_size()
{
   return instance().m_ideal_dsmd_block_size ;
}

int StaffordModel::Params::alt_dsmd_block_size()
{
   return instance().m_alt_dsmd_block_size ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
