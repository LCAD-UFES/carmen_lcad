/**
   \file  Robots/LoBot/control/LoCalibrateLET.H
   \brief A do-nothing behaviour used to compute the probability values
   for lgmd_extricate_tti's sensor model.

   This file defines a class that computes probability values for the
   lgmd_extricate_tti behaviour's sensor model. The lgmd_extricate_tti
   behaviour works by applying recursive Bayesian updates to a
   probability distribution containing P(tti|lgmd) values. For the
   Bayesian update to work, the behaviour needs a table of P(lgmd|tti)
   values, the so-called sensor model.

   Now, given a time-to-impact, the Gabbiani LGMD model yields the
   corresponding spike rate. Thus, to figure out P(lgmd|tti) values, all
   we need to do is pick random times-to-impact and apply the Gabbiani
   model to get the corresponding LGMD spike rates. We then discretize
   both these quantities to get the correct bin in the sensor model's
   probability table and simply increment that bin.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoCalibrateLET.H $
// $Id: LoCalibrateLET.H 13732 2010-07-29 14:16:35Z mviswana $
//

#ifndef LOBOT_CALIBRATE_LET_DOT_H
#define LOBOT_CALIBRATE_LET_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/misc/singleton.hh"

// Standard C++ headers
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::CalibrateLET

   \brief A do-nothing behaviour for calibrating lgmd_extricate_tti.

   This class implements a behaviour that produces the table of
   probability values required by lgmd_extricate_tti's sensor model. This
   behaviour does not actually control lobot or even use the robot's
   sensors. It is meant to be a purely computational, off-line task.

   The idea here is to run the Gabbiani "forward" model for LGMD spike
   generation for all the time-to-impact discretizations. We then find
   the correct bin in the P(lgmd|tti) table and increment it. In the end,
   right before quitting, this table is normalized to yield a bunch of
   probabilities (viz., values between zero and one).

   The lgmd_extricate_tti behaviour then uses these P(lgmd|tti) values to
   compute P(tti|lgmd) by applying recursive Bayesian updates to a
   probability distribution over a discretized TTI space.

   NOTE: The table of P(lgmd|tti) values, viz., the sensor model,
   contains causal information, i.e., the state that causes a particular
   measurement. The goal of Bayesian state estimation is to go in the
   opposite direction, i.e., given a particular measurement, what is the
   most likely state we are in?

   Usually, causal information is easier to obtain. One way is by direct
   experimentation and sampling, for example, driving the robot about in
   a known environment and examining the correlation between its states
   and corresponding sensor readings.

   To go this route in lobot's case, we would simply drive it around,
   using the laser range finder and encoders to compute time-to-impact.
   The locust module can be used to generate LGMD spikes. Then, all we'd
   have to do is discretize the TTI and LGMD values and increment the
   correct bin.

   Unfortunately, in practice, the above approach yields very bad results
   due to two factors. Firstly the TTI computed is almost always
   concentrated at the upper end of the TTI discretization's range
   because of the effects of other behaviours that slow the robot down,
   stop it and perform extrication. Secondly, the LGMD activity only
   becomes significant very near impact, peaking briefly followed by a
   rapid decay. For the most part, the LGMD remains silent. Thus,
   sampling it frequently under normal driving conditions results in a
   concentration at the lower end of the LGMD spectrum.

   The alternative to direct experimentation and sampling for obtaining
   causal information is to apply a computational model. In situations
   where the mathematics and physics of the underlying process are
   well-understood, this might actually be a better approach.

   In lobot's case, thanks to Gabbiani, et al., we already have a neat
   model relating LGMD spikes to TTI. Thus, there really is no need to
   drive the robot around and collect statistics. Instead, we simply
   apply the model and obtain the necessary information.
*/
class CalibrateLET : public Behavior {
   // Prevent copy and assignment
   CalibrateLET(const CalibrateLET&) ;
   CalibrateLET& operator=(const CalibrateLET&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<CalibrateLET, base> ;
   typedef register_factory<CalibrateLET, base> my_factory ;
   static  my_factory register_me ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   CalibrateLET() ;

   /// This method initializes the sensor model.
   void pre_run() ;

   /// This is an interactive behaviour designed to respond to keypress
   /// events. Thus, the action method remains empty.
   void action() ;

   /// When the user presses the 'k', '+', 'u' or 'i' keys, the
   /// calibrator will increment the standard deviation of the Gaussian
   /// used to weight the sensor model updates. The 'j', '-' or 'd' keys
   /// will decrement it.
   ///
   /// These keys allow users to experiment with different settings to
   /// achieve the desired sensor model.
   void keypress(unsigned char key) ;

   /// Visualization to help with development and debugging.
   void render_me() ;

   /// OpenGL initialization and clean-up.
   //@{
   void gl_init() ;
   void gl_cleanup() ;
   //@}

   /// Clean-up.
   ~CalibrateLET() ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the calibrate_lgmd_extricate_tti
   /// behaviour.
   class Params : public singleton<Params> {
      /// This behaviour is an interactive one that responds to keypress
      /// events and updates the sensor models containing the causal
      /// probabilities for the Bayesian time-to-impact estimation.
      /// Robolocust uses two sensor models: one for the LOOMING phase of
      /// the LGMD signal and the other for the BLANKING phase.
      ///
      /// To prevent pathological situations, we refrain from putting
      /// zeros (or extremely low) probabilities in the sensor models by
      /// applying a Gaussian weighting formula to update bins
      /// neighbouring the one "pointed" to by various [TTI, LGMD] pairs.
      /// The parameter used to effect this weighting is a standard
      /// deviation for the Gaussian.
      ///
      /// When this behaviour starts up, it will set the "input focus" to
      /// the LOOMING sensor model. Thus, when the user presses the 'k',
      /// '+', 'u' or 'i' keys, the calibrator will increment the
      /// standard deviation of the Gaussian used to weight the LOOMING
      /// sensor model updates (see above). The 'j', '-' or 'd' keys will
      /// decrement the Gaussian.
      ///
      /// When the user presses the TAB key, the behaviour will set the
      /// "input focus" to the BLANKING sensor model. The "iku+" and
      /// "dj-" keys will then affect the BLANKING sensor model's
      /// Gaussian. Pressing TAB again will switch back to the LOOMING
      /// sensor model; so on and so forth.
      ///
      /// This setting specifies the amount by which the behaviour should
      /// increment/decrement the sigma value associated with the
      /// above-mentioned Gaussian weighting formula.
      float m_dsigma ;

      /// Private constructor because this is a singleton.
      Params() ;

      // Boilerplate code to make generic singleton design pattern work
      friend class singleton<Params> ;

   public:
      /// Accessing the various parameters.
      //@{
      static float dsigma() {return instance().m_dsigma ;}
      //@}

      /// Clean-up.
      ~Params() ;
   } ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
