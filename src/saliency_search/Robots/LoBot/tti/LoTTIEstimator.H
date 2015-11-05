/**
   \file  Robots/LoBot/tti/LoTTIEstimator.H
   \brief A Bayesian time-to-impact state estimator.

   This file defines a class that estimates time-to-impact from LGMD
   spikes.

   As per the research by Gabbiani, et al., we know that LGMD spikes are
   related to the time-to-impact of approaching objects. Unfortunately,
   given a spike rate, we cannot easily determine the corresponding
   time-to-impact (TTI) because the spike rate function is
   non-invertible.

   Thus, for each (actual or virtual) locust connected to the robot, we
   will have a corresponding TTI estimator that will perform Bayesian
   state estimation in an attempt to determine the corresponding TTI
   given spike rate. Once we have the TTI for a locust, we can use the
   velocity information returned by the robot's motor system to calculate
   a distance estimate for that locust. This would, in effect, allow us
   to use the locust array as a kind of range sensor.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/tti/LoTTIEstimator.H $
// $Id: LoTTIEstimator.H 14018 2010-09-23 07:10:34Z mviswana $
//

#ifndef LOBOT_TTI_ESTIMATOR_DOT_H
#define LOBOT_TTI_ESTIMATOR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/tti/LoSensorModel.H"
#include "Robots/LoBot/lgmd/LocustModel.H"

#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/misc/wma.hh"
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <deque>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::TTIEstimator

   \brief Encapsulation of Bayesian time-to-impact estimation using LGMD
   spike rates.

   This class implements a time-to-impact (TTI) estimator for each (real
   or virtual) locust connected to the robot.

   Since LGMD spikes are related to the TTI of approaching objects, we
   can use Bayesian state estimation to determine the TTI given the
   current spike rate. Combining this TTI with the motor system's
   velocity info allows us to compute the distance to obstacles for each
   locust direction, in effect, converting our array of visual collision
   detectors into a range sensor.
*/
class TTIEstimator : public Drawable {
   // Prevent copy and assignment
   TTIEstimator(const TTIEstimator&) ;
   TTIEstimator& operator=(const TTIEstimator&) ;

   /// Each instance of this class must be "linked" to a corresponding
   /// locust.
   const LocustModel* m_locust ;

   /// The LGMD spike rate is a function of an approaching object's
   /// time-to-impact. Unfortunately, that function is not invertible,
   /// meaning that we cannot simply apply an inverse function to go
   /// from spike rate to TTI. Therefore, we use Bayesian state
   /// estimation to determine an approximate TTI given a locust's
   /// current LGMD spike rate.
   ///
   /// The state estimation works by discretizing the possible TTI
   /// values and continually updating a posterior probability
   /// distribution (aka belief) every time we get new sensor (i.e.,
   /// LGMD) readings. The discretization is specified in the config
   /// file via a range of TTI values and a step size.
   ///
   /// This data structure holds the current belief state.
   //@{
   typedef std::vector<float> Belief ;
   Belief m_belief ;
   //@}

   /// When we update the belief, we also update the current estimate of
   /// the time-to-impact by using the belief state with the maximum
   /// likelilood value and store that estimate in this variable.
   float m_tti ;

   /// This variable keeps track of the likelilood value associated with
   /// the belief state that currently has the max likelilood.
   float m_confidence ;

   /// These two data structures are used to maintain a history of recent
   /// actual and predicted TTI or distance values for visualization
   /// purposes.
   std::deque<float> m_actual, m_predicted ;

   /// Each locust is setup to look in a particular direction.
   /// lobot::LocustModel specifies that direction in polar form. For
   /// this class, we prefer a Cartesian vector representation, which
   /// gets used in the distance computation based on the TTI
   /// estimate.
   ///
   /// DEVNOTE: This data member is used simply to avoid having to
   /// keep computing the same vector over and over. Each locust is
   /// initialized to look in some direction. That direction doesn't
   /// change. When we want to compute a distance reading from the TTI
   /// estimate, we will have to project the robot's velocity vector
   /// onto the locust's direction vector. Since the locust's
   /// direction vector will always be the same, creating a vector by
   /// computing the sine and cosine of the direction stored in
   /// m_locust is somewhat wasteful. So we compute it once when the
   /// estimator is created.
   const Vector m_direction ;

   /// Although the TTI estimator can obtain the LGMD spike rate for
   /// its locust directly using the above data member, we choose to
   /// instead copy that value before performing the Bayesian state
   /// estimation so as to minimize the amount of time spent holding
   /// on to the Robolocust update lock.
   ///
   /// DEVNOTE: If we don't copy the individual spike rates, client
   /// behaviours will have to hold the update lock for quite a while as
   /// the estimators compute the TTI for each locust using the recursive
   /// Bayesian update equations. This can hold up the main thread's
   /// sensorimotor updates. Bad Idea.
   ///
   /// DEVNOTE 2: The LGMD-vs-TTI curve can be partitioned into two
   /// phases, viz., LOOMING and BLANKING. To detect the signal peak that
   /// divides these two phases, we need to keep track of the second
   /// derivative of the LGMD signal. Therefore, we need the two most
   /// recent LGMD values and also two values for the first derivative.
   /// The second derivative itself is low-pass filtered to avoid sudden
   /// transitions from one state to the other.
   //@{
   float m_lgmd[2] ;   // input signal
   float m_fder[2] ;   // first derivative
   wma<float> m_sder ; // low-pass filtered second derivative
   //@}

   /// After the TTI estimate has been made, client behaviours can gauge
   /// the distance to obstacles in each locust direction by projecting
   /// the robot's current velocity in that direction and getting the
   /// estimator to compute an estimate of the distance based on its TTI
   /// belief. This data member is used to hold the current distance
   /// estimate for the locust to which this TTI estimator is linked.
   float m_distance ;

   /// In order for the whole Bayesian state estimation to work, we need
   /// a sensor model that supplies probability values for LGMD spike
   /// rates given times-to-impact, i.e., a table of P(lgmd|tti), the
   /// so-called causal data.
   ///
   /// The LGMD spike rate is a function of an approaching object's
   /// time-to-impact. When the object is far away, the LGMD's spike rate
   /// will be fairly low. As it approaches, the LGMD starts firing very
   /// rapidly. Shortly before impact, the LGMD firing rate reaches a
   /// peak and then drops off sharply until impact.
   ///
   /// The peak described above "partitions" the LGMD firing rate vs.
   /// time-to-impact curve into two distinct "phases." We refer to the
   /// first phase, wherein the curve rises to its peak, as LOOMING
   /// because the object is looming large in the LGMD's field of view.
   /// The second phase, we call BLANKING because feedforward inhibition
   /// kicks in after the peak to shutdown the LGMD right before impact.
   ///
   /// To get the Bayesian time-to-impact estimation to work well, it
   /// would be best to use different causal likelihood profiles
   /// corresponding to each of these two "phases" described above. These
   /// two functions provide sensor models for the two different
   /// likelihood profiles.
   //@{
public:
   static SensorModel&  looming_sensor_model() ;
   static SensorModel& blanking_sensor_model() ;
private:
   //@}

   /// This member variable points to the correct sensor model depending
   /// on the LGMD input signal's phase as described above. Thus, in the
   /// LOOMING phase, it will point to lobot::TTIEstimator::m_looming;
   /// and to lobot::TTIEstimator::m_blanking in the BLANKING phase.
   const SensorModel* m_sensor_model ;

   /// This enumeration specifies symbols for the different LGMD phases
   /// recognized by the TTI estimator.
   enum LGMDPhase {
      LOOMING,
      BLANKING,
   } ;

   /// This method returns the current phase of the input LGMD signal by
   /// checking what the current sensor model is pointing to.
   LGMDPhase lgmd_phase() const ;

   /// This method switches the sensor model to the specified likelihood
   /// profile, taking care to perform any necessary mutex locking (e.g.,
   /// to avoid clashes with the visualization thread).
   void sensor_model(const SensorModel*) ;

public:
   /// When instantiating a TTI estimator, client behaviours must specify
   /// the corresponding locust for which the estimator is to determine
   /// time-to-impact given LGMD spike rate.
   TTIEstimator(const LocustModel*) ;

   /// Before performing the TTI estimation, client behaviours must first
   /// copy the LGMD spike rates from the locusts into their
   /// corresponding estimators. This two-step approach is necessary to
   /// minimize the duration that client behaviours will hold on to the
   /// Robolocust update locks.
   void copy_lgmd() ;

   /// This method implements the recursive Bayesian state estimation
   /// update equations in order to determine the time-to-impact given
   /// an LGMD spike rate.
   void update() ;

   /// This method projects the supplied velocity vector (assumed to
   /// be the robot's current velocity) onto this TTI estimator's
   /// locust's direction vector and uses the latest estimate of the
   /// time-to-impact to compute a corresponding obstacle distance in
   /// that locust's direction.
   void compute_distance(const Vector& velocity) ;

   /// Accessors.
   //@{
   float  lgmd()               const {return m_lgmd[0]       ;}
   float  actual_tti()         const {return m_locust->tti() ;}
   float  predicted_tti()      const {return m_tti           ;}
   float  actual_distance()    const {return m_locust->distance() ;}
   float  predicted_distance() const {return m_distance           ;}
   float  distance()           const {return predicted_distance() ;}
   float  confidence()         const {return m_confidence         ;}
   float  locust_direction()   const {return m_locust->direction();}
   Vector direction()          const {return m_direction          ;}
   range<int> lrf_range()      const {return m_locust->get_lrf_range() ;}
   //@}

private:
   /// Visualization
   //@{
   static void render_hook(unsigned long) ;
   void render_belief() ;
   void render_tti() ;
   void render_distance() ;
   //@}

public:
   /// Clean-up.
   ~TTIEstimator() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
