/**
   \file  Robots/LoBot/lgmd/LocustModel.H
   \brief Abstract base class for different locust models.

   This file defines a class that supplies a common interface for all the
   different locust/LGMD models supported by Lobot/Robolocust.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/lgmd/LocustModel.H $
// $Id: LocustModel.H 13183 2010-04-08 07:17:49Z mviswana $
//

#ifndef LOBOT_LOCUST_MODEL_BASE_DOT_H
#define LOBOT_LOCUST_MODEL_BASE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/io/LoInputSource.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/range.hh"

// INVT image support
#include "Image/Rectangle.H"

// Standard C++ headers
#include <deque>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LocustModel
   \brief Abstract base class for locust models of different types.

   This class acts as an abstract base for all the different locust/LGMD
   models supported by Lobot/Robolocust. Since it is an abstract class,
   it cannot be instantiated. However, its descendants are meant to be
   instantiated using the generic factory<LocustModel> defined in
   factory.hh.
*/
class LocustModel : public Drawable {
   // Prevent copy and assignment
   LocustModel(const LocustModel&) ;
   LocustModel& operator=(const LocustModel&) ;

   /// Every locust model will read its assigned subportion of the input
   /// image from its source and then spit out a number that somehow
   /// measures the output of the locust LGMD (e.g., a firing rate or
   /// membrane potential or something similar).
   float m_lgmd ;

   /// The LGMD measure computed by each different locust model must lie
   /// within some range. This range can be fixed or adaptive. Derived
   /// classes are free to implement the range computation in any way
   /// they see fit. This class ensures that all LGMD values get clamped
   /// to this range.
   range<float> m_range ;

protected:
   /// This method allows derived classes to implement something other
   /// than a fixed range for the LGMD values they compute. Usually, they
   /// will call this method as part of their update cycle.
   void adjust_range(const range<float>& r) {m_range = r ;}

   /// Each instance of a locust model must have some input image source.
   /// Usually, this will be the compositor that reads the current frames
   /// from all the video streams and then stitches them together to
   /// create a crude sort of panaromic image.
   const InputSource* m_source ;

   /// Each locust will be setup to look in a particular direction. Zero
   /// degrees corresponds to looking straight ahead. Positive angles are
   /// for the left and negative angles for locusts looking to the right.
   /// The direction angles are specified in degrees.
   float m_direction ;

   /// When working with vision as the primary sensing modality, each
   /// locust model will usually read only a subportion of the input
   /// image so as to allow simulation of multiple locusts with limited
   /// FOVs.
   Rectangle m_rect ;

   /// In case the locust model is set up to read its input from a laser
   /// range finder rather than a video frame source, we use a range of
   /// angles for the distance measurements instead of a rectangular
   /// subregion of an input image.
   range<int> m_lrf_range ;

   /// If the input source is a laser range finder, then each locust can
   /// (if required) keep track of the distance to approaching obstacles
   /// in its field of view. This can be useful, for instance, to gauge
   /// how well a particual model works w.r.t. ground truth.
   float m_distance ;

   /// In addition to computing an LGMD spike rate, some locust models
   /// might also work with a time-to-impact value.
   float m_tti ;

public:
   /// Methods to return the cached result of the LGMD computations.
   //@{
   float get_lgmd() const {return m_lgmd    ;}
   float value()    const {return get_lgmd();} // for compatibility with EMD
   //@}

   /// This method returns the minimum and maximum possible values of the
   /// LGMD measure this model can compute. This range is useful for
   /// normalization and other scaling operations.
   range<float> get_range() const {return m_range ;}

   /// Retrieve the direction along which this locust is setup to look.
   float direction() const {return m_direction ;}

   /// This method returns the subportion of the input image that the
   /// this particular locust is "monitoring."
   const Rectangle& get_rect() const {return m_rect ;}

   /// This method returns the LRF angular range that specifies a
   /// locust's FOV when an LRF (rather than a camera) is the input
   /// source.
   range<int> get_lrf_range() const {return m_lrf_range ;}

   /// This method returns the current distance in this locust's FOV as
   /// reported by the laser range finder.
   float distance() const {return m_distance ;}

   /// This method returns the current time-to-impact computed by the
   /// LGMD model.
   float tti() const {return m_tti ;}

   /// On instantiation, each locust model will need to be provided these
   /// parameters.
   struct InitParams {
      range<float>       spike_range ; //< min and max spike rates
      float              direction ;   //< angle in which locust is looking
      const InputSource* source ;      //< from where model is getting i/p data
      Rectangle          rect ;        //<  source rectangle for image sources
      range<int>         lrf_range ;   //< angular range for LRF input sources
      std::string        name ;        //< name of this model's drawable
      Drawable::Geometry geometry ;    //< location and size of drawing area

      InitParams() ;
   } ;

protected:
   /// A protected constructor because locust models are usually
   /// instantiated via a factory and config file setting. During
   /// initialization, derived class instances should pass in suitable
   /// values for the various initialization parameters using an
   /// InitParams structure.
   LocustModel(const InitParams&) ;

public:
   /// This method triggers the LGMD computations that are to be
   /// performed by the locust model and caches the result internally.
   virtual void update() = 0 ;

protected:
   /// Once a derived class has computed a suitable value for the LGMD
   /// measure, it should call this method to store that value for later
   /// retrieval by other parts of the Lobot/Robolocust framework.
   void update_lgmd(float lgmd) {
      m_lgmd = clamp(lgmd, m_range) ;
      add_spike(m_lgmd) ;
   }

private:
   /// It is useful to be able to visualize the LGMD spiking activity.
   /// These functions and data structures are used for that purpose.
   //@{
   std::deque<float> m_spikes ;

   void add_spike(float) ;
   void render_me() ;
   //@}

public:
   /// Clean-up.
   virtual ~LocustModel() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
