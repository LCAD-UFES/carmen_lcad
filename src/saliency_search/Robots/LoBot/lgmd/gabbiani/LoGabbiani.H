/**
   \file  Robots/LoBot/lgmd/gabbiani/LoGabbiani.H
   \brief Gabbiani's LGMD model.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/lgmd/gabbiani/LoGabbiani.H $
// $Id: LoGabbiani.H 13628 2010-06-28 23:48:02Z mviswana $
//

#ifndef LOBOT_GABBIANI_LGMD_MODEL_DOT_H
#define LOBOT_GABBIANI_LGMD_MODEL_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/lgmd/LocustModel.H"
#include "Robots/LoBot/io/LoInputSource.H"
#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/misc/singleton.hh"

// INVT headers
#include "Image/Rectangle.H"

// Standard C++ headers
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::GabbianiModel
   \brief Implementation of Gabbiani's LGMD model.
*/
class GabbianiModel : public LocustModel {
   // Prevent copy and assignment
   GabbianiModel(const GabbianiModel&) ;
   GabbianiModel& operator=(const GabbianiModel&) ;

   // Handy type to have around
   typedef LocustModel base ;

   // Boilerplate code to make the factory work
   friend  class subfactory<GabbianiModel, base, base::InitParams> ;
   typedef register_factory<GabbianiModel, base, base::InitParams> my_factory ;
   static  my_factory register_me ;

   /// Private constructor because this model is instantiated using a
   /// factory and accessed solely through the interface provided by its
   /// abstract base class.
   GabbianiModel(const base::InitParams&) ;

   /// These methods perform the LGMD computations.
   ///@{
   void update() ;
public:
   static float spike_rate(float tti) ;
   ///@}

   /// Destructor
private:
   ~GabbianiModel() ;

   // This inner class encapsulates various parameters that can be used
   // to tweak different aspects of the LGMD model implemented by the
   // GabbianiModel class.
   class Params : public singleton<Params> {
      // Initialization
      Params() ; // private because this is a singleton
      friend class singleton<Params> ;

      /// Gabbiani's multiplicative model of the LGMD correlates this
      /// neuron's firing rate with the product of a logarithmic and an
      /// exponential term with a constant of proportionality thrown in.
      float m_C ;

      /// The model uses a pair of parameters, viz., alpha and delta, for
      /// some reason. FIXME! WHY DO WE NEED THESE PARAMS? WHAT DO THEY
      /// MEAN?
      float m_alpha, m_delta ;

      /// According to Gabbiani, et al. the LGMD firing rate is strongly
      /// correlated to the ratio of the half-size of the approaching
      /// object and its velocity. This ratio is a time
      /// (length/velocity). This is the time it takes for the
      /// approaching object to cover a distance equal to its own (half)
      /// size.
      float m_l_over_v ;

      /// To better simulate LGMD spikes generated by actual locusts, we
      /// can corrupt the "ideal" spikes generated by the Gabbiani model
      /// with some Gaussian noise. This parameter specifies the standard
      /// deviation to use for the spike noise.
      float m_sigma ;

   public:
      // Accessing the various parameters
      static float C()        {return instance().m_C ;}
      static float alpha()    {return instance().m_alpha ;}
      static float delta()    {return instance().m_delta ;}
      static float sigma()    {return instance().m_sigma ;}
      static float l_over_v() {return instance().m_l_over_v ;}

      // Clean-up
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
