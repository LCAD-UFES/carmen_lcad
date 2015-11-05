/**
   \file Robots/LoBot/lgmd/rind/LoStafford.H

   \brief Stafford's LGMD model.

   This file defines a class that implements the LGMD model described in:

      Stafford, R., Santer, R. D., Rind, F. C.
      ``A Bio-inspired Visual Collision Detection Mechanism for Cars:
        Combining Insect Inspired Neurons to Create a Robust System.''
      BioSystems, 87, 162--169, 2007.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/lgmd/rind/LoStafford.H $
// $Id: LoStafford.H 12860 2010-02-18 15:55:20Z mviswana $
//

#ifndef LOBOT_STAFFORD_LGMD_MODEL_DOT_H
#define LOBOT_STAFFORD_LGMD_MODEL_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/lgmd/LocustModel.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/misc/singleton.hh"

// INVT image support
#include "Image/Image.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::StaffordModel
   \brief Implementation of Stafford's LGMD model.

   This class implements the LGMD model described in the following paper:

      Stafford, R., Santer, R. D., Rind, F. C.
      ``A Bio-inspired Visual Collision Detection Mechanism for Cars:
        Combining Insect Inspired Neurons to Create a Robust System.''
      BioSystems, 87, 162--169, 2007.

   The above paper computes LGMD spikes using a three-layer neural
   network. The final layer is a summing layer which spits out an LGMD
   membrane potential. Spikes are generated when this potential crosses a
   predetermined threshold.

   However, in this implementation, we use the raw membrane potential
   computed by the S-layer as the final LGMD "value."

   Furthermore, this implementation does not (yet) take care of the
   directionally sensitive motion detector (DSMD) as described in the
   paper. Thus, it does not quite suppress lateral motion. This means
   that any motion detected by the model, whether on a collision course
   or not, will result in LGMD spikes, which, of course, is not faithful
   to how the actual LGMD in real locusts operates.

   This is alright in our case because a separate portion of the
   Lobot/Robolocust framework takes care of integrating multiple LGMD
   signals into a coherent steering angle. It is this integration
   algorithm part that will suppress lateral motion and other ill
   effects that might drive our locust-controlled robot astray.
*/
class StaffordModel : public LocustModel {
   // Prevent copy and assignment
   StaffordModel(const StaffordModel&) ;
   StaffordModel& operator=(const StaffordModel&) ;

   // Handy type to have around
   typedef LocustModel base ;

   // Boilerplate code to make the factory work
   friend  class subfactory<StaffordModel, base, base::InitParams> ;
   typedef register_factory<StaffordModel, base, base::InitParams> my_factory ;
   static  my_factory register_me ;

   /**
      \class StaffordModel::layer

      The Stafford model uses a multi-layer neural network to compute
      LGMD spikes. Some of the connections between the layers are delayed
      by one time-step. This convenience structure holds the image for
      the current and previous time-stpes as it propagates through the
      layers.
   */
   struct layer {
      GrayImage previous ;
      GrayImage current ;
   } ;

   /// As mentioned above, the Stafford model computes LGMD spikes using
   /// a multi-layer neural network. The L-layer is not actually part of
   /// the Stafford model. We use it for the sake of convenience. It
   /// caches the luminance values of the composited input image from the
   /// various video sources Lobot/Robolocust reads from.
   ///
   /// The P-layer corresponds to the photoreceptor layer described in
   /// the Stafford paper. This layer is responsible for computing the
   /// basic motion detection using a simple subtraction between the
   /// previous and current images in the L-layer.
   ///
   /// The I-layer is the inhibition layer. It convolves the results of
   /// the P-layer from the current and previous time-steps with a fixed
   /// 3x3 kernel.
   ///
   /// The S-layer is the summation layer that takes the results from the
   /// P and I layers and produces an LGMD membrane potential by a simple
   /// summation. However, prior to the summation, negative values in the
   /// S-layer are discarded.
   ///
   /// Multiple instances of the Stafford model will all share the same
   /// source of input images with the only real difference between the
   /// instances being the subportion of the input image each one reads.
   /// This is done so as to simulate multiple locusts looking in
   /// different directions with limited (but overlapping) fields of
   /// view. Nonetheless, since they all share the same source, it makes
   /// sense for all instances to also share the different layers.
   ///
   /// This allows us to perform the computations for each layer just
   /// once. Then, each instance can read its assigned subportion of the
   /// S-layer to compute its LGMD potential.
   //@{
   static layer l_layer ;
   static layer p_layer ;
   static layer i_layer ;
   static layer s_layer ;
   //@}

   /// To save on the amount of number-crunching involved in computing
   /// LGMD spikes using a multi-layer neural net, we have all instances
   /// of the Stafford model share the different layers and perform the
   /// necessary subtractions, convolutions and summations just once.
   /// Then, each instance reads its assigned subportion of the final
   /// layer of the neural net to compute the LGMD membrane potential for
   /// itself.
   ///
   /// To get this setup to work, we need to keep track of the number of
   /// instances of this class and the number of instances that have been
   /// updated. Only the first one to be updated will result in the layer
   /// computations. And the last one will result in the layers being
   /// reset for the next round.
   //@{
   static int m_instances ;
   static int m_layers_computed ;
   //@}

   /// Private constructor because this model is instantiated using a
   /// factory and accessed solely through the interface provided by its
   /// abstract base class.
   StaffordModel(const base::InitParams&) ;

   /// These methods perform the LGMD computations.
   //@{
   void update() ;
   void compute_layers() ;
   void reset_layers() ;
   void prime_previous_layers() ;
   bool suppress_lgmd(const float spikes[], const float potentials[]) ;

   template<typename rect_dim, typename rect_comp, typename pot_comp>
   float compute_dsmd_potential(rect_dim, rect_comp, pot_comp) ;
   //@}

   /// Private destructor because this model is instantiated using a
   /// factory and accessed solely through the interface provided by its
   /// abstract base class.
   ~StaffordModel() ;

   // In addition to the LGMD, the Stafford model also implements four
   // directionally sensitive motion detectors (DSMD neurons). And
   // variations of it use a feed-forward inhibition neuron as well. It is
   // convenient to club potentials, etc. for all these neurons into a
   // single array indexed by the following symbols.
   enum {
      LGMD,      FFI,
      DSMD_LEFT, DSMD_RIGHT,
      DSMD_UP,   DSMD_DOWN,
      NUM_NEURONS
   } ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the LGMD model implemented by the
   /// StaffordModel class.
   class Params : public singleton<Params> {
      // Initialization
      Params() ; // private because this is a singleton
      friend class singleton<Params> ;

      /// LGMD, FFI and DSMD spike counts are computed by scaling down
      /// their raw membrane potentials to a number in the range [.5,1]
      /// and then counting one spike when that number exceeds the
      /// corresponding threshold in the following array.
      float m_spike_thresholds[NUM_NEURONS] ;

      /// To decide whether the LGMD neural network should emit a spike
      /// in response to the stimuli detected by each of the component
      /// neurons (i.e., the LGMD, the FFI cell and the 4 DSMDs), we can
      /// combine their individual spikes using a weighted sum.
      ///
      /// NOTE: This weighted sum procedure is an invention of the author
      /// of this class and is not described in any of the LGMD related
      /// papers by Stafford, Blanchard, Yue, Rind and gang. It can be
      /// turned off by setting the weight for the LGMD to one and all
      /// the others to zero.
      float m_spike_weights[NUM_NEURONS] ;

      /// The formula used to scale the raw membrane potentials down to
      /// the [.5,1] range is the following sigmoid function:
      ///
      ///                                       1
      ///                           u = -----------------
      ///                               1 + exp(-U/(n*N))
      ///
      /// where U is the raw membrane potential obtained by summing the
      /// relevant portion(s) of the S-layer; N is the total number of
      /// pixels in the locust's FOV; and n is a fudge factor defined
      /// below.
      ///
      /// This term acts as a scaling factor that magnifies the total
      /// visual area of our virtual locust. It doesn't necessarily make
      /// any real sense; it's really just a knob that seems to get
      /// things working a wee bit smoother when twiddled just right.
      ///
      /// Since the DSMD membrane potentials are computed using a
      /// multiplication and addition, they tend to be quite large
      /// numbers (usually in the order of millions). This makes it
      /// necessary to quell them quite a bit compared to the LGMD
      /// membrane potential. Thus, we use an array of area magnification
      /// factors instead of just one for all the neurons.
      float m_area_magnifiers[NUM_NEURONS] ;

      /// The LGMD spike rate is computed as a running average of the
      /// spike count using the usual formula:
      ///       running average = w*current + (1-w)*old
      ///
      /// The following parameter specifies the value of w in the above
      /// formula. It should be a number between 0 and 1. Higher values
      /// indicate greater confidence in the current readings while lower
      /// ones give greater weight to older readings.
      ///
      /// For this model, the raw membrane potentials tend to be quite
      /// jumpy. So lower running average weights tend to do a better job
      /// of smoothing out the overall spike rate.
      float m_running_average_weight ;

      /// Since the FFI and DSMDs don't seem to work too well, they can
      /// be turned off.
      bool m_ffi_on ;
      bool m_dsmd_on ;

      /// If the DSMDs are on, the following thresholds will be applied
      /// to check for horizontal and vertical lateral motions. Here's
      /// how these thresholds work: let us say that we want to check for
      /// horizontal lateral motion and its threshold is 10. Then, we
      /// will measure the membrane potentials of the left and right
      /// DSMDs and if one is greater than the other by a factor of 10,
      /// we conclude that there is lateral motion in the horizontal
      /// direction. Ditto for the vertical case.
      float m_horizontal_motion_threshold ;
      float m_vertical_motion_threshold ;

      /// The DSMD potential is computed by using blocks of pixels in the
      /// S-layer that run across the center of the entire image along
      /// both axes. In their paper, Stafford et al. use 10x10 blocks. In
      /// our case, we try to do that too. However, if this size doesn't
      /// quite work out, we fall back to an alternative (smaller) size
      /// to be able to cover the entire image width and height while
      /// still getting at least 2-3 EMDs.
      int m_ideal_dsmd_block_size ;
      int m_alt_dsmd_block_size ;

   public:
      // Accessing the various parameters
      static const float* spike_thresholds() ;
      static const float* spike_weights() ;
      static const float* area_magnifiers() ;
      static float running_average_weight() ;
      static bool ffi_on() ;
      static bool dsmd_on() ;
      static float horizontal_motion_threshold() ;
      static float vertical_motion_threshold() ;
      static int ideal_dsmd_block_size() ;
      static int alt_dsmd_block_size() ;

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
