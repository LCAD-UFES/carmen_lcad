/**
   \file Neuro/GistEstimatorBeyondBoF.H

   \brief Implementation of ``Beyond Bags of Features: Spatial Pyramid
   Matching for Recognizing Natural Scene Categories'' by Lazebnik, et
   al.

   The GistEstimatorBeyondBoF class implements the following paper
   within the INVT framework:

   Lazebnik, S., Schmid, C., Ponce, J.
   Beyond Bags of Features: Spatial Pyramid Matching for Recognizing
      Natural Scene Catgories
   CVPR, 2006.

   In the paper, the authors describe the use of weak features (oriented
   edge points) and strong features (SIFT descriptors) as the basis for
   classifying images. In this implementation, however, we only concern
   ourselves with strong features clustered into 200 categories (i.e.,
   the vocabulary size is 200 ``words''). Furthermore, we restrict the
   spatial pyramid used as part of the matching process to 2 levels.

   We restrict ourselves to the above configuration because, as Lazebnik
   et al. report, it yields the best results (actually, a vocabulary
   size of 400 is better, but not by much).

   To compute the gist vector for an image, we first divide the image
   into 16x16 pixel patches and compute SIFT descriptors for each of
   these patches. We then assign these descriptors to bins corresponding
   to the nearest of the 200 SIFT descriptors (vocabulary) gleaned from
   the training phase. This grid of SIFT descriptor indices is then
   converted into a feature map that specifies the grid coordinates for
   each of the 200 feature types. This map allows us to compute the
   multi-level histograms as described in the paper. The gist vectors we
   are interested in are simply the concatenation of all the multi-level
   histograms into a flat array of numbers.

   Once we have these gist vectors, we can classify images using an SVM.
   The SVM kernel is the histogram intersection function, which takes the
   gist vectors for the input and training images and returns the sum of
   the minimums of each dimension (once again, see the paper for the gory
   details).

   This class, viz., GistEstimatorBeyondBoF, only computes gist vectors
   (i.e., normalized multi-level histograms) given the 200 ``word''
   vocabulary of SIFT descriptors to serve as the bins for the
   histograms. The actual training and classification must be performed
   by client programs. To assist with the training process, however, this
   class sports a training mode in which it does not require the SIFT
   descriptors database. Instead, in training mode, it simply returns the
   raw grid of SIFT descriptors for the images it is supplied. This
   allows clients to store those descriptors for clustering, etc.
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorBeyondBoF.H $
// $Id: GistEstimatorBeyondBoF.H 11138 2009-04-22 22:02:38Z mviswana $
//

#ifndef GE_BEYOND_BAGS_OF_FEATURES_H_DEFINED
#define GE_BEYOND_BAGS_OF_FEATURES_H_DEFINED

//------------------------------ HEADERS --------------------------------

// Gist specific headers
#include "Neuro/GistEstimator.H"

// Other INVT headers
#include "SIFT/Keypoint.H"
#include "Neuro/NeuroSimEvents.H"

// Standard C++ headers
#include <ostream>
#include <list>
#include <string>

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class GistEstimatorBeyondBoF
   \brief Gist estimator for ``Beyond Bags of Features ...'' by Lazebnik,
   et al.

   This class computes the gist vector for an input image using the
   feature extraction and spatial pyramid matching scheme described in
   sections 4 and 3 (respectively) of ``Beyond Bags of Features: Spatial
   Pyramid Matching for Recognizing Natural Scene Categories'' by
   Lazebnik, et al.

   While the authors of the above-mentioned paper experiment with weak
   features (oriented edge points) and strong features (SIFT descriptors)
   and with different resolutions of the spatial matching pyramid, this
   class only implements strong features clustered into 200 categories
   and a two-level pyramid because other configurations were either not
   as good as this one or did not offer any significant advantages over
   it (as reported in the paper).

   Thus, utilizing the terminology employed by Lazebnik, et al., we have
   the number of channels M = 200 and the number of levels of the spatial
   matching pyramid L = 2. This will result in gist vectors of
   dimensionality:

      M * (4^(L+1) - 1)/3 = M * (2^(2L+2) - 1)/3
                          = 200 * (2^(2*2+2) - 1)/3
                          = 200 * (2^6 - 1)/3
                          = 200 * 63/3
                          = 200 * 21
                          = 4200

   See the paper for the gory details.
*/
class GistEstimatorBeyondBoF : public GistEstimatorAdapter {
   // This ought to work fine, but for some weird reason, doesn't. The
   // 32-bit nightly builds are carried out by GCC version 3.4.1 and it
   // compiles this and the .C files fine. But the linker chokes and
   // complains about undefined references to these variables. This does
   // not seem to be a problem with newer versions of GCC (>= 4.2).
   //
   // Oddly enough, other classes (e.g., GistEstimatorContextBased and
   // GistEstimatorTexton) that also define static const members do not
   // suffer from this affliction. But those classes define static const
   // uints rather than static const ints. Still, this doesn't make
   // sense!
   //
   // Anyhoo, just to resolve this issue: we explicitly define these
   // static data members in the .C file.
   //static const int NUM_CHANNELS = 200 ;
   //static const int NUM_LEVELS   = 2 ;
   static int NUM_CHANNELS ;
   static int NUM_LEVELS   ;
   static int GIST_VECTOR_SIZE ;

public:
   /// Accessors for some parameters used internally by the Lazebnik
   /// algorithm.
   ///@{
   static int num_channels()     {return NUM_CHANNELS ;}
   static int num_levels()       {return NUM_LEVELS ;}
   static int gist_vector_size() {return GIST_VECTOR_SIZE ;}
   ///@}

   /// Modifiers for some parameters used internally by the Lazebnik
   /// algorithm.
   ///
   /// WARNING: These methods should be used with care. Do not call them
   /// to change the size of the SIFT vocabulary and spatial pyramid size
   /// in the "middle" of an "entire" run. That is, if you use 100
   /// channels and a 4-level pyramid for training, don't switch to 200
   /// channels and a 3-level pyramid during the testing phase!
   ///@{
   static void num_channels(int) ;
   static void num_levels(int) ;
   ///@}

   /// The constructor expects to be passed an option manager, which it
   /// uses to set itself up within the INVT simulation framework.
   GistEstimatorBeyondBoF(OptionManager& mgr,
      const std::string& descrName = "GistEstimatorBeyondBoF",
      const std::string& tagName   = "GistEstimatorBeyondBoF") ;

   /// A SIFT descriptor is just a vector of 128 numbers. The following
   /// inner class provides a convenient wrapper for this vector.
   struct SiftDescriptor {
      static const int SIZE = 128 ;

      SiftDescriptor() ;
      SiftDescriptor(const Image<float>&) ;
      SiftDescriptor(const rutz::shared_ptr<Keypoint>&) ;

      // WARNING: NO RANGE CHECK! i better be in [0, 128).
      float& operator[](int i) {return values[i] ;}
      const float& operator[](int i) const {return values[i] ;}
   private :
      float values[SIZE] ;
   } ;

   /// Like other gist estimators, this one too filters the input image.
   /// Its filteration process involves subdividing the input image into
   /// 16x16 pixel patches and running SIFT on each of these patches. The
   /// filteration results are, therefore, a grid of SIFT descriptors.
   /// The following type is used to represent these results.
   typedef Image<SiftDescriptor> SiftGrid ;

   /// In order to compute a gist vector, this estimator needs to know
   /// the vocabulary associated with the bag of features. This
   /// vocabulary is usually obtained by a clustering process as part of
   /// the training. Each ``word'' or ``vis-term'' in this vocabulary is
   /// also known as a channel. The gist vector essentially represents a
   /// ``spatial histogram'' for each of these channels.
   ///
   /// The vocabulary itself is merely a collection of 200 SIFT
   /// descriptors (the centroids of the clusters) passed in via an
   /// Image of floating point numbers. Thus, the size of this Image
   /// would be 128x200. (The 128 comes from the dimensionality of a
   /// SIFT descriptor.)
   typedef std::list<SiftDescriptor> Vocabulary ;

   /// This method should be called once during the client's
   /// initialization process prior to attempting to obtain gist
   /// vectors for input images. Thus, the clustering phase of the
   /// training must be complete before this estimator can be used to
   /// compute gist vectors.
   void setVocabulary(const Image<float>&) ;

   /// To assist with training, GistEstimatorBeyondBoF can be configured
   /// to operate in a special training mode in which it does not have a
   /// vocabulary from which to form gist vectors but rather simply
   /// passes back (to its client) the grid of SIFT descriptors for the
   /// input image, i.e., the results of the filteration step. The
   /// client may then store these descriptors, perform the clustering
   /// required to create the vocabulary necessary for subsequent normal
   /// use of this gist estimator, and then run the estimator in
   /// non-training mode to compute the actual gist vectors.
   ///
   /// Training mode is set by specifying a hook function that accepts
   /// the filteration results, i.e., the grid/Image of SIFT
   /// descriptors.
   typedef void (*TrainingHook)(const SiftGrid&) ;

   /// This method should be called once during the client's
   /// initialization sequence to specify the training mode hook
   /// function to configure GistEstimatorBeyondBoF to run in training
   /// mode. If this hook is not specified, the estimator will run in
   /// ``normal'' mode and compute gist vectors from the vocabulary.
   ///
   /// It is an error to not specify either the training hook or the
   /// vocabulary. If both are specified, the training hook takes
   /// precedence, i.e., the estimator runs in training mode, wherein it
   /// passes back filteration results (grid of SIFT descriptors) to the
   /// client rather than computing gist vectors.
   void setTrainingHook(TrainingHook) ;

   /// Return the gist vector (useless in training mode).
   Image<double> getGist() ;

   /// Destructor
   virtual ~GistEstimatorBeyondBoF() ;

protected:
  /// Callback for when a new input (retina) frame is available
  SIMCALLBACK_DECLARE(GistEstimatorBeyondBoF, SimEventRetinaImage);

private :
   Image<double> itsGistVector ; // gist feature vector
   Vocabulary    itsVocabulary ;
   TrainingHook  itsTrainingHook ;
} ;

//---------------------- MISCELLANEOUS FUNCTIONS ------------------------

std::ostream& operator<<(std::ostream&,
                         const GistEstimatorBeyondBoF::SiftDescriptor&) ;

//-------------------- INLINE FUNCTION DEFINITIONS ----------------------

inline Image<double> GistEstimatorBeyondBoF::getGist()
{
   return itsGistVector ;
}

inline void
GistEstimatorBeyondBoF::
setTrainingHook(GistEstimatorBeyondBoF::TrainingHook H)
{
   itsTrainingHook = H ;
}

//-----------------------------------------------------------------------

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
