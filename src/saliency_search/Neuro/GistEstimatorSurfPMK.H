/**
   \file Neuro/GistEstimatorSurfPMK.H

   \brief Implementation of ``SURF features for efficient robot
   localization with omnidirectional images'' by Murillo, et al.

   The GistEstimatorSurfPMK class implements the following paper
   within the INVT framework:

   Murillo, A. C., Guerrero, J. J., Sagues, C.
   SURF features for efficient robot localization with omnidirectional images
   ICRA, 2007.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorSurfPMK.H $
// $Id: GistEstimatorSurfPMK.H 12068 2009-11-23 00:44:06Z mviswana $
//

#ifndef GE_SURF_PMK_H_DEFINED
#define GE_SURF_PMK_H_DEFINED

//--------------------------- LIBRARY CHECK -----------------------------

#ifndef INVT_HAVE_LIBSURF

// Gist specific headers
#include "Neuro/GistEstimator.H"

// Dummy class defnition
class GistEstimatorSurfPMK : public GistEstimatorAdapter {
public:
   GistEstimatorSurfPMK(OptionManager& mgr,
      const std::string& descrName = "GistEstimatorSurfPMK",
      const std::string& tagName   = "GistEstimatorSurfPMK") ;
   virtual ~GistEstimatorSurfPMK() ;

protected:
   Image<double> getGist() {return Image<double>() ;}
} ;

#else // regular SURF-PMK gist estimator

//------------------------------ HEADERS --------------------------------

// Gist specific headers
#include "Neuro/GistEstimator.H"

// surf headers
#include <opensurf/opensurf.hh>

// Standard C++ headers
#include <ostream>
#include <list>
#include <vector>
#include <string>

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class GistEstimatorSurfPMK

   \brief Gist estimator for ``SURF features for efficient robot
   localization with omnidirectional images'' by Murillo, et al.

   See the paper for the gory details.
*/
class GistEstimatorSurfPMK : public GistEstimator {
public:
   static const int GIST_VECTOR_SIZE ;

   /// The constructor expects to be passed an option manager, which it
   /// uses to set itself up within the INVT simulation framework.
   GistEstimatorSurfPMK(OptionManager& mgr,
      const std::string& descrName = "GistEstimatorSurfPMK",
      const std::string& tagName   = "GistEstimatorSurfPMK") ;

   /// Destructor
   virtual ~GistEstimatorSurfPMK() ;

   /// A SURF descriptor is just a vector of 64 numbers.
   static const int SURF_DESCRIPTOR_SIZE = 64 ;

   /// This type provides a convenient alias for a SURF descriptor's
   /// vector of 64 numbers.
   typedef opensurf::IPoint SurfDescriptor ;

   /// Like other gist estimators, this one too filters the input image.
   /// Its filteration process involves running SURF on the input image.
   /// The filteration results are returned as a grid of SURF
   /// descriptors. The following type is used to represent these
   /// results.
   typedef std::vector<SurfDescriptor> SurfKeypoints ;

   /// In order to compute a gist vector, this estimator needs to know
   /// the vocabulary, i.e., the ``prototypical'' SURF descriptors as
   /// extracted by the hierarchical K-means clustering of the training
   /// descriptors.
   typedef std::list<SurfDescriptor> Vocabulary ;

   /// This method should be called once during the client's
   /// initialization process prior to attempting to obtain gist
   /// vectors for input images. Thus, the clustering phase of the
   /// training must be complete before this estimator can be used to
   /// compute gist vectors.
   void setVocabulary(const Image<float>&) ;

   /// To assist with training, GistEstimatorSurfPMK can be configured
   /// to operate in a special training mode in which it does not have a
   /// vocabulary from which to form gist vectors but rather simply
   /// passes back (to its client) the grid of SURF descriptors for the
   /// input image, i.e., the results of the filteration step. The
   /// client may then store these descriptors, perform the clustering
   /// required to create the vocabulary necessary for subsequent normal
   /// use of this gist estimator, and then run the estimator in
   /// non-training mode to compute the actual gist vectors.
   ///
   /// Training mode is set by specifying a hook function that accepts
   /// the filteration results, i.e., the grid/Image of SURF
   /// descriptors.
   typedef void (*TrainingHook)(const SurfKeypoints&) ;

   /// This method should be called once during the client's
   /// initialization sequence to specify the training mode hook
   /// function to configure GistEstimatorSurfPMK to run in training
   /// mode. If this hook is not specified, the estimator will run in
   /// ``normal'' mode and compute gist vectors from the vocabulary.
   ///
   /// It is an error to not specify either the training hook or the
   /// vocabulary. If both are specified, the training hook takes
   /// precedence, i.e., the estimator runs in training mode, wherein it
   /// passes back filteration results (grid of SURF descriptors) to the
   /// client rather than computing gist vectors.
   void setTrainingHook(TrainingHook) ;

   /// Return the gist vector (useless in training mode).
   Image<double> getGist() ;

protected:
  /// Callback for when a new input (retina) frame is available
  SIMCALLBACK_DECLARE(GistEstimatorSurfPMK, SimEventRetinaImage);

private :
   Image<double> itsGistVector ; // gist feature vector
   Vocabulary    itsVocabulary ;
   TrainingHook  itsTrainingHook ;
} ;

//---------------------- MISCELLANEOUS FUNCTIONS ------------------------

std::ostream& operator<<(std::ostream&,
                         const GistEstimatorSurfPMK::SurfDescriptor&) ;

//-------------------- INLINE FUNCTION DEFINITIONS ----------------------

inline Image<double> GistEstimatorSurfPMK::getGist()
{
   return itsGistVector ;
}

inline void
GistEstimatorSurfPMK::
setTrainingHook(GistEstimatorSurfPMK::TrainingHook H)
{
   itsTrainingHook = H ;
}

//-----------------------------------------------------------------------

#endif // #ifndef INVT_HAVE_LIBSURF
#endif // #ifndef GE_SURF_PMK_H_DEFINED

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
