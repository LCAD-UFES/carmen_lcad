/*!@file Neuro/GistEstimatorTexton.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorTexton.H $
// $Id: GistEstimatorTexton.H 11104 2009-04-13 18:36:37Z lzc $
//

#ifndef NEURO_GISTESTIMATORTEXTON_H_DEFINED
#define NEURO_GISTESTIMATORTEXTON_H_DEFINED

//------------------------------ HEADERS --------------------------------

// Gist specific headers
#include "Neuro/GistEstimator.H"

// Other INVT headers
#include "Neuro/NeuroSimEvents.H"

// Standard C++ headers
#include <string>

//------------------------- CLASS DEFINITION ----------------------------

class GistEstimatorTexton : public GistEstimatorAdapter {
public:
   //! Constructor
   GistEstimatorTexton(OptionManager& mgr,
                       const std::string& descrName = "GistEstimatorTexton",
                       const std::string& tagName = "GistEstimatorTexton") ;

   static const uint NUM_ORIENTATIONS = 6 ;
   static const uint NUM_SCALES = 3 ;
   static const uint NUM_FILTERS = NUM_ORIENTATIONS * NUM_SCALES ;

   typedef float PixelType ;
   typedef Image<PixelType> ImageType ;

   //! Destructor
   virtual ~GistEstimatorTexton() ;

protected:
   //! Return the gist feature values
   Image<double> getGist() ;

   // Texton-based gist computations require a "database" of universal
   // textons in place.
   static void setUniversalTextons(const ImageType*) ;

   // Training related settings
   typedef void (*TrainingHook)(const ImageType&) ;
   void setTrainingHook(TrainingHook) ;

  //! Callback for when new data is available from Visuc=alCortex
  SIMCALLBACK_DECLARE(GistEstimatorTexton, SimEventVisualCortexOutput);

private :
   Image<double> itsGistVector ; // gist feature vector
   static const ImageType* itsUniversalTextons ;
   TrainingHook itsTrainingHook ;
} ;

//-------------------- INLINE FUNCTION DEFINITIONS ----------------------

inline Image<double> GistEstimatorTexton::getGist()
{
   return itsGistVector ;
}

inline void
GistEstimatorTexton::
setUniversalTextons(const GistEstimatorTexton::ImageType* U)
{
   itsUniversalTextons = U ;
}

inline void
GistEstimatorTexton::
setTrainingHook(GistEstimatorTexton::TrainingHook H)
{
   itsTrainingHook = H ;
}

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
