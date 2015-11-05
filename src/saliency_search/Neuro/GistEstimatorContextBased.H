/*!
   \file Neuro/GistEstimatorContextBased.H
   \brief Implementation of ``Context-based vision system for place and
   object recognition'' by Torralba, et al.

   The GistEstimatorContextBased class implements (within the INVT
   framework) the gist related portions of the following paper:

   Torralba, A., Murphy, K. P., Freeman, W. T., Rubin, M. A.
   Context-based vision system for place and object recognition.
   Ninth IEEE International Conference on Computer Vision,
   1:273--280, 2003.

   In the paper, the authors use a wavelet image decomposition tuned to 6
   orientations and 4 scales to compute texture features. However, this
   class uses Gabor filters (with the same number of orientations and
   scales) because INVT readily provides them and, as per the authors,
   the two approaches produce similar results.

   To compute the gist vector for an image, we apply the 24 Gabor
   filters and then subdivide each resulting image into a 4x4 grid. Then,
   we compute the mean pixel value in each grid. This gives us 16 numbers
   per filter. Since there are 24 filters in all, we get 16x24 = 384
   numbers per image. These 384 numbers form the gist vector for the
   input image.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorContextBased.H $
// $Id: GistEstimatorContextBased.H 11104 2009-04-13 18:36:37Z lzc $
//

#ifndef NEURO_GIST_ESTIMATOR_CONTEXT_BASED_H_DEFINED
#define NEURO_GIST_ESTIMATOR_CONTEXT_BASED_H_DEFINED

//------------------------------ HEADERS --------------------------------

// Gist specific headers
#include "Neuro/GistEstimator.H"

// Other INVT headers
#include "Neuro/NeuroSimEvents.H"

// Standard C++ headers
#include <string>

//------------------------- CLASS DEFINITION ----------------------------

/*!
   \class GistEstimatorContextBased
   \brief Gist estimator for ``Context-based vision system...'' by
   Torralba, et al.

   This class computes the gist vector for an input image using the
   technique described in section 2 of ``Context-based vision system for
   place and object recognition'' by Torralba, et al.

   In their paper, the authors apply 24 filters to the image (6
   orientations, 4 scales) and average the results over 4x4 grids to get
   a total of 384 texture features which comprise the gist vector. This
   class computes and returns this vector of 384 numbers.
*/
class GistEstimatorContextBased : public GistEstimatorAdapter {
public:
   static const uint NUM_ORIENTATIONS = 6 ;
   static const uint NUM_SCALES = 4 ;
   static const uint NUM_FILTERS = NUM_ORIENTATIONS * NUM_SCALES ;
   static const uint GRID_SIZE = 4 ;
   static const uint NUM_FEATURES = GRID_SIZE * GRID_SIZE * NUM_FILTERS ;

   typedef float PixelType ;
   typedef Image<PixelType> ImageType ;

   //! The constructor expects to be passed an option manager, which it
   //! uses to set itself up in the INVT simulation framework.
   GistEstimatorContextBased(OptionManager& mgr,
                             const std::string& descrName =
                                "GistEstimatorContextBased",
                             const std::string& tagName =
                                "GistEstimatorContextBased") ;
   virtual ~GistEstimatorContextBased() ;

   //! This method returns the gist vector computed for the input image.
   //! The gist vector consists of 384 numbers that are obtained by
   //! averaging the pixel values in each of the 16 subimages of each of
   //! the 24 filteration results.
   Image<double> getGist() ;

protected:
  //! Callback for when new data is available from Visuc=alCortex
  SIMCALLBACK_DECLARE(GistEstimatorContextBased, SimEventVisualCortexOutput);

private :
   Image<double> itsGistVector ; // gist feature vector
} ;

//-------------------- INLINE FUNCTION DEFINITIONS ----------------------

inline Image<double> GistEstimatorContextBased::getGist()
{
   return itsGistVector ;
}

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
