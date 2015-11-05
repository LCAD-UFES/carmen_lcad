/*! \file Neuro/GistEstimatorContextBased.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorContextBased.C $
// $Id: GistEstimatorContextBased.C 13065 2010-03-28 00:01:00Z itti $
//

//------------------------------ HEADERS --------------------------------

// Gist specific headers
#include "Neuro/GistEstimatorContextBased.H"

// Other INVT headers
#include "Neuro/VisualCortex.H"
#include "Neuro/NeuroSimEvents.H"

#include "Simulation/SimEventQueue.H"

#include "Channels/GaborChannel.H"
#include "Channels/OrientationChannel.H"
#include "Channels/RawVisualCortex.H"

#include "Image/CutPaste.H"
#include "Image/MathOps.H"
#include "Image/Point2D.H"
#include "Image/Dims.H"

#include "nub/ref.h"
#include "rutz/shared_ptr.h"

// Standard C++ headers
#include <vector>
#include <algorithm>
#include <cmath>
#include <ctime>

//-------------------------- INITIALIZATION -----------------------------

GistEstimatorContextBased::
GistEstimatorContextBased(OptionManager& mgr,
                          const std::string& descrName,
                          const std::string& tagName)
   : GistEstimatorAdapter(mgr, descrName, tagName),
     SIMCALLBACK_INIT(SimEventVisualCortexOutput),
     itsGistVector(GRID_SIZE * GRID_SIZE * NUM_FILTERS, 1, ZEROS)
{}

//----------------------------- CLEAN-UP --------------------------------

GistEstimatorContextBased::~GistEstimatorContextBased()
{}

//------------------ GIST FEATURE VECTOR COMPUTATION --------------------

namespace { // prevent global namespace pollution and possible linker errors

// Some useful shortcuts
typedef GistEstimatorContextBased::PixelType PixelType ;
typedef GistEstimatorContextBased::ImageType ImageType ;

// As per the Torralba paper, their wavelet image decompositions are
// equivalent to Gabor filters applied at different orientations and
// scales. Thus, the following function simply retrieves the Gabor
// channel for the specified orientation and scale.
ImageType
apply_gabor_filter(const RawVisualCortex* vc, uint orientation, uint scale)
{
   nub::soft_ref<OrientationChannel> oc ;
   dynCastWeakToFrom(oc, vc->subChan("orientation")) ;
   GaborChannel& gc = oc->gabor(orientation) ;
   return gc.getImage(scale) ;
}

// The following function divides the supplied filtered image into the
// specified grid sizes and returns the average pixel values for each of
// these subimages.
std::vector<double> grid_averages(const ImageType& I, const Dims& grid_size)
{
   const int M = GistEstimatorContextBased::GRID_SIZE ;
   std::vector<double> averages(M*M) ;

   int A = 0 ;
   for (int y = 0, Y = 0; y < M; ++y, Y += grid_size.h())
      for (int x = 0, X = 0; x < M; ++x, X += grid_size.w())
      {
         ImageType sub = crop(I, Point2D<int>(X, Y), grid_size, true) ;
         averages[A++] = mean(sub) ;
      }

   return averages ;
}

} // end of local namespace encapsulating above helpers

// The evolve method filters the "current" image passed in by the INVT
// simulation framework and computes this image's gist vector. To compute
// this vector, it first applies Gabor filters to the input image at
// different orientations and scales. Then, it subdivides each of the
// filteration results into smaller "chunks" and populates the gist
// vector with the average pixel values in these coarse chunks.
void GistEstimatorContextBased::
onSimEventVisualCortexOutput(SimEventQueue& q, rutz::shared_ptr<SimEventVisualCortexOutput>& e)
{
  ///////////// VisualCortex* vc = dynamic_cast<VisualCortex*>(e->source()) ;

  const double G = GRID_SIZE ;
  const int N = GRID_SIZE * GRID_SIZE ;
  Image<double>::iterator gist_vector = itsGistVector.beginw() ;

  clock_t start_time = clock() ;
  for (uint orientation = 0; orientation < NUM_ORIENTATIONS; ++orientation)
    for (uint scale = 0; scale < NUM_SCALES; ++scale, gist_vector += N)
      {

        LFATAL("Please talk to Laurent to fix this");

        ImageType I; /////// = apply_gabor_filter(vc, orientation, scale) ;
        //LINFO("Gabor filter [O:%u, S:%u] returned %dx%d image",
        //orientation, scale, I.getWidth(), I.getHeight()) ;

        Dims grid_size(static_cast<int>(std::ceil(I.getWidth()/G)),
                       static_cast<int>(std::ceil(I.getHeight()/G))) ;
        //LINFO("computing averages for %dx%d subimages of filtered image",
        //grid_size.w(), grid_size.h()) ;
        std::vector<double> averages = grid_averages(I, grid_size) ;

        //LINFO("copying %d averages to gist vector at offset %d",
        //int(averages.size()),
        //int(gist_vector - itsGistVector.beginw())) ;
        std::copy(averages.begin(), averages.end(), gist_vector) ;
      }
  LINFO("%g seconds to compute %dx%d gist vector",
        static_cast<double>(clock() - start_time)/CLOCKS_PER_SEC,
        itsGistVector.getHeight(), itsGistVector.getWidth()) ;

  rutz::shared_ptr<SimEventGistOutput>
    gist_event(new SimEventGistOutput(this, itsGistVector)) ;
  q.post(gist_event) ;
}

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
