/*!@file Neuro/GistEstimatorTexton.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorTexton.C $
// $Id: GistEstimatorTexton.C 13103 2010-03-31 02:24:47Z itti $
//

//------------------------------ HEADERS --------------------------------

// Gist specific headers
#include "Neuro/GistEstimatorTexton.H"
//#include "Neuro/gistParams.H"

// Other INVT headers
#include "Neuro/VisualCortex.H"
#include "Neuro/NeuroSimEvents.H"

#include "Simulation/SimEventQueue.H"

#include "Channels/GaborChannel.H"
#include "Channels/OrientationChannel.H"

#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Image/Dims.H"

#include "nub/ref.h"
#include "rutz/shared_ptr.h"

// Standard C++ headers
#include <sstream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <utility>
#include <limits>
#include <cmath>
#include <ctime>

//----------------------------- TYPEDEFS --------------------------------

// Some useful shortcuts
typedef GistEstimatorTexton::PixelType PixelType ;
typedef GistEstimatorTexton::ImageType ImageType ;
typedef std::vector<ImageType> FilterationResults ;

//------------------------ STATIC DATA MEMBERS --------------------------

// The GistEstimatorTexton relies on its client to load the universal
// textons prior to using the textons computation facility. We could load
// the universal textons here and thereby not bother the client with this
// chore. However, client programs may not be interested in figuring out
// which universal textons occur in a given input image. Instead, they
// may just be interested in getting their hands on the input image's
// "raw" textons via the training hook. (The train-texton program, for
// instance, has such a mode of operation.)
//
// If we were to always load the universal textons each time this class
// is used, it would always impose the universal textons load overhead
// even if the client is not interested in using that portion of this
// class. By moving the universal textons loading responsibility into
// client space, we avoid this unnecessary overhead.
//
// Moreover, this approach gives clients the flexibility to save textons
// to disk in whatever format they choose. All the GistEstimatorTexton
// needs is an Image specifying the universal textons regardless of how
// that Image is actually obtained.
//
// To ensure that clients properly set the universal textons, we require
// them to provide the address of such an Image rather than the contents.
// This allows us to check that this pointer is valid prior to performing
// the texton histogram computations.
const ImageType* GistEstimatorTexton::itsUniversalTextons ;

//-------------------------- INITIALIZATION -----------------------------

GistEstimatorTexton::GistEstimatorTexton(OptionManager& mgr,
                                         const std::string& descrName,
                                         const std::string& tagName)
   : GistEstimatorAdapter(mgr, descrName, tagName),
     SIMCALLBACK_INIT(SimEventVisualCortexOutput),
     itsTrainingHook(0)
{}

//----------------------------- CLEAN-UP --------------------------------

GistEstimatorTexton::~GistEstimatorTexton()
{}

//------------------ GIST FEATURE VECTOR COMPUTATION --------------------

// Forward declarations
namespace {

ImageType apply_texton_filter(const VisualCortex*, uint, uint) ;
ImageType compute_textons(const FilterationResults&) ;
Image<double> histogram(const ImageType&, const ImageType&) ;

}

// The processing method filters the "current" image passed in by the INVT
// simulation framework and computes this image's textons. Then,
// depending on whether we are in training mode or not, it either passes
// the textons to the texton trainer (src/Gist/train-texton.C) via the
// training hook or performs K-nearest neighbour search to figure out
// the instances of the universal textons in the input image.
//
// DEVNOTE: We already know how many filters we're going to be applying
// to the input image. So we ought to be able to initialize the
// filteration results (STL) vector with these many elements so as to
// avoid reallocations later on.
//
// Unfortunately, for some strange reason, preallocating this number of
// elements and then calling push_back() doesn't quite work. The vector
// ends up with the specified number of preallocated elements and then
// push_back() appends beyond this preallocated range!
//
// This is *not* standards-compliant behaviour. The vector reserve()
// method is not supposed to affect the vector's size; it should only
// change the vector's capacity. A small test program written for
// verification purposes confirmed this. Yet the application of the same
// technique here backfires. (Perhaps something to do with the INVT build
// environment?)

// ######################################################################
void GistEstimatorTexton::
onSimEventVisualCortexOutput(SimEventQueue& q, rutz::shared_ptr<SimEventVisualCortexOutput>& e)
{
  LFATAL("FIXME, this should be done using a SimReq");
  /*
  VisualCortex* vc = dynamic_cast<VisualCortex*>(e->source()) ;

  //FilterationResults results(NUM_FILTERS) ; // doesn't work; see above
  FilterationResults results ;
  for (uint orientation = 0; orientation < NUM_ORIENTATIONS; ++orientation)
    for (uint scale = 0; scale < NUM_SCALES; ++scale)
      results.push_back(apply_texton_filter(vc, orientation, scale)) ;

  ImageType textons = compute_textons(results) ;
  LINFO("MVN: computed %dx%d texton \"matrix\" for input image",
        textons.getHeight(), textons.getWidth()) ;
  if (itsTrainingHook)
    itsTrainingHook(textons) ;
  else
    {
      if (! itsUniversalTextons)
        throw std::runtime_error("GistEstimatorTexton requires "
                                 "universal textons \"database\"") ;
      clock_t start_time = clock() ;
      itsGistVector = histogram(textons, *itsUniversalTextons) ;
      LINFO("MVN: %g seconds to compute histogram, i.e., %dx%d gist vector",
            static_cast<double>(clock() - start_time)/CLOCKS_PER_SEC,
            itsGistVector.getHeight(), itsGistVector.getWidth()) ;
    }

  rutz::shared_ptr<SimEventGistOutput>
    gistOutputEvent(new SimEventGistOutput(this, itsGistVector)) ;
  q.post(gistOutputEvent) ;
  */
}

//------------------------ TEXTON COMPUTATIONS --------------------------

// This section is local to this file. Thus, stuffing it in an anonymous
// namespace ensures that its definitions don't clash with identically
// named entities in other modules.
namespace {

// As per the Renninger and Malik paper, their texton filters are
// equivalent to Gabor filters applied at different orientations and
// scales. Thus, the following function simply retrieves the Gabor
// channel for the specified orientation and scale.
//
// One thing to keep in mind though is that at coarser scales, the
// filtered image will have a correspondingly smaller size. At scale 0,
// the filtered image is the same size as the input image; at scale 1, it
// is half the size of the input image; at scale 2, 1/4th the size of the
// input image; so on and so forth. Basically, at scale n, it will be
// 1/(2^n) the size of the input image.
//
// To get the texton for the (i,j)th pixel, we extract the (i,j)th pixel
// from each filter's resultant image. However, if the resultant and
// input images are different sizes, there may be no pixel to extract
// from the resultant image. For example, if the input image is 320x240
// pixels, at scale 2, the filtered image will be 80x60 pixels. And we
// won't be able to create the texton for all pixels beyond the 80th row
// and 60th column.
//
// To fix this problem, we rescale the filtered image back to the input
// image's size.
ImageType
apply_texton_filter(const VisualCortex* vc, uint orientation, uint scale)
{

  LFATAL("Please talk to Laurent to fix this");
  return ImageType();
  /*
   nub::soft_ref<OrientationChannel> oc ;
   dynCastWeakToFrom(oc, vc->subChan("orientation")) ;
   GaborChannel& gc = oc->gabor(orientation) ;
   ImageType I = gc.getImage(scale) ;
   if (scale > 0)
      I = rescale(I, I.getDims() * (1 << scale)) ; // blow up by 2^scale
   return I ;
  */
}

// Forward declaration
ImageType get_textons(int i, int j, const FilterationResults&) ;

// The following function returns the textons for the entire input image
// given the filteration results. The textons are returned as an NxR
// image where N is the number of filters applied and R is the product
// of the width and height of the input image. Thus, the textons Image
// returned by this function will have R (i.e., WxH) rows and N columns.
ImageType compute_textons(const FilterationResults& results)
{
   int width  = results[0].getWidth() ;
   int height = results[0].getHeight() ;
   ImageType textons(GistEstimatorTexton::NUM_FILTERS, width * height,
                     NO_INIT) ;
   int row = 0 ;
   for (int i = 0; i < width; ++i)
      for (int j = 0; j < height; ++j)
         inplacePaste(textons, get_textons(i, j, results),
                      Point2D<int>(0, row++)) ;

   return textons ;
}

// Quick helper to extract the (i,j)th pixel from a given Image.
//
// DEVNOTE: We could write this as a std::binary_function and then use
// it in conjunction with std::bind2nd(). But that doesn't really reduce
// the amount of code to be written here. Keeping this a unary_function
// and using it directly makes the intent somewhat clearer.
//
// DEVNOTE 2: Another possibility is to use std::mem_fun_ref() in
// conjunction with std::bind2nd(). Unfortunately, std::mem_fun_ref()
// doesn't work when the argument of its function call operator is a
// reference (compiler issues "reference to reference" error). That
// requires partial specialization of the std::binary_function so that
// the second argument is a reference.
//
// But that too doesn't really reduce the amount of code to be written.
// As a matter of fact, with this second approach, we have to write a
// lot more code because of the extra partial specialization plus several
// typedefs required to disambiguate the call to Image<T>::getVal().
//
// These extra bits of code simply serve to further obfuscate intent.
// Thus, it's best to just stick with this basic custom function object.
// It gets the job done and makes fairly clear what's going on.
class get_pixel : std::unary_function<ImageType, PixelType> {
   Point2D<int> coordinates ;
public :
   get_pixel(int i, int j) : coordinates(i, j) {}
   PixelType operator()(const ImageType& I) const {
      return I.getVal(coordinates) ;
   }
} ;

// A texton is simply the vector of filter responses for a given pixel.
// That is, if we apply 36 filters to an input image, we will get 36
// Images as the filteration results. The texton for pixel (i,j) will be
// the vector of 36 numbers formed by taking pixel (i,j) from each of the
// 36 Images in the filteration results.
//
// The following function returns the textons corresponding to the
// (i,j)th pixel of the input image given the filteration results. The
// textons for this pixel are returned as an Nx1 Image, i.e., 1 row of N
// values (where N is the number of filters applied).
ImageType get_textons(int i, int j, const FilterationResults& images)
{
   ImageType textons(GistEstimatorTexton::NUM_FILTERS, 1, NO_INIT) ;
   std::transform(images.begin(), images.end(), textons.beginw(),
                  get_pixel(i, j)) ;
   return textons ;
}

} // end of local namespace encapsulating above definitions

//---------------------- HISTOGRAM COMPUTATIONS -------------------------

namespace {

// Quick helper to extract the r-th row of an Image
inline ImageType get_row(int r, const ImageType& I)
{
   return crop(I, Point2D<int>(0, r), Dims(I.getWidth(), 1)) ;
}

// Forward declarations
int nearest_universal_texton(const ImageType&, const ImageType&) ;
Image<double> normalized_histogram(const std::vector<int>&, int) ;
double dist2(const ImageType&, const ImageType&, double) ;

// Given the set of input textons, I, and the universal textons, U, this
// function returns the normalized histogram counting the occurences of
// the supplied universal textons in I. This histogram is the "gist
// signature" of the input image and forms the basis for image
// classification.
Image<double> histogram(const ImageType& I, const ImageType& U)
{
   std::vector<int> counts(U.getHeight()) ;
   std::fill(counts.begin(), counts.end(), 0) ;

   for (int i = 0; i < I.getHeight(); ++i)
      ++counts[nearest_universal_texton(get_row(i, I), U)] ;

   return normalized_histogram(counts, I.getHeight()) ;
}

// Given a row of the input image and the set of universal textons, this
// function returns the index of the universal texton nearest to the
// input image row.
//
// DEVNOTE: The Renninger-Malik implementation uses the Netlab (Matlab)
// toolbox's knn function to compute the nearest universal texton (i.e.,
// k = 1). Rather than implement the K-nn algorithm here and call it
// with k = 1, we simply take a shortcut and perform a simple nearest
// neighbour test.
int nearest_universal_texton(const ImageType& I, const ImageType& U)
{
   double D ;

   std::pair<double, int> min(std::numeric_limits<double>::max(), -1) ;
   for (int i = 0; i < U.getHeight(); ++i)
      if ((D = dist2(I, get_row(i, U), min.first)) < min.first)
         min = std::make_pair(D, i) ;

   return min.second ;
}

// The following function returns the square of the Euclidean distance
// between two vectors subject to a supplied minimum, i.e., if the
// square distance exceeds the minimum, the computation is
// short-circuited and the minimum returned.
//
// This short-circuiting helps reduce the nearest universal texton
// computation by about 2 seconds. Without it, each image's histogram
// was taking over 8 seconds to compute no thanks to the O(nU) nature of
// the problem (where n is the number of input image textons and U the
// number of universal textons).
//
// The vectors are passed in as Images that are assumed to have a single
// row and the same width.
double dist2(const ImageType& L, const ImageType& R, double min)
{
   double D = 0 ;
   for (int i = 0; i < L.getWidth(); ++i) {
      double l = L.getVal(i, 0) ;
      double r = R.getVal(i, 0) ;
      D += (l-r) * (l-r) ;
      if (D > min)
         return min ;
   }
   return D ;
}

// The following function converts a vector of counts and the total of
// all those counts to an Image containing the normalized counts.
Image<double> normalized_histogram(const std::vector<int>& counts, int total)
{
   Image<double> I(counts.size(), 1, ZEROS) ;
   std::transform(counts.begin(), counts.end(), I.beginw(),
                  std::bind2nd(std::divides<double>(), total)) ;
   return I ;
}

} // end of local namespace encapsulating above definitions

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
