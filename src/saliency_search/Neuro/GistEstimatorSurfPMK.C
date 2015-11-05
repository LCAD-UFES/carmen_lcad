/*!
   \file Neuro/GistEstimatorSurfPMK.C

   This file defines the member functions, static members, etc. of the
   GistEstimatorSurfPMK class. Further details are in the header file.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorSurfPMK.C $
// $Id: GistEstimatorSurfPMK.C 13065 2010-03-28 00:01:00Z itti $
//

//--------------------------- LIBRARY CHECK -----------------------------

#if !defined(INVT_HAVE_LIBSURF) || !defined(HAVE_OPENCV)

// Gist specific headers
#include "Neuro/GistEstimatorSurfPMK.H"

// INVT utils
#include "Util/log.H"

// Dummy class definition
GistEstimatorSurfPMK::
GistEstimatorSurfPMK(OptionManager& mgr,
                     const std::string& descrName, const std::string& tagName)
  : GistEstimatorAdapter(mgr, descrName, tagName)
{
   LFATAL("Sorry, GistEstimatorSurfPMK requires OpenSURF and OpenCV") ;
}

GistEstimatorSurfPMK::~GistEstimatorSurfPMK(){}

#else // the regular SURF-PMK gist estimator in all its hideous glory

//------------------------------ HEADERS --------------------------------

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

// Gist specific headers
#include "Neuro/GistEstimatorSurfPMK.H"

// Other INVT headers
#include "Neuro/VisualCortex.H"
#include "Neuro/NeuroSimEvents.H"

#include "Simulation/SimEventQueue.H"

#include "Image/Kernels.H"
#include "Image/Convolutions.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Image/Point2D.H"
#include "Image/Dims.H"

#include "Util/Timer.H"

#include "nub/ref.h"
#include "rutz/shared_ptr.h"

// Standard C++ headers
#include <sstream>
#include <numeric>
#include <algorithm>
#include <functional>
#include <map>
#include <list>
#include <stdexcept>
#include <utility>
#include <limits>
#include <ctime>

//------------------------------ DEFINES --------------------------------

// Error message for exceptions when vocabulary is required but not
// specified.
#define GE_SPMK_ERR_NO_VOCABULARY \
   "GistEstimatorSurfPMK requires vocabulary of " \
   "prototypical SURF descriptors"

//----------------------------- TYPEDEFS --------------------------------

// Some useful shortcuts
typedef GistEstimatorSurfPMK    SurfPMK ;
typedef SurfPMK::Vocabulary     Vocabulary ;
typedef SurfPMK::SurfDescriptor SurfDescriptor ;
typedef SurfPMK::SurfKeypoints  SurfKeypoints ;

// Other frequently used types
typedef float PixelType ;
typedef Image<PixelType> ImageType ;
typedef Image<double>    GistVectorType ;

//-------------- STATIC DATA MEMBERS AND OTHER CONSTANTS ----------------

const int GistEstimatorSurfPMK::GIST_VECTOR_SIZE = 200 ; // FIXME!

//-------------------------- INITIALIZATION -----------------------------

GistEstimatorSurfPMK::
GistEstimatorSurfPMK(OptionManager& mgr,
                     const std::string& descrName,
                     const std::string& tagName)
   : GistEstimator(mgr, descrName, tagName),
     SIMCALLBACK_INIT(SimEventRetinaImage),
     itsTrainingHook(0)
{}

// Quick helper to extract the r-th row of an Image
static inline ImageType get_row(int r, const ImageType& I)
{
   return crop(I, Point2D<int>(0, r), Dims(I.getWidth(), 1)) ;
}

// The vocabulary is a list of SURF descriptors. But for convenience,
// clients can pass it in as an image. The image must have 128 columns
// (for the 128 values that make up a SURF descriptor) and will usually
// have 200 rows (the size of the vocabulary). Thus, the dimensions of
// the input image used to represent the SURF descriptor vocabulary in
// client space will be 128x200.
//
// The following method simply converts this image into the list of SURF
// descriptors used to represent the vocabulary internally by this class.
void GistEstimatorSurfPMK::setVocabulary(const ImageType& V)
{
   itsVocabulary.clear() ;

   const int H = V.getHeight() ;
   for (int i = 0; i < H; ++i)
      //itsVocabulary.push_back(SurfDescriptor(get_row(i, V))) ;
      itsVocabulary.push_back(SurfDescriptor()) ; // FIXME!
}

//----------------------------- CLEAN-UP --------------------------------

GistEstimatorSurfPMK::~GistEstimatorSurfPMK(){}

//------------------ GIST FEATURE VECTOR COMPUTATION --------------------

// Forward declarations
static SurfKeypoints apply_surf_on_image(ImageType I) ;

// The processing method divides the current frame of the series of input
// images into 16x16 pixel patches and computes the SURF descriptors for
// each of these patches. Then, it either passes this grid of
// descriptors back to its client (in training mode) or computes the
// required gist vector using the supplied vocabulary of prototypical
// descriptors (in normal operational mode).
//
// COMPLEXITY ANALYSIS: O(?) [? time]
// ---------------------------------------
// During normal operation (i.e., non-training mode), this function
// calls apply_surf_on_patches() and flattened_multi_level_histogram().
// The time complexity latter is O(n). That of the former is not yet
// known as it depends on how the SURF black box will work.
//
// The other operations in this function are all constant time or at
// worst O(n), where n is the number of pixels in the input image.
// Therefore, as it stands now, the overall complexity of this function
// will be determined by that of the SURF black box; if that is O(n),
// then so is this; if that is O(n^2), so is this; so on and so forth.
void GistEstimatorSurfPMK::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
   Timer T ;
   ImageType current_frame = e->frame().grayFloat() ;
   SurfKeypoints surf_descriptors = apply_surf_on_image(current_frame) ;
   LINFO("MVN: %g seconds to compute SURF grid", T.getSecs()) ;
   if (itsTrainingHook)
     itsTrainingHook(surf_descriptors) ;
   else
     {
       if (itsVocabulary.empty())
         throw std::runtime_error(GE_SPMK_ERR_NO_VOCABULARY) ;

       T.reset() ;
       //itsGistVector =
       //flattened_multi_level_histogram(surf_descriptors, itsVocabulary) ;
       LINFO("MVN: %g seconds to compute %dx%d gist vector", T.getSecs(),
             itsGistVector.getHeight(), itsGistVector.getWidth()) ;
     }

   rutz::shared_ptr<SimEventGistOutput>
     gist_output_event(new SimEventGistOutput(this, itsGistVector)) ;
   q.post(gist_output_event) ;
}

//---------------------- INPUT IMAGE FILTERATION ------------------------

// A quick helper to convert INVT images to IPL images and properly
// release the IPL images when we're done with them. This is essentially
// a convenience class whose constructor does the conversion and whose
// destructor takes care of releasing the IPL data structures so that
// client functions don't have to worry about these details.
namespace {

class IplImg {
   IplImage* img ;
public:
   IplImg(Image<float>&) ;
   ~IplImg() ;
   operator IplImage*() const {return img ;} // cast operator
} ;

// Constructor to create float images
IplImg::IplImg(Image<float>& I)
   : img(cvCreateImageHeader(cvSize(I.getWidth(), I.getHeight()),
                             IPL_DEPTH_32F, 1))
{
   if (! img)
      throw std::runtime_error("IplImage init error") ;
   img->imageData = reinterpret_cast<char*>(I.getArrayPtr()) ;
}

// Release the OpenCV resources created by constructor
IplImg::~IplImg()
{
   cvReleaseImageHeader(& img) ;
}

} // end of local namespace encapsulating above helper

/*

   COMPLEXITY ANALYSIS: O(?) [linear time]
   ---------------------------------------
*/
static SurfKeypoints apply_surf_on_image(ImageType I)
{
   return opensurf::doSurf(IplImg(I)) ;
}

//---------------------- HISTOGRAM COMPUTATIONS -------------------------

namespace {

} // end of local namespace encapsulating histogram computations section

//---------------------- MISCELLANEOUS FUNCTIONS ------------------------

// Stream I/O for SURF descriptors
std::ostream& operator<<(std::ostream& os, const SurfDescriptor& d)
{
   for (int i = 0; i < GistEstimatorSurfPMK::SURF_DESCRIPTOR_SIZE; ++i)
      os << d.descriptor[i] << ' ' ;
   return os ;
}

//-----------------------------------------------------------------------

#endif // #if !defined(INVT_HAVE_LIBSURF) || !defined(HAVE_OPENCV)

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
