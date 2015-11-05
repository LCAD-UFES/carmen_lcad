/*!
   \file Neuro/GistEstimatorBeyondBoF.C

   This file defines the member functions, static members, etc. of the
   GistEstimatorBeyondBoF class. Further details are in the header file.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorBeyondBoF.C $
// $Id: GistEstimatorBeyondBoF.C 13065 2010-03-28 00:01:00Z itti $
//

//------------------------------ HEADERS --------------------------------

// Gist specific headers
#include "Neuro/GistEstimatorBeyondBoF.H"

// Other INVT headers
#include "Neuro/VisualCortex.H"
#include "Neuro/NeuroSimEvents.H"

#include "Simulation/SimEventQueue.H"

#include "SIFT/ScaleSpace.H"
#include "SIFT/Keypoint.H"

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
#define GE_BBOF_ERR_NO_VOCABULARY \
   "GistEstimatorBeyondBoF requires vocabulary of " \
   "prototypical SIFT descriptors"

//----------------------------- TYPEDEFS --------------------------------

// Some useful shortcuts
typedef GistEstimatorBeyondBoF BBoF ;
typedef BBoF::Vocabulary       Vocabulary ;
typedef BBoF::SiftDescriptor   SiftDescriptor ;
typedef BBoF::SiftGrid         SiftGrid ;

// Other frequently used types
typedef float PixelType ;
typedef Image<PixelType> ImageType ;
typedef Image<double>    GistVectorType ;

//-------------- STATIC DATA MEMBERS AND OTHER CONSTANTS ----------------

// As noted in the header file, the following static members, being
// consts, do not really need to be defined here (i.e., they can be
// initialized directly in the header). But they are defined here to
// resolve some weirdness with GCC 3.4.
int GistEstimatorBeyondBoF::NUM_CHANNELS = 200 ;
int GistEstimatorBeyondBoF::NUM_LEVELS   = 2 ;

// The gist vector consists of counts of feature types at each level of
// the spatial matching pyramid (see Lazebnik paper and comments in other
// parts of this file for the low-down). At level 0 of the pyramid, the
// histogram for each type consists of a single number; at level 1, the
// histogram has 4 numbers; at level 2, 16; and so on.
//
// The following constant uses the formula in the paper to compute the
// total size of the "spatial" histogram for all the levels of the
// pyramid that are used.
//
// If we use a two-level pyramid, we will get a total of 1 + 4 + 16 = 21
// numbers. These 21 numbers will simply be "collapsed" into long
// (21-dimensional) vectors to create the histograms for each feature
// type.
static int HISTOGRAM_SIZE_PER_LEVEL = 21 ;

// Just as the histograms for each feature type are formed by
// concatenating the spatial histograms at each level of the pyramid into
// one long vector, the entire gist vector is constructed from the
// "collapsed" spatial histograms of all feature types by stringing them
// all together.
//
// For a two-level pyramid and a vocabulary of 200 feature types, we will
// get 4200-dimensional gist vectors.
int BBoF::GIST_VECTOR_SIZE = BBoF::NUM_CHANNELS * HISTOGRAM_SIZE_PER_LEVEL ;

// Changing the number of channels and pyramid size.
void GistEstimatorBeyondBoF::num_channels(int n)
{
   if (n >= 5 && n <= 500) { // basic sanity checks
      NUM_CHANNELS = n ;
      GIST_VECTOR_SIZE = n * HISTOGRAM_SIZE_PER_LEVEL ;
   }
}

void GistEstimatorBeyondBoF::num_levels(int n)
{
   if (n >= 0 && n <= 5) { // basic sanity checks
      NUM_LEVELS = n ;
      HISTOGRAM_SIZE_PER_LEVEL = ((1 << (2*n + 2)) - 1)/3 ;
      GIST_VECTOR_SIZE = NUM_CHANNELS * HISTOGRAM_SIZE_PER_LEVEL ;
   }
}

//-------------------------- INITIALIZATION -----------------------------

GistEstimatorBeyondBoF::
GistEstimatorBeyondBoF(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName)
   : GistEstimatorAdapter(mgr, descrName, tagName),
     SIMCALLBACK_INIT(SimEventRetinaImage),
     itsTrainingHook(0)
{}

GistEstimatorBeyondBoF::SiftDescriptor::SiftDescriptor()
{
   const int n = sizeof values / sizeof values[0] ;
   std::fill_n(values, n, 0) ;
}

// The following SiftDescriptor constructor expects the input image to
// essentially be a vector of 128 values and simply copies them into its
// internal SIFT descriptor values array. If the input image does not
// have at least 128 values, this *will* bomb!
GistEstimatorBeyondBoF::SiftDescriptor::SiftDescriptor(const ImageType& I)
{
   const int n = sizeof values / sizeof values[0] ;
   std::copy(I.begin(), I.begin() + n, values) ;
}

GistEstimatorBeyondBoF::
SiftDescriptor::SiftDescriptor(const rutz::shared_ptr<Keypoint>& kp)
{
   std::copy(kp->begin(), kp->end(), values) ;
}

// Quick helper to extract the r-th row of an Image
static inline ImageType get_row(int r, const ImageType& I)
{
   return crop(I, Point2D<int>(0, r), Dims(I.getWidth(), 1)) ;
}

// The vocabulary is a list of SIFT descriptors. But for convenience,
// clients can pass it in as an image. The image must have 128 columns
// (for the 128 values that make up a SIFT descriptor) and will usually
// have 200 rows (the size of the vocabulary). Thus, the dimensions of
// the input image used to represent the SIFT descriptor vocabulary in
// client space will be 128x200.
//
// The following method simply converts this image into the list of SIFT
// descriptors used to represent the vocabulary internally by this class.
void GistEstimatorBeyondBoF::setVocabulary(const ImageType& V)
{
   itsVocabulary.clear() ;

   const int H = V.getHeight() ;
   for (int i = 0; i < H; ++i)
      itsVocabulary.push_back(SiftDescriptor(get_row(i, V))) ;
   num_channels(H) ;
}

//----------------------------- CLEAN-UP --------------------------------

GistEstimatorBeyondBoF::~GistEstimatorBeyondBoF(){}

//------------------ GIST FEATURE VECTOR COMPUTATION --------------------

// Forward declarations
namespace {

SiftGrid       apply_sift_on_patches(ImageType&) ;
GistVectorType flattened_multi_level_histogram(const SiftGrid&,
                                               const Vocabulary&) ;

}

// The processing method divides the current frame of the series of input
// images into 16x16 pixel patches and computes the SIFT descriptors for
// each of these patches. Then, it either passes this grid of
// descriptors back to its client (in training mode) or computes the
// required gist vector using the supplied vocabulary of prototypical
// descriptors (in normal operational mode).
//
// COMPLEXITY ANALYSIS: O(?) [? time]
// ---------------------------------------
// During normal operation (i.e., non-training mode), this function
// calls apply_sift_on_patches() and flattened_multi_level_histogram().
// The time complexity latter is O(n). That of the former is not yet
// known as it depends on how the SIFT black box will work.
//
// The other operations in this function are all constant time or at
// worst O(n), where n is the number of pixels in the input image.
// Therefore, as it stands now, the overall complexity of this function
// will be determined by that of the SIFT black box; if that is O(n),
// then so is this; if that is O(n^2), so is this; so on and so forth.
void
GistEstimatorBeyondBoF::
onSimEventRetinaImage(SimEventQueue& q,
                      rutz::shared_ptr<SimEventRetinaImage>& e)
{
   Timer T ;

   ImageType current_frame = e->frame().grayFloat() ;
   SiftGrid sift_patches = apply_sift_on_patches(current_frame) ;
   LINFO("MVN: %g seconds to compute SIFT grid", T.getSecs()) ;
   if (itsTrainingHook)
     itsTrainingHook(sift_patches) ;
   else
   {
      if (itsVocabulary.empty())
         throw std::runtime_error(GE_BBOF_ERR_NO_VOCABULARY) ;

      T.reset() ;
      itsGistVector =
         flattened_multi_level_histogram(sift_patches, itsVocabulary) ;
      LINFO("MVN: %g seconds to compute %dx%d gist vector", T.getSecs(),
            itsGistVector.getHeight(), itsGistVector.getWidth()) ;
   }

   rutz::shared_ptr<SimEventGistOutput>
     gist_output_event(new SimEventGistOutput(this, itsGistVector)) ;
   q.post(gist_output_event) ;
}

//---------------------- INPUT IMAGE FILTERATION ------------------------

namespace {

// This function applies a Gaussian blur using the supplied sigma value
// to take out salt and pepper noise from the input image. The process
// involves a simple sequence of 2X interpolation, blurring and
// decimation back to the original size.
ImageType denoise(ImageType& lum, float sigma = 1.6f)
{
   lum = interpolate(lum) ;

   const float blursig = sqrtf(sigma * sigma - 1) ;
   Image<float> kernel = gaussian<float>(1, blursig, lum.getWidth(), 1) ;
   kernel = kernel / static_cast<float>(sum(kernel)) ;
   lum = sepFilter(lum, kernel, kernel, CONV_BOUNDARY_CLEAN) ;

   lum = decXY(lum) ;

   return lum ;
}

/*
   The following function breaks the image into fixed patches of 16x16
   pixels with 8 pixel overlaps and computes SIFT descriptors for each
   patch. All SIFT descriptors in the output grid are computed at
   orientation zero (i.e., upright).

   COMPLEXITY ANALYSIS: O(n) [linear time]
   ---------------------------------------
   In the worst case, i.e., when the regular grid into which the input
   image is to be divided is just one pixel (rather than the usual
   16x16), the outer loop will execute a maximum of H times, where H is
   the height of the input image. Similarly, the inner loop will iterate
   a maximum of W times, where W is the input image's width. Together,
   these two loops will end up executing a total of n = WxH times.

   Inside the loops, we have an image crop operation, which will be
   O(n). The next step involves computing the SIFT descriptor for the
   image patch cropped out by the previous one. The overall complexity
   of this function depends on the complexity of this SIFT computation
   step. If it is O(n), so is this function; if it is O(n^2), so is this
   function; so on and so forth.
*/
SiftGrid apply_sift_on_patches(ImageType& I)
{
   I = denoise(I) ;

   // ScaleSpace needs image set rather than single image
   ImageSet<PixelType> images(1, I.getDims()) ;
   images.getImageMut(0) = I ;

   // Divide input image into regular grid of cells sized 16x16 pixels
   // with 8 pixels overlap and get SIFT keypoints for each cell in the
   // grid. The gridding is handled internally by ScaleSpace.
   typedef rutz::shared_ptr<Keypoint> KP ;
   std::vector<KP> keypoints ;
   ScaleSpace ss(images, 1);
   const int n = ss.getGridKeypoints(keypoints) ;

   // Convert std::vector of keypoints retuned by ScaleSpace to grid of
   // SIFT descriptors required by other functions here and by this
   // class's clients (mainly just src/Gist/train-bbof.C).
   const int W = I.getWidth()/8  - 1 ;
   const int H = I.getHeight()/8 - 1 ;
   LINFO("SIFT descriptor grid size for %dx%d image: %dx%d (= %d descriptors)",
         I.getWidth(), I.getHeight(), H, W, n) ;

   int i = 0 ;
   SiftGrid G(W, H, NO_INIT) ;
   for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x)
         G.setVal(x, y, SiftDescriptor(keypoints[i++])) ;
   return G ;
}

} // end of local namespace encapsulating input image filteration section

//---------------------- HISTOGRAM COMPUTATIONS -------------------------

namespace {

/*
   Once we have the SIFT descriptors for the 16x16 pixel patches, we
   compute a feature map using the vocabulary of prototypical SIFT
   descriptors. The feature map holds the list of coordinates
   corresponding to each feature type.

   For example, with a vocabulary size of 200, the feature map will hold
   (at most) 200 "bins" corresponding to the 200 feature types (or
   words). Each bin will contain a list of the coordinates at which that
   feature type occurs. These coordinates are in the SIFT descriptors
   grid space.

   To clarify further: consider a 160x120 input image. We first divide
   this image into 16x16 pixel patches with an 8 pixel overlap in each
   direction. We then proceed to compute SIFT descriptors for these
   patches. These descriptors are arranged in a grid whose size (in this
   case) is 20x15.

   Then, we figure out which bin each of the descriptors in the SIFT
   descriptor grid belongs to by matching against the vocabulary of
   prototypical SIFT descriptors. This "transforms" the grid of SIFT
   descriptors into a grid of bin indices.

   The feature map is obtained (essentially) by "flipping" the grid of
   bin indices so that each bin index points to a list of coordinates. As
   mentioned above these coordinates are the coordinates of the cells in
   20x15 descriptor grid.

   Hopefully, the following picture helps visualize the entire process:


                  0   1   ...   19              0   1   ...   19
                +---+---+-----+---+           +---+---+-----+---+
              0 | S | S | ... | S |         0 | 7 | 3 | ... | 3 |
                +---+---+-----+---+           +---+---+-----+---+
              1 | S | S | ... | S |         1 | 4 | 7 | ... | 3 |
                +---+---+-----+---+ =====>    +---+---+-----+---+
              : | : | : |  :  | : |         : | : | : |  :  | : |
              : | : | : |  :  | : |         : | : | : |  :  | : |
                +---+---+-----+---+           +---+---+-----+---+
             14 | S | S | ... | S |        14 |123| 8 | ... | 9 |
                +---+---+-----+---+           +---+---+-----+---+

                   20x15 grid of                 20x15 grid of
                 SIFT descriptors                 bin indices
                                                       |
                                                       |
                +---+                                  |
                | : |                                  |
                | : |                                  |
                +---+                                  |
              3 | --|--> (1,0) (19,0) (19,1) ...       |
                +---+                                  |
              4 | --|--> (0,1) ...                     |
                +---+                                  |
                | : |                                  |
                | : |                                  |
                +---+                                  |
              7 | --|--> (0,0) (1,1) ...         <=====+
                +---+
              8 | --|--> (1,14) ...
                +---+
              9 | --|--> (19,14) ...
                +---+
                | : |
                | : |
                +---+
            123 | --|--> (0,14) ...
                +---+
                | : |
                | : |
                +---+

               Feature
                 Map

   The following types are for the above data structure.
*/
typedef Point2D<int>             GridCoord ;
typedef std::list<GridCoord>     CoordList ;
typedef std::map<int, CoordList> FeatureMap ;

// To compute the gist vector for an input image, we need to count the
// number of instances of each feature type at each level of the spatial
// matching pyramid. At level 0, there will be only one count for a given
// feature type. At level 1, the SIFT descriptor grid is divided into a
// 2x2 grid and so there are 4 counts. At level 2, the SIFT descriptor
// grid is divided into a 4x4 grid for a total of 16 counts.
//
// The following type is used to store the weighted, normalized feature
// type counts at each level of the pyramid.
typedef Image<float> Histogram ;

// Forward declarations
FeatureMap compute_feature_map(const SiftGrid&, const Vocabulary&) ;
void       compute_spatial_histogram(const FeatureMap&, const Dims&,
                                     GistVectorType*) ;
Histogram  spatial_counts_vector(const CoordList&, int, const Dims&) ;
int        find_bin(const SiftDescriptor&, const Vocabulary&) ;
float      dist2(const SiftDescriptor&, const SiftDescriptor&, float) ;
void       normalize(GistVectorType*, double) ;

// Top-level routine to compute the gist vector for the input image given
// the grid of SIFT descriptors for the pixel patches and the vocabulary
// of prototypical SIFT descriptors.
//
// COMPLEXITY ANALYSIS: O(n) [linear time]
// ---------------------------------------
// Both the compute_feature_map() and compute_spatial_histogram()
// functions are O(n), where n is the number of pixels in the input
// image. The normalize routine is constant time. Therefore, this
// function runs in time O(n).
//
// DEVNOTE: See the complexity analyses for each of the above-mentioned
// functions to understand why they are linear and constant time
// respectively.
GistVectorType
flattened_multi_level_histogram(const SiftGrid& G, const Vocabulary& V)
{
   const Dims gist_vector_size(BBoF::gist_vector_size(), 1) ;

   GistVectorType H(gist_vector_size, ZEROS) ;
   FeatureMap F = compute_feature_map(G, V) ;
   compute_spatial_histogram(F, G.getDims(), &H) ;
   normalize(&H, G.getSize()) ;
   return H ;
}

// The following function implements the procedure described in the
// comment and figure at the start of this section to produce the feature
// map from the SIFT descriptors grid and vocabulary.
//
// COMPLEXITY ANALYSIS: O(n) [linear time]
// ---------------------------------------
// The outer and inner loops combined will execute a total of WxH times
// where W is the width of the SIFT descriptor for the input image and H
// this grid's height. W and H are both going to be some fraction of the
// width and height of the input image itself. Thus, in the worst case,
// the inner and outer loops execute O(n) times, where n is the number
// of pixels in the input image.
//
// To illustrate, consider a 160x120 input image. It is partitioned into
// a regular grid of 16x16 pixels with an 8 pixel overlap for each cell.
// This yields a 20x15 grid of SIFT descriptors for the original 160x120
// image. A 320x240 image will have a 40x30 corresponding SIFT grid and
// an 80x60 image will have a 10x8 grid.
//
// A WxH image will have a (W/8)x(H/8) SIFT grid. If we use n = WxH, we
// can say that the size of the SIFT grid for an input image containing
// n pixels is some constant k times n. (NOTE: k won't always be 1/64
// because the spacing/overlap between cells as well as the size of the
// cells can be something other than 8 and 16x16 pixels respectively.)
//
// Now, having established that the inner and outer loops in this
// function execute O(n) times, we only have to determine the complexity
// of the code inside the loops to ascertain the overall complexity of
// this function.
//
// The find_bin() function is constant time (refer to its complexity
// analysis for the explanation of why this is so).
//
// As per the complexity guarantees provided by the STL, finding items
// in an std::map is at worst logarithmic. In our case, the size of the
// feature map can be no more than M, the number of channels we are
// using. Therefore, looking up feature types in the feature map can be
// no worse than O(log M).
//
// Similarly, inserting an element into the feature map is O(log M). And
// appending an element to the end of an std::list (L.push_back) is
// O(1).
//
// Therefore, the inner loop is dominated by the map look-up and
// insertion operations. This means that the overall complexity of the
// following function is O(log M * n).
//
// M, the number of channels, is a fixed external parameter that cannot
// be arbitrarily changed. That is, we cannot train the scene classifier
// with M = 200 (say) and then classify with M = 400. This fact allows
// us to treat M as a constant.
//
// As a result, we can write the time complexity for this function as
// O(kn) or O(n), where n is the number of pixels in the input image.
FeatureMap compute_feature_map(const SiftGrid& G, const Vocabulary& V)
{
   const int W = G.getWidth() ;
   const int H = G.getHeight() ;

   FeatureMap F ;
   for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x)
      {
         int bin = find_bin(G.getVal(x,y), V) ;
         FeatureMap::iterator it = F.find(bin) ;
         if (it == F.end()) {
            CoordList L ;
            L.push_back(GridCoord(x,y)) ;
            F.insert(std::make_pair(bin, L)) ;
         }
         else
            it->second.push_back(GridCoord(x,y)) ;
      }

   return F ;
}

// Once we have the feature map, we need to compute the histogram for
// each feature at different levels of the spatial match pyramid. The
// spatial histograms for each level are concatenated to form a single
// vector for each feature. All these vectors for all the features are
// then concatenated to form the final gist vector. (See the Lazebnik
// paper for details.)
//
// COMPLEXITY ANALYSIS: O(n) [linear time]
// ---------------------------------------
// The outer loop will execute M times, where M is the number of
// channels. Finding the coordinate list for a feature type is at most
// O(log M). The inner for loop will execute L times, where L is the
// number of levels in the spatial matching pyramid. The
// spatial_counts_vector() function will thus be called a maximum of ML
// times.
//
// For an input image with n pixels, spatial_counts_vector() takes time
// O(n). Therefore, the overall time complexity for this function is
// O(M * L * log M * n).
//
// M and L are both external parameters to the algorithm but remain
// fixed for any given "experiment" involving all the steps related to
// training, gist vector computations and subsequent classification.
// That is, we cannot train with some particular values of M and L and
// then classify using some other values. Consequently, we can treat
// these parameters as constants and replace them in the above
// expression with k = M * L * log M.
//
// Thus, this function's run time grows linearly with the size of the
// input image. In other words, its complexity is O(kn) or just O(n) if
// we ignore the constant of proportionality.
void compute_spatial_histogram(const FeatureMap& F,
                               const Dims& sift_grid_size,
                               GistVectorType* H)
{
   GistVectorType::iterator target = H->beginw() ;
   for (int m = 0; m < BBoF::num_channels(); ++m)
   {
      FeatureMap::const_iterator it = F.find(m) ;
      if (it == F.end()) // no instances of feature type m in input image
         target += HISTOGRAM_SIZE_PER_LEVEL ;
      else // count instances of feature type m at each level of spatial pyr.
      {
         const CoordList& coords = it->second ;
         for (int l = 0; l <= BBoF::num_levels(); ++l)
         {
            Histogram h = spatial_counts_vector(coords, l, sift_grid_size) ;
            std::copy(h.begin(), h.end(), target) ;
            target += h.getSize() ;
         }
      }
   }
}

/*
   In an ordinary bag-of-features matching, we would only be interested
   in knowing how many times each feature type occurs in the input
   image. The gist vector would thus simply be a histogram showing the
   counts for each feature type. For example, for 200 feature types, the
   gist vector is just 200 counts.

   When we factor in the spatial matching pyramid as described in the
   Lazebnik paper, we have to subdivide the SIFT descriptors grid into
   smaller regions and count the occurences of each feature type in each
   of these regions. This is why the feature map needs to record the
   coordinates of each feature type within the SIFT descriptors grid.

   The spatial matching pyramid is simply a mechanism for specifying the
   resolution of the "grid space" subdivisions. At higher resolutions,
   we perform a more fine-grained spatial feature matching; at lower
   resolutions, we get a coarse "overview" of the feature occurences. In
   fact, at level 0, the spatial matching pyramid is exactly a standard
   bag-of-features with a single count for each feature type.

   To illustrate how the spatial matching process works, let us consider
   a 160x120 input image that has been "transformed" into a 20x15 grid
   of bin indices as described in an earlier comment. At level 1 of the
   pyramid, our 20x15 grid space will be partitioned into 4 regions as
   shown below. At level 2 of the pyramid, the grid space will be
   divided into 16 regions (also shown below).


                Level 0                          Level 1

          0                 19            0        9        19
         +--------------------+          +----------+---------+
        0|                    |         0|          |         |
         |                    |          |          |         |
         |                    |          |          |         |
         |                    |          |          |         |
         |                    |         7|          |         |
         |                    |          +----------+---------+
         |                    |          |          |         |
         |                    |          |          |         |
         |                    |          |          |         |
       14|                    |        14|          |         |
         +--------------------+          +----------+---------+



                                  Level 2

                           0   4    9   14   19
                          +----+----+----+----+
                         0|    |    |    |    |
                          |    |    |    |    |
                         3+----+----+----+----+
                          |    |    |    |    |
                          |    |    |    |    |
                         7+----+----+----+----+
                          |    |    |    |    |
                          |    |    |    |    |
                        11+----+----+----+----+
                          |    |    |    |    |
                        14+----+----+----+----+


   Thus, at each level of the pyramid, the grid space is partitioned
   into a grid with N = 2^l cells in each direction. The histogram for
   each feature type at each level mirrors this partitioning and is
   representend as an NxN image of integers (counts).

   To compute this histogram, we need to figure out the cell of the
   pyramid in which a given coordinate in grid space lies. To illustrate
   how to do this, consider the pyramid at level 1. The 20x15 grid space
   has to be partitioned into a 2x2 area. Thus, each "quadrant" will
   encompass a 10x7.5 area of the 20x15 grid space. Now, let us say we
   have grid space coordinates (5,9). This will lie in the bottom left
   quadrant. Thus, its coordinates in "pyramid space" are (0,1). Notice
   that to transform (5,9) to (0,1) all we need to do is divide the x and
   y components of the grid space coordinates by 10 and 7.5 respectively
   and discard any fractional portion of the resulting quotient.

   In essence, the transformation from grid space to pyramid space is a
   scaling operation. We compute appropriate x and y scale factors and
   then simply scale each feature type's coordinates by these factors.

   The scale factors for each direction are:

         Sx = w/N
         Sy = h/N

   Each coordinate in grid space simply needs to be divided by the above
   factors to effect its transformation into pyramid space.

   NOTE: Scale factors are usually multiplicative, i.e., to scale from
   one coordinate system to another, we multiply rather than divide by an
   appropriate scale factor. Here, we use the scale factors as divisors
   because the above example explains this entire rigmarole in terms of
   division and it's just easier to think of it in that way. We could
   also have computed the reciprocals of the above factors and then used
   multiplication instead.
*/

// The following function object performs the above transformation from
// grid space to pyramid space and updates the histogram count associated
// with each cell in pyramid space. When invoked on every element of a
// coordinate list corresponding to a feature type from the feature map,
// it results in computing the spatial histogram of that feature type at
// the pyramid level currently in effect.
class update_spatial_histogram {
   Histogram& target ;
   float Sx, Sy ;
public:
   update_spatial_histogram(Histogram&, const Dims&) ;
   void operator()(const GridCoord&) const ;
} ;

update_spatial_histogram::update_spatial_histogram(Histogram& H, const Dims& d)
   : target(H),
     Sx(d.w()/static_cast<float>(H.getWidth())),
     Sy(d.h()/static_cast<float>(H.getHeight()))
{}

void update_spatial_histogram::operator()(const GridCoord& c) const
{
   GridCoord p(static_cast<int>(c.i/Sx), static_cast<int>(c.j/Sy)) ;
   ++target[p] ;
}

// The following function returns the histogram for the specified
// feature type at the specified level of the spatial matching pyramid.
// The histogram values are weighted according to the pyramid level as
// described in the Lazebnik paper.
//
// Briefly: at level 0, we divide the feature count by 2^L, where L is
// the maximum number of levels of the spatial matching pyramid. At
// higher levels l, we divide by 2^(L-l+1). The idea is to give more
// weightage to feature matches at higher resolutions and lower
// weightage to matches at coarser resolutions.
//
// Rather than using the feature type to look up the list of SIFT grid
// coordinates from the SIFT descriptors grid, it expects its caller to
// pass it the coordinate list for the desired feature type. Then, for
// each of the coordinates where the feature type occurs, it figures out
// (using the above function object) which cell of the spatial match
// pyramid the feature is in and updates the count associated with that
// cell.
//
// There is a special case for level 0 of the spatial matching pyramid.
// At this level, the grid associated with the pyramid has just one
// cell. So all occurences of all feature types will be in this cell.
// Thus, the histogram for level 0 consists of a single number, viz.,
// the number of entries in the coordinate list for the feature type.
// This handy fact allows us to short-circuit the counting procedure we
// would normally have to perform for other levels of the pyramid.
//
// COMPLEXITY ANALYSIS: O(n) [linear time]
// ---------------------------------------
// For level 0, this function executes a single operation. But at higher
// levels, it needs to compute, for each coordinate where a given
// feature type appears in the input image's SIFT grid, the appropriate
// mapping from "grid space" to "pyramid space" (refer to the comment
// preceding the definition of the update_spatial_histogram function
// object).
//
// The number of coordinate mapping operations to be performed will be
// determined by the size of the coordinates list passed in. This list
// cannot ever have more elements than there are cells in the source
// SIFT grid of the input image.
//
// To illustrate, consider a 160x120 input image. It is partitioned into
// a regular grid of 16x16 pixels with an 8 pixel overlap. This yields a
// 20x15 grid of SIFT descriptors for the original 160x120 image. Thus,
// the SIFT grid contains a total of 300 cells.
//
// Now, in the worst case, we could have some feature type appear in
// every cell of the SIFT grid. In this situation, the coordinates list
// for this feature type will have 300 elements.
//
// The number of cells in the SIFT grid is proportional to the size of
// the input image. A 320x240 image will have a 40x30 corresponding SIFT
// grid and an 80x60 image will have a 10x8 grid. A WxH image will have
// a (W/8)x(H/8) SIFT grid. If we use n = WxH, we can say that the size
// of the SIFT grid for an input image containing n pixels is some
// constant k times n.
//
// Thus, in the worst case, this function executes the operations
// implemented by the update_spatial_histogram function object at most
// kn times. Since the above-mentioned function object only uses a
// couple of elementary operations (division to perform mapping from
// grid space to pyramid space and addition to increment the feature
// type count), the complexity of this function is O(kn) or just O(n).
//
// NOTE: After the spatial histogram updating, there is weighting
// computation. This loop (implicit in the std::transform call) executes
// N^2 times where N is 2^L. However, L, the number of levels in the
// spatial matching pyramid is a fixed external parameter to the gist
// vector computation algorithm. Thus, we can consider the weighting
// step to be constant time. This means that the preceding
// update_spatial_histogram step is the one that dominates the running
// time of this function.
Histogram spatial_counts_vector(const CoordList& coords, int l,
                                const Dims& source_grid_size)
{
   const int L = BBoF::num_levels() ;
   const int N = 1 << l ;

   Histogram H(N, N, ZEROS) ; // NxN partition of grid space at pyramid level l
   if (N == 1) // level 0 ==> no point iterating over entire coord. list
      H.setVal(0, 0, coords.size()/(1 << L)) ;
   else // level > 0 ==> do spatial pyramid thingummy
   {
      std::for_each(coords.begin(), coords.end(),
                    update_spatial_histogram(H, source_grid_size)) ;
      std::transform(H.begin(), H.end(), H.beginw(),
                     std::bind2nd(std::divides<float>(), 1 << (L - l + 1))) ;
   }
   return H ;
}

// The following function finds the prototypical SIFT descriptor that
// most closely matches the supplied descriptor and returns the
// vocabulary index of the prototypical descriptor. This index is the
// input descriptor's bin.
//
// COMPLEXITY ANALYSIS: 0(1) [constant time]
// -----------------------------------------
// The loop in this function calls the dist2() function and performs
// elementary operations. The dist2() function is O(1) [as per its
// complexity analysis]. Therefore, this function's complexity is O(M),
// where M is the number of channels and the size of the SIFT descriptor
// vocabulary.
//
// M is an external parameter to the gist vector computation algorithm.
// However, once picked, it cannot be arbitrarily changed. That is, if
// we train the scene classifier using gist vectors computed on the
// basis of a vocabulary consisting of (say) 200 words, then we cannot
// change this to 400 and attempt subsequent classifications.
//
// Therefore, for all practical purposes this function is constant time.
// More accurately, it is O(M) and M is essentially a constant.
int find_bin(const SiftDescriptor& S, const Vocabulary& V)
{
   int bin = -1 ;
   float min_dist = std::numeric_limits<float>::max() ;

   int i = 0 ;
   for (Vocabulary::const_iterator it = V.begin(); it != V.end(); ++it, ++i)
   {
      float d = dist2(S, *it, min_dist) ;
      if (d < min_dist) {
         bin = i ;
         min_dist = d ;
      }
   }

   return bin ;
}

// The following function returns the square of the Euclidean distance
// between two SIFT descriptors subject to a supplied minimum, i.e., if
// the square distance exceeds the minimum, the computation is
// short-circuited and the minimum returned.
//
// COMPLEXITY ANALYSIS: O(1) [constant time]
// -----------------------------------------
// The loop in this function contains only elementary operations. The
// number of iterations is equal to the size of a SIFT descriptor, i.e.,
// 128. Thus, this function executes a bunch of elementary operations
// exactly 128 times. Since the size of a SIFT descriptor is independent
// of the size of the input images to the gist vector computations, we
// can consider this function as a constant time operation, i.e., O(1).
float dist2(const SiftDescriptor& L, const SiftDescriptor& R, float min)
{
   float d = 0 ;
   for (int i = 0; i < SiftDescriptor::SIZE; ++i) {
      d += (L[i] - R[i]) * (L[i] - R[i]) ;
      if (d > min)
         return min ;
   }
   return d ;
}

// COMPLEXITY ANALYSIS: O(1) [constant time]
// -----------------------------------------
// This function divides each element of the gist vector by the
// normalizer. Therefore, it is linear in the size of the gist vector.
// This size is determined by the number of channels (M) and the number
// of levels of the spatial matching pyramid (L) [see Lazebnik paper for
// details].
//
// The exact formula for the size of the gist vector is
// (M * (4^(L+1) - 1))/3. Both M and L are parameters to the algorithm
// but remain fixed for any given "experiment" involving all the steps
// related to training, gist vector computations and subsequent
// classification. That is, we cannot train with some particular values
// of M and L and then classify using some other values.
//
// Thus, in some sense, both M and L are predetermined constants so that
// the gist vector size remains fixed. In our case, M = 200 and L = 2.
// So, our gist vectors have 4200 dimensions. This won't change.
// Consequently, we can consider this normalization step as being
// constant time, i.e., O(1).
void normalize(GistVectorType* G, double normalizer)
{
   std::transform(G->begin(), G->end(), G->beginw(),
                  std::bind2nd(std::divides<double>(), normalizer)) ;
}

} // end of local namespace encapsulating histogram computations section

//---------------------- MISCELLANEOUS FUNCTIONS ------------------------

// Stream I/O for SIFT descriptors
std::ostream& operator<<(std::ostream& os, const SiftDescriptor& d)
{
   for (int i = 0; i < SiftDescriptor::SIZE; ++i)
      os << d[i] << ' ' ;
   return os ;
}

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
