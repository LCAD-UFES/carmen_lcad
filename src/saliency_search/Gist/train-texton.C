/*!
   @file Gist/train-texton.C create the prototypical universal textons
         "database" from the training data
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/train-texton.C $
// $Id: train-texton.C 14605 2011-03-15 02:25:06Z dparks $
//

//------------------------------ HEADERS --------------------------------

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

// Gist specific headers
#include "Neuro/GistEstimatorTexton.H"

// Other INVT headers
#include "Neuro/StdBrain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"

#include "Media/SimFrameSeries.H"
#include "Media/MediaOpts.H"

#include "Simulation/SimEventQueue.H"
#include "Simulation/SimEventQueueConfigurator.H"

#include "Channels/ChannelOpts.H"
#include "Component/ModelManager.H"
#include "Component/ModelOptionDef.H"

#include "Image/Point2D.H"

#include "nub/ref.h"

#ifndef HAVE_OPENCV // fake OpenCV API so as to not break builds
namespace {

struct CvMat {int rows, cols, type ;} ;

inline CvMat* cvCreateMat(int, int, int) {return 0 ;}
inline void   cvZero(CvMat *) {}
inline void   cvReleaseMat(CvMat**) {}
inline double cvmGet(CvMat*, int, int) {return 0 ;}
inline void   cvmSet(CvMat*, int, int, double) {}
inline int    cvTermCriteria(int, int, double) {return 0 ;}
inline void   cvKMeans2(CvMat*, int, CvMat*, int) {}

#define CV_32FC1 0
#define CV_32SC1 0
inline int CV_MAT_TYPE(int) {return 0 ;}
#define CV_MAT_ELEM(matrix, type, row, col) (type(0))

#define CV_TERMCRIT_EPS  0
#define CV_TERMCRIT_ITER 0

}

#endif // OpenCV availability check

// Standard C++ headers
#include <fstream>
#include <sstream>
#include <ios>
#include <numeric>
#include <algorithm>
#include <functional>
#include <map>
#include <vector>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <limits>
#include <cmath>

//-------------------------- UTILITY ROUTINES ---------------------------

namespace {

// Convenient (but perhaps not the most efficient) helper to convert
// various data types to strings.
//
// DEVNOTE: Works as long as type T defines an operator << that writes to
// an ostream.
template<typename T>
std::string to_string(const T& t)
{
   std::ostringstream str ;
   str << t ;
   return str.str() ;
}

// Count the number of lines in a file (wc -l)
int count_lines(const std::string& file_name)
{
   int n = -1 ; // because EOF is read after final \n (1 extra iter. of loop)
   std::ifstream ifs(file_name.c_str()) ;

   std::string dummy ;
   while (ifs) {
      getline(ifs, dummy) ;
      ++n ;
   }
   return n ;
}

// Returns true if a floating point number is near zero
bool is_zero(double d)
{
   return std::fabs(d) <= std::numeric_limits<double>::epsilon() ;
}

} // end of local namespace encapsulating utility routines section

//------------------------ TEXTON ACCUMULATION --------------------------

// Given an input image, GistEstimatorTexton works by performing
// K-nearest neighbour search on the textons in the input image. Each of
// the input textons is matched against the database of "universal"
// textons and the frequency of the occurrence of universal textons in
// the input image is used as the basis of image classification.
//
// The universal textons database is just a collection of the 100 most
// frequently occuring textons in the training set. It is stored as a
// 100x36 matrix (the number 36 comes from 6x3x2, where 6 is the number
// of orientations input images are filtered at, 3 is the number of
// scales for each orientation and 2 is due to the even and odd filters
// applied at each scale and orientation; refer to the Renninger-Malik
// paper for further details).
//
// This program has an operational mode that spits out a plain text file
// containing the universal textons for a given set of input images by
// first accumulating all the textons and then performing K-means
// clustering on them. To get at the training set's textons, we need to
// "hook" into the GistEstimatorTexton's processing pipeline.
//
// These textons are then stored in a plain text file that is then loaded
// as part of the data for the K-means clustering procedure.
//
// This section of code takes care of accumulating the textons for the
// training set in the above-mentioned plain text file. The code for
// performing K-means analysis on the accumulated textons is in the next
// section.
namespace {

// A texton is simply the vector of filter responses for a given pixel.
// That is, if we apply 36 filters to an input image, we will get 36
// Images as the filteration results. The texton for pixel (i,j) will be
// the vector of 36 numbers formed by taking pixel (i,j) from each of the
// 36 Images in the filteration results.
//
// Rather than implement some custom object to represent a texton and a
// collection of textons, we simply piggyback off INVT's Image<T> class,
// which is used to store the textons for an entire image.
typedef GistEstimatorTexton::ImageType Texton ;

// Quick wrapper around an output file used to store the training
// textons. This file is populated during the texton accumulation phase
// and then loaded as the data matrix for the K-means computation.
class textons_accumulator {
   static std::string out_file ;

   textons_accumulator() ; // private to disallow instantiation
   ~textons_accumulator() ;
public :
   static void output_file(const std::string& file_name) ;
   static void write(const Texton&) ;
} ;

// Static data member for storing the training textons file name
// persistently across multiple invocations of the GistEstimatorTexton's
// training hook.
std::string textons_accumulator::out_file ;

// The GistEstimatorTexton client must set the above variable
// appropriately prior to setting up the GistEstimatorTexton training
// hook.
void textons_accumulator::output_file(const std::string& file_name)
{
   out_file = file_name ;
}

// The following function is meant to be used by the GistEstimatorTexton
// training hook. It simply appends the texton Image passed to it to the
// output file row by row.
//
// DEVNOTE: We could open the output file once (i.e., in the previous
// function) and use that object to avoid reopening (by using a static
// ostream data member rather than a static string). However, if the
// program were to somehow crash halfway through, then the training
// textons output file would be in an inconsistent state and rerunning
// the program can result in appending data to a possibly inconsistent
// dataset, which would only make things worse.
//
// Thus, we choose to open and close the output file each time the
// GistEstimatorTexton training hook is triggered. (Of course, if the
// program cashes while this function is executing, then all bets are
// off; the training textons file's inconsistency will be unavoidable in
// this case.)
void textons_accumulator::write(const Texton& textons)
{
   if (out_file.empty())
      throw std::runtime_error("textons accumulator output file "
                               "not specified") ;

   std::ofstream ofs(out_file.c_str(), std::ios::out | std::ios::app) ;
   for (int y = 0; y < textons.getHeight(); ++y) {
      for (int x = 0; x < textons.getWidth(); ++x)
         ofs << textons.getVal(x, y) << ' ' ;
      ofs << '\n' ;
   }
}

// The following function is the callback for the GistEstimatorTexton's
// training hook. The gist estimator object will pass this function an
// Image that serves as the current input image's textons. This
// function simply shoves this texton Image into the accumulator defined
// above.
void accumulate_textons(const Texton& textons)
{
   textons_accumulator::write(textons) ;
}

} // end of local namespace encapsulating texton accumulation section

//------------------- UNIVERSAL TEXTONS COMPUTATION ---------------------

// Once the textons have been accumulated from the filteration results
// of each of the input images, we compute the universal textons using
// the K-means implementation available in OpenCV.
//
// DEVNOTE: Renninger and Malik used the K-means implementation in the
// Netlab (Matlab) toolbox. Unfortunately, Matlab was unable to handle
// the volume of data being passed to it for the gist models comparison
// project at iLab (for which this implementation was developed).
namespace {

// Crude encapsulation of OpenCV matrices
class OpenCVMatrix {
   CvMat* matrix ;
public :
   OpenCVMatrix(int num_rows, int num_cols, int type) ;
   OpenCVMatrix(CvMat*) ;
   ~OpenCVMatrix() ;

   int num_rows() const {return matrix->rows ;}
   int num_cols() const {return matrix->cols ;}
   int type()     const {return CV_MAT_TYPE(matrix->type) ;}

   template<typename T> // T must match matrix->type (float for CV_32FC1, etc.)
   T get(int i, int j) const {return CV_MAT_ELEM(*matrix, T, i, j) ;}

   operator CvMat*() const {return matrix ;} // auto conv. (usually a bad idea)
} ;

OpenCVMatrix::OpenCVMatrix(int num_rows, int num_cols, int type)
   : matrix(cvCreateMat(num_rows, num_cols, type))
{
   if (! matrix)
      throw std::runtime_error("unable to create OpenCV matrix") ;
}

OpenCVMatrix::OpenCVMatrix(CvMat* M)
   : matrix(M)
{
   if (! matrix)
      throw std::runtime_error("cannot create empty/null matrix") ;
}

OpenCVMatrix::~OpenCVMatrix()
{
   cvReleaseMat(& matrix) ;
}

// The following function reads the training textons into an OpenCV
// matrix. It must know how many lines the training textons file has.
CvMat* load_training_textons(const std::string& file_name, int num_lines)
{
   CvMat* M =
      cvCreateMat(num_lines, GistEstimatorTexton::NUM_FILTERS, CV_32FC1) ;

   double d ;
   std::ifstream ifs(file_name.c_str()) ;
   for (int i = 0; i < num_lines; ++i)
      for (int j = 0; j < int(GistEstimatorTexton::NUM_FILTERS); ++j) {
         if (! ifs) {
            cvReleaseMat(& M) ;
            throw std::runtime_error(file_name + ": out of data?!?") ;
         }
         ifs >> d ;
         cvmSet(M, i, j, d) ;
      }

   return M ;
}

// OpenCV's K-means implementation returns cluster assignments. But we
// need the cluster centroids. This function takes the data matrix and
// cluster assignments and returns the K centroids.
CvMat* compute_centroids(int K, const OpenCVMatrix& data,
                         const OpenCVMatrix& cluster_assignments)
{
   CvMat* centroids = cvCreateMat(K, data.num_cols(), data.type()) ;
   cvZero(centroids) ;

   std::vector<int> cluster_counts(K) ;
   std::fill(cluster_counts.begin(), cluster_counts.end(), 0) ;

   for (int i = 0; i < data.num_rows(); ++i)
   {
      int C = cluster_assignments.get<int>(i, 0) ;
      ++cluster_counts[C] ;

      // Compute sum of C-th centroid and i-th row
      for (int j = 0; j < data.num_cols(); ++j)
         cvmSet(centroids, C, j,
                cvmGet(centroids, C, j) + data.get<float>(i, j)) ;
   }

   // Compute the K centroids by averaging the totals accumulated in the
   // centroids matrix using the cluster counts.
   for (int C = 0; C < K; ++C)
      for (int j = 0; j < data.num_cols(); ++j)
         cvmSet(centroids, C, j,
                cvmGet(centroids, C, j) / cluster_counts[C]) ;

   return centroids ;
}

// K-means parameters
#ifndef TT_KMEANS_ITERATIONS
   #define TT_KMEANS_ITERATIONS (100)
#endif
#ifndef TT_KMEANS_PRECISION
   #define TT_KMEANS_PRECISION (.01)
#endif

// This function performs K-means clustering on the supplied data matrix
// and returns the cluster centers.
CvMat* kmeans(int K, const OpenCVMatrix& data)
{
   OpenCVMatrix cluster_assignments(data.num_rows(), 1, CV_32SC1) ;

   LINFO("MVN: computing K-means cluster assignments with OpenCV") ;
   cvKMeans2(data, K, cluster_assignments,
             cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                            TT_KMEANS_ITERATIONS, TT_KMEANS_PRECISION)) ;

   LINFO("MVN: cluster assignments done; computing centroids...") ;
   return compute_centroids(K, data, cluster_assignments) ;
}

// Write the universal textons, row by row, to a plain text file.
void save_universal_textons(const OpenCVMatrix& universal_textons,
                            const std::string& file_name)
{
   std::ofstream ofs(file_name.c_str()) ;
   for (int i = 0; i < universal_textons.num_rows(); ++i) {
      for (int j = 0; j < universal_textons.num_cols(); ++j)
         ofs << universal_textons.get<float>(i, j) << ' ' ;
      ofs << '\n' ;
   }
}

// Read the universal textons from a plain text file into an Image<T>
Texton load_universal_textons(const std::string& file_name)
{
   const int M = count_lines(file_name) ;
   const int N = GistEstimatorTexton::NUM_FILTERS ;
   Texton U(N, M, ZEROS) ;

   float f ;
   std::ifstream ifs(file_name.c_str()) ;
   for (int j = 0; j < M; ++j)
      for (int i = 0; i < N; ++i) {
         if (! ifs)
            throw std::runtime_error(file_name + ": out of data?!?") ;
         ifs >> f ;
         U.setVal(i, j, f) ;
      }

   return U ;
}

// The training textons are agglomerated into the following number of
// clusters.
//
// DEVNOTE: Although not used in this section, it makes most sense to
// define this symbol here. It does not fit well into the other sections
// of this file.
#ifndef TT_NUM_UNIVERSAL_TEXTONS
   #define TT_NUM_UNIVERSAL_TEXTONS 100
#endif

} // end of local namespace encapsulating universal textons section

//------------------- TRAINING HISTOGRAM PROCESSING ---------------------

// Training is a two step process: first, we use K-means to cluster the
// training set's textons to create the universal textons. Then, we
// collect the histograms counting the universal textons in the training
// images. The universal textons and training set's histogram "database"
// are both used for image classification.
namespace {

// Some useful types for dealing with texton histograms
typedef Image<double> Histogram ;
typedef std::map<std::string, Histogram> HistogramMap ;
typedef HistogramMap::value_type HistogramMapEntry ;

// This function appends a training image's histogram to the training
// histograms database file under the supplied "entry" name. As we did in
// the textons accumulation function, in order to minimize possible
// inconsistencies in this database, we choose to open and close the
// training histograms file with each invocation of this helper rather
// than keep a persistent ostream object around that obviates the need
// for repeated file open/close operations.
void save_histogram(const Histogram& histogram,
                    const std::string& hist_name,
                    const std::string& file_name)
{
   LINFO("MVN: saving histogram %s to %s",
         hist_name.c_str(), file_name.c_str()) ;
   std::ofstream ofs(file_name.c_str(), std::ios::out | std::ios::app) ;
   ofs << hist_name << ' ' ;
   for (int y = 0; y < histogram.getHeight(); ++y) // should be just one row
      for (int x = 0; x < histogram.getWidth(); ++x) // should be 100 columns
         ofs << histogram.getVal(x, y) << ' ' ;
   ofs << '\n' ;
}

// The following function reads the training histograms "database," which
// is a plain text file containing one histogram per line. Each line
// starts with the name of the training histogram and then come the
// hundred numbers making up that histogram.
HistogramMap load_training_histograms(const std::string& file_name)
{
   HistogramMap histograms ;

   std::ifstream ifs(file_name.c_str()) ;
   for(;;)
   {
      std::string str ;
      std::getline(ifs, str) ;
      if (! ifs || str.empty())
         break ;
      std::istringstream line(str) ;

      std::string histogram_name ;
      line >> histogram_name ;

      Histogram H(TT_NUM_UNIVERSAL_TEXTONS, 1, ZEROS) ;
      double d ; int i = 0 ;
      while (line >> d)
         H.setVal(i++, 0, d) ;

      histograms.insert(std::make_pair(histogram_name, H)) ;
   }

   return histograms ;
}

} // end of local namespace encapsulating training histograms section

//----------------------- IMAGE CLASSIFICATION --------------------------

// Given the histograms for an input image and each of the training
// images, we can tell which training image the input image matches most
// closely by performing a chi-squared distance check between the input
// image's histogram and the histograms of each of the training images.
namespace {

// When computing the chi-square distance between the input image's
// histogram and that of each of the training images, we want to be able
// to tell which training image is the closest. For that, we use the
// following pair that "maps" a training histogram name to its
// corresponding distance.
typedef std::pair<std::string, double> HistogramDistance ;

// To sort histogram distances, we want to compare the chi-square
// measure rather than their names.
bool chi_square_cmp(const HistogramDistance& L, const HistogramDistance& R)
{
   return L.second < R.second ;
}

// But when writing classification results, we're only interested in the
// matching training image's name and not really in the chi-square
// distance between its histogram and that of the input image.
std::ostream& operator<<(std::ostream& os, const HistogramDistance& D)
{
   return os << D.first ;
}

// Given an entry from the training histograms map, the following
// function object returns the chi-square distance between the input
// image's histogram and the training image's histogram.
class chi_square {
   const Histogram& input ;
   double distance(const Histogram&, const Histogram&) const ;
public :
   chi_square(const Histogram& H) ;
   HistogramDistance operator()(const HistogramMapEntry& E) const {
      return std::make_pair(E.first, distance(input, E.second)) ;
   }
} ;

chi_square::chi_square(const Histogram& H)
   : input(H)
{}

double chi_square::distance(const Histogram& L, const Histogram& R) const
{
   const int n = L.getWidth() ; // both should have same dimensions (100x1)
   double sum = 0 ;
   for (int i = 0; i < n; ++i)
   {
      double l = L.getVal(i, 0) ;
      double r = R.getVal(i, 0) ;
      double l_minus_r = l - r ;
      double l_plus_r  = l + r ;
      if (is_zero(l_minus_r) || is_zero(l_plus_r))
         continue ;
      sum += (l_minus_r * l_minus_r)/l_plus_r ;
   }
   return sum/2 ;
}

// This function computes the chi-square distance between the input
// image's histogram and the histograms of the training images and then
// writes the top five matches to the specified results file.
//
// DEVNOTE: To output the top five matches to the results file, we ought
// to be able to use the std::copy algorithm in conjunction with
// std::ostream_iterator<HistogramDistance>. Unfortunately, ostream
// iterators cannot be used with user-defined types. This is entirely in
// keeping with C++'s philosophy of sucking ass most of the time but
// sucking ass big-time only every now and then.
void classify_image(const HistogramMapEntry& input,
                    const HistogramMap& training_histograms,
                    const std::string& results_file)
{
   std::vector<HistogramDistance> chi_square_distances ;
   std::transform(training_histograms.begin(), training_histograms.end(),
                  std::back_inserter(chi_square_distances),
                  chi_square(input.second)) ;
   std::sort(chi_square_distances.begin(), chi_square_distances.end(),
             chi_square_cmp) ;

   std::ofstream ofs(results_file.c_str(), std::ios::out | std::ios::app) ;
   ofs << input.first << ' ' ;
   //std::copy(chi_square_distances.begin(), chi_square_distances.begin() + 5,
             //std::ostream_iterator<HistogramDistance>(ofs, " ")) ; // ERROR!
   for (unsigned int i = 0; i < chi_square_distances.size() && i < 5; ++i)
      ofs << chi_square_distances[i] << ' ' ;
   ofs << '\n' ;
}

} // end of local namespace encapsulating image classification section

//----------------------- COMMAND LINE OPTIONS --------------------------

// This program has four distinct phases/modes of operation, each one
// specified via a suitable non-option command line argument.
// Additionally, it supports several command line options to allow users
// to tweak various parameters such as the name of the universal textons
// file, the training histograms database, and so on.
namespace {

const ModelOptionCateg MOC_TEXTONS = {
   MOC_SORTPRI_3,
   "Options specific to the Renninger-Malik textons program",
} ;

// In the training textons accumulation phase, we collect all the textons
// of the input images into a plain text file.
#ifndef TT_DEFAULT_TRAINING_TEXTONS_FILE
   #define TT_DEFAULT_TRAINING_TEXTONS_FILE "training_textons.txt"
#endif

const ModelOptionDef OPT_TrainingTextons = {
   MODOPT_ARG_STRING, "TrainingTextons", & MOC_TEXTONS, OPTEXP_CORE,
   "This option specifies the name of the file where training textons\n"
   "should be accumulated or read from. This is a plain text file containing\n"
   "the training textons matrix that will be fed into the K-means procedure\n"
   "during the texton training phase. Each line of this file will contain a\n"
   "row of training textons.\n",
   "training-textons", '\0', "training-textons-file",
   TT_DEFAULT_TRAINING_TEXTONS_FILE,
} ;

// In the texton training phase, we use the accumulated training textons
// and perform K-means on them to produce the universal textons.
#ifndef TT_DEFAULT_UNIVERSAL_TEXTONS_FILE
   #define TT_DEFAULT_UNIVERSAL_TEXTONS_FILE "universal_textons.txt"
#endif

const ModelOptionDef OPT_UniversalTextons = {
   MODOPT_ARG_STRING, "UniversalTextons", & MOC_TEXTONS, OPTEXP_CORE,
   "This option specifies the name of the file in which the universal\n"
   "textons are (or are to be) stored. This is a plain text file containing\n"
   "the universal_textons matrix that is used for image classification.\n",
   "universal-textons", '\0', "universal-textons-file",
   TT_DEFAULT_UNIVERSAL_TEXTONS_FILE,
} ;

// In the second phase of texton training, we count the universal textons
// in the training images and store them in a training histograms
// "database" under the specified "entry name."
//
// DEVNOTE: The default value for this option (i.e., --histogram-name) is
// not very useful. This particular option really ought to be specified
// on the command line.
#ifndef TT_DEFAULT_TRAINING_HISTOGRAM_NAME
   #define TT_DEFAULT_TRAINING_HISTOGRAM_NAME "training_image"
#endif

const ModelOptionDef OPT_HistogramName = {
   MODOPT_ARG_STRING, "HistogramName", & MOC_TEXTONS, OPTEXP_CORE,
   "This option specifies the \"root\" name of the histogram entry in\n"
   "the training histograms database. The histogram number will be\n"
   "appended to this \"root\" name. The training histograms database\n"
   "is a plain text file containing one histogram entry per line. The\n"
   "first field specifies the name plus number of the entry (e.g.,\n"
   "foo_1, foo_2, bar_1, and so on). The remaining fields are simply the\n"
   "hundred numbers making up the image's universal textons histogram.\n\n"
   "In classification mode, this option specifies the name of the input\n"
   "image's histogram that is written to the results file.\n",
   "histogram-name", '\0', "histogram-name-root",
   TT_DEFAULT_TRAINING_HISTOGRAM_NAME,
} ;

#ifndef TT_DEFAULT_TRAINING_HISTOGRAMS_FILE
   #define TT_DEFAULT_TRAINING_HISTOGRAMS_FILE "training_histograms.txt"
#endif

const ModelOptionDef OPT_HistogramFile = {
   MODOPT_ARG_STRING, "HistogramFile", & MOC_TEXTONS, OPTEXP_CORE,
   "This option specifies the name of the training histograms database,\n"
   "a plain text file containing one histogram entry per line. The\n"
   "first field specifies the name plus number of the entry (e.g.,\n"
   "foo_1, foo_2, bar_1, and so on). The remaining fields are simply the\n"
   "hundred numbers making up the image's universal textons histogram.\n",
   "histogram-file", '\0', "training-histograms-file",
   TT_DEFAULT_TRAINING_HISTOGRAMS_FILE,
} ;

// In image classification mode, we write the results to the following
// file.
#ifndef TT_DEFAULT_CLASSIFICATION_RESULTS_FILE
   #define TT_DEFAULT_CLASSIFICATION_RESULTS_FILE "texton_classifications.txt"
#endif

const ModelOptionDef OPT_ResultsFile = {
   MODOPT_ARG_STRING, "ResultsFile", & MOC_TEXTONS, OPTEXP_CORE,
   "This option specifies the name of the classification results file,\n"
   "a plain text file containing one result entry per line. The first\n"
   "field specifies the name of the input image plus number of the entry,\n"
   "(e.g., foo_1, foo_2, bar_1, and so on). Then come the names of the\n"
   "top five matching images from the training set.\n",
   "results-file", '\0', "classification-results-file",
   TT_DEFAULT_CLASSIFICATION_RESULTS_FILE,
} ;

// The different operational modes of this program must be specified as
// the one and only non-option command line argument. This "action"
// command must be one of the following strings (case-sensitive!):
//
// 1. accumulate -- accumulate the training textons in the plain text
//    file specified by the --training-textons option (default is to
//    accumulate the training textons in training_textons.txt in the
//    current directory.
//
// 2. kmeans -- compute the universal textons from the training textons
//    using the K-means implementation in OpenCV.
//
//    The --training-textons option can be used to specify the input file
//    for the K-means and --universal-textons option can be used to
//    specify the output file. The defaults are to read from
//    training_textons.txt and write to universal_textons.mat (in the
//    current directory).
//
// 3. histogram -- compute the histograms for the training set. The
//    output is sent to the text file specified by the --histogram-file
//    option. It is a good idea to also supply the --histogram-name
//    option when saving training histograms from an MPEG. A good choice
//    of the entry's name would be the basename of the MPEG file sans
//    extension.
//
// 4. classify -- uses the universal textons and histograms produced by
//    the kmeans and histogram commands to classify the input images
//    streaming in.
#ifndef TT_ACCUMULATE_CMD
   #define TT_ACCUMULATE_CMD "accumulate"
#endif
#ifndef TT_KMEANS_CMD
   #define TT_KMEANS_CMD "kmeans"
#endif
#ifndef TT_HISTOGRAM_CMD
   #define TT_HISTOGRAM_CMD "histogram"
#endif
#ifndef TT_CLASSIFY_CMD
   #define TT_CLASSIFY_CMD "classify"
#endif

// For printing usage info
#ifndef TT_ACTIONS
   #define TT_ACTIONS ("{"TT_ACCUMULATE_CMD"|"TT_KMEANS_CMD"|"\
                          TT_HISTOGRAM_CMD"|"TT_CLASSIFY_CMD"}")
#endif

} // end of local namespace encapsulating command line options section

//--------------------- SIMULATION ENCAPSULATION ------------------------

// The following helper class wraps around the ModelManager and
// associated objects, providing a neatly encapsulated API for the main
// program.
namespace {

class TextonSimulation {
   ModelManager model_manager ;
   nub::soft_ref<SimEventQueueConfigurator> configurator ;
   nub::soft_ref<StdBrain> brain ;
   nub::ref<SimInputFrameSeries> input_frame_series ;

   // Various command line options specific to this program
   OModelParam<std::string> training_option ;
   OModelParam<std::string> universal_option ;
   OModelParam<std::string> hist_name_option ;
   OModelParam<std::string> hist_file_option ;
   OModelParam<std::string> results_option ;

public :
   TextonSimulation(const std::string& model_name) ;
   void parse_command_line(int argc, const char* argv[]) ;
   void run() ;
   ~TextonSimulation() ;

private :
   // The different actions performed by this program
   typedef void (TextonSimulation::*Action)() ;
   typedef std::map<std::string, Action> ActionMap ;
   ActionMap action_map ;

   void accumulate_training_textons() ;
   void compute_universal_textons() ;
   void compute_training_histograms() ;
   void classify_input_images() ;

   // Accessors for retrieving some of the command line arguments
   std::string training_textons_file()  {return training_option.getVal()  ;}
   std::string universal_textons_file() {return universal_option.getVal() ;}
   std::string histogram_name() {return hist_name_option.getVal() ;}
   std::string histogram_file() {return hist_file_option.getVal() ;}
   std::string results_file() {return results_option.getVal() ;}
} ;

// On instantiation, create the model manager and the simulation's
// various components.
TextonSimulation::TextonSimulation(const std::string& model_name)
   : model_manager(model_name),
     configurator(new SimEventQueueConfigurator(model_manager)),
     brain(new StdBrain(model_manager)),
     input_frame_series(new SimInputFrameSeries(model_manager)),
     training_option(& OPT_TrainingTextons, & model_manager),
     universal_option(& OPT_UniversalTextons, & model_manager),
     hist_name_option(& OPT_HistogramName, & model_manager),
     hist_file_option(& OPT_HistogramFile, & model_manager),
     results_option(& OPT_ResultsFile, & model_manager)
{
   model_manager.addSubComponent(configurator) ;
   model_manager.addSubComponent(brain) ;
   model_manager.addSubComponent(input_frame_series) ;

   typedef TextonSimulation me ; // typing shortcut
   action_map[TT_ACCUMULATE_CMD] = & me::accumulate_training_textons ;
   action_map[TT_KMEANS_CMD]     = & me::compute_universal_textons ;
   action_map[TT_HISTOGRAM_CMD]  = & me::compute_training_histograms ;
   action_map[TT_CLASSIFY_CMD]   = & me::classify_input_images ;
}

// TODO: Do we really need the single channel save raw maps option for
// this texton training program? And how can we force the gist estimator
// type to be always GistEstimatorTexton? This program doesn't make sense
// for any other gist estimator.
void TextonSimulation::parse_command_line(int argc, const char* argv[])
{
   model_manager.setOptionValString(& OPT_SingleChannelSaveRawMaps, "true") ;
   model_manager.setOptionValString(& OPT_GistEstimatorType, "Texton") ;
   model_manager.setOptionValString(& OPT_NumOrientations, "6") ;

   model_manager.setOptionValString(& OPT_TrainingTextons,
                                    TT_DEFAULT_TRAINING_TEXTONS_FILE) ;
   model_manager.setOptionValString(& OPT_UniversalTextons,
                                    TT_DEFAULT_UNIVERSAL_TEXTONS_FILE) ;

   model_manager.setOptionValString(& OPT_HistogramName,
                                    TT_DEFAULT_TRAINING_HISTOGRAM_NAME) ;
   model_manager.setOptionValString(& OPT_HistogramFile,
                                    TT_DEFAULT_TRAINING_HISTOGRAMS_FILE) ;

   model_manager.setOptionValString(& OPT_ResultsFile,
                                    TT_DEFAULT_CLASSIFICATION_RESULTS_FILE) ;

   if (! model_manager.parseCommandLine(argc, argv, TT_ACTIONS, 1, 1))
      throw std::runtime_error("command line parse error") ;
}

// To run the simulation, we simply dispatch to the function
// corresponding to the action (non-option) command line argument.
void TextonSimulation::run()
{
   std::string cmd(model_manager.getExtraArg(0)) ;
   ActionMap::iterator action = action_map.find(cmd) ;
   if (action == action_map.end())
      throw std::runtime_error(cmd + ": sorry, unknown action") ;
   (this->*(action->second))() ;
}

// Quick helper class to start and stop model manager (useful when
// exceptions are thrown because destructor automatically stops the model
// manager without requiring an explicit call to the stop method prior to
// throwing the exception).
class ModelManagerStarter {
   ModelManager& mgr ;
public :
   ModelManagerStarter(ModelManager& m) : mgr(m) {mgr.start() ;}
   ~ModelManagerStarter() {mgr.stop() ;}
} ;

// This method implements the simulation's main loop for the "accumulate"
// action. The main loop which evolves the different components of the
// simulation. Prior to starting the main loop though, it configures the
// texton gist estimator's training callback, which is triggered at each
// step of the brain's evolution. The texton gist estimator passes the
// textons for the "current" input image to this callback, which then
// proceeds to accumulate the textons in the file specified by the
// --training-textons option.
//
// The code for the actual accumulation is in the TEXTON ACCUMULATION
// section of this file.
void TextonSimulation::accumulate_training_textons()
{
   ModelManagerStarter M(model_manager) ;

   LFATAL("sorry, this gist program is broken and needs to be fixed") ;
   /*
   nub::soft_ref<GistEstimatorTexton> ge =
      dynCastWeak<GistEstimatorTexton>(brain->getGE()) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorTexton") ;

   ge->setTrainingHook(accumulate_textons) ;
   textons_accumulator::output_file(training_textons_file()) ;

   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         input_frame_series->evolve(*event_queue) ;
         brain->evolve(*event_queue) ; // triggers training hook
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         return ; // prevent LFATAL induced abortion
      }
   }
   // */
}

// The following method implements the "kmeans" action of this program
// for clustering the training textons to obtain the 100 universal
// textons.
void TextonSimulation::compute_universal_textons()
{
   LINFO("MVN: counting lines in %s", training_textons_file().c_str()) ;
   int num_rows = count_lines(training_textons_file()) ;

   LINFO("MVN: reading %d training textons from %s",
         num_rows, training_textons_file().c_str()) ;
   OpenCVMatrix training_textons =
      load_training_textons(training_textons_file(), num_rows) ;

   const int K = TT_NUM_UNIVERSAL_TEXTONS ;
   LINFO("MVN: doing K-means on training textons to get %d clusters", K) ;
   OpenCVMatrix universal_textons = kmeans(K, training_textons) ;

   LINFO("MVN: K-means done; saving universal textons to %s",
         universal_textons_file().c_str()) ;
   save_universal_textons(universal_textons, universal_textons_file()) ;
}

// This method implements the "histogram" action of this program. Like
// the accumulate action, it implements a "main loop" for the simulation,
// evolving different components with each iteration. But rather than
// dipping into the GistEstimatorTexton's processing pipeline, it starts
// the Matlab engine, loads the universal textons and then uses the
// GistEstimatorTexton to obtain the histogram for each of the training
// images. These histograms are saved to the training histograms database
// specified by the --histogram-file option.
void TextonSimulation::compute_training_histograms()
{
   ModelManagerStarter M(model_manager) ;

   LFATAL("sorry, this gist program is broken and needs to be fixed") ;
   /*
   nub::soft_ref<GistEstimatorTexton> ge =
      dynCastWeak<GistEstimatorTexton>(brain->getGE()) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorTexton") ;

   Texton U = load_universal_textons(universal_textons_file()) ;
   ge->setUniversalTextons(& U) ;
   LINFO("MVN: loaded %d universal textons from %s",
         U.getHeight(), universal_textons_file().c_str()) ;

   int i = 1 ;
   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         input_frame_series->evolve(*event_queue) ;
         brain->evolve(*event_queue) ;
         SeC<SimEventGistOutput> gist_out =
            event_queue->check<SimEventGistOutput>(brain.get(),
                                                   SEQ_UNMARKED | SEQ_MARK,
                                                   ge.get()) ;
         if (gist_out) // texton GE has a gist vector waiting to be picked up
            save_histogram(ge->getGist(), histogram_name() + to_string(i++),
                           histogram_file()) ;
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         return ; // prevent LFATAL induced abortion
      }
   }
   // */
}

// The following method implements this program's "classify" action. It
// reads the training histograms database and the universal textons and
// then uses a chi-square measure to compute the closest match for the
// input image.
void TextonSimulation::classify_input_images()
{
   ModelManagerStarter M(model_manager) ;

   LFATAL("sorry, this gist program is broken and needs to be fixed") ;
   /*
   nub::soft_ref<GistEstimatorTexton> ge =
      dynCastWeak<GistEstimatorTexton>(brain->getGE()) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorTexton") ;

   Texton U = load_universal_textons(universal_textons_file()) ;
   ge->setUniversalTextons(& U) ;
   LINFO("MVN: loaded %d universal textons from %s",
         U.getHeight(), universal_textons_file().c_str()) ;

   HistogramMap training_histograms =
      load_training_histograms(histogram_file()) ;
   LINFO("MVN: loaded %d training histograms from %s",
         int(training_histograms.size()), histogram_file().c_str()) ;

   int i = 1 ;
   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         input_frame_series->evolve(*event_queue) ;
         brain->evolve(*event_queue) ;
         SeC<SimEventGistOutput> gist_out =
            event_queue->check<SimEventGistOutput>(brain.get(),
                                                   SEQ_UNMARKED | SEQ_MARK,
                                                   ge.get()) ;
         if (gist_out) // texton GE has a gist vector waiting to be picked up
            classify_image(std::make_pair(histogram_name() + to_string(i++),
                                          ge->getGist()),
                           training_histograms, results_file()) ;
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         return ; // prevent LFATAL induced abortion
      }
   }
   // */
}

// Do we really not have to delete the configurator, brain and input
// frame series? If it turns out we do, this empty destructor will have
// to be filled out with the necessary delete calls...
TextonSimulation::~TextonSimulation() {}

} // end of local namespace encapsulating simulation encapsulation section

//------------------------------- MAIN ----------------------------------

#ifdef HAVE_OPENCV

int main(int argc, const char* argv[])
{
   MYLOGVERB = LOG_INFO ; // suppress debug messages
   try
   {
      TextonSimulation S("train-texton Model") ;
      S.parse_command_line(argc, argv) ;
      S.run() ;
   }
   catch (std::exception& e)
   {
      LFATAL("%s", e.what()) ;
      return 1 ;
   }
   return 0 ;
}

#else

int main()
{
   LINFO("Sorry, this program needs OpenCV.") ;
   return 1 ;
}

#endif

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
