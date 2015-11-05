/**
   \file Gist/train-bbof.C

   \brief Interface for training and testing GistEstimatorBeyondBoF.

   The train-bbof program in conjunction with the GistEstimatorBeyondBoF
   class implements the following paper within the INVT framework:

   Lazebnik, S., Schmid, C., Ponce, J.
   Beyond Bags of Features: Spatial Pyramid Matching for Recognizing
      Natural Scene Catgories
   CVPR, 2006.

   Whereas the GistEstimatorBeyondBoF class is only concerned with the
   portions of the above paper that deal with gist vector computations,
   this program provides the remaining structure required to implement
   the necessary training and image classification functionalities.

   train-bbof has two modes of operation, viz., training and testing.
   Training mode consists of four distinct phases: SIFT descriptor
   accumulation, K-means clustering, training histograms collection, and
   SVM generation. Testing mode operates in a single phase that uses the
   results of the clustering, histograms collection and SVM generation
   training phases to classify input images into appropriate categories.
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/train-bbof.C $
// $Id: train-bbof.C 14605 2011-03-15 02:25:06Z dparks $
//

//--------------------------- LIBRARY CHECK -----------------------------

#if !defined(HAVE_OPENCV) || !defined(INVT_HAVE_LIBTORCH)

#include "Util/log.H"

int main()
{
   LERROR("Sorry, this program needs the OpenCV and torch libraries.") ;
   return 255 ;
}

#else // the actual program in all its hideous glory

//------------------------------ HEADERS --------------------------------

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

#include <fstream>

// Gist specific headers
#include "Neuro/GistEstimatorBeyondBoF.H"

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

// torch headers
#include <torch/general.h>
#include <torch/QCTrainer.h>
#include <torch/SVMClassification.h>
#include <torch/Kernel.h>
#include <torch/MatDataSet.h>

// Unix headers
#include <glob.h>
#include <unistd.h>

// Standard C++ headers
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

//------------------------ TEMPLATE UTILITIES ---------------------------

// Convenient (but perhaps not the most efficient) helper to convert
// various data types to strings.
//
// DEVNOTE: Works as long as type T defines an operator << that writes to
// an ostream.
template<typename T>
static std::string to_string(const T& t)
{
   std::ostringstream str ;
   str << t ;
   return str.str() ;
}

/// Read from string. As above, works as long as type T defines an
/// operator >> that reads from an istream.
template<typename T>
static T from_string(const std::string& s, const T& defval = T())
{
   T t(defval) ;
   std::istringstream str(s) ;
   str >> t ;
   return t ;
}

/// from_string() partial specialization for strings. If the client wants
/// a string from the input string, we just return the input string. If
/// we were apply the default version of this template function, we would
/// end up parsing the input string as a whitespace separated string
/// stream and only return the first string from this stream.
template<>
std::string from_string(const std::string& s, const std::string&)
{
   return s ;
}

//----------------------- COMMAND LINE OPTIONS --------------------------

/**
   This program has five distinct phases/modes of operation, each one
   specified via a suitable non-option command line argument.
   Additionally, it supports several command line options to allow users
   to tweak various parameters such as the name of the vocabulary file,
   the training histograms database, and so on.
*/
namespace {

const ModelOptionCateg MOC_BBOF = {
   MOC_SORTPRI_3,
   "Options specific to the Beyond Bag-of-Features program",
} ;

/// In the SIFT descriptors accumulation phase, we collect all the
/// descriptors from the training images and store them in a plain text
/// file.
#ifndef BBOF_DEFAULT_TRAINING_DESCRIPTORS_FILE
   #define BBOF_DEFAULT_TRAINING_DESCRIPTORS_FILE "sift_descriptors.txt"
#endif

const ModelOptionDef OPT_SiftDescriptors = {
   MODOPT_ARG_STRING, "SiftDescriptors", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the name of the file where SIFT descriptors\n"
   "for the training images are to be accumulated. This is a plain text\n"
   "file containing the descriptors that will be fed into the K-means\n"
   "procedure during the second training phase.\n",
   "sift-descriptors", '\0', "sift-descriptors-file",
   BBOF_DEFAULT_TRAINING_DESCRIPTORS_FILE,
} ;

/// In the second phase of training, we perform K-means clustering on the
/// SIFT descriptors accumulated in the first phase and store the results
/// in yet another plain text file.
#ifndef BBOF_DEFAULT_VOCABULARY_FILE
   #define BBOF_DEFAULT_VOCABULARY_FILE "sift_vocabulary.txt"
#endif

const ModelOptionDef OPT_SiftVocabulary = {
   MODOPT_ARG_STRING, "SiftVocabulary", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the name of the file in which the \"prototypical\"\n"
   "SIFT descriptors are (or are to be) stored. This is a plain text\n"
   "file containing the centroids of the K-means clusters, which are used\n"
   "during gist vector computation to create feature maps and, subsequently,\n"
   "the multi-level histograms using the spatial matching pyramid as\n"
   "described in the Lazebnik paper.\n",
   "sift-vocabulary", '\0', "sift-vocabulary-file",
   BBOF_DEFAULT_VOCABULARY_FILE,
} ;

/// In the third phase of training, we compute and store the gist vectors
/// for the training images. These gist vectors are used in the next
/// training phase as the data points that will be used to create
/// appropriate SVM classifiers for each image category.
#ifndef BBOF_DEFAULT_TRAINING_HISTOGRAMS_FILE
   #define BBOF_DEFAULT_TRAINING_HISTOGRAMS_FILE "training_histograms.txt"
#endif

const ModelOptionDef OPT_HistogramsFile = {
   MODOPT_ARG_STRING, "HistogramsFile", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the name of the training histograms database,\n"
   "a plain text file containing one histogram entry per line. The\n"
   "first field specifies the name plus number of the entry (e.g.,\n"
   "foo.mpg:1, bar.mpg:5, and so on). The second field specifies the ground\n"
   "truth for this particular image. The remaining fields are simply the\n"
   "4200 numbers making up the image's flattened out multi-level histogram,\n"
   "which serves as its gist vector.\n",
   "training-histograms", '\0', "training-histograms-file",
   BBOF_DEFAULT_TRAINING_HISTOGRAMS_FILE,
} ;

/// In the fourth phase of training, we create SVM classifiers for each
/// of the categories and store the relevant parameters to a text file
/// for later use during image classification. Each segment will have its
/// own SVM classifier. Therefore, the default value of this symbol is
/// not a good one to use and it should be explicitly specified on the
/// command line.
#ifndef BBOF_DEFAULT_SVM_CLASSIFIER_FILE
   #define BBOF_DEFAULT_SVM_CLASSIFIER_FILE "svm_classifier.txt"
#endif

const ModelOptionDef OPT_SvmClassifierFile = {
   MODOPT_ARG_STRING, "SvmClassifierFile", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the name of the file that will hold the SVM\n"
   "classifier for a given segment. This file is read and written by the\n"
   "torch library.",
   "svm-classifier", '\0', "svm-classifier-file",
   BBOF_DEFAULT_SVM_CLASSIFIER_FILE,
} ;

/// While creating SVM classifiers for each of the categories, we need a
/// temp file to store the training histograms data in the format
/// required by the torch library. Usually, it would be a good idea to
/// explicitly specify this on the command line rather than relying on
/// the compiled in default.
#ifndef BBOF_DEFAULT_SVM_TEMP_FILE
   #define BBOF_DEFAULT_SVM_TEMP_FILE "/tmp/train-bbof-torch-dataset.txt"
#endif

const ModelOptionDef OPT_SvmTempFile = {
   MODOPT_ARG_STRING, "SvmTempFile", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the name of the temp file that will hold the SVM\n"
   "training data in the format required by the torch library. This file is\n"
   "is automatically deleted when it is no longer required.",
   "svm-temp", '\0', "svm-temp-file",
   BBOF_DEFAULT_SVM_TEMP_FILE,
} ;

/// In image classification mode, we write the results to a plain text
/// file.
#ifndef BBOF_DEFAULT_CLASSIFICATION_RESULTS_FILE
   #define BBOF_DEFAULT_CLASSIFICATION_RESULTS_FILE "bbof_classifications.txt"
#endif

const ModelOptionDef OPT_ResultsFile = {
   MODOPT_ARG_STRING, "ResultsFile", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the name of the classification results file,\n"
   "a plain text file containing one result entry per line. The first\n"
   "field specifies the name of the input image plus number of the entry,\n"
   "(e.g., foo.mpg:1, bar.mpg:5, and so on). Then comes the ground truth\n"
   "for this image followed by its classification result.\n",
   "results-file", '\0', "classification-results-file",
   BBOF_DEFAULT_CLASSIFICATION_RESULTS_FILE,
} ;

/// Several of the data files output by different operational modes of
/// this program require inclusion of the current image/frame name and
/// number and the ground truth segment/category number. These options
/// allow users to specify appropriate values for this required info.
///
/// NOTE: The default values for these options are not very useful.
/// They really ought to be explicitly specified on the command line.
#ifndef BBOF_DEFAULT_IMAGE_NAME
   #define BBOF_DEFAULT_IMAGE_NAME "some_image"
#endif
#ifndef BBOF_DEFAULT_SEGMENT_NUMBER
   #define BBOF_DEFAULT_SEGMENT_NUMBER "0"
#endif

const ModelOptionDef OPT_ImageName = {
   MODOPT_ARG_STRING, "ImageName", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the \"root\" name for an image. The image number\n"
   "will be automatically appended to this \"root\" name with a colon as the\n"
   "separator between name and frame number. The current input MPEG file\n"
   "name is a good choice for the value of this option.\n",
   "image-name", '\0', "input-MPEG-file-name",
   BBOF_DEFAULT_IMAGE_NAME,
} ;

const ModelOptionDef OPT_SegmentNumber = {
   MODOPT_ARG_STRING, "SegmentNumber", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the segment number for an image in the training\n"
   "set. The segment number is used to specify the ground truth for the\n"
   "image classification.\n",
   "segment-number", '\0', "image-segment-number",
   BBOF_DEFAULT_SEGMENT_NUMBER,
} ;

/// classification can be performed either with input images or with
/// precomputed gist vectors. These precomputed vectors are stored in a
/// plain text file. This is the default name of that file.
#ifndef BBOF_DEFAULT_GIST_VECTORS_FILE
   #define BBOF_DEFAULT_GIST_VECTORS_FILE "gist_vectors.txt"
#endif

const ModelOptionDef OPT_GistVectors = {
   MODOPT_ARG_STRING, "GistVectors", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the name of the file in which gist vectors are\n"
   "stored. This option is used when classification must be performed\n"
   "using gist vectors computed by some other entity. This is useful, for\n"
   "example, when a client program running on some other host computes gist\n"
   "vectors and passes these vectors to this program for classification.\n"
   "In such situations, we do not need to read images and compute gist\n"
   "vectors for them. Instead, we bypass all that and simply perform the\n"
   "classification using the precomputed vectors.\n",
   "gist-vectors", '\0', "gist-vectors-file",
   BBOF_DEFAULT_GIST_VECTORS_FILE,
} ;

/// The vocabulary consists of a bunch of "prototypical" SIFT descriptors
/// that are obtained by clustering the SIFT descriptors for the training
/// images. It is possible to change the size of the vocabulary. But the
/// default is 200.
#ifndef BBOF_DEFAULT_VOCABULARY_SIZE
   #define BBOF_DEFAULT_VOCABULARY_SIZE "200"
#endif

const ModelOptionDef OPT_VocabularySize = {
   MODOPT_ARG_STRING, "VocabularySize", & MOC_BBOF, OPTEXP_CORE,
   "This option specifies the size of the SIFT vocabulary.\n",
   "vocabulary-size", '\0', "vocabulary-size",
   BBOF_DEFAULT_VOCABULARY_SIZE,
} ;

/**
   The different operational modes of this program must be specified as
   the one and only non-option command line argument. This "action"
   command must be one of the following strings (case-sensitive!):

   1. sift -- accumulate the SIFT descriptors for the training images
      in the plain text file specified by the --sift-descriptors option.
      By default, the descriptors will be accumulated in
      ./sift_descriptors.txt.

      Additionally, the --image-name and --segment-number options are
      required as this information is also recorded in the SIFT
      descriptors file.

   2. vocab -- compute the SIFT descriptors vocabulary, i.e., the
      "protototypical" SIFT descriptors, from the accumulated
      descriptors using the OpenCV K-means implementation.

      For this action, the --sift-descriptors option specifies the input
      file for the K-means while the --sift-vocabulary option specifies
      the output file. The defaults are to read from
      ./sift_descriptors.txt and write to ./sift_vocabulary.txt.

   3. hist -- compute the flattened out multi-level histograms for the
      training set. The output is sent to the text file specified by the
      --histograms-file option.

      The --image-name and --segment-number options are also required.

   4. svm -- generate the SVM classifiers for each of the categories.
      The --svm-classifier file specifies the name of the file to which
      the SVM parameters will be stored. By default, this is
      ./svm_classifier.txt. Users should supply a file name different
      from the default. Otherwise, this file will get overwritten for
      each segment.

      The --histograms-file can be used to specify the input data for
      this action.

      In addition to the above two options, this action also needs the
      --svm-temp option to store the histograms data in the format
      required by the torch library. The default value is okay for this
      option. However, if several instances of this program can be
      executing in parallel, it would be best to supply different temp
      files explicitly on the command line.

   5. classify -- uses the vocabulary and SVM classifiers produced by the
      vocab and svm actions to classify the input images streaming in.
      Classification results are written, by default, to
      ./classification_results.txt; but this can be changed with the
      --results-file option.

      The --sift-vocabulary and --svm-classifier options can be used to
      specify appropriate values for the different pieces of input
      required by the classify action. Note that the --svm-classifier
      option does not point to a specific classifier, but really is a
      "root" name to use. This program will automatically load all the
      classifiers that begin with is this root. For example, if the user
      specifies --svm-classifier="ACB_svm_classifier", this program will
      load all the classifiers whose file names begin with
      "ACB_svm_classifier." and append numbers starting at 1.

   6. classify_gv -- same action as classify except that input images are
      not used; instead, we assume that some other entity (e.g., a client
      application running on a phone) has computed the gist vector for
      one or more images and is passing these vectors via the
      --gist-vector option.
*/
#ifndef BBOF_SIFT_CMD
   #define BBOF_SIFT_CMD "sift"
#endif
#ifndef BBOF_VOCABULARY_CMD
   #define BBOF_VOCABULARY_CMD "vocab"
#endif
#ifndef BBOF_HISTOGRAM_CMD
   #define BBOF_HISTOGRAM_CMD "hist"
#endif
#ifndef BBOF_SVM_CMD
   #define BBOF_SVM_CMD "svm"
#endif
#ifndef BBOF_CLASSIFY_CMD
   #define BBOF_CLASSIFY_CMD "classify"
#endif
#ifndef BBOF_CLASSIFY_GV_CMD
   #define BBOF_CLASSIFY_GV_CMD "classify_gv"
#endif

// For printing usage info
#ifndef BBOF_ACTIONS
   #define BBOF_ACTIONS ("{"BBOF_SIFT_CMD"|"BBOF_VOCABULARY_CMD"|"\
                          BBOF_HISTOGRAM_CMD"|"BBOF_SVM_CMD"|"\
                          BBOF_CLASSIFY_CMD"|"BBOF_CLASSIFY_GV_CMD"}")
#endif

} // end of local namespace encapsulating command line options section

//--------------------- SIMULATION ENCAPSULATION ------------------------

// The following helper class wraps around the ModelManager and
// associated objects, providing a neatly encapsulated API for the main
// program.
namespace {

class BBoFSimulation {
   ModelManager model_manager ;
   nub::soft_ref<SimEventQueueConfigurator> configurator ;
   nub::soft_ref<StdBrain> brain ;
   nub::ref<SimInputFrameSeries> input_frame_series ;

   // Various command line options specific to this program
   OModelParam<std::string> sd_option ; // --sift-descriptors
   OModelParam<std::string> sv_option ; // --sift-vocabulary
   OModelParam<std::string> th_option ; // --training-histograms
   OModelParam<std::string> sc_option ; // --svm-classifier
   OModelParam<std::string> st_option ; // --svm-temp
   OModelParam<std::string> rf_option ; // --results-file
   OModelParam<std::string> in_option ; // --image-name (not --in!)
   OModelParam<std::string> sn_option ; // --segment-number
   OModelParam<std::string> gv_option ; // --gist-vectors
   OModelParam<std::string> vs_option ; // --vocabulary-size

public :
   BBoFSimulation(const std::string& model_name) ;
   void parse_command_line(int argc, const char* argv[]) ;
   void run() ;
   ~BBoFSimulation() ;

private :
   // The different actions performed by this program
   typedef void (BBoFSimulation::*Action)() ;
   typedef std::map<std::string, Action> ActionMap ;
   ActionMap action_map ;

   void accumulate_sift_descriptors() ;
   void compute_sift_vocabulary() ;
   void compute_training_histograms() ;
   void generate_svm_classifier() ;
   void classify_input_images() ;
   void classify_using_gist_vectors() ;

   // Accessors for retrieving some of the command line arguments
   std::string sift_descriptors_file() {return sd_option.getVal() ;}
   std::string sift_vocabulary_file()  {return sv_option.getVal() ;}
   std::string histograms_file()       {return th_option.getVal() ;}
   std::string svm_classifier_file()   {return sc_option.getVal() ;}
   std::string svm_temp_file()         {return st_option.getVal() ;}
   std::string results_file()          {return rf_option.getVal() ;}
   std::string image_name()            {return in_option.getVal() ;}
   std::string segment_number()        {return sn_option.getVal() ;}
   std::string gist_vectors_file()     {return gv_option.getVal() ;}
   int vocabulary_size() {return from_string<int>(vs_option.getVal()) ;}
} ;

// On instantiation, create the model manager and the simulation's
// various components.
BBoFSimulation::BBoFSimulation(const std::string& model_name)
   : model_manager(model_name),
     configurator(new SimEventQueueConfigurator(model_manager)),
     brain(new StdBrain(model_manager)),
     input_frame_series(new SimInputFrameSeries(model_manager)),
     sd_option(& OPT_SiftDescriptors,   & model_manager),
     sv_option(& OPT_SiftVocabulary,    & model_manager),
     th_option(& OPT_HistogramsFile,    & model_manager),
     sc_option(& OPT_SvmClassifierFile, & model_manager),
     st_option(& OPT_SvmTempFile,       & model_manager),
     rf_option(& OPT_ResultsFile,       & model_manager),
     in_option(& OPT_ImageName,         & model_manager),
     sn_option(& OPT_SegmentNumber,     & model_manager),
     gv_option(& OPT_GistVectors,       & model_manager),
     vs_option(& OPT_VocabularySize,    & model_manager)
{
   model_manager.addSubComponent(configurator) ;
   model_manager.addSubComponent(brain) ;
   model_manager.addSubComponent(input_frame_series) ;

   typedef BBoFSimulation me ; // typing shortcut
   action_map[BBOF_SIFT_CMD]        = & me::accumulate_sift_descriptors ;
   action_map[BBOF_VOCABULARY_CMD]  = & me::compute_sift_vocabulary ;
   action_map[BBOF_HISTOGRAM_CMD]   = & me::compute_training_histograms ;
   action_map[BBOF_SVM_CMD]         = & me::generate_svm_classifier ;
   action_map[BBOF_CLASSIFY_CMD]    = & me::classify_input_images ;
   action_map[BBOF_CLASSIFY_GV_CMD] = & me::classify_using_gist_vectors ;
}

void BBoFSimulation::parse_command_line(int argc, const char* argv[])
{
   model_manager.setOptionValString(& OPT_GistEstimatorType, "BBoF") ;

   model_manager.setOptionValString(& OPT_SiftDescriptors,
                                    BBOF_DEFAULT_TRAINING_DESCRIPTORS_FILE) ;
   model_manager.setOptionValString(& OPT_SiftVocabulary,
                                    BBOF_DEFAULT_VOCABULARY_FILE) ;
   model_manager.setOptionValString(& OPT_HistogramsFile,
                                    BBOF_DEFAULT_TRAINING_HISTOGRAMS_FILE ) ;
   model_manager.setOptionValString(& OPT_SvmClassifierFile,
                                    BBOF_DEFAULT_SVM_CLASSIFIER_FILE ) ;
   model_manager.setOptionValString(& OPT_SvmTempFile,
                                    BBOF_DEFAULT_SVM_TEMP_FILE ) ;
   model_manager.setOptionValString(& OPT_ResultsFile,
                                    BBOF_DEFAULT_CLASSIFICATION_RESULTS_FILE) ;

   model_manager.setOptionValString(& OPT_ImageName,
                                    BBOF_DEFAULT_IMAGE_NAME) ;
   model_manager.setOptionValString(& OPT_SegmentNumber,
                                    BBOF_DEFAULT_SEGMENT_NUMBER) ;

   model_manager.setOptionValString(& OPT_GistVectors,
                                    BBOF_DEFAULT_GIST_VECTORS_FILE) ;

   model_manager.setOptionValString(& OPT_VocabularySize,
                                    BBOF_DEFAULT_VOCABULARY_SIZE) ;

   if (! model_manager.parseCommandLine(argc, argv, BBOF_ACTIONS, 1, 1))
      throw std::runtime_error("command line parse error") ;
}

// To run the simulation, we simply dispatch to the function
// corresponding to the action (non-option) command line argument.
void BBoFSimulation::run()
{
   std::string cmd(model_manager.getExtraArg(0)) ;
   ActionMap::iterator action = action_map.find(cmd) ;
   if (action == action_map.end())
      throw std::runtime_error(cmd + ": sorry, unknown action") ;
   (this->*(action->second))() ;
}

// Do we really not have to delete the configurator, brain and input
// frame series? If it turns out we do, this empty destructor will have
// to be filled out with the necessary delete calls...
BBoFSimulation::~BBoFSimulation(){}

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

} // end of local namespace encapsulating simulation encapsulation section

//------------------------------- MAIN ----------------------------------

int main(int argc, const char* argv[])
{
   MYLOGVERB = LOG_INFO ; // suppress debug messages
   try
   {
      BBoFSimulation S("train-bbof Model") ;
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

//------------------- SIFT DESCRIPTORS ACCUMULATION ---------------------

// This section contains the code for accumulating the SIFT descriptors
// of the training images, i.e., phase one of training.
namespace {

// Useful shortcut
typedef GistEstimatorBeyondBoF::SiftGrid SiftGrid ;

// Quick helper for storing the SIFT descriptors of the training images
// to a file.
class sift_descriptors_accumulator {
   sift_descriptors_accumulator() ; // private to disallow instantiation
   ~sift_descriptors_accumulator() ;
public :
   static std::string output_file ;
   static std::string image_name ;
   static int         frame_number ;
   static std::string segment_number ;

   static void write(const SiftGrid&) ;
} ;

// This method implements the simulation's main loop for the "sift"
// action. Prior to starting the main loop though, it configures the
// BBoF gist estimator's training callback, which is triggered at each
// step of the brain's evolution. The BBoF gist estimator passes the
// SIFT descriptors for the current input image to this callback, which
// then proceeds to accumulate them in the file specified by the
// --sift-descriptors option.
void BBoFSimulation::accumulate_sift_descriptors()
{
   ModelManagerStarter M(model_manager) ;

   nub::soft_ref<GistEstimatorBeyondBoF> ge =
      dynCastWeak<GistEstimatorBeyondBoF>(
         model_manager.subComponent("GistEstimatorBeyondBoF", MC_RECURSE)) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorBeyondBoF") ;

   typedef sift_descriptors_accumulator acc ;
   acc::output_file    = sift_descriptors_file() ;
   acc::image_name     = image_name() ;
   acc::segment_number = segment_number() ;
   ge->setTrainingHook(acc::write) ;

   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         acc::frame_number = input_frame_series->frame() ;
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         return ; // prevent LFATAL induced abortion
      }
   }
}

// Static data members for storing the SIFT descriptors file name and
// other pertinent info persistently across multiple invocations of the
// GistEstimatorBeyondBoF's training hook.
std::string sift_descriptors_accumulator::output_file ;
std::string sift_descriptors_accumulator::image_name ;
int         sift_descriptors_accumulator::frame_number ;
std::string sift_descriptors_accumulator::segment_number ;

// The following function is meant to be used as the
// GistEstimatorBeyondBoF training hook. It simply appends the SIFT
// descriptors grid passed to it (stored as an
// Image<GistEstimatorBeyondBoF::SiftDescriptor>) to the output file.
// The format of this file is as shown below:
//
// MPEG-file-name:frame-number  segment-number  row  col  SIFT-descriptor
//
// The MPEG file name should be explicitly specified with the
// --image-name option. The frame number is extracted automatically from
// the input frame series. The segment number represents the ground
// truth for the input image's category and should be specified
// explicitly on the command line with the --segment-number option. The
// row and col values are the SIFT grid coordinates. And, finally, the
// SIFT descriptor itself consists of 128 numbers.
//
// DEVNOTE: We could open the output file once and use that object to
// avoid reopening (by using a static ostream data member rather than a
// static string). However, if the program were to somehow crash halfway
// through, then the training SIFT descriptors output file would be in an
// inconsistent state and rerunning the program can result in appending
// data to a possibly inconsistent dataset, which would only make things
// worse.
//
// Thus, we choose to open and close the output file each time the
// GistEstimatorBeyondBoF training hook is triggered. (Of course, if the
// program cashes while this function is executing, then all bets are
// off; the training SIFT descriptors file's inconsistency will be
// unavoidable in this case.)
void sift_descriptors_accumulator::write(const SiftGrid& G)
{
   if (output_file.empty())
      throw std::runtime_error("SIFT descriptors accumulator output file "
                               "not specified") ;

   std::ofstream ofs(output_file.c_str(), std::ios::out | std::ios::app) ;
   for (int y = 0; y < G.getHeight(); ++y)
      for (int x = 0; x < G.getWidth(); ++x)
         ofs << image_name << ':' << frame_number << ' '
             << segment_number << ' '
             << y << ' ' << x << ' ' << G.getVal(x, y) << '\n' ;
}

} // end of local namespace encapsulating SIFT descriptors accumulation section

//-------------------------- OPENCV MATRICES ----------------------------

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

} // end of local namespace encapsulating OpenCV matrices section

//-------------------- SIFT VOCABULARY COMPUTATION ----------------------

// This section contains the code for the K-means clustering of the SIFT
// descriptors of the training images (i.e., training phase two).
namespace {

// Useful types
typedef Image<float> Vocabulary ;

// Forward declarations
int    count_lines(const std::string& file_name) ;
CvMat* load_sift_descriptors(const std::string& file_name, int num_lines) ;
CvMat* kmeans(int K, const OpenCVMatrix& data) ;
void   save_vocabulary(const OpenCVMatrix&, const std::string& file_name) ;

// The following method implements the "vocab" action of this program
// for clustering the SIFT descriptors of the training images to obtain
// the 200 "prototypical" SIFT descriptors that form the basis of the
// gist vector computation in terms of these "words" or "vis-terms".
void BBoFSimulation::compute_sift_vocabulary()
{
   LINFO("MVN: counting lines in %s", sift_descriptors_file().c_str()) ;
   int num_rows = count_lines(sift_descriptors_file()) ;

   LINFO("MVN: reading %d SIFT descriptors from %s",
         num_rows, sift_descriptors_file().c_str()) ;
   OpenCVMatrix sift_descriptors =
      load_sift_descriptors(sift_descriptors_file(), num_rows) ;

   //GistEstimatorBeyondBoF::num_channels(vocabulary_size()) ;
   //const int K = GistEstimatorBeyondBoF::num_channels() ;
   const int K = vocabulary_size() ;
   LINFO("MVN: doing K-means on SIFT descriptors to get %d clusters", K) ;
   OpenCVMatrix vocabulary = kmeans(K, sift_descriptors) ;

   LINFO("MVN: K-means done; saving SIFT vocabulary to %s",
         sift_vocabulary_file().c_str()) ;
   save_vocabulary(vocabulary, sift_vocabulary_file()) ;
}

// The following function reads the SIFT descriptors for the training
// images into an OpenCV matrix. It must know how many lines the SIFT
// descriptors file has. This quantity is the number of rows in resulting
// matrix. The number of columns is simply the size of each SIFT
// descriptor (usually: 128 values make up a SIFT descriptor).
CvMat* load_sift_descriptors(const std::string& file_name, int num_rows)
{
   int num_cols = GistEstimatorBeyondBoF::SiftDescriptor::SIZE ;
   CvMat* M = cvCreateMat(num_rows, num_cols, CV_32FC1) ;

   double d ; std::string dummy ; // for ignoring first four fields
   std::ifstream ifs(file_name.c_str()) ;
   for (int i = 0; i < num_rows; ++i)
   {
      std::string str ;
      std::getline(ifs, str) ;
      if (! ifs || str.empty()) {
        if (i == num_rows - 1) // okay; read all rows
          break ;
        else { // descriptors file missing data or some other error
          cvReleaseMat(& M) ;
          throw std::runtime_error(file_name +
                                   ": missing SIFT descriptors or other read error") ;
        }
      }
      std::istringstream line(str) ;
      line >> dummy >> dummy >> dummy >> dummy ;

      for (int j = 0; j < num_cols; ++j) {
         if (! line) {
            cvReleaseMat(& M) ;
            throw std::runtime_error(file_name +
               ": missing SIFT descriptor values on line " + to_string(i)) ;
         }
         line >> d ;
         cvmSet(M, i, j, d) ;
      }
   }

   return M ;
}

// K-means parameters
#ifndef BBOF_KMEANS_ITERATIONS
   #define BBOF_KMEANS_ITERATIONS (100)
#endif
#ifndef BBOF_KMEANS_PRECISION
   #define BBOF_KMEANS_PRECISION (.01)
#endif

// Forward declaration
CvMat* compute_centroids(int K, const OpenCVMatrix& data,
                         const OpenCVMatrix& cluster_assignments) ;

// This function performs K-means clustering on the supplied data matrix
// and returns the cluster centers.
CvMat* kmeans(int K, const OpenCVMatrix& data)
{
   OpenCVMatrix cluster_assignments(data.num_rows(), 1, CV_32SC1) ;

   LINFO("MVN: computing K-means cluster assignments with OpenCV") ;
   cvKMeans2(data, K, cluster_assignments,
             cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                            BBOF_KMEANS_ITERATIONS, BBOF_KMEANS_PRECISION)) ;

   LINFO("MVN: cluster assignments done; computing centroids...") ;
   return compute_centroids(K, data, cluster_assignments) ;
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

// Write the SIFT vocabulary, row by row, to a plain text file.
void save_vocabulary(const OpenCVMatrix& vocabulary,
                     const std::string& file_name)
{
   std::ofstream ofs(file_name.c_str()) ;
   for (int i = 0; i < vocabulary.num_rows(); ++i) {
      for (int j = 0; j < vocabulary.num_cols(); ++j)
         ofs << vocabulary.get<float>(i, j) << ' ' ;
      ofs << '\n' ;
   }
}

// Read the SIFT vocabulary from a plain text file into an Image<T>
Vocabulary load_vocabulary(const std::string& file_name)
{
   const int M = count_lines(file_name) ;
   const int N = GistEstimatorBeyondBoF::SiftDescriptor::SIZE ;
   Vocabulary V(N, M, ZEROS) ;

   float f ;
   std::ifstream ifs(file_name.c_str()) ;
   for (int j = 0; j < M; ++j)
      for (int i = 0; i < N; ++i) {
         if (! ifs)
            throw std::runtime_error(file_name + ": out of data?!?") ;
         ifs >> f ;
         V.setVal(i, j, f) ;
      }

   return V ;
}

} // end of local namespace encapsulating SIFT vocabulary computation section

//------------------- TRAINING HISTOGRAM PROCESSING ---------------------

// Training is a two step process: first, we use K-means to cluster the
// training set's SIFT descriptors to create the vocabulary of
// "prototypical" SIFT descriptors. Then, we collect the histograms
// counting these "vis-terms" in the training images. The vocabulary of
// prototypical SIFT descriptors (or vis-terms) and the training set's
// histogram "database" are both used for image classification.
namespace {

// Some useful types for dealing with vis-term histograms
typedef Image<double> Histogram ;

// Forward declarations
void save_histogram(const Histogram& histogram, const std::string& file_name,
                    const std::string& image_name, int frame_number,
                    const std::string& segment_number) ;

// This method implements the "hist" action of this program. Like the
// accumulate action, it implements a "main loop" for the simulation,
// evolving different components with each iteration. But rather than
// dipping into the GistEstimatorBeyondBoF's processing pipeline, it
// loads the SIFT vocabulary and then uses GistEstimatorBeyondBoF to
// obtain the flattened out multi-level histogram for each of the
// training images. These histograms are saved to the training
// histograms database specified by the --histograms-file option.
void BBoFSimulation::compute_training_histograms()
{
   ModelManagerStarter M(model_manager) ;

   nub::soft_ref<GistEstimatorBeyondBoF> ge =
      dynCastWeak<GistEstimatorBeyondBoF>(
         model_manager.subComponent("GistEstimatorBeyondBoF", MC_RECURSE)) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorBeyondBoF") ;

   Vocabulary V = load_vocabulary(sift_vocabulary_file()) ;
   ge->setVocabulary(V) ;
   LINFO("MVN: loaded SIFT vocabulary of %d vis-terms from %s",
         V.getHeight(), sift_vocabulary_file().c_str()) ;

   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         SeC<SimEventGistOutput> gist_out =
            event_queue->check<SimEventGistOutput>(brain.get(),
                                                   SEQ_UNMARKED | SEQ_MARK,
                                                   ge.get()) ;
         if (gist_out) // BBoF GE has a gist vector waiting to be picked up
            save_histogram(ge->getGist(), histograms_file(),
                           image_name(), input_frame_series->frame(),
                           segment_number()) ;
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         return ; // prevent LFATAL induced abortion
      }
   }
}

// This function appends a training image's histogram to the training
// histograms database file under the supplied "entry" name. As we did in
// the SIFT descriptors accumulation function, in order to minimize
// possible inconsistencies in this database, we choose to open and close
// the training histograms file with each invocation of this helper
// rather than keep a persistent ostream object around that obviates the
// need for repeated file open/close operations.
void save_histogram(const Histogram& histogram, const std::string& file_name,
                    const std::string& image_name, int frame_number,
                    const std::string& segment_number)
{
   std::ofstream ofs(file_name.c_str(), std::ios::out | std::ios::app) ;
   ofs << image_name << ':' << frame_number << ' '
       << segment_number << ' ' ;
   for (int y = 0; y < histogram.getHeight(); ++y) // should be just one row
      for (int x = 0; x < histogram.getWidth(); ++x) // should be 4200 columns
         ofs << histogram.getVal(x, y) << ' ' ;
   ofs << '\n' ;
}

} // end of local namespace encapsulating training histograms section

//--------------------- SVM CLASSIFIER GENERATION -----------------------

namespace {

// Forward declarations
void create_torch_dataset(const std::string&, const std::string&,
                          const std::string&) ;
Torch::SVMClassification* create_torch_classifier(const std::string&) ;
std::string temp_file_name() ;

// The following method implements this program's "svm" action.
void BBoFSimulation::generate_svm_classifier()
{
   GistEstimatorBeyondBoF::num_channels(vocabulary_size()) ;

   create_torch_dataset(histograms_file(), segment_number(), svm_temp_file()) ;
   Torch::SVMClassification* svm = create_torch_classifier(svm_temp_file()) ;
   svm->save(svm_classifier_file().c_str()) ;

   delete svm ;
   unlink(svm_temp_file().c_str()) ;
}

// Quick helper for reading and writing gist vectors from/to a file
struct GistVector {
   std::vector<double> values ;
   GistVector() ;
} ;

GistVector::GistVector()
   : values(GistEstimatorBeyondBoF::gist_vector_size())
{}

std::istream& operator>>(std::istream& is, GistVector& g)
{
   for (int i = 0; i < GistEstimatorBeyondBoF::gist_vector_size(); ++i)
      if (is)
         is >> g.values[i] ;
      else
         throw std::runtime_error("missing gist vector data") ;
   return is ;
}

std::ostream& operator<<(std::ostream& os, const GistVector& g)
{
   for (int i = 0; i < GistEstimatorBeyondBoF::gist_vector_size(); ++i)
      os << g.values[i] << ' ' ;
   return os ;
}

// The torch library needs its datasets in a particular format.
// Unfortunately, this program works with some other format. The
// following function reads the histograms file saved by an earlier run
// of this program and creates a corresponding torch dataset for
// subsequent training of an SVM classifier for the specified target
// segment.
void create_torch_dataset(const std::string& hist_file,
                          const std::string& target,
                          const std::string& torch_dataset)
{
   const int n = count_lines(hist_file) ;

   std::ifstream in(hist_file.c_str()) ;
   std::ofstream out(torch_dataset.c_str()) ;

   std::string dummy, segment, str ;
   GistVector gist_vector ;
   out << n << ' ' << (GistEstimatorBeyondBoF::gist_vector_size() + 1) << '\n';
   for (int i = 0; i < n; ++i)
   {
      std::getline(in, str) ;
      if (! in || str.empty()) {
        if (i == n - 1) // okay; all training histograms read successfully
          break ;
        else {
          out.close() ;
          unlink(torch_dataset.c_str()) ;
          throw std::runtime_error(hist_file +
                                   ": missing data or other read error") ;
        }
      }
      std::istringstream line(str) ;
      line >> dummy >> segment >> gist_vector ;
      out << gist_vector << ' ' << ((segment == target) ? +1 : -1) << '\n' ;
   }
}

// The histogram intersection kernel for matching gist vectors of
// different images.
class HistIntKernel : public Torch::Kernel {
  real eval(Torch::Sequence*, Torch::Sequence*) ;
} ;

real HistIntKernel::eval(Torch::Sequence* a, Torch::Sequence* b)
{
   real sum = 0 ;
   for (int i = 0; i < a->frame_size; ++i)
      sum += min(a->frames[0][i], b->frames[0][i]) ;
   return sum ;
}

// The following function creates an SVM classifier using the histogram
// intersection kernel defined above.
Torch::SVMClassification* create_torch_classifier(const std::string& dataset)
{
   HistIntKernel kernel ;
   Torch::SVMClassification* svm = new Torch::SVMClassification(& kernel) ;
   Torch::QCTrainer trainer(svm) ;
   Torch::MatDataSet data(dataset.c_str(),
                          GistEstimatorBeyondBoF::gist_vector_size(), 1) ;
   trainer.train(& data, 0) ;
   return svm ;
}

} // end of local namespace encapsulating SVM classifier generation section

//----------------------- IMAGE CLASSIFICATION --------------------------

namespace {

// Useful typedefs
typedef std::vector<Torch::SVMClassification*> Classifiers ;

// Forward declarations
Classifiers load_classifiers(std::string, HistIntKernel*) ;
Histogram read_gist_vector(std::istream&) ;
void classify_image(const Histogram&, const Classifiers&,
                    const std::string&, int, const std::string&,
                    const std::string&) ;
void nuke_classifiers(Classifiers&) ;

// The following method implements this program's "classify" action. It
// reads the SIFT descriptors vocabulary and computes gist vectors for
// input images using the BBoF gist estimator. Then, it uses the SVM
// classifiers generated by the "svm" action to decide which category the
// input image belongs to.
void BBoFSimulation::classify_input_images()
{
   ModelManagerStarter M(model_manager) ;

   nub::soft_ref<GistEstimatorBeyondBoF> ge =
      dynCastWeak<GistEstimatorBeyondBoF>(
         model_manager.subComponent("GistEstimatorBeyondBoF", MC_RECURSE)) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorBeyondBoF") ;

   Vocabulary V = load_vocabulary(sift_vocabulary_file()) ;
   ge->setVocabulary(V) ;
   LINFO("MVN: loaded SIFT vocabulary of %d vis-terms from %s",
         V.getHeight(), sift_vocabulary_file().c_str()) ;

   HistIntKernel kernel ;
   Classifiers svm_classifiers =
      load_classifiers(svm_classifier_file(), & kernel) ;

   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         SeC<SimEventGistOutput> gist_out =
            event_queue->check<SimEventGistOutput>(brain.get(),
                                                   SEQ_UNMARKED | SEQ_MARK,
                                                   ge.get()) ;
         if (gist_out) // BBoF GE has a gist vector waiting to be picked up
            classify_image(ge->getGist(), svm_classifiers,
                           image_name(), input_frame_series->frame(),
                           segment_number(), results_file()) ;
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         nuke_classifiers(svm_classifiers) ;
         return ; // prevent LFATAL induced abortion
      }
   }

   nuke_classifiers(svm_classifiers) ;
}

// The following method implements this program's "classify_gv" action.
// It reads the SIFT descriptors vocabulary and loads the SVM
// classifiers. Then, it uses the classifiers and the precomputed gist
// vectors to decide to which category the input images belong.
void BBoFSimulation::classify_using_gist_vectors()
{
   GistEstimatorBeyondBoF::num_channels(vocabulary_size()) ;

   HistIntKernel kernel ;
   Classifiers svm_classifiers =
      load_classifiers(svm_classifier_file(), & kernel) ;

   int line_number = 1 ;
   std::ifstream ifs(gist_vectors_file().c_str()) ;
   while (ifs)
      try
      {
         classify_image(read_gist_vector(ifs), svm_classifiers,
                        image_name(), line_number++,
                        segment_number(), results_file()) ;
      }
      catch (std::exception&) // ifs ran out of gist vector data
      {
      }

   nuke_classifiers(svm_classifiers) ;
}

// Given an input image's gist vector and the SVM classifiers for all the
// categories, this function checks which categories the input image
// belongs to and writes the results to the classification results file.
void classify_image(const Histogram&   gist_vector,
                    const Classifiers& classifiers,
                    const std::string& image_name, int frame_number,
                    const std::string& ground_truth,
                    const std::string& results_file)
{
   std::ofstream ofs(results_file.c_str(), std::ios::out | std::ios::app) ;
   ofs << image_name << ':' << frame_number << ' ' << ground_truth << ' ' ;

   Torch::Sequence gv(1, GistEstimatorBeyondBoF::gist_vector_size()) ;
   std::copy(gist_vector.begin(), gist_vector.end(), gv.frames[0]) ;

   int n = 0 ; // num categories into which input image can be classified
   const int N = classifiers.size() ;
   for (int i = 0; i < N; ++i) {
      classifiers[i]->forward(& gv) ;
      if (classifiers[i]->outputs->frames[0][0] > 0) {
         ofs << (i+1) << ' ' ;
         ++n ;
      }
   }

   if (! n) // input image could not be classified into any category
      ofs << '0' ;
   ofs << '\n' ;
}

// This function loads all the SVM classifiers beginning with the
// specified "root" name. Here's how this is supposed to work:
//
// Let's say we 9 categories. Earlier runs of this program ought to have
// created 9 SVM classifiers. Usually, these would be named
// "XXX_svm_classifier.1", "XXX_svm_classifier.2", "XXX_svm_classifier.3"
// and so on. This function will read each of these files back into
// memory using the torch library (which is what created those files in
// the first place).
//
// The kernel for each of these SVM classifiers is the histogram
// intersection kernel as described in the Lazebnik paper.
Classifiers
load_classifiers(std::string classifiers_root_name, HistIntKernel* kernel)
{
   classifiers_root_name += ".*" ;
   glob_t buf ;
   if (glob(classifiers_root_name.c_str(), 0, 0, & buf) != 0)
      throw std::runtime_error("couldn't find/load the SVM classifiers") ;

   const int N = buf.gl_pathc ;
   Classifiers classifiers(N) ;
   for (int i = 0; i < N; ++i) {
      classifiers[i] = new Torch::SVMClassification(kernel) ;
      classifiers[i]->load(buf.gl_pathv[i]) ;
   }

   globfree(& buf) ;
   return classifiers ;
}

// Delete all SVM classifier objects created in previous function
void nuke_classifiers(Classifiers& C)
{
   const int N = C.size() ;
   for (int i = 0; i < N; ++i)
      delete C[i] ;
}

// This function reads a gist vector from the specified input stream
Histogram read_gist_vector(std::istream& is)
{
   GistVector G ;
   is >> G ;

   Histogram H(GistEstimatorBeyondBoF::gist_vector_size(), 1, NO_INIT) ;
   std::copy(G.values.begin(), G.values.end(), H.beginw()) ;
   return H ;
}

} // end of local namespace encapsulating image classification section

//-------------------------- UTILITY ROUTINES ---------------------------

namespace {

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

//-----------------------------------------------------------------------

#endif // #if !defined(HAVE_OPENCV) || !defined(INVT_HAVE_TORCH)

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
