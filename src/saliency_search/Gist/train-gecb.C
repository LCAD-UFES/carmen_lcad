/*!
   \file Gist/train-gecb.C
   \brief Train and classify images using the context-based gist
   estimator.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/train-gecb.C $
// $Id: train-gecb.C 14818 2011-06-13 04:19:24Z igpu2 $
//

//------------------------------ HEADERS --------------------------------

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

// Gist specific headers
#include "Neuro/GistEstimatorContextBased.H"

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

#include "Image/MathOps.H"
#include "Image/MatrixOps.H"
#include "Image/Point2D.H"

#include "nub/ref.h"

#ifndef HAVE_OPENCV // fake OpenCV API so as to not break builds

namespace {

struct CvMat {int rows, cols, type ;} ;

inline CvMat* cvCreateMat(int, int, int) {return 0 ;}
inline void   cvReleaseMat(CvMat**) {}
inline double cvmGet(CvMat*, int, int) {return 0 ;}
inline void   cvmSet(CvMat*, int, int, double) {}
inline void   cvTranspose(const CvMat*, CvMat*) {}

#define CV_32FC1 0
inline int CV_MAT_TYPE(int) {return 0 ;}
#define CV_MAT_ELEM(matrix, type, row, col) (type(0))

#define CV_PCA_DATA_AS_COL 0
inline void cvCalcPCA(const CvMat*, CvMat*, CvMat*, CvMat*, int) {}

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

//------------------------------ DEFINES --------------------------------

// "Raw" gist vectors are collections of 384 numbers. To reduce the
// dimensionality of these vectors to make them faster to compare, etc.
// we use PCA and extract the following number of principal components.
#ifndef GECB_NUM_PRINCIPAL_COMPONENTS
   #define GECB_NUM_PRINCIPAL_COMPONENTS 80
#endif

//----------------------- FORWARD DECLARATIONS --------------------------

namespace {

// Some useful types for dealing with gist training vectors
typedef Image<double> GistVector ;
typedef std::map<int, GistVector> TrainingDB ;
typedef TrainingDB::value_type TrainingDBEntry ;

// PCA
class OpenCVMatrix ;
CvMat* load_training_vectors(const std::string& file_name, int M, int N) ;
CvMat* pca(const OpenCVMatrix& data, int num_principal_components) ;

// Image classification
typedef std::pair<std::string, GistVector> InputImageData ;
void classify_image(const InputImageData&, const TrainingDB&,
                    const std::string& results_file) ;

// I/O
void save(const OpenCVMatrix&, const std::string& file_name) ;

void append(const Image<double>&,
            const std::string& file_name,
            const std::string& image_name = std::string()) ;
Image<double> load_image(const std::string& file_name,
                         int width, int height) ;
std::ostream& operator<<(std::ostream&, const Image<double>&) ;
std::istream& operator>>(std::istream&, Image<double>&) ;

TrainingDB load_training_database(const std::string& file_name) ;

std::string getline(std::istream&) ;

// Utilities
int count_lines(const std::string& file_name) ;
template<typename T> std::string to_string(const T&) ;

}

//-------------------------- OPENCV MATRICES ----------------------------

// Crude encapsulation of OpenCV matrices
namespace {

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

} // end of local namespace encapsulating above helper

//----------------------- COMMAND LINE OPTIONS --------------------------

// This program has four distinct phases/modes of operation, each one
// specified via a suitable non-option command line argument.
// Additionally, it supports several command line options to allow users
// to tweak various parameters such as the name of the PCA matrix file,
// training database, and so on.
namespace {

const ModelOptionCateg MOC_GECB = {
   MOC_SORTPRI_3,
   "Options specific to the context-based gist program",
} ;

// In the training vectors accumulation phase, we collect all the gist
// vectors that will be used as the input to the PCA into a plain text
// file.
#ifndef GECB_DEFAULT_TRAINING_VECTORS_FILE
   #define GECB_DEFAULT_TRAINING_VECTORS_FILE "gecb_training_vectors.txt"
#endif

const ModelOptionDef OPT_TrainingVectors = {
   MODOPT_ARG_STRING, "TrainingVectors", & MOC_GECB, OPTEXP_CORE,
   "This option specifies the name of the file where the training vectors\n"
   "should be accumulated or read from. This is a plain text file containing\n"
   "the training vectors matrix that will be fed into the PCA procedure.\n"
   "Each line of this file will contain a row of \"raw\" (i.e., 384-\n"
   "dimensional) gist vectors. For PCA, these rows will be read in as the\n"
   "columns of the data matrix.\n",
   "training-vectors", '\0', "training-vectors-file",
   GECB_DEFAULT_TRAINING_VECTORS_FILE,
} ;

// For PCA, we use the accumulated training vectors, i.e., the
// 384-dimensional "raw" gist vectors, and produce the transformation
// matrix that will convert these raw gist vectors into 80-dimensional
// vectors. This transformation matrix is stored in the following file.
#ifndef GECB_DEFAULT_PCA_MATRIX_FILE
   #define GECB_DEFAULT_PCA_MATRIX_FILE "gecb_pca_matrix.txt"
#endif

const ModelOptionDef OPT_PCAMatrix = {
   MODOPT_ARG_STRING, "PCAMatrix", & MOC_GECB, OPTEXP_CORE,
   "This option specifies the name of the file in which the 384x80 PCA\n"
   "transformation matrix is (or will be) stored. \"Raw\" 384-dimensional\n"
   " gist vectors can be reduced to 80 dimensions by muliplying with this\n"
   "matrix. The PCA transformation matrix is stored in a plain text file\n"
   "with each line containing one row of the matrix.\n",
   "pca-matrix", '\0', "pca-matrix-file",
   GECB_DEFAULT_PCA_MATRIX_FILE,
} ;

// In the second phase of training, we use the PCA transformation matrix
// to output the 80-dimensional gist vectors of the training images and
// store them in a training set under the specified "entry name" and
// segment number.
//
// DEVNOTE: The default values for these options are not very useful.
// They really ought to be specified on the command line.
#ifndef GECB_DEFAULT_IMAGE_NAME
   #define GECB_DEFAULT_IMAGE_NAME "some_image"
#endif
#ifndef GECB_DEFAULT_SEGMENT_NUMBER
   #define GECB_DEFAULT_SEGMENT_NUMBER "0"
#endif

const ModelOptionDef OPT_ImageName = {
   MODOPT_ARG_STRING, "ImageName", & MOC_GECB, OPTEXP_CORE,
   "This option specifies the \"root\" name of an entry in the training\n"
   "set or the results file. The image number will be automatically\n"
   "appended to this \"root\" name. The training database is a plain text\n"
   "file containing one entry per line. The first field specifies the name\n"
   "plus number of the entry (e.g., foo.1, foo.2, bar.1, and so on). The\n"
   "second field is the segment number for this image. The remaining fields\n"
   "are the 80 numbers making up the image's gist vector.\n\n"
   "In classification mode, this option specifies the name of the input\n"
   "image's gist vector that is written to the results file.\n",
   "image-name", '\0', "image-name-root",
   GECB_DEFAULT_IMAGE_NAME,
} ;

const ModelOptionDef OPT_SegmentNumber = {
   MODOPT_ARG_STRING, "SegmentNumber", & MOC_GECB, OPTEXP_CORE,
   "This option specifies the segment number for an image in the training\n"
   "set. The segment number is used in the third phase of training to\n"
   "compute the mean 80-D gist vectors for each segment and during\n"
   "classification to ascertain the segment number for each input image.\n",
   "segment-number", '\0', "image-segment-number",
   GECB_DEFAULT_SEGMENT_NUMBER,
} ;

// The output of the second phase of training, i.e., the 80-D gist
// vectors for each of the training images, is sent to the following
// file.
#ifndef GECB_DEFAULT_TRAINING_SET
   #define GECB_DEFAULT_TRAINING_SET "gecb_training_set.txt"
#endif

const ModelOptionDef OPT_TrainingSet = {
   MODOPT_ARG_STRING, "TrainingSet", & MOC_GECB, OPTEXP_CORE,
   "This option specifies the name of the training set, a plain text\n"
   "file containing one entry per line. The first field specifies the name\n"
   "plus number of the entry (e.g., foo.1, foo.2, bar.1, and so on). The\n"
   "second field is the segment number for this image. And the remaining\n"
   "fields are the 80 numbers that make up the image's gist vector.\n",
   "training-set", '\0', "training-set-file",
   GECB_DEFAULT_TRAINING_SET,
} ;

// The training database is a plain text file that specifies the mean
// gist vector for each segment.
#ifndef GECB_DEFAULT_TRAINING_DATABASE
   #define GECB_DEFAULT_TRAINING_DATABASE "gecb_training_db.txt"
#endif

const ModelOptionDef OPT_TrainingDB = {
   MODOPT_ARG_STRING, "TrainingDB", & MOC_GECB, OPTEXP_CORE,
   "This option specifies the name of the training database, a plain text\n"
   "file containing one entry per line. The first field specifies the\n"
   "segment number. And the remaining fields are the 80 numbers that make\n"
   "up the segment's mean gist vector.\n",
   "training-db", '\0', "training-db-file",
   GECB_DEFAULT_TRAINING_DATABASE,
} ;

// In image classification mode, we write the results to the following
// file.
#ifndef GECB_DEFAULT_CLASSIFICATION_RESULTS_FILE
   #define GECB_DEFAULT_CLASSIFICATION_RESULTS_FILE "gecb_classifications.txt"
#endif

const ModelOptionDef OPT_ResultsFile = {
   MODOPT_ARG_STRING, "ResultsFile", & MOC_GECB, OPTEXP_CORE,
   "This option specifies the name of the classification results file,\n"
   "a plain text file containing one result entry per line. The first\n"
   "field specifies the name plus number of the input image, (e.g., foo.1,\n"
   "foo.2, bar.1, and so on). Then come the numbers of the top five matching\n"
   "segments from the training database.\n",
   "results-file", '\0', "classification-results-file",
   GECB_DEFAULT_CLASSIFICATION_RESULTS_FILE,
} ;

// The different operational modes of this program must be specified as
// the one and only non-option command line argument. This "action"
// command must be one of the following strings (case-sensitive!):
//
// 1. accumulate -- accumulate the training vectors in the plain text
//    file specified by the --training-vectors option (default is to
//    accumulate the training vectors in ./gecb_training_vectors.txt.
//
// 2. pca -- compute the transformation matrix that will allow us to
//    reduce the 384-dimensional "raw" gist vectors to their 80 principal
//    components using the PCA support in OpenCV.
//
//    The --training-vectors option can be used to specify the input file
//    for the PCA and --pca-matrix option can be used to specify the file
//    in which the transformation matrix should be saved. The defaults
//    are to read from ./gecb_training_vectors.txt and write to
//    ./gecb_pca_matrix.txt.
//
// 3. train -- compute the gist vectors for the training set. The output
//    is sent to the text file specified by the --training-set option.
//    It is a good idea to also supply the --image-name and
//    --segment-number options when saving training gist vectors from an
//    MPEG. A good choice of the entry's name would be the basename of
//    the MPEG file sans extension.
//
// 4. means -- given the 80-dimensional gist vectors for the training
//    images, this action computes the mean gist vectors for each
//    segment. The output is sent to the text file specified by the
//    --training-db option.
//
// 5. classify -- uses the PCA transformation matrix and training
//    database produced by the pca and means commands to classify the
//    input images streaming in.
#ifndef GECB_ACCUMULATE_CMD
   #define GECB_ACCUMULATE_CMD "accumulate"
#endif
#ifndef GECB_PCA_CMD
   #define GECB_PCA_CMD "pca"
#endif
#ifndef GECB_TRAIN_CMD
   #define GECB_TRAIN_CMD "train"
#endif
#ifndef GECB_MEANS_CMD
   #define GECB_MEANS_CMD "means"
#endif
#ifndef GECB_CLASSIFY_CMD
   #define GECB_CLASSIFY_CMD "classify"
#endif

// For printing usage info
#ifndef GECB_ACTIONS
   #define GECB_ACTIONS ("{"GECB_ACCUMULATE_CMD"|"GECB_PCA_CMD"|"\
                          GECB_TRAIN_CMD"|"GECB_MEANS_CMD"|"\
                          GECB_CLASSIFY_CMD"}")
#endif

} // end of local namespace encapsulating command line options section

//--------------------- SIMULATION ENCAPSULATION ------------------------

namespace {

// The following helper class wraps around the ModelManager and
// associated objects, providing a neatly encapsulated API for the main
// program.
class ContextBasedSimulation {
   ModelManager model_manager ;
   nub::soft_ref<SimEventQueueConfigurator> configurator ;
   nub::soft_ref<StdBrain> brain ;
   nub::ref<SimInputFrameSeries> input_frame_series ;

   // Various command line options specific to this program
   OModelParam<std::string> tv_option ; // --training-vectors
   OModelParam<std::string> pm_option ; // --pca-matrix
   OModelParam<std::string> ts_option ; // --training-set
   OModelParam<std::string> td_option ; // --training-db
   OModelParam<std::string> rf_option ; // --results-file
   OModelParam<std::string> in_option ; // --image-name (not --in!)
   OModelParam<std::string> sn_option ; // --segment-number

public :
   ContextBasedSimulation(const std::string& model_name) ;
   void parse_command_line(int argc, const char* argv[]) ;
   void run() ;
   ~ContextBasedSimulation() ;

private :
   // The different actions performed by this program
   typedef void (ContextBasedSimulation::*Action)() ;
   typedef std::map<std::string, Action> ActionMap ;
   ActionMap action_map ;

   void accumulate_training_vectors() ; // for input to PCA
   void compute_pca_matrix() ;          // using training vectors
   void compute_training_vectors() ;    // using PCA matrix
   void compute_segment_means() ;       // using training vectors
   void classify_input_images() ;       // using PCA matrix & segment means

   // Accessors for retrieving some of the command line arguments
   std::string training_vectors_file() {return tv_option.getVal() ;}
   std::string pca_matrix_file()   {return pm_option.getVal() ;}
   std::string training_set()      {return ts_option.getVal() ;}
   std::string training_database() {return td_option.getVal() ;}
   std::string results_file()      {return rf_option.getVal() ;}
   std::string image_name()        {return in_option.getVal() ;}
   std::string segment_number()    {return sn_option.getVal() ;}
} ;

// On instantiation, create the model manager and the simulation's
// various components.
ContextBasedSimulation::ContextBasedSimulation(const std::string& model_name)
   : model_manager(model_name),
     configurator(new SimEventQueueConfigurator(model_manager)),
     brain(new StdBrain(model_manager)),
     input_frame_series(new SimInputFrameSeries(model_manager)),
     tv_option(& OPT_TrainingVectors, & model_manager),
     pm_option(& OPT_PCAMatrix, & model_manager),
     ts_option(& OPT_TrainingSet, & model_manager),
     td_option(& OPT_TrainingDB, & model_manager),
     rf_option(& OPT_ResultsFile, & model_manager),
     in_option(& OPT_ImageName, & model_manager),
     sn_option(& OPT_SegmentNumber, & model_manager)
{
   model_manager.addSubComponent(configurator) ;
   model_manager.addSubComponent(brain) ;
   model_manager.addSubComponent(input_frame_series) ;

   typedef ContextBasedSimulation me ; // typing shortcut
   action_map[GECB_ACCUMULATE_CMD] = & me::accumulate_training_vectors ;
   action_map[GECB_PCA_CMD]        = & me::compute_pca_matrix ;
   action_map[GECB_TRAIN_CMD]      = & me::compute_training_vectors ;
   action_map[GECB_MEANS_CMD]      = & me::compute_segment_means ;
   action_map[GECB_CLASSIFY_CMD]   = & me::classify_input_images ;
}

// TODO: Do we really need the single channel save raw maps option for
// this texton training program? And how can we force the gist estimator
// type to be always GistEstimatorTexton? This program doesn't make sense
// for any other gist estimator.
void ContextBasedSimulation::parse_command_line(int argc, const char* argv[])
{
   model_manager.setOptionValString(& OPT_SingleChannelSaveRawMaps, "true") ;
   model_manager.setOptionValString(& OPT_GistEstimatorType, "ContextBased") ;
   model_manager.setOptionValString(& OPT_NumOrientations, "6") ;

   model_manager.setOptionValString(& OPT_TrainingVectors,
                                    GECB_DEFAULT_TRAINING_VECTORS_FILE) ;
   model_manager.setOptionValString(& OPT_PCAMatrix,
                                    GECB_DEFAULT_PCA_MATRIX_FILE) ;
   model_manager.setOptionValString(& OPT_TrainingSet,
                                    GECB_DEFAULT_TRAINING_SET) ;
   model_manager.setOptionValString(& OPT_TrainingDB,
                                    GECB_DEFAULT_TRAINING_DATABASE) ;
   model_manager.setOptionValString(& OPT_ResultsFile,
                                    GECB_DEFAULT_CLASSIFICATION_RESULTS_FILE) ;
   model_manager.setOptionValString(& OPT_ImageName,
                                    GECB_DEFAULT_IMAGE_NAME) ;
   model_manager.setOptionValString(& OPT_SegmentNumber,
                                    GECB_DEFAULT_SEGMENT_NUMBER) ;

   if (! model_manager.parseCommandLine(argc, argv, GECB_ACTIONS, 1, 1))
      throw std::runtime_error("command line parse error") ;
}

// To run the simulation, we simply dispatch to the function
// corresponding to the action (non-option) command line argument.
void ContextBasedSimulation::run()
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
// action, which evolves the different components of the simulation and
// saves the raw (384-dimensional) gist vectors to the training vectors
// file.
void ContextBasedSimulation::accumulate_training_vectors()
{
   ModelManagerStarter M(model_manager) ;

   LFATAL("fixme");
   nub::soft_ref<GistEstimatorContextBased> ge;///// =
   ///////      dynCastWeak<GistEstimatorContextBased>(brain->getGE()) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorContextBased") ;

   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         SeC<SimEventGistOutput> gist_out =
            event_queue->check<SimEventGistOutput>(brain.get(),
                                                   SEQ_UNMARKED | SEQ_MARK,
                                                   ge.get()) ;
         if (gist_out) // gist vector waiting to be picked up
            append(ge->getGist(), training_vectors_file()) ;
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         return ; // prevent LFATAL induced abortion
      }
   }
}

// The following method implements the "pca" action of this program,
// which produces the PCA transformation matrix that allows us to reduce
// the dimensionality of gist vectors from 384 to 80.
void ContextBasedSimulation::compute_pca_matrix()
{
   LINFO("MVN: counting lines in %s", training_vectors_file().c_str()) ;
   const int cols = count_lines(training_vectors_file()) ;
   const int rows = static_cast<int>(GistEstimatorContextBased::NUM_FEATURES) ;

   LINFO("MVN: reading %d training vectors from %s",
         cols, training_vectors_file().c_str()) ;
   OpenCVMatrix training_vectors =
      load_training_vectors(training_vectors_file(), rows, cols) ;

   LINFO("MVN: doing PCA on training vectors to get %d principal components",
         GECB_NUM_PRINCIPAL_COMPONENTS) ;
   OpenCVMatrix pca_matrix = pca(training_vectors,
                                 GECB_NUM_PRINCIPAL_COMPONENTS) ;

   LINFO("MVN: PCA done; saving PCA transformation matrix to %s",
         pca_matrix_file().c_str()) ;
   save(pca_matrix, pca_matrix_file()) ;
}

// This method implements the "train" action of this program. Like the
// accumulate action, it implements a "main loop" for the simulation,
// evolving different components with each iteration. But rather than
// saving raw gist vectors to a file, it uses the transformation matrix
// computed by the "pca" action to reduce the dimensionality of the raw
// gist vectors from 384 to 80 and then saves these 80-dimensional
// vectors to the training database.
void ContextBasedSimulation::compute_training_vectors()
{
   ModelManagerStarter M(model_manager) ;

   LFATAL("fixme");
   nub::soft_ref<GistEstimatorContextBased> ge;///// =
   ////////      dynCastWeak<GistEstimatorContextBased>(brain->getGE()) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorContextBased") ;

   Image<double> W = load_image(pca_matrix_file(),
                                GECB_NUM_PRINCIPAL_COMPONENTS,
                                count_lines(pca_matrix_file())) ;
   LINFO("MVN: loaded %dx%d PCA transformation matrix from %s",
         W.getHeight(), W.getWidth(), pca_matrix_file().c_str()) ;

   int i = 1 ;
   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         SeC<SimEventGistOutput> gist_out =
            event_queue->check<SimEventGistOutput>(brain.get(),
                                                   SEQ_UNMARKED | SEQ_MARK,
                                                   ge.get()) ;
         if (gist_out) // gist vector waiting to be picked up
            append(vmMult(ge->getGist(), W),
                   training_set(),
                   image_name() + to_string(i++) + " " + segment_number()) ;
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         return ; // prevent LFATAL induced abortion
      }
   }
}

// Once we have the training set, we compute the segment means by
// looking at the segment number for each image in the training set and
// summing its gist vector to the running total for that segment. At the
// end of this process, we simply divide the totals for each segment by
// the count of gist vectors encountered for that segment.
//
// This averaging operation is facilitated by a map that associates a
// segment number with a gist vector sum and a corresponding count.
void ContextBasedSimulation::compute_segment_means()
{
   typedef std::pair<int, GistVector> SegmentInfo ; // count and sum
   typedef std::map<int, SegmentInfo> MeansMap ;
   MeansMap means ;

   LINFO("reading training set data from %s", training_set().c_str()) ;
   std::ifstream ifs(training_set().c_str()) ;
   for(;;)
   {
      try
      {
         std::istringstream line(getline(ifs)) ;

         std::string name ; int segment ;
         GistVector G(GECB_NUM_PRINCIPAL_COMPONENTS, 1, ZEROS) ;
         line >> name >> segment >> G ;

         MeansMap::iterator it = means.find(segment) ;
         if (it == means.end())
         {
            means.insert(std::make_pair(segment, SegmentInfo(1, G))) ;
         }
         else
         {
            SegmentInfo& I = it->second ;
            ++I.first ; // count
            I.second += G ; // gist vector sum
         }
      }
      catch (std::exception&) // ought to be just EOF for ifs
      {
         break ;
      }
   }

   LINFO("computing gist vector averages for each segment and saving to %s",
         training_database().c_str()) ;
   std::ofstream ofs(training_database().c_str()) ;
   for (MeansMap::iterator it = means.begin(); it != means.end(); ++it)
   {
      SegmentInfo& I = it->second ;
      I.second /= I.first ; // gist vector totals by num vectors for segment
      ofs << it->first << ' ' << I.second ;
   }
}

// The following method implements this program's "classify" action. It
// loads the PCA transformation matrix and the training vectors database
// and then uses a simple Euclidean distance measure to compute the
// closest match for the input image.
void ContextBasedSimulation::classify_input_images()
{
   ModelManagerStarter M(model_manager) ;

   LFATAL("fixme");
   nub::soft_ref<GistEstimatorContextBased> ge;///////// =
   //////      dynCastWeak<GistEstimatorContextBased>(brain->getGE()) ;
   if (ge.isInvalid())
      throw std::runtime_error("can only use GistEstimatorContextBased") ;

   Image<double> W = load_image(pca_matrix_file(),
                                GECB_NUM_PRINCIPAL_COMPONENTS,
                                count_lines(pca_matrix_file())) ;
   LINFO("MVN: loaded %dx%d PCA transformation matrix from %s",
         W.getHeight(), W.getWidth(), pca_matrix_file().c_str()) ;

   TrainingDB training_db = load_training_database(training_database()) ;
   LINFO("MVN: loaded %d training vectors from %s",
         int(training_db.size()), training_database().c_str()) ;

   int i = 1 ;
   nub::ref<SimEventQueue> event_queue = configurator->getQ() ;
   for(;;)
   {
      try
      {
         SeC<SimEventGistOutput> gist_out =
            event_queue->check<SimEventGistOutput>(brain.get(),
                                                   SEQ_UNMARKED | SEQ_MARK,
                                                   ge.get()) ;
         if (gist_out) // gist vector waiting to be picked up
            classify_image(std::make_pair(image_name() + to_string(i++),
                                          vmMult(ge->getGist(), W)),
                           training_db, results_file()) ;
         if (event_queue->evolve() != SIM_CONTINUE)
            break ;
      }
      catch (lfatal_exception&) // if we seek beyond end of frame series
      {
         return ; // prevent LFATAL induced abortion
      }
   }
}

// Do we really not have to delete the configurator, brain and input
// frame series? If it turns out we do, this empty destructor will have
// to be filled out with the necessary delete calls...
ContextBasedSimulation::~ContextBasedSimulation() {}

} // end of local namespace encapsulating simulation encapsulation section

//------------------------------- MAIN ----------------------------------

#ifdef HAVE_OPENCV

int main(int argc, const char* argv[])
{
   MYLOGVERB = LOG_INFO ; // suppress debug messages
   try
   {
      ContextBasedSimulation S("train-gecb Model") ;
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

//-------------------------------- PCA ----------------------------------

namespace {

// Create the MxN data matrix for PCA from the training vectors file.
// Each line of this file becomes a column of the data matrix.
CvMat* load_training_vectors(const std::string& file_name, int M, int N)
{
   CvMat* data = cvCreateMat(M, N, CV_32FC1) ;

   double d ;
   std::ifstream ifs(file_name.c_str()) ;
   for (int j = 0; j < N; ++j)
      for (int i = 0; i < M; ++i) { // column-major reading
         if (! ifs) {
            cvReleaseMat(& data) ;
            throw std::runtime_error(file_name + ": out of data?!?") ;
         }
         ifs >> d ;
         cvmSet(data, i, j, d) ;
      }

   return data ;
}

// Return the PCA transformation matrix that will allow extraction of the
// D principal components from input data.
CvMat* pca(const OpenCVMatrix& data, int D)
{
   OpenCVMatrix means = cvCreateMat(data.num_rows(), 1, CV_32FC1) ;
   OpenCVMatrix eigenvalues = cvCreateMat(D, 1, CV_32FC1) ;
   OpenCVMatrix eigenvectors = cvCreateMat(D, data.num_rows(), CV_32FC1) ;

   cvCalcPCA(data, means, eigenvalues, eigenvectors, CV_PCA_DATA_AS_COL) ;

   CvMat* pca_matrix = cvCreateMat(data.num_rows(), D, CV_32FC1) ;
   cvTranspose(eigenvectors, pca_matrix) ;
   return pca_matrix ;
}

} // end of local namespace encapsulating training vectors section

//----------------------- IMAGE CLASSIFICATION --------------------------

// Given the 80-D gist vectors for an input image and each of the
// training segments, we can tell which training segment the input image
// matches most closely by performing a Euclidean distance check between
// the input image's gist vector and the vectors of each of the training
// segments.
namespace {

// When computing the Euclidean distance between the input image's gist
// vector and that of each of the training segments, we want to be able
// to tell which training segment is the closest. For that, we use the
// following pair that "maps" a training segment number to its
// corresponding distance.
typedef std::pair<int, double> SegmentDistance ;

// To sort segment distances, we want to compare the Euclidean distances
// rather than their names.
bool distance_cmp(const SegmentDistance& L, const SegmentDistance& R)
{
   return L.second < R.second ;
}

// But when writing classification results, we're only interested in the
// matching training segment's number and not really in the Euclidean
// distance between its gist vector and that of the input image.
std::ostream& operator<<(std::ostream& os, const SegmentDistance& D)
{
   return os << D.first ;
}

// Given an entry from the training database, the following function
// object returns the Euclidean distance between the supplied input
// image's gist vector and the training segment's gist vector.
struct euclidean_distance
   : public std::binary_function<TrainingDBEntry, GistVector, SegmentDistance>
{
   SegmentDistance
   operator()(const TrainingDBEntry& E, const GistVector& input) const {
     LFATAL("FIXME!!");
     return std::make_pair(E.first, 0.0F);//////distance<float>(input, E.second));
   }
} ;

// This function computes the Euclidean distance between the input
// image's 80-D gist vector and each of the 80-D gist vectors for the
// training segments and then writes the top five matches to the
// specified results file.
//
// DEVNOTE: To output the top five matches to the results file, we ought
// to be able to use the std::copy algorithm in conjunction with
// std::ostream_iterator<SegmentDistance>. Unfortunately, ostream
// iterators cannot be used with user-defined types. This is entirely in
// keeping with C++'s philosophy of sucking ass most of the time but
// sucking ass big-time only every now and then.
void classify_image(const InputImageData& input, const TrainingDB& db,
                    const std::string& results_file)
{
   std::vector<SegmentDistance> distances ;
   std::transform(db.begin(), db.end(), std::back_inserter(distances),
                  std::bind2nd(euclidean_distance(), input.second)) ;
   std::sort(distances.begin(), distances.end(), distance_cmp) ;

   std::ofstream ofs(results_file.c_str(), std::ios::out | std::ios::app) ;
   ofs << input.first << ' ' ;
   //std::copy(distances.begin(), distances.begin() + 5,
             //std::ostream_iterator<SegmentDistance>(ofs, " ")) ; // ERROR!
   for (unsigned int i = 0; i < distances.size() && i < 5; ++i)
      ofs << distances[i] << ' ' ;
   ofs << '\n' ;
}

} // end of local namespace encapsulating image classification section

//-------------------------------- I/O ----------------------------------

namespace {

// Save an OpenCV matrix to the specified file
void save(const OpenCVMatrix& M, const std::string& file_name)
{
   std::ofstream ofs(file_name.c_str()) ;
   for (int i = 0; i < M.num_rows(); ++i) {
      for (int j = 0; j < M.num_cols(); ++j)
         ofs << M.get<float>(i, j) << ' ' ;
      ofs << '\n' ;
   }
}

// Append an Image to the specified file, optionally writing an image
// name to the first row.
void append(const Image<double>& I, const std::string& file_name,
            const std::string& image_name)
{
   if (! I.initialized())
      throw std::runtime_error("save empty image to " + file_name + "?!?") ;
   if (file_name.empty())
      throw std::runtime_error("must specify file name for saving Image") ;

   std::ofstream ofs(file_name.c_str(), std::ios::out | std::ios::app) ;
   if (! image_name.empty())
      ofs << image_name << ' ' ;
   ofs << I ;
}

// Load Image stored in a file
Image<double>
load_image(const std::string& file_name, int width, int height)
{
   try
   {
      Image<double> I(width, height, NO_INIT) ;
      std::ifstream ifs(file_name.c_str()) ;
      ifs >> I ;
      return I ;
   }
   catch (std::exception&)
   {
      throw std::runtime_error(file_name + ": out of data?!?") ;
   }
}

// Stream insertion operator for an Image<double>
std::ostream& operator<<(std::ostream& os, const Image<double>& I)
{
   for (int y = 0; y < I.getHeight(); ++y) {
      for (int x = 0; x < I.getWidth(); ++x)
         os << I.getVal(x, y) << ' ' ;
      os << '\n' ;
   }
   return os ;
}

// The following extraction operator reads in an Image<double> from the
// supplied input stream.
//
// WARNING: It *assumes* that the image has already allocated some memory
// for itself and that the input stream has those many elements.
std::istream& operator>>(std::istream& is, Image<double>& I)
{
   double d ;
   for (int y = 0; y < I.getHeight(); ++y)
      for (int x = 0; x < I.getWidth(); ++x)
         if (is >> d)
            I.setVal(x, y, d) ;
         else
            throw std::runtime_error("not enough data for Image<double>?!?") ;
   return is ;
}

// The following function reads the training "database," which is a plain
// text file containing one entry per line. Each line starts with the
// segment number and then come the eighty numbers making up that
// segment's mean gist vector.
TrainingDB load_training_database(const std::string& file_name)
{
   TrainingDB db ;

   std::ifstream ifs(file_name.c_str()) ;
   for(;;)
   {
      try
      {
         std::istringstream line(getline(ifs)) ;

         int segment ;
         GistVector G(GECB_NUM_PRINCIPAL_COMPONENTS, 1, ZEROS) ;
         line >> segment >> G ;

         db.insert(std::make_pair(segment, G)) ;
      }
      catch (std::exception&) // ought only to happen on ifs EOF
      {
         break ;
      }
   }

   return db ;
}

// Read a line from the input stream and return it as a string
std::string getline(std::istream& is)
{
   std::string line ;
   getline(is, line) ;
   if (! is || line.empty()) // EOF
      throw std::runtime_error("unable to read from input stream") ;
   return line ;
}

} // end of local namespace encapsulating texton accumulation section

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

} // end of local namespace encapsulating utility routines section

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
