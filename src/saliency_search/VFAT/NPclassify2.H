/*!@file VFAT/NPclassify2.H  Test the nonparametric classifier
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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/NPclassify2.H $
// $Id: NPclassify2.H 6182 2006-01-31 18:41:41Z rjpeters $
//

// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itt itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

#ifndef NPCLASSIFY2_H_DEFINED
#define NPCLASSIFY2_H_DEFINED

#include <vector>
#include "Util/readConfig.H"
#include "Image/All.H"
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "VFAT/covEstimate.H"

//! Non parametric classifier version 2.0
/*! When called this class will attempt to classify all data provided
    into an unknown number of classed. Thus, NPclassify is ment to
    work the same as other classifiers such as EM and Kmeans
    but without strong priors about class number or size.
    Soft parameters are used and are either provied at command line
    or in a configuration file
*/

template <class FLOAT> class NPclassify2
{
private:
  // Adjustable parameters from config file
  // see config file for details
  FLOAT NP_Con1, NP_Con2, NP_Con3;
  FLOAT NP_DWeight1, NP_DWeight2, NP_CWeight1, NP_CWeight2;
  FLOAT NP_IDWeight1, NP_IDWeight2, NP_ICWeight1, NP_ICWeight2;
  FLOAT NP_DenWeight1, NP_DenWeight2;
  FLOAT NP_preDenWeight1, NP_preDenWeight2;
  FLOAT NP_entDWeight1, NP_entDWeight2, NP_entCWeight1, NP_entCWeight2;
  FLOAT NP_entIDWeight1, NP_entIDWeight2, NP_entICWeight1, NP_entICWeight2;
  FLOAT NP_entDenWeight1, NP_entDenWeight2;
  FLOAT NP_entpreDenWeight1, NP_entpreDenWeight2;
  FLOAT NP_trainChildWeight;
  int    NP_defaultSize;
  int    NP_hardClassSize, NP_hardLinkSize;
  FLOAT NP_enthardClassSize, NP_enthardLinkSize;
  bool   NP_CLS;
  bool   NP_useIter;
  bool   NP_useBias;

  FLOAT NP_polyDensObjectCut1,NP_polyDensObjectCut2,NP_polyDensObjectCut3;
  FLOAT NP_polySpaceChildCut1,NP_polySpaceChildCut2,NP_polySpaceChildCut3;
  FLOAT NP_fZERO;
  int   NP_ZERO;

  // Internal Variables used by NPclassify
  //! Holds space of samples, is the input in essense
  typename std::vector<std::vector<FLOAT> > *NP_Space;
  //! The distance between any two nodes in space
  typename std::vector<std::vector<FLOAT> > NP_Dis;
  //! Used with k-means
  typename std::vector<std::vector<FLOAT> > NP_Kmean;
  //! Pointer reverse map to NP_Dis for use with iterators
  typename std::vector<std::vector<FLOAT*> > NP_reverseDisMap;

  //! holds the measure from metaclassify for a train
  std::vector<FLOAT> NP_trainMeasure;
  //! Holds the distance from a node to its parent node
  std::vector<FLOAT> NP_distance;
  //! the density of this node
  std::vector<FLOAT> NP_density;
  //! general measure of entropy for samples
  std::vector<FLOAT> NP_entropy;
  //! this is the mean distance within a class
  std::vector<FLOAT> NP_meanInterDistance;
  //! The standard deviation for meanInterDistance
  std::vector<FLOAT> NP_stdInterDistance;
  //! Mean number of decendants per node inter class
  std::vector<FLOAT> NP_meanInterChild;
  //! the standard deviation for meanInterClass
  std::vector<FLOAT> NP_stdInterChild;
  //! Mean density for nodes inter class
  std::vector<FLOAT> NP_meanInterDensity;
  //! Sum density for nodes inter class (used in entropy computation)
  std::vector<FLOAT> NP_sumInterDensity;
  //! Standard deviation for meanInterDensity
  std::vector<FLOAT> NP_stdInterDensity;
  //! Entropy measure for intercalss
  std::vector<FLOAT> NP_interEntropy;
  //! Final mean density per class
  std::vector<FLOAT> NP_meanClassDensity;
  //! Final Standard deviation per class
  std::vector<FLOAT> NP_stdClassDensity;



  // POINTER MAPS to KEY NODE reference

  //! Holds class membership for nodes
  std::vector<std::vector<int*> > NP_Class;
  //! Holds a list of all children for a node
  std::vector<std::vector<int*> > NP_childMap;
  //! Holds a list of all of nodes decendants
  std::vector<std::vector<int*> > NP_decend;
  //! Holds a list of all my predecessors
  std::vector<std::vector<int*> > NP_pred;
  //! List of all nodes I am master of
  std::vector<std::vector<int*> > NP_masterIndex;
  //! list of all nodes which have a lower density
  std::vector<std::vector<int*> > NP_lessDensity;

  //! the list of covariance features for biasing densities
  typename std::vector<covHolder<double> > *NP_covHolder;

  //! holds the node ID of each stem for each class
  std::vector<int*> NP_stem;
  //! holds parent for a sample in cluster space
  std::vector<int*> NP_parent;
  //! lists who a nodes master is
  std::vector<int*> NP_master;
  //! lists the index to the class I am in (this is the count value for stem)
  std::vector<int*> NP_classIndex;
  //! reverse index to the stem for which I am a master
  std::vector<int*> NP_revStemVal;

  //! lists the key number for any given node
  std::vector<int> NP_keyVal;
  //! Returns the number of samples in each class
  std::vector<int> NP_classSize;
  //! Holds the total number of decendants for a node
  std::vector<int> NP_decCount;
  //! Tells which links would have been ideal in the meta-classifier
  std::vector<int> NP_idealLinks;
  //! number of children for a given node
  std::vector<int> NP_children;
  //! tells how many predecessors I have
  std::vector<int> NP_predCount;
  //! counts how many nodes I am master of
  std::vector<int> NP_masterIndexCount;
  //! Holds the total number of decendants for a node in this class
  std::vector<int> NP_subDecCount;
  //! Tracks how many nodes have a lower density than me
  std::vector<int> NP_lessDensityCount;

  //! Returns if the sample has a low density in the map
  std::vector<bool> NP_lowDensity;
  //! tells if this node has a stem attached to it
  std::vector<bool> NP_revStem;
  //! Tells if this node has been selected or not
  std::vector<bool> NP_selected;

  //! returns the max density value from metaclassify
  FLOAT NP_maxDensity;
  //! the total entropy in this system as measured against density
  FLOAT NP_totalEntropy;
  //! The distance at which to cut a link between two nodes
  FLOAT NP_distanceCut;
  //! the cut off point for children
  FLOAT NP_childCut;
  //! the total mean distance between nodes in the set
  FLOAT NP_meanDistance;
  //! the total standard deviation between all nodes in the set
  FLOAT NP_stdDistance;
  //! total mean density of space
  FLOAT NP_meanDensity;
  //! total sum density of space (used in entropy computation)
  FLOAT NP_sumDensity;
  //! total standard deviation of all node density
  FLOAT NP_stdDensity;
  //! this is the mean number of children for nodes
  FLOAT NP_meanChild;
  //! this is the standard deviation for children
  FLOAT NP_stdChildren;
  //! The mean number of decendents per node
  FLOAT NP_meanDecCount;
  //! the standard deviation for meanDecCount
  FLOAT NP_stdDecCount;
  //! the weight to give to biasing if used
  FLOAT NP_biasWeight;

  //! Simple holder for space size, for faster access
  int NP_spaceSize;
  //! dimensions in this space
  int NP_dimSize;
  //! Simple holder for space size, for faster access (unsigned)
  unsigned int NP_uspaceSize;
  //! dimensions in this space (unsigned)
  unsigned int NP_udimSize;
  //! holds the number of root nodes (stems) for each class
  int NP_stems;
  //! Holds the number of roots
  int NP_roots;
  //! minumum distance between two nodes
  int NP_minDist;
  //! minimum number of decentants for a node
  int NP_minChild;
  //! counts how many nodes have been removed due to low density
  int NP_lowDensCount;
  //! new sample size minus the low density guys
  int NP_newSampleSize;
  //! size of the covHolder in NP_covHolder
  int NP_covHolderSize;

  // *****************************************
  // PRIVATE METHODS
  // *****************************************
  //! convolve your space (first method)  void NPconvolveSpace();
  //! convolve your space, use Euclidian distance (revision 1)
  void NPconvolveSpace();
  //! convolve your space (revision 1) when adding a single point to space
  void NPconvolveSpace(long item);
  //! bias the densities of clustering with prior densities
  void NPbiasConvolveSpace();
  //! find the basic mean for density of nodes
  void NPbasicMeanDensity();
  //! compute entropy using density as a rough estimate as P
  void NPcomputeEntropy();
  //! link your points together
  void NPlinkSpace();
  //! link your points together incrementally
  void NPlinkSpaceIncr();
  //! link your points together called from NPlinkSpace, used for single runs
  void NPdoLinkSpace();
  //! link your points together called from NPlinkSpace, used for incremental
  void NPdoLinkSpace(long item);
  //! map space. Who is below me
  void NPmapSpace();
  //! run your analisys on the space group based upon group weights, on length
  void NPanalizeSpace();
  //! cut lengths, link groups, traverse tree
  void NPevolveSpace();
  //! find interclass variance
  void NPanalizeInterSpace();
  //! evolve space again using interclass variance
  void NPevolveInterSpace();
  //! computer the final class densities
  void NPanalizeClassDensity();
  //! computes the master lists of all points in classes
  void NPcomputeMasters();
  //! resets space size parameters
  void NPparamSpace(int samples, int dim);
  //! create a reverse mapping for NP_Dis for use with iterators
  void NPreverseMap();

public:
  int NP_doLinkMap,NP_doDensityMap,NP_doClassMap,NP_usePolySet;
  // *****************************************
  // PUBLIC METHODS
  // *****************************************
  //! create object with readConfig as defaults
  NPclassify2(readConfig &settings, readConfig &polySet,
             bool commandLineSettings = false);
  NPclassify2();
  ~NPclassify2();
  //! input command line settings for some variables insted of using .conf
  void NPsetup(readConfig &settings, readConfig &polySet,
             bool commandLineSettings = false);

  void NPinputCommandLineSettings(FLOAT inVals[31]);
  //! resizes all vectors if more space is added
  void NPresizeSpace(int samples, int dim);

  //! add a point to the space
  void NPaddPoint(std::vector<int> point);
  //! add whole space (concatinates if data is there)
  /*!
    @param space This is a set of feature vectors as a vector of vectors
    @param sSize This is how many elements you are adding leave blank if \
    the number of elements equals the vector size
  */
  void NPaddSpace(typename std::vector<std::vector<FLOAT> > &space);
  //! echo the space you are working in
  void NPechoSpace();
  //! reset the dataset to empty (does not resize vectors)
  void NPresetSpace(int features, int dims);
  //! reset the dataset to empty (does not resize vectors)
  void NPresetSpace();
  //! bias the convolution of space with prior densities
  void NPsetConvolveBias(std::vector<covHolder<double> > *ch,
                         int listSize,
                         FLOAT bias);
  //! classify space based upon rules given, for the first time
  void NPclassifySpaceNew(bool doBias);
  //! ulternativly use Kmeans to classify space, return error
  /*!
    @param K the number of classes to find
    @param minDiff the minimum difference between errors on which to stop
    @param stopIter the maximum number of iterations to run
  */
  FLOAT NPclassifySpaceKmeans(unsigned int *K, FLOAT *minDiff,
                               unsigned int *stopIter);

  //! classify space in subsequent iterations
  void NPclassifySpacePartial();
  //! return how many stems there are (how many classes)
  int NPgetStemNumber();
  //! return how many stems there are (how many classes)
  /*! account for classes that are too small */
  int NPgetStemNumberEdit();
  //! return the max density value
  FLOAT NPgetMaxDensity();
  //! return the mean density
  FLOAT NPgetMeanDensity();
  //! get the Standard Deviation of density
  FLOAT NPgetStdDensity();
  //! return the Mean Density per inter group
  typename std::vector<FLOAT>* NPgetMeanClassDensity();
  //! return the Standard Deviation per inter group
  typename std::vector<FLOAT>* NPgetStdClassDensity();
  //! get a measure of entropy as measured with density as P
  FLOAT NPgetEntropy();
  //! return if this point is a low density point
  bool NPisLowDensity(int item);
  //! returns if this item is a stem
  bool NPisStem(int item);
  //! return density weight map (linear)
  typename std::vector<FLOAT> NPgetDensity();
  //! return density weight map (linear)
  typename std::vector<FLOAT>* NPgetDensityPtr();
  //! Returns a vector of all class parents
  std::vector<int*> NPgetStems();
  //! Return how many points/vectors are in class n (1... n)
  int NPgetClassSize(int _Class);
  //! return what is set as the minimum class size
  int NPgetMinClassSize();
  //! Return the who map for the class list as a FLOAT nested vector
  std::vector<std::vector<int*> >* NPgetClass();
  //! Return class n as a vector (Set of all points/vectors in this class)
  std::vector<int*> NPgetClass(int _Class);
  //! Return item n from class m
  int NPgetClass(int _Class, int item);
  //! Return feature m of vector n
  FLOAT NPgetFeature(int m_feature_index, int n_vector_index);
  //! returns parents
  std::vector<int*> NPgetParents();
  //! Returns a vector of all decendants (children, grandchildren) or all nodes
  std::vector<std::vector<int*> > NPgetChildren();
  //! calculate and return simple bounding boxes for classes
  std::vector<std::vector<int> > NPgetBoundingBoxes(bool ignore);
  //! draw the bounding boxes to an image for you from NPgetBoundingBoxes
  void NPdrawBoundingBoxes(std::vector<std::vector<int> > *BB,
                           Image<PixRGB<FLOAT> > *ibox,
                           int offsetX, int offsetY,
                           PixRGB<FLOAT> pix, bool ignorMin);
  //! analyze classification stratagy. create meta classifier
  void NPmetaClassify(int objects);
  //! dump stats to files about the links for analysis purposes
  void NPdumpLinkInfo(std::string fileName);
  //! gaussian kernel
  inline FLOAT NPgaussKernel(FLOAT x, FLOAT mu, FLOAT sig) const;
  //! gaussian kernel with mu = 0 and sig = 1.0
  inline FLOAT NPgaussKernel(FLOAT z) const;
  //! sigmoid kernel
  inline FLOAT NPsigmoidKernel(FLOAT beta, FLOAT v) const;

};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
