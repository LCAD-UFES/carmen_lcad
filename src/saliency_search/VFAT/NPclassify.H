/*!@file VFAT/NPclassify.H  Test the nonparametric classifier
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/NPclassify.H $
// $Id: NPclassify.H 6393 2006-03-26 00:57:36Z rjpeters $
//

#ifndef NPCLASSIFY_H_DEFINED
#define NPCLASSIFY_H_DEFINED

#include <vector>
#include "Util/readConfig.H"

class NPclassify
{
private:
  std::vector<long> children;
  std::vector<long> parent;
  std::vector<long> childMapTot;
  std::vector<long> childMapTotLast;
  std::vector<long> stem;
  std::vector<long> root;
  std::vector<long> master;
  std::vector<long> masterIndex;
  std::vector<long> classSize;
  std::vector<long> childInterCount;
  std::vector<long> idealLinks;
  //! density for this point
  std::vector<double> density;
  std::vector<double> distance;
  std::vector<double> meanInterDistance;
  std::vector<double> meanInterChild;
  std::vector<double> meanInterDensity;
  std::vector<double> stdInterDistance;
  std::vector<double> stdInterDensity;
  std::vector<double> stdInterChild;
  std::vector<double> trainMeasure;
  //! am I a stem for this class
  std::vector<bool> revStem;
  std::vector<bool> childMapDone;
  std::vector<bool> lowDensity;
  std::vector<bool> selected;
  std::vector< std::vector<long> > child;
  std::vector< std::vector<long> > childMap;
  std::vector< std::vector<long> > childInterMap;
  std::vector< std::vector<long> > Class;
  std::vector< std::vector<double> > D;
  std::vector< std::vector<double> > Dis;
  std::vector< std::vector<double> > Space;
  long spaceSize,stems,roots,iteration,defaultSize;
  long minDist,minChild;
  long hardClassSize,hardLinkSize;
  double distanceCut, childCut;
  double maxDensity,trainChildWeight;
  double Con1,Con2,Con3;
  double sumSquares,sum;
  double meanDistance,stdDistance,meanChildren,stdChildren;
  double meanChildMap,stdChildMap,meanDensity,stdDensity;
  double DWeight1, CWeight1, IDWeight1, ICWeight1;
  double DWeight2, CWeight2, IDWeight2, ICWeight2;
  double DenWeight1, DenWeight2, preDenWeight1, preDenWeight2;
  double thresh1,thresh2,thresh3,thresh4,measure1,measure2;
  bool notDone;
  double polyDensObjectCut1,polyDensObjectCut2,polyDensObjectCut3;
  double polySpaceChildCut1,polySpaceChildCut2,polySpaceChildCut3;
  bool CLS;

  // *****************************************
  // PRIVATE METHODS
  // *****************************************
  //! convolve your space (first method)
  void convolveSpace();
  //! convolve your space (revision 1)
  void convolveSpace2();
  //! link your points together
  void linkSpace();
  //! map space. Who is below me
  void mapSpace();
  //! run your analisys on the space group based upon group weights, on length
  void analizeSpace();
  //! cut lengths, link groups, traverse tree
  void evolveSpace();
  //! find interclass variance
  void analizeInterSpace();
  //! evolve space again using interclass variance
  void evolveInterSpace();
  //! computes the master lists of all points in classes
  void computeMasters();
  //! resizes all vectors if more space is added
  void resizeSpace();
public:
  int doLinkMap,doDensityMap,doClassMap,usePolySet;
  // *****************************************
  // PUBLIC METHODS
  // *****************************************
  //! create object with readConfig as defaults
  NPclassify(readConfig &settings, readConfig &polySet,
             bool commandLineSettings = false);
  ~NPclassify();
  //! input command line settings for some variables insted of using .conf
  void inputCommandLineSettings(double _distance, double _children,
                               double Idistance, double Ichildren,
                               long _hardClassSize, long _hardLinkSize,
                               double _polyDensObjectCut1,
                               double _polyDensObjectCut2,
                               double _polyDensObjectCut3);
  //! add a point to the space
  void addPoint(std::vector<long> point);
  //! add whole space (concatinates if data is there)
  /*!
    @param space This is a set of feature vectors as a vector of vectors
    @param sSize This is how many elements you are adding leave blank if \
    the number of elements equals the vector size
  */
  void addSpace(std::vector<std::vector<double> > &space,long sSize = 0);
  //! echo the space you are working in
  void echoSpace();
  //! reset the dataset to empty (dose not resize vectors)
  void resetSpace();
  //! classify space based upon rules given, for the first time
  void classifySpaceNew();
  //! classify space using K-Means
  void classifySpaceKmeans(unsigned int K);
  //! classify space in subsequent iterations
  void classifySpacePartial();
  //! return how many stems there are (how many classes)
  long getStemNumber();
  //! return the max density value
  double getMaxDensity();
  //! return if this point is a low density point
  bool isLowDensity(long item);
  //! return if this item is a stem
  bool isStem(long item);
  //! return density weight map (linear)
  std::vector<double> getDensity();
  //! Returns a vector of all class parents
  std::vector<long> getStems();
  //! Return how many points/vectors are in class n (1... n)
  long getClassSize(long _Class);
  //! return what is set as the minimum class size
  long getMinClassSize();
  //! Return class n as a vector (Set of all points/vectors in this class)
  std::vector<long> getClass(long _Class);
  //! Return item n from class m
  long getClass(long _Class, long item);
  //! Return feature m of vector n
  double getFeature(long m_feature_index, long n_vector_index);
  //! returns parents
  std::vector<long> getParents();
  //! Returns a vector of all decendants (children, grandchildren) or all nodes
  std::vector<std::vector<long> > getChildren();
  //! calculate and return simple bounding boxes for classes
  std::vector<std::vector<long> > getBoundingBoxes();
  //! analyze classification stratagy. create meta classifier
  void metaClassify(int objects);
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
