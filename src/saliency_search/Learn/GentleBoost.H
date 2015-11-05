/*!@file Learn/GentleBoost.H GentleBoost Multi-Class Classifier */
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
// Primary maintainer for this file: Dan Parks <danielfp@usc.edu>
// $HeadURL$
// $Id$
//

#ifndef GENTLEBOOST_H_DEFINED
#define GENTLEBOOST_H_DEFINED

#include "Learn/DecisionTree.H"
#include "GentleBoostBinary.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/SortUtil.H"
#include <limits>
#include <math.h>
#include <stdio.h>
#include <map>
#include <iostream>
#include <fstream>

// The data is assumed to be in data[N][M] dimensions where N is the # of dimensions, and M is the # of samples

//! Multi-Class Gentle-AdaBoost using a One vs All, MAX wins voting scheme
class GentleBoost
{
public:
  GentleBoost(int maxTreeSize=1);
  // Multi Class Boost
  //! Get PDF map of each class, with a vector of each observation
  std::map<int,std::vector<float> > predictPDF(const std::vector<std::vector<float> >& data);
  //! Get the most likely class for a particular index in a set of observations
  int getMostLikelyClass(const std::map<int,std::vector<float> >& pdf, int index);
  //! Get the most likely class for a set of observations
  std::vector<int> getMostLikelyClass(const std::map<int,std::vector<float> >& pdf);
  //! Get most likely class per observation
  std::vector<int> predict(const std::vector<std::vector<float> >& data);
  //! Train a set of binary (1vsAll) GentleBoost classifiers on the given data
  void train(const std::vector<std::vector<float> >& data, const std::vector<int>& labels, int maxIters);
  //! Convert class id labels to binary per class labels
  std::map<int,std::vector<int> > convertLabels(const std::vector<int>& labels);
  //! Remove all training
  void clear();
  //! Convenience function to transpose the data dimensions, since the ordering is unusual
  std::vector<std::vector<float> > transpose(const std::vector<std::vector<float> >& data);
  //! Convenience function to print the underlying decision trees for each 1vsAll Classifier
  void printAllTrees();
  //! Create a string representation of the heirarchy
  void writeAllTrees(std::ostream& outstream);
  //! Read a string representation of the heirarchy and build the class
  void readAllTrees(std::istream& instream);
  //! Load a BOOSTed system from a file
  void load(std::string file);
  //! Save a BOOSTed system to a file
  void save(std::string file);

private:
  // Limit the size of the underlying trees (number of splits)
  int itsMaxTreeSize;
  std::map<int,GentleBoostBinary> itsLearners;
};



#endif // GENTLEBOOST_H_DEFINED
