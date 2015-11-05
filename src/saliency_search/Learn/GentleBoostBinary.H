/*!@file Learn/GentleBoostBinary.H GentleBoost 2-Class Classifier */
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

#ifndef GENTLEBOOSTBINARY_H_DEFINED
#define GENTLEBOOSTBINARY_H_DEFINED

#include "Learn/DecisionTree.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/SortUtil.H"
#include <limits>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

class GentleBoostBinary
{
public:
  GentleBoostBinary(int maxTreeSize=1);
  // Two Class Boost
  std::vector<float> predict(const std::vector<std::vector<float> >& data);
  std::vector<float> predict(const std::vector<std::vector<float> >& data, std::vector<float> weights);
  void train(const std::vector<std::vector<float> >& data, const std::vector<int>& labels, int maxIters);
  void train(const std::vector<std::vector<float> >& data, const std::vector<int>& labels, int maxIters, std::vector<float>& predictions);
  //! Remove all training
  void clear();
  //! Print underlying decision tree
  void printTree();
  void writeTree(std::ostream& outstream);
  void readTree(std::istream& instream);

private:
  //! Number of times to allow the underlying decision tree to split during it's training
  int itsMaxTreeSize;
  std::deque<rutz::shared_ptr<DecisionNode> > itsNodes;
  std::vector<float> itsWeights;
};

#endif // GENTLEBOOSTBINARY_H_DEFINED
