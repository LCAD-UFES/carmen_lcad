/*!@file Learn/DecisionTree.H Decision Tree Classifier */
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

#ifndef DECISIONTREE_H_DEFINED
#define DECISIONTREE_H_DEFINED

#include "rutz/shared_ptr.h"
#include <cstdlib>
#include <deque>
#include <map>
#include <list>
#include <vector>
#include <iostream>
#include <fstream>

class DecisionNode; // Forward declaration

class DecisionTree
{
  public:
    DecisionTree(int maxSplits=1);
    // Train a tree on the data, limited by the maximum allowed splits
    void train(const std::vector<std::vector<float> >& data, const std::vector<int>& labels,std::vector<float> weights=std::vector<float>());
    // Predict the result for this binary classifier on the given data using the given weights on the nodes
    std::vector<int> predict(const std::vector<std::vector<float> >& data, std::vector<float> weights=std::vector<float>());
    void printTree();
    std::deque<rutz::shared_ptr<DecisionNode> > getNodes();
    void addNode(rutz::shared_ptr<DecisionNode> node);
  private:
    size_t itsMaxSplits;
    std::deque<rutz::shared_ptr<DecisionNode> > itsNodes;
};

// One node within a decision tree
class DecisionNode
{
  public:
    DecisionNode();
    std::vector<int> decide(const std::vector<std::vector<float> >& data);
    float split(const std::vector<std::vector<float> >& data, const std::vector<int>& labels, const std::vector<float>& weights, rutz::shared_ptr<DecisionNode>& left, rutz::shared_ptr<DecisionNode>& right, const rutz::shared_ptr<DecisionNode> parent=rutz::shared_ptr<DecisionNode>(NULL));
    size_t getDim();
    bool isValid();
    int printNode(std::string& output, int depth=0);
    void writeNode(std::ostream& outstream, bool needEnd=true);
    rutz::shared_ptr<DecisionNode> readNode(std::istream& instream);
    void setDim(size_t dim);
    void setLeaf(bool isLeaf);
    void setParent(rutz::shared_ptr<DecisionNode> parent);
    void setLeftConstraint(float constraint);
    void setRightConstraint(float constraint);
    void setClass(int classId);
    int getClass();
  private:
    int itsDim;
    //float itsThreshold;
    //int itsSign;
    bool itsLeaf;
    float itsLeftConstraint;
    float itsRightConstraint;
    int itsClass;
    // Parent node to the current node
    rutz::shared_ptr<DecisionNode> itsParent;
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


