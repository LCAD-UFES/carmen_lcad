/*!@file Learn/DecisionTree.C Decision Tree Classifier */
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

#include "DecisionTree.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/SortUtil.H"
#include "Util/sformat.H"
#include <limits>
#include <math.h>
#include <stdio.h>

DecisionNode::DecisionNode() :
    itsDim(-1),
    itsLeaf(true),
    itsLeftConstraint(-std::numeric_limits<float>::infinity()),
    itsRightConstraint(std::numeric_limits<float>::infinity()),
    itsClass(1),
    itsParent(NULL)
{
  
}

bool DecisionNode::isValid()
{
  return itsDim>=0;
}

int DecisionNode::printNode(std::string& output,int depth)
{
  int retDepth;
  if(itsParent.is_valid())
    retDepth=itsParent->printNode(output,depth)+1;
  else retDepth=depth;
  char indent[250];
  indent[0]='\0';
  for(int i=0;i<retDepth;i++)
    sprintf(indent,"%s\t",indent);
  output = sformat("%s%sNode[%p]:<Leaf:%s> <Dim:%d, Class:%d, LeftConstraint:%f, RightConstraint:%f, Parent:%p\n",output.c_str(),indent,this,(itsLeaf)?"t":"f",itsDim,itsClass,itsLeftConstraint,itsRightConstraint,itsParent.get());
  return retDepth;
}

void DecisionNode::writeNode(std::ostream& outstream, bool needEnd)
{  
  outstream << sformat("DIM:%d,CLASS:%d,LC:%f,RC:%f; \n",itsDim,itsClass,itsLeftConstraint,itsRightConstraint);
  if(itsParent.is_valid())
    itsParent->writeNode(outstream,false);
  if(needEnd)
    outstream << std::string("END\n");
}


rutz::shared_ptr<DecisionNode> DecisionNode::readNode(std::istream& instream)
{
  bool nodeIsValid = true;
  const int BUFFER_SIZE = 256;
  char buf[BUFFER_SIZE];
  int depth=0;
  rutz::shared_ptr<DecisionNode> root(NULL), curNode(NULL);
  while(nodeIsValid)
  {
    instream.getline(buf,BUFFER_SIZE);
    int dim,cls;
    float lc,rc;
    int numItemsFound = sscanf(buf,"DIM:%d,CLASS:%d,LC:%f,RC:%f; ",&dim,&cls,&lc,&rc);
    if(numItemsFound == 4)
    {
      rutz::shared_ptr<DecisionNode> node(new DecisionNode());
      node->setDim(dim);
      node->setClass(cls);
      node->setLeftConstraint(lc);
      node->setRightConstraint(rc);
      // If no root, set it up
      if(!root.is_valid())
      {
        root = node;
        curNode = root;
      }
      // Otherwise set parent and move up the chain
      else
      {
        curNode->setParent(node);
        curNode = node;
      }
      depth++;
    }
    else if(std::string(buf).compare("END")==0)
    {
      // We have hit a leaf node, so mark it as such
      if(curNode.is_valid())
      {
        curNode->setLeaf(true);
      }
      else
      {
        LFATAL("Empty node list in decision tree");
      }
      nodeIsValid = false;
    }
    else
    {
      LFATAL("Incomplete node representation at depth %d num found %d buffer[%s]",depth,numItemsFound,buf);
      nodeIsValid = false;
    }
  }
  return root;
}



// Given vector of data, decide whether the data is within or outside the class
// Returns vector of [classId/0] where 0 if data not determined by this node, and classId if it is
std::vector<int> DecisionNode::decide(const std::vector<std::vector<float> >& data)
{
  ASSERT(int(data.size())>itsDim && itsDim>=0);
  std::vector<int> y(data[itsDim].size(),1);
  rutz::shared_ptr<DecisionNode> parNode = itsParent;
  // Handle parents weighting
  if(itsParent.is_valid())
  {
    std::vector<int> py = parNode->decide(data);
    for(uint s=0;s<data[itsDim].size();s++)
	{
      y[s] *= py[s];
	}
  }

  int inVal=1;
  // The leaf node actually makes the class judgement
  if(itsLeaf)
  {
    inVal = itsClass;
  }

  // Check if data is within right/left constraint 
  for(uint s=0;s<data[itsDim].size();s++)
  {
    if(y[s]>0)
      y[s] *= (data[itsDim][s] < itsRightConstraint && data[itsDim][s] >= itsLeftConstraint) ? inVal : 0;
  }

  return y;
}

size_t DecisionNode::getDim()
{
  return itsDim;
}

void DecisionNode::setDim(size_t dim)
{
  itsDim = dim;
}

void DecisionNode::setLeaf(bool isLeaf)
{
  itsLeaf = isLeaf;
  // Non leaf nodes do not have a class
  if(!itsLeaf)
    itsClass=0;
}

void DecisionNode::setParent(rutz::shared_ptr<DecisionNode> parent)
{
  itsParent = parent;
}


void DecisionNode::setLeftConstraint(float constraint)
{
  itsLeftConstraint = constraint;
}

void DecisionNode::setRightConstraint(float constraint)
{
  itsRightConstraint = constraint;
}

void DecisionNode::setClass(int classId)
{
  itsClass = classId;
}

int DecisionNode::getClass()
{
  return itsClass;
}

float DecisionNode::split(const std::vector<std::vector<float> >& data, const std::vector<int>& labels,const std::vector<float>& weights, rutz::shared_ptr<DecisionNode>& left, rutz::shared_ptr<DecisionNode>& right, const rutz::shared_ptr<DecisionNode> parent)
{
  left = rutz::shared_ptr<DecisionNode>(new DecisionNode);
  right = rutz::shared_ptr<DecisionNode>(new DecisionNode);
  left->setParent(parent);
  right->setParent(parent);
  //LINFO("Splitting data on node %p",this);
  // Data is of size NDxNT, where ND = # of feature dimensions, and NT = # of training samples
  ASSERT(data.size() > 0);
  // Number of training samples
  uint tr_size = data[0].size();
  // Store the lowest error, dimension of lowest error, and direciton [+1/-1] of lowest error
  std::vector<float> bestErr;
  std::vector<size_t> bestErrIdx;
  std::vector<float> bestErrDir;
  // Iterate over feature dimensions
  for(uint d=0;d<data.size();d++)
  {      
    // Get the rank order of data for this dimension
    std::vector<size_t> dindices;
    util::sortrank(data[d],dindices);
    std::vector<float> dsorted=data[d];
    // Sort data for this dimension
    std::sort(dsorted.begin(),dsorted.end());

    // For the current dimension, build a weighted value for each of the positive and negative data samples (and consolidate identical data)
    std::vector<float> vPos(tr_size);
    std::vector<float> vNeg(tr_size);
  
    uint i=0,j=0;
    while(i<dsorted.size())
	{
	  uint k = 0;
	  while(i + k < dsorted.size() && dsorted[i] == dsorted[i+k])
      {
        if(labels[dindices[i+k]] > 0)
          vPos[j] += weights[dindices[i+k]];
        else
          vNeg[j] += weights[dindices[i+k]];
        k++;
      }
	  i += k;
	  j++;
	}
    // Resize to the number of unique data points
    vNeg.resize(j);
    vPos.resize(j);

    std::vector<float> err(vPos.size());
    std::vector<float> invErr(vPos.size());
      
    std::vector<float> iPos(vPos.size());
    std::vector<float> iNeg(vNeg.size());
    // Build cumulative sum over the weights of the sorted data
    for(i=0;i<iPos.size();i++)
	{
      if(i==0)
      {
        iPos[0] = vPos[0];
        iNeg[0] = vNeg[0];
      }
      else
      {
        iPos[i] = iPos[i-1] + vPos[i];
        iNeg[i] = iNeg[i-1] + vNeg[i];
      }
	}
      
    // Total negative/positive training weight
    float totalN = (iNeg.size()>0) ? iNeg[iNeg.size()-1] : 0;
    float totalP = (iPos.size()>0) ? iPos[iPos.size()-1] : 0;
    
    // If there is no weight on the negative or positive side, then this will result in a trivial split where all the data will end up in one child
    // as a result, and this will have no error, so no point in doing any more evaluation on this split()
    if(totalN<0.00001 || totalP<0.00001)
       return std::numeric_limits<float>::max();

    // Calculate the error if we were to split the data at each index (in  both directions)
    for(i=0;i<j;i++)
	{
      // Deviation from original: treat error as a percentage of positive and negative 
	  err[i] = (iPos[i])/totalP + (totalN - iNeg[i])/totalN;
	  invErr[i] = (iNeg[i])/totalN + (totalP - iPos[i])/totalP;
      //printf("i %d, j %d err %f, inverr %f iPos %f, iNeg %f, totalP %f, totalN %f\n",i,j,err[i],invErr[i],iPos[i],iNeg[i],totalP,totalN);
	}
  
    // Find minimum error
    size_t errMinIdx = std::distance(err.begin(),std::min_element(err.begin(),err.end()));

    // Find minimum inverse error
    size_t invErrMinIdx = std::distance(invErr.begin(),std::min_element(invErr.begin(),invErr.end()));

    //LINFO("For dimension %u, err[%Zu] %f, invErr[%Zu] %f, totalN %f, totalP %f line %f, invline %f",d,errMinIdx,err[errMinIdx],invErrMinIdx,invErr[invErrMinIdx],totalN,totalP,dsorted[errMinIdx],dsorted[invErrMinIdx]);

    // Technically for err/invErr, the higher one is the % right, and the lower one is the % wrong (if we flip the error direction accordingly)
    //Determine lowest error and store for this dimension
    if(err[errMinIdx] < invErr[invErrMinIdx])
	{
	  bestErr.push_back(err[errMinIdx]);
	  bestErrIdx.push_back(errMinIdx);
	  bestErrDir.push_back(-1);
	}
    else
	{
	  bestErr.push_back(invErr[invErrMinIdx]);
	  bestErrIdx.push_back(invErrMinIdx);
	  bestErrDir.push_back(1);
	}
  }  

  // Find the dimension that will minimize the error
  size_t bestDim = std::distance(bestErr.begin(),std::min_element(bestErr.begin(),bestErr.end()));

  std::vector<float> tmpDimSorted = data[bestDim];
  std::sort(tmpDimSorted.begin(),tmpDimSorted.end());
  std::vector<float> dimSorted(tmpDimSorted.size());
  
  uint i = 0;
  uint j = 0;

  // Consolidate dimension as it was done in the previous loop, and then use this to extract the correct columns for the threshold
  while(i<dimSorted.size())
  {
    uint k = 0;
    while(i+k<dimSorted.size() && tmpDimSorted[i] == tmpDimSorted[i+k])
	{
	  dimSorted[j] = tmpDimSorted[i];
	  k++;
	}
    i += k;
    j++;
  }

  dimSorted.resize(j);

  // Select the midpoint between the two data points that act as the dividing point
  float threshold = (dimSorted[bestErrIdx[bestDim]] + dimSorted[std::min(bestErrIdx[bestDim]+1,dimSorted.size()-1)])/2.0;

  left->setDim(bestDim);
  left->setRightConstraint(threshold);
  left->setClass(bestErrDir[bestDim]);

  right->setDim(bestDim);
  right->setLeftConstraint(threshold);
  right->setClass(-bestErrDir[bestDim]);
  return bestErr[bestDim];
}


DecisionTree::DecisionTree(int maxSplits) :
    itsMaxSplits(maxSplits)
{
}

std::deque<rutz::shared_ptr<DecisionNode> > DecisionTree::getNodes()
{
  return itsNodes;
}

void DecisionTree::addNode(rutz::shared_ptr<DecisionNode> node)
{
  itsNodes.push_back(node);
}
   
void DecisionTree::printTree()
{
  std::deque<rutz::shared_ptr<DecisionNode> >::iterator itr;
  LINFO("Printing Tree of %Zu nodes",itsNodes.size());
  int i=0;
  for(itr=itsNodes.begin();itr!=itsNodes.end();itr++)
  {
    rutz::shared_ptr<DecisionNode> n=*itr;
    if(!n.is_valid())
    {
      LINFO("Node[%d] <Invalid Pointer>",i);
      continue;
    }
    std::string output;
    n->printNode(output);
    LINFO("%s",output.c_str());
    i++;
  }
}

std::vector<int> DecisionTree::predict(const std::vector<std::vector<float> >& data, std::vector<float> weights)
{
  ASSERT(data.size()>0);
  if(weights.size()==0)
    weights.resize(data[0].size(),1.0F);
  size_t sampleDim = data[0].size();
  std::vector<int> prediction(sampleDim);
  std::deque<rutz::shared_ptr<DecisionNode> >::iterator itr;
  for(itr=itsNodes.begin();itr!=itsNodes.end();itr++)
  {
    std::vector<int> y = (*itr)->decide(data);
    for(uint i=0;i<y.size();i++)
    {
      prediction[i] += (y[i])*weights[i];
    }
  }
  return prediction;
}

//function train(data, labels, weights=<empty>)
// This is a binary classifier, and labels must be -1/1
void DecisionTree::train(const std::vector<std::vector<float> >& data, const std::vector<int>& labels, std::vector<float> weights)
{
  ASSERT(labels.size()==weights.size() || weights.size()==0);
  // If weights not specified, set them all to 1.0
  if(weights.size()==0)
  {
    weights.resize(labels.size(),1.0/float(labels.size()));
  }
  // Clear out the nodes for this tree
  itsNodes.clear();

  rutz::shared_ptr<DecisionNode> tmpNode(new DecisionNode());

  // Split the temporary node to get the first binary threshold split
  rutz::shared_ptr<DecisionNode> left,right;
  tmpNode->split(data,labels,weights,left,right);

  if(!left->isValid() || !right->isValid())
  {
    LINFO("Split could not find a non-trivial cut in the remaining data");
    return;
  }
  itsNodes.push_back(left);
  itsNodes.push_back(right);

  // Determine how well the predictions correlate with the ground truth labels 
  // Separately test the correlation and anticorrelation for both the left and right nodes
  std::vector<int> propAns = left->decide(data);
  float leftPos=0,leftNeg=0;
  for(uint a=0;a<propAns.size();a++)
  {
    if(propAns[a] == labels[a])
      leftPos += weights[a];
    else if(propAns[a] == -labels[a])
      leftNeg += weights[a];
  }
  propAns = right->decide(data);
  float rightPos=0,rightNeg=0;
  for(uint a=0;a<propAns.size();a++)
  {
    if(propAns[a] == labels[a])      
      rightPos += weights[a];
    else if(propAns[a] == -labels[a])
      rightNeg += weights[a];
  }

  // Build a list of left/right node errors
  // Here max(pos,neg) is the % correct, and min(pos,neg) is % wrong
  std::deque<float> errs;
  errs.push_back(std::min(leftPos,leftNeg));
  errs.push_back(std::min(rightPos,rightNeg));

  // If we have no classification going on at left/right, then splitting is useless
  if(leftPos + leftNeg == 0)
    return;

  if(rightPos + rightNeg == 0)
    return;

  // Rank the errors based on ascending order
  std::deque<size_t> eIndex;
  util::sortrank(errs,eIndex);

  // Copy the  errors and nodes into tmp variables
  std::deque<float> eTmp = errs;
  std::deque<rutz::shared_ptr<DecisionNode> > tmpNodes = itsNodes;
  // Clear originals
  errs.clear();
  itsNodes.clear();
  // Put errors and nodes back in reverse order, with highest first
  for(int i=eIndex.size()-1;i>=0;i--)
  {
    errs.push_back(eTmp[eIndex[i]]);
    itsNodes.push_back(tmpNodes[eIndex[i]]);
  }

  std::deque<rutz::shared_ptr<DecisionNode> > splits;
  std::deque<float> splitErrs;
  std::deque<float> deltas;

  // Already split once, now split the remaining desired number of times (unless we get no error first)
  for(uint i=1;i<itsMaxSplits;i++)
  {
    ASSERT(itsNodes.size()>deltas.size());
    std::deque<rutz::shared_ptr<DecisionNode> >::iterator nodeItr=itsNodes.begin()+deltas.size();
    // Go through each node and determine the optimal split for that node
    // Only bother to determine the optimal split if it hasn't been done for that node yet (a delta has not been calculated yet)
    for(uint j=deltas.size();j<errs.size();j++,nodeItr++)
	{        
      ASSERT(nodeItr!=itsNodes.end());
      // Select the current node to test
	  rutz::shared_ptr<DecisionNode> curNode = *(nodeItr);
	  // Run the data through the node being tested
	  std::vector<int> curNodeOut = curNode->decide(data);

      // Take only the data where the node classified it as within class as a mask
	  std::vector<uint> mask;
	  for(uint idx=0;idx<curNodeOut.size();idx++)
      {
        if(curNodeOut[idx] == curNode->getClass())
          mask.push_back(idx);
      }

      leftPos=0,leftNeg=0;
      rightPos=0,rightNeg=0;
      float spliterr;
      // Check that there is actually masked data
      if(mask.size()>0)
      {
        // Apply mask to data, labels, and weights, and then split the nodes based on this weighted subset
        std::vector<std::vector<float> > maskedData(data.size());
        std::vector<int> maskedLabels;
        std::vector<float> maskedWeights;
        bool allTrueLabels=true;
        bool allFalseLabels=true;
        for(uint idx=0;idx<mask.size();idx++)
        {
          for(uint idx2=0;idx2<data.size();idx2++)
          {
            maskedData[idx2].push_back(data[idx2][mask[idx]]);
          }
          if(labels[mask[idx]]==-1)
            allTrueLabels=false;
          if(labels[mask[idx]]==1)
            allFalseLabels=false;
          maskedLabels.push_back(labels[mask[idx]]);
          maskedWeights.push_back(weights[mask[idx]]);
        }
        // Check to make sure that all classified data is not already correct (in which case splitting is pointless)
        if((allTrueLabels && curNode->getClass()==1)||(allFalseLabels && curNode->getClass()==-1))
        {
          LINFO("Ignoring split of node with no misclassifications");
          leftPos=leftNeg=rightPos=rightNeg=0;
          spliterr = std::numeric_limits<float>::max();
        }
        else
        {
          // Calculate the split error
          spliterr = curNode->split(maskedData,maskedLabels,maskedWeights,left,right,curNode);
	  
          // Determine how well the predictions correlate with the ground truth labels 
          // Separately test the correlation and anticorrelation for both the left and right nodes
          std::vector<int> propAns = left->decide(data);
          for(uint a=0;a<propAns.size();a++)
          {
            if(propAns[a] == labels[a])
              leftPos += weights[a];
            else if(propAns[a] == -labels[a])
              leftNeg += weights[a];
          }
          propAns = right->decide(data);
          for(uint a=0;a<propAns.size();a++)
          {
            if(propAns[a] == labels[a])      
              rightPos += weights[a];
            else if(propAns[a] == -labels[a])
              rightNeg += weights[a];
          }
        }
      }
      else
      {
        // Should not ever happen: No masked data, which means node doesn't contain any training data?!?!?
        LFATAL("No masked data, node does not contain any training data");
        leftPos=leftNeg=rightPos=rightNeg=0;
        spliterr = std::numeric_limits<float>::max();
      }
      // Append the left/right nodes to the list of split nodes (which will be used to select the best split later)
      splits.push_back(left);
      splits.push_back(right);
          
      // Build delta error
      if(leftPos + leftNeg == 0 || rightPos + rightNeg == 0)
	    deltas.push_back(0);
	  else
      {
        LINFO("Delta for splitting node %d is %f, errs[] %f, spliterr %f",j,errs[j]-spliterr,errs[j],spliterr);
        deltas.push_back(errs[j]-spliterr);
      }
        
      splitErrs.push_back(std::min(leftPos,leftNeg));
	  splitErrs.push_back(std::min(rightPos,rightNeg));
	}
    
    std::deque<float>::iterator maxElemItr = std::max_element(deltas.begin(),deltas.end());
    LINFO("Best Delta %f For Iter %u",*maxElemItr,i);
    // If the smallest delta is zero, we're done
    if(*maxElemItr < 0.000001)
    {
      LINFO("Delta is zero or too small, done");
      return;
    }
    // Get the best split index
    uint bestSplit = std::distance(deltas.begin(),maxElemItr);
      
    // Make the split node not a leaf anymore
    itsNodes[bestSplit]->setLeaf(false);

    // Remove the node that we are splitting
    itsNodes.erase(itsNodes.begin()+bestSplit);
    errs.erase(errs.begin()+bestSplit);
    deltas.erase(deltas.begin()+bestSplit);
      
    // Insert the new left/right pair at the end
    ASSERT((splits.begin()+2*bestSplit)->is_valid());
    ASSERT((splits.begin()+2*bestSplit+1)->is_valid());
    itsNodes.push_back(*(splits.begin()+2*bestSplit));
    itsNodes.push_back(*(splits.begin()+2*bestSplit+1));
      
    // Insert the corresponding split errs into the node err list
    errs.push_back(*(splitErrs.begin()+2*bestSplit));
    errs.push_back(*(splitErrs.begin()+2*bestSplit+1));
      
    // Remove the chosen splits from the split list, since they are now official nodes
    splitErrs.erase(splitErrs.begin()+2*bestSplit,splitErrs.begin()+2*bestSplit+2);
    splits.erase(splits.begin()+2*bestSplit,splits.begin()+2*bestSplit+2);
  }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

