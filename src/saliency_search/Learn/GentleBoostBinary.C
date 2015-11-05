/*!@file Learn/GentleBoostBinary.C GentleBoost 2-Class Classifier */
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

#include "Learn/GentleBoostBinary.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "Util/SortUtil.H"
#include <limits>
#include <math.h>
#include <stdio.h>



GentleBoostBinary::GentleBoostBinary(int maxTreeSize) :
    itsMaxTreeSize(maxTreeSize)
{
}


std::vector<float> GentleBoostBinary::predict(const std::vector<std::vector<float> >& data)
{
  return predict(data,itsWeights);
}

// Real valued approximation of the answer by committee of weak learners - 2 class problem
std::vector<float> GentleBoostBinary::predict(const std::vector<std::vector<float> >& data, std::vector<float> weights)
{
  ASSERT(weights.size()==itsNodes.size());
  ASSERT(data.size()>0);
  std::vector<float> pred(data[0].size());
  for(size_t i=0;i<itsNodes.size();i++)
  {
    std::vector<int> tmppred=itsNodes[i]->decide(data);
    for(size_t j=0;j<pred.size();j++)
	{
	  pred[j]+=float(tmppred[j])*weights[i];
	}
  }
  return pred;
}


void GentleBoostBinary::train(const std::vector<std::vector<float> >& data, const std::vector<int>& labels, int maxIters)
{
  std::vector<float> predictions;
  train(data,labels,maxIters,predictions);

}

void GentleBoostBinary::train(const std::vector<std::vector<float> >& data, const std::vector<int>& labels, int maxIters, std::vector<float>& predictions)
{
  ASSERT(data.size()>0);
  int nSamples = int(data[0].size());
  ASSERT(int(labels.size())==nSamples);
  std::vector<float> dataWeights;
  if(predictions.size()>0)
  {
    dataWeights = std::vector<float>(nSamples);
    for(size_t i=0;i<predictions.size();i++)
    {
      dataWeights[i]=exp(-(labels[i]*predictions[i]));
    }
  }
  else
  {
    dataWeights = std::vector<float>(nSamples,1.0F/float(nSamples));
    predictions = std::vector<float>(nSamples);
  }
  
  for(int iter=0;iter<maxIters;iter++)
  {
    rutz::shared_ptr<DecisionTree> learner(new DecisionTree(itsMaxTreeSize));
    learner->train(data,labels,dataWeights);
    std::deque<rutz::shared_ptr<DecisionNode> > curNodes = learner->getNodes();
    if(curNodes.size()==0)
    {
      LINFO("Training complete, only trivial cuts found");
      return;
    }
    for(size_t idx=0;idx<curNodes.size();idx++)
	{
	  rutz::shared_ptr<DecisionNode> curNode = curNodes[idx];
	  std::vector<int> curNodeOut = curNode->decide(data);
	  float s1=0.0F,s2=0.0F;
	  for(size_t i=0;i<curNodeOut.size();i++)
      {
        if(curNodeOut[i]==1)
		{
		  if(labels[i]==1)
          {
            // Weighted sum of true positives 
            s1 += dataWeights[i];
          }
		  else if(labels[i]==-1)
          {
            // Weighted sum of false positives
            s2 += dataWeights[i];
          }
		}
        // Deviation from original, take into account true negatives/false negatives when evaluating weights
        else if(curNodeOut[i]==-1)
		{
		  if(labels[i]==-1)
          {
            // Weighted sum of true negatives 
            s1 += dataWeights[i];
          }
		  else if(labels[i]==1)
          {
            // Weighted sum of false negatives
            s2 += dataWeights[i];
          }
		}
      }
	  if(s1==0.0F && s2==0.0F)
	    continue;
	  float alpha = (s1-s2)/(s1+s2);
	  itsWeights.push_back(alpha);
	  itsNodes.push_back(curNode);
	  for(size_t i=0;i<predictions.size();i++)
      {
        predictions[i] += curNodeOut[i]*alpha;
      }
	}
    float sumDW=0;
    for(int i=0;i<nSamples;i++)
	{
	  dataWeights[i] = exp(-1.0F * (labels[i]*predictions[i]));
	  sumDW+=dataWeights[i];
	}
    if(sumDW>0)
	{
	  for(int i=0;i<nSamples;i++)
      {
        dataWeights[i]/=sumDW;
      }
	}
  }
}

void GentleBoostBinary::printTree()
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
    LINFO("Weight: %f\n%s",itsWeights[i],output.c_str());
    i++;
  }
}

void GentleBoostBinary::writeTree(std::ostream& outstream)
{
  rutz::shared_ptr<std::string> output = rutz::shared_ptr<std::string>(new std::string);
  std::deque<rutz::shared_ptr<DecisionNode> >::iterator itr;
  int i=0;
  for(itr=itsNodes.begin();itr!=itsNodes.end();itr++)
  {
    rutz::shared_ptr<DecisionNode> n=*itr;
    if(!n.is_valid())
    {
      continue;
    }
    outstream << sformat("TREEWEIGHT:%f; \n",itsWeights[i]);
    n->writeNode(outstream);
  }
  outstream << std::string("END\n");
}

void GentleBoostBinary::readTree(std::istream& instream)
{
  DecisionNode tmp;
  const int BUFFER_SIZE = 256;
  char buf[BUFFER_SIZE];
  int treeIdx=0;
  
  bool nodeIsValid = true;
  while(nodeIsValid)
  {
    instream.getline(buf,BUFFER_SIZE);
    float treeWeight;
    int numItemsFound = sscanf(buf,"TREEWEIGHT:%f; ",&treeWeight);
    if(numItemsFound == 1)
	{
	  rutz::shared_ptr<DecisionNode> node = tmp.readNode(instream);
	  if(!node.is_valid())
      {
        LFATAL("No tree associated with tree weight at index %d",treeIdx);
        nodeIsValid = false;
      }
	  itsWeights.push_back(treeWeight);
	  itsNodes.push_back(node);
	  treeIdx++;
	}
    else if(std::string(buf).compare("END")==0)
    {
      nodeIsValid = false;
    }
    else
    {
      LFATAL("Incomplete tree representation at index %d",treeIdx);
      nodeIsValid = false;
    }      
  }
  
}


void GentleBoostBinary::clear()
{
  itsNodes.clear();
  itsWeights.clear();
}
