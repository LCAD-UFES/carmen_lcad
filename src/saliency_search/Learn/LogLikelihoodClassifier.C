/*!@file Learn/LogLikelikhoodClassifier.C Log Likelihood Classifier for Histograms module */
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

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <map>
#include <numeric>
#include "Util/Assert.H"
#include "Util/log.H"
#include "LogLikelihoodClassifier.H"


LogLikelihoodClassifier::LogLikelihoodClassifier(int k)
{
  itsK=k;
}

float LogLikelihoodClassifier::calcLogLikelihood(const std::vector<float>& sample, const std::vector<float>& model)
{
  float ll=0;
  ASSERT(sample.size()==model.size());
  for(uint i=0;i<sample.size();i++)
    {
      ASSERT(model[i]!=0);
      // Can't take log of zero
      ll+=sample[i]*log(model[i]);
    }
  ASSERT(!isnan(ll));
  return ll;
}

void LogLikelihoodClassifier::addModel(std::vector<float> hist, int id)
{
  if(itsHistLength <= 0)
    itsHistLength = hist.size();
  ASSERT(hist.size() == itsHistLength);
  // Initialize if no key is present
  if(itsModels.find(id) == itsModels.end())
    itsModels[id] = std::vector<std::vector<float> >();
  itsModels[id].push_back(hist);
}

void LogLikelihoodClassifier::setModels(MapModelVector models)
{
  itsModels = models;
}

LogLikelihoodClassifier::MapModelVector LogLikelihoodClassifier::getModels()
{
  return itsModels;
}

//! Predict using classifier
int LogLikelihoodClassifier::predict(const std::vector<float>& hist)
{
  std::map<int,double>pdf = predictPDF(hist);
  std::map<int,double>::const_iterator piter;
  double maxVal=-std::numeric_limits<double>::max();
  int maxClass = -1;
  for(piter=pdf.begin();piter!=pdf.end();piter++)
    {
      //LINFO("Comparing curVal[%d] %f to currentMax:%f",piter->first,piter->second,maxVal);
      if(piter->second > maxVal)
	{
	  maxClass=piter->first;
	  maxVal=piter->second;
	}
    }
  return maxClass;
}

std::map<int,double> LogLikelihoodClassifier::predictPDF(const std::vector<float>& hist)
{
  // This is a "hybrid" operation.  The K nearest exemplars are determined, in addition to the best performing exemplar for each class
  // This will leave us with a total number of stored likelihoods between NumClasses (if the best K are all among them) and NumClasses+K-1 (if all the best K sit in one class)
  // We add all of these likelihoods up per class and then normalize to get a PDF
  
  // Store the maximum log likelihoods and their corresponding locations with the nearest neighbors
  std::list<float> maxLL = std::list<float>(itsK,-std::numeric_limits<float>::max());
  std::list<int> maxLLPos = std::list<int>(itsK,0);
  MapModelVector::const_iterator mmitr = itsModels.begin(), mmstop = itsModels.end();
  std::map<int,std::list<float> > maxLLClass;

  for(;mmitr!=mmstop;mmitr++)
    {
      maxLLClass[mmitr->first] = std::list<float>(itsK,-std::numeric_limits<double>::max());
      for(uint i=0;i<mmitr->second.size();i++)
	{
	  // Calculate log likelihood on histogram
	  float ll = calcLogLikelihood(mmitr->second[i],hist);
	  bool found=false;
	  // Check if this is one of the best K in class
	  std::list<float>::iterator maxClassIter = maxLLClass[mmitr->first].begin(), maxClassStop=maxLLClass[mmitr->first].end();
	  
	  for(;maxClassIter!=maxClassStop;maxClassIter++)
	    {
	      if(*maxClassIter < ll)
		{
		  maxLLClass[mmitr->first].insert(maxClassIter,ll);
		  found=true;
		  break;
		}
	    }
	  // Only check for overall max if this exemplar is in the best K exemplars in its own class
	  if(found)
	    {
	      ASSERT(maxLL.size()==maxLLPos.size());
	      std::list<float>::iterator maxLLIter = maxLL.begin(), maxLLStop=maxLL.end();	  
	      std::list<int>::iterator maxLLPosIter = maxLLPos.begin();//, maxLLPosStop=maxLLPos.end();
	      for(;maxLLIter!=maxLLStop;maxLLIter++,maxLLPosIter++)
		{
		  if(*maxLLIter < ll)
		    {
		      maxLL.insert(maxLLIter,ll);
		      maxLLPos.insert(maxLLPosIter,mmitr->first);
		      break;
		    }
		}
	    }
	  // Truncate best fits to be number of nearest neighbors in size
	  if(found)
	    {
	      if(maxLLClass[mmitr->first].size()>itsK)
		{
		  maxLLClass[mmitr->first].resize(itsK);
		}
	      if(maxLL.size()>itsK)
		{
		  maxLL.resize(itsK);
		  maxLLPos.resize(itsK);
		}
	    }
	}
    }
  // Count the number of elements per class in the maxLL
  std::map<int,int> numEntries;
  for(std::list<int>::const_iterator lit=maxLLPos.begin();lit!=maxLLPos.end();lit++)
    {
      numEntries[*lit]++;
    }
  // Now resize each class list based on the number of entries
  std::map<int,std::list<float> >::iterator maxClassIter = maxLLClass.begin(), maxClassStop=maxLLClass.end();
  for(;maxClassIter!=maxClassStop;maxClassIter++)
    {
      // Each class gets a guaranteed one entry
      maxClassIter->second.resize(std::max(numEntries[maxClassIter->first],1));
    }
  std::map<int,double> pdf;
  nearestNeighborVotePDF(maxLLClass,pdf);
  return pdf;
}

void LogLikelihoodClassifier::nearestNeighborVotePDF(const std::map<int,std::list<float> >& logLikelihood, std::map<int,double>& pdf)
{
  std::map<int,std::list<float> >::const_iterator llIter = logLikelihood.begin(), llStop = logLikelihood.end();
  float nrm=0;
  int numClasses=0;
  float minVal = std::numeric_limits<float>::max();
  float offset=0;
  for(;llIter!=llStop;llIter++)
    {
      float val=std::accumulate(llIter->second.begin(),llIter->second.end(),0);
      pdf[llIter->first] = val;
      nrm+=val;
      if(val<minVal)
	minVal=val;
      numClasses++;
    }
  ASSERT(numClasses>0 && nrm != 0);

  // Subtract off minimum and normalize to get a pdf
  std::map<int,double>::iterator piter;
  // Now take off minval for each class, if that doesn't cause a discontinuity
  if(fabs(nrm-minVal*numClasses) > 0.0001)
    {
      nrm-=minVal*numClasses;
      offset=minVal;
    }
  for(piter=pdf.begin();piter!=pdf.end();piter++)
    {
      piter->second=(piter->second-offset)/nrm;
      ASSERT(!isnan(piter->second));
    }
}



//! Get number of models
uint LogLikelihoodClassifier::getNumModels()
{
  return itsModels.size();
}




