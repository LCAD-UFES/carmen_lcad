/*!@file Learn/GentleBoost.C GentleBoost Classifier */
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

#include "GentleBoost.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/SortUtil.H"
#include "Util/sformat.H"
#include <limits>
#include <math.h>
#include <stdio.h>


GentleBoost::GentleBoost(int maxTreeSize) :
itsMaxTreeSize(maxTreeSize)
{
}

std::vector<std::vector<float> > GentleBoost::transpose(const std::vector<std::vector<float> >& data)
{
  std::vector<std::vector<float> > out;
  if(data.size()==0)
    return out;
  out.resize(data[0].size());
  for(uint i=0;i<data[0].size();i++)
    {
      for(uint j=0;j<data.size();j++)
	{
	  out[i].push_back(data[j][i]);
	}
    }
  return out;
}

std::map<int,std::vector<float> > GentleBoost::predictPDF(const std::vector<std::vector<float> >& data)
{
  ASSERT(itsLearners.size()>0);
  ASSERT(data.size()>0);
  std::map<int,std::vector<float> > perClassPreds;
  std::map<int,GentleBoostBinary>::iterator litr;
  for(litr=itsLearners.begin();litr!=itsLearners.end();litr++)
    {
      perClassPreds[litr->first] = litr->second.predict(data);
    }
  return perClassPreds;
}

int GentleBoost::getMostLikelyClass(const std::map<int,std::vector<float> >& pdf, int index)
{
  float maxClassVal=-1000;
  int maxClassIdx=-1;
  std::map<int,GentleBoostBinary>::iterator litr;
  for(litr=itsLearners.begin();litr!=itsLearners.end();litr++)
    {
      std::map<int,std::vector<float> >::const_iterator pitr=pdf.find(litr->first);
      ASSERT(pitr != pdf.end());
      if(maxClassVal < pitr->second[index])
	{
	  maxClassVal=pitr->second[index];
	  maxClassIdx=litr->first;
	}
    }
  ASSERT(maxClassIdx>=0);
  return maxClassIdx;
}

std::vector<int> GentleBoost::getMostLikelyClass(const std::map<int,std::vector<float> >& pdf)
{
  ASSERT(itsLearners.size()>0);
  ASSERT(pdf.size()>0);
  std::vector<int> preds;
  int nSamples=(pdf.begin())->second.size();
  for(int s=0;s<nSamples;s++)
    {
      int maxClassIdx = getMostLikelyClass(pdf,s);
      preds.push_back(maxClassIdx);
    }
  return preds;
}

std::vector<int> GentleBoost::predict(const std::vector<std::vector<float> >& data)
{
  std::map<int,std::vector<float> > perClassPreds=predictPDF(data);
  return getMostLikelyClass(perClassPreds);
}


void GentleBoost::train(const std::vector<std::vector<float> >& data, const std::vector<int>& labels, int maxIters)
{
  std::map<int,std::vector<int> > perClassLabels = convertLabels(labels);
  std::map<int,std::vector<int> >::iterator litr;
  for(litr=perClassLabels.begin();litr!=perClassLabels.end();litr++)
    {
      itsLearners[litr->first] = GentleBoostBinary(itsMaxTreeSize);
      itsLearners[litr->first].train(data,litr->second,maxIters);
    }
}

void GentleBoost::printAllTrees()
{
  std::map<int,GentleBoostBinary>::iterator litr;
  for(litr=itsLearners.begin();litr!=itsLearners.end();litr++)
    {
      LINFO("Printing Gentle Boost Binary Classification Tree for Class %d",litr->first);
      litr->second.printTree();
    }
}

void GentleBoost::writeAllTrees(std::ostream& outstream)
{
  rutz::shared_ptr<std::string> output = rutz::shared_ptr<std::string>(new std::string());
  std::map<int,GentleBoostBinary>::iterator litr;
  outstream << sformat("MAXTREES:%d; \n",itsMaxTreeSize);
  for(litr=itsLearners.begin();litr!=itsLearners.end();litr++)
    {     
      outstream << sformat("TREECLASS:%d; \n",litr->first);
      litr->second.writeTree(outstream);
    }
  outstream << std::string("END\n");
}

void GentleBoost::readAllTrees(std::istream& instream)
{
  int treeIdx=0;
  bool treeIsValid = true;
  const int BUFFER_SIZE = 256;
  char buf[BUFFER_SIZE];
  instream.getline(buf,BUFFER_SIZE);
  int numItemsFound = sscanf(buf,"MAXTREES:%d; ",&itsMaxTreeSize);
  if(numItemsFound != 1)
    LFATAL("Invalid GentleBoost format, MAXTREES undefined");
  while(treeIsValid)
    {
      instream.getline(buf,BUFFER_SIZE);
      int treeClass;
      int numItemsFound = sscanf(buf,"TREECLASS:%d; ",&treeClass);
      if(numItemsFound == 1)
	{
	  GentleBoostBinary gbb;
	  gbb.readTree(instream);
	  // Add learner to map
	  itsLearners[treeClass] = gbb;
	  // Another class was added to the map
	  treeIdx++;
	}
      else if(std::string(buf).compare("END")==0)
        {
          treeIsValid = false;
        }
      else
        {
          LFATAL("Incomplete tree representation at index %d",treeIdx);
          treeIsValid = false;
        }
      
    }
  
}

void GentleBoost::save(std::string file)
{
  std::ofstream outf(file.c_str(),std::ofstream::out);
  writeAllTrees(outf);
}

void GentleBoost::load(std::string file)
{
  std::ifstream inf(file.c_str(),std::ofstream::in);  
  readAllTrees(inf);
}


//! Convert per class labels to a set of binary +1/-1 labels for each class
std::map<int,std::vector<int> > GentleBoost::convertLabels(const std::vector<int>& labels)
{
  std::map<int,std::vector<int> > perClassLabels;
  // Number of samples
  int nSamples=labels.size();
  // 
  for(int i=0;i<nSamples;i++)
    {
      if(perClassLabels.find(labels[i])==perClassLabels.end())
	{
	  // Unless specified, all samples are assumed to not be a part of the class
	  perClassLabels[labels[i]] = std::vector<int>(nSamples,-1);
	}
      perClassLabels[labels[i]][i]=1;
    }
  
  return perClassLabels;
}
