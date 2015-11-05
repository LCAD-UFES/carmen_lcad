/*!@file src/Features/test-GentleBoost.C */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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

#include "Component/ModelManager.H"
#include "Learn/GentleBoost.H"
#include "rutz/rand.h"
#include "rutz/trace.h"
#include "Util/SortUtil.H"
#include "Util/Assert.H"
#include <math.h>
#include <fcntl.h>
#include <limits>
#include <string>
#include <stdio.h>


void makeData(const int numCategories, const uint sampleDim, std::vector<std::vector<float> >& data, std::vector<int>& labels, bool printData);

int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager manager("Test Decision Tree");


  // Create log likelihood classifier and local binary patterns objects
  uint nDim=4;
  int numCategories=3;
  int maxIters=1;
  int maxTreeSize = 4;
  GentleBoost gb(maxTreeSize);
  std::string saveDataFile("tmp.dat");
  std::string compareDataFile("tmp.cmp.dat");

  if (manager.parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false)
    return 0;

  manager.start();
  std::vector<std::vector<float> > traindata(nDim);
  std::vector<int> trainlabels;
  std::vector<float> dimMeanIn(nDim), dimMeanOut(nDim), dimVarIn(nDim,1.0F), dimVarOut(nDim,1.0F);
  for(uint i=0;i<nDim;i++)
    {
      dimMeanIn[i] = nDim-i;
      dimMeanOut[i] = -(nDim-i);
    }
  makeData(numCategories,1000,traindata,trainlabels,false);
  // Train the classifier on the training set
  gb.train(traindata,trainlabels,maxIters);
  gb.save(saveDataFile);
  // Do a cycle of saving and loading and compare to the original save file
  GentleBoost tmpGB;
  tmpGB.load(saveDataFile);
  tmpGB.save(compareDataFile);

  std::map<int,std::vector<float> > trainPDF = gb.predictPDF(traindata);
  std::vector<int> trainResults = gb.getMostLikelyClass(trainPDF);
  // Validate on training set
  int numCorrect=0;
  for(uint i=0;i<trainlabels.size();i++)
    {
      if(trainResults[i]==trainlabels[i]) numCorrect++;
      //printf("Train Guess %d [Ground Truth %d]\n",trainResults[i],trainlabels[i]);
    }
  printf("Training Accuracy:[Correct/Total]=[%d/%Zu]:%f\n",numCorrect,trainlabels.size(),numCorrect/float(trainlabels.size()));
  gb.printAllTrees();
  std::vector<std::vector<float> > testdata(nDim);
  std::vector<int> testlabels;
  // Create new data from same distribution as test set
  makeData(numCategories,10,testdata,testlabels,true);
  // Classify test set
  std::map<int,std::vector<float> > testPDF = gb.predictPDF(testdata);
  std::vector<int> testResults = gb.getMostLikelyClass(testPDF);
  numCorrect=0;
  for(uint i=0;i<testlabels.size();i++)
    {
      if(testResults[i]==testlabels[i]) numCorrect++;
      std::map<int,std::vector<float> >::iterator litr;
      printf("Guess %d [",testResults[i]);
      for(litr=testPDF.begin();litr!=testPDF.end();litr++)
        {
          printf("(%d)%f, ",litr->first,litr->second[i]);
        }
      printf("] *** Ground Truth %d\n",testlabels[i]);
    }
  printf("Accuracy:[Correct/Total]=[%d/%Zu]:%f\n",numCorrect,testlabels.size(),numCorrect/float(testlabels.size()));
  manager.stop();

}

void makeData(const int numCategories, const uint sampleDim, std::vector<std::vector<float> >& data, std::vector<int>& labels, bool printData)
{
  // Create uniform random number generator
  rutz::urand rgen(time((time_t*)0)+getpid());
  ASSERT(data.size()>0);
  // Create data and labels
  const uint dataDim=(uint) data.size();

  for(uint i=0;i<sampleDim;i++)
    {
      int l=rgen.idraw(numCategories)+1;
      if(printData) printf("data[][%u]: l=%d; ",i,l);
      for(uint j=0;j<dataDim;j++)
	{
	  data[j].push_back(rgen.fdraw_range(l-0.75,l+0.75));//*dimVarIn[j]+dimMeanIn[j]);
          if(printData) printf("%f, ",data[j][i]);
	}      
      if(printData) printf("\n");
      labels.push_back(l);
    }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */



