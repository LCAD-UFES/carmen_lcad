/*!@file src/Features/test-DecisionTree.C */

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
#include "Learn/DecisionTree.H"
#include "rutz/rand.h"
#include "rutz/trace.h"
#include "Util/SortUtil.H"
#include "Util/Assert.H"
#include <math.h>
#include <fcntl.h>
#include <limits>
#include <string>
#include <stdio.h>
// #include <boost/random/normal_distribution.hpp>
// #include <boost/random/mersenne_twister.hpp>
// #include <boost/random/variate_generator.hpp>


void makeData(const std::vector<float>& dimMeanIn,const std::vector<float>& dimVarIn, const std::vector<float>& dimMeanOut,const std::vector<float>& dimVarOut, const uint sampleDim, std::vector<std::vector<float> >& data, std::vector<int>& labels);

int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager manager("Test Decision Tree");


  // Create log likelihood classifier and local binary patterns objects
  DecisionTree dt(3);

  if (manager.parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false)
    return 0;

  manager.start();
  uint nDim=4;
  std::vector<std::vector<float> > traindata(nDim);
  std::vector<int> trainlabels;
  std::vector<float> dimMeanIn(nDim), dimMeanOut(nDim), dimVarIn(nDim,1.0F), dimVarOut(nDim,1.0F);
  for(uint i=0;i<nDim;i++)
    {
      dimMeanIn[i] = nDim-i;
      dimMeanOut[i] = -(nDim-i);
    }
  makeData(dimMeanIn,dimVarIn,dimMeanOut,dimVarOut,50,traindata,trainlabels);
  // Train the classifier on the training set
  dt.train(traindata,trainlabels);
  dt.printTree();
  std::vector<int> trainResults = dt.predict(traindata);
  // Validate on training set
  int numCorrect=0;
  for(uint i=0;i<trainlabels.size();i++)
    {
      if(trainResults[i]==trainlabels[i]) numCorrect++;
      printf("Train Guess %d [Ground Truth %d]\n",trainResults[i],trainlabels[i]);
    }
  printf("Training Accuracy:[Correct/Total]=[%d/%Zu]:%f\n",numCorrect,trainlabels.size(),numCorrect/float(trainlabels.size()));

  std::vector<std::vector<float> > testdata(nDim);
  std::vector<int> testlabels;
  // Create new data from same distribution as test set
  makeData(dimMeanIn,dimVarIn,dimMeanOut,dimVarOut,10,testdata,testlabels);
  // Classify test set
  std::vector<int> testResults = dt.predict(testdata);
  numCorrect=0;
  for(uint i=0;i<testlabels.size();i++)
    {
      if(testResults[i]==testlabels[i]) numCorrect++;
      printf("Guess %d [Ground Truth %d]\n",testResults[i],testlabels[i]);
    }
  printf("Accuracy:[Correct/Total]=[%d/%Zu]:%f\n",numCorrect,testlabels.size(),numCorrect/float(testlabels.size()));
  manager.stop();

}

void makeData(const std::vector<float>& dimMeanIn,const std::vector<float>& dimVarIn, const std::vector<float>& dimMeanOut,const std::vector<float>& dimVarOut, const uint sampleDim, std::vector<std::vector<float> >& data, std::vector<int>& labels)
{
  // Create uniform random number generator
  rutz::urand rgen(time((time_t*)0)+getpid());

  // // Create mersenne twister generator, attached to a Normal Distribution
  // boost::mt19937 igen(time((time_t*)0)+getpid());
  // boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
  //   gen(igen, 
  //       boost::normal_distribution<>(0.0,1.0));
  // double randVar = gen();
  // Create data and labels
  const uint dataDim=(uint) dimMeanIn.size();
  //  const uint numCategories=2;
  ASSERT(data.size()==dataDim);
  for(uint i=0;i<sampleDim;i++)
    {
      int l=rgen.idraw(2)*2-1;
      printf("data[][%u]: l=%d; ",i,l);
      for(uint j=0;j<dataDim;j++)
	{
          if(l==1)
            data[j].push_back(rgen.fdraw_range(0.0,0.5));//*dimVarIn[j]+dimMeanIn[j]);
          else
            data[j].push_back(rgen.fdraw_range(-0.5,0.0));//*dimVarOut[j]+dimMeanOut[j]);
          printf("%f, ",data[j][i]);
	}      
      printf("\n");
      labels.push_back(l);
    }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */



