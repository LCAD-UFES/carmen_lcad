/*!@file HMAX/test-GBC.C Test Gentle Boost Component */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/test-hmax.C $
// $Id: test-hmax.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "Component/ModelManager.H"
#include "Learn/GentleBoostComponent.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Kernels.H"   // for dogFilterHmax()
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Raster/Raster.H"
#include "Util/log.H"
#include "rutz/rand.h"

#include <cmath>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Run this program:
// ./bin/test-GBC --gb-model-names="test" --gb-model-outputfiles="test.dat" --gb-mode="Train"


void makeData(const int numCategories, const uint sampleDim, const uint dataDim, std::vector<std::vector<float> >& data, std::vector<int>& labels, bool printData);

//! Debug Main
int main(int argc, char **argv)
{
  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test GentleBoost Component");

  nub::ref<GentleBoostComponent> gbc(new GentleBoostComponent(*mgr));
  mgr->addSubComponent(gbc);

  mgr->exportOptions(MC_RECURSE);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false)
    return 1;


  mgr->start();

  // 
  // Create log likelihood classifier and local binary patterns objects
  uint nDim=4;
  int numCategories=3;
  int numSamples=100;

  std::vector<std::vector<float> > traindata;
  std::vector<int> trainlabels;

  makeData(numCategories,numSamples,nDim,traindata,trainlabels,false);
  // Train the classifier on the training set
  for(int i=0;i<numSamples;i++)
    gbc->addTrainVector(traindata[i],trainlabels[i]);

  // Train the classifiers
  gbc->train();

  // Validate on training set
  int numCorrect=0;
  for(int i=0;i<numSamples;i++)
  {
    int predId = gbc->predict(traindata[i]);
    if(trainlabels[i]==predId) numCorrect++;
  }
  printf("Training Accuracy:[Correct/Total]=[%d/%Zu]:%f\n",numCorrect,trainlabels.size(),numCorrect/float(trainlabels.size()));

  std::vector<std::vector<float> > testdata;
  std::vector<int> testlabels;

  makeData(numCategories,numSamples,nDim,testdata,testlabels,false);
  // Test the classifier on the testing set
  numCorrect = 0;
  for(int i=0;i<numSamples;i++)
  {
    std::map<int,float> testPDF = gbc->predictPDF(testdata[i]);
    int predId = gbc->getMostLikelyClass(testPDF);
    if(testlabels[i]==predId) numCorrect++;
    printf("Guess %d [",predId);
    std::map<int,float>::iterator litr;
    for(litr=testPDF.begin();litr!=testPDF.end();litr++)
    {
      printf("(%d)%f, ",litr->first,litr->second);
    }
    printf("] *** Ground Truth %d\n",testlabels[i]);
  }
  printf("Accuracy:[Correct/Total]=[%d/%Zu]:%f\n",numCorrect,testlabels.size(),numCorrect/float(testlabels.size()));

  mgr->stop();


}//end of main


void makeData(const int numCategories, const uint sampleDim, const uint dataDim, std::vector<std::vector<float> >& data, std::vector<int>& labels, bool printData)
{
  // Create uniform random number generator
  rutz::urand rgen(time((time_t*)0)+getpid());
  // Create data and labels
  data = std::vector<std::vector<float> >(sampleDim);

  for(uint i=0;i<data.size();i++)
  {
    int l=rgen.idraw(numCategories)+1;
    if(printData) printf("data[][%u]: l=%d; ",i,l);
    for(uint j=0;j<dataDim;j++)
	{
	  data[i].push_back(rgen.fdraw_range(l-0.75,l+0.75));//*dimVarIn[j]+dimMeanIn[j]);
      if(printData) printf("%f, ",data[i][j]);
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
