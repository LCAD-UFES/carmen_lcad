/*!@file Learn/test-FuzzyART.C  */

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
// Primary maintainer for this file: John Shen <shenjohn@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/test-FuzzyART.C $
// $Id: test-FuzzyART.C 13373 2010-05-09 04:28:40Z lior $
//

#include "Component/ModelComponent.H"
#include "Component/ModelManager.H"
#include "GUI/DebugWin.H"
#include "Image/ColorMap.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Learn/FuzzyART.H"
#include "Util/MathFunctions.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "nub/ref.h"
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
struct NormalDataCluster // gives normally distributed data
{
  std::vector<double> mu, sigma;

  PixRGB<byte> diagColor;

  std::vector<double> generateData() {
    std::vector<double> ret;    
    for(uint i = 0; i < mu.size(); i++) 
      ret.push_back(mu[i]+randomDoubleFromNormal(sigma[i]));
    
    return ret;
  }
};

//returns a vector with values uniformly distributed between the two 
std::vector<double> randomVectorIn(const std::vector<double> min, const std::vector<double> max)
{
  ASSERT(min.size() == max.size());
  std::vector<double> ret;
  for(uint i = 0; i < min.size(); i++) 
    ret.push_back(min[i] + randomDouble()*(max[i]-min[i]));

  return ret;
}

void defaultTest(nub::soft_ref<FuzzyART> FuzzyLearner);

std::string toString(const std::vector<double>& c);

PixRGB<byte> randColor();

int submain(const int argc, char** argv)
{
  MYLOGVERB = LOG_INFO;

  //initRandomNumbers();
  ModelManager mgr("Test Fuzzy ART");
  nub::soft_ref<FuzzyART> FuzzyARTModule(new FuzzyART(mgr));
  mgr.addSubComponent(FuzzyARTModule);
    if (mgr.parseCommandLine(
                              (const int)argc, (const char**)argv, "<input file> <output file>", 0,2) == false)
return 1;

  mgr.start();
  
  if(mgr.numExtraArgs() == 0) {
    defaultTest(FuzzyARTModule);
  }
  else {
    // input file has one pattern per line, scaled from 0 to 1, space separated
    //    const uint NFeatures = FuzzyARTModule->getMaxCategories();
    const uint NDims = FuzzyARTModule->getInputSize();

    std::string fn_in = mgr.getExtraArg(0);
    std::string fn_out = mgr.getExtraArg(1);

    std::ifstream fin(fn_in.c_str());
    if (!fin.is_open()) PLFATAL("Cannot open '%s'", fn_in.c_str());

    std::ofstream fout(fn_out.c_str());
    if (!fout.is_open()) PLFATAL("Cannot open '%s'", fn_out.c_str());

    std::string line; int linenum = -1;

    while(getline(fin, line)) {
      ++linenum;
      
      std::stringstream ss(line);

      std::vector<double> input_vec;
      for(uint i = 0; i < NDims; i++) {
	double foo;
    	ss >> foo;
	input_vec.push_back(foo);
      }
      uint result;
      result = FuzzyARTModule->learnInput(input_vec);
      //      LINFO("%d",FuzzyARTModule->numUnitsCommitted());
      fout << result << "\n";
      LINFO("input %02d to unit %02d", linenum, result);
    }

    fin.close();
    fout.close();
  }
  mgr.stop();

  return 0;
}

extern "C" int main(const int argc, char** argv)
{
  try
    {
      return submain(argc, argv);
    }
  catch (...)
    {
      REPORT_CURRENT_EXCEPTION;
    }

  return 1;
}

// ######################################################################
// A default test, using normally distributed clusters and displaying 2D
// clustering results
void defaultTest(nub::soft_ref<FuzzyART> FuzzyLearner) 
{
  const uint NFeatures = FuzzyLearner->getMaxCategories();
  const uint NDims = FuzzyLearner->getInputSize();

  const Dims dim(600,600);
  const PixRGB<byte> white(255,255,255);
  Image<PixRGB<byte> > grnd_truth(dim, ZEROS), 
    test_result(dim,ZEROS);
  grnd_truth+=white;
  test_result+=white;
  const ColorMap cm = ColorMap::JET(NFeatures);
  
  // setup: initialize data clusters
  std::vector<NormalDataCluster> dataClusters;
  for(uint i = 0; i < NFeatures; i++) {
    NormalDataCluster d;
    d.mu = randomVectorIn(std::vector<double>(NDims, 0.2), std::vector<double>(NDims, 0.8));
    d.sigma = randomVectorIn(std::vector<double>(NDims, 0.01), std::vector<double>(NDims, 0.05));
    d.diagColor = cm[i];
    dataClusters.push_back(d);

    // draw an ellipse for each cluster
    Point2D<int> p(d.mu[0]*dim.w(),d.mu[1]*dim.h()); 

    int rx = d.sigma[0]*dim.w();
    int ry = d.sigma[1]*dim.h();
    drawEllipse(grnd_truth, p, rx, ry, PixRGB<byte>(d.diagColor*0.8));
    drawCross(grnd_truth, p, d.diagColor);
  }

  
  // sample data clusters, one at a time
  uint cluster;
  for(uint i = 0; i < NFeatures; i++) {
    LINFO("cluster #%d: mu (%s) sigma (%s)",i,toString(dataClusters[i].mu).c_str(),
	  toString(dataClusters[i].sigma).c_str());
    for(uint j = 0; j < 10; j++) {
      std::vector<double> sample = dataClusters[i].generateData();
      Point2D<int> p(sample[0]*dim.w(),sample[1]*dim.h()); 
      drawDisk(grnd_truth,p,2,dataClusters[i].diagColor);

      cluster = FuzzyLearner->learnInput(sample);
      
      if(cluster != uint(-1))
	drawDisk(test_result,p,2,cm[cluster]);
      else
	drawCross(test_result, p, PixRGB<byte>(0,0,0));

      //      LINFO("generate pt (%s) into cluster %u", toString(sample).c_str(),cluster);

      
    }
  }
  Image< PixRGB<byte> > disp = concatX(grnd_truth, test_result);
  drawLine(disp, Point2D<int>(dim.w()-1,0), Point2D<int>(dim.w()-1,dim.h()-1),
	   PixRGB<byte>(0,0,255),1);
  LINFO("left: ground truth, right: ART clusters.  press q to close window.");
  SHOWIMG(disp);
}

// ######################################################################
// Utility functions
std::string toString(const std::vector<double>& c)
{ 
  std::string ret = "";
  for(uint i = 0; i < c.size(); i++) 
    ret += convertToString(c[i]) + ",";

  ret.erase(ret.end()-1);
  return ret;
}

// ######################################################################
PixRGB<byte> randColor()
{
  return PixRGB<byte>(randomUpToIncluding(255),
		      randomUpToIncluding(255),
		      randomUpToIncluding(255));
}

