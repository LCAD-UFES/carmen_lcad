/*!@file Learn/GentleBoostComponent.C Gentle BOOST Classifier component */

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
// Primary maintainer for this file: Daniel Parks <danielfp@usc.edu>
// $HeadURL$
// $Id$
//

#include "Learn/GentleBoost.H"
#include "GentleBoostComponent.H"
#include "Util/StringUtil.H"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <limits>

const ModelOptionCateg MOC_GentleBoost = {
  MOC_SORTPRI_3,   "GentleBoost Related Options" };

const ModelOptionDef OPT_GBModelInputFileNames =
{ MODOPT_ARG_STRING, "GB Model Input File Names", &MOC_GentleBoost, OPTEXP_CORE,
  "Colon separated list of input filenames to load GB models",
  "gb-model-inputfiles", '\0', "<filename1:filename2>", "" };

const ModelOptionDef OPT_GBModelOutputFileNames =
{ MODOPT_ARG_STRING, "GB Model Output File Names", &MOC_GentleBoost, OPTEXP_CORE,
  "Colon separated list of output filenames to write GB models",
  "gb-model-outputfiles", '\0', "<filename1:filename2>", "" };

const ModelOptionDef OPT_GBModelNames =
{ MODOPT_ARG_STRING, "GB Model Names", &MOC_GentleBoost, OPTEXP_CORE,
  "Colon separated list of model names for the GB",
  "gb-model-names", '\0', "<name1:name2>", "" };

const ModelOptionDef OPT_GBMode =
{ MODOPT_ARG_STRING, "GB Mode", &MOC_GentleBoost, OPTEXP_CORE,
  "The mode of GB Classifier. Train|Rec",
  "gb-mode", '\0', "<Train|Rec>", "Rec" };

const ModelOptionDef OPT_GBMaxIters =
{ MODOPT_ARG(int), "GB Max Iters", &MOC_GentleBoost, OPTEXP_CORE,
  "Maximum number of iterations to train boosted classifier",
  "gb-maxiters", '\0', "<Train|Rec>", "10" };



GentleBoostComponent::GentleBoostComponent(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName) :
    ModelComponent(mgr, descrName, tagName),
    itsGBModelInputFileNamesStr(&OPT_GBModelInputFileNames, this),
    itsGBModelOutputFileNamesStr(&OPT_GBModelOutputFileNames, this),
    itsGBModelNamesStr(&OPT_GBModelNames, this),
    itsGBMode(&OPT_GBMode, this),
    itsMaxIters(&OPT_GBMaxIters, this),
    itsLoadComplete(false)
{
}

GentleBoostComponent::~GentleBoostComponent()
{
}



void GentleBoostComponent::start2()
{
  ModelComponent::start2();
  
  // Parse which model input files should be loaded
  split(itsGBModelInputFileNamesStr.getVal(), ":", std::back_inserter(itsGBModelInputFiles));

  // Parse which model output files should be written
  split(itsGBModelOutputFileNamesStr.getVal(), ":", std::back_inserter(itsGBModelOutputFiles));

  // Parse which model names should be loaded
  split(itsGBModelNamesStr.getVal(), ":", std::back_inserter(itsGBModelNames));

  ASSERT(itsMaxIters.getVal() > 0);

  if (itsGBMode.getVal().compare("Train") == 0)          // training
  {
    if(itsGBModelOutputFiles.size() == 0 || itsGBModelOutputFiles.size() != itsGBModelNames.size()) {
      LFATAL("Must specify gb model output file(s) (and equal number of model names) if in training mode");
    }
    // Load basic classifier for training
    GentleBoost classifier;
    itsClassifiers.push_back(classifier);
    itsClassifierLabels.push_back(std::vector<int>());
    itsClassifierVectors.push_back(std::vector<std::vector<float> >());
  }
  else if (itsGBMode.getVal().compare("Rec") == 0)      // Recognition
  {
    if(itsGBModelInputFiles.size() == 0 || itsGBModelInputFiles.size() != itsGBModelNames.size()){
      LFATAL("Must specify gb model input file(s) (and equal number of model names) if in recognition mode");
    }
    for(size_t c=0;c<itsGBModelInputFiles.size();c++)
    {
      GentleBoost classifier;
      // Load model file
      classifier.load(itsGBModelInputFiles[c]);
      itsClassifiers.push_back(classifier);
      itsClassifierLabels.push_back(std::vector<int>());
      itsClassifierVectors.push_back(std::vector<std::vector<float> >());
    }
  }
  else
    LFATAL("Unknown GB Mode type %s", itsGBMode.getVal().c_str());
  itsLoadComplete = true;
}

// ######################################################################
void GentleBoostComponent::stop1()
{
  ModelComponent::stop1();
  if(itsLoadComplete)
  {
    train(itsMaxIters.getVal());
    save();
  }
}

std::string GentleBoostComponent::getMode()
{
  return itsGBMode.getVal();
}


std::vector<std::string> GentleBoostComponent::getModelNames()
{
  return itsGBModelNames;
}

int GentleBoostComponent::getMostLikelyClass(const std::map<int,float>& pdf)
{
  float maxClassVal=-std::numeric_limits<float>::max();
  int maxClassIdx=-1;
  std::map<int,float>::const_iterator pitr;
  for(pitr=pdf.begin();pitr!=pdf.end();pitr++)
  {
    if(maxClassVal < pitr->second)
	{
	  maxClassVal=pitr->second;
	  maxClassIdx=pitr->first;
	}
  }
  ASSERT(maxClassIdx>=0);
  return maxClassIdx;
}


// Add training examplar
void GentleBoostComponent::addTrainVector(std::vector<float> featureVector, int id, int classifierId)
{
  itsClassifierVectors[classifierId].push_back(featureVector);
  itsClassifierLabels[classifierId].push_back(id);
}

// Train the classifier on the final data
void  GentleBoostComponent::train(int classifierId)
{
  // NOTE: Training vectors must be transposed here
  // If not specified, train all classifiers
  if(classifierId == -1)
  {
    for(size_t c=0;c<itsClassifiers.size();c++)
    {
      if(itsClassifierLabels[c].size() > 0)
      {
        itsClassifiers[c].train(itsClassifiers[c].transpose(itsClassifierVectors[c]),itsClassifierLabels[c],itsMaxIters.getVal());
      }
    }
  }
  else
  {
    if(itsClassifierLabels[classifierId].size() > 0)
      itsClassifiers[classifierId].train(itsClassifiers[classifierId].transpose(itsClassifierVectors[classifierId]),itsClassifierLabels[classifierId],itsMaxIters.getVal());
  }
}

// Save the classifier data
void  GentleBoostComponent::save(int classifierId)
{
  // If not specified, train all classifiers
  if(classifierId == -1)
  {
    for(size_t c=0;c<itsClassifiers.size();c++)
    {
      itsClassifiers[c].save(itsGBModelOutputFiles[c]);
    }
  }
  else
  {
    itsClassifiers[classifierId].save(itsGBModelOutputFiles[classifierId]);
  }
}


int GentleBoostComponent::predict(std::vector<float> featureVector, int classifierId)
{
  std::vector<std::vector<float> > convVec(1);
  convVec[0] = featureVector;
  // Transpose vector from 1xn to nx1
  convVec = itsClassifiers[classifierId].transpose(convVec);
  std::vector<int> tmpVec;
  tmpVec = itsClassifiers[classifierId].predict(convVec);
  return tmpVec[0];
}

std::map<int, float> GentleBoostComponent::predictPDF(std::vector<float> featureVector, int classifierId)
{
  std::map<int, float> out;
  std::vector<std::vector<float> > convVec(1);
  convVec[0] = featureVector;
  // Transpose vector from 1xn to nx1
  convVec = itsClassifiers[classifierId].transpose(convVec);
  std::map<int, std::vector<float> > tmpMap;
  tmpMap = itsClassifiers[classifierId].predictPDF(convVec);
  std::map<int,std::vector<float> >::iterator litr;
  for(litr=tmpMap.begin();litr!=tmpMap.end();litr++)
  {
    out[litr->first] = litr->second[0];
  }
  return out;
}


