/*!@file Learn/SVMClassifierModule.C Support Vector Machine Classifier module */

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
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SVMClassifierModule.C $
// $Id: SVMClassifierModule.C 13332 2010-04-28 18:50:09Z dparks $
//

#include "Learn/svm.h"
#include "SVMClassifierModule.H"
#include "Learn/SVMClassifier.H"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>

const ModelOptionCateg MOC_SVMClassifier = {
  MOC_SORTPRI_3,   "SVMClassifier Related Options" };

const ModelOptionDef OPT_SVMModelFileNames =
  { MODOPT_ARG_STRING, "SVM Model File Names", &MOC_SVMClassifier, OPTEXP_CORE,
    "Colon separated list of filenames for the SVM model",
    "svm-model-filenames", '\0', "<filename1:filename2>", "" };

const ModelOptionDef OPT_SVMModelNames =
  { MODOPT_ARG_STRING, "SVM Model Names", &MOC_SVMClassifier, OPTEXP_CORE,
    "Colon separated list of names for the SVM",
    "svm-model-names", '\0', "<name1:name2>", "" };

const ModelOptionDef OPT_SVMRangeFileNames =
  { MODOPT_ARG_STRING, "SVM Range File Names", &MOC_SVMClassifier, OPTEXP_CORE,
    "Colon separated list of filenames for the SVM range",
    "svm-range-filenames", '\0', "<filename1:filename2>", "" };

const ModelOptionDef OPT_SVMOutputFileNames =
  { MODOPT_ARG_STRING, "SVM Training Output File Names", &MOC_SVMClassifier, OPTEXP_CORE,
    "Filename(s) for the SVM training to output to",
    "svm-output-filenames", '\0', "<filename1:filename2>", "" };

const ModelOptionDef OPT_SVMObjDBFileName =
  { MODOPT_ARG_STRING, "Object DB File Name", &MOC_SVMClassifier, OPTEXP_CORE,
    "Filename for the object database file",
    "svm-objdb-filename", '\0', "<filename>", "" };

const ModelOptionDef OPT_SVMTrainObjName =
  { MODOPT_ARG_STRING, "SVM Training Object Name", &MOC_SVMClassifier, OPTEXP_CORE,
    "Name of the object used in training",
    "svm-train-objname", '\0', "<name>", "" };

const ModelOptionDef OPT_SVMTrainObjId =
  { MODOPT_ARG(int), "SVM Training Object Id", &MOC_SVMClassifier, OPTEXP_CORE,
    "Id of the object used in training",
    "svm-train-objid", '\0', "<id>", "-1" };

const ModelOptionDef OPT_SVMMode =
  { MODOPT_ARG_STRING, "SVM Mode", &MOC_SVMClassifier, OPTEXP_CORE,
    "The mode of SVM Classifier. Train|Rec",
    "svm-mode", '\0', "<Train|Rec>", "Rec" };



SVMClassifierModule::SVMClassifierModule(OptionManager& mgr,
			     const std::string& descrName,
			     const std::string& tagName) :
  SimModule(mgr, descrName, tagName),
  itsSVMModelFileNamesStr(&OPT_SVMModelFileNames, this),
  itsSVMModelNamesStr(&OPT_SVMModelNames, this),
  itsSVMRangeFileNamesStr(&OPT_SVMRangeFileNames, this),
  itsSVMOutputFileNamesStr(&OPT_SVMOutputFileNames, this),
  itsSVMObjDBFileName(&OPT_SVMObjDBFileName, this),
  itsSVMTrainObjName(&OPT_SVMTrainObjName, this),
  itsSVMTrainObjId(&OPT_SVMTrainObjId, this),
  itsSVMMode(&OPT_SVMMode, this)
{
}

SVMClassifierModule::~SVMClassifierModule()
{
}



void SVMClassifierModule::start2()
{
  SimModule::start2();
  
  // Parse which model files should be loaded
  split(itsSVMModelFileNamesStr.getVal(), ":", std::back_inserter(itsSVMModelFiles));

  // Parse which model names should be loaded
  split(itsSVMModelNamesStr.getVal(), ":", std::back_inserter(itsSVMModelNames));

  // Parse which ranges should be loaded
  split(itsSVMRangeFileNamesStr.getVal(), ":", std::back_inserter(itsSVMRangeFiles));

  // Parse which output files
  split(itsSVMOutputFileNamesStr.getVal(), ":", std::back_inserter(itsSVMOutputFiles));

  // If no range files are specified, resize the list to be the same as the model file list
  if(itsSVMRangeFiles.size()==0)
    {
      itsSVMRangeFiles = std::vector<std::string>(itsSVMModelFiles.size());
    }
  if(itsSVMRangeFiles.size()!= itsSVMModelFiles.size())
    LFATAL("If range files are specified, must be same number as model files");

  if(itsSVMObjDBFileName.getVal().compare("") == 0) {
    LFATAL("Must specify object db file using --svm-objdb-filename");
  }
  itsObjDB.loadObjDB(itsSVMObjDBFileName.getVal());

  if (itsSVMMode.getVal().compare("Train") == 0)          // training
  {
    if(itsSVMOutputFiles.size() == 0 || itsSVMOutputFiles.size() != itsSVMModelNames.size()) {
      LFATAL("Must specify training output file(s) (and equal number of model names) if in training mode");
    }
    // Load basic classifier for training
    SVMClassifier classifier;
    itsClassifiers.push_back(classifier);
  }
  else if (itsSVMMode.getVal().compare("Rec") == 0)      // Recognition
  {
    if(itsSVMModelFiles.size() == 0 || itsSVMModelFiles.size() != itsSVMModelNames.size()){
      LFATAL("Must specify svm model file(s) (and equal number of model names) if in recognition mode");
    }
    for(size_t c=0;c<itsSVMModelFiles.size();c++)
      {
	SVMClassifier classifier;
	// Load model file
	classifier.readModel(itsSVMModelFiles[c]);
	// Load the range file
	if(itsSVMRangeFiles[c].compare("") != 0) {
	  classifier.readRange(itsSVMRangeFiles[c]);
	}
	itsClassifiers.push_back(classifier);
      }
  }
  else
    LFATAL("Unknown SVM Mode type %s", itsSVMMode.getVal().c_str());
}

// ######################################################################
void SVMClassifierModule::stop1()
{
  LINFO("Writing out object db in module %p",this);
  itsObjDB.writeObjDB(itsSVMObjDBFileName.getVal());
  SimModule::stop1();
}

std::string SVMClassifierModule::getMode()
{
  return itsSVMMode.getVal();
}

void SVMClassifierModule::attentionShift(SimEventQueue& q,
					const Point2D<int>& location)
{
}


std::vector<std::string> SVMClassifierModule::getModelNames()
{
  return itsSVMModelNames;
}

SVMObject SVMClassifierModule::determineLabel(std::vector<float> featureVector, int id, std::string name, int classifierId)
{
 if (itsSVMMode.getVal().compare("Rec") == 0)      // Recognition
   {
     return recognizeLabel(featureVector,id,name,classifierId);
   }
 else if (itsSVMMode.getVal().compare("Train") == 0)      // Train
   {
     return trainLabel(featureVector,id,name,classifierId);
   }
 else
   {
     LFATAL("Invalid SVM Classification Mode");
   }
 return SVMObject();
}

SVMObject SVMClassifierModule::trainLabel(std::vector<float> featureVector, int id, std::string name, int classifierId)
{
  SVMObject so;
  printf("In SVMClassifierModule::determineLabel %s\n",itsSVMMode.getVal().c_str());
  // Preprocess the id and name if we are in training mode
  // If the id or name for these images is given, assign it
  if(itsSVMTrainObjId.getVal() != -1) {
    id = itsSVMTrainObjId.getVal();
  }
  if(itsSVMTrainObjName.getVal().compare("") != 0) {
    name = itsSVMTrainObjName.getVal();
  }
  if(name.compare("") == 0) {
    // If the name is still not defined, prompt the user
    // LINFO("Enter name for new object or [RETURN] to skip training:");
    // std::getline(std::cin, name, '\n');
    LFATAL("Name is not specified while in Train mode");
  }
  // If the id is still not defined, try to pull the id out
  if(id == -1 && itsObjDB.getObject(name).initialized()) {
    so = itsObjDB.getObject(name);
    id = so.id;
  }
  // Make sure the id is in the database.  Check for mismatch, and add it, if not present
  LINFO("Training on object %s[%d]\n",name.c_str(),id);
  so = itsObjDB.updateObject(id,name);
  so.confidence = 1.0; 
  so.id = id;
  so.name = name;
  itsClassifiers[classifierId].train(itsSVMOutputFiles[classifierId],id,featureVector); 

  return so;

}

SVMObject SVMClassifierModule::recognizeLabel(std::vector<float> featureVector, int id, std::string name, int classifierId)
{
  std::vector<SVMObject> objects = getLabelPDF(featureVector,id,name,classifierId);
  return getBestLabel(objects);
}

SVMObject SVMClassifierModule::getBestLabel(const std::vector<SVMObject> &objects)
{
 double maxProb=-1;
  int bestLabelIdx=0;
  if(objects.size() == 0)
    return SVMObject();
  for(size_t i=0;i<objects.size();i++)
    {
      if(maxProb<objects[i].confidence)
	{
	  bestLabelIdx=i;
	  maxProb=objects[i].confidence;
	}
    }
  return objects[bestLabelIdx];
}

std::vector<SVMObject> SVMClassifierModule::getLabelPDF(std::vector<float> featureVector, int id, std::string name, int classifierId)
{
  std::map<int,double> pdf = itsClassifiers[classifierId].predictPDF(featureVector);

 if (itsSVMMode.getVal().compare("Rec") == 0)      // Recognition
   {
     std::vector<SVMObject> svmObjects;
     for(std::map<int,double>::iterator pdfIt=pdf.begin(); pdfIt!=pdf.end(); ++pdfIt)
       {
	 SVMObject obj = itsObjDB.getObject(int(pdfIt->first));
	 if (obj.id == -1)  //Assign the object ID that we have, if we did not find it the DB
	   obj.id = pdfIt->first;
	 obj.confidence = pdfIt->second;
	 svmObjects.push_back(obj);
       }
     return svmObjects;
     
   }
 else if (itsSVMMode.getVal().compare("Train") == 0)      // Train
   {
     std::vector<SVMObject> svmObjects;
     for(std::map<int,double>::iterator pdfIt=pdf.begin(); pdfIt!=pdf.end(); ++pdfIt)
       {
	 SVMObject obj = itsObjDB.getObject(int(pdfIt->first));
	 if(obj.id==id)
	   obj.confidence = 1;
	 else
	   obj.confidence = 0;
	 svmObjects.push_back(obj);
       }
     return svmObjects;
   }
 else
   {
     LFATAL("Invalid SVM Classification Mode");
   }

 return std::vector<SVMObject>(); 
}

