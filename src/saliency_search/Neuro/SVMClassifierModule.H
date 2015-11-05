/*!@file Neuro/SVMClassifier.H Object recognition module */

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
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SVMClassifierModule.H $
// $Id: SVMClassifier.H 10747 2009-02-03 08:24:04Z itti $
//

#ifndef SVMCLASSIFIERMODULE_H_DEFINED
#define SVMCLASSIFIERMODULE_H_DEFINED

#include "Learn/SVMClassifier.H"
#include "Learn/SVMObjectDB.H"
#include "Learn/SVMObject.H"
#include "Channels/InputFrame.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"

#include "Image/Pixels.H"
#include "Image/Image.H"
#include "Neuro/NeuroSimEvents.H"
#include "SIFT/VisualObjectMatchAlgo.H"
#include "Simulation/SimModule.H"

namespace nub { template <class T> class ref; }

// ######################################################################
//! Double inheritence from SimModule SVMClassifier
class SVMClassifierModule : public SimModule
{
public:
  //! Constructor
  SVMClassifierModule(OptionManager& mgr,
                 const std::string& descrName,
                 const std::string& tagName);

  //! Destructor
  virtual ~SVMClassifierModule();

  // Given a feature vector, determine the object
  SVMObject determineLabel(std::vector<float> featureVector, int id, std::string name, int classifierId=0);
  SVMObject getBestLabel(const std::vector<SVMObject> &objects);
  std::vector<SVMObject> getLabelPDF(std::vector<float> featureVector, int id, std::string name, int classifierId=0);
  std::string getMode();
  std::vector<std::string> getModelNames();

protected:
  SVMObject recognizeLabel(std::vector<float> featureVector, int id, std::string name, int classifierId=0);
  SVMObject trainLabel(std::vector<float> featureVector, int id, std::string name, int classifierId=0);
  //! (re-)initialization - load object database
  virtual void start2();

  //! destruction - save object database
  virtual void stop1();

  //! Object recognition
  virtual void attentionShift(SimEventQueue& q, const Point2D<int>& location);

  OModelParam<std::string> itsSVMModelFileNamesStr;
  OModelParam<std::string> itsSVMModelNamesStr;
  OModelParam<std::string> itsSVMRangeFileNamesStr;
  OModelParam<std::string> itsSVMOutputFileNamesStr;
  OModelParam<std::string> itsSVMObjDBFileName;
  OModelParam<std::string> itsSVMTrainObjName;
  OModelParam<int> itsSVMTrainObjId;
  OModelParam<std::string> itsSVMMode;

private:
  std::vector<std::string> itsSVMModelFiles;
  std::vector<std::string> itsSVMModelNames;
  std::vector<std::string> itsSVMRangeFiles;
  std::vector<std::string> itsSVMOutputFiles;
  std::vector<SVMClassifier> itsClassifiers;
  SVMObjectDB itsObjDB;
};


#endif
