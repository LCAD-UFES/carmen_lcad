/*!@file Neuro/GentleBoostComponent.H Object recognition component */

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
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GentleBoostComponent.H $
// $Id: GentleBoost.H 10747 2009-02-03 08:24:04Z itti $
//

#ifndef GENTLEBOOSTCOMPONENT_H_DEFINED
#define GENTLEBOOSTCOMPONENT_H_DEFINED

#include "Learn/GentleBoost.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"

#include "Image/Pixels.H"
#include "Image/Image.H"

namespace nub { template <class T> class ref; }

// ######################################################################
//! Double inheritence from SimComponent GentleBoost
class GentleBoostComponent : public ModelComponent
{
  public:
    //! Constructor
    GentleBoostComponent(OptionManager& mgr,
                      const std::string& descrName="GentleBoostComponent",
                      const std::string& tagName="");

    //! Destructor
    virtual ~GentleBoostComponent();


    // Train model
    void train(int classifierId=-1);
    void save(int classifierId=-1);
    void addTrainVector(std::vector<float> featureVector, int id, int classifierId=0);

    // Given a feature vector, determine the most likely ID
    int predict(std::vector<float> featureVector, int classifierId=0);
    // Given a feature vector, determine the PDF over classes
    std::map<int,float> predictPDF(std::vector<float> featureVector, int classifierId=0);
    // Get the most likely class out of the returned pdf
    int getMostLikelyClass(const std::map<int,float>& pdf);
    std::string getMode();
    std::vector<std::string> getModelNames();

  protected:
    //! (re-)initialization - load object database
    virtual void start2();

    //! destruction - save object database
    virtual void stop1();

    OModelParam<std::string> itsGBModelInputFileNamesStr;
    OModelParam<std::string> itsGBModelOutputFileNamesStr;
    OModelParam<std::string> itsGBModelNamesStr;
    OModelParam<std::string> itsGBMode;
    OModelParam<int> itsMaxIters;

  private:
    bool itsLoadComplete;
    std::vector<std::string> itsGBModelInputFiles;
    std::vector<std::string> itsGBModelOutputFiles;
    std::vector<std::string> itsGBModelNames;
    
    std::vector<GentleBoost> itsClassifiers;
    std::vector<std::vector<std::vector<float> > > itsClassifierVectors;
    std::vector<std::vector<int> > itsClassifierLabels;
};


#endif
