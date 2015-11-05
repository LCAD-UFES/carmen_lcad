/*!@file Learn/FuzzyART.C  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/FuzzyART.C $
// $Id: FuzzyART.C 13373 2010-05-09 04:28:40Z jshen $
//

#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Learn/FuzzyART.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include <vector>

const ModelOptionCateg MOC_Learn_ART = {
  MOC_SORTPRI_3,   "ART Model-related Options" };

const ModelOptionDef OPT_FuzzyART_InputSize = 
  { MODOPT_ARG(uint), "FuzzyART_InputSize", &MOC_Learn_ART, OPTEXP_CORE,
    "The size of the input vectors to ART",
    "fuzzy-art-inputsize", '\0', "<uint>", "2"};

const ModelOptionDef OPT_FuzzyART_NumCategories = 
  { MODOPT_ARG(uint), "FuzzyART_MaxNumCategories", &MOC_Learn_ART, OPTEXP_CORE,
    "The maximum number of categories fuzzy ART can learn",
    "fuzzy-art-max-categories", '\0', "<uint>", "10"};

const ModelOptionDef OPT_FuzzyART_ComplementCode = 
  { MODOPT_FLAG, "FuzzyART_ComplementCoding", &MOC_Learn_ART, OPTEXP_CORE,
    "Use complement coding for the fuzzy ART input layer",
    "fuzzy-art-complement-code", '\0', "", "true"};

const ModelOptionDef OPT_FuzzyART_Alpha = 
  { MODOPT_ARG(double), "FuzzyART_Alpha", &MOC_Learn_ART, OPTEXP_CORE,
    "Set the choice parameter alpha for fuzzy ART dynamics",
    "fuzzy-art-alpha", '\0', "<0..1>", "0.01"};

const ModelOptionDef OPT_FuzzyART_Beta = 
  { MODOPT_ARG(double), "FuzzyART_Beta", &MOC_Learn_ART, OPTEXP_CORE,
    "Set the learning rate parameter beta for fuzzy ART dynamics",
    "fuzzy-art-beta", '\0', "<0..1>", "0.1"};

const ModelOptionDef OPT_FuzzyART_Rho = 
  { MODOPT_ARG(double), "FuzzyART_Rho", &MOC_Learn_ART, OPTEXP_CORE,
    "Set the vigilance parameter rho for fuzzy ART dynamics",
    "fuzzy-art-rho", '\0', "<0..1>", "0.9"};

// ######################################################################
FuzzyART::FuzzyART(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsInputSize(&OPT_FuzzyART_InputSize, this), 
  itsNumCategories(&OPT_FuzzyART_NumCategories, this),
  itsComplementCoded(&OPT_FuzzyART_ComplementCode, this),
  itsAlpha(&OPT_FuzzyART_Alpha, this),
  itsBeta(&OPT_FuzzyART_Beta, this),
  itsRho(&OPT_FuzzyART_Rho, this)
{
}

void FuzzyART::start2()
{
  for(uint i = 0; i < itsNumCategories.getVal(); i++) { 
    
    Unit L;
    if(itsComplementCoded.getVal())
      L.weights.assign(2*itsInputSize.getVal(),1.0);
    else
      L.weights.assign(itsInputSize.getVal(),1.0);
    L.committed = false;
    itsF1.push_back(L);
  }
}

// ######################################################################
uint FuzzyART::learnInput(std::vector<double> input)
{
  ASSERT(input.size() == itsInputSize.getVal());

  if(itsComplementCoded.getVal()) 
    itsCurrInput = complementCode(input);
  else
    itsCurrInput = input;

  // set activations for all units
  for(uint i = 0; i < numUnitsCommitted(); i++) {
    itsF1[i].activity = choiceFunction(i);
  }
  bool resonant = false;
  uint max_active = 0;
  while(!resonant) {
    // find pattern of maximum activity
    for(uint i = 0; i < numUnitsCommitted(); i++)
      if(itsF1[i].activity > itsF1[max_active].activity) 
        max_active = i;

    if(itsF1[max_active].activity == -1 || numUnitsCommitted() == 0) {
      // all units are made inactive      
      max_active = numUnitsCommitted(); // create new node
      break;
    }
    resonant = vigilanceCrit(max_active);
    if(!resonant) itsF1[max_active].activity = -1; //inactivate this node
  }
  updateWeights(max_active);

  if(max_active == itsNumCategories.getVal()) return -1;
  return max_active;
}

// ######################################################################
std::vector<double> FuzzyART::complementCode(const std::vector<double> c) const
{
  std::vector<double> d = c;
  const uint N = c.size();
  for(uint i = 0; i < N; i++)
    d.push_back(1-d[i]);

  return d;
}

// ######################################################################
double FuzzyART::choiceFunction(const uint cat) const
{
  std::vector<double> ha = itsF1[cat].weights;
  double nom = norm(fuzzyAnd(itsCurrInput,itsF1[cat].weights));
  double denom = itsAlpha.getVal() + norm(itsF1[cat].weights);
  return nom/denom;
  //  return norm(fuzzyAnd(itsCurrInput,itsF1[cat].weights))/
  //    (itsAlpha.getVal() + norm(itsF1[cat].weights));
}

// ######################################################################
bool FuzzyART::vigilanceCrit(const uint cat) const
{
  return norm(fuzzyAnd(itsCurrInput,itsF1[cat].weights))/norm(itsCurrInput) >= 
    itsRho.getVal();
  // if true, resonance
  // if false, mismatch reset
}

// ######################################################################
void FuzzyART::updateWeights(const uint cat)
{
  if(cat >= itsNumCategories.getVal()) {
    LINFO("Max number of categories #%d hit: input unlearned", itsNumCategories.getVal());
    return;
  }

  if(!itsF1[cat].committed) { //fast learning mode
    itsF1[cat].weights = itsCurrInput;
    itsF1[cat].committed = true;
    LINFO("activating new node #%d", cat);
  }
  else {
    std::vector<double> fuz = fuzzyAnd(itsCurrInput,itsF1[cat].weights);
    for(uint i = 0; i < itsInputSize.getVal(); i++)
      itsF1[cat].weights[i] = itsBeta.getVal()*fuz[i] + (1 - itsBeta.getVal()) * itsF1[cat].weights[i];
  }
}

// ######################################################################
uint FuzzyART::numUnitsCommitted() const
{
  uint i;
  for(i = 0; i < itsNumCategories.getVal(); i++) if (!itsF1[i].committed) break;
  return i;
}

// ######################################################################
std::vector<double> FuzzyART::fuzzyAnd(const std::vector<double> A, const std::vector<double> B) const
{
  std::vector<double> ret(itsInputSize.getVal());
  for(uint i = 0; i < itsInputSize.getVal(); i++) ret[i] = (A[i] < B[i] ? A[i] : B[i]);
  return ret;
}

// ######################################################################
double FuzzyART::norm(const std::vector<double> A) const
{
  double sum = 0;
  for(uint i = 0; i < itsInputSize.getVal(); i++) sum += A[i];
  return sum;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

