/*!@file Learn/FuzzyART.H  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/FuzzyART.H $
// $Id: FuzzyART.H 13373 2010-05-09 04:28:40Z jshen $
//

#ifndef FuzzyART_H_DEFINED
#define FuzzyART_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"

class FuzzyART : public ModelComponent
{
public:
  FuzzyART(OptionManager& mgr, 
           const std::string& descrName = "Fuzzy ART Learning Module", 
           const std::string& tagName="FuzzyART");

  void start2();
  struct Unit {
    std::vector<double> weights;
    double activity;
    bool committed; 
    //    Unit(uint siz, bool comp) : committed(false) {
    //if(!comp) weights.assign(1,siz);
    //else weights.assign(1,siz*2);
    //}
  };

  typedef std::vector<Unit> Layer;

  // returns -1 if max number of clusters is hit
  uint learnInput(std::vector<double> input);

  uint getInputSize() const {return itsInputSize.getVal();}

  uint getMaxCategories() const {return itsNumCategories.getVal();}


private:
  std::vector<double> complementCode(const std::vector<double> c) const;
  
  double choiceFunction(const uint cat) const; //choice function T_j
  bool vigilanceCrit(const uint cat) const; //vigilance criterion;
  void updateWeights(const uint cat);
  uint numUnitsCommitted() const;

  // utility math functions
  std::vector<double> fuzzyAnd(const std::vector<double> A, const std::vector<double> B) const;
  double norm(const std::vector<double> A) const;

  Layer itsF1;
  std::vector<double> itsCurrInput;

  OModelParam<uint> itsInputSize;
  OModelParam<uint> itsNumCategories;
  OModelParam<bool> itsComplementCoded;  
  OModelParam<double> itsAlpha; //choice parameter
  OModelParam<double> itsBeta; //learning rate parameter
  OModelParam<double> itsRho; //vigilance parameter
  
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_FUZZYART_H_DEFINED
