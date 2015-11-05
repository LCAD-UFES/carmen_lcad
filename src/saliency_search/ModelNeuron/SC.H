/*!@file ModelNeuron/SC.H a model of the mamillian SC. */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/ModelNeuron/SC.H $

#ifndef  NEURALMODEL_SC_H_DEFINED
#define  NEURALMODEL_SC_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

/*
#include "ModelNeuron/CompLayer.H"

// ######################################################################
//  SC base interface
//  ######################################################################
struct SCInterface : public Structure<SimStructure*>
{
  SCInterface(const SimTime& timestep, const uint width, const uint height, 
              const std::string name = "", const std::string units = "") 
    : Structure<SimStructure*>(timestep, width, height, name, units) { }
  
  virtual ~SCInterface() { }
  
  //set bottom-up input
  virtual void input_sgs(const Image<double>& inp) = 0;
  
  //set top-down input
  virtual void input_sgi(const Image<double>& inp) = 0;
};
  
// ######################################################################
//  SC model
//  ######################################################################
class SC : public SCInterface
{
public:
  //! constructor
  SC(const double& SCs_SCi, const double& SCi_PreM, const double& PreM_SCi, const double& SCi_SCs,
     const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height, 
     const std::string name = "", const std::string units = "")
    : SCInterface(timestep, width, height, name, units), 
      itsS_I(SCs_SCi), itsI_M(SCi_PreM), itsI_S(SCi_SCs), itsM_I(PreM_SCi)
  { 
    //derived classes should configure the SCs and SCi layers and set the motor unit with setLayers
  }
  
  //default copy and assignment OK
  
  //!destructor 
  virtual ~SC() { };  
  
  template<class EUnit, class IUnit, class MUnit>
  void setLayers(const CompLayer<EUnit, IUnit>& SCs, const CompLayer<EUnit, IUnit>& SCi, const MUnit& premotor)
  {
    //superficial and intermediate layers
    addSub(SCs);
    addSub(SCi);
    editSub(0).setName("SCs");
    editSub(1).setName("SCi");

    //deep premotor layer
    Layer<MUnit,WeightsEmpty> motorlayer(getTimeStep(), getOutWidth(), getOutHeight(), "PreMotor", "");
    motorlayer.setModule(premotor);
    addSub(motorlayer);

    //SCs is the default input and pre-motor is the default output
    setDefaultIO(0,2);
  }

  //set bottom-up input
  void input_sgs(const Image<double>& inp) 
  {
    input(inp,0);
  }

  //set top-down input
  void input_sgi(const Image<double>& inp)
  {
    input(inp,1);
  }

  //!clone the unit
  SC* clone() const {return new SC(*this); }
  
private:
  //!do our class specific interactions
  void interact()
  { 
    //sgs -> sgi  : uniform weights
    Image<double> sgs = getSub(0).getOutput();
    sgs *= itsS_I;
    input(sgs, 1);
    
    //sgi -> motor layer : uniform weights
    Image<double> sgi = getSub(1).getOutput();
    sgi *= itsI_M;
    input(sgi, 2);
    
    //inhibitory cells of sgi -> sgs excitatory : uniform Weights
    Image<double> sgi_i = getSub(1).getSub(1).getOutput();
    sgi_i *= itsI_S;
    input(sgi_i, 0);
    
    //pre-motor nurons to inhibitory of sgi : Fully connected 
    Image<double> prem = getSub(2).getOutput();
    prem = itsM_I.compute(prem);
    editSub(1).input(prem, 1);
  }
  
  //our weights
  const double itsS_I, itsI_M, itsI_S;
  WeightsAll itsM_I;
};
*/

#endif
#endif
  
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
