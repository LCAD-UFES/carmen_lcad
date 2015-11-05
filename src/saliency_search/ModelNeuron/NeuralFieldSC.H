/*!@file ModelNeuron/NeuralFieldSC.H Class declarations for various
   simulation structures */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkvit are protected //
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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/ModelNeuron/NeuralFieldSC.H $

#ifndef MODELNEURON_NEURALFIELDSC_H_DEFINED
#define MODELNEURON_NEURALFIELDSC_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

/*
#include "Util/SimTime.H"
#include "ModelNeuron/SimStructures.H"
#include "ModelNeuron/SC.H"

//######################################################################
// A neural field model which aims to mimmick the types of computations
// which take place in the mammalian supperior colliculus. The basic 
// idea is that the superficial layers (SCs) represent a spatial map
// where very early sensory components merge, and compete or coorporate
// for representation through strong local excitation and long-range 
// (but not global) inhibition. The field does not stabalize, and so
// is only avtive when input is present. We call this a N-bump 
// degenerative solution. 
//
// The intermediate layers (SCi) represent a spatial map of priority
// for the next sensor movement, which incorporates activity from the 
// SCs with external spatial signals. This layers is modeled with a 
// neural field with local excitation and global inhibition. This profile
// is to represent the combination of local inhbition in the SCi, as well 
// as a basal ganglia SCi loop which we hypothesize alters SC activity
// dynamically by increasing inhibition with more SCi activity. The
// gain of the loop can be controlled by external (top-down)
// input or from a measure of confidence on the sensor input (even a 
// measure on the SCs map itself). The effect is an layer which can 
// be dynamically switch from a stable N-bump solution to a stable
// 1-bump solution (winner-take-all). 
//
// we need to fledge this out a bit, but I am beginning to like this model for 
// signal processing purposes
//######################################################################
struct NeuralFieldSC : public SCInterface
{
  NeuralFieldSC(const double tau, 
                const double& SCs_SCi, const double& SCi_PreM, const double& PreM_SCi, const double& SCi_SCs,
                const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height, 
                const std::string name = "NeuralFieldSC", const std::string units = "") : 
    SCInterface(timestep, width, height, name, units), 
    itsS_I(SCs_SCi), itsI_M(SCi_PreM), itsM_I(PreM_SCi), itsI_S(SCi_SCs)
    
  {
    //set up SCs - local excitation and long range (but not global) inhibition
    //unstable solution purposely chosen so that the network only outputs when 
    //input is present
    NeuralFieldDoG SCs(tau, -0.05, 2.0, 3.0, 1.25, bp, timestep, width, height);
    SCs.setName("SCs");                                        
    
    //set up SCi
    NeuralFieldCS SCi(tau,-0.05, 1.5,.3, bp, timestep, width, height);    
    SCi.setName("SCi");

    //setup premotor layer
    Layer<LowPassSigmoid, WeightsEmpty> premotor(timestep, width, height, "Premotor","");
    premotor.setModule(LowPassSigmoid(tau, 0.0, 0.55, 12.0, timestep));

    addSub(SCs); addSub(SCi); addSub(premotor);
  }

  //modules interact
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
    
    //pre-motor nurons to inhibitory of sgs : 
    Image<double> prem = getSub(2).getOutput();
    input(itsM_I.compute(prem), 0);
    input(itsI_S.compute(prem), 1);
  }

  //set bottom-up input
  void input_sgs(const Image<double>& inp) 
  { input(inp, 0); }

  //set top-down input
  void input_sgi(const Image<double>& inp)
  { input(inp, 1); }
  
  //clone the object
  NeuralFieldSC* clone() const
  { return new NeuralFieldSC(*this); }

private:
  //our weights
  const double itsS_I, itsI_M;
  WeightsAll itsM_I, itsI_S;
};

// ######################################################################
// register the neural field SC
// ######################################################################
namespace 
{
  typedef SimStructure::Factory NFSCFactory;
  typedef SimStructure::Creator NFSCCreator;
  
  //define creation functions
  struct RegisterNeuralFieldSC
  {
    RegisterNeuralFieldSC() 
    {
      const double tau = 25.0;                    //decay constant for low pass modules
      const SimTime time = SimTime::MSECS(5.0);   //default simulation time
      const uint w(100), h(100);                  //default dimensions

      //an SC model were the sgs is modeled as NFNbumpDoG and the SCi as a NFNbumpCS which can turn to an NF1bumpCS
      //######################################################################
      NFSCFactory::instance().add("NeuralFieldSC",NFSCCreator::make<NeuralFieldSC>(tau, 1.0, 1.0, -0.1, -0.25, NONE,
                                                                               time,w,h));
    }
  };
  static RegisterNeuralFieldSC registernfsc;  
}
*/

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
