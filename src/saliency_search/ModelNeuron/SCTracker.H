/*!@file ModelNeuron/SCTracker.H Class declarations for various
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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/ModelNeuron/SCTracker.H $

#ifndef MODELNEURON_SCTRACKER_H_DEFINED
#define MODELNEURON_SCTRACKER_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

/*
#include "Util/SimTime.H"
#include "ModelNeuron/SC.H"
#include "ModelNeuron/CompLayer.H"
#include "ModelNeuron/LowpassNeuron.H"

//######################################################################
// A tracker based on the SC, the superficial layer serves as a sensory 
// saliency map consisting of two layers, while the deeper layers acts
// as and N-bump or 1-bump stable solution. 
//######################################################################
struct SCTracker : public SCInterface
{
  SCTracker(const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height, 
            const std::string name = "SCTracker", const std::string units = "") : 
    SCInterface(timestep, width, height, name, units)
  {
*/
    /*setup the sgs: The sgs is a single lamina network containing
      interacting excitatory and inhhibitory units where the
      excitatory layer (input layer) has local recurrent
      excitation. Each unit in the exciatory layer connects directly
      to a single unit in the inhibitory layer. The inhibitory units
      connect in a gaussian pattern back to the excitatory network.
      
      The input layer units are LowpassNeuron's, which is an abstract
      neural model where positive and negative inputs are split to
      different first order lowpass filter (synapses) which then drive
      a third first order lowpass system with rectified output. A
      variety of lowpass, bandpass and shunting behavior can be
      achieved.  The inhibitory layer is a simple rectified lowpass
      filter.
    */
/*
    //for our SCs excitatory neuron    
    const double taue = 10;    //excite tau
    const double taues = 150;    //excite tau slow
    const double taui = 15;    //inhibit tau
    const double tauis = 150;    //inhibit tau slow
    const double taus = 15;    //system tau
    const double h = -0.5;     //resting state
    const double thresh = 0.0; //rectify at 0.0
    const double exc_rev = -1.0; //system low bound
    const double inh_rev = 1.0; //system high bound

    LowpassNeuronRectify e(taue, taues, taui, tauis, taus, exc_rev, inh_rev, h, thresh, timestep);
    
    const double inhtau = 10;  //inhibitory neuron tau
    const double hinh = -0.5;   //resting state
    LowPassRectify i(inhtau, hinh, thresh, timestep);
    
    //for our SCs excitatory / inhibitory competitive layer
    const double estd = 0.0;
    const double istd = 0.0;
    const double ew = 0.0; 
    const double ffwd = 1.0; 
    const double fdbck = 1000.0;   
    CompLayer<LowpassNeuronRectify, LowPassRectify> sgs(estd, istd, ew, ffwd, fdbck, 
                                                        bp, timestep, width, height, name, units);
    sgs.setModules(e,i);
    addSub(sgs);
  }

  //modules interact
  void interact()
  {
    //sgs -> sgi  : uniform weights
    //Image<double> sgs = getSub(0).getOutput();
    //input(sgs, 1);
  }
  
  //set bottom-up input
  void input_sgs(const Image<double>& inp) 
  { input(inp, 0); }
  
  //set top-down input
  void input_sgi(const Image<double>& inp)
  { input(inp, 1); }
  
  //clone the object
  SCTracker* clone() const
  { return new SCTracker(*this); }
};

// ######################################################################
// register the neural field SC
// ######################################################################
namespace 
{
  typedef SimStructure::Factory SCTRFactory;
  typedef SimStructure::Creator SCTRCreator;
  
  //define creation functions
  struct RegisterSCTracker
  {
    RegisterSCTracker() 
    {
      const SimTime time = SimTime::MSECS(1.0);   //default simulation time
      const uint w(100), h(100);                  //default dimensions
      
      //an SC model were the sgs is modeled as NFNbumpDoG and the SCi as a NFNbumpCS which can turn to an NF1bumpCS
      //######################################################################
      SCTRFactory::instance().add("SCTracker",SCTRCreator::make<SCTracker>(NONE, time, w, h));
    }
  };
  static RegisterSCTracker registersctr;  
}

*/

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


    //LowpassCompLayer SCs(tau, -0.05, -0.05, 1.5, 3.0, 0.25, 1.0, -0.75, bp, timestep, width, height);
    // set up SCs - local excitation and long range (but not global), in a two layer structure, 
    // allowing for adaptation. 
    //LowpassCompLayer SCs(tau, -0.05, -0.05, 1.5, 3.0, 0.25, 1.0, -0.75, bp, timestep, width, height);
    //IZCompLayer SCs(1.0, 3.0, 1.0, 5.0, -1.0, NONE, timestep, width, height);
    //FSNeuron fs;
    //fs.setDecoderPost(AlphaDecoder(timestep, SimTime::MSECS(15.0)));
    //SCs.setModules(fs, fs);
    //SCs.setName("SCs");  
    //addSub(SCs); 

    //set up SCi, an n-bump solution which can be switch to an 1-bump solution
    //NeuralFieldCS SCi(tau,-0.05, 1.5,.3, bp, timestep, width, height);    
    //SCi.setName("SCi");
    //addSub(SCi); the problem is in the SC module
