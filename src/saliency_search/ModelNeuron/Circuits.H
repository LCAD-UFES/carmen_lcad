/*!@file ModelNeuron/Circuits.H Class for various types of simple
   neural circuits */

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
// $HeadURL: svn://ilab.usc.edu:/trunk/saliency/src/ModelNeuron/Circuits.H $

#ifndef MODELNEURON_CIRCUITS_H_DEFINED
#define MODELNEURON_CIRCUITS_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/NeuralColumn.H"
#include "ModelNeuron/IZNeuron.H"
#include "ModelNeuron/LowpassNeuron.H"

namespace nsu
{
  // ######################################################################
  //! A Threshold circuit that slowly accumulates input and then
  //releases a short burst of spikes. The circuit contains a regular
  //spiking neuron that will spike tonically when stimulated and
  //excite a burst neuron that will integrate its input and produce a
  //short burst of spikes when a threshold is reached. The regular
  //spiking neuron also feeds to a fast spiking interneuron, which
  //inhibits the burst neuron.
  //######################################################################
  struct ThresholdCircuit : public NeuralColumn<IZNeuron>
  {
      ThresholdCircuit(const double& thresh = 1, const std::string& name="Threshold", const std::string& units="Spike")
          : NeuralColumn<IZNeuron>(SimTime::MSECS(1.0), SimUnit::STRICT, name, units)
      {
        addSub(RSNeuron());
        addSub(EBNeuron());
        addSub(FSNeuron());
    
        addWeight(NeuralColumn::IO, 0, 1.0F);//input->RS
        addWeight(0,  1, thresh * 1.0F);//RS->EB
        addWeight(1, NeuralColumn::IO, 1.0F);//EB->output
        addWeight(0,  2,  3.0F);//RS->FS
        addWeight(2,  1, -1.0F);//FS->EB
      }
  };

  // ######################################################################
  //! A bandpass filter to demonstrate use of NeuralColumn
  //! calculates a bandpass as a lowpass subtract a larger lowpass
  //######################################################################
  struct BandpassFilter : public NeuralColumn<SimUnit*>
  {
      BandpassFilter(const double& tau, const double& bandwidth, const SimTime time, 
                     const std::string& name="Bandpass", const std::string& units="pA") 
          :   NeuralColumn<SimUnit*>(time, SimUnit::NORMAL, name, units)
      {
        double simtime = 1;
        if (tau / 5.0 >= 1.0)
          simtime = tau/5;

        addSub( LowPassMaxRect(tau, 0.0, 0.0, 1.0, SimTime::MSECS(simtime)) );
        addSub( LowPass(tau + tau*bandwidth, 0.0, SimTime::MSECS(simtime)) );
    
        addWeight(NeuralColumn::IO, 0, 1.0F);
        addWeight(NeuralColumn::IO, 1, 1.0F);
        addWeight(0, NeuralColumn::IO, 1.0F);
        addWeight(1, NeuralColumn::IO, -1.0F);
      }
  };

  // ######################################################################
  //! A DampedOscillator circuit
  //######################################################################
  struct DampedOscillator : public NeuralColumn<SimUnit*>
  {
      DampedOscillator(const double& tau1, const double& tau2, 
                       const double& ff, const double& fb, const SimTime time, 
                       const std::string& name="DampedOscillator", const std::string& units="pA") 
          :   NeuralColumn<SimUnit*>(time, SimUnit::NORMAL, name, units) //,itsSigmoid(0.55, 12.0)
      {
        double simtime = time.msecs();
        addSub( LowPass(tau1, 0.0, SimTime::MSECS(simtime)) );
        addSub( LowPass(tau2, 0.0, SimTime::MSECS(simtime)) );
      
        addWeight(NeuralColumn::IO, 0, 1.0F);
        addWeight(0, 1, ff);
        addWeight(1, 0, fb);
        addWeight(0, NeuralColumn::IO, 1.0F);
      }
  };

  // ######################################################################
  //! A NeuralFeedback controller
  //######################################################################
  struct NeuralFeedback : public NeuralColumn<SimUnit*>
  {
      NeuralFeedback(const double& tau1, const double& tau2, 
                     const double& ff, const double& fb, const SimTime time, 
                     const std::string& name="NeuralFeedback", const std::string& units="pA") 
          :   NeuralColumn<SimUnit*>(time, SimUnit::NORMAL, name, units)
      {

          const double tauef = 25.0;
          const double taues = 150.0;
          const double tauif = 50.0;
          const double tauis = 150.0;
          const double steady  = -0.05;
          const double exc_rev = 1.0;
          const double inh_rev = -1.0;

        addSub( LowpassNeuronSigmoid(tauef, taues, tauif, tauis, tau1, exc_rev, inh_rev, steady, 0.55, 12.0, time) );
        addSub( LowPass(tau2, 0.0, time) );
        
        addWeight(NeuralColumn::IO, 0, 1.0F); //input
        addWeight(0, 1, ff);    //feedforward
        addWeight(1, 0, fb);    //feedback
        addWeight(0, NeuralColumn::IO, 1.0F); //output
      }
  };

  // ######################################################################
  // Register circuits types
  // ######################################################################
  namespace 
  {
    typedef SimUnit::Factory CircuitFactory;
    typedef SimUnit::Creator CircuitCreator;

    //define creation functions
    struct RegisterCircuits
    {
        RegisterCircuits()
        {
          CircuitFactory::instance().add("ThresholdCircuit", CircuitCreator::make<ThresholdCircuit>(1.0));
          CircuitFactory::instance().add("BandpassFilter", CircuitCreator::make<BandpassFilter>(50.0, 1.0, SimTime::MSECS(1.0)));
          CircuitFactory::instance().add("DampedOscillator", CircuitCreator::make<DampedOscillator>(25.0, 150.0, 1.0, -10.0, SimTime::MSECS(1.0)));
          CircuitFactory::instance().add("NeuralFeedback", CircuitCreator::make<NeuralFeedback>(50.0, 50.0, 1.0, -100.0, SimTime::MSECS(1.0)));
        }
    };
    static RegisterCircuits registercircuits;
  }
}
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
