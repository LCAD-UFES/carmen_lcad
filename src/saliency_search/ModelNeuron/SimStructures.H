/*!@file ModelNeuron/SimStructures.H Class declarations for various
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SimStructures.H $

#ifndef MODELNEURON_SIMSTRUCTURES_H_DEFINED
#define MODELNEURON_SIMSTRUCTURES_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Util/SimTime.H"
#include "ModelNeuron/Layer.H"
#include "ModelNeuron/CompLayer.H"
#include "ModelNeuron/LowPass.H"
#include "ModelNeuron/LowpassNeuron.H"
#include "ModelNeuron/Circuits.H"

//#include "ModelNeuron/IZNeuron.H"
//#include "ModelNeuron/SC.H"


namespace nsu
{
  //######################################################################
  // Some different neural field type models
  //######################################################################
  //  All equations for the below classes are inspired by Neural Field
  //  dynamics: Amari & Arbib, 77; Coombs, 05; Erlhagen & Bicho, 2006
  //######################################################################

  //######################################################################
  // A neural field with local gaussian connectivity
  //######################################################################
  struct NeuralFieldGauss : public Layer<LowPassSigmoid, WeightsBinomial>
  {
      NeuralFieldGauss(const double& tau, const double& h, const double& estd, const BorderPolicy bp, 
                       const SimTime& timestep, const uint width, const uint height) 
          : Layer<LowPassSigmoid, WeightsBinomial>(estd, 1.0, true, bp,         //weights related options
                                                   timestep, width, height,     //structuremodule related options
                                                   "NFG", "pA")                 //set the name and units
      {
        //set a configured lowpass filter for all positions in layer
        setModule(LowPassSigmoid(tau, h, 0.55, 12, timestep));
      }
  };

  //######################################################################
  // A neural field where the interaction kernel is a Gaussian minus
  // a constant
  //######################################################################
  struct NeuralFieldCS : public Layer<LowPassSigmoid, WeightsCS>
  {
      NeuralFieldCS(const double& tau, const double& h, const double& estd, const double& inhibit, 
                    const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) 
          : Layer<LowPassSigmoid, WeightsCS>(estd, 1.0, inhibit, false, bp,     //weights related options
                                             timestep, width, height,          //structuremodule related options
                                             "NFCS", "pA")                     //set the name and units
      {
        //set a configured lowpass filter for all positions in layer
        setModule(LowPassSigmoid(tau, h, 0.55, 12, timestep));
      }
  };

  //######################################################################
  // A neural field where the interaction kernel is a difference of 
  // Gaussians.
  //######################################################################
  struct NeuralFieldDoG : public Layer<LowPassSigmoid, WeightsDoG>
  {
      NeuralFieldDoG(const double& tau, const double& h, const double& estd, const double& istd, const double& e_i_ratio,
                     const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) 
          : Layer<LowPassSigmoid, WeightsDoG>(estd, istd, e_i_ratio, 1.0, false, bp,   //weights related options
                                              timestep, width, height,                 //structuremodule related options
                                              "NFDoG", "pA")                           //set the name and units
      {
        //set a configured lowpass filter for all positions in layer
        setModule(LowPassSigmoid(tau, h, 0.55, 12, timestep));
      }
  };

  //######################################################################
  // a feeback controler with a gaussian connectivity kernel 
  //######################################################################
  struct LowpassNeuronFeedbackLayer : public Layer<NeuralFeedback, WeightsBinomial>
  {    
      LowpassNeuronFeedbackLayer(const double& tau1, const double& tau2, const double& feedforward, const double& feedback,
                                 const double& eweight, const double& estd, const BorderPolicy bp,
                                 const SimTime& timestep, const uint width, const uint height) :
          Layer<NeuralFeedback, WeightsBinomial>(eweight, estd, true, bp, timestep, width, height, "LowpassNeuronFeedbackLayer", "pA")
      { 
        setModule(NeuralFeedback(tau1, tau2, feedforward, feedback, timestep));
      }
  };
  
  //######################################################################
  // 2-layer (excitatory and inhibitory) competitive network with a
  // LowpassNeuronRectify
  //######################################################################
  struct SCsRectify : public CompLayer<LowpassNeuronRectify, LowPass>
  {    
      SCsRectify(const double& excite_std, const double& inhibit_std,
                 const double& excite_w, const double& feedforward, const double& feedback,
                 const double tauef,       //fast linear excitatory tau
                 const double taues,      //slow quadratic  excitatory tau
                 const double tauif,       //fast linear inhibitory tau
                 const double tauis,      //slow linear inhibitory tau
                 const double taus ,       //system tau
                 const double taufeedback, //feedback tau
                 const double steady,    //steady state of system
                 const double exc_rev,      //excitatory reversal point
                 const double inh_rev,     //inhibitory reversal point
                 const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) :
          CompLayer<LowpassNeuronRectify, LowPass>(excite_std, inhibit_std, excite_w, feedforward, feedback,
                                                   bp, timestep, width, height, "LowpassSCs", "pA")
      { 
        setModules( LowpassNeuronRectify(tauef, taues, tauif, tauis, taus, exc_rev, inh_rev, steady, 0.0, timestep), 
                    LowPass(taufeedback, 0.0, timestep) );
      }
  };
  
  //######################################################################
  // 2-layer (excitatory and inhibitory) competitive network with a
  // LowpassNeuronSigmoid
  //######################################################################
  struct SCsSigmoid : public CompLayer<LowpassNeuronSigmoid, LowPass>
  {    
      SCsSigmoid(const double& excite_std, const double& inhibit_std,
                 const double& excite_w, const double& feedforward, const double& feedback,
                 const double tauef,       //fast linear excitatory tau
                 const double taues,      //slow quadratic  excitatory tau
                 const double tauif,       //fast linear inhibitory tau
                 const double tauis,      //slow linear inhibitory tau
                 const double taus ,       //system tau
                 const double taufeedback, //feedback tau
                 const double steady ,    //steady state of system
                 const double exc_rev,      //excitatory reversal point
                 const double inh_rev,     //inhibitory reversal point
                 const double thresh,      //threshold of sigmoid
                 const double slope,       //slope of sigmoid
                 const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) :
          CompLayer<LowpassNeuronSigmoid, LowPass>(excite_std, inhibit_std, excite_w, feedforward, feedback, 
                                                   bp, timestep, width, height, "LowpassSCs", "pA")
      { 
        setModules( LowpassNeuronSigmoid(tauef, taues, tauif, tauis, taus, exc_rev, inh_rev, steady, thresh, slope, timestep), 
                    LowPass(taufeedback, 0.0, timestep) );
      }
  };
  
  //######################################################################
  // 2-layer (excitatory and inhibitory) competitive network with a
  // Lowpass Rectify
  //######################################################################
  struct LowpassSCsRectify : public CompLayer<LowPassRectify, LowPass>
  {    
      LowpassSCsRectify(const double& excite_std, const double& inhibit_std,
                        const double& excite_w, const double& feedforward, const double& feedback,
                        const double taus ,       //system tau
                        const double taufeedback, //feedback tau
                        const double steady,    //steady state of system
                        const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) :
          CompLayer<LowPassRectify, LowPass>(excite_std, inhibit_std, excite_w, feedforward, feedback,
                                             bp, timestep, width, height, "LowpassSCs", "pA")
      { 
        setModules( LowPassRectify(taus, steady, 0.0, timestep), 
                    LowPass(taufeedback, 0.0, timestep) );
      }
  };
  
  //######################################################################
  // 2-layer (excitatory and inhibitory) competitive network with a
  // Lowpass Sigmoid
  //######################################################################
  struct LowpassSCsSigmoid : public CompLayer<LowPassSigmoid, LowPass>
  {    
      LowpassSCsSigmoid(const double& excite_std, const double& inhibit_std,
                        const double& excite_w, const double& feedforward, const double& feedback,
                        const double taus ,       //system tau
                        const double taufeedback, //feedback tau
                        const double steady ,    //steady state of system
                        const double thresh,      //threshold of sigmoid
                        const double slope,       //slope of sigmoid
                        const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) :
          CompLayer<LowPassSigmoid, LowPass>(excite_std, inhibit_std, excite_w, feedforward, feedback, 
                                             bp, timestep, width, height, "LowpassSCs", "pA")
      { 
        setModules( LowPassSigmoid(taus, steady, thresh, slope, timestep), 
                    LowPass(taufeedback, 0.0, timestep) );
      }
  };
  
  // ######################################################################
  // register the neural field models
  // ######################################################################
  namespace 
  {
    typedef SimStructure::Factory NFFactory;
    typedef SimStructure::Creator NFCreator;
  
    //define creation functions
    struct RegisterNeuralField
    {
        RegisterNeuralField() 
        {
          const double tau = 25.0;                    //decay constant for low pass modules
          const SimTime time = SimTime::MSECS(5.0);   //default simulation time
          const uint w(100), h(100);                  //default dimensions

          //a simple gaussian neural field with either a null stable or full stable solution
          // ######################################################################
          NFFactory::instance().add("NFGauss", NFCreator::make<NeuralFieldGauss>(tau, -0.5, 1.5, NONE, time, w, h));

          //a global inhibition neural field - global inhibition (4th param) adjusts selection process.  values around 0.95
          //and below will lead to stable solutions, with values close to 0 approaching full activity solution. N-bump
          //solutions for values < around .45
          // ######################################################################
          //stable n-bump solution
          NFFactory::instance().add("NFNbumpCS", NFCreator::make<NeuralFieldCS>(tau,-0.05, 1.5,.3, NONE, time, w, h));
          //stable 1-bump solution
          NFFactory::instance().add("NF1bumpCS", NFCreator::make<NeuralFieldCS>(tau,-0.05, 1.5,.6, NONE, time, w, h));

          //a local inhibition neural field - ratio fo excitation/inhibition strength (5th param) can be used to adjust the
          //selection process for a fixed size of excitation and inhibition (3rd and 4th param). ratio > 1 and < around 2.5
          //will lead to a stable solution of N-regions each consisting of 1 bump. < 1 solutions are degenerating.
          //######################################################################
          NFFactory::instance().add("NFNbumpDoG",NFCreator::make<NeuralFieldDoG>(tau,-0.05,2.0,8.0,2.00,NONE,time,w,h));
          NFFactory::instance().add("NF0bumpDoG",NFCreator::make<NeuralFieldDoG>(tau,-0.05,2.0,8.0,1.0,NONE,time,w,h));

          //nerual feeback controler
          //######################################################################
          NFFactory::instance().add("LowpassNeuronFeedbackLayer", NFCreator::make<LowpassNeuronFeedbackLayer>(25.0, 50.0, 1.0, -1000.0, 0.0, 0.0, NONE, time, w, h));

          //a simple two-layer spatiotemporal competetive network where an excitatory layer connects 'horizontally' and
          //to an inhbitory layer that projects in feeback over a large spatial extent
          //######################################################################
          NFFactory::instance().add("SCsSigmoid",NFCreator::make<SCsSigmoid>(1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 150.0, 50.0, 150.0, 25.0, 50.0, -0.05, 1.0, -1.0, 0.55, 12.0, NONE, time, w, h));
          NFFactory::instance().add("SCsRectify",NFCreator::make<SCsRectify>(1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 150.0, 50.0, 150.0, 25.0, 50.0, -0.05, 1.0, -1.0, NONE, time, w, h));
          
          NFFactory::instance().add("LowPassSCsSigmoid",NFCreator::make<LowpassSCsSigmoid>(1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -0.5, 0.55, 12.0, NONE, time, w, h));
          NFFactory::instance().add("LowPassSCsRectify",NFCreator::make<LowpassSCsRectify>(1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -0.5, NONE, time, w, h));
        }
    };
    static RegisterNeuralField registernf;  
  }

  /*
  //######################################################################
  // Models were the units are still esentially low-pass filters, but
  // the excitatory and inhibitory layers are more explicitly modeled. 
  //######################################################################

  //######################################################################
  // 2-layer (excitatory and inhibitory) competitive network
  //######################################################################
  struct LowpassCompLayer : public CompLayer<LowPassSigmoid, LowPassRectify>
  {
  LowpassCompLayer(const double& tau, const double& h1, const double& h2, 
  const double& excite_std, const double& inhibit_std,
  const double& excite_w, const double& feedforward, const double& feedback,
  const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) :
  CompLayer<LowPassSigmoid, LowPassRectify>(excite_std, inhibit_std, excite_w, feedforward, feedback, 
  bp, timestep, width, height, "LowpassCompLayer", "")
  { 
  setModules(LowPassSigmoid(tau, h1, 0.55, 12, timestep), LowPassRectify(tau, h2, 0.0, timestep));
  }
  };

  //######################################################################
  // 2-layer (excitatory and inhibitory) competitive network with a
  // LowpassNeuron
  //######################################################################
  struct LowpassNeuronCompLayer : public CompLayer<LowpassNeuronSigmoid, LowPass>
  {    
  LowpassNeuronCompLayer(const double& excite_std, const double& inhibit_std,
  const double& excite_w, const double& feedforward, const double& feedback,
  const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) :
  CompLayer<LowpassNeuronSigmoid, LowPass>(excite_std, inhibit_std, excite_w, feedforward, feedback, 
  bp, timestep, width, height, "LowpassNeuronCompLayer", "pA")
  { 
  //NeuronSigmoid
  const double tauef = 25.0;
  const double taues = 150.0;
  const double tauif = 50.0;
  const double tauis = 150.0;
  const double taus  = 25.0;
  const double taufeedback = 50.0;
  const double steady  = -0.05;
  const double gain  = 1.0;
  const double exc_rev = 1.0;
  const double inh_rev = -1.0; 
  const double thresh = 0.55;
  const double slope = 12.0;
      
  setModules( LowpassNeuronSigmoid(tauef, taues, tauif, tauis, taus, exc_rev, inh_rev,  gain, steady, thresh, slope, timestep), 
  LowPass(taufeeback, 0.0, timestep));
  }
  };

  //######################################################################
  // A Lowpass filter model of the mammilian superior colliculus where 
  // superficial and deep layers are each modeled with two layer competitive
  // networks of lowpass filters with sigmoidal firing rate functions
  //######################################################################
  struct LowpassSC : public SC
  {
  LowpassSC(const double tau, 
  const double& SCs_SCi, const double& SCi_PreM, const double& PreM_SCi, const double& SCi_SCs,
  const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height, 
  const std::string name = "LowpassSC", const std::string units = "") :
  SC(SCs_SCi, SCi_PreM, PreM_SCi, SCi_SCs, bp, timestep, width, height, name, units)
  {
  LowpassCompLayer SCs(tau, -0.05, -0.05, 1.5, 3.0, 0.25, 1.0, -0.75, bp, timestep, width, height);
  LowpassCompLayer SCi(tau, -0.05, -0.05, 1.0, 1.0, 0.15, 1.0, -0.10, bp, timestep, width, height);
  setLayers(SCs, SCi, LowPassSigmoid(tau, 0.0, 1.0, 1.0, timestep));
  }
  };

  // ######################################################################
  // register the neural field models
  // ######################################################################
  namespace 
  {
  typedef SimStructure::Factory SSFactory;
  typedef SimStructure::Creator SSCreator;
  
  //define creation functions
  struct RegisterLowpass
  {
  RegisterLowpass() 
  {
  const double tau = 25.0;                    //decay constant for low pass modules
  const SimTime time = SimTime::MSECS(5.0);   //default simulation time
  const uint w(100), h(100);                  //default dimensions

      
  //two layer competitive neural field, a weak wta version
  //######################################################################
  SSFactory::instance().add("WeakWTA",SSCreator::make<LowpassCompLayer>(tau,-0.05, -0.05, 
  1.0, 9.0, 0.1, 1.0, -10.0, 
  NONE,time,w,h));

  //######################################################################
  SSFactory::instance().add("LowpassNeuronCompLayer",SSCreator::make<LowpassNeuronCompLayer>(1.0, 4.0, 1.0, 1.0, -250.0, NONE, time, w, h));
      
  //two layer competitive neural field, growing blob version
  //######################################################################
  SSFactory::instance().add("WeakBlob",SSCreator::make<LowpassCompLayer>(tau,-0.05, -0.05, 
  1.0, 1.0, 0.15, 1.0, -0.1, 
  NONE,time,w,h));
  //lowpass filter version of the SC model
  //######################################################################
  SSFactory::instance().add("LowpassSC",SSCreator::make<LowpassSC>(tau, 1.0, 1.0, 1.0, -1.0, NONE, time, w, h));
  }
  };
  static RegisterLowpass registerlowpass;  
  }

  //######################################################################
  // spiking models
  //######################################################################

  //######################################################################
  // Two layer competetive networks using spiking neurons, inspired by the 
  // construction of each layer in the mammalian superior colliculus. 
  //######################################################################
  struct IZCompLayer : public CompLayer<FSNeuron, FSNeuron>
  {
  IZCompLayer(const double& excite_std, const double& inhibit_std,
  const double& excite_w, const double& feedforward, const double& feedback,
  const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height) :
  CompLayer<FSNeuron, FSNeuron>(excite_std,inhibit_std,excite_w,feedforward,feedback, 
  bp, timestep, width, height, "IZCompLayer", "")
  { 
  setModules(FSNeuron(), FSNeuron());
  }
  };

  //######################################################################
  // A spiking model of the mamillian superior colliculus
  //######################################################################
  struct SpikingSC : public SC
  {
  SpikingSC(const double& SCs_SCi, const double& SCi_PreM, const double& PreM_SCi, const double& SCi_SCs,
  const BorderPolicy bp, const SimTime& timestep, const uint width, const uint height, 
  const std::string name = "SpikingSC", const std::string units = "") :
  SC(SCs_SCi, SCi_PreM, PreM_SCi, SCi_SCs, bp, timestep, width, height, name, units)
  {
  IZCompLayer SCs(1.0, 9.0, 3.0, 2.0, -1, bp, timestep, width, height);
  IZCompLayer SCi(1.0, 0.0, 2.0, 6.0, -0.3, bp, timestep, width, height);
  setLayers(SCs, SCi, EBNeuron());
  }
  };

  // ######################################################################
  // register the neural simulation models
  // ######################################################################
  namespace 
  {
  typedef SimStructure::Factory SpikeLayerFactory;
  typedef SimStructure::Creator SpikeLayerCreator;
  
  //define creation functions
  struct RegisterSpikeLayers
  {
  RegisterSpikeLayers() 
  {
  const SimTime time = SimTime::MSECS(1.0);
  const uint w(100), h(100); //default dimensions

  // superficial SGs model
  // ######################################################################
  SpikeLayerFactory::instance().add("SCs", SpikeLayerCreator::make<IZCompLayer>(1.0, 9.0, 3.0, 2.0, -0.05, 
  NONE, time, w, h));
      
  // our intermediate SC model
  // ######################################################################
  SpikeLayerFactory::instance().add("SCi", SpikeLayerCreator::make<IZCompLayer>(1.0, 0.0, 2.0, 6.0, -0.3, 
  NONE, time, w, h));

  //spiking version of the SC model
  //######################################################################
  SpikeLayerFactory::instance().add("SC", SpikeLayerCreator::make<SpikingSC>(1.0, 1.0, 1.0, -1.0, 
  NONE, time, w, h));
  }
  };
  static RegisterSpikeLayers registersspikelayers;  
  }
  */
}


#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
