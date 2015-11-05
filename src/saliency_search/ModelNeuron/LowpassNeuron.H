/*!@file ModelNeuron/LowpassNeuron.H a very simple and abstract kind of neuron */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/LowpassNeuron.H $

#ifndef MODELNEURON_LOWPASSNEURON_H_DEFINED
#define MODELNEURON_LOWPASSNEURON_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/SimUnit.H"
#include "ModelNeuron/LowPass.H"

namespace nsu
{
  // ##x####################################################################
  // rate functions for the LowpassNeuron synapses
  // ######################################################################
  namespace LowpassNeuronRate
  {
    struct LowpassNeuronRate
    {
        LowpassNeuronRate() : itsState(0.0) { }
        void setState(const double& state) { itsState = state; };
        
      protected:
        double itsState;
    };
    // ######################################################################
    // linear rate function with a reversal
    // ######################################################################
    struct LinearExcitatory : public LowpassNeuronRate
    {
        //constructor
        LinearExcitatory(const double& reversal) : 
            LowpassNeuronRate(), itsReversal(reversal) { };
        //destructor
        ~LinearExcitatory() { };
        //get the name
        static const char* name() { return "Linear Excitatory"; };

        //overload function call operator
        const double operator()(const double& in) const 
        { return in * (itsReversal - itsState); };

      private:
        double itsReversal;
    };

    // ######################################################################
    // Squared rate function with a reversal
    // ######################################################################
    struct SquaredExcitatory : public LowpassNeuronRate
    {
        //constructor
        SquaredExcitatory(const double& reversal, const double& rest) : 
            LowpassNeuronRate(), itsReversal(reversal), itsRest(rest) { };
        //destructor
        ~SquaredExcitatory() { };
        //get the name
        static const char* name() { return "Squared Excitatory"; };

        //overload function call operator
        const double operator()(const double& in) const 
        { 
          const double diff = (itsState-itsRest);
          return in * (itsReversal - itsState) * diff * diff; 
        };

      private:
        double itsReversal;
        double itsRest;
    };

    // ######################################################################
    // linear rate function with a reversal
    // ######################################################################
    struct LinearInhibitory : public LowpassNeuronRate
    {
        //constructor
        LinearInhibitory(const double& reversal) : 
            LowpassNeuronRate(), itsReversal(reversal) { };
        //destructor
        ~LinearInhibitory() { };
        //get the name
        static const char* name() { return "Linear Inhibitory"; };

        //overload function call operator
        const double operator()(const double& in) const 
        { return in*-1*(itsState - itsReversal) ; };

      private:
        double itsReversal;
    };
  }

  // ######################################################################
  // a simple lowpass synapse
  // ######################################################################
  template <class RateFunc = RateFunctions::EmptyFunction, class IntType = LowPassExpEuler>
  class LowpassSynapse : public LowPassFilter<RateFunc, IntType>
  {
    public:
      //! Constructor with default params, overload for RateFuncs with 1 param
      LowpassSynapse(const double& tau, //time constant msecs
                     const double& param1,//param 1 for the rate function
                     const SimTime& timestep, //simulaton timestep,
                     const std::string& name = "Lowpass Synapse", 
                     const std::string& units = "pA");
      
      //! Constructor with default params, overload for RateFuncs with 2 param
      LowpassSynapse(const double& tau, //time constant msecs
                     const double& param1,//param 1 for the rate function
                     const double& param2,//param 2 for the rate function
                     const SimTime& timestep, //simulaton timestep,
                     const std::string& name = "Lowpass Synapse", 
                     const std::string& units = "pA");

      //! sets the voltage of the system
      void setVoltage(const double & voltage);

    private:     
      //! clone this object
      LowpassSynapse<RateFunc,IntType>* doClone() const;
  };

  // ######################################################################
  /*
    abstract neural model made from simple first order linear dynamic systems 
  */
  // ######################################################################
  template <class RateFunc = RateFunctions::EmptyFunction, class IntType = LowPassExpEuler>
  class LowpassNeuronImpl: public LowPassFilter<RateFunc, IntType>
  {
    public:  
      //! Constructor with default params
      LowpassNeuronImpl(const double& tau_ef,         // fast excite time constant msecs
                        const double& tau_es,         // slow excite time constant msecs
                        const double& tau_if,         // fast inhibit time constant msecs
                        const double& tau_is,         // slow inhibit time constant msecs
                        const double& tau_s,          // system time constant msecs
                        const double& exc_reversal,   // excitatory reversal point
                        const double& inh_reversal,   // inhibitory reversal point
                        const double& h,              // system steady state
                        const SimTime& timestep,      // simulaton timestep
                        const std::string& name = "LowpassNeuron", 
                        const std::string& units = "pA");
  
      //! Constructor with default params
      LowpassNeuronImpl(const double& tau_ef,         // fast excite time constant msecs
                        const double& tau_es,         // slow excite time constant msecs
                        const double& tau_if,         // fast inhibit time constant msecs
                        const double& tau_is,         // slow inhibit time constant msecs
                        const double& tau_s,          // system time constant msecs
                        const double& exc_reversal,   // excitatory reversal point
                        const double& inh_reversal,   // inhibitory reversal point
                        const double& h,              // system steady state
                        const double& param1,         // firing rate param1
                        const SimTime& timestep,      // simulaton timestep
                        const std::string& name = "LowpassNeuron", 
                        const std::string& units = "pA");
  
      //! Constructor with default params
      LowpassNeuronImpl(const double& tau_ef,         // fast excite time constant msecs
                        const double& tau_es,         // slow excite time constant msecs
                        const double& tau_if,         // fast inhibit time constant msecs
                        const double& tau_is,         // slow inhibit time constant msecs
                        const double& tau_s,          // system time constant msecs
                        const double& exc_reversal,   // excitatory reversal point
                        const double& inh_reversal,   // inhibitory reversal point
                        const double& h,              // system steady state
                        const double& param1,         // firing rate param1 1
                        const double& param2,         // firing rate param1 2
                        const SimTime& timestep,      // simulaton timestep
                        const std::string& name = "LowpassNeuron", 
                        const std::string& units = "pA");
  
      //! destructor
      ~LowpassNeuronImpl() { };

      //! get the number of subs
      const uint numSubs() const;

      //!get the sub at the index
      const SimUnit& getSub(const uint i) const;

      //!edit the sub at the index
      SimUnit& editSub(const uint i);
  
    private:
      //! integrate a time step using exponential euler
      const double doIntegrate(const SimTime& dt, const double& ine, const double& inh);
  
      //!initialize or reset internal variables
      void doInit();
  
      //! clone this object
      LowpassNeuronImpl<RateFunc,IntType>* doClone() const;
  
      //4 synapses
      LowpassSynapse<LowpassNeuronRate::LinearExcitatory,  IntType> itsEf;
      LowpassSynapse<LowpassNeuronRate::SquaredExcitatory, IntType> itsEs;
      LowpassSynapse<LowpassNeuronRate::LinearInhibitory,  IntType> itsIf;
      LowpassSynapse<LowpassNeuronRate::LinearInhibitory,  IntType> itsIs;
  };

  // ######################################################################
  // typedefs
  // ######################################################################
  typedef 
  LowpassNeuronImpl<RateFunctions::EmptyFunction, LowPassExpEuler> LowpassNeuron;
  typedef 
  LowpassNeuronImpl<RateFunctions::RectifyFunction, LowPassExpEuler> LowpassNeuronRectify;
  typedef 
  LowpassNeuronImpl<RateFunctions::FullRectifyFunction, LowPassExpEuler> LowpassNeuronFullRectify;
  typedef 
  LowpassNeuronImpl<RateFunctions::StepFunction, LowPassExpEuler> LowpassNeuronStep;
  typedef 
  LowpassNeuronImpl<RateFunctions::MaxRectFunction, LowPassExpEuler> LowpassNeuronMaxRect;
  typedef 
  LowpassNeuronImpl<RateFunctions::LogThreshFunction, LowPassExpEuler> LowpassNeuronLog;
  typedef 
  LowpassNeuronImpl<RateFunctions::SigmoidFunction, LowPassExpEuler> LowpassNeuronSigmoid;

  // ######################################################################
  // register our objects
  // ######################################################################
  namespace 
  {
    typedef SimUnit::Factory LPNFactory;
    typedef SimUnit::Creator LPNCreator;

    //define creation functions
    struct RegisterLowpassNeuron
    {
        RegisterLowpassNeuron()
        {
          const double tauef = 25.0;
          const double taues = 150.0;
          const double tauif = 50.0;
          const double tauis = 150.0;
          const double taus  = 25.0;
          const double steady  = -0.05;
          const double exc_rev = 1.0;
          const double inh_rev = -1.0;
      
          const SimTime time = SimTime::MSECS(1.0);//so the auto time step is used
      
          LPNFactory::instance().add("LowpassNeuron", 
                                     LPNCreator::make<LowpassNeuron>(tauef, taues, tauif, tauis, 
                                                                     taus, exc_rev, inh_rev, steady, time));
          LPNFactory::instance().add("LowpassNeuronRectify", 
                                     LPNCreator::make<LowpassNeuronRectify>(tauef, taues, tauif, tauis, 
                                                                            taus, exc_rev, inh_rev, steady, 0.0, time));
          LPNFactory::instance().add("LowpassNeuronFullRectify", 
                                     LPNCreator::make<LowpassNeuronFullRectify>(tauef, taues, tauif, 
                                                                                tauis, taus, exc_rev, inh_rev, steady, time));
          LPNFactory::instance().add("LowpassNeuronStep", 
                                     LPNCreator::make<LowpassNeuronStep>(tauef, taues, tauif, 
                                                                         tauis, taus, exc_rev, inh_rev, steady, 0.0, 1.0, time));
          LPNFactory::instance().add("LowpassNeuronMaxRect", 
                                     LPNCreator::make<LowpassNeuronMaxRect>(tauef, taues, tauif, 
                                                                            tauis, taus, exc_rev, inh_rev, steady, 0.0, 1.0, time));
          LPNFactory::instance().add("LowpassNeuronLog", 
                                     LPNCreator::make<LowpassNeuronLog>(tauef, taues, tauif, 
                                                                        tauis, taus, exc_rev, inh_rev, steady, 0.1, time));
          LPNFactory::instance().add("LowpassNeuronSigmoid",
                                     LPNCreator::make<LowpassNeuronSigmoid>(tauef, taues, tauif, 
                                                                            tauis, taus, exc_rev, inh_rev, steady, 0.55,12.0, time));
        }
    };
    static RegisterLowpassNeuron registerlpn;  
  }

  // ######################################################################
  // ##### implementation for LowpassSynapse
  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowpassSynapse<RateFunc, IntType>::LowpassSynapse(const double& tau, const double& param1, const SimTime& timestep, 
                                                    const std::string& name, const std::string& units) :
      LowPassFilter<RateFunc, IntType>(tau, 0.0, param1, timestep, name, units) { }
  
  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowpassSynapse<RateFunc, IntType>::LowpassSynapse(const double& tau, const double& param1, const double&  param2, const SimTime& timestep, 
                                                    const std::string& name, const std::string& units) :
      LowPassFilter<RateFunc, IntType>(tau, 0.0, param1, param2, timestep, name, units) { }
  
  // ######################################################################
  template <class RateFunc, class IntType> inline
  void LowpassSynapse<RateFunc, IntType>::setVoltage(const double & voltage)
  {
    LowPassFilter<RateFunc, IntType>::firingRate.setState(voltage);
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowpassSynapse<RateFunc,IntType>* LowpassSynapse<RateFunc, IntType>::doClone() const
  {
    return new LowpassSynapse<RateFunc,IntType>(*this);
  }

  // ######################################################################
  // ##### implementation for LowpassNeuronImpl
  // ######################################################################
  // default case rate function takes 0 params
  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowpassNeuronImpl<RateFunc,IntType>::LowpassNeuronImpl(const double& tauef, 
                                                         const double& taues, 
                                                         const double& tauif, 
                                                         const double& tauis, 
                                                         const double& taus, 
                                                         const double& exc_reversal,
                                                         const double& inh_reversal,
                                                         const double& h,
                                                         const SimTime& timestep,
                                                         const std::string& name, 
                                                         const std::string& units) : 
      LowPassFilter<RateFunc, IntType>(taus, h, timestep, name, units), 
      itsEf(tauef, exc_reversal, timestep, "Fast", units), 
      itsEs(taues, exc_reversal, h, timestep, "Slow", units),
      itsIf(tauif, inh_reversal, timestep, "Fast", units), 
      itsIs(tauis, inh_reversal, timestep, "Slow", units) 
  { };

  // ######################################################################
  // default case rate function takes 1 params
  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowpassNeuronImpl<RateFunc,IntType>::LowpassNeuronImpl(const double& tauef, 
                                                         const double& taues, 
                                                         const double& tauif, 
                                                         const double& tauis, 
                                                         const double& taus, 
                                                         const double& exc_reversal,
                                                         const double& inh_reversal,
                                                         const double& h,
                                                         const double& param1,
                                                         const SimTime& timestep,
                                                         const std::string& name, 
                                                         const std::string& units) : 
      LowPassFilter<RateFunc, IntType>(taus, h, param1, timestep, name, units), 
      itsEf(tauef, exc_reversal, timestep, "Fast", units), 
      itsEs(taues, exc_reversal, h, timestep, "Slow", units),
      itsIf(tauif, inh_reversal, timestep, "Fast", units), 
      itsIs(tauis, inh_reversal, timestep, "Slow", units)
  { };

  // ######################################################################
  // default case rate function takes 2 params
  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowpassNeuronImpl<RateFunc,IntType>::LowpassNeuronImpl(const double& tauef,
                                                         const double& taues,  
                                                         const double& tauif,
                                                         const double& tauis,  
                                                         const double& taus, 
                                                         const double& exc_reversal,
                                                         const double& inh_reversal,
                                                         const double& h,
                                                         const double& param1,
                                                         const double& param2,
                                                         const SimTime& timestep,
                                                         const std::string& name, 
                                                         const std::string& units) : 
      LowPassFilter<RateFunc, IntType>(taus, h, param1, param2, timestep,name, units), 
      itsEf(tauef, exc_reversal, timestep, "Fast", units), 
      itsEs(taues, exc_reversal, h, timestep, "Slow", units),
      itsIf(tauif, inh_reversal, timestep, "Fast", units), 
      itsIs(tauis, inh_reversal, timestep, "Slow", units)
  { };

  // ######################################################################
  template <class RateFunc, class IntType> inline
  const uint LowpassNeuronImpl<RateFunc,IntType>::numSubs() const
  {
    return 4;
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline
  const SimUnit& LowpassNeuronImpl<RateFunc,IntType>::getSub(const uint i) const
  {
    switch (i)
    {
    case 0:
      return itsEf;
    case 1:
      return itsEs;
    case 2:
      return itsIf;
    case 3:
      return itsIs;
    default:
      LFATAL("index out of range. check numSubs() before calling getSub()");
      return *this;//will never execute
    }
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline
  SimUnit& LowpassNeuronImpl<RateFunc,IntType>::editSub(const uint i)
  {
    switch (i)
    {
    case 0:
      return itsEf;
    case 1:
      return itsEs;
    case 2:
      return itsIf;
    case 3:
      return itsIs;
    default:
      LFATAL("index out of range. check numSubs() before calling getSub()");
      return *this;//will never execute
    }
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline
  const double LowpassNeuronImpl<RateFunc,IntType>::doIntegrate(const SimTime& dt, 
                                                                const double& ine,
                                                                const double& inh)
  {
    //feed inputs to our excitatory synapses
    itsEf.input(ine); //fast
    itsEs.input(ine); //slow
  
    //feed inputs to our inhibitory synapses
    const double ninh = inh*-1.0;
    itsIf.input(ninh); //fast
    itsIs.input(ninh); //slow
  
    //set the neurons state (voltage) in each synapse
    itsEf.setVoltage(LowPassFilter<RateFunc,IntType>::getState());
    itsEs.setVoltage(LowPassFilter<RateFunc,IntType>::getState());
    itsIf.setVoltage(LowPassFilter<RateFunc,IntType>::getState());
    itsIs.setVoltage(LowPassFilter<RateFunc,IntType>::getState());

    //evolve the synapses
    const SimTime ctime = LowPassFilter<RateFunc,IntType>::getTime();
    itsEf.evolve(ctime);
    itsEs.evolve(ctime);
    itsIf.evolve(ctime);
    itsIs.evolve(ctime);

    //excitatory and inhibitory drives
    const double exc_drive = itsEf.getOutput() + itsEs.getOutput(); //positive value
    const double inh_drive = itsIf.getOutput() + itsIs.getOutput(); //negative value 
  
    //update system dynamics and return rate
    return LowPassFilter<RateFunc,IntType>::doIntegrate(dt, exc_drive, inh_drive);
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline
  void LowpassNeuronImpl<RateFunc,IntType>::doInit()
  {
    itsEs.initialize();
    itsEf.initialize();
    itsIs.initialize();
    itsIf.initialize();
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline 
  LowpassNeuronImpl<RateFunc,IntType>* LowpassNeuronImpl<RateFunc,IntType>::doClone() const
  {
    return new LowpassNeuronImpl<RateFunc,IntType>(*this);
  }
}

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
