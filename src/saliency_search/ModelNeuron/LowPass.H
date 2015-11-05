/*!@file ModelNeuron/LowPass.H Class declarations for temporal
   low-pass filter implemented as first-order differential with a
   possible firing rate function  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/LowPass.H $

#ifndef MODELNEURON_LOWPASS_H_DEFINED
#define MODELNEURON_LOWPASS_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/SimUnit.H"

#include<cmath>

namespace nsu
{
  // ######################################################################
  // ! A set of policies for the integration method to use in our
  // lowpass filters. The equation to be solved is:
  //
  // dg/dt = (-g + H + I) / tau;
  // or
  // dg/dt = (-g + H) / tau + I;
  //
  // where g is the state variable, tau is a decay factor, H is the
  // steady state and I is the input.

  //All integration policy classes should implement three functions:
  //
  // MyPolicy(const double& timestep, const double& steadystate, 
  //          const double& tau);
  // const double update(const double& in, const double& g) const;
  // const double update_nodecay_input(const double& in, const double& g) const;
  //
  // where MyPolicy is a constructor that takes the time step in ms, the
  // steady state value and the decay factor (in ms) as arguments. The
  // 'update' and 'update_nodecay_input' functions takes the current
  // input and value of the state variable to compute and return the
  // state at the next time step.
  //
  // setting timestep <= 0 should turn on a default time step
  // ######################################################################
  // ! policy for the forward Euler method
  // ######################################################################
  class LowPassFwdEuler
  { 
    public:
      static const SimUnit::RateType RateType = SimUnit::NORMAL;

      LowPassFwdEuler(const double& timestep, const double& h, const double& tau) : 
          H(h), Tau(tau), TimeStep((timestep <= 0.0) ? tau / 5 : timestep) { };
  
      ~LowPassFwdEuler() { };
  
      //! update using fwd euler
      void update(const double& in, double& g, const double& timestep) const
      { g = g + timestep * (-1.0 * g + in + H) / Tau; };

      //! update without applying the decay factor to the input using fwd euler
      void update_nodecay_input(const double& in, double& g, const double& timestep) const
      { g = g + timestep * (-1.0 * g + H) / Tau + in; };

      //!return integrator timestep
      const double getIntegratorTimeStep() 
      { return TimeStep; };

      //timestep in ms, steady state value, kinetic time constant (msecs) 
      double H, Tau;

    private:
      const double TimeStep;
  };

  // ######################################################################
  // ! policy for the exponential Euler method
  // ######################################################################
  class LowPassExpEuler 
  {
    public:
      static const SimUnit::RateType RateType = SimUnit::STRICT;
  
      LowPassExpEuler(const double& timestep, const double& h, const double& tau) : 
          H(h), Tau(tau), TimeStep((timestep <= 0.0) ? tau / 2 : timestep), B(1.0/Tau), E(exp(-1.0 * B * TimeStep)), E1(1 - E) { };
  
      ~LowPassExpEuler() { };
  
      //! update using exp euler
      void update(const double& in, double& g, const double& timestep) const
      { 
        double A = (H + in) / Tau;
        g = g * E + (A / B) * E1;  
      };

      //! update without applying the decay factor to the input using fwd euler
      void update_nodecay_input(const double& in, double& g, const double& timestep) const
      { 
        double A = H / Tau + in;
        g =  g * E + (A / B) * E1;  
      };

      //!return integrator timestep
      const double getIntegratorTimeStep() 
      { return TimeStep; };

      //timestep in ms, steady state value, kinetic time constant (msecs) 
      double H, Tau;                 
      
    private:
      double TimeStep;
      double B;   //steady state + input / tau
      double E;   //exp (-1 * B * dt)
      double E1;  //1 - E
  };

  // ######################################################################
  // ! Functors that can serve as firing rate functions. Operator() must
  // take a double and return a double.
  // ######################################################################
  namespace RateFunctions
  {
    struct RateFunctionBase
    {
        RateFunctionBase() : itsState(0.0) { }
        void setState(const double& state) { itsState = state; };

      protected:
        double itsState;
    };

    // ######################################################################
    // an empty rate function
    // ######################################################################
    struct EmptyFunction : public RateFunctionBase
    {
        //constructor
        EmptyFunction() : 
            RateFunctionBase() { };
        //destructor
        ~EmptyFunction() { };
        //get the name
        static const char* name() { return ""; };
        //overload function call operator
        const double operator()(const double& in) const { return in; }
    };
  
    // ######################################################################
    // full wave rectification firing rate function:
    // 
    // f(x) = -x : x < 0
    // f(x) =  x : x > 0
    // ######################################################################
    struct FullRectifyFunction : public RateFunctionBase
    {
        //constructor
        FullRectifyFunction() : 
            RateFunctionBase() { };
        //destructor
        ~FullRectifyFunction() { };
        //get the name
        static const char* name() { return "FullRectify"; };
        //overload function call operator
        const double operator()(const double& in) const
        {
          return (in >= 0.0) ? in : -1.0 * in;
        }
    };

    // ######################################################################
    // half wave rectification firing rate function:
    // 
    // f(x) = 0 : x <= thresh
    // f(x) = x : x > thresh 
    // ######################################################################
    struct RectifyFunction : public RateFunctionBase
    {
        //constructor
        RectifyFunction(const double& thresh) : 
            RateFunctionBase(), Thresh(thresh) { };
        //destructor
        ~RectifyFunction() { };
        //get the name
        static const char* name() { return "Rectify"; };
        //overload function call operator
        const double operator()(const double& in) const
        {
          return (in > Thresh) ? in : 0.0;
        }
        //paramters
        double Thresh; 
    };
  
    // ######################################################################
    // half wave rectification with clipping. Make thresh and max the same 
    // values to create a step function:
    // 0.0 in <= thresh 
    // in if thresh < in < max
    // max if in >= max
    // ######################################################################
    struct MaxRectFunction : public RateFunctionBase
    {
        //constructor
        MaxRectFunction(const double& thresh, const double& max) : 
            RateFunctionBase(), Thresh(thresh), Max(max) { };
        //destructor
        ~MaxRectFunction() { };
        //get the name
        static const char* name() { return "MaxRect"; };
        //overload function call operator
        const double operator()(const double& in) const
        {
          if (in <= Thresh)
            return 0.0;
          else if (in > Max)
            return Max;
          else
            return in;
        }
        //paramters
        double Thresh, Max;
    };

    // ######################################################################
    // Step Function.
    // 0.0 in <= thresh 
    // max if in > thresh
    // ######################################################################
    struct StepFunction : public RateFunctionBase
    {
        //constructor
        StepFunction(const double& thresh, const double& max) : 
            RateFunctionBase(), Thresh(thresh), Max(max) { };
        //destructor
        ~StepFunction() { };
        //get the name
        static const char* name() { return "Step"; };
        //overload function call operator
        const double operator()(const double& in) const
        {
          if (in <= Thresh)
            return 0.0;
          else
            return Max;
        }
        //paramters
        double Thresh, Max;
    };
  
    // ######################################################################
    // A version of the lowpass filter module with a firing threshold
    // and rate function described by: 
    //     R = 0; if I < theta 
    //     R = ln(I/theta) if I > theta 
    //
    // Where R is the output rate, I is the
    // integrators current value, and theta is a threshold.
    //
    // From  Durstewitz et al, 2000, Nature Reviews
    // ######################################################################
    struct LogThreshFunction : public RateFunctionBase
    {
        //constructor
        LogThreshFunction(const double& theta) : 
            RateFunctionBase(), Thresh(theta) { };
        //destructor
        ~LogThreshFunction() { };
        //get the name
        static const char* name() { return "LogThresh"; };
        //overload function call operator
        const double operator()(const double& in) const 
        {
          if (in < Thresh)
            return 0.0;
          else
            return log(in / Thresh); 
        }
        //paramters
        double Thresh;
    };
  
    // ######################################################################
    // A version of the lowpass filter module with a sigmoidal firing 
    // rate function:
    //
    // f(x) = 1 / (1 + exp(-beta(u - u_not)))
    //
    // From Erlhagen & Bicho, 2006, J. Neural Engineering
    // ######################################################################
    struct SigmoidFunction : public RateFunctionBase
    {
        //constructor
        SigmoidFunction(const double& thresh, const double& slope) : 
            Thresh(thresh), Slope(slope) { };
        //destructor
        ~SigmoidFunction() { };
        //get the name
        static const char* name() { return "Sigmoid"; };
        //overload function call operator
        const double operator()(const double& in) const
        {
          return 1 / ( 1 + exp(-1.0 * Slope * (in - Thresh)) ); 
        }
        //paramters
        double Thresh, Slope;
    };
  }

  // ######################################################################
  // lowpass filter: Defines a low pass filter with simple first order
  // dynamics:
  //
  // tau * dg/dt = -g + H + I;
  //
  // Users can choose between one of the solver types and rate functors
  // defined above. See solver and functor definitions for interface
  // requirements. Some rate functors constructors take parameters and
  // so three constructors are provided that take 0, 1 or 2 extra
  // parameters, which are passed to the rate functors. The parameters
  // will be assigned in the same order as in the functor
  // definition. Several typedefs are defined below for convenience.
  // ######################################################################
  template <class RateFunc = RateFunctions::EmptyFunction, 
            class IntType = LowPassExpEuler>
  class LowPassFilter: public IntType, public SimUnit
  {
    public:  
      //! Constructor with default params
      LowPassFilter(const double& tau, //time constant msecs
                    const double& h, //steady state
                    const SimTime& timestep,//simulaton timestep
                    const std::string& name = "Lowpass", 
                    const std::string& units = "pA");
  
      //! Constructor with default params, overload for RateFuncs with 1 param
      LowPassFilter(const double& tau, //time constant msecs
                    const double& h, //steady state
                    const double& param1,//param 1 for the rate function
                    const SimTime& timestep, //simulaton timestep,
                    const std::string& name = "Lowpass", 
                    const std::string& units = "pA");
  
      //! Constructor with default params, overload for RateFuncs with 2 param
      LowPassFilter(const double& tau, //time constant msecs
                    const double& h, //steady state
                    const double& param1,//param 1 for the rate function
                    const double& param2,//param 2 for the rate function
                    const SimTime& timestep, //simulaton timestep
                    const std::string& name = "Lowpass", 
                    const std::string& units = "pA");
  
      //! destructor
      ~LowPassFilter() { };
  
      //! get the display output
      const double getDisplayOutput() const;

      //! get steady state
      const double getSteadyState() const;

      //!get the state variable
      const double getState() const;

    protected:
      RateFunc firingRate; // our firing rate functor

      //! integrate a time step using exponential euler
      const double doIntegrate(const SimTime& dt, 
                               const double& ine, const double& inh);
    private:
      //!initialize or reset internal variables
      void doInit();
  
      //! clone this object
      LowPassFilter<RateFunc, IntType>* doClone() const;

      double itsG; // internal state
  };

  // ######################################################################
  // typedefs
  // ######################################################################
  typedef 
  LowPassFilter<RateFunctions::EmptyFunction, LowPassExpEuler> LowPass;
  typedef 
  LowPassFilter<RateFunctions::RectifyFunction, LowPassExpEuler> LowPassRectify;
  typedef 
  LowPassFilter<RateFunctions::FullRectifyFunction, LowPassExpEuler> LowPassFullRectify;
  typedef 
  LowPassFilter<RateFunctions::StepFunction, LowPassExpEuler> LowPassStep;
  typedef 
  LowPassFilter<RateFunctions::MaxRectFunction, LowPassExpEuler> LowPassMaxRect;
  typedef 
  LowPassFilter<RateFunctions::LogThreshFunction, LowPassExpEuler> LowPassLog;
  typedef 
  LowPassFilter<RateFunctions::SigmoidFunction, LowPassExpEuler> LowPassSigmoid;

  // ######################################################################
  // register our objects
  // ######################################################################
  namespace 
  {
    typedef SimUnit::Factory LPFactory;
    typedef SimUnit::Creator LPCreator;
    //define creation functions
    struct RegisterLowPassFilter
    {
        RegisterLowPassFilter()
        {
          const double tau = 50.0;
          const SimTime time = SimTime::MSECS(0.0);//so auto time us used. 
          LPFactory::instance().add("LowPass", 
                                    LPCreator::make<LowPass>(tau, -0.5, time));
          LPFactory::instance().add("LowPassRectify", 
                                    LPCreator::make<LowPassRectify>(tau, -0.5, 0.0, time));
          LPFactory::instance().add("LowPassFullRectify", 
                                    LPCreator::make<LowPassFullRectify>(tau, 0.0, time));
          LPFactory::instance().add("LowPassStep", 
                                    LPCreator::make<LowPassStep>(tau, -0.5, 0.0, 1.0, time));
          LPFactory::instance().add("LowPassMaxRect", 
                                    LPCreator::make<LowPassMaxRect>(tau, -0.5, 0.0, 1.0, time));
          LPFactory::instance().add("LowPassLog", 
                                    LPCreator::make<LowPassLog>(tau, 0.0, 0.1, time));
          LPFactory::instance().add("LowPassSigmoid",
                                    LPCreator::make<LowPassSigmoid>(tau, -0.5, 0.55, 12.0, time));
        }
    };
    static RegisterLowPassFilter registerlpf;  
  }

  // ######################################################################
  // ##### implementation for LowPassFilter
  // ######################################################################
  // default case rate function takes 0 params
  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowPassFilter<RateFunc, IntType>::LowPassFilter(const double& tau, 
                                                  const double& h,
                                                  const SimTime& timestep,
                                                  const std::string& name, 
                                                  const std::string& units) : 
      IntType(timestep.msecs(), h, tau), 
      SimUnit(SimTime::MSECS(IntType::getIntegratorTimeStep()), IntType::RateType, name + ":" + RateFunc::name(), units), 
      firingRate(), itsG(h) { };

  // ######################################################################
  // default case rate function takes 1 params
  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowPassFilter<RateFunc, IntType>::LowPassFilter(const double& tau, 
                                                  const double& h,
                                                  const double& param1, 
                                                  const SimTime& timestep,
                                                  const std::string& name, 
                                                  const std::string& units) : 
      IntType(timestep.msecs(), h, tau), 
      SimUnit(SimTime::MSECS(IntType::getIntegratorTimeStep()), IntType::RateType, name + ":" + RateFunc::name(), units), 
      firingRate(param1), itsG(h) { };

  // ######################################################################
  // default case rate function takes 2 params
  // ######################################################################
  template <class RateFunc, class IntType> inline
  LowPassFilter<RateFunc, IntType>::LowPassFilter(const double& tau, 
                                                  const double& h,
                                                  const double& param1, 
                                                  const double& param2, 
                                                  const SimTime& timestep,
                                                  const std::string& name, 
                                                  const std::string& units) : 
      IntType(timestep.msecs(), h, tau), 
      SimUnit(SimTime::MSECS(IntType::getIntegratorTimeStep()), IntType::RateType, name + ":" + RateFunc::name(), units), 
      firingRate(param1, param2), itsG(h) { };

  // ######################################################################
  template <class RateFunc, class IntType> inline 
  const double LowPassFilter<RateFunc, IntType>::getDisplayOutput() const
  {
    return itsG;
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline 
  const double LowPassFilter<RateFunc, IntType>::getSteadyState() const
  {
    return IntType::H;
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline 
  const double LowPassFilter<RateFunc, IntType>::getState() const
  {
    return itsG;
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline
  const double LowPassFilter<RateFunc, IntType>::doIntegrate(const SimTime& dt, 
                                                             const double& ine,
                                                             const double& inh)
  {
    IntType::update(ine + inh, itsG, dt.msecs());//update inherited from template parameter
    return firingRate(itsG);
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline
  void LowPassFilter<RateFunc, IntType>::doInit()
  {
    itsG = IntType::H;
  }

  // ######################################################################
  template <class RateFunc, class IntType> inline 
  LowPassFilter<RateFunc, IntType>* 
  LowPassFilter<RateFunc, IntType>::doClone() const
  {
    return new LowPassFilter<RateFunc, IntType>(*this);
  }
}
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
