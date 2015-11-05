/*!@file ModelNeuron/SimUnit.H abstract classes for a neural
   simulation module, which is anything that can evolve its time
   state, recieve input and give output. A SimUnit can also
   have sub-modules. */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/SimUnit.H$

#ifndef MODELNEURON_SIMUNIT_H_DEFINED
#define MODELNEURON_SIMUNIT_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Util/SimTime.H"
#include "Util/log.H"
#include "ModelNeuron/NeuralDecoder.H"
#include <typeinfo>

namespace nsu
{
  // ######################################################################
  // ! An abstract interface for a simulation unit. Classes should not
  // derive from this class directly, but rather from
  // SimUnitDerived, defined below. A simulation module is a
  // computational unit that takes input, gives output, and evolves its
  // internal state. Users must override doIntegrate(time, excite,
  // inhibit) to implement the right-hand-side of their differential
  // equation(s)(see Rectify.H for the simplest example, and LowPass.H
  // for a more interesting one), and doInit() to perform any derived
  // class specific initialization. SimUnits can have other
  // SimUnits as sub-modules however for this abstract class
  // there is no implementation of this functionality (see
  // NeuralColumn.H or IZNeuron.H for SimUnits that implement
  // sub-modules). Modules and sub-modules need not be simulated at the
  // same time step. SimUnits can also have decoders on the
  // input or output to transform the signal before or after processing
  // (see NeuralDecoder.H). When evolve(time) is called the module will
  // check to see if it needs to be integrated, and if so integrate
  // itself to the requested 'world' time. Then any submodules will be
  // integrated to the super modules current time. Input to a sub module
  // should only come from its super module to ensure that proper
  // scaling is performed. Each call to input will segregate positive
  // (excitatory) and negative (inhibitory) input and add the input to
  // the current totals. Upon the next call to evolve the inputs will be
  // pre-processed and rescaled if oversampling is detected (multiple
  // calls to evolve(time) without an internal time change), before
  // being passed to the user defined doIntegrate function. After a call
  // to evolve(time), the modules internal time will be simulated up to
  // (or close, depending on the RateType, see below) 'time', itsInput
  // will be set to 0.0, and a call to getOutput() will represent the
  // output of the system at (or close to) 'time'.
  // ######################################################################
  class SimUnit
  {
    public:
      //a typdef used to register classes
      typedef CreateFunctor<SimUnit, ParamList<SimTime> > Creator;
      typedef GenericFactory<SimUnit, std::string, Creator> Factory;
  
      //For sampling rate control options
      enum RateType
      {
        NORMAL, //Use the sampling rate that is closest to the requsted
        //sampling rate (time step) and will result in an integer
        //number of integrations after each call to evolve()
      
        STRICT, //enforce that each call to evolve results in an integer
        //number of integrations with exactly the requested time
        //step. This may result in a module that does not evolve up to
        //the current time step on every call to evolve(), but will be
        //less than a time step away.
      
        OPTIM //choose the time step automatically if available
      };
  
      /*! Constructor with default params
          @param timeStep is the integration time step, in milliseconds 
          @param name is the name of the module
          @param units are the units in SI
      */
      SimUnit(const SimTime& timestep = SimTime::MSECS(1.0), 
              const RateType ratetype = NORMAL,
              const std::string& name = "", 
              const std::string& units = "");
  
      //! virtual destructor for propper inheritance
      virtual ~SimUnit();
  
      // ######################################################################
      // i/o and simulation functions
      // ######################################################################  
  
      //!input to the computation unit
      void input(const double& in);

      //!input excitation to the computation unit
      void inputExc(const double& in);

      //!input inhibition to the computation unit
      void inputInh(const double& in);
  
      //!set the voltage (or other state value)
      virtual void setV(const double& in);
  
      //!get the output since the last call to evolve
      const double getOutput() const;
  
      //!get an output for display
      virtual const double getDisplayOutput() const;
  
      //!integrate to time t, updating any decoders and internal variables
      void evolve(const SimTime& t);
  
      // ######################################################################
      // sub-module functions
      // ######################################################################  

      //!get the number of sub units (default implementation returns 0)
      virtual const uint numSubs() const;
  
      //!Returns a reference to the subcomponent. You should check with
      //!numSubs before calling (default implementation produces LFATAL)
      //!and should use very carefully to ensure the object will not be
      //!destroyed by the owner while in use!
      virtual const SimUnit& getSub(const uint i) const;

      // ######################################################################
      // change state functions
      // ######################################################################  

      //! reset the object to post construction then call
      //! doInit() to perform any subclass specific initialization.
      void initialize();
  
      //!hook up a decoder of desired
      void setDecoderPre(const NeuralDecoder& nd);
  
      //!hook up a decoder of desired
      void setDecoderPost(const NeuralDecoder& nd);
  
      //!set the modules name
      void setName(const std::string& name);
  
      //!set the  output units in SI
      void setUnits(const std::string& units);

      //!set the initial output
      void setOutput(const double& val, const bool recursive = true);

      //!set the current time
      void setTime(const SimTime& time, const bool recursive = true);
  
      // ######################################################################
      // get state functions
      // ######################################################################  

      //! return our internal time
      const SimTime& getTime() const;
  
      //! return our internal time step
      const SimTime& getTimeStep() const;
  
      //! return the name of this module
      const std::string getName() const;
  
      //! return the units of this modules output
      const std::string getUnits() const;
  
      //!return this modules rate type
      const RateType getRateType() const;
  
      //!clone the object, like a virtual copy constructor. This function
      //!implemented in NeurSimModuleDerived, which other objects should
      //!derive from.
      SimUnit* clone() const;

      //!Returns a reference to the subcomponent. You should check with
      //!numSubs before calling (default implementation produces LFATAL)
      //!and should use very carefully to ensure the object will not be
      //!destroyed by the owner while in use!
      virtual SimUnit& editSub(const uint i);
  
    protected:
      //!restrict copy constructor and assignment, use clone instead so
      //!derived classes create the correct objects and we avoid slicing
      SimUnit(const SimUnit& rhs);
      SimUnit& operator=(const SimUnit& rhs);  
  
    private:  
      //!overide to set the optimal time step 'simtime' for the unit,
      //!called every evolution  if the rate type is set to OPTIM.
      virtual void setOptimTimeStep(SimTime& simtime) const;
  
      //! perform any user defined integration steps
      virtual const double doIntegrate(const SimTime& dt, const double& exc, const double& inh) = 0; 
  
      //! perform subclass specific initialization
      virtual void doInit();  

      //!actually do the cloning work
      virtual SimUnit* doClone() const = 0;
  
      double itsInpExc; // to hold our excitatory input till next integration
      double itsInpInh; // to hold our inhibitory input till next integration
      double itsOutput;  // our output after the last integration time step
      std::string itsName, itsUnits; //name of module and SI output units
      SimTime itsTimeStep;  // time step to use for difference equations (in msecs)
      SimTime itsT;         // time of last integration
      int itsSampScale; // the number of times evolve gets called but does not need to integrate
      RateType itsRateType; //the sampling rate restrictions, if any
  
      //hooks to decoder
      NeuralDecoder *itsDecoderPreE, *itsDecoderPreI,  *itsDecoderPost;
  };

  // ######################################################################
  //!A class to derive from to create new SimUnits. New
  //SimUnit derived types can derive from this class to
  //inherit the doClone() function if desired. 
  /*
    Programmer Note: This class uses the 'quriously recursive template
    pattern' to simplify creation of new classes for the programmer. As
    such, to create a new simulation module, classes should adhear the
    following convention:
  
    class mymodule : public SimUnitDerived<mymodule>
    {
    mymodule(//params//) : SimUnit<mymodule>(//params//) { };
    //...rest of functions
    }
  */
  // ######################################################################
  template <class Derived>
  class SimUnitDerived : public SimUnit
  {
    protected:
      SimUnitDerived(const SimTime& timestep = SimTime::MSECS(1.0), 
                     const RateType ratetype = NORMAL,
                     const std::string& name = "", 
                     const std::string& units = "") : 
          SimUnit(timestep, ratetype, name, units) { };
  
      ~SimUnitDerived() { };
  
    private:
      SimUnit* doClone() const 
      { return new Derived(dynamic_cast<const Derived&>(*this)); };               
  };

  // ######################################################################
  //! implementation of inlined SimUnit functions
  // ######################################################################
  inline
  SimUnit::SimUnit(const SimTime& timestep,
                   const RateType ratetype,
                   const std::string& name, 
                   const std::string& units) : 
      itsInpExc(0.0), itsInpInh(0.0), itsOutput(0.0), itsName(name), 
      itsUnits(units), itsTimeStep(timestep), itsT(SimTime::ZERO()), itsSampScale(1),
      itsRateType(ratetype), itsDecoderPreE(NULL), itsDecoderPreI(NULL),
      itsDecoderPost(NULL)
  { 
  }

  // ######################################################################
  inline
  SimUnit::~SimUnit()
  {
    delete itsDecoderPreE;
    delete itsDecoderPreI;
    delete itsDecoderPost;
  }

  // ######################################################################
  inline
  void SimUnit::input(const double& input)
  {
    if (input > 0.0)
      itsInpExc += input; 
    else 
      itsInpInh += input;
  }

  // ######################################################################
  inline
  void SimUnit::inputExc(const double& input)
  {
    itsInpExc += input; 
  }

  // ######################################################################
  inline
  void SimUnit::inputInh(const double& input)
  {
    itsInpInh += input;
  }

  // ######################################################################
  inline
  void SimUnit::setV(const double& input)
  { }

  // ######################################################################
  inline
  const double SimUnit::getOutput() const
  {
    return itsOutput; 
  }

  // ######################################################################
  inline
  const double SimUnit::getDisplayOutput() const
  {
    return itsOutput; 
  }

  // ######################################################################
  inline 
  void SimUnit::evolve(const SimTime& t)
  {
    // we run our difference equations with a time step of itsTimeStep;
    // let's here figure out how many iterations we will need to go from
    // itsT to t in an equal number of steps. if itsRateType=NORMAL each
    // step is as close to itsTimeStep as possible so that we end up at
    // time t after iterating for an integer number of time steps. If
    // itsRateType=STRICT the we enfoce that the time step is exactly
    // itsTimeStep, which may result in not ending up exactly (but <
    // itsTimestep away) at t.
    const SimTime interval(t - itsT);
    const int64 nsteps = interval.nsecs() / itsTimeStep.nsecs();
    const int steps = (int)nsteps;  
    SimTime currTimeStep;

    if (steps <= 0)
      ++itsSampScale;//keep track of how many times we don't integrate
    else
    {
      //set our sampling rate
      switch (itsRateType) 
      {
      case NORMAL:
        currTimeStep = SimTime::NSECS(interval.nsecs() / nsteps);  
        break;
      case OPTIM:
        currTimeStep = itsTimeStep;
        setOptimTimeStep(currTimeStep);
        break;
      case STRICT:
        currTimeStep = itsTimeStep;
        break;
      }
     
      //adjust the input by the number of times it was oversampled
      if (itsSampScale > 1)
      {
        itsInpExc /= itsSampScale;
        itsInpInh /= itsSampScale;
        itsSampScale = 1;
      }
      
      //loop over our difference equations
      for (int ii = 0; ii < steps; ++ii)
      {      
        double inpe = itsInpExc;
        double inpi = itsInpInh;
          
        //pass our latest input through the decoder if one exists
        if (itsDecoderPreE != NULL)
        {
          itsDecoderPreE->push(inpe);
          itsDecoderPreI->push(inpi);
          inpe = itsDecoderPreE->getOutput();
          inpi = itsDecoderPreI->getOutput();
        }
          
        itsT += currTimeStep;//update our current time
        itsOutput = doIntegrate(currTimeStep, inpe, inpi);//update internals
          
        //pass our newest output through a decoder if one is set
        if (itsDecoderPost != NULL)
        {
          itsDecoderPost->push(itsOutput);
          itsOutput = itsDecoderPost->getOutput();
        }
      }

      //reset our input after we evolve to the current time step
      itsInpExc = 0.0;
      itsInpInh = 0.0;
    }
  }

  // ######################################################################
  inline
  const uint SimUnit::numSubs() const 
  {
    return 0;
  }

  // ######################################################################
  inline 
  const SimUnit& SimUnit::getSub(const uint i) const
  {
    LFATAL("Not implemented for this class. Make sure to check the size with "
           "numSubs before calling getSub");
    return *this;//this will never execute
  }

  // ######################################################################
  inline 
  SimUnit& SimUnit::editSub(const uint i)
  {
    LFATAL("Not implemented for this class. Make sure to check the size with "
           "numSubs before calling getSub");
    return *this;//this will never execute
  }

  // ######################################################################
  inline 
  void SimUnit::initialize()
  {
    itsInpExc = 0.0;
    itsInpInh = 0.0;
    itsOutput = 0.0;
    itsT = SimTime(SimTime::ZERO());
    itsSampScale = 1;
    itsDecoderPreE->reset();
    itsDecoderPreI->reset();
    itsDecoderPost->reset();
    doInit();
  }

  // ######################################################################
  inline
  void SimUnit::setDecoderPre(const NeuralDecoder& nd)
  {
    delete itsDecoderPreE;
    delete itsDecoderPreI;
    itsDecoderPreE = nd.clone();
    itsDecoderPreI = nd.clone();
  }

  // ######################################################################
  inline
  void SimUnit::setDecoderPost(const NeuralDecoder& nd)
  {
    delete itsDecoderPost;
    itsDecoderPost = nd.clone();
  }

  // ######################################################################
  inline
  void SimUnit::setName(const std::string& name)
  {
    itsName = name;
  }

  // ######################################################################
  inline
  void SimUnit::setUnits(const std::string& units)
  {
    itsUnits = units;
  }

  // ######################################################################
  inline
  void SimUnit::setOutput(const double& val, const bool recursive)
  {
    itsOutput = val;
    if (recursive)
      for (uint ii = 0; ii < numSubs(); ++ii)
        editSub(ii).setOutput(val, true);
  }

  // ######################################################################
  inline
  void SimUnit::setTime(const SimTime& time, const bool recursive)
  {
    itsT = time;
    if (recursive)
      for (uint ii = 0; ii < numSubs(); ++ii)
        editSub(ii).setTime(time, true);
  }

  // ######################################################################
  inline 
  const SimTime& SimUnit::getTime() const 
  { 
    return itsT; 
  };

  // ######################################################################
  inline 
  const SimTime& SimUnit::getTimeStep() const 
  { 
    return itsTimeStep; 
  }; 

  // ######################################################################
  inline
  const std::string SimUnit::getName() const
  {
    return itsName;
  }

  // ######################################################################
  inline
  const std::string SimUnit::getUnits() const
  {
    return itsUnits;
  }

  // ######################################################################
  inline
  const SimUnit::RateType SimUnit::getRateType() const
  {
    return itsRateType;
  }

  // ######################################################################
  inline
  SimUnit::SimUnit(const SimUnit& rhs)  :
      itsInpExc(rhs.itsInpExc), itsInpInh(rhs.itsInpInh), itsOutput(rhs.itsOutput),
      itsName(rhs.itsName), itsUnits(rhs.itsUnits), itsTimeStep(rhs.itsTimeStep), 
      itsT(rhs.itsT), itsSampScale(rhs.itsSampScale), itsRateType(rhs.itsRateType),
      itsDecoderPreE(NULL), itsDecoderPreI(NULL), itsDecoderPost(NULL)
  {
    if (rhs.itsDecoderPreE)
    {
      itsDecoderPreE = rhs.itsDecoderPreE->clone();
      itsDecoderPreI = rhs.itsDecoderPreI->clone();
    }
  
    if (rhs.itsDecoderPost)
      itsDecoderPost = rhs.itsDecoderPost->clone();
  }

  // ######################################################################
  inline 
  SimUnit& SimUnit::operator=(const SimUnit& rhs)
  {
    if (this != &rhs)
    {
      itsInpExc = rhs.itsInpExc;
      itsInpInh = rhs.itsInpInh;
      itsOutput = rhs.itsOutput;
      itsName = rhs.itsName;
      itsUnits = rhs.itsUnits;     
      itsTimeStep = rhs.itsTimeStep; 
      itsT = rhs.itsT;
      itsSampScale = rhs.itsSampScale;
      itsRateType = rhs.itsRateType;
      delete itsDecoderPreE;
      delete itsDecoderPreI;
      delete itsDecoderPost;

      if (rhs.itsDecoderPreE)
      {
        itsDecoderPreE = rhs.itsDecoderPreE->clone();
        itsDecoderPreI = rhs.itsDecoderPreI->clone();
      }
      else
      {
        itsDecoderPreE = NULL;
        itsDecoderPreI = NULL;
      }
      
      itsDecoderPost = (rhs.itsDecoderPost) ?
        rhs.itsDecoderPost->clone() :
        NULL;
    }
    return *this;
  }

  // ######################################################################
  inline
  SimUnit* SimUnit::clone() const
  {
    SimUnit* d = doClone();
    if ( typeid(*d) != typeid(*this) )
      LFATAL("DoClone incorrectly overridden" );
    return d;
  }

  // ######################################################################
  inline
  void SimUnit::setOptimTimeStep(SimTime& simtime) const
  {
    LFATAL("Not Implemented for this class");
  }

  // ######################################################################
  inline
  void SimUnit::doInit()
  {
  }
}

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
