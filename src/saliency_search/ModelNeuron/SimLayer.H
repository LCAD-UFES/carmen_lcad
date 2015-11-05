/*!@file ModelNeuron/SimLayer.H Class declarations for partial 
implementations of SimStruture for 2-D simulation units.  */

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
// Primary maintainer for this file: David J Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/ModelNeuron/SimLayer.H $

#ifndef MODELNEURON_SIMLAYER_H_DEFINED
#define MODELNEURON_SIMLAYER_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Util/SimTime.H"
#include "ModelNeuron/SimUnit.H"
#include "ModelNeuron/SimStructure.H"
#include "ModelNeuron/NeuralSimUtils.H"

namespace nsu
{
  class Location;
  // ######################################################################
  /*! This partial implementation of SimStructure represents a container for
      SimUnits (See SimUnit.H) that takes 2D input, gives 2D output, and evolves
      the internal states of its SimUnits. This class may have sub structures, but
      this implementation only allows 1 input and output (ie, trying to input or 
      get output to a layer > 0 will fail). See Structure.H for a class which 
      implements sub structures such that they can be input to our output from.
   
      After a call to evolve(time), the modules internal time will be simulated up
      to 'time' and a call to getOutput() will represent the output of the system
      at 'time'. The logic is the same as SimUnit (see SimUnit.H). Sub structures
      are not required to be simulated at the same time step. 
  */
  //######################################################################
  class SimLayer : public SimStructure
  {
    public:  
      //! constructor
      SimLayer(const SimTime& timestep, const uint width, const uint height,
               const std::string name = "", const std::string units = "");
  
      //!destructor 
      virtual ~SimLayer() { };
  
      // ######################################################################
      // i/o and simulation functions
      // ######################################################################  

      //!set the input
      void input(const Image<double>& in, const int pos = SimStructure::DEFAULT);

      //!set the excitatory input
      void inputExc(const Image<double>& in, const int pos = SimStructure::DEFAULT);

      //!set the inhibitory input
      void inputInh(const Image<double>& in, const int pos = SimStructure::DEFAULT);
  
      //!get the current output 
      Image<double> getOutput(const int pos = SimStructure::DEFAULT) const;

      //!evolve up to specified time
      void evolve(const SimTime& simtime);

      // ######################################################################
      // get/set unit functions
      // ######################################################################  

      //!set the type of neural simulation module to use
      virtual void setModule(const SimUnit& mod) = 0;
  
      //!set the type of neural simulation module to use, at a specific
      //!position 
      virtual void setModule(const SimUnit& mod, const Location& pos) = 0;

      // ######################################################################
      // change state functions
      // ######################################################################  

      //reset all counters, output and call doInit() to reset all subclasses
      void initialize();

      //set the time
      void setTime(const SimTime& time, const bool recurse = true);

      // ######################################################################
      // get state functions
      // ######################################################################  

      //!get current time
      const SimTime& getTime() const;


      //!get simulation time step
      const SimTime& getTimeStep() const;

      //!clone the module
      virtual SimLayer* clone() const = 0;

    protected:
      //protect copy and asignment, use clone
      SimLayer(const SimLayer& nlc);
      SimLayer& operator=(const SimLayer& nlc);

      //!set the time step
      void setTimeStep(const SimTime& timeStep);

      //the output
      Image<double>  itsOutput;

    private:
      //!integrate one time step
      virtual void doIntegrate(const SimTime& dt, const Image<double>& inpe, const Image<double>& inpi) = 0;
  
      //!an initializer
      virtual void doInit() = 0;
  
      SimTime itsTimeStep, itsT;
      uint itsSampScale;
      Image<double> itsInpExc, itsInpInh;
  };

  // ######################################################################
  //!A class to derive from to create new SimLayers. New
  //SimLayer derived types can derive from this class to
  //inherit the doClone() function if desired. 
  /*
    Programmer Note: This class uses the 'quriously recursive template
    pattern' to simplify creation of new classes for the programmer. As
    such, to create a new simulation module, classes should adhear the
    following convention:
  
    class mymodule : public SimLayerDerived<mymodule>
    {
    mymodule(//params//) : SimLayer<mymodule>(//params//) { };
    //...rest of functions
    }
  */
  // ######################################################################
  template <class Derived>
  class SimLayerDerived : public SimLayer
  {
    protected:
      SimLayerDerived(const SimTime& timestep, const uint width, const uint height,
                      const std::string name = "", const std::string units = "") : 
          SimLayer(timestep, width, height, name, units) { };
  
      virtual ~SimLayerDerived() { };
  
    private:
      Derived* clone() const { return new Derived(dynamic_cast<const Derived&>(*this)); }
  };


  // ######################################################################
  // SimLayer implementation
  // ######################################################################
  inline
  SimLayer::SimLayer(const SimTime& timestep, 
                     const uint width, 
                     const uint height,
                     const std::string name, 
                     const std::string units) :
      SimStructure(width, height, name, units),
      itsOutput(width, height, ZEROS), itsTimeStep(timestep), 
      itsT(SimTime::ZERO()), itsSampScale(1), 
      itsInpExc(width, height, ZEROS), itsInpInh(width, height, ZEROS)
  { }

  // ######################################################################
  inline
  SimLayer::SimLayer(const SimLayer& rhs) :
      SimStructure(rhs),
      itsOutput(rhs.itsOutput), itsTimeStep(rhs.itsTimeStep), itsT(rhs.itsT), 
      itsSampScale(rhs.itsSampScale), itsInpExc(rhs.itsInpExc), itsInpInh(rhs.itsInpInh)
  { }

  // ######################################################################
  inline
  SimLayer& SimLayer::operator=(const SimLayer& rhs)
  { 
    if (this != &rhs)
    {
      SimStructure::operator=(rhs);
      
      itsOutput = rhs.itsOutput;
      itsTimeStep = rhs.itsTimeStep;
      itsT = rhs.itsT;
      itsSampScale = rhs.itsSampScale;
      itsInpExc = rhs.itsInpExc;
      itsInpInh = rhs.itsInpInh;
    }
    return *this;
  }

  // ######################################################################
  inline 
  void SimLayer::input(const Image<double>& inp, const int pos)
  {
    ASSERT(inp.getDims() == itsOutput.getDims()); ASSERT(pos < 1);
    SimStructure::splitExcInh(inp, itsInpExc, itsInpInh);
  }

  // ######################################################################
  inline 
  void SimLayer::inputExc(const Image<double>& inp, const int pos)
  {
    ASSERT(inp.getDims() == itsOutput.getDims()); ASSERT(pos < 1);
    itsInpExc += inp;
  }

  // ######################################################################
  inline 
  void SimLayer::inputInh(const Image<double>& inp, const int pos)
  {
    ASSERT(inp.getDims() == itsOutput.getDims()); ASSERT(pos < 1);
    itsInpInh += inp;
  }

  // ######################################################################
  inline 
  Image<double> SimLayer::getOutput(const int pos) const
  {
    ASSERT(pos < 1);
    return itsOutput;
  }

  // ######################################################################
  inline
  void SimLayer::evolve(const SimTime& t)
  {
    // we run our difference equations with a time step of itsTimeStep;
    // let's here figure out how many iterations we will need to go from
    // itsT to t in an equal number of steps. Each step will be as close
    // to itsTimeStep as possible so that we end up at time t after
    // iterating for an integer number of time steps. 
    const SimTime interval(t - itsT);
    const int64 nsteps = interval.nsecs() / itsTimeStep.nsecs();
    const int steps = (int)nsteps;  
    SimTime currTimeStep;

    if (steps <= 0)
      ++itsSampScale;//keep track of how many times we don't integrate
    else
    {
      //set our current time step 
      currTimeStep = SimTime::NSECS(interval.nsecs() / nsteps);  
      
      if (itsSampScale > 1)
      {      
        //rescale our inputs
        itsInpExc /= itsSampScale;
        itsInpInh /= itsSampScale;
        itsSampScale = 1;
      }
      
      for (int ii = 0; ii < steps; ++ii)
      {      
        //update our current time
        itsT += currTimeStep;

        //integrate our internals
        doIntegrate(currTimeStep, itsInpExc, itsInpInh);
      }

      //reset our input after we evolve to the current time step
      itsInpExc.clear(0.0);
      itsInpInh.clear(0.0);
    }
  }

  // ######################################################################
  inline
  void SimLayer::initialize()
  {
    itsT = SimTime::ZERO();
    itsSampScale = 1;
    itsInpExc.clear(0.0);
    itsInpInh.clear(0.0);
    itsOutput.clear(0.0);
    doInit();
  }

  // ######################################################################
  inline
  const SimTime& SimLayer::getTime() const
  {
    return itsT;
  }

  // ######################################################################
  inline
  void SimLayer::setTime(const SimTime& time, const bool recurse)
  {
    itsT = time;
  }

  // ######################################################################
  inline
  const SimTime& SimLayer::getTimeStep() const
  {
    return itsTimeStep;
  }

  // ######################################################################
  inline
  void SimLayer::setTimeStep(const SimTime& timestep)
  {
    itsTimeStep = timestep;
  }
}
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
