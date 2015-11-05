/*!@file ModelNeuron/SimStructure.H Class declarations for a
  structure module, a simulation unit that supports 2D image input and
  output. */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SimStructure.H $

#ifndef MODELNEURON_SIMSTRUCTURE_H_DEFINED
#define MODELNEURON_SIMSTRUCTURE_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Util/SimTime.H"
#include "Image/Image.H"
#include "ModelNeuron/Location.H"
#include "GenericUtils/GenericFactory.H"
#include "GenericUtils/CreateFunctor.H"
#include "Image/LowPassLpt.H"
#include "ModelNeuron/SimulationWorkQueue.H"

#include <typeinfo>

namespace nsu
{
  class SimUnit;
  // ######################################################################
  /* 
     SimStructure is an abstract container intended to serve as an interface
     to collections of SimUnit's with 2D input and output. Most classes will 
     derive from one of the partial implementations (see Structure.H and 
     SimLayer.H) or from one of the full implementations (see Layer.H, SC.H 
     and others for example).
  */
  //######################################################################
  class SimStructure
  {
    public:
      enum {DEFAULT = -1};
      
      //a typdef used to register classes
      typedef CreateFunctor<SimStructure, ParamList<BorderPolicy, SimTime, uint, uint> > Creator;
      typedef GenericFactory<SimStructure, std::string, Creator> Factory;
      
      //!constructor
      SimStructure(const uint width, const uint height, const std::string name = "", const std::string units = "");
      
      //!destructor 
      virtual ~SimStructure();
      
      // ######################################################################
      // i/o and simulation functions
      // ######################################################################  
      
      //!set the input
      virtual void input(const Image<double>& in, const int pos = DEFAULT) = 0;
      
      //!set the input
      virtual void inputExc(const Image<double>& in, const int pos = DEFAULT) = 0;

      //!set the input
      virtual void inputInh(const Image<double>& in, const int pos = DEFAULT) = 0;

      //!get the current output 
      virtual Image<double> getOutput(const int pos = DEFAULT) const = 0;
  
      //!get the current display output 
      virtual Image<double> getDisplayOutput(const int pos = DEFAULT) const;
  
      //!evolve up to specified time
      virtual void evolve(const SimTime& simtime) = 0;

      //!sets the number of threads to use
      void setNumThreads(const uint threads);

      //!get the number of simulation threads
      const uint getNumThreads() const;
      
      // ######################################################################
      // get/set unit functions
      // ######################################################################  
  
      //!get unit(s) at desired location
      virtual void getSimUnit(const Location& loc, std::vector<const SimUnit*>& units) = 0;
  
      //!edit unit(s) at the desired output position
      virtual void editSimUnit(const Location& loc, std::vector<SimUnit*>& units) = 0;
  
      // ######################################################################
      // sub-module functions
      // ######################################################################  

      //get the number of sub modules
      virtual const uint numSubs() const;
  
      //get a referene to a sub module const version
      virtual const SimStructure& getSub(const uint sub) const;  
  
      //get a referene to a sub module mutable version
      virtual SimStructure& editSub(const uint sub);  

      // ######################################################################
      // change state functions
      // ######################################################################  

      //!set units type
      void setName(const std::string& name);
  
      //!set units type
      void setUnits(const std::string& units);
  
      //reset all counters, output and call doInit() to reset all subclasses
      virtual void initialize() = 0;

      //!set simulation time step
      virtual void setTime(const SimTime& time, const bool recursive = true) = 0;  

      // ######################################################################
      // get state functions
      // ######################################################################  

      //!get name
      const std::string getName() const;

      //!get units
      const std::string getUnits() const;

      //!get output dimensions
      const Dims getOutDims() const;

      //!get the width
      const uint getOutWidth() const;

      //!get the height
      const uint getOutHeight() const;
  
      //!get the total output size
      const uint getOutSize() const;

      //!get simulation time step
      virtual const SimTime& getTime() const = 0;

      //!get simulation time step
      virtual const SimTime& getTimeStep() const = 0;
  
      //!clone the module
      virtual SimStructure* clone() const = 0;
  
    protected:
      //protect copy and asignment, use clone
      SimStructure(const SimStructure& nlc);
      SimStructure& operator=(const SimStructure& nlc);
      
      //split the positive and negative values of input into Exc and Inh
      void splitExcInh(const Image<double>& input, Image<double>& Exc, Image<double>& Inh);
      
      //! push a job for processing
      void process(std::function<void()>&& job);

      //! push jobs for processing
      void process(std::vector<std::function<void()> >&& jobs);
    
    private:
      std::string itsName, itsUnits;
      Dims itsOutDims;
      static SimulationWorkQueue itsWorkQueue;
  };

  // ######################################################################
  // A job for splitting images into pos and negative
  // ######################################################################
  class SplitJob
  {
    public:
      SplitJob(const Image<double>::const_iterator& in, const Image<double>::const_iterator& inend, const Image<double>::iterator& oute, const Image<double>::iterator& outi) :
          itsIn(in), itsInEnd(inend), itsOutE(oute), itsOutI(outi) { }
      void operator()() 
      {
        while (itsIn != itsInEnd)
        {
          if (*itsIn < 0.0)
            *itsOutI += *itsIn;
          else 
            *itsOutE += *itsIn;
          
          ++itsIn; ++itsOutE; ++itsOutI;
        }
      }
      
    private:
      Image<double>::const_iterator itsIn, itsInEnd;
      Image<double>::iterator itsOutE, itsOutI;
  };
  
  // ######################################################################
  // SimStructure implementation
  // ######################################################################
  //! constructor
  inline
  SimStructure::SimStructure(const uint width, const uint height, const std::string name, const std::string units)
      : itsName(name), itsUnits(units), itsOutDims(width, height)
  { }
  
  // ######################################################################
  inline
  SimStructure::~SimStructure()
  { }

  // ######################################################################
  inline
  Image<double> SimStructure::getDisplayOutput(const int pos) const
  {
    return getOutput(pos);
  }

  // ######################################################################
  inline
  void SimStructure::setNumThreads(const uint threads)
  {
    itsWorkQueue.resize(threads);
  }

  // ######################################################################
  inline
  const uint SimStructure::getNumThreads() const
  {
    return itsWorkQueue.numThreads();
  }

  // ######################################################################
  inline
  const uint SimStructure::numSubs() const
  {
    return 0;
  }

  // ######################################################################
  inline
  const SimStructure& SimStructure::getSub(const uint sub) const
  {
    LFATAL("No subcomponents, check numSubs before calling.");
    return *this;//will never execute
  }

  // ######################################################################
  inline
  SimStructure& SimStructure::editSub(const uint sub)
  {
    LFATAL("No subcomponents, check numSubs before calling.");
    return *this;//will never execute
  }

  // ######################################################################
  inline
  void SimStructure::setName(const std::string& name)
  {
    itsName = name;
  }

  // ######################################################################
  inline
  void SimStructure::setUnits(const std::string& units)
  {
    itsUnits = units;
  }

  // ######################################################################
  inline
  const std::string SimStructure::getName() const
  {
    return itsName;
  }

  // ######################################################################
  inline
  const std::string SimStructure::getUnits() const
  {
    return itsUnits;
  }

  // ######################################################################
  inline
  const Dims SimStructure::getOutDims() const
  {
    return itsOutDims;
  }

  // ######################################################################
  inline
  const uint SimStructure::getOutWidth() const
  {
    return itsOutDims.w();
  }

  // ######################################################################
  inline
  const uint SimStructure::getOutHeight() const
  {
    return itsOutDims.h();
  }

  // ######################################################################
  inline
  const uint SimStructure::getOutSize() const
  {
    return itsOutDims.h() * itsOutDims.w();
  }

  // ######################################################################
  inline
  SimStructure::SimStructure(const SimStructure& rhs) 
      : itsName(rhs.itsName), itsUnits(rhs.itsUnits), itsOutDims(rhs.itsOutDims)
  { }
  
  // ######################################################################
  inline
  SimStructure& SimStructure::operator=(const SimStructure& rhs)
  { 
    if (this != &rhs)
    {
      itsName = rhs.itsName;
      itsUnits = rhs.itsUnits;
      itsOutDims = rhs.itsOutDims;
    }
    return *this;
  }
  
  // ######################################################################
  inline
  void SimStructure::splitExcInh(const Image<double>& input, Image<double>& Exc, Image<double>& Inh)
  {
    const uint size = input.size();
    const uint threads = getNumThreads();
    const uint perThread = size / threads;
    const uint left = size - perThread*threads;

    Image<double>::const_iterator in_iter(input.begin()), in_mid(in_iter + perThread + left), in_end(input.end());
    Image<double>::iterator oute_iter(Exc.beginw()), outi_iter(Inh.beginw());

    std::vector<std::function<void()> > jobs;
    while (in_iter != in_end)
    {
      SplitJob s(in_iter, in_mid, oute_iter, outi_iter);
      jobs.push_back(std::move(s));
      in_iter += perThread;
      in_mid += perThread;
      oute_iter += perThread;
      outi_iter += perThread;
    }
    process(std::move(jobs));
  }

  // ######################################################################
  inline
  void SimStructure::process(std::function<void()>&& job)
  {
    (itsWorkQueue.push(std::move(job))).get();
  }
  
  // ######################################################################
  inline
  void SimStructure::process(std::vector<std::function<void()> >&& jobs)
  {
    itsWorkQueue.push(std::move(jobs));
  }
}

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
