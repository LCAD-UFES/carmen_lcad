/*!@file ModelNeuron/Structure.H Class declarations for partial 
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/Structure.H $

#ifndef MODELNEURON_STRUCTURE_H_DEFINED
#define MODELNEURON_STRUCTURE_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Util/SimTime.H"
#include "Image/ImageSet.H"
#include "Image/ImageSetOps.H"
#include "ModelNeuron/SimStructure.H"
#include "ModelNeuron/SimUnit.H"
#include "ModelNeuron/NeuralSimUtils.H"

namespace nsu
{
  class Location;
  // ######################################################################
  //  Structure
  //  ######################################################################
  
  /*  A Structure which is a collection of SimStructures. Derive from this
      class and use addmodule to set up the structure and and override interact to
      implement interactions between sub modules.
  
      when using the getSimUnit and editSimUnit functions, the N-D location 
      argument is interpreted as the position in a 3-D volume, where the 
      third dimensions is the sub module number. This recursivly dives down
      sub modules so the return vector may have many units if the structure's
      sub modules have sub modules. 
  
      if not all the layers of the exact same type, then derive from or use
      Structure<SimStructure*> which will inact the virtual mode
  */
  //  ######################################################################
  template <class T>
  class Structure : public SimStructure
  {
    public:
      //! constructor
      Structure(const SimTime& timeStep, const uint w = 40, const uint h = 30, 
                const std::string name = "", const std::string units = "");
  
      //!default assignment and copy constructor OK

      virtual ~Structure() { };

      //!set the input
      void input(const Image<double>& in, const int pos = SimStructure::DEFAULT);

      //!set the input
      void inputExc(const Image<double>& in, const int pos = SimStructure::DEFAULT);

      //!set the input
      void inputInh(const Image<double>& in, const int pos = SimStructure::DEFAULT);
  
      //!get the current output
      virtual Image<double> getOutput(const int pos = SimStructure::DEFAULT) const;
  
      //!get the current display output 
      virtual Image<double> getDisplayOutput(const int pos = SimStructure::DEFAULT) const;

      //!set the default input and output modules
      void setDefaultIO(const uint input_pos, const uint output_pos);
  
      //!evolve up to specified time
      void evolve(const SimTime& simtime);

      //!reset/initialize
      void initialize();

      //!get unit(s) at desired location
      void getSimUnit(const Location& loc, std::vector<const SimUnit*>& units);
  
      //!edit unit(s) at the desired output position
      void editSimUnit(const Location& loc, std::vector<SimUnit*>& units);
  
      //set the current time
      void setTime(const SimTime& time, const bool recursive = true);

      //get the number of sub modules
      const uint numSubs() const;
  
      //get a referene to a sub module const version
      const typename nsu::TypeTraits<T>::PointeeType& getSub(const uint sub) const;
  
      //!get the unit at desired position mutable version
      typename nsu::TypeTraits<T>::PointeeType& editSub(const uint pos);

      //get the simulation time
      const SimTime& getTime() const;

      //get the simulation time step
      const SimTime& getTimeStep() const;

      //!clone the module
      virtual Structure* clone() const = 0;

    protected:
      //protect copy and asignment, use clone
      Structure(const Structure<T>& nlc);
      Structure<T>& operator=(const Structure<T>& nlc);
  
      //!add a sub module
      void addSub(const typename nsu::TypeTraits<T>::PointeeType& structure);

      //add a layer interaction
      void addInteraction(uint const from, uint const to, Weights const & weight);
  
    private:
      //do our class specific interactions; default non-op
      virtual void interact();
  
      //do any class specific inititialization
      virtual void doInit();

      //!a simple class to hold weight interactions
      struct LayerInteraction
      {
          LayerInteraction();
          ~LayerInteraction();
          LayerInteraction(uint const from, uint const to, Weights const & weights);
          LayerInteraction(LayerInteraction const & rhs);
          void operator=(LayerInteraction const & rhs);

          uint from;
          uint to;
          Weights* weight;
      };
  
      int itsInpPos, itsOutPos;
      nsu::vector<T> itsSubs;
      ImageSet<double> itsInpExc, itsInpInh;
      SimTime itsTimeStep, itsT;
      uint itsSampleScale;
      bool isSet;
      std::vector<LayerInteraction> itsInteractions;
      Location itsProbe;
      bool itsAutoTime;
  };

  // ######################################################################
  /*

    A class to derive from to create new Structures. New
    Structure derived types can derive from this class to
    inherit the doClone() function if desired. 
    
    Programmer Note: This class uses the 'quriously recursive template
    pattern' to simplify creation of new classes for the programmer. As
    such, to create a new simulation module, classes should adhear the
    following convention:
    
    class mymodule : public StructureDerived<mymodule>
    {
    mymodule(//params//) : Structure<mymodule>(//params//) { };
    //...rest of functions
    };

  */
  // ######################################################################
  template <class T, class Derived>
  class StructureDerived : public Structure<T>
  {
    protected:
      StructureDerived(const SimTime& timestep, const uint width, const uint height,
                       const std::string name = "", const std::string units = "") : 
          Structure<T>(timestep, width, height, name, units) { };
  
      virtual ~StructureDerived() { };
  
    private:
      Derived* clone() const { return new Derived(dynamic_cast<const Derived&>(*this)); }
  };

  // ######################################################################
  // Structure implementation
  // ######################################################################
  template <class T> inline
  Structure<T>::Structure(const SimTime& timestep,const uint w, const uint h, 
                          const std::string name, const std::string units) :
      SimStructure(w, h, name, units), itsInpPos(SimStructure::DEFAULT), itsOutPos(SimStructure::DEFAULT), itsSubs(),
      itsInpExc(), itsInpInh(), itsTimeStep(timestep), itsT(SimTime::ZERO()), itsSampleScale(1), isSet(false), itsInteractions(), itsProbe(), 
      itsAutoTime( (timestep < SimTime::ZERO()) ? true : false )
  { }

  // ######################################################################
  template <class T> inline
  Structure<T>::Structure(const Structure<T>& rhs) : SimStructure(rhs), itsInpPos(rhs.itsInpPos), 
                                                     itsOutPos(rhs.itsOutPos), itsSubs(rhs.itsSubs),
                                                     itsInpExc(rhs.itsInpExc), itsInpInh(rhs.itsInpInh),
                                                     itsTimeStep(rhs.itsTimeStep), itsT(rhs.itsT), 
                                                     itsSampleScale(rhs.itsSampleScale), isSet(rhs.isSet),
                                                     itsInteractions(rhs.itsInteractions), itsProbe(rhs.itsProbe),
                                                     itsAutoTime(rhs.itsAutoTime)
  { }

  // ######################################################################
  template <class T> inline
  Structure<T>& Structure<T>::operator=(const Structure<T>& rhs)
  {
    if (this != &rhs)
    {
      SimStructure::operator=(rhs);
      itsInpPos = rhs.itsInpPos;
      itsOutPos = rhs.itsOutPos;
      itsSubs = rhs.itsSubs;
      itsInpExc = rhs.itsInpExc; 
      itsInpInh = rhs.itsInpInh;
      itsTimeStep = rhs.itsTimeStep; 
      itsT = rhs.itsT;
      itsSampleScale = rhs.itsSampleScale;
      isSet = rhs.isSet;
      itsInteractions = rhs.itsInteractions;
      itsProbe = rhs.itsProbe;
      itsAutoTime = rhs.itsAutoTime;
    }
    return *this;
  }

  // ######################################################################
  template <class T> inline
  void Structure<T>::input(const Image<double>& in, const int pos)
  {   
    int p = (pos >= 0) ? pos : itsInpPos;
  
    if (p < (int)itsInpExc.size())
      SimStructure::splitExcInh(in, itsInpExc[p], itsInpInh[p]);
    else
      LFATAL("This Structure has no %d'th sub module", p);
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::inputExc(const Image<double>& in, const int pos)
  { 
    int p = (pos >= 0) ? pos : itsInpPos;
  
    if (p < (int)itsInpExc.size())
      itsInpExc[p] += in;
    else
      LFATAL("This Structure has no %d'th sub module", p);
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::inputInh(const Image<double>& in, const int pos)
  { 
    int p = (pos >= 0) ? pos : itsInpPos;

    if (p < (int)itsInpInh.size())
      itsInpInh[p] += in;
    else
      LFATAL("This Structure has no %d'th sub module", p);
  }
  
  // ######################################################################
  template <class T> inline 
  Image<double> Structure<T>::getOutput(const int pos) const
  {
    int p = (pos >= 0) ? pos : itsInpPos;
    
    if (p >= (int)numSubs())
      LFATAL("This Structure has no %d'th sub module", p);
    
    return itsSubs[p].getOutput();
  }
  
  // ######################################################################
  template <class T> inline 
  Image<double> Structure<T>::getDisplayOutput(const int pos) const
  {
    int p = (pos >= 0) ? pos : itsInpPos;

    if (p >= (int)numSubs())
      LFATAL("This Structure has no %d'th sub module", p);
  
    return itsSubs[p].getDisplayOutput();
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::setDefaultIO(const uint input_pos, const uint output_pos)
  {
    itsInpPos = input_pos;
    itsOutPos = output_pos;
    if (getUnits().empty())
      setUnits(itsSubs[itsOutPos].getUnits());
    isSet = true;
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::evolve(const SimTime& t)
  {
    const SimTime interval(t - itsT);
    const int64 nsteps = interval.nsecs() / itsTimeStep.nsecs();
    const int steps = (int)nsteps;  
    SimTime currTimeStep;

    if (steps <= 0)
      ++itsSampleScale;//keep track of how many times we don't integrate
    else
    {
      //set our current time step 
      currTimeStep = SimTime::NSECS(interval.nsecs() / nsteps);  
      
      if (itsSampleScale > 1)
      {      
        //rescale our inputs
        itsInpExc /= (double)itsSampleScale;
        itsInpInh /= (double)itsSampleScale;
        itsSampleScale = 1;
      }
      
      for (int ii = 0; ii < steps; ++ii)
      {      
        //update our current time
        itsT += currTimeStep;

        //do the weight interactions
        typename std::vector<LayerInteraction>::iterator interaction(itsInteractions.begin()), end(itsInteractions.end());
        while (interaction != end)
        {
          editSub(interaction->to).input(interaction->weight->compute(getSub(interaction->from).getOutput()));
          ++interaction;
        }

        //do any extra sub interactions
        interact();
          
        //pass the input around and evolve
        for (uint ii = 0; ii < numSubs(); ++ii)
        {
          //a lambda to do our evolution of layers in parallel. 
          std::function<void()> func = [&itsSubs, &itsInpExc, &itsInpInh, &itsT, ii]()
            { 
              itsSubs[ii].inputExc(itsInpExc[ii]);
              itsSubs[ii].inputInh(itsInpInh[ii]);
              itsSubs[ii].evolve(itsT); 
            };
          
          process(std::move(func));
        }
      }
      
      //reset our input after we evolve to the current time step
      for (uint ii = 0; ii < numSubs(); ++ii)
      {
        itsInpExc[ii].clear();
        itsInpInh[ii].clear();
      }
    }
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::initialize()
  {
    itsT = SimTime::ZERO();
  
    typename nsu::vector<T>::iterator iter(itsSubs.begin());
    while (iter != itsSubs.end())
      (iter++)->initialize();
  
    doInit();
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::setTime(const SimTime& time, const bool recursive)
  {
    itsT = time;
    if (recursive)
      for (uint ii = 0; ii < numSubs(); ++ii)
        editSub(ii).setTime(time, true);
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::getSimUnit(const Location& loc, std::vector<const SimUnit*>& units)
  {
    const uint size = getOutSize();
    itsProbe.setLocation(loc);
    std::vector<int> positions = itsProbe.getLinearPos();
    std::vector<int>::iterator pos(positions.begin()), end(positions.end());
    while (pos != end)
      editSub(*pos++ / size).getSimUnit(loc, units);
  }
  
  // ######################################################################
  template <class T> inline 
  void Structure<T>::editSimUnit(const Location& loc, std::vector<SimUnit*>& units) 
  {
    const uint size = getOutSize();
    itsProbe.setLocation(loc);
    std::vector<int> positions = itsProbe.getLinearPos();
    std::vector<int>::iterator pos(positions.begin()), end(positions.end());
    while (pos != end)
      editSub(*pos++ / size).editSimUnit(loc, units);
  }

  // ######################################################################
  template <class T> inline 
  const uint Structure<T>::numSubs() const
  {
    return (uint)itsSubs.size();
  }

  // ######################################################################
  template <class T> inline 
  const typename nsu::TypeTraits<T>::PointeeType& Structure<T>::getSub(const uint sub) const
  {
    return itsSubs[sub];
  }

  // ######################################################################
  template <class T> inline 
  typename nsu::TypeTraits<T>::PointeeType& Structure<T>::editSub(const uint pos)
  {
    return itsSubs[pos];
  }

  // ######################################################################
  template <class T> inline 
  const SimTime& Structure<T>::getTime() const
  {
    return itsT;
  }

  // ######################################################################
  template <class T> inline 
  const SimTime& Structure<T>::getTimeStep() const
  {
    return itsTimeStep;
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::addSub(const typename nsu::TypeTraits<T>::PointeeType& structure)
  {
    if ((structure.getOutWidth() != getOutWidth()) || (structure.getOutHeight() != getOutHeight()))
      LFATAL("sub output dimensions must the be same as the structures");
    
    itsSubs.push_back(structure);
    itsInpExc.push_back(Image<double>(structure.getOutDims(), ZEROS));
    itsInpInh.push_back(Image<double>(structure.getOutDims(), ZEROS));
    
    if (!isSet)
    {
      itsInpPos = 0;
      ++itsOutPos;
      if (getUnits().empty())
        setUnits(itsSubs[itsOutPos].getUnits());
    }

    itsProbe.setHyperCubeDims(getOutWidth(), getOutHeight(), numSubs());
    
    if ( (itsAutoTime) && ((structure.getTimeStep() < getTimeStep()) || (getTimeStep() == SimTime::ZERO())) )
      itsTimeStep = structure.getTimeStep();
  }
  
  // ######################################################################
  template <class T> inline 
  void Structure<T>::interact()
  {
  }

  // ######################################################################
  template <class T> inline 
  void Structure<T>::doInit() 
  { 
  };

  // ######################################################################
  template <class T> inline 
  void Structure<T>::addInteraction(uint const from, uint const to, Weights const & weight)
  {
    if ((from < numSubs()) && (to < numSubs()))
      itsInteractions.push_back(LayerInteraction(from, to, weight));
    else
      LFATAL("sub module index out of range");   
  }
  
  // ######################################################################
  template <class T> inline 
  Structure<T>::LayerInteraction::LayerInteraction() : from(0), to(0), weight(NULL)
  {
  }

  // ######################################################################
  template <class T> inline 
  Structure<T>::LayerInteraction::~LayerInteraction()
  {
    delete weight;
  }
  
  // ######################################################################
  template <class T> inline 
  Structure<T>::LayerInteraction::LayerInteraction(uint const f, uint const t, Weights const & w) 
      : from(f), to(t), weight(w.clone())
  {
  }
  
  // ######################################################################
  template <class T> inline 
  Structure<T>::LayerInteraction::LayerInteraction(LayerInteraction const & rhs) 
      : from(rhs.from), to(rhs.to), weight(rhs.weight->clone())
  {
  }
  
  // ######################################################################
  template <class T> inline 
  void Structure<T>::LayerInteraction::operator=(LayerInteraction const & rhs)
  {
    from = rhs.from;
    to = rhs.to;
    if (weight)
      delete weight;
    
    weight = rhs.weight;
  }
}
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
