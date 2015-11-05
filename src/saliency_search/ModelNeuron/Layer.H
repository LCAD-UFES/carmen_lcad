/*!@file ModelNeuron/Layer.H Class declarations for a layer of
   SimUnits that supports lateral interactions, in a 2D grid. */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/Layer.H $

#ifndef MODELNEURON_LAYER_H_DEFINED
#define MODELNEURON_LAYER_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/SimLayer.H"
#include "ModelNeuron/Weights.H"
#include "ModelNeuron/SimUnit.H"
#include "ModelNeuron/NeuralSimUtils.H"

namespace nsu
{
  // ######################################################################
  //  Layer
  //  ######################################################################
  /*  
      A SimLayer (see SimLayer.H) that models a
      2-dimensional layer of cortex. Neurons are placed on a grid, with
      laterial interactions computed by a Weights (see Weights.H)
      object.
  */
  //  ######################################################################
  template<class T, class W>
  class Layer : public SimLayer
  {
    public:
      //! constructor
      Layer(const SimTime& timestep, const uint width, const uint height,
            const std::string& name = "", const std::string& units = "");
      
      //! constructor for weights with 1 param
      Layer(const double& param1, const SimTime& timestep, const uint width, const uint height,
            const std::string& name = "", const std::string& units = "");
      
      //! constructor for weights with two params, self connection and boarder options
      Layer(const double& param1,  const double& param2,
            const bool selfConnection, const BorderPolicy bp, 
            const SimTime& timestep, const uint width, const uint height,
            const std::string& name = "", const std::string& units = "");
      
      //! constructor for weights with three params, self connection and boarder options
      Layer(const double& param1, const double& param2, 
            const double& param3, const bool selfConnection,
            const BorderPolicy bp, 
            const SimTime& timestep, const uint width, const uint height,
            const std::string& name = "", const std::string& units = "");
      
      //! constructor for weights with four params, self connection and boarder options
      Layer(const double& param1, const double& param2,  
            const double& param3, const double& param4,
            const bool selfConnection, const BorderPolicy bp,
            const SimTime& timestep, const uint width, const uint height,
            const std::string& name = "", const std::string& units = "");
      
      //!copy constructor
      Layer(const Layer& nlc);

      //assignment
      Layer& operator=(const Layer& nlc);
  
      //!destructor 
      virtual ~Layer();

      //!get the current display output 
      Image<double> getDisplayOutput(const int pos = -1) const;
  
      //!get unit(s) at desired location
      void getSimUnit(const Location& loc, std::vector<const SimUnit*>& units);
  
      //!edit unit(s) at the desired output position
      void editSimUnit(const Location& loc, std::vector<SimUnit*>& units);
  
      //!number of simunits - just returns getOutSize()
      const uint numSimUnits() const;
  
      //!get the unit at desired position [pos / w, pos % w],
      //!(where '/' is integer division, '%' is modulus, and w is the
      //!width of the layer)
      const SimUnit& getSimUnit(const uint pos) const;
  
      //get the unit, mutable version
      SimUnit& editSimUnit(const uint pos);
  
      //!set type of simulation module
      void setModule(const SimUnit& nsm);
  
      //!set type of simulation module at the position 
      void setModule(const SimUnit& nsm, const Location& p);

      //!set the time of this structure and all contained SimUnits if true
      void setTime(const SimTime& time, const bool recursive = true);

      //!set the lateral interaction weights, which describe how activity
      //!is spread throughout the layer.
      void setWeights(const W& weights);

      //clone the object
      Layer<T,W>* clone () const;
  
    private:
      //!integrate one time step
      void doIntegrate(const SimTime& dt, const Image<double>& inpe, const Image<double>& inpi);
   
      //!reset this object and all the SimUnits in the layer
      void doInit();
  
      //!is our layer full?
      const bool isFull(const Location& pos = Location(-1));

      //set the module non-polymorphic
      void setModule(const SimUnit& nsm, nsu::Int2Type<false>);
      void setModule(const SimUnit& nsm, const Location& pos, nsu::Int2Type<false>);
  
      //set the module polymorphic
      void setModule(const SimUnit& nsm, nsu::Int2Type<true>);
      void setModule(const SimUnit& nsm, const Location& pos, nsu::Int2Type<true>);

      //! an iterator into the layer
      typedef typename nsu::vector<T>::iterator iterator;
      typedef typename nsu::vector<T>::const_iterator const_iterator;

      //!iterator for images
      typedef typename Image<double>::iterator image_iterator;
      typedef typename Image<double>::const_iterator const_image_iterator;

      //if we are polymorhic, get the pointee type
      typedef typename nsu::TypeTraits<T>::PointeeType cast_type;

      nsu::vector<T> itsL; //the layer
      W itsW; //weight interaction
      bool itsFull; //true if the entire layer is filled
      Location itsProbe; //copy of the probe, if one

      class IntegrationJob
      {
        public:
          //! constructor
          IntegrationJob(iterator const & startNeuron, iterator const & endNeuron, const_image_iterator const & neighborhoodInput, 
                         const_image_iterator const & exciteInput, const_image_iterator const & inhibitInput, image_iterator const & outValue, 
                         SimTime const & simTime);
          
          //! perform operation
          void operator()();
          
        private:
          iterator itsStartNeuron, itsEndNeuron;
          const_image_iterator itsNeighborhoodInput, itsExciteInput, itsInhibitInput;
          image_iterator itsOutValue;
          SimTime const & itsSimTime;
      };
  };

  // ######################################################################
  // Layer implementation
  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>::Layer(const SimTime& timestep, const uint width, const uint height,
                    const std::string& name, const std::string& units) 
      : SimLayer(timestep, width, height, name, units), itsL(), itsW(), itsFull(false), itsProbe() 
  { 
    itsProbe.setHyperCubeDims(width, height); 
  }
  
  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>::Layer(const double& param1, const SimTime& timestep,
                    const uint width, const uint height,
                    const std::string& name, const std::string& units)
      : SimLayer(timestep, width, height, name, units), itsL(), itsW(param1), itsFull(false), itsProbe() 
  { 
    itsProbe.setHyperCubeDims(width, height); 
  }
    
  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>::Layer(const double& param1,  const double& param2,
                    const bool selfConnection, const BorderPolicy bp, 
                    const SimTime& timestep,
                    const uint width, const uint height,
                    const std::string& name, const std::string& units)
      : SimLayer(timestep, width, height, name, units), itsL(), 
        itsW(param1,param2,selfConnection,bp), itsFull(false), itsProbe() 
  { 
    itsProbe.setHyperCubeDims(width, height); 
  }

  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>::Layer(const double& param1, const double& param2, 
                    const double& param3, const bool selfConnection,
                    const BorderPolicy bp, const SimTime& timestep,
                    const uint width, const uint height,
                    const std::string& name, const std::string& units)
      : SimLayer(timestep, width, height, name, units), 
        itsL(), itsW(param1,param2,param3,selfConnection,bp), itsFull(false), itsProbe() 
  { 
    itsProbe.setHyperCubeDims(width, height); 
  }

  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>::Layer(const double& param1, const double& param2,  
                    const double& param3, const double& param4,
                    const bool selfConnection, 
                    const BorderPolicy bp,const SimTime& timestep,
                    const uint width, const uint height,
                    const std::string& name, 
                    const std::string& units) :
      SimLayer(timestep, width, height, name, units), 
      itsL(), itsW(param1,param2,param3,param4,selfConnection,bp), itsFull(false), itsProbe() 
  { 
    itsProbe.setHyperCubeDims(width, height); 
  }

  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>::Layer(const Layer<T,W>& rhs) :
      SimLayer(rhs), itsL(rhs.itsL), itsW(rhs.itsW), itsFull(rhs.itsFull), itsProbe(rhs.itsProbe)
  { }

  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>& Layer<T,W>::operator=(const Layer<T,W>& rhs)
  {
    if (this != &rhs)
    {
      SimLayer::operator=(rhs);
      itsL = rhs.itsL;
      itsW = rhs.itsW;
      itsFull = rhs.itsFull;
      itsProbe = rhs.itsProbe;
    }
    return *this;
  }

  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>::~Layer()
  { }

  // ######################################################################
  template<class T, class W> inline
  Image<double> Layer<T,W>::getDisplayOutput(const int pos) const
  {
    ASSERT(pos < 1);
    Image<double> out(getOutDims(), NO_INIT);
    if (itsFull)
    {
      image_iterator outiter(out.beginw());    
      const_iterator nsmptr(itsL.begin()), end(itsL.end());
      
      while (nsmptr != end) 
        *outiter++ = (nsmptr++)->getDisplayOutput();
    }
    return out;
  }

  // ######################################################################
  template<class T, class W> inline 
  void Layer<T,W>::getSimUnit(const Location& loc, std::vector<const SimUnit*>& units)
  {
    //update our location and get the vector of points, the hypercube dims have been set already
    itsProbe.setLocation(loc);
    const std::vector<int>& locs = itsProbe.getLinearPos();

    //loop through all the locations
    std::vector<int>::const_iterator iter(locs.begin());
    while (iter != locs.end())
      units.push_back(&getSimUnit(*iter++));
  }
  
  // ######################################################################
  template<class T, class W> inline 
  void Layer<T,W>::editSimUnit(const Location& loc, std::vector<SimUnit*>& units)
  {
    //update our location and get the vector of points, the hypercube dims have been set already
    itsProbe.setLocation(loc);
    const std::vector<int>& locs = itsProbe.getLinearPos();
  
    std::vector<int>::const_iterator iter(locs.begin());
    while (iter != locs.end())
      units.push_back(&editSimUnit(*iter++));
  }

  // ######################################################################
  template<class T, class W> inline 
  const uint Layer<T,W>::numSimUnits() const
  {
    return getOutSize();
  }

  // ######################################################################
  template<class T, class W> inline 
  const SimUnit& Layer<T,W>::getSimUnit(const uint pos) const
  {
    if (pos > itsL.size())
      if (itsL.isNull(pos))
        LFATAL("That unit has not yet been assigned");
    return itsL[pos];
  }
  
  // ######################################################################
  template<class T, class W> inline 
  SimUnit& Layer<T,W>::editSimUnit(const uint pos)
  {
    if (pos > itsL.size())
      if (itsL.isNull(pos))
        LFATAL("That unit has not yet been assigned");
    return itsL[pos];
  }
  // ######################################################################
  template<class T, class W> inline 
  void Layer<T,W>::setModule(const SimUnit& mod)
  { 
    setModule(mod, nsu::Int2Type<nsu::TypeTraits<T>::isPointer >());

    if (itsFull && (getTimeStep() <= SimTime::ZERO()) )
    {
      SimTime fastest = SimTime::SECS(100);
      for (uint ii = 0; ii < getOutSize(); ++ii)
      {
        const SimTime current = getSimUnit(ii).getTimeStep();
        if (current < fastest)
          fastest = current;
      }
      setTimeStep(fastest);
    }
  }

  // ######################################################################
  template<class T, class W> inline 
  void Layer<T,W>::setModule(const SimUnit& mod, const Location& pos)
  {
    setModule(mod, pos, nsu::Int2Type<nsu::TypeTraits<T>::isPointer >());

    if (itsFull && (getTimeStep() <= SimTime::ZERO()) )
    {
      SimTime fastest = SimTime::SECS(100);
      for (uint ii = 0; ii < getOutSize(); ++ii)
      {
        const SimTime current = getSimUnit(ii).getTimeStep();
        if (current < fastest)
          fastest = current;
      }
      setTimeStep(fastest);
    }
  }

  // ######################################################################
  template<class T, class W> inline 
  void Layer<T,W>::setWeights(const W& weights)
  {
    itsW = weights;
  }

  // ######################################################################
  template<class T, class W> inline 
  void Layer<T,W>::setTime(const SimTime& time, const bool recursive)
  {
    //call the base class version, but with no recursion since we wont have any subs. 
    SimLayer::setTime(time, false);
    
    if (recursive)
    {
      iterator nsmptr(itsL.begin()), end(itsL.end());
      uint count = 0;
      while (nsmptr != end) 
      {
        
        if (!itsL.isNull(count++))
          nsmptr->setTime(time, true);
        
        ++nsmptr;
      }
    }
  }

  // ######################################################################
  template<class T, class W> inline
  void Layer<T,W>::doIntegrate(const SimTime& dt, const Image<double>& inpe, const Image<double>& inpi)
  {
    if (!itsFull || !itsW.initialized())
      LFATAL("All positions in the layer have not been filled with modules"
             " or the weights have not yet been setup"); 
    
    const uint size = getOutSize();
    const uint threads = getNumThreads();
    const uint perThread = size / threads;
    const uint left = size - perThread*threads;

    //compute any neighborhood activity
    const Image<double> neighborhood  = itsW.compute(itsOutput);
    const_image_iterator i_n(neighborhood.begin());
      
    //get an iterator for our layer
    iterator i_l(itsL.begin()), i_mid(i_l + perThread + left), end(itsL.end());
      
    //output iterator
    image_iterator i_out(itsOutput.beginw());
      
    //input iterators
    const_image_iterator i_exc(inpe.begin()), i_inh(inpi.begin());

    //current time      
    const SimTime& time = getTime();
      
    //loop over neurons
    std::vector<std::function<void()> > jobs;
    while (i_l != end)
    {
      IntegrationJob job(i_l, i_mid, i_n, i_exc, i_inh, i_out, time);
      jobs.push_back(std::move(job));
      
      i_l+=perThread;
      i_mid+=perThread; 
      i_n+=perThread;
      i_exc+=perThread;
      i_inh+=perThread;
      i_out+=perThread;
    }
    process(std::move(jobs));
  }

  // ######################################################################
  template<class T, class W> inline
  void Layer<T,W>::doInit()
  {
    for (uint i = 0; i < itsL.size(); ++i)
      if (!itsL.isNull(i))
        itsL[i].initialize();
  }

  // ######################################################################
  template<class T, class W> inline 
  Layer<T,W>* Layer<T,W>::clone () const
  { 
    return new Layer<T,W>(*this); 
  };

  // ######################################################################
  template<class T, class W> inline
  void Layer<T,W>::setModule(const SimUnit& nsm, nsu::Int2Type<false>)
  {
    this->setUnits(nsm.getUnits());
    const cast_type temp = dynamic_cast<const cast_type&>(nsm);
    itsL.clear();

    for (uint i = 0; i < getOutSize(); ++i)
      itsL.push_back(temp);

    itsFull = true;
  }

  // ######################################################################
  template<class T, class W> inline
  void Layer<T,W>::setModule(const SimUnit& nsm, nsu::Int2Type<true>)
  {
    this->setUnits(nsm.getUnits());
    itsL.clear();

    for (uint i = 0; i < getOutSize(); ++i)
      itsL.push_back(nsm);

    itsFull = true;
  }

  // ######################################################################
  template<class T, class W> inline
  void Layer<T,W>::setModule(const SimUnit& nsm, const Location& pos, nsu::Int2Type<false>)
  {
    const cast_type temp = dynamic_cast<const cast_type&>(nsm);
    Location p = pos;

    const std::vector<int>& locs = p.getLinearPos(getOutWidth(), getOutHeight());
    std::vector<int>::const_iterator iter(locs.begin());

    while (iter != locs.end())
    {
      if (*iter < (int)itsL.size())
        itsL.set_at(*iter, temp);
      else
        for (int i = itsL.size(); i <= *iter; ++i)
          itsL.push_back(temp);
    }
    
    itsFull = isFull();
  }

  // ######################################################################
  template<class T, class W> inline
  void Layer<T,W>::setModule(const SimUnit& nsm, const Location& pos, nsu::Int2Type<true>)
  {
    Location p = pos;
    const std::vector<int>& locs = p.getLinearPos(getOutWidth(), getOutHeight());
    std::vector<int>::const_iterator iter(locs.begin());
    
    while (iter != locs.end())
    {
      if (*iter < (int)itsL.size())
        itsL.set_at(*iter, nsm);
      else
        for (int i = (int)itsL.size(); i <= *iter; ++i)
          itsL.push_back(nsm);
      
    }

    itsFull = isFull();
  }

  // ######################################################################
  template<class T, class W> inline
  const bool Layer<T,W>::isFull(const Location& pos)
  {
    for (uint i = 0; i < itsL.size(); ++i)
      if (itsL.isNull(i))
        return false;
    return true;
  }

  // ######################################################################
  //  implementation
  // ######################################################################
  template<class T, class W> inline
  Layer<T,W>::IntegrationJob::IntegrationJob(iterator const & startNeuron, iterator const & endNeuron, 
                                             const_image_iterator const & neighborhoodInput, 
                                             const_image_iterator const & exciteInput, 
                                             const_image_iterator const & inhibitInput, 
                                             image_iterator const & outValue, 
                                             SimTime const & simTime)
      : itsStartNeuron(startNeuron), itsEndNeuron(endNeuron), itsNeighborhoodInput(neighborhoodInput), 
        itsExciteInput(exciteInput), itsInhibitInput(inhibitInput), itsOutValue(outValue), itsSimTime(simTime)
  {
  }
  
  // ######################################################################
  template<class T, class W> inline
  void Layer<T,W>::IntegrationJob::operator()()
  {
    while (itsStartNeuron != itsEndNeuron)
    {
      itsStartNeuron->input(*itsNeighborhoodInput++);
      itsStartNeuron->inputExc(*itsExciteInput++);
      itsStartNeuron->inputInh(*itsInhibitInput++);
      itsStartNeuron->evolve(itsSimTime);
      *itsOutValue++ = (itsStartNeuron++)->getOutput();
    }
  }
}
#endif
#endif  
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
