/*!@file ModelNeuron/Synapse.H Class to represent a synapse */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/Synapse.H $

#ifndef MODELNEURON_SYNAPSE_H_DEFINED
#define MODELNEURON_SYNAPSE_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/LowPass.H"
#include<cmath>

#define GMAX 1000

namespace nsu
{
  // ######################################################################
  //! A collection of functors to compute current (pA) across a
  //! synapse given the conductance (millisiemens) and membrane voltage
  //! (mV). Functors should define the following functions and data:
  //!
  //! static const double tau = <time constant>
  //! static const Type name = <type name (integer)>
  //! static const char* name(); 
  //! const double operator()(const double& conductance, 
  //!                         const double& voltage) const;
  //! void setVoltage(const double& voltage);
  //!
  //! from Izhikevich (2008, 2004), Dayan and abbot (2001). 
  // ######################################################################
  namespace Receptor
  {
    //integer names for our types
    enum Type {AMPAR, NMDAR, GABAAR, GABABR};
  
    // ######################################################################
    //!return current from AMPA receptor
    // ######################################################################
    struct AMPA
    {
        constexpr static double tau = 5.0;
        constexpr static Type id = AMPAR;
        constexpr static const char* name() { return "AMPA"; };

        AMPA() : membrane_voltage(0.0) { };

        const double operator()(const double& g) const 
        { return g * (membrane_voltage - 0.0); };

        void setVoltage(const double& v) { membrane_voltage = v; };

        double membrane_voltage; //the last external voltage (mV)
    };
  
    // ######################################################################
    //!return current from NMDA receptor
    // ######################################################################
    struct NMDA
    {
        constexpr static double tau =  150.0;
        constexpr static Type id = NMDAR;
        constexpr static const char* name() { return "NMDA"; };

        NMDA() : membrane_voltage(0.0) { };
    
        const double operator()(const double& g)  const 
        {  
          double temp = ((membrane_voltage+80.0)/60.0) * 
            ((membrane_voltage+80.0)/60.0);
          return g * ( temp / (1.0 + temp) ) * (membrane_voltage - 0.0);
        }

        void setVoltage(const double& v) { membrane_voltage = v; };

        double membrane_voltage; //the last external voltage (mV)
    };
  
    // ######################################################################
    //!return current from GABAA receptor
    // ######################################################################
    struct GABAA
    {
        constexpr static double tau = 6.0;
        constexpr static Type id = GABAAR;
        constexpr static const char* name() { return "GABAA"; };

        GABAA() : membrane_voltage(0.0) { };

        const double operator()(const double& g)  const 
        { return g * (membrane_voltage + 70.0); };

        void setVoltage(const double& v) { membrane_voltage = v; };

        double membrane_voltage; //the last external voltage (mV)
    };
  
    // ######################################################################
    //!return current from GABAB receptor
    // ######################################################################
    struct GABAB
    {
        constexpr static double tau = 150.0;
        constexpr static Type id = GABABR;    
        constexpr static const char* name() { return "GABAB"; };

        GABAB() : membrane_voltage(0.0) { };

        const double operator()(const double& g)  const 
        { return g * (membrane_voltage + 90.0); };

        void setVoltage(const double& v) { membrane_voltage = v; };

        double membrane_voltage; //the last external voltage (mV)
    };
  }  
  // ######################################################################
  //! A class to represent the overall effect of a large number of
  //! synapses of a particular type, which is modeled by a simple first 
  //! order differential:
  //!
  //! dg/dt = -g/tau + I, where tau is the decay constant in
  //! milliseconds, I is the number of spikes received and g is the
  //! conductance in millisiemens.
  //!
  //! The first template argument should be a functor that descibes the
  //! synapses current-voltage relationship.Several suitable functors
  //! for standard channel types are defined in class Receptor, given
  //! above. The second template paramter should be the integration
  //! method as descibed in LowPass.H.
  //! ######################################################################
  template <class SynType, class IntType = LowPassExpEuler>
  class Synapse: public IntType, public SimUnit
  {
    public:  
      //! Constructor with default params
      Synapse(const SimTime& timestep = SimTime::MSECS(1.0));
  
      //! destructor
      ~Synapse();

      //! set current voltage
      void setV(const double& v);
  
      //! get the display output
      const double getDisplayOutput() const;
  
      //! set current maximum conductance
      void setGmax(const double& gmax);
   
    private:  
      //! integrate our internals
      const double doIntegrate(const SimTime& dt, 
                               const double& inpe, const double& inh);
  
      //!initialize
      void doInit();
  
      //!return a copy of this object
      Synapse<SynType, IntType>* doClone() const;
  
      double itsGmax; //maximum conductance (mS)
      double itsG; // internal state
      SynType getCurrent; // our current (pA)
  };

  typedef Synapse<Receptor::AMPA, LowPassExpEuler> AMPASynapse;
  typedef Synapse<Receptor::NMDA, LowPassExpEuler> NMDASynapse;
  typedef Synapse<Receptor::GABAA, LowPassExpEuler> GABAASynapse;
  typedef Synapse<Receptor::GABAB, LowPassExpEuler> GABABSynapse;

  // ######################################################################
  // ##### implementation for Synapse
  // ######################################################################
  template <class SynType, class IntType> inline
  Synapse<SynType, IntType>::Synapse(const SimTime& timestep) :     
      IntType(timestep.msecs(), 0.0, SynType::tau),
      SimUnit(SimTime::MSECS(IntType::getIntegratorTimeStep()), IntType::RateType, std::string(SynType::name()) + " Synapse", "pA"),
      itsGmax(GMAX), itsG(0.0), getCurrent() { };

  // ######################################################################
  template <class SynType, class IntType> inline
  Synapse<SynType, IntType>::~Synapse()
  { }

  // ######################################################################
  template <class SynType, class IntType> inline
  void Synapse<SynType, IntType>::setV(const double& v) 
  { 
    getCurrent.setVoltage(v);
  }

  // ######################################################################
  template <class SynType, class IntType> inline 
  const double Synapse<SynType, IntType>::getDisplayOutput() const
  {
    return itsG;
  }

  // ######################################################################
  template <class SynType, class IntType> inline
  void Synapse<SynType, IntType>::setGmax(const double& gmax) 
  { 
    itsGmax = gmax; 
  }

  // ######################################################################
  template <class SynType, class IntType> inline
  const double 
  Synapse<SynType, IntType>::doIntegrate(const SimTime& dt, 
                                         const double& inpe, const double& inpinh)
  {
    //update_nodecay_input inherited from template parameter
    IntType::update_nodecay_input(inpe, itsG, dt.msecs()); 
    if (itsG > itsGmax) 
      itsG = itsGmax;
    return getCurrent(itsG);
  }

  // ######################################################################
  template <class SynType, class IntType> inline
  void Synapse<SynType, IntType>::doInit()
  {
    itsGmax = GMAX;
    itsG = 0.0;
    getCurrent = SynType();
  }

  // ######################################################################
  template <class SynType, class IntType> inline
  Synapse<SynType, IntType>* Synapse<SynType, IntType>::doClone() const
  {
    return new Synapse<SynType, IntType>(*this);
  }
}
#endif
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
