/*!@file ModelNeuron/IZNeuron.H Class declarations for an Izhikevich neuron */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/IZNeuron.H $

#ifndef MODELNEURON_IZNEURON_H_DEFINED
#define MODELNEURON_IZNEURON_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/Synapse.H"

namespace nsu
{
  // ######################################################################
  //! The Izhikevich neural model of spike generation as presented in
  //! his 2003,2008 paper
  //######################################################################
  class IZNeuronFunc
  {    
    public:
      enum Type {FSNeuron, RSNeuron, EBNeuron};
      
      //! Constructor with default params
      IZNeuronFunc(const double& a = 0.15,      // timescale of recovery variable(u)
                   const double& b = 8.0,       // sensitivity of u to voltage
                   const double& c = -55.0,     // after-spike reset value of v
                   const double& d = 200.0,     // after-spike reset value of u
                   const double& k = 1.0,       // current voltage relationship
                   const double& Cm = 20.0,     // in microfarads
                   const double& V_rest = -55.0, // in millivolts
                   const double& V_thresh = -40.0, // in millivolts
                   const double& V_peak = 25.0) :  // in millivolts
          itsSpike(false), itsV(V_rest), itsI(0.0), itsU(0.0),itsa(a), itsb(b), 
          itsc(c), itsd(d), itsk(k),itsCm(Cm), itsVr(V_rest), itsVth(V_thresh), 
          itsVp(V_peak) { };

      //!destructor
      ~IZNeuronFunc() { };

      //since our last integration time step did we spike?
      const bool getSpike() const {return itsSpike; };

      //! integrate 1ms 
      void integrate();

      //! initialize or reset 
      void initialize();    

      //! set membrane potential to given value (in mV)
      void setV(const double& val) { itsV = val; };

      //!set the current (picoamps) of the cell
      void setCurrent(const double& current) {itsI = current; };
  
      //! get current membrane potential (mV)
      const double getV() const { return itsV; };

      //! get peak voltage (mV)
      const double getVpeak() const { return itsVp; };
  
      //! get the value of the recovery variable
      const double getU() const { return itsU; };
  
      //! get the last set input current (picoAmps)
      const double getCurrent() const { return itsI; };  

      //! setup different neuron types
      void setup(const Type type, const bool use_random = false);
  
    private:
      //! reset state when a spike occurs
      void reset();
  
      bool itsSpike;      //did we spike in the last time step
  
      double itsV;        // membrane potential in millivolts
      double itsI;        // input current in picoamperes
      double itsU;        //recovery variable

      double itsa;        //dimenionless param
      double itsb;        //dimenionless param
      double itsc;        //dimenionless param
      double itsd;        //dimenionless param
      double itsk;        //gain factor
      double itsCm;       //membrane capacitance 
      double itsVr;       //resting voltage
      double itsVth;      //threshold to spike
      double itsVp;       //peak voltage
  };

  // ######################################################################
  // ! The Izhikevich spike generation model with channels (from Dayan & Abbot 2001, and Izhikevich, 2008, 2004, 2003).
  // This class represents a neuron with four 'lumped' channel types, where all synapses of a given type are goverened by
  // a single equation with first order kinematics (see Synapse.H). The neuron contains AMPA and NMDA as excitatory
  // channels and GABAA and GABAB as inhibitory channles. Alternatevly, setI() can be used to set an external current (pA)
  // to the neuron. After a call to evolve, the external input will be reset to 0.
  // ######################################################################
  class IZNeuron : public SimUnit
  {
    public:
      //! Constructor with default params
      IZNeuron(const double& a = 0.15,         // timescale of recovery variable
               const double& b = 8.0,          // sensitivity of u to voltage
               const double& c = -55.0,        // after-spike reset value of v
               const double& d = 200.0,        // after-spike reset value of u
               const double& k = 1.0,          // current voltage relationship
               const double& Cm = 20.0,        // in microfarads
               const double& V_rest = -55.0,   // in millivolts
               const double& V_thresh = -40.0, // in millivolts
               const double& V_peak = 25.0,    // in millivolts
               const std::string& name = "Neuron", 
               const std::string& units = "spike");
  
      //!destructor
      ~IZNeuron() { };

      //set the voltage
      void setV(const double& voltage);
  
      //!get the output for display purposes, the membrane voltage
      const double getDisplayOutput() const;
  
      //!get the number of sub units
      const uint numSubs() const;
  
      //!get sub unit
      const SimUnit& getSub(const uint i) const;
  
      //!get the current voltage (mV)
      const double getV() const;
  
      //!get the current current (pA)
      const double getI() const;
  
      //!set an external current. 
      void setI (const double& current);

      //!setup the neuron and its parameters
      const IZNeuron& setup(const IZNeuronFunc::Type type, const bool use_random = false);

    protected:
      //!get the sub, unit mutable version
      SimUnit& editSub(const uint i);
  
    private:
      //! integrate 1 time step 
      double const doIntegrate(const SimTime& dt, 
                               const double& ine, const double& ini);

      //!initialize or reset all receptors and variables
      void doInit();

      //!perform copy of IZNeuron
      IZNeuron* doClone() const;

      double itsI; //for external current
      IZNeuronFunc itsN; //our neuron

      //synapses
      AMPASynapse ampa;
      NMDASynapse nmda;
      GABAASynapse gabaa;
      GABABSynapse gabab;
  };

  struct RSNeuron : public IZNeuron
  { RSNeuron() : IZNeuron() { setup(IZNeuronFunc::Type::RSNeuron); }; 
      RSNeuron* doClone() const { return new RSNeuron(*this); }; }; 

  struct FSNeuron : public IZNeuron
  { FSNeuron() : IZNeuron() { setup(IZNeuronFunc::Type::FSNeuron); }; 
      FSNeuron* doClone() const { return new FSNeuron(*this); }; };

  struct EBNeuron : public IZNeuron
  { EBNeuron() : IZNeuron() { setup(IZNeuronFunc::Type::EBNeuron); }; 
      EBNeuron* doClone() const { return new EBNeuron(*this); }; };

  // ######################################################################
  // Register neuron types
  // ######################################################################
  namespace 
  {
    typedef SimUnit::Factory IZFactory;
    typedef SimUnit::Creator IZCreator;
    //define creation functions
    struct RegisterIZNeuron
    {
        RegisterIZNeuron()
        {
          IZFactory::instance().add("RSNeuron", IZCreator::make<RSNeuron>());
          IZFactory::instance().add("FSNeuron", IZCreator::make<FSNeuron>());
          IZFactory::instance().add("EBNeuron", IZCreator::make<EBNeuron>());
        }
    };
    static RegisterIZNeuron registerizn;  
  }
}

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
