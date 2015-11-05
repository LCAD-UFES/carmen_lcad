/*!@file ModelNeuron/IZNeuron.C Class declarations for an izhikevich neuron */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/IZNeuron.C $

#include "ModelNeuron/IZNeuron.H"

// ######################################################################
// #####  functions for IZNeuronFunc:
// ######################################################################
void nsu::IZNeuronFunc::integrate()
{
  //lets just run our difference equation one time step, dt.
  //Integrate V
  double temp = itsk * (itsV - itsVr) * (itsV - itsVth) - itsU + itsI;
  
  //multiply timestep converted to ms, and devide capacitance      
  itsV += (temp / itsCm);
  
  //Integrate U      
  itsU += itsa * (itsb * (itsV-itsVr) - itsU); 
  
  // Check if voltage has exceeded threshold -> if so, then fire:
  if (itsV > itsVp) 
    {
      itsSpike = true; 
      reset();
    }
  else
    itsSpike = false;
}

// ######################################################################
void nsu::IZNeuronFunc::initialize() {
  itsSpike = false; 
  itsV = itsVr;
  itsI = 0.0;
  itsU = 0.0;
}

// ######################################################################
void nsu::IZNeuronFunc::reset()
{
  itsV =  itsc;
  itsU += itsd;
}

// ######################################################################
void nsu::IZNeuronFunc::setup(const nsu::IZNeuronFunc::Type name, const bool use_random)
{
  switch (name)
  {
  case IZNeuronFunc::Type::RSNeuron:
    itsa = 0.01;         // timescale of recovery variable
    itsb = 5.0;          // sensitivity of u to voltage
    itsc = -60.0;        // after-spike reset value of v
    itsd = 400.0;        // after-spike reset value of u
    itsk = 3.0;          // current voltage relationship
    itsCm = 100.0;       // in microfarads
    itsVr = -60.0;       // in millivolts
    itsVth = -50.0;      // in millivolts
    itsVp = 50.0;        // in millivolts
    break;
    
  case IZNeuronFunc::Type::FSNeuron:
    itsa = 0.15;         // timescale of recovery variable
    itsb = 8.0;          // sensitivity of u to voltage
    itsc = -55.0;        // after-spike reset value of v
    itsd = 200.0;        // after-spike reset value of u
    itsk = 1.0;          // current voltage relationship
    itsCm = 20.0;        // in microfarads
    itsVr = -55.0;       // in millivolts
    itsVth = -40.0;      // in millivolts
    itsVp = 25.0;        // in millivolts
    
  case IZNeuronFunc::Type::EBNeuron:
    itsa = 0.01;         // timescale of recovery variable
    itsb = 17.0;         // sensitivity of u to voltage
    itsc = -42.0;        // after-spike reset value of v
    itsd = 1.5;          // after-spike reset value of u
    itsk = 1.4;          // current voltage relationship
    itsCm = 20.0;        // in microfarads
    itsVr = -55.0;       // in millivolts
    itsVth = -50.0;      // in millivolts
    itsVp = 25.0;        // in millivolts
  }   
  if (use_random)
    {
      //perterbate the parameters a little bit 
    }
}

// ######################################################################
// ##### Functions for IZNeuron:
// ######################################################################
nsu::IZNeuron::IZNeuron(const double& a, const double& b,
                        const double& c, const double& d, const double& k, 
                        const double& Cm, const double& V_rest, 
                        const double& V_thresh, const double& V_peak, 
                        const std::string& name, const std::string& units) : 
    SimUnit(SimTime::MSECS(1.0), SimUnit::STRICT, name, units), 
    itsI(0.0), itsN(a,b,c,d,k,Cm,V_rest,V_thresh,V_peak), 
    ampa(SimTime::MSECS(1.0)), nmda(SimTime::MSECS(1.0)), 
    gabaa(SimTime::MSECS(1.0)), gabab(SimTime::MSECS(1.0))
{ 
}

// ######################################################################
void nsu::IZNeuron::setV(const double& v)
{
  itsN.setV(v);
}

// ######################################################################
const double nsu::IZNeuron::getDisplayOutput() const
{
  return getV();
}

// ######################################################################
const uint nsu::IZNeuron::numSubs() const 
{
  return 4; 
}
 
// ######################################################################
const nsu::SimUnit& nsu::IZNeuron::getSub(const uint i) const
{
  switch (i)
    {
    case 0:
      return ampa;
    case 1:
      return nmda;
    case 2:
      return gabaa;
    case 3:
      return gabab;
    default:
      LFATAL("Sub module position out of range.");
      return ampa;//will never execute;
    }
}

// ######################################################################
nsu::SimUnit& nsu::IZNeuron::editSub(const uint i)
{
  switch (i)
    {
    case 0:
      return ampa;
    case 1:
      return nmda;
    case 2:
      return gabaa;
    case 3:
      return gabab;
    default:
      LFATAL("Sub module position out of range.");
      return ampa;//will never execute;
    }
}

// ######################################################################
const double nsu::IZNeuron::getV() const 
{
  return itsN.getV(); 
}

// ######################################################################
const double nsu::IZNeuron::getI() const 
{
  return itsN.getCurrent(); 
}

// ######################################################################
void nsu::IZNeuron::setI(const double& current)
{
  itsI = current;
}

// ######################################################################
const nsu::IZNeuron& nsu::IZNeuron::setup(const nsu::IZNeuronFunc::Type name, const bool use_random)
{
  switch (name)
  {
  case IZNeuronFunc::Type::RSNeuron:
    setName("RS Neuron");
    break;
    
  case IZNeuronFunc::Type::FSNeuron:
    setName("FS Neuron");
    break;
  case IZNeuronFunc::Type::EBNeuron:
    setName("EB Neuron");
    break;
  }   

  itsN.setup(name, use_random);
  return *this;
}

// ######################################################################
const double nsu::IZNeuron::doIntegrate(const SimTime& dt, 
                                        const double& exc, const double& inh)
{
  //input to synapses
  const double inh1 = inh * -1.0;
  ampa.input(exc);
  nmda.input(exc);
  gabaa.input(inh1);
  gabab.input(inh1);
  
  //set the synapse voltage to cells membrane voltage
  ampa.setV(getV());
  nmda.setV(getV());
  gabaa.setV(getV());
  gabab.setV(getV());
  
  //evolve our synapse to current time
  ampa.evolve(getTime());
  nmda.evolve(getTime());
  gabaa.evolve(getTime());
  gabab.evolve(getTime());
  
  const double synCurr = ampa.getOutput() + nmda.getOutput() + 
    gabaa.getOutput() + gabab.getOutput();
  
  const double inp = itsI - synCurr;
  itsN.setCurrent(inp);  //set the current level
  itsN.integrate(); //integrate 1ms in the future

  return (double)itsN.getSpike(); //return any spikes
}

// ######################################################################
void nsu::IZNeuron::doInit()
{
  itsI = 0.0;
  itsN.initialize();
  ampa.initialize();
  nmda.initialize();
  gabaa.initialize();
  gabab.initialize();
}

// ######################################################################
nsu::IZNeuron* nsu::IZNeuron::doClone() const 
{ return new IZNeuron(*this); };

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
