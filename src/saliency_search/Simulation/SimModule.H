/*!@file Simulation/SimModule.H A module in a simulation framework */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimModule.H $
// $Id: SimModule.H 13390 2010-05-12 04:10:03Z rand $
//

#ifndef SIMULATION_SIMMODULE_H_DEFINED
#define SIMULATION_SIMMODULE_H_DEFINED

#include "Component/ModelComponent.H"
#include "Transport/FrameOstream.H"
#include "Simulation/SimCallback.H"
#include "Simulation/SimCallbackClient.H"
#include "Simulation/SimReqHandler.H"
#include "Simulation/SimReqHandlerClient.H"
class SimEventQueue;


//! SimModule is an extension of ModelComponent for simulation modules
/*! In addition to the base functionality inherited from
  ModelComponents, SimModule objects can post/check for SimEvent. This
  base class merely provides the interface.

  PROGRAMMER NOTE: SimModule has ModelComponent as a virtual
  base. This is so that some classes can inherit from SimModule as
  well as from some other class which itself also virtually derives
  from ModelComponent, with sharing of the underlying ModelComponent
  virtual base.

  The reason for this is for example that, at the time of this
  writing, VisualCortex derives from ComplexChannel which derives from
  ChannelBase which derives from ModelComponent. To make VisualCortex
  a SimModule (so that it can post SimEvents) would then require that
  ChannelBase derives from SimModule, which is not desirable since
  having an evolve() does not make sense for ChannelBase. So we would
  like to keep ChannelBase a standard ModelComponent (non-SimModule)
  while still making VisualCortex a SimModule. The solution is that
  VisualCortex should derive both from ComplexChannel and from
  SimModule. Since both ChannelBase and SimModule derive from
  ModelComponent, making that ModelComponent base virtual is necessary
  to avoid ending up with two ModelComponent objects inside
  VisualCortex.

  Here is how to use this virtual base mechanism:

  - a protected default constructor was added to ModelComponent which
  initializes the rep and sets bogus names;

  - a protected init(mgr, descr, tag) function was added to
  ModelComponent which does the same as the normal constructor

  - in the constructor of the class that virtually derives from
  ModelComponent, make sure you call init(). In exchange, you don't
  really need to call any ModelComponent c'tor, the default will be
  called anyway

  - in further concrete derivations of that one, no change

  so, for example, what happens when you construct BlueChannel is:

  BlueChannel constructor called
   First initialize virtual bases -> ModelComponent::ModelComponent()
   called. The ModelComponent part is left in a bogus uninitialized state
   except that const Impl* rep has been allocated.

   Then initialize concrete bases -> SingleChannel constructor called
     In there, virtual bases already constructed, so move on to concrete
     ones -> ChannelBase constructor called

     In there, virtual bases already constructed, so just initialize
     data members. Then in the { } of the constructor,
     ModelComponent::init() is called, which will fix the manager and names
     of the component. Those names will correctly be those passed down from
     the BlueChannel constructor.

  same thing on the SimModule side: it virtually derives from ModelComponent
  and calls ModelComponent::init()

  then when you construct, eg, SaliencyMap, first you get
  ModelComponent::ModelComponent() and later once the { } constructor code
  of SimModule runs, the call to init() will fix the component.

  with multiple inheritance, eg VisualCortex derives from ComplexChannel and
  SimModule, pretty much the same thing happens, except that init() gets
  called twice, once from the ChannelBase c'tor and once from the SimModule
  c'tor... One way to avoid this would be to give a default c'tor to
  SimModule, but I'd rather not because things really become complicated
  then in case one does not know what he is doing (e.g., us 5 years from
  now) ;-)
*/

class SimModule : virtual public ModelComponent, public SimCallbackClient, public SimReqHandlerClient {
public:
  //! Constructor
  SimModule(OptionManager& mgr, const std::string& descrName, const std::string& tagName);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SimModule();

protected:
  // overload ModelComponent's start to also register our callbacks and request handlers
  virtual void start1();
};

// ######################################################################
//! Provide info that SimModule objects need to save() their results
class SimModuleSaveInfo : public ModelComponentSaveInfo {
public:
  //! Constructor
  SimModuleSaveInfo(nub::ref<FrameOstream> outfs, SimEventQueue& seq) :
    ModelComponentSaveInfo(), ofs(outfs), q(&seq) { }

  //! Destructor
  inline virtual ~SimModuleSaveInfo() { }

  // We will save to a FrameOstream
  const nub::ref<FrameOstream> ofs;

  // May need access to the SimEventQueue to fetch data off it
  SimEventQueue *q;
};

//! SIMMODULEINSTFUNC(ClsName): macro for instantiating Class of type SimMOdule for dynamically loaded modules
#define SIMMODULEINSTFUNC(ClsName) 					\
extern "C" nub::ref<SimModule> createModule##ClsName( OptionManager& manager) \
{ return nub::ref<SimModule>(new ClsName(manager));  } 

//Used as a function pointer for the SIMModuleInstFunc
typedef nub::ref<SimModule> createSimEventModule(OptionManager& manager);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMMODULE_H_DEFINED
