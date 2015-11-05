/*!@file Neuro/AttentionGuidanceMap.C Implementation for task-relevance map class */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/AttentionGuidanceMap.C $
// $Id: AttentionGuidanceMap.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Neuro/AttentionGuidanceMap.H"

#include "Channels/ChannelBase.H"
#include "Component/OptionManager.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"  // for rescale()
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimulationOpts.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/log.H"

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work
#include "ModelNeuron/StructurePlot.H"
#include "ModelNeuron/SimStructureOpts.H"
#endif 

// ######################################################################
// ######################################################################
// ########## AttentionGuidanceMap implementation
// ######################################################################
// ######################################################################

AttentionGuidanceMap::
AttentionGuidanceMap(OptionManager& mgr,
                     const std::string& descrName,
                     const std::string& tagName) :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventSaliencyMapOutput),
  SIMCALLBACK_INIT(SimEventTaskRelevanceMapOutput),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsSaveResults(&OPT_AGMsaveResults, this), // see Neuro/NeuroOpts.{H,C}
  itsOutputCache()
{ }

// ######################################################################
AttentionGuidanceMap::~AttentionGuidanceMap()
{ }

// ######################################################################
void AttentionGuidanceMap::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void AttentionGuidanceMap::save1(const ModelComponentSaveInfo& sinfo)
{
  if (itsSaveResults.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

      ofs->writeFloat(this->getV(), FLOAT_NORM_PRESERVE, "AGM",
                      FrameInfo("overall attention guidance map", SRC_POS));
    }
}

// ######################################################################
void AttentionGuidanceMap::
onSimEventSaliencyMapOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaliencyMapOutput>& e)
{
  itsOutputCache.freeMem();
  this->inputBU(e->sm());
}

// ######################################################################
void AttentionGuidanceMap::
onSimEventTaskRelevanceMapOutput(SimEventQueue& q, rutz::shared_ptr<SimEventTaskRelevanceMapOutput>& e)
{
  itsOutputCache.freeMem();
  this->inputTD(e->trm());
}

// ######################################################################
void AttentionGuidanceMap::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
  doClockTick(q);
}

// ######################################################################
void AttentionGuidanceMap::doClockTick(SimEventQueue& q)
{
    // post an event with our output:
  if (itsOutputCache.initialized() == false) itsOutputCache = this->getV();
  if (itsOutputCache.initialized())
    q.post(rutz::make_shared(new SimEventAttentionGuidanceMapOutput(this, itsOutputCache)));
}

// ######################################################################
// ######################################################################
// ########## AttentionGuidanceMapConfigurator implementation
// ######################################################################
// ######################################################################
AttentionGuidanceMapConfigurator::
AttentionGuidanceMapConfigurator(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsAGMtype(&OPT_AttentionGuidanceMapType, this),
  itsAGM(new AttentionGuidanceMapStd(mgr))
{
  addSubComponent(itsAGM);
}

// ######################################################################
AttentionGuidanceMapConfigurator::~AttentionGuidanceMapConfigurator()
{  }

// ######################################################################
nub::ref<AttentionGuidanceMap>
AttentionGuidanceMapConfigurator::getAGM() const
{ return itsAGM; }

// ######################################################################
void AttentionGuidanceMapConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsAGMtype) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current AttentionGuidanceMap will unexport its
    // command-line options):
    removeSubComponent(*itsAGM);

    // instantiate a SM of the appropriate type:
    if (itsAGMtype.getVal().compare("Std") == 0)          // standard
      itsAGM.reset(new AttentionGuidanceMapStd(getManager()));
    else if (itsAGMtype.getVal().compare("Opt") == 0)    // optimized
      itsAGM.reset(new AttentionGuidanceMapOpt(getManager()));
#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work
    else if (itsAGMtype.getVal().compare("SC") == 0)    // superior colliculus
      itsAGM.reset(new AttentionGuidanceMapSC(getManager()));
    else if (itsAGMtype.getVal().compare("NF") == 0)    // neural field
      itsAGM.reset(new AttentionGuidanceMapNF(getManager()));
#endif
    else
      LFATAL("Unknown AGM type %s", itsAGMtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsAGM);

    // tell the controller to export its options:
    itsAGM->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected AGM of type %s", itsAGMtype.getVal().c_str());
  }
}

// ######################################################################
// ######################################################################
// ########## AttentionGuidanceMapStd implementation
// ######################################################################
// ######################################################################ccc_

// ######################################################################
AttentionGuidanceMapStd::
AttentionGuidanceMapStd(OptionManager& mgr,
                        const std::string& descrName,
                        const std::string& tagName) :
  AttentionGuidanceMap(mgr, descrName, tagName),
  itsBUmap(), itsTDmap()
{  }

// ######################################################################
AttentionGuidanceMapStd::~AttentionGuidanceMapStd()
{ }

// ######################################################################
void AttentionGuidanceMapStd::reset()
{ itsBUmap.freeMem(); itsTDmap.freeMem(); }

// ######################################################################
void AttentionGuidanceMapStd::inputBU(const Image<float>& current)
{ itsBUmap = current; }

// ######################################################################
void AttentionGuidanceMapStd::inputTD(const Image<float>& current)
{ itsTDmap = current; }

// ######################################################################
Image<float> AttentionGuidanceMapStd::getV() const
{
  Image<float> ret;
  if (!itsBUmap.initialized())
    ret = itsTDmap;
  else if (!itsTDmap.initialized())
    ret = itsBUmap;
  else if (itsBUmap.getDims() == itsTDmap.getDims())
    ret = itsBUmap * itsTDmap;
  else
    LINFO("Bottom-up %dx%d vs. top-down %dx%d dims mismatch",
           itsBUmap.getWidth(), itsBUmap.getHeight(),
           itsTDmap.getWidth(), itsTDmap.getHeight());

  return ret;
}

// ######################################################################
// ######################################################################
// ########## AttentionGuidanceMapOpt implementation
// ######################################################################
// ######################################################################

// ######################################################################
AttentionGuidanceMapOpt::
AttentionGuidanceMapOpt(OptionManager& mgr,
                        const std::string& descrName,
                        const std::string& tagName) :
  AttentionGuidanceMap(mgr, descrName, tagName),
  itsBUmap(), itsTDmap()
{  }

// ######################################################################
AttentionGuidanceMapOpt::~AttentionGuidanceMapOpt()
{ }

// ######################################################################
void AttentionGuidanceMapOpt::reset()
{ itsBUmap.freeMem(); itsTDmap.freeMem(); }

// ######################################################################
void AttentionGuidanceMapOpt::inputBU(const Image<float>& current)
{ itsBUmap = current; }

// ######################################################################
void AttentionGuidanceMapOpt::inputTD(const Image<float>& current)
{ itsTDmap = current; }

// ######################################################################
Image<float> AttentionGuidanceMapOpt::getV() const
{
  Image<float> ret;
  float a = 0.998F, b = 6.603F;

  if (!itsBUmap.initialized())
    ret = itsTDmap;
  else if (!itsTDmap.initialized())
    ret = itsBUmap;
  else if (itsBUmap.getDims() == itsTDmap.getDims())
    ret = itsBUmap*a + itsTDmap*b + itsBUmap * itsTDmap;
  else
    LINFO("Bottom-up %dx%d vs. top-down %dx%d dims mismatch",
           itsBUmap.getWidth(), itsBUmap.getHeight(),
           itsTDmap.getWidth(), itsTDmap.getHeight());

  return ret;
}

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

// ######################################################################
// ######################################################################
// ########## AttentionGuidanceMapNeuralSim implementation
// ######################################################################
// ######################################################################

// ######################################################################
AttentionGuidanceMapNeuralSim::AttentionGuidanceMapNeuralSim(OptionManager& mgr,
                                                             const std::string& descrName,
                                                             const std::string& tagName) : 
  AttentionGuidanceMap(mgr, descrName, tagName),
  itsOutRate(&OPT_AGMoutputRate, this), itsTime(SimTime::ZERO())
{ }

// ######################################################################
AttentionGuidanceMapNeuralSim::~AttentionGuidanceMapNeuralSim() { }

// ######################################################################
void AttentionGuidanceMapNeuralSim::doClockTick(SimEventQueue& q)
{
  const SimTime interval(q.now() - itsTime);
  const int steps = (int)(interval.nsecs() / itsOutRate.getVal().nsecs());

  if (steps <= 0)
    update(q.now());
  else
    for (int ii = 0; ii < steps; ++ii)
      {
        itsTime += itsOutRate.getVal();
        update(itsTime);
        postMessage(q);
      }
}

// ######################################################################
// ######################################################################
// ########## AttentionGuidanceMapSC implementation
// ######################################################################
// ######################################################################

// ######################################################################
AttentionGuidanceMapSC::AttentionGuidanceMapSC(OptionManager& mgr,
                                               const std::string& descrName,
                                               const std::string& tagName) 
    : AttentionGuidanceMapNeuralSim(mgr, descrName, tagName), itsSC(new nsu::SupColliculusModule(mgr))
{ 
  addSubComponent(itsSC);
}

// ######################################################################
AttentionGuidanceMapSC::~AttentionGuidanceMapSC()
{ }

// ######################################################################
void AttentionGuidanceMapSC::reset()
{ itsSC->reset(); }

// ######################################################################
void AttentionGuidanceMapSC::inputBU(const Image<float>& current)
{ itsSC->setInput(current, 0); }//input to sgs

// ######################################################################
void AttentionGuidanceMapSC::inputTD(const Image<float>& current)
{ itsSC->setInput(current, 1); }//input to sgs

// ######################################################################
Image<float> AttentionGuidanceMapSC::getV() const
{ 
  LINFO("Superior Colliculus Guidance Map is an ImageSet, so this function is depricated here");
  return Image<float>();
}

// ######################################################################
void AttentionGuidanceMapSC::update(const SimTime& time)
{
  itsSC->update(time);
}

// ######################################################################
void AttentionGuidanceMapSC::postMessage(SimEventQueue& q)
{
  q.post(rutz::make_shared(new SimEventAttentionGuidanceMapOutput(this, itsSC->getSubV())));
}

// ######################################################################
void AttentionGuidanceMapSC::save1(const ModelComponentSaveInfo& sinfo)
{
  if (itsSaveResults.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;      
      ofs->writeRgbLayout(itsSC->getDisplay(), "AGM-SC", FrameInfo("SC Model Output", SRC_POS));
    }
}

// ######################################################################
// ######################################################################
// ########## AttentionGuidanceMapNF implementation
// ######################################################################
// ######################################################################

// ######################################################################
AttentionGuidanceMapNF::AttentionGuidanceMapNF(OptionManager& mgr,
                                               const std::string& descrName,
                                               const std::string& tagName) 
  : AttentionGuidanceMapNeuralSim(mgr, descrName, tagName), itsNF(new NeuralFieldModule(mgr))
{ 
  addSubComponent(itsNF);
}

// ######################################################################
AttentionGuidanceMapNF::~AttentionGuidanceMapNF()
{ }

// ######################################################################
void AttentionGuidanceMapNF::reset()
{ itsNF->reset(); }

// ######################################################################
void AttentionGuidanceMapNF::inputBU(const Image<float>& current)
{ itsNF->setInput(current, 0); }//input to sgs

// ######################################################################
void AttentionGuidanceMapNF::inputTD(const Image<float>& current)
{ itsNF->setInput(current, 0); }//input to sgs

// ######################################################################
Image<float> AttentionGuidanceMapNF::getV() const
{ return itsNF->getV(0); }//output from sgi

// ######################################################################
void AttentionGuidanceMapNF::update(const SimTime& time)
{
  itsNF->update(time);
}

// ######################################################################
void AttentionGuidanceMapNF::postMessage(SimEventQueue& q)
{
  q.post(rutz::make_shared(new SimEventAttentionGuidanceMapOutput(this, itsNF->getV(0))));
}

// ######################################################################
void AttentionGuidanceMapNF::save1(const ModelComponentSaveInfo& sinfo)
{
  if (itsSaveResults.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;      
      ofs->writeRgbLayout(itsNF->getDisplay(), "AGM-NF", FrameInfo("NF Model Output", SRC_POS));
    }
}

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
