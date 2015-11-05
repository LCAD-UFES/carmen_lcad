/*!@file Neuro/VisualCortex.C Implementation for visual cortex class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortex.C $
// $Id: VisualCortex.C 13065 2010-03-28 00:01:00Z itti $
//

#include "Neuro/VisualCortex.H"

#include "Channels/ChannelOpts.H"
#include "Channels/RawVisualCortex.H"
#include "Channels/IntegerRawVisualCortex.H"
#include "Channels/ChannelFacets.H"
#include "Component/GlobalOpts.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Image/ColorOps.H"   // for luminance()
#include "Image/MathOps.H"    // for distance()
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H" // for chamfer34()
#include "Media/MediaSimEvents.H"
#include "Neuro/EnvVisualCortex.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/WTAwinner.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/sformat.H"
#include "rutz/mutex.h"
#include "rutz/trace.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <vector>

// ######################################################################
// ######################################################################
VisualCortex::VisualCortex(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tag) :
  SimModule(mgr, descrName, tag)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
VisualCortex::~VisualCortex()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
// ######################################################################
VisualCortexStd::VisualCortexStd(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tag) :
  VisualCortex(mgr, descrName, tag),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  SIMREQHANDLER_INIT(SimReqVCXfeatures),
  SIMREQHANDLER_INIT(SimReqVCXmaps),
  SIMREQHANDLER_INIT(SimReqVCXchanVis),
  itsVCX(new RawVisualCortex(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);
 addSubComponent(itsVCX);
}

// ######################################################################
VisualCortexStd::VisualCortexStd(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tag, nub::ref<RawVisualCortex> vcx) :
  VisualCortex(mgr, descrName, tag),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  SIMREQHANDLER_INIT(SimReqVCXfeatures),
  SIMREQHANDLER_INIT(SimReqVCXmaps),
  SIMREQHANDLER_INIT(SimReqVCXchanVis),
  itsVCX(vcx)
{
GVX_TRACE(__PRETTY_FUNCTION__);
 addSubComponent(itsVCX);
}

// ######################################################################
VisualCortexStd::~VisualCortexStd()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void VisualCortexStd::onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // process the retina image:
  itsVCX->input(e->frame());

  // post an event with our output:
  q.post(rutz::make_shared(new SimEventVisualCortexOutput(this, itsVCX->getOutput())));
}

// ######################################################################
void VisualCortexStd::handleSimReqVCXfeatures(SimEventQueue& q, rutz::shared_ptr<SimReqVCXfeatures>& r)
{
  itsVCX->getFeatures(r->loc(), r->features());
}

// ######################################################################
void VisualCortexStd::handleSimReqVCXmaps(SimEventQueue& q, rutz::shared_ptr<SimReqVCXmaps>& r)
{
  r->populateChannelMaps(itsVCX.get());
}

// ######################################################################
void VisualCortexStd::handleSimReqVCXchanVis(SimEventQueue& q, rutz::shared_ptr<SimReqVCXchanVis>& r)
{
  LINFO("Triggering visitor %s", r->toString().c_str());
  r->preProcessing(itsVCX.get());
  itsVCX->accept(*(r->visitor()));
  r->postProcessing(itsVCX.get());
}

// ######################################################################
void VisualCortexStd::onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

  itsVCX->saveResults(ofs);
}

// ######################################################################
// ######################################################################
VisualCortexInt::VisualCortexInt(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tag) :
  VisualCortex(mgr, descrName, tag),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  SIMREQHANDLER_INIT(SimReqVCXfeatures),
  SIMREQHANDLER_INIT(SimReqVCXmaps),
  SIMREQHANDLER_INIT(SimReqVCXchanVis),
  itsIME(new IntegerMathEngine(mgr)),
  itsVCX(new IntegerRawVisualCortex(mgr, itsIME))
{
GVX_TRACE(__PRETTY_FUNCTION__);
 addSubComponent(itsIME);
 addSubComponent(itsVCX);

 // set some defaults:
 mgr.setOptionValString(&OPT_MaxNormType, "Maxnorm");
 mgr.setOptionValString(&OPT_DirectionChannelLowThresh, "0");
 mgr.setOptionValString(&OPT_IntChannelScaleBits, "16");
 mgr.setOptionValString(&OPT_IntMathLowPass5, "lp5optim");
 mgr.setOptionValString(&OPT_IntMathLowPass9, "lp9optim");
 mgr.setOptionValString(&OPT_RawVisualCortexOutputFactor, "1.0e-12");
}

// ######################################################################
VisualCortexInt::VisualCortexInt(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tag, nub::ref<IntegerMathEngine> ime,
                                 nub::ref<IntegerRawVisualCortex> vcx) :
  VisualCortex(mgr, descrName, tag),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  SIMREQHANDLER_INIT(SimReqVCXfeatures),
  SIMREQHANDLER_INIT(SimReqVCXmaps),
  SIMREQHANDLER_INIT(SimReqVCXchanVis),
  itsIME(ime),
  itsVCX(vcx),
  itsPcache()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 addSubComponent(itsIME);
 addSubComponent(itsVCX);

 // set some defaults:
 mgr.setOptionValString(&OPT_MaxNormType, "Maxnorm");
 mgr.setOptionValString(&OPT_DirectionChannelLowThresh, "0");
 mgr.setOptionValString(&OPT_IntChannelScaleBits, "16");
 mgr.setOptionValString(&OPT_IntMathLowPass5, "lp5optim");
 mgr.setOptionValString(&OPT_IntMathLowPass9, "lp9optim");
 mgr.setOptionValString(&OPT_RawVisualCortexOutputFactor, "1.0e-12");
}

// ######################################################################
VisualCortexInt::~VisualCortexInt()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void VisualCortexInt::onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // process the retina image:
  IntegerInput inp = IntegerInput::fromRgb(e->frame().colorByte(), itsIME->getNbits());
  itsVCX->inputInt(inp, q.now(), &itsPcache);

  // post an event with our output:
  Image<float> fout = itsVCX->getOutput();
  q.post(rutz::make_shared(new SimEventVisualCortexOutput(this, fout)));
}

// ######################################################################
void VisualCortexInt::handleSimReqVCXfeatures(SimEventQueue& q, rutz::shared_ptr<SimReqVCXfeatures>& r)
{
  itsVCX->getFeatures(r->loc(), r->features());
}

// ######################################################################
void VisualCortexInt::handleSimReqVCXmaps(SimEventQueue& q, rutz::shared_ptr<SimReqVCXmaps>& r)
{
  r->populateChannelMaps(itsVCX.get());
}

// ######################################################################
void VisualCortexInt::handleSimReqVCXchanVis(SimEventQueue& q, rutz::shared_ptr<SimReqVCXchanVis>& r)
{
  LFATAL("Not supported yet...");
  /*
  r->preProcessing(itsVCX.get());
  itsVCX->accept(*(r->visitor()));
  r->postProcessing(itsVCX.get());
  */
}

// ######################################################################
void VisualCortexInt::onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

  itsVCX->saveResults(ofs);
}

// ######################################################################
// ######################################################################
VisualCortexEnv::VisualCortexEnv(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tag) :
  VisualCortex(mgr, descrName, tag),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  SIMREQHANDLER_INIT(SimReqVCXfeatures),
  SIMREQHANDLER_INIT(SimReqVCXmaps),
  SIMREQHANDLER_INIT(SimReqVCXchanVis),
  itsVCX(new EnvVisualCortexFloat(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);
 addSubComponent(itsVCX);
}

// ######################################################################
VisualCortexEnv::~VisualCortexEnv()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void VisualCortexEnv::onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // process the retina image:
  itsVCX->input(e->frame().colorByte());

  // post an event with our output:
  q.post(rutz::make_shared(new SimEventVisualCortexOutput(this, itsVCX->getVCXmap())));
}

// ######################################################################
void VisualCortexEnv::handleSimReqVCXfeatures(SimEventQueue& q, rutz::shared_ptr<SimReqVCXfeatures>& r)
{
  LFATAL("Not supported yet...");
  /*
  itsVCX->getFeatures(r->loc(), r->features());
  */
}

// ######################################################################
void VisualCortexEnv::handleSimReqVCXmaps(SimEventQueue& q, rutz::shared_ptr<SimReqVCXmaps>& r)
{
  r->populateChannelMaps(itsVCX.get());
}

// ######################################################################
void VisualCortexEnv::handleSimReqVCXchanVis(SimEventQueue& q, rutz::shared_ptr<SimReqVCXchanVis>& r)
{
  LFATAL("Not supported yet...");
  /*
  r->preProcessing(itsVCX.get());
  itsVCX->accept(*(r->visitor()));
  r->postProcessing(itsVCX.get());
  */
}

// ######################################################################
void VisualCortexEnv::onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;
  /*
  itsVCX->saveResults(ofs);
  */
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
