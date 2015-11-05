/*!@file Neuro/VisualCortexEyeMvt.C Implementation for the fake
  human-eye-movement visual cortex class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortexEyeMvt.C $
// $Id: VisualCortexEyeMvt.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Neuro/VisualCortexEyeMvt.H"

#include "Channels/InputFrame.H"
#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/Kernels.H"    // for gaussianBlob()
#include "Image/MathOps.H"    // for takeMax()
#include "Media/MediaOpts.H"  // for OPT_InputFrameDims
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Psycho/EyeTrace.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/SimTime.H"
#include "Util/sformat.H"
#include "Util/StringUtil.H"

#include <algorithm>
#include <cctype>
#include <cstdio>

// ######################################################################
VisualCortexEyeMvt::VisualCortexEyeMvt(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName) :
  VisualCortex(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  SIMREQHANDLER_INIT(SimReqVCXfeatures),
  SIMREQHANDLER_INIT(SimReqVCXmaps),
  itsFnames(&OPT_VCEMeyeFnames, this),
  itsSigma(&OPT_VCEMsigma, this),
  itsForgetFac(&OPT_VCEMforgetFac, this),
  itsDelay(&OPT_VCEMdelay, this),
  itsUseMax(&OPT_VCEMuseMax, this),
  itsSaccadeOnly(&OPT_VCEMsaccadeOnly, this),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsSaveOutput(&OPT_RawVisualCortexSaveOutput, this),// see Neuro/NeuroOpts.{H,C}
  itsOutputFactor(&OPT_RawVisualCortexOutputFactor, this),
  itsMaps(), itsEyeTrace(), itsEyeSample(), itsOutputCache()
{  }

// ######################################################################
VisualCortexEyeMvt::~VisualCortexEyeMvt()
{  }

// ######################################################################
void VisualCortexEyeMvt::start1()
{
  VisualCortex::start1();

  // parse our filename config string and instantiate all our eye traces:
  std::vector<std::string> tok;
  split(itsFnames.getVal(), ",", std::back_inserter(tok));
  if (tok.empty()) LFATAL("I cannot run without at least one eyetrace.");

  for (uint i = 0; i < tok.size(); i ++)
    {
      LINFO("Instantiating EyeTrace %03d with file '%s'", i, tok[i].c_str());
      rutz::shared_ptr<EyeTrace> et(new EyeTrace(tok[i], PixRGB<byte>(255)));
      itsEyeTrace.push_back(et);
      itsEyeSample.push_back(itsDelay.getVal());
    }

  // create empty maps:
  itsMaps.reset(itsEyeTrace.size());
}

// ######################################################################
void VisualCortexEyeMvt::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
  const SimTime t = q.now();
  const uint sml = itsLevelSpec.getVal().mapLevel();

  // do some forgetting:
  if (itsForgetFac.getVal() != 1.0f) {
    for (uint i = 0; i < itsMaps.size(); ++i) itsMaps[i] *= itsForgetFac.getVal();
    itsOutputCache.freeMem();
  }

  // evolve all our eye traces and see what they have:
  bool keepgoing = true;
  while(keepgoing) { // will go on until no tracker has any new data
    keepgoing = false;

    // loop over our eye traces:
    for (uint i = 0; i < itsEyeTrace.size(); i ++)
      if (itsEyeTrace[i]->hasData(itsEyeSample[i] - itsDelay.getVal(), t)) {
        // ok, this warrants that we continue our while() loop:
        keepgoing = true;

        // get the next data sample:
        rutz::shared_ptr<EyeData> data = itsEyeTrace[i]->data(itsEyeSample[i]);

        CLDEBUG("Eye trace %03u [%07" ZU "] (%d, %d) at %.1fms",
                i, itsEyeSample[i], data->position().i, data->position().j,
                itsEyeSample[i] * itsEyeTrace[i]->period().msecs());

        // stop here if we are only interested in saccades but there is
        // not one now, otherwise use the saccade endpoint for our
        // plots; if not only interested in saccades, use all samples:
        float ex, ey;
        if (itsSaccadeOnly.getVal()) {
          if (data->hasSaccadeTargetData() == false) { ++ itsEyeSample[i]; continue; }
          data->getSaccadeTarget(ex, ey);
        } else data->getPosition(ex, ey);

        // convert eye coords to scale of saliency map:
        Point2D<int> p(int(ex / float(1 << sml) + 0.4999f), int(ey / float(1 << sml) + 0.4999f));

        // inject new blob if valid coords:
        if (itsMaps[i].coordsOk(p))
          {
            if (itsSigma.getVal() > 0.0f)
              {
                // draw a blob at current eye:
                Image<float> blob = gaussianBlob<float>(itsMaps[i].getDims(), p, itsSigma.getVal(), itsSigma.getVal());

                // take max between current and old blobs:
                itsMaps[i] = takeMax(itsMaps[i], blob);
              }
            else
              // single spike mode:
              itsMaps[i].setVal(p, itsMaps[i].getVal(p) + 1.0f);

            // our maps have changed, hence our output cache is now invalid:
            itsOutputCache.freeMem();
          }

        // ready for next eye movement sample:
        ++ itsEyeSample[i];
      }
  }
}

// ######################################################################
void VisualCortexEyeMvt::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // start new maps fresh if first time:
  if (itsMaps[0].initialized() == false)
    {
      // compute map dims:
      const int w = e->frame().getWidth() >> itsLevelSpec.getVal().mapLevel();
      const int h = e->frame().getHeight() >> itsLevelSpec.getVal().mapLevel();

      for (uint i = 0; i < itsMaps.size(); i ++) itsMaps[i].resize(w, h, true);

      // invalidate our output cache
      itsOutputCache.freeMem();
    }

  // Our internal maps change at every clock tick, but it may be a bit
  // too intensive to post a new VisualCortexOutput SimEvent at that
  // rate. So instead we will do it only each time there is a new
  // input frame, like other VisualCortex derivatives do:
  q.post(rutz::make_shared(new SimEventVisualCortexOutput(this, getOutput())));
}

// ######################################################################
void VisualCortexEyeMvt::handleSimReqVCXfeatures(SimEventQueue& q, rutz::shared_ptr<SimReqVCXfeatures>& r)
{
  // just return a single feature, our map value at the requested location:
  const uint sml = itsLevelSpec.getVal().mapLevel();
  const Image<float> out = getOutput();
  Point2D<int> p((r->loc().i + sml/2) >> sml, (r->loc().j + sml/2) >> sml);
  if (out.coordsOk(p)) r->features().push_back(out.getVal(p));
  else r->features().push_back(0.0F);
}

// ######################################################################
void VisualCortexEyeMvt::handleSimReqVCXmaps(SimEventQueue& q, rutz::shared_ptr<SimReqVCXmaps>& r)
{
  r->populateChannelMaps(this);
}

// ######################################################################
Image<float> VisualCortexEyeMvt::getOutput()
{
  // do we already have it cached?
  if (itsOutputCache.initialized()) return itsOutputCache;

  // compute output from all human maps:
  Image<float> out = itsMaps[0];
  if (itsUseMax.getVal()) for (uint idx = 1; idx < itsMaps.size(); idx ++) out = takeMax(out, itsMaps[idx]);
  else for (uint idx = 1; idx < itsMaps.size(); idx ++) out += itsMaps[idx];

  // output is now typically in the (0.0..0.1) range; we want saliency map input current in nA:
  out *= 50.0f * itsOutputFactor.getVal();
  float mi, ma; getMinMax(out, mi, ma);
  LINFO("Salmap input range is [%f .. %f] nA", mi * 1.0e9F, ma * 1.0e9F);
  LINFO("Computed VisualCortex output.");

  itsOutputCache = out;

  return out;
}

// ######################################################################
void VisualCortexEyeMvt::onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

  // save our own output:
  if (itsSaveOutput.getVal())
    ofs->writeFloat(getOutput(), FLOAT_NORM_PRESERVE, "VCO",
                    FrameInfo("visual cortex eyemvt output (input to saliency map)", SRC_POS));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
