/*!@file Channels/IntegerRawVisualCortex.C */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerRawVisualCortex.C $
// $Id: IntegerRawVisualCortex.C 11584 2009-08-12 05:24:46Z itti $
//

#include "Channels/IntegerRawVisualCortex.H"

#include "Channels/ChannelOpts.H"
#include "Channels/IntegerColorChannel.H"
#include "Channels/IntegerFlickerChannel.H"
#include "Channels/IntegerIntensityChannel.H"
#include "Channels/IntegerMotionChannel.H"
#include "Channels/IntegerOrientationChannel.H"
#include "Component/GlobalOpts.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Image/ColorOps.H"   // for luminance()
#include "Image/IntegerMathOps.H"
#include "Image/MathOps.H"    // for distance()
#include "Image/Pixels.H"
#include "Image/PyramidCache.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H" // for chamfer34()
#include "Channels/ChannelOpts.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/TextLog.H"
#include "Util/sformat.H"
#include "rutz/mutex.h"
#include "rutz/trace.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <vector>
#include <sstream>

// ######################################################################
IntegerRawVisualCortex::
IntegerRawVisualCortex(OptionManager& mgr,
                    nub::ref<IntegerMathEngine> eng,
                    const std::string& descrName,
                    const std::string& tag) :
  IntegerComplexChannel(mgr, descrName, tag, UNKNOWN, eng),
  itsType(&OPT_IntegerRawVisualCortexChans, this),
  itsLogFile(&OPT_TextLogFile, this),
  itsNormType(&OPT_MaxNormType, this), // see Channels/ChannelOpts.{H,C}
  itsUseRandom(&OPT_UseRandom, this),  // see Component/ModelManager.{H,C}
  itsOutputFactor(&OPT_RawVisualCortexOutputFactor, this), // idem
  itsUseOlderVersion(&OPT_UseOlderVersion, this), // Channels/ChannelOpts.{H,C}
  itsLevelSpec(&OPT_LevelSpec, this),
  itsSaveOutput(&OPT_RawVisualCortexSaveOutput, this),
  itsUseMax(&OPT_VCXuseMax, this)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
IntegerRawVisualCortex::~IntegerRawVisualCortex()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void IntegerRawVisualCortex::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  IntegerComplexChannel::start1();

  for (uint i = 0; i < this->numChans(); ++i)
    LINFO("Top-level channel (%u/%u): level=%s feature=%s submaps=%u name=%s",
          i+1, uint(this->numChans()),
          featureHierarchyName(featureHierarchyLevel(this->subChan(i)->visualFeature())),
          featureName(this->subChan(i)->visualFeature()),
          this->subChan(i)->numSubmaps(), this->subChan(i)->descriptiveName().c_str());
}

// ######################################################################
void IntegerRawVisualCortex::stop2()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  IntegerComplexChannel::stop2();
}

// ######################################################################
Image<int> IntegerRawVisualCortex::getChannelOutputMap(IntegerChannel& chan) const
{
  float total_weight = 0.0;

  for (uint i = 0; i < this->numChans(); ++i)
    {
      nub::ref<IntegerChannel> c = this->subChan(i);

      if (itsUseOlderVersion.getVal())
        {
          const float w = float(this->getSubchanTotalWeight(*c) / c->numSubmaps());
          total_weight += w;
        }
      else
        {
          const float w = float(this->getSubchanTotalWeight(*c));
          total_weight += w;
        }
    }

  /* We want to compute

                  weight
         img * ------------
               total_weight


     To do that without overflowing, we compute it as


                  weight      256
         img * ------------ * ---
               total_weight   256

          img       weight * 256
      = ( --- ) * ( ------------ )
          256       total_weight
  */

  const int scalebits = 8;

  Image<int> chanOut = chan.getOutputInt();
  chanOut >>= scalebits;

  if (itsUseOlderVersion.getVal())
    {
      const float w = float(this->getSubchanTotalWeight(chan) / chan.numSubmaps());

      const int iw = int( 0.5 + (w*(1<<scalebits)) / total_weight );

      chanOut *= iw;
      LINFO("%s weight %d/%d",
            chan.tagName().c_str(), iw, (1<<scalebits));
    }
  else
    {
      const float w = float(this->getSubchanTotalWeight(chan));

      const int iw = int( 0.5 + (w*(1<<scalebits)) / total_weight );

      chanOut *= iw;
      LINFO("%s weight %d/%d", chan.tagName().c_str(), iw, (1<<scalebits));
    }

  return chanOut;
}

// ######################################################################
Image<int> IntegerRawVisualCortex::postProcessOutputMap(const Image<int>& outmap)
{
  Image<int> result = outmap;

  // let's apply a last maxNormalization to the map:
  switch(itsNormType.getVal())
    {
    case VCXNORM_LANDMARK:
    case VCXNORM_SURPRISE:
      LFATAL("Unsupported VCXNORM type");
      break;
    default:
      result = intgMaxNormalize(result, 0, 32768, itsNormType.getVal());
      LDEBUG("%s(%d .. %d)", maxNormTypeName(itsNormType.getVal()), 0, 32768);
      break;
    }

  return result;
}

// ######################################################################
void IntegerRawVisualCortex::doInputInt(const IntegerInput& inp,
                                     const SimTime& t,
                                     PyramidCache<int>* cache,
                                     const Image<byte>& clipMask)
{
  ASSERT(inp.grayInt().initialized());

  // optimization: if we have no channels, then do a quick return:
  if (this->numChans() == 0) return;

  rutz::mutex_lock_class lock;
  if (cache && cache->gaussian5.beginSet(inp.grayInt(), &lock))
    {
      cache->gaussian5.endSet
        (inp.grayInt(),
         intgBuildPyrGaussian
         (inp.grayInt(), itsLevelSpec.getVal().maxDepth(),
          5, this->getImath()),
         &lock);
    }

  // process the channels:
  for (uint i = 0; i < this->numChans(); ++i)
    {
      // get a pointer to the channel of interest:
      nub::ref<IntegerChannel> chan = this->subChan(i);

      // feed the input to the channel of interest:
      chan->inputInt(inp, t, cache, clipMask);
      LINFO("Input to %s channel %s ok.", featureHierarchyName(featureHierarchyLevel(chan->visualFeature())),
            chan->tagName().c_str());
    }
}

// ######################################################################
Image<int> IntegerRawVisualCortex::combineOutputsInt()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<int> output;

  // ... OK, we have to recompute the output:
  for (uint i = 0; i < this->numChans(); ++i)
    {
      nub::ref<IntegerChannel> ch = subChan(i);

      // CAUTION: if you modify code here, modify input() as well,
      // in the section that is triggered by itsConserveMemory:
      if (this->getSubchanTotalWeight(*ch) == 0.0) continue;
      if (ch->outputAvailable() == false) continue;

      const Image<int> chanOut = getChannelOutputMap(*ch);

      // downSizeClean() gracefully does nothing if chanOut is
      // already the right size. (Eventually we might like to have a
      // way for IntegerRawVisualCortex to enforce a particular chanOut size
      // on its own, rather than just taking the size from the first
      // channel in the array.)
      if (output.initialized() == false)
        output = chanOut;
      else
        {
          // sum or take max across channels?
          Image<int> o = intgDownSize(chanOut, output.getDims(), 9, this->getImath());
          if (itsUseMax.getVal()) output = takeMax(output, o); else output += o;
        }
    }

  // Ensure that we still return a valid image even if we have no channels
  // that has a valid output in the IntegerRawVisualCortex:
  if (output.initialized() == false)
    {
      int sml = itsLevelSpec.getVal().mapLevel();
      output = Image<int>(this->getInputDims().w() >> sml, this->getInputDims().h() >> sml, ZEROS);
      return output;
    }

  if (MYLOGVERB >= LOG_DEBUG)
    {
      int mi, ma; getMinMax(output, mi, ma);
      LDEBUG("Raw output range is [%d .. %d]", mi, ma);
    }

  // post-process the output in a manner than may depend on which
  // variant of IntegerRawVisualCortex we may embody:
  output = postProcessOutputMap(output);

  LINFO("Computed IntegerRawVisualCortex output.");
  return output;
}

// ######################################################################
Image<float> IntegerRawVisualCortex::getOutput()
{
  // using Image::operator*() we will simultenaously convert to float
  // and apply our output factor, looping only once over the pixels:
  const float fac = itsOutputFactor.getVal() / (1 << this->getMathEngine()->getNbits());
  Image<float> output = getOutputInt() * fac;

  float mi, ma; getMinMax(output, mi, ma);
  LINFO("Salmap input range is [%f .. %f] nA", mi * 1.0e9F, ma * 1.0e9F);

  return output;
}

// ######################################################################
void IntegerRawVisualCortex::paramChanged(ModelParamBase* const param,
                                          const bool valueChanged,
                                          ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);
  OptionManager& mgr = this->getManager();
  nub::ref<IntegerMathEngine> eng = this->getMathEngine();

  if (param == &itsType) {
    // kill any existing channels:
    this->removeAllSubChans();
    ASSERT(this->numChans() == 0);

    // Parse the param string. format here is a series of "X[:w.w]"
    // where X is any of "ICOFM..." and the optional w.w may be
    // channel weights. See note in RawVisualCortex for why we here
    // first parse the arg string (where letters may be specified in
    // various orders), and then implement the channels (in fixed order):
    uint i = 0; const std::string& str = itsType.getVal(); const uint len = str.length();
    double wc = 0.0, wi = 0.0, wo = 0.0, wf = 0.0, wm = 0.0;

    while (i < len) {
      char c = str[i];  // the channel to implement
      double weight = 1.0;  // default weight value

      // do we have a weight specification?
      if (i < len - 1 && str[i+1] == ':') {
        if (i >= len - 2) LFATAL("Missing channel weight value");
        uint end = str.find_first_not_of(".0123456789", i+2);
        std::stringstream s; s << str.substr(i+2, end); s >> weight;
        i = end;  // ready for next one
      } else ++i;

      switch(c) {
      case 'C': wc = weight; break;
      case 'I': wi = weight; break;
      case 'O': wo = weight; break;
      case 'F': wf = weight; break;
      case 'M': wm = weight; break;
      default: LFATAL("Unsupported channel type '%c' with weight %f", c, weight);
      }
    }

      // Create the channel and assign weight:
    if (wc) {
      LINFO("Using a Double Opponent ColorChannel with weight %f", wc);
      addSubChan(makeSharedComp(new IntegerColorChannel(mgr, eng)), "int-color");
      setSubchanTotalWeight("int-color", wc);
    }
    if (wf) {
      LINFO("Using a FlickerChannel with weight %f", wf);
      addSubChan(makeSharedComp(new IntegerFlickerChannel(mgr, eng)), "int-flicker");
      setSubchanTotalWeight("int-flicker", wf);
    }
    if (wi) {
      LINFO("Using an IntensityChannel with weight %f", wi);
      addSubChan(makeSharedComp(new IntegerIntensityChannel(mgr, eng)), "int-intensity");
      setSubchanTotalWeight("int-intensity", wi);
    }
    if (wo) {
      LINFO("Using an OrientationChannel with weight %f", wo);
      addSubChan(makeSharedComp(new IntegerOrientationChannel(mgr, eng)), "int-orientation");
      setSubchanTotalWeight("int-orientation", wo);
    }
    if (wm) {
      LINFO("Using a MotionChannel with weight %f", wm);
      addSubChan(makeSharedComp(new IntegerMotionChannel(mgr, eng)), "int-motion");
      setSubchanTotalWeight("int-motion", wm);
    }

    // make sure options are exported for all channels:
    for (uint i = 0; i < this->numChans(); ++i) this->subChan(i)->exportOptions(MC_RECURSE);
  }
}

// ######################################################################
void IntegerRawVisualCortex::saveResults(const nub::ref<FrameOstream>& ofs)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // save our own output:
  if (itsSaveOutput.getVal())
    ofs->writeFloat(Image<float>(getOutputInt()), FLOAT_NORM_0_255, "IVCO",
                    FrameInfo("integer visual cortex output (input to saliency map)", SRC_POS));

  // save our channel outputs:
  for (uint i = 0; i < numChans(); ++i) subChan(i)->saveResults(ofs);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
