/*!@file Channels/RawVisualCortex.C Implementation for visual cortex class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/RawVisualCortex.C $
// $Id: RawVisualCortex.C 15258 2012-04-18 23:05:58Z dberg $
//

#include "Image/OpenCVUtil.H" // must be included first to avoid possible type conflicts

#include "Channels/RawVisualCortex.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ChannelFacets.H"
#include "Channels/ColorChannel.H"
#include "Channels/CompositeColorChannel.H"
#include "Channels/ContourChannel.H"
#include "Channels/DepthChannel.H"
#include "Channels/DepthMotionChannel.H"
#include "Channels/DKLcolorChannel.H"
#include "Channels/DummyChannel.H"
#include "Channels/EndPointChannel.H"
#include "Channels/FlickerChannel.H"
#include "Channels/FlickerNBackChannel.H"
#include "Channels/ForegroundDetectionChannel.H"
#include "Channels/H2SVChannel.H"
#include "Channels/CIELabChannel.H"
#include "Channels/ImagizeColorChannel.H"
#include "Channels/InputFrame.H"
#include "Channels/IntensityBandChannel.H"
#include "Channels/IntensityChannel.H"
#include "Channels/Jet.H"
#include "Channels/LJunctionChannel.H"
#include "Channels/MotionChannel.H"
#include "Channels/MotionSpatioTemporalChannel.H"
#include "Channels/MotionOpticalFlowChannel.H"
#include "Channels/MSTChannel.H"
#include "Channels/FoeMSTChannel.H"
#include "Channels/MultiColorBandChannel.H"
#include "Channels/OrientationChannel.H"
#include "Channels/PedestrianChannel.H"
#include "Channels/SIFTChannel.H"
#include "Channels/ObjDetChannel.H"
#include "Channels/SkinHueChannel.H"
#include "Channels/SoxChannel.H"
#include "Channels/TJunctionChannel.H"
#include "Channels/XJunctionChannel.H"
#include "Channels/ZeroChannel.H"
#include "Component/GlobalOpts.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Image/ColorOps.H"   // for luminance()
#include "Image/MathOps.H"    // for distance()
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H" // for chamfer34()
#include "Media/MgzDecoder.H"
#include "Media/MgzEncoder.H"
#include "Media/MediaSimEvents.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/sformat.H"
#include "rutz/mutex.h"
#include "rutz/trace.h"
#include "Util/StringUtil.H" // for split

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

// ######################################################################
RawVisualCortex::RawVisualCortex(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tag) :
  ComplexChannel(mgr, descrName, tag, UNKNOWN),
  itsType(&OPT_RawVisualCortexChans, this),
  itsNormType(&OPT_MaxNormType, this), // see Channels/ChannelOpts.{H,C}
  itsUseRandom(&OPT_UseRandom, this),  // see Component/ModelManager.{H,C}
  itsOutputFactor(&OPT_RawVisualCortexOutputFactor, this), // idem
  itsNoise(&OPT_RawVisualCortexNoise, this), // idem
  itsUseOlderVersion(&OPT_UseOlderVersion, this), // Channels/ChannelOpts.{H,C}
  itsLevelSpec(&OPT_LevelSpec, this),
  itsSaveOutTo(&OPT_VCXsaveOutTo, this),
  itsLoadOutFrom(&OPT_VCXloadOutFrom, this),
  itsSaveRawCSOutTo(&OPT_VCXsaveRawCSOutTo, this),
  itsSaveOutput(&OPT_RawVisualCortexSaveOutput, this),
  itsUseMax(&OPT_VCXuseMax, this),
  itsWeightThresh(&OPT_VCXweightThresh, this),
  itsRawCSDims(&OPT_VCXsaveRawCSOutDims, this), 
  itsOutputMgzIn(),
  itsOutputMgzOut(),
  itsRawCSMgzOut()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 itsFrame = 0;
}

// ######################################################################
RawVisualCortex::~RawVisualCortex()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
namespace
{
  struct ChannelSorter
  {
    ChannelSorter(const RawVisualCortex* vcx_,
                  bool dosubmaps_)
      :
      vcx(vcx_), dosubmaps(dosubmaps_)
    {}

    bool operator()(uint i, uint j)
    {
      nub::ref<ChannelBase> chan1 = vcx->subChan(i);
      nub::ref<ChannelBase> chan2 = vcx->subChan(j);

      const int level1 = int(featureHierarchyLevel(chan1->visualFeature()));
      const int level2 = int(featureHierarchyLevel(chan2->visualFeature()));

      if (level1 < level2)
        return true;

      // else ...

      if (dosubmaps && level1 == level2)
        return chan1->numSubmaps() > chan2->numSubmaps();

      // else ...

      return false;
    }

    const RawVisualCortex* vcx;
    bool dosubmaps;
  };
}

// ######################################################################
void RawVisualCortex::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ComplexChannel::start1();

  // initialize out input or output MGZ handlers if necessary:
  if (itsSaveOutTo.getVal().empty() == false)
    itsOutputMgzOut = rutz::make_shared(new MgzEncoder(itsSaveOutTo.getVal(), 9));

  if (itsLoadOutFrom.getVal().empty() == false)
    itsOutputMgzIn = rutz::make_shared(new MgzDecoder(itsLoadOutFrom.getVal()));
  
  //write out some info to command line
  for (uint i = 0; i < this->numChans(); ++i)
    {
      LINFO("Top-level channel (%u/%u): level=%s feature=%s submaps=%u name=%s",
            i+1, this->numChans(),
            featureHierarchyName(featureHierarchyLevel(this->subChan(i)->visualFeature())),
            featureName(this->subChan(i)->visualFeature()),
            this->subChan(i)->numSubmaps(),
            this->subChan(i)->descriptiveName().c_str());
    }
}

// ######################################################################
void RawVisualCortex::start2()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ComplexChannel::start2();

  //prepare mgz for raw cs output and a descriptive text file
  if (itsSaveRawCSOutTo.getVal().empty() == false)
  {
    itsRawCSMgzOut = rutz::make_shared(new MgzEncoder(itsSaveRawCSOutTo.getVal(), 9));
    
    std::vector<std::string> tokens;
    split(itsSaveRawCSOutTo.getVal(), ".", std::back_inserter(tokens));
    const std::string fname(tokens[0] + "-SubMapInfo");
    
    std::ofstream *itsOutFile = new std::ofstream(fname.c_str());
    if (itsOutFile->is_open() == false)
      LFATAL("Cannot open '%s' for writing", fname.c_str());
    else
    {
      for (uint i = 0; i < this->numSubmaps(); ++i)
        (*itsOutFile) << getSubmapName(i) << std::endl;

      itsOutFile->close();
    }
  }
}

// ######################################################################
void RawVisualCortex::stop2()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsOutputMgzOut.is_valid()) itsOutputMgzOut->close();

  if (itsRawCSMgzOut.is_valid()) itsRawCSMgzOut->close();

  ComplexChannel::stop2();
}

// ######################################################################
void RawVisualCortex::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);
  OptionManager& mgr = this->getManager();

  if (param == &itsType) {
    // kill any existing channels:
    this->removeAllSubChans();
    ASSERT(this->numChans() == 0);

    // Parse the param string. format here is a series of "X[:w.w]"
    // where X is any of "ICOFM..." and the optional w.w may be
    // channel weights. NOTE: sadly, the final saliency map is (very
    // slightly) affected by the order in which the different channels
    // sum into it. If we just create the channel objects as we parse
    // the param string here, this will lead to --vc-chans=CIO might
    // give a slightly different saliency map than
    // --vc-chans=IOC. This seems to be due to rounding during
    // floating-point computations. Hence, here we parse the param
    // string in two steps: first, parse the user input and find out
    // which channels should be built; second, build the channels, in
    // a fixed order than does not depend on the ordering of the
    // letters in the user input:
    uint i = 0; const std::string& str = itsType.getVal(); const uint len = str.length();
    std::vector<double> ww(('z' - 'A')+1, 0.0);

    while (i < len) {
      char c = str[i];  // the channel to implement
      if (! ((c >= 'A' && c <= 'Z') || (c >= 'a' && c < 'z')) )
        LFATAL("Invalid channel character 0x%x (%c)", c, c);
      ww[c - 'A'] =  1.0;   // default weight value

      // do we have a weight specification?
      if (i < len - 1 && str[i+1] == ':') {
        if (i >= len - 2) LFATAL("Missing channel weight value");
        uint end = str.find_first_not_of(".0123456789", i+2);
        std::stringstream s; s << str.substr(i+2, end); s >> ww[c - 'A'];
        i = end;  // ready for next one
      } else ++i;
    }

    // Create the channels and assign weights, creating in a fixed order:
    if (double w = ww['C' - 'A']) {
      LINFO("Using a Double Opponent ColorChannel with weight %f", w);
      addSubChan(makeSharedComp(new ColorChannel(mgr)), "color", w);
    }
    if (double w = ww['S' - 'A']) {
      LINFO("Using a Composite ColorChannel with weight %f", w);
      addSubChan(makeSharedComp(new CompositeColorChannel(mgr)), "composite-color", w);
    }
    if (double w = ww['F' - 'A']) {
      LINFO("Using a FlickerChannel with weight %f", w);
      addSubChan(makeSharedComp(new FlickerChannel(mgr)), "flicker", w);
    }
    if (double w = ww['I' - 'A']) {
      LINFO("Using an IntensityChannel with weight %f", w);
      addSubChan(makeSharedComp(new IntensityChannel(mgr)), "intensity", w);
    }
    if (double w = ww['O' - 'A']) {
      LINFO("Using an OrientationChannel with weight %f", w);
      addSubChan(makeSharedComp(new OrientationChannel(mgr)), "orientation", w);
    }
    if (double w = ww['M' - 'A']) {
      LINFO("Using a MotionChannel with weight %f", w);
      addSubChan(makeSharedComp(new MotionChannel(mgr)), "motion", w);
    }

    if (double w = ww['s' - 'A']) {
     LINFO("Using a MotionSpatioTemporalChannel with weight %f", w);
     addSubChan(makeSharedComp(new MotionSpatioTemporalChannel(mgr)), "motionSpatioTemporal", w);
    }

    if (double w = ww['o' - 'A']) {
     LINFO("Using a MotionOpticalFlowChannel with weight %f", w);
     addSubChan(makeSharedComp(new MotionOpticalFlowChannel(mgr)), "motionOpticalFlow", w);
    }

    if (double w = ww['B' - 'A']) {
     LINFO("Using a FOE MSTChannel with weight %f", w);
      if (!this->hasSubChan("motion"))
        LFATAL("Must have a Motion Channel to build an FoeMSTChannel; specify 'M' before 'B' in --vc-chans");
      addSubChan(makeSharedComp(new FoeMSTChannel(mgr, dynCastWeak<MotionChannel>
                                                     (subChan("motion")))), "FoeMST", w);
    }

    if (double w = ww['L' - 'A']) {
      LINFO("Using an L Junction Channel with weight %f", w);
      if (!this->hasSubChan("orientation"))
        LFATAL("Must have an OrientationChannel to build an LJunctionChannel; specify 'O' before 'L' in --vc-chans");
      addSubChan(makeSharedComp(new LJunctionChannel(mgr, dynCastWeak<OrientationChannel>
                                                     (subChan("orientation")))), "LJunction", w);
    }

    if (double w = ww['T' - 'A']) {
      LINFO("Using a T Junction Channel with weight %f", w);
      if (!this->hasSubChan("orientation"))
        LFATAL("Must have an OrientationChannel to build an TJunctionChannel; specify 'O' before 'T' in --vc-chans");
      addSubChan(makeSharedComp(new TJunctionChannel(mgr, dynCastWeak<OrientationChannel>
                                                     (subChan("orientation")))), "TJunction", w);
    }

    if (double w = ww['X' - 'A']) {
      LINFO("Using an X Junction Channel with weight %f", w);
      if (!this->hasSubChan("orientation"))
        LFATAL("Must have an OrientationChannel to build an XJunctionChannel; specify 'O' before 'X' in --vc-chans");
      addSubChan(makeSharedComp(new XJunctionChannel(mgr, dynCastWeak<OrientationChannel>
                                                     (subChan("orientation")))), "XJunction", w);
    }

    if (double w = ww['E' - 'A']) {
      LINFO("Using an EndPoint Channel with weight %f", w);
      if (!this->hasSubChan("orientation"))
        LFATAL("Must have an OrientationChannel to build an EndPointChannel; specify 'O' before 'E' in --vc-chans");
      addSubChan(makeSharedComp(new EndPointChannel(mgr, dynCastWeak<OrientationChannel>
                                                    (subChan("orientation")))), "EndPoint", w);
    }

    if (double w = ww['G' - 'A']) {
      LINFO("Using Gaussian multi-color band channels with weight %f", w);
      addSubChan(makeSharedComp(new MultiColorBandChannel(mgr)), "MultiColorBand", w);
    }
    if (double w = ww['D' - 'A']) {
      LINFO("Using a Dummy Channel with weight %f", w);
      addSubChan(makeSharedComp(new DummyChannel(mgr)), "Dummy", w);
    }
    if (double w = ww['K' - 'A']) {
      LINFO("Using a SkinHue channel with weight %f", w);
      addSubChan(makeSharedComp(new SkinHueChannel(mgr)), "SkinHue", w);
    }
    if (double w = ww['N' - 'A']) {
      LINFO("Using an Intensity Band channel with weight %f", w);
      addSubChan(makeSharedComp(new IntensityBandChannel(mgr)), "IntensityBand", w);
    }
    if (double w = ww['P' - 'A']) {
      LINFO("Using a SIFT channel with weight %f", w);
      addSubChan(makeSharedComp(new SIFTChannel(mgr)), "SIFTBand", w);
    }
    if (double w = ww['H' - 'A']) {
      LINFO("Using a H2SV channel with weight %f", w);
      addSubChan(makeSharedComp(new H2SVChannel(mgr)), "H2SVcolor", w);
    }
    if (double w = ww['Q' - 'A']) {
      LINFO("Using a CIELab channel with weight %f", w);
      addSubChan(makeSharedComp(new CIELabChannel(mgr)), "CIELabcolor", w);
    }
    if (double w = ww['Y' - 'A']) {
      LINFO("Using a Depth channel with weight %f", w);
      addSubChan(makeSharedComp(new DepthChannel(mgr)), "Depth", w);
    }
    if (double w = ww['y' - 'A']) {
      LINFO("Using a Depth Motion channel with weight %f", w);
      addSubChan(makeSharedComp(new DepthMotionChannel(mgr)), "DepthMotion", w);
    }
    if (double w = ww['R' - 'A']) {
      LINFO("Using a Pedestrian channel with weight %f", w);
      addSubChan(makeSharedComp(new PedestrianChannel(mgr)), "Pedestrian", w);
    }
    if (double w = ww['V' - 'A']) {
      LINFO("Using a SoxChannel with weight %f", w);
      addSubChan(makeSharedComp(new SoxChannel(mgr)), "Sox", w);
    }
    if (double w = ww['W' - 'A']) {
      LINFO("Using a ContourChannel with weight %f", w);
      addSubChan(makeContourChannel(mgr, "contour"), "Contour", w);
    }
    if (double w = ww['Z' - 'A']) {
      LINFO("Using a dummy Zero Channel with weight %f", w);
      addSubChan(makeSharedComp(new ZeroChannel(mgr)), "Zero", w);
    }
    if (double w = ww['A' - 'A']) {
      LINFO("Using an Object Detection channel with weight %f", w);
      addSubChan(makeSharedComp(new ObjDetChannel(mgr)), "ObjDet", w);
    }
    if (double w = ww['J' - 'A']) {
      LINFO("Using a DKL Color channel with weight %f", w);
      addSubChan(makeSharedComp(new DKLcolorChannel(mgr)), "DKLcolor", w);
    }
    if (double w = ww['U' - 'A']) {
      LINFO("Using Foreground Detection channel with weight %f", w);
      addSubChan(makeSharedComp(new ForegroundDetectionChannel(mgr)), "Foreground", w);
    }
    if (double w = ww['i' - 'A']) {
      LINFO("Using an Imagize Composite channel with weight %f", w);
      addSubChan(makeSharedComp(new ImagizeColorChannel(mgr)), "ImagizeComposite", w);
    }
    if (double w = ww['f' - 'A']) {
      LINFO("Using a FlickerNBackChannel with weight %f", w);
      addSubChan(makeSharedComp(new FlickerNBackChannel(mgr)), "flicker", w);
    }
    LINFO("RawVisualCortex loaded with %u top-level channels.", numChans());

    // make sure options are exported for all channels:
    for (uint i = 0; i < this->numChans(); ++i) this->subChan(i)->exportOptions(MC_RECURSE);
  }
}

// ######################################################################
const Image<float> RawVisualCortex::getVCOutput(const Image<PixRGB<byte> >&rgbin)
{

  static SimTime time = SimTime::ZERO();

  time += SimTime::MSECS(10);

  const InputFrame ifr = InputFrame::fromRgb(&rgbin, time);

  this->input(ifr);

  return getOutput();
}

// ######################################################################
Image<float> RawVisualCortex::getChannelOutputMap(const uint idx) const
{
  nub::ref<ChannelBase> ch = subChan(idx);

  // Determine the weight for each subchan. This is the product of
  // the intrinsic weight of each subchan (which is set either
  // through the --vc-chans config string of RawVisualCortex for
  // top-level channels, or subject to static modifications by
  // using --load at the start of the entire model and setting the
  // NModelParam value associated with each channel), multiplied
  // by any possible extra top-down gain if we have a
  // ChannelFacetGainComplex:
  float w = float(this->getSubchanTotalWeight(*ch));

  if (itsUseOlderVersion.getVal()) w /= ch->numSubmaps();

  if (hasFacet<ChannelFacetGainComplex>())
    w *= getFacet<ChannelFacetGainComplex>()->getVal(idx);

  if (w == 0.0)
    {
      LDEBUG("%12s weight is 0.0 (%2u submaps) -- skipped",
             ch->tagName().c_str(),ch->numSubmaps());
      return Image<float>();
    }

  // get the raw channel output and compute its range (for debug msg):
  Image<float> chanOut = ch->getOutput();

  float mi1 = -1.0f, ma1 = -1.0f;
  if (MYLOGVERB >= LOG_DEBUG) getMinMax(chanOut, mi1, ma1);

  if (w != 1.0f) chanOut *= w;

  if (MYLOGVERB >= LOG_DEBUG)
    {
      float mi, ma; getMinMax(chanOut, mi, ma);
      LDEBUG("%12s weight is %.4f (%2u submaps); raw range is "
             "%.2g .. %.2g; weighted range is %.2g .. %.2g",
             ch->tagName().c_str(), w, ch->numSubmaps(), mi1,ma1,mi,ma);
    }

  return chanOut;
}

// ######################################################################
Image<float> RawVisualCortex::getRawChannelOutputMap(const uint idx) const
{
  nub::ref<ChannelBase> ch = subChan(idx);
  return ch->getOutput();
}

// ######################################################################
Image<float> RawVisualCortex::postProcessOutputMap(const Image<float>& outmap)
{
  // CAUTION: if you make major changes to the default behavior here,
  // make sure you check that this is compatible with the various
  // derivatives of RawVisualCortex (e.g., RawVisualCortexBeo, etc).
  Image<float> result = outmap;

  // let's apply a last maxNormalization to the map, but only if we
  // have two or more channels with non-zero weights, otherwise, we
  // just pass through:
  uint num_nzw = 0;
  for (uint i = 0; i < numChans(); ++i) if (getSubchanTotalWeight(i) > itsWeightThresh.getVal()) ++num_nzw;

  if (num_nzw > 1)
    {
      switch(itsNormType.getVal())
        {
        case VCXNORM_LANDMARK:
          result = maxNormalize(result, 0.0f, 2.0f, VCXNORM_NONE);
          break;
        case VCXNORM_SURPRISE:
          LFATAL("Surprise maxnorm must use RawVisualCortexSurprise");
          break;
        default:
          result = maxNormalize(result, 0.0f, 2.0f, itsNormType.getVal());
          LDEBUG("%s(%f .. %f)", maxNormTypeName(itsNormType.getVal()),
                 0.0f, 2.0f);
          break;
        }
    }

  return result;
}

// ######################################################################
void RawVisualCortex::doInput(const InputFrame& inframe)
{
  // optimization: if we have no channels, then do a quick return:
  if (this->numChans() == 0) return;

  // we will not process the input at all if itsLoadOutFrom is set,
  // since we will instead read our final output map from disk in
  // getOutput():
  if (itsLoadOutFrom.getVal().empty() == false)
    {
      LINFO("Not feeding channels since we will load output from MGZ file.");
      return;
    }

  // build our pyramid cache?
  rutz::mutex_lock_class lock;
  if (inframe.pyrCache().get() != 0
      && inframe.pyrCache()->gaussian5.beginSet(inframe.grayFloat(), &lock))
    {
      if (itsUseSpaceVariantBoundary.getVal())
        inframe.pyrCache()->gaussian5.endSet
          (inframe.grayFloat(),
           buildRadialPyrGaussian
           (inframe.grayFloat(), 0, itsLevelSpec.getVal().maxDepth()),
           &lock);
      else
        inframe.pyrCache()->gaussian5.endSet
          (inframe.grayFloat(),
           buildPyrGaussian
           (inframe.grayFloat(), 0, itsLevelSpec.getVal().maxDepth(), 5),
           &lock);
    }

  // process the channels:
  for (uint i = 0; i < this->numChans(); ++i)
    {
      // get a pointer to the channel of interest:
      nub::ref<ChannelBase> chan = this->subChan(i);

      // feed the input to the channel of interest:
      chan->input(inframe);
      LINFO("Input to %s channel %s ok.",
            featureHierarchyName(featureHierarchyLevel(chan->visualFeature())),
            chan->tagName().c_str());
    }
}

// ######################################################################
Image<float> RawVisualCortex::combineOutputs()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<float> output;
 
  if (!itsSaveRawCSOutTo.getVal().empty())
  {
    LINFO("Saving raw center-surround maps for %d maps to '%s'", 
          numSubmaps(), itsSaveRawCSOutTo.getVal().c_str());

    const Dims dims = itsRawCSDims.getVal();
    
    for (uint ii = 0; ii < numSubmaps(); ++ii)
    {
      uint subchan = 0, subidx = 0;
      lookupSubmap(ii, subchan, subidx);
      
      Image<float> map;
      if (subChan(subchan)->outputAvailable())
      {
        map = subChan(subchan)->getRawCSmap(subidx);
        map = rescaleNI(map, dims);
      }
      else
      {
        map = Image<float>(dims, ZEROS);
      }

      itsRawCSMgzOut->writeFrame(GenericFrame(map, FLOAT_NORM_PRESERVE));
    }
 
    return output;
  }
  
  // are we reading our precomputed outputs from an MGZ file?
  if (itsLoadOutFrom.getVal().empty() == false)
  {
      LINFO("Bypassing RawVisualCortex output computations.");
      LINFO("Loading RawVisualCortex output from '%s'",
            itsLoadOutFrom.getVal().c_str());
      output = (itsOutputMgzIn->readFrame()).asFloat();
    }
  else
    {
      // ... OK, we have to recompute the output:
      for (uint i = 0; i < this->numChans(); ++i)
        {
          if (subChan(i)->outputAvailable() == false) continue;

          Image<float> chanOut = getChannelOutputMap(i);

          if (chanOut.initialized() == false) continue;

          // downSizeClean() gracefully does nothing if chanOut is
          // already the right size. (Eventually we might like to have a
          // way for RawVisualCortex to enforce a particular chanOut size
          // on its own, rather than just taking the size from the first
          // channel in the array.)
          if (output.initialized() == false)
            output = chanOut;
          else
            {
              // sum or take max across channels?
              Image<float> o = downSizeClean(chanOut, output.getDims());
              if (itsUseMax.getVal()) output = takeMax(output, o); else output += o;
            }
        }
    }
  
  // Ensure that we still return a valid image even if we have no channels
  // that has a valid output in the RawVisualCortex:
  if (output.initialized() == false)
    {
      int sml = itsLevelSpec.getVal().mapLevel();
      output = Image<float>(this->getInputDims().w() >> sml,
                            this->getInputDims().h() >> sml, ZEROS);
    }

  if (MYLOGVERB >= LOG_DEBUG)
    {
      float mi, ma; getMinMax(output, mi, ma);
      LDEBUG("Raw output range is [%f .. %f]", mi, ma);
    }

  // before we finalize the output, do we want to save it to an MGZ file?
  if (itsSaveOutTo.getVal().empty() == false)
    {
      LINFO("Saving raw unnormalized RawVisualCortex output to '%s'",
            itsSaveOutTo.getVal().c_str());
      itsOutputMgzOut->writeFrame(GenericFrame(output, FLOAT_NORM_PRESERVE));
    }

  // post-process the output in a manner than may depend on which
  // variant of RawVisualCortex we may embody:
  output = postProcessOutputMap(output);

  // output is now typically in the (0.0..8.0) range;
  // typical images are in (0..4) range; we want input current in nA
  output *= itsOutputFactor.getVal();
  float mi, ma; getMinMax(output, mi, ma);
  LINFO("Salmap input range is [%f .. %f] nA", mi * 1.0e9F, ma * 1.0e9F);

  // add a tiny background activity to ensure that the entire image
  // will be visited:
  if (itsUseRandom.getVal())
    {
      LINFO("Adding random background noise to output.");
      inplaceAddBGnoise(output, itsNoise.getVal());
      output += itsNoise.getVal() * 0.01F;
    }



  LINFO("Computed RawVisualCortex output.");
  
  // uint width  = output.getWidth();
  // uint height = output.getHeight();
  // LINFO("w: %d h: %d", width, height);
  
  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(width,height), 0, 0, "rVC"));
  // itsWin->setDims(Dims(width*16, height*16));
  // itsWin->drawImage(zoomXY(output,16),0,0);
  // Raster::waitForKey();

 // std::string folder("/lab/tmpib/u/siagian/neuroscience/Data/FOE/"
 //                     "DARPA_nv2_2011/Germany_K_CIOFM/");
 // Image<float> img = zoomXY(output,16);
 // inplaceNormalize(img, 0.0F, 255.0F);
 // Image<byte>  res(img);
 // Raster::WriteRGB(res, sformat("%sSalMap_CIOFM_%06d.ppm", 
 //                               folder.c_str(), itsFrame));

   // itsFrame++;

  return output;
}


// ######################################################################
void RawVisualCortex::saveResults(const nub::ref<FrameOstream>& ofs)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // save our own output:
  if (itsSaveOutput.getVal())
    ofs->writeFloat(getOutput(), FLOAT_NORM_PRESERVE, "VCO",
                    FrameInfo("visual cortex output "
                              "(input to saliency map)", SRC_POS));

  // save our channel outputs:
  for (uint i = 0; i < numChans(); ++i) subChan(i)->saveResults(ofs);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
