/*!@file Neuro/NeuroSimEvents.C SimEvent derivatives for neuro modules */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/NeuroSimEvents.C $
// $Id: NeuroSimEvents.C 14762 2011-05-03 01:13:16Z siagian $
//

#include "Neuro/NeuroSimEvents.H"
#include "Component/ParamMap.H"
#include "Channels/ChannelMaps.H"
#include "Channels/IntegerRawVisualCortex.H"
#include "Channels/RawVisualCortex.H"
#include "Util/sformat.H"
#include "Image/MathOps.H"
#include "Neuro/VisualBuffer.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/VisualCortexEyeMvt.H"
#include "Psycho/EyeData.H"
#include "Psycho/HandData.H"
#include "Transport/FrameOstream.H"
#include "SpaceVariant/SpaceVariantTransforms.H"

// ######################################################################
SimEventWTAwinner::SimEventWTAwinner(SimModule* src, const WTAwinner w,
                                     const uint s) :
  SimEvent(src), itsWinner(w), itsShiftNum(s)
{ }

SimEventWTAwinner::~SimEventWTAwinner()
{ }

std::string SimEventWTAwinner::toString() const
{
  return SimEvent::toString() +
    sformat(", winner[%u]=(%d, %d) %.3fmV%s", itsShiftNum,
            itsWinner.p.i, itsWinner.p.j,
            itsWinner.sv * 1000.0, itsWinner.boring ? " [boring]" : "");
}

const WTAwinner& SimEventWTAwinner::winner() const
{ return itsWinner; }

uint SimEventWTAwinner::shiftNum() const
{ return itsShiftNum; }

// ######################################################################
SimEventTargetsHit::SimEventTargetsHit(SimModule* src, const int numhit) :
  SimEvent(src), itsNumHits(numhit)
{ }

SimEventTargetsHit::~SimEventTargetsHit()
{ }

std::string SimEventTargetsHit::toString() const
{ return SimEvent::toString() + sformat(", %d hits", itsNumHits); }

int SimEventTargetsHit::numHits() const
{ return itsNumHits; }

// ######################################################################
SimEventRetinaImage::SimEventRetinaImage(SimModule* src,
                                         const InputFrame& ifr,
                                         const Rectangle& rawinprect,
                                         const Point2D<int> offset) :
  SimEvent(src), itsFrame(ifr), itsRawInputRectangle(rawinprect),
  itsOffset(offset), itsRetTransform(/*NULL*/), itsMapTransform(/*NULL*/)
{ }

SimEventRetinaImage::SimEventRetinaImage(SimModule* src,
                                         const InputFrame& ifr,
                                         const Rectangle& rawinprect,
                                         const Point2D<int> offset,
                                         rutz::shared_ptr<SpaceVariantTransform> rettransform,
                                         rutz::shared_ptr<SpaceVariantTransform> maptransform) :
  SimEvent(src), itsFrame(ifr), itsRawInputRectangle(rawinprect),itsOffset(offset),
  itsRetTransform(rettransform),  itsMapTransform(maptransform)
{ }

SimEventRetinaImage::~SimEventRetinaImage()
{ }

std::string SimEventRetinaImage::toString() const
{
  return SimEvent::toString() +
    sformat(", [%dx%d] t=%.1fms", itsFrame.colorByte().getWidth(),
            itsFrame.colorByte().getHeight(), itsFrame.time().msecs());
}

const InputFrame& SimEventRetinaImage::frame() const
{ return itsFrame; }

const Rectangle& SimEventRetinaImage::rawInputRectangle() const
{ return itsRawInputRectangle; }

Point2D<int>
SimEventRetinaImage::rawToRetinal(const Point2D<int>& rawpos) const
{ return rawpos + itsOffset; }

const Dims& SimEventRetinaImage::getRawInputDims() const
{ if (itsRetTransform.is_valid())
    return itsRetTransform->getCTDims();
  else
    return itsFrame.getDims(); 
}

Point2D<int>
SimEventRetinaImage::retinalToRaw(const Point2D<int>& retpos) const
{ return retpos - itsOffset; }

Point2D<int> SimEventRetinaImage::center() const
{ return Point2D<int>(itsFrame.getWidth() / 2, itsFrame.getHeight() / 2); }

const Point2D<int>& SimEventRetinaImage::offset() const
{ return itsOffset; }

rutz::shared_ptr<SpaceVariantTransform> SimEventRetinaImage::getRetTransform() const
{ return itsRetTransform; };

rutz::shared_ptr<SpaceVariantTransform> SimEventRetinaImage::getMapTransform() const
{ return itsMapTransform; };

// ######################################################################
SimEventVisualCortexOutput::
SimEventVisualCortexOutput(SimModule* src, const Image<float>& vcout) :
  SimEvent(src), itsMap(vcout)
{ }

SimEventVisualCortexOutput::~SimEventVisualCortexOutput()
{ }

std::string SimEventVisualCortexOutput::toString() const
{
  return SimEvent::toString() +
    sformat(", [%dx%d]", itsMap.getWidth(), itsMap.getHeight());
}

const Image<float> SimEventVisualCortexOutput::vco(const float factor) const
{
  Image<float> ret = itsMap;
  if (factor == 0.0F) inplaceNormalize(ret, 0.0F, 255.0F);
  else if (factor != 1.0F) ret *= factor;
  return ret;
}

// ######################################################################
SimEventGistOutput::
SimEventGistOutput(SimModule* src, const Image<float>& gout) :
  SimEvent(src), itsGistVector(gout)
{ }

SimEventGistOutput::~SimEventGistOutput()
{ }

std::string SimEventGistOutput::toString() const
{
  return SimEvent::toString() +
    sformat(", [%dx%d]", itsGistVector.getWidth(), itsGistVector.getHeight());
}

const Image<float> SimEventGistOutput::gv() const
{ return itsGistVector; }

// ######################################################################
SimEventSaliencyMapOutput::
SimEventSaliencyMapOutput(SimModule* src, const Image<float>& smout, const int maplevel) :
  SimEvent(src), itsMap(smout), itsMapLevel(maplevel)
{ }

SimEventSaliencyMapOutput::~SimEventSaliencyMapOutput()
{ }

std::string SimEventSaliencyMapOutput::toString() const
{
  return SimEvent::toString() +
    sformat(", [%dx%d]", itsMap.getWidth(), itsMap.getHeight());
}

const Image<float> SimEventSaliencyMapOutput::sm(const float factor) const
{
  Image<float> ret = itsMap;
  if (factor == 0.0F) inplaceNormalize(ret, 0.0F, 255.0F);
  else if (factor != 1.0F) ret *= factor;
  return ret;
}

Point2D<int> SimEventSaliencyMapOutput::smToOrig(const Point2D<int>& p) const
{
  return Point2D<int>((p.i << itsMapLevel), (p.j << itsMapLevel));
}

Point2D<int> SimEventSaliencyMapOutput::origToSm(const Point2D<int>& p) const
{
  return Point2D<int>((p.i >> itsMapLevel), (p.j >> itsMapLevel));
}

// // ######################################################################
// SimEventMTfeatureMapOutput::
// SimEventMTfeatureMapOutput(SimModule* src, std::vector< Image<float> > mtFeat) :
//   SimEvent(src), itsMTfeatures(mtFeat)
// { }

// SimEventMTfeatureMapOutput::~SimEventMTfeatureMapOutput()
// { }

// std::string SimEventMTfeatureMapOutput::toString() const
// {
//   return SimEvent::toString() +
//     sformat(", number of directions: %d", itsMTfeatures.size());
// }

// const std::vector<Image<float> > 
// SimEventMTfeatureMapOutput::mtFeatures() const
// {
//   return itsMTfeatures;
// }

// ######################################################################
SimEventTaskRelevanceMapOutput::
SimEventTaskRelevanceMapOutput(SimModule* src, const Image<float>& trmout) :
  SimEvent(src), itsMap(trmout)
{ }

SimEventTaskRelevanceMapOutput::~SimEventTaskRelevanceMapOutput()
{ }

std::string SimEventTaskRelevanceMapOutput::toString() const
{
  return SimEvent::toString() +
    sformat(", [%dx%d]", itsMap.getWidth(), itsMap.getHeight());
}

const Image<float> SimEventTaskRelevanceMapOutput::
trm(const float factor) const
{
  Image<float> ret = itsMap;
  if (factor == 0.0F) inplaceNormalize(ret, 0.0F, 255.0F);
  else if (factor != 1.0F) ret *= factor;
  return ret;
}

// ######################################################################
SimEventAttentionGuidanceMapOutput::
SimEventAttentionGuidanceMapOutput(SimModule* src, const Image<float>& agmout) :
  SimEvent(src), itsMap(1)
{ itsMap[0] = agmout; }

SimEventAttentionGuidanceMapOutput::
SimEventAttentionGuidanceMapOutput(SimModule* src, const ImageSet<float>& agmout) :
  SimEvent(src), itsMap(agmout)
{ }

SimEventAttentionGuidanceMapOutput::~SimEventAttentionGuidanceMapOutput()
{ }

std::string SimEventAttentionGuidanceMapOutput::toString() const
{
  return SimEvent::toString() +
    sformat(", [%dx%dx%d]", itsMap[0].getWidth(), itsMap[0].getHeight(), itsMap.size());
}

uint SimEventAttentionGuidanceMapOutput::numMaps() const
{ return itsMap.size(); }

const Image<float> SimEventAttentionGuidanceMapOutput::agm(const float factor, const uint pos) const
{
  Image<float> ret;
  if (pos < itsMap.size()){
    ret = itsMap[pos];
    if (factor == 0.0F) inplaceNormalize(ret, 0.0F, 255.0F);
    else if (factor != 1.0F) ret *= factor;
  }
  return ret;
}

const ImageSet<float> SimEventAttentionGuidanceMapOutput::allAgm(const float factor) const
{
  if (factor == 1.0)
    return itsMap;
  else
    {
      ImageSet<float> ret = itsMap;
      for (uint ii = 0; ii < ret.size(); ++ii)
        {
          if (factor == 0.0F) inplaceNormalize(ret[ii], 0.0F, 255.0F);
          else ret[ii] *= factor;
        }
      return ret;
    }
}

// ######################################################################
SimEventAttentionGateOutput::
SimEventAttentionGateOutput(SimModule* src,
                            const Image<float>& agout,
                            const Image<float>& lamout,
                            const Image<float>& camout,
                            const unsigned int lamframe) :
  SimEvent(src), itsMap(agout), itsLastAttMap(lamout), itsCurrAttMap(camout),
  itsLastFrame(lamframe)
{ }

SimEventAttentionGateOutput::~SimEventAttentionGateOutput()
{ }

std::string SimEventAttentionGateOutput::toString() const
{
  return SimEvent::toString() +
    sformat(", [%dx%d]", itsMap.getWidth(), itsMap.getHeight());
}

const Image<float> SimEventAttentionGateOutput::
ag(const float factor) const
{
  Image<float> ret = itsMap;
  if (factor == 0.0F) inplaceNormalize(ret, 0.0F, 255.0F);
  else if (factor != 1.0F) ret *= factor;
  return ret;
}

const Image<float> SimEventAttentionGateOutput::
lam(const float factor) const
{
  Image<float> ret = itsLastAttMap;
  if (factor == 0.0F) inplaceNormalize(ret, 0.0F, 255.0F);
  else if (factor != 1.0F) ret *= factor;
  return ret;
}

const Image<float> SimEventAttentionGateOutput::
cam(const float factor) const
{
  Image<float> ret = itsCurrAttMap;
  if (factor == 0.0F) inplaceNormalize(ret, 0.0F, 255.0F);
  else if (factor != 1.0F) ret *= factor;
  return ret;
}

const unsigned int SimEventAttentionGateOutput::lamFrame() const
{ return itsLastFrame; }

// ######################################################################
SimEventAttentionGateStageTwoSegments::
SimEventAttentionGateStageTwoSegments(SimModule* src,
                              const Image<bool>& candidates,
                              const SimEventAttentionGateStageTwoObjects& obj,
                              const int          segnum) :
  SimEvent(src), itsCandidates(candidates), itsObjects(obj),
  itsSegmentNum(segnum)
{ }

SimEventAttentionGateStageTwoSegments::~SimEventAttentionGateStageTwoSegments()
{}

const Image<bool> SimEventAttentionGateStageTwoSegments::candidates() const
{ return itsCandidates;    }

const SimEventAttentionGateStageTwoObjects
SimEventAttentionGateStageTwoSegments::obj() const
{ return itsObjects;      }

const int SimEventAttentionGateStageTwoSegments::segnum() const
{ return itsSegmentNum;    }

// ######################################################################
SimEventShapeEstimatorOutput::
SimEventShapeEstimatorOutput(SimModule* src,
                             const Image<float>& winmap,
                             const Image<byte>& objmask,
                             const Image<byte>& iormask,
                             const Image<float>& smoothmask,
                             const Image<float>& cumsmoothmask,
                             const std::string& winlabel,
                             const bool isshaped) :
  SimEvent(src), itsWinningMap(winmap), itsObjMask(objmask),
  itsIORmask(iormask), itsSmoothMask(smoothmask),
  itsCumSmoothMask(cumsmoothmask), itsWinLabel(winlabel),
  itsIsShaped(isshaped)
{ }

SimEventShapeEstimatorOutput::~SimEventShapeEstimatorOutput()
{ }

std::string SimEventShapeEstimatorOutput::toString() const
{
  return SimEvent::toString() +
    sformat("Winning map: %s [%dx%d], Smooth mask: [%dx%d]",
            itsWinLabel.c_str(),
            itsObjMask.getWidth(), itsObjMask.getHeight(),
            itsSmoothMask.getWidth(), itsSmoothMask.getHeight());
}

const Image<float>& SimEventShapeEstimatorOutput::winningMap() const
{ return itsWinningMap; }

const Image<byte>& SimEventShapeEstimatorOutput::objectMask() const
{ return itsObjMask; }

const Image<byte>& SimEventShapeEstimatorOutput::iorMask() const
{ return itsIORmask; }

const Image<float>& SimEventShapeEstimatorOutput::smoothMask() const
{ return itsSmoothMask; }

Image<float> SimEventShapeEstimatorOutput::cumSmoothMask() const
{
  Image<float> result(itsCumSmoothMask);
  inplaceClamp(result, 0.4F /*CLAMP*/, 1.0F);
  return result;
}

Image<float> SimEventShapeEstimatorOutput::negCumSmoothMask() const
{
  Image<float> result(itsCumSmoothMask * (-1.0F) + 1.0F);
  inplaceClamp(result, 1.0F-0.4F /*CLAMP*/, 1.0F);
  return result;
}

const std::string& SimEventShapeEstimatorOutput::winningLabel() const
{ return itsWinLabel; }

uint SimEventShapeEstimatorOutput::objectArea() const
{
  const double scale = itsSmoothMask.getSize() / itsObjMask.getSize();
  return uint(sum(itsObjMask) / 255.0 * scale);
}

bool SimEventShapeEstimatorOutput::isShaped() const
{ return itsIsShaped; }

// ######################################################################
SimEventSaccadeStatus::
SimEventSaccadeStatus(SimModule* src, const Point2D<int>& pos,
                      const SaccadeState state,
                      const SaccadeState prevState,
                      const bool blinkState,
                      const bool prevBlinkState) :
  SimEvent(src), itsPosition(pos),
  itsState(state), itsPrevState(prevState),
  itsBlinkState(blinkState), itsPrevBlinkState(prevBlinkState)
{ }

SimEventSaccadeStatus::~SimEventSaccadeStatus()
{ }

std::string SimEventSaccadeStatus::toString() const
{
  return SimEvent::toString() +
    sformat(" %s (%d,%d) Unk:%s Fix:%s Sac:%s Bli:%s Smo:%s",
            saccadeBodyPartName(bodyPart()),
            itsPosition.i, itsPosition.j,
            transientStatusToStr(unknownStatus()),
            transientStatusToStr(fixationStatus()),
            transientStatusToStr(saccadeStatus()),
            transientStatusToStr(blinkStatus()),
            transientStatusToStr(smoothPursuitStatus()));
}

const Point2D<int>& SimEventSaccadeStatus::position() const
{ return itsPosition; }

TransientStatus SimEventSaccadeStatus::unknownStatus() const
{ return transientStatus(itsPrevState == SACSTATE_UNK,
                         itsState == SACSTATE_UNK); }

TransientStatus SimEventSaccadeStatus::fixationStatus() const
{ return transientStatus(itsPrevState == SACSTATE_FIX,
                         itsState == SACSTATE_FIX); }

TransientStatus SimEventSaccadeStatus::saccadeStatus() const
{ return transientStatus(itsPrevState == SACSTATE_SAC,
                         itsState == SACSTATE_SAC); }

TransientStatus SimEventSaccadeStatus::blinkStatus() const
{ return transientStatus(itsPrevBlinkState, itsBlinkState); }

TransientStatus SimEventSaccadeStatus::smoothPursuitStatus() const
{ return transientStatus(itsPrevState == SACSTATE_SMO,
                         itsState == SACSTATE_SMO); }


// ######################################################################
SimEventSaccadeStatusEye::
SimEventSaccadeStatusEye(SimModule* src, const Point2D<int>& pos,
                         const SaccadeState state,
                         const SaccadeState prevState,
                         const bool blinkState,
                         const bool prevBlinkState) :
  SimEventSaccadeStatus(src, pos, state, prevState, blinkState, prevBlinkState)
{ }

SimEventSaccadeStatusEye::~SimEventSaccadeStatusEye()
{ }

SaccadeBodyPart SimEventSaccadeStatusEye::bodyPart() const
{ return SaccadeBodyPartEye; }

// ######################################################################
SimEventSaccadeStatusHead::
SimEventSaccadeStatusHead(SimModule* src, const Point2D<int>& pos,
                          const SaccadeState state,
                          const SaccadeState prevState,
                          const bool blinkState,
                          const bool prevBlinkState) :
  SimEventSaccadeStatus(src, pos, state, prevState, blinkState, prevBlinkState)
{ }

SimEventSaccadeStatusHead::~SimEventSaccadeStatusHead()
{ }

SaccadeBodyPart SimEventSaccadeStatusHead::bodyPart() const
{ return SaccadeBodyPartHead; }

// ######################################################################
SimEventEyeTrackerData::
SimEventEyeTrackerData(SimModule* src, rutz::shared_ptr<EyeData> d,
                       const uint trackernum,
                       const std::string& trackerfname,
                       const PixRGB<byte>& trackercolor, 
                       const PixPerDeg& ppd, const SimTime samplingrate) :
  SimEvent(src), itsData(d), itsTrackerNum(trackernum),
  itsTrackerFname(trackerfname), itsTrackerColor(trackercolor), itsPpd(ppd), 
  itsHz(samplingrate)
{ }

SimEventEyeTrackerData::~SimEventEyeTrackerData()
{ }

rutz::shared_ptr<EyeData> SimEventEyeTrackerData::data() const
{ return itsData; }

uint SimEventEyeTrackerData::trackerNum() const
{ return itsTrackerNum; }

std::string SimEventEyeTrackerData::trackerFilename() const
{ return itsTrackerFname; }

PixRGB<byte> SimEventEyeTrackerData::trackerColor() const
{ return itsTrackerColor; }

PixPerDeg SimEventEyeTrackerData::trackerPpd() const
{ return itsPpd; }

SimTime SimEventEyeTrackerData::trackerHz() const
{ return itsHz; }

// ######################################################################
SimEventHandTrackerData::
SimEventHandTrackerData(SimModule* src, rutz::shared_ptr<HandData> d,
                        const uint trackernum,
                        const std::string& trackerfname,
                        const PixRGB<byte>& trackercolor, 
                        const SimTime samplingrate) :
  SimEvent(src), itsData(d), itsTrackerNum(trackernum),
  itsTrackerFname(trackerfname), itsTrackerColor(trackercolor),
  itsHz(samplingrate)
{ }

SimEventHandTrackerData::~SimEventHandTrackerData()
{ }

rutz::shared_ptr<HandData> SimEventHandTrackerData::data() const
{ return itsData; }

uint SimEventHandTrackerData::trackerNum() const
{ return itsTrackerNum; }

std::string SimEventHandTrackerData::trackerFilename() const
{ return itsTrackerFname; }

PixRGB<byte> SimEventHandTrackerData::trackerColor() const
{ return itsTrackerColor; }

SimTime SimEventHandTrackerData::trackerHz() const
{ return itsHz; }

// ######################################################################
SimReqVCXchanVis::
SimReqVCXchanVis(SimModule* src, rutz::shared_ptr<ChannelVisitor> vis) :
  SimReq(src), itsVisitor(vis)
{ }

SimReqVCXchanVis::~SimReqVCXchanVis()
{ }

void SimReqVCXchanVis::preProcessing(RawVisualCortex *vcx)
{ }

rutz::shared_ptr<ChannelVisitor> SimReqVCXchanVis::visitor() const
{ return itsVisitor; }

void SimReqVCXchanVis::postProcessing(RawVisualCortex *vcx)
{ }

// ######################################################################
SimEventObjectToBias::
SimEventObjectToBias(SimModule* src, const std::string& objName) :
  SimEvent(src), itsObjName(objName)
{ }

SimEventObjectToBias::~SimEventObjectToBias()
{ }

const std::string& SimEventObjectToBias::name() const
{ return itsObjName; }

// ######################################################################
SimEventTargetMask::
SimEventTargetMask(SimModule* src, const Image<byte>& tmask) :
  SimEvent(src), itsMask(tmask)
{ }

SimEventTargetMask::~SimEventTargetMask()
{ }

std::string SimEventTargetMask::toString() const
{
  return SimEvent::toString() + sformat(", [%dx%d]", itsMask.getWidth(), itsMask.getHeight());
}

const Image<byte> SimEventTargetMask::mask() const
{ return itsMask; }

// ######################################################################
SimReqVCXfeatures::
SimReqVCXfeatures(SimModule* src, const Point2D<int>& loc) :
  SimReq(src), itsLoc(loc), itsFeatures()
{ }

SimReqVCXfeatures::~SimReqVCXfeatures()
{ }

std::string SimReqVCXfeatures::toString() const
{
  return SimReq::toString() + sformat(", [%d,%d]", itsLoc.i, itsLoc.j);
}

const Point2D<int>& SimReqVCXfeatures::loc() const
{ return itsLoc; }

std::vector<float>& SimReqVCXfeatures::features()
{ return itsFeatures; }

// ######################################################################
SimReqVCXmaps::SimReqVCXmaps(SimModule* src) :
  SimReq(src), itsChannelMaps(0)
{ }

SimReqVCXmaps::~SimReqVCXmaps()
{ }

rutz::shared_ptr<ChannelMaps> SimReqVCXmaps::channelmaps() const
{ return itsChannelMaps; }

void SimReqVCXmaps::populateChannelMaps(RawVisualCortex *vcx)
{
  // are we being handled by several respondents?
  if (itsChannelMaps.is_valid()) LFATAL("I can only be handled by one respondent");

  itsChannelMaps = rutz::make_shared(new ChannelMaps(vcx));
}

void SimReqVCXmaps::populateChannelMaps(IntegerRawVisualCortex *vcx)
{
  // are we being handled by several respondents?
  if (itsChannelMaps.is_valid()) LFATAL("I can only be handled by one respondent");

  itsChannelMaps = rutz::make_shared(new ChannelMaps(vcx));
}

void SimReqVCXmaps::populateChannelMaps(EnvVisualCortexFloat *vcx)
{
  // are we being handled by several respondents?
  if (itsChannelMaps.is_valid()) LFATAL("I can only be handled by one respondent");

  itsChannelMaps = rutz::make_shared(new ChannelMaps(vcx));
}

void SimReqVCXmaps::populateChannelMaps(VisualCortexEyeMvt *vcx)
{
  // are we being handled by several respondents?
  if (itsChannelMaps.is_valid()) LFATAL("I can only be handled by one respondent");

  itsChannelMaps = rutz::make_shared(new ChannelMaps(NamedImage<float>(vcx->getOutput(), "SaliencyMap")));
}

// ######################################################################
SimEventVisualBufferOutput::
SimEventVisualBufferOutput(SimModule* src, const Image<float>& buf, const int smlev,
                           const Dims& smdims, const Point2D<int>& retoff) :
  SimEvent(src), itsVisualBuffer(buf), itsSMlev(smlev), itsSMdims(smdims), itsRetinaOffset(retoff)
{ }

SimEventVisualBufferOutput::~SimEventVisualBufferOutput()
{ }

std::string SimEventVisualBufferOutput::toString() const
{
  return SimEvent::toString() + sformat(", [%dx%d]", itsVisualBuffer.getWidth(), itsVisualBuffer.getHeight());
}

const Image<float>& SimEventVisualBufferOutput::buffer() const
{ return itsVisualBuffer; }

const Dims& SimEventVisualBufferOutput::smdims() const
{ return itsSMdims; }

int SimEventVisualBufferOutput::smlev() const
{ return itsSMlev; }

Point2D<int> SimEventVisualBufferOutput::retinalToBuffer(const Point2D<int>& p) const
{
  return retinalToVisualBuffer(p, itsRetinaOffset, itsSMlev, itsSMdims, itsVisualBuffer.getDims());
}

Point2D<int> SimEventVisualBufferOutput::bufferToRetinal(const Point2D<int>& p) const
{
  return visualBufferToRetinal(p, itsRetinaOffset, itsSMlev, itsSMdims, itsVisualBuffer.getDims());

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
