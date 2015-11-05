/*!@file Channels/SingleChannel.C Channel for a single stream of processing. */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SingleChannel.C $
// $Id: SingleChannel.C 14632 2011-03-23 20:08:44Z dberg $
//

#include "Channels/SingleChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ChannelFacets.H"
#include "Channels/ChannelVisitor.H"
#include "Channels/SubmapAlgorithmStd.H"
#include "Component/GlobalOpts.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Image/FFTWWrapper.H"
#include "Image/FilterOps.H"  // for single-opponent's centerSurround()
#include "Image/Image.H"
#include "Image/ImageSetOps.H"
#include "Image/MathOps.H"    // for binaryReverse()
#include "Image/PyrBuilder.H"
#include "Image/PyramidOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H" // for learningCoeff()
#include "Image/fancynorm.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

#include "Image/CutPaste.H"
#include <cmath>
#include <iostream>
#include <fstream>

// ######################################################################
SingleChannel::SingleChannel(OptionManager& mgr, const std::string& descrName,
                             const std::string& tag,
                             const VisualFeature vs,
                             rutz::shared_ptr<PyrBuilder<float> > pbuild) :
  ChannelBase(mgr, descrName, tag, vs),
  itsTakeAbs("SingleChannelTakeAbs", this, false),
  itsNormalizeOutput("SingleChannelNormalizeOutput", this, false),
  itsScaleNoiseToMax("SingleChannelScaleNoiseToMax", this, false),
  itsLowThresh("SingleChannelLowThresh", this, 0.0F),
  itsRectifyPyramid("SingleChannelRectifyPyramid", this, false),
  itsComputeFullPyramid("SingleChannelComputeFullPyramid", this, false),
  itsUseRandom(&OPT_UseRandom, this),  // see Component/ModelManager.{H,C}
  itsUseSplitCS(&OPT_SingleChannelUseSplitCS, this), // ModelOptionDefs.C
  itsLevelSpec(&OPT_LevelSpec, this),    // see Channels/ChannelOpts.{H,C}
  itsNormType(&OPT_MaxNormType, this), // see Channels/ChannelOpts.{H,C}
  itsQlen(&OPT_SingleChannelQueueLen, this), // see Channels/ChannelOpts.{H,C}
  itsUseOlderVersion(&OPT_UseOlderVersion, this), // Channels/ChannelOpts.{H,C}
  itsTimeDecay(&OPT_SingleChannelTimeDecay, this), //Channels/ChannelOpts.{H,C}
  itsSaveRawMaps(&OPT_SingleChannelSaveRawMaps, this),
  itsComputeFullPyramidForGist(&OPT_SingleChannelComputeFullPyramidForGist, this),
  itsSaveFeatureMaps(&OPT_SingleChannelSaveFeatureMaps, this),
  itsSaveOutputMap(&OPT_SingleChannelSaveOutputMap, this),
  itsSubmapAlgoType(&OPT_SubmapAlgoType, this),
  itsGetSingleChannelStats(&OPT_GetSingleChannelStats, this),
  itsSaveStatsPerChannel(&OPT_SaveStatsPerChannel, this),
  itsSaveStatsPerChannelFreq(&OPT_SaveStatsPerChannelFreq, this),
  itsGetSingleChannelStatsFile(&OPT_GetSingleChannelStatsFile, this),
  itsGetSingleChannelStatsTag(&OPT_GetSingleChannelStatsTag, this),
  itsOutputRangeMin(&OPT_ChannelOutputRangeMin, this),
  itsOutputRangeMax(&OPT_ChannelOutputRangeMax, this),
  itsPq(),
  itsOutputCache(),
  itsSubmapCache(NULL),
  itsTempl(),
  itsPyrBuilder(pbuild),
  itsClipPyr(),
  itsInputHandler(),
  itsSubmapAlgo(new SubmapAlgorithmStd(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ComponentFactory<SubmapAlgorithm>& f = SubmapAlgorithm::getFactory();
  if (!f.is_valid_key("Std"))
    f.registerType<SubmapAlgorithmStd>("Std", mgr);

  itsSubmapAlgo = SubmapAlgorithm::make(itsSubmapAlgoType.getVal());

  this->addSubComponent(itsSubmapAlgo);
}

// ######################################################################
SingleChannel::~SingleChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SingleChannel::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int maxind = maxIndex();
  itsSubmapCache = new Image<float>[maxind];
  itsTempl = ImageSet<float>(maxind);
  itsFrameIdx = 0;

  // in the new version, leave our output range open rather than
  // forcing it to a given range of values:
  if (itsUseOlderVersion.getVal() == false)
    {
      itsOutputRangeMin.setVal(0.0f);
      itsOutputRangeMax.setVal(0.0f);
    }

  // in the older version, we used to set the map range as we would
  // also apply spatial competition for salience to the output map,
  // only if using the MAXNORM type of competition, and otherwise we
  // would not touch the range:
  if (itsUseOlderVersion.getVal() && itsNormType.getVal() != VCXNORM_MAXNORM)
   {
      itsOutputRangeMin.setVal(0.0f);
      itsOutputRangeMax.setVal(0.0f);
    }
}

// ######################################################################
void SingleChannel::stop2()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  delete [] itsSubmapCache; itsSubmapCache = 0;
}

// ######################################################################
void SingleChannel::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // reset some stuff for SingleChannel
  itsPyrBuilder->reset();
  itsPq.clear();
  itsClipPyr.clear();

  // propagate to our base class:
  ChannelBase::reset1();
}

// ######################################################################
void SingleChannel::accept(ChannelVisitor& v)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  v.visitSingleChannel(*this);
}

// ######################################################################
void SingleChannel::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ChannelBase::paramChanged(param, valueChanged, status);

  if (param == &itsSubmapAlgoType && valueChanged)
    {
      nub::ref<SubmapAlgorithm> algo =
        SubmapAlgorithm::make(itsSubmapAlgoType.getVal());

      this->setSubmapAlgorithm(algo);

      algo->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void SingleChannel::readFrom(const ParamMap& pmap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ChannelBase::readFrom(pmap);
  ChannelFacetMap::readFacetsFrom(pmap);
}

// ######################################################################
void SingleChannel::writeTo(ParamMap& pmap) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ChannelBase::writeTo(pmap);
  ChannelFacetMap::writeFacetsTo(pmap);
}

// ######################################################################
void SingleChannel::setTempl(const uint cntr, const uint surr,
                             Image<float> &templ)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  killCaches();
  itsTempl[csToIndex(cntr, surr)] = templ;
}

// ######################################################################
void SingleChannel::setBiasMask(Image<float> &biasMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  killCaches();
  itsBiasMask = biasMask;
}

// ######################################################################
Image<float> SingleChannel::getBiasMask() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsBiasMask;
}

// ######################################################################
Image<float> SingleChannel::getTempl(const uint cntr, const uint surr) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsTempl[csToIndex(cntr, surr)];

}

// ######################################################################
bool SingleChannel::outputAvailable() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // if we have an input handler, let's have it wait for output to
  // become available:
  if (itsInputHandler.is_valid())
    itsInputHandler->waitForOutput(const_cast<SingleChannel&>(*this));

  return (itsPq.empty() == false) || itsOutputCache.initialized();
}

// ######################################################################
bool SingleChannel::hasPyramid() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return (itsPq.empty() == false);
}

// ######################################################################
bool SingleChannel::hasOutputCache() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsOutputCache.initialized();
}

// ######################################################################
Dims SingleChannel::getMapDims() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int lev = itsLevelSpec.getVal().mapLevel();

  return Dims(this->getInputDims().w() / (1 << lev),
              this->getInputDims().h() / (1 << lev));
}

// ######################################################################
const Image<float>& SingleChannel::getImage(const uint lev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsPq.empty()) CLFATAL("I have no input pyramid yet!");
  return itsPq.front().pyr.getImage(lev);
}

// ######################################################################
Image<float> SingleChannel::centerSurround(const uint cntrlev,
                                           const uint surrlev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(itsLevelSpec.getVal().csOK(cntrlev, surrlev));
  if (itsPq.empty()) CLFATAL("I have no input pyramid yet!");

  // do basic center-surround for the front (latest) pyramid in queue:
  const ImageSet<float>& pyr = itsPq.front().pyr;
  double t = itsPq.front().t.secs();

  // compute center-surround:
  Image<float> cs = ::centerSurround(pyr, cntrlev, surrlev,
                                     itsTakeAbs.getVal(), &itsClipPyr);

  // do additional processing with other pyramids in queue:
  for (uint i = 1; i < itsPq.size(); ++i)
    {
      const ImageSet<float>& pyr2 = itsPq[i].pyr;
      double t2 = itsPq[i].t.secs();

      // compute a decay factor based on how old the second pyramid is
      // compared to the latest one:
      float fac = exp( (t2 - t) * itsTimeDecay.getVal());
      cs += ::centerSurroundDiff(pyr, pyr2, cntrlev, surrlev,
                                 itsTakeAbs.getVal(), &itsClipPyr) * fac;
    }

  return cs;
}

// ######################################################################
void SingleChannel::centerSurround(const uint cntrlev, const uint surrlev,
                                   Image<float>& pos, Image<float>& neg) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(itsLevelSpec.getVal().csOK(cntrlev, surrlev));
  if (itsPq.empty()) CLFATAL("I have no input pyramid yet!");

  // do basic center-surround for the front (latest) pyramid in queue:
  const ImageSet<float>& pyr = itsPq.front().pyr;
  double t = itsPq.front().t.secs();

  // compute center-surround:
  ::centerSurround(pyr, cntrlev, surrlev, pos, neg, &itsClipPyr);

  // do additional processing with other pyramids in queue:
  for (uint i = 1; i < itsPq.size(); ++i)
    {
      const ImageSet<float>& pyr2 = itsPq[i].pyr;
      double t2 = itsPq[i].t.secs();

      // compute a decay factor based on how old the second pyramid is
      // compared to the latest one:
      float fac = exp( (t2 - t) * itsTimeDecay.getVal());
      Image<float> pos2, neg2;
      ::centerSurroundDiff(pyr, pyr2, cntrlev, surrlev,
                           pos2, neg2, &itsClipPyr);
      pos += pos2 * fac;
      neg += neg2 * fac;
    }
}

// ######################################################################
uint SingleChannel::numSubmaps() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return maxIndex();
}

// ######################################################################
Image<float> SingleChannel::getSubmap(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // cache the results for later use if we don't already have cached it:
  if (itsSubmapCache[idx].initialized() == false)
    itsSubmapCache[idx] = itsSubmapAlgo->compute(*this, idx);

  return itsSubmapCache[idx];
}

// ######################################################################
Image<float> SingleChannel::getRawCSmap(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(itsLevelSpec.getVal().indexOK(idx));
  if (itsPq.empty()) CLFATAL("I have no input pyramid yet!");

  // determine any gain factor (possibly from a
  // ChannelFacetGainSingle). But note that we don't apply that gain
  // factor here, rather we just care to see whether we can avoid
  // computing the map if its gain factor is zero. Applying the gain
  // factors is done in combineSubmaps():
  float w = 1.0;
  if (hasFacet<ChannelFacetGainSingle>())
    w = getFacet<ChannelFacetGainSingle>()->getVal(idx);

  // if we have a zero weight, just return an empty image:
  if (w == 0.0f) return Image<float>(getMapDims(), ZEROS);

  // otherwise, compute center-surround map:
  uint clev = 0, slev = 0;
  indexToCS(idx, clev, slev);

  Image<float> submap;
  if (itsUseSplitCS.getVal())
    {
      // if using split center-surround, compute both the positive
      // and negative submaps, normalize them, sum them and we
      // will below further normalize the sum:
      Image<float> subpos, subneg, subpos2, subneg2;

      // positive and negative submaps are computed from a
      // single center-surround difference:
      this->centerSurround(clev, slev, subpos, subneg);

      // also compute maps containing max(map)-map:
      float pmin, pmax, nmin, nmax;
      getMinMax(subpos, pmin, pmax); subpos2 = binaryReverse(subpos, pmax);
      getMinMax(subneg, nmin, nmax); subneg2 = binaryReverse(subneg, nmax);

      // apply spatial competition for salience, independently to
      // both positive and negative parts of the submap,
      // preserving their original range rather than first
      // normalizing them to [MAXNORMMIN..MAXNORMMAX] as we
      // usually do, so that those maps who contain nothing do not
      // get artificially amplified:
      subpos = maxNormalize(subpos, 0.0f, 0.0f, itsNormType.getVal());
      subneg = maxNormalize(subneg, 0.0f, 0.0f, itsNormType.getVal());
      subpos2 = maxNormalize(subpos2, 0.0f, 0.0f, itsNormType.getVal());
      subneg2 = maxNormalize(subneg2, 0.0f, 0.0f, itsNormType.getVal());

      // the raw submap is the sum of the normalized positive and
      // negative sides:
      submap = subpos + subneg + subpos2 + subneg2;
    }
  else
    // submap is computed from a center-surround difference:
    submap = this->centerSurround(clev, slev);

  // print some debug info if in debug mode:
  if (MYLOGVERB >= LOG_DEBUG)
    {
      float mi, ma; getMinMax(submap, mi, ma);
      LDEBUG("%s(%d,%d): raw range [%f .. %f]", tagName().c_str(),
             clev, slev, mi, ma);
    }

  return submap;
}

// ######################################################################
Image<float> SingleChannel::postProcessMap(const Image<float>& smap,
                                           const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> submap(smap);
  uint clev = 0, slev = 0;
  indexToCS(idx, clev, slev);


  // resize submap to fixed scale if necessary:
  if (submap.getWidth() > getMapDims().w())
    submap = downSize(submap, getMapDims());
  else if (submap.getWidth() < getMapDims().w())
    submap = rescale(submap, getMapDims());

  // add noise if wanted:
  if (itsUseRandom.getVal())
    {
      // check if we want to scale noise to maximum of image
      if (itsScaleNoiseToMax.getVal())
        {
          float fMin, fMax; getMinMax(submap, fMin, fMax);
          inplaceAddBGnoise(submap, fMax);
        }
      else
        inplaceAddBGnoise(submap, 255.0);
    }

  // if using the older version: first normalize the submap to a
  // fixed dynamic range and then apply spatial competition for
  // salience to the submap; otherwise, just apply competition:
  if (itsUseOlderVersion.getVal())
    {
      LDEBUG("%s(%d,%d): applying %s(%f .. %f)", tagName().c_str(), clev, slev,
             maxNormTypeName(itsNormType.getVal()), MAXNORMMIN, MAXNORMMAX);
      submap = maxNormalize(submap, MAXNORMMIN, MAXNORMMAX,
                            itsNormType.getVal());
    }
  else
    {
      LDEBUG("%s(%d,%d): applying %s(0.0 .. 0.0)", tagName().c_str(),
             clev, slev, maxNormTypeName(itsNormType.getVal()));
      submap = maxNormalize(submap, 0.0f, 0.0f, itsNormType.getVal());
    }

  // print some debug info if in debug mode:
  if (MYLOGVERB >= LOG_DEBUG)
    {
      float mi, ma; getMinMax(submap, mi, ma);
      LDEBUG("%s(%d,%d): final range [%f .. %f]",
             tagName().c_str(), clev, slev, mi, ma);
    }

  return submap;
}

// ######################################################################
std::string SingleChannel::getSubmapName(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT( itsLevelSpec.getVal().indexOK(idx) );

  uint clev = 0, slev = 0;
  indexToCS(idx, clev, slev);

  return sformat("%s lev: %d delta: %d",
                 descriptiveName().c_str(), clev, slev-clev);
}

// ######################################################################
std::string SingleChannel::getSubmapNameShort(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT( itsLevelSpec.getVal().indexOK(idx) );

  uint clev = 0, slev = 0;
  indexToCS(idx, clev, slev);

  return sformat("%s(%d,%d)", tagName().c_str(), clev, slev);
}

// ######################################################################
void SingleChannel::getFeatures(const Point2D<int>& locn,
                                std::vector<float>& mean) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!this->outputAvailable())
    {
      CLDEBUG("I have no input pyramid yet -- RETURNING ZEROS");
      for (uint idx = 0; idx < numSubmaps(); idx ++) mean.push_back(0.0F);
      return;
    }

  // The coordinates we receive are at the scale of the original
  // image, and we will need to rescale them to the size of the
  // various submaps we read from. The first image in our first
  // pyramid has the dims of the input:
  const ImageSet<float>& pyr = itsPq.front().pyr;
  const Dims indims = this->getInputDims();

  for (uint idx = 0; idx < numSubmaps(); idx ++)
    {
      // get center and surround scales for this submap index:
      uint clev = 0, slev = 0;
      indexToCS(idx, clev, slev);

      // read center value with bilinear interpolation:
      ASSERT(pyr[clev].initialized());
      const float cval = pyr[clev].getValInterpScaled(locn, indims);

      // read surround value with bilinear interpolation:
      ASSERT(pyr[slev].initialized());
      const float sval = pyr[slev].getValInterpScaled(locn, indims);

      // compute center - surround and take absolute value if our
      // channel wants that:
      float val = cval - sval; if (itsTakeAbs.getVal()) val = fabs(val);

      // store the submap value at the chosen location:
      mean.push_back(val);
    }
}

// ######################################################################
void SingleChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                     std::vector<std::vector<float> > *mean,
                                     int *count) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!this->outputAvailable())
    {
      CLDEBUG("I have no input pyramid yet -- RETURNING ZEROS");
      for (uint idx = 0; idx < numSubmaps(); idx ++)
        {
          std::vector<std::vector<float> >::iterator imean = mean->begin();
          for(int i = 0; i < *count; i++, ++imean)
            imean->push_back(0.0);
        }
      return;
    }

  // The coordinates we receive are at the scale of the original
  // image, and we will need to rescale them to the size of the
  // various submaps we read from. The first image in our first
  // pyramid has the dims of the input:
  const ImageSet<float>& pyr = itsPq.front().pyr;
  const Dims indims = this->getInputDims();
  const uint sm = numSubmaps();
  for (uint idx = 0; idx < sm; ++idx)
    {
      // get center and surround scales for this submap index:
      uint clev = 0, slev = 0;
      indexToCS(idx, clev, slev);
      std::vector<Point2D<int>*>::iterator ilocn = locn->begin();
      std::vector<std::vector<float> >::iterator imean = mean->begin();

      for (int i = 0; i < *count; ++i, ++ilocn, ++imean)
        {
          // read center value with bilinear interpolation:
          ASSERT(pyr[clev].initialized());
          const float cval = pyr[clev].getValInterpScaled(**ilocn, indims);

          // read surround value with bilinear interpolation:
          ASSERT(pyr[slev].initialized());
          const float sval = pyr[slev].getValInterpScaled(**ilocn, indims);

          const float val = cval - sval;

          // store the submap value at the chosen location:
          imean->push_back(itsTakeAbs.getVal()
                           ? fabs(val)
                           : val);
        }
    }
}

// ######################################################################
void SingleChannel::killCaches()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ChannelBase::killCaches(); // call our base class's implementation

  itsOutputCache.freeMem();

  // our caches exist only when we are in started() state:
  if (started())
    for (uint i = 0; i < maxIndex(); ++i)
      itsSubmapCache[i] = Image<float>();
}

// ######################################################################
void SingleChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!this->started())
    CLFATAL("must be start()-ed before using receiving any input");

  ASSERT(inframe.grayFloat().initialized());

  // if we have an input handler, that will override our default
  // behavior:
  if (itsInputHandler.is_valid())
    {
      itsInputHandler->handleInput(*this, inframe.grayFloat(),
                                   inframe.time(), inframe.clipMask(),
                                   inframe.pyrCache());
    }
  else
    {
      setClipPyramid(inframe.clipMask());
      storePyramid(computePyramid(inframe.grayFloat(), inframe.pyrCache()),
                   inframe.time());
    }
}

// ######################################################################
void SingleChannel::inputPyramid(const ImageSet<float>& pyr,
                                 const SimTime& t,
                                 const Image<byte>& clipMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  killCaches();
  setClipPyramid(clipMask);

  storePyramid(pyr, t);
}

// ######################################################################
ImageSet<float> SingleChannel::
computePyramid(const Image<float>& bwimg,
               const rutz::shared_ptr<PyramidCache<float> >& cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // Compute our pyramid:
  ImageSet<float> py = itsPyrBuilder->
    build(bwimg, this->getMinPyrLevel(),
          this->getMaxPyrLevel(), cache.get());

  // Eliminate small values if and/or rectify the pyramid desired:
  if (itsLowThresh.getVal() > 0.0f)
    {
      if (itsRectifyPyramid.getVal())
        doLowThresh(py, itsLowThresh.getVal());
      else
        doLowThreshAbs(py, itsLowThresh.getVal());
    }
  else if (itsRectifyPyramid.getVal())
    doRectify(py);

  // return pyramid:
  return py;
}

// ######################################################################
void SingleChannel::storeOutputCache(const Image<float>& m)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsOutputCache = m;
}

// ######################################################################
void SingleChannel::setClipPyramid(const Image<byte>& clipMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // if necessary create pyramid of clipping masks
  if (clipMask.initialized())
    {
      itsClipPyr = buildPyrGaussian(Image<float>(clipMask)/255.0f,
                                    0, itsLevelSpec.getVal().maxDepth(), 9);
      doLowThresh(itsClipPyr, 1.0f, 0.0f);
    }
  else
    itsClipPyr.clear();
}

// ######################################################################
void SingleChannel::storeClipPyramid(const ImageSet<float>& p)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsClipPyr = p;
}

// ######################################################################
void SingleChannel::storePyramid(const ImageSet<float>& p,
                                 const SimTime& t)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // load our pyramid into our front pyramid:
  itsPq.push_front(TPyr(p, t));

  // truncate the pyramid queue if necessary:
  while(int(itsPq.size()) > itsQlen.getVal()) itsPq.pop_back();

  // We only want dyadic pyramids here:
  ASSERT(isDyadic(itsPq.front().pyr.subSet
                  (this->getMinPyrLevel(),
                   this->getMaxPyrLevel())));
}

// ######################################################################
void SingleChannel::storeSubmapCache(const ImageSet<float>& p)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(this->numSubmaps() == p.size());
  const uint maxind = this->numSubmaps();
  for (uint i = 0; i < maxind; ++i)
    this->itsSubmapCache[i] = p[i];
}

// ######################################################################
size_t SingleChannel::numPyramids() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsPq.size();
}

// ######################################################################
const ImageSet<float>& SingleChannel::pyramid(const uint index) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsPq.empty()) CLFATAL("I have no input pyramid yet!");
  ASSERT(index < itsPq.size());
  return itsPq[index].pyr;
}

// ######################################################################
SimTime SingleChannel::pyramidTime(const uint index) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsPq.empty()) return SimTime::ZERO();
  ASSERT(index < itsPq.size());
  return itsPq[index].t;
}

// ######################################################################
const ImageSet<float>& SingleChannel::clipPyramid() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsClipPyr;
}

// ######################################################################
LevelSpec SingleChannel::getLevelSpec() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsLevelSpec.getVal();
}

// ######################################################################
int SingleChannel::getNormType() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsNormType.getVal();
}

// ######################################################################
Image<float> SingleChannel::combineSubMaps()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> output(getMapDims(), ZEROS);

  // compute max-normalized weighted sum of center-surround at all levels:
  for (uint idx = 0; idx < maxIndex(); ++idx)
    {
      // determine any gain factor (possibly from a ChannelFacetGainSingle):
      float w = 1.0;
      if (hasFacet<ChannelFacetGainSingle>())
        w = getFacet<ChannelFacetGainSingle>()->getVal(idx);

      if (w != 0.0f)
        {
          Image<float> submap = getSubmap(idx); // get the unweighted map

          // Perform templ matching on submap?
          if (itsTempl[idx].initialized())
            {
              Image<float> img = templMatch(submap, itsTempl[idx]);
              //after templ matching the min is the winner, reverse the map
              //TODO (faster way?)
              Point2D<int> p; float maxval; findMax(img, p, maxval);
              img *= -1; img += maxval;
              //Expand the img to the origianl size since the tmplMatch returns
              //a smaller image by templ size
              //TODO (faster way?)
              submap.clear(0.0F);
              inplacePaste(submap, img, //center the image
                           Point2D<int>(itsTempl[idx].getWidth()/2,
                                        itsTempl[idx].getHeight()/2));
            }

          if (w != 1.0f) submap *= w;           // weigh the submap
          output += submap;                     // add submap to our sum

          if (MYLOGVERB >= LOG_DEBUG)
            {
              uint clev = 0, slev = 0;
              indexToCS(idx, clev, slev);
              LDEBUG("%s(%d,%d): weight %f", tagName().c_str(), clev, slev, w);
            }

          if (itsGetSingleChannelStats.getVal()) saveStats(submap, idx);
        }
    }

  // apply max-normalization on the output as needed:
  if(itsNormType.getVal() == VCXNORM_LANDMARK)
    {
      float goodness = pow(goodness_map(output), 0.1);
      LINFO("GOODNESS = %f", goodness);
      output *= goodness;
    }
  else if (itsNormalizeOutput.getVal())
    {
      LDEBUG("%s: Normalizing output: %s(%f .. %f)", tagName().c_str(),
             maxNormTypeName(itsNormType.getVal()), itsOutputRangeMin.getVal(),
             itsOutputRangeMax.getVal());

      output = maxNormalize(output, itsOutputRangeMin.getVal(),
                            itsOutputRangeMax.getVal(), itsNormType.getVal());
    }

  // print some debug info if in debug mode:
  if (MYLOGVERB >= LOG_DEBUG)
    {
      float mi, ma; getMinMax(output, mi, ma);
      LDEBUG("%s: final range [%f .. %f]", tagName().c_str(), mi, ma);
    }

  LINFO("Computed %s Conspicuity Map", descriptiveName().c_str());

  if (itsGetSingleChannelStats.getVal()) saveStats(output, -1);

  return output;
}

// ######################################################################
void SingleChannel::saveStats(const Image<float> img, const short idx)
{
  std::string fileName;

  if(itsSaveStatsPerChannel.getVal())
    {
      std::string dot = ".";
      std::string txt = ".txt";
      fileName = itsGetSingleChannelStatsFile.getVal()+ dot + tagName() + txt;
    }
  else
    fileName = itsGetSingleChannelStatsFile.getVal();

  ushort minx = 0, miny = 0, maxx = 0, maxy = 0;
  float  min,  max,  avg,  std;
  uint   N;
  // get a whole bunch of stats about this output image
  getMinMaxAvgEtc(img, min, max, avg, std, minx, miny, maxx, maxy, N);

  std::ofstream statsFile(fileName.c_str(), std::ios::app);

  statsFile << itsGetSingleChannelStatsTag.getVal() << "\t";

  statsFile << itsFrameIdx << "\t";

  // differentiate between scale image stats and the combined max norm
  // for this channel
  if(idx == -1)
    {
      statsFile << "COMBINED\t-1\t";
      itsFrameIdx++;
    }
  else
    statsFile << "SCALE\t" << idx << "\t";

  statsFile << tagName().c_str() << "\t" << descriptiveName().c_str() << "\t";
  statsFile << min  << "\t" << max  << "\t" << avg  << "\t" << std  << "\t"
            << minx << "\t" << miny << "\t" << maxx << "\t" << maxy << "\t"
            << N    << "\n";
  statsFile.close();

  if(itsSaveStatsPerChannelFreq.getVal())
    {
      std::string dot = ".";
      std::string txt = ".freq.txt";
      fileName = itsGetSingleChannelStatsFile.getVal()+ dot + tagName() + txt;

      FFTWWrapper fft(img.getWidth(), img.getHeight());
      double dimg[img.getHeight() * img.getWidth()];
      Image<float>::const_iterator itr = img.begin();
      double *ditr = &dimg[0];
      while(itr != img.end()) *ditr++ = double(*itr++);
      fft.init(dimg);
      double mag[img.getHeight() * (img.getWidth()/2 + 1)];
      fft.compute(mag);

      std::ofstream freqFile(fileName.c_str(), std::ios::app);
      freqFile << itsGetSingleChannelStatsTag.getVal() << "\t";
      freqFile << itsFrameIdx << "\t";

      // differentiate between scale image stats and the combined max norm
      // for this channel
      if(idx == -1) freqFile << "COMBINED\t-1\t";
      else freqFile << "SCALE\t" << idx << "\t";

      freqFile << "SIZE\t" << img.getWidth() <<"\t"<< img.getHeight() << "\n";

      for(int i = 0; i < img.getHeight(); i++)
        {
          for(int j = 0; j < (img.getWidth()/2 + 1); j++)
            freqFile << mag[i * (img.getWidth()/2 + 1) + j] << "\t";
          freqFile << "\n";
        }
      freqFile.close();
    }
}

// ######################################################################
Image<float> SingleChannel::getOutput()
{
GVX_TRACE(__PRETTY_FUNCTION__);

 if (!this->hasInput())
   // if you think this LFATAL() has been triggered incorrectly, then
   // first make sure that somebody has called setInputDims()
   CLFATAL("Oops! can't get output -- I don't even have any input yet");

  if (!this->outputAvailable())
    {
      // it's possible that we have input but don't yet have output in
      // the case of a channel that requires several input frames
      // before it can start generating output (such as a flicker or
      // motion channel); in that case we just return an empty image
      // of the appropriate size:
      LERROR("No %s channel yet! -- IGNORING.", this->tagName().c_str());
      return Image<float>(this->getMapDims(), ZEROS);
    }

  if (!itsOutputCache.initialized())
    itsOutputCache = combineSubMaps();

  return itsOutputCache;
}

// ######################################################################
void SingleChannel::setPyramid(rutz::shared_ptr<PyrBuilder<float> > pbuild)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(pbuild.get() != 0);
  itsPq.clear();  // forget about our old pyramids
  itsPyrBuilder = pbuild;
}

// ######################################################################
ImageSet<float>& SingleChannel::pyrMut(const uint index)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(index < itsPq.size());
  return itsPq[index].pyr;
}

// ######################################################################
uint SingleChannel::csToIndex(const uint centerlev,
                              const uint surroundlev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsLevelSpec.getVal().csToIndex(centerlev, surroundlev);
}


// ######################################################################
void SingleChannel::indexToCS(const uint index, uint& centerlev, uint& surroundlev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsLevelSpec.getVal().indexToCS(index, centerlev, surroundlev);
}

// ######################################################################
uint SingleChannel::maxIndex() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsLevelSpec.getVal().maxIndex();
}

// ######################################################################
void SingleChannel::saveResults(const nub::ref<FrameOstream>& ofs)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsPq.empty() == false)
    {
      // save raw pyramid levels?
      if (itsSaveRawMaps.getVal()) {
        const ImageSet<float>& pyr = itsPq.front().pyr;
        for (uint i = 0; i < pyr.size(); i ++)
        {
          // use --save-raw-map
          ofs->writeFloat(pyr[i], FLOAT_NORM_0_255,
                          sformat("SR%s-%d-", tagName().c_str(),i),
                          FrameInfo(sformat("%s SingleChannel raw map (%u of %u)",
                                            this->descriptiveName().c_str(),
                                            i, pyr.size()),
                                    SRC_POS));
        }
      }

      // save center-surround feature submaps?
      if (itsSaveFeatureMaps.getVal())
        for (uint i = 0; i < numSubmaps(); i ++) {
          uint clev = 0, slev = 0;
          indexToCS(i, clev, slev);
          ofs->writeFloat(getSubmap(i),
                          FLOAT_NORM_0_255,
                          sformat("SF%s-%d-%d-", tagName().c_str(),clev, slev),
                          FrameInfo(sformat("%s SingleChannel center-surround map (c=%u s=%u)",
                                            this->descriptiveName().c_str(),
                                            clev, slev),
                                    SRC_POS));
        }
    }

  // save output map?
  // --save-channel-outputs
  if (itsSaveOutputMap.getVal())
    ofs->writeFloat(getOutput(), FLOAT_NORM_0_255,
                    sformat("SO%s-", tagName().c_str()),
                    FrameInfo(sformat("%s SingleChannel output",
                                      this->descriptiveName().c_str()),
                              SRC_POS));
}

// ######################################################################
void SingleChannel::setInputHandler(rutz::shared_ptr<InputHandler> h)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsInputHandler = InputHandler::clone(h);
}

// ######################################################################
rutz::shared_ptr<InputHandler> SingleChannel::cloneInputHandler() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return InputHandler::clone(itsInputHandler);
}

// ######################################################################
void SingleChannel::setSubmapAlgorithm(nub::ref<SubmapAlgorithm> algo)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->removeSubComponent(*itsSubmapAlgo);
  itsSubmapAlgo = algo;
  this->addSubComponent(itsSubmapAlgo);

  if (this->started() && !algo->started())
    itsSubmapAlgo->start();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
