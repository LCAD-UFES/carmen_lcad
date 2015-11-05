/*!@file Channels/IntegerSimpleChannel.C IntegerSimpleChannel is like SingleChannel, but avoids floating-point arithmetic */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerSimpleChannel.C $
// $Id: IntegerSimpleChannel.C 10983 2009-03-05 07:19:14Z itti $
//

#ifndef CHANNELS_INTEGERSIMPLECHANNEL_C_DEFINED
#define CHANNELS_INTEGERSIMPLECHANNEL_C_DEFINED

#include "Channels/IntegerSimpleChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/GlobalOpts.H"
#include "Component/ModelOptionDef.H"
#include "Image/Image.H"
#include "Image/ImageSetOps.H"
#include "Image/IntegerMathOps.H"
#include "Image/MathOps.H"    // for binaryReverse()
#include "Image/PyrBuilder.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

#include "Image/CutPaste.H"
#include <cmath>
#include <limits>

const int INTMAXNORMMIN = 0;
const int INTMAXNORMMAX = 32768;

// ######################################################################
IntegerSimpleChannel::
IntegerSimpleChannel(OptionManager& mgr, const std::string& descrName,
                     const std::string& tag,
                     const VisualFeature vs,
                     rutz::shared_ptr<PyrBuilder<int> > pbuild,
                     nub::ref<IntegerMathEngine> eng) :
  IntegerChannel(mgr, descrName, tag, vs, eng),
  itsTakeAbs("IntegerSimpleChannelTakeAbs", this, false),
  itsNormalizeOutput("IntegerSimpleChannelNormalizeOutput", this, false),
  itsScaleNoiseToMax("IntegerSimpleChannelScaleNoiseToMax", this, false),
  itsLowThresh("IntegerSimpleChannelLowThresh", this, 0),
  itsRectifyPyramid("IntegerSimpleChannelRectifyPyramid", this, false),
  itsUseRandom(&OPT_UseRandom, this),  // see Component/ModelManager.{H,C}
  itsLevelSpec(&OPT_LevelSpec, this),    // see Channels/ChannelOpts.{H,C}
  itsNormType(&OPT_MaxNormType, this), // see Channels/ChannelOpts.{H,C}
  itsQlen(&OPT_SingleChannelQueueLen, this),       // see Channels/ChannelOpts.{H,C}
  itsUseOlderVersion(&OPT_UseOlderVersion, this), // see Channels/ChannelOpts.{H,C}
  itsSaveRawMaps(&OPT_SingleChannelSaveRawMaps, this),
  itsSaveFeatureMaps(&OPT_SingleChannelSaveFeatureMaps, this),
  itsSaveOutputMap(&OPT_SingleChannelSaveOutputMap, this),
  itsOutputRangeMin(&OPT_IntChannelOutputRangeMin, this),
  itsOutputRangeMax(&OPT_IntChannelOutputRangeMax, this),
  itsPyr(),
  itsT(SimTime::ZERO()),
  itsOutputCache(),
  itsSubmapCache(NULL),
  itsPyrBuilder(pbuild),
  itsClipPyr()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
IntegerSimpleChannel::~IntegerSimpleChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void IntegerSimpleChannel::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int maxind = itsLevelSpec.getVal().maxIndex();
  itsSubmapCache = new Image<int>[maxind];

  // in the new version, leave our output range open rather than
  // forcing it to a given range of values:
  if (itsUseOlderVersion.getVal() == false)
    {
      itsOutputRangeMin.setVal(0);
      itsOutputRangeMax.setVal(0);
    }

  // in the older version, we used to set the map range as we would
  // also apply spatial competition for salience to the output map,
  // only if using the MAXNORM type of competition, and otherwise we
  // would not touch the range:
  if (itsUseOlderVersion.getVal() && itsNormType.getVal() != VCXNORM_MAXNORM)
    {
      itsOutputRangeMin.setVal(0);
      itsOutputRangeMax.setVal(0);
    }

  if (itsPyrBuilder.get() == 0)
    LFATAL("Oops! I have no PyrBuilder!");
}

// ######################################################################
void IntegerSimpleChannel::stop2()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  delete [] itsSubmapCache; itsSubmapCache = 0;
}

// ######################################################################
void IntegerSimpleChannel::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // reset some stuff for IntegerSimpleChannel
  itsPyrBuilder->reset();
  itsPyr.clear();
  itsClipPyr.clear();
  killCaches();

  // propagate to our base class:
  IntegerChannel::reset1();
}

// ######################################################################
void IntegerSimpleChannel::readFrom(const ParamMap& pmap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!this->started())
    CLFATAL("must be start()-ed before using readFrom()");

  IntegerChannel::readFrom(pmap);
  ChannelFacetMap::readFacetsFrom(pmap);
}

// ######################################################################
void IntegerSimpleChannel::writeTo(ParamMap& pmap) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!this->started())
    CLFATAL("must be start()-ed before using readFrom()");

  IntegerChannel::writeTo(pmap);
  ChannelFacetMap::writeFacetsTo(pmap);
}

// ######################################################################
bool IntegerSimpleChannel::outputAvailable() const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  return (itsPyr.isEmpty() == false) || itsOutputCache.initialized();
}

// ######################################################################
Dims IntegerSimpleChannel::getMapDims() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int lev = itsLevelSpec.getVal().mapLevel();

  return Dims(this->getInputDims().w() / (1 << lev),
              this->getInputDims().h() / (1 << lev));
}

// ######################################################################
uint IntegerSimpleChannel::numSubmaps() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsLevelSpec.getVal().maxIndex();
}

// ######################################################################
Image<int> IntegerSimpleChannel::getSubmapInt(const uint i) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // cache the results for later use if we don't already have cached it:
  if (itsSubmapCache[i].initialized() == false)
    {
      itsSubmapCache[i] = this->getRawCSmapInt(i);
      uint clev = 0, slev = 0;
      itsLevelSpec.getVal().indexToCS(i, clev, slev);


      // resize itsSubmapCache[i] to fixed scale if necessary:
      if (itsSubmapCache[i].getWidth() > getMapDims().w())
        itsSubmapCache[i] = intgDownSize(itsSubmapCache[i], getMapDims(),
                                         9, this->getImath());
      else if (itsSubmapCache[i].getWidth() < getMapDims().w())
        itsSubmapCache[i] = intgRescale(itsSubmapCache[i], getMapDims());

      // add noise if wanted:
      if (itsUseRandom.getVal())
        {
          // check if we want to scale noise to maximum of image
          if (itsScaleNoiseToMax.getVal())
            {
              int fMin, fMax; getMinMax(itsSubmapCache[i], fMin, fMax);
              intgInplaceAddBGnoise(itsSubmapCache[i], fMax);
            }
          else
            intgInplaceAddBGnoise(itsSubmapCache[i],
                                  (1 << this->getImath()->nbits));
        }

      // if using the older version: first normalize the submap to a
      // fixed dynamic range and then apply spatial competition for
      // salience to the submap; otherwise, just apply competition:
      if (itsUseOlderVersion.getVal())
        {
          LDEBUG("%s(%d,%d): applying %s(%f .. %f)", tagName().c_str(), clev, slev,
                 maxNormTypeName(itsNormType.getVal()), MAXNORMMIN, MAXNORMMAX);
          itsSubmapCache[i] =
            intgMaxNormalize(itsSubmapCache[i], INTMAXNORMMIN, INTMAXNORMMAX,
                         itsNormType.getVal());
        }
      else
        {
          LDEBUG("%s(%d,%d): applying %s(0.0 .. 0.0)", tagName().c_str(),
                 clev, slev, maxNormTypeName(itsNormType.getVal()));
          itsSubmapCache[i] =
            intgMaxNormalize(itsSubmapCache[i], 0, 0,
                         itsNormType.getVal());
        }

      // print some debug info if in debug mode:
      if (MYLOGVERB >= LOG_DEBUG)
        {
          int mi, ma; getMinMax(itsSubmapCache[i], mi, ma);
          LDEBUG("%s(%d,%d): final range [%d .. %d]",
                 tagName().c_str(), clev, slev, mi, ma);
        }
    }

  return itsSubmapCache[i];
}

// ######################################################################
Image<int> IntegerSimpleChannel::getRawCSmapInt(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(itsLevelSpec.getVal().indexOK(idx));
  if (itsPyr.isEmpty()) CLFATAL("I have no input pyramid yet!");

  // otherwise, compute center-surround map:
  uint clev = 0, slev = 0;
  itsLevelSpec.getVal().indexToCS(idx, clev, slev);

  // submap is computed from a center-surround difference:
  const Image<int> submap =
    intgCenterSurround(itsPyr, clev, slev, itsTakeAbs.getVal(), &itsClipPyr);

  // print some debug info if in debug mode:
  if (MYLOGVERB >= LOG_DEBUG)
    {
      int mi, ma; getMinMax(submap, mi, ma);
      LDEBUG("%s(%d,%d): raw range [%d .. %d]", tagName().c_str(),
             clev, slev, mi, ma);
    }

  return submap;
}

// ######################################################################
std::string IntegerSimpleChannel::getSubmapName(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT( itsLevelSpec.getVal().indexOK(idx) );

  uint clev = 0, slev = 0;
  itsLevelSpec.getVal().indexToCS(idx, clev, slev);

  return sformat("%s lev: %d delta: %d",
                 descriptiveName().c_str(), clev, slev-clev);
}

// ######################################################################
std::string IntegerSimpleChannel::getSubmapNameShort(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT( itsLevelSpec.getVal().indexOK(idx) );

  uint clev = 0, slev = 0;
  itsLevelSpec.getVal().indexToCS(idx, clev, slev);

  return sformat("%s(%d,%d)", tagName().c_str(), clev, slev);
}

// ######################################################################
void IntegerSimpleChannel::getFeatures(const Point2D<int>& locn,
                                std::vector<float>& mean) const
{
  CLFATAL("not implemented");
}

// ######################################################################
void IntegerSimpleChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                     std::vector<std::vector<float> > *mean,
                                     int *count) const
{
  CLFATAL("not implemented");
}

// ######################################################################
void IntegerSimpleChannel::killCaches()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  IntegerChannel::killCaches(); // call our base class's implementation

  itsOutputCache.freeMem();

  // our caches exist only when we are in started() state:
  if (started())
    for (uint i = 0; i < itsLevelSpec.getVal().maxIndex(); ++i)
      itsSubmapCache[i] = Image<int>();
}

// ######################################################################
void IntegerSimpleChannel::doInputInt(const IntegerInput& inp,
                                      const SimTime& t,
                                      PyramidCache<int>* cache,
                                      const Image<byte>& clipMask)
{
  ASSERT(inp.grayInt().initialized());

  // if necessary create pyramid of clipping masks
  if (clipMask.initialized())
    {
      itsClipPyr = intgBuildPyrGaussian
        (intgScaleFromByte(&clipMask, this->getImath()->nbits),
         itsLevelSpec.getVal().maxDepth(), 9,
         this->getImath());
      intgDoLowThresh(itsClipPyr, 255, 0);
    }
  else
    itsClipPyr.clear();

  // Compute our pyramid:
  itsPyr = itsPyrBuilder->
    build(inp.grayInt(),
          this->getMinPyrLevel(), this->getMaxPyrLevel(), cache);

  // Eliminate small values if and/or rectify the pyramid desired:
  if (itsLowThresh.getVal() > 0)
    {
      if (itsRectifyPyramid.getVal())
        intgDoLowThresh(itsPyr, itsLowThresh.getVal());
      else
        intgDoLowThreshAbs(itsPyr, itsLowThresh.getVal());
    }
  else if (itsRectifyPyramid.getVal())
    intgDoRectify(itsPyr);

  itsT = t;

  // We only want dyadic pyramids here:
  ASSERT(isDyadic(itsPyr.subSet
                  (this->getMinPyrLevel(), this->getMaxPyrLevel())));
}

// ######################################################################
LevelSpec IntegerSimpleChannel::getLevelSpec() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsLevelSpec.getVal();
}

// ######################################################################
Image<int> IntegerSimpleChannel::getOutputInt()
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
      // of the appropriate size

      LERROR("No %s channel yet! -- IGNORING.", this->tagName().c_str());

      return Image<int>(this->getMapDims(), ZEROS);
    }

  if (!itsOutputCache.initialized())
    {
      itsOutputCache = Image<int>(getMapDims(), ZEROS);

      // compute max-normalized weighted sum of center-surround at all levels:
      for (uint idx = 0; idx < itsLevelSpec.getVal().maxIndex(); ++idx)
        {
          const Image<int> submap = getSubmapInt(idx); // get the unweighted map

          // add submap to our sum
          itsOutputCache += (submap / int(itsLevelSpec.getVal().maxIndex()));

          if (MYLOGVERB >= LOG_DEBUG)
            {
              uint clev = 0, slev = 0;
              itsLevelSpec.getVal().indexToCS(idx, clev, slev);
              LDEBUG("%s(%d,%d): weight %f", tagName().c_str(), clev, slev, 1.0f);
            }
        }


      // apply max-normalization on the output as needed:
      if (itsNormalizeOutput.getVal())
        {
          LDEBUG("%s: Normalizing output: %s(%d .. %d)", tagName().c_str(),
                 maxNormTypeName(itsNormType.getVal()), itsOutputRangeMin.getVal(),
                 itsOutputRangeMax.getVal());

          itsOutputCache =
            intgMaxNormalize(itsOutputCache, itsOutputRangeMin.getVal(),
                         itsOutputRangeMax.getVal(), itsNormType.getVal());
        }

      // print some debug info if in debug mode:
      if (MYLOGVERB >= LOG_DEBUG)
        {
          int mi, ma; getMinMax(itsOutputCache, mi, ma);
          LDEBUG("%s: final range [%d .. %d]", tagName().c_str(), mi, ma);
        }

      LINFO("Computed %s Conspicuity Map", descriptiveName().c_str());
    }

  return itsOutputCache;
}

// ######################################################################
uint IntegerSimpleChannel::csToIndex(const uint centerlev,
                              const uint surroundlev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsLevelSpec.getVal().csToIndex(centerlev, surroundlev);
}

// ######################################################################
void IntegerSimpleChannel::saveResults(const nub::ref<FrameOstream>& ofs)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsPyr.isEmpty() == false)
    {
      // save raw pyramid levels?
      if (itsSaveRawMaps.getVal()) {
        for (uint i = 0; i < itsPyr.size(); i ++)
          ofs->writeFloat(Image<float>(itsPyr[i]), FLOAT_NORM_0_255,
                          sformat("ISR%s-%d-", tagName().c_str(),i),
                          FrameInfo(sformat("%s IntegerSimpleChannel raw map (%u of %u)",
                                            this->descriptiveName().c_str(),
                                            i, itsPyr.size()),
                                    SRC_POS));
      }

      // save center-surround feature submaps?
      if (itsSaveFeatureMaps.getVal())
        for (uint i = 0; i < numSubmaps(); i ++) {
          uint clev = 0, slev = 0;
          itsLevelSpec.getVal().indexToCS(i, clev, slev);
          ofs->writeFloat(Image<float>(getSubmapInt(i)),
                          FLOAT_NORM_0_255,
                          sformat("ISF%s-%d-%d-", tagName().c_str(),clev, slev),
                          FrameInfo(sformat("%s IntegerSimpleChannel center-surround map (c=%u s=%u)",
                                            this->descriptiveName().c_str(),
                                            clev, slev),
                                    SRC_POS));
        }
    }

  // save output map?
  if (itsSaveOutputMap.getVal())
    ofs->writeFloat(getOutput(), FLOAT_NORM_0_255,
                    sformat("ISO%s-", tagName().c_str()),
                    FrameInfo(sformat("%s IntegerSimpleChannel output",
                                      this->descriptiveName().c_str()),
                              SRC_POS));
}

// ######################################################################
void IntegerSimpleChannel::setPyrBuilder(rutz::shared_ptr<PyrBuilder<int> > pbuild)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(pbuild.get() != 0);
  this->killCaches();
  itsPyrBuilder = pbuild;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERSIMPLECHANNEL_C_DEFINED
