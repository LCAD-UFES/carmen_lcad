/*!@file Channels/IntegerComplexChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerComplexChannel.C $
// $Id: IntegerComplexChannel.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef CHANNELS_INTEGERCOMPLEXCHANNEL_C_DEFINED
#define CHANNELS_INTEGERCOMPLEXCHANNEL_C_DEFINED

#include "Channels/IntegerComplexChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ChannelVisitor.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Image/IntegerMathOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/fancynorm.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "nub/ref.h"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"

#include <algorithm>
#include <vector>

namespace dummy_namespace_to_avoid_gcc411_bug_IntegerComplexChannel_C
{
  struct SubchanInfo
  {
    SubchanInfo(IntegerComplexChannel* owner, nub::ref<IntegerChannel> c)
      :
      chan(c),
      weight(NModelParam<double>::make
             (sformat("%s_weight", c->tagName().c_str()), owner, 1.0))
    {}

    nub::ref<IntegerChannel> chan;
    rutz::shared_ptr<NModelParam<double> > weight;
  };

  struct ChannelHierarchySorter
  {
    ChannelHierarchySorter(bool dosubmaps_)
      :
      dosubmaps(dosubmaps_)
    {}

    bool operator()(const SubchanInfo& i1, const SubchanInfo& i2)
    {
      nub::ref<ChannelBase> chan1 = i1.chan;
      nub::ref<ChannelBase> chan2 = i2.chan;

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

    bool dosubmaps;
  };
}

using namespace dummy_namespace_to_avoid_gcc411_bug_IntegerComplexChannel_C;

struct IntegerComplexChannel::Impl
{
  Impl()
    :
    sortChannelsByNumSubmaps(false)
  {}

  Image<int> output;
  rutz::shared_ptr<ChannelVisitor>  subchanVisitor; // apply to all newly-added subchannels

  std::vector<SubchanInfo> subchans;

  bool sortChannelsByNumSubmaps;

  SubchanInfo& findSubchanInfo(IntegerComplexChannel::SubchanKey key)
  {
    if (key.index != uint(-1))
      {
        // ok, the index is valid, let's look up based on that:

        if (key.index >= subchans.size())
          LFATAL("Ooops, no such sub-channel %u "
                 "(I have only %" ZU " sub-channels)",
                 key.index, subchans.size());

        return subchans[key.index];
      }
    else if (key.tag != 0)
      {
        // ok, the tagname is valid, let's look up based on that:

        for (uint i = 0; i < subchans.size(); ++i)
          if (subchans[i].chan->tagName().compare(key.tag) == 0)
            return subchans[i];

        LFATAL("Ooops, no such sub-channel '%s'", key.tag);
      }
    else if (key.addr != 0)
      {
        // ok, the address is valid, let's look up based on that:
        for (uint i = 0; i < subchans.size(); ++i)
          if (subchans[i].chan.get() == key.addr)
            return subchans[i];

        LFATAL("no such sub-channel '%s'", key.addr->tagName().c_str());
      }

    LFATAL("Ooops, invalid SubchanKey (one of index, tag, or address "
           "must be valid)");
    return subchans[0]; // keep compiler happy
  }
};

// ######################################################################
// ######################################################################
// IntegerComplexChannel member definitions:
// ######################################################################
// ######################################################################


// ######################################################################
IntegerComplexChannel::
IntegerComplexChannel(OptionManager& mgr,
                      const std::string& descrName,
                      const std::string& tag,
                      const VisualFeature vs,
                      nub::ref<IntegerMathEngine> eng)
  :
  IntegerChannel(mgr, descrName, tag, vs, eng),
  itsNormType(&OPT_MaxNormType, this),
  itsSaveOutputMap(&OPT_ComplexChannelSaveOutputMap, this),
  itsUseOlderVersion(&OPT_UseOlderVersion, this),
  itsOutputRangeMin(&OPT_IntChannelOutputRangeMin, this),
  itsOutputRangeMax(&OPT_IntChannelOutputRangeMax, this),
  rep(new Impl)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
IntegerComplexChannel::~IntegerComplexChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  delete rep;
}

// ######################################################################
void IntegerComplexChannel::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  std::stable_sort(rep->subchans.begin(), rep->subchans.end(),
                   ChannelHierarchySorter(rep->sortChannelsByNumSubmaps));
}

// ######################################################################
void IntegerComplexChannel::start2()
{
GVX_TRACE(__PRETTY_FUNCTION__);

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
}

// ######################################################################
void IntegerComplexChannel::stop2()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
bool IntegerComplexChannel::isHomogeneous() const
{
GVX_TRACE("IntegerComplexChannel::isHomogeneous");

  for (uint i = 0; i < this->numChans(); ++i)
    {
      if (subChan(i)->visualFeature() != this->visualFeature()
          || (subChan(i)->isHomogeneous() == false))
        return false;
    }

  return true;
}

// ######################################################################
uint IntegerComplexChannel::numChans() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return rep->subchans.size();
}

// ######################################################################
nub::ref<IntegerChannel> IntegerComplexChannel::subChan(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (idx >= rep->subchans.size())
    LFATAL("Ooops, no such sub-channel %u (I have only %" ZU " sub-channels)",
           idx, rep->subchans.size());

  return rep->subchans[idx].chan;
}

// ######################################################################
nub::ref<IntegerChannel> IntegerComplexChannel::subChan(const std::string& tagname) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  for (uint i = 0; i < rep->subchans.size(); ++i)
    if (rep->subchans[i].chan->tagName() == tagname)
      return rep->subchans[i].chan;

  LFATAL("Ooops, no such sub-channel '%s'", tagname.c_str());

  /* can't happen */ return nub::ref<IntegerChannel>((IntegerChannel*)0);
}

// ######################################################################
nub::ref<IntegerChannel> IntegerComplexChannel::
subChanFromSubmapNum(const uint oldIdx,
                     uint& newIdx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(oldIdx < numSubmaps());
  uint subchan = 0;
  lookupSubmap(oldIdx, subchan, newIdx);

  return subChan(subchan);
}

// ######################################################################
void IntegerComplexChannel::readFrom(const ParamMap& pmap)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  IntegerChannel::readFrom(pmap);
  ChannelFacetMap::readFacetsFrom(pmap);

  for (uint i = 0; i < numChans(); ++i)
    {
      const std::string tagname = subChan(i)->tagName();
      if (pmap.hasParam(tagname))
        {
          rutz::shared_ptr<ParamMap> submap = pmap.getSubpmap(tagname);
          subChan(i)->readFrom(*submap);

          double wt = rep->subchans[i].weight->getVal();

          if (submap->queryDoubleParam("weight", wt) == ParamMap::MISSING)
            rep->subchans[i].weight->setVal(1.0);
          else
            rep->subchans[i].weight->setVal(wt);
        }
    }
}

// ######################################################################
void IntegerComplexChannel::writeTo(ParamMap& pmap) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  IntegerChannel::writeTo(pmap);
  ChannelFacetMap::writeFacetsTo(pmap);

  for (uint i = 0; i < numChans(); ++i)
    {
      rutz::shared_ptr<ParamMap> submap(new ParamMap);
      subChan(i)->writeTo(*submap);
      submap->putDoubleParam("weight", rep->subchans[i].weight->getVal());
      pmap.putSubpmap(subChan(i)->tagName(), submap);
    }
}

// ######################################################################
bool IntegerComplexChannel::outputAvailable() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for (uint i = 0; i < numChans(); ++i)
    if (subChan(i)->outputAvailable() == false) return false;
  return true;
}

// ######################################################################
Dims IntegerComplexChannel::getMapDims() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return subChan(0)->getMapDims();
}

// ######################################################################
uint IntegerComplexChannel::numSubmaps() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint sum = 0;
  for (uint i = 0; i < numChans(); ++i) sum += subChan(i)->numSubmaps();
  return sum;
}

// ######################################################################
void IntegerComplexChannel::lookupSubmap(const uint idx, uint& subchan,
                                  uint& subidx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(idx < numSubmaps());
  uint offset = 0;
  for (subchan = 0; subchan < numChans(); ++subchan)
    {
      subidx = idx - offset;
      const uint nsub = subChan(subchan)->numSubmaps();
      if (subidx < nsub)
        {
          // OK, we have found the right subchan+submap combination:
          return;
        }
      else
        offset += nsub;
    }
  LFATAL("invalid submap index: %d", idx);
}

// ######################################################################
Image<int> IntegerComplexChannel::getSubmapInt(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint subchan = 0, subidx = 0;
  lookupSubmap(idx, subchan, subidx);
  return subChan(subchan)->getSubmapInt(subidx);
}

// ######################################################################
Image<int> IntegerComplexChannel::getRawCSmapInt(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint subchan = 0, subidx = 0;
  lookupSubmap(idx, subchan, subidx);
  return subChan(subchan)->getRawCSmapInt(subidx);
}

// ######################################################################
std::string IntegerComplexChannel::getSubmapName(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint subchan = 0, subidx = 0;
  lookupSubmap(idx, subchan, subidx);
  return subChan(subchan)->getSubmapName(subidx);
}

// ######################################################################
std::string IntegerComplexChannel::getSubmapNameShort(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint subchan = 0, subidx = 0;
  lookupSubmap(idx, subchan, subidx);
  return subChan(subchan)->getSubmapNameShort(subidx);
}

// ######################################################################
void IntegerComplexChannel::getFeatures(const Point2D<int>& locn,
                                 std::vector<float>& mean) const
{
  LFATAL("not implemented");
}

// ######################################################################
void IntegerComplexChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                      std::vector<std::vector<float> > *mean,
                                      int *count) const
{
  LFATAL("not implemented");
}

// ######################################################################
void IntegerComplexChannel::addSubChan(nub::ref<IntegerChannel> ch,
                                const char* tagname)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // if the user gave us a name for the channel, then let's rename it:
  if (tagname != 0 && *tagname != '\0')
    ch->setTagName(tagname);

  // take ownership of the subchannel:
  this->addSubComponent(ch);

  // add it to our list of subchannels:
  rep->subchans.push_back(SubchanInfo(this, ch));

  // apply our subchannel visitor to the new subchannel:
  if (rep->subchanVisitor.is_valid())
    ch->accept(*rep->subchanVisitor);
}

// ######################################################################
void IntegerComplexChannel::removeSubChan(nub::ref<IntegerChannel> ch)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  std::vector<SubchanInfo>::iterator itr = rep->subchans.begin();

  while (itr != rep->subchans.end())
    {
      if ((*itr).chan == ch)
        {
          itr = rep->subchans.erase(itr);
          this->removeSubComponent(*ch);
        }
      else
        ++itr;
    }
}

// ######################################################################
void IntegerComplexChannel::removeAllSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  while (rep->subchans.size() > 0)
    {
      this->removeSubComponent(*(rep->subchans.back().chan));
      rep->subchans.pop_back();
    }

  ASSERT(rep->subchans.size() == 0);
}

// ######################################################################
bool IntegerComplexChannel::hasSubChan(const char* tagname) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for (uint i = 0; i < rep->subchans.size(); ++i)
    if (rep->subchans[i].chan->tagName().compare(tagname) == 0)
      return true;

  return false;
}

// ######################################################################
void IntegerComplexChannel::setSubchanVisitor(rutz::shared_ptr<ChannelVisitor> v)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  rep->subchanVisitor = v;

  // now apply the visitor to any subchannels that we already have:
  if (rep->subchanVisitor.is_valid())
    for (uint i = 0; i < this->numChans(); ++i)
      subChan(i)->accept(*rep->subchanVisitor);
}

// ######################################################################
void IntegerComplexChannel::setSubchanTotalWeight(SubchanKey key, const double wt)
{
  this->killCaches();
  rep->findSubchanInfo(key).weight->setVal(wt);
}

// ######################################################################
double IntegerComplexChannel::getSubchanTotalWeight(SubchanKey key) const
{
  return rep->findSubchanInfo(key).weight->getVal();
}

// ######################################################################
void IntegerComplexChannel::sortChannelsByNumSubmaps(bool dosort)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (this->started())
    LFATAL("This must be called before start()");

  rep->sortChannelsByNumSubmaps = dosort;
}

// ######################################################################
void IntegerComplexChannel::killCaches()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  IntegerChannel::killCaches(); // call our base class's implementation

  // free our own output cache:
  rep->output.freeMem();

  // propagate to our subchannels:
  for (uint i = 0; i < numChans(); ++i) subChan(i)->killCaches();
}

// ######################################################################
Image<int> IntegerComplexChannel::getOutputInt()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!rep->output.initialized()) rep->output = combineOutputsInt();
  return rep->output;
}

// ######################################################################
Image<int> IntegerComplexChannel::combineOutputsInt()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<int> output(getMapDims(), ZEROS);
  for (uint i = 0; i < numChans(); ++i)
    {
      const Image<int> subChanOut = subChan(i)->getOutputInt();

      ASSERT(subChanOut.initialized());

      output += (subChanOut / int(numChans()));
    }

  LDEBUG("%s: Normalizing output: %s(%d .. %d)", tagName().c_str(),
         maxNormTypeName(itsNormType.getVal()),
         itsOutputRangeMin.getVal(),
         itsOutputRangeMax.getVal());
  output = intgMaxNormalize(output, itsOutputRangeMin.getVal(),
                            itsOutputRangeMax.getVal(), itsNormType.getVal());

  // apply max-normalization on the output as needed:
  // print some debug info if in debug mode:
  if (MYLOGVERB >= LOG_DEBUG)
    {
      int mi, ma; getMinMax(output, mi, ma);
      LDEBUG("%s: final range [%d .. %d]", tagName().c_str(), mi, ma);
    }

  LINFO("Computed %s Conspicuity Map", descriptiveName().c_str());

  return output;
}

// ######################################################################
void IntegerComplexChannel::saveResults(const nub::ref<FrameOstream>& ofs)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // save our own map:
  if (itsSaveOutputMap.getVal())
    ofs->writeFloat(Image<float>(getOutputInt()), FLOAT_NORM_0_255,
                    sformat("ICO%s-", tagName().c_str()),
                    FrameInfo(sformat("%s IntegerComplexChannel output",
                                      this->descriptiveName().c_str()),
                              SRC_POS));

  // now do it for our subchannels:
  for (uint i = 0; i < numChans(); ++i) subChan(i)->saveResults(ofs);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERCOMPLEXCHANNEL_C_DEFINED
