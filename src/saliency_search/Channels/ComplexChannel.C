/*!@file Channels/ComplexChannel.C Channel class that pools across multiple
  subchannels. */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ComplexChannel.C $
// $Id: ComplexChannel.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Channels/ComplexChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ChannelVisitor.H"
#include "Channels/ChannelFacets.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/fancynorm.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"

#include <algorithm>
#include <vector>

namespace dummy_namespace_to_avoid_gcc411_bug_ComplexChannel_C
{
  struct SubchanInfo
  {
    SubchanInfo(ComplexChannel* owner, nub::ref<ChannelBase> c,
                const double wt = 1.0)
      :
      chan(c),
      weight(NModelParam<double>::make
             (sformat("%s_weight", c->tagName().c_str()), owner, wt))
    {}

    nub::ref<ChannelBase> chan;
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

using namespace dummy_namespace_to_avoid_gcc411_bug_ComplexChannel_C;

struct ComplexChannel::Impl
{
  Impl()
    :
    sortChannelsByNumSubmaps(false)
  {}

  Image<float> output;
  rutz::shared_ptr<ChannelVisitor>  subchanVisitor; // apply to all newly-added subchannels

  std::vector<SubchanInfo> subchans;

  bool sortChannelsByNumSubmaps;

  SubchanInfo& findSubchanInfo(ComplexChannel::SubchanKey key)
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
// ComplexChannel member definitions:
// ######################################################################
// ######################################################################


// ######################################################################
ComplexChannel::ComplexChannel(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tag,
                               const VisualFeature vs) :
  ChannelBase(mgr, descrName, tag, vs),
  itsNormType(&OPT_MaxNormType, this),
  itsCombineType(&OPT_ComplexChannelMapCombineType, this),
  itsSaveOutputMap(&OPT_ComplexChannelSaveOutputMap, this),
  itsUseOlderVersion(&OPT_UseOlderVersion, this),
  itsUseSpaceVariantBoundary("UseSpaceVariantBoundary", this, false),
  itsOutputRangeMin(&OPT_ChannelOutputRangeMin, this),
  itsOutputRangeMax(&OPT_ChannelOutputRangeMax, this),
  rep(new Impl)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
ComplexChannel::~ComplexChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  delete rep;
}

// ######################################################################
void ComplexChannel::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  std::stable_sort(rep->subchans.begin(), rep->subchans.end(),
                   ChannelHierarchySorter(rep->sortChannelsByNumSubmaps));
}

// ######################################################################
void ComplexChannel::start2()
{
GVX_TRACE(__PRETTY_FUNCTION__);

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

  // if we are in space variant mode, make sure our subchannels
  // handle those types of images
  for (uint i = 0; i < this->numChans(); ++i)
    if (itsUseSpaceVariantBoundary.getVal())
      if (this->subChan(i)->hasModelParam("UseSpaceVariantBoundary", MC_IGNORE_MISSING) == false)
        LFATAL("All channels must support space variant image boundary "
               "conditions when using a retinal type 'RetinaCT'");
}

// ######################################################################
void ComplexChannel::stop2()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void ComplexChannel::accept(ChannelVisitor& v)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  v.visitComplexChannel(*this);
}

// ######################################################################
bool ComplexChannel::isHomogeneous() const
{
GVX_TRACE("ComplexChannel::isHomogeneous");

  for (uint i = 0; i < this->numChans(); ++i)
    {
      if (subChan(i)->visualFeature() != this->visualFeature()
          || (subChan(i)->isHomogeneous() == false))
        return false;
    }

  return true;
}

// ######################################################################
uint ComplexChannel::numChans() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return rep->subchans.size();
}

// ######################################################################
nub::ref<ChannelBase> ComplexChannel::subChan(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (idx >= rep->subchans.size())
    LFATAL("Ooops, no such sub-channel %u (I have only %" ZU " sub-channels)",
           idx, rep->subchans.size());

  return rep->subchans[idx].chan;
}

// ######################################################################
nub::ref<ChannelBase> ComplexChannel::subChan(const std::string& tagname) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  for (uint i = 0; i < rep->subchans.size(); ++i)
    if (rep->subchans[i].chan->tagName() == tagname)
      return rep->subchans[i].chan;

  LFATAL("Ooops, no such sub-channel '%s'", tagname.c_str());

  /* can't happen */ return nub::ref<ChannelBase>((ChannelBase*)0);
}

// ######################################################################
nub::ref<ChannelBase> ComplexChannel::
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
void ComplexChannel::readFrom(const ParamMap& pmap)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ChannelBase::readFrom(pmap);
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
void ComplexChannel::writeTo(ParamMap& pmap) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ChannelBase::writeTo(pmap);
  ChannelFacetMap::writeFacetsTo(pmap);

  for (uint i = 0; i < numChans(); ++i)
    {
      rutz::shared_ptr<ParamMap> submap(new ParamMap);
      subChan(i)->writeTo(*submap);
      submap->putDoubleParam("weight", rep->subchans[i].weight->getVal());
      submap->putDoubleParam("subchanindex", i);
      pmap.putSubpmap(subChan(i)->tagName(), submap);
    }
}

// ######################################################################
bool ComplexChannel::outputAvailable() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for (uint i = 0; i < numChans(); ++i)
    if (subChan(i)->outputAvailable() == false) return false;
  return true;
}

// ######################################################################
Dims ComplexChannel::getMapDims() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return subChan(0)->getMapDims();
}

// ######################################################################
uint ComplexChannel::numSubmaps() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint sum = 0;
  for (uint i = 0; i < numChans(); ++i) sum += subChan(i)->numSubmaps();
  return sum;
}

// ######################################################################
void ComplexChannel::lookupSubmap(const uint idx, uint& subchan,
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
Image<float> ComplexChannel::getSubmap(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint subchan = 0, subidx = 0;
  lookupSubmap(idx, subchan, subidx);
  return subChan(subchan)->getSubmap(subidx);
}

// ######################################################################
Image<float> ComplexChannel::getRawCSmap(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint subchan = 0, subidx = 0;
  lookupSubmap(idx, subchan, subidx);
  return subChan(subchan)->getRawCSmap(subidx);
}

// ######################################################################
std::string ComplexChannel::getSubmapName(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint subchan = 0, subidx = 0;
  lookupSubmap(idx, subchan, subidx);
  return subChan(subchan)->getSubmapName(subidx);
}

// ######################################################################
std::string ComplexChannel::getSubmapNameShort(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  uint subchan = 0, subidx = 0;
  lookupSubmap(idx, subchan, subidx);
  return subChan(subchan)->getSubmapNameShort(subidx);
}

// ######################################################################
void ComplexChannel::getFeatures(const Point2D<int>& locn,
                                 std::vector<float>& mean) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for(uint i = 0; i < numChans(); ++i)
    subChan(i)->getFeatures(locn, mean);
}

// ######################################################################
void ComplexChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                      std::vector<std::vector<float> > *mean,
                                      int *count) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for (uint i = 0; i < numChans(); ++i)
    subChan(i)->getFeaturesBatch(locn, mean, count);
}

// ######################################################################
void ComplexChannel::addSubChan(nub::ref<ChannelBase> ch,
                                const char* tagname,
                                const double weight,
                                bool exportOpts)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // if the user gave us a name for the channel, then let's rename it:
  if (tagname != 0 && *tagname != '\0')
    ch->setTagName(tagname);

  // take ownership of the subchannel:
  this->addSubComponent(ch);

  // add it to our list of subchannels:
  rep->subchans.push_back(SubchanInfo(this, ch, weight));

  // apply our subchannel visitor to the new subchannel:
  if (rep->subchanVisitor.is_valid())
    ch->accept(*rep->subchanVisitor);

  if (exportOpts)
    ch->exportOptions(MC_RECURSE);
}

// ######################################################################
void ComplexChannel::removeSubChan(nub::ref<ChannelBase> ch)
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
void ComplexChannel::removeAllSubChans()
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
bool ComplexChannel::hasSubChan(const char* tagname) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for (uint i = 0; i < rep->subchans.size(); ++i)
    if (rep->subchans[i].chan->tagName().compare(tagname) == 0)
      return true;

  return false;
}

// ######################################################################
void ComplexChannel::setSubchanVisitor(rutz::shared_ptr<ChannelVisitor> v)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  rep->subchanVisitor = v;

  // now apply the visitor to any subchannels that we already have:
  if (rep->subchanVisitor.is_valid())
    for (uint i = 0; i < this->numChans(); ++i)
      subChan(i)->accept(*rep->subchanVisitor);
}

// ######################################################################
void ComplexChannel::setSubchanTotalWeight(SubchanKey key, const double wt)
{
  this->killCaches();
  rep->findSubchanInfo(key).weight->setVal(wt);
}

// ######################################################################
double ComplexChannel::getSubchanTotalWeight(SubchanKey key) const
{
  return rep->findSubchanInfo(key).weight->getVal();
}

// ######################################################################
void ComplexChannel::sortChannelsByNumSubmaps(bool dosort)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (this->started())
    LFATAL("This must be called before start()");

  rep->sortChannelsByNumSubmaps = dosort;
}

// ######################################################################
void ComplexChannel::killCaches()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ChannelBase::killCaches(); // call our base class's implementation

  // free our own output cache:
  rep->output.freeMem();

  // propagate to our subchannels:
  for (uint i = 0; i < numChans(); ++i) subChan(i)->killCaches();
}

// ######################################################################
Image<float> ComplexChannel::getOutput()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!rep->output.initialized()) rep->output = combineOutputs();
  return rep->output;
}

// ######################################################################
Image<float> ComplexChannel::combineOutputs()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> output; // leave uninitialized so that mapCombine() can
                       // do the Right Thing on the first call

  uint num_nzw = 0; // number of subchans with non-zero weights

  for (uint i = 0; i < numChans(); ++i)
    {
      // Determine the weight for each subchan. This is the product of
      // the intrinsic weight of each subchan (which is set either
      // through the --vc-chans config string of VisualCortex for
      // top-level channels, or subject to static modifications by
      // using --load at the start of the entire model and setting the
      // NModelParam value associated with each channel), multiplied
      // by any possible extra top-down gain if we have a
      // ChannelFacetGainComplex:
      float w = float(rep->subchans[i].weight->getVal());

      if (w != 0.0F) ++num_nzw;

      if (hasFacet<ChannelFacetGainComplex>())
        w *= getFacet<ChannelFacetGainComplex>()->getVal(i);

      if (w != 0.0f)
        {
          Image<float> subChanOut = subChan(i)->getOutput();

          if (subChanOut.getDims() != this->getMapDims())
            LFATAL("Oops! In \"%s\", the output of "
                   "\"%s\" (sub-channel %u/%u) was expected to be "
                   "%dx%d, but was actually %dx%d",
                   this->tagName().c_str(),
                   subChan(i)->tagName().c_str(),
                   i+1, this->numChans(),
                   output.getWidth(), output.getHeight(),
                   subChanOut.getWidth(), subChanOut.getHeight());

          if (w != 1.0f) subChanOut *= w;

          output = mapCombine(itsCombineType.getVal(),
                              output, subChanOut);

          if (MYLOGVERB >= LOG_DEBUG)
            {
              float mi, ma, av; getMinMaxAvg(subChanOut, mi, ma, av);
              LDEBUG("%s: %s weight %f, range %g .. %g (mean %g)",
                     tagName().c_str(),
                     subChan(i)->tagName().c_str(), w, mi, ma, av);
            }
        }
    }

  // if we didn't have any subchannel outputs, then just fill our
  // output with zeros
  if (!output.initialized())
    output = Image<float>(getMapDims(), ZEROS);

  // apply max-normalization on the output as needed, but only if we
  // have more than one subchan with non-zero weight; otherwise, that
  // means that we are just a shell for a singlechannel and should not
  // add this extra maxnorm:
  if (num_nzw > 1)
    {
      if (itsNormType.getVal() == VCXNORM_LANDMARK)
        {
          float goodness = pow(goodness_map(output), 0.05);
          LINFO("GOODNESS = %f", goodness);
          output *= goodness;
        }
      else
        {
          LDEBUG("%s: Normalizing output: %s(%f .. %f)", tagName().c_str(),
                 maxNormTypeName(itsNormType.getVal()), itsOutputRangeMin.getVal(),
                 itsOutputRangeMax.getVal());
          output = maxNormalize(output, itsOutputRangeMin.getVal(),
                                itsOutputRangeMax.getVal(), itsNormType.getVal());
        }
    }

  // print some debug info if in debug mode:
  if (MYLOGVERB >= LOG_DEBUG)
    {
      float mi, ma; getMinMax(output, mi, ma);
      LDEBUG("%s: final range [%f .. %f]", tagName().c_str(), mi, ma);
    }

  LINFO("Computed %s Conspicuity Map", descriptiveName().c_str());

  return output;
}

// ######################################################################
void ComplexChannel::saveResults(const nub::ref<FrameOstream>& ofs)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // save our own map:
  if (itsSaveOutputMap.getVal())
    {
      std::string name = sformat("CO%s-", tagName().c_str());

      LINFO("SAVING RESULTS %s",name.c_str());

      ofs->writeFloat(getOutput(), FLOAT_NORM_0_255, name,
                      FrameInfo(sformat("%s ComplexChannel output",
                                        this->descriptiveName().c_str()),
                                SRC_POS));
    }

  // now do it for our subchannels:
  for (uint i = 0; i < numChans(); ++i) subChan(i)->saveResults(ofs);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
