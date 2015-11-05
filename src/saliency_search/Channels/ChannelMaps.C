/*!@file Channels/ChannelMaps.C Classes to hold maps from a Channel hierarchy */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelMaps.C $
// $Id: ChannelMaps.C 12820 2010-02-11 05:44:51Z itti $
//

#include "Channels/ChannelMaps.H"
#include "Channels/ChannelBase.H"
#include "Channels/SingleChannel.H"
#include "Channels/ComplexChannel.H"
#include "Channels/IntegerSimpleChannel.H"
#include "Channels/IntegerComplexChannel.H"
#include "Neuro/EnvVisualCortex.H"

// ######################################################################
ChannelMaps::ChannelMaps(ChannelBase* chan, const std::string& prefix) :
  itsOutputMap(), itsSubmaps(), itsRawCSmaps(), itsPyramid(), itsSubchanMaps()
{
  const std::string newprefix = prefix.empty() ? "" : prefix + ":";

  if (prefix.empty()) // VisualCortex always has an output
    itsOutputMap = NamedImage<float>(chan->getOutput(), "SaliencyMap");
  else
    {
      if (chan->outputAvailable())
        itsOutputMap = NamedImage<float>(chan->getOutput(), newprefix + chan->tagName());
      else
        itsOutputMap = NamedImage<float>(newprefix + chan->tagName());
    }

  // let's traverse the hierarchy
  if (SingleChannel* ch = dynamic_cast<SingleChannel*>(chan))
    {
      // get all the submaps:
      const uint n = ch->numSubmaps();
      for (uint i = 0; i < n; ++i)
        {
          const std::string name = newprefix + ch->getSubmapNameShort(i);
          if (ch->outputAvailable())
            {
              itsSubmaps.push_back(NamedImage<float>(ch->getSubmap(i), name));
              itsRawCSmaps.push_back(NamedImage<float>(ch->getRawCSmap(i), name + "raw"));
            }
          else
            {
              itsSubmaps.push_back(NamedImage<float>(name)); // empty image
              itsRawCSmaps.push_back(NamedImage<float>(name + "raw"));
            }
        }

      // get our latest pyramid:
      if (ch->hasPyramid()) itsPyramid = ch->pyramid(0);
    }
  else if (ComplexChannel* ch = dynamic_cast<ComplexChannel*>(chan))
    {
      const uint n = ch->numChans();
      for (uint i = 0; i < n; ++i)
        itsSubchanMaps.push_back(rutz::make_shared(new ChannelMaps(ch->subChan(i).get(), newprefix + ch->tagName())));
    }
  else if (IntegerSimpleChannel* ch = dynamic_cast<IntegerSimpleChannel*>(chan))
    {
      // get all the submaps:
      const uint n = ch->numSubmaps();
      for (uint i = 0; i < n; ++i)
        {
          const std::string name = newprefix + ch->getSubmapNameShort(i);
          if (ch->outputAvailable())
            {
              itsSubmaps.push_back(NamedImage<float>(ch->getSubmap(i), name));
              itsRawCSmaps.push_back(NamedImage<float>(ch->getRawCSmap(i), name + "raw"));
            }
          else
            {
              itsSubmaps.push_back(NamedImage<float>(name)); // empty image
              itsRawCSmaps.push_back(NamedImage<float>(name + "raw"));
            }
        }

      // get our latest pyramid:
      itsPyramid = ImageSet<float>(ch->intgPyramid());
    }
  else if (IntegerComplexChannel* ch = dynamic_cast<IntegerComplexChannel*>(chan))
    {
      const uint n = ch->numChans();
      for (uint i = 0; i < n; ++i)
        itsSubchanMaps.push_back(rutz::make_shared(new ChannelMaps(ch->subChan(i).get(), newprefix + ch->tagName())));
    }
  else if (ChannelBase* ch = dynamic_cast<ChannelBase*>(chan))
    {
      // get all the submaps:
      const uint n = ch->numSubmaps();
      for (uint i = 0; i < n; ++i)
        {
          const std::string name = newprefix + ch->getSubmapNameShort(i);
          if (ch->outputAvailable())
            itsSubmaps.push_back(NamedImage<float>(ch->getSubmap(i), name));
          else
              itsSubmaps.push_back(NamedImage<float>(name)); // empty image
        }
    }
  else LFATAL("Inconsistency in Channel hierarchy!");
}

// ######################################################################
ChannelMaps::ChannelMaps(const NamedImage<float>& outmap) :
  itsOutputMap(outmap), itsSubmaps(), itsRawCSmaps(), itsSubchanMaps()
{ }

// ######################################################################
ChannelMaps::ChannelMaps(EnvVisualCortexFloat* v, const std::string& prefix) :
  itsOutputMap(), itsSubmaps(), itsRawCSmaps(), itsSubchanMaps()
{
  const std::string npfx = prefix.empty() ? "VisualCortex:" : prefix + ":";

  // things are very simple here given the limitations of EnvVisualCortex:
  itsOutputMap = NamedImage<float>(v->getVCXmap(), "SaliencyMap");
  itsSubchanMaps.push_back(rutz::make_shared(new ChannelMaps(NamedImage<float>(v->getImap(), npfx + "intensity"))));
  itsSubchanMaps.push_back(rutz::make_shared(new ChannelMaps(NamedImage<float>(v->getCmap(), npfx + "color"))));
  itsSubchanMaps.push_back(rutz::make_shared(new ChannelMaps(NamedImage<float>(v->getOmap(), npfx + "orientation"))));
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  itsSubchanMaps.push_back(rutz::make_shared(new ChannelMaps(NamedImage<float>(v->getFmap(), npfx + "flicker"))));
  itsSubchanMaps.push_back(rutz::make_shared(new ChannelMaps(NamedImage<float>(v->getMmap(), npfx + "motion"))));
#endif
}

// ######################################################################
ChannelMaps::~ChannelMaps()
{ }

// ######################################################################
const NamedImage<float>& ChannelMaps::getMap() const
{ return itsOutputMap; }

// ######################################################################
uint ChannelMaps::numSubchans() const
{ return itsSubchanMaps.size(); }

// ######################################################################
rutz::shared_ptr<ChannelMaps> ChannelMaps::subChanMaps(const uint idx) const
{
  ASSERT(idx < itsSubchanMaps.size());
  return itsSubchanMaps[idx];
}

// ######################################################################
uint ChannelMaps::numSubmaps() const
{
  uint count = 0;
  for (uint i = 0; i < itsSubchanMaps.size(); ++i)
    count += itsSubchanMaps[i]->numSubmaps();

  return count + itsSubmaps.size();
}

// ######################################################################
const NamedImage<float>& ChannelMaps::getSubmap(const uint idx) const
{
  if (itsSubchanMaps.size()) // recurse through the subchans
    {
      uint subchan = 0, subidx = 0;
      lookupSubmap(idx, subchan, subidx);
      return itsSubchanMaps[subchan]->getSubmap(subidx);
    }
  else
    {
      ASSERT(idx < itsSubmaps.size());
      return itsSubmaps[idx];
    }
}

// ######################################################################
const NamedImage<float>& ChannelMaps::getRawCSmap(const uint idx) const
{
  if (itsSubchanMaps.size()) // recurse through the subchans
    {
      uint subchan = 0, subidx = 0;
      lookupSubmap(idx, subchan, subidx);
      return itsSubchanMaps[subchan]->getRawCSmap(subidx);
    }
  else
    {
      ASSERT(idx < itsRawCSmaps.size());
      return itsRawCSmaps[idx];
    }
}

// ######################################################################
void ChannelMaps::lookupSubmap(const uint idx, uint& subchan, uint& subidx) const
{
  uint offset = 0;
  for (subchan = 0; subchan < itsSubchanMaps.size(); ++subchan)
    {
      subidx = idx - offset;
      const uint nsub = itsSubchanMaps[subchan]->numSubmaps();
      if (subidx < nsub) return;  // found the right subchan+submap combination
      else offset += nsub;
    }
  LFATAL("invalid submap index: %d", idx);
}

// ######################################################################
bool ChannelMaps::hasPyramid() const
{ return itsPyramid.isNonEmpty(); }

// ######################################################################
const ImageSet<float>& ChannelMaps::getPyramid() const
{ return itsPyramid; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
